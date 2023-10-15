/* 
 * rosserial Planar Odometry Example
 */

// #include <Arduino.h>
#include <RotaryEncoder.h>

#include "defines.h"
#include "other.h"
#include "FaBoPWM_PCA9685.h"

FaBoPWM faboPWM(0x70);
#define MAX_VALUE 4095
#define MIN_VALUE 0

//Encoder Example for Arduino UNO with input signals on pin 2 and 3
#define PIN_IN1B 2
#define PIN_IN1A 3

#define PIN_IN2B 4
#define PIN_IN2A 6

/*
 * Variables
 */
hardware_s hardware;
encoder_s encoder_L;
encoder_s encoder_R;
rx_s rx_comms;

uint32_t _encoder_update_diff_ms_ = 0;
uint32_t last_encoder_update = 0;
uint32_t last_motor_update = 0;
uint32_t last_encoder_speed_update = 0;
char global_data[GLOBAL_ARRAY_SIZE];
volatile uint32_t ms_timer = 0;
uint32_t crc32 = 0;
uint8_t len = 4; // pulse,pulse,time_change,CRC
Imu imu;

uint16_t encoder_update_time = 0;
uint32_t last_imu_update = 0;
uint32_t last_pid_update = 0;

volatile uint32_t r_count = 0;
volatile uint32_t l_count = 0;


volatile unsigned long millisCount = 0;

/*
 * FUNCTIONS
 */
 
// Variable to hold an input character
char chr;


char packet_handler(void){
  switch(rx_comms.cmd) {
    case SET_MOTOR_SPEEDS:
      Serial.print("SET_MOTOR_SPEEDS ");
      break;

    case READ_ENCODERS:
      Serial.println("READ_ENCODERS ");
      *(long *)&global_data[0] = encoder_L.pulses;
      *(long *)&global_data[4] = encoder_R.pulses;
      send_packet(READ_ENCODERS_RESPONSE,8,global_data);
      break;

    case RESET_ENCODERS:
      encoder_L.pulses = 0;
      encoder_R.pulses = 0;
      encoder_L.last_pulses = 0;
      encoder_R.last_pulses = 0;
      Serial.println("OK Reset my encoders then. Be like that");
      break;

    case UPDATE_PID:
      // while ((str = strtok_r(p, ":", &p)) != '\0') {
      //   pid_args[i] = atoi(str);
      //   i++;
      // }
      // hardware.control.p_gain = int(pid_args[0]);
      // hardware.control.d_gain = pid_args[1];
      // hardware.control.i_gain = pid_args[2];
      // hardware.control.o_gain = pid_args[3];
      Serial.println("OK");
      break;

    case READ_IMU:
      Serial.println("READ IMU");
      *(int16_t *)&global_data[0] = int16_t(imu.heading*1000);
      *(int16_t *)&global_data[2] = int16_t(imu.raw_accel.x*1000);
      *(int16_t *)&global_data[4] = int16_t(imu.raw_accel.y*1000);
      *(int16_t *)&global_data[6] = int16_t(imu.raw_accel.z*1000);
      *(int16_t *)&global_data[8] = int16_t(imu.raw_gyro.x*1000);
      *(int16_t *)&global_data[10] = int16_t(imu.raw_gyro.y*1000);
      *(int16_t *)&global_data[12] = int16_t(imu.raw_gyro.z*1000);
      send_packet(READ_IMU_RESPONSE,14,global_data);
      break;

    case CALIBRATE_IMU:
      Serial.println("Calibrate IMU");

      // This needs to set a flag to calibrate in the main loop
      //  The calibration will get gyro offsets to zero them,
        // Rotation of the tank in the x,y plane to zero offsets . Find the mins and maxes for each to center the accel
        //  Get offsets and scalers for heading


      send_packet(CALIBRATE_IMU_RESPONSE,0,global_data);
      break;
    default:
      Serial.print("Invalid Command ");
      Serial.println(rx_comms.cmd,HEX);
      break;
  }
}


void setup()
{
  Serial.begin(115200);
  imu.init();
  init_pins();
  init_variables();
  interrupt_setup();

  if(faboPWM.begin()) {
    Serial.println("Found PCA9685");
    faboPWM.init(MIN_VALUE);
  }
  faboPWM.set_hz(15000);
  Serial.println("Init");
    //  Convert from mm/s to um/s (so it can stay as int16_t)
  hardware.motor_right.target_speed = 0.0;
  hardware.motor_left.target_speed = 0.0;

  digitalWrite(LED,HIGH);
  last_imu_update = millis();
}

void loop()
{ 
  float iValueL = 0, iValueR = 0;
  int16_t diff_error_L = 0, diff_error_R = 0;
  //float ax,ay,az,headingDegrees,heading,declinationAngle;

  if (Serial.available() > 0) {
    // Read the next character
    chr = Serial.read();
      // Message has started
    if (chr == SOF){
      //  Clear buffer and cmd variable
      rx_comms.cmd = -1;
      rx_comms.data_length = -1;
      rx_comms.packet_started = TRUE;
      rx_comms.count = 0;
      // start storing the serial input untill end of buffer
    } else if (chr == EOF){
      rx_comms.packet_started = FALSE;
      // Call handler function
      packet_handler();
    } else if (rx_comms.packet_started == TRUE){
      Serial.print(rx_comms.count);
      if (rx_comms.cmd  == -1){
        rx_comms.cmd = chr;
        Serial.println(rx_comms.cmd,HEX);
      }else if (rx_comms.data_length == -1){
        rx_comms.data_length = atoi(chr);
      } else {
        rx_comms.buffer[rx_comms.count] = atoi(chr);
        rx_comms.count ++;
      }
    }
  }

  if (diff_millis(last_imu_update) >= 100){

    imu.heading = imu.read_mag();
    imu.raw_accel = imu.read_accel();
    imu.raw_gyro = imu.read_gyro();
    Serial.print (String(imu.heading) + " ");
    Serial.print (String(imu.raw_accel.z) + " ");
    Serial.println (String(imu.raw_gyro.z) + " ");
    // digitalWrite(LED, !digitalRead(LED));  // Toggle the LED state

    // digitalWrite(LED,~digitalRead(LED));
    // Serial.println("LED "+String(digitalRead(LED)));

    last_imu_update = millis();
  }

  if ((encoder_update_time = diff_millis(last_pid_update)) >= 100){

    hardware.motor_left.current_speed = (((encoder_L.pulses - encoder_L.last_pulses)*1000/encoder_update_time)*MM_PER_PULSE);
    hardware.motor_right.current_speed = (((encoder_R.pulses - encoder_R.last_pulses)*1000/encoder_update_time)*MM_PER_PULSE);
    encoder_R.last_pulses = encoder_R.pulses;
    encoder_L.last_pulses = encoder_L.pulses;

    // Serial.print(String(hardware.motor_left.current_speed) + " ");
    // Serial.println(String(hardware.motor_right.current_speed));
      //  Calculate the Motor pwm errors
    hardware.motor_left.control.error = hardware.motor_left.target_speed-hardware.motor_left.current_speed;
    hardware.motor_right.control.error = hardware.motor_right.target_speed-hardware.motor_right.current_speed;

    hardware.motor_left.control.error_sum += hardware.motor_left.control.error;
    hardware.motor_right.control.error_sum += hardware.motor_right.control.error;

    // Cap summed errors
    if (hardware.motor_left.control.error_sum > MAX_SUM_ERROR) hardware.motor_left.control.error_sum = MAX_SUM_ERROR;
    else if (hardware.motor_left.control.error_sum < -MAX_SUM_ERROR) hardware.motor_left.control.error_sum = -MAX_SUM_ERROR;

    if (hardware.motor_right.control.error_sum > MAX_SUM_ERROR) hardware.motor_right.control.error_sum = MAX_SUM_ERROR;
    else if (hardware.motor_right.control.error_sum < -MAX_SUM_ERROR) hardware.motor_right.control.error_sum = -MAX_SUM_ERROR;

    diff_error_L = hardware.motor_left.control.last_error - hardware.motor_left.control.error;
    diff_error_R = hardware.motor_right.control.last_error - hardware.motor_right.control.error;
    
    iValueL = hardware.motor_left.control.error*hardware.motor_left.control.p_gain + diff_error_L*hardware.motor_left.control.d_gain + hardware.motor_left.control.error_sum*hardware.motor_left.control.i_gain;
    iValueR = float(hardware.motor_right.control.error*hardware.motor_right.control.p_gain) + float(diff_error_R*hardware.motor_right.control.d_gain) + float(hardware.motor_right.control.error_sum*hardware.motor_right.control.i_gain);


    hardware.motor_left.control.last_error = hardware.motor_left.control.error;
    hardware.motor_right.control.last_error = hardware.motor_right.control.error;


    hardware.motor_left.current_pwm += int16_t(iValueL * float(SPEED_TO_PWM));
    if (hardware.motor_left.current_pwm > 0){
      hardware.motor_left.flag.direction = DIR_FORWARD;
      if (hardware.motor_left.current_pwm > MAX_PWM_VAL) hardware.motor_left.current_pwm = MAX_PWM_VAL;
      else if (hardware.motor_left.current_pwm < 0) hardware.motor_left.current_pwm = 0;
      faboPWM.set_channel_value(0, (hardware.motor_left.current_pwm));      
      faboPWM.set_channel_value(1, 0); // anti-clockwise 

    } else {
      hardware.motor_left.flag.direction = DIR_BACKWARD;
      if (hardware.motor_left.current_pwm < -MAX_PWM_VAL) hardware.motor_left.current_pwm = -MAX_PWM_VAL;
      else if (hardware.motor_left.current_pwm > 0) hardware.motor_left.current_pwm = 0;
      faboPWM.set_channel_value(0, 0);      
      faboPWM.set_channel_value(1, hardware.motor_left.current_pwm*-1); // clockwise 
    }

    hardware.motor_right.current_pwm += int16_t(iValueR * float(SPEED_TO_PWM));
    if (hardware.motor_right.current_pwm > 0){
      hardware.motor_right.flag.direction = DIR_FORWARD;
      if (hardware.motor_right.current_pwm > MAX_PWM_VAL) hardware.motor_right.current_pwm = MAX_PWM_VAL;
      else if (hardware.motor_right.current_pwm < 0) hardware.motor_right.current_pwm = 0;

      faboPWM.set_channel_value(2, 0);      
      faboPWM.set_channel_value(3, hardware.motor_right.current_pwm); // clockwise 
    } else {
      hardware.motor_right.flag.direction = DIR_BACKWARD;
      if (hardware.motor_right.current_pwm < -MAX_PWM_VAL) hardware.motor_right.current_pwm = -MAX_PWM_VAL;
      else if (hardware.motor_right.current_pwm > 0) hardware.motor_right.current_pwm = 0;
      faboPWM.set_channel_value(2, (hardware.motor_right.current_pwm*-1));      
      faboPWM.set_channel_value(3, 0); // anti-clockwise 
    }

    last_pid_update = millis();
  }
}
    
ISR (PCINT2_vect)
{
// void rotaryEncoder_right (void){
  // digitalWrite(LED,HIGH);
  cli();

    // 136 pulses per rev on RIght side
  if ((digitalRead(PIN_ENCODER_RIGHT_A) != encoder_R.state.a) || (digitalRead(PIN_ENCODER_RIGHT_B) != encoder_R.state.b)){
    encoder_R.state.a = (digitalRead(PIN_ENCODER_RIGHT_A) != 0);
    encoder_R.state.b = (digitalRead(PIN_ENCODER_RIGHT_B) != 0);
if  (encoder_R.state.a > encoder_R.state.last_a){
        // Serial.println(" A State " + String(encoder_R.state.a) + " A last State " + String(encoder_R.state.a));
      if (encoder_R.state.b == 0){
        // Expect B to go high meaning we are going clkwise
        encoder_R.direction = ENCODER_DIR_COUNTER_CLOCKWISE;
      } else{
        // Expect B to go low mean we are going anti-clkwise
        encoder_R.direction = ENCODER_DIR_CLOCKWISE;
      }
    } else if  (encoder_R.state.a < encoder_R.state.last_a){
        // Serial.println(" A State " + String(encoder_R.state.a) + " A last State " + String(encoder_R.state.last_a));
      if (encoder_R.state.b == 0){
        // Expect B to go high meaning we are going clkwise
        encoder_R.direction = ENCODER_DIR_CLOCKWISE;
      } else{
        // Expect B to go low mean we are going anti-clkwise
        encoder_R.direction = ENCODER_DIR_COUNTER_CLOCKWISE;
      }
    }
       // Change has been high
    if  (encoder_R.state.b > encoder_R.state.last_b){
        // Serial.println(" B State " + String(encoder_R.state.b) + " B last State " + String(encoder_R.state.last_b));
      if (encoder_R.state.a == 0){
        // Expect A to go high meaning we are going clkwise
        encoder_R.direction = ENCODER_DIR_CLOCKWISE;
      } else{
        // Expect A to go low mean we are going anti-clkwise
        encoder_R.direction = ENCODER_DIR_COUNTER_CLOCKWISE;
      }
    } else if  (encoder_R.state.b < encoder_R.state.last_b){
        // Serial.println(" B State " + String(encoder_R.state.b) + " B last State " + String(encoder_R.state.last_b));
      if (encoder_R.state.a == 0){
        // Expect A to go high meaning we are going Counter clockwise
        encoder_R.direction = ENCODER_DIR_COUNTER_CLOCKWISE;
      } else{
        // Expect B to go low mean we are going clkwise
        encoder_R.direction = ENCODER_DIR_CLOCKWISE;
      }
    }

    if (encoder_R.direction == ENCODER_DIR_CLOCKWISE){
      encoder_R.pulses++;
      // Serial.println("RIGHT Count Increase " + String(encoder_R.pulses));
    } else if (encoder_R.direction == ENCODER_DIR_COUNTER_CLOCKWISE) {
      encoder_R.pulses--;
      // Serial.println("RIGHT Count Decrease " + String(encoder_R.pulses));
    }
    encoder_R.state.last_a = encoder_R.state.a;
    encoder_R.state.last_b = encoder_R.state.b;
  }
    // 136 pulses per rev
  if ((digitalRead(PIN_ENCODER_LEFT_A) != encoder_L.state.a) || (digitalRead(PIN_ENCODER_LEFT_B) != encoder_L.state.b)){
    
    encoder_L.state.a = (digitalRead(PIN_ENCODER_LEFT_A) != 0);
    encoder_L.state.b = (digitalRead(PIN_ENCODER_LEFT_B) != 0);

        // Change has been high
    if  (encoder_L.state.a > encoder_L.state.last_a){
      if (encoder_L.state.b == 0){
        // Expect B to go high meaning we are going clkwise
        encoder_L.direction = ENCODER_DIR_COUNTER_CLOCKWISE;
      } else{
        // Expect B to go low mean we are going anti-clkwise
        encoder_L.direction = ENCODER_DIR_CLOCKWISE;
      }
    } else if  (encoder_L.state.a < encoder_L.state.last_a){
      if (encoder_L.state.b == 0){
        // Expect B to go high meaning we are going clkwise
        encoder_L.direction = ENCODER_DIR_CLOCKWISE;
      } else{
        // Expect B to go low mean we are going anti-clkwise
        encoder_L.direction = ENCODER_DIR_COUNTER_CLOCKWISE;
      }
    }
       // Change has been high
    if  (encoder_L.state.b > encoder_L.state.last_b){
      if (encoder_L.state.a == 0){
        // Expect A to go high meaning we are going clkwise
        encoder_L.direction = ENCODER_DIR_CLOCKWISE;
      } else{
        // Expect A to go low mean we are going anti-clkwise
        encoder_L.direction = ENCODER_DIR_COUNTER_CLOCKWISE;
      }
    } else if  (encoder_L.state.b < encoder_L.state.last_b){
        // Serial.println(" B State " + String(encoder_L.state.b) + " B last State " + String(encoder_L.state.last_b));
      if (encoder_L.state.a == 0){
        // Expect A to go high meaning we are going Counter clockwise
        encoder_L.direction = ENCODER_DIR_COUNTER_CLOCKWISE;
      } else{
        // Expect B to go low mean we are going clkwise
        encoder_L.direction = ENCODER_DIR_CLOCKWISE;
      }
    }

    if (encoder_L.direction == ENCODER_DIR_CLOCKWISE){
      encoder_L.pulses--;
      // Serial.println("Left Count Decrease " + String(encoder_L.pulses));
    } else if (encoder_L.direction == ENCODER_DIR_COUNTER_CLOCKWISE) {
      encoder_L.pulses++;
      // Serial.println("Left Count Increase " + String(encoder_L.pulses));
    }
    encoder_L.state.last_b = encoder_L.state.b;
    encoder_L.state.last_a = encoder_L.state.a;  
  }
   // restart interrupts
  // Enable interrupts globally
  sei();
}

