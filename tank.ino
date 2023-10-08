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
uint32_t last_imu_update;

volatile uint32_t r_count = 0;
volatile uint32_t l_count = 0;

/*
 * FUNCTIONS
 */
 

// A pair of varibles to help parse serial commands
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

// /* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}



// /* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  
}

void send_packet(int cmd,int data_length,char *data){
  int i = 0;
  Serial.print(SOF);
  Serial.print(cmd);
  Serial.print(data_length);
  for (i = 0;i < data_length;i++){
    Serial.print(String(data[i]));
  }
  Serial.print(EOF);
}

char packet_handler(void){
  switch(rx_comms.cmd) {
    case READ_ENCODERS:
      // Serial.print(SOF);
      // Serial.print(READ_ENCODERS);
      // Serial.print("2");
      // Serial.print(String(encoder_L.pulses));
      // Serial.print(" ");
      // Serial.print(String(encoder_R.pulses));
      // Serial.println(EOF);
      *(long *)global_data[0] = encoder_L.pulses;
      *(long *)global_data[4] = encoder_R.pulses;
      send_packet(READ_ENCODERS_RESPONSE,8,global_data);
      break;
    case RESET_ENCODERS:
      encoder_L.pulses = 0;
      encoder_R.pulses = 0;
      encoder_L.last_pulses = 0;
      encoder_R.last_pulses = 0;
      Serial.println("OK");
      break;
    case SET_MOTOR_SPEEDS:
      /* Reset the auto stop timer */
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
    default:
      Serial.println("Invalid Command");
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

  digitalWrite(LED,HIGH);
  last_imu_update = millis();
}

void loop()
{ 
  //float ax,ay,az,headingDegrees,heading,declinationAngle;

  while (Serial.available() > 0) {
    // Read the next character
    chr = Serial.read();

      // Message has started
    if (chr == SOF){
      //  Clear buffer and cmd variable
      rx_comms.cmd = -1;
      rx_comms.data_length = -1;
      rx_comms.packet_started = TRUE;
      // start storing the serial input untill end of buffer
    }else if (chr == EOF){
      rx_comms.packet_started = FALSE;
      // Call handler function
      packet_handler();

    } else if (rx_comms.packet_started == TRUE)
    if (rx_comms.count == 0){
      rx_comms.cmd = atoi(chr);
    }else if (rx_comms.count == 1){
      rx_comms.data_length = atoi(chr);
    }
    rx_comms.buffer[rx_comms.count] = atoi(chr);
    rx_comms.count ++;
  }

  
  if ((encoder_update_time = diff_millis(last_encoder_speed_update)) >= 100){
    uint32_t encoder_diff = 0;
    hardware.motor_left.speed = ((encoder_L.pulses - encoder_L.last_pulses)/encoder_update_time)*METERS_PER_PULSE;
    hardware.motor_right.speed = ((encoder_R.pulses - encoder_R.last_pulses)/encoder_update_time)*METERS_PER_PULSE;
    encoder_R.last_pulses = encoder_R.pulses;
    encoder_L.last_pulses = encoder_L.pulses;
    Serial.print(String(encoder_R.pulses)+ " ");
    Serial.println(String(encoder_L.pulses));
    Serial.print(String(hardware.motor_left.speed) + " ");
    Serial.println(String(hardware.motor_right.speed));

    // faboPWM.set_channel_value(2,0);      
    // faboPWM.set_channel_value(3,0);

    // faboPWM.set_channel_value(0, 0);      
    // faboPWM.set_channel_value(1, 0); // clockwise 
    last_encoder_speed_update = millis();
  } 

  // Serial.println ("hi");
  if (diff_millis(last_imu_update) >= 1000){
    imu.read_mag();
    imu.read_accel();
    imu.read_gyro();
    // digitalWrite(LED, !digitalRead(LED));  // Toggle the LED state

    // digitalWrite(LED,~digitalRead(LED));
    // Serial.println("LED "+String(digitalRead(LED)));

    last_imu_update = millis();
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

/*
 * INTERRUPTS
 */
  // millisecond timer interrupt
// ISR(TIMER1_COMPA_vect){
// void motorTimerCb() {
//   // Serial.println("HI timer " + String(ms_timer));
//   noInterrupts();
//   ms_timer++;
//     // Motor control section
//   if (diff_millis(last_motor_update) > 500){
//     last_motor_update = millis();
//     Serial.println(String(ms_timer));

//     // Need to add PID controller for speed


//     // if (hardware.motor_left.flag.direction == DIR_FORWARD){
//     //   faboPWM.set_channel_value(2, 0);      
//     //   faboPWM.set_channel_value(3, hardware.motor_left.current_pwm); // clockwise 
//     // } else{
//     //   faboPWM.set_channel_value(2, hardware.motor_left.current_pwm);      
//     //   faboPWM.set_channel_value(3, 0); // clockwise 
//     // }

//     // if (hardware.motor_right.flag.direction == DIR_FORWARD){
//     //     faboPWM.set_channel_value(0, 0);      
//     //     faboPWM.set_channel_value(1, hardware.motor_right.current_pwm); // clockwise 
//     // } else{
//     //   faboPWM.set_channel_value(0, hardware.motor_right.current_pwm);      
//     //   faboPWM.set_channel_value(1, 0); // clockwise 
//     // }

//         // check if the target pwm hasn't been reached or there is a change in direction
//     // if ((hardware.motor_left.current_pwm != hardware.motor_left.target_pwm) || (hardware.motor_left.flag.direction != hardware.motor_left.flag.target_direction)){
//     //     // change in direction?
//     //   if (hardware.motor_left.flag.direction != hardware.motor_left.flag.target_direction){
//     //       // Make sure current pwm doesnt go negative
//     //     if (hardware.motor_left.current_pwm != STOPPED_PWM_VAL)hardware.motor_left.current_pwm--;
//     //       // Decrease speed till minimum speed is hit.
//     //     if (hardware.motor_left.current_pwm <= MIN_PWM_VAL){
//     //       hardware.motor_left.flag.direction = hardware.motor_left.flag.target_direction;
//     //     }
//     //       // direction is the same, increment to get to the target PWM
//     //   } else {
//     //       // If in range set to target PWM
//     //     if (hardware.motor_left.current_pwm > (hardware.motor_left.target_pwm-PWM_INCREMENTAL_VAL)){
//     //       hardware.motor_left.current_pwm = hardware.motor_left.target_pwm;
//     //     } else {
//     //         // Increment to the target PWM
//     //       if ((hardware.motor_left.target_pwm-hardware.motor_left.current_pwm) > NEAR_TARGET_VAL) hardware.motor_left.current_pwm += PWM_INCREMENTAL_VAL*10;
//     //       else hardware.motor_left.current_pwm += PWM_INCREMENTAL_VAL;
//     //     }
//     //   }
//     // }
//     //      // check if the target pwm hasn't been reached or there is a change in direction
//     // if ((hardware.motor_right.current_pwm != hardware.motor_right.target_pwm) || (hardware.motor_right.flag.direction != hardware.motor_right.flag.target_direction)){
//     //     // change in direction?
//     //   if (hardware.motor_right.flag.direction != hardware.motor_right.flag.target_direction){
//     //       // Make sure current pwm doesnt go negative
//     //     if (hardware.motor_right.current_pwm != STOPPED_PWM_VAL)hardware.motor_right.current_pwm--;
//     //       // Decrease speed till minimum speed is hit.
//     //     if (hardware.motor_right.current_pwm <= MIN_PWM_VAL){
//     //       hardware.motor_right.flag.direction = hardware.motor_right.flag.target_direction;
//     //     }
//     //       // direction is the same, increment to get to the target PWM
//     //   } else {
//     //       // If in range set to target PWM
//     //     if (hardware.motor_right.current_pwm > (hardware.motor_right.target_pwm-PWM_INCREMENTAL_VAL)){
//     //       hardware.motor_right.current_pwm = hardware.motor_right.target_pwm;
//     //     } else {
//     //         // Increment to the target PWM
//     //       if ((hardware.motor_right.target_pwm-hardware.motor_right.current_pwm) > NEAR_TARGET_VAL) hardware.motor_right.current_pwm += PWM_INCREMENTAL_VAL*10;
//     //       else hardware.motor_right.current_pwm += PWM_INCREMENTAL_VAL;
//     //     }
//     //   } 
//     // } 
//   }

//   interrupts();
// }
