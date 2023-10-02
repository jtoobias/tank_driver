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
 * FUNCTIONS
 */
// void messageCb(const geometry_msgs::Twist& data){
//     // Define a geometry Twist object. Save values into that and handle the speed control in the interrupt
//     // Check if x vel is 0 and their is a twist command set the motors in different directions to turn the correct way.
//   float right_wheel_speed = data.linear.x + (data.angular.z*WHEEL_TRACK/2);
//   float left_wheel_speed = data.linear.x - (data.angular.z*WHEEL_TRACK/2);
//     // Stop wrap around
//   if (right_wheel_speed > MAX_SPEED) right_wheel_speed = MAX_SPEED;
//   if (left_wheel_speed > MAX_SPEED) left_wheel_speed = MAX_SPEED;
  
//     // convert to PWM
//   hardware.motor_right.target_pwm = (right_wheel_speed*MAX_PWM_VAL)/MAX_SPEED;
//   hardware.motor_left.target_pwm = (left_wheel_speed*MAX_PWM_VAL)/MAX_SPEED;
//     // Set up function to convert from speed m/s to PWM based on experiments.
// }
// // This interrupt routine will be called on any change of one of the input signals
// void checkPosition()
// {
//   //encoder.tick(); // just call tick() to check the state.
// }

// A pair of varibles to help parse serial commands (thanks Fergs)
// int arg = 0;
// int index = 0;

// // Variable to hold an input character
// char chr;

// // Variable to hold the current single-character command
// char cmd;

// // Character arrays to hold the first and second arguments
// char argv1[16];
// char argv2[16];

// // The arguments converted to integers
// long arg1;
// long arg2;

// /* Clear the current command parameters */
// void resetCommand() {
//   cmd = NULL;
//   memset(argv1, 0, sizeof(argv1));
//   memset(argv2, 0, sizeof(argv2));
//   arg1 = 0;
//   arg2 = 0;
//   arg = 0;
//   index = 0;
// }

// /* Run a command.  Commands are defined in commands.h */
// int runCommand() {
//   int i = 0;
//   char *p = argv1;
//   char *str;
//   int pid_args[4];
//   arg1 = atoi(argv1);
//   arg2 = atoi(argv2);
  
//   switch(cmd) {
//   case GET_BAUDRATE:
//     Serial.println(BAUDRATE);
//     break;
//   case ANALOG_READ:
//     Serial.println(analogRead(arg1));
//     break;
//   case DIGITAL_READ:
//     Serial.println(digitalRead(arg1));
//     break;
//   case ANALOG_WRITE:
//     analogWrite(arg1, arg2);
//     Serial.println("OK"); 
//     break;
//   case DIGITAL_WRITE:
//     if (arg2 == 0) digitalWrite(arg1, LOW);
//     else if (arg2 == 1) digitalWrite(arg1, HIGH);
//     Serial.println("OK"); 
//     break;
//   case PIN_MODE:
//     if (arg2 == 0) pinMode(arg1, INPUT);
//     else if (arg2 == 1) pinMode(arg1, OUTPUT);
//     Serial.println("OK");
//     break;
//   case PING:
//     Serial.println(Ping(arg1));
//     break;
// #ifdef USE_SERVOS
//   case SERVO_WRITE:
//     servos[arg1].setTargetPosition(arg2);
//     Serial.println("OK");
//     break;
//   case SERVO_READ:
//     Serial.println(servos[arg1].getServo().read());
//     break;
// #endif
    
// #ifdef USE_BASE
//   case READ_ENCODERS:
//     Serial.print(readEncoder(LEFT));
//     Serial.print(" ");
//     Serial.println(readEncoder(RIGHT));
//     break;
//    case RESET_ENCODERS:
//     resetEncoders();
//     resetPID();
//     Serial.println("OK");
//     break;
//   case MOTOR_SPEEDS:
//     /* Reset the auto stop timer */
//     lastMotorCommand = ms_timer;
//     if (arg1 == 0 && arg2 == 0) {
//       setMotorSpeeds(0, 0);
//       resetPID();
//       moving = 0;
//     }
//     else moving = 1;
//     leftPID.TargetTicksPerFrame = arg1;
//     rightPID.TargetTicksPerFrame = arg2;
//     Serial.println("OK"); 
//     break;
//   case MOTOR_RAW_PWM:
//     /* Reset the auto stop timer */
//     lastMotorCommand = ms_timer;
//     resetPID();
//     moving = 0; // Sneaky way to temporarily disable the PID
//     setMotorSpeeds(arg1, arg2);
//     Serial.println("OK"); 
//     break;
//   case UPDATE_PID:
//     while ((str = strtok_r(p, ":", &p)) != '\0') {
//        pid_args[i] = atoi(str);
//        i++;
//     }
//     Kp = pid_args[0];
//     Kd = pid_args[1];
//     Ki = pid_args[2];
//     Ko = pid_args[3];
//     Serial.println("OK");
//     break;
// #endif
//   default:
//     Serial.println("Invalid Command");
//     break;
//   }
// }

/*
 * Variables
 */
hardware_s hardware;
encoder_s encoder_L;
encoder_s encoder_R;

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

  // if ((encoder_L.flag.ab_changed || encoder_R.flag.ab_changed) && ((diff_millis(last_encoder_update) >= 50))){
  //   // Build and send packet
  //   encoder_L.flag.ab_changed = false;
  //   encoder_R.flag.ab_changed = false;
  //   _encoder_update_diff_ms_ = diff_millis(last_encoder_update);
  //   crc32 = encoder_L.pulses + encoder_R.pulses + _encoder_update_diff_ms_;
  //   crc32 = ~crc32;
  //   sprintf(global_data,"{%u %d %d %lu %l}",len,encoder_L.pulses,encoder_R.pulses,_encoder_update_diff_ms_,crc32);
  //   //Serial.println(pulse_data);
  //   str_msg.data = global_data;
  //   pulses.publish(&str_msg);
  //   last_encoder_update = ms_timer;

  // } 
  // while (Serial.available() > 0) {
  //   // Read the next character
  //   chr = Serial.read();

  //   // Terminate a command with a CR
  //   if (chr == 13) {
  //     if (arg == 1) argv1[index] = NULL;
  //     else if (arg == 2) argv2[index] = NULL;
  //     runCommand();
  //     resetCommand();
  //   }
  //   // Use spaces to delimit parts of the command
  //   else if (chr == ' ') {
  //     // Step through the arguments
  //     if (arg == 0) arg = 1;
  //     else if (arg == 1)  {
  //       argv1[index] = NULL;
  //       arg = 2;
  //       index = 0;
  //     }
  //     continue;
  //   }
  //   else {
  //     if (arg == 0) {
  //       // The first arg is the single-letter command
  //       cmd = chr;
  //     }
  //     else if (arg == 1) {
  //       // Subsequent arguments can be more than one character
  //       argv1[index] = chr;
  //       index++;
  //     }
  //     else if (arg == 2) {
  //       argv2[index] = chr;
  //       index++;
  //     }
  //   }

  
  if ((encoder_update_time = diff_millis(last_encoder_speed_update)) >= 100){
    uint32_t encoder_diff = 0;
    hardware.motor_left.speed = (encoder_L.pulses - encoder_L.last_pulses)/encoder_update_time;
    hardware.motor_right.speed = (encoder_R.pulses - encoder_R.last_pulses)/encoder_update_time;
    encoder_R.last_pulses = encoder_R.pulses;
    encoder_L.last_pulses = encoder_L.pulses;
    Serial.print(String(encoder_R.pulses)+ " ");
    Serial.println(String(encoder_L.pulses));
    // Serial.print(String(hardware.motor_left.speed))
    // Serial.print(String(hardware.motor_right.speed))

    faboPWM.set_channel_value(2, 0);      
    faboPWM.set_channel_value(3,0);

    faboPWM.set_channel_value(0, 0);      
    faboPWM.set_channel_value(1, 0); // clockwise 
    last_encoder_speed_update = millis();
  } 

  // if (diff_millis(last_imu_update) >= 1000){
  //   Serial.print(String(digitalRead(PIN_ENCODER_RIGHT_A)));
  //   Serial.println(" Right A ENCODER");
  //   Serial.print(String(digitalRead(PIN_ENCODER_RIGHT_B)));
  //   Serial.println(" Right B ENCODER");
  // }

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
  if ((digitalRead(PIN_ENCODER_RIGHT_A) != encoder_R.state.a) || (digitalRead(PIN_ENCODER_RIGHT_B) != encoder_R.state.b)){
    encoder_R.state.a = (digitalRead(PIN_ENCODER_RIGHT_A) != 0);
    encoder_R.state.b = (digitalRead(PIN_ENCODER_RIGHT_B) != 0);
    if (encoder_R.state.a /*&& !encoder_R.state.last_a*/){
      // digitalWrite(DEBUG,HIGH);
          // Both pins are high and we expected A line, This means B line was already high
      if (encoder_R.state.a && encoder_R.state.b && encoder_R.flag.a){
          encoder_R.direction = ENCODER_DIR_COUNTER_CLOCKWISE;                // Not checked
          encoder_R.pulses++;
          encoder_R.flag.ab_changed = TRUE;
          encoder_R.flag.a = FALSE;
          encoder_R.flag.b = FALSE;
          
      } else if (encoder_R.state.a && !encoder_R.state.b){
        encoder_R.flag.b = TRUE;
      // digitalWrite(DEBUG,LOW);
      }
    } 
    
    if (encoder_R.state.b /*&& !encoder_R.state.last_b*/){
        // Both pins are high and we expected B line, This means A line was already high
      if (encoder_R.state.a && encoder_R.state.b && encoder_R.flag.b){
        encoder_R.direction = ENCODER_DIR_CLOCKWISE;  // Not checked
        encoder_R.pulses--;
        encoder_R.flag.ab_changed = TRUE;
        encoder_R.flag.a = FALSE;
        encoder_R.flag.b = FALSE;
      } else if (!encoder_R.state.a && encoder_R.state.b){
        encoder_R.flag.a = TRUE;
      }
    }
    encoder_R.flag.ab_changed = TRUE;
  }
  if ((digitalRead(PIN_ENCODER_LEFT_A) != encoder_L.state.a) || (digitalRead(PIN_ENCODER_LEFT_B) != encoder_L.state.b)){

    encoder_L.state.a = (digitalRead(PIN_ENCODER_LEFT_A) != 0);
    encoder_L.state.b = (digitalRead(PIN_ENCODER_LEFT_B) != 0);
    
  //  Serial.println(String(encoder.state_value));
    if (encoder_L.state.a /*&& !encoder_L.state.last_a*/){
        // Both pins are high and we expected A line, This means B line was already high
      if (encoder_L.state.a && encoder_L.state.b && encoder_L.flag.a){
          encoder_L.direction = ENCODER_DIR_COUNTER_CLOCKWISE;  // Not checked
          encoder_L.pulses++;
          encoder_L.flag.ab_changed = TRUE;
          encoder_L.flag.a = FALSE;
          encoder_L.flag.b = FALSE;
            // 
      } else if (encoder_L.state.a && !encoder_L.state.b) encoder_L.flag.b = TRUE;
    }
    
    if (encoder_L.state.b /*&& !encoder_L.state.last_b*/){
        // Both pins are high and we expected B line, This means A line was already high
      if (encoder_L.state.a && encoder_L.state.b && encoder_L.flag.b){
        encoder_L.direction = ENCODER_DIR_CLOCKWISE;  // Not checked
        encoder_L.pulses--;
        encoder_L.flag.ab_changed = TRUE;
        encoder_L.flag.a = FALSE;
        encoder_L.flag.b = FALSE;
      } else if (!encoder_L.state.a && encoder_L.state.b) encoder_L.flag.a = TRUE;
    }

    encoder_L.flag.ab_changed = TRUE;
  }
   // restart interrupts
  // Enable interrupts globally
  sei();
  // digitalWrite(LED,LOW);
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
