/*
    OTHER FUNCTIONS USED

Notes:
- 

*/
#include "EEPROM.h"
#include "other.h"
#include "defines.h"

/*******************************************************************************************/
/*                                     G L O B A L S                                       */
/*******************************************************************************************/

extern hardware_s hardware;
extern encoder_s encoder_L;
extern encoder_s encoder_R;
extern char global_data[GLOBAL_ARRAY_SIZE];
extern rx_s rx_comms;
extern calibration_s calibration;

#ifdef __cplusplus
extern "C" {
#endif
/*******************************************************************************************/
/*                                    F U N C T I O N S                                    */
/*******************************************************************************************/

void rotaryEncoder_left (void);
void rotaryEncoder_right (void);
void motorTimerCb(void);

	// Performs a system reset:
void system_reset (void) {
		// Save EEPROM variables:
	save_eeprom_variables ();
    // Reset the system:
}

	// Saves EEPROM variables:
void save_eeprom_variables (void) {
}
// this returns the difference between the passed value and the ms_timer
unsigned long diff_millis(unsigned long  ms_timestamp)
{
  volatile unsigned long diff = 0;
  diff = millis() - ms_timestamp;
  // return the difference
  return diff;
}

	// Loads EEPROM variables:
void load_eeprom_variables (void) {
}

	// Initializes variables (Run at startup):
void init_variables (void) {
	//char i = 0;
  /*encoder.msg.revCount1 = 0;   //this variable stores our current encoder 1 revolution count
  encoder.msg.revCount2 = 0;   //this variable stores our current encoder 2 pulse count*/
 /* encoder.state.z1 = (digitalRead(PIN_ENCODER1_Z) != 0);
  encoder.state.last_z1 = encoder.state.z1;
  encoder.state.z2 = (digitalRead(PIN_ENCODER2_Z) != 0);
  encoder.state.last_z2 = encoder.state.z2;*/
  

  encoder_L.pulses = 0; //this variable stores our current encoder Left pulse count
  encoder_L.last_pulses = 0; //this variable stores our current encoder Left pulse count
  encoder_L.state.a = (digitalRead(PIN_ENCODER_LEFT_A) != 0);
  encoder_L.state.b = (digitalRead(PIN_ENCODER_LEFT_B) != 0);
  encoder_L.state.last_a = encoder_L.state.a;
  encoder_L.state.last_b = encoder_L.state.b;
  encoder_L.flag.a = 0;
  encoder_L.flag.b = 0;
  
  encoder_L.flag.ab_changed = 0;
  encoder_L.flag_value = 0; 
  encoder_L.direction = ENCODER_DIR_UNKNOWN;

  encoder_R.pulses = 0; //this variable stores our current encoder Right pulse count
  encoder_R.last_pulses = 0; //this variable stores our current encoder Left pulse count
  encoder_R.state.a = (digitalRead(PIN_ENCODER_RIGHT_A) != 0);
  encoder_R.state.b = (digitalRead(PIN_ENCODER_RIGHT_B) != 0);
  encoder_R.state.last_a = encoder_R.state.a;
  encoder_R.state.last_b = encoder_R.state.b;
  encoder_R.flag.a = 0;
  encoder_R.flag.b = 0;
  
  encoder_R.flag.ab_changed = 0;
  encoder_R.flag_value = 0; 
  encoder_R.direction = ENCODER_DIR_UNKNOWN;

  hardware.io.state_value = 0;
  hardware.motor_left.control.error = 0;
  hardware.motor_left.control.last_error = 0;
  hardware.motor_left.control.error_sum = 0;
  hardware.motor_right.control.error = 0;
  hardware.motor_right.control.last_error = 0;
  hardware.motor_right.control.error_sum = 0;

  hardware.motor_left.control.p_gain = P_GAIN;
  hardware.motor_right.control.p_gain = P_GAIN;
  hardware.motor_left.control.i_gain = I_GAIN;
  hardware.motor_right.control.i_gain = I_GAIN;
  hardware.motor_left.control.d_gain = D_GAIN;
  hardware.motor_right.control.d_gain = D_GAIN;
}

void init_pins(void){
/* Arduino Uno and nano
  //enabling PCI
  PCICR  |= 0x01; //enable port B for PCI
  PCMSK0 |= 0x07; //enable INT 0,1,2
  
  PCICR  |= 0x04; //enable port D for PCI
  PCMSK2 |= 0xE0; //enable INT 21,22,23
  //PCICR  |= 0x07; //enable ALL ports for PCI
  
  */
    // Mirco has only one interrupt channel. Set interrupts on pins 8,9,10,16 which is PCINT 6,5,4,2 = 0x74
  //PCMSK0 |= 0x74;//0x44;//
    // Clear the interrupt flag. This is also cleared when the interrupt is complete
 // PCIFR = 0x01;
    // enable interrupts
  //PCICR |= 0x01;
  
  
  pinMode(PIN_ENCODER_RIGHT_A, INPUT); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(PIN_ENCODER_RIGHT_B, INPUT); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)  
  pinMode(PIN_ENCODER_LEFT_A, INPUT); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(PIN_ENCODER_LEFT_B, INPUT); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)  

  
  pinMode(LED,INPUT);
  pinMode(LED,OUTPUT);
   //pinMode(DEBUG,OUTPUT);

}

void interrupt_setup(void){
  
  // Timer1.initialize(1000);  // Set the timer period to 1 second (in microseconds)
  // Timer1.attachInterrupt(motorTimerCb);  // Attach the timer callback function
  // attachInterrupt(digitalPinToInterrupt(pin), ISR, mode) (recommended)
  // attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_A), rotaryEncoder_left, RISING);
  // attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_B), rotaryEncoder_left, RISING);
  cli();           // Disable global interrupts
  // TCCR1A = 0;      // Set entire TCCR1A register to 0
  // TCCR1B = 0;      // Same for TCCR1B

  // // Set compare match register to desired interval
  // OCR1A = 1250;  // 16 MHz / 64 (prescaler) / 1000 (1 ms) - 1

  // // Enable timer compare interrupt
  // TIMSK1 |= (1 << OCIE1A);

  // // Set prescaler to 64 and start the timer
  // TCCR1B |= (1 << CS11);

 
  // Enable PCIE2 Bit3 = 1 (Port D)
  PCICR |= 0x04;
  // Select PCINT22 and 20 Bit6 and 4 = 1 (Pin D6&4) also 2&3
  PCMSK2 |= 0x5C;
  // Enable interrupts globally
  sei();
}

	// Initializes the unit by setting all parameters to defaults:
void do_first_boot (void) {
}


void send_packet(char cmd,int data_length,char *data){
  int i = 0;
  Serial.print(SOF);
  Serial.print(cmd);
  Serial.print(data_length);
  for (i = 0;i < data_length;i++){
    Serial.print(data[i]);
  }
  Serial.println(EOF);
}
// Add cmd to packet if more commands are added
void send_str (int8_t *data,int8_t len){
  int8_t i =0;
  int32_t crc = 0;
 // Serial.write(cmd);
  Serial.write(len);
  crc += len; // +cmd
  for (i=0;i < len;i++){
    Serial.write(*data);
    crc += *data;
    data++;
  }
  crc = ~crc;
  Serial.write(crc);
  Serial.println("");
}

// Function to set motor speeds based on linear and angular velocities
void setMotorSpeeds(int16_t linearVelocity, float angularVelocity,int16_t *vRight, int16_t *vLeft) {
  // Calculate angular velocities for each wheel
  vRight = linearVelocity + int16_t((WHEEL_TRACK/2) * angularVelocity);
  vLeft = linearVelocity - int16_t((WHEEL_TRACK/2) * angularVelocity); 
}

#ifdef __cplusplus
} //extern "C" 
#endif
