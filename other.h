/*
  OTHER FUNCTIONS - Header File
*/
#ifndef _OTHER_FUNCTIONS_
#define _OTHER_FUNCTIONS_

#include <EEPROM.h>
#include "defines.h"
//#include "structure_definitions.h"

#if (ARDUINO >=100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#ifdef __cplusplus
extern "C" {
#endif
/*******************************************************************************************/
/*                                     D E F I N E S                                       */
/*******************************************************************************************/
	// Externals:
	
/*******************************************************************************************/
/*                                     G L O B A L S                                       */
/*******************************************************************************************/
	// Externals:




/*******************************************************************************************/
/*                        F U N C T I O N   P R O T O T Y P E S                            */
/*******************************************************************************************/

void system_reset (void);
unsigned long diff_millis(unsigned long  ms_timestamp);
void save_eeprom_variables (void);
void load_eeprom_variables (void);
void init_variables (void);
void init_pins(void);
void interrupt_setup(void);
void do_first_boot (void);
void manage_encoder_pulses(encoder_s *encoder);
void send_packet(char cmd,int data_length,char *data);
  // Declared in main
extern void rotaryEncoder_left (void);
extern void rotaryEncoder_right (void);
extern void motorTimerCb(void);
extern void setMotorSpeeds(int32_t linearVelocity, int32_t angularVelocity,int16_t *vRight, int16_t *vLeft);


//String processor(const String& var);
#ifdef __cplusplus
} //extern "C" 
#endif
#endif
