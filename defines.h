// Define global variables and Defines

#include "10DOF_IMU.h"
#include <stdint.h>
/*
 * Defines
 */
#ifndef _DEFINES_
#define _DEFINES_
#define TRUE  1
#define FALSE 0

#define GLOBAL_ARRAY_SIZE 50

// PORT D
#define PIN_ENCODER_LEFT_A 2//9      // A leads in counter clockwise direction (Grey wire)
#define PIN_ENCODER_LEFT_B 3//10     // B leads in clockwise direction (Green wire)
//#define PIN_ENCODER1_Z 7//11     // White wire
// PORT B
#define PIN_ENCODER_RIGHT_A 4     
#define PIN_ENCODER_RIGHT_B 6
//#define PIN_ENCODER2_Z 10     // Z pulses every rotation

#define MY_SOF "{ " // Space required for splitting in the ros node
#define MY_EOF " }"

#define LED 13
//#define DEBUG 7

#define DIR_FORWARD   0
#define DIR_BACKWARD  1


#define STOPPED_PWM_VAL  0
#define MIN_PWM_VAL      0
#define MAX_PWM_VAL      4095
#define PWM_INCREMENTAL_VAL 10
#define NEAR_TARGET_VAL   200

#define MAX_SPEED       5

#define ENCODER_DIR_UNKNOWN             0
#define ENCODER_DIR_COUNTER_CLOCKWISE   1
#define ENCODER_DIR_CLOCKWISE           2
#define ENCODER_PULSES_PER_REVOLUTION   240   // Re-check This is kinda of a guess

#define BELT_LENGTH   0.409         // m
#define WHEEL_TRACK   0.150  //m 

#define P_GAIN        0.75
// #define I_GAIN        0.75
// #define D_GAIN        0.75
/*
 * Classes
 */
// class Tank{
//   public:
//   Tank(void);
//   imu = Imu()
//   private:

  
// };
  // Struct to hold control variables
typedef struct{
  // int p_gain;
  // int i_gain;
  // int d_gain;
  volatile int error;
  volatile int last_error;
}pid_s;

/*
 * Structures & Variables
 */

typedef struct{
    union{
        struct{
            volatile uint8_t a             :1;        // State of the a line on encoder
            volatile uint8_t last_a        :1;        // last State of the a line on encoder

            volatile uint8_t b             :1;        // State of the b line on encoder
            volatile uint8_t last_b       :1;        // last State of the b line on encoder
        }state;
        volatile uint8_t state_value;
    };


    union{
        struct{
            volatile uint8_t ab_changed      :1;        // if any encoder changes, show it has changed
            volatile uint8_t a              :1;  
            volatile uint8_t b              :1;  
        }flag;
        volatile uint8_t flag_value; 
    };
    volatile int32_t pulses;       //this variable stores our current encoder 1 pulse count
    int32_t last_pulses;       //this variable stores our current encoder 1 pulse count
    
    uint8_t fsm_state;
    volatile uint8_t direction;     // holds the direction of the encoder rotation (0 = unknown, 1 = counter clkwise 2 = clkwise)      S
}encoder_s;

typedef struct{
  union{
        struct{
          char relay_1       :1;        // State of relay 1
          char relay_2       :1;        // State of relay 2
          char relay_3       :1;        // State of relay 3
          char relay_4       :1;        // State of relay 4
          
          /*
          uint8_t q       :1;        // State of the a line on encoder
          uint8_t w       :1;        // last State of the a line on encoder
          uint8_t e       :1;        // State of the b line on encoder
          uint8_t r       :1;        // last State of the b line on encoder
          */
        }state;
        char state_value;
    };
    char last_state_value;
}io_s;

typedef struct{
  union{
        struct{
            char direction              :1;        // current direction
            char target_direction       :1;        // target diection
//          char d       :1;        // State of relay 4
        }flag;
        char flags_value;
    };
    unsigned int target_pwm;
    unsigned int current_pwm;
    float speed;
    pid_s control;
}motor_s;

typedef struct{
  io_s io;
  motor_s motor_left;
  motor_s motor_right;

}hardware_s;

extern hardware_s hardware;
extern encoder_s encoder_L;
extern encoder_s encoder_R;
extern char global_data[GLOBAL_ARRAY_SIZE];
extern volatile uint32_t ms_timer;

#endif
