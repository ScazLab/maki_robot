#ifndef MAKIv14_h
#define MAKIv14_h

#define SERVOCOUNT  6  // MAKIv1.4 has 6 servos
/* Dynamixel AX-12 enumerations corresponding to MAKIv1.4 assembly instructions  */
#define LR  1    // EYELID_RIGHT
#define LL  2    // EYELID_LEFT
#define EP  3    // EYE_PAN
#define ET  4    // EYE_TILT
#define HT  5    // HEAD_TILT    // vertical inclination of face; also known as pitch
#define HP  6    // HEAD_PAN    // horizontal inclination of face; also known as yaw

#define DIRECTION_BIT 1024  // load direction; see http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_10

#define DEFAULT_POS  512    // Dynamixel Ax-12 value when centered; 150 degrees
#define DEFAULT_ANG   150
#define MAX_DYN_POS       1023  // Dynamixel Ax-12 value when max CCW; 300 degrees
#define MAX_DYN_ANG       300
#define MIN_DYN_POS       0     // Dynamixel Ax-12 value when min CW; 0 degrees
#define MIN_DYN_ANG       0

#define MAX_POS MAX_DYN_POS
#define MAX_ANG MAX_DYN_ANG
#define MIN_POS MIN_DYN_POS
#define MIN_ANG MIN_DYN_ANG

// emperically determined reasonable values
int min_servo_pos[] = {484,   // EYELID_RIGHT
                                    361,   // EYELID_LEFT
                                    460,   // EYE_PAN
                                    425,  //390,   // EYE_TILT
                                    435,   // HEAD_TILT
                                    256    //300   // HEAD_PAN
                                  };
int max_servo_pos[] = {666,   // EYELID_RIGHT
                                    535,   // EYELID_LEFT
                                    578,   // EYE_PAN
                                    610,   // EYE_TILT
                                    582,   // HEAD_TILT
                                    768    //696   // HEAD_PAN
                                  };
/*
// deprecated
PROGMEM prog_uint16_t neutral_servo_pos[] = {SERVOCOUNT,  // first number is # of servos
                                                                            525,   // EYELID_RIGHT
                                                                            500,   // EYELID_LEFT
                                                                            DEFAULT_POS,   // EYE_PAN
                                                                            DEFAULT_POS,   // EYE_TILT
                                                                            DEFAULT_POS,   //490,  // HEAD_TILT
                                                                            DEFAULT_POS   // HEAD_PAN
                                                                            };  
*/                                                                          

// see also fmorgner post on 6/3/2012 at forum.arduino.cc/index.php?topic=92364.0
// Added to top of /usr/share/arduino/hardware/arbotix/cores/arbotix/Arduino.h
// #define __ARV_LIBC_DEPRECATED_ENABLE__ 1

// see github.com/arduino/Arduino/wiki/1.6-Frequently-Asked-Questions
const int16_t neutral_servo_pos[] PROGMEM = {SERVOCOUNT,  // first number is # of servos
                                                                            525,   // EYELID_RIGHT
                                                                            500,   // EYELID_LEFT
                                                                            DEFAULT_POS,   // EYE_PAN
                                                                            DEFAULT_POS,   // EYE_TILT
                                                                            DEFAULT_POS,   //490,  // HEAD_TILT
                                                                            DEFAULT_POS   // HEAD_PAN
                                                                            };                                                                            

int default_goal_speed[] = {100,  // EYELID_RIGHT
                                            100,  // EYELID_LEFT
                                            100,  // EYE_PAN
                                            200,  // EYE_TILT
                                            15,    // HEAD_TILT
                                            51     // HEAD_PAN
                                          };

#endif
