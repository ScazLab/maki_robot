/* ========================================================================== */
/*  MAKIv1.4 Controller
/*  v 0.2
/*  
/*  Listens for serial port input in the following forms:
/*       F{MX|MN|PP|GP|PS|GS|PT|PL|ER|DP|DS}Z
/*       FMXZ              // get FEEDBACK of all servo MAXIMUM POSITION
/*       FMNZ              // get FEEDBACK of all servo MINIMUM POSITION
/*       FPPZ              // get FEEDBACK of all servo PRESENT POSITION
/*       FGPZ              // get FEEDBACK of all servo GOAL POSITION
/*       FPSZ              // get FEEDBACK of all servo PRESENT SPEED
/*       FGSZ              // get FEEDBACK of all servo GOAL SPEED
/*       FPTZ              // get FEEDBACK of all servo PRESENT TEMPERATURE (Celsius)
/*       FPLZ              // get FEEDBACK of all servo PRESENT LOAD
/*       FTLZ              // get FEEDBACK of all servo TORQUE LIMIT (percentage of MAX TORQUE)
/*       FERZ              // get FEEDBACK of all servo ERROR (AX_ALARM_LED)
/*       FDPZ              // get FEEDBACK of all servo default position values
/*       FDSZ              // get FEEDBACK of all servo default speed values         
/*
/*       mm{GP|GS}xxx{IPT}yyyyyZ
/*  where mm = {LR, LL, EP, ET, HT, HP} means eyelid right, eyelid left, eye pan, eye  tilt, head tilt, and head pan, respectively,
/*  xxx = [0, 1023], and yyyyy in milliseconds
/*       HPGP0400Z    // set HEAD_PAN servo to the GOAL POSITION 400
/*       EPGS0100Z    // set EYE_PAN servo to the GOAL SPEED 100
/*       HPGP768GS51Z  // set HEAD_PAN servo to the GOAL POSITION 768 with GOAL SPEED of 51
/*       HPGP512HTGP512IPT2500Z    // set HEAD_PAN servo to the GOAL POSITION of 512, HEAD_TILT servo to 512, and 
/*                                                        // locally adjust each servo's GOAL SPEED such that both movements take 2.5 seconds
/*
/*       mmTLxxxxZ
/*  where xxx = [0, 1023]  default 1023; 0 is 0% and 1023 is 100%
/*       HTTL1023Z  // set HEAD_TILT servo to TORQUE LIMIT 1023, rather 100% of MAX TORQUE
/*                  // NOTE: Use this if FTLZ indicates that a servo is at 0%; this will re-enable the servo
/*                  // WARNING: Before re-enabling, make sure that the servo's GOAL POSITION and PRESENT POSITION are the same;
/*                  // otherwise, servo will rapidly snap into place upon executing this servo command
/*
/*  Authors:
/*    Kate Tsui, Drew O'Donnell
/*
/*  Updated: 2016-01-28
/*
/*  Acknowledgements:
/*  * Andre Pereira (FullServoKeeponNewBop.ino)
/*  * AXTurretTest.ino
/*  * http://forums.trossenrobotics.com/showthread.php?4434-Controlling-AX-or-RX-servos-using-arbotix&p=44430#post44430
/*  * http://support.robotis.com/en/techsupport_eng.htm#product/dynamixel/ax_series/dxl_ax_actuator.htm
/* ========================================================================== */

#include <ax12.h>  //import ax12 library to send DYNAMIXEL commands; download from https://code.google.com/p/arbotix/source/browse/trunk/arbotix/libraries/#libraries%2FBioloid
#include <avr/pgmspace.h>
#include <string.h>
#include <math.h>
#include "MAKIv14.h"

#define DEBUG  false    //true

/* ---- CONSTANTS ---- */
#define USER_LED  0    // Pin 0 maps to the USER LED on the ArbotiX Robocontroller.
#define INVALID_INT 9999

#define TERM_CHAR  'Z'    // syntax for end of servo command

#define SC_SET_GP "GP"    // servo command syntax for setting a specified servo's GOAL POSITION
#define SC_SET_IPT  "IPT"  // servo command syntax for setting the INTERPOLATION POSE TIME
#define SC_SET_GS  "GS"  // servo command syntax for setting a specified servo's GOAL SPEED
#define SC_SET_TM  "TM"  // servo command syntax for setting a specified servo's MAX TORQUE. Note: This value exists in EEPROM and persists when power is removed; default 1023
#define SC_SET_TL  "TL"  // servo command syntax for setting a specified servo's TORQUE LIMIT. Note: If set to 0 by ALARM SHUTDOWN, servo won't move until set to another value[1,1023]; default MAX TORQUE
#define SC_SET_TS  "TS"  // servo command syntax for setting a specified servo's TORQUE ENABLE. Note: Boolean [0, 1]; default 1. Doesn't appear to disable servo's movement when set to 0

#define SC_FEEDBACK "F"    // servo command synatx for FEEDBACK or status of all servos
#define SC_GET_MX  "MX"  // servo command syntax for FEEDBACK of all servo MAXIMUM POSITION
#define SC_GET_MN  "MN"  // servo command syntax for FEEDBACK of all servo MINIMUM POSITION
#define SC_GET_GP "GP"      // servo command synatx for feedback with GOAL POSITION
#define SC_GET_PP "PP"      // servo command synatx for feedback with PRESENT POSITION
#define SC_GET_PS "PS"      // servo command syntax for feedback with PRESENT SPEED
#define SC_GET_GS  "GS"    // servo command syntax for feedback with GOAL SPEED
#define SC_GET_PT  "PT"    // servo command syntax for feedback with PRESENT TEMPERATURE (in Celsius)
#define SC_GET_PL  "PL"    // servo command syntax for feedback with PRESENT LOAD
#define SC_GET_TM  "TM"  // servo command syntax for feedback with MAX TORQUE
#define SC_GET_TL  "TL"  // servo command syntax for feedback with TORQUE LIMIT
#define SC_GET_TS  "TS"  // servo command syntax for feedback with TORQUE ENABLE
#define SC_GET_ER  "ER"  // servo command syntax for feedback with error returned from AX_ALARM_LED
#define SC_GET_DP  "DP"  // servo command syntax for default positions
#define SC_GET_DS  "DS"  // servo command syntax for default speed

/* ---- USER DEFINED GLOBALS ---- */
unsigned int baud_rate = 9600;    // NOTE: Make sure to set the serial monitor to this baud_rate
unsigned int interpolation_time = 500; // setup for interpolation from current->next over 1/2 second
boolean my_verbose_debug = DEBUG;
unsigned int EC_timer_duration = 30000;  // ms, or 30s
                                                                            
/* ---- DYNAMIC GLOBALS ---- modified programatically ---- */
/* -------- DO NOT MODIFY -------- */
String commandString;  // variable length string containing motor commands instead of char[]
int makiServoPos[SERVOCOUNT];                                                                           
int makiGoalPos[SERVOCOUNT];
int makiGoalSpeed[SERVOCOUNT];
boolean pp_flag = false;
boolean ipt_flag = false;
unsigned long errorCheck_timer;
//boolean ht_flag = true;  // KATE debugging
                                                                            
/* ------------------------------------------------------------------------------------------------------------ */
void setup()
{
  int nServo = 0;  
  pinMode(USER_LED, OUTPUT);     // initialize the digital pin as an output.
  
  // need to start the serial port
  Serial.begin(baud_rate);
  delay(100);  // wait to make sure serial connection established 
  
   for (nServo=1; nServo <= SERVOCOUNT; nServo++)  {
     // set reasonable speed limits for MAKI's servo motors
     setServoGoalSpeed(nServo, default_goal_speed[nServo-1]);
     makiGoalSpeed[nServo-1] = default_goal_speed[nServo-1];
     
     /*
     // KATE debugging
     // reset AX_ALARM_*
     ax12SetRegister(nServo, AX_ALARM_LED, 0);
     ax12SetRegister(nServo, AX_ALARM_SHUTDOWN, 0);
     */
   }
   
  commandString = String(" ");    // initalize

  resetMotorPositions();    // loads bioloid pose; previously undefined -- DO NOT REMOVE
  errorCheck_timer = millis();

  if (my_verbose_debug)  printServoInfoHeader();
  for (nServo=1; nServo <= SERVOCOUNT; nServo++)  {
    makiServoPos[nServo-1] = getServoPos(nServo);
    makiGoalPos[nServo-1] = makiServoPos[nServo-1];
    if (my_verbose_debug)  printServoInfo(nServo);
  }

  if (my_verbose_debug)  {
    Serial.print("\nError levels\n");
    Serial.print("\nERR_NONE = ");
    Serial.print(ERR_NONE);
    Serial.print(", ");
    Serial.print(ERR_NONE, BIN);
    Serial.print("\nERR_VOLTAGE = ");
    Serial.print(ERR_VOLTAGE);
    Serial.print(", ");
    Serial.print(ERR_VOLTAGE, BIN);
    Serial.print("\nERR_ANGLE_LIMIT = ");
    Serial.print(ERR_ANGLE_LIMIT);
    Serial.print(", ");
    Serial.print(ERR_ANGLE_LIMIT, BIN);    
    Serial.print("\nERR_OVERHEATING = ");
    Serial.print(ERR_OVERHEATING);
    Serial.print(", ");
    Serial.print(ERR_OVERHEATING, BIN);    
    Serial.print("\nERR_RANGE = ");
    Serial.print(ERR_RANGE);
    Serial.print(", ");
    Serial.print(ERR_RANGE, BIN);
    Serial.print("\nERR_CHECKSUM = ");
    Serial.print(ERR_CHECKSUM);
    Serial.print(", ");
    Serial.print(ERR_CHECKSUM, BIN);
    Serial.print("\nERR_OVERLOAD = ");
    Serial.print(ERR_OVERLOAD);
    Serial.print(", ");
    Serial.print(ERR_OVERLOAD, BIN);
    Serial.print("\nERR_INSTRUCTION = ");
    Serial.print(ERR_INSTRUCTION);
    Serial.print(", ");
    Serial.print(ERR_INSTRUCTION, BIN);
    
    Serial.print("\nStatus return level: (default 2; return for all commands) "); 
     for (nServo=1; nServo <= SERVOCOUNT; nServo++)  {
       Serial.print( ax12GetRegister(nServo, AX_RETURN_LEVEL, 1) );
       Serial.print(", ");
     }
     Serial.print("\n");
     Serial.flush();

    Serial.print("\nTemperature limit: (default 70) "); 
     for (nServo=1; nServo <= SERVOCOUNT; nServo++)  {
       Serial.print( ax12GetRegister(nServo, AX_LIMIT_TEMPERATURE, 1) );
       Serial.print(", ");
     }
     Serial.print("\n");
     Serial.flush();

  }

  if (my_verbose_debug)  {  
    printMainHelp();
    Serial.println("-- READY FOR COMMANDS --");
  } else  {
    readMotorValues(String(SC_GET_PP)); 
  }
}

/* ------------------------------------------------------------------------------------------------------------ */
void loop() {
  if (pp_flag && 
      (countAllMovingServos() == 0))  {
    pp_flag = false;
    readMotorValues(String(SC_GET_PP));
  }
  
  // make sure that periodically we're checking the error state of the motors
  if ((millis() - errorCheck_timer) > EC_timer_duration)  {
    readMotorValues(String(SC_GET_ER));
    errorCheck_timer = millis();
    
    //KATE debugging
    /*
    if (my_verbose_debug)  {
      printServoInfoHeader();
      for (int nServo=1; nServo <= SERVOCOUNT; nServo++)  {
        printServoInfo(nServo);
      }
    }
    */
  }
  
  //KATE debugging
  /*
  if (my_verbose_debug)  {
    int HT_torque_limit = ax12GetRegister(HT, AX_TORQUE_LIMIT_L, 2);
    if (ht_flag &&
        (HT_torque_limit == 0))  {
      Serial.print("\n***********************");
      printServoInfo(HT);
      Serial.print("***********************\n");
      Serial.flush();
      ht_flag = false;
    }
  }
  */
  
  if (Serial.available() <= 0)  {
    digitalWrite(USER_LED, LOW);   // turn the LED off by making the voltage LOW
    return;  // nothing to read, so return immediately
  }

  boolean break_flag = false;
  char newByte;
  while (Serial.available() > 0)  {
    digitalWrite(USER_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    
    break_flag = false;
    newByte = (char)Serial.read();    // read in the message one byte at a time
    
    if (newByte == 'CR')  {
      break_flag = true;
      if (my_verbose_debug)  Serial.println("*carriageReturn*");  // carriage return
    }
    else if (newByte == '\n')  {
      break_flag = true;
      if (my_verbose_debug)  Serial.println("*newline*");  // newline
    }
    else if (newByte == 'LF')  {
      break_flag = true;
      if (my_verbose_debug)  Serial.println("*linefeed*");  // linefeed == newline
    } else if (newByte  == TERM_CHAR)  {
      if (Serial.peek() == -1)  {
        break_flag = true;    // break at the termination char capital zed
//        mode = SERVO_DRIVER;
      } else  {
        commandString += newByte;
      }
    } else  {   
        commandString += newByte;      // append new byte to commandString
    }
    
    if (break_flag == true) {
      if (my_verbose_debug) {
        // DEBUGGING
        //Serial.println("");
        Serial.print("input:^");
        Serial.print(commandString);
        Serial.print("$\n");
        //Serial.println("");
      }

      commandString.trim();    // remove whitespace at the end
      commandString.replace(" ", "");  // remove spaces
      commandString.toUpperCase();    // make alphabetic characters uppercase; simpler to parse
      
      break;  // break on carriage  return, newline, linefeed, or the termination char capital zed
    } else if (Serial.peek() == -1)  {
      if (my_verbose_debug)  Serial.print(newByte);    // echo input back out to the serial monitor
      return;  // nothing more to read, but we shouldn't be processing commandString yet...
    } else  {}
  }  // end while (Serial.available() > 0)
 
   
  if (newByte == TERM_CHAR)     // yay! process this properly terminated message
  {
    parseServoDriverCommand(commandString);
    commandString = String();    // reset to empty string
    
  }
} // end loop()


/* --------------------------------------------------------------- */
//  UTILITY FUNCTIONS
/* --------------------------------------------------------------- */
void printMainHelp()  {
  if (!my_verbose_debug)  return;
  
  Serial.println("[SERVO DRIVER] Default command syntax:");
  Serial.println("NOTE: Z (capital zed)\tTerminal character for servo driver commands");
  Serial.println("");
  Serial.println("FMXZ\tget FEEDBACK of all servo MAXIMUM POSITION");
  Serial.println("FMNZ\tget FEEDBACK of all servo MINIMUM POSITION");
  Serial.println("FPPZ\tget FEEDBACK of all servo PRESENT POSITION");
  Serial.println("FGPZ\tget FEEDBACK of all servo GOAL POSITION");
  Serial.println("FPSZ\tget FEEDBACK of all servo PRESENT SPEED");
  Serial.println("FGSZ\tget FEEDBACK of all servo GOAL SPEED");
  Serial.println("FPTZ\tget FEEDBACK of all servo PRESENT TEMPERATURE (Celsius)");
  Serial.println("FPLZ\tget FEEDBACK of all servo PRESENT LOAD");
  Serial.println("FTLZ\tget FEEDBACK of all servo TORQUE LIMIT (percentage of MAX TORQUE)");
  Serial.println("FERZ\tget FEEDBACK of all servo ERROR STATE (AX_ALARM_LED)");
  Serial.println("FDPZ\tget FEEDBACK of all servo DEFAULT POSITION");
  Serial.println("FDSZ\tget FEEDBACK of all servo DEFAULT SPEED");
  Serial.println("");
  Serial.flush();
  
  Serial.println("mm{GP|GS}xxxIPTyyyyZ\tset GOAL POSITION or GOAL SPEED"); 
  Serial.println("\twhere xxx = [0, 1023], yyyy is INTERPOLATION TIME in milliseconds, and");
  Serial.println("\twhere mm = {LR, LL, EP, ET, HT, HP} means eyelid right, eyelid left, ");
  Serial.println("\teye pan, eye  tilt, head tilt, and head pan, respectively.");
  Serial.println("");
  Serial.flush();
  
  Serial.println("mmTLxxxxZ\tset TORQUE LIMIT (percentage of MAX TORQUE");
  Serial.println("\twhere xxx = [0, 1023]  default 1023; 0 is 0% and 1023 is 100%");
  Serial.println("\tNOTE: Use this if FTLZ indicates that a servo is at 0%; this will re-enable the servo.");
  Serial.println("\tWARNING: Before re-enabling, make sure that the servo's GOAL POSITION and PRESENT POSITION are the same;");
  Serial.println("\totherwise, servo will rapidly snap into place upon executing this servo command.");
  Serial.println("");
  Serial.flush();
  
}  // end printMainHelp


/* ----------------------------- */
void parseServoDriverCommand(String servoCommand) {
  if (my_verbose_debug)  {
    Serial.print("\nparseServoDriverCommand: ");
    Serial.print(servoCommand);
    Serial.print("\n");
    Serial.flush();
  }
  
  servoCommand.trim();    // remove whitespace at the end
  servoCommand.replace(" ", "");  // remove spaces
  servoCommand.toUpperCase();    // make alphabetic characters uppercase; simpler to parse
  
  String tmp_string = String(servoCommand);
  int index = 0;
  int target_motor = 0;
  boolean interpolation_flag = false;  
  unsigned int parsed_int = INVALID_INT;    // expected input [0, 1023]
  unsigned int ipt = INVALID_INT;    // milliseconds


  // parse left to right
  while (tmp_string.length() > 0)  {
    if (tmp_string.startsWith(SC_FEEDBACK))    // syntax for printing feedback
    { 
      index += 1;
      tmp_string = String(servoCommand.substring(index));
     
     // servo command format: Fxx, where xx = {PP, GP, MX, MN, PS, GS, PT, PL, TM, TL, TS, ER, DP, DS} 
     if (tmp_string.startsWith(SC_GET_PP)  ||
         tmp_string.startsWith(SC_GET_GP)  ||
         tmp_string.startsWith(SC_GET_MX)  ||
         tmp_string.startsWith(SC_GET_MN)  ||
         tmp_string.startsWith(SC_GET_PS)  ||
         tmp_string.startsWith(SC_GET_GS)  || 
         tmp_string.startsWith(SC_GET_PT)  ||
         tmp_string.startsWith(SC_GET_PL)  ||
         tmp_string.startsWith(SC_GET_TM)  ||
         tmp_string.startsWith(SC_GET_TL)  ||
         tmp_string.startsWith(SC_GET_TS)  ||
         tmp_string.startsWith(SC_GET_ER)  ||
         tmp_string.startsWith(SC_GET_DP)  ||
         tmp_string.startsWith(SC_GET_DS)
         ) {
        index += 2;
        String feedbackType = String(tmp_string.substring(0,2));
        readMotorValues(feedbackType);    // print current servo information to serial port output
     } 
     
    } else if (tmp_string.startsWith("LL")) {
      index += 2;
      target_motor = LL;
      
    } else if (tmp_string.startsWith("LR")) {
      index += 2;
      target_motor = LR;
      
    } else if (tmp_string.startsWith("EP"))  {
      index += 2;
      target_motor = EP;
      
    } else if (tmp_string.startsWith("ET")) {
      index += 2;
      target_motor = ET;
      
    } else if (tmp_string.startsWith("HP")) {
      index += 2;
      target_motor = HP;
      
    } else if (tmp_string.startsWith("HT")) {
      index += 2;
      target_motor = HT;
  
    } else if (tmp_string.startsWith(SC_SET_GP) ||
                  tmp_string.startsWith(SC_SET_GS))  {
      index += 2;
      // servo command format: mmGPxxxx or mmGSxxxx
      // deal smartly with string length and incrementing index
      unsigned int tmp_i = parseSCDIntIndex(tmp_string.substring(2));
      parsed_int = (tmp_string.substring(2,2+tmp_i+1)).toInt();
      //Serial.println(parsed_int);    //debugging
      index += tmp_i;

      if (tmp_string.startsWith(SC_SET_GP))  {
        if (setServoGoalPos(target_motor, parsed_int))    // setServoGoalPos error checks motor and position
          interpolation_flag = true;
      } else if (tmp_string.startsWith(SC_SET_GS)) {
        setServoGoalSpeed(target_motor, parsed_int);    // setServoGoalSpeed error checks motors and speed
        makiGoalSpeed[target_motor-1] = getServoGoalSpeed(target_motor);
      } else {
        // shouldn't get here
      }
                 
    } else  if (tmp_string.startsWith(SC_SET_IPT))  {
      index += 3;
      // servo command format: IPTxxxx
      // deal smartly with string length and incrementing index
      unsigned int tmp_i = parseSCDIntIndex(tmp_string.substring(3));
      ipt = (tmp_string.substring(3,3+tmp_i+1)).toInt();    // milliseconds
      //Serial.println(ipt);    // debugging
      index += tmp_i;      

    } else if (tmp_string.startsWith(SC_SET_TM) ||
                tmp_string.startsWith(SC_SET_TL) ||
                tmp_string.startsWith(SC_SET_TS)
                )  {
      index += 2;
      // servo command format: mmTMxxxx, mmTLxxxx, or mmTSx
      // deal smartly with string length and incrementing index
      unsigned int tmp_i = parseSCDIntIndex(tmp_string.substring(2));
      parsed_int = (tmp_string.substring(2,2+tmp_i+1)).toInt();
      //Serial.println(parsed_int);    //debugging
      index += tmp_i;              
      
      // KATE
      if (tmp_string.startsWith(SC_SET_TM))  {
        // Note: This value MAX TORQUE exists in EEPROM and persists when power is removed; default 1023
        setServoMaxTorque(target_motor, parsed_int);
      } else if (tmp_string.startsWith(SC_SET_TL))  {
        // Note: If set to 0 by ALARM SHUTDOWN, servo won't move until TORQUE LIMIT set to another value[1,1023]; default MAX TORQUE
        setServoTorqueLimit(target_motor, parsed_int);
      } else if (tmp_string.startsWith(SC_SET_TS))  {
      // TORQUE ENABLE. Note: Boolean [0, 1]; default 1. Doesn't appear to disable servo's movement when set to 0
        setServoTorqueState(target_motor, parsed_int);
      } else {
        // shouldn't get here
      }
      
    } else {
      if (my_verbose_debug)  {
        Serial.println("!!!!! Can't parse remaining servoCommand: ");
        Serial.println(tmp_string);
      }
      tmp_string = String("");
      break;
    }
    
    tmp_string = String(servoCommand.substring(index));
  }  // end while
  
  if (interpolation_flag)  {
    interpolation_flag = false;
    pp_flag = true;
    
    if (my_verbose_debug)  {
      tmp_string = String("makiServoPos[]={");
      for (index=1; index<=SERVOCOUNT; index++)  {
        tmp_string += String(makiServoPos[index-1]) + ((index<SERVOCOUNT)?':':'}');
      }
      Serial.println(tmp_string);
      Serial.flush();
      tmp_string = String("makiGoalPos[]={");
      for (index=1; index<=SERVOCOUNT; index++)  {
        tmp_string += String(makiGoalPos[index-1]) + ((index<SERVOCOUNT)?':':'}');
      }
      Serial.println(tmp_string);
      Serial.flush();
      tmp_string = String("makiGoalSpeed[]={");
      for (index=1; index<=SERVOCOUNT; index++)  {
        tmp_string += String(makiGoalSpeed[index-1]) + ((index<SERVOCOUNT)?':':'}');
      }
      Serial.println(tmp_string);
      Serial.flush();
    }
    
    if ((ipt != INVALID_INT) && (ipt > 0))  {
      // adjust servo goal speeds such that all movement begins and ends at the same time
      float c;  
      int delta = 0;
      for (index=1; index<=SERVOCOUNT; index++)  {
        delta = makiGoalPos[index-1] - getServoPos(index);
        if (delta != 0)  {
          ipt_flag = true;
          // c = (abs(GP-PP))/(2*seconds) + 0.5
          c = (abs(delta)) / (((float)ipt/1000) * 2);
          c += 0.5;      // round implicitly
          //Serial.println(c);    // debugging
          setServoGoalSpeed(index, (int)c);    // need to reset GS to makiGoalSpeed after the movement has finished
        }
      }  // end  for (index=1; index<=SERVOCOUNT; index++)
      writeGPSync();    // TODO: should this be a blocking action instead??
    } else  {
      writeGPSync();    // write makiGoalPos to all servos at the same time, but don't wait for resulting movement
    }
     // need to reset GS to makiGoalSpeed after the movement has finished
  
    for (index=1; index<=SERVOCOUNT; index++)  {
       setServoGoalSpeed(index, default_goal_speed[index-1]);
    }
    ipt_flag = false;
    
  } 
}  // end parseServoDriverCommand

unsigned int parseSCDIntIndex(String servoCommand) {
  unsigned int i=0;
  for (i=0; i<servoCommand.length(); i++)  {
    if (isDigit(servoCommand[i]) == false)  break;
  }
  return i;
}  // end parseSCDIntIndex

/* ----------------------------- */
// write to all servos at the same time with the goal position (BROADCAST)
// returns immediately; does not block and wait for completion of movement
void writeGPSync()  {
  // based on BioloidController::writePose()
  // For communication 1.0 protocol, see http://support.robotis.com/en/product/dynamixel/communication/dxl_packet.htm
  
  int temp;
  int length = 4 + (SERVOCOUNT * 3);   // 3 = id + pos(2byte)
  int checksum = 254 + length + AX_SYNC_WRITE + 2 + AX_GOAL_POSITION_L;
  setTXall();       // declare transmission to all servos
  ax12write(0xFF);
  ax12write(0xFF);  // header
  ax12write(0xFE);  // 0xfe is 254 in decimal, thus this is a broadcast message and not status packet in return
  ax12write(length);  // length
  ax12write(AX_SYNC_WRITE);    //0x83  // instruction
  // parameters
  ax12write(AX_GOAL_POSITION_L);
  ax12write(2);
  for (int nServo=1; nServo<=SERVOCOUNT; nServo++)
  {
      temp = makiGoalPos[nServo-1];  
      checksum += (temp&0xff) + (temp>>8) + nServo;
      ax12write(nServo);     // servo id
      ax12write(temp&0xff);  // GP
      ax12write(temp>>8);    // GP
  } 
  ax12write(0xff - (checksum % 256));  // checksum
  setRX(0);  // declare transmission to specific servo complete
}  // end  writeGPSync

void writeGPSyncBlocking()  {
  writeGPSyncBlocking(interpolation_time);    // use existing value for interpolation_time = 500 ms
}  // end  writeGPSyncBlocking

void writeGPSyncBlocking(unsigned int t)  {
  writeGPSync();
//  delay(100);
  
  // TODO: Do this better!!!
  delay(t);
}  // end  writeGPSyncBlocking

/* ----------------------------- */
// Based on Andre's FullServoKeeponNewBop.ino
void readMotorValues()  {
  readMotorValues("");
}  // end readMotorValues

// Based on Andre's FullServoKeeponNewBop.ino
void readMotorValues(String feedbackType)  {
  int read_val = INVALID_INT;
  
  if ((feedbackType.length() == 0) || feedbackType.equalsIgnoreCase(""))  {
    feedbackType = String(SC_GET_PP);
  }
  String state = String(feedbackType);
  
  for (int nServo=1; nServo<=SERVOCOUNT; nServo++)   {
    read_val = INVALID_INT;  // reset
    
    if (feedbackType.equalsIgnoreCase(SC_GET_PP))  {
      read_val = getServoPos(nServo);        // ax12GetRegister(nServo, AX_PRESENT_POSITION_L, 2)
      makiServoPos[nServo-1] = read_val;   

    } else if (feedbackType.equalsIgnoreCase(SC_GET_GP))  {
      read_val = ax12GetRegister(nServo, AX_GOAL_POSITION_L, 2);   
      
    } else if (feedbackType.equalsIgnoreCase(SC_GET_PS))  {
      read_val = ax12GetRegister(nServo, AX_PRESENT_SPEED_L, 2);    // getServoSpeed
      
    } else if (feedbackType.equalsIgnoreCase(SC_GET_GS))  {
      read_val = getServoGoalSpeed(nServo);
      makiGoalSpeed[nServo-1] = read_val;
      
    } else if (feedbackType.equalsIgnoreCase(SC_GET_PT))  {
      read_val = ax12GetRegister(nServo, AX_PRESENT_TEMPERATURE, 1);
      
    } else if (feedbackType.equalsIgnoreCase(SC_GET_PL))  {
      read_val = ax12GetRegister(nServo, AX_PRESENT_LOAD_L, 2);
 
    } else if (feedbackType.equalsIgnoreCase(SC_GET_TM))  {    //KATE
      read_val = ax12GetRegister(nServo, AX_MAX_TORQUE_L, 2);

    } else if (feedbackType.equalsIgnoreCase(SC_GET_TL))  {    //KATE
      read_val = ax12GetRegister(nServo, AX_TORQUE_LIMIT_L, 2);
      
    } else if (feedbackType.equalsIgnoreCase(SC_GET_TS))  {    //KATE
      read_val = ax12GetRegister(nServo, AX_TORQUE_ENABLE, 1);
      
    } else if (feedbackType.equalsIgnoreCase(SC_GET_MX))  {
      read_val = max_servo_pos[nServo-1];
      
    } else if (feedbackType.equalsIgnoreCase(SC_GET_MN))  {
       read_val = min_servo_pos[nServo-1];
       
    } else if (feedbackType.equalsIgnoreCase(SC_GET_ER))  {
      read_val = errorCheckMotor(nServo, false);    // surpress printing the error string
      errorCheck_timer = millis();                             // reset timer

      if (my_verbose_debug)  {
        Serial.print("\nreadMotorValues: ");
        Serial.print(read_val, BIN);
        Serial.print("\n");
        Serial.flush();
      } 
      
    } else if (feedbackType.equalsIgnoreCase(SC_GET_DP))  {
        read_val = neutral_servo_pos[nServo];
        
    } else if (feedbackType.equalsIgnoreCase(SC_GET_DS))  {
        read_val = default_goal_speed[nServo - 1];
          
    } else  {
      // shouldn't get here
      read_val = INVALID_INT;
    }
    
    if ((read_val >= 0) && (read_val != INVALID_INT))  {
      state += String(read_val);
    } else  {
      state += String(INVALID_INT);  // for simplicity parsing on the other side, we don't want to send value of -1
    }
    state += ((nServo<SERVOCOUNT)?':':';');
  }  // end  for (int nServo=1; nServo<=SERVOCOUNT; nServo++)
  
  int str_len = state.length() + 1; 
  // Prepare the character array (the buffer) 
  char char_array[str_len];
  // Copy it over 
  state.toCharArray(char_array, str_len);
  Serial.write(char_array);
  Serial.flush();
}  // end readMotorValues
 
/* ----------------------------- */ 
// take boolean flag for printing
int getServoPos(int nServo, boolean print_flag) {   
  int pos = INVALID_INT;
  
  if (validServo(nServo))  {
    pos = ax12GetRegister(nServo, AX_PRESENT_POSITION_L, 2);            // ax12.h; shorthand for ax12GetRegister(nServo, AX_PRESENT_POSITION_L, 2)
    if (print_flag) {   
      Serial.print("(Servo ID: ");    
      Serial.print(nServo);   
      Serial.print(" ), Servo Position: ");   
      Serial.println(pos); 
    }
    return pos;
  }
  Serial.flush();
  return pos;
}

int getServoPos(int nServo) {     
  return getServoPos(nServo, false);     
}                                

int printServoPos(int nServo)  {  
  return getServoPos(nServo, true);      
}  

/* ----------------------------- */ 
boolean validServo(int nServo)  {
  if ((nServo > 0) && (nServo <= SERVOCOUNT))  {
    return true;
  } else  {
    if (my_verbose_debug)  {
      Serial.print("\n!!! Invalid servo: ");
      Serial.print(nServo);
    }
    return false;
  }
}  // end validServo

/* ----------------------------- */ 
boolean setServoGoalPos(int nServo, int absPos)  {
  if (validServo(nServo))  {    
    // don't exceed MAKI's servo position software limits
    if (nServo != HP)  {
      if (absPos < min_servo_pos[nServo-1])  absPos=min_servo_pos[nServo-1];
      if (absPos > max_servo_pos[nServo-1])  absPos=max_servo_pos[nServo-1];
    } else  {
      if (absPos < MIN_DYN_POS)  absPos=MIN_DYN_POS;
      if (absPos > MAX_DYN_POS)  absPos=MAX_DYN_POS;
    }
    
    makiGoalPos[nServo-1] = absPos;
    return true;
  } else {
    return false;
  }
}  // end setServoGoalPos

int getServoGoalPos(int nServo)  {
  if (validServo(nServo))  {
    return makiGoalPos[nServo-1];
  } else  {
    return (int)INVALID_INT;
  }
}  // end  getServoGoalPos


/* ----------------------------- */
int getServoGoalSpeed(int nServo)  {
  int ret = ax12GetRegister(nServo, AX_GOAL_SPEED_L, 2);
  int c = makiGoalSpeed[nServo-1];
  
  if (ret == c)  {
    return ret;
  } else if (ret > 0)  {
    return c;    // THIS CONDITION IS A SIDE EFFECT OF SERVO COMMAND WITH IPT VALUE
  } else  {
    return ret;  // ret = -1
  }
}  // getServoGoalSpeed

boolean setServoGoalSpeed(int nServo, int v)  {
  if (validServo(nServo))  {
    int goal_v = getServoGoalSpeed(nServo);
    if (goal_v == v)  return false;
  
    if (my_verbose_debug)  {
      if (goal_v == 0)  {
        Serial.print("\ngoal speed: 0=motor max/no speed control");
      } else  {
        Serial.print("\ngoal speed: ");
        Serial.print(goal_v);
      }
    }  // end if (my_verbose_debug)
  
    if ((v >= 0) && (v <= 1023)) {   
      ax12SetRegister2(nServo, AX_GOAL_SPEED_L, v);
      //makiGoalSpeed[nServo-1] = v;     // COMMENTED OUT INTENTIONALLY; ktsui 2015-07-21
      delay(100);  // needs time to write to value to register
      return true;
    } else  {
      if (my_verbose_debug)  {
        Serial.println("[setServoGoalSpeed()] Invalid speed; range = [0=max, 1,..., 1023]"); 
      } 
      return false;
    }
  
    if (my_verbose_debug)  {
      goal_v = getServoGoalSpeed(nServo);   
      Serial.print("; new goal speed: ");
      Serial.print(goal_v);
    
      if (goal_v != 0)  {
        Serial.print(" (");
        goal_v = convertToRPM(goal_v);    
        Serial.print(goal_v);
        Serial.print(" RPM)\n");
      } else  {
        Serial.print("=motor max/no speed control\n");
      }
    }  // end if (my_verbose_debug)
    
  } else  {    // not valid servo
    return false;
  }
}  // end setServoSpeed

/* ----------------------------- */
// TODO: Refactor the repeated code in this section
// KATE
// Note: This value MAX TORQUE exists in EEPROM and persists when power is removed; default 1023
boolean setServoMaxTorque(int nServo, int v)  {
  if (validServo(nServo))  {
    if (my_verbose_debug)  {
      Serial.print("BEFORE: ");
      readMotorValues(String(SC_GET_TM));  // debugging
      Serial.flush();
    }
    
    ax12SetRegister2(nServo, AX_MAX_TORQUE_L, v);
    delay(100);  // needs time to write to value to register
    
    if (my_verbose_debug)  {
      Serial.print("AFTER: ");
      Serial.print("Servo ");
      Serial.print(nServo);
      Serial.print(" changed to ");
      Serial.print( ax12GetRegister(nServo, AX_MAX_TORQUE_L, 2) );
      Serial.print("; desired ");
      Serial.print(v);
      
      readMotorValues(String(SC_GET_TM));  // debugging
      Serial.flush();
    }
    
    return true;
  } else {
    return false;
  }
}  // end  setServoMaxTorque

// Note: If set to 0 by ALARM SHUTDOWN, servo won't move until TORQUE LIMIT set to another value[1,1023]; default MAX TORQUE
boolean setServoTorqueLimit(int nServo, int v)  {
  if (validServo(nServo))  {
    if (my_verbose_debug)  {
      Serial.print("BEFORE: ");
      readMotorValues(String(SC_GET_TL));  // debugging
      Serial.flush();
    }
    
    ax12SetRegister2(nServo, AX_TORQUE_LIMIT_L, v);
    delay(100);  // needs time to write to value to register
    
    if (my_verbose_debug)  {
      Serial.print("AFTER: ");
      Serial.print("Servo ");
      Serial.print(nServo);
      Serial.print(" changed to ");
      Serial.print( ax12GetRegister(nServo, AX_TORQUE_LIMIT_L, 2) );
      Serial.print("; desired ");
      Serial.print(v);
      
      readMotorValues(String(SC_GET_TL));  // debugging
      Serial.flush();
    }
    
    return true;
  } else {
    return false;
  }
}  // end  setServoTorqueLimit

// both enable and disable a specific motor's torque
// TORQUE ENABLE. Note: Boolean [0, 1]; default 1. Doesn't appear to disable servo's movement when set to 0
boolean setServoTorqueState(int nServo, int v)  {
  if (validServo(nServo))  {
    if (my_verbose_debug)  {
      Serial.print("BEFORE: ");
      readMotorValues(String(SC_GET_TS));  // debugging
      Serial.flush();
    }
      
    ax12SetRegister(nServo, AX_TORQUE_ENABLE, v);
    delay(100);  // needs time to write to value to register

    if (my_verbose_debug)  {
      Serial.print("AFTER: ");
      Serial.print("Servo ");
      Serial.print(nServo);
      Serial.print(" changed to ");
      Serial.print( ax12GetRegister(nServo, AX_TORQUE_ENABLE, 1) );
      Serial.print("; desired ");
      Serial.print(v);
      
      readMotorValues(String(SC_GET_TS));  // debugging
      Serial.flush();
    }

    return true;
  } else {
    return false;
  }
}  // end  setServoTorqueState

/* ----------------------------- */
// how many servos are currently moving?
int countAllMovingServos()  {
  int ret = 0;
  for (int nServo = 1; nServo <= SERVOCOUNT; nServo++)  {
    if ((ax12GetRegister(nServo, AX_MOVING, 1) == 1) ||
        (ax12GetRegister(nServo, AX_PRESENT_SPEED_L, 2) > 0))  {
      ret++;
    }
  }
  return ret;
}  // end countAllMovingServos

/* ----------------------------- */
void resetMotorPositions()  {
  // see http://www.arduino.cc/en/Reference/PROGMEM
  for (int nServo=1; nServo<=SERVOCOUNT; nServo++)  {
    setServoGoalPos(nServo, neutral_servo_pos[nServo]);
  }
  writeGPSyncBlocking(2000);    // give sufficient time to reset
  
  if (my_verbose_debug)  {
    Serial.println("\n---- Servo positions reset! ----");
  }
}  // end resetMotorPositions 
 
/* ----------------------------- */
int convertToRPM(int speed) {     
  float rpm = (float)speed * 0.111;   
  rpm += 0.5;   // hack for rounding    
  return (int)rpm;    
}     // end convertToRPM

/* -------------------------- */
void printServoInfoHeader(int nServo)  { 
  Serial.print("==== ServoID: ");
  Serial.print(nServo);
  Serial.println(" ====");
  printServoInfoHeader();
}  // end printServoInfoHeader

void printServoInfoHeader()  {
  Serial.println("====================");
  
 String detailedServoHeader = String();
  detailedServoHeader += "SERVO ID\t";
  detailedServoHeader += "ANG LIM\t";
  detailedServoHeader += "SPEED\t";
  detailedServoHeader += "\t\tPOSITION\t";
  detailedServoHeader += "TORQUE\t";
  detailedServoHeader += "\tVOLTAGE\t";
  detailedServoHeader += "\tTEMPERATURE\t";
  detailedServoHeader += "LED \ ERROR\t";
  Serial.println(detailedServoHeader);
  
  detailedServoHeader = String("\t");
  detailedServoHeader += "(cw, ccw)\t";
  detailedServoHeader += "(goal (RPM), cur (RPM))\t\t";
  detailedServoHeader += "(goal, cur)\t";
  detailedServoHeader += "(p?, max, lim, cur (%))\t";
  detailedServoHeader += "(max(dwn, up), cur)\t";
  detailedServoHeader += "(max, cur)\t\t";
  detailedServoHeader += "p?, binary error code\t";
  Serial.println(detailedServoHeader);
}  // end printServoInfoHeader

/* ----------------------------- */
// NOTE: for message structure, see http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm
void printServoInfo(int nServo)  {
  if (ax12GetRegister(nServo, AX_ID, 1) != nServo)  {
    return;
  }
  
  String detailedServoInfo = String();
  
  //WHEEL if (0,0); JOINT otherwise
  detailedServoInfo += String(nServo) + "\t";
  detailedServoInfo += "(" + String(ax12GetRegister(nServo, AX_CW_ANGLE_LIMIT_L, 2)) + "," 
                                          + String(ax12GetRegister(nServo, AX_CCW_ANGLE_LIMIT_L, 2)) + ")\t";
  
  /*  It is a moving speed to Goal Position.
  JOINT MODE
  0~1023 (0X3FF) can be used, and the unit is about 0.111rpm.
  If it is set to 0, it means the maximum rpm of the motor is used without controlling the speed.
  If it is 1023, it is about 114rpm.  */
  int my_speed = ax12GetRegister(nServo, AX_GOAL_SPEED_L, 2);
  int my_rpm = convertToRPM(my_speed);           
  detailedServoInfo += "(" + String(my_speed) + " (" 
                                  + String(my_rpm) + "rpm), ";
  
  my_speed = ax12GetRegister(nServo, AX_PRESENT_SPEED_L, 2);
  boolean cw_flag = false;

  if ((my_speed & DIRECTION_BIT) == DIRECTION_BIT)  {     
    cw_flag = true;     
    my_speed = my_speed & ~DIRECTION_BIT;    
  }              
  my_rpm = convertToRPM(my_speed); 
  detailedServoInfo += String(my_speed) + " (" + String(my_rpm) + "rpm ";
  if (!cw_flag)  detailedServoInfo += "c";
  detailedServoInfo += "cw))\t";
  
  /*  It is a position value of destination.
  /*  0 to 1023 (0x3FF) is available.  The unit is 0.29 degree.
  /*  If Goal Position is out of the range, Angle Limit Error Bit (Bit1) of Status Packet is returned as ‘1’ and Alarm is triggered as set in Alarm LED/Shutdown. */
  detailedServoInfo += "\t(" + String(ax12GetRegister(nServo, AX_GOAL_POSITION_L, 2)) + ","
                                          + String(ax12GetRegister(nServo, AX_PRESENT_POSITION_L, 2)) + ")\t";

  // torque and load
  detailedServoInfo += "(" + String(ax12GetRegister(nServo, AX_TORQUE_ENABLE, 1)) + "," 
                                          + String(ax12GetRegister(nServo, AX_MAX_TORQUE_L, 2)) + ","
                                          + String(ax12GetRegister(nServo, AX_TORQUE_LIMIT_L, 2)) +  ",";
  int my_load = ax12GetRegister(nServo, AX_PRESENT_LOAD_L, 2);    
  cw_flag = false;   
  if ((my_load & DIRECTION_BIT) == DIRECTION_BIT)  {     
    cw_flag = true;     
    my_load = my_load & ~DIRECTION_BIT;     
  }               
  detailedServoInfo += String(my_load) + " ";  
  if (!cw_flag)  detailedServoInfo += "c";    
  detailedServoInfo += "cw (";    
  my_load  = (int) round( (float)my_load / (float)MAX_POS );   
  detailedServoInfo += String(my_load) + "%))\t";   

  // voltage
  detailedServoInfo += "((" + String(ax12GetRegister(nServo, AX_DOWN_LIMIT_VOLTAGE, 1)) + "," 
                                          + String(ax12GetRegister(nServo, AX_UP_LIMIT_VOLTAGE, 1)) + "), " 
                                          + String(ax12GetRegister(nServo, AX_PRESENT_VOLTAGE, 1)) + ")\t";

  // temperature in Celcius
  detailedServoInfo += "(" + String(ax12GetRegister(nServo, AX_LIMIT_TEMPERATURE, 1)) + ","
                                          + String(ax12GetRegister(nServo, AX_PRESENT_TEMPERATURE, 1)) + ")\t";

  detailedServoInfo += "\t" + String(ax12GetRegister(nServo, AX_LED, 1))  + ", ";

  Serial.print(detailedServoInfo);                                          
  Serial.print(errorCheckMotor(nServo, true), BIN);
  Serial.print("\n");
  Serial.flush();
}  // end printServoInfo

/* ----------------------------- */
int errorCheckMotor(int nServo, boolean PE)  {
  if (ax12GetRegister(nServo, AX_ID, 1) != nServo)  {
    return (int)INVALID_INT;
  }
    
  // ax12GetLastError();  // doesn't retain which motor yielded error
                          // value is only populated after calling ax12GetRegister()
                          
  int my_error = (int)INVALID_INT;
  if (ax12GetRegister(nServo, AX_LED, 1) == 1)  {
    my_error = ax12GetRegister(nServo, AX_ALARM_LED, 1);
  } else  {
    my_error = ax12GetRegister(nServo, AX_ALARM_SHUTDOWN, 1);
  }

  // 2016-01-29, ktsui: Currently unsure if the Dynamixel AX-12 servos actually update the
  // values for the AX_ALARM_LED and AX_ALARM_SHUTDOWN registers

  // KATE: debugging
  if (my_verbose_debug)  {
    Serial.print("** Servo ");
    Serial.print(nServo);
    Serial.print("; AX_LED: ");
    Serial.print(ax12GetRegister(nServo, AX_LED, 1));
    Serial.print("; AX_ALARM_LED: ");
    Serial.print(ax12GetRegister(nServo, AX_ALARM_LED, 1), BIN);
    Serial.print("\tAX_ALARM_SHUTDOWN: ");
    Serial.print(ax12GetRegister(nServo, AX_ALARM_SHUTDOWN, 1), BIN);
    Serial.print(" **\n");
    Serial.flush();
  }

  // explicitly check to see if the TORQUE LIMIT is 0 as a result of AX_ALARM_SHUTDOWN
  // see section TORQUE LIMIT at http://support.robotis.com/en/techsupport_eng.htm#product/dynamixel/ax_series/dxl_ax_actuator.htm
  if ( (ax12GetRegister(nServo, AX_PRESENT_LOAD_L, 2) == 0) &&
      (ax12GetRegister(nServo, AX_TORQUE_LIMIT_L, 2) == 0) )  {
    if (my_error == (int)INVALID_INT)  {
      my_error = ERR_OVERLOAD;
    } else  {
      my_error = my_error | ERR_OVERLOAD;  // bitwise OR
    }
    if (my_verbose_debug)  {
      Serial.print("AFTER TORQUE LIMIT check: my_error = ");
      Serial.print(my_error);
      Serial.flush();
    }
  }

  // explicitly check TEMPERATURE
  // When the internal temperature is out of the range of operating temperature set in the Control Table
  /*
  if ( ax12GetRegister(nServo, AX_PRESENT_TEMPERATURE, 1) >=
        ax12GetRegister(nServo, AX_LIMIT_TEMPERATURE, 1) )  {  
  */
  int max_temp = ax12GetRegister(nServo, AX_LIMIT_TEMPERATURE, 1);
  int my_temp = ax12GetRegister(nServo, AX_PRESENT_TEMPERATURE, 1);
      
  if (my_temp >= max_temp)  {
    // KATE debugging
    if (my_verbose_debug)  {
      Serial.print("\nTEMP: my_temp=");
      Serial.print(my_temp);
      Serial.print("\tmax_temp=");
      Serial.print(max_temp);
      Serial.print("\n");
      Serial.flush();
    }
    
    if (my_error == (int)INVALID_INT)  {
      my_error = ERR_OVERHEATING;
    } else  {
      my_error = my_error | ERR_OVERHEATING;
    }
    if (my_verbose_debug)  {
      Serial.print("AFTER TEMPERATURE check: my_error = ");
      Serial.print(my_error);
      Serial.flush();
    }
  }


  if (my_error == ERR_NONE) {
    return ERR_NONE;
    
  } else {
    boolean print_errorString = false;
    String errorString = String("ERROR! ");
    errorString += "ServoID " + String(nServo) + ": ";
  
    // TODO: REWRITE THESE AS BITSHIFT FOR FASTER COMPUTATION
    if ((my_error & ERR_OVERLOAD) == ERR_OVERLOAD)  {
      /*
      // When the current load cannot be controlled with the set maximum torque according to documentation
      if (ax12GetRegister(nServo, AX_TORQUE_LIMIT_L, 2) == 0)  {
        errorString += "ERR_OVERLOAD ";
        print_errorString = true;
      } else  {
        //Serial.println(my_error, BIN);    // debugging
        my_error = my_error & ~ERR_OVERLOAD;
        //Serial.println(my_error, BIN);    // debugging
        ax12SetRegister(nServo, AX_ALARM_LED, my_error);
      }
      */
      errorString += "ERR_OVERLOAD ";
      print_errorString = true;
    }

    if ((my_error & ERR_ANGLE_LIMIT) == ERR_ANGLE_LIMIT) {
      // When Goal Position is written with the value that is not between CW Angle Limit and CCW Angle Limit
      errorString += "ERR_ANGLE_LIMIT ";
      print_errorString = true;
    }

    if ((my_error & ERR_OVERHEATING) == ERR_OVERHEATING) {
      /*
      // When the internal temperature is out of the range of operating temperature set in the Control Table
      int max_temp = ax12GetRegister(nServo, AX_LIMIT_TEMPERATURE, 1);
      int my_temp = ax12GetRegister(nServo, AX_PRESENT_TEMPERATURE, 1);
      
      if (my_temp >= max_temp)  {
        errorString += "ERR_OVERHEATING ";
        print_errorString = true;
      } else  {
        //Serial.println(my_error, BIN);    // debugging
        my_error = my_error & ~ERR_OVERHEATING;
        //Serial.println(my_error, BIN);    // debugging
        ax12SetRegister(nServo, AX_ALARM_LED, my_error);
      }
      */
      errorString += "ERR_OVERHEATING";
      print_errorString = true;
    }
    
    if ((my_error & ERR_VOLTAGE) == ERR_VOLTAGE) {
      // When the applied voltage is out of the range of operating voltage set in the Control Table
      errorString += "ERR_VOLTAGE ";
      print_errorString = true;
    }
    
    if ((my_error & ERR_RANGE) == ERR_RANGE) {
      // When the command is given beyond the range of usage
      errorString += "ERR_RANGE ";
      print_errorString = true;
    }
    
    if ((my_error & ERR_CHECKSUM) == ERR_CHECKSUM) {
      // When the Checksum of the transmitted Instruction Packet is invalid
      errorString += "ERR_CHECKSUM ";
      print_errorString = true;
    }
    
    if ((my_error & ERR_INSTRUCTION) == ERR_INSTRUCTION) {
      // When undefined Instruction is transmitted or the Action command is delivered without the reg_write command
      errorString += "ERR_INSTRUCTION ";
      print_errorString = true;
    } 
    
    if (print_errorString && PE)  Serial.print(errorString);
  }
  return my_error;
} // end errorCheckMotor
