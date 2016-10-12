/* ========================================================================== */
/*  MAKIv1.4 Controller
/*  v 0.3
/*  
/*  Listens for serial port input in the following forms:
/*       F{MX|MN|PP|GP|PS|GS|PT|PL|TL|ER|DP|DS}Z
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
/*  returns results in the form {MX|MN|PP|GP|PS|GS|PT|PL|TL|ER|DP|DS}xxx:xxx:xxx:xxx:xxx:xxx;
/*  xxx = [0, 1023]
/*
/*       mm{GP|GS}xxx{IPT}yyyyyZ
/*  where mm = {LR, LL, EP, ET, HT, HP} means eyelid right, eyelid left, eye pan, eye  tilt, head tilt, and head pan, respectively,
/*  xxx = [0, 1023], and yyyyy in milliseconds
/*       HPGP0400Z    // set HEAD_PAN servo to the GOAL POSITION 400
/*       EPGS0100Z    // set EYE_PAN servo to the GOAL SPEED 100
/*       HPGP768GS51Z  // set HEAD_PAN servo to the GOAL POSITION 768 with GOAL SPEED of 51
/*       HPGP512HTGP512IPT2500Z    // set HEAD_PAN servo to the GOAL POSITION of 512, HEAD_TILT servo to 512, and 
/*                                 // locally adjust each servo's GOAL SPEED such that both movements take 2.5 seconds
/*
/*       mmTLxxxxZ
/*  where xxx = [0, 1023]  default 1023; 0 is 0% and 1023 is 100%
/*       HTTL1023Z  // set HEAD_TILT servo to TORQUE LIMIT 1023, rather 100% of MAX TORQUE
/*                  // NOTE: Use this if FTLZ indicates that a servo is at 0%; this will re-enable the servo
/*                  // WARNING: Before re-enabling, make sure that the servo's GOAL POSITION and PRESENT POSITION are the same;
/*                  // otherwise, servo will rapidly snap into place upon executing this servo command
/*
/*  RESETZ to request reset to default positions and speeds
/*
/*  Original Authors:
/*    Kate Tsui, Drew O'Donnell

/*  This version written by Acshi Haggenmiller
/*
/*  Updated: Oct 8 2016
/*-
/*  Acknowledgements:
/*  * Andre Pereira (FullServoKeeponNewBop.ino)
/*  * AXTurretTest.ino
/*  * http://forums.trossenrobotics.com/showthread.php?4434-Controlling-AX-or-RX-servos-using-arbotix&p=44430#post44430
/*  * http://support.robotis.com/en/techsupport_eng.htm#product/dynamixel/ax_series/dxl_ax_actuator.htm
/* ========================================================================== */

#include <Streaming.h>
#include <ax12.h>
#include "MAKIv14.h"

#define INVALID_INT 9999
#define COMP_BAUD_RATE 115200
#define DYN_BAUD_RATE 1000000 // 1Mbps

int lastErrorCheckMillis = 0;

char receiveBuffer[100];
unsigned int receiveBufferI = 0;

// User set or default goal speeds
// When using the "IPT"/MOVEMENT_TIME we change goal speeds
// but should set them back when those motions are complete
int userGoalSpeed[SERVO_COUNT];
bool hasTemporaryGoalSpeed[SERVO_COUNT];
bool performingMovement = false;

bool removeHeadTiltTorqueAfterMove = false;

void doReset() {
    //Serial << "Resetting goal speeds and positions\n";
    for (int i = 1; i <= SERVO_COUNT; i++) {
        dxlSetGoalSpeed(i, default_goal_speed[i - 1]);
        delayMicroseconds(100);
        dxlSetGoalPosition(i, default_servo_pos[i - 1]);
        delayMicroseconds(100);
        
        userGoalSpeed[i - 1] = default_goal_speed[i - 1];
        hasTemporaryGoalSpeed[i - 1] = false;
    }

    performingMovement = true;
    // Default to having the head tilt motor torque disabled for safety
    removeHeadTiltTorqueAfterMove = true;
}

void setup() {
    Serial.begin(COMP_BAUD_RATE);
    dxlInit(DYN_BAUD_RATE);
    // minimal delay for status packet return
    dxlSetReturnDelayTime(DXL_BROADCAST, 1);
    delayMicroseconds(100);

    for (int i = 1; i <= SERVO_COUNT; i++) {
        // These are stored in EEPROM, so check before saving to keep it from wearing down
        if (dxlGetMode(i) != 1) {
            axSetJointMode(i);
            delayMicroseconds(100);
        }
        if (dxlGetCWAngleLimit(i) != min_servo_pos[i - 1]) {
            dxlSetCWAngleLimit(i, min_servo_pos[i - 1]);
            delayMicroseconds(100);
        }
        if (dxlGetCCWAngleLimit(i) != max_servo_pos[i - 1]) {
            dxlSetCCWAngleLimit(i, max_servo_pos[i - 1]);
            delayMicroseconds(100);
        }
    }
    
    doReset();
    
    Serial << "Controller ready\n";
}

// Gets the ID of the servo from the abbreviation made by the first two characters of message
int getIDFromAbbreviation(const char* message, const char* nameAbbreviationsString) {
    char abbreviation[3];
    strncpy(abbreviation, message, 2); // So we have a 2-char null-terminated string
    abbreviation[2] = 0;
    const char* foundAbbreviation = strstr(nameAbbreviationsString, abbreviation); // gives pointer within nameAbbreviationsString
    if (foundAbbreviation) {
        return (foundAbbreviation - nameAbbreviationsString) / 3 + 1;
    }
    return -1;
}

// Gets the ID of the servo from the abbreviation made by the first two characters of message
int getServoID(const char* message) {
    return getIDFromAbbreviation(message, SERVO_NAMES_STR);
}

// Gets the ID of the feedback type from the abbreviation made by the first two characters of message
int getFeedbackType(const char* message) {
    return getIDFromAbbreviation(message, FEEDBACK_NAMES_STR);
}

// Gets the ID of the parameter set type from the abbreviation made by the first two characters of message
int getSetType(const char* message) {
    return getIDFromAbbreviation(message, SET_NAMES_STR);
}

int errorCorrected(int val) {
    return val == -1 ? INVALID_INT : val;
}

void reportOn(int feedbackType, const char* feedbackAbbreviation) {
    Serial << feedbackAbbreviation[0] << feedbackAbbreviation[1]; // The feedback type
    for (byte i = 1; i <= SERVO_COUNT; i++) {
        switch (feedbackType) {
            case MAX_POS_ID:
                Serial << max_servo_pos[i - 1];
                break;
            case MIN_POS_ID:
                Serial << min_servo_pos[i - 1];
                break;
            case PRESENT_POS_ID:
                Serial << errorCorrected(dxlGetPosition(i));
                break;
            case GOAL_POS_ID:
                Serial << errorCorrected(dxlGetGoalPosition(i));
                break;
            case PRESENT_SPEED_ID:
                Serial << errorCorrected(dxlGetSpeed(i));
                break;
            case GOAL_SPEED_ID:
                Serial << errorCorrected(dxlGetGoalSpeed(i));
                break;
            case PRESENT_TEMP_ID:
                Serial << errorCorrected(dxlGetTemperature(i));
                break;
            case PRESENT_LOAD_ID:
                Serial << errorCorrected(dxlGetTorque(i));
                break;
            case TORQUE_MAX_ID:
                Serial << errorCorrected(dxlGetStartupMaxTorque(i));
                break;
            case TORQUE_LIM_ID:
                Serial << errorCorrected(dxlGetTorqueLimit(i));
                break;
            case TORQUE_ENABLE_ID:
                Serial << errorCorrected(dxlGetTorqueEnable(i));
                break;
            case ERROR_ID:
                Serial << errorCorrected(dxlGetError(i));
                break;
            case DEFAULT_POS_ID:
                Serial << default_servo_pos[i - 1];
                break;
            case DEFAULT_SPEED_ID:
                Serial << default_goal_speed[i - 1];
                break;
        }
        if (i <= SERVO_COUNT - 1) {
            Serial << ':';
        }
    }
    Serial << ";\n";
}

void parseFeedback() {
    int feedbackType = getFeedbackType(receiveBuffer + 1);
    if (feedbackType == -1) {
        Serial << "Error: Feedback command " << receiveBuffer[1] <<  receiveBuffer[2] << " malformed\n";
        return;
    }
    reportOn(feedbackType, receiveBuffer + 1);
}

void parseSetCommand() {
    // Are we adjusting goal speed for position commands?
    const char* movementTimePtr = strstr(receiveBuffer, MOVEMENT_TIME_STR);
    int movementTimeMs = 0; // milliseconds to spend on all movements in this command, 0 meaning don't regulate this
    if (movementTimePtr) {
        movementTimeMs = atoi(movementTimePtr + sizeof(MOVEMENT_TIME_STR) - 1);
        // atoi returns 0 on failure, which we will use again to mean not to use this feature.
        if (movementTimeMs == 0) {
            Serial << "Warning: malformed specification of time to be moving. Ignoring this parameter : " << movementTimePtr << endl;
        }
    }
    
    int lastTargetServo = -1;
    
    const char* command = receiveBuffer;
    while (command[0]) {
        // Reached the end of the commands.
        if (strncmp(command, MOVEMENT_TIME_STR, sizeof(MOVEMENT_TIME_STR) - 1) == 0) {
            return;
        }
        
        int targetServo = getServoID(command);
        if (targetServo != -1) {
            lastTargetServo = targetServo;
            command += 2;
        } else {
            if (lastTargetServo != -1) {
                targetServo = lastTargetServo;
            } else {
                Serial << "Error: Target servo '" << command[0] << command[1] << "' malformed\n";
                return;
            }
        }

        int setType = getSetType(command);
        if (setType == -1) {
            // Don't error for the movement time string at the end, just ignore it and we are done here
            if (true || command[0] != MOVEMENT_TIME_STR[0] && command[1] != MOVEMENT_TIME_STR[1]) {
                Serial << "Error: Parameter to set '" << command[0] << command[1] << "' malformed\n";
            }
            return;
        }
        command += 2;
        
        // Also get a pointer to the end of the number as well as the value of the number
        char* afterNumberPtr;
        int value = strtol(command, &afterNumberPtr, 10);
        
        switch (setType) {
            case SET_POS_ID:
                if (movementTimeMs > 0) {
                    // The unit for speed: ~= .111rpm ~= 2/3 degrees per 1000ms
                    // the unit for position is .29 degrees
                    // so speed is distance * 0.29f / (2 / 3) / (movementTimesMs / 1000)
                    int currentPos = dxlGetPosition(targetServo);
                    int distance = abs(currentPos - value);
                    // we add movementTimeMs / 2 for rounding
                    int speed = min(1023, ((unsigned int)distance * 435 + (unsigned int)movementTimeMs / 2) / movementTimeMs);
                    dxlSetGoalSpeed(targetServo, speed);
                    delayMicroseconds(100); // I'm disappointed this delay is needed at all, but the Dynamixel will ignore the second command without it.
                    dxlSetGoalPosition(targetServo, value);
                    delayMicroseconds(100); // We put these after all of the set calls, just in case another might end up going to the same dynamixel soon after.
                    hasTemporaryGoalSpeed[targetServo - 1] = true;
                } else {
                    dxlSetGoalPosition(targetServo, value);
                    delayMicroseconds(100);
                }
                performingMovement = true;
                break;
            case SET_SPEED_ID:
                dxlSetGoalSpeed(targetServo, value);
                delayMicroseconds(100);
                userGoalSpeed[targetServo - 1] = value;
                break;
            case SET_MAX_TORQUE_ID:
                // In EEPROM, so check before saving to prevent eeprom from wearing down
                if (dxlGetStartupMaxTorque(targetServo) != value) {
                    dxlSetStartupMaxTorque(targetServo, value);
                    delayMicroseconds(100);
                }
                break;
            case SET_TORQUE_LIM_ID:
                dxlSetRunningTorqueLimit(targetServo, value);
                delayMicroseconds(100);
                break;
            case SET_TORQUE_ENABLE_ID:
                dxlSetTorqueEnable(targetServo, value);
                delayMicroseconds(100);
                break;
        }
        
        command = afterNumberPtr;
    }
}

void parseCommand() {
    if (receiveBuffer[0] == 'F') {
        parseFeedback();
    } else if (strncmp(receiveBuffer, "RESET", 5) == 0) {
        doReset();
    } else {
        parseSetCommand();
    }
}

void loop() {
    // Periodically report on errors
    int nowMs = millis();
    if (nowMs - lastErrorCheckMillis >= ERROR_CHECK_MS) {
        reportOn(ERROR_ID, "ER");
        lastErrorCheckMillis = nowMs;
    }
    
    // Check for whether motors need their speeds changed after motion is complete
    bool allMovementComplete = true;
    for (int i = 1; i <= SERVO_COUNT; i++) {
        int moving = dxlGetMoving(i);
        if (moving == 0) {
            if (hasTemporaryGoalSpeed[i - 1]) {
                hasTemporaryGoalSpeed[i - 1] = false;
                dxlSetGoalSpeed(i, userGoalSpeed[i - 1]);
                delayMicroseconds(100);
                //Serial << "Resetting goal speed of servo: " << i << endl;
            }
            if (i == HEAD_TILT && removeHeadTiltTorqueAfterMove) {
                removeHeadTiltTorqueAfterMove = false;
                dxlSetTorqueEnable(HEAD_TILT, 0);
                delayMicroseconds(100);
                //Serial << "Disabling head tilt torque enable\n";
            }
        } else if (moving != -1) {
            allMovementComplete = false;
        }
    }
    
    // Check if movement has completed, and report if so.
    if (performingMovement && allMovementComplete) {
        performingMovement = false;
        reportOn(PRESENT_POS_ID, "PP");
    }
    
    int serialBytes = Serial.available();
    while(serialBytes-- > 0) {
        unsigned char newChar = Serial.read();
        // lower-case to upper-case conversion
        if (newChar >= 'a' && newChar <= 'z') {
            newChar += 'A' - 'a';
        }
        // Refuse garbage characters -- only accept ASCII
        if (newChar < 128) {
            digitalWrite(USER_LED, serialBytes > 0); // led on when we have stuff to read
            Serial << (char)newChar;
            // Reset on white-space
            if (newChar == ' ' || newChar == '\n' || newChar == '\r') {
                receiveBufferI = 0;
            } else if (newChar == 'Z') {
                // Command fully received (note we don't add Z to the receiveBuffer)
                // make sure to null-terminated
                receiveBuffer[receiveBufferI] = 0;
                parseCommand();
                receiveBufferI = 0;
            } else {
                if (receiveBufferI + 1 >= sizeof(receiveBuffer)) {
                    Serial << "Error: Buffer overflow! Throwing away: " << receiveBuffer << endl;
                    receiveBufferI = 0;
                }
                receiveBuffer[receiveBufferI++] = newChar;
            }
        }
    }
}
