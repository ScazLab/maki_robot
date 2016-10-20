import rospy
from makicom.msg import MakiCommand, MakiSetting, MakiFeedback

INVALID_INT = 9999

EYELID_RIGHT = 'LR'
EYELID_LEFT = 'LL'
EYE_PAN = 'EP'
EYE_TILT = 'ET'
HEAD_TILT = 'HT'
HEAD_PAN = 'HP'
servoOptions = [EYELID_RIGHT, EYELID_LEFT, EYE_PAN, EYE_TILT, HEAD_TILT, HEAD_PAN]

MAX_POS = 'MX'
MIN_POS = 'MN'
PRESENT_POS = 'PP'
GOAL_POS = 'GP'
PRESENT_SPEED = 'PS'
GOAL_SPEED  = 'GS'
PRESENT_TEMP = 'PT'
PRESENT_LOAD = 'PL'
TORQUE_MAX = 'TM'
TORQUE_LIM = 'TL'
TORQUE_ENABLE = 'TS'
MOVING = 'MV'
ERROR = 'ER'
DEFAULT_POS = 'DP'
DEFAULT_SPEED = 'DS'
feedbackOptions = [MAX_POS, MIN_POS, PRESENT_POS, GOAL_POS, PRESENT_SPEED, GOAL_SPEED, PRESENT_TEMP, PRESENT_LOAD, TORQUE_MAX, TORQUE_LIM, TORQUE_ENABLE, MOVING, ERROR, DEFAULT_POS, DEFAULT_SPEED]

setTypes = [GOAL_POS, GOAL_SPEED, TORQUE_MAX, TORQUE_LIM, TORQUE_ENABLE]

def feedbackCommand(feedbackType):
    if feedbackType not in feedbackOptions:
        raise ValueError('invalid feedback selected')
    cmd = MakiCommand()
    cmd.type = 'feedback'
    cmd.feedbackType = feedbackType
    return cmd
    
def setCommand(servos, setTypes, values, movementTimeMs=0):
    if any(servo not in servoOptions for servo in servos):
        raise ValueError('invalid servo selected')
    if any(setType not in setOptions for setType in setTypes):
        raise ValueError('invalid set type selected')
    cmd = MakiCommand()
    cmd.type = 'set'
    servos = list(servos)
    setTypes = list(setTypes)
    values = list(values)
    cmd.settings = [MakiSetting(servos[i], setTypes[i], values[i]) for i in xrange(len(servos))]
    cmd.movementTimeMs = movementTimeMs
    return cmd
    
def getSetCommandSetting(cmd, servo, setType):
    if cmd.type != 'set':
        raise ValueError('command is not type "set"')
    if servo not in servoOptions:
        raise ValueError('invalid servo selected')
    if setType not in setOptions:
        raise ValueError('invalid set type selected')
    for s in cmd.settings:
        if s.servo == servo and s.setType == setType:
            return s
    return None
    
    
def updateSetCommand(cmd, servo, setTypes, values):
    if cmd.type != 'set':
        raise ValueError('command is not type "set"')
    if any(servo not in servoOptions for servo in servos):
        raise ValueError('invalid servo selected')
    if any(setType not in setOptions for setType in setTypes):
        raise ValueError('invalid set type selected')
    servos = list(servos)
    setTypes = list(setTypes)
    values = list(values)
    for i in xrange(len(servos)):
        s = getSetCommandSetting(cmd, servos[i], setTypes[i])
        if s:
            s.value = values[i]
        else:
            cmd.settings.append(MakiSetting(servos[i], setTypes[i], values[i]))
    return cmd
    
def resetCommand():
    return MakiCommand(type = 'reset')
    
def newMakiFeedback():
    return MakiFeedback([[INVALID_INT] * len(servoOptions) for i in xrange(len(feedbackOptions))])

def getFeedback(servo, feedbackType, makiFeedback):
    if servo not in servoOptions:
        raise ValueError('invalid servo selected')
    if feedbackType not in feedbackOptions:
        raise ValueError('invalid feedback selected')
    return makiFeedback[feedbackOptions.index(feedbackType)][servoOptions.index(servo)]
 
def getFeedbackDict(feedbackType, makiFeedback):
    if feedbackType not in feedbackOptions:
        raise ValueError('invalid feedback selected')
    return [makiFeedback[feedbackOptions.index(feedbackType)] for servo in servoOptions]
 
def updateFeedback(feedbackType, values, makiFeedback):
    if feedbackType not in feedbackOptions:
        raise ValueError('invalid feedback selected')
    makiFeedback[feedbackOptions.index(feedbackType)] = values