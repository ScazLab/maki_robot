import rospy
from makicontrol.msg import MakiCommand

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
ERROR = 'ER'
DEFAULT_POS = 'DP'
DEFAULT_SPEED = 'DS'
feedbackOptions = [MAX_POS, MIN_POS, PRESENT_POS, GOAL_POS, PRESENT_SPEED, GOAL_SPEED, PRESENT_TEMP, PRESENT_LOAD, TORQUE_MAX, TORQUE_LIM, TORQUE_ENABLE, ERROR, DEFAULT_POS, DEFAULT_SPEED]

def feedbackCommand(feedbackType):
    if feedbackType not in feedbackOptions:
        raise ValueError('invalid feedback selected')
    cmd = MakiCommand()
    cmd.type = 'feedback'
    cmd.feedbackType = feedbackType
    return cmd
    
def setCommand(servo, setOption, value):
    if servo not in servoOptions:
        raise ValueError('invalid servo selected')
    if feedbackType not in feedbackOptions:
        raise ValueError('invalid feedback selected')
    cmd = MakiCommand()
    cmd.servo = [servo]
    cmd.setOption = [setOption]
    cmd.value = [value]
    return cmd