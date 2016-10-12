import rospy

## all instances of this class share the same value
## specs for Dynamixel motors AX-12 / AX-18
TICKS = 1024	## number of steps
OPERATING_ANGLE = 300.0	## degrees
TICKS_PER_DEGREE = 3.41	## TICKS / OPERATING_ANGLE
UNIT_SPEED = 0.111	## rpm

DEGREES_PER_REVOLUTION = 360.0
SECONDS_PER_MINUTE = 60.0
RADIANS_PER_DEGREE = 0.0174533

def convertToDegrees_ticks( self, ticks ):
    ret = None
    if (ticks == 0):	return 0
    if (ticks < 0):	ticks = abs(ticks)
    ret = float(ticks / TICKS_PER_DEGREE)
    return ret

def convertToRadians_ticks( self, ticks ):
    ret = convertToDegrees( self, ticks )
    ret = float(ret * RADIANS_PER_DEGREE)
    return ret

def convertToTicks_degrees( self, degrees ):
    ret = None
    if (degrees == 0):	return 0
    if (degrees < 0):	degrees = abs(degrees)
    ret = float(degrees * TICKS_PER_DEGREE)
    ret = int( ret + 0.5 )	## implict  rounding
    return ret

def convertToTicks_radians( self, radians ):
    rad = float(radians / RADIANS_PER_DEGREE)
    ret = convertToTicks_degrees( self, rad )
    return ret

def getTurnDuration_ticks_goalSpeed( self, ticks, goalSpeed ):
    ## in seconds

    ret = None
    ## check validitry of inputs
    if (goalSpeed == 0):
        rospy.logerr("getGoalSpeed_ticks_duration: INVALID INPUT: goalSpeed=" + str(s_duration) + "; 0 means unlimited")
        return ret
    else:
        goalSpeed = abs(goalSpeed)	## do some work since we already had to do a comparison
    if (ticks == 0):
        rospy.logerr("getGoalSpeed_ticks_duration: INVALID INPUT: ticks=" + str(ticks))
        return ret
    else:
        ticks = abs(ticks)	## do some work since we already had to do a comparison

    _rpm = float(goalSpeed * UNIT_SPEED)
    _ticks_per_second = ( (_rpm / SECONDS_PER_MINUTE) * DEGREES_PER_REVOLUTION * TICKS_PER_DEGREE )
    ret = float( ticks / _ticks_per_second )
    return ret

def getTurnDurationMS_ticks_goalSpeed( self, ticks, goalSpeed ):
    ## in milliseconds
    ret = getTurnDuration_ticks_goalSpeed( self, ticks, goalSpeed )
    ret = float( ret * 1000 )
    return ret

def getTurnDuration_degrees_goalSpeed( self, degrees, goalSpeed ):
    ## in seconds
    ret = None
    _ticks = float( degrees * TICKS_PER_DEGREE )
    ret = getTurnDuration_ticks_goalSpeed( self, _ticks, goalSpeed )
    return ret

def getTurnDurationMS_degrees_goalSpeed( self, degrees, goalSpeed ):
    ## in milliseconds
    ret = getTurnDuration_degrees_goalSpeed( self, degrees, goalSpeed )
    ret = float( ret * 1000 )
    return ret

def getTurnDuration_radians_goalSpeed( self, radians, goalSpeed ):
    ## in seconds
    ret = None
    _degrees = float(radians / RADIANS_PER_DEGREE)
    ret = getTurnDuration_degrees_goalSpeed( self, _degrees, goalSpeed )
    return ret

def getTurnDurationMS_radians_goalSpeed( self, radians, goalSpeed ):
    ## in milliseconds
    ret = getTurnDuration_radians_goalSpeed( self, radians, goalSpeed )
    ret = float( ret * 1000 )
    return ret

def getGoalSpeed_ticks_duration( self, ticks, s_duration, return_int=True, disable_unlimited_speed=True ):
    ## in dynamixel speed units
    ## [0, 1023], where 0 = unlimited
    ## 1 = min, 1023 = max (114 rpm)

    ## duration given in seconds

    ret = None
    ## check validitry of inputs
    if (s_duration < 0):
        rospy.logerr("getGoalSpeed_ticks_duration: INVALID INPUT: s_duration=" + str(s_duration) + "; cannot have negative duration")
        return ret
    if (s_duration == 0):
        rospy.logerr("getGoalSpeed_ticks_duration: INVALID INPUT: s_duration=" + str(s_duration) + "; cannot divide by zero")
        return ret
    if (ticks == 0):
        rospy.logerr("getGoalSpeed_ticks_duration: INVALID INPUT: ticks=" + str(ticks))
        return ret
    else:
        ticks = abs(ticks)	## do some work since we already had to do a comparison

    _ticks_per_second = float( ticks / s_duration )
    _rpm = (((_ticks_per_second * SECONDS_PER_MINUTE) / TICKS_PER_DEGREE) / DEGREES_PER_REVOLUTION)
    ret = _rpm / UNIT_SPEED

    ## round so we can return int
    if return_int:
        ret = int(ret + 0.5)	## implicit rounding

    ## check computed goal speed is within bounds
    if (ret > 1023):	ret = 1023
    if (ret < 0):		ret = 0
    if (ret == 0):	rospy.logwarn("getGoalSpeed_ticks_duration: WARNING: computed goal speed is 0")
    if disable_unlimited_speed and (ret == 0):	
        ret = 1
        rospy.logdebug("getGoalSpeed_ticks_duration: overriding computed goal speed; change from 0 to " + str(ret))

    return ret

def getGoalSpeed_ticks_durationMS( self, ticks, ms_duration, return_int=True, disable_unlimited_speed=True):
    ## in dynamixel speed units
    ## [0, 1023], where 0 = unlimited
    ## 1 = min, 1023 = max (114 rpm)

    ## duration given in milliseconds
    _s_duration = float( ms_duration / 1000.0 )
    ret = getGoalSpeed_ticks_duration( self, ticks, _s_duration, return_int, disable_unlimited_speed )
    return ret

def getGoalSpeed_degrees_duration( self, degrees, s_duration ):
    ## in dynamixel speed units
    ## [0, 1023], where 0 = unlimited
    ## 1 = min, 1023 = max (114 rpm)

    ## duration given in seconds
    _ticks = float( degrees * TICKS_PER_DEGREE )
    ret = getGoalSpeed_ticks_duration( self, _ticks, s_duration )
    return ret

def getGoalSpeed_degrees_durationMS( self, degrees, ms_duration ):
    ## in dynamixel speed units
    ## [0, 1023], where 0 = unlimited
    ## 1 = min, 1023 = max (114 rpm)

    ## duration given in milliseconds
    _s_duration = float( ms_duration / 1000.0 )
    ret = getGoalSpeed_degrees_duration( self, degrees, _s_duration )
    return ret


def getGoalSpeed_radians_duration( self, radians, s_duration ):
    ## in dynamixel speed units
    ## [0, 1023], where 0 = unlimited
    ## 1 = min, 1023 = max (114 rpm)

    ## duration given in seconds
    _degrees = float(radians / RADIANS_PER_DEGREE)
    ret = getGoalSpeed_degrees_duration( self, _degrees, s_duration )
    return ret

def getGoalSpeed_radians_durationMS( self, radians, ms_duration ):
    ## in dynamixel speed units
    ## [0, 1023], where 0 = unlimited
    ## 1 = min, 1023 = max (114 rpm)

    ## duration given in milliseconds
    _s_duration = float( ms_duration / 1000.0 )
    ret = getGoalSpeed_radians_duration( self, radians, _s_duration )
    return ret
