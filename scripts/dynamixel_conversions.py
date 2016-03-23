class dynamixelConversions:

	## specs for Dynamixel motors AX-12 / AX-18
	TICKS = 1024	## number of steps
	OPERATING_ANGLE = 300.0	## degrees
	TICKS_PER_DEGREE = 3.41	## TICKS / OPERATING_ANGLE
	UNIT_SPEED = 0.111	## rpm

	DEGREES_PER_REVOLUTION = 360.0
	SECONDS_PER_MINUTE = 60.0
	RADIANS_PER_DEGREE = 0.0174533

	def __init__(self):
		pass	## does nothing

	def getTurnDuration_ticks_goalSpeed( self, ticks, goalSpeed ):
		## in seconds
		ret = None
		_rpm = float(goalSpeed * dynamixelConversions.UNIT_SPEED)
		_ticks_per_second = ( (_rpm / dynamixelConversions.SECONDS_PER_MINUTE) * dynamixelConversions.DEGREES_PER_REVOLUTION * dynamixelConversions.TICKS_PER_DEGREE )
		ret = float( ticks / _ticks_per_second )
		return ret

	def getTurnDurationMS_ticks_goalSpeed( self, ticks, goalSpeed ):
		## in milliseconds
		ret = dynamixelConversions.getTurnDuration_ticks_goalSpeed( self, ticks, goalSpeed )
		ret = float( ret * 1000 )
		return ret

	def getTurnDuration_degrees_goalSpeed( self, degrees, goalSpeed ):
		## in seconds
		ret = None
		_ticks = float( degrees * dynamixelConversions.TICKS_PER_DEGREE )
		ret = dynamixelConversions.getTurnDuration_ticks_goalSpeed( self, _ticks, goalSpeed )
		return ret

	def getTurnDurationMS_degrees_goalSpeed( self, degrees, goalSpeed ):
		## in milliseconds
		ret = dynamixelConversions.getTurnDuration_degrees_goalSpeed( self, degrees, goalSpeed )
		ret = float( ret * 1000 )
		return ret

	def getTurnDuration_radians_goalSpeed( self, radians, goalSpeed ):
		## in seconds
		ret = None
		_degrees = float(radians / dynamixelConversions.RADIANS_PER_DEGREE)
		ret = dynamixelConversions.getTurnDuration_degrees_goalSpeed( self, _degrees, goalSpeed )
		return ret

	def getTurnDurationMS_radians_goalSpeed( self, radians, goalSpeed ):
		## in milliseconds
		ret = dynamixelConversions.getTurnDuration_radians_goalSpeed( self, radians, goalSpeed )
		ret = float( ret * 1000 )
		return ret

	def getGoalSpeed_ticks_duration( self, ticks, s_duration):
		## in dynamixel speed units
		## [0, 1023], where 0 = unlimited
		## 1 = min, 1023 = max (114 rpm)

		## duration given in seconds
		ret = None
		_ticks_per_second = float( ticks / s_duration )
		_rpm = (((_ticks_per_second * dynamixelConversions.SECONDS_PER_MINUTE) / dynamixelConversions.TICKS_PER_DEGREE) / dynamixelConversions.DEGREES_PER_REVOLUTION)
		ret = _rpm / dynamixelConversions.UNIT_SPEED
		return ret

	def getGoalSpeed_ticks_durationMS( self, ticks, ms_duration):
		## in dynamixel speed units
		## [0, 1023], where 0 = unlimited
		## 1 = min, 1023 = max (114 rpm)

		## duration given in milliseconds
		_s_duration = float( ms_duration / 1000.0 )
		ret = dynamixelConversions.getGoalSpeed_ticks_duration( self, ticks, _s_duration )
		return ret

	def getGoalSpeed_degrees_duration( self, degrees, s_duration ):
		## in dynamixel speed units
		## [0, 1023], where 0 = unlimited
		## 1 = min, 1023 = max (114 rpm)

		## duration given in seconds
		_ticks = float( degrees * dynamixelConversions.TICKS_PER_DEGREE )
		ret = dynamixelConversions.getGoalSpeed_ticks_duration( self, _ticks, s_duration )
		return ret

	def getGoalSpeed_degrees_durationMS( self, degrees, ms_duration ):
		## in dynamixel speed units
		## [0, 1023], where 0 = unlimited
		## 1 = min, 1023 = max (114 rpm)

		## duration given in milliseconds
		_s_duration = float( ms_duration / 1000.0 )
		ret = dynamixelConversions.getGoalSpeed_degrees_duration( self, degrees, _s_duration )
		return ret

	
	def getGoalSpeed_radians_duration( self, radians, s_duration ):
		## in dynamixel speed units
		## [0, 1023], where 0 = unlimited
		## 1 = min, 1023 = max (114 rpm)

		## duration given in seconds
		_degrees = float(radians / dynamixelConversions.RADIANS_PER_DEGREE)
		ret = dynamixelConversions.getGoalSpeed_degrees_duration( self, _degrees, s_duration )
		return ret

	def getGoalSpeed_radians_durationMS( self, radians, ms_duration ):
		## in dynamixel speed units
		## [0, 1023], where 0 = unlimited
		## 1 = min, 1023 = max (114 rpm)

		## duration given in milliseconds
		_s_duration = float( ms_duration / 1000.0 )
		ret = dynamixelConversions.getGoalSpeed_radians_duration( self, radians, _s_duration )
		return ret
