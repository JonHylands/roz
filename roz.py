#!/usr/bin/env python

"""
  Roz - a Bioloid Quad Walker

"""

import random
from BioloidController import BioloidController, millis
from FSM import State, FiniteStateMachine
from Nuke import IKEngine


FRONT_SENSOR_OBSTACLE = 25
SIDE_SENSOR_OBSTACLE = 25

FRONT_OBSTACLE_TURN_SPEED = 0.5
SIDE_OBSTACLE_TURN_SPEED = 0.1

FRONT_OBSTACLE_TURN_TIME = 2000
MAX_FORWARD_SPEED = 150
START_FORWARD_SPEED = (MAX_FORWARD_SPEED * 0.3)

MINIMUM_VOLTAGE = 10.0
MAXIMUM_TEMPERATURE = 85

WATCHDOG_TIME_INTERVAL = 2000 # milliseconds between watchdog checks

MINI_IO_ID = 122

MINI_IO_DIGITAL_0_DIRECTION = 6
MINI_IO_DIGITAL_2_DIRECTION = 8
MINI_IO_DIGITAL_4_DIRECTION = 10

MINI_IO_INPUT_DIRECTION = 0
MINI_IO_OUTPUT_DIRECTION = 1

MINI_IO_ANALOG_0 = 26
MINI_IO_ANALOG_1 = 28
MINI_IO_ANALOG_2 = 30
MINI_IO_ANALOG_3 = 32
MINI_IO_ANALOG_4 = 34
MINI_IO_ANALOG_5 = 36

MINI_IO_DIGITAL_0 = 38
MINI_IO_DIGITAL_2 = 40
MINI_IO_DIGITAL_4 = 42

HEAD_YAW_ID = 13
HEAD_PITCH_ID = 14

AX_GOAL_POSITION = 30
AX_12_VOLTAGE = 42
AX_12_TEMPERATURE = 43

FORWARD_ROT_Z = -0.02

class Roz:

	def __init__(self):
		self.standingPose = [450, 570, 600, 420, 550, 460, 570, 450, 420, 600, 460, 550]
		self.controller = BioloidController()
		self.ikEngine = IKEngine()
		self.ikEngine.setController(self.controller)
		self.ikEngine.setupForWalk(self.standingPose)
		self.ikEngine.setTranTime(100)
		self.debug = False
		self.logDebug = True
		self.readSensors()
		self.shutdown = False
		self.heartbeatOn = False
		self.setupMiniIOBoard()
		self.setupHeadPosition()
		self.watchdogServoId = 1

	def setupMiniIOBoard(self):
		# set the push button on Digital(0) to input
		self.controller.writeData(MINI_IO_ID, MINI_IO_DIGITAL_0_DIRECTION, [MINI_IO_INPUT_DIRECTION])

		# set the LEDs on Digital(2) & (4) to output, and clear them
		self.controller.writeData(MINI_IO_ID, MINI_IO_DIGITAL_2_DIRECTION, [MINI_IO_OUTPUT_DIRECTION])
		self.controller.writeData(MINI_IO_ID, MINI_IO_DIGITAL_4_DIRECTION, [MINI_IO_OUTPUT_DIRECTION])
		self.controller.writeData(MINI_IO_ID, MINI_IO_DIGITAL_2, [0])
		self.controller.writeData(MINI_IO_ID, MINI_IO_DIGITAL_4, [0])

	def setupHeadPosition(self):
		position = 511
		bufferData = [position & 0xFF, position >> 8]
		self.controller.writeData(HEAD_YAW_ID, AX_GOAL_POSITION, bufferData)
		position = 490
		bufferData = [position & 0xFF, position >> 8]
		self.controller.writeData(HEAD_PITCH_ID, AX_GOAL_POSITION, bufferData)

	def isButtonPushed(self):
		byte = self.controller.readOneByteRegister(MINI_IO_ID, MINI_IO_DIGITAL_0) #control_digital_0
		return byte == 0

	def turnOnHeartbeatLED(self):
		if not self.heartbeatOn:
			self.controller.writeData(MINI_IO_ID, MINI_IO_DIGITAL_2, [1])
			self.heartbeatOn = True

	def turnOffHeartbeatLED(self):
		if self.heartbeatOn:
			self.controller.writeData(MINI_IO_ID, MINI_IO_DIGITAL_2, [0])
			self.heartbeatOn = False

	def readBatteryVoltage(self):
		return self.controller.readOneByteRegister(1, AX_12_VOLTAGE) / 10.0

	def checkBatteryVoltageAndTemperature(self, servoId):
		data = self.controller.readData(servoId, AX_12_VOLTAGE, 2)
		voltage = ord(data[0])
		temperature = ord(data[1])
		if voltage < MINIMUM_VOLTAGE:
			print 'Battery too low: %3.1f volts - shutting down' % voltage
			self.stateMachine.transitionTo(self.shutdownState)
		if temperature > MAXIMUM_TEMPERATURE:
			print 'Servo %d temperature too high: %d degrees C' % (servoId, temperature)
			self.stateMachine.transitionTo(self.shutdownState)

	def log(self, logString):
		if self.logDebug:
			print logString

	def convertSharpIRSensor(self, rawValue):
		sample = rawValue / 4
		if sample < 10:
			return 254
		sample = 1309 / (sample - 3)
		return sample - 1

	def readSharpIRSensor(self, channel):
		sample = self.controller.readTwoByteRegister(MINI_IO_ID, MINI_IO_ANALOG_0 + (2 * channel))
		return self.convertSharpIRSensor(sample)

	def readSensors(self):
		# we could read the registers one at a time using readSharpIRSensor(), but its much faster to do it this way
		rawData = self.controller.readData(MINI_IO_ID, MINI_IO_ANALOG_3, 6) # read channels 3, 4, and 5 all at once
		frontRaw = ord(rawData[4]) + (ord(rawData[5]) << 8) # extract channel 5 - front
		self.sensorFrontDistance = self.convertSharpIRSensor(frontRaw)
		leftRaw = ord(rawData[2]) + (ord(rawData[3]) << 8) # extract channel 4 - left
		self.sensorLeftDistance = self.convertSharpIRSensor(leftRaw)
		rightRaw = ord(rawData[0]) + (ord(rawData[1]) << 8) # extract channel 3 - right
		self.sensorRightDistance = self.convertSharpIRSensor(rightRaw)

	#=====================================
	#
	#       Waiting for Button State
	#

	def enterWaitingForButtonState(self):
		self.log('Entering WaitingForButtonState')
		self.ikEngine.travelX = 0
		self.ikEngine.travelRotZ = 0.0

	def handleWaitingForButtonState(self):
		if self.isButtonPushed():
			self.stateMachine.transitionTo(self.waitingForNoButtonState)

	#=====================================
	#
	#       Waiting for No Button State
	#

	def enterWaitingForNoButtonState(self):
		self.log('Entering WaitingForNoButtonState')

	def handleWaitingForNoButtonState(self):
		if not self.isButtonPushed():
			self.stateMachine.transitionTo(self.walkingState)

	#=====================================
	#
	#       Walking State
	#

	def enterWalkingState(self):
		self.log('Entering walking state')
		self.ikEngine.travelX = START_FORWARD_SPEED
		self.ikEngine.travelRotZ = FORWARD_ROT_Z

	def handleWalkingState(self):
		if self.ikEngine.travelX < MAX_FORWARD_SPEED:
			self.ikEngine.travelX += 2
		if self.sensorFrontDistance < FRONT_SENSOR_OBSTACLE:
			self.stateMachine.transitionTo (self.obstacleAvoidanceState)
		elif self.sensorRightDistance < SIDE_SENSOR_OBSTACLE:
			self.ikEngine.travelRotZ = SIDE_OBSTACLE_TURN_SPEED
		elif self.sensorLeftDistance < SIDE_SENSOR_OBSTACLE:
			self.ikEngine.travelRotZ = -SIDE_OBSTACLE_TURN_SPEED
		else:
			self.ikEngine.travelRotZ = FORWARD_ROT_Z
		if self.isButtonPushed():
			self.stateMachine.transitionTo(self.shutdownState)

	#=====================================
	#
	#       Obstacle Avoidance State
	#

	def enterObstacleAvoidanceState(self):
		self.log('Entering ObstacleAvoidanceState')
		self.ikEngine.travelX = 0
		self.turnEndTime = millis() + FRONT_OBSTACLE_TURN_TIME
		if (self.sensorLeftDistance < SIDE_SENSOR_OBSTACLE) & (self.sensorRightDistance < SIDE_SENSOR_OBSTACLE):
			self.log('Obstacles on both sides')
			self.ikEngine.travelRotZ = -FRONT_OBSTACLE_TURN_SPEED
			self.turnEndTime += FRONT_OBSTACLE_TURN_TIME
		elif self.sensorLeftDistance < SIDE_SENSOR_OBSTACLE:
			self.log('Obstacle on left side')
			self.ikEngine.travelRotZ = -FRONT_OBSTACLE_TURN_SPEED
		elif self.sensorRightDistance < SIDE_SENSOR_OBSTACLE:
			self.log('Obstacle on right side')
			self.ikEngine.travelRotZ = FRONT_OBSTACLE_TURN_SPEED
		else: #nothing on either side, so pick a side at random
			self.log('Only front obstacle')
			value = random.randint(0, 1)
			if value == 0:
				self.ikEngine.travelRotZ = -FRONT_OBSTACLE_TURN_SPEED
			else:
				self.ikEngine.travelRotZ = FRONT_OBSTACLE_TURN_SPEED

	def handleObstacleAvoidanceState(self):
		if millis() > self.turnEndTime:
			self.stateMachine.transitionTo(self.walkingState)
		if self.isButtonPushed():
			self.stateMachine.transitionTo(self.shutdownState)

	#=====================================
	#
	#       Shutdown State
	#

	def enterShutdownState(self):
		self.ikEngine.travelX = 0
		self.ikEngine.travelRotZ = 0
		self.turnOffHeartbeatLED()
		self.shutdown = True


	#=====================================
	#
	#       Heartbeat State Machine
	#

	def enterHeartbeatState(self):
		self.heartbeatOnTime = 100
		self.heartbeatCycleTime = 1000

	def handleHeartbeatState(self):
		elapsedTime = self.heartbeatStateMachine.getCurrentStateMillis()
		if elapsedTime % self.heartbeatCycleTime > self.heartbeatOnTime:
			self.turnOffHeartbeatLED()
		else:
			self.turnOnHeartbeatLED()


	#=====================================
	#
	#       Watchdog State Machine
	#

	def handleWatchdogState(self):
		# round robin the servos when checking voltage and temperature
		self.checkBatteryVoltageAndTemperature(self.watchdogServoId)
		self.watchdogServoId += 1
		if self.watchdogServoId > 12:
			self.watchdogServoId = 1
		self.watchdogStateMachine.transitionTo(self.watchdogWaitState)

	def handleWatchdogWaitState(self):
		elapsedTime = self.watchdogStateMachine.getCurrentStateMillis()
		if elapsedTime > WATCHDOG_TIME_INTERVAL:
			self.watchdogStateMachine.transitionTo(self.watchdogState)


#=====================================

roz = Roz()
voltage = roz.readBatteryVoltage()
print 'Battery: ', voltage, ' volts'

roz.waitingForButtonState = State(roz.enterWaitingForButtonState, roz.handleWaitingForButtonState, None)
roz.waitingForNoButtonState = State(roz.enterWaitingForNoButtonState, roz.handleWaitingForNoButtonState, None)
roz.walkingState = State(roz.enterWalkingState, roz.handleWalkingState, None)
roz.obstacleAvoidanceState = State(roz.enterObstacleAvoidanceState, roz.handleObstacleAvoidanceState, None)
roz.shutdownState = State(roz.enterShutdownState, None, None)
roz.stateMachine = FiniteStateMachine(roz.waitingForButtonState)

roz.heartbeatState = State(roz.enterHeartbeatState, roz.handleHeartbeatState, None)
roz.heartbeatStateMachine = FiniteStateMachine(roz.heartbeatState)

roz.watchdogState = State(None, roz.handleWatchdogState, None)
roz.watchdogWaitState = State(None, roz.handleWatchdogWaitState, None)
roz.watchdogStateMachine = FiniteStateMachine(roz.watchdogState)

while not roz.shutdown:
	roz.readSensors()
	roz.heartbeatStateMachine.update()
	roz.stateMachine.update()
	roz.ikEngine.handleIK()
	roz.watchdogStateMachine.update()

print 'Shutdown'

