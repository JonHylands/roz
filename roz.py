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

FORWARD_ROT_Z = -0.02

class Roz:

	def __init__(self, opt=4, debug=False, gaitGen = None):
		self.debug = debug	# do we print debug messages or not?

		self.controller = BioloidController()
		self.ikEngine = IKEngine()
		self.ikEngine.setController(self.controller)
		self.debug = False
		self.logDebug = True
		self.readSensors()
		self.shutdown = False
		self.heartbeatOn = False

	def isButtonPushed(self):
		byte = self.controller.readOneByteRegister(122, 0x26) #control_digital_0
		return byte == 0

	def turnOnHeartbeatLED(self):
		if not self.heartbeatOn:
			self.controller.writeData(122, 40, [1])
			self.heartbeatOn = True

	def turnOffHeartbeatLED(self):
		if self.heartbeatOn:
			self.controller.writeData(122, 40, [0])
			self.heartbeatOn = False

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
		sample = self.controller.readTwoByteRegister(122, 0x1A + (2 * channel))
		return self.convertSharpIRSensor(sample)

	def readSensors(self):
		# we could read the registers one at a time using readSharpIRSensor(), but its much faster to do it this way
		rawData = self.controller.readData(122, 32, 6) # read channels 3, 4, and 5 all at once
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
	#       Heartbeat State
	#

	def enterHeartbeatState(self):
		self.heartbeatOnTime = 100
		self.heartbeatCycleTime = 1000

	def handleHeartbeatState(self):
		elapsedTime = roz.heartbeatStateMachine.getCurrentStateMillis()
		if elapsedTime % self.heartbeatCycleTime > self.heartbeatOnTime:
			self.turnOffHeartbeatLED()
		else:
			self.turnOnHeartbeatLED()


#=====================================

roz = Roz()
voltage = roz.controller.readOneByteRegister(1, 0x2A) / 10.0
print 'Battery: ', voltage, ' volts'

roz.waitingForButtonState = State(roz.enterWaitingForButtonState, roz.handleWaitingForButtonState, None)
roz.waitingForNoButtonState = State(roz.enterWaitingForNoButtonState, roz.handleWaitingForNoButtonState, None)
roz.walkingState = State(roz.enterWalkingState, roz.handleWalkingState, None)
roz.obstacleAvoidanceState = State(roz.enterObstacleAvoidanceState, roz.handleObstacleAvoidanceState, None)
roz.shutdownState = State(roz.enterShutdownState, None, None)
roz.stateMachine = FiniteStateMachine(roz.waitingForButtonState)

roz.heartbeatState = State(roz.enterHeartbeatState, roz.handleHeartbeatState, None)
roz.heartbeatStateMachine = FiniteStateMachine(roz.heartbeatState)

roz.ikEngine.setupForWalk()
roz.ikEngine.setTranTime(100)

while not roz.shutdown:
	roz.readSensors()
	roz.heartbeatStateMachine.update()
	roz.stateMachine.update()
	roz.ikEngine.handleIK()

print 'Shutdown'

