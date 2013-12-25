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
MAX_FORWARD_SPEED = 160
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
		self.shutdown = False


	def isButtonPushed(self):
		byte = self.controller.readOneByteRegister(122, 0x26) #control_digital_0
		return byte == 0

	def log(self, logString):
		if self.logDebug:
			print logString

	def readSharpIRSensor(self, channel):
		sample = self.controller.readTwoByteRegister(122, 0x1A + (2 * channel)) / 4
		if sample < 10:
			return 254
		sample = 1309 / (sample - 3)
		return sample - 1

	def readSmoothSharpIRSensor(self, channel):
		value = self.readSharpIRSensor(channel)
		value += self.readSharpIRSensor(channel)
		value += self.readSharpIRSensor(channel)
		value += self.readSharpIRSensor(channel)
		return value / 4

	def readSensors(self):
		self.sensorFrontDistance = self.readSharpIRSensor(5)
		self.sensorLeftDistance = self.readSharpIRSensor(4)
		self.sensorRightDistance = self.readSharpIRSensor(3)

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
		self.shutdown = True

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

roz.ikEngine.setupForWalk()
roz.ikEngine.setTranTime(100)

while not roz.shutdown:
	roz.readSensors()
	roz.stateMachine.update()
	roz.ikEngine.handleIK()

print 'Shutdown'

