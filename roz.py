#!/usr/bin/env python

"""
  Roz - a Bioloid Quad Walker

"""

import pyb
from BioloidController import BioloidController
from FSM import State, FiniteStateMachine
from Nuke import IKEngine
from Support import RangeFinder, HeartbeatLED, OneShotButton, Logger

import struct


FRONT_SENSOR_OBSTACLE = 18 # cm
SIDE_SENSOR_OBSTACLE = 18

FRONT_OBSTACLE_TURN_SPEED = 0.5
SIDE_OBSTACLE_TURN_SPEED = 0.1

FRONT_OBSTACLE_TURN_TIME = 2000
MAX_FORWARD_SPEED = 180
START_FORWARD_SPEED = (MAX_FORWARD_SPEED * 0.3)

MINIMUM_VOLTAGE = 10.0
MAXIMUM_TEMPERATURE = 85

LEG_SERVO_COUNT = 12

WATCHDOG_TIME_INTERVAL = 2000 # milliseconds between watchdog checks

HEAD_YAW_ID = 13

AX_GOAL_POSITION = 30
AX_12_VOLTAGE = 42
AX_12_TEMPERATURE = 43

FORWARD_ROT_Z = -0.02

RIGHT_RANGE_PIN = 'C1'
LEFT_RANGE_PIN = 'A1'
FRONT_RANGE_PIN = 'C3'

BUTTON_PIN = 'C2'

RANGE_MAX = 20

RED_LED = 1
GREEN_LED = 2
YELLOW_LED = 3
BLUE_LED = 4

#================================================
#
#       Class Roz
#

class Roz:

    def __init__(self):
        self.standingPose = [450, 570, 600, 420, 550, 460, 570, 450, 420, 600, 460, 550]
        self.controller = BioloidController()
        self.ikEngine = IKEngine()
        self.ikEngine.setController(self.controller)
        self.ikEngine.setupForWalk(self.standingPose)
        self.ikEngine.setTranTime(170)
        self.debug = False
        self.logger = Logger('logfile.txt')
        self.frontRangeFinder = RangeFinder(FRONT_RANGE_PIN, RANGE_MAX)
        self.leftRangeFinder = RangeFinder(LEFT_RANGE_PIN, RANGE_MAX)
        self.rightRangeFinder = RangeFinder(RIGHT_RANGE_PIN, RANGE_MAX)
        self.button = OneShotButton(BUTTON_PIN, False, 0)
        self.readSensors()
        self.shutdown = False
        self.setupHeadPosition()
        self.watchdogServoId = 1

    def setupHeadPosition(self):
        position = 511
        self.controller.writeData(HEAD_YAW_ID, AX_GOAL_POSITION, struct.pack('<H', position))

    def isButtonPushed(self):
        return self.button.isPressed()

    def readBatteryVoltage(self):
        return self.controller.readOneByteRegister(1, AX_12_VOLTAGE) / 10.0

    def checkBatteryVoltageAndTemperature(self, servoId):
        data = self.controller.readData(servoId, AX_12_VOLTAGE, 2)
        voltage = data[0] / 10
        temperature = data[1]
        self.log("Watchdog: Servo ID %d - Voltage: %3.1f volts, Temperature: %d C" % (servoId, voltage, temperature))
        if voltage < MINIMUM_VOLTAGE:
            self.log ('Battery too low: %3.1f volts - shutting down' % voltage)
            self.mainStateMachine.transitionTo(self.shutdownState)
        if temperature > MAXIMUM_TEMPERATURE:
            self.log ('Servo %d temperature too high: %d degrees C' % (servoId, temperature))
            self.mainStateMachine.transitionTo(self.shutdownState)

    def log(self, logString):
        self.logger.log(logString)

    def readSensors(self):
        self.frontRangeDistance = self.frontRangeFinder.getDistance()
        self.leftRangeDistance = self.leftRangeFinder.getDistance()
        self.rightRangeDistance = self.rightRangeFinder.getDistance()

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
            self.mainStateMachine.transitionTo(self.waitingForNoButtonState)

    #=====================================
    #
    #       Waiting for No Button State
    #

    def enterWaitingForNoButtonState(self):
        self.log('Entering WaitingForNoButtonState')

    def handleWaitingForNoButtonState(self):
        if not self.isButtonPushed():
            self.mainStateMachine.transitionTo(self.walkingState)

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
        if self.frontRangeDistance < FRONT_SENSOR_OBSTACLE:
            self.mainStateMachine.transitionTo (self.obstacleAvoidanceState)
        elif self.rightRangeDistance < SIDE_SENSOR_OBSTACLE:
            self.ikEngine.travelRotZ = SIDE_OBSTACLE_TURN_SPEED
        elif self.leftRangeDistance < SIDE_SENSOR_OBSTACLE:
            self.ikEngine.travelRotZ = -SIDE_OBSTACLE_TURN_SPEED
        else:
            self.ikEngine.travelRotZ = FORWARD_ROT_Z
        if self.isButtonPushed():
            self.mainStateMachine.transitionTo(self.shutdownState)

    #=====================================
    #
    #       Obstacle Avoidance State
    #

    def enterObstacleAvoidanceState(self):
        self.log('Entering ObstacleAvoidanceState')
        self.ikEngine.travelX = 0
        self.turnEndTime = pyb.millis() + FRONT_OBSTACLE_TURN_TIME
        if (self.leftRangeDistance < SIDE_SENSOR_OBSTACLE) & (self.rightRangeDistance < SIDE_SENSOR_OBSTACLE):
            self.log('Obstacles on both sides')
            self.ikEngine.travelRotZ = -FRONT_OBSTACLE_TURN_SPEED
            self.turnEndTime += FRONT_OBSTACLE_TURN_TIME
        elif self.leftRangeDistance < SIDE_SENSOR_OBSTACLE:
            self.log('Obstacle on left side')
            self.ikEngine.travelRotZ = -FRONT_OBSTACLE_TURN_SPEED
        elif self.rightRangeDistance < SIDE_SENSOR_OBSTACLE:
            self.log('Obstacle on right side')
            self.ikEngine.travelRotZ = FRONT_OBSTACLE_TURN_SPEED
        else: #nothing on either side, so pick a side at random
            self.log('Only front obstacle')
            if pyb.rng() & 1 == 0:
                self.ikEngine.travelRotZ = -FRONT_OBSTACLE_TURN_SPEED
            else:
                self.ikEngine.travelRotZ = FRONT_OBSTACLE_TURN_SPEED

    def handleObstacleAvoidanceState(self):
        if pyb.millis() > self.turnEndTime:
            self.mainStateMachine.transitionTo(self.walkingState)
        if self.isButtonPushed():
            self.mainStateMachine.transitionTo(self.shutdownState)

    #=====================================
    #
    #       Shutdown State
    #

    def enterShutdownState(self):
        self.ikEngine.travelX = 0
        self.ikEngine.travelRotZ = 0
        self.heartbeat.shutdown()
        self.shutdown = True


    #=====================================
    #
    #       Watchdog State Machine
    #

    def handleWatchdogState(self):
        # round robin the servos when checking voltage and temperature
        # assumes the leg servo ids are 1-LEG_SERVO_COUNT
        # note that checkBatteryVoltageAndTemperature() both checks and does a shutdown if required
        self.checkBatteryVoltageAndTemperature(self.watchdogServoId)
        self.watchdogServoId += 1
        if self.watchdogServoId > LEG_SERVO_COUNT:
            self.watchdogServoId = 1
        self.watchdogStateMachine.transitionTo(self.watchdogWaitState)

    def handleWatchdogWaitState(self):
        elapsedTime = self.watchdogStateMachine.getCurrentStateMillis()
        if elapsedTime > WATCHDOG_TIME_INTERVAL:
            self.watchdogStateMachine.transitionTo(self.watchdogState)


#=====================================

roz = Roz()
voltage = roz.readBatteryVoltage()
roz.log ('Battery: %3.1f volts' % voltage)

roz.waitingForButtonState = State("waitingForButton", roz.enterWaitingForButtonState, roz.handleWaitingForButtonState, None)
roz.waitingForNoButtonState = State("waitingForNoButton", roz.enterWaitingForNoButtonState, roz.handleWaitingForNoButtonState, None)
roz.walkingState = State("walking", roz.enterWalkingState, roz.handleWalkingState, None)
roz.obstacleAvoidanceState = State("obstacleAvoidance", roz.enterObstacleAvoidanceState, roz.handleObstacleAvoidanceState, None)
roz.shutdownState = State("shutdown", roz.enterShutdownState, None, None)
roz.mainStateMachine = FiniteStateMachine(roz.waitingForButtonState)

roz.heartbeat = HeartbeatLED(RED_LED)

roz.watchdogState = State("watchdog", None, roz.handleWatchdogState, None)
roz.watchdogWaitState = State("watchdog-wait", None, roz.handleWatchdogWaitState, None)
roz.watchdogStateMachine = FiniteStateMachine(roz.watchdogState)

while not roz.shutdown:
    roz.readSensors()
    roz.mainStateMachine.update()
    roz.ikEngine.handleIK()
    roz.watchdogStateMachine.update()
    roz.heartbeat.update()

roz.log ('Shutdown')
roz.logger.close()

