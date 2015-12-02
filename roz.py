#!/usr/bin/env python

"""
  Roz - a Bioloid Quad Walker

"""

import pyb

from BioloidController import BioloidController
from FSM import State, FiniteStateMachine
from Nuke import IKEngine
from Support import RangeFinder, HeartbeatLED, OneShotButton, Logger, arduino_map
import math

FRONT_SENSOR_OBSTACLE = 30 # cm
SIDE_SENSOR_OBSTACLE = 30
SIDE_SENSOR_CLEAR_OBSTACLE = 45

FRONT_OBSTACLE_TURN_SPEED = 0.5
SIDE_OBSTACLE_TURN_SPEED = 0.3
FRONT_OBSTACLE_TURN_TIMEOUT = 5000
FRONT_OBSTACLE_TURN_CONTINUE_TIMEOUT = 500

TRANSITION_TIME = 150
MAX_FORWARD_SPEED = 160

START_FORWARD_SPEED = (MAX_FORWARD_SPEED * 0.3)

MINIMUM_VOLTAGE = 10.0
MAXIMUM_TEMPERATURE = 85

LEG_SERVO_COUNT = 12

WATCHDOG_TIME_INTERVAL = 5000 # milliseconds between watchdog checks

HEAD_YAW_ID = 13
HEAD_YAW_CENTER = 511
HEAD_YAW_MIN = 306
HEAD_YAW_MAX = 717

AX_1_DEGREE_INCREMENT = 3.41
AX_5_DEGREE_INCREMENT = AX_1_DEGREE_INCREMENT * 5

AX_GOAL_POSITION = 30
AX_MOVING_SPEED = 32
AX_PRESENT_POSITION = 36
AX_12_VOLTAGE = 42
AX_12_TEMPERATURE = 43

FORWARD_ROT_Z = 0 # we can use this to offset any tendency of the robot to turn while moving forwards

RIGHT_RANGE_PIN = 'C1'
LEFT_RANGE_PIN = 'A1'
FRONT_RANGE_PIN = 'C3'

BUTTON_PIN = 'C2'

RANGE_MAX = 50

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
        self.logger = Logger('logfile.txt')
        self.standingPose = [450, 570, 600, 420, 550, 460, 570, 450, 420, 600, 460, 550]
        self.controller = BioloidController()
        self.ikEngine = IKEngine()
        self.ikEngine.setController(self.controller)
        self.ikEngine.setLogger(self.logger)
        self.setupHeadPosition()
        self.ikEngine.setupForWalk(self.standingPose)
        self.ikEngine.setTranTime(TRANSITION_TIME)
        self.debug = False
        self.frontRangeFinder = RangeFinder(FRONT_RANGE_PIN, RANGE_MAX)
        self.leftRangeFinder = RangeFinder(LEFT_RANGE_PIN, RANGE_MAX)
        self.rightRangeFinder = RangeFinder(RIGHT_RANGE_PIN, RANGE_MAX)
        self.button = OneShotButton(BUTTON_PIN, False, 0)
        self.readSensors()
        self.shutdown = False
        self.watchdogServoId = 1
        self.waitingForButtonState = State("waitingForButton", self.enterWaitingForButtonState, self.handleWaitingForButtonState, None)
        self.waitingForNoButtonState = State("waitingForNoButton", self.enterWaitingForNoButtonState, self.handleWaitingForNoButtonState, None)
        self.walkingState = State("walking", self.enterWalkingState, self.handleWalkingState, None)
        self.obstacleAvoidanceState = State("obstacleAvoidance", self.enterObstacleAvoidanceState, self.handleObstacleAvoidanceState, None)
        self.obstacleAvoidanceScanState = State("obstacleAvoidanceScan", self.enterObstacleAvoidanceScanState, self.handleObstacleAvoidanceScanState, None)
        self.obstacleAvoidanceContinueState = State("obstacleAvoidanceContinue", self.enterObstacleAvoidanceContinueState, self.handleObstacleAvoidanceContinueState, None)
        self.shutdownState = State("shutdown", self.enterShutdownState, None, None)
        self.mainStateMachine = FiniteStateMachine(self.waitingForButtonState)
        self.heartbeat = HeartbeatLED(RED_LED)
        self.watchdogState = State("watchdog", None, self.handleWatchdogState, None)
        self.watchdogWaitState = State("watchdog-wait", None, self.handleWatchdogWaitState, None)
        self.watchdogStateMachine = FiniteStateMachine(self.watchdogState)

    def setupHeadPosition(self):
        self.controller.rampServoTo(HEAD_YAW_ID, HEAD_YAW_CENTER)

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

    def mapObstacleSpace(self):
        # do a scan, return the angle (0-center-relative) to the largest chunk of empty space
        # if no empty space is found, return None
        self.controller.rampServoTo(HEAD_YAW_ID, HEAD_YAW_MIN)
        pyb.delay(250) # pause for the movement to settle
        values = []
        for position in range(HEAD_YAW_MIN * 100, HEAD_YAW_MAX * 100, int(AX_5_DEGREE_INCREMENT * 100)):
            start = pyb.millis()
            end = start + 40
            i_pos = int(position / 100)
            self.controller.setPosition(HEAD_YAW_ID, i_pos)
            values.append(self.frontRangeFinder.getDistance())
            pyb.delay(max(0, end - pyb.millis())) # we want each loop iteration to take 40ms

        self.controller.rampServoTo(HEAD_YAW_ID, HEAD_YAW_CENTER)
        groups = []
        lastIndex = None
        startGroup = 0
        for (index, value) in enumerate(values):
            if value == 50:
                if lastIndex == index - 1:
                    startGroup = lastIndex
                if lastIndex is None:
                    lastIndex = index
            else:
                if lastIndex is not None:
                    groups.append((startGroup, index - 1))
                lastIndex = None
        if values[-1] == 50:
            groups.append((startGroup, len(values) - 1))

        if len(groups) == 0:
            return None

        maxLength = 0
        maxGroup = None
        for pairs in groups:
            groupLength = pairs[1] - pairs[0] + 1
            if groupLength > maxLength:
                maxGroup = pairs
                maxLength = groupLength

        center = (maxGroup[0] + maxGroup[1]) // 2
        centerPosition = arduino_map(center, 0, len(values), 306, 717)
        centerAngle = arduino_map(centerPosition, 0, 1023, 0, 300)
        return centerAngle

    def log(self, logString):
        self.logger.log(logString)

    def readSensors(self):
        self.frontRangeDistance = self.frontRangeFinder.getDistance()
        self.leftRangeDistance = self.leftRangeFinder.getDistance()
        self.rightRangeDistance = self.rightRangeFinder.getDistance()

    def update(self):
        self.readSensors()
        self.mainStateMachine.update()
        self.ikEngine.handleIK()
        self.watchdogStateMachine.update()
        self.heartbeat.update()


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
            self.log("Obstacle on right side, angling left")
            self.ikEngine.travelRotZ = SIDE_OBSTACLE_TURN_SPEED
        elif self.leftRangeDistance < SIDE_SENSOR_OBSTACLE:
            self.log("Obstacle on left side, angling right")
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
        self.turnTimeoutTime = pyb.millis() + FRONT_OBSTACLE_TURN_TIMEOUT
        if (self.leftRangeDistance < SIDE_SENSOR_CLEAR_OBSTACLE) & (self.rightRangeDistance < SIDE_SENSOR_CLEAR_OBSTACLE):
            # stuff on both sides, pick a direction at random to turn
            if pyb.rng() & 1 == 0:
                self.log('Obstacles on both sides, turning right')
                self.ikEngine.travelRotZ = -FRONT_OBSTACLE_TURN_SPEED
            else:
                self.log('Obstacles on both sides, turning left')
                self.ikEngine.travelRotZ = FRONT_OBSTACLE_TURN_SPEED
        elif self.leftRangeDistance < SIDE_SENSOR_CLEAR_OBSTACLE:
            self.log('Obstacle on left side, turning right')
            self.ikEngine.travelRotZ = -FRONT_OBSTACLE_TURN_SPEED
        elif self.rightRangeDistance < SIDE_SENSOR_CLEAR_OBSTACLE:
            self.log('Obstacle on right side, turning left')
            self.ikEngine.travelRotZ = FRONT_OBSTACLE_TURN_SPEED
        else: #nothing on either side, so pick a side at random
            if pyb.rng() & 1 == 0:
                self.log('Only front obstacle, turning right')
                self.ikEngine.travelRotZ = -FRONT_OBSTACLE_TURN_SPEED
            else:
                self.log('Only front obstacle, turning left')
                self.ikEngine.travelRotZ = FRONT_OBSTACLE_TURN_SPEED

    def handleObstacleAvoidanceState(self):
        if self.frontRangeDistance >= FRONT_SENSOR_OBSTACLE:
            self.mainStateMachine.transitionTo(self.obstacleAvoidanceContinueState)
        if pyb.millis() > self.turnTimeoutTime:
            self.log("Obstacle turn timeout, do scan")
            self.mainStateMachine.transitionTo(self.obstacleAvoidanceScanState)
        if self.isButtonPushed():
            self.mainStateMachine.transitionTo(self.shutdownState)

    #=====================================
    #
    #       Obstacle Avoidance Scan State
    #

    def enterObstacleAvoidanceScanState(self):
        # this function shuts down the FSM for a second or two
        self.log('Entering ObstacleAvoidanceScanState')
        self.ikEngine.travelX = 0
        self.ikEngine.travelRotZ = 0
        self.ikEngine.setupForWalk(self.standingPose)
        self.ikEngine.bodyPosX = -50 # move the body forwards so the head clears the legs
        self.ikEngine.setupForWalk(self.standingPose)
        self.ikEngine.bodyPosX = 0
        openAngle = self.mapObstacleSpace()
        if openAngle is None:
            self.obstacleScanTurnTime = pyb.millis()
        else:
            openAngleRadians = math.radians(openAngle)
            self.obstacleScanTurnTime = pyb.millis() + int((abs(openAngleRadians) / FRONT_OBSTACLE_TURN_SPEED) * 1000)
            if openAngle > 0:
                self.log("Found opening at angle %d - turning left" % openAngle)
                self.ikEngine.travelRotZ = FRONT_OBSTACLE_TURN_SPEED
            else:
                self.log("Found opening at angle %d - turning right" % openAngle)
                self.ikEngine.travelRotZ = -FRONT_OBSTACLE_TURN_SPEED

    def handleObstacleAvoidanceScanState(self):
        if pyb.millis() >= self.obstacleScanTurnTime:
            self.log("Obstacle scan turn done, back to walking")
            self.mainStateMachine.transitionTo(self.walkingState)
        if self.isButtonPushed():
            self.mainStateMachine.transitionTo(self.shutdownState)

    #=====================================
    #
    #       Obstacle Avoidance Continue State
    #

    def enterObstacleAvoidanceContinueState(self):
        self.log('Entering ObstacleAvoidanceContinueState')
        self.turnTimeoutTime = pyb.millis() + FRONT_OBSTACLE_TURN_CONTINUE_TIMEOUT

    def handleObstacleAvoidanceContinueState(self):
        if pyb.millis() > self.turnTimeoutTime:
            self.log("Done turning, back to walking")
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
roz.log ('Battery: %3.1f volts' % roz.readBatteryVoltage())

while not roz.shutdown:
    roz.update()

roz.log ('Shutdown')
roz.logger.close()
