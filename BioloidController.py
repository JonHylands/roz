#!/usr/bin/env python

from stm_uart_port import UART_Port
from bus import Bus, BusError
from packet import ErrorCode
from Support import Logger
import pyb
import struct

BIOLOID_FRAME_LENGTH = 33

AX_GOAL_POSITION = 30
AX_MOVING_SPEED = 32
AX_PRESENT_POSITION = 36
AX_READ_DATA = 2
AX_WRITE_DATA = 3
AX_SYNC_WRITE = 131

SLOW_SERVO_MOVE_SPEED = 100

class BioloidController:

    def __init__(self, useLogger = False):
        self.id = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
        self.pose = [512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512]
        self.nextPose = [512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512]
        self.speed = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.interpolating = False
        self.playing = False
        self.servoCount = 12
        self.lastFrame = pyb.millis()
        self.port = UART_Port(1, 1000000)
        self.bus = Bus(self.port, show=Bus.SHOW_PACKETS)
        # Bus.SHOW_NONE
        # Bus.SHOW_COMMANDS
        # Bus.SHOW_PACKETS
        if useLogger:
            self.logger = Logger('sync_log.txt')
        else:
            self.logger = None

    # Load a pose into nextPose
    def loadPose(self, poseArray):
        for i in range(self.servoCount):
            self.nextPose[i] = (poseArray[i]) # << BIOLOID_SHIFT)
            #print ('loadPose[', self.id[i], '] = ', self.nextPose[i])

    def isLogging(self):
        return self.logger is not None

    # read the current robot's pose
    def readPose(self):
        for i in range(self.servoCount):
            self.pose[i] = (self.readTwoByteRegister(self.id[i], AX_GOAL_POSITION)) # << BIOLOID_SHIFT)
            #print ('readPose[', self.id[i], '] = ', self.pose[i])
            pyb.delay(25)

    def writePose(self):
        values = []
        logging = self.isLogging()
        if logging:
            logValues = []
        for i in range(self.servoCount):
            poseValue = int(self.pose[i])
            values.append(struct.pack('<H', poseValue))
            if logging:
                logValues.append(poseValue)
        self.bus.sync_write(self.id, AX_GOAL_POSITION, values)
        if logging:
            self.logger.log(logValues)

    def slowMoveServoTo(self, deviceId, targetPosition, speed = SLOW_SERVO_MOVE_SPEED, scanFunction = None):
        oldSpeed = self.readTwoByteRegister(deviceId, AX_MOVING_SPEED)
        currentPosition = self.readTwoByteRegister(deviceId, AX_PRESENT_POSITION)
        self.writeTwoByteRegister(deviceId, AX_MOVING_SPEED, speed)
        self.writeTwoByteRegister(deviceId, AX_GOAL_POSITION, targetPosition)
        done = False
        scanCount = 0
        lastPosition = 0
        startTime = pyb.millis()
        while abs(currentPosition - targetPosition) > 5:
            currentPosition = self.readTwoByteRegister(deviceId, AX_PRESENT_POSITION)
            if scanFunction is not None:
                if currentPosition != lastPosition:
                    scanCount += 1
                    lastPosition = currentPosition
                    scanFunction(currentPosition, scanCount)
            pyb.delay(1)
        self.writeTwoByteRegister(deviceId, AX_MOVING_SPEED, oldSpeed)
        if scanFunction is not None:
            scanFunction(targetPosition, scanCount + 1)
        print("Elapsed Time: %d" % (pyb.millis() - startTime))

    def rampServoTo(self, deviceId, targetPosition):
        currentPosition = self.readTwoByteRegister(deviceId, AX_PRESENT_POSITION) # present position
        if targetPosition > currentPosition:
            stepDelta = 1
            stepAccel = 2
            comparison = lambda: targetPosition > (currentPosition + stepDelta)
        else:
            stepDelta = -1
            stepAccel = -2
            comparison = lambda: currentPosition > (targetPosition - stepDelta)
        while comparison():
            movePosition = currentPosition + stepDelta
            stepDelta += stepAccel
            self.setPosition(deviceId, movePosition)
            currentPosition = self.readTwoByteRegister(deviceId, AX_PRESENT_POSITION) # present position
            pyb.delay(25)
        self.setPosition(deviceId, targetPosition)

    def setPosition(self, deviceId, position):
        self.writeTwoByteRegister(deviceId, AX_GOAL_POSITION, position)

    def writeData(self, deviceId, controlTableIndex, byteData):
        try:
            result = self.bus.write(deviceId, controlTableIndex, byteData)
        except BusError as ex:
            if ex.get_error_code() == ErrorCode.CHECKSUM:
                print ("CHECKSUM Error - retrying...")
                return self.bus.write(deviceId, controlTableIndex, byteData)
            raise
        return result

    def writeTwoByteRegister(self, deviceId, controlTableIndex, value):
        return self.writeData(deviceId, controlTableIndex, struct.pack('<H', value))

    def writeOneByteRegister(self, deviceId, controlTableIndex, value):
        return self.writeData(deviceId, controlTableIndex, struct.pack('B', value))

    def readTwoByteRegister(self, deviceId, controlTableIndex):
        values = self.readData(deviceId, controlTableIndex, 2)
        return struct.unpack('<H', values)[0]

    def readOneByteRegister(self, deviceId, controlTableIndex):
        values = self.readData(deviceId, controlTableIndex, 1)
        return struct.unpack('B', values)[0]

    def readData(self, deviceId, controlTableIndex, count):
        try:
            result = self.bus.read(deviceId, controlTableIndex, count)
        except BusError as ex:
            if ex.get_error_code() == ErrorCode.CHECKSUM:
                print ("CHECKSUM Error - retrying...")
                return self.bus.read(deviceId, controlTableIndex, count)
            raise
        return result

    def interpolateSetup(self, time):
        frames = (time / BIOLOID_FRAME_LENGTH) + 1
        self.lastFrame = pyb.millis()
        for i in range(self.servoCount):
            if self.nextPose[i] > self.pose[i]:
                self.speed[i] = (self.nextPose[i] - self.pose[i]) / frames + 1
            else:
                self.speed[i] = (self.pose[i] - self.nextPose[i]) / frames + 1
        self.interpolating = True

    def interpolateStep(self):
        if not self.interpolating:
            return
        complete = self.servoCount
        while (pyb.millis() - self.lastFrame < BIOLOID_FRAME_LENGTH):
            pass
        self.lastFrame = pyb.millis()
        for i in range(self.servoCount):
            diff = self.nextPose[i] - self.pose[i]
            if diff == 0:
                complete -= 1
            else:
                if diff > 0:
                    if diff < self.speed[i]:
                        self.pose[i] = self.nextPose[i]
                        complete -= 1
                    else:
                        self.pose[i] += self.speed[i]
                else:
                    if (-diff) < self.speed[i]:
                        self.pose[i] = self.nextPose[i]
                        complete -= 1
                    else:
                        self.pose[i] -= self.speed[i]
        if complete <= 0:
            self.interpolating = False
        self.writePose()
