#!/usr/bin/env python

import time
import serial

def millis():
	return int(round(time.time() * 1000))

BIOLOID_SHIFT = 3
BIOLOID_FRAME_LENGTH = 33

AX_GOAL_POSITION = 30
AX_READ_DATA = 2
AX_SYNC_WRITE = 131

class BioloidController:

	def __init__(self):
		self.id = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
		self.pose = [512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512]
		self.nextPose = [512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512]
		self.speed = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
		self.interpolating = False
		self.playing = False
		self.servoCount = 12
		self.lastFrame = millis()
		self.serialPort = serial.Serial('/dev/ttyUSB0', 1000000, timeout=1)
		# self.serialPort = serial.Serial('COM4', 1000000, timeout=1)
		self.standingPose = [450, 570, 600, 420, 550, 460, 570, 450, 420, 600, 460, 550]

	# Load a pose into nextPose
	def loadPose(self, poseArray):
		for i in range(self.servoCount):
			self.nextPose[i] = (poseArray[i]) # << BIOLOID_SHIFT)
			#print 'loadPose[', self.id[i], '] = ', self.nextPose[i]

	# read the current robot's pose
	def readPose(self):
		for i in range(self.servoCount):
			self.pose[i] = (self.readTwoByteRegister(self.id[i], AX_GOAL_POSITION)) # << BIOLOID_SHIFT)
			#print 'readPose[', self.id[i], '] = ', self.pose[i]
			time.sleep(0.025)

	def writePose(self):
		#print 'Servo 1: ', (self.pose[0]) # >> BIOLOID_SHIFT)
		length = 4 + (self.servoCount * 3)
		checksum = 254 + length + AX_SYNC_WRITE + 2 + AX_GOAL_POSITION
		self.buffer = ""
		self.buffer += chr(0xFF)
		self.buffer += chr(0xFF)
		self.buffer += chr(0xFE) # broadcast id
		self.buffer += chr(length)
		self.buffer += chr(AX_SYNC_WRITE)
		self.buffer += chr(AX_GOAL_POSITION) # write to this location
		self.buffer += chr(2) # write two bytes for each servo
		for i in range(self.servoCount):
			temp = self.pose[i] # >> BIOLOID_SHIFT
			checksum += (temp & 0xFF) + (temp >> 8) + (self.id[i])
			self.buffer += chr(self.id[i])
			self.buffer += chr(temp & 0xFF)
			self.buffer += chr(temp >> 8)
		self.buffer += chr(0xFF - (checksum % 256))
		self.serialPort.write(self.buffer)
		self.buffer = ""

	def setHeadPosition(self):
		length = 4 + (2 * 3)
		checksum = 254 + length + AX_SYNC_WRITE + 2 + AX_GOAL_POSITION
		self.buffer = ""
		self.buffer += chr(0xFF)
		self.buffer += chr(0xFF)
		self.buffer += chr(0xFE) # broadcast id
		self.buffer += chr(length)
		self.buffer += chr(AX_SYNC_WRITE)
		self.buffer += chr(AX_GOAL_POSITION) # write to this location
		self.buffer += chr(2) # write two bytes for each servo
		temp = 511
		checksum += (temp & 0xFF) + (temp >> 8) + 13
		self.buffer += chr(13)
		self.buffer += chr(temp & 0xFF)
		self.buffer += chr(temp >> 8)
		temp = 490
		checksum += (temp & 0xFF) + (temp >> 8) + 14
		self.buffer += chr(14)
		self.buffer += chr(temp & 0xFF)
		self.buffer += chr(temp >> 8)
		self.buffer += chr(0xFF - (checksum % 256))
		self.serialPort.write(self.buffer)
		self.buffer = ""

	def readTwoByteRegister(self, deviceId, controlTableIndex):
		checksum = 0xFF - ((deviceId + 6 + controlTableIndex + 2) % 256)
		self.buffer = ""
		self.buffer += chr(0xFF)
		self.buffer += chr(0xFF)
		self.buffer += chr(deviceId)
		self.buffer += chr(4) # length
		self.buffer += chr(AX_READ_DATA)
		self.buffer += chr(controlTableIndex)
		self.buffer += chr(2)
		self.buffer += chr(checksum)
		self.serialPort.write(self.buffer)
		self.buffer = ""
		readBuffer = self.serialPort.read(8)
		if len(readBuffer) > 0:
			return ord(readBuffer[5]) + (ord(readBuffer[6]) << 8)
		else:
			return -1

	def readOneByteRegister(self, deviceId, controlTableIndex):
		checksum = 0xFF - ((deviceId + 6 + controlTableIndex + 1) % 256)
		self.buffer = ""
		self.buffer += chr(0xFF)
		self.buffer += chr(0xFF)
		self.buffer += chr(deviceId)
		self.buffer += chr(4) # length
		self.buffer += chr(AX_READ_DATA)
		self.buffer += chr(controlTableIndex)
		self.buffer += chr(1)
		self.buffer += chr(checksum)
		self.serialPort.write(self.buffer)
		self.buffer = ""
		readBuffer = self.serialPort.read(7)
		if len(readBuffer) > 0:
			return ord(readBuffer[5])
		else:
			return -1

	def interpolateSetup(self, time):
		frames = (time / BIOLOID_FRAME_LENGTH) + 1
		self.lastFrame = millis()
		#print 'pose diff = ', (self.nextPose[0] - self.pose[0])
		#print 'nextPose[ 1 ] = ', self.nextPose[0]
		#print 'pose[ 1 ] = ', self.pose[0]
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
		while (millis() - self.lastFrame < BIOLOID_FRAME_LENGTH):
			pass
		self.lastFrame = millis()
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


#============================================================

