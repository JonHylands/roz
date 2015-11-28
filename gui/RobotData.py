#!/usr/bin/env python

import random
import math

class RobotData:
	def __init__(self):
		self.frontRange = 0
		self.leftRange = 0
		self.rightRange = 0
		self.frontClose = True
		self.leftClose = True
		self.rightClose = True
		self.leftRPM = 0
		self.leftCurrent = 0
		self.rightRPM = 0
		self.rightCurrent = 0
		self.heading = 0
		self.pitch = 0
		self.roll = 0
		self.motionSpeed = 0
		self.motionRotation = 0
		self.odometer = 0
		self.batteryVoltage = 12.3

	def update(self):
		self.heading = (self.heading + 5) % 360
		self.frontRange = (self.frontRange + 1) % 25
		self.frontClose = (self.frontRange <= 4)
		self.leftRange = (self.leftRange - 1) % 25
		self.leftClose = (self.leftRange <= 4)
		self.rightRange = (self.rightRange + 1) % 25
		self.rightClose = (self.rightRange <= 4)
		self.leftRPM = (self.leftRPM + 1) % 300
		self.rightRPM = (self.rightRPM + 1) % 300
		self.leftCurrent = 35
		self.rightCurrent = 37
		self.motionSpeed = 27
		self.motionRotation = -0.02
		self.odometer += 3

		if random.randint(1,100) < 40:
			self.pitch += math.copysign(2, self.pitch)
		else:
			self.pitch += math.copysign(2, -self.pitch)

		if random.randint(1,100) < 40:
			self.roll += math.copysign(2, self.roll)
		else:
			self.roll += math.copysign(2, -self.roll)
