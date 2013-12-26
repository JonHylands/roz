#!/usr/bin/env python

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
