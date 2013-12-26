#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""
  RobotMonitor - a Tkinter-based robot monitor GUI

"""

import os, sys, math
import platform
import datetime as dt

from Tkinter import *
import tkFont

from PIL import Image, ImageTk

from RobotData import RobotData


WINDOW_WIDTH = 500
WINDOW_HEIGHT = 1000
LINE_WIDTH = 3
THIN_WIDTH = 1

FRAME_DELAY = 100

LIGHT_BACK_COLOR = "WhiteSmoke"
BACK_COLOR = "LightGray"
DETECTED_COLOR = "FireBrick"

COMPASS_FONT_HEIGHT = 15
COMPASS_FONT = ("Arial", str(COMPASS_FONT_HEIGHT))

def unix_time(aDate):
	epoch = dt.datetime.utcfromtimestamp(0)
	delta = aDate - epoch
	return delta.total_seconds()

def unix_time_millis(aDate):
	return unix_time(aDate) * 1000.0

class RobotMonitor:
	def __init__(self):
		self.master = Tk()
		self.master.title("Robot Monitor")
# 		self.master.protocol("WM_DELETE_WINDOW", self.handleCloseButton)
		self.sensorCanvas = Canvas(self.master, width=WINDOW_WIDTH, height=WINDOW_WIDTH, bg=LIGHT_BACK_COLOR)
		self.sensorCanvas.pack(side=LEFT, anchor=NW)

		self.redrawSet = []
		self.data = RobotData()
		self.drawSensorBackground()
		self.frameCount = 0
		self.startTime = unix_time_millis(dt.datetime.utcnow())
		self.updateSensorValues()


#######################################
#
#	Support Methods
#

	def percentToWindow(self, percentage):
		return int(math.floor(percentage * WINDOW_WIDTH))

	def rotatePoint(self, points, center, angle):
		rads = math.radians(angle)
		deltaX = center[0] - points[0]
		deltaY = center[1] - points[1]
		resultX = (deltaX * math.cos(rads)) - (deltaY * math.sin(rads)) + center[0]
		resultY = (deltaX * math.sin(rads)) + (deltaY * math.cos(rads)) + center[1]
		return (resultX, resultY)

	def rotateLine(self, points, center, angle):
		one = self.rotatePoint((points[0], points[1]), (center[0], center[1]), angle)
		two = self.rotatePoint((points[2], points[3]), (center[0], center[1]), angle)
		return (one[0], one[1], two[0], two[1])

	def removeRedrawSet(self):
		for item in self.redrawSet:
			self.sensorCanvas.delete(item)
		self.redrawSet = []

	def addRedraw(self, item):
		self.redrawSet.append(item)

	def drawTwoLine(self, amount, unit, center):
		self.addRedraw(self.sensorCanvas.create_text((center[0], (center[1] - COMPASS_FONT_HEIGHT)), font=COMPASS_FONT, text=amount))
		self.addRedraw(self.sensorCanvas.create_text((center[0], (center[1] + COMPASS_FONT_HEIGHT)), font=COMPASS_FONT, text=unit))

	def drawLabel(self, label, center):
		self.addRedraw(self.sensorCanvas.create_text(center, font=COMPASS_FONT, text=label))

	def drawOneLine(self, amount, unit, center):
		text = str(amount) + " " + str(unit)
		self.addRedraw(self.sensorCanvas.create_text(center, font=COMPASS_FONT, text=text))

	def drawPrefixedOneLine(self, prefix, amount, unit, center):
		text = str(prefix) + " " + str(amount) + " " + str(unit)
		self.addRedraw(self.sensorCanvas.create_text(center, font=COMPASS_FONT, text=text))


#######################################
#
#	Individual Sensor Redraw Methods
#

	def drawIRSensorIndicator(self, points, startAngle, endAngle, color):
		self.addRedraw(self.sensorCanvas.create_arc(points, start=startAngle, extent=endAngle, outline="black", fill=color, width=THIN_WIDTH))

	def drawFrontIRSensor(self, detected):
		percentagePoints = [0.45, -0.02, 0.55, 0.04]
		points = map(self.percentToWindow, percentagePoints)
		if detected:
			color = DETECTED_COLOR
		else:
			color = BACK_COLOR
		self.drawIRSensorIndicator(points, 0, -180, color)

	def drawLeftIRSensor(self, detected):
		percentagePoints = [0.05, 0.48, 0.11, 0.58]
		points = map(self.percentToWindow, percentagePoints)
		if detected:
			color = DETECTED_COLOR
		else:
			color = BACK_COLOR
		self.drawIRSensorIndicator(points, -90, 180, color)

	def drawRightIRSensor(self, detected):
		percentagePoints = [0.89, 0.48, 0.95, 0.58]
		points = map(self.percentToWindow, percentagePoints)
		if detected:
			color = DETECTED_COLOR
		else:
			color = BACK_COLOR
		self.drawIRSensorIndicator(points, 90, 180, color)

	def drawHeading(self):
		percentagePoints = [0.38, 0.22, 0.38, 0.16]
		points = map(self.percentToWindow, percentagePoints)
		centerX = self.percentToWindow(0.38)
		centerY = self.percentToWindow(0.26)
		self.addRedraw(self.sensorCanvas.create_line(self.rotateLine(points, (centerX, centerY), 180 + self.data.heading), width=LINE_WIDTH, fill="Red"))
		self.drawLabel(self.data.heading, (centerX, centerY))
		self.addRedraw(self.sensorCanvas.create_text(self.percentToWindow(0.38), self.percentToWindow(0.18), font=COMPASS_FONT, text="N"))
		self.addRedraw(self.sensorCanvas.create_text(self.percentToWindow(0.30), self.percentToWindow(0.26), font=COMPASS_FONT, text="W"))
		self.addRedraw(self.sensorCanvas.create_text(self.percentToWindow(0.38), self.percentToWindow(0.34), font=COMPASS_FONT, text="S"))
		self.addRedraw(self.sensorCanvas.create_text(self.percentToWindow(0.46), self.percentToWindow(0.26), font=COMPASS_FONT, text="E"))

	def drawPitch(self):
		percentagePoints = [0.27, 0.51, 0.49, 0.51]
		points = map(self.percentToWindow, percentagePoints)
		centerX = self.percentToWindow(0.38)
		centerY = self.percentToWindow(0.51)
		self.addRedraw(self.sensorCanvas.create_line(self.rotateLine(points, (centerX, centerY), -self.data.pitch), width=LINE_WIDTH, fill="Blue"))
		image = Image.open("Roz-Pitch.png")
		newSize = self.percentToWindow(0.16)
		image = image.resize((newSize, newSize), Image.ANTIALIAS)
		image = image.rotate(self.data.pitch, Image.BICUBIC, True)
		photo = ImageTk.PhotoImage(image)
		self.pitchImage = photo
		self.addRedraw(self.sensorCanvas.create_image(centerX, centerY, image=photo))

	def drawRoll(self):
		percentagePoints = [0.51, 0.51, 0.73, 0.51]
		points = map(self.percentToWindow, percentagePoints)
		centerX = self.percentToWindow(0.62)
		centerY = self.percentToWindow(0.51)
		self.addRedraw(self.sensorCanvas.create_line(self.rotateLine(points, (centerX, centerY), -self.data.roll), width=LINE_WIDTH, fill="Blue"))
		image = Image.open("Roz-Roll.png")
		newSize = self.percentToWindow(0.16)
		image = image.resize((newSize, newSize), Image.ANTIALIAS)
		image = image.rotate(self.data.roll, Image.BICUBIC, True)
		photo = ImageTk.PhotoImage(image)
		self.rollImage = photo
		self.addRedraw(self.sensorCanvas.create_image(centerX, centerY, image=photo))


#######################################
#
#	Draw Background Parts
#

	def drawSensorBackground(self):

		# left tread
		percentagePoints = [0.01, 0.08, 0.22, 0.99]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_rectangle(points, width=LINE_WIDTH, fill=BACK_COLOR)

		# right tread
		percentagePoints = [0.78, 0.08, 0.99, 0.99]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_rectangle(points, width=LINE_WIDTH, fill=BACK_COLOR)

		# main body
		percentagePoints = [0.25, 0.05, 0.75, 0.93]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_rectangle(points, width=LINE_WIDTH, fill=BACK_COLOR)

		# front IR sensor
		percentagePoints = [0.35, 0.05, 0.42, 0.01, 0.58, 0.01, 0.65, 0.05, 0.65, 0.13, 0.35, 0.13]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_polygon(points, outline="black", fill=LIGHT_BACK_COLOR, width=THIN_WIDTH)

		# left IR sensor
		percentagePoints = [0.12, 0.38, 0.20, 0.38, 0.20, 0.68, 0.12, 0.68, 0.08, 0.60, 0.08, 0.46]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_polygon(points, outline="black", fill=LIGHT_BACK_COLOR, width=THIN_WIDTH)

		# right IR sensor
		percentagePoints = [0.88, 0.38, 0.80, 0.38, 0.80, 0.68, 0.88, 0.68, 0.92, 0.60, 0.92, 0.46]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_polygon(points, outline="black", fill=LIGHT_BACK_COLOR, width=THIN_WIDTH)

		# left rpm
		percentagePoints = [0.03, 0.10, 0.20, 0.27]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_rectangle(points, outline="black", fill=LIGHT_BACK_COLOR, width=THIN_WIDTH)

		# left current
		percentagePoints = [0.03, 0.80, 0.20, 0.97]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_rectangle(points, outline="black", fill=LIGHT_BACK_COLOR, width=THIN_WIDTH)

		# right rpm
		percentagePoints = [0.80, 0.10, 0.97, 0.27]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_rectangle(points, outline="black", fill=LIGHT_BACK_COLOR, width=THIN_WIDTH)

		# right current
		percentagePoints = [0.80, 0.80, 0.97, 0.97]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_rectangle(points, outline="black", fill=LIGHT_BACK_COLOR, width=THIN_WIDTH)

		# motion
		percentagePoints = [0.51, 0.15, 0.73, 0.37]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_rectangle(points, width=THIN_WIDTH, fill=LIGHT_BACK_COLOR)

		# heading
		percentagePoints = [0.27, 0.15, 0.49, 0.37]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_oval(points, width=THIN_WIDTH, fill=LIGHT_BACK_COLOR)

		# pitch
		percentagePoints = [0.27, 0.40, 0.49, 0.62]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_oval(points, width=THIN_WIDTH, fill=LIGHT_BACK_COLOR)

		# roll
		percentagePoints = [0.51, 0.40, 0.73, 0.62]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_oval(points, width=THIN_WIDTH, fill=LIGHT_BACK_COLOR)

		# misc
		percentagePoints = [0.27, 0.68, 0.73, 0.91]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_rectangle(points, outline="black", fill=LIGHT_BACK_COLOR, width=THIN_WIDTH)


#######################################
#
#	Sensor Update
#

	def updateSensorValues(self):
		startTime = unix_time_millis(dt.datetime.utcnow())
		self.removeRedrawSet()
		self.drawFrontIRSensor(self.data.frontClose)
		self.drawOneLine(self.data.frontRange, "mm", (self.percentToWindow(0.5), self.percentToWindow(0.09)))
		self.drawLeftIRSensor(self.data.leftClose)
		self.drawTwoLine(self.data.leftRange, "mm", (self.percentToWindow(0.16), self.percentToWindow(0.53)))
		self.drawRightIRSensor(self.data.rightClose)
		self.drawTwoLine(self.data.rightRange, "mm", (self.percentToWindow(0.84), self.percentToWindow(0.53)))
		self.drawTwoLine(self.data.leftRPM, "RPM", (self.percentToWindow(0.11), self.percentToWindow(0.18)))
		self.drawTwoLine(self.data.leftCurrent, "mA", (self.percentToWindow(0.11), self.percentToWindow(0.88)))
		self.drawTwoLine(self.data.rightRPM, "RPM", (self.percentToWindow(0.88), self.percentToWindow(0.18)))
		self.drawTwoLine(self.data.rightCurrent, "mA", (self.percentToWindow(0.88), self.percentToWindow(0.88)))
		self.drawLabel("MOTION", (self.percentToWindow(0.62), self.percentToWindow(0.20)))
		self.drawOneLine(self.data.motionSpeed, "mm/s", (self.percentToWindow(0.62), self.percentToWindow(0.27)))
		self.drawOneLine(self.data.motionRotation, "°/s", (self.percentToWindow(0.62), self.percentToWindow(0.32)))
		self.drawHeading()
		self.drawPitch()
		self.drawRoll()
		self.drawLabel("MISC", (self.percentToWindow(0.5), self.percentToWindow(0.73)))
		self.drawPrefixedOneLine("odometer:", self.data.odometer, "mm", (self.percentToWindow(0.5), self.percentToWindow(0.80)))
		self.drawPrefixedOneLine("battery:", self.data.batteryVoltage, "volts", (self.percentToWindow(0.5), self.percentToWindow(0.85)))
		self.data.update()
		drawTime = int(unix_time_millis(dt.datetime.utcnow()) - startTime)
		waitTime = max(FRAME_DELAY - drawTime, 1)
		self.drawPrefixedOneLine("draw time:", drawTime, "ms", (self.percentToWindow(0.5), self.percentToWindow(0.98)))
		self.sensorCanvas.after(waitTime, self.updateSensorValues)


#######################################
#
#	Main
#

monitor = RobotMonitor()
mainloop()
