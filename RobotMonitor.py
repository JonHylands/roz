#!/usr/bin/env python

"""
  RobotMonitor - a Tkinter-based robot monitor GUI

"""

import os, sys, math
import platform

from Tkinter import *
import tkFont

from PIL import Image, ImageTk

WINDOW_WIDTH = 500
WINDOW_HEIGHT = 1000
LINE_WIDTH = 3
THIN_WIDTH = 1

LIGHT_BACK_COLOR = "WhiteSmoke"
BACK_COLOR = "LightGray"
DETECTED_COLOR = "FireBrick"

COMPASS_FONT = ("Arial", "15")

class RobotMonitor:
	def __init__(self):
		self.master = Tk()
		self.master.title("Robot Monitor")
# 		self.master.protocol("WM_DELETE_WINDOW", self.handleCloseButton)
		self.sensorCanvas = Canvas(self.master, width=WINDOW_WIDTH, height=WINDOW_WIDTH, bg=LIGHT_BACK_COLOR)
		self.sensorCanvas.pack(side=LEFT, anchor=NW)
		self.drawSensorBackground()

	def percentToWindow(self, percentage):
		return int(math.floor(percentage * WINDOW_WIDTH))

	def drawIRSensorIndicator(self, points, startAngle, endAngle, color):
		self.sensorCanvas.create_arc(points, start=startAngle, extent=endAngle, outline="black", fill=color, width=THIN_WIDTH)

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

	def drawCompassFrame(self):
		percentagePoints = [0.27, 0.15, 0.49, 0.37]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_oval(points, width=THIN_WIDTH, fill=LIGHT_BACK_COLOR)
		percentagePoints = [0.36, 0.24, 0.40, 0.28]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_oval(points, width=THIN_WIDTH, fill=BACK_COLOR)
		self.sensorCanvas.create_text(self.percentToWindow(0.38), self.percentToWindow(0.18), font=COMPASS_FONT, text="N")
		self.sensorCanvas.create_text(self.percentToWindow(0.30), self.percentToWindow(0.26), font=COMPASS_FONT, text="W")
		self.sensorCanvas.create_text(self.percentToWindow(0.38), self.percentToWindow(0.34), font=COMPASS_FONT, text="S")
		self.sensorCanvas.create_text(self.percentToWindow(0.46), self.percentToWindow(0.26), font=COMPASS_FONT, text="E")

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

	def drawPitch(self, pitchAngle):
		percentagePoints = [0.27, 0.40, 0.49, 0.62]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_oval(points, width=THIN_WIDTH, fill=LIGHT_BACK_COLOR)
		percentagePoints = [0.27, 0.51, 0.49, 0.51]
		points = map(self.percentToWindow, percentagePoints)
		centerX = self.percentToWindow(0.38)
		centerY = self.percentToWindow(0.51)
		self.sensorCanvas.create_line(self.rotateLine(points, (centerX, centerY), -pitchAngle), width=LINE_WIDTH)
		image = Image.open("Roz-Pitch.png")
		newSize = self.percentToWindow(0.16)
		image = image.resize((newSize, newSize), Image.ANTIALIAS)
		image = image.rotate(pitchAngle, Image.BICUBIC, True)
		photo = ImageTk.PhotoImage(image)
		self.pitchImage = photo
		self.sensorCanvas.create_image(centerX, centerY, image=photo)

	def drawRoll(self, rollAngle):
		percentagePoints = [0.51, 0.40, 0.73, 0.62]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_oval(points, width=THIN_WIDTH, fill=LIGHT_BACK_COLOR)
		percentagePoints = [0.51, 0.51, 0.73, 0.51]
		points = map(self.percentToWindow, percentagePoints)
		centerX = self.percentToWindow(0.62)
		centerY = self.percentToWindow(0.51)
		self.sensorCanvas.create_line(self.rotateLine(points, (centerX, centerY), -rollAngle), width=LINE_WIDTH)
		image = Image.open("Roz-Roll.png")
		newSize = self.percentToWindow(0.16)
		image = image.resize((newSize, newSize), Image.ANTIALIAS)
		image = image.rotate(rollAngle, Image.BICUBIC, True)
		photo = ImageTk.PhotoImage(image)
		self.rollImage = photo
		self.sensorCanvas.create_image(centerX, centerY, image=photo)

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
		self.drawFrontIRSensor(False)

		# left IR sensor
		percentagePoints = [0.12, 0.38, 0.20, 0.38, 0.20, 0.68, 0.12, 0.68, 0.08, 0.60, 0.08, 0.46]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_polygon(points, outline="black", fill=LIGHT_BACK_COLOR, width=THIN_WIDTH)
		self.drawLeftIRSensor(False)

		# right IR sensor
		percentagePoints = [0.88, 0.38, 0.80, 0.38, 0.80, 0.68, 0.88, 0.68, 0.92, 0.60, 0.92, 0.46]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_polygon(points, outline="black", fill=LIGHT_BACK_COLOR, width=THIN_WIDTH)
		self.drawRightIRSensor(False)

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

		# heading, pitch, roll
		self.drawCompassFrame()
		self.drawPitch(0.0)
		self.drawRoll(0.0)

		# misc
		percentagePoints = [0.27, 0.68, 0.73, 0.91]
		points = map(self.percentToWindow, percentagePoints)
		self.sensorCanvas.create_rectangle(points, outline="black", fill=LIGHT_BACK_COLOR, width=THIN_WIDTH)

##############################################

monitor = RobotMonitor()
mainloop()
