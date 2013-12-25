#!/usr/bin/env python

"""
  PyPose: Bioloid pose system for arbotiX robocontroller
  Copyright (c) 2008-2010 Michael E. Ferguson.  All right reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

from math import cos,sin,atan2,sqrt,acos
import time

# Some preliminaries
def sq(x):
	return x*x

# Convert radians to servo position offset.
def radToServo(rads, resolution = 1024):
	if resolution == 4096:
		val = (rads*100)/51 * 25;
		return int(val)
	else:
		val = (rads*100)/51 * 100;
		return int(val)

COXA = 0
FEMUR = 1
TIBIA = 2

class IKEngine:
	def __init__(self, opt=4, debug=False, gaitGen = None):
		self.debug = debug	# do we print debug messages or not?
		self.gaitGen = self.defaultGait  # any gait generation?

		self.bodyRotX = 0.0
		self.bodyRotY = 0.0
		self.bodyRotZ = 0.0

		self.bodyPosX = 0.0
		self.bodyPosY = 0.0

		# Used for gait generation.
		self.gait = {}
		self.gait["RIGHT_FRONT"] = [50, 130, 90]
		self.gait["RIGHT_REAR"] = [-50, 120, 90]
		self.gait["LEFT_FRONT"] = [50, -130, 90]
		self.gait["LEFT_REAR"] = [-50, -130, 90]

		self.gait["RF_GAIT"] = [0,0,0,0]
		self.gait["LF_GAIT"] = [0,0,0,0]
		self.gait["RR_GAIT"] = [0,0,0,0]
		self.gait["LR_GAIT"] = [0,0,0,0]
		self.order = {"RF_GAIT":0,"LR_GAIT":0,"LF_GAIT":2,"RR_GAIT":2}
		self.stepsInCycle = 4
		self.pushSteps = 2
		self.tranTime = 175
		self.cycleTime = (self.stepsInCycle * self.tranTime) / 1000.0;

		# Used to generate servo values for IK
		self.mins = [0, 247, 378, 164, 165, 228, 158, 376, 246, 158, 165, 164, 228]
		self.maxs = [0, 649, 780, 860, 858, 867, 799, 784, 654, 866, 866, 785, 861]
		self.resolutions = [0, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024]
		self.neutrals = [0, 350, 673, 511, 511, 511, 511, 673, 350, 511, 511, 511, 511]
		self.signs = [0, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1]
		self.step = 0

		self.travelX = 0
		self.travelY = 0
		self.travelRotZ = 0

		self.config()

		self.logDebug = False

	def setController(self, aBioloidController):
		self.controller = aBioloidController

	def setTranTime(self, newTranTime):
		self.tranTime = newTranTime

	def setNextPose(self, servo, pos):
		self.controller.nextPose[servo - 1] = pos

	def defaultGait(self,leg):

		if abs(self.travelX)>5 or abs(self.travelY)>5 or abs(self.travelRotZ) > 0.05:   # are we moving?
			if(self.order[leg] == self.step):
				# up, middle position
				self.gait[leg][0] = 0	   # x
				self.gait[leg][1] = 0	   # y
				self.gait[leg][2] = -20   # z
				self.gait[leg][3] = 0	   # r
			elif (self.order[leg]+1 == self.step) or (self.order[leg]-(self.stepsInCycle-1) == self.step):   # gaits in step -1
				# leg down!
				self.gait[leg][0] = self.travelX/2 #(self.travelX * self.cycleTime * self.pushSteps) / (2 * self.stepsInCycle) # travelX/2
				self.gait[leg][1] = self.travelY/2 #(self.travelY * self.cycleTime * self.pushSteps) / (2 * self.stepsInCycle) # travelY/2
				self.gait[leg][2] = 0
				self.gait[leg][3] = self.travelRotZ/2 # (self.travelRotZ * self.cycleTime * self.pushSteps) / (2 * self.stepsInCycle) # travelRotZ/2
			else:
				# move body forward
				self.gait[leg][0] = self.gait[leg][0] - self.travelX/2 # (self.travelX * self.cycleTime) / self.stepsInCycle # travelX/6
				self.gait[leg][1] = self.gait[leg][1] - self.travelY/2 # (self.travelY * self.cycleTime) / self.stepsInCycle # travelY/6
				self.gait[leg][2] = 0
				self.gait[leg][3] = self.gait[leg][3] - self.travelRotZ/2 # (self.travelRotZ * self.cycleTime) / self.stepsInCycle # travelRotZ/6
		#print 'Gait: ', leg, ' -> ', self.gait[leg]
		return self.gait[leg]


	def setupForWalk(self):
		self.controller.readPose()
		self.controller.loadPose(self.controller.standingPose)
		self.controller.interpolateSetup(1000)
		while self.controller.interpolating:
			self.controller.interpolateStep()
			time.sleep(0.003)
		self.controller.setHeadPosition()


	def handleIK(self):
		if not self.controller.interpolating:
			self.doIK();
			self.controller.interpolateSetup(self.tranTime);
			# self.odometer += (self.tranTime / STRAIGHT_ODOMETER_FACTOR);
			# turnOdometer += (TURN_ODOMETER_FACTOR * (travelRotZ / (float(stepsInGait * tranTime) / 1000.0)));
		self.controller.interpolateStep();


	def config(self):
		# VARS = coxaLen, femurLen, tibiaLen, xBody, yBody, xCOG, yCOG
		self.L_COXA = 52
		self.L_FEMUR = 68
		self.L_TIBIA = 120
		self.X_COXA = 75
		self.Y_COXA = 36

		# SERVOS = Coxa, Femur, Tibia (LF, RF, LR, RR)
		self.servos = {}
		self.servos["RF Coxa"] = 1
		self.servos["LF Coxa"] = 2
		self.servos["RF Femur"] = 3
		self.servos["LF Femur"] = 4
		self.servos["RF Tibia"] = 5
		self.servos["LF Tibia"] = 6

		self.servos["RR Coxa"] = 7
		self.servos["LR Coxa"] = 8
		self.servos["RR Femur"] = 9
		self.servos["LR Femur"] = 10
		self.servos["RR Tibia"] = 11
		self.servos["LR Tibia"] = 12


	def bodyIK(self, X, Y, Z, Xdisp, Ydisp, Zrot):
		""" Compute offsets based on Body positions.
		  BodyIK based on the work of Xan """
		ans = [0,0,0]   # (X,Y,Z)

		cosB = cos(self.bodyRotX)
		sinB = sin(self.bodyRotX)
		cosG = cos(self.bodyRotY)
		sinG = sin(self.bodyRotY)
		cosA = cos(self.bodyRotZ+Zrot)
		sinA = sin(self.bodyRotZ+Zrot)

		totalX = int(X + Xdisp + self.bodyPosX);
		totalY = int(Y + Ydisp + self.bodyPosY);

		ans[0] = int(totalX - int(totalX*cosG*cosA + totalY*sinB*sinG*cosA + Z*cosB*sinG*cosA - totalY*cosB*sinA + Z*sinB*sinA)) + self.bodyPosX
		ans[1] = int(totalY - int(totalX*cosG*sinA + totalY*sinB*sinG*sinA + Z*cosB*sinG*sinA + totalY*cosB*cosA - Z*sinB*cosA)) + self.bodyPosY
		ans[2] = int(Z - int(-totalX*sinG + totalY*sinB*cosG + Z*cosB*cosG))

		if self.debug:
			print "BodyIK:",ans
		return ans

	def legIK(self, X, Y, Z, resolution):
		""" Compute leg servo positions. """
		ans = [0,0,0,0] # (coxa, femur, tibia)

		try:
			# first, make this a 2DOF problem... by solving coxa
			ans[0] = radToServo(atan2(X,Y), resolution)
			trueX = int(sqrt(sq(X)+sq(Y))) - self.L_COXA
			im = int(sqrt(sq(trueX)+sq(Z)))  # length of imaginary leg

			# get femur angle above horizon...
			q1 = -atan2(Z,trueX)
			d1 = sq(self.L_FEMUR)-sq(self.L_TIBIA)+sq(im)
			d2 = 2*self.L_FEMUR*im
			q2 = acos(d1/float(d2))
			ans[1] = radToServo(q1+q2, resolution)

			# and tibia angle from femur...
			d1 = sq(self.L_FEMUR)-sq(im)+sq(self.L_TIBIA)
			d2 = 2*self.L_TIBIA*self.L_FEMUR;
			ans[2] = radToServo(acos(d1/float(d2))-1.57, resolution)
		except:
			if self.debug:
				"LegIK FAILED"
			return [1024,1024,1024,0]

		if self.debug:
			print "LegIK:",ans
		return ans

	def doIK(self):
		fail = 0
		gait = [0,0,0,0]	# [x,y,z,r]

		# right front leg
		if self.gaitGen != None:
			gait = self.gaitGen("RF_GAIT")
		if self.debug:
			print "RIGHT_FRONT: ", [self.gait["RIGHT_FRONT"][i] + gait[i] for i in range(3)]
		servo = self.servos["RF Coxa"]
		req = self.bodyIK(self.gait["RIGHT_FRONT"][0]+gait[0], self.gait["RIGHT_FRONT"][1]+gait[1], self.gait["RIGHT_FRONT"][2]+gait[2], self.X_COXA, self.Y_COXA, gait[3])
		sol = self.legIK(self.gait["RIGHT_FRONT"][0]+req[0]+gait[0],self.gait["RIGHT_FRONT"][1]+req[1]+gait[1],self.gait["RIGHT_FRONT"][2]+req[2]+gait[2], self.resolutions[servo])
		output = self.neutrals[servo]+self.signs[servo]*sol[COXA]
		if output < self.maxs[servo] and output > self.mins[servo]:
			self.setNextPose(servo, output)
		else:
			if self.debug:
				print "RF_COXA FAIL: ", output
			fail = fail + 1
		servo = self.servos["RF Femur"]
		output = self.neutrals[servo]+self.signs[servo]*sol[FEMUR]
		if output < self.maxs[servo] and output > self.mins[servo]:
			self.setNextPose(servo, output)
		else:
			if self.debug:
				print "RF_FEMUR FAIL: ", output
			fail = fail + 1
		servo = self.servos["RF Tibia"]
		output = self.neutrals[servo]+self.signs[servo]*sol[TIBIA]
		if output < self.maxs[servo] and output > self.mins[servo]:
			self.setNextPose(servo, output)
		else:
			if self.debug:
				print "RF_TIBIA FAIL: ",output
			fail = fail + 1

		# right rear leg
		if self.gaitGen != None:
			gait = self.gaitGen("RR_GAIT")
		if self.debug:
			print "RIGHT_REAR: ", [self.gait["RIGHT_REAR"][i] + gait[i] for i in range(3)]
		servo = self.servos["RR Coxa"]
		req = self.bodyIK(self.gait["RIGHT_REAR"][0]+gait[0],self.gait["RIGHT_REAR"][1]+gait[1],self.gait["RIGHT_REAR"][2]+gait[2], -self.X_COXA, self.Y_COXA, gait[3])
		sol = self.legIK(-self.gait["RIGHT_REAR"][0]-req[0]-gait[0],self.gait["RIGHT_REAR"][1]+req[1]+gait[1],self.gait["RIGHT_REAR"][2]+req[2]+gait[2], self.resolutions[servo])
		output = self.neutrals[servo]+self.signs[servo]*sol[COXA]
		if output < self.maxs[servo] and output > self.mins[servo]:
			self.setNextPose(servo, output)
		else:
			if self.debug:
				print "RR_COXA FAIL: ", output
			fail = fail + 1
		servo = self.servos["RR Femur"]
		output = self.neutrals[servo]+self.signs[servo]*sol[FEMUR]
		if output < self.maxs[servo] and output > self.mins[servo]:
			self.setNextPose(servo, output)
		else:
			if self.debug:
				print "RR_FEMUR FAIL:", output
			fail = fail + 1
		servo = self.servos["RR Tibia"]
		output = self.neutrals[servo]+self.signs[servo]*sol[TIBIA]
		if output < self.maxs[servo] and output > self.mins[servo]:
			self.setNextPose(servo, output)
		else:
			if self.debug:
				print "RR_TIBIA FAIL:", output
			fail = fail + 1

		# left front leg
		if self.gaitGen != None:
			gait = self.gaitGen("LF_GAIT")
		if self.debug:
			print "LEFT_FRONT: ", [self.gait["LEFT_FRONT"][i] + gait[i] for i in range(3)]
		servo = self.servos["LF Coxa"]
		req = self.bodyIK(self.gait["LEFT_FRONT"][0]+gait[0],self.gait["LEFT_FRONT"][1]+gait[1],self.gait["LEFT_FRONT"][2]+gait[2], self.X_COXA, -self.Y_COXA, gait[3])
		sol = self.legIK(self.gait["LEFT_FRONT"][0]+req[0]+gait[0],-self.gait["LEFT_FRONT"][1]-req[1]-gait[1],self.gait["LEFT_FRONT"][2]+req[2]+gait[2], self.resolutions[servo])
		output = self.neutrals[servo]+self.signs[servo]*sol[COXA]
		if output < self.maxs[servo] and output > self.mins[servo]:
			self.setNextPose(servo, output)
		else:
			if self.debug:
				print "LF_COXA FAIL:", output
			fail = fail + 1
		servo = self.servos["LF Femur"]
		output = self.neutrals[servo]+self.signs[servo]*sol[FEMUR]
		if output < self.maxs[servo] and output > self.mins[servo]:
			self.setNextPose(servo, output)
		else:
			if self.debug:
				print"LF_FEMUR FAIL:", output
			fail = fail + 1
		servo = self.servos["LF Tibia"]
		output = self.neutrals[servo]+self.signs[servo]*sol[TIBIA]
		if output < self.maxs[servo] and output > self.mins[servo]:
			self.setNextPose(servo, output)
		else:
			if self.debug:
				print "LF_TIBIA FAIL:", output
			fail = fail + 1

		# left rear leg
		if self.gaitGen != None:
			gait = self.gaitGen("LR_GAIT")
		if self.debug:
			print "LEFT_REAR: ", [self.gait["LEFT_REAR"][i] + gait[i] for i in range(3)]
		servo = self.servos["LR Coxa"]
		req = self.bodyIK(self.gait["LEFT_REAR"][0]+gait[0],self.gait["LEFT_REAR"][1]+gait[1],self.gait["LEFT_REAR"][2]+gait[2], -self.X_COXA, -self.Y_COXA, gait[3])
		sol = self.legIK(-self.gait["LEFT_REAR"][0]-req[0]-gait[0],-self.gait["LEFT_REAR"][1]-req[1]-gait[1],self.gait["LEFT_REAR"][2]+req[2]+gait[2], self.resolutions[servo])
		output = self.neutrals[servo]+self.signs[servo]*sol[COXA]
		if output < self.maxs[servo] and output > self.mins[servo]:
			self.setNextPose(servo, output)
		else:
			if self.debug:
				print "LR_COXA FAIL:", output
			fail = fail + 1
		servo = self.servos["LR Femur"]
		output = self.neutrals[servo]+self.signs[servo]*sol[FEMUR]
		if output < self.maxs[servo] and output > self.mins[servo]:
			self.setNextPose(servo, output)
		else:
			if self.debug:
				print "LR_FEMUR FAIL:",output
			fail = fail + 1
		servo = self.servos["LR Tibia"]
		output = self.neutrals[servo]+self.signs[servo]*sol[TIBIA]
		if output < self.maxs[servo] and output > self.mins[servo]:
			self.setNextPose(servo, output)
		else:
			if self.debug:
				print "LR_TIBIA FAIL:", output
			fail = fail + 1

		self.step = self.step + 1
		if self.step > (self.stepsInCycle - 1):
			self.step = 0   #gaitStep = (gaitStep+1)%stepsInGait
		return fail

	def log(self, logString):
		if self.logDebug:
			print logString

