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
import pyb


# Some preliminaries
def sq(x):
    return x * x


# Convert radians to servo position offset.
def radToServo(rads, resolution = 1024):
    if resolution == 4096:
        val = (rads * 100) / 51 * 25
        return int(val)
    else:
        val = (rads * 100) / 51 * 100
        return int(val)


class IKEngine:
    def __init__(self, debug=False):
        self.debug = debug    # do we print debug messages or not?

        #===========================
        #
        #   These eight variables are how the IK engine
        #   is controlled from outside

        self.bodyRotX = 0.0 # roll
        self.bodyRotY = 0.0 # pitch
        self.bodyRotZ = 0.0 # yaw

        self.bodyPosX = 0.0 # forwards/backwards
        self.bodyPosY = 0.0 # sideways

        # Note that X is forwards/backwards, Y is sideways
        self.travelX = 0
        self.travelY = 0
        self.travelRotZ = 0
        #===========================

        # Used for gait generation, holds the offsets for each leg
        self.gaitGen = self.defaultGait
        self.gait = {}
        self.gait["RF_GAIT"] = [0,0,0,0]
        self.gait["LF_GAIT"] = [0,0,0,0]
        self.gait["RR_GAIT"] = [0,0,0,0]
        self.gait["LR_GAIT"] = [0,0,0,0]
        self.setAmbleGait()
        self.step = 0

        # transitionTime is the # of milliseconds to transition from one gait position to the next
        # we recalculate IK every transitionTime ms, and then interpolate to move all the servos from
        # their current positions to the newly calculated positions over this amount of time
        self.setTransitionTime(150)

        self.COXA = 0
        self.FEMUR = 1
        self.TIBIA = 2

        #===========================
        #
        # All the following values are specific to the robot

        # End points are the locations of the feet, relative to the coxa pivot, in mm, in the standing position
        self.endPoints = {}
        self.endPoints["RIGHT_FRONT"] = [50, 130, 90]
        self.endPoints["RIGHT_REAR"] = [-50, 120, 90]
        self.endPoints["LEFT_FRONT"] = [50, -130, 90]
        self.endPoints["LEFT_REAR"] = [-50, -130, 90]

        # Used to generate servo values for IK
        self.mins = [0, 247, 378, 164, 165, 228, 158, 378, 247, 158, 165, 164, 228]
        self.maxs = [0, 649, 780, 860, 858, 867, 799, 780, 649, 866, 866, 785, 861]
        self.resolutions = [0, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024]
        self.neutrals = [0, 348, 675, 511, 511, 511, 511, 675, 348, 511, 511, 511, 511]
        self.signs = [0, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1]
        self.liftHeight = 20

        # These lengths measure pivot to pivot in the neutral position
        self.LENGTH_COXA = 52
        self.LENGTH_FEMUR = 68
        self.LENGTH_TIBIA = 120

        # Offsets of coxa pivot from body center point
        self.X_COXA = 75
        self.Y_COXA = 36

        # Servo IDs
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
        #===========================

    def setAmbleGait(self):
        # order holds which step in the gait the specified leg is starting its return stroke
        self.order = {"RF_GAIT":0,"LR_GAIT":0,"LF_GAIT":2,"RR_GAIT":2}
        self.stepsInCycle = 4
        self.pushSteps = 2

    def setSmoothAmbleGait(self):
        self.order = {"RF_GAIT":1,"LR_GAIT":2,"LF_GAIT":4,"RR_GAIT":5}
        self.stepsInCycle = 6
        self.pushSteps = 4

    def setController(self, aBioloidController):
        self.controller = aBioloidController

    def setLogger(self, aLogger):
        self.logger = aLogger

    def setTransitionTime(self, newTranTime):
        self.transitionTime = newTranTime
        self.cycleTime = (self.stepsInCycle * self.transitionTime) / 1000.0

    def setNextPose(self, servo, pos):
        self.controller.nextPose[servo - 1] = pos

    def defaultGait(self, leg):
        # Set the gait object to contain the offsets for the given leg, on the current step
        if abs(self.travelX) > 5 or abs(self.travelY) > 5 or abs(self.travelRotZ) > 0.05:  # are we moving?
            if self.order[leg] == self.step:
                # up, middle position
                self.gait[leg][0] = 0      # x
                self.gait[leg][1] = 0      # y
                self.gait[leg][2] = -self.liftHeight   # z
                self.gait[leg][3] = 0      # r
            elif (self.order[leg] + 1 == self.step) or (self.order[leg] - (self.stepsInCycle - 1) == self.step):
                # leg down, full forwards
                self.gait[leg][0] = self.travelX / 2
                self.gait[leg][1] = self.travelY / 2
                self.gait[leg][2] = 0
                self.gait[leg][3] = self.travelRotZ / 2
            else:
                # move body forward, leg backwards
                self.gait[leg][0] = self.gait[leg][0] - self.travelX / self.pushSteps
                self.gait[leg][1] = self.gait[leg][1] - self.travelY / self.pushSteps
                self.gait[leg][2] = 0
                self.gait[leg][3] = self.gait[leg][3] - self.travelRotZ / self.pushSteps
        return self.gait[leg]

    def standingGait(self, leg):
        # This is a fake gait, that basically specifies a standing pose
        return [0, 0, 0, 0]

    def setupForWalk(self):
        # Run IK while interpolating from the current pose to the default standing pose
        oldTransitionTime = self.transitionTime
        self.transitionTime = 1000
        self.gaitGen = self.standingGait
        self.controller.readPose()
        self.handleIK()
        while self.controller.interpolating:
            self.handleIK()
            pyb.delay(3) # milliseconds
        self.transitionTime = oldTransitionTime
        self.gaitGen = self.defaultGait

    def handleIK(self):
        if not self.controller.interpolating:
            self.doIK()
            self.controller.interpolateSetup(self.transitionTime)
            # self.odometer += (self.transitionTime / STRAIGHT_ODOMETER_FACTOR);
            # turnOdometer += (TURN_ODOMETER_FACTOR * (travelRotZ / (float(stepsInGait * transitionTime) / 1000.0)));
        self.controller.interpolateStep()

    def bodyIK(self, X, Y, Z, Xdisp, Ydisp, Zrot):
        # Compute offsets based on Body positions.
        # BodyIK based on the work of Xan
        answer = [0,0,0]   # (X,Y,Z)

        cosB = cos(self.bodyRotX)
        sinB = sin(self.bodyRotX)
        cosG = cos(self.bodyRotY)
        sinG = sin(self.bodyRotY)
        cosA = cos(self.bodyRotZ + Zrot)
        sinA = sin(self.bodyRotZ + Zrot)

        totalX = int(X + Xdisp + self.bodyPosX)
        totalY = int(Y + Ydisp + self.bodyPosY)

        answer[0] = int(totalX - int(totalX * cosG * cosA + totalY * sinB * sinG * cosA + Z * cosB * sinG * cosA - totalY * cosB * sinA + Z * sinB * sinA)) + self.bodyPosX
        answer[1] = int(totalY - int(totalX * cosG * sinA + totalY * sinB * sinG * sinA + Z * cosB * sinG * sinA + totalY * cosB * cosA - Z * sinB * cosA)) + self.bodyPosY
        answer[2] = int(Z - int(-totalX * sinG + totalY * sinB * cosG + Z * cosB * cosG))

        if self.debug:
            self.logger.log ("NUKE: BodyIK: %s" % answer)
        return answer

    def legIK(self, X, Y, Z, resolution):
        # Compute leg servo positions.
        answer = [0,0,0,0] # (coxa, femur, tibia)

        try:
            # first, make this a 2DOF problem... by solving coxa
            answer[self.COXA] = radToServo(atan2(X,Y), resolution)
            trueX = int(sqrt(sq(X) + sq(Y))) - self.LENGTH_COXA
            im = int(sqrt(sq(trueX) + sq(Z)))  # length of imaginary leg

            # get femur angle above horizon...
            q1 = -atan2(Z,trueX)
            d1 = sq(self.LENGTH_FEMUR) - sq(self.LENGTH_TIBIA) + sq(im)
            d2 = 2 * self.LENGTH_FEMUR * im
            q2 = acos(d1 / float(d2))
            answer[self.FEMUR] = radToServo(q1 + q2, resolution)

            # and tibia angle from femur...
            d1 = sq(self.LENGTH_FEMUR) - sq(im) + sq(self.LENGTH_TIBIA)
            d2 = 2 * self.LENGTH_TIBIA * self.LENGTH_FEMUR
            answer[self.TIBIA] = radToServo(acos(d1 / float(d2)) - 1.57, resolution)
        except:
            if self.debug:
                self.logger.log ("NUKE: LegIK FAILED")
            return [1024,1024,1024,0]

        if self.debug:
            self.logger.log ("NUKE: LegIK: %s" % answer)
        return answer

    def doLegIK(self, legName, legAbbreviation, xSign, ySign):
        fail = 0
        if self.gaitGen is not None:
            gait = self.gaitGen("%s_GAIT" % legAbbreviation)
        else:
            return fail
        if self.debug:
            self.logger.log ("NUKE: %s: %s " % (legName, [self.endPoints[legName][i] + gait[i] for i in range(3)]))
        servo = self.servos["%s Coxa" % legAbbreviation]
        request = self.bodyIK(self.endPoints[legName][0] + gait[0],
            self.endPoints[legName][1] + gait[1],
            self.endPoints[legName][2] + gait[2],
            xSign * self.X_COXA, ySign * self.Y_COXA, gait[3])
        solution = self.legIK(xSign * (self.endPoints[legName][0] + request[0] + gait[0]),
            ySign * (self.endPoints[legName][1] + request[1] + gait[1]),
            self.endPoints[legName][2] + request[2] + gait[2],
            self.resolutions[servo])
        output = self.neutrals[servo] + self.signs[servo] * solution[self.COXA]
        if self.mins[servo] < output < self.maxs[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                self.logger.log ("NUKE: %s_COXA FAIL: %s" % (legAbbreviation, output))
            fail += 1
        servo = self.servos["%s Femur" % legAbbreviation]
        output = self.neutrals[servo] + self.signs[servo] * solution[self.FEMUR]
        if self.mins[servo] < output < self.maxs[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                self.logger.log ("NUKE: %s_FEMUR FAIL: %s" % (legAbbreviation, output))
            fail += 1
        servo = self.servos["%s Tibia" % legAbbreviation]
        output = self.neutrals[servo] + self.signs[servo] * solution[self.TIBIA]
        if self.mins[servo] < output < self.maxs[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                self.logger.log ("NUKE: %s_TIBIA FAIL: %s" % (legAbbreviation, output))
            fail += 1
        return fail

    def doIK(self):
        fail = self.doLegIK("RIGHT_FRONT", "RF", 1, 1)
        fail += self.doLegIK("RIGHT_REAR", "RR", -1, 1)
        fail += self.doLegIK("LEFT_FRONT", "LF", 1, -1)
        fail += self.doLegIK("LEFT_REAR", "LR", -1, -1)

        #self.log ("step: " + self.step + " of " + self.stepsInCycle)
        self.step = (self.step + 1) % self.stepsInCycle
        return fail
