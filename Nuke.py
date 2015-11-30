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

COXA = 0
FEMUR = 1
TIBIA = 2

class IKEngine:
    def __init__(self, debug=False, gaitGen = None):
        self.debug = debug    # do we print debug messages or not?
        if gaitGen is None:
            self.gaitGen = self.defaultGait  # any gait generation?

        self.bodyRotX = 0.0 # roll
        self.bodyRotY = 0.0 # pitch
        self.bodyRotZ = 0.0 # yaw

        self.bodyPosX = 0.0 # forwards/backwards
        self.bodyPosY = 0.0 # sideways

        # End points are the locations of the feet, relative to the coxa pivot, in mm, in the neutral position
        self.endPoints = {}
        self.endPoints["RIGHT_FRONT"] = [50, 130, 90]
        self.endPoints["RIGHT_REAR"] = [-50, 120, 90]
        self.endPoints["LEFT_FRONT"] = [50, -130, 90]
        self.endPoints["LEFT_REAR"] = [-50, -130, 90]

        # Used for gait generation, holds the offsets for each leg
        self.gait = {}
        self.gait["RF_GAIT"] = [0,0,0,0]
        self.gait["LF_GAIT"] = [0,0,0,0]
        self.gait["RR_GAIT"] = [0,0,0,0]
        self.gait["LR_GAIT"] = [0,0,0,0]
        self.setAmbleGait()
        
        # transitionTime is the # of milliseconds to transition from one gait position to the next
        # we recalculate IK every transitionTime ms, and then interpolate to move all the servos from
        # their current positions to the newly calculated positions
        self.transitionTime = 175
        self.cycleTime = (self.stepsInCycle * self.transitionTime) / 1000.0;

        # Used to generate servo values for IK
        # These values are driven from the physical layout of the robot
        self.mins = [0, 247, 378, 164, 165, 228, 158, 378, 247, 158, 165, 164, 228]
        self.maxs = [0, 649, 780, 860, 858, 867, 799, 780, 649, 866, 866, 785, 861]
        self.resolutions = [0, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024]
        self.neutrals = [0, 348, 675, 511, 511, 511, 511, 675, 348, 511, 511, 511, 511]
        self.signs = [0, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1]
        self.step = 0
        self.liftHeight = 20

        # Note that X is forwards/backwards, Y is sideways
        self.travelX = 0
        self.travelY = 0
        self.travelRotZ = 0

        self.config()

    # order holds which step in the gait the specified leg is starting its return stroke
    def setAmbleGait(self):
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

    def setTranTime(self, newTranTime):
        self.transitionTime = newTranTime

    def setNextPose(self, servo, pos):
        self.controller.nextPose[servo - 1] = pos

    # set the gait object to contain the offsets for the given leg, on the current step
    def defaultGait(self,leg):
        if abs(self.travelX)>5 or abs(self.travelY)>5 or abs(self.travelRotZ) > 0.05:   # are we moving?
            if(self.order[leg] == self.step):
                # up, middle position
                self.gait[leg][0] = 0      # x
                self.gait[leg][1] = 0      # y
                self.gait[leg][2] = -20   # z
                self.gait[leg][3] = 0      # r
            elif (self.order[leg]+1 == self.step) or (self.order[leg]-(self.stepsInCycle-1) == self.step):   # gaits in step -1
                # leg down!
                self.gait[leg][0] = self.travelX / 2 #(self.travelX * self.cycleTime * self.pushSteps) / (2 * self.stepsInCycle) # travelX/2
                self.gait[leg][1] = self.travelY / 2 #(self.travelY * self.cycleTime * self.pushSteps) / (2 * self.stepsInCycle) # travelY/2
                self.gait[leg][2] = 0
                self.gait[leg][3] = self.travelRotZ / 2 # (self.travelRotZ * self.cycleTime * self.pushSteps) / (2 * self.stepsInCycle) # travelRotZ/2
            else:
                # move body forward
                self.gait[leg][0] = self.gait[leg][0] - self.travelX / self.pushSteps # (self.travelX * self.cycleTime) / self.stepsInCycle # travelX/6
                self.gait[leg][1] = self.gait[leg][1] - self.travelY / self.pushSteps # (self.travelY * self.cycleTime) / self.stepsInCycle # travelY/6
                self.gait[leg][2] = 0
                self.gait[leg][3] = self.gait[leg][3] - self.travelRotZ / self.pushSteps # (self.travelRotZ * self.cycleTime) / self.stepsInCycle # travelRotZ/6
        #print 'Gait: ', leg, ' -> ', self.gait[leg]
        return self.gait[leg]


    def setupForWalk(self, pose):
        self.controller.readPose()
        self.controller.loadPose(pose)
        oldTransitionTime = self.transitionTime
        self.transitionTime = 1000
        self.handleIK()
        while self.controller.interpolating:
            self.handleIK()
            pyb.delay(3) # milliseconds
        self.transitionTime = oldTransitionTime

    def handleIK(self):
        if not self.controller.interpolating:
            self.doIK()
            self.controller.interpolateSetup(self.transitionTime)
            # self.odometer += (self.transitionTime / STRAIGHT_ODOMETER_FACTOR);
            # turnOdometer += (TURN_ODOMETER_FACTOR * (travelRotZ / (float(stepsInGait * transitionTime) / 1000.0)));
        self.controller.interpolateStep()

    def config(self):
        # VARS = coxaLen, femurLen, tibiaLen, xBody, yBody, xCOG, yCOG
        self.LENGTH_COXA = 52
        self.LENGTH_FEMUR = 68
        self.LENGTH_TIBIA = 120
        # Offsets of coxa pivot from body center point
        self.X_COXA = 75
        self.Y_COXA = 36

        # SERVOS = Coxa, Femur, Tibia
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
        """ Compute leg servo positions. """
        answer = [0,0,0,0] # (coxa, femur, tibia)

        try:
            # first, make this a 2DOF problem... by solving coxa
            answer[0] = radToServo(atan2(X,Y), resolution)
            trueX = int(sqrt(sq(X) + sq(Y))) - self.LENGTH_COXA
            im = int(sqrt(sq(trueX) + sq(Z)))  # length of imaginary leg

            # get femur angle above horizon...
            q1 = -atan2(Z,trueX)
            d1 = sq(self.LENGTH_FEMUR) - sq(self.LENGTH_TIBIA) + sq(im)
            d2 = 2 * self.LENGTH_FEMUR * im
            q2 = acos(d1 / float(d2))
            answer[1] = radToServo(q1 + q2, resolution)

            # and tibia angle from femur...
            d1 = sq(self.LENGTH_FEMUR) - sq(im) + sq(self.LENGTH_TIBIA)
            d2 = 2 * self.LENGTH_TIBIA * self.LENGTH_FEMUR
            answer[2] = radToServo(acos(d1 / float(d2)) - 1.57, resolution)
        except:
            if self.debug:
                self.logger.log ("NUKE: LegIK FAILED")
            return [1024,1024,1024,0]

        if self.debug:
            self.logger.log ("NUKE: LegIK: %s" % answer)
        return answer

    def doRightFrontIK(self):
        fail = 0
        if self.gaitGen != None:
            gait = self.gaitGen("RF_GAIT")
        else:
            return fail
        if self.debug:
            self.logger.log ("NUKE: RIGHT_FRONT: %s " % [self.endPoints["RIGHT_FRONT"][i] + gait[i] for i in range(3)])
        servo = self.servos["RF Coxa"]
        request = self.bodyIK(self.endPoints["RIGHT_FRONT"][0] + gait[0],
            self.endPoints["RIGHT_FRONT"][1] + gait[1],
            self.endPoints["RIGHT_FRONT"][2] + gait[2],
            self.X_COXA, self.Y_COXA, gait[3])
        solution = self.legIK(self.endPoints["RIGHT_FRONT"][0] + request[0] + gait[0],
            self.endPoints["RIGHT_FRONT"][1] + request[1] + gait[1],
            self.endPoints["RIGHT_FRONT"][2] + request[2] + gait[2],
            self.resolutions[servo])
        output = self.neutrals[servo] + self.signs[servo] * solution[COXA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                self.logger.log ("NUKE: RF_COXA FAIL: %s" % output)
            fail = fail + 1
        servo = self.servos["RF Femur"]
        output = self.neutrals[servo] + self.signs[servo] * solution[FEMUR]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                self.logger.log ("NUKE: RF_FEMUR FAIL: %s" % output)
            fail = fail + 1
        servo = self.servos["RF Tibia"]
        output = self.neutrals[servo] + self.signs[servo] * solution[TIBIA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                self.logger.log ("NUKE: RF_TIBIA FAIL: %s" % output)
            fail = fail + 1
        return fail

    def doRightRearIK(self):
        fail = 0
        if self.gaitGen != None:
            gait = self.gaitGen("RR_GAIT")
        else:
            return fail
        if self.debug:
            self.logger.log ("NUKE: RIGHT_REAR: %s" % [self.endPoints["RIGHT_REAR"][i] + gait[i] for i in range(3)])
        servo = self.servos["RR Coxa"]
        request = self.bodyIK(self.endPoints["RIGHT_REAR"][0] + gait[0],
            self.endPoints["RIGHT_REAR"][1] + gait[1],
            self.endPoints["RIGHT_REAR"][2] + gait[2],
            -self.X_COXA, self.Y_COXA, gait[3])
        solution = self.legIK(-self.endPoints["RIGHT_REAR"][0] - request[0] - gait[0],
            self.endPoints["RIGHT_REAR"][1] + request[1] + gait[1],
            self.endPoints["RIGHT_REAR"][2] + request[2] + gait[2],
            self.resolutions[servo])
        output = self.neutrals[servo] + self.signs[servo] * solution[COXA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                self.logger.log ("NUKE: RR_COXA FAIL: %s" % output)
            fail = fail + 1
        servo = self.servos["RR Femur"]
        output = self.neutrals[servo] + self.signs[servo] * solution[FEMUR]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                self.logger.log ("NUKE: RR_FEMUR FAIL: %s" % output)
            fail = fail + 1
        servo = self.servos["RR Tibia"]
        output = self.neutrals[servo] + self.signs[servo] * solution[TIBIA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                self.logger.log ("NUKE: RR_TIBIA FAIL: %s" % output)
            fail = fail + 1
        return fail

    def doLeftFrontIK(self):
        fail = 0
        if self.gaitGen != None:
            gait = self.gaitGen("LF_GAIT")
        else:
            return fail
        if self.debug:
            self.logger.log ("NUKE: LEFT_FRONT: %s" % [self.endPoints["LEFT_FRONT"][i] + gait[i] for i in range(3)])
        servo = self.servos["LF Coxa"]
        request = self.bodyIK(self.endPoints["LEFT_FRONT"][0] + gait[0],
            self.endPoints["LEFT_FRONT"][1] + gait[1],
            self.endPoints["LEFT_FRONT"][2] + gait[2],
            self.X_COXA, -self.Y_COXA, gait[3])
        solution = self.legIK(self.endPoints["LEFT_FRONT"][0] + request[0] + gait[0],
            -self.endPoints["LEFT_FRONT"][1] - request[1] - gait[1],
            self.endPoints["LEFT_FRONT"][2] + request[2] + gait[2],
            self.resolutions[servo])
        output = self.neutrals[servo] + self.signs[servo] * solution[COXA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                self.logger.log ("NUKE: LF_COXA FAIL: %s" % output)
            fail = fail + 1
        servo = self.servos["LF Femur"]
        output = self.neutrals[servo] + self.signs[servo] * solution[FEMUR]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                self.logger.log ("NUKE: LF_FEMUR FAIL: %s" % output)
            fail = fail + 1
        servo = self.servos["LF Tibia"]
        output = self.neutrals[servo] + self.signs[servo] * solution[TIBIA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                self.logger.log ("NUKE: LF_TIBIA FAIL: %s" % output)
            fail = fail + 1
        return fail

    def doLeftRearIK(self):
        fail = 0
        if self.gaitGen != None:
            gait = self.gaitGen("LR_GAIT")
        else:
            return fail
        if self.debug:
            self.logger.log ("NUKE: LEFT_REAR: %s" % [self.endPoints["LEFT_REAR"][i] + gait[i] for i in range(3)])
        servo = self.servos["LR Coxa"]
        request = self.bodyIK(self.endPoints["LEFT_REAR"][0] + gait[0],
            self.endPoints["LEFT_REAR"][1] + gait[1],
            self.endPoints["LEFT_REAR"][2] + gait[2],
            -self.X_COXA, -self.Y_COXA, gait[3])
        solution = self.legIK(-self.endPoints["LEFT_REAR"][0] - request[0] - gait[0],
            -self.endPoints["LEFT_REAR"][1] - request[1] - gait[1],
            self.endPoints["LEFT_REAR"][2] + request[2] + gait[2],
            self.resolutions[servo])
        output = self.neutrals[servo] + self.signs[servo] * solution[COXA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                self.logger.log ("NUKE: LR_COXA FAIL: %s" % output)
            fail = fail + 1
        servo = self.servos["LR Femur"]
        output = self.neutrals[servo] + self.signs[servo] * solution[FEMUR]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                self.logger.log ("NUKE: LR_FEMUR FAIL: %s" % output)
            fail = fail + 1
        servo = self.servos["LR Tibia"]
        output = self.neutrals[servo] + self.signs[servo] * solution[TIBIA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                self.logger.log ("NUKE: LR_TIBIA FAIL: %s" % output)
            fail = fail + 1
        return fail

    def doIK(self):
        fail = self.doRightFrontIK()
        fail += self.doRightRearIK()
        fail += self.doLeftFrontIK()
        fail += self.doLeftRearIK()

        #fail = self.doRightRearIK()
        #if fail > 0:
            #print ("Failed: ", fail)

        #print ("step: ", self.step, " of ", self.stepsInCycle)
        self.step = self.step + 1
        if self.step > (self.stepsInCycle - 1):
            self.step = 0   #gaitStep = (gaitStep+1)%stepsInGait
        return fail
