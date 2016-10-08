
# import pyb
import math


"""
    Class Servo

        Servos represent a specific physical actuator, specifically Robotis servos
        like AX-12 or MX-64.

"""

class Servo:

    def __init__(self, id):
        self.id = id
        self.desiredSpeed = 0
        self.desiredPosition = 511
        self.currentPosition = None
        self.temperature = None
        self.bus = None
        self.setResolution(1024)

    def setResolution(self, resolution):
        """ setResolution() must be called when setting or changing servo resolution """
        # some Robotis servos have 10 bit resolution, and some have 12 bit
        resolutionFactor = resolution / 1024
        # Robotis servos typically have 0-300 degrees full swing
        servoFullDegrees = 300
        # conversionRatio converts radians to servo units
        self.conversionRatio = 180 / math.pi * (1024 * resolutionFactor - 1) / servoFullDegrees

    def angleToPosition(self, angle):
        """convert radians to servo units"""
        return int(angle * self.conversionRatio)

    def positionToAngle(self, position):
        """convert servo units to radians"""
        return position / self.conversionRatio


"""
    Class Joint

        Joints are the pivot points of legs. They represent the robot's view
        of the joint, and handle things like gait and translating between the
        robot's coordinate system and a servo's coordinate system.

"""

class Joint:

    def __init__(self, name, servo):
        self.name = name
        self.servo = servo
        self.length = 0
        self.angle = 0
        self.maxAngle = 0
        self.minAngle = 0
        self.neutralAngle = 0
        self.signFactor = 1

    def angleToPosition(self, bodyAngle):
        """convert a neutral-relative angle (in radians) to an absolute servo position"""
        clampedBodyAngle = min(max(bodyAngle, self.minAngle), self.maxAngle)
        clampedServoAngle = self.signFactor * clampedBodyAngle + self.neutralAngle
        return self.servo.angleToPosition(clampedServoAngle)

    def positionToAngle(self, position):
        """convert an absolute servo position to a neutral-relative angle (in radians)"""
        servoAngle = self.servo.positionToAngle(position)
        return (servoAngle - self.neutralAngle) * self.signFactor


"""
    Class Appendage

        Appendages are abstract, jointed body parts, which include legs, heads, and tails.

"""

class Appendage:

    def __init__(self, name):
        self.name = name
        self.robot = None
        self.joints = None
        self.endPosition = None
        self.xSign = None
        self.ySign = None


# ================================================
#
#       Class Leg
#

class Leg(Appendage):

    def doIk(self):
        pass


# ================================================
#
#       Class Head
#

class Head(Appendage):

    def doIk(self):
        pass


# ================================================
#
#       Class Tail
#

class Tail(Appendage):

    def doIk(self):
        pass


# ================================================
#
#       Class Orientation
#

class Orientation:

    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.sideOffset = 0
        self.forwardOffset = 0
        self.heightOffset = 0


# ================================================
#
#       Class Motion
#

class Motion:

    def __init__(self):
        self.forwardSpeed = 0
        self.sideSpeed = 0
        self.rotationSpeed = 0


# ================================================
#
#       Class GaitFoot
#

class GaitFoot:

    def __init__(self, footName):
        self.name = footName
        self.firstStep = 0
        self.step = 0


# ================================================
#
#       Class Gait
#

class Gait:

    def __init__(self, name):
        self.name = name
        self.robot = None
        self.gaitFeet = {}
        for foot in ["RF", "LF", "RR", "LR"]:
            self.gaitFeet[foot] = GaitFoot(foot)
        self.steps = GaitStepGenerator().generateStepPositions(150, 20, 20)


class GaitStepGenerator:

    def xfrange(self, start, stop, step):
        while start < stop:
            yield start
            start += step


    def generateStepPositions(self, totalLength, liftHeight, stepCount):
        a = totalLength / 2
        b = liftHeight

        circumference = 0.0
        for t in self.xfrange(0.0, math.pi / 2.0, 0.01):
            circumference += math.sqrt((a * math.sin(t)) ** 2 + (b * math.cos(t)) ** 2 )

        points = []
        nextPoint = 0
        halfStepCount = stepCount / 2
        run = 0.0

        for t in self.xfrange(0.0, math.pi / 2.0, 0.01):
            if halfStepCount * run / circumference >= nextPoint:
                x = -a * math.cos(t)
                y = b * math.sin(t)
                points.append((round(x, 2), round(y, 2)))
                nextPoint += 1
            run += math.sqrt((a * math.sin(t)) ** 2 + (b * math.cos(t)) ** 2)

        reverse = list(reversed(points))
        points.append((0.0, liftHeight))
        for each in reverse:
            points.append((-each[0], each[1]))
        stepLength = totalLength / stepCount
        x = points[-1][0]
        for index in range(0, stepCount - 1):
            x -= stepLength
            points.append((x, 0))
        return points


# ================================================
#
#       Class Robot
#

class Robot:

    def __init__(self):
        self.joints = self.createJoints()
        self.legs = self.createLegs()
        self.head = self.createHead()
        self.tail = self.createTail()
        self.orientation = self.createOrientation()
        self.motion = self.createMotion()
        self.setupGait()
#        self.controller = self.createController()
#        self.ikEngine = self.createIkEngine()
#        self.navigator = self.createNavigator()
#        self.watchdog = self.createWatchdog()
#        self.logger = self.createLogger()
#        self.heartbeat = self.createHeartbeat()
        self.gaitStep = 0

    def createJoints(self):
        joints = []
        for name, attributes in self.jointNamesAndAttributes().items():
            servoId = attributes[0]
            servo = Servo(servoId)
            servo.setResolution(attributes[3])
            joint = Joint(name, servo)
            # note that the neutral angle is specified in servo space (although in radians)
            joint.neutralAngle = servo.positionToAngle(attributes[4])
            joint.signFactor = attributes[5]
            # whereas min and max angle are specified relative to the neutral
            joint.minAngle = joint.positionToAngle(attributes[1])
            joint.maxAngle = joint.positionToAngle(attributes[2])
            joints.append(joint)
        return sorted(joints, key=lambda j: j.name)

    def jointNamed(self, name):
        for joint in self.joints:
            if joint.name == name:
                return joint
        raise Exception("No joint with name %s" % name)

    def legNamed(self, name):
        for leg in self.legs:
            if leg.name == name:
                return leg
        raise Exception("No leg with name %s" % name)

    def createLegs(self):
        legs = []
        for name in self.legNames():
            leg = Leg(name)
            leg.robot = self
            leg.joints = [self.jointNamed(i) for i in [name + " Coxa", name + " Femur", name + " Tibia"]]
            legs.append(leg)
        return legs

    def createOrientation(self):
        return Orientation()

    def createMotion(self):
        return Motion()

    def forwardSpeed(self):
        return self.motion.forwardSpeed

    def sideSpeed(self):
        return self.motion.sideSpeed

    def rotationSpeed(self):
        return self.motion.rotationSpeed

    def createHead(self):
        raise Exception("Implemented by subclass")

    def createTail(self):
        raise Exception("Implemented by subclass")

    def legNames(self):
        raise Exception("Implemented by subclass")

    def jointNamesAndAttributes(self):
        raise Exception("Implemented by subclass")

    def setupGait(self):
        raise Exception("Implemented by subclass")


# ================================================
#
#       Class Roz
#

class Roz(Robot):

    def jointNamesAndAttributes(self):
        return {
            "RF Coxa": [1, 247, 649, 1024, 348, 1],
            "LF Coxa": [2, 378, 780, 1024, 675, -1],
            "RF Femur": [3, 164, 860, 1024, 511, 1],
            "LF Femur": [4, 165, 858, 1024, 511, -1],
            "RF Tibia": [5, 228, 867, 1024, 511, 1],
            "LF Tibia": [6, 158, 799, 1024, 511, -1],
            "RR Coxa": [7, 378, 780, 1024, 675, -1],
            "LR Coxa": [8, 247, 649, 1024, 348, 1],
            "RR Femur": [9, 158, 866, 1024, 511, -1],
            "LR Femur": [10, 165, 866, 1024, 511, 1],
            "RR Tibia": [11, 164, 785, 1024, 511, -1],
            "LR Tibia": [12, 228, 861, 1024, 511, 1],
            "Head Yaw": [13, 0, 1023, 1024, 511, 1],
            "Tail Base": [14, 0, 1023, 1024, 511, 1],
            "Tail End": [15, 0, 1023, 1024, 511, 1]}

    def legNames(self):
        return ["RF", "LF", "RR", "LR"]

    def createHead(self):
        head = Head("Head")
        head.robot = self
        head.joints = [self.jointNamed("Head Yaw")]
        return head

    def createTail(self):
        tail = Tail("Tail")
        tail.robot = self
        tail.joints = [self.jointNamed(i) for i in ["Tail Base", "Tail End"]]
        return tail

    def setupGait(self):
        self.setupAmbleGait()

    def setupAmbleGait(self):
        for legName, legOrder, xSign, ySign in [("RF", 0, 1, 1), ("LR", 0, 1, -1), ("LF", 2, -1, 1), ("RR", 2, -1, -1)]:
            gait = Gait("%s Amble" % legName)
            leg = self.legNamed(legName)
            leg.xSign = xSign
            leg.ySign = ySign
            leg.gait = gait
            gait.appendage = leg
            gait.robot = self
            gait.firstStep = legOrder
            gait.stepCount = 4
            gait.pushStepCount = 2
            gait.liftHeight = 20
            gait.endRestPosition = [50, 130, 90]
