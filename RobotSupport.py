
import pyb


#================================================
#
#       Class Servo
#

class Servo:

    def __init__(self, id, name):
        self.id = id
        self.name = name
        self.robot = None
        self.desiredSpeed = 0
        self.desiredPosition = 511
        self.currentPosition = None
        self.temperature = None
        self.minimumPosition = None
        self.maximumPosition = None
        self.neutralPosition = None
        self.resolution = 1024
        self.signFactor = 1

