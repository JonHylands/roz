import pyb
from BioloidController import BioloidController

controller = BioloidController()

targetPosition = 511
currentPosition = controller.readTwoByteRegister(13, 36) # present position
if targetPosition > currentPosition:
    stepDelta = 1
    stepAccel = 2
    comparison = lambda: targetPosition > (currentPosition + stepDelta)
else:
    stepDelta = -1
    stepAccel = -2
    comparison = lambda: currentPosition > (targetPosition - stepDelta)

while comparison():
    movePosition = currentPosition + stepDelta
    stepDelta += stepAccel
    controller.setPosition(13, movePosition)
    currentPosition = controller.readTwoByteRegister(13, 36) # present position
    pyb.delay(25)

controller.setPosition(13, targetPosition)

print("Done - stepDelta = ", stepDelta, ' stepAccel = ', stepAccel)
