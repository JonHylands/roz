import pyb
from BioloidController import BioloidController

controller = BioloidController()


while True:
    positions = []
    for id in range(13):
        position = controller.readTwoByteRegister(id + 1, 36)
        positions.append((id + 1, position))
    print(positions)
    pyb.delay(500)
