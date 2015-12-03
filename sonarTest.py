import pyb
from BioloidController import BioloidController
from Support import RangeFinder


def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

blue = pyb.LED(4)

ranger = RangeFinder('C3')
controller = BioloidController()
controller.rampServoTo(13, 306)
values = []
pyb.delay(250)
blue.on()
totalStart = pyb.millis()
for position in range(30600, 71700, 1705):
    start = pyb.millis()
    end = start + 40
    i_pos = int(position / 100)
    controller.setPosition(13, i_pos)
    values.append(ranger.getDistance())
    pyb.delay(max(0, end - pyb.millis()))

blue.off()

print ("Total time: ", (pyb.millis() - totalStart))
# controller.rampServoTo(13, 511)
print (values)

groups = []
lastIndex = None
for (index, value) in enumerate(values):
    if value == 50:
        if lastIndex == index - 1:
            startGroup = lastIndex
        if lastIndex is None:
            lastIndex = index
    else:
        if lastIndex is not None:
            groups.append((startGroup, index - 1))
        lastIndex = None
if values[-1] == 50:
    groups.append((startGroup, len(values) - 1))
print ("\nGroups: ", groups)

if len(groups) == 0:
    print ("No groups identified")
    controller.rampServoTo(13, 511)
else:
    maxLength = 0
    maxGroup = None
    for pairs in groups:
        groupLength = pairs[1] - pairs[0] + 1
        if groupLength > maxLength:
            groupLength = maxLength
            maxGroup = pairs

    print ("\nMax Group: ", maxGroup)

    center = (maxGroup[0] + maxGroup[1]) // 2
    centerPosition = arduino_map(center, 0, len(values), 306, 717)

    centerAngle = arduino_map(centerPosition, 0, 1023, 0, 300)

    print ("Center of Max Group: ", centerPosition)
    controller.rampServoTo(13, centerPosition)

    print("Center Angle: ", -150 + centerAngle)
