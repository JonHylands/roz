
import pyb

#================================================
#
#       Class Logger
#

class Logger:

    def __init__(self, logFilename):
        self.file = open('/sd/%s' % logFilename, 'wt')
        self.startCount = pyb.millis()
        self.file.write("Logger Start\n")

    def timestamp(self):
        return pyb.millis() - self.startCount

    def log(self, logString):
        line = "%08d: %s\n" % (self.timestamp(), logString)
        self.file.write(line)
        self.file.flush()

    def close(self):
        self.file.close()

#================================================
#
#       Class RangeFinder
#

class RangeFinder:

    def __init__(self, rangePinNumber, rangeMax = 20):
        self.adc = pyb.ADC(rangePinNumber)
        self.rangeMax = rangeMax

    def getDistance(self):
        value = self.adc.read()
        voltage = value * 0.0032258 # 0-1023 -> 0-3.3 volts
        exactDistance = 100 * ((1.25 / voltage) - 0.15)
        distance = exactDistance
        if distance > self.rangeMax:
            return self.rangeMax
        else:
            return distance

#================================================
#
#       Class Metro
#

class Metro:

    def __init__(self, interval_millis):
        self.interval = interval_millis
        self.previous = pyb.millis()

    def setInterval(self, interval_millis):
        self.interval = interval_millis

    def check(self):
        now = pyb.millis()
        if self.interval == 0:
            self.previous = now
            return True
        if (now - self.previous) >= self.interval:
            self.previous = now
            return True
        return False

    def reset(self):
        self.previous = pyb.millis()

#================================================
#
#       Class HeartbeatLED
#

class HeartbeatLED:

    def __init__(self, ledNumber):
        self.led = pyb.LED(ledNumber)
        self.led.off()
        self.timer = Metro(0)
        self.set(100, 900)

    def set(self, newOnInterval, newOffInterval):
        self.onInterval = newOnInterval
        self.offInterval = newOffInterval
        self.timer.setInterval(self.offInterval)
        self.ledState = 0
        self.led.off()

    def update(self):
        if self.timer.check():
            if self.ledState:
                self.ledState = 0
                self.led.off()
                if self.onInterval != self.offInterval:
                    self.timer.setInterval(self.offInterval)
            else:
                self.ledState = 1
                self.led.on()
                if self.onInterval != self.offInterval:
                    self.timer.setInterval(self.onInterval)

    def shutdown(self):
        self.led.off()


#================================================
#
#       Class OneShotButton
#

class OneShotButton:

    def __init__(self, pinName, pullupNeeded = True, targetValue = 0):
        if pullupNeeded:
            self.pin = pyb.Pin(pinName, pyb.Pin.IN, pull=pyb.Pin.PULL_UP)
        else:
            self.pin = pyb.Pin(pinName, pyb.Pin.IN, pull=pyb.Pin.PULL_NONE)
        self.targetValue = targetValue
        self.pressedLastTime = False

    def isPressed(self):
        if self.pin.value() == self.targetValue:
            if self.pressedLastTime:
                return False
            self.pressedLastTime = True
            return True
        else:
            self.pressedLastTime = False
            return False
