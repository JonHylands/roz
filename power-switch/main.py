# main.py -- put your code here!

import pyb


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
#       Class RgbLED
#

class RgbLED:

    def __init__(self, redPin, greenPin, bluePin, timerNumber, redChannelNumber, greenChannelNumber, blueChannelNumber):
        self.timer = pyb.Timer(timerNumber, freq=1000)
        self.redChannel = timer.channel(redChannelNumber, pyb.Timer.PWM, pin=redPin, pulse_width_percent=0)
        self.greenChannel = timer.channel(greenChannelNumber, pyb.Timer.PWM, pin=greenPin, pulse_width_percent=0)
        self.blueChannel = timer.channel(blueChannelNumber, pyb.Timer.PWM, pin=bluePin, pulse_width_percent=0)

    def setRed(self, intensity):
        self.redChannel.pulse_width_percent(intensity)

    def setGreen(self, intensity):
        self.greenChannel.pulse_width_percent(intensity)

    def setBlue(self, intensity):
        self.blueChannel.pulse_width_percent(intensity)

    def set(self, redIntensity, greenIntensity, blueIntensity):
        self.setRed(redIntensity)
        self.setGreen(greenIntensity)
        self.setBlue(blueIntensity)

    def white(self):
        self.set(100, 100, 100)

    def off(self):
        self.set(0, 0, 0)

    def red(self):
        self.set(100, 0, 0)

    def green(self):
        self.set(0, 100, 0)

    def blue(self):
        self.set(0, 0, 100)

    def yellow(self):
        self.set(100, 100, 0)

    def orange(self):
        self.set(100, 50, 0)

    def cyan(self):
        self.set(0, 100, 100)

    def purple(self):
        self.set(50, 0, 100)

    def pink(self):
        self.set(100, 0, 100)


#=============================================

greenPin = pyb.Pin(pyb.Pin.cpu.B1, pyb.Pin.OUT_PP)
bluePin = pyb.Pin(pyb.Pin.cpu.B5, pyb.Pin.OUT_PP)

greenPin.value(0)
bluePin.value(0)

print("BIOLOID_POWER")

startTime = pyb.millis()
poweredUp = False
gatePin = pyb.Pin(pyb.Pin.cpu.A14, pyb.Pin.OUT_PP)
gatePin.value(0)

timer = pyb.Timer(1, freq=1000)
channel = timer.channel(2, pyb.Timer.PWM, pin=pyb.Pin.cpu.B0, pulse_width_percent=0)
pulseWidth = 0
ledTime = pyb.millis()

try:
    while True:
        if poweredUp:
            heartbeat.update()
        else:
            if pyb.elapsed_millis(ledTime) >= 5:
                ledTime = pyb.millis()
                pulseWidth += 1
                channel.pulse_width_percent(pulseWidth)
            if pyb.elapsed_millis(startTime) > 500:
                print("Powered up")
                gatePin.value(1)
                timer.deinit()
                redPin = pyb.Pin(pyb.Pin.cpu.B0, pyb.Pin.OUT_PP)
                redPin.value(0)
                heartbeat = HeartbeatLED(redPin)
                heartbeat.set(100, 900)
                poweredUp = True
except:
    heartbeat.shutdown()

