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
#       Class HeartbeatLED
#

class HeartbeatLED:

    def __init__(self, ledPin):
        self.led = ledPin
        self.led.value(0)
        self.timer = Metro(0)
        self.set(100, 900)

    def set(self, newOnInterval, newOffInterval):
        self.onInterval = newOnInterval
        self.offInterval = newOffInterval
        self.timer.setInterval(self.offInterval)
        self.ledState = 0
        self.led.value(0)

    def update(self):
        if self.timer.check():
            if self.ledState:
                self.ledState = 0
                self.led.value(0)
                if self.onInterval != self.offInterval:
                    self.timer.setInterval(self.offInterval)
            else:
                self.ledState = 1
                self.led.value(1)
                if self.onInterval != self.offInterval:
                    self.timer.setInterval(self.onInterval)

    def shutdown(self):
        self.led.value(0)


#=============================================


startTime = pyb.millis()
poweredUp = False
gatePin = pyb.Pin(pyb.Pin.cpu.C11, pyb.Pin.OUT_PP)
gatePin.value(0)

timer = pyb.Timer(2, freq=1000)
channel = timer.channel(2, pyb.Timer.PWM, pin=pyb.Pin.cpu.B3, pulse_width_percent=0)
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
                gatePin.value(1)
                timer.deinit()
                flasherPin = pyb.Pin(pyb.Pin.cpu.B3, pyb.Pin.OUT_PP)
                flasherPin.value(0)
                heartbeat = HeartbeatLED(flasherPin)
                heartbeat.set(100, 900)
                poweredUp = True
except:
    heartbeat.shutdown()

