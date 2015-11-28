# boot.py -- run on boot-up
# can run arbitrary Python, but best to keep it minimal

import pyb, sys
sys.path.append('/sd/bioloid3')
pyb.main('roz.py') # main script to run after this one
pyb.usb_mode('CDC') # act as a serial device
#pyb.usb_mode('CDC+MSC') # act as a serial and a storage device
#pyb.usb_mode('CDC+HID') # act as a serial device and a mouse
