#!/usr/bin/python

# MIT License
# 
# Copyright (c) 2017 John Bryan Moore
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import time
import VL53L0X
import RPi.GPIO as GPIO


GPIO.setwarnings(False)
GPIO = GPIO
GPIO.setmode(GPIO.BCM)

# Setup pwr pin as HIGH (off)
gpio_pin = 10
GPIO.setup(gpio_pin, GPIO.OUT, initial=1)
time.sleep(0.5)
GPIO.output(gpio_pin, 0)       # Set the pin low, sensor on
time.sleep(0.2)                # Wait for chip to wake6

# Create a VL53L0X object
tof = VL53L0X.VL53L0X(address=0x2a)

# Start ranging
tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

timing = tof.get_timing()
if (timing < 20000):
    timing = 20000
print ("Timing %d ms" % (timing/1000))

for count in range(1,101):
    distance = tof.get_distance()
    if (distance > 0):
        print ("%d mm, %d cm, %d" % (distance, (distance/10), count))

    time.sleep(timing/1000000.00)

tof.stop_ranging()

