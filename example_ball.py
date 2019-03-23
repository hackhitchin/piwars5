##!/usr/bin/env python
# coding: Latin

# Load library functions we want
import os
import time
import sys
import core
import RPi.GPIO as GPIO
import picamera
from stream_processor import StreamProcessor
from stream_processor import State
from image_capture import ImageCapture


def main(core_module):
    # Startup sequence
    # Load our previously learned arena colour order
    camera = picamera.PiCamera()
    processor = StreamProcessor(core_module, camera)
    print('Wait ...')
    time.sleep(2)

    filename = "arenacolours.txt"
    if os.path.isfile(filename):
        with open(filename) as f:
            content = f.readlines()
        if len(content) > 0:
            processor.arenacolours = [x.strip() for x in content]
            processor.state = State.ORIENTING
        f.close()

    if core_module is None:
        # Initialise GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # Instantiate CORE / Chassis module and store in the launcher.
        core_module = core.Core(GPIO)
        # Limit motor speeds in AutoMode
        core_module.speed_factor = 0.6  # 0.6 on old motors
        core_module.enable_motors(True)

    # wait for the user to enable motors
    while not core_module.motors_enabled:
        time.sleep(0.25)

    # Setup the camera
    frameRate = 30  # Camera image capture frame rate
    camera.resolution = (processor.imageWidth, processor.imageHeight)
    camera.framerate = frameRate
    camera.awb_mode = 'off'

    # Load the exposure calibration
    with open("rbgains.txt") as f:
        content = f.readlines()
    content = [x.strip() for x in content]
    redgain = float(content[0][2:])
    bluegain = float(content[1][2:])
    camera.awb_gains = (redgain, bluegain)
    f.close()

    captureThread = ImageCapture(camera, processor)

    try:
        print('Press CTRL+C to quit')
        # TB.MotorsOff()
        # TB.SetLedShowBattery(True)
        # Loop indefinitely until we are no longer running
        while processor.state != State.FINISHED:
            # Wait for the interval period
            #
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("User shutdown\n")
    except:
        e = sys.exc_info()[0]
        print
        print(e)

    core_module.enable_motors(False)
    processor.state = State.FINISHED
    captureThread.join()
    processor.terminated = True
    processor.join()
    del camera
    print("Program terminated")


if __name__ == '__main__':
    main(None)
