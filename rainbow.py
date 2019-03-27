import os
import time
import sys
import core
import RPi.GPIO as GPIO
import picamera
from stream_processor import StreamProcessor
from stream_processor import State
from image_capture import ImageCapture


class Rainbow:
    def __init__(self, core_module, oled):
        """Class Constructor"""
        self.killed = False
        self.core_module = core_module
        self.ticks = 0
        self.oled = oled
        self.camera = None
        self.captureThread = None

    def show_state(self):
        """ Show motor/aux config on OLED display """
        if self.oled is not None:
            # Format the speed to 2dp
            if self.core_module and self.core_module.motors_enabled():
                message = "Rainbow: %0.2f" % (
                    self.core_module.get_speed_factor())
            else:
                message = "Rainbow: NEUTRAL"

            self.oled.cls()  # Clear Screen
            self.oled.canvas.text((10, 10), message, fill=1)
            # Now show the mesasge on the screen
            self.oled.display()

    def stop(self):
        """Simple method to stop the RC loop"""
        self.killed = True
        if self.processor:
            self.processor.state = State.FINISHED
        if self.captureThread:
            self.captureThread.stop()

    def run(self):
        """ Main Challenge method. Has to exist and is the
            start point for the threaded challenge. """
        # Startup sequence
        if self.core_module is None:
            # Initialise GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)

            # Instantiate CORE / Chassis module and store in the launcher.
            self.core_module = core.Core(GPIO)
            # Limit motor speeds in AutoMode
            self.core_module.set_speed_factor(0.6)  # 0.6 on old motors
            self.core_module.enable_motors(True)

        # wait for the user to enable motors
        while not self.core_module.motors_enabled():
            time.sleep(0.25)

        # Load our previously learned arena colour order
        self.camera = picamera.PiCamera()
        self.processor = StreamProcessor(self.core_module, self.camera)
        print('Wait ...')
        time.sleep(2)

        filename = "arenacolours.txt"
        if os.path.isfile(filename):
            with open(filename) as f:
                content = f.readlines()
            if len(content) > 0:
                self.processor.arenacolours = [x.strip() for x in content]
                self.processor.state = State.ORIENTING
            f.close()

        # Setup the camera
        frameRate = 30  # Camera image capture frame rate
        self.camera.resolution = (
            self.processor.imageWidth,
            self.processor.imageHeight)
        self.camera.framerate = frameRate
        self.camera.awb_mode = 'off'

        # Load the exposure calibration
        redgain = 1.5  # Default Gain values
        bluegain = 1.5
        filename = "rbgains.txt"
        if os.path.isfile(filename):
            with open(filename) as f:
                content = f.readlines()
            content = [x.strip() for x in content]
            redgain = float(content[0][2:])
            bluegain = float(content[1][2:])
            f.close()
        self.camera.awb_gains = (redgain, bluegain)

        self.captureThread = ImageCapture(self.camera, self.processor)

        try:
            # Loop indefinitely until we are no longer running
            while self.processor.state != State.FINISHED:
                # Wait for the interval period
                #
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("User shutdown\n")
        except Exception as e:
            print(e)

        self.core_module.enable_motors(False)
        self.processor.state = State.FINISHED
        self.captureThread.join()
        self.processor.terminated = True
        self.processor.join()
        del self.camera
        self.camera = None
        print("Program terminated")


if __name__ == "__main__":
    challenge = Rainbow(None, None)
    challenge.run()
