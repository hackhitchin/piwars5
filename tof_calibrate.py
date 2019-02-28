# Import triangula module to interact with SixAxis
import core
from core import I2C_Lidar
import time
from lib_oled96 import ssd1306


class Tof_Calibrate:
    def __init__(self, core_module, oled):
        """Class Constructor"""
        self.killed = False
        self.core = core_module
        self.oled = oled
        self.d_left = 0.0
        self.d_front = 0.0
        self.d_right = 0.0

    def stop(self):
        """Simple method to stop the RC loop"""
        self.killed = True

    def show_state(self):
        """ Show motor/aux config on OLED display """
        if self.oled is not None:
            # Format the speed to 2dp
            message = ("L:{} F:{} R:{}").format(
                self.d_left, self.d_front, self.d_right)

            self.oled.cls()  # Clear Screen
            self.oled.canvas.text((10, 10), message, fill=1)
            # Now show the mesasge on the screen
            self.oled.display()
            print(message)

    def run(self):
        print("Start calibrate")
        """Read a sensor and store"""
        count = 0
        while not self.killed:
            # New API
            self.d_left = 0.0
            self.d_front = 0.0
            self.d_right = 0.0
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_RIGHT)
                ]
                self.d_left = lidar_dev['device'].get_distance()
            except KeyError:
                self.d_left = -1
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_FRONT)
                ]
                self.d_front = lidar_dev['device'].get_distance()
            except KeyError:
                self.d_front = -1
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_LEFT)
                ]
                self.d_right = lidar_dev['device'].get_distance()
            except KeyError:
                self.d_right = -1

            self.show_state()

            time.sleep(0.1)
            count += 1

        self.core.set_neutral(False)


if __name__ == "__main__":
    import RPi.GPIO as GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    core = core.Core(GPIO)
    oled = None
    try:
        oled = ssd1306(core.i2cbus)
    except:
        print("Failed to get OLED")
        oled = None

    follower = Tof_Calibrate(core, oled)
    try:
        follower.run()
    except (KeyboardInterrupt) as e:
        # except (Exception, KeyboardInterrupt) as e:
        # Stop any active threads before leaving
        follower.stop()
        core.cleanup()
        print("Quitting")
