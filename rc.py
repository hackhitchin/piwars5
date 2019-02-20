import core
import time


class rc:
    def __init__(self, core_module, controller, oled):
        """Class Constructor"""
        self.killed = False
        self.core_module = core_module
        self.ticks = 0
        # Bind to any available joystick.
        self.controller = controller
        self.oled = oled

    def show_motor_speeds(self, left_motor, right_motor):
        """ Show motor/aux config on OLED display """
        if self.oled is not None:
            # Format the speed to 2dp
            message = "[L: %0.2f] [R: %0.2f]" % (left_motor, right_motor)

            self.oled.cls()  # Clear Screen
            self.oled.canvas.text((10, 10), message, fill=1)
            # Now show the mesasge on the screen
            self.oled.display()

    def show_state(self):
        """ Show motor/aux config on OLED display """
        if self.oled is not None:
            # Format the speed to 2dp
            if self.core_module.motors_enabled:
                message = "RC: %0.2f" % (self.core_module.speed_factor)
            else:
                message = "RC: NEUT (%0.2f)" % (self.core_module.speed_factor)

            self.oled.cls()  # Clear Screen
            self.oled.canvas.text((10, 10), message, fill=1)
            # Now show the mesasge on the screen
            self.oled.display()

    def stop(self):
        """Simple method to stop the RC loop"""
        self.killed = True

    def mixer(self, yaw, throttle, max_power=100):
        left = throttle + yaw
        right = throttle - yaw
        scale = float(max_power) / max(1, abs(left), abs(right))
        return int(left * scale), int(right * scale)

    def run(self):
        """ Main Challenge method. Has to exist and is the
            start point for the threaded challenge. """
        # nTicksSinceLastMenuUpdate = -1
        # nTicksBetweenMenuUpdates = 10  # 10*0.05 seconds = every half second

        try:
            # Loop indefinitely, or until this thread is flagged as stopped.
            while self.controller.connected and not self.killed:
                # While in RC mode, get joystick
                # states and pass speeds to motors.

                # Get joystick values from the left analogue stick
                # x_axis, y_axis = self.controller['lx', 'ly']
                x_axis, y_axis = self.controller['rx', 'ly']

                l_throttle, r_throttle = self.mixer(
                    x_axis, y_axis, max_power=100)

                if self.core_module:
                    self.core_module.throttle(l_throttle, r_throttle)

                    if self.core_module.motors_enabled:
                        print("Motors %0.2f, %0.2f" % (l_throttle, r_throttle))
                    else:
                        print("RC: NEUTRAL")

                # Show motor speeds on LCD
                # if (nTicksSinceLastMenuUpdate == -1 or
                #    nTicksSinceLastMenuUpdate >= nTicksBetweenMenuUpdates):
                #     # self.show_motor_speeds(l_throttle, r_throttle)
                #     self.show_state()
                #     nTicksSinceLastMenuUpdate = 0
                # else:
                #     nTicksSinceLastMenuUpdate = nTicksSinceLastMenuUpdate + 1

                # Sleep between loops to allow other stuff to
                # happen and not over burden Pi and Arduino.
                time.sleep(0.05)

        except IOError:
            logging.error(
                "Could not connect to "
                "controller. please try again"
            )
