#!/usr/bin/env python
# coding: Latin
import example_ball


class Rainbow:
    def __init__(self, core_module, oled):
        """Class Constructor"""
        self.killed = False
        self.core_module = core_module
        self.ticks = 0
        self.oled = oled

    def show_state(self):
        """ Show motor/aux config on OLED display """
        if self.oled is not None:
            # Format the speed to 2dp
            if self.core_module.motors_enabled:
                message = "Rainbow: %0.2f" % (self.core_module.speed_factor)
            else:
                message = "Rainbow: NEUTRAL"

            self.oled.cls()  # Clear Screen
            self.oled.canvas.text((10, 10), message, fill=1)
            # Now show the mesasge on the screen
            self.oled.display()

    def stop(self):
        """Simple method to stop the RC loop"""
        self.killed = True

    def run(self):
        """ Main Challenge method. Has to exist and is the
            start point for the threaded challenge. """
        example_ball.main(self.core_module)
