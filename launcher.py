#!/usr/bin/env python
import os
import sys
import logging
import time
import threading

import RPi.GPIO as GPIO

import core
import rc
import speed
import wall_follower
import rainbow
import tof_calibrate

# import VL53L0X

from enum import Enum

try:
    from collections import OrderedDict
except ImportError:
    from ordereddict import OrderedDict

# import smbus
from lib_oled96 import ssd1306
from approxeng.input.selectbinder import ControllerResource

logging.basicConfig(stream=sys.stdout, level=logging.INFO)


class Mode(Enum):
    # Enum class for robot mode/challenge.
    MODE_NONE = 0
    MODE_POWER = 1
    MODE_REBOOT = 2
    MODE_RC = 3
    MODE_MAZE = 4
    MODE_SPEED = 5
    MODE_SPEED_LINEAR = 6
    MODE_RAINBOW = 7
    MODE_TOF_CALIBRATE = 8
    MODE_KILL_PROCESS = 9


class launcher:
    def __init__(self):
        # Initialise controller and bind now
        self.controller = None
        self.current_batt = None

        # Initialise GPIO
        self.GPIO = GPIO
        self.GPIO.setwarnings(False)
        self.GPIO.setmode(self.GPIO.BCM)

        # Instantiate CORE / Chassis module and store in the launcher.
        self.core = core.Core(self.GPIO)

        self.challenge = None
        self.challenge_thread = None

        # Shutting down status
        self.shutting_down = False

        self.killed = False

        # Mode/Challenge Dictionary
        self.menu_list = OrderedDict((
            (Mode.MODE_POWER, "Power Off"),
            (Mode.MODE_REBOOT, "Reboot"),
            (Mode.MODE_KILL_PROCESS, "Kill Process"),
            (Mode.MODE_RC, "RC"),
            (Mode.MODE_MAZE, "Maze"),
            (Mode.MODE_SPEED, "Speed"),
            (Mode.MODE_SPEED_LINEAR, "Linear Speed"),
            (Mode.MODE_RAINBOW, "Rainbow"),
            (Mode.MODE_TOF_CALIBRATE, "TOF Calibrate")
        ))
        self.current_mode = Mode.MODE_NONE
        self.menu_mode = Mode.MODE_RC

        # Create oled object
        # Note: Set to None if you need to disable screen
        try:
            self.oled = ssd1306(self.core.i2cbus)
        except:
            print("Failed to get OLED")
            self.oled = None

    def stop_threads(self):
        """ Single point of call to stop any RC or Challenge Threads """
        if self.challenge:
            try:
                self.challenge.stop()
            finally:
                self.challenge = None
                self.challenge_thread = None
            logging.info("Stopping Challenge Thread")
        else:
            logging.info("No Challenge Thread")

        # Reset current mode index
        self.current_mode = Mode.MODE_NONE

        # Safety setting
        self.core.enable_motors(False)

        # Show state on OLED display
        self.show_menu()

    def get_mode_name(self, mode):
        """ Return appropriate mode name """
        mode_name = ""
        if mode != Mode.MODE_NONE:
            mode_name = self.menu_list[mode]
        return mode_name

    def get_next_mode(self, mode):
        """ Find the previous menu item """
        # mode_index = self.menu_list.index(mode)
        # next_index = mode_index + 1
        # if next_index >= len(self.menu_list):
        #     next_index = 0  # Wrapped round to end
        # return list(self.menu_list.items())[next_index]
        index = 0
        count = 0
        for x in self.menu_list.keys():
            if x == mode:
                index = count
            count = count + 1
        list_keys = list(self.menu_list.keys())
        if index + 1 >= len(list_keys):
            index = -1  # Loop back to start of list
        return list_keys[index + 1]

    def get_previous_mode(self, mode):
        """ Find the previous menu item """
        # mode_index = self.menu_list.index(mode)
        # previous_index = mode_index - 1
        # if previous_index < 0:
        #     previous_index = len(self.menu_list) - 1  # Wrapped round to end
        # return list(self.menu_list.items())[previous_index]
        index = 0
        count = 0
        for x in self.menu_list.keys():
            if x == mode:
                index = count
            count = count + 1
        list_keys = list(self.menu_list.keys())
        if index - 1 < 0:
            index = len(list_keys)  # Loop back to end of list
        return list_keys[index - 1]

    def show_message(self, message):
        """ Show state on OLED display """
        if self.oled is not None:
            self.oled.cls()  # Clear Screen
            self.oled.canvas.text((10, 10), message, fill=1)
            # Now show the mesasge on the screen
            self.oled.display()

    def show_mode(self):
        """ Display current menu item. """
        if self.oled is not None:
            # Clear Screen
            self.oled.cls()
            # Get current mode name and display it.
            mode_name = self.get_mode_name(self.current_mode)
            self.oled.canvas.text((10, 10), 'Mode: ' + mode_name, fill=1)
            # Now show the mesasge on the screen
            self.oled.display()

    def menu_item_pressed(self):
        """ Current menu item pressed. Do something """
        if self.menu_mode == Mode.MODE_POWER:
            logging.info("Power Off")
            self.power_off()
        elif self.menu_mode == Mode.MODE_REBOOT:
            logging.info("Reboot")
            self.reboot()
        elif self.menu_mode == Mode.MODE_KILL_PROCESS:
            logging.info("Kill Process")
            self.kill_process()
        elif self.menu_mode == Mode.MODE_RC:
            logging.info("RC Mode")
            self.start_rc_mode()
        elif self.menu_mode == Mode.MODE_SPEED:
            logging.info("Speed Mode")
            self.start_speed_mode(False)
        elif self.menu_mode == Mode.MODE_SPEED_LINEAR:
            logging.info("Linear Speed Mode")
            self.start_speed_mode(True)
        elif self.menu_mode == Mode.MODE_MAZE:
            self.start_maze_mode()
            logging.info("Maze Mode")
        elif self.menu_mode == Mode.MODE_RAINBOW:
            self.start_rainbow_mode()
            logging.info("Rainbow Mode")
        elif self.menu_mode == Mode.MODE_TOF_CALIBRATE:
            self.start_tof_calibrate_mode()
            logging.info("TOF Calibrate Mode")

    def menu_up(self):
        self.menu_mode = self.get_previous_mode(self.menu_mode)
        self.show_menu()

    def menu_down(self):
        self.menu_mode = self.get_next_mode(self.menu_mode)
        self.show_menu()

    def show_menu(self):
        """ Display menu. """
        # Display current menu item to prompt for when no OLED attached
        mode_name = self.get_mode_name(self.menu_mode)
        print(mode_name)
        if self.current_batt is not None:
            print(self.current_batt)

        # Clear Screen
        if self.oled is not None:
            self.oled.cls()
            # Get next and previous list items
            previous_mode = self.get_previous_mode(self.menu_mode)
            next_mode = self.get_next_mode(self.menu_mode)

            # Get mode names and display them.
            current_mode_name = self.get_mode_name(self.current_mode)
            mode_name_up = self.get_mode_name(previous_mode)
            mode_name_down = self.get_mode_name(next_mode)

            header_y = 0
            previous_y = 20
            current_y = 30
            next_y = 40

            # Display Bot name and header information
            self.oled.canvas.text(
                (10, header_y),
                'TITO 5: ' + current_mode_name,
                fill=1)
            # Line underneath header
            self.oled.canvas.line(
                (0, 9, self.oled.width - 1, 9),
                fill=1)

            # Draw rect around current selection.
            # NOTE: Has to be done BEFORE text below
            self.oled.canvas.rectangle(
                (10, current_y, self.oled.width - 1, current_y + 10),
                outline=1,
                fill=0)

            # show current mode as well as one mode either side
            self.oled.canvas.text(
                (15, previous_y),
                'Mode: ' + mode_name_up,
                fill=1)
            self.oled.canvas.text(
                (15, current_y),
                'Mode: ' + mode_name,
                fill=1)
            self.oled.canvas.text(
                (15, next_y),
                'Mode: ' + mode_name_down,
                fill=1)

            # 2x triangles indicating menu direction
            self.oled.canvas.polygon(
                ((1, previous_y + 9),
                 (5, previous_y + 1),
                 (9, previous_y + 9),
                 (1, previous_y + 9)),
                outline=1,
                fill=0)
            self.oled.canvas.polygon(
                ((1, next_y + 1),
                 (5, next_y + 9),
                 (9, next_y + 1),
                 (1, next_y + 1)),
                outline=1,
                fill=0)

            # Now show the mesasge on the screen
            self.oled.display()

    def power_off(self):
        """ Power down the pi """
        self.stop_threads()
        if self.oled is not None:
            self.oled.cls()  # Clear Screen
            self.oled.canvas.text((10, 10), 'Powering off...', fill=1)
            # Now show the mesasge on the screen
            self.oled.display()
        # Call system OS to shut down the Pi
        logging.info("Shutting Down Pi")
        os.system("sudo shutdown -h now")

    def reboot(self):
        """ Power down the pi """
        self.stop_threads()
        if self.oled is not None:
            self.oled.cls()  # Clear Screen
            self.oled.canvas.text((10, 10), 'Rebooting...', fill=1)
            # Now show the mesasge on the screen
            self.oled.display()
        # Call system OS to shut down the Pi
        logging.info("Rebooting Pi")
        os.system("sudo reboot")

    def kill_process(self):
        """ Power down the pi """
        self.stop()
        if self.oled is not None:
            self.oled.cls()  # Clear Screen
            self.oled.canvas.text((10, 10), 'Killing Process...', fill=1)
            # Now show the mesasge on the screen
            self.oled.display()
        # Stop running this python module
        logging.info("Exiting Process")
        quit()

    def start_speed_mode(self, linear):
        # Kill any previous Challenge / RC mode
        self.stop_threads()

        # Set Wiimote LED to RC Mode index
        if linear:
            self.current_mode = Mode.MODE_SPEED_LINEAR
            style = "LINEAR"
        else:
            self.current_mode = Mode.MODE_SPEED
            style = None  # Default to whatever.

        # Set sensible speed
        self.core.speed_factor = 0.4

        # Inform user we are about to start RC mode
        logging.info("Entering into SPEED Mode")
        self.challenge = speed.Speed(self.core, self.oled, style)

        # Create and start a new thread
        # running the remote control script
        logging.info("Starting SPEED Thread")
        self.challenge_thread = threading.Thread(
            target=self.challenge.run)
        self.challenge_thread.start()
        logging.info("SPEED Thread Running")

    def start_rc_mode(self):
        # Kill any previous Challenge / RC mode
        self.stop_threads()

        # Set Wiimote LED to RC Mode index
        self.current_mode = Mode.MODE_RC

        # Set maximum power for RC
        self.core.speed_factor = 1.0

        # Inform user we are about to start RC mode
        logging.info("Entering into RC Mode")
        self.challenge = rc.rc(self.core, self.controller, self.oled)

        # Create and start a new thread
        # running the remote control script
        logging.info("Starting RC Thread")
        self.challenge_thread = threading.Thread(
            target=self.challenge.run)
        self.challenge_thread.start()
        logging.info("RC Thread Running")

    def start_maze_mode(self):
        # Kill any previous Challenge / RC mode, yada yada as above
        self.stop_threads()

        self.current_mode = Mode.MODE_MAZE
        self.core.speed_factor = 0.4

        logging.info("Entering into Maze mode")
        self.challenge = wall_follower.WallFollower(self.core, self.oled)

        logging.info("Starting maze thread")
        self.challenge_thread = threading.Thread(
            target=self.challenge.run)
        self.challenge_thread.start()
        logging.info("Maze thread running")

    def start_rainbow_mode(self):
        # Kill any previous Challenge / RC mode, yada yada as above
        self.stop_threads()

        self.current_mode = Mode.MODE_RAINBOW
        self.core.speed_factor = 0.4

        logging.info("Entering into Rainbow mode")
        self.challenge = rainbow.Rainbow(self.core, self.oled)

        logging.info("Starting Rainbow thread")
        self.challenge_thread = threading.Thread(
            target=self.challenge.run)
        self.challenge_thread.start()
        logging.info("Rainbow thread running")

    def start_tof_calibrate_mode(self):
        # Kill any previous Challenge / RC mode, yada yada as above
        self.stop_threads()

        self.current_mode = Mode.MODE_TOF_CALIBRATE

        logging.info("Entering into TOF Calibrate mode")
        self.challenge = tof_calibrate.Tof_Calibrate(self.core, self.oled)

        logging.info("Starting TOF Calibrate thread")
        self.challenge_thread = threading.Thread(
            target=self.challenge.run)
        self.challenge_thread.start()
        logging.info("TOF Calibrate thread running")

    def stop(self):
        """ Stop the entire program safely. """
        launcher.controller = None
        launcher.stop_threads()  # This will set neutral for us.
        print("Clearing up")
        launcher.core.cleanup()
        launcher.GPIO.cleanup()

    def run(self):
        """ Main Running loop controling bot mode and menu state """
        # Show state on OLED display
        self.show_message('Booting...')
        self.show_message('Initialising Bluetooth...')

        start = time.time()

        # Never stop looking for controller.
        while not self.killed:

            try:
                if self.oled is not None:
                    # Show state on OLED display
                    self.oled.cls()  # Clear screen
                    self.oled.canvas.text(
                        (10, 10),
                        'Waiting for controller...',
                        fill=1)
                    # self.oled.canvas.text(
                    #     (10, 30),
                    #     '***Press 1+2 now ***',
                    #     fill=1)
                    self.oled.display()

                # Initialise controller and bind now

                with ControllerResource(dead_zone=0.1, hot_zone=0.2) as self.controller:
                    # Get battery level
                    self.current_batt = 0
                    battery_level = self.controller.battery_level
                    if battery_level:
                        self.current_batt = battery_level

                    # Show state on OLED display
                    self.show_menu()

                    # Constantly check controller for button presses
                    while self.controller.connected:

                        # Get joystick values from the left analogue stick
                        # x_axis, y_axis = self.controller['lx', 'ly']

                        # Get a ButtonPresses object containing everything
                        # that was pressed since the last time around this
                        # loop.
                        self.controller.check_presses()

                        # Print out any buttons that were
                        # pressed, if we had any
                        # print(self.controller.presses)

                        # Test whether a button is pressed
                        if self.controller.has_presses:

                            if ('dright' in self.controller.presses and
                               self.challenge is None):
                                # Only works when NOT in a challenge
                                print("Menu Item Pressed")
                                self.menu_item_pressed()
                                self.show_menu()

                            if 'home' in self.controller.presses:
                                # Kill any previous Challenge / RC mode
                                # NOTE: will ALWAYS work
                                self.core.enable_motors(False)
                                self.stop_threads()

                            if ('dup' in self.controller.presses and
                               self.challenge is None):
                                # Only works when NOT in a challenge
                                self.menu_up()

                            if ('ddown' in self.controller.presses and
                               self.challenge is None):
                                # Only works when NOT in a challenge
                                self.menu_down()

                            if 'start' in self.controller.presses:
                                # Toggle motor enable/disable
                                # allow motors to move freely.
                                # NOTE: will ALWAYS work
                                self.core.enable_motors(
                                    not self.core.motors_enabled()
                                )
                                if self.core.motors_enabled():
                                    print("Enabled")
                                else:
                                    print("Neutral")

                            # Increase or Decrease motor speed factor
                            if 'r1' in self.controller.presses:
                                self.core.increase_speed_factor()
                            if 'l1' in self.controller.presses:
                                self.core.decrease_speed_factor()

                            # Increase or Decrease motor speed factor
                            # if 'r2' in self.controller.presses:
                            #     self.core.increase_speed_factor()
                            # if 'l2' in self.controller.presses:
                            #     self.core.decrease_speed_factor()
                            # if 'ps4_pad' in self.controller.presses:
                            #     self.core.decrease_speed_factor()

                            # toggle on/off the gun motor
                            if 'square' in self.controller.presses:
                                self.core.enable_gun(not self.core.gun_enabled)
                            # Fire the gun
                            if 'circle' in self.controller.presses:
                                self.core.fire_gun(5)
                                time.sleep(1.0)
                                self.core.fire_gun(175)

                            # Move turret up/down
                            if 'triangle' in self.controller.presses:
                                self.core.move_turret_increment(10)
                            if 'cross' in self.controller.presses:
                                self.core.move_turret_increment(-10)

                            # Show current challenge state if we press buttons
                            if self.challenge:
                                self.challenge.show_state()

                        time.sleep(0.05)
                        done = time.time()
                        elapsed = done - start
                        if elapsed > 60:
                            battery_level = self.controller.battery_level
                            if battery_level:
                                self.current_batt = battery_level

                    # If bot out of range, set neutral as safety
                    print('Controller disconnected!')
                    self.core.set_neutral()

            except IOError:
                logging.error(
                    "Could not connect to "
                    "controller. please try again"
                )
                # kill any active challenges as RC no longer connected
                self.stop_threads()
                # We get an IOError when using the ControllerResource
                # if we don't have a controller yet, so in this case
                # we just wait a second and try again after printing a message.
                time.sleep(1)


if __name__ == "__main__":
    launcher = launcher()
    try:
        launcher.run()
    except (Exception, KeyboardInterrupt) as e:
        # Stop any active threads before leaving
        print("Stopping: " + str(e))
        launcher.controller = None
        launcher.stop_threads()  # This will set neutral for us.
        print("Clearing up")
        launcher.core.cleanup()
        launcher.GPIO.cleanup()


        print(str(e))
        # Show state on OLED display
        launcher.show_message('Exited Python Code')
