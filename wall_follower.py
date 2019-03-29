# Import triangula module to interact with SixAxis
import core
from core import I2C_Lidar
import time
import PID
import numpy as np
# import sounds

''' 10-2-2017: This code is completely untested; don't be surprised when it
doesn't compile, run or do anything sensible.'''

''' 15-3-2017: This code makes a decent stab at driving around
    the minimal maze, though it sometimes bumps walls '''


class WallFollower:
    def __init__(self, core_module, oled):
        """Class Constructor"""
        self.killed = False
        self.core = core_module
        self.oled = oled
        self.ticks = 0
        self.tick_time = 0.1  # How many seconds per control loop
        self.time_limit = 30  # How many seconds to run for
        self.follow_left = False  # Start by following RIGHT wall
        self.switches_count = 0
        self.exit_speed = 1.0
        self.last_switch_ticks = 0

        self.distance_midpoint = 250.0
        self.distance_range = 100.0

        # Keep X times more distance from the
        # bot's front than from the side
        # self.front_cautious = 3.5  # 2.5
        self.front_cautious = 4.0  # 2.5

        # Known working speeds
        # speed_mid = 40 * self.exit_speed
        # # Positive for Left/Right, Negative for Right/Left
        # speed_range = -55
        self.speed_mid = 60 * self.exit_speed
        # Positive for Left/Right, Negative for Right/Left
        self.speed_range = -70

        self.front_high_speed_threshold = 600
        self.front_low_speed_threshold = 250
        self.low_speed_factor = 0.70

        # PID class
        self.pidc = PID.PID(0.5, 0.0, 0.1)  # Maze values
        # self.pidc = PID.PID(0.5, 0.0, 0.05)  # Speed run values

    def stop(self):
        """Simple method to stop the RC loop"""
        self.killed = True

    def set_control_mode(self, mode):
        self.control_mode = mode

    def show_state(self):
        """ Show motor/aux config on OLED display """
        if self.oled is not None:
            # Format the speed to 2dp
            if self.core.motors_enabled():
                message = "SPEED: %0.2f" % (self.core.get_speed_factor())
            else:
                message = "SPEED: NEUTRAL (%0.2f)" % (self.core.get_speed_factor())

            self.oled.cls()  # Clear Screen
            self.oled.canvas.text((10, 10), message, fill=1)
            # Now show the mesasge on the screen
            self.oled.display()

    def decide_speeds(self, sensorvalue, ignore_d, d_front):
        """ Set up return values at the start"""
        leftspeed = 0
        rightspeed = 0

        if self.control_mode == "PID":
            error = (sensorvalue - self.distance_midpoint)
            self.pidc.update(error, ignore_d)

            deviation = self.pidc.output / self.distance_range
            c_deviation = max(-1.0, min(1.0, deviation))

            #print("PID out: %f" % deviation)

            if self.follow_left:
                leftspeed = (self.speed_mid - (c_deviation * self.speed_range))
                rightspeed = (self.speed_mid + (c_deviation * self.speed_range))
            else:
                leftspeed = (self.speed_mid + (c_deviation * self.speed_range))
                rightspeed = (self.speed_mid - (c_deviation * self.speed_range))

            if d_front < self.front_high_speed_threshold:
                # Variable speed variance
                self.front_high_speed_threshold = 600
                self.front_low_speed_threshold = 250
                self.low_speed_factor = 0.70

                xp = [self.front_low_speed_threshold,
                      self.front_high_speed_threshold]
                fp = [self.low_speed_factor, 1.0]
                new_factor = np.interp(d_front, xp, fp)

                if new_factor > 1.0:
                    new_factor = 1.0
                if new_factor < self.low_speed_factor:
                    new_factor = self.low_speed_factor
                leftspeed *= new_factor
                rightspeed *= new_factor

                # Simple speed variance
                # leftspeed *= self.low_speed_factor
                # rightspeed *= self.low_speed_factor

            if (leftspeed < rightspeed):
                print("Turning left")
            else:
                print("Turning right")

        else:
            leftspeed = self.speed_mid
            rightspeed = self.speed_mid

        return leftspeed, rightspeed

    def run(self):
        print("Start run")
        """Read a sensor and set motor speeds accordingly"""
        # self.core.enable_motors(True)

        print("Waiting for motor enable")
        while not self.killed and not self.core.motors_enabled():
            time.sleep(0.5)

        # Stop processing if we have killed the thread
        if self.killed:
            return

        print("Starting maze run now...")

        tick_limit = self.time_limit / self.tick_time
        #print("Tick limit %d" % (tick_limit))
        self.set_control_mode("PID")

        side_prox = 0
        prev_prox = 100  # Make sure nothing bad happens on startup

        while not self.killed and self.ticks < tick_limit and side_prox != -1:
            prev_prox = side_prox

            # New API
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_RIGHT)
                ]
                d_left = lidar_dev['device'].get_distance()
                #print("Left: %d" % (d_left))
            except KeyError:
                d_left = -1
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_FRONT)
                ]
                d_front = lidar_dev['device'].get_distance()
                #print("Front: %d" % (d_front))
            except KeyError:
                d_front = -1
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_LEFT)
                ]
                d_right = lidar_dev['device'].get_distance()
                #print("Right: %d" % (d_right))
            except KeyError:
                d_right = -1

            # Which wall are we following?
            if self.follow_left:
                side_prox = d_left  # 0:Left, 2: right
            else:
                side_prox = d_right

            # Keep X times more distance from the
            # bot's front than from the side
            front_prox = d_front / self.front_cautious

            # Have we fallen out of the end of the course?
            # if d_left > 400 and d_right > 400 and d_front > 400:
            #     print("*** EXIT CONDITION ***")
            #     self.killed = True
            #     break

            # print("Distance is %d" % (side_prox))

            ignore_d = False
            # Have we crossed over the middle of the course?
            if (side_prox > 350 and side_prox - 150 > prev_prox and self.ticks > 5):
                print("*** SWITCH EVENT {} ***".format(self.switches_count))
                if (self.switches_count == 0):
                    print("Distance above threshold, left through hairpins")
                    self.follow_left = True
                    self.switches_count = 1
                    # Tell PID not to wig out too much
                    ignore_d = True
                    self.last_switch_ticks = self.ticks
                elif (self.switches_count == 1):
                    print("Distance above threshold, follow right again")
                    self.follow_left = False
                    self.switches_count = 2
                    # Tell PID not to wig out too much
                    ignore_d = True
                    self.last_switch_ticks = self.ticks
                elif (self.switches_count == 2 and self.ticks > self.last_switch_ticks + 40):
                    print("Distance above threshold, exit maze")
                    self.follow_left = True
                    self.switches_count = 3
                    # Slow down to make us exit gracefully?
                    self.exit_speed = 1.0  # Slowing down is for wimps
                    # Tell PID not to wig out too much
                    ignore_d = True
                    self.last_switch_ticks = self.ticks

            # Potentially irrelevant if we exit on stage 2
            elif (self.switches_count == 3 and front_prox < 350):
                    print("Close enough to the last wall, turn right")
                    self.switches_count = 4
            leftspeed = 0
            rightspeed = 0

            leftspeed, rightspeed = self.decide_speeds(
                min(side_prox, front_prox),
                ignore_d,
                d_front
            )

            self.core.throttle(leftspeed, rightspeed)
            #print("Motors %0.2f, %0.2f" % (leftspeed, rightspeed))
            # print("Are we dead?")
            # print(self.killed)
            # print("%d ticks" % (self.ticks))

            self.ticks = self.ticks + 1
            time.sleep(0.1)

        #print("Ticks %d" % self.ticks)

        self.core.set_neutral(braked=False)


if __name__ == "__main__":
    core = core.Core()
    follower = WallFollower(core)
    try:
        follower.run()
    except (KeyboardInterrupt) as e:
        # except (Exception, KeyboardInterrupt) as e:
        # Stop any active threads before leaving
        follower.stop()
        core.stop()
        print("Quitting")
