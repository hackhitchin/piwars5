# Import triangula module to interact with SixAxis
import core
from core import I2C_Lidar
import time
import PID
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
        self.time_limit = 16  # How many seconds to run for
        self.follow_left = True
        self.switched_wall = False

# known good for straight line, underdamped
#        self.pidc = PID.PID(0.5, 0.0, 0.2)

# test for maze:
#        self.pidc = PID.PID(0.5, 0.0, 0.1)
        self.pidc = PID.PID(0.5, 0.0, 0.1)

    def stop(self):
        """Simple method to stop the RC loop"""
        self.killed = True

    def set_control_mode(self, mode):
        self.control_mode = mode
        
    def show_state(self):
        """ Show motor/aux config on OLED display """
        if self.oled is not None:
            # Format the speed to 2dp
            if self.core.motors_enabled:
                message = "SPEED: %0.2f" % (self.core.speed_factor)
            else:
                message = "SPEED: NEUTRAL (%0.2f)" % (self.core.speed_factor)

            self.oled.cls()  # Clear Screen
            self.oled.canvas.text((10, 10), message, fill=1)
            # Now show the mesasge on the screen
            self.oled.display()        

    def decide_speeds(self, sensorvalue, ignore_d):
        """ Set up return values at the start"""
        leftspeed = 0
        rightspeed = 0

        if self.control_mode == "LINEAR":
            speed_mid = -0.2
            speed_range = 0.06
            """ Deviation is distance from intended midpoint.
                Right is positive, left is negative
                Rate is how much to add/subtract from motor speed """

            distance_midpoint = 200.0  # mm
            distance_range = 100.0  # mm
            deviation = (sensorvalue - distance_midpoint) / distance_range  # [-1, 1]

            # Gate value to [-1,1] for the sake of not driving backwards
            if (deviation < -1):
                deviation = -1
            if (deviation > 1):
                deviation = 1
            if self.follow_left:
                leftspeed = (speed_mid - (deviation * speed_range))
                rightspeed = (speed_mid + (deviation * speed_range))
            else:
                leftspeed = (speed_mid + (deviation * speed_range))
                rightspeed = (speed_mid - (deviation * speed_range))

            return leftspeed, rightspeed

        elif self.control_mode == "EXPO":
            speed_mid = 0.05
            speed_range = 0.05

            distance_midpoint = 200.0  # mm
            distance_range = 100.0  # mm
            deviation = (sensorvalue - distance_midpoint) / distance_range  # [-1, 1]

            if (deviation < 0):
                deviation = 0 - (deviation * deviation)
            else:
                deviation = deviation * deviation

            leftspeed = (speed_mid - (deviation * speed_range))
            rightspeed = (speed_mid + (deviation * speed_range))

        elif self.control_mode == "PID":
            # straight line, cautious: mid -0.2, range -0.2
            # maze, cautious: mid -0.1, range -0.2
            # maze, tuned: mid -0.14, range -0.2

            speed_mid = 14 # 0.14
            speed_range = -20

            distance_midpoint = 200.0
            distance_range = 100.0
            error = (sensorvalue - distance_midpoint)
            self.pidc.update(error, ignore_d)

            deviation = self.pidc.output / distance_range
            c_deviation = max(-1.0, min(1.0, deviation))

            print("PID out: %f" % deviation)

            if self.follow_left:
                leftspeed = (speed_mid - (c_deviation * speed_range))
                rightspeed = (speed_mid + (c_deviation * speed_range))
            else:
                leftspeed = (speed_mid + (c_deviation * speed_range))
                rightspeed = (speed_mid - (c_deviation * speed_range))

            if (leftspeed < rightspeed):
                print("Turning left")
            else:
                print("Turning right")

        else:
            leftspeed = speed_mid
            rightspeed = speed_mid

        return leftspeed, rightspeed

    def run(self):
        print("Start run")
        """Read a sensor and set motor speeds accordingly"""
        self.core.enable_motors(True)

        tick_limit = self.time_limit / self.tick_time
        print("Tick limit %d" % (tick_limit) )
        self.set_control_mode("PID")

        side_prox = 0
        prev_prox = 100  # Make sure nothing bad happens on startup

        while not self.killed and self.ticks < tick_limit and side_prox != -1:
            prev_prox = side_prox
            
            # Old API
            #d_left = self.core.read_sensor(0)
            #d_front = self.core.read_sensor(1) - 150
            #d_right = self.core.read_sensor(2)
            
            # New API
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_RIGHT)
                ]
                d_left = lidar_dev['device'].get_distance()
                print("Left: %d" % (d_left) )
            except KeyError:
                d_left = -1
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_FRONT)
                ]
                d_front = lidar_dev['device'].get_distance()
                print("Front: %d" % (d_front) )
            except KeyError:
                d_front = -1
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_LEFT)
                ]
                d_right = lidar_dev['device'].get_distance()
                print("Right: %d" % (d_right) )
            except KeyError:
                d_right = -1            

            # Which wall are we following?
            if self.follow_left:
                side_prox = d_left  # 0:Left, 2: right
            else:
                side_prox = d_right

            # Keep X times more distance from the bot's front than from the side
            front_cautious = 1.6
            front_prox = d_front / front_cautious

            # Have we fallen out of the end of the course?
            if d_left > 400 and d_right > 400:
                self.killed = True
                break

            print("Distance is %d" % (side_prox))

            ignore_d = False
            # Have we crossed over the middle of the course?
            if (side_prox > 350 and
               (side_prox - 100 > prev_prox) and
               self.switched_wall is False):
                print("Distance above threshold, follow right")
                self.follow_left = False
                self.switched_wall = True
                # Tell PID not to wig out too much
                ignore_d = True
            leftspeed = 0
            rightspeed = 0

            leftspeed, rightspeed = self.decide_speeds(
                min(side_prox, front_prox),
                ignore_d
            )
            
            self.core.throttle(leftspeed, rightspeed)
            print("Motors %0.2f, %0.2f" % (leftspeed, rightspeed))
            print("Are we dead?")
            print(self.killed)
            print("%d ticks" % (self.ticks) )

            self.ticks = self.ticks + 1
            time.sleep(0.1)

        print("Ticks %d" % self.ticks)

        self.core.set_neutral(False)


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
