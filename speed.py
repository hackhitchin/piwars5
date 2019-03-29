from core import I2C_Lidar
import time
import PID
import numpy as np


class Speed:

    def __init__(self, core_module, oled, control_mode):
        """Class Constructor"""
        self.oled = oled
        self.killed = False
        self.core = core_module
        self.time_limit = 20  # How many seconds before auto cutoff
        # self.pidc = PID.PID(1.3, 0.4, 0.6)  # 2wd speed test Initial Settings
        # self.pidc = PID.PID(1.5, 0.4, 0.8) # works at 20% constant speed
        self.pidc = PID.PID(0.5, 0.0, 0.05) # OK at 25% speed
        if control_mode is None:
            self.control_mode = "PID"
        else:
            self.control_mode = control_mode
        self.deadband = 50  # size of deadband in mm

        # self.pidc = PID.PID(1.0, 0.0, 0.0)
        self.threshold_side = 600.0
        # 20mm Small distance, basically
        # its just going to hit and we need to stop
        self.threshold_front = 100.0

        self.off_the_line_time = 0.25

        self.front_high_speed_threshold = 800
        self.front_low_speed_threshold = 500
        self.low_speed_factor = 0.15

    def stop(self):
        """Simple method to stop the RC loop"""
        self.killed = True

    def decide_speeds_linear(self, distance_left, distance_right):
        """ Use the linear method to decide motor speeds. """
        speed_max = 1.0
        distance_offset = distance_left - distance_right
        turn_force_per_mm = 0.01

        if (abs(distance_offset) <= self.deadband):
            # Within reasonable tolerance of centre, don't bother steering
            # print("Deadband")
            leftspeed = speed_max
            rightspeed = speed_max
        else:
            # Calculate how much to reduce speed by on ONE MOTOR ONLY
            speed_drop = (abs(distance_offset) * turn_force_per_mm)

            # Cap speed drop to [0.0, 1.0]
            if speed_drop < 0:
                speed_drop = 0
            if speed_drop > 1.0:
                speed_drop = 1.0
            # ("DropSpeed = {}".format(speed_drop))

            if distance_offset < 0:
                leftspeed = speed_max - speed_drop
                rightspeed = speed_max
            elif distance_offset > 0:
                leftspeed = speed_max
                rightspeed = speed_max - speed_drop
            else:
                leftspeed = speed_max
                rightspeed = speed_max

        # Left motors are ever so slightly slower,
        # fudge the rign speed down a tad to balance
        # rightspeed *= 0.8

        return leftspeed, rightspeed

    def decide_speeds_pid(self, distance_left, distance_right, distance_front):
        """ Use the pid  method to decide motor speeds. """
        distance_offset = distance_left - distance_right
        speed_mid = 1  # 0.3 safe
        speed_range = 1  # -0.2 - backwards motors?
        distance_range = 150.0  # was 50
        # speed_max = 1.0

        ignore_d = False
        self.pidc.update(distance_offset, ignore_d)
        # self.pidc.update(pow(abs(distance_offset), 1.06)*np.sign(distance_offset)*0.75, ignore_d)

        deviation = self.pidc.output / distance_range
        c_deviation = max(-1.0, min(1.0, deviation))

        # print("PID out: %f" % deviation)

        new_factor = 1.0
        if distance_front < self.front_high_speed_threshold:
            # Variable speed variance
            xp = [self.front_low_speed_threshold,
                  self.front_high_speed_threshold]
            fp = [self.low_speed_factor/self.core.get_speed_factor(), 1.0]
            new_factor = np.interp(distance_front, xp, fp)

            if new_factor > 1.0:
                new_factor = 1.0

        if (c_deviation > 0):
            leftspeed = ((speed_mid * new_factor) - (c_deviation * speed_range))
            rightspeed = ((speed_mid * new_factor) + (c_deviation * speed_range * 1.0))
        else:
            leftspeed = ((speed_mid * new_factor) - (c_deviation * speed_range * 1.0))
            rightspeed = ((speed_mid * new_factor) + (c_deviation * speed_range))

            # Equal % drop but looses steering %
            # leftspeed *= new_factor
            # rightspeed *= new_factor


            print("{}  {}".format(leftspeed, rightspeed))

        # if distance_front < 250.0:
        #     if distance_left < distance_Right:
        #         leftspeed = 0.0
        #     else:
        #         rightspeed = 0.0

        # rightspeed *= 0.8  # FUDGE the right motors slower a bit because they are stronger

        return leftspeed, rightspeed

    def decide_speeds(self, distance_left, distance_right, distance_front, time_delta):
        """ Set up return values at the start"""
        leftspeed = 0
        rightspeed = 0

        if self.control_mode == "LINEAR":
            leftspeed, rightspeed = self.decide_speeds_linear(
                distance_left, distance_right
            )
        elif self.control_mode == "PID":
            leftspeed, rightspeed = self.decide_speeds_pid(
                distance_left, distance_right, distance_front
            )

        # Linearly increase motor speeds off the line
        if time_delta <= self.off_the_line_time:
            factor = time_delta / self.off_the_line_time
            if factor > 1.0:
                factor = 1.0
            leftspeed *= factor
            rightspeed *= factor
        return leftspeed, rightspeed

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

    def run(self):
        """Read a sensor and set motor speeds accordingly"""

        # Wait (and do nothing) until we enable
        # motors or kill the challenge thread.
        print("Waiting for motor enable")
        while not self.killed and not self.core.motors_enabled():
            time.sleep(0.5)

        # Stop processing if we have killed the thread
        if self.killed:
            return

        print("Starting speed run now...")

        # Reduce MAX speed to 80%, stops motor controller kicking off.
        # self.core.speed_factor = 0.5

        start_time = time.time()
        time_delta = 0
        soft_start_power = 0.5  # 50% power during soft start
        soft_start_time = 0.5  # seconds of soft start

        while not self.killed and time_delta < self.time_limit:
            # Calculate time delta FIRST so we can
            # pass it into the motor speed function.
            current_time = time.time()
            time_delta = current_time - start_time
            # print("{}".format(time_delta))
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_LEFT)
                ]
                distance_left = lidar_dev['device'].get_distance()
                # print("Left: %d" % (distance_left))
            except KeyError:
                distance_left = -1
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_FRONT)
                ]
                distance_front = lidar_dev['device'].get_distance()
                # print("Front: %d" % (distance_front))
            except KeyError:
                distance_front = -1
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_RIGHT)
                ]
                distance_right = lidar_dev['device'].get_distance()
                # print("Right: %d" % (distance_right))
            except KeyError:
                distance_right = -1

            # Have we fallen out of the end of
            # the course or nearing obstruction?
            # if ((distance_left > self.threshold_side and
            #    distance_right > self.threshold_side) or
            #    distance_front < self.threshold_front):
            #     print("Outside of speed run")
            #     self.killed = True
            #     break

            if not self.killed:
                # Report offset from centre
                distance_offset = distance_left - distance_right
                # print("Offset is %d (%d : %d)" % (distance_offset, distance_left, distance_right))

                #if self.control_mode == "LINEAR":
                #    self.deadband = (distance_left + distance_right) / 4.0

                # Calculate motor speeds
                leftspeed, rightspeed = self.decide_speeds(
                    distance_left,
                    distance_right,
                    distance_front,
                    time_delta
                )

                if (time.time() < soft_start_time):
                    leftspeed = leftspeed * soft_start_power
                    rightspeed = rightspeed * soft_start_power

                # Send speeds to motors
                self.core.throttle(leftspeed * 100.0, rightspeed * 100.0)
                # print("Motors %f, %f" % (leftspeed, rightspeed))

                time.sleep(0.1)

        if time_delta < self.time_limit:
            print("Timeout")

        # Turn motors off and set into neutral (stops the vehicle moving)
        print("Appling Brakes")
        #self.core.throttle(-50.0, -50.0)
        #time.sleep(0.25)
        print("Neutral")
        self.core.enable_motors(False)


def main():
    """ Method run when codule called separately. """
    import RPi.GPIO as GPIO
    import core

    # Initialise GPIO
    GPIO = GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    # Instantiate CORE / Chassis module and store in the launcher.
    core_module = core.Core(GPIO)

    speed = Speed(core_module, None)

    # Manually enable motors in this mode as controller not hooked up
    core_module.enable_motors(True)
    speed.run()


if __name__ == '__main__':
    main()
