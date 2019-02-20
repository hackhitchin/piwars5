from core import I2C_Lidar
import time
import PID


class Speed:

    def __init__(self, core_module, oled, control_mode):
        """Class Constructor"""
        self.oled = oled
        self.killed = False
        self.core = core_module
        self.time_limit = 6  # How many seconds before auto cutoff
        # self.pidc = PID.PID(0.7, 0.3, 0.4) # best I've got so far
        self.pidc = PID.PID(1.3, 0.4, 0.6)
        if control_mode is None:
            self.control_mode = "PID"
        else:
            self.control_mode = control_mode
        self.deadband = 100  # size of deadband in mm

        # self.pidc = PID.PID(1.0, 0.0, 0.0)
        self.threshold_side = 400.0
        self.threshold_front = 50.0  # 200.0 - too prone to stopping unnecessarily

        self.off_the_line_time = 0.25

    def stop(self):
        """Simple method to stop the RC loop"""
        self.killed = True

    def decide_speeds_linear(self, distance_offset):
        """ Use the linear method to decide motor speeds. """
        speed_max = 1.0
        arbitrary_offset = 100

        if (abs(distance_offset) <= self.deadband):
            # Within reasonable tolerance of centre, don't bother steering
            print("Deadband")
            leftspeed = speed_max
            rightspeed = speed_max
        else:
            # Clamp offset to 100mm (arbitrary value for now)
            if distance_offset > arbitrary_offset:
                distance_offset = arbitrary_offset
            elif distance_offset < -arbitrary_offset:
                distance_offset = -arbitrary_offset

            # Calculate how much to reduce speed by on ONE MOTOR ONLY
            speed_drop = (abs(distance_offset) / float(arbitrary_offset))
            print("DropSpeed = {}".format(speed_drop))

            # Linear drop in turning ability.speed_factor
            # the faster we go, the less we turn.
            min_ff = 0.4
            max_ff = 1.0
            speed_drop_ff = 1.0 - self.core.speed_factor
            if speed_drop_ff < min_ff:
                speed_drop_ff = min_ff
            if speed_drop_ff > max_ff:
                speed_drop_ff = max_ff

            # Reduce speed drop by factor to turning sensitivity/affect.
            speed_drop *= speed_drop_ff  # 0.8

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

    def decide_speeds_pid(self, distance_offset):
        """ Use the pid  method to decide motor speeds. """
        speed_mid = 1  # 0.3 safe
        speed_range = 1  # -0.2 - backwards motors?
        distance_range = 150.0  # was 50
        speed_max = 1.0

        ignore_d = False
        self.pidc.update(distance_offset, ignore_d)

        deviation = self.pidc.output / distance_range
        c_deviation = max(-1.0, min(1.0, deviation))

        print("PID out: %f" % deviation)

        if (abs(distance_offset) <= self.deadband):
            # Within reasonable tolerance of centre, don't bother steering
            print("Deadband")
            leftspeed = speed_max
            rightspeed = speed_max
        else:
            # Slow one motor more than we speed the other one up
            if (c_deviation > 0):
                leftspeed = (speed_mid - (c_deviation * speed_range))
                rightspeed = (speed_mid + (c_deviation * speed_range * 0.8))
            else:
                leftspeed = (speed_mid - (c_deviation * speed_range * 0.8))
                rightspeed = (speed_mid + (c_deviation * speed_range))

        # rightspeed *= 0.8  # FUDGE the right motors slower a bit because they are stronger

        return leftspeed, rightspeed

    def decide_speeds(self, distance_offset, time_delta):
        """ Set up return values at the start"""
        leftspeed = 0
        rightspeed = 0

        if self.control_mode == "LINEAR":
            leftspeed, rightspeed = self.decide_speeds_linear(
                distance_offset
            )
        elif self.control_mode == "PID":
            leftspeed, rightspeed = self.decide_speeds_pid(
                distance_offset
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
            if self.core.motors_enabled:
                message = "SPEED: %0.2f" % (self.core.speed_factor)
            else:
                message = "SPEED: NEUTRAL (%0.2f)" % (self.core.speed_factor)

            self.oled.cls()  # Clear Screen
            self.oled.canvas.text((10, 10), message, fill=1)
            # Now show the mesasge on the screen
            self.oled.display()

    def run(self):
        """Read a sensor and set motor speeds accordingly"""

        # Wait (and do nothing) until we enable
        # motors or kill the challenge thread.
        print("Waiting for motor enable")
        while not self.killed and not self.core.motors_enabled:
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
            print("{}".format(time_delta))
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_LEFT)
                ]
                distance_left = lidar_dev['device'].get_distance()
                print("Left: %d" % (distance_left))
            except KeyError:
                distance_left = -1
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_FRONT)
                ]
                distance_front = lidar_dev['device'].get_distance()
                print("Front: %d" % (distance_front))
            except KeyError:
                distance_front = -1
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_RIGHT)
                ]
                distance_right = lidar_dev['device'].get_distance()
                print("Right: %d" % (distance_right))
            except KeyError:
                distance_right = -1

            # Have we fallen out of the end of
            # the course or nearing obstruction?
            if ((distance_left > self.threshold_side and distance_right > self.threshold_side) or
               distance_front < self.threshold_front):
                print("Outside of speed run")
                self.killed = True
                break

            if not self.killed:
                # Report offset from centre
                distance_offset = distance_left - distance_right
                print("Offset is %d (%d : %d)" % (distance_offset, distance_left, distance_right))

                if self.control_mode == "LINEAR":
                    self.deadband = (distance_left + distance_right) / 4.0

                # Got too close, ensure motors are actually working
                #if distance_left <= 80 or distance_right <= 80:
                #    print("Resetting motors")
                #    self.core.reset_motors()

                # Calculate motor speeds
                leftspeed, rightspeed = self.decide_speeds(distance_offset, time_delta)

                if (time.time() < soft_start_time):
                    leftspeed = leftspeed * soft_start_power
                    rightspeed = rightspeed * soft_start_power

                # Send speeds to motors
                self.core.throttle(leftspeed * 100.0, rightspeed * 100.0)
                print("Motors %f, %f" % (leftspeed, rightspeed))

                time.sleep(0.1)

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
