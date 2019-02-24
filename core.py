from __future__ import division
import time
# import i2c_lidar
from enum import Enum
import VL53L0X
import motor

MOTOR_LEFT_PWM = 17
MOTOR_LEFT_A = 22
MOTOR_LEFT_B = 27

MOTOR_RIGHT_PWM = 18
MOTOR_RIGHT_A = 23
MOTOR_RIGHT_B = 24

MOTOR_GUN_SERVO = 13
MOTOR_TURRET_SERVO = 6
MOTOR_GUN_PWM = 12
MOTOR_GUN_A = 5
MOTOR_GUN_B = 25

MAX_SPEED = 90


class I2C_Lidar(Enum):
    # Enum listing each servo that we can control
    LIDAR_FRONT = 10
    LIDAR_LEFT = 9
    LIDAR_RIGHT = 11


class Core():
    """ Instantiate a 4WD drivetrain, utilising 2x H Bridges,
        controlled using a 2 axis (throttle, steering)
        system """

    def __init__(self, GPIO):
        """ Constructor """

        self.i2cbus = VL53L0X.i2cbus
        self.resetting_motors = False
        self.ena_pins = False

        # Motors will be disabled by default.
        self.motors_enabled = False
        self.gun_enabled = False
        self.GPIO = GPIO
        self.DEBUG = False

        self.GPIO.setup(MOTOR_GUN_SERVO, GPIO.OUT)
        self.gun_servo = self.GPIO.PWM(MOTOR_GUN_SERVO, 100)  # pin 33
        duty = float(175.0) / 10.0 + 2.5
        self.gun_servo.start(duty)

        # turret servo config
        self.turret_max = 180.0 / 2.0
        self.turret_min = 15.0
        self.turret_current = self.turret_min
        self.GPIO.setup(MOTOR_TURRET_SERVO, GPIO.OUT)
        duty = float(self.turret_min) / 10.0 + 2.5

        # Configure motor pins with GPIO
        self.motor = dict()
        self.motor['left'] = motor.Motor(
            GPIO,
            MOTOR_LEFT_A,
            MOTOR_LEFT_B,
            MOTOR_LEFT_PWM
        )
        self.motor['right'] = motor.Motor(
            GPIO,
            MOTOR_RIGHT_A,
            MOTOR_RIGHT_B,
            MOTOR_RIGHT_PWM
        )

        self.motor['gun'] = self.setup_motor(
            MOTOR_GUN_PWM,
            MOTOR_GUN_A,
            MOTOR_GUN_B
        )

        # Create a list of I2C time of flight lidar sensors
        # Note: we need to dynamically alter each
        # tof lidar sensors i2c address on boot
        self.lidars = dict()

        # FIRST: we must turn off all of the i2c
        # devices that have the same initial address.
        for pin in I2C_Lidar:
            gpio_pin = int(pin.value)
            self.GPIO.setup(gpio_pin, self.GPIO.OUT)
            self.GPIO.output(gpio_pin, GPIO.HIGH)
            # print("{} pin is ON".format(gpio_pin))
        # Wait half second to ensure devices are OFF
        time.sleep(0.5)

        # Now loop again and change each ones address individually.
        loop = 0
        for pin in I2C_Lidar:
            gpio_pin = int(pin.value)
            # Wait for chip to wake

            # New method to create multiple I2C lidar devices.
            new_address = 0x2B + loop
            lidar_dev = dict()
            lidar_dev['gpio_pin'] = gpio_pin
            lidar_dev['i2c_address'] = new_address
            try:
                lidar_dev['device'] = VL53L0X.VL53L0X(address=new_address)
            except:
                lidar_dev['device'] = None

            # Set the pin low to turn sensor on
            self.GPIO.output(gpio_pin, self.GPIO.LOW)
            # print("{} pin is OFF".format(gpio_pin))
            # Wait half second to ensure devices are ON
            time.sleep(0.5)

            try:
                lidar_dev['device'].start_ranging(VL53L0X.VL53L0X_LONG_RANGE_MODE)
            except:
                pass

            # Assign the newly created dictionary
            # into the dictionary of lidar devices.
            self.lidars[str(pin)] = lidar_dev
            loop += 1

        # DEBUG testing
        if self.DEBUG:
            time.sleep(1.0)
            # start_ranging
            for pin in I2C_Lidar:
                try:
                    lidar_dev = self.lidars[str(pin)]
                    distance_front = lidar_dev['device'].get_distance()
                except KeyError:
                    distance_front = -1

                print('### {} ###'.format(str(pin)))
                print("{}mm".format(distance_front))
                print('######')

    def increase_speed_factor(self):
        self.motor['left'].increase_speed_factor()

    def decrease_speed_factor(self):
        self.motor['left'].decrease_speed_factor()

    def cleanup(self):
        self.motor['left'].cleanup()  # stop the PWM output
        self.motor['right'].cleanup()  # stop the PWM output
        self.motor['gun'].stop()  # stop the PWM output

        # Turn off i2c lidar tof sensors
        print("Turning off I2C TOF sensors")
        for pin in I2C_Lidar:
            try:
                lidar_dev = self.lidars[str(pin)]
                lidar_dev['device'].stop_ranging()
                self.GPIO.output(lidar_dev['gpio_pin'], self.GPIO.HIGH)
            except KeyError:
                pass

        self.GPIO.cleanup()  # clean up GPIO

    def setup_motor(self, pwm_pin, a, b, frequency=1000):
        """ Setup the GPIO for a single motor.

        Return: PWM controller for single motor.
        """
        self.GPIO.setup(pwm_pin, self.GPIO.OUT)
        self.GPIO.setup(a, self.GPIO.OUT)
        self.GPIO.setup(b, self.GPIO.OUT)

        # Initialise a and b pins to zero (neutral)
        self.GPIO.output(a, 0)
        self.GPIO.output(b, 0)

        # create object D2A for PWM
        D2A = self.GPIO.PWM(pwm_pin, frequency)
        D2A.start(0)  # Initialise the PWM with a 0 percent duty cycle (off)
        return D2A

    def set_neutral(self, braked=False):
        """ Send neutral to the motors IMEDIATELY. """

        # Setting MOTOR pins to LOW will make it free wheel.
        pin_value = 0
        if braked:
            pin_value = 1  # Setting to HIGH will do active braking.
        self.GPIO.output(MOTOR_LEFT_A, pin_value)
        self.GPIO.output(MOTOR_LEFT_B, pin_value)
        self.GPIO.output(MOTOR_RIGHT_A, pin_value)
        self.GPIO.output(MOTOR_RIGHT_B, pin_value)

        # Turn motors off by setting duty cycle back to zero.
        dutycycle = 0.0
        self.motor['left'].ChangeDutyCycle(dutycycle)
        self.motor['right'].ChangeDutyCycle(dutycycle)

    def enable_motors(self, enable):
        """ Called when we want to enable/disable the motors.
            When disabled, will ignore any new motor commands. """

        self.motors_enabled = enable

        # Set motors in neutral if disabling.
        if not enable:
            self.set_neutral()

    def event_callback(self, channel):
        """ GPIO Event callback function """
        # Test the resetting motors boolean variable.
        # This will veto multiple failures whilst we
        # are already midst motor reset.
        if not self.resetting_motors:
            self.resetting_motors = True  # Enable Veto
            self.reset_motors()  # Reset motors
            self.resetting_motors = False  # Disable Veto

    def reset_motors(self):
        """ Reset BOTH motor inouts (keeping PWM values) """
        self.reset_motor(
            self.motor['left'],
            MOTOR_LEFT_A,
            MOTOR_LEFT_B
        )
        self.reset_motor(
            self.motor['right'],
            MOTOR_RIGHT_A,
            MOTOR_RIGHT_B
        )
        print("Reset Motors")

    def reset_motor(self, motor, a, b):
        """ Reset motor inouts (keeping PWM values) """

        # Read current motor directions
        in_a = self.GPIO.input(a)
        in_b = self.GPIO.input(b)

        forward = True
        if in_a and not in_b:
            forward = True
        elif not in_a and in_b:
            forward = False

        # To Reset, put motor in reverse
        self.GPIO.output(a, 0)
        self.GPIO.output(b, 1)

        # Pause a very small time to allow it to reset
        time.sleep(0.005)

        # Now reset motor directional pins
        if forward:
            self.GPIO.output(a, 1)
            self.GPIO.output(b, 0)
        else:
            self.GPIO.output(a, 0)
            self.GPIO.output(b, 1)

    def set_motor_speed(self, motor, a, b, speed=0.0):
        """ Change a motors speed.

        Method expects a value in the range of [-100.0, 100.0]
        """
        forward = True

        # If speed is < 0.0, we are driving in reverse.
        if speed < 0.0:
            speed = -speed
            forward = False

        speed *= self.speed_factor

        # Set motor directional pins
        if forward:
            self.GPIO.output(a, 1)
            self.GPIO.output(b, 0)
        else:
            self.GPIO.output(a, 0)
            self.GPIO.output(b, 1)

        # Convert speed into PWM duty cycle
        # and clamp values to min/max ranges.
        dutycycle = speed
        if dutycycle < 0.0:
            dutycycle = 0.0
        elif dutycycle > MAX_SPEED:
            dutycycle = MAX_SPEED

        print(dutycycle)

        # Change the PWM duty cycle based on fabs() of speed value.
        motor.ChangeDutyCycle(dutycycle)

    def enable_gun(self, enable):
        speed = 0
        if enable:
            speed = -50
        self.set_motor_speed(
            self.motor['gun'],
            MOTOR_GUN_A,
            MOTOR_GUN_B,
            speed=speed
        )
        self.gun_enabled = enable

    def fire_gun(self, angle):
        duty = float(angle) / 10.0 + 2.5
        self.gun_servo.ChangeDutyCycle(duty)

    def move_turret(self, angle):
        duty = float(angle) / 10.0 + 2.5
        self.turret_servo.ChangeDutyCycle(duty)

    def move_turret_increment(self, increment):
        new_angle = self.turret_current + increment
        if new_angle < self.turret_min:
            new_angle = self.turret_min
        if new_angle > self.turret_max:
            new_angle = self.turret_max

        self.turret_current = new_angle
        self.move_turret(new_angle)
        print(new_angle)

    def throttle(
        self,
        left_speed,
        right_speed
    ):
        """ Send motors speed value in range [-100.0,100.0]
            where 0 = neutral """
        self.motor['left'].set_motor_speed(left_speed)
        self.motor['right'].set_motor_speed(right_speed)


def main():
    """ Simple method used to test motor controller. """
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)

    print("Creating CORE object")
    core = Core(GPIO)

    # Enable motors and drive forwards for x seconds.
    print("Enabling Motors")
    core.enable_motors(True)

    print("Setting Full Throttle")
    core.throttle(0.1, 0.1)
    time.sleep(5)

    print("Disabling Motors")
    core.enable_motors(False)
    # Stop PWM's and clear up GPIO
    core.cleanup()
    print("Finished")


if __name__ == '__main__':
    main()
