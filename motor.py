import logging
import time
import sys

logging.basicConfig(stream=sys.stdout, level=logging.INFO)


class Motor():
    """ Instantiate a single motor module. """

    def __init__(
        self,
        GPIO,
        a_pin,  # -1 if not used
        b_pin,  # -1 if not used
        pwm_pin,
        motor_name,
        enabled=False,
        pwm_frequency=1000,
        speed_factor=1.0
    ):
        """ Constructor """
        self.GPIO = GPIO
        self.motor_name = motor_name
        self.running = False
        self.a_pin = a_pin
        self.b_pin = b_pin
        self.pwm_pin = pwm_pin
        self.enabled = enabled  # Disabled by default by
        self.pwm_frequency = pwm_frequency
        self.percent_change_per_interval = 0.0  # zero means instant
        self.set_full_accelleration_time(0.0)

        # Setup the GPIO pins as OUTPUTS
        self.GPIO.setup(pwm_pin, self.GPIO.OUT)
        if self.a_pin >= 0:
            self.GPIO.setup(a_pin, self.GPIO.OUT)
        if self.b_pin >= 0:
            self.GPIO.setup(b_pin, self.GPIO.OUT)

        # Initialise a and b pins to zero (neutral)
        if self.a_pin >= 0:
            self.GPIO.output(a_pin, 0)
        if self.b_pin >= 0:
            self.GPIO.output(b_pin, 0)

        # create object D2A for PWM
        self.PWM = self.GPIO.PWM(pwm_pin, pwm_frequency)
        self.PWM.start(0)  # Init the PWM with a 0 percent duty cycle (off)

        self.forward = True  # Remember which direction it was travelling
        self.target_speed = 0.0  # User defined speed in range [-100.0, 100.0]
        self.current_speed = 0.0  # Actual motor speed in range [-100.0, 100.0]
        self.speed_factor = speed_factor  # speed multiplier, range [0.0, 1.0]
        self.max_speed = 90.0  # Speed limit

    def cleanup(self):
        """ Perform all necessary tasks to destruct this class. """
        self.stop()
        self.PWM.stop()  # stop the PWM output

    def get_speed_factor(self):
        return self.speed_factor

    def set_speed_factor(self, factor):
        self.speed_factor = factor
        # Clamp speed factor to [0.1, 1.0]
        if self.speed_factor > 1.0:
            self.speed_factor = 1.0
        elif self.speed_factor < 0.1:
            self.speed_factor = 0.1
        print ("New speed factor %0.1f" % (self.speed_factor))

    def increase_speed_factor(self, increment=0.05):
        self.speed_factor += increment
        # Clamp speed factor to [0.1, 1.0]
        if self.speed_factor > 1.0:
            self.speed_factor = 1.0
        elif self.speed_factor < 0.1:
            self.speed_factor = 0.1
        print ("New speed factor %0.1f" % (self.speed_factor))

    def decrease_speed_factor(self, increment=0.05):
        self.speed_factor -= increment
        # Clamp speed factor to [0.1, 1.0]
        if self.speed_factor > 1.0:
            self.speed_factor = 1.0
        elif self.speed_factor < 0.1:
            self.speed_factor = 0.1
        print ("New speed factor %0.1f" % (self.speed_factor))

    def set_neutral(self, braked=False):
        """ Send neutral to the motor IMEDIATELY. """
        # Setting MOTOR pins to LOW will make it free wheel.
        pin_value = 0
        if braked:
            pin_value = 1  # Setting to HIGH will do active braking.

        # Set a and b pins to either 1 or 0.
        if self.a_pin >= 0:
            self.GPIO.output(self.a_pin, pin_value)
        if self.b_pin >= 0:
            self.GPIO.output(self.b_pin, pin_value)

        # Turn motors off by setting duty cycle back to zero.
        dutycycle = 0.0
        self.target_speed = 0.0
        self.current_speed = 0.0
        self.PWM.ChangeDutyCycle(dutycycle)

    def enable_motor(self, enabled):
        """ Change the enabled/disabled flag in this class. """
        self.enabled = enabled

        # Set motors in neutral if disabling.
        if not self.enabled:
            self.set_neutral()

    def set_motor_speed(self, speed=0.0):
        """ Change a motors speed.
            Method expects a value in the range of [-100.0, 100.0] """
        self.target_speed = speed

    def change_motor_speed(self, speed=0.0):
        """ Called from this class's RUN loop
            to change actual speed to target speed. """
        if not self.enabled:
            self.set_neutral(braked=False)
            return

        logging.info("{} Motor Speed: {}".format(self.motor_name, speed))
        self.current_speed = speed  # Store current set speed

        # If speed is < 0.0, we are driving in reverse.
        self.forward = True
        if speed < 0.0:
            # Normalise speed value to be in range [0, 100]
            speed = -speed
            # Store direction
            self.forward = False

        # Apply a factor to the speed to limit speed
        speed *= self.speed_factor

        # Set motor directional pins
        if self.forward:
            if self.a_pin >= 0:
                self.GPIO.output(self.a_pin, 1)
            if self.b_pin >= 0:
                self.GPIO.output(self.b_pin, 0)
        else:
            if self.a_pin >= 0:
                self.GPIO.output(self.a_pin, 0)
            if self.b_pin >= 0:
                self.GPIO.output(self.b_pin, 1)

        # Convert speed into PWM duty cycle
        # and clamp values to min/max ranges.
        dutycycle = speed
        if dutycycle < 0.0:
            dutycycle = 0.0
        elif dutycycle > self.max_speed:
            dutycycle = self.max_speed

        # Change the PWM duty cycle based on fabs() of speed value.
        self.PWM.ChangeDutyCycle(dutycycle)

    def set_full_accelleration_time(self, time_in_seconds):
        """ Set the time it should take to
        accellerate from zero to 100% in seconds. """
        if time_in_seconds == 0.0:
            self.percent_change_per_interval = 0.0
        else:
            number_of_intervals = time_in_seconds / 0.005
            self.percent_change_per_interval = 100.0 / number_of_intervals

    def stop(self):
        """ Call this method to stop the thread. """
        self.running = False

    def run(self):
        """ Method loops constantly. Call as a new thread. """
        try:
            self.running = True  # Flag set to TRUE, runs until its turned off.
            while self.running:
                # Does nothing at the moment.
                # Future improvement to cope with acceleration.
                if self.current_speed != self.target_speed:

                    # zero means instant
                    if self.percent_change_per_interval == 0.0:
                        self.change_motor_speed(self.target_speed)
                    else:
                        percent_change = self.percent_change_per_interval
                        # Invert percent change if decelerating
                        new_speed = self.target_speed
                        if self.target_speed > self.current_speed:
                            # Calculate new speed and set it
                            new_speed = self.current_speed + percent_change
                            if new_speed > self.target_speed:
                                new_speed = self.target_speed
                        elif self.target_speed < self.current_speed:
                            # Calculate new speed and set it
                            percent_change = -percent_change
                            new_speed = self.current_speed + percent_change
                            if new_speed < self.target_speed:
                                new_speed = self.target_speed

                        self.change_motor_speed(new_speed)

                # Very small delay between run loops
                time.sleep(0.005)
        except Exception as e:
            # Handle EVERY exception to try and fail safe
            print(e)

        # If thread terminated, ensure motors are OFF
        self.set_neutral(braked=True)
