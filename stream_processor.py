import picamera
import picamera.array
import threading
import cv2
import numpy
from enum import Enum
import time


class State(Enum):
    LEARNING = 1
    ORIENTING = 2
    HUNTING = 3
    FINISHED = 4


# Image stream processing thread
class StreamProcessor(threading.Thread):

    def __init__(self, core_module, camera):
        self.core_module = core_module
        self.state = State.LEARNING  # Default state
        super(StreamProcessor, self).__init__()
        self.camera = camera
        self.stream = picamera.array.PiRGBArray(self.camera)
        self.event = threading.Event()
        self.terminated = False
        self.start()
        self.begin = 0

        # Auto drive settings
        self.autoMaxPower = 1.0  # Maximum output in automatic mode
        self.autoMinPower = 0.6  # Minimum output in automatic mode
        self.autoMinArea = 100  # Smallest target to move towards
        # full image
        # self.autoMaxArea = 55000  # Largest target to move towards
        # Cropped Image
        self.autoMaxArea = 40000  # Largest target to move towards
        # Target size at which we use the maximum allowed output
        self.autoFullSpeedArea = 5000

        # Colour order to visit
        self.challengecolours = ['blue', 'yellow', 'green', 'red']
        self.arenacolours = []  # Order we've detected in the arena, clockwise
        self.colourindex = 0
        self.colour = self.challengecolours[self.colourindex]
        self.lookingatcolour = ''

        # Camera settings
        self.imageWidth = 320  # Camera image width
        self.imageHeight = 240  # Camera image height

        self.imageCentreX = self.imageWidth / 2.0
        self.imageCentreY = self.imageHeight / 2.0
        self.tickInt = 0

    def run(self):
        # This method runs in a separate thread
        while not self.terminated and self.state != State.FINISHED:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                try:
                    # Read the image and do some processing on it
                    self.stream.seek(0)
                    self.ProcessImage(self.stream.array)
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()

    # Image processing function
    def ProcessImage(self, image):
        # View the original image seen by the camera.
        # Crop the image down to just the bit with the arena in
        image = image[100:240, 0:320]

        debug = False
        if debug:
            cv2.imshow('original', image)
            cv2.waitKey(0)

        # Blur the image
        # image = cv2.medianBlur(image, 5)
        # if debug:
        #     cv2.imshow('blur', image)
        #     cv2.waitKey
        # Convert the image from 'BGR' to HSV colour space
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # if debug:
        #    cv2.imshow('cvtColour', image)
        #    cv2.waitKey(0)

        if self.colour == "red":
            imrange = cv2.inRange(
                image,
                # numpy.array((113, 96, 64)),
                # numpy.array((125, 255, 255))
                numpy.array((0, 100, 50)),
                numpy.array((10, 255, 255))
            )
        elif self.colour == 'yellow':
            imrange = cv2.inRange(
                image,
                # numpy.array((15, 85, 64)),
                # numpy.array((35, 255, 255))
                numpy.array((20, 100, 75)),
                numpy.array((40, 255, 255))
            )
        elif self.colour == "green":
            imrange = cv2.inRange(
                image,
                # numpy.array((50, 96, 64)),
                # numpy.array((85, 255, 255))
                numpy.array((50, 100, 50)),
                numpy.array((80, 255, 255))
            )
        elif self.colour == 'blue':
            imrange = cv2.inRange(
                image,
                numpy.array((90, 150, 64)),
                numpy.array((130, 255, 255))
            )

        # Blur the mask, not the image
        imrange = cv2.medianBlur(imrange, 5)
        if debug:
            cv2.imshow('imrange', imrange)
            cv2.waitKey()

        # Find the contours
        contourimage, contours, hierarchy = cv2.findContours(
            imrange,
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE
        )
        # if debug:
        #    cv2.imshow('contour', contourimage)
        #    cv2.waitKey(0)

        # Go through each contour
        squareness = -1
        x = -1
        y = -1
        area = 0
        for (idx, contour) in enumerate(contours):
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + (w / 2)
            cy = y + (h / 2)
            area = w * h
            contourarea = cv2.contourArea(contour)

            extent = float(contourarea) / area
            aspect = float(w) / h

            cont_squareness = (1.0 / aspect if aspect > 1 else aspect)
            cont_squareness *= (1.0 / extent if extent > 1.0 else extent)
            if (cont_squareness > squareness):
                if (debug):
                    print("New squarest: %f" % squareness)
                    print(" extent " + str(extent))
                    print(" aspect = " + str(aspect))
                    print(" area = " + str(contourarea))
                squareness = cont_squareness

        if area > 0:
            ball = [cx, cy, area]
        else:
            ball = None
        # Set drives or report ball status
        self.SetSpeedFromBall(ball)

    def SetSpeedFromBall(self, ball):
        """ Make decisions about what we're doing and
            set the motor speed from the ball position """

        # Tuning constants
        backoff = -0.4  # how fast to back out of the corner
        seek = 1.0  # how fast to turn when we can't see a ball
        # how fast we may turn a wheel backwards when a ball is in sight
        hunt_reverse = -0.2

        driveLeft = 0.0
        driveRight = 0.0
        if ball:
            x = ball[0]
            # y = ball[1]
            area = ball[2]

            # If we're learning, just seeing a colour is enough
            if self.state == State.LEARNING:
                # We've seen a ball of a colour - is it what we want?
                if not (self.colour in self.arenacolours):
                    # It's a new colour
                    self.arenacolours.append(self.colour)
                    self.lookingatcolour = self.colour
                    # Have we found all four colours?
                    if len(self.arenacolours) == 4:
                        print('Lets remember these for next time')
                        f = open('arenacolours.txt', 'w')
                        f.write("{0}\n{1}\n{2}\n{3}".format(
                            *self.arenacolours))
                        f.close()
                        print(
                            ('I found all the colours, now '
                             'm looking at {0} hunting a {1}'
                             ).format(
                                self.lookingatcolour,
                                self.challengecolours[0]
                            )
                        )
                        self.colourindex = 0
                        self.colour = self.challengecolours[0]
                        self.state = State.HUNTING
                        # time.sleep(2)
            elif self.state == State.ORIENTING and self.lookingatcolour == '':
                # If we can see a colour, set lookingatcolour and go hunting
                print(('Im looking at a {0} ball,'
                      ' lets hunt a {1} one').format(
                    self.colour,
                    self.challengecolours[0])
                )
                self.lookingatcolour = self.colour
                self.colourindex = 0
                self.colour = self.challengecolours[0]
                self.state = State.HUNTING
                # time.sleep(2)
            elif self.state == State.HUNTING:
                if area < self.autoMinArea:
                    print('Too small / far')
                    driveLeft = self.autoMinPower
                    driveRight = self.autoMinPower
                elif area > self.autoMaxArea:
                    print('Close enough')

                    # Remember we're looking at the current colour
                    self.lookingatcolour = self.colour
                    self.colourindex = self.colourindex + 1
                    if (self.colourindex >= len(self.challengecolours)):
                        print('Donezo!')
                        self.state = State.FINISHED
                    else:
                        self.colour = self.challengecolours[self.colourindex]
                        print('Now looking for %s ball' % (self.colour))
                        driveLeft = backoff
                        driveRight = backoff
                else:
                    if area < self.autoFullSpeedArea:
                        speed = 1.0
                    else:
                        speed = 1.0 / (area / self.autoFullSpeedArea)
                    speed *= self.autoMaxPower - self.autoMinPower
                    speed += self.autoMinPower
                    direction = (self.imageCentreX - x) / self.imageCentreX
                    direction = direction * 5
                    if direction > 0.0:
                        # Turn right
                        print('Turn right for %s' % self.colour)
                        driveLeft = speed
                        driveRight = speed * (1.0 - direction)
                        if driveRight < hunt_reverse:
                            driveRight = hunt_reverse
                    else:
                        # Turn left
                        print('Turn left for %s' % self.colour)
                        driveLeft = speed * (1.0 + direction)
                        driveRight = speed
                        if driveLeft < hunt_reverse:
                            driveLeft = hunt_reverse
        else:
            # Figure out which direction to seek from arenacolours
            if (self.state == State.HUNTING and
                (self.arenacolours.index(self.colour) == self.arenacolours.index(self.lookingatcolour) - 1 or
                self.arenacolours.index(self.colour) == self.arenacolours.index(self.lookingatcolour) + 3)):
                # colour we want is left of looking-at-colour, turn leftwards
                print('No {0} ball, {0} is left of {1}, turn left'.format(
                    self.colour,
                    self.lookingatcolour)
                )
                driveLeft = 0 - seek
                driveRight = seek
            else:
                print('No {0} ball, turn right'.format(self.colour))
                # turn right like we normally do
                driveLeft = seek
                driveRight = 0 - seek

        if self.tickInt == 0:
            asciiTick = "|   "
        elif self.tickInt == 1:
            asciiTick = " |  "
            if (driveLeft != backoff):
                driveLeft = 0
                driveRight = 0
        elif self.tickInt == 2:
            asciiTick = "  | "
        else:
            asciiTick = "   |"
            if (driveLeft != backoff):
                driveLeft = 0
                driveRight = 0

        self.tickInt = self.tickInt + 1 if self.tickInt < 3 else 0
        # If we're figuring out what colour we're looking at, cycle through
        #  colours on each tick; otherwise focus on the coloru we're hunting
        if self.state == State.LEARNING or self.state == State.ORIENTING:
            self.colour = self.challengecolours[self.tickInt]
        print('{0} ({1}) {2:4.1f}, {3:4.1f} - {4}, x {5}, {6} > {7}'.format(
            self.state,
            asciiTick,
            driveLeft,
            driveRight,
            self.arenacolours,
            ball[0] if ball else 0,
            self.lookingatcolour,
            self.colour)
        )
        self.core_module.throttle(driveLeft * 100, driveRight * 100)
        if (driveLeft == backoff):
            time.sleep(0.8)
