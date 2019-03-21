##!/usr/bin/env python
# coding: Latin

# Load library functions we want
import time
# import os
import sys
# import ThunderBorg
# import io
import threading
import picamera
import picamera.array
import cv2
import numpy
import core
import RPi.GPIO as GPIO
from enum import Enum

print('Libraries loaded')

# Global values
global state
# global TB
global camera
global processor
global debug
global colour            # what we're looking for right now
global colourindex      # number of the colour we're looking for
global imageCentreX
global imageCentreY


class State(Enum):
     LEARNING = 1
     ORIENTING = 2
     HUNTING = 3
     FINISHED = 4

state = State.LEARNING  # What are we doing?
debug = False
challengecolours = ['red', 'blue', 'yellow', 'green'] # order to visit
arenacolours = []       # Order we've detected in the arena, clockwise
colourindex = 0
colour = challengecolours[colourindex]
lookingatcolour = ''

# Camera settings
imageWidth = 320  # Camera image width
imageHeight = 240  # Camera image height
frameRate = 30  # Camera image capture frame rate

# Auto drive settings
autoMaxPower = 1.0  # Maximum output in automatic mode
autoMinPower = 0.6  # Minimum output in automatic mode
autoMinArea = 100  # Smallest target to move towards
autoMaxArea = 10000  # Largest target to move towards
autoFullSpeedArea = 300  # Target size at which we use the maximum allowed output


# Image stream processing thread
class StreamProcessor(threading.Thread):

    def __init__(self, core_module):
        self.core_module = core_module
        super(StreamProcessor, self).__init__()
        self.stream = picamera.array.PiRGBArray(camera)
        self.event = threading.Event()
        self.terminated = False
        self.start()
        self.begin = 0

    def run(self):
        # This method runs in a separate thread
        global colour
        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                try:
                    # Read the image and do some processing on it
                    self.stream.seek(0)
                    self.ProcessImage(self.stream.array, colour)
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()

    # Image processing function
    def ProcessImage(self, image, colour):
        # View the original image seen by the camera.
        # Crop the image down to just the bit with the arena in
        image = image[100:240,0:320]

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

        if colour == "red":
            imrange = cv2.inRange(
                image,
                # numpy.array((113, 96, 64)),
                # numpy.array((125, 255, 255))
                numpy.array((0, 100, 50)),
                numpy.array((10, 255, 255))
            )
        elif colour == 'yellow':
            imrange = cv2.inRange(
                image,
                # numpy.array((15, 85, 64)),
                # numpy.array((35, 255, 255))
                numpy.array((20, 100, 75)),
                numpy.array((40, 255, 255))
            )
        elif colour == "green":
            imrange = cv2.inRange(
                image,
                # numpy.array((50, 96, 64)),
                # numpy.array((85, 255, 255))
                numpy.array((50, 100, 50)),
                numpy.array((80, 255, 255))
            )
        elif colour == 'blue':
            imrange = cv2.inRange(
                image,
                #numpy.array((0, 64, 64)),
                #numpy.array((15, 255, 255))
                numpy.array((90, 150, 64)),
                numpy.array((130, 255, 255))
            )

        # I used the following code to find
        # the approximate 'hue' of the ball in
        # front of the camera
        # for crange in range(100,114,2):
        #    imrange = cv2.inRange(image, numpy.array((crange, 64, 64)), numpy.array((crange+2, 255, 255)))
        #    print(crange)
        #    cv2.imshow('range',imrange)
        #    cv2.waitKey(0)

        # View the filtered image found by 'imrange'

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
            # cx = x + (w / 2)
            # cy = y + (h / 2)
            area = w * h
            contourarea = cv2.contourArea(contour)

            extent = float(contourarea)/area
            aspect = float(w)/h

            cont_squareness = (1.0/aspect if aspect > 1 else aspect)
            cont_squareness *= (1.0/extent if extent > 1.0 else extent)
            if (cont_squareness > squareness):
                if (debug):
                    print("New squarest: %f" % squareness)
                    print(" extent " + str(extent))
                    print(" aspect = " + str(aspect))
                    print(" area = " + str(contourarea))
                squareness = cont_squareness
                # ballsiest_index = idx

        if area > 0:
            ball = [x, y, area]
        else:
            ball = None
        # Set drives or report ball status
        self.SetSpeedFromBall(ball)

    # Make decisions about wht we're doing and
    # set the motor speed from the ball position
    def SetSpeedFromBall(self, ball):
        global TB
        global colour
        global challengecolours
        global arenacolours
        global colourindex
        global state
        global imageCentreX
        global imageCentreY
        global tickInt
        global lookingatcolour

        # Tuning constants
        backoff = -0.6 # how fast to back out of the corner
        seek = 1.0 # how fast to turn when we can't see a ball
        hunt_reverse = -0.2 # how fast we may turn a wheel backwards when a ball is in sight


        driveLeft = 0.0
        driveRight = 0.0
        if ball:
            x = ball[0]
            y = ball[1]
            area = ball[2]

            # If we're learning, just seeing a colour is enough
            if state == State.LEARNING:
                # We've seen a ball of a colour - is it what we want?
                if not (colour in arenacolours):
                    # It's a new colour
                    arenacolours.append(colour)
                    lookingatcolour = colour
                    # Have we found all four colours?
                    if len(arenacolours) == 4:
                        print('Lets remember these for next time')
                        f = open('arenacolours.txt','w')
                        f.write("{0}\n{1}\n{2}\n{3}".format(*arenacolours))
                        f.close()                        
                        print('I found all the colours, now im looking at {0} hunting a {1}'.format(lookingatcolour, challengecolours[0]))
                        colourindex = 0
                        colour = challengecolours[0]
                        state = State.HUNTING
                        time.sleep(2)
            elif state == State.ORIENTING and lookingatcolour == '':
                # If we can see a colour, set lookingatcolour and go hunting
                print('Im looking at a {0} ball, lets hunt a {1} one'.format(colour, challengecolours[0]))
                lookingatcolour = colour
                colourindex = 0
                colour = challengecolours[0]
                state = State.HUNTING
                time.sleep(2)             
            elif state == State.HUNTING:
                if area < autoMinArea:
                    print('Too small / far')
                    driveLeft = autoMinPower
                    driveRight = autoMinPower
                elif area > autoMaxArea:
                    print('Close enough')

                    # Remember we're looking at the current colour
                    lookingatcolour = colour
                    colourindex = colourindex + 1
                    if (colourindex >= len(challengecolours)):
                        print('Donezo!')
                        state = State.FINISHED
                    else:
                        colour = challengecolours[colourindex]
                        print('Now looking for %s ball' % (colour))
                        driveLeft = backoff
                        driveRight = backoff
                else:
                    if area < autoFullSpeedArea:
                        speed = 1.0
                    else:
                        speed = 1.0 / (area / autoFullSpeedArea)
                    speed *= autoMaxPower - autoMinPower
                    speed += autoMinPower
                    direction = (imageCentreX - x) / imageCentreX
                    direction = direction * 5
                    if direction > 0.0:
                        # Turn right
                        print('Turn right for %s' % colour)
                        driveLeft = speed
                        driveRight = speed * (1.0 - direction)
                        if driveRight < hunt_reverse:
                            driveRight = hunt_reverse
                    else:
                        # Turn left
                        print('Turn left for %s' % colour)
                        driveLeft = speed * (1.0 + direction)
                        driveRight = speed
                        if driveLeft < hunt_reverse:
                            driveLeft = hunt_reverse
        else:
            # Figure out which direction to seek from arenacolours
            if state == State.HUNTING and (arenacolours.index(colour) == arenacolours.index(lookingatcolour)-1 or 
                arenacolours.index(colour) == arenacolours.index(lookingatcolour)+3):
                # colour we want is left of looking-at-colour, turn leftwards
                print('No {0} ball, {0} is left of {1}, turn left'.format(colour, lookingatcolour))
                driveLeft = 0-seek
                driveRight = seek
            else:
                print('No {0} ball, turn right'.format(colour))
                # turn right like we normally do
                driveLeft = seek
                driveRight = 0-seek

        if tickInt == 0:
            asciiTick = "|   "
        elif tickInt == 1:
            asciiTick = " |  "
            if (driveLeft != backoff):
                driveLeft = 0
                driveRight = 0
        elif tickInt == 2:
            asciiTick = "  | "
        else:
            asciiTick = "   |"
            if (driveLeft != backoff):
                driveLeft = 0
                driveRight = 0

        tickInt = tickInt + 1 if tickInt < 3 else 0
        # If we're figuring out what colour we're looking at, cycle through
        #  colours on each tick; otherwise focus on the coloru we're hunting
        if state == State.LEARNING or state == State.ORIENTING:
            colour = challengecolours[tickInt]
        print('{0} ({1}) {2:4.1f}, {3:4.1f} - {4}, {5} > {6}'.format(state, asciiTick, driveLeft, driveRight, arenacolours, lookingatcolour, colour))
        self.core_module.throttle(driveLeft*100, driveRight*100)
        if (driveLeft == backoff):
            time.sleep(0.8)


# SetMotor1(driveLeft)
# SetMotor2(driveRight)

# Image capture thread
class ImageCapture(threading.Thread):
    def __init__(self):
        super(ImageCapture, self).__init__()
        self.start()

    def run(self):
        global camera
        global processor
        print('Start the stream using the video port')
        camera.capture_sequence(
            self.TriggerStream(),
            format='bgr',
            use_video_port=True
        )
        print('Terminating camera processing...')
        processor.terminated = True
        processor.join()
        print('Processing terminated.')

    # Stream delegation loop
    def TriggerStream(self):
        global state
        while state != State.FINISHED:
            if processor.event.is_set():
                time.sleep(0.01)
            else:
                yield processor.stream
                processor.event.set()


def main(core_module):

    # Startup sequence
    global camera
    global processor
    global imageCentreX
    global imageCentreY
    global state
    global tickInt
    global arenacolours

    if core_module is None:
        # Initialise GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # Instantiate CORE / Chassis module and store in the launcher.
        core_module = core.Core(GPIO)
        # Limit motor speeds in AutoMode
        core_module.speed_factor = 0.6  # 0.6 on old motors
        core_module.enable_motors(True)

    # wait for the user to enable motors
    while not core_module.motors_enabled:
        time.sleep(0.25)

    # Setup the camera
    print('Setup camera')
    camera = picamera.PiCamera()
    camera.resolution = (imageWidth, imageHeight)
    camera.framerate = frameRate
    camera.awb_mode = 'off'

    # Load the exposure calibration
    with open("rbgains.txt") as f:
        content = f.readlines()
    content = [x.strip() for x in content]
    redgain = float(content[0][2:])
    bluegain = float(content[1][2:])
    camera.awb_gains = (redgain, bluegain)
    f.close()

    # Load our previously learned arena colour order
    with open("arenacolours.txt") as f:
        content = f.readlines()
    if len(content) > 0:
        arenacolours = [x.strip() for x in content]
        state = State.ORIENTING
    f.close()        


    imageCentreX = imageWidth / 2.0
    imageCentreY = imageHeight / 2.0

    print('Setup the stream processing thread')
    processor = StreamProcessor(core_module)

    print('Wait ...')
    time.sleep(2)
    tickInt = 0
    captureThread = ImageCapture()

    try:
        print('Press CTRL+C to quit')
        # TB.MotorsOff()
        # TB.SetLedShowBattery(True)
        # Loop indefinitely until we are no longer running
        while state != State.FINISHED:
            # Wait for the interval period
            #
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("User shutdown\n")
    except:
        e = sys.exc_info()[0]
        print
        print(e)

    core_module.enable_motors(False)
    state = State.FINISHED
    captureThread.join()
    processor.terminated = True
    processor.join()
    del camera
    print("Program terminated")


if __name__ == '__main__':
    main(None)
