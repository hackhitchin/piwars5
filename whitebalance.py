# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
# camera.hflip = True
# camera.vflip = True

camera.awb_mode = 'off'
# Start off with low gains
rg, bg = (1, 1.8)
camera.awb_gains = (rg, bg)

rawCapture = PiRGBArray(camera)
 
# allow the camera to warmup
time.sleep(0.1)
 
# grab an image from the camera
camera.capture(rawCapture, format="bgr")

# Show the full unbalanced image
image = rawCapture.array
# cv2.imshow("Initial", image)


rawCapture.seek(0)
rawCapture.truncate()

for i in range(10):
   camera.capture(rawCapture, format="bgr")
   # get the centre section, average RGB channels
   imagecentre = rawCapture.array[190:290, 270:370]
   
   #if i == 9:
   #   cv2.imshow("Image" + str(i), imagecentre)
   b, g, r = (np.mean(imagecentre[..., i]) for i in range(3))
   print('R:%5.2f, B:%5.2f = (%5.2f, %5.2f, %5.2f)' % (rg, bg, r, g, b))

   if abs(r - g) > 20:
      if r > g:
         rg -= 0.1
      else:
         rg += 0.1
   if abs(b - g) > 20:
      if b > g:
         bg -= 0.1
      else:
         bg += 0.1
         
   if abs(r - g) > 2:
      if r > g:
         rg -= 0.1
      else:
         rg += 0.1
   if abs(b - g) > 1:
      if b > g:
         bg -= 0.1
      else:
         bg += 0.1
   camera.awb_gains = (rg, bg)
   rawCapture.seek(0)
   rawCapture.truncate()




# display the image on screen and wait for a keypress
camera.capture(rawCapture, format="bgr")
final_image = rawCapture.array
#cv2.imshow("Final", final_image)
#cv2.waitKey(0)

f = open('rbgains.txt','w')
f.write("R:%5.2f\nB:%5.2f\n" % (rg, bg))
f.close()
