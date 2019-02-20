import cv2
import numpy as np
import time
import picamera
import picamera.array
from picamera.array import PiRGBArray
import sys

lower_bounds = [np.array([160,100,50]),
      np.array([15,100,50]),
      np.array([60,100,50]),
      np.array([90,150,50])]
upper_bounds = [np.array([180,255,255]),
      np.array([35,255,255]),
      np.array([80,255,255]),
      np.array([130,255,255])]
colourindex = 0

def showimage(img, title = "CV image"):
   # Boilerplate image show in window
   cv2.imshow('image', img)
   cv2.waitKey(0)
   cv2.destroyAllWindows()   
   

def overlay(img1, img2, pos):
   # img1 is the main image
   # img2 is an overlay with black background
   # pos is the (row, column) to add the overlay at
   
   # make mask from thresholding img2
   # invert for negmask
   # make blacked out backgroudn from bitwise and with negmask
   # add overlay to img with bitwise and, mask
   
   ol_rows, ol_columns, ol_depth = img2.shape
   ol_top, ol_left = pos
   
   roi = img1[ol_top:ol_top+ol_rows,ol_left:ol_left+ol_columns]
   
   overlaygray = cv2.cvtColor( img2, cv2.COLOR_BGR2GRAY )
   ret, overlaymask = cv2.threshold( overlaygray, 10, 255, cv2.THRESH_BINARY )
   overlaymask_inv = cv2.bitwise_not( overlaymask )
   
   roi_bg = cv2.bitwise_and( roi, roi, mask = overlaymask_inv )
   
   roi_fg = cv2.bitwise_and( img2, img2, mask = overlaymask )
   
   combined = cv2.add( roi_bg, roi_fg )
   
   img1[ol_top:ol_top+ol_rows,ol_left:ol_left+ol_columns] = combined
   
def takepicture():
   c = cv2.VideoCapture(0)
   time.sleep(2)
   c.grab()
   time.sleep(1)
   retval, img = c.retrieve()
   
   return img

def pipicture(picam):
        picam.capture('image.jpg')

def arraycapture(picam, array):
   picam.capture(array, format="bgr")

def smartthreshold(img, val):
   return cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, val, 1)
   
   
def colourmask(img, index):
   hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
   # red ball ~178, 128, 245 - 160-180,100-255,50-255
   # yellow ball 32, 145, 237 - 20-40,100-200,0-255
   # green ball 73, 211, 180 - 60-80,100-255,50-255
   # blue ball 109, 235, 182 - 80-120,150,255,50-255

   lower_bound = lower_bounds[index]
   upper_bound = upper_bounds[index]

   print lower_bound
   print upper_bound
   
   mask = cv2.inRange(hsv, lower_bound, upper_bound)
   res = cv2.bitwise_and(img,img, mask= mask)
   
   return (mask, res)
   
def erode(img):
   kernel = np.ones((5,5), np.uint8) # 5,5
   eroded = cv2.erode(img, kernel, iterations = 1)
   return eroded
    
def main():
   showme = "masked"
   if (len(sys.argv) > 1):
      colourindex = int(sys.argv[1]) 
   else:
      colourindex = 0

   print("BALL CALIBRATION v0.1")
   print("Commands:")
   print(" lh-/+: Change lower hue bound")
   print(" s-/+: Change lower saturation bound")
   print(" v-/+: Change lower value bound")
   print(" rb/yb/gb/bb: Change ball colour")
   print(" r: Show raw camera input")
   print(" m: Show masked camera input")
   print(" p: Print current bounds")
   print(" x: Exit")  

   picam = picamera.PiCamera()
   picam.resolution = (300, 240)
   picam.awb_mode = 'off'

   # picam.awb_gains = (1.2, 2.4)
   with open("rbgains.txt") as f:
       content = f.readlines()
   content = [x.strip() for x in content]
   redgain = float(content[0][2:])
   bluegain = float(content[1][2:])
   picam.awb_gains = (redgain, bluegain)   
   time.sleep(1)
   print("speed %f" % (picam.shutter_speed) )
   # picam.exposure_mode = 'off'
   picam.exposure_compensation = 0
   picam.exposure_mode = 'off'
   time.sleep(1)
   
   while(True):
      cmd = raw_input("Command? ")

      cv2.destroyAllWindows()   

      if (cmd == "x"):
         quit()
      elif (cmd == "s+"):
         lower_bounds[colourindex][1] = lower_bounds[colourindex][1] + 10
      elif (cmd == "s-"):
         lower_bounds[colourindex][1] = lower_bounds[colourindex][1] - 10
      elif (cmd == "v+"):
         lower_bounds[colourindex][2] = lower_bounds[colourindex][2] + 10
      elif (cmd == "v-"):
         lower_bounds[colourindex][2] = lower_bounds[colourindex][2] - 10
      elif (cmd == "lh+"):
         lower_bounds[colourindex][0] = lower_bounds[colourindex][0] + 5
      elif (cmd == "lh-"):
         lower_bounds[colourindex][0] = lower_bounds[colourindex][0] - 5
      elif (cmd == "uh+"):
         upper_bounds[colourindex][0] = upper_bounds[colourindex][0] + 5
      elif (cmd == "uh-"):
         upper_bounds[colourindex][0] = upper_bounds[colourindex][0] - 5
      elif (cmd == "rb"):
         colourindex = 0
      elif (cmd == "yb"):
         colourindex = 1
      elif (cmd == "gb"):
         colourindex = 2
      elif (cmd == "bb"):
         colourindex = 3
      elif (cmd == "p"):
         print lower_bounds 
         print upper_bounds
      elif (cmd == "r"):    
         showme = "raw"
      elif (cmd == "m"):    
         showme = "masked"         

      captureArray = PiRGBArray(picam)
      arraycapture(picam, captureArray)

      # img = cv2.imread("blue_ball.JPG", -1)
      # img = cv2.imread("hackspace4.jpg", -1)
           # img = cv2.imread("image.jpg", -1)
      img = captureArray.array
      
      # make a mask of ball colours according to argument
      mask, res = colourmask(img, colourindex)
      
      # clean up the mask using blurring, erosion etc.
      mask = cv2.medianBlur(mask,15)
      # mask = erode(mask)
      
      # mask the image with it, for fun
      res = cv2.bitwise_and(img, img, mask = mask)
      
      # take a deep copy as findContours messes with image we give it
      contourmask = np.copy(mask)
      
      # find the contours of the mask
      _, contours, hierarchy = cv2.findContours(contourmask, 1, 2)
      
      print len(contours)
      
      # for each contour, find centroid and blob on image
      for cnt in contours:
         # draw contour in blue
         cv2.drawContours(res,contours,-1,(255,128,0),1)
         
         # find area, ditch ones that are too small
         area = cv2.contourArea(cnt)
         
         print area
         
         if (area > 100):
            # find aspect ratio, area ratio of contour
            # if aspect ratio ~1 and area ratio ~0.75, it's round
            
            # we could use cv2.minEnclosingCircle() but that feels like cheating
            area = cv2.contourArea(cnt)
            x,y,w,h = cv2.boundingRect(cnt)
            rect_area = w*h
            
            extent = float(area)/rect_area
            aspect = float(w)/h
            print "extent = " + str(extent)
            print "aspect = " + str(aspect)
            
            # centroid!
            M = cv2.moments(cnt)
            
            centroid_x = int(M['m10']/M['m00'])
            centroid_y = int(M['m01']/M['m00'])
         
            cv2.circle(img, (centroid_x,centroid_y), 3, (0,255,0), 1);
            if (extent > 0.65 and extent < 0.9 and aspect > 0.85 and aspect < 1.15):
               cv2.rectangle(img, (x, y), (x+w, y+h), (0,255,0), 1);
            else:
               cv2.rectangle(img, (x, y), (x+w, y+h), (0,0,255), 1);
      
      cv2.startWindowThread()
      cv2.namedWindow("preview")   
      if (showme == "raw"):
         cv2.imshow('preview', img)      
      else:
         cv2.imshow('preview', res)      
      # cv2.imshow('image', img)
      # cv2.imshow('mask', mask)
      
      # cv2.waitKey(0)
   
   quit()
   
   
if __name__ == "__main__":
   main()
