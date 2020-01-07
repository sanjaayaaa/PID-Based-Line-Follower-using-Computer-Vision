from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
from smbus import SMBus


addr = 0x8
bus = SMBus(1)


camera = PiCamera()
camera.resolution = (320, 208)
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(320, 208))
time.sleep(0.1)

x_last = 160
y_last = 104



p=0
I=0
D=0
kp =0.35
kd =0.11
ki=0.00003
lastError = 0





for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):	
	image = frame.array
	Blackline = cv2.inRange(image, (0,0,0), (75,75,75))	
	kernel = np.ones((3,3), np.uint8)
	Blackline = cv2.erode(Blackline, kernel, iterations=5)
	Blackline = cv2.dilate(Blackline, kernel, iterations=9)	
	img_blk,contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	contours_blk_len = len(contours_blk)
	if contours_blk_len > 0 :
	 if contours_blk_len == 1 :
	  blackbox = cv2.minAreaRect(contours_blk[0])
	 else:
	   canditates=[]
	   off_bottom = 0
	   for con_num in range(contours_blk_len):
		blackbox = cv2.minAreaRect(contours_blk[con_num])
		(x_min, y_min), (w_min, h_min), ang = blackbox		
		box = cv2.boxPoints(blackbox)
		(x_box,y_box) = box[0]
		if y_box > 206 :		 
		 off_bottom += 1
		canditates.append((y_box,con_num,x_min,y_min))		
	   canditates = sorted(canditates)
	   if off_bottom > 1:	    
		canditates_off_bottom=[]
		for con_num in range ((contours_blk_len - off_bottom), contours_blk_len):
		   (y_highest,con_highest,x_min, y_min) = canditates[con_num]		
		   total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5
		   canditates_off_bottom.append((total_distance,con_highest))
		canditates_off_bottom = sorted(canditates_off_bottom)         
		(total_distance,con_highest) = canditates_off_bottom[0]         
		blackbox = cv2.minAreaRect(contours_blk[con_highest])	   
	   else:		
		(y_highest,con_highest,x_min, y_min) = canditates[contours_blk_len-1]		
		blackbox = cv2.minAreaRect(contours_blk[con_highest])	 
	 (x_min, y_min), (w_min, h_min), ang = blackbox
	 x_last = x_min
	 y_last = y_min
	 setpoint = 160
 
	 error = int(x_min - setpoint)

	 P=error
	 I=error+lastError
	 D=error-lastError
	 second = int (kp*P+kd*D+I*ki)
	 lastError = error

	 bus.write_byte(addr, second) 
	 print (second)
	 box = cv2.boxPoints(blackbox)
	 box = np.int0(box)
	 cv2.drawContours(image,[box],0,(0,0,255),3)	 

	rawCapture.truncate(0)






