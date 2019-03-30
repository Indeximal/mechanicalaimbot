import numpy as np
import cv2


# no negative samples
#
# !ssize.empty() in function 'cv::resize'

i = 0

consistColor = 2

contours = 0

#defining range of color
boxColorUpper = np.array([255,10,255],np.uint8)
boxColorLower = np.array([250,0,250],np.uint8)

#load Video file
cap = cv2.VideoCapture('Gaimer Training Mirage CT.mp4')

#number of frames
length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

_ ,frame = cap.read()


for i in range(length-1):
	
	#check if 1 video with pink blob other no blob
	while consistColor != 0 or len(contours) == 0:
		frame0 = frame
		_, frame = cap.read()

		i += 1

		#detect color in frame0
		contours, _ = cv2.findContours(cv2.inRange(frame0,boxColorLower,boxColorUpper), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		consistColor = len(contours)
		#detect color in frame
		contours, _ = cv2.findContours(cv2.inRange(frame, boxColorLower, boxColorUpper), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	

		
	blob = max(contours, key=lambda el: cv2.contourArea(el))
	M = cv2.moments(blob)
		

	#size of head
	x,y,w,_ = cv2.boundingRect(blob)
	pixels = w*4
			
	#crop image
	s = frame0[int(y-(pixels/2)):int(y+(pixels/2)),int(x-(pixels/2)):int(x+(pixels/2)) ,:]
	#scale image
	savedFrame = cv2.resize(s, dsize=(50,50))
	#store frame
	cv2.imwrite("frame%d.jpeg" %i, savedFrame)

	#activate while loop
	consistColor = 3

cap.release()
cv2.destroyAllWindows()