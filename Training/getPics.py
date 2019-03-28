import numpy as np
import cv2


# no negative samples
#
# !ssize.empty() in function 'cv::resize'
#





#set value to not zero
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
	#get first frame
	
	#check if 1 video with pink blob other no blob
	while consistColor != 0 or len(contours) == 0:
		frame0 = frame
		_, frame = cap.read()
		#1 frame read
		i+=1

		#detect color in frame0
		contours, _ = cv2.findContours(cv2.inRange(frame0,boxColorLower,boxColorUpper), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		consistColor = len(contours)
		#detect color in frame
		contours, _ = cv2.findContours(cv2.inRange(frame, boxColorLower, boxColorUpper), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		#print(contours)
	

		
	blob = max(contours, key=lambda el: cv2.contourArea(el))
	M = cv2.moments(blob)
		
	#division by 0
	if M["m00"] != 0:

		#size of head picture
		pixels = (M["m00"])*3
		#x,y coordinates
		center = (M["m10"] / M["m00"], M["m01"] / M["m00"])
			
		#crop image
		frame0 = frame0[int(center[1]-(pixels/2)):int(center[1]+(pixels/2)),int(center[0]-(pixels/2)):int(center[0]+(pixels/2)) ,:]
		#check if frame0 isnt 0
		if len(frame0) > 0 :
			#scale image
			frame0 = cv2.resize(frame0, dsize=(50,50))
			#store frame
			cv2.imwrite("frame%d.jpeg" %i, frame0)

	#activate while loop
	consistColor = 3

cap.release()
cv2.destroyAllWindows()