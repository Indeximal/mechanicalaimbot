import numpy as np
import cv2


#Blob Size
#size = 5


#picture size pixels
#pixels = 70

#picture enumeration
i = 0

#defining range of color
boxColorUpper = np.array([255,10,255],np.uint8)
boxColorLower = np.array([250,0,250],np.uint8)

#load Video file
cap = cv2.VideoCapture('Gaimer Training Mirage CT.mp4')

length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

ret, frame0 = cap.read()

for i in range(length-1):
	#get first frame
	
		
	ret, frame = cap.read()
	#detect color
	mask = cv2.inRange(frame, boxColorLower, boxColorUpper)
	#check if frame0 contains blob
	contours, _ = cv2.findContours(cv2.inRange(frame0,boxColorLower,boxColorUpper),  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	consistColor =  len(contours)

	#try:# NB: using _ as the variable name for two of the outputs, as they're not used
	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

		#sizeCont = len(contours)
	
	#_,frame2 = cap.read()
	#contours2,_ = cv2.findContours(cv2.inRange(frame2,boxColorLower, boxColorUpper),cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	if(len(contours)!=0): #check if contours found
		
		blob = max(contours, key=lambda el: cv2.contourArea(el))
		M = cv2.moments(blob)
		
		pixels = int(M["m00"] * 3)
		
		#if(M['m00'] > size ) :#color detected
		#get coordinates of blob
		if M["m00"] != 0:
			
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			#get previous frame and crop it
			if consistColor == 0:
				frame0 = frame0[center[1]-int((pixels/2)):center[1]+int((pixels/2)),center[0]-int((pixels/2)):center[0]+int((pixels/2)) ,:]
			#else:
				#ret, frame0 = cap.read()
				#frame0 = frame0[center[1]-(pixels/2):center[1]+(pixels/2),center[0]-(pixels/2):center[0]+(pixels/2) ,:]
				#save new frame in folder
				frame0 = cv2.resize(frame0, dsize=(50,50))
				cv2.imwrite("frame%d.jpeg" %i, frame0)
				
	frame0 = frame
cap.release()
cv2.destroyAllWindows()