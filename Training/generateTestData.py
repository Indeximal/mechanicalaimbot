import numpy as np
import cv2
import sys, os
import json

# Settings
pinkMargin = 30
contourMinArea = 50
samplePaddingMultiplier = 2.0
maxBlobInertia = 0.31
sampleSize = 50

# Input and Output
videoIn = sys.argv[1]
framesOut = os.path.join(sys.argv[2], "f{:05}.png")
dataFileOut = sys.argv[3]

# Range of pink
boxColorUpper = np.array([255,  pinkMargin, 255], np.uint8)
boxColorLower = np.array([255 - pinkMargin, 0, 250 - pinkMargin], np.uint8)

cap = cv2.VideoCapture(sys.argv[1])

# This is the last frame if it is doesn't contain boxes otherwise this is None.
# Samples are croped from this frame.
lastCleanFrameOrNone = None

testData = {}

testFrameCounter = 0
ret, frame = cap.read()

while ret:
    # Generates a binary image based on if the pixels are pink
    pinkMask = cv2.inRange(frame, boxColorLower, boxColorUpper)

    # Finds blobs bigger than contourMinArea in the mask
    retTuple = cv2.findContours(pinkMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = retTuple[-2] # Allows for OpenCV 3 and 4
    bigContours = [c for c in contours if cv2.contourArea(c) > contourMinArea]

    # If this frame doesn't contain any pink, it is saved for possible later use.
    if len(bigContours) == 0:
        lastCleanFrameOrNone = frame
    # If this is a pink frame after non-pink ones, samples are croped.
    elif lastCleanFrameOrNone is not None:
        headPosArr = []

        for c in bigContours:
            top, left, w, h = cv2.boundingRect(c)
            x = top + w // 2
            y = left + h // 2
            padding = (w + h) / 2 * samplePaddingMultiplier

            # Store the positon and size
            headPosArr.append([x, y, padding*2])

        # Save the frame-headinfo pair
        testData["{:05}".format(testFrameCounter)] = headPosArr
        cv2.imwrite(framesOut.format(testFrameCounter), lastCleanFrameOrNone)

        testFrameCounter += 1

        # The last frame (this) is no longer a clean one.
        lastCleanFrameOrNone = None

    ret, frame = cap.read()

print("Generated", testFrameCounter, "test frames.")

with open(dataFileOut, "w") as datafile:
    json.dump(testData, datafile)
