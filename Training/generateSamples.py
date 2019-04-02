import numpy as np
import cv2
import sys, os
import random

# Settings
pinkMargin = 30
contourMinArea = 50
samplePaddingMultiplier = 2.0
maxBlobInertia = 0.31
sampleSize = 150

# Input and Output
videoIn = sys.argv[1]
sampleOut = os.path.join(sys.argv[2], "p{:05}.png")
negativeSampleOut = os.path.join(sys.argv[3], "p{:05}.png")

# Range of pink
boxColorUpper = np.array([255,  pinkMargin, 255], np.uint8)
boxColorLower = np.array([255 - pinkMargin, 0, 250 - pinkMargin], np.uint8)

cap = cv2.VideoCapture(sys.argv[1])

# get width and height of frames
frameWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
frameHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

# This is the last frame if it is doesn't contain boxes otherwise this is None.
# Samples are croped from this frame.
lastCleanFrameOrNone = None

sampleCounter = 0
ret, frame = cap.read()

while ret:
    # Generates a binary image based on if the pixels are pink
    pinkMask = cv2.inRange(frame, boxColorLower, boxColorUpper)

    # Finds blobs bigger than contourMinArea in the mask
    _, contours, _ = cv2.findContours(pinkMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    bigContours = [c for c in contours if cv2.contourArea(c) > contourMinArea]

    # If this frame doesn't contain any pink, it is saved for possible later use.
    if len(bigContours) == 0:
        lastCleanFrameOrNone = frame
    # If this is a pink frame after non-pink ones, samples are croped.
    elif lastCleanFrameOrNone is not None:
        for c in bigContours:
            top, left, w, h = cv2.boundingRect(c)
            x = top + w // 2
            y = left + h // 2
            padding = int((w + h) / 2 * samplePaddingMultiplier)

            # Calculate the how squareish the blob is
            inertia = abs((h - w) / w)
            if inertia > maxBlobInertia: 
                continue

            # Print Debug Information
            # print("Sample", sampleCounter, "({}, {})".format(w, h), "{:.2f}".format(inertia), cv2.contourArea(c))

            # Crop a sample from the last frame and resize it
            sample = lastCleanFrameOrNone[y - padding : y + padding, x - padding : x + padding]
            sample = cv2.resize(sample, (sampleSize, sampleSize))

            # Get random coordinates in pic
            x = random.randint(int(padding)+1, int(frameWidth-padding) )
            y = random.randint(int(padding)+1, int(frameHeight-padding) )
            # Crop a negative sample from the last frame and resize it
            negativeSample = lastCleanFrameOrNone[y - padding : y + padding, x - padding : x + padding]
            negativeSample = cv2.resize(negativeSample, (sampleSize, sampleSize))

            # Debug
            # cv2.imshow("Sample", sample)
            # samplePink = cv2.resize(frame[y - padding : y + padding, x - padding : x + padding], (sampleSize, sampleSize))
            # cv2.imshow("Pink", samplePink)
            # if cv2.waitKey(0) & 0xFF == ord('q'):
            #     exit()

            cv2.imwrite(sampleOut.format(sampleCounter), sample)
            cv2.imwrite(negativeSampleOut.format(sampleCounter), negativeSample)

            sampleCounter += 1

        # The last frame (this) is no longer a clean one.
        lastCleanFrameOrNone = None

    ret, frame = cap.read()

print("Generated", sampleCounter, "positive samples.")
