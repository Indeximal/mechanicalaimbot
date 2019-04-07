import numpy as np
import cv2
import mss
import sys, os
import time
import signal

# Settings
pinkMargin = 30
contourMinArea = 50
samplePaddingMultiplier = 2.0
maxBlobInertia = 0.31
sampleSize = 50

# Input and Output
sampleOut = os.path.join(sys.argv[1], "p{:05}.png")
negativeSampleOut = os.path.join(sys.argv[2], "n{:05}.png")

# Range of pink
boxColorUpper = np.array([255, pinkMargin, 255, 255], np.uint8)
boxColorLower = np.array([255 - pinkMargin, 0, 255 - pinkMargin, 0], np.uint8)

# This is the last frame if it is doesn't contain boxes otherwise this is None.
# Samples are croped from this frame.
lastCleanFrameOrNone = None

sampleCounter = 0

def endProgram(signal, frame):
    print("\nGenerated", sampleCounter, "samples.")
    sys.exit(0)

signal.signal(signal.SIGINT, endProgram)

with mss.mss() as sct:
    # Get the dimenstions of the monitor (e.g. {"top": 40, "left": 0, "width": 800, "height": 640})
    monitor = sct.monitors[1]

    # get width and height of frames
    frameWidth = monitor["width"]
    frameHeight = monitor["height"]

    while True:
        startTime = time.time()

        # Get raw pixels from the screen, save it to a Numpy array
        frame = np.array(sct.grab(monitor))

        grabTime = (time.time() - startTime) * 1000

        # Generates a binary image based on if the pixels are pink
        pinkMask = cv2.inRange(frame, boxColorLower, boxColorUpper)

        # cv2.imshow("pink", pinkMask)

        # Finds blobs bigger than contourMinArea in the mask
        retTuple = cv2.findContours(pinkMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = retTuple[-2] # Allows for OpenCV 3 and 4
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

                # Crop a sample from the last frame and resize it
                sample = lastCleanFrameOrNone[y - padding : y + padding, x - padding : x + padding]
                sample = cv2.resize(sample, (sampleSize, sampleSize))

                # Get random coordinates in pic
                x = np.random.randint(int(padding)+1, int(frameWidth-padding) )
                y = np.random.randint(int(padding)+1, int(frameHeight-padding) )
                # Crop a negative sample from the last frame and resize it
                negativeSample = lastCleanFrameOrNone[y - padding : y + padding, x - padding : x + padding]
                negativeSample = cv2.resize(negativeSample, (sampleSize, sampleSize))

                cv2.imwrite(sampleOut.format(sampleCounter), sample)
                cv2.imwrite(negativeSampleOut.format(sampleCounter), negativeSample)

                sampleCounter += 1

            # The last frame (this) is no longer a clean one.
            lastCleanFrameOrNone = None

        print("\rGrab {:.1f}ms, Frametime: {:.1f}ms ({:.2f} fps)".format(grabTime, (time.time() - startTime) * 1000, 1 / (time.time() - startTime)), end="")
