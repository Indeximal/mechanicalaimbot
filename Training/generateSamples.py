import sys
import os

import numpy as np
import cv2


# An algorithm to compute the path of each point from a start and an end set 
# of points. Used to interpolate the coordinates.
# BUG: empty lists returns []
def computePaths(start, end, max_dist):
    points1 = np.array(start)
    points2 = np.array(end)

    # Get the squared distances of all the possible combinations of points.
    all_distances = [(sum((points1[n] - points2[m]) ** 2), n, m) 
                     for n in range(len(points1)) for m in range(len(points2))]

    paths = []

    # Go through the list, shortest distance first and remove entries involving
    # used points until the list is empty and every point has a path.
    running_list = list(sorted(all_distances, key=lambda arr: arr[0]))
    while running_list:
        dist, this_n, this_m = running_list[0]
        if dist < max_dist ** 2:
            paths.append((points1[this_n], points2[this_m]))
        else:
            paths.append((None, points2[this_m]))
            paths.append((points1[this_n], None))

        new_list = [(d, n, m) for d, n, m in running_list 
                    if n != this_n and m != this_m]
        # If all points from one list are used up, this will assign a 
        # path to the remaining points.
        if not new_list:
            for d, n, m in running_list[1:]:
                if n == this_n:
                    paths.append((None, points2[m]))
                if m == this_m:
                    paths.append((points1[n], None))

        running_list = new_list

    return paths


# Settings
pinkMargin = 30
contourMinArea = 50
samplePaddingMultiplier = 2.0
maxBlobInertia = 0.31
sampleSize = 50
avgNumNegPerFrame = 0.03
meanNegSampleSize = np.log(22)
stdevNegSample = np.log(1.6)
maxInterpFrameDiff = 4
maxHeadMovement = 100

# Input and Output
videoIn = sys.argv[1]
sampleOut = os.path.join(sys.argv[2], "p{:05}.png")
negSampleOut = os.path.join(sys.argv[3], "n{:05}.png")

# Range of pink
boxColorUpper = np.array([255,  pinkMargin, 255], np.uint8)
boxColorLower = np.array([255 - pinkMargin, 0, 250 - pinkMargin], np.uint8)

videoCapture = cv2.VideoCapture(sys.argv[1])

# get width and height of frames
frameWidth = videoCapture.get(cv2.CAP_PROP_FRAME_WIDTH)
frameHeight = videoCapture.get(cv2.CAP_PROP_FRAME_HEIGHT)

# This is the last frame if it is doesn't contain boxes otherwise this is None.
# Samples are croped from this frame.
lastCleanFrameOrNone = None

lastHeads = []
framesSinceLastHead = 0

sampleCounter = 0
negSampleCounter = 0
ret, frame = videoCapture.read()

while ret:
    # Generates a binary image based on if the pixels are pink
    pinkMask = cv2.inRange(frame, boxColorLower, boxColorUpper)

    # Finds blobs in the mask
    retTuple = cv2.findContours(pinkMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Allows for the different return types of OpenCV 3 and 4
    contours = retTuple[-2]

    bigContours = [c for c in contours if cv2.contourArea(c) > contourMinArea]

    # If this frame doesn't contain any pink, it is saved for possible later use.
    if len(bigContours) == 0:
        lastCleanFrameOrNone = frame

        # Negative sample generation on clean frames
        if np.random.random() < avgNumNegPerFrame:
            # Generate a random positive size with medium values more common.
            size = np.random.lognormal(meanNegSampleSize, stdevNegSample)
            size = int(np.ceil(size)) # Avoid zero-size by ceiling.

            x = np.random.randint(size, frameWidth - size)
            y = np.random.randint(size, frameHeight - size)

            # Crop a negative sample from the this frame and resize it
            sample = frame[y - size : y + size, x - size : x + size]
            sample = cv2.resize(sample, (sampleSize, sampleSize))

            cv2.imwrite(negSampleOut.format(negSampleCounter), sample)
            negSampleCounter += 1

    # If this is a pink frame after non-pink ones, samples are croped.
    elif lastCleanFrameOrNone is not None:
        currentHeads = []
        for c in bigContours:
            moments = cv2.moments(c)
            x = moments['m10'] / moments['m00']
            y = moments['m01'] / moments['m00']

            _, _, w, h = cv2.boundingRect(c)
            padding = (w + h) / 2 * samplePaddingMultiplier

            # Calculate the how squareish the blob is, to reject squished heads
            # or two heads fused together.
            inertia = abs((h - w) / w)
            if inertia > maxBlobInertia: 
                continue

            currentHeads.append((x, y, padding))

        # Interpolate head positon and sizes if useful.
        if lastHeads and framesSinceLastHead <= maxInterpFrameDiff:
            paths = computePaths(lastHeads, currentHeads, maxHeadMovement)
            ongoingPaths = [(np.array(start), np.array(end)) for start, end 
                            in paths if start is not None and end is not None]
            t = (framesSinceLastHead - 1) / framesSinceLastHead

            # Set interpolated head positions.
            heads = [start * (1 - t) + end * t for start, end in ongoingPaths]
            # Add heads only visible in the new frame.
            heads += [end for start, end 
                      in paths if start is None and end is not None]
        else:
            heads = currentHeads

        for x, y, padding in heads:
            ymin = int(y - padding)
            ymax = int(y + padding)
            xmin = int(x - padding)
            xmax = int(x + padding)

            # Check bounds
            if ymin < 0 or ymax >= frameHeight or \
               xmin < 0 or xmax >= frameWidth:
                continue

            # Crop a sample from the last frame and resize it
            sample = lastCleanFrameOrNone[ymin : ymax, xmin : xmax]
            sample = cv2.resize(sample, (sampleSize, sampleSize))
            
            cv2.imwrite(sampleOut.format(sampleCounter), sample)
            sampleCounter += 1

        # The last frame (this) is no longer a clean one.
        lastCleanFrameOrNone = None
        lastHeads = currentHeads
        framesSinceLastHead = 0

    ret, frame = videoCapture.read()
    framesSinceLastHead += 1

print("Generated", sampleCounter, "positive samples and", negSampleCounter,
      "negative samples.")
