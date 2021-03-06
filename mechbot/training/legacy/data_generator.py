import argparse
import json
import sys
import os

import numpy as np
import cv2


# An algorithm to compute the path of each point from a start and an end set 
# of points. Used to interpolate the coordinates.
def computePaths(start, end, max_dist):
    points1 = np.array(start)
    points2 = np.array(end)

    if points1.size == 0:
        return [(None, p) for p in points2]
    if points2.size == 0:
        return [(p, None) for p in points1]

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


def boundingSquare(midX, midY, halfSize):
    return (
        int(y - padding),
        int(y + padding),
        int(x - padding),
        int(x + padding)
    )

# Input and Output
parser = argparse.ArgumentParser(
    description="Generates haarcascade training samples from different inputs.",
    epilog="Other more technical options are defined in the script.")
parser.add_argument("video", help="input video path")
parser.add_argument("-v", "--show_progress", action="store_true", dest="progress")
parser.add_argument("-p", "--positives_path", dest="positives_path",
    help="output folder for the positives", metavar="PATH")
parser.add_argument( "-n", "--negatives_path", dest="negatives_path",
    help="output folder for the negatives", metavar="PATH")
parser.add_argument("--num_negatives_target", type=int, default=1000, 
    metavar="N", help="approximate number of negatives to be generated")
parser.add_argument("--sample_name_format", default="s{:05}.png", metavar="FORMAT")
parser.add_argument("--frame_name_format", default="f{:05}.png", metavar="FORMAT")
parser.add_argument("--no_interpolation", action="store_false", dest="interpolate")
parser.add_argument("--output_frames", metavar="PATH",
    help="output folder for the good frames")
parser.add_argument("--head_pos_data", metavar="PATH",
    help="output path for a json file with the head positions")
parser.add_argument("--sample_size", type=int, default=50, metavar="PIXELS")
parser.add_argument("--padding", type=float, default=2.0, metavar="FACTOR")
parser.add_argument("--cascade_file")
parser.add_argument("--cascade_scale_factor", type=float, default=1.1)
parser.add_argument("--cascade_min_neighbors", type=int, default=3)
parser.add_argument("--drop_hits", action="store_true", help="don't output "
    "correctly identified samples")
parser.add_argument("--negatives_mode", choices=["random", "false_positives"],
    default="random")

args = parser.parse_args()


# Settings
pinkMargin = 30
contourMinArea = 50
samplePaddingMultiplier = args.padding
sampleSize = args.sample_size
meanNegSampleSize = np.log(20)
stdevNegSample = np.log(1.6)
maxInterpFrameDiff = 4
maxHeadMovement = 100
maxDistForCorrect = 30


# Output options
generatePositives = args.positives_path is not None
if generatePositives:
    posSampleOut = os.path.join(args.positives_path, args.sample_name_format)
generateNegatives = args.negatives_path is not None
if generateNegatives:
    negSampleOut = os.path.join(args.negatives_path, args.sample_name_format)
outputFrames = args.output_frames is not None
if outputFrames:
    framesOut = os.path.join(args.output_frames, args.frame_name_format)
useCascade = args.cascade_file is not None
if useCascade:
    haarCascade = cv2.CascadeClassifier(args.cascade_file)

# Range of pink
boxColorUpper = np.array([255,  pinkMargin, 255], np.uint8)
boxColorLower = np.array([255 - pinkMargin, 0, 250 - pinkMargin], np.uint8)

videoCapture = cv2.VideoCapture(args.video)

# get width and height of frames
frameWidth = videoCapture.get(cv2.CAP_PROP_FRAME_WIDTH)
frameHeight = videoCapture.get(cv2.CAP_PROP_FRAME_HEIGHT)
videoLength = int(videoCapture.get(cv2.CAP_PROP_FRAME_COUNT))
avgNumNegPerFrame = args.num_negatives_target / videoLength

sampleCounter = 0
negSampleCounter = 0
frameCounter = 0
headData = {}

# This is the last frame if it is doesn't contain boxes otherwise this is None.
# Samples are croped from this frame.
lastCleanFrameOrNone = None
lastHeads = []
framesSinceLastHead = 0
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
        if generateNegatives and args.negatives_mode == "random" and \
           np.random.random() < avgNumNegPerFrame:
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

        if outputFrames:
            cv2.imwrite(framesOut.format(frameCounter), lastCleanFrameOrNone)

        # Find heads
        currentHeads = []
        for c in bigContours:
            moments = cv2.moments(c)
            x = moments['m10'] / moments['m00']
            y = moments['m01'] / moments['m00']

            _, _, w, h = cv2.boundingRect(c)
            padding = (w + h) / 2

            currentHeads.append((x, y, padding))

        # Interpolate head positon and sizes if useful.
        if args.interpolate and lastHeads and \
           framesSinceLastHead <= maxInterpFrameDiff:
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

        # Head data file
        if args.head_pos_data:
            headsArr = [[int(x), int(y), int(r)] for x, y, r in heads]
            headData[args.frame_name_format.format(frameCounter)] = headsArr

        # Haarcascade detection
        if useCascade:
            detectedRects = haarCascade.detectMultiScale(lastCleanFrameOrNone, 
                args.cascade_scale_factor, args.cascade_min_neighbors)
            detectedHeads = [(int(x + w / 2), int(y + h / 2), int((w + h) / 4))
                for x, y, w, h in detectedRects]
            # Compare the detected heads and the ones from the video
            paths = computePaths(detectedHeads, heads, maxDistForCorrect)
            # Cateorize based on detection
            hits = [contr for detec, contr in paths if detec is not None and \
                    contr is not None]
            misses = [contr for detec, contr in paths if detec is None]
            falsePos = [detec for detec, contr in paths if contr is None]

        # False-alarm negative samples
        if generateNegatives and args.negatives_mode == "false_positives":
            for x, y, padding in falsePos:
                ymin, ymax, xmin, xmax = boundingSquare(x, y, padding)

                # Check bounds
                if ymin < 0 or ymax >= frameHeight or \
                   xmin < 0 or xmax >= frameWidth:
                    continue

                # Crop a sample from the last frame and resize it
                sample = lastCleanFrameOrNone[ymin : ymax, xmin : xmax]
                sample = cv2.resize(sample, (sampleSize, sampleSize))
                
                cv2.imwrite(negSampleOut.format(negSampleCounter), sample)
                negSampleCounter += 1

        # Positives
        if generatePositives:
            headsToOutput = misses if args.drop_hits else heads

            for x, y, size in headsToOutput:
                padding = size * samplePaddingMultiplier
                ymin, ymax, xmin, xmax = boundingSquare(x, y, padding)

                # Check bounds
                if ymin < 0 or ymax >= frameHeight or \
                   xmin < 0 or xmax >= frameWidth:
                    continue

                # Crop a sample from the last frame and resize it
                sample = lastCleanFrameOrNone[ymin : ymax, xmin : xmax]
                sample = cv2.resize(sample, (sampleSize, sampleSize))
                
                cv2.imwrite(posSampleOut.format(sampleCounter), sample)
                sampleCounter += 1

        # The last frame (this) is no longer a clean one.
        lastCleanFrameOrNone = None
        lastHeads = currentHeads
        framesSinceLastHead = 0
        if outputFrames or args.head_pos_data: 
            frameCounter += 1

    ret, frame = videoCapture.read()
    framesSinceLastHead += 1

    if args.progress:
        currFrame = videoCapture.get(cv2.CAP_PROP_POS_FRAMES)
        print(int(currFrame * 100 / videoLength), "%", end="\r", sep="")

if args.head_pos_data:
    with open(args.head_pos_data, "w") as datafile:
        json.dump(headData, datafile)

print("Generated", sampleCounter, "positive samples,", negSampleCounter,
      "negative samples,", frameCounter, "good frames.")
