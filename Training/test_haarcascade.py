import argparse
import json
import os

import cv2
import numpy as np


# An algorithm to compute the path of each point from a start and an end set 
# of points. Used to interpolate the coordinates.
def compute_paths(start, end, max_dist):
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


parser = argparse.ArgumentParser(description="Test haarcascade.")
parser.add_argument("xml_file")
parser.add_argument("json_file")
parser.add_argument("--path_prefix", default="")
parser.add_argument("--scale_factor", type=float, default=1.1)
parser.add_argument("--min_neighbors", type=int, default=3)
parser.add_argument("--max_dist", type=float, default=50)
parser.add_argument("--show_output", action="store_true")

args = parser.parse_args()

haar_cascade = cv2.CascadeClassifier(args.xml_file)


with open(args.json_file, "r") as file:
    ground_truth_dict = json.load(file)

total_hits = 0
total_misses = 0
total_false_pos = 0
total_frames = 0

for file_name in ground_truth_dict:
    file_path = os.path.join(args.path_prefix, file_name)

    frame = cv2.imread(file_path)

    if frame is None:
        continue

    ground_truth = ground_truth_dict[file_name]

    detected_rects = haar_cascade.detectMultiScale(frame, 
        args.scale_factor, args.min_neighbors)
    prediction = [(int(x + w / 2), int(y + h / 2), int((w + h) / 4))
        for x, y, w, h in detected_rects]
    # Compare the detected heads and the ones from the video
    paths = compute_paths(prediction, ground_truth, args.max_dist)

    # Cateorize based on detection
    hits = [detec for detec, contr in paths if detec is not None and \
            contr is not None]
    misses = [contr for detec, contr in paths if detec is None]
    false_pos = [detec for detec, contr in paths if contr is None]

    total_hits += len(hits)
    total_misses += len(misses)
    total_false_pos += len(false_pos)
    total_frames += 1


    if args.show_output:
        print(file_name, end="\r")
        
        for x, y, r in ground_truth:
            cv2.rectangle(frame, (x-r, y-r), (x+r, y+r), (255, 0, 0), 2)
        for x, y, r in hits:
            cv2.rectangle(frame, (x-r, y-r), (x+r, y+r), (0, 255, 0), 2)
        for x, y, r in false_pos:
            cv2.rectangle(frame, (x-r, y-r), (x+r, y+r), (0, 0, 0), 2)

        cv2.imshow('Frame', frame)
        key_pressed = cv2.waitKey(0)

        if key_pressed == ord("q"):
            break

total_features = total_hits + total_misses
false_pos_per_frame = total_false_pos / total_frames
print("Total Frames:", total_frames)
print("Total heads:", total_features)
print("Hits: {} ({:.1f}%)".format(total_hits, total_hits / total_features * 100))
print("Mispredictions: {} ({:.1f} per frame)".format(total_false_pos, false_pos_per_frame))

