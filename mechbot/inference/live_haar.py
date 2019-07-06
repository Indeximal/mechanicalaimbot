import argparse
import time

import cv2
import mss
import numpy as np

from PIL import Image


class Clock:
    """Util for measuring time differences"""
    def __init__(self):
        self.t = time.time()

    def time_elapsed(self):
        now = time.time()
        diff = now - self.t
        self.t = now
        return diff


parser = argparse.ArgumentParser(description="Live haarcascade.")
parser.add_argument("xml_file")
parser.add_argument("--scale_factor", type=float, default=1.1)
parser.add_argument("--min_neighbors", type=int, default=3)

args = parser.parse_args()

# Detection Loop
start_time = time.time()
haar_cascade = cv2.CascadeClassifier(args.xml_file)
print("Initialize:", time.time() - start_time)

with mss.mss() as sct:
    main_clock = Clock()
    partial_clock = Clock()
    while True: 
        # Grab screenshot and convert it
        screen_shot = sct.grab(sct.monitors[1])
        grab_time = partial_clock.time_elapsed() * 1000

        image = np.array(screen_shot)
        convert_time = partial_clock.time_elapsed() * 1000

        # Run inference
        detections = haar_cascade.detectMultiScale(image, 
            args.scale_factor, args.min_neighbors)
        detect_time = partial_clock.time_elapsed() * 1000

        # Display the detection in a window
        im_w, im_h = screen_shot.size
        for x, y, w, h in detections:
            p1 = (int(x), int(y))
            p2 = (int(x + w), int(y + h))
            cv2.rectangle(image, p1, p2, (162, 43, 173), 3)
        display_image = cv2.resize(image, (960, 540))
        cv2.imshow("Detections", display_image)

        if cv2.waitKey(1) == ord("q"):
            print("quitting...")
            break

        # Basic profiling
        overhead_time = partial_clock.time_elapsed() * 1000 
        frame_time = main_clock.time_elapsed()

        print("{:.1f} fps ({:.0f}ms: {:.0f}ms|{:.0f}ms|{:.0f}ms|{:.0f}ms)".format(
            1 / frame_time, frame_time * 1000, grab_time, convert_time, 
            detect_time, overhead_time))