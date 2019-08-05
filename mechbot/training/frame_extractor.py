import argparse
import os

import cv2

from box_extractor import annotated_frames, ColoredBoxExtractor


parser = argparse.ArgumentParser(description="Extracts frames from a video.")
parser.add_argument("video_path")
parser.add_argument("output_folder")

args = parser.parse_args()

# Colors used by Blink Boxes Script
extractors = [
    ColoredBoxExtractor((200, 0, 200), (255, 50, 255), 210),  # T Head
    ColoredBoxExtractor((200, 200, 0), (255, 255, 50), 210),  # T Body
    ColoredBoxExtractor((0, 200, 200), (50, 255, 255), 500),  # CT Head
    ColoredBoxExtractor((0, 200, 0), (50, 255, 50), 500)  # CT Body
]

counter = 0
for frame, boxes_list, info in annotated_frames(args.video_path, extractors):
    frame_id = "{:05d}".format(counter)
    output_path = os.path.join(args.output_folder, frame_id)

    width, height, _ = frame.shape

    rel_y0x0y1x1_boxes = [[(y / height, x / height, (y + h) / height,
        (x + w) / height) for x, y, w, h in boxes] for boxes in boxes_list]

    cv2.imwrite(output_path + ".png", frame)
    with open(output_path + ".txt", "w") as file:
        file.write(repr(rel_y0x0y1x1_boxes))

    print("\r{:>3.0f}%".format(info.get_progress() * 100), end="")

    counter += 1

print()  # New line
