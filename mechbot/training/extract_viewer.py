import argparse

import cv2

import box_extractor


parser = argparse.ArgumentParser(description="Shows extractions.")
parser.add_argument("video_path")
parser.add_argument("-e", "--extractor", type=int, action="append", nargs=7,
    metavar=("R1", "G1", "B1", "R2", "G2", "B2", "MinArea"),
    help="consumes 7 itegers: R1, G1, B1 (lower), R2, G2, B2 (upper), MinArea")
parser.add_argument("--line_thinkness", type=int, default=2)
parser.add_argument("--show_keyframe", action="store_true")

args = parser.parse_args()


extractors = [box_extractor.ColoredBoxExtractor((r1, g1, b1), (r2, g2, b2), a)
    for r1, g1, b1, r2, g2, b2, a in args.extractor]

for frame, boxes_list, obj in box_extractor.annotated_frames(args.video_path, extractors):
    img = obj.keyframe if args.show_keyframe else frame
    for boxes, extractor in zip(boxes_list, extractors):
        color = [int(x) for x in extractor.upper_color]
        if args.show_keyframe:
            color = (255, 255, 255)
        for x, y, w, h in boxes:
            cv2.rectangle(img, (x, y), (x + w, y + h),
                          color, args.line_thinkness)

    cv2.imshow("Frame", img)
    key_pressed = cv2.waitKey(0)
    if key_pressed == ord("q") or key_pressed == 27:
        break
