import argparse
import glob
import os

import numpy as np
from object_detection.utils import metrics
from object_detection.utils import per_image_evaluation

from mechbot.inference.detection import ObjectDetector


class_names = ["unused", "T_head", "T_body", "CT_head", "CT_body"]

parser = argparse.ArgumentParser(description="Evaluates frames.")
parser.add_argument("frame_folder")
parser.add_argument("frozen_graph")
parser.add_argument("--max_frames", type=int, default=None)
parser.add_argument("--batch_size", type=int, default=8)

args = parser.parse_args()

frames = glob.glob(os.path.join(args.frame_folder, "*.png"))
frames = frames[:args.max_frames]  # Only process N frames
label_files = [frame_name[:-4] + ".txt" for frame_name in frames]

# Parse label files
ground_truth_boxes_list = []
ground_truth_classes_list = []
for filename in label_files:
    with open(filename, "r") as file:
        content = file.read()
        if content.upper() != content.lower() or "=" in content:
            raise Exception("The label file contains letters, which might pose"
                            " a security risk!")

        label = eval(content)
        assert len(label) == 4

        boxes = []
        classes = []

        # add boxes and their class ids to the lists
        for i in range(4):
            boxes.extend(label[i])
            classes.extend([i + 1] * len(label[i]))

        # # Heads
        # for box in label[1] + label[3]:
        #     y0_, x0, y1_, x1 = box
        #     y0 = y0_ * 1920 / 1080  # fix for typo in frame_extractor
        #     y1 = y1_ * 1920 / 1080  # fix for typo in frame_extractor
        #     boxes.append((y0, x0, y1, x1))
        #     classes.append(1)  # id for head, maybe
        # # Bodies
        # for box in label[0] + label[2]:
        #     y0_, x0, y1_, x1 = box
        #     y0 = y0_ * 1920 / 1080  # fix for typo in frame_extractor
        #     y1 = y1_ * 1920 / 1080  # fix for typo in frame_extractor
        #     boxes.append((y0, x0, y1, x1))
        #     classes.append(2)  # id for body

        ground_truth_boxes_list.append(np.array(boxes))
        ground_truth_classes_list.append(np.array(classes))

# print(ground_truth_boxes_list, ground_truth_classes_list)

predicted_boxes = []
predicted_classes = []
confidence_scores = []

# Run inference on the frames
batch_size = args.batch_size
with ObjectDetector(args.frozen_graph) as detector:
    for i in range(0, len(frames), batch_size):
        batch = frames[i: i + batch_size]
        boxes, scores, classes = detector.load_and_run_images(batch)
        predicted_boxes.extend(boxes)
        # predicted_classes.extend(np.expand_dims(classes, 2))
        # confidence_scores.extend(np.expand_dims(scores, 2))
        predicted_classes.extend(classes)
        confidence_scores.extend(scores)

# print("f:", frames)
# print("gt b:", ground_truth_boxes_list, "gt c:", ground_truth_classes_list)
# print("b:", predicted_boxes, "c:", predicted_classes, "s", confidence_scores)

evaluator = per_image_evaluation.PerImageEvaluation(5,   # 4 classes + unused
                                                    matching_iou_threshold=0.4,
                                                    nms_iou_threshold=0.2)

for i in range(len(frames)):
    results = evaluator.compute_object_detection_metrics(
        predicted_boxes[i],
        confidence_scores[i],
        predicted_classes[i],
        ground_truth_boxes_list[i],
        ground_truth_classes_list[i],
        np.zeros(len(ground_truth_classes_list[i]), dtype=bool),
        np.zeros(len(ground_truth_classes_list[i]), dtype=bool)
    )
    scores, tp_fp_labels, _ = results

    mAP_list = []
    for c_id, score, labels in zip([1, 2, 3, 4], scores[1:], tp_fp_labels[1:]):
        num_gt = sum(ground_truth_classes_list[i] == c_id)
        pres, recall = metrics.compute_precision_recall(score, labels, num_gt)
        ap = metrics.compute_average_precision(pres, recall)
        mAP_list.append(ap)

    clean_mAP = [ap for ap in mAP_list if not np.isnan(ap)]
    mAP = sum(clean_mAP) / len(clean_mAP)
    print(frames[i][-9:], "{:2f}".format(mAP), mAP_list)
