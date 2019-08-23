import argparse
import glob
import os

import numpy as np
from object_detection.utils import metrics
from object_detection.utils import per_image_evaluation

from mechbot.inference.detection import ObjectDetector


class_names = ["head", "body"]

parser = argparse.ArgumentParser(description="Evaluates frames.")
parser.add_argument("frame_folder")
parser.add_argument("frozen_graph")
parser.add_argument("--max_frames", type=int, default=None)

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
        # Heads
        for box in label[0] + label[2]:
            boxes.append(box)
            classes.append(1)  # id for head
        # Bodies
        for box in label[1] + label[3]:
            boxes.append(box)
            classes.append(2)  # id for body

        ground_truth_boxes_list.append(np.array(boxes))
        ground_truth_classes_list.append(np.array(classes))

# print(ground_truth_boxes_list, ground_truth_classes_list)

predicted_boxes = []
predicted_classes = []
conficence_scores = []

# Run inference on the frames
batch_size = 8
with ObjectDetector(args.frozen_graph) as detector:
    for i in range(0, len(frames), batch_size):
        batch = frames[i: i + batch_size]
        boxes, scores, classes = detector.load_and_run_images(batch)
        predicted_boxes.extend(boxes)
        # predicted_classes.extend(np.expand_dims(classes, 2))
        # conficence_scores.extend(np.expand_dims(scores, 2))
        predicted_classes.extend(classes)
        conficence_scores.extend(scores)

# print("f:", frames)
# print("gt b:", ground_truth_boxes_list, "gt c:", ground_truth_classes_list)
# print("b:", predicted_boxes, "c:", predicted_classes, "s", conficence_scores)

evaluator = per_image_evaluation.PerImageEvaluation(2)  # 2 classes

for i in range(len(frames)):
    results = evaluator.compute_object_detection_metrics(
        predicted_boxes[i],
        conficence_scores[i],
        predicted_classes[i],
        ground_truth_boxes_list[i],
        ground_truth_classes_list[i],
        np.zeros(len(ground_truth_classes_list[i]), dtype=bool),
        np.zeros(len(ground_truth_classes_list[i]), dtype=bool)
    )
    scores, tp_fp_labels, _ = results

    for class_id, score, labels in zip(range(1, len(class_names) + 1), scores, tp_fp_labels):
        num_gt = sum([1 for c in ground_truth_classes_list[i] if c == class_id])
        print(class_id, metrics.compute_precision_recall(score, labels, num_gt))
