import argparse
import glob
import os

from object_detection.utils import metrics
from object_detection.utils import per_image_evaluation


parser = argparse.ArgumentParser(description="Evaluates frames.")
parser.add_argument("frame_folder")
parser.add_argument("frozen_graph")

args = parser.parse_args()

# evaluator = PerImageEvaluation()

for frame_name in glob.glob(os.path.join(args.frame_folder, "*.png")):
    label_name = frame_name[:-4] + ".txt"
    print(frame_name, label_name)
