import os
import glob

# Absolute path of the folder this file is in.
_module_path = os.path.abspath(os.path.dirname(__file__))

# Path of the csgo inference graph file
FROZEN_DETECTION_GRAPH = os.path.join(_module_path,
                                      "ssd_mobilenet_v1_csgo_2019_05_21",
                                      "frozen_inference_graph.pb")

# Find photo paths for testing in resources/test_images
_test_image_dir = os.path.join(_module_path, "test_images")
_test_jpgs = sorted(glob.glob(os.path.join(_test_image_dir, "*.jpg")))
_test_pngs = sorted(glob.glob(os.path.join(_test_image_dir, "*.png")))
TEST_IMAGES = _test_jpgs + _test_pngs

# Find frame paths for testing in resources/test_frames
_test_frame_dir = os.path.join(_module_path, "test_frames")
TEST_FRAMES = sorted(glob.glob(os.path.join(_test_frame_dir, "*.png")))

DEFAULT_CONFIG = os.path.join(_module_path, "default_config.ini")
