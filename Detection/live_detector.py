import os
import time

import mss
import numpy as np
import tensorflow as tf

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


def convert_screen_shot_to_tensor_format(im):
    (im_width, im_height) = im.size
    data = Image.frombytes('RGB', im.size, im.bgra, 'raw', 'BGRX').tobytes()
    # The model expects images to have shape: [1, None, None, 3]
    return np.frombuffer(data, dtype=np.uint8).reshape((1, im_height, im_width, 3))


PATH_TO_FROZEN_GRAPH = os.path.join('ssd_mobilenet_v1_coco_2017_11_17',
    'frozen_inference_graph.pb')

od_graph_def = tf.GraphDef()
with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')

# Actual detection.
start_time = time.time()
with tf.Session() as sess:

    # Get handles to input and output tensors
    ops = tf.get_default_graph().get_operations()
    all_tensor_names = {output.name for op in ops for output in op.outputs}
    tensor_dict = {}
    for key in [
        'num_detections', 'detection_boxes', 'detection_scores',
        'detection_classes'
    ]:
        tensor_name = key + ':0'
        if tensor_name in all_tensor_names:
            tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
                tensor_name)

    image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

    print("Initialize:", time.time() - start_time)

    with mss.mss() as sct:
        main_clock = Clock()
        partial_clock = Clock()
        while True:
            overhead_time = partial_clock.time_elapsed() * 1000        
            screen_shot = sct.grab(sct.monitors[1])
            grab_time = partial_clock.time_elapsed() * 1000
            image_data = convert_screen_shot_to_tensor_format(screen_shot)
            convert_time = partial_clock.time_elapsed() * 1000
            # Run inference
            output_dict = sess.run(tensor_dict,
                                   feed_dict={image_tensor: image_data})
            detect_time = partial_clock.time_elapsed() * 1000

            # all outputs are float32 numpy arrays, so convert types as appropriate
            classes = output_dict['detection_classes'][0].astype(np.int64)
            boxes = output_dict['detection_boxes'][0]
            scores = output_dict['detection_scores'][0]

            # Detections with a confidence score abouve 80%
            valid_detections = [(score, box, classID) for score, box, classID 
                                in zip(scores, boxes, classes) if score > .8]
            print(valid_detections)

            fps = 1 / main_clock.time_elapsed()

            print("{:.1f} fps ({:.0f}ms|{:.0f}ms|{:.0f}ms|{:.0f}ms)".format(
                fps, grab_time, convert_time, detect_time, overhead_time))
