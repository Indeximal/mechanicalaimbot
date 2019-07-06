import os
import time

import cv2
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


color_map = [None, (181, 73, 61), (162, 43, 173)] + [(0, 0, 0)] * 100

# Load graph from drive
PATH_TO_FROZEN_GRAPH = os.path.join('ssd_mobilenet_v1_csgo_2019_05_21',
    'frozen_inference_graph.pb')
graph_def = tf.GraphDef()
with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
    serialized_graph = fid.read()
    graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(graph_def, name='')

# Detection Loop
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
            # Grab screenshot and convert it
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

            # Detections with a confidence score above 50%
            valid_detections = [(score, box, classID) for score, box, classID 
                                in zip(scores, boxes, classes) if score > .5]

            # Display the detection in a window
            image = np.array(screen_shot)
            im_w, im_h = screen_shot.size
            for _, box, classID in valid_detections:
                y1, x1, y2, x2 = box
                p1 = (int(x1 * im_w), int(y1 * im_h))
                p2 = (int(x2 * im_w), int(y2 * im_h))
                cv2.rectangle(image, p1, p2, color_map[classID], 3)
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