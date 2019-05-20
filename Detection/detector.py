import os
import sys
import time

import numpy as np
import tensorflow as tf
from PIL import Image


def load_image_into_numpy_array(pil_image):
    (im_width, im_height) = pil_image.size
    return np.array(pil_image.getdata()).reshape(
            (im_height, im_width, 3)).astype(np.uint8)


def run_inference_for_single_image(image, graph):
    start_time = time.time()
    with graph.as_default():
        with tf.Session() as sess:
            print("init", time.time() - start_time)

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

            print("get tensors", time.time() - start_time)

            # Run inference
            output_dict = sess.run(tensor_dict,
                                   feed_dict={image_tensor: image})

            print("run", time.time() - start_time)

            # all outputs are float32 numpy arrays, so convert types as appropriate
            # output_dict['num_detections'] = int(output_dict['num_detections'][0])
            detection_classes = output_dict['detection_classes'][0].astype(np.int64)
            detection_boxes = output_dict['detection_boxes'][0]
            detection_scores = output_dict['detection_scores'][0]
    print("done", time.time() - start_time)
    return detection_scores, detection_boxes, detection_classes


PATH_TO_FROZEN_GRAPH = os.path.join('ssd_mobilenet_v1_coco_2017_11_17',
    'frozen_inference_graph.pb')

detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

image = Image.open(sys.argv[1])
# the array based representation of the image will be used later in order to prepare the
# result image with boxes and labels on it.
image_np = load_image_into_numpy_array(image)
# Expand dimensions since the model expects images to have shape: [1, None, None, 3]
image_np_expanded = np.expand_dims(image_np, axis=0)

# Actual detection.
scores, boxes, classes = run_inference_for_single_image(image_np_expanded, detection_graph)
# Detections with a confidence score abouve 30%
valid_detections = [(score, box, classID) for score, box, classID 
                    in zip(scores, boxes, classes) if score > .3]
print(valid_detections)
