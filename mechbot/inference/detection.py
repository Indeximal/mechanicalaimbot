import numpy as np
import tensorflow as tf
from PIL import Image


class ObjectDetector(object):
    def __init__(self, frozen_graph_path, required_format):
        self.required_format = required_format

        self.graph = tf.Graph()
        # Load graph from drive
        with self.graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(frozen_graph_path, "rb") as file:
                serialized_graph = file.read()
                graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(graph_def, name="")

        # Get handles to input and output tensors
        ops = self.graph.get_operations()
        self.tensor_dict = {}
        for key in ["num_detections", "detection_boxes", "detection_scores",
                    "detection_classes"]:
            name = key + ":0"
            self.tensor_dict[key] = self.graph.get_tensor_by_name(name)

        self.image_tensor = self.graph.get_tensor_by_name("image_tensor:0")

    def run(self, images_data, image_format):
        """Runs object detection on a list of images.

        Args:
            images_data: An uint8 numpy array of shape [N, None, None, Q]
                representing the N input images.
            image_format: A string identifier for the binary image format.
                len(image_format) should be Q. Used for automatic conversion.

        Returns:
            boxes: A float numpy array of shape [N, C, 4] representing each box
                as [y_min, x_min, y_max, x_max] with floats between 0 and 1.
            scores: A float numpy array of shape [N, C] representing a
                confidence score from 0 (uncertain) to 1 (very certain).
            classes: A int numpy array of shape [N, C] representing the class
                ids.
        """
        if image_format != self.required_format:
            images_data = self.convert_image_data(images_data, image_format)

        input_dict = {self.image_tensor: images_data}
        output_dict = self.session.run(self.tensor_dict,
                                       feed_dict=input_dict)

        boxes = output_dict["detection_boxes"]
        scores = output_dict["detection_scores"]
        classes = output_dict["detection_classes"].astype(int)

        print(scores[:, :4])

        return boxes, scores, classes

    def run_single(self, image, im_format):
        box, score, classes = self.run(np.expand_dims(image, axis=0), im_format)
        return box[0], score[0], classes[0]

    def convert_image_data_broken(self, images, from_format):
        # TODO: Find best performance
        num, im_width, im_height, channels = images.shape
        size = (im_width, im_height)

        converted = [Image.frombytes(self.required_format, size, image, "raw",
                        from_format).tobytes() for image in images]
        data = np.stack(converted)
        return np.frombuffer(data, dtype=np.uint8).reshape((num, im_height,
                                                            im_width, -1))

    def load_and_run_images(self, paths):
        # TODO: Check performance
        images = np.array([np.array(Image.open(path)) for path in paths])
        return self.run(images, "RGB")

    def load_and_run_image(self, img_path):
        box, score, classes = self.load_and_run_images([img_path])
        return box[0], score[0], classes[0]

    def __enter__(self):
        self.session = tf.Session(graph=self.graph)
        return self

    def __exit__(self, type, value, traceback):
        self.session.close()
