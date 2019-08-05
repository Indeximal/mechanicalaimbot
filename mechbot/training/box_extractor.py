import cv2
import numpy as np


class ColoredBoxExtractor():
    def __init__(self, lower_color, upper_color, min_area):
        self.lower_color = np.array(lower_color, dtype=np.uint8)
        self.upper_color = np.array(upper_color, dtype=np.uint8)
        self.min_area = min_area

    def extract(self, frame):
        """Extracts colored regions in an image.

        Returns:
            rects: a list of 4-tuples representing [x, y, width, height] in
                absolute pixels.
        """
        # Finds color matching regions
        mask = cv2.inRange(frame, self.lower_color, self.upper_color)
        # Finds blobs in the mask
        # subindex allows for the different return types of OpenCV 3 and 4
        contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]
        # Returns bounding boxes of big enough blobs
        rects = [cv2.boundingRect(c) for c in contours 
            if cv2.contourArea(c) > self.min_area]
        return rects


class annotated_frames:
    """This class iterates over the frames of a video and extracts boxes first"""
    def __init__(self, video_path, extractors):
        self.video_capture = cv2.VideoCapture(video_path)
        self.extractors = extractors

    def __iter__(self):
        return self

    def __next__(self):
        last_clean_frame = None

        while True:
            ret, frame = self.video_capture.read()
            if not ret:
                raise StopIteration()

            boxes_list = [extractor.extract(frame) for extractor in self.extractors]
            total = sum([len(boxes) for boxes in boxes_list])

            # If this frame doesn't contain any matches, it is saved for possible later use.
            if total == 0:
                last_clean_frame = frame

            # If this is a dirty frame after non-dirty ones, it is returned.
            elif last_clean_frame is not None:
                self.keyframe = frame
                return last_clean_frame, boxes_list, self

    def get_progress(self):
        curr_frame = self.video_capture.get(cv2.CAP_PROP_POS_FRAMES)
        video_length = self.video_capture.get(cv2.CAP_PROP_FRAME_COUNT)
        return curr_frame / video_length
