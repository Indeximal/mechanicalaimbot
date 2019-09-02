import logging
import threading
import time

import numpy as np
import mss
from PIL import Image

from mechbot.inference.detection import ObjectDetector
from mechbot.utils import profiling


class InferenceThread(threading.Thread):
    """Thread deticated to running object detection"""
    def __init__(self, run_until, config):
        super(InferenceThread, self).__init__(name="InferenceThread")
        self.run_until = run_until
        self.config = config
        self.detection_listeners = []
    
    def run(self):
        graph_path = self.config.inference_graph
        monitor_nr = self.config.monitor_number
        score_thresh = self.config.score_thresh
        input_format = self.config.inference_input_format

        profiler = profiling.MultiStopwatch(maxlaps=10)

        with ObjectDetector(graph_path) as detector, mss.mss() as sct:
            while not self.run_until.is_set():
                profiler.lap()
                screen_shot = sct.grab(sct.monitors[monitor_nr])
                profiler.partial("capture")

                if input_format == "RGB":
                    (im_width, im_height) = screen_shot.size
                    data = Image.frombytes("RGB", screen_shot.size, screen_shot.bgra, "raw", "BGRX").tobytes()
                    image = np.frombuffer(data, dtype=np.uint8).reshape((im_height, im_width, 3))
                else:
                    image = screen_shot.bgra
                profiler.partial("conversion")

                # Run inference
                results = detector.run_single(image)

                # print(results)
                detections = [(box, classID) for box, score, classID 
                              in zip(*results) if score > score_thresh]

                profiler.partial("detection")

                timings = profiler.get_frozen_data()
                for listener in self.detection_listeners:
                    listener(np.array(screen_shot), detections, timings)

        logging.info("exit")

    def add_detection_listener(self, listener):
        self.detection_listeners.append(listener)
