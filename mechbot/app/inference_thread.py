import threading
import time

import numpy as np


class InferenceTimings:
    def __init__(self, t_capture_start, t_capture_end, t_inference_start, 
                 t_inference_end):
        self.start_time = t_capture_start
        self.capture_duration = t_capture_end - t_capture_start
        self.conversion_duration = t_inference_start - t_capture_end
        self.inference_duration = t_inference_end - t_inference_start


class InferenceThread(threading.Thread):
    """Thread deticated to running object detection"""
    def __init__(self, run_until):
        super(InferenceThread, self).__init__(name="Inference-Thread")
        self.run_until = run_until
        self.detection_listeners = []
    
    def run(self):
        while not self.run_until.is_set():
            frame = np.zeros((1920, 1080, 3), dtype=np.uint8)

            detections = np.zeros((100, 4), dtype=float)

            for listener in self.detection_listeners:
                timings = InferenceTimings(.1, .2, .3, .4)
                listener(frame, detections, timings)

            time.sleep(1 / 60)  # Limit thread while testing

    def add_detection_listener(self, listener):
        self.detection_listeners.append(listener)
