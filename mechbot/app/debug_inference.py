import logging
import threading

import numpy as np
import mss
from PIL import Image

from mechbot.training.box_extractor import ColoredBoxExtractor
from mechbot.utils import profiling


class DebugInferenceThread(threading.Thread):
    """Thread dedicated to detecting pink debug boxes as a replacement for
    proper object detection
    """
    def __init__(self, run_until, config):
        super(DebugInferenceThread, self).__init__(name="DInferenceThread")
        self.run_until = run_until
        self.config = config
        self.detection_listeners = []

    def run(self):
        # Get options from config
        monitor_nr = self.config.monitor_number
        class_id = self.config.pink_class_id

        profiler = profiling.MultiStopwatch(maxlaps=10)

        # Helper to extract pink boxes from a frame
        extractor = ColoredBoxExtractor((200, 0, 200), (255, 50, 255), 210)

        with mss.mss() as sct:
            while not self.run_until.is_set():
                # Capture screenshot
                profiler.lap()
                screen_shot = sct.grab(sct.monitors[monitor_nr])
                profiler.partial("capture")

                (im_width, im_height) = screen_shot.size
                # Convert to RGB, because the UI needs it
                data = Image.frombytes("RGB", screen_shot.size,
                                       screen_shot.bgra, "raw",
                                       "BGRX").tobytes()
                image = np.frombuffer(data, dtype=np.uint8).reshape(
                    (im_height, im_width, 3))
                profiler.partial("conversion")

                # Run extraction
                results = extractor.extract(image)

                boxes = [np.array(
                    [y / im_height, x / im_width, (y + h) / im_height,
                    (x + w) / im_width]) for x, y, w, h in results]

                # print(results)
                detections = [(box, class_id) for box in boxes]

                profiler.partial("detection")

                timings = profiler.get_frozen_data()
                for listener in self.detection_listeners:
                    listener(image, detections, timings)

        logging.info("exit")

    def add_detection_listener(self, listener):
        self.detection_listeners.append(listener)
