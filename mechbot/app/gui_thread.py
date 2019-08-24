import threading
import time
import queue

class GUIThread(threading.Thread):
    """Thread deticated to running the pygame display"""
    def __init__(self, run_until):
        super(GUIThread, self).__init__(name="GUI-Thread")
        self.run_until = run_until
        self.shutdown_listeners = []
        self.detection_queue = queue.Queue(maxsize=5)
        self.display_device = None
        self.target_pos = None

    def active(self):
        return not self.run_until.is_set()
    
    def run(self):
        while self.active():
            if not self.detection_queue.empty():
                frame, detection_rects, timings = self.detection_queue.get()

            if False:  # Shutdown
                for listener in self.shutdown_listeners:
                    listener()

            time.sleep(1 / 60)  # Limit thread while testing

    def set_device(self, device):
        self.display_device = device

    def set_target(self, pos):
        self.target_pos = pos

    def push_detections(self, frame, rects, timings):
        self.detection_queue.put((frame, rects, timings))

    def add_shutdown_listener(self, listener):
        self.shutdown_listeners.append(listener)