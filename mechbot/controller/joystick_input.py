import threading
import time

import numpy as np

from mechbot.app import configuration
from mechbot.utils.profiling import Stopwatch


class JoystickInputThread(threading.Thread):
    def __init__(self, run_until, config, interface):
        super(JoystickInputThread, self).__init__(name="JoystickThread")
        self.total_motion = np.zeros(2, dtype=float)
        self.daemon = True
        self.run_until = run_until
        self.config = config
        self.delta_time = self.config.joystick_input_dt
        self.interface = interface
        self.total_lock = threading.Lock()

    def run(self):
        stopwatch = Stopwatch()
        sensitivity_curve = configuration.setup_sensitivity_curve(self.config)

        while not self.run_until.is_set():
            raw_input = self.interface.get_input()
            angular_speed = sensitivity_curve(raw_input)  # [360°/s]

            delta_t = stopwatch.get_and_reset()
            with self.total_lock:
                # [360°] = [360°/s] * [s]
                self.total_motion += angular_speed * delta_t

            time.sleep(self.delta_time)

    def get_and_reset_total(self):
        with self.total_lock:
            ret = self.total_motion
            self.total_motion = np.zeros(2, dtype=float)
        return ret
