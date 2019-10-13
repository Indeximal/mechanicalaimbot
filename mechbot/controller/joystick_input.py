import threading
import time

import numpy as np
import pygame

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

        while not self.run_until.is_set():
            raw_input = self.interface.get_input()
            deadzone = self.config.controller_deadzone
            # Values below the deadzone are interpreted as 0 input on that axis
            j_input = (np.absolute(raw_input) > deadzone) * raw_input
            with self.total_lock:
                self.total_motion += j_input * stopwatch.get_and_reset()

            time.sleep(self.delta_time)

    def get_and_reset_total(self):
        with self.total_lock:
            ret = self.total_motion
            self.total_motion = np.zeros(2, dtype=float)
        return ret
