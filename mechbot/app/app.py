import threading
import signal

import configargparse

from mechbot.app.inference_thread import InferenceThread
from mechbot.app.motion_thread import MotionThread
from mechbot.app.gui_thread import GUIThread

def run():
    parser = configargparse.ArgParser()
    parser.add("--serial_baud", type=int, required=True)
    parser.add("--serial_port", type=str, required=True)

    options = parser.parse_args()
    option_dict = vars(options)

    print(option_dict)

    shutdown_event = threading.Event()

    def shutdown(signum, frame):
        print("Shutting down...")
        shutdown_event.set()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    inference_thread = InferenceThread(run_until=shutdown_event)
    motion_thread = MotionThread(run_until=shutdown_event, **option_dict)
    gui_thread = GUIThread(run_until=shutdown_event)

    # Wiring up event listeners
    inference_thread.add_detection_listener(motion_thread.push_detections)
    inference_thread.add_detection_listener(gui_thread.push_detections)

    motion_thread.add_calibrated_listener(gui_thread.set_device)
    motion_thread.add_target_listener(gui_thread.set_target)

    gui_thread.add_shutdown_listener(shutdown)

    # Start threads
    inference_thread.start()
    motion_thread.start()
    gui_thread.start()


if __name__ == '__main__':
    run()


# Serial
# load_graph()

# with session:
#     init_graph()
#     with mms:
#         while True:
#             handle_pygame()

#             if calibrated:
#                 get_frame()
#                 run_inference()
#                 send_to_hardware()
#             else:
#                 calibrate_hareware()

