import threading
import signal
import time
from pathlib import Path

import configargparse
import numpy as np
import yappi

from mechbot.app.inference_thread import InferenceThread
from mechbot.app.motion_thread import MotionThread
from mechbot.app.gui_thread import GUIThread
from mechbot import resources


def run():
    parser = configargparse.ArgParser(
        default_config_files=[resources.DEFAULT_CONFIG])

    parser.add("-c", "--config", is_config_file=True, help="config file path")
    parser.add("--serial_port", type=str, required=True)
    parser.add("--serial_baud", type=int, required=True)
    parser.add("--display_width", type=int, required=True)
    parser.add("--display_height", type=int, required=True)
    parser.add("--inference_graph", type=str, required=True)
    parser.add("--inference_input_format", type=str, required=True,
               choices=["RGB", "BGRA"])
    parser.add("--display_name", type=str, required=True)
    parser.add("--display_fps", type=int, required=True)
    parser.add("--score_thresh", type=float, required=True)
    parser.add("--monitor_number", type=int, required=True)
    parser.add("--rect_width", type=int, required=True)
    parser.add("--rect_color_per_class", type=int, action="append",
               required=True)

    options = parser.parse_args()

    options.rect_color_per_class = np.array(
        options.rect_color_per_class).reshape(-1, 3)

    # Handle special cases for paths in options
    if options.inference_graph.startswith("resources."):
        inference_path = vars(resources)[options.inference_graph[10:]]
    else:
        inference_path = str(Path(options.inference_graph))
    options.inference_graph = inference_path

    option_dict = vars(options)

    shutdown_event = threading.Event()

    def shutdown(*args):
        print("Shutting down...")
        shutdown_event.set()

    # signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    inference_thread = InferenceThread(run_until=shutdown_event,
                                       config=options)
    motion_thread = MotionThread(run_until=shutdown_event, **option_dict)
    gui_thread = GUIThread(run_until=shutdown_event, config=options)

    # Wiring up event listeners
    inference_thread.add_detection_listener(motion_thread.push_detections)
    inference_thread.add_detection_listener(gui_thread.push_detections)

    motion_thread.add_calibrated_listener(gui_thread.set_device)
    motion_thread.add_target_listener(gui_thread.set_target)

    gui_thread.add_shutdown_listener(shutdown)

    # Start threads
    inference_thread.start()
    # motion_thread.start()
    gui_thread.start()

    begin = time.time()

    # Shutdown when gui thread exits
    while not shutdown_event.is_set():
        if not gui_thread.is_alive():
            print("Gui thread has exited unexpectedly!")
            shutdown()
            break
        time.sleep(.1)
        if not yappi.is_running() and time.time() - begin > 8.:
            print("start")
            yappi.start()

    func_stats = yappi.get_func_stats()
    func_stats.save("prof.out." + str(time.time()), "pstat")
    func_stats.print_all()
    yappi.get_thread_stats().print_all()


if __name__ == "__main__":
    run()
