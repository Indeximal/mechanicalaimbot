import threading
import signal
import logging
import time

# import yappi
from mechbot.app import configuration
from mechbot.app.debug_inference import DebugInferenceThread
from mechbot.app.inference_thread import InferenceThread
from mechbot.app.motion_thread import MotionThread
from mechbot.app.gui_thread import GUIThread


def run():
    options = configuration.parse_config()

    level = logging.DEBUG if options.debug_output else logging.INFO
    fmt = "%(levelname)s (%(threadName)s): %(message)s"
    logging.basicConfig(level=level, format=fmt)
    logging.debug(options)

    # Proper shutdown
    shutdown_event = threading.Event()

    def shutdown(*args, **kwargs):
        logging.info("Shutting down...")
        shutdown_event.set()

    # signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    if options.use_debug_detection:
        inference_thread = DebugInferenceThread(run_until=shutdown_event,
                                                config=options)
    else:
        inference_thread = InferenceThread(run_until=shutdown_event,
                                           config=options)
    motion_thread = MotionThread(run_until=shutdown_event, config=options)
    gui_thread = GUIThread(run_until=shutdown_event, config=options)

    # Wiring up event listeners
    inference_thread.add_detection_listener(motion_thread.push_detections)
    inference_thread.add_detection_listener(gui_thread.push_detections)

    motion_thread.add_status_listener(gui_thread.push_device_status)

    gui_thread.add_shutdown_listener(shutdown)
    gui_thread.add_select_team_listener(motion_thread.team_selection_listener)

    # Start threads
    inference_thread.start()
    motion_thread.start()
    gui_thread.start()

    # GUI might have to run in main thread for macOS
    # gui_thread.run()

    begin = time.time()

    # Shutdown when gui thread exits
    while not shutdown_event.is_set():
        if not gui_thread.is_alive():
            logging.warning("Gui thread has exited unexpectedly!")
            shutdown()
            break
        time.sleep(.1)
        # if not yappi.is_running() and time.time() - begin > 8.:
        #     print("start")
        #     yappi.start()

    # func_stats = yappi.get_func_stats()
    # func_stats.save("prof.out." + str(time.time()), "pstat")
    # func_stats.print_all()
    # yappi.get_thread_stats().print_all()

    logging.info("exit")


from mechbot.utils.fields import CsgoTeamEnum


if __name__ == "__main__":
    run()
