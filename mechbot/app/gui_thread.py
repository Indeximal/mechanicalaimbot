import threading
import queue

import pygame
import numpy as np

from mechbot.utils import pygame_utils


class GUIThread(threading.Thread):
    """Thread deticated to running the pygame display"""
    def __init__(self, run_until, config):
        super(GUIThread, self).__init__(name="GUI-Thread")
        self.run_until = run_until
        self.config = config
        self.shutdown_listeners = []
        self.detection_queue = queue.Queue(maxsize=5)
        self.display_device = None
        self.target_pos = None

    def active(self):
        return not self.run_until.is_set()
    
    def run(self):
        # Initialize pygame
        pygame.init()
        pygame.font.init()

        screen_size = self.config.display_width, self.config.display_height
        width, height = screen_size
        screen = pygame.display.set_mode(screen_size, pygame.RESIZABLE)
        pygame.display.set_caption(self.config.display_name)
        clock = pygame.time.Clock()

        display_surface = None
        timings = None

        info_display = pygame_utils.LineWriter(20, 20, color=(204, 20, 20))

        while self.active():
            for event in pygame.event.get():
                if event.type == pygame.QUIT: 
                    for listener in self.shutdown_listeners:
                        listener()
                # Key event
                if event.type == pygame.KEYDOWN:
                    # Quit
                    if event.key == pygame.K_ESCAPE:
                        for listener in self.shutdown_listeners:
                            listener()
                # Resize
                if event.type == pygame.VIDEORESIZE:
                    screen_size = width, height = event.w, event.h
                    screen = pygame.display.set_mode(screen_size, 
                                                     pygame.RESIZABLE)

            # Fetch new data if availiable
            if not self.detection_queue.empty():
                frame, detections, timings = self.detection_queue.get()
                w, h, d = frame.shape
                rgb_data = np.empty((w, h, 3), dtype=np.uint8)
                rgb_data[:, :, 0] = frame[:, :, 2]
                rgb_data[:, :, 1] = frame[:, :, 1]
                rgb_data[:, :, 2] = frame[:, :, 0]
                frame_surface = pygame.surfarray.make_surface(rgb_data)

                for (y_min, x_min, y_max, x_max), class_id in detections:
                    color = self.config.rect_color_per_class[class_id-1]
                    color = tuple(color)
                    rect = (y_min * w, x_min * h,
                           (y_max - y_min) * w, (x_max - x_min) * h)
                    pygame.draw.rect(frame_surface, color, rect, 
                                     self.config.rect_width)

                scale = min(width / h, height / w)  # rotation swaps h and w
                surf = pygame.transform.rotozoom(frame_surface, 90, scale)
                display_surface = pygame.transform.flip(surf, False, True)


            if timings is not None:
                avg_ms = timings.avg_lap_time() * 1000
                info_display.print("Total: {:.0f}ms".format(avg_ms))
                deltas = timings.partial_durations_min_avg_max()
                for name, (t_min, t_avg, t_max) in deltas.items():
                    info_display.print("{}: {:.0f}ms>{:.0f}ms>{:.0f}ms".format(
                        name, t_min * 1000, t_avg * 1000, t_max * 1000))

            screen.fill((255, 255, 255))

            if display_surface is not None:
                screen.blit(display_surface, (0, 0))

            info_display.draw(screen)

            clock.tick(self.config.display_fps)
            pygame.display.flip()


    def set_device(self, device):
        self.display_device = device

    def set_target(self, pos):
        self.target_pos = pos

    def push_detections(self, frame, rects, timings):
        self.detection_queue.put((frame, rects, timings))

    def add_shutdown_listener(self, listener):
        self.shutdown_listeners.append(listener)