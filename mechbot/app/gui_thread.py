import logging
import threading
import queue

import pygame
import numpy as np

from mechbot.app.motion_thread import DeviceStatusEnum
from mechbot.utils import pygame_utils
from mechbot.utils.fields import CsgoTeamEnum


class GUIThread(threading.Thread):
    """Thread dedicated to running the pygame display"""
    def __init__(self, run_until, config):
        super(GUIThread, self).__init__(name="GUIThread")
        self.team_selection_listeners = []
        self.run_until = run_until
        self.config = config
        self.shutdown_listeners = []
        self.detection_queue = queue.Queue(maxsize=5)
        self.display_device = None
        self.target_pos = None
        self.joystick_pos = np.zeros(2)
        self.team = CsgoTeamEnum(self.config.default_team)
        self.device_status = "Initializing"

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

        def create_camera():
            x_limit = 1.5
            y_limit = 1.2
            cam_scale = self.config.device_size / 2.0 / x_limit * width
            x = width - x_limit * cam_scale
            y = y_limit * cam_scale
            return pygame_utils.Camera(cam_scale, x, y, False)

        device_camera = create_camera()

        info_display = pygame_utils.LineWriter(20, 20,
                                               color=self.config.text_color,
                                               size=self.config.text_size,
                                               bg_color=self.config.text_bg)

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
                    # Select team ct
                    if event.key == pygame.K_c:
                        for listener in self.team_selection_listeners:
                            self.team = CsgoTeamEnum.COUNTER_TERRORISTS
                            listener(self.team)
                    # Select team t
                    if event.key == pygame.K_t:
                        for listener in self.team_selection_listeners:
                            self.team = CsgoTeamEnum.TERRORISTS
                            listener(self.team)
                # Resize
                if event.type == pygame.VIDEORESIZE:
                    screen_size = width, height = event.w, event.h
                    screen = pygame.display.set_mode(screen_size,
                                                     pygame.RESIZABLE)
                    device_camera = create_camera()

            # Fetch new data if availiable
            if not self.detection_queue.empty():
                frame, detections, timings = self.detection_queue.get()
                w, h, d = frame.shape
                # rgb_data = np.empty((w, h, 3), dtype=np.uint8)
                # rgb_data[:, :, 0] = frame[:, :, 2]
                # rgb_data[:, :, 1] = frame[:, :, 1]
                # rgb_data[:, :, 2] = frame[:, :, 0]
                frame_surface = pygame.surfarray.make_surface(frame)

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
            # info_display.print("Device status: "
            #                    + self.device_status.capitalize())

            info_display.print("Your Team: " + self.team.name.capitalize())
            info_display.print("Press C or T to change team")

            # DRAWING
            screen.fill((255, 255, 255))

            if display_surface is not None:
                screen.blit(display_surface, (0, 0))

            team_color_index = 0 if self.team == CsgoTeamEnum.TERRORISTS else 2
            team_color = self.config.rect_color_per_class[team_color_index]
            pygame.draw.rect(screen, tuple(team_color),
                             (0, 0, width, height), 8)

            info_display.draw(screen)

            # Draw device
            if self.display_device is not None:
                # TODO: Background
                self.display_device.draw(screen, device_camera)

            # Draw Joystick
            pygame.draw.circle(screen, (33, 34, 41),
                               device_camera.pixel(self.joystick_pos),
                               device_camera.pixel_len(
                                   self.config.joystick_radius * 1.1), 0)

            # Draw target
            if self.target_pos is not None:
                pygame.draw.circle(screen, (110, 110, 200),
                                   device_camera.pixel(self.target_pos),
                                   device_camera.pixel_len(
                                       self.config.joystick_radius), 3)

            clock.tick(self.config.display_fps)
            pygame.display.flip()

        pygame.quit()
        logging.info("exit")

    def push_detections(self, frame, rects, timings):
        self.detection_queue.put((frame, rects, timings))

    def add_shutdown_listener(self, listener):
        self.shutdown_listeners.append(listener)

    def push_device_status(self, status_type, *args):
        if status_type == DeviceStatusEnum.CALIBRATED:
            device, = args
            self.display_device = device
            self.device_status = "Calibrated"
        elif status_type == DeviceStatusEnum.TARGET:
            pos, s2, s2, current = args
            self.target_pos = pos
            self.joystick_pos = current
        else:
            self.device_status = status_type.name

    def add_select_team_listener(self, team_selection_listener):
        self.team_selection_listeners.append(team_selection_listener)
