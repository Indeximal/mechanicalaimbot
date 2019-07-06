import pygame

class LineWriter:
    def __init__(self, x, y, h=30, size=24, color=(0, 0, 0)):
        pygame.font.init()
        self.font = pygame.font.Font(None, size)
        self.x = x
        self.y = y
        self.h = h
        self.color = color
        self.lines = []

    def print(self, text):
        self.lines.append(str(text))

    def draw(self, screen):
        y = self.y
        for line in self.lines:
            text_surface = self.font.render(line, True, self.color)
            screen.blit(text_surface, (self.x, y))
            y += self.h
        self.lines = []


class Camera:
    def __init__(self, scale, shift_x, shift_y):
        self.scale = scale
        self.dx = shift_x
        self.dy = shift_y

    def pixel_len(self, l):
        return int(l * self.scale)

    def pixel(self, pos):
        x, y = pos
        return (int(x * self.scale + self.dx), int(y * self.scale + self.dy))

    def world_pos(self, pixel):
        x, y = pixel
        return ((x - self.dx) / self.scale, (y - self.dy) / self.scale)
        