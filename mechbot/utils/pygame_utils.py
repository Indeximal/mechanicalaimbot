import pygame

class LineWriter:
    def __init__(self, x, y, h=30, size=24, color=(0, 0, 0), bg_color=None,
                 bg_border=3, bg_width=400):
        pygame.font.init()
        self.font = pygame.font.Font(None, size)
        self.x = x
        self.y = y
        self.h = h
        self.color = color
        self.lines = []
        self.bg_color = bg_color
        self.bg_border = bg_border
        self.bg_width = bg_width

    def print(self, text):
        self.lines.append(str(text))

    def draw(self, screen):
        y = self.y
        if self.bg_color is not None:
            height = len(self.lines) * self.h + 2 * self.bg_border
            s = pygame.Surface((self.bg_width, height))
            s.set_alpha(self.bg_color[3])  # transparency
            s.fill(self.bg_color[:3])
            # unlike pygame.draw screen.blit allows transparency
            screen.blit(s, (self.x - self.bg_border, self.y - self.bg_border))
        for line in self.lines:
            text_surface = self.font.render(line, True, self.color)
            screen.blit(text_surface, (self.x, y))
            y += self.h
        self.lines = []


class Camera:
    def __init__(self, scale, shift_x, shift_y, invert_y=False):
        self.scale = scale
        self.y_scale = -scale if invert_y else scale
        self.dx = shift_x
        self.dy = shift_y

    def pixel_len(self, l):
        return int(l * self.scale)

    def pixel(self, pos):
        x, y = pos
        return int(x * self.scale + self.dx), int(y * self.y_scale + self.dy)

    def world_pos(self, pixel):
        x, y = pixel
        return (x - self.dx) / self.scale, (y - self.dy) / self.y_scale
