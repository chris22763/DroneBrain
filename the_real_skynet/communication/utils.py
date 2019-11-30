import random
import pygame

COLORS = [
    (114, 59, 18),
    (120, 65, 14),
    (125, 72, 8),
    (130, 79, 1),
    (133, 86, 0),
    (136, 94, 0),
    (139, 102, 0),
    (140, 110, 0),
    (141, 118, 0),
    (140, 127, 0),
    (139, 136, 0),
    (136, 145, 0),
    (133, 154, 0),
    (128, 163, 0),
    (121, 172, 0),
    (113, 181, 0),
    (102, 190, 0),
    (87, 199, 0),
    (65, 209, 24),
    (18, 218, 45)
]

def random_color():
    return random.choice(COLORS)

class button():
    def __init__(self, color, x, y, width, height, text=''):
        self.color = color
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.text = text

    def draw(self, win, outline=None):
        #Call this method to draw the button on the screen
        if outline:
            pygame.draw.rect(win, outline, (self.x-2, self.y-2, self.width + 4, self.height + 4), 0)

        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.height), 0)

        if self.text != '':
            font = pygame.font.SysFont('comicsans', 60)
            text = font.render(self.text, 1, (0, 0, 0))
            win.blit(text, (self.x + (self.width / 2 - text.get_width() / 2), self.y + (self.height / 2 - text.get_height() / 2)))

    def isOver(self, pos):
        #Pos is the mouse position or a tuple of (x,y) coordinates
        if pos[0] > self.x and pos[0] < self.x + self.width:
            if pos[1] > self.y and pos[1] < self.y + self.height:
                return True

        return False
