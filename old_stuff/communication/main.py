import pygame
import drone
import utils

pygame.init()

# set screen
DISPLAY_INFO = pygame.display.Info()
SCREEN_WIDTH = int(DISPLAY_INFO.current_w * 0.8)
SCREEN_HEIGHT = int(DISPLAY_INFO.current_h * 0.8)
WINDOW = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

CELL = 16
WINDOW_MARGIN = 1

pygame.display.set_caption("Drone Simulation")

# panel
PANEL = pygame.Surface((int(SCREEN_WIDTH * 0.1), SCREEN_HEIGHT))
PANEL.fill((255, 255, 255))

# grid
GRID = pygame.Surface((SCREEN_WIDTH - PANEL.get_width(), SCREEN_HEIGHT))
GRID.fill((0, 0, 0))

# grid array
ROW_MARGIN = GRID.get_height() % CELL
COLUMN_MARGIN = GRID.get_width() % CELL

ROWS = GRID.get_height() // CELL
COLUMNS = GRID.get_width() // CELL

ARRAY = [[utils.random_color() for i in range(COLUMNS)] for j in range(ROWS)]

# drone constants
X = (COLUMNS // 2) * CELL + PANEL.get_width() + COLUMN_MARGIN + WINDOW_MARGIN
Y = (ROWS // 2) * CELL + ROW_MARGIN + WINDOW_MARGIN

DRONE1 = drone.Drone()

RUN = True

greenbutton = utils.button((0, 200, 0), int(PANEL.get_width() // 5), 50, int(PANEL.get_width() // 2), int(PANEL.get_width() // 5), 'test')

def redrawWindow():
    pygame.draw.circle(WINDOW, DRONE1.color, (X, Y), DRONE1.radius)
    greenbutton.draw(WINDOW, (0, 0, 0))

grid = []
for row in range(ROWS):
    grid.append([])
    for column in range(COLUMNS):
        grid[row].append(0)

while RUN:
    pygame.time.delay(5)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            RUN = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            if greenbutton.isOver(pygame.mouse.get_pos()):
                print('clicked green')

            #pos = pygame.mouse.get_pos()
            #row = int((pos[1] - WINDOW_MARGIN) / CELL)
            #column = int((pos[0] - int(PANEL.get_width()) - ( 3 * WINDOW_MARGIN)) / CELL)
            #grid[row][column] = 1

    KEYS = pygame.key.get_pressed()

    

    if KEYS[pygame.K_LEFT] and X > PANEL.get_width() + DRONE1.radius + COLUMN_MARGIN+ WINDOW_MARGIN:
        X -= DRONE1.velocity
    if KEYS[pygame.K_RIGHT] and X < SCREEN_WIDTH - DRONE1.radius - DRONE1.velocity:
        X += DRONE1.velocity
    if KEYS[pygame.K_UP] and Y > DRONE1.velocity + ROW_MARGIN + WINDOW_MARGIN:
        Y -= DRONE1.velocity
    if KEYS[pygame.K_DOWN] and Y < SCREEN_HEIGHT - DRONE1.radius - DRONE1.velocity:
        Y += DRONE1.velocity

    WINDOW.fill((0, 0, 0))
    WINDOW.blit(PANEL, (0 + WINDOW_MARGIN, 0 + WINDOW_MARGIN))

    WINDOW.blit(GRID, (PANEL.get_width() + WINDOW_MARGIN * 2, 0 + WINDOW_MARGIN))

    for row in range(ROWS):
        for column in range(COLUMNS):
            color = ARRAY[row][column]
            if grid[row][column] == 1:
                color = (255, 0, 0)
            pygame.draw.rect(GRID, color, [
                (COLUMN_MARGIN // 2) + CELL * column,
                (ROW_MARGIN // 2) + CELL * row,
                CELL, CELL
            ])

    redrawWindow()
    pygame.display.update()

pygame.quit()
