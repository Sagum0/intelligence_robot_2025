import math
import sys
import pygame
import numpy as np

WIDTH, HEIGHT = 1000, 700
# Colors (R, G, B)
BG_COLOR = (25, 28, 34)
GRID_COLOR = (40, 46, 54)
CAR_COLOR = (80, 200, 255)
CAR_OUTLINE = (230, 245, 255)

def draw_grid(screen, grid_spacing=50):
    for x in range(0, WIDTH, grid_spacing):
        pygame.draw.line(screen, GRID_COLOR, (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, grid_spacing):
        pygame.draw.line(screen, GRID_COLOR, (0, y), (WIDTH, y))

def draw_car(screen, x, y, angle=0):
    car_length = 80
    car_width = 40
    car_center = (WIDTH // 2 + x, HEIGHT // 2 + y)

    # Calculate rectangle corners
    rect = pygame.Rect(0, 0, car_length, car_width)
    rect.center = car_center

    # Draw body (no rotation)
    pygame.draw.rect(screen, CAR_COLOR, rect)
    pygame.draw.rect(screen, CAR_OUTLINE, rect, 3)

    
def get_joystick(joystick):
    pygame.event.pump()
    sticks = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
    hat_control = [joystick.get_hat(i) for i in range(joystick.get_numhats())]

    return sticks, buttons, hat_control

def main():
    pygame.init()
    pygame.display.set_caption("Xbox Controller Car")
    
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("No joystick connected")
    
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    running = True
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    v = 0
    t = 0
    accel_ratio = 0.00001
    brake_ratio = 0.00001
    steer_ratio = 0.1
    
    center = [WIDTH // 2, HEIGHT // 2]
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
        
        sticks, _, _ = get_joystick(joystick)
        
        accel = round(sticks[1], 1) # -1에 가까울 수록 전진, 1에 가까울 수록 후진
        steer = round(sticks[3], 1) # 왼쪽 = -1, 오른쪽 = 1
        
        if accel == 0:
            pass
            
        # 속도 제어가 들어감
        else:
            if accel > 0:
                v += accel * accel_ratio
            elif accel < 0:
                v += accel * brake_ratio
        
        t += steer * steer_ratio
        
        center[0] += v * np.cos(t)
        center[1] += v * np.sin(t)

        screen.fill(BG_COLOR)
        draw_grid(screen)
        draw_car(screen, center, t)
        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
