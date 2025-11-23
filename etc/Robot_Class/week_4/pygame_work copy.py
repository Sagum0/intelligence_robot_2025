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

def draw_car(screen, cx, cy, t):
    car_length = 80
    car_width = 40
    
    angle_rad = np.deg2rad(t)
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)
    
    dx = car_width // 2
    dy = car_length // 2
    
    corners = [
        (-dx, -dy),
        (+dx, -dy),
        (+dx, +dy),
        (-dx, +dy)
    ]
    
    rotated_points = []
    for x, y in corners:
        xr = x * cos_a - y * sin_a
        yr = x * sin_a + y * cos_a
        rotated_points.append((cx + xr, cy + yr))
        
    pygame.draw.polygon(screen, CAR_COLOR, rotated_points)
    pygame.draw.polygon(screen, CAR_OUTLINE, rotated_points, 3)

    
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

    clock = pygame.time.Clock()
    v = 0.0       
    heading_deg = 0.0     
    accel_power = 600.0 
    brake_power = 800.0 
    steer_speed = 160.0
    dead_zone = 0.1 

    x, y = [WIDTH // 2, HEIGHT // 2]
    
    while running:
        dt = clock.tick(120) / 1000.0 

        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
        
        sticks, _, _ = get_joystick(joystick)
        if len(sticks) > 1:
            accel_input = -sticks[1]  
        else:
            accel_input = 0.0
            
        if len(sticks) > 3:
            steer_input = sticks[3] 
        else: 
            steer_input = 0.0

        if abs(accel_input) > dead_zone:
            if accel_input > 0:
                v += accel_input * accel_power * dt
            else:
                v += accel_input * brake_power * dt
        else:
            if v > 0:
                v = max(0.0, v - accel_power * dt * 0.1)
            elif v < 0:
                v = min(0.0, v + accel_power * dt * 0.1)

        v = max(-350.0, min(350.0, v))

        if abs(steer_input) > dead_zone and abs(v) > 1.0:
            if v >= 0:
                steer_direction = 1  
            else: 
                steer_direction = -1
                
            heading_deg += steer_input * steer_direction * steer_speed * dt

        heading_rad = np.deg2rad(heading_deg)
        x += np.sin(heading_rad) * v * dt
        y -= np.cos(heading_rad) * v * dt

        x = max(0, min(WIDTH, x))
        y = max(0, min(HEIGHT, y))

        screen.fill(BG_COLOR)
        draw_grid(screen)
        draw_car(screen, x, y, heading_deg)
        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
