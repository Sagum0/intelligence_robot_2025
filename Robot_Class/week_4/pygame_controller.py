import pygame
import os, time

if __name__ == '__main__':

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick connected")

    print(pygame.joystick.get_count())
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print("Joystick name: {}".format(joystick.get_name()))
    print("Number of axes: {}".format(joystick.get_numaxes()))
    print("Number of buttons: {}".format(joystick.get_numbuttons()))

    try:
        while True:
            os.system('clear')
            pygame.event.pump()

            sticks = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
            print(sticks)
            buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
            # print(buttons)
            hat_control = [joystick.get_hat(i) for i in range(joystick.get_numhats())]
            #print(hat_control)
            
            time.sleep(1/60)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        pygame.quit()