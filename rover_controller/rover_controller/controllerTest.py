import pygame

pygame.init()
pygame.joystick.init()

# List the available joystick devices
joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]

# Initialize all detected joysticks
for joystick in joysticks:
    joystick.init()
    print(f"Joystick {joystick.get_id()}: {joystick.get_name()}")

times = 0
# Example: Print the position of the left and right analog sticks for all detected joysticks
while True:
    pygame.event.get()
    for joystick in joysticks:
        left_x_axis = joystick.get_axis(0)
        left_y_axis = joystick.get_axis(1)
        right_x_axis = joystick.get_axis(2)
        right_y_axis = joystick.get_axis(3)
        left_trigger = joystick.get_axis(5)
        right_trigger = joystick.get_axis(4)
        

        print(f"Left Joystick: X={left_x_axis}, Y={left_y_axis}, Right Joystick: X={right_x_axis}, Y={right_y_axis}, Left Trigger = {left_trigger},  Right Trigger = {right_trigger}")