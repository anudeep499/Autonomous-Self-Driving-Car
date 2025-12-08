import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty, select

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')
        self.cmd_pub = self.create_publisher(Twist, 'rover_vel', 10)
        
    def update_cmd_vel(self, speed=0.0, rate=0.0):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = -rate
        self.cmd_pub.publish(msg)
        
def main():

    rclpy.init()
    controller = RoverController()
    print("Use controller to move rover. Press CTRL+C to exit. \n -------------------------")
    print("Controls: \n")
    print("    Right Trigger - Throttle \n")
    print("    Left Stick - Steering \n")
    
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
    try:
        while rclpy.ok():
            pygame.event.get()
            for joystick in joysticks:
                left_x_axis = joystick.get_axis(0)
                left_y_axis = joystick.get_axis(1)
                right_x_axis = joystick.get_axis(2)
                right_y_axis = joystick.get_axis(3)
                left_trigger = joystick.get_axis(5)
                right_trigger = joystick.get_axis(4)
                
                max_speed = 0.4
                max_turn_rate = 2
                speed = max(-max_speed, (((right_trigger + 1.0) - (left_trigger + 1)) / 2.0) * max_speed)   # map -1→1 to 0→1
                steering = left_x_axis * max_turn_rate
                
                controller.update_cmd_vel(speed=speed, rate=steering)
                
                #print(f"Left Joystick: X={left_x_axis}, Y={left_y_axis}, Right Joystick: X={right_x_axis}, Y={right_y_axis}, Left Trigger = {left_trigger},  Right Trigger = {right_trigger}")
            
            rclpy.spin_once(controller, timeout_sec=0.01)
            pygame.time.wait(10)  # avoid 100% CPU
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        pygame.quit()