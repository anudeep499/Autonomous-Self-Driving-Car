#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty, select

def get_key():
    """Non-blocking key capture from terminal"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(3)  # arrow keys are 3 chars long
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')
        self.cmd_pub = self.create_publisher(Twist, 'rover_vel', 10)

    def move_forward(self, speed=0.5):
        msg = Twist()
        msg.linear.x = speed
        self.cmd_pub.publish(msg)

    def move_backward(self, speed=0.5):
        msg = Twist()
        msg.linear.x = -speed
        self.cmd_pub.publish(msg)

    def turn_left(self, rate=0.5):
        msg = Twist()
        msg.angular.z = rate
        self.cmd_pub.publish(msg)

    def turn_right(self, rate=0.5):
        msg = Twist()
        msg.angular.z = -rate
        self.cmd_pub.publish(msg)

    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    controller = RoverController()
    print("Use arrow keys to move the rover. Press CTRL+C to exit.")

    try:
        while rclpy.ok():
            key = get_key()

            if key == '\x1b[A':  # UP arrow
                controller.move_forward(0.4)
            elif key == '\x1b[B':  # DOWN arrow
                controller.move_backward(0.4)
            elif key == '\x1b[D':  # LEFT arrow
                controller.turn_left(2.0)
            elif key == '\x1b[C':  # RIGHT arrow
                controller.turn_right(2.0)
            elif key == '':  
                controller.stop()
            else:
                controller.stop()

    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
