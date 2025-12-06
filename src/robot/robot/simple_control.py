import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

TIMER_PERIOD = 0.1
LINEAR_SPEED = 0.2
ANGULAR_SPEED = 0.


class SimpleControl(Node):
    def __init__(self):
        super().__init__('simple_control')

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def timer_callback(self):
        command_message = Twist()
        command_message.linear.x = LINEAR_SPEED
        command_message.angular.z = ANGULAR_SPEED
        self.__publisher.publish(command_message)

def main(args=None):
    rclpy.init(args=args)
    simple_control = SimpleControl()
    rclpy.spin(simple_control)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simple_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()