#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    """
    A simple talker node that publishes string messages to a topic.
    """
    
    def __init__(self):
        super().__init__('talker')
        
        # Create a publisher for String messages on the 'chatter' topic
        self.publisher = self.create_publisher(String, 'chatter', 1)
        
        # Create a timer that calls the timer_callback function every 1 second
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Counter for message numbering
        self.i = 0
        
        self.get_logger().info('Talker node has been started')
    
    def timer_callback(self):
        """
        Timer callback function that publishes a message.
        """
        msg = String()
        msg.data = f'Hello World: {self.i}'
        
        # Publish the message
        self.publisher.publish(msg)
        
        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function to initialize and run the talker node.
    """
    rclpy.init(args=args)
    
    talker_node = TalkerNode()
    
    try:
        # Spin the node so the callback function is called
        rclpy.spin(talker_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        talker_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 