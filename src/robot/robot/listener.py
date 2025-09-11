#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
    """
    A simple listener node that subscribes to string messages from a topic.
    """
    
    def __init__(self):
        super().__init__('listener')
        
        # Create a subscriber for String messages on the 'chatter' topic
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            1
        )
        
        self.get_logger().info('Listener node has been started')
    
    def listener_callback(self, msg):
        """
        Callback function that is called when a message is received.
        """
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function to initialize and run the listener node.
    """
    rclpy.init(args=args)
    
    listener_node = ListenerNode()
    
    try:
        # Spin the node so the callback function is called
        rclpy.spin(listener_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        listener_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 