import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import openai
import base64
import cv2
from cv_bridge import CvBridge
import time
from collections import deque


class OpenAIAPINode(Node):
    def __init__(self, linear_speed_constant=0.05):
        self.linear_speed_constant = linear_speed_constant
        super().__init__('openai_api_node')
        
        # Initialize OpenAI client
        self.openai_client = openai.OpenAI()
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Subscribe to camera topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_color',
            self.image_callback,
            1
        )
        
        # Publisher for robot movement commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            1
        )
        
        # Timer to process images periodically (every 2 seconds)
        self.timer = self.create_timer(2.0, self.process_image)
        
        # Store the latest image
        self.latest_image = None
        self.image_received = False
        
        # Conversation management
        self.conversation_history = deque(maxlen=20)  # Keep last 20 messages
        self.robot_state = {
            'position': 'unknown',
            'last_action': 'none',
            'target_visible': False,
            'frames_processed': 0,
            'conversation_id': int(time.time())
        }
        
        # Initialize conversation with system message
        self.initialize_conversation()
        
        self.get_logger().info('OpenAI API Node started')
        self.get_logger().info('Subscribed to camera topic: /camera/image_color')
    
    def initialize_conversation(self):
        """Initialize the conversation with system context"""
        system_message = {
            "role": "system",
            "content": """You are a small robot navigating in a Webots simulation environment.

IMPORTANT RULES:
1. You can GO forward or STOP
2. Your goal is to get close to yellow duck
3. Visible size of yellow duck must be approximately 90% of the frame size
4. Always give an estimation of the distance to the yellow duck
5. Always give an estimation of the visible size of the yellow duck
6. Always explain your decision before giving your command
7. Use '<GO>' to move forward or '<STOP>' to stop
8. Consider your previous actions and the current situation
9. If you're getting too close, STOP. If you need to get closer, GO
01. Be consistent with your previous decisions unless the situation clearly changes

Current conversation ID: {conversation_id}""".format(conversation_id=self.robot_state['conversation_id'])
        }
        
        self.conversation_history.append(system_message)
        self.get_logger().info(f'Conversation initialized with ID: {self.robot_state["conversation_id"]}')
    
    def image_callback(self, msg):
        """Callback function to receive camera images"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            self.image_received = True
            self.get_logger().debug('Image received successfully')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def update_robot_state(self, response_text, action_taken):
        """Update robot state based on the response and action taken"""
        self.robot_state['last_action'] = action_taken
        self.robot_state['frames_processed'] += 1
        
        # Update position based on action
        if action_taken == 'GO':
            self.robot_state['position'] = 'moving_forward'
        elif action_taken == 'STOP':
            self.robot_state['position'] = 'stopped'
        
        # Log state update
        self.get_logger().info(f'Robot state updated: {action_taken}, Total frames: {self.robot_state["frames_processed"]}')
    
    def create_context_message(self):
        """Create a context message with current robot state and conversation info"""
        context = f"""Current Robot State:
- Position: {self.robot_state['position']}
- Last Action: {self.robot_state['last_action']}
- Frames Processed: {self.robot_state['frames_processed']}
- Conversation ID: {self.robot_state['conversation_id']}

Remember: You are continuing the same conversation. Consider your previous decisions and maintain consistency."""
        
        return {
            "role": "user",
            "content": context
        }
    
    def process_image(self):
        """Process the latest image and send to OpenAI API with conversation context"""
        if not self.image_received or self.latest_image is None:
            self.get_logger().debug('No image available for processing')
            return
        
        try:
            # Encode image to base64
            self.get_logger().info(f'Processing image with shape: {self.latest_image.shape}')
            _, buffer = cv2.imencode('.jpg', self.latest_image)
            image_base64 = base64.b64encode(buffer).decode("utf-8")
            
            # Create the current frame message
            current_frame_message = {
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_base64}"}},
                    {"type": "text", "text": "What do you see in this frame? Explain your decision and add to your answer '<GO>' or '<STOP>'"}
                ]
            }
            
            # Build messages array with conversation history
            messages = list(self.conversation_history)
            
            # Add context message if this isn't the first frame
            if self.robot_state['frames_processed'] > 0:
                context_message = self.create_context_message()
                messages.append(context_message)
            
            # Add current frame
            messages.append(current_frame_message)
            
            # Send to OpenAI API with full conversation context
            response = self.openai_client.chat.completions.create(
                model="gpt-4o",
                messages=messages,
                max_tokens=400
            )
            
            # Get the response text
            response_text = response.choices[0].message.content
            self.get_logger().info(f'OpenAI Response (Frame {self.robot_state["frames_processed"] + 1}): {response_text}')
            
            # Add current frame and response to conversation history
            self.conversation_history.append(current_frame_message)
            self.conversation_history.append({
                "role": "assistant",
                "content": response_text
            })
            
            # Parse response and control robot
            action_taken = self.control_robot(response_text)
            
            # Update robot state
            self.update_robot_state(response_text, action_taken)
            
        except Exception as e:
            self.get_logger().error(f'Error calling OpenAI API: {str(e)}')
    
    def control_robot(self, response_text):
        """Control robot based on OpenAI response and return the action taken"""
        command = Twist()
        action_taken = 'STOP'
        
        if '<GO>' in response_text:
            # Move forward
            command.linear.x = self.linear_speed_constant
            command.angular.z = 0.0
            self.get_logger().info('Moving forward - GO signal received')
            action_taken = 'GO'
        if '<STOP>' in response_text:
            # Stop
            command.linear.x = 0.0
            command.angular.z = 0.0
            self.get_logger().info('Stopping - STOP signal received')
            action_taken = 'STOP'
        
        # Publish the command
        self.cmd_vel_publisher.publish(command)
        
        return action_taken
    
    def get_conversation_summary(self):
        """Get a summary of the current conversation"""
        return {
            'conversation_id': self.robot_state['conversation_id'],
            'total_frames': self.robot_state['frames_processed'],
            'conversation_length': len(self.conversation_history),
            'current_state': self.robot_state
        }


def main(args=None):
    rclpy.init(args=args)
    node = OpenAIAPINode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Print conversation summary before shutting down
        summary = node.get_conversation_summary()
        node.get_logger().info(f'Conversation Summary: {summary}')
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()