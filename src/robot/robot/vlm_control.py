import base64
import io
from pathlib import Path

import cv2
import rclpy
import requests
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from PIL import Image
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String

qos = QoSProfile(
    depth=1,  # Keep only the last message
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
)

class VlmControl(Node):
    def __init__(self):
        super().__init__('vlm_control')

        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_color')
        self.declare_parameter('service_url', 'http://localhost:11434/api/chat')
        self.declare_parameter('model', 'qwen2.5vl:latest')
        self.declare_parameter('prompt', 'if you see a CUP print <GO>, if you see a red duck print <STOP>')
        self.declare_parameter('max_width', 480)
        self.declare_parameter('max_height', 480)
        self.declare_parameter('quality', 85)
        self.declare_parameter('period_s', 1.0)
        self.declare_parameter('linear_speed_constant', 0.15) #0.15 1
        self.declare_parameter('angular_speed_constant', 0.2) #0.2 0.3

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.service_url = self.get_parameter('service_url').get_parameter_value().string_value
        self.model = self.get_parameter('model').get_parameter_value().string_value
        self.prompt = self.get_parameter('prompt').get_parameter_value().string_value
        self.max_width = self.get_parameter('max_width').get_parameter_value().integer_value
        self.max_height = self.get_parameter('max_height').get_parameter_value().integer_value
        self.quality = self.get_parameter('quality').get_parameter_value().integer_value
        self.period_s = self.get_parameter('period_s').get_parameter_value().double_value
        self.linear_speed_constant = self.get_parameter('linear_speed_constant').get_parameter_value().double_value
        self.angular_speed_constant = self.get_parameter('angular_speed_constant').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False
        self.processing = False
        self.messages_pub = self.create_publisher(String, 'vlm/result', qos)
        self.debug_pub = self.create_publisher(String, 'vlm/debug', qos)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.create_subscription(RosImage, self.camera_topic, self.image_callback, qos)
        self.history = []

        # Synchronous HTTP client; no async loop
        self.get_logger().info(f'VLM Control started. Subscribed to {self.camera_topic}')

    def image_callback(self, msg: RosImage):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    # def timer_callback(self):
        print(f'image_received: {self.image_received}')
        if not self.image_received or self.latest_image is None:
            return
        if self.processing:
            return
        self.processing = True
        # Process synchronously
        self.process_latest_image()

    def process_latest_image(self):
        try:
            img_b64 = self.encode_and_optimize(self.latest_image,
                                               (self.max_width, self.max_height),
                                               self.quality)
            if img_b64 is None:
                self.get_logger().warning('Image encoding failed')
                return

            prompt = ""
            # prompt += "\nSTATES HISTORY (-3 -> -2 -> -1 -> current):\n"
            # prompt += "\n".join([f"step {i-3}:\n{txt}" for i, txt in enumerate(self.history[-3:])])
            prompt += "\nPROMPT:\n" + self.prompt
            # self.messages_pub.publish(String(data=f"{prompt}"))

            payload = {
                'model': self.model,
                'stream': False,
                'messages': [{
                    'role': 'user',
                    'content': prompt,
                    'images': [img_b64]
                }]
            }
            try:
                self.get_logger().info(f'Sending request to {self.service_url}')
                resp = requests.post(self.service_url, json=payload, timeout=60)
                self.get_logger().info(f'Response from {self.service_url}: {resp.status_code}')
                if resp.status_code == 200:
                    data = resp.json()
                    message = data.get('message', {}).get('content', {})
                    text = message if message else str(data)
                    self.history.append(text)
                    self.messages_pub.publish(String(data=text))
                    # Optionally control robot if special tokens are present
                    self.control_robot(text)
                else:
                    text = f'Error: {resp.status_code} - {resp.text}'
                    self.debug_pub.publish(String(data=text))
            except Exception as e:
                self.get_logger().error(f'HTTP request failed: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
        finally:
            self.processing = False

    def encode_and_optimize(self, cv_image, max_size, quality):
        try:
            # Convert BGR to RGB
            rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(rgb)
            if pil_img.mode in ('RGBA', 'LA', 'P'):
                pil_img = pil_img.convert('RGB')
            # Resize
            if pil_img.size[0] > max_size[0] or pil_img.size[1] > max_size[1]:
                pil_img.thumbnail(max_size, Image.Resampling.LANCZOS)
            # Compress and convert to base64
            buffer = io.BytesIO()
            pil_img.save(buffer, format='JPEG', quality=quality, optimize=True)
            buffer.seek(0)
            return base64.b64encode(buffer.getvalue()).decode('utf-8')
        except Exception as e:
            self.get_logger().error(f'encode_and_optimize failed: {e}')
            return None

    def parse_command(self, sentence, word_list):
        """
        Finds the last occurrence of any word from the given list in the sentence.
        
        Args:
            sentence (str): The input sentence.
            word_list (list): List of target words to search for.
        
        Returns:
            tuple: (word, index) of the last occurring word from the list, or (None, -1) if not found.
        """
        line = sentence.splitlines()[-1]
        
        for target_word in word_list:
            if target_word in line:
                self.get_logger().info(f'Parse VLM answer: Found occurrence of {target_word}')
                return target_word
                
        return None


    def control_robot(self, response_text: str):
        command = Twist()
        command.linear.x = 0.0
        command.angular.z = 0.0

        last_command = self.parse_command(response_text, ['<STOP>', '<FORWARD>', '<BACK>', '<LEFT>', '<RIGHT>'])

        if last_command is not None:
            if last_command == '<STOP>':
                command.linear.x = 0.0
                self.get_logger().info('STOP signal received')        
            if last_command == '<FORWARD>':
                command.linear.x = self.linear_speed_constant
                self.get_logger().info('FORWARD signal received')
            if last_command == '<BACK>':
                command.linear.x = -self.linear_speed_constant
                self.get_logger().info('BACKWARD signal received')
            if last_command == '<LEFT>':
                command.angular.z = -self.angular_speed_constant
                self.get_logger().info('TURN_LEFT signal received')
            if last_command == '<RIGHT>':
                command.angular.z = self.angular_speed_constant
                self.get_logger().info('TURN_RIGHT signal received')
            
        self.cmd_vel_publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = VlmControl()
    node.prompt=Path('src/robot/resource/vlm_prompt.txt').read_text()
    try:
        print('spinning')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


