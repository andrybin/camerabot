import base64
import io
from pathlib import Path

import cv2
import numpy as np
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
        self.declare_parameter('prompt', 'What do you see?')
        self.declare_parameter('max_width', 640)
        self.declare_parameter('max_height', 480)
        self.declare_parameter('quality', 85)
        self.declare_parameter('linear_speed_constant', 0.15)
        self.declare_parameter('angular_speed_constant', 0.2)
        self.declare_parameter('ollama_timeout', 300)
        self.declare_parameter('max_tokens', 256)
        self.declare_parameter('add_last_command', True)
        self.declare_parameter('add_last_scene', True)

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.service_url = self.get_parameter('service_url').get_parameter_value().string_value
        self.model = self.get_parameter('model').get_parameter_value().string_value
        self.prompt = self.get_parameter('prompt').get_parameter_value().string_value
        self.max_width = self.get_parameter('max_width').get_parameter_value().integer_value
        self.max_height = self.get_parameter('max_height').get_parameter_value().integer_value
        self.quality = self.get_parameter('quality').get_parameter_value().integer_value
        self.linear_speed_constant = self.get_parameter('linear_speed_constant').get_parameter_value().double_value
        self.angular_speed_constant = self.get_parameter('angular_speed_constant').get_parameter_value().double_value
        self.ollama_timeout = self.get_parameter('ollama_timeout').get_parameter_value().integer_value
        self.max_tokens = self.get_parameter('max_tokens').get_parameter_value().integer_value
        self.add_last_command =  self.get_parameter('add_last_command').get_parameter_value().bool_value
        self.add_last_scene = self.get_parameter('add_last_scene').get_parameter_value().bool_value

        self.command_txt_list = ['<STOP>', '<FORWARD>', '<BACK>', '<LEFT>', '<RIGHT>']
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False
        self.processing = False
        self.msg_count = 0
        self.count_scale = 0.7
        self.count_thickness = 1
        self.messages_pub = self.create_publisher(String, 'vlm/result', qos)
        self.debug_pub = self.create_publisher(String, 'vlm/debug', qos)
        self.annotated_image_pub = self.create_publisher(RosImage, 'vlm/annotated_image', qos)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.create_subscription(RosImage, self.camera_topic, self.image_callback, qos)
        self.history = []

        self.get_logger().info(f'VLM Control started with with {self.model} model. Subscribed to {self.camera_topic}')

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
            if len(self.history):
                if self.add_last_command:
                    prompt = f"Last step command: {self.history[-1]['last_command']}" + "\n" + prompt
                if self.add_last_scene:
                    prompt = f"Last step scene: {self.history[-1]['last_scene']}" + "\n" + prompt
            prompt += self.prompt
            self.debug_pub.publish(String(data=prompt))

            chat_payload = {
                'model': self.model,
                'stream': False,
                'options': {'num_predict': self.max_tokens},
                'messages': [{
                    'role': 'user',
                    'content': prompt,
                    'images': [img_b64]
                }]
            }
            try:
                self.get_logger().info(f'Sending request to {self.service_url}')
                resp = requests.post(self.service_url, json=chat_payload, timeout=self.ollama_timeout)
                self.get_logger().info(f'Response from {self.service_url}: {resp.status_code}')
                used_generate = False
                if resp.status_code == 404 and self.service_url.endswith('/api/chat'):
                    generate_url = f"{self.service_url.rsplit('/api/chat', 1)[0]}/api/generate"
                    generate_payload = {
                        'model': self.model,
                        'stream': False,
                        'prompt': prompt,
                        'images': [img_b64],
                        'options': {'num_predict': self.max_tokens},
                    }
                    self.get_logger().warning(
                        f'{self.service_url} returned 404, falling back to {generate_url}'
                    )
                    resp = requests.post(generate_url, json=generate_payload, timeout=self.ollama_timeout)
                    self.get_logger().info(f'Response from {generate_url}: {resp.status_code}')
                    used_generate = True
                if resp.status_code == 200:
                    data = resp.json()
                    if used_generate:
                        response_text = data.get('response', '') or str(data)
                    else:
                        message = data.get('message', {}).get('content', {})
                        response_text = message if message else str(data)
                    
                    self.messages_pub.publish(String(data=response_text))

                    command_parsed = self.parse_command(response_text)
                    scene_parsed = self.parse_scene_description(response_text)
                    self.history.append(
                        {"last_command": command_parsed[1:-1].lower(), "last_scene": scene_parsed})

                    self.msg_count += 1
                    annotated = self.annotate_image_with_text(response_text, command_parsed)
                    if annotated is not None:
                        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                        self.annotated_image_pub.publish(annotated_msg)
                    # Optionally control robot if special tokens are present
                    self.control_robot(command_parsed)
                else:
                    response_text = f'Error: {resp.status_code} - {resp.text}'
                    self.debug_pub.publish(String(data=response_text))
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

    def wrap_text(self, text, font, scale, thickness, max_width):
        """Wrap text by words, preserving explicit newline breaks."""
        if text == "":
            return [""]
        lines = []
        for segment in text.split('\n'):
            words = segment.split()
            if not words:
                lines.append("")
                continue
            current = ""
            for word in words:
                trial = word if not current else f"{current} {word}"
                width, _ = cv2.getTextSize(trial, font, scale, thickness)[0]
                if width <= max_width or not current:
                    current = trial
                else:
                    lines.append(current)
                    current = word
            if current:
                lines.append(current)
        return lines

    def wrap_text_by_chars(self, text, font, scale, thickness, max_width):
        """Wrap text by characters, preserving explicit newline breaks."""
        lines = []
        current = ""
        for ch in text:
            if ch == '\n':
                lines.append(current)
                current = ""
                continue
            trial = f"{current}{ch}"
            width, _ = cv2.getTextSize(trial, font, scale, thickness)[0]
            if width <= max_width or not current:
                current = trial
            else:
                lines.append(current)
                current = ch
        if current:
            lines.append(current)
        return lines

    def annotate_image_with_text(self, text, command_txt=None):
        try:
            cv_image = np.zeros((100, 720, 3), dtype=np.uint8)
            img_h, img_w = cv_image.shape[:2]
            img_with_cmd = cv_image.copy()
            font = cv2.FONT_HERSHEY_SIMPLEX

            if command_txt:
                center_x = img_w // 2
                center_y = img_h // 2
                arrow_len = max(1, img_h - 1)
                arrow_thickness = max(4, int(img_w / 240))
                arrow_tip = 0.3
                color = (255, 150, 0)

                def arrow_endpoints(cmd):
                    if cmd == '<FORWARD>':
                        start = (center_x, img_h - 1)
                        end = (center_x, img_h - 1 - arrow_len)
                    elif cmd == '<BACK>':
                        start = (center_x, 0)
                        end = (center_x, arrow_len)
                    elif cmd == '<LEFT>':
                        start = (img_w - 1, center_y)
                        end = (img_w - 1 - arrow_len, center_y)
                    elif cmd == '<RIGHT>':
                        start = (0, center_y)
                        end = (arrow_len, center_y)
                    else:
                        return None
                    return start, end

                directions = {
                    '<FORWARD>': (0, -1),
                    '<BACK>': (0, 1),
                    '<LEFT>': (-1, 0),
                    '<RIGHT>': (1, 0),
                }

                if command_txt == '<STOP>':
                    for cmd in ('<FORWARD>', '<BACK>', '<LEFT>', '<RIGHT>'):
                        endpoints = arrow_endpoints(cmd)
                        if endpoints is None:
                            continue
                        start, end = endpoints
                        cv2.arrowedLine(
                            img_with_cmd,
                            start,
                            end,
                            color,
                            arrow_thickness,
                            cv2.LINE_AA,
                            tipLength=arrow_tip,
                        )
                elif command_txt in directions:
                    endpoints = arrow_endpoints(command_txt)
                    if endpoints is not None:
                        start, end = endpoints
                        cv2.arrowedLine(
                            img_with_cmd,
                            start,
                            end,
                            color,
                            arrow_thickness,
                            cv2.LINE_AA,
                            tipLength=arrow_tip,
                        )

            count_text = f"cmd #{self.msg_count}"
            count_margin = max(6, int(img_w * 0.02))
            (count_w, count_h), count_base = cv2.getTextSize(
                count_text, font, self.count_scale, self.count_thickness
            )
            count_y = min(img_h - 1, count_margin + count_h)
            count_x = min(img_w - 1, count_margin)
            cv2.putText(
                img_with_cmd,
                count_text,
                (count_x, count_y),
                font,
                self.count_scale,
                (255, 255, 255),
                self.count_thickness,
                cv2.LINE_AA,
            )
            panel_h = img_h*4
            panel = np.zeros((panel_h, img_w, 3), dtype=np.uint8)

            margin = max(10, int(img_w * 0.02))
            max_text_width = img_w - 2 * margin
            thickness = 1
            min_scale = 0.2
            max_scale = 1.5
            scale = max_scale

            lines = []
            while scale >= min_scale:
                lines = self.wrap_text(text, font, scale, thickness, max_text_width)
                if len(lines) == 1 and lines[0] == "":
                    break
                (_, line_height), baseline = cv2.getTextSize("Ag", font, scale, thickness)
                line_spacing = int(line_height * 0.3)
                total_height = len(lines) * (line_height + line_spacing) + baseline
                if total_height <= panel_h - 2 * margin:
                    break
                scale -= 0.1

            if scale < min_scale:
                scale = min_scale
                lines = self.wrap_text_by_chars(text, font, scale, thickness, max_text_width)
                (_, line_height), baseline = cv2.getTextSize("Ag", font, scale, thickness)
                line_spacing = int(line_height * 0.2)
            else:
                (_, line_height), baseline = cv2.getTextSize("Ag", font, scale, thickness)
                line_spacing = int(line_height * 0.3)

            y = margin + line_height
            for line in lines:
                if y + baseline > panel_h - margin:
                    break
                cv2.putText(panel, line, (margin, y), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)
                y += line_height + line_spacing

            combined = np.vstack([img_with_cmd, panel])
            return combined
        except Exception as e:
            self.get_logger().error(f'annotate_image_with_text failed: {e}')
            return None

    def parse_command(self, sentence):
        """
        Finds the last occurrence of any word from the given list in the sentence.
        
        Args:
            sentence (str): The input sentence.
            word_list (list): List of target words to search for.
        
        Returns:
            tuple: (word, index) of the last occurring word from the list, or (None, -1) if not found.
        """
        lines = sentence.splitlines()
        final_word = "<STOP>"
        self.get_logger().info(f'Parse VLM answer: try to find target command...')
        for line in lines:
            for target_word in self.command_txt_list:
                if target_word in line:
                    self.get_logger().info(f'Found occurrence of {target_word}')
                    final_word = target_word
        if final_word is None:
            self.get_logger().warning(f'NO command found!')
                
        return final_word

    def parse_scene_description(self, sentence):
        """
        Extracts the scene description from a line starting with '???'.
        Returns the last found description if multiple are present.
        """
        lines = sentence.splitlines()
        scene = ""
        self.get_logger().info('Parse VLM answer: try to find scene description...')
        for line in lines:
            if '???' not in line:
                continue
            marker_index = line.find('???')
            candidate = line[marker_index + 3:].strip()
            if candidate:
                self.get_logger().info('Found scene description line')
                scene = candidate
        if not scene:
            self.get_logger().warning('NO scene description found!')
        return scene


    def control_robot(self, command_txt: str):
        command = Twist()
        command.linear.x = 0.0
        command.angular.z = 0.0

        if command_txt in self.command_txt_list:
            if command_txt == '<STOP>':
                command.linear.x = 0.0
                self.get_logger().info('STOP signal received')        
            if command_txt == '<FORWARD>':
                command.linear.x = self.linear_speed_constant
                self.get_logger().info('FORWARD signal received')
            if command_txt == '<BACK>':
                command.linear.x = -self.linear_speed_constant
                self.get_logger().info('BACKWARD signal received')
            if command_txt == '<LEFT>':
                command.angular.z = -self.angular_speed_constant
                self.get_logger().info('TURN_LEFT signal received')
            if command_txt == '<RIGHT>':
                command.angular.z = self.angular_speed_constant
                self.get_logger().info('TURN_RIGHT signal received')
            
        self.cmd_vel_publisher.publish(command)
        return command


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


