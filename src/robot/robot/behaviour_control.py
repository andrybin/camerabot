"""ROS 2 node: run trained behaviour-clone policy (ONNX) on camera stack, publish cmd_vel."""

from __future__ import annotations

import os
from collections import deque

import cv2
import numpy as np
import onnx
import onnxruntime as ort
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image as RosImage

_IMAGE_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
)


def _dim_value(dim) -> int | None:
    if dim.dim_param:
        return None
    return int(dim.dim_value)


def _read_onnx_io_shapes(m: onnx.ModelProto) -> tuple[list[int | None], list[int | None]]:
    """Return (input_shape, output_shape) with None for dynamic axes."""
    g = m.graph
    inp0 = g.input[0]
    shape_in = [_dim_value(d) for d in inp0.type.tensor_type.shape.dim]
    out0 = g.output[0]
    shape_out = [_dim_value(d) for d in out0.type.tensor_type.shape.dim]
    return shape_in, shape_out


def _meta_bool(meta: dict[str, str], key: str, default: bool) -> bool:
    if key not in meta:
        return default
    v = meta[key].strip().lower()
    return v in ('1', 'true', 'yes')


def _build_stacked_chw(
    past_chw: list[np.ndarray],
    num_past: int,
    cur_chw: np.ndarray,
) -> np.ndarray:
    """Match behavclon/train.py stacking: past entries are (3,H,W), cur_chw current frame."""
    tail = past_chw[-num_past:] if past_chw else []
    out = list(tail)
    while len(out) < num_past:
        oldest = out[0] if out else cur_chw
        out.insert(0, oldest.copy())
    out.append(cur_chw)
    return np.concatenate(out, axis=0)


def _ros_image_to_chw(bridge: CvBridge, msg: RosImage, img_size: int) -> np.ndarray:
    """RGB CHW float32 [0,1], square resize (matches torchvision Resize + ToTensor)."""
    bgr = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    rgb = cv2.resize(rgb, (img_size, img_size), interpolation=cv2.INTER_LINEAR)
    chw = np.ascontiguousarray(rgb.transpose(2, 0, 1).astype(np.float32)) * (1.0 / 255.0)
    return chw


class BehaviourControlNode(Node):
    def __init__(self):
        super().__init__('behaviour_control')

        self.declare_parameter('weights_path', '')
        self.declare_parameter('camera_topic', '/camera/image_color')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('max_lin_speed', 0.5)
        self.declare_parameter('max_ang_speed', 0.5)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('num_past', 4)
        self.declare_parameter('img_size', 128)
        self.declare_parameter('publish_rate_hz', 0.0)

        weights_path = os.path.expanduser(
            self.get_parameter('weights_path').get_parameter_value().string_value
        )
        self._camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self._max_lin = float(self.get_parameter('max_lin_speed').get_parameter_value().double_value)
        self._max_ang = float(self.get_parameter('max_ang_speed').get_parameter_value().double_value)
        device_str = self.get_parameter('device').get_parameter_value().string_value.strip().lower()
        param_num_past = int(self.get_parameter('num_past').get_parameter_value().integer_value)
        param_img_size = int(self.get_parameter('img_size').get_parameter_value().integer_value)
        rate_hz = float(self.get_parameter('publish_rate_hz').get_parameter_value().double_value)

        if not weights_path:
            raise RuntimeError(
                'Parameter weights_path must point to an ONNX file '
                '(export with: python -m behavclon.export --ckpt <weights.pt>).'
            )
        if not weights_path.lower().endswith('.onnx'):
            raise RuntimeError(
                f'weights_path must be a .onnx file, got {weights_path!r}. '
                'Export the checkpoint to ONNX and point weights_path to that file.'
            )
        if not os.path.isfile(weights_path):
            raise RuntimeError(f'ONNX model not found: {weights_path}')

        model_onnx = onnx.load(weights_path)
        meta = {p.key: p.value for p in model_onnx.metadata_props}
        self._head_tanh = _meta_bool(meta, 'head_tanh', True)
        self._target_space_atanh = _meta_bool(meta, 'target_space_atanh', False)

        shape_in, _ = _read_onnx_io_shapes(model_onnx)
        if len(shape_in) != 4:
            raise RuntimeError(f'Expected NCHW ONNX input, got rank {len(shape_in)}')
        _, in_ch, h_in, w_in = shape_in
        if in_ch is None or in_ch % 3 != 0:
            raise RuntimeError(f'ONNX input channels must be known and a multiple of 3, got {in_ch!r}')
        num_past_from_model = in_ch // 3 - 1
        if h_in is None or w_in is None or h_in != w_in:
            raise RuntimeError(f'ONNX input must have fixed square H=W, got H={h_in!r} W={w_in!r}')
        img_size = int(h_in)

        if param_num_past != num_past_from_model or param_img_size != img_size:
            self.get_logger().warn(
                f'ROS params num_past={param_num_past} img_size={param_img_size} differ from '
                f'ONNX model (num_past={num_past_from_model}, img_size={img_size}); using model values.'
            )
        self._num_past = num_past_from_model
        self._img_size = img_size

        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        if device_str == 'cuda':
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        else:
            providers = ['CPUExecutionProvider']
        self._session = ort.InferenceSession(weights_path, sess_options=so, providers=providers)
        self._in_name = self._session.get_inputs()[0].name
        self._out_name = self._session.get_outputs()[0].name

        self._bridge = CvBridge()
        self._past_buf: deque = deque(maxlen=self._num_past)

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 1)
        self.create_subscription(RosImage, self._camera_topic, self._image_callback, _IMAGE_QOS)

        self._latest_twist = Twist()
        self._have_cmd = False
        if rate_hz > 0.0:
            self._timer = self.create_timer(1.0 / rate_hz, self._timer_publish)
        else:
            self._timer = None

        self.get_logger().info(
            f'behaviour_control (ONNX): model={weights_path!r} providers={self._session.get_providers()} '
            f'num_past={self._num_past} img_size={self._img_size} camera={self._camera_topic!r} '
            f'cmd_vel={cmd_vel_topic!r} timer_hz={rate_hz} '
            f'head_tanh={self._head_tanh} target_space_atanh={self._target_space_atanh}'
        )

    def _image_callback(self, msg: RosImage):
        try:
            cur_chw = _ros_image_to_chw(self._bridge, msg, self._img_size)
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        past_list = list(self._past_buf)
        stacked = _build_stacked_chw(past_list, self._num_past, cur_chw)
        x = np.expand_dims(stacked, axis=0).astype(np.float32, copy=False)

        outs = self._session.run([self._out_name], {self._in_name: x})
        pred = np.asarray(outs[0]).reshape(-1)
        if pred.size < 2:
            self.get_logger().error('Model output has fewer than 2 values')
            return
        self.get_logger().debug(f'model prediction: {pred[:2].tolist()}')

        if self._head_tanh:
            lx_n = float(pred[0])
            wz_n = float(pred[1])
        elif self._target_space_atanh:
            lx_n = float(np.tanh(pred[0]))
            wz_n = float(np.tanh(pred[1]))
        else:
            lx_n = max(-1.0, min(1.0, float(pred[0])))
            wz_n = max(-1.0, min(1.0, float(pred[1])))

        twist = Twist()
        twist.linear.x = lx_n * self._max_lin
        twist.angular.z = wz_n * self._max_ang

        self._past_buf.append(cur_chw.copy())

        if self._timer is None:
            self._cmd_pub.publish(twist)
        else:
            self._latest_twist = twist
            self._have_cmd = True

    def _timer_publish(self):
        if self._have_cmd:
            self._cmd_pub.publish(self._latest_twist)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviourControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
