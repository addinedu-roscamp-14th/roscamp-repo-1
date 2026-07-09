#!/usr/bin/env python3

import os
from pathlib import Path
import yaml

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class UdpCameraBridge(Node):
    def __init__(self):
        super().__init__('udp_camera_bridge')

        self.declare_parameter('port', 5000)
        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('gst_pipeline', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('camera_info_yaml', 'config/main_camera/camera_info.yaml')
        self.declare_parameter('frame_id', 'camera_optical_frame')

        self.port = int(self.get_parameter('port').value)
        self.bind_address = str(self.get_parameter('bind_address').value)
        self.gst_pipeline = str(self.get_parameter('gst_pipeline').value).strip()
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.camera_info_yaml = self.resolve_yaml_path(str(self.get_parameter('camera_info_yaml').value))
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.image_pub = self.create_publisher(Image, self.image_topic, 10)
        self.info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)

        self.camera_info_msg = self.load_camera_info(self.camera_info_yaml)
        self._warned_size_mismatch = False

        Gst.init(None)

        self.pipeline_str = self.gst_pipeline or self.build_default_pipeline()

        self.get_logger().info(f'Opening UDP GStreamer pipeline:\n{self.pipeline_str}')

        self.pipeline = Gst.parse_launch(self.pipeline_str)
        self.appsink = self.pipeline.get_by_name('sink')

        if self.appsink is None:
            raise RuntimeError('Failed to get appsink')

        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect('message', self.on_bus_message)

        self.appsink.set_property('emit-signals', True)
        self.appsink.set_property('sync', False)
        self.appsink.set_property('max-buffers', 1)
        self.appsink.set_property('drop', True)
        self.appsink.connect('new-sample', self.on_new_sample)

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('Failed to start GStreamer pipeline')

        self.get_logger().info('UDP camera bridge started')
        self.get_logger().info(f'Publishing {self.image_topic} and {self.camera_info_topic}')

    def build_default_pipeline(self):
        return (
            f'udpsrc address={self.bind_address} port={self.port} '
            f'caps="application/x-rtp, media=(string)video, '
            f'encoding-name=(string)JPEG, payload=(int)26, clock-rate=(int)90000" ! '
            f'rtpjpegdepay ! '
            f'queue leaky=downstream max-size-buffers=1 ! '
            f'jpegdec ! '
            f'queue leaky=downstream max-size-buffers=1 ! '
            f'videoconvert ! '
            f'video/x-raw,format=RGB,width={self.width},height={self.height} ! '
            f'appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true'
        )

    def resolve_yaml_path(self, yaml_path):
        path = Path(yaml_path)
        if path.is_absolute():
            return path

        candidates = [Path.cwd() / path]
        module_path = Path(__file__).resolve()
        for parent in module_path.parents:
            candidates.append(parent / path)

        for candidate in candidates:
            if candidate.exists():
                return candidate

        raise FileNotFoundError(f'Camera info yaml not found: {yaml_path}')

    def load_camera_info(self, yaml_path):
        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f'Camera info yaml not found: {yaml_path}')

        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        msg = CameraInfo()
        msg.width = int(data['image_width'])
        msg.height = int(data['image_height'])
        msg.k = [float(x) for x in data['camera_matrix']['data']]
        msg.d = [float(x) for x in data['distortion_coefficients']['data']]
        msg.r = [float(x) for x in data['rectification_matrix']['data']]
        msg.p = [float(x) for x in data['projection_matrix']['data']]
        msg.distortion_model = data.get('distortion_model', 'plumb_bob')

        return msg

    def on_new_sample(self, sink):
        sample = sink.emit('pull-sample')

        if sample is None:
            return Gst.FlowReturn.OK

        self.publish_sample(sample)
        return Gst.FlowReturn.OK

    def publish_sample(self, sample):
        buffer = sample.get_buffer()
        caps = sample.get_caps()
        structure = caps.get_structure(0)

        width = int(structure.get_value('width'))
        height = int(structure.get_value('height'))

        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            self.get_logger().warn('Failed to map buffer')
            return

        try:
            data = bytes(map_info.data)
        finally:
            buffer.unmap(map_info)

        expected_size = width * height * 3
        if len(data) != expected_size:
            self.get_logger().warn(
                f'Unexpected frame size: got {len(data)}, expected {expected_size}'
            )
            return

        if (
            not self._warned_size_mismatch and
            (width != self.camera_info_msg.width or height != self.camera_info_msg.height)
        ):
            self.get_logger().warn(
                f'Image size ({width}x{height}) differs from camera_info '
                f'({self.camera_info_msg.width}x{self.camera_info_msg.height})'
            )
            self._warned_size_mismatch = True

        now = self.get_clock().now().to_msg()

        image_msg = Image()
        image_msg.header.stamp = now
        image_msg.header.frame_id = self.frame_id
        image_msg.height = height
        image_msg.width = width
        image_msg.encoding = 'rgb8'
        image_msg.is_bigendian = 0
        image_msg.step = width * 3
        image_msg.data = data

        info_msg = CameraInfo()
        info_msg.header.stamp = now
        info_msg.header.frame_id = self.frame_id
        info_msg.width = width
        info_msg.height = height
        info_msg.distortion_model = self.camera_info_msg.distortion_model
        info_msg.d = list(self.camera_info_msg.d)
        info_msg.k = list(self.camera_info_msg.k)
        info_msg.r = list(self.camera_info_msg.r)
        info_msg.p = list(self.camera_info_msg.p)

        self.image_pub.publish(image_msg)
        self.info_pub.publish(info_msg)

    def on_bus_message(self, bus, message):
        message_type = message.type

        if message_type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            self.get_logger().error(f'GStreamer error: {error.message}')
            if debug:
                self.get_logger().error(debug)
        elif message_type == Gst.MessageType.WARNING:
            warning, debug = message.parse_warning()
            self.get_logger().warn(f'GStreamer warning: {warning.message}')
            if debug:
                self.get_logger().warn(debug)
        elif message_type == Gst.MessageType.EOS:
            self.get_logger().warn('GStreamer reached EOS')

    def destroy_node(self):
        if hasattr(self, 'pipeline') and self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpCameraBridge()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
