"""Direct ESP32-CAM MJPEG to ROS images; no phone relay required."""
import queue
import threading
import time
from urllib.parse import urlsplit
from urllib.request import ProxyHandler, build_opener

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from .mjpeg import JpegBuffer


class Camera(Node):
    def __init__(self):
        super().__init__('wheelz_camera')
        descriptor = ParameterDescriptor(read_only=True)
        self.url = self.declare_parameter('camera_url', '', descriptor).value
        self.frame = self.declare_parameter('frame_id', 'camera_optical_frame', descriptor).value
        parts = urlsplit(self.url)
        if parts.scheme not in ('http', 'https') or not parts.hostname:
            raise ValueError('camera_url must be the working ESP32 MJPEG stream URL')
        self.bridge = CvBridge()
        self.raw_pub = self.create_publisher(Image, 'camera/image_raw', qos_profile_sensor_data)
        self.jpeg_pub = self.create_publisher(
            CompressedImage, 'camera/image_raw/compressed', qos_profile_sensor_data)
        self.frames = queue.Queue(maxsize=1)
        self.quit = threading.Event()
        self.error = ''
        self.last_error = ''
        self.thread = threading.Thread(target=self.receive, daemon=True, name='wheelz-camera')
        self.thread.start()
        self.create_timer(0.02, self.publish_frame)
        self.create_timer(2.0, self.report_error)

    def receive(self):
        opener = build_opener(ProxyHandler({}))
        while not self.quit.is_set():
            try:
                with opener.open(self.url, timeout=2.0) as stream:
                    parser = JpegBuffer()
                    while not self.quit.is_set():
                        chunk = stream.read1(8192)
                        if not chunk:
                            raise OSError('Camera stream ended')
                        for jpeg in parser.feed(chunk):
                            try:
                                self.frames.get_nowait()
                            except queue.Empty:
                                pass
                            self.frames.put_nowait((jpeg, time.monotonic()))
                            self.error = ''
            except Exception as error:
                self.error = str(error)
                self.quit.wait(1.0)

    def report_error(self):
        if self.error and self.error != self.last_error:
            self.get_logger().warning(self.error)
        self.last_error = self.error

    def publish_frame(self):
        try:
            jpeg, received = self.frames.get_nowait()
        except queue.Empty:
            return
        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.error = 'Invalid JPEG received'
            return
        stamp = (self.get_clock().now() - Duration(
            seconds=max(0.0, time.monotonic() - received))).to_msg()
        compressed = CompressedImage()
        compressed.header.stamp, compressed.header.frame_id = stamp, self.frame
        compressed.format, compressed.data = 'bgr8; jpeg compressed bgr8', jpeg
        self.jpeg_pub.publish(compressed)
        raw = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        raw.header = compressed.header
        self.raw_pub.publish(raw)

    def destroy_node(self):
        self.quit.set()
        self.thread.join(timeout=4.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Camera()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
