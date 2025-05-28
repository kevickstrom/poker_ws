#!/usr/bin/env python3

# poker_camera.py
#
# Kyle Vickstrom
#
# Poker camera publisher node
# publishes image stream from camera when parameter is set

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node

from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image
from poker_msgs.msg import GameLog


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("poker_camera")

        # TODO: params
        self.camera_index = 0
        self.width = 1920
        self.height = 1080
        self.fps = 30
        self.logging = True

        # TODO: make this a service call to connect?
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            raise RuntimeError("could not open camera")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_PROP_FPS, self.fps)

        sensor_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10
        )
        self.imgPub = self.create_publisher(Image, 'raw_camera', sensor_qos)

        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / self.fps, self.frame)

    def frame(self):
        """
        Publish an image from the webcam
        """
        ok, frame = self.cap.read()
        if not ok:
            self.log("Camera read Failed")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.imgPub.publish(msg)

    def destroy_node(self):
        """
        Close connection to the camera
        """
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

    def log(self, msg):
        """
        log to topic and to curses terminal window
        """
        if self.logging:
            newmsg = GameLog()
            newmsg.stamp = self.get_clock().now().to_msg()
            newmsg.node_name = self.get_name()
            newmsg.content = msg
            self.pubLog.publish(newmsg)
        self.get_logger().info(msg)

def main():
    rclpy.init(args=args)
    node = CameraPublisher
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
