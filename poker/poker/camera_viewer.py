#!/usr/bin/env python3

# camera_view.py
#
# Kyle Vickstrom
#
# Poker camera subscriber node
# Depending on the topic this is subscribed to, we can watch what
# the camera sees
# the filtered image at a couple different points down the pipeline
# to verify the camera is detecting the correct cards

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node

from cv_bridge import CvBridge, CvBridgeError
import cv2

from sensor_msgs.msg import Image
from poker_msgs.msg import GameLog

class CameraViewer(Node):
    def __init__(self):
        super().__init__("camera_viewer")
        self.logging = True
        self.pubLog = self.create_publisher(GameLog, 'game_log', 10)
        cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera View", 640, 480)
        self.bridge = CvBridge()
        sensor_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10
        )
        self.imgSub = self.create_subscription(Image, "edit_image", self.showImg, sensor_qos)

    def showImg(self, imgmsg):
        """
        Display the image
        """
        try:
            # color
            # cv_img = self.bridge.imgmsg_to_cv2(imgmsg, desired_encoding="bgr8")
            # bw
            cv_img = self.bridge.imgmsg_to_cv2(imgmsg, desired_encoding='mono8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        if not cv2.getWindowProperty("Camera View", cv2.WND_PROP_VISIBLE) >= 1:
            cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Camera View", 640, 480)

        cv2.imshow("Camera View", cv_img)
        cv2.waitKey(1)

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

def main(args=None):
    rclpy.init(args=args)
    viewer = CameraViewer()
    rclpy.spin(viewer)
    rclpy.shutdown()

if __name__ == "__main__":
    main()