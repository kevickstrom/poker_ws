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

from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image
from poker_msgs.msg import GameLog

class CameraViewer(Node):
    def __init__(self):
        super().__init__("camera_viewer")
        self.logging = True

        self.bridge = CvBridge()
        self.imgSub = self.create_subscription(Image, "raw_camera", self.showImg,1)

    def showImg(self, imgmsg):
        """
        Display the image
        """
        openCVImage = self.bridge.imgmsg_to_cv2(imgmsg)
        cv2.imshow("Camera View", openCVImage)
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