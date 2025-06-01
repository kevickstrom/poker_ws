#!/usr/bin/env python3

# image_manip.py
#
# Kyle Vickstrom
#
# Poker camera subscriber manipulator node
# This node handles incoming images and converts to greyscale etc for
# the next node to pick cards out of

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from sensor_msgs.msg import Image
from poker_msgs.msg import GameLog

BKG_THRESH = 20

class ImageManip(Node):
    def __init__(self):
        super().__init__("image_manip")
        self.logging = True
        self.pubLog = self.create_publisher(GameLog, 'game_log', 10)
        self.bridge = CvBridge()
        sensor_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10
        )
        self.imgPub = self.create_publisher(Image, 'edit_image', sensor_qos)
        self.imgSub = self.create_subscription(Image, "raw_camera", self.editImg, sensor_qos)

    def editImg(self, img):
        """
        Edit incoming image and publish to new camera feed topic
        """
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # greyed
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        # blurred
        blur = cv2.GaussianBlur(gray, (5,5), 0)

        # thresholding
        img_w, img_h = np.shape(cv_img)[:2]
        bkg_lvl = gray[int(img_h/100)][int(img_w/2)]
        thresh_lvl = bkg_lvl + BKG_THRESH

        retval, thresh = cv2.threshold(blur, thresh_lvl, 255, cv2.THRESH_BINARY)

        msg = self.bridge.cv2_to_imgmsg(thresh, encoding='mono8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.imgPub.publish(msg)
        self.log("thresh")

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
    node = ImageManip()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()