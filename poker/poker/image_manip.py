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
CARD_MAX_AREA = 120000
CARD_MIN_AREA = 25000

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
        cnts_sort, cnt_is_card = self.find_cards(thresh)
        self.log("contoured image")
        if len(cnts_sort) == 0:
            return
            # published threshold image
            self.log("thresh")
            msg = self.bridge.cv2_to_imgmsg(thresh, encoding='mono8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.imgPub.publish(msg)
        else:
            # draw contours
            color_mask = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
            cv2.drawContours(color_mask, cnts_sort, -1, (0, 255, 0), 3)
            # published a card
            msg = self.bridge.cv2_to_imgmsg(color_mask, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            self.imgPub.publish(msg)
            self.log("Contour")


    def find_cards(self, thresh):
        """
        Take thresholded image and find largest countours (cards)
        """
        # get all contours, find indices of largest to smallest
        cnts,heir = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        index_sort = sorted(range(len(cnts)), key=lambda i : cv2.contourArea(cnts[i]), reverse=True)
        if len(cnts) == 0:
            return [], []
        # arrange into sorted
        cnts_sort = []
        heir_sort = []
        cnt_is_card = np.zeros(len(cnts),dtype=int)
        for i in index_sort:
            cnts_sort.append(cnts[i])
            heir_sort.append(heir[0][i])
        
        # get rid of anthing thats probably not a card
        # keep contours smaller than max size, larger than min size, has 4 corners
        for i in range((len(cnts_sort))):
            size = cv2.contourArea(cnts_sort[i])
            peri = cv2.arcLength(cnts_sort[i], True)
            approx = cv2.approxPolyDP(cnts_sort[i], 0.01*peri, True)

            if ((size < CARD_MAX_AREA) and (size > CARD_MIN_AREA) and (heir_sort[i][3] == -1) and (len(approx) == 4)):
                cnt_is_card[i] = 1

        return cnts_sort, cnt_is_card



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