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


    '''
    This function from edje electronics tutorial, i want to rewrite it myself before finishing the camera
    '''
    def preprocess_card(self, contour, image):
        """Uses contour to find information about the query card. Isolates rank
        and suit images from the card."""

        # Initialize new Query_card object
        qCard = Query_card()

        qCard.contour = contour

        # Find perimeter of card and use it to approximate corner points
        peri = cv2.arcLength(contour,True)
        approx = cv2.approxPolyDP(contour,0.01*peri,True)
        pts = np.float32(approx)
        qCard.corner_pts = pts

        # Find width and height of card's bounding rectangle
        x,y,w,h = cv2.boundingRect(contour)
        qCard.width, qCard.height = w, h

        # Find center point of card by taking x and y average of the four corners.
        average = np.sum(pts, axis=0)/len(pts)
        cent_x = int(average[0][0])
        cent_y = int(average[0][1])
        qCard.center = [cent_x, cent_y]

        # Warp card into 200x300 flattened image using perspective transform
        qCard.warp = flattener(image, pts, w, h)

        # Grab corner of warped card image and do a 4x zoom
        Qcorner = qCard.warp[0:CORNER_HEIGHT, 0:CORNER_WIDTH]
        Qcorner_zoom = cv2.resize(Qcorner, (0,0), fx=4, fy=4)

        # Sample known white pixel intensity to determine good threshold level
        white_level = Qcorner_zoom[15,int((CORNER_WIDTH*4)/2)]
        thresh_level = white_level - CARD_THRESH
        if (thresh_level <= 0):
            thresh_level = 1
        retval, query_thresh = cv2.threshold(Qcorner_zoom, thresh_level, 255, cv2. THRESH_BINARY_INV)
        
        # Split in to top and bottom half (top shows rank, bottom shows suit)
        Qrank = query_thresh[20:185, 0:128]
        Qsuit = query_thresh[186:336, 0:128]

        # Find rank contour and bounding rectangle, isolate and find largest contour
        dummy, Qrank_cnts, hier = cv2.findContours(Qrank, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        Qrank_cnts = sorted(Qrank_cnts, key=cv2.contourArea,reverse=True)

        # Find bounding rectangle for largest contour, use it to resize query rank
        # image to match dimensions of the train rank image
        if len(Qrank_cnts) != 0:
            x1,y1,w1,h1 = cv2.boundingRect(Qrank_cnts[0])
            Qrank_roi = Qrank[y1:y1+h1, x1:x1+w1]
            Qrank_sized = cv2.resize(Qrank_roi, (RANK_WIDTH,RANK_HEIGHT), 0, 0)
            qCard.rank_img = Qrank_sized

        # Find suit contour and bounding rectangle, isolate and find largest contour
        dummy, Qsuit_cnts, hier = cv2.findContours(Qsuit, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        Qsuit_cnts = sorted(Qsuit_cnts, key=cv2.contourArea,reverse=True)
        
        # Find bounding rectangle for largest contour, use it to resize query suit
        # image to match dimensions of the train suit image
        if len(Qsuit_cnts) != 0:
            x2,y2,w2,h2 = cv2.boundingRect(Qsuit_cnts[0])
            Qsuit_roi = Qsuit[y2:y2+h2, x2:x2+w2]
            Qsuit_sized = cv2.resize(Qsuit_roi, (SUIT_WIDTH, SUIT_HEIGHT), 0, 0)
            qCard.suit_img = Qsuit_sized

        return qCard

    def flattener(image, pts, w, h):
        """Flattens an image of a card into a top-down 200x300 perspective.
        Returns the flattened, re-sized, grayed image.
        See www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/"""
        temp_rect = np.zeros((4,2), dtype = "float32")
        
        s = np.sum(pts, axis = 2)

        tl = pts[np.argmin(s)]
        br = pts[np.argmax(s)]

        diff = np.diff(pts, axis = -1)
        tr = pts[np.argmin(diff)]
        bl = pts[np.argmax(diff)]

        # Need to create an array listing points in order of
        # [top left, top right, bottom right, bottom left]
        # before doing the perspective transform

        if w <= 0.8*h: # If card is vertically oriented
            temp_rect[0] = tl
            temp_rect[1] = tr
            temp_rect[2] = br
            temp_rect[3] = bl

        if w >= 1.2*h: # If card is horizontally oriented
            temp_rect[0] = bl
            temp_rect[1] = tl
            temp_rect[2] = tr
            temp_rect[3] = br

        # If the card is 'diamond' oriented, a different algorithm
        # has to be used to identify which point is top left, top right
        # bottom left, and bottom right.
        
        if w > 0.8*h and w < 1.2*h: #If card is diamond oriented
            # If furthest left point is higher than furthest right point,
            # card is tilted to the left.
            if pts[1][0][1] <= pts[3][0][1]:
                # If card is titled to the left, approxPolyDP returns points
                # in this order: top right, top left, bottom left, bottom right
                temp_rect[0] = pts[1][0] # Top left
                temp_rect[1] = pts[0][0] # Top right
                temp_rect[2] = pts[3][0] # Bottom right
                temp_rect[3] = pts[2][0] # Bottom left

            # If furthest left point is lower than furthest right point,
            # card is tilted to the right
            if pts[1][0][1] > pts[3][0][1]:
                # If card is titled to the right, approxPolyDP returns points
                # in this order: top left, bottom left, bottom right, top right
                temp_rect[0] = pts[0][0] # Top left
                temp_rect[1] = pts[3][0] # Top right
                temp_rect[2] = pts[2][0] # Bottom right
                temp_rect[3] = pts[1][0] # Bottom left
                
            
        maxWidth = 200
        maxHeight = 300

        # Create destination array, calculate perspective transform matrix,
        # and warp card image
        dst = np.array([[0,0],[maxWidth-1,0],[maxWidth-1,maxHeight-1],[0, maxHeight-1]], np.float32)
        M = cv2.getPerspectiveTransform(temp_rect,dst)
        warp = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
        warp = cv2.cvtColor(warp,cv2.COLOR_BGR2GRAY) 

        return warp

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

class Query_card:
    """Structure to store information about query cards in the camera image."""

    def __init__(self):
        self.contour = [] # Contour of card
        self.width, self.height = 0, 0 # Width and height of card
        self.corner_pts = [] # Corner points of card
        self.center = [] # Center point of card
        self.warp = [] # 200x300, flattened, grayed, blurred image
        self.rank_img = [] # Thresholded, sized image of card's rank
        self.suit_img = [] # Thresholded, sized image of card's suit
        self.best_rank_match = "Unknown" # Best matched rank
        self.best_suit_match = "Unknown" # Best matched suit
        self.rank_diff = 0 # Difference between rank image and best matched train rank image
        self.suit_diff = 0 # Difference between suit image and best matched train suit image



def main(args=None):
    rclpy.init(args=args)
    node = ImageManip()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()