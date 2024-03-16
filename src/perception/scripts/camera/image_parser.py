#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg, math, random

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# WIDTH = 640
# HEIGHT = 480
# OFFSET = 340
# GAP = 40
WIDTH = 420
HEIGHT = 90
OFFSET = 250

class IMGParser:
    def __init__(self):
        rospy.init_node('img_parser', anonymous=True)
        self.img_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.callback)
        self.is_image = False
        self.img_bgr = None

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            os.system('clear')
            if not self.is_image:
                cv2.destroyAllWindows()
                print("[error] can't subscribe '/image_jpeg/compressed' topic")
            else:
                cv2.imshow('image', self.img_bgr)
                cv2.waitKey(1)
                print("Camera sensor was connected")

            rate.sleep()

    def callback(self, msg):
        self.is_image = True
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.img_bgr = self.process_img(self.img_bgr)

    def draw_lines(self, img, lines):
        global OFFSET
        for line in lines:
            x1, y1, x2, y2 = line[0]
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            img = cv2.line(img, (x1, y1+OFFSET), (x2, y2+OFFSET), color, 2)
        return img

    def get_line_params(self, lines):
        # sum of x, y, m
        x_sum = 0.0
        y_sum = 0.0
        m_sum = 0.0

        size = len(lines)
        if size == 0:
            return 0, 0

        for line in lines:
            x1, y1, x2, y2 = line[0]

            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)
        m = m_sum / size
        b = y_avg - m * x_avg

        return m, b

    def get_line_pos(self, img, lines, left=False, right=False):
        global WIDTH, HEIGHT
        global OFFSET

        m, b = self.get_line_params(lines)

        if m == 0 and b == 0:
            if left:
                pos = 0
            if right:
                pos = WIDTH
        else:
            y = HEIGHT / 2
            pos = (y - b) / m

            b += OFFSET
            x1 = int((HEIGHT - b) / float(m))
            x2 = int(((HEIGHT/2) - b) / float(m))

            cv2.line(img, (x1, HEIGHT), (x2, int(HEIGHT/2)), (255, 0,0), 3)

        return img, int(pos)

    def divide_left_right(self, lines):
        global WIDTH

        low_slope_threshold = 0
        high_slope_threshold = 10

        # calculate slope & filtering with threshold
        slopes = []
        new_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]

            if x2 - x1 == 0:
                slope = 0
            else:
                slope = float(y2-y1) / float(x2-x1)

            if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
                slopes.append(slope)
                new_lines.append(line[0])

        # divide lines left to right
        left_lines = []
        right_lines = []

        for j in range(len(slopes)):
            Line = new_lines[j]
            slope = slopes[j]

            x1, y1, x2, y2 = Line

            if (slope < 0) and (x2 < WIDTH/2 - 90):
                left_lines.append([Line.tolist()])
            elif (slope > 0) and (x1 > WIDTH/2 + 90):
                right_lines.append([Line.tolist()])

        return left_lines, right_lines

    def process_img(self, img):
        global WIDTH, HEIGHT
        global OFFSET

        # Grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Gaussian blur
        kernel_size = 5
        blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

        # Canny edge
        low_threshold = 60
        high_threshold = 70
        edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

        # ROI and HoughLinesP : 480, 640 -> 90, 420
        # roi = edge_img[250:340,110:-110]
        roi = edge_img[OFFSET : OFFSET + HEIGHT, 110 : -110]
        all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

        # divide left, right lines
        if all_lines is None:
            return 0, 640
        left_lines, right_lines = self.divide_left_right(all_lines)

        # get center of lines
        roi, lpos = self.get_line_pos(roi, left_lines, left=True)
        roi, rpos = self.get_line_pos(roi, right_lines, right=True)

        # draw lines
        roi = self.draw_lines(roi, left_lines)
        roi = self.draw_lines(roi, right_lines)
        roi = cv2.line(roi, (230, 235), (410, 235), (255,255,255), 2)

        return roi

if __name__ == '__main__':
    try:
        IMGParser = IMGParser()
    except rospy.ROSInterruptException:
        pass
