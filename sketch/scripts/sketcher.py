#!/usr/bin/env python3.8
"""
Program to sketch the contours in a turtle-sim
Author: Shilpaj Bhalerao
Date: Aug 30, 2020
"""

import rospy
import numpy as np
import math
import sys
import time
import multiprocessing
import cv2
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import *
from std_srvs.srv import Empty
from Turtle import *
# from sketch.msg import points, groups, segments
# from sketch.srv import test
import itertools

INTERNAL = False


class Robot:
    def __init__(self, parallel=1):
        self.count = 1
        self.list = []
        self.processes = []
        self.PARALLEL = parallel
        self.start = False
        self.image = None
        self.activate = True
        self.draw_contour = False

        # Variables to access contours data
        self.collection_points = []
        self.collection_segments = []
        self.collection_groups = []

        # Initialize sketcher node
        rospy.init_node('sketcher', anonymous=False)

        # Publisher
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Subscriber
        # rospy.Subscriber('contours', segments, self.callback)

        # Rate to control frequency of operation of node
        self.rate = rospy.Rate(1)  # 10hz

        # Reset the turtle-sim simulator
        reset_sim()

        time.sleep(2)  # Delay to make sure dynamic reconfigure is ready

        while not rospy.is_shutdown():
            if INTERNAL:  # Test the code without the data exchange from outside node
                center = [5.54, 5.54]
                origin = [4.0, 4.0]
                l_top = [4.0, 9.0]
                r_top = [9.0, 9.0]
                l_bottom = [4.0, 4.0]
                r_bottom = [9.0, 4.0]

                self.start = True
                self.contours = [[(1, 1), (4, 2), (3, 3,), (4, 9), origin],
                                 [(9, 1), (1, 2), (1, 3,), (9, 9), (9, 4), origin],
                                 [center, l_top, r_top, r_bottom, l_bottom, center]]
                self.numbers = len(self.contours)

            elif not INTERNAL:  # Test the code with other nodes data
                # Wait for user to select image from a path or capture an image using camera
                if self.activate and not self.draw_contour:
                    self.load_img()

                # After selecting image, start drawing contours and adjust the threshold
                if self.activate and rospy.get_param('ImagePath/Capture'):
                    self.find_contours()

            # After finalizing contours, spawn an army of turtles and start sketching
            if self.start:
                # Spawn turtle at the first point of the contours
                self.spawn_source()

                if self.PARALLEL == 0:  # If sequential implementation mode is selected
                    self.trace()
                elif self.PARALLEL == 1:  # If parallel implementation mode is selected
                    rospy.loginfo("Sketching ....")

                    # Code for Multi-processing
                    for i in range(self.numbers):
                        p = multiprocessing.Process(target=self.trace_parallel, args=[i])
                        p.start()
                        self.processes.append(p)
                    for process in self.processes:
                        process.join()

                # Remove the turtles after sketching is done
                self.kill_destination()
                rospy.loginfo("Press Ctrl+C to terminate the program")
                rospy.spin()
            else:
                pass

    # ------------------------------- Functions related to image processing ----------------------------
    def load_img(self):
        """
        Load image using either:
        - Path of an image
        - Using a camera
        """
        while self.image is None:
            types = rospy.get_param('ImagePath/CaptureType')  # Check either camera or path is selected

            # If image path is selected
            if types == 0:
                image_path = rospy.get_param('ImagePath/img_path')  # Read the path
                self.image = cv2.imread(image_path)  # Load an image from the path

                # If image is loaded properly, start finding edges
                if self.image is not None:
                    rospy.loginfo("Address selected")
                    self.draw_contour = True
                    break

            # If camera is selected
            elif types == 1:
                cap = cv2.VideoCapture(0)
                while True:
                    # Check if frame is selected
                    condition = rospy.get_param('ImagePath/Capture')

                    # Capture frame-by-frame
                    ret, frame = cap.read()

                    # Display the resulting frame
                    cv2.imshow('Capture Image to Sketch', frame)
                    cv2.waitKey(1)

                    # If frame is selected properly, start finding edges
                    if condition:
                        self.image = frame
                        cv2.destroyWindow('Capture Image to Sketch')
                        self.draw_contour = True
                        break

        # Resize image since turtle-sim dimensions are 500 x 500
        self.image = cv2.resize(self.image, (500, 500))
        rospy.loginfo("Image Loaded")
        rospy.loginfo("Select minimum and maximum threshold")

    def find_contours(self):
        """
        Find edges and contours in an image
        """
        # Grayscale
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Find Canny edges using threshold inputs
        gains = rospy.get_param('/Thresholds')
        min_val = gains[0]
        max_val = gains[1]
        edged = cv2.Canny(gray, min_val, max_val)

        cv2.imshow("Select Threshold Values", edged)
        cv2.waitKey(1)

        # If edges are selected, start drawing sketch
        start = rospy.get_param('/Activate')

        if start:
            # Find contours
            contours, hierarchy = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            # Convert contours to accessible data format
            self.contour_to_accessible_format(contours)

            # Draw all contours on a blank image
            blank = np.zeros(shape=[500, 500, 3], dtype=np.uint8)
            cv2.drawContours(blank, contours, -1, (255, 255, 255), 1)

    def contour_to_accessible_format(self, contours):
        """
        Function to convert the opencv contours to accessible data format
        - Collection of x,y coordinates is a point
        - Collection of points is a segment
        - Collection of segments is a group
        - Collection of groups is a contour
        :param contours: Extracted contours from the image
        :type contours: opencv contours
        """
        for i in range(len(contours)):  # For all the contours
            if len(contours[i]) > 10:  # If they have more than 10 points
                for j in range(len(contours[i])):  # Find the x,y coordinates of all the points
                    x_cord = (contours[i][j][0][0] * 11) / 500  # Convert points within (11, 11) i.e. size of turtle-sim
                    y_cord = (contours[i][j][0][1] * 11) / 500

                    # Save these coordinates in a list named collection_points and subtract y from 11 to make upright
                    # image
                    self.collection_points.append((x_cord, 11.0 - y_cord))

                # Collect all line segments in collection_segments list
                self.collection_segments.append(self.collection_points)
                self.collection_points = []

            # Collect all groups in collection_groups list
            self.collection_groups.append(self.collection_segments)

        # Collection of all contours in contours
        self.contours = self.collection_segments

        # Total number of contours
        self.numbers = len(self.contours)

        # Reset and code flow
        self.collection_segments = []
        self.activate = False
        self.start = True

    # ------------------------------- Functions related to sketching turtle path ----------------------------
    def trace_parallel(self, j):
        """
        Function to trace the contours using multi-processing
        :param j: Number of multi-processes
        :type j: Int
        """
        # Take x,y coord. of next point in contour and teleport turtle to that point(for all contours simultaneously)
        for k in range(len(self.contours[j])):
            self.list[j].teleport(self.contours[j][k][0], self.contours[j][k][1], 0.0)

    def trace(self):
        """
        Function to trace the contours using sequential programming
        """
        # Take x,y coord. of next point in contour and teleport turtle to that point(for one contour at a time)
        for j in range(self.numbers):
            for k in range(len(self.contours[j])):
                self.list[j].teleport(self.contours[j][k][0], self.contours[j][k][1], 0.0)

    # ------------------------------- Functions related to turtle actions ----------------------------
    def spawn_source(self):
        """
        Spawn multiple turtles on the first point of each contour
        """
        rospy.loginfo("Spawning an army of turtles to sketch your image")
        for i in range(self.numbers):
            self.list.append(Turtle(i + 1))
            if i == 0:
                self.list[0].set_pen(0)
                self.list[0].teleport(self.contours[i][0][0], self.contours[i][0][1], 0.0)
                self.list[0].set_pen(1)
            else:
                self.list[i].spawn(self.contours[i][0][0], self.contours[i][0][1], 0.0)

    def kill_destination(self):
        """
        Remove the turtles from simulation at the end of sketching
        """
        for i in range(self.numbers):
            self.list[i].kill_turtle()


def reset_sim():
    """
    Function to reset the simulator
    """
    try:
        reset_serv = rospy.ServiceProxy('/reset', Empty)
        reset_serv()
    except rospy.ServiceException as e:
        rospy.loginfo("Service execution failed: %s" + str(e))


if __name__ == '__main__':
    try:
        turtle = Robot(int(sys.argv[1]))
    except KeyboardInterrupt:
        exit()
