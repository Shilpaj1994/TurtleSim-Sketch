#!/usr/bin/env python
"""
Program to get threshold values for canny edge detection in a node
Author: Shilpaj Bhalerao
Date: Sep 08, 2020
"""

import rospy
import time

def main():
	while not rospy.is_shutdown():
		gains = rospy.get_param('/Thresholds')
		min_val = gains[0]
		max_val = gains[1]
		# print("Received values: ", min_val, max_val)


if __name__ == "__main__":
	try:
		time.sleep(1)
		main()
	except KeyboardInterrupt:
		exit()