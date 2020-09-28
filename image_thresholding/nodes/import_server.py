#!/usr/bin/env python
"""
Program to get image capture information - path and capture type in a node
Author: Shilpaj Bhalerao
Date: Sep 08, 2020
"""

import rospy

from dynamic_reconfigure.server import Server
from image_thresholding.cfg import importConfig

def callback(config, level):
    # print(config.img_path, config.Capture) #, config.Address, config.Camera)
    # rospy.set_param('IMAGEPATH', config.img_path)
    # rospy.set_param('Capture', config.Capture)
    return config


if __name__ == "__main__":
    rospy.init_node("image_thresholding", anonymous = False)

    srv = Server(importConfig, callback)
    rospy.spin()
