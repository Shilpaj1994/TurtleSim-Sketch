#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from image_thresholding.cfg import thresholdsConfig

def callback(config, level):
    print("Max = ", config.max, "\tMin = ", config.min)
    rospy.set_param('Thresholds', [config.min, config.max])
    rospy.set_param('Activate', config.Start)
    return config


if __name__ == "__main__":
    rospy.init_node("image_thresholding", anonymous = False)

    srv = Server(thresholdsConfig, callback)
    rospy.spin()
