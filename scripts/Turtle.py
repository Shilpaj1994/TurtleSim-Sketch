#!/usr/bin/env python3.8
"""
Code for Turtles class
Author: Shilpaj Bhalerao
Date: Sep 03, 2020
"""

import rospy
from turtlesim.srv import *
from std_srvs.srv import Empty


def reset_sim():
    """
    Function to Reset the Simulator
    """
    try:
        reset_serv = rospy.ServiceProxy('/reset', Empty)
        reset_serv()
    except rospy.ServiceException as e:
        rospy.loginfo("Service execution failed: %s" + str(e))


class Turtle:
    def __init__(self, i):
        self.name = 'turtle' + str(i)

    def spawn(self, x, y, theta):
        """
        Function to spawn turtles in the Turtle-sim
        :param x: x-position with respect to origin at bottom-left
        :type x: float
        :param y: y-position with respect to origin at bottom-left
        :type y: float
        :param theta: orientation with respect to x-axis
        :type theta: float between [0 to 3] OR [0 to -3]
        """
        try:
            serv = rospy.ServiceProxy('/spawn', Spawn)
            serv(x, y, theta, self.name)
        except rospy.ServiceException as e:
            rospy.loginfo("Service execution failed: %s" + str(e))

    def set_pen(self, flag=True):
        """
        Function to sketch the turtle movements
        :param flag: To turn sketching pen - ON[True]/OFF[False]
        :type flag: bool
        """
        try:
            if not flag:
                set_serv = rospy.ServiceProxy('/' + self.name + '/set_pen', SetPen)
                set_serv(0, 0, 0, 0, 1)
            elif flag:
                set_serv = rospy.ServiceProxy('/' + self.name + '/set_pen', SetPen)
                set_serv(255, 255, 255, 2, 0)
        except rospy.ServiceException as e:
            rospy.loginfo("Service execution failed: %s" + str(e))

    def teleport(self, x, y, theta):
        """
        Function to teleport the turtle
        :param x: x-position with respect to origin at bottom-left
        :type x: float
        :param y: y-position with respect to origin at bottom-left
        :type y: float
        :param theta: orientation with respect to x-axis
        :type theta: float between [0 to 3] OR [0 to -3]
        """
        try:
            serv = rospy.ServiceProxy('/' + self.name + '/teleport_absolute', TeleportAbsolute)
            serv(x, y, theta)
        except rospy.ServiceException as e:
            rospy.loginfo("Service execution failed: %s" + str(e))

    def kill_turtle(self):
        """
        Function to remove the turtle from Turtle-sim
        """
        try:
            serv = rospy.ServiceProxy('/kill', Kill)
            serv(self.name)
        except rospy.ServiceException as e:
            rospy.loginfo("Service execution failed: %s" + str(e))


if __name__ == '__main__':
    try:
        # Create a turtle
        turtle2 = Turtle(2)

        # Spawn the turtle at 3, 3, 0
        turtle2.spawn(3, 3, 0)

        # Teleport the turtle to 3, 9, 0
        turtle2.teleport(3, 9, 0)

        # Stop sketching turtle movement
        turtle2.set_pen(False)

        # Teleport the turtle to 5, 9, 0
        turtle2.teleport(5, 9, 0)

        # Start sketching turtle movement
        turtle2.set_pen(True)

        # Teleport the turtle to 9, 9, 0
        turtle2.teleport(9, 9, 0)
    except KeyboardInterrupt:
        exit()
