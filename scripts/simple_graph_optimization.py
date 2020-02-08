#! /usr/bin/python

import rospy
import sys

from graph_optimization.rosbot_interface import RosbotInterface

if __name__ == "__main__":
    rospy.init_node("husarion_graph_optimization", anonymous=False)

    rosbot = RosbotInterface()

    rospy.logdebug("Initialization complete.")

    rospy.spin()
