#!/usr/bin/env python
import json
import random
import os
import rospy

import roslib;

# TODO Fix this
from mid_route.msg import TocMdpState


def get_nearest_location():
    return "shlomoOffice"


def in_control():
    return True


def main():
    rospy.init_node("toc_mdp_state_monitor", anonymous=True)

    rospy.loginfo("Info[toc_mdp_state_monitor.main]: Instantiating the toc_mdp_state_monitor node...")

    publisher = rospy.Publisher("monitor/toc_mdp_state_monitor", TocMdpState, queue_size=10)
    rate = rospy.Rate(rospy.get_param("/toc_mdp_state_monitor/rate"))

    while not rospy.is_shutdown():
        message = TocMdpState()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "/toc_mdp_state_monitor"
        message.location = get_nearest_location()
        message.in_control = in_control()

        publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
