#!/usr/bin/env python
import json
import random

import rospy

import roslib; roslib.load_manifest('mid_level_robot_planner')
from mid_level_robot_planner.srv import GetLocationSrv

from task_execution.msg import DeliveryMdpState


# TODO Test this function 
def get_location():
    rospy.wait_for_service('/mid_level_planner/map_management/get_loc')

    try:
        get_location = rospy.ServiceProxy('/mid_level_planner/map_management/get_loc', GetLocationSrv)
        location_response = get_location()
        return location_response.curr_loc
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s", e)
        
    return False


# TODO Implement this function 
def has_package():
    return False


def main():
    rospy.init_node("delivery_mdp_state_monitor", anonymous=True)

    rospy.loginfo("Info[delivery_mdp_state_monitor.main]: Instantiating the delivery_mdp_state_monitor node...")

    publisher = rospy.Publisher("monitor/delivery_mdp_state", DeliveryMdpState, queue_size=10)
    rate = rospy.Rate(rospy.get_param("/delivery_mdp_state_monitor/rate"))

    while not rospy.is_shutdown():
        message = DeliveryMdpState()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "/delivery_mdp_state_monitor"
        message.location = get_location()
        message.has_package = has_package()

        rospy.loginfo(message)
        publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
