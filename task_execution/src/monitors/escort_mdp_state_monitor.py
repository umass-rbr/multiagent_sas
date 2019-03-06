#!/usr/bin/env python
import json
import random

import rospy

from task_execution.msg import EscortMdpState
from mid_level_robot_planner.srv import GetLocationSrv
#from task_execution.srv import GetLocationSrv


# TODO Test this function 
def get_location():
    rospy.wait_for_service('/mid_level_planner/map_management/get_loc')

    try:
        get_location = rospy.ServiceProxy('mid_level_robot_planner/get_loc', GetLocationSrv)
        location_response = get_location()
        return location_response.curr_loc
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s", e)
        
    return False


# TODO Implement this function 
def with_person():
    return True


def main():
    rospy.init_node("escort_mdp_state_monitor", anonymous=True)

    rospy.loginfo("Info[escort_mdp_state_monitor.main]: Instantiating the escort_mdp_state_monitor node...")

    publisher = rospy.Publisher("monitor/escort_mdp_state", EscortMdpState, queue_size=10)
    rate = rospy.Rate(rospy.get_param("/escort_mdp_state_monitor/rate"))

    while not rospy.is_shutdown():
        message = EscortMdpState()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "/escort_mdp_state_monitor"
        message.location = get_location()
        message.with_person = with_person()

        rospy.loginfo(message)
        publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
