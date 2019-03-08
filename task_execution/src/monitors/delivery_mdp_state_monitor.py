#!/usr/bin/env python
import json
import random

import rospy

import roslib; roslib.load_manifest('mid_level_robot_planner')

from mid_level_robot_planner.srv import GetLocationSrv
from task_execution.msg import DeliveryMdpState

current_file_path = os.path.dirname(os.path.realpath(__file__))

def get_nearest_location():
    rospy.wait_for_service('/mid_level_planner/map_management/get_loc')

    try:
        get_location_proxy = rospy.ServiceProxy('/mid_level_planner/map_management/get_loc', GetLocationSrv)
        location_response = get_location_proxy()
        return location_response.curr_loc, location_response.dist_to_loc
    except rospy.ServiceException as e:
        rospy.loginfo("Could not get location: %s", e)
        
    return False


# TODO Implement this function 
def has_package():
    return False


def main():
    rospy.init_node("delivery_mdp_state_monitor", anonymous=True)

    rospy.loginfo("Info[delivery_mdp_state_monitor.main]: Instantiating the delivery_mdp_state_monitor node...")

    publisher = rospy.Publisher("monitor/delivery_mdp_state", DeliveryMdpState, queue_size=10)
    rate = rospy.Rate(rospy.get_param("/delivery_mdp_state_monitor/rate"))

    current_location = None
    world_map = json.load( open(current_file_path + '../tmp/LGRC3_plan_map.json') )

    while not rospy.is_shutdown():
        message = DeliveryMdpState()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "/delivery_mdp_state_monitor"

        location, distance = get_nearest_location()
        if current_location is None or (distance < 3.0 and location in world_map['paths'][current_location]): #3.0 is a placeholder that should be changed.
            current_location = location

        message.location = current_location
        message.has_package = has_package()

        rospy.loginfo(message)
        publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
