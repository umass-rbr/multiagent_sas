#!/usr/bin/env python
import json
import random

import rospy

from task_execution.msg import DeliveryMdpState


# TODO test this function 
def get_location():
    rospy.wait_for_service('mid_level_planner/map_management/get_loc')
    try:
        get_location = rospy.ServiceProxy('mid_level_planner/map_management/get_loc',GetLocationSrv)
        location_response = get_location()
        return location_response.curr_loc
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
    return rospy.ServiceProxy

    # with open('/home/justin/Documents/Development/catkin_ws/src/task_execution/src/tmp/lgrc.json') as world_map_file:
    #     world_map = json.load(world_map_file)
    #     return random.choice(world_map["locations"].keys())


# TODO Implement this function 
def has_package():
    return True


def main():
    rospy.loginfo("Info[delivery_mdp_state_monitor.main]: Instantiating the delivery_mdp_state_monitor node...")
    rospy.init_node("delivery_mdp_state_monitor", anonymous=True)

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
