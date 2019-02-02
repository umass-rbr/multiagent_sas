#!/usr/bin/env python
import json
import random

import rospy

from task_execution.msg import DeliveryMdpState


# TODO Implement this function 
def get_location():
    with open('/home/justin/Documents/Development/catkin_ws/src/task_execution/src/tmp/lgrc.json') as f:
        map = json.load(f)
        return random.choice(map["locations"].keys())


# TODO Implement this function 
def has_package():
    return True


def main():
    rospy.init_node("delivery_mdp_state_monitor", anonymous=True)
    rospy.loginfo("Info[delivery_mdp_state_monitor.main]: Instantiated the delivery_mdp_state_monitor node")

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
