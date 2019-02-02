#!/usr/bin/env python
import json
import random

import rospy

from task_execution.msg import EscortMdpState


# TODO Implement this function 
def get_location():
    with open('/home/justin/Documents/Development/catkin_ws/src/task_execution/src/tmp/lgrc.json') as f:
        map = json.load(f)
        return random.choice(map["locations"].keys())


# TODO Implement this function 
def with_person():
    return True


def main():
    rospy.init_node("escort_mdp_state_monitor", anonymous=True)
    rospy.loginfo("Info[escort_mdp_state_monitor.main]: Instantiated the escort_mdp_state_monitor node")

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
