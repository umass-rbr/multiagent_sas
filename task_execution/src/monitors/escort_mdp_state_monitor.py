#!/usr/bin/env python
import rospy

from task_execution.msg import EscortMdpState


# TODO Implement this function 
def get_location():
    return "shlomoOffice"


# TODO Implement this function 
def with_person():
    return True


def main():
    rospy.init_node("escort_mdp_state_monitor", anonymous=True)
    rospy.loginfo("Info[escort_mdp_state_monitor.main]: Instantiated the escort_mdp_state_monitor node")

    publisher = rospy.Publisher("monitor/escort_mdp_state", EscortMdpState, queue_size=10)

    rate = rospy.Rate(rospy.get_param("/escort_mdp_state_monitor/rate"))
    
    while not rospy.is_shutdown():
        msg = EscortMdpState()
        msg.location = get_location()
        msg.with_person = with_person()

        rospy.loginfo(msg)
        publisher.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
