#!/usr/bin/env python
import rospy

from task_execution.msg import EscortMdpState


# TODO Implement a way to a get a location
def get_current_location():
    return True


def main():
    rospy.init_node("escort_mdp_state_monitor", anonymous=True)
    rospy.loginfo("Info[escort_mdp_state_monitor.main]: Instantiated the escort_mdp_state_monitor node")

    publisher = rospy.Publisher("monitor/escort_mdp_state", EscortMdpState, queue_size=10)

    rate = rospy.Rate(rospy.get_param("/monitor/rate"))
    
    while not rospy.is_shutdown():
        msg = EscortMdpState()
        msg.location = get_current_location()

        rospy.loginfo(msg)
        publisher.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
