#!/usr/bin/env python
import rospy

from task_execution.msg import DeliveryMdpState


# TODO Implement this function 
def get_location():
    return "shlomoOffice"


# TODO Implement this function 
def has_package():
    return True


def main():
    rospy.init_node("delivery_mdp_state_monitor", anonymous=True)
    rospy.loginfo("Info[delivery_mdp_state_monitor.main]: Instantiated the delivery_mdp_state_monitor node")

    publisher = rospy.Publisher("monitor/delivery_mdp_state", DeliveryMdpState, queue_size=10)

    rate = rospy.Rate(rospy.get_param("/delivery_mdp_state_monitor/rate"))
    
    while not rospy.is_shutdown():
        msg = DeliveryMdpState()
        msg.location = get_location()
        msg.has_package = has_package()

        rospy.loginfo(msg)
        publisher.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
