#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def main():

    rospy.init_node('toc_mdp_state_monitor', anonymous=True)
    rospy.loginfo(
        "Info[toc_mdp_state_monitor.main]: Instantiating the toc_mdp_state_monitor node...")

    publisher = rospy.Publisher('monitor/toc_mdp_state', String, queue_size=10)

    rate = rospy.Rate(10)  # 10hz is default tutorial rate
    # rate = rospy.Rate(rospy.get_param("/delivery_mdp_state_monitor/rate"))

    # get current location of robot from some topic

    # fetch action from MDP

    while not rospy.is_shutdown():

        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        publisher.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    main()
