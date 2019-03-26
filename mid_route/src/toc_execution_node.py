#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)


def main():

    rospy.loginfo("Info[toc_execution_node.main]: Instantiating the toc_execution node...")
    rospy.init_node('toc_execution_node', anonymous=True)

    rospy.Subscriber('monitor/toc_mdp_state', String, callback)

    rospy.loginfo("Info[toc_execution_node.main]: Spinning...")
    rospy.spin()


if __name__ == '__main__':
    main()
