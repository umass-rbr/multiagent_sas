#!/usr/bin/env python
import rospy

from task_assignment.msg import WorldState


# TODO Implement this function 
def get_tasks():
    pass


# TODO Implement this function 
def get_robot_statuses():
    pass


def main():
    rospy.init_node("world_state_monitor", anonymous=True)
    rospy.loginfo("Info[world_state_monitor.main]: Instantiated the world_state_monitor node")

    publisher = rospy.Publisher("monitor/world_state", WorldState)

    #rate = rospy.Rate(rospy.get_param("/world_state_monitor/rate"))
    
    while not rospy.is_shutdown():
        state_msg = WorldState()
        state_msg.tasks = get_tasks()
        state_msg.robot_status = get_robot_statuses()

        rospy.loginfo(state_msg)
        publisher.publish(state_msg)

        rate.sleep()


if __name__ == '__main__':
    main()
