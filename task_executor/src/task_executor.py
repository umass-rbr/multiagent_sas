#!/usr/bin/env python
import rospy

import problem_selector
from task_executor.msg import DeliveryMdpState, EscortMdpState, TaskAssignment, TaskExecutorAction

DURATION = 10
ROBOT_ID = 42

rospy.Subscriber("task_assigner/task_assignment", TaskAssignment, task_executor, queue_size=1)
rospy.Subscriber("monitor/delivery_mdp_state", DeliveryMdpState, delivery_mdp_state_callback)
rospy.Subscriber("monitor/escort_mdp_state", EscortMdpState, escort_mdp_state_callback)

action_publisher = rospy.Publisher("task_executor/task_executor_action", TaskExecutorAction, queue_size=1)

escort_mdp_state = None
delivery_mdp_state = None


def escort_mdp_state_callback(state):
    escort_mdp_state = state


def delivery_mdp_state_callback(state):
    delivery_mdp_state = state


def get_current_state(task_type):
    if task_type == "delivery":
        return delivery_mdp_state

    if task_type == "escort":
        return escort_mdp_state


def task_executor(task_assignment):
    # TODO Parameterize the model
    # TODO Send details to the execute function
    # TODO Insert timeout
    # TODO Possibly change the policy?
    print("here")
    # if task_assignment.robot_id == ROBOT_ID:
    #     problem = problem_selector.get(task_assignment)
        
    #     problem.initialize()
    #     policy,state_map,action_map = problem.solve()
        
    #     current_state = get_current_state(task_assignment.task_type)

    #     while not rospy.is_shutdown():
    #         new_state = get_current_state(task_assignment.task_type)
            
    #         if new_state != current_state:
    #             current_state = new_state
    #             current_action = action_map[policy[state_map[current_state]]]

    #             msg = TaskExecutorAction()
    #             msg.action = current_action

    #             rospy.loginfo(msg)
    #             action_publisher.publish(msg)

    #         rospy.sleep(DURATION)


def main():
    rospy.init_node("task_executor", anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    main()
