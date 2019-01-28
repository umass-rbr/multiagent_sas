#!/usr/bin/env python
import rospy

from mdps.delivery_mdp import DeliveryMDP
from mdps.escort_mdp import EscortMDP
from task_assignment.msg import TaskAssignmentAction
from task_execution.msg import DeliveryMdpState, EscortMdpState, TaskExecutionAction

action_publisher = rospy.Publisher("task_execution/task_execution_action", TaskExecutionAction, queue_size=1)

escort_mdp_state = None
delivery_mdp_state = None


def escort_mdp_state_callback(state):
    escort_mdp_state = state


def delivery_mdp_state_callback(state):
    delivery_mdp_state = state


# TODO Parameterize the model
def get_problem(task_type, problem):
    if task_type == "delivery":
        return DeliveryMDP(problem)

    if task_type == "escort":
        return EscortMDP(problem)

    return False


def get_current_state(task_type):
    if task_type == "delivery":
        return delivery_mdp_state

    if task_type == "escort":
        return escort_mdp_state

    return False


def execute(task_assignment):
    rospy.loginfo("Info[task_execution_node.execute]: Received a task assignment")

    robot_id = rospy.get_param('robot_id')
    if task_assignment.robot_id == robot_id:
        problem = get_problem(task_assignment.task_type, task_assignment.problem)
        if not problem:
            rospy.logerr("Error[task_execution_node.execute]: Received an invalid task assignment")
        
        problem.initialize()
        policy, state_map, action_map = problem.solve()
        
        current_state = get_current_state(task_assignment.task_type)
        if not current_state: 
            rospy.logerr("Error[task_execution_node.execute]: Received an invalid task assignment")

        while not rospy.is_shutdown():
            new_state = get_current_state(task_assignment.task_type)
            
            if new_state != current_state:
                current_state = new_state
                current_action = action_map[policy[state_map[current_state]]]

                msg = TaskExecutionAction()
                msg.action = current_action

                rospy.loginfo(msg)
                action_publisher.publish(msg)

            duration = rospy.get_param('duration')
            rospy.sleep(duration)


def main():
    rospy.init_node("task_execution_node", anonymous=True)
    rospy.loginfo("Info[task_execution_node.main]: Instantiated the task_execution node")

    rospy.Subscriber("task_assignment/task_assignment_action", TaskAssignmentAction, execute, queue_size=1)
    rospy.Subscriber("monitor/delivery_mdp_state", DeliveryMdpState, delivery_mdp_state_callback, queue_size=1)
    rospy.Subscriber("monitor/escort_mdp_state", EscortMdpState, escort_mdp_state_callback, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
