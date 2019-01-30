#!/usr/bin/env python
import json

import rospy

from mdps.delivery_mdp import DeliveryMDP
from mdps.escort_mdp import EscortMDP
from task_assignment.msg import TaskAssignmentAction
from task_execution.msg import DeliveryMdpState, EscortMdpState, TaskExecutionAction

action_publisher = rospy.Publisher("task_execution/task_execution_action", TaskExecutionAction, queue_size=1)

delivery_mdp_state = ("shlomoOffice", 0)
escort_mdp_state = None


def delivery_mdp_state_callback(state):
    global delivery_mdp_state
    delivery_mdp_state = state


def escort_mdp_state_callback(state):
    global escort_mdp_state
    escort_mdp_state = state


# TODO Implement the request to the map service
def get_map(task_assignment):
    # rospy.wait_for_service('get_map')
    # try:
    #     get_map = rospy.ServiceProxy('get_map', GetMap)
    #     map = get_map(task_assignment.task_data.map_name)
    #     return map
    # except rospy.ServiceException:
    #     return None

    import json
    with open('/home/justin/Documents/Development/catkin_ws/src/task_execution/src/tmp/lgrc.json') as f:
        return json.load(f)


def get_problem(task_assignment, map):
    task_data = json.loads(task_assignment.task_data)

    if task_assignment.task_type == "delivery":
        return DeliveryMDP(map, task_data["start_location"], task_data["end_location"])

    if task_assignment.task_type == "escort":
        return EscortMDP(map, task_data["start_location"], task_data["end_location"])

    return None


# TODO Retrieve the solution from a file if available
def get_solution(task_assignment, problem):
    if task_assignment.task_type == "delivery":
        return problem.solve()

    if task_assignment.task_type == "escort":
        return problem.solve()

    return None


def get_current_state(task_assignment):
    if task_assignment.task_type == "delivery":
        return delivery_mdp_state

    if task_assignment.task_type == "escort":
        return escort_mdp_state

    return None


# TODO Get a location name from the policy
def execute(task_assignment):
    rospy.loginfo("Info[task_execution_node.execute]: Received a task assignment: %s", task_assignment)

    robot_id = rospy.get_param('/task_execution_node/robot_id')
    if task_assignment.robot_id == robot_id:
        rospy.loginfo("Info[task_execution_node.execute]: Retrieving the map...")
        map = get_map(task_assignment)
        if not map:
            rospy.logerr("Error[task_execution_node.execute]: Failed to get map")
            return

        rospy.loginfo("Info[task_execution_node.execute]: Generating the problem...")
        problem = get_problem(task_assignment, map)
        if not problem:
            rospy.logerr("Error[task_execution_node.execute]: Received an invalid task assignment")
            return
        
        rospy.loginfo("Info[task_execution_node.execute]: Solving the problem...")
        solution = problem.solve()
        if not solution:
            rospy.logerr("Error[task_execution_node.execute]: Received an invalid task assignment")
            return

        state_map = solution["state_map"]
        action_map = solution["action_map"]
        policy = solution["policy"]

        rospy.loginfo("Info[task_execution_node.execute]: Getting the current state...")
        current_state = get_current_state(task_assignment)
        if not current_state:
            rospy.logerr("Error[task_execution_node.execute]: Received an invalid task assignment")
            return

        while not rospy.is_shutdown():
            new_state = get_current_state(task_assignment)
            if not current_state: 
                rospy.logerr("Error[task_execution_node.execute]: Received an invalid task assignment")
                return
            
            if new_state != current_state:
                current_state = new_state
                current_action = action_map[policy[state_map[current_state]]]

                msg = TaskExecutionAction()
                msg.x = map[current_action]["position"]["x"]
                msg.y = map[current_action]["position"]["y"]
                msg.theta = map[current_action]["position"]["theta"]

                rospy.loginfo(msg)
                action_publisher.publish(msg)

            duration = rospy.get_param('/task_execution_node/duration')
            rospy.sleep(duration)


# TODO Separate monitoring code into another file
# TODO Rename everything across all files
def main():
    rospy.init_node("task_execution_node", anonymous=True)
    rospy.loginfo("Info[task_execution_node.main]: Instantiated the task_execution node")

    rospy.Subscriber("monitor/delivery_mdp_state", DeliveryMdpState, delivery_mdp_state_callback, queue_size=1)
    rospy.Subscriber("monitor/escort_mdp_state", EscortMdpState, escort_mdp_state_callback, queue_size=1)
    rospy.Subscriber("task_assignment/task_assignment_action", TaskAssignmentAction, execute, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
