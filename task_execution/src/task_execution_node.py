#!/usr/bin/env python
import json
import os

import rospy

from task_handler import DeliveryTaskHandler, EscortTaskHandler
from task_assignment.msg import TaskAssignmentAction
from task_execution.msg import DeliveryMdpState, EscortMdpState, TaskExecutionAction

PUBLISHER = rospy.Publisher("task_execution/task_execution_action", TaskExecutionAction, queue_size=1)

TASK_MAP = {
    "delivery": {
        "task_handler": DeliveryTaskHandler(),
        "message_selector": lambda: delivery_mdp_state_message
    },
    "escort": {
        "task_handler": EscortTaskHandler(),
        "message_selector": lambda: escort_mdp_state_message
    }
}

delivery_mdp_state_message = None
escort_mdp_state_message = None


def delivery_mdp_state_callback(message):
    global delivery_mdp_state_message
    delivery_mdp_state_message = message


def escort_mdp_state_callback(message):
    global escort_mdp_state_message
    escort_mdp_state_message = message


def get_map(task_assignment):
    with open('/home/justin/Documents/Development/catkin_ws/src/task_execution/src/tmp/lgrc.json') as f:
        return json.load(f)


def execute(task_assignment):
    rospy.loginfo("Info[task_execution_node.execute]: Received a task assignment: %s", task_assignment)

    robot_id = rospy.get_param('/task_execution_node/robot_id')
    wait_duration = rospy.get_param('/task_execution_node/wait_duration')
    timeout_duration = rospy.get_param('/task_execution_node/timeout_duration')

    if task_assignment.robot_id == robot_id:
        task_handler = TASK_MAP[task_assignment.task_type]["task_handler"]
        message_selector = TASK_MAP[task_assignment.task_type]["message_selector"]
        task_data = json.loads(task_assignment.task_data)

        rospy.loginfo("Info[task_execution_node.execute]: Retrieving the map...")
        map = get_map(task_assignment)

        rospy.loginfo("Info[task_execution_node.execute]: Generating the problem...")
        problem = task_handler.get_problem(map, task_data)
                    
        rospy.loginfo("Info[task_execution_node.execute]: Solving the problem...")
        solution = task_handler.get_solution(problem)

        state_map = solution["state_map"]
        action_map = solution["action_map"]
        policy = solution["policy"]

        current_state = None
        
        while not task_handler.is_goal(current_state, task_data):
            rospy.loginfo("Info[task_execution_node.execute]: Retrieving the current state...")
            state_message = message_selector()
            new_state = task_handler.get_state(state_message)

            if new_state != current_state:
                current_state = new_state
                
                state_index = state_map[current_state]
                action_index = policy[state_index]
                current_action = action_map[action_index]

                action_message = TaskExecutionAction()
                action_message.header.stamp = rospy.Time.now()
                action_message.header.frame_id = "/task_execution_node"
                action_message.x = map["locations"][current_action]["pose"]["x"]
                action_message.y = map["locations"][current_action]["pose"]["y"]
                action_message.theta = map["locations"][current_action]["pose"]["theta"]

                activation_time = rospy.Time.now()

                rospy.loginfo("Info[task_execution_node.execute]: Publishing the action: %s", action_message)
                PUBLISHER.publish(action_message)
            
            current_time = rospy.Time.now()
            if current_time - activation_time > rospy.Duration(timeout_duration):
                raise RuntimeError("Exceeded the time limit to execute the task")

            rospy.sleep(wait_duration)


# TODO Implement the request to the map service
# TODO Rename everything across all files
def main():
    rospy.init_node("task_execution_node", anonymous=True)
    rospy.loginfo("Info[task_execution_node.main]: Instantiated the task_execution node")

    rospy.Subscriber("monitor/delivery_mdp_state", DeliveryMdpState, delivery_mdp_state_callback, queue_size=1)
    rospy.Subscriber("monitor/escort_mdp_state", EscortMdpState, escort_mdp_state_callback, queue_size=1)
    rospy.Subscriber("task_assignment/task_assignment_action", TaskAssignmentAction, execute, queue_size=1)

    rospy.loginfo("Info[task_execution_node.main]: Spinning...")
    rospy.spin()


if __name__ == '__main__':
    main()
