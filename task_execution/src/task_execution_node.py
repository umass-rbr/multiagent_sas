#!/usr/bin/env python
import json
import os
import rospy

current_file_path = os.path.dirname(os.path.realpath(__file__))

import roslib; roslib.load_manifest('jackal_msgs_amrl')
from jackal_msgs_amrl.msg import NavGoal

from task_handler import DeliveryTaskHandler, EscortTaskHandler
from task_assignment.msg import TaskAssignmentAction
from task_execution.msg import DeliveryMdpState, EscortMdpState, InterfaceAction

NAVIGATION_ACTION_PUBLISHER = rospy.Publisher("orb_nav/goal", NavGoal, queue_size=1)
INTERFACE_ACTION_PUBLISHER = rospy.Publisher("task_execution/interface_action", InterfaceAction, queue_size=1)

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


def get_world_map():
    with open(current_file_path + '/tmp/LGRC3_plan_map.json') as world_map_file:
        return json.load(world_map_file)


def execute(task_assignment):
    rospy.loginfo("Info[task_execution_node.execute]: Received a task assignment: %s", task_assignment)

    robot_id = rospy.get_param('/task_execution_node/robot_id')
    wait_duration = rospy.get_param('/task_execution_node/wait_duration')
    timeout_duration = rospy.get_param('/task_execution_node/timeout_duration')

    if task_assignment.robot_id == robot_id:
        task_handler = TASK_MAP[task_assignment.task_request.type]["task_handler"]
        message_selector = TASK_MAP[task_assignment.task_request.type]["message_selector"]
        task_data = json.loads(task_assignment.task_request.data)

        rospy.loginfo("Info[task_execution_node.execute]: Retrieving the map...")
        world_map = get_world_map()

        rospy.loginfo("Info[task_execution_node.execute]: Generating the problem...")
        problem = task_handler.get_problem(world_map, task_data)

        rospy.loginfo("Info[task_execution_node.execute]: Solving the problem...")
        solution = task_handler.get_solution(problem)

        state_map = solution["state_map"]
        action_map = solution["action_map"]
        policy = solution["policy"]

        has_package = False
        current_state = None

        while not task_handler.is_goal(current_state, task_data):
            new_state = task_handler.get_state(message_selector())
            new_state = (new_state[0], has_package)
            rospy.loginfo("Info[task_execution_node.execute]: Retrieved the current state: %s", new_state)

            if new_state != current_state:
                current_state = new_state

                state_index = state_map[current_state]
                action_index = policy[state_index]
                current_action = action_map[action_index]

                if current_action == "pickup":
                    rospy.loginfo("Please place the package on me.")
                    raw_input("Please hit any key after you've placed the package on me.")
                    has_package = True
                elif current_action == "dropoff":
                    rospy.loginfo("Please take the package from me.")
                    raw_input("Please hit any key after you've taken the package from me.")
                    break
                else:
                    action_message = NavGoal()
                    #action_message.header.stamp = rospy.Time.now()
                    #action_message.header.frame_id = "/task_execution_node"
                    action_message.x = world_map["locations"][current_action]["pose"]["x"]
                    action_message.y = world_map["locations"][current_action]["pose"]["y"]

                    rospy.loginfo("Info[task_execution_node.execute]: Executing the current action: %s", current_action)
                    NAVIGATION_ACTION_PUBLISHER.publish(action_message)
                
                activation_time = rospy.Time.now()

            current_time = rospy.Time.now()
            if current_time - activation_time > rospy.Duration(timeout_duration):
                raise RuntimeError("Exceeded the time limit to execute the task")

            rospy.sleep(wait_duration)
        
        rospy.loginfo("Info[task_execution_node.execute]: Reached the goal :)")


def main():
    rospy.loginfo("Info[task_execution_node.main]: Instantiating the task_execution node...")
    rospy.init_node("task_execution_node", anonymous=True)

    rospy.Subscriber("monitor/delivery_mdp_state", DeliveryMdpState, delivery_mdp_state_callback, queue_size=1)
    rospy.Subscriber("monitor/escort_mdp_state", EscortMdpState, escort_mdp_state_callback, queue_size=1)
    rospy.Subscriber("task_assignment/task_assignment_action", TaskAssignmentAction, execute,queue_size=1)

    rospy.loginfo("Info[task_execution_node.main]: Spinning...")
    rospy.spin()


if __name__ == '__main__':
    main()
