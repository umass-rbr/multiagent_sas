#!/usr/bin/env python
import json

import rospy

from mdps.delivery_mdp import DeliveryMDP
from mdps.escort_mdp import EscortMDP
from task_assignment.msg import TaskAssignmentAction
from task_execution.msg import DeliveryMdpState, EscortMdpState, TaskExecutionAction

PUBLISHER = rospy.Publisher("task_execution/task_execution_action", TaskExecutionAction, queue_size=1)

TASK_MAP = {
    "delivery": {
        "problem": lambda map, task_data: DeliveryMDP(map, task_data["start_location"], task_data["end_location"]),
        "solution": lambda problem: problem.solve(),
        "state": lambda: delivery_mdp_state
    },
    "escort": {
        "problem": lambda map, task_data: EscortMDP(map, task_data["start_location"], task_data["end_location"]),
        "solution": lambda problem: problem.solve(),
        "state": lambda: escort_mdp_state
    }
}

delivery_mdp_state = None
escort_mdp_state = None

def delivery_mdp_state_callback(state):
    global delivery_mdp_state
    delivery_mdp_state = (state.location, state.has_package)


def escort_mdp_state_callback(state):
    global escort_mdp_state
    escort_mdp_state = (state.location, state.with_person)


def get_map(task_assignment):
    with open('/home/justin/Documents/Development/catkin_ws/src/task_execution/src/tmp/lgrc.json') as f:
        return json.load(f)


def execute(task_assignment):
    rospy.loginfo("Info[task_execution_node.execute]: Received a task assignment: %s", task_assignment)

    try:
        robot_id = rospy.get_param('/task_execution_node/robot_id')
        wait_duration = rospy.get_param('/task_execution_node/wait_duration')
        timeout_duration = rospy.get_param('/task_execution_node/timeout_duration')

        if task_assignment.robot_id == robot_id:
            rospy.loginfo("Info[task_execution_node.execute]: Retrieving the map...")
            map = get_map(task_assignment)

            rospy.loginfo("Info[task_execution_node.execute]: Generating the problem...")
            task_data = json.loads(task_assignment.task_data)
            problem = TASK_MAP[task_assignment.task_type]["problem"](map, task_data)
            
            rospy.loginfo("Info[task_execution_node.execute]: Solving the problem...")
            solution = TASK_MAP[task_assignment.task_type]["solution"](problem)

            state_map = solution["state_map"]
            action_map = solution["action_map"]
            policy = solution["policy"]

            activation_time = rospy.Time.now()

            current_state = None
            
            while not rospy.is_shutdown():
                rospy.loginfo("Info[task_execution_node.execute]: Getting the current state...")
                new_state = TASK_MAP[task_assignment.task_type]["state"]()

                if new_state != current_state:
                    current_state = new_state
                    
                    state_index = state_map[current_state]
                    action_index = policy[state_index]
                    current_action = action_map[action_index]

                    msg = TaskExecutionAction()
                    msg.x = map["locations"][current_action]["position"]["x"]
                    msg.y = map["locations"][current_action]["position"]["y"]
                    msg.theta = map["locations"][current_action]["position"]["theta"]

                    rospy.loginfo("Info[task_execution_node.execute]: Publishing an action: %s", msg)
                    PUBLISHER.publish(msg)
                
                current_time = rospy.Time.now()
                if current_time - activation_time > rospy.Duration(timeout_duration):
                    raise RuntimeError("Exceeded the time limit to execute the task")

                rospy.sleep(wait_duration)
    except Exception as e:
        rospy.logerr("Error[task_execution_node.execute]: Encountered an exception while executing the task: %s", e)


# TODO Implement the request to the map service
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
