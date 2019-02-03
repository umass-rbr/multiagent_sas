#!/usr/bin/env python
import datetime
import json
import os
import sys

import numpy as np
import rospy

import utils
from robot import Robot, RobotType
from task_assignment.msg import TaskAssignmentAction, TaskRequest, WorldState
from task_handler import DeliveryTaskHandler, EscortTaskHandler

PUBLISHER = rospy.Publisher('task_assignment/task_assignment_action', TaskAssignmentAction, queue_size=1)

TASK_MAP = {
    'delivery': {
        'task_handler': DeliveryTaskHandler()
    },
    'escort': {
        'task_handler': EscortTaskHandler()
    }
}

# TODO Implement a topic or service that has this information
PUMPKIN = Robot('pumpkin', RobotType.TURTLEBOT)
JAKE = Robot('jake', RobotType.JACKAL)
SHLOMO = Robot('human', RobotType.HUMAN)
ROBOTS = [PUMPKIN, JAKE, SHLOMO]


def get_map(world_state):
    with open('/home/justin/Documents/Development/catkin_ws/src/task_assignment/src/tmp/lgrc.json') as f:
        return json.load(f)


def generate_assignments(tasks, robots):
    options = utils.get_cartesian_product(tasks, robots)
    assignments = utils.get_power_set(options)

    trimmed_assignments = [assignment for assignment in assignments if len(assignment) <= len(tasks)]

    feasible_assignments = []
    for assignment in trimmed_assignments:
        is_feasible = True
        
        is_task_used = [False for _ in tasks]
        is_robot_used = [False for _ in robots]

        for task, robot in assignment:
            task_index = tasks.index(task)
            robot_index = robots.index(robot)

            if is_task_used[task_index] or is_robot_used[robot_index]:
                is_feasible = False
                break

            is_task_used[task_index] = True
            is_robot_used[robot_index] = True

        if is_feasible: 
            feasible_assignments.append(assignment)

    return feasible_assignments


def calculate_expected_cost(assignment, map):
    cost = 0

    for task, robot in assignment:
        break_probability = robot.get_break_probability(task.pickup_location, task.dropoff_location)
        time = robot.get_time_duration(map, task.pickup_location, task.dropoff_location)
        cost += 1000 * break_probability + time * (1 - break_probability)

    return cost


def find_best_assignment(tasks, assignments, map):
    best_assignment = None
    best_expected_cost = float('inf')

    for assignment in assignments:
        expected_cost = calculate_expected_cost(assignment, map) + 1000 * (len(tasks) - len(assignment))
        
        if expected_cost < best_expected_cost:
            best_assignment = assignment
            best_expected_cost = expected_cost

    return best_assignment, best_expected_cost


def get_tasks(task_requests):
    tasks = []

    for task_request in task_requests:
        task_handler = TASK_MAP[task_request.type]['task_handler']
        task = task_handler.get_task(task_request.id, task_request.start_time, task_request.end_time, task_request, json.loads(task_request.data))
        tasks.append(task)

    return tasks


def get_available_robots(robot_status):
    return [robot for i, robot in enumerate(ROBOTS) if robot_status[i]]

        
def assign(world_state):
    tasks = get_tasks(world_state.task_requests)
    available_robots = get_available_robots(world_state.robot_status)

    map = get_map(world_state)

    if available_robots:
        rospy.loginfo('Info[greedy_task_assignment_node.assign]: Generating tasks assignments...')
        assignments = generate_assignments(tasks, available_robots)

        rospy.loginfo('Info[greedy_task_assignment_node.assign]: Determining best assignments...')
        best_assignment, best_cost = find_best_assignment(tasks, assignments, map)
        
        rospy.loginfo('Info[greedy_task_assignment_node.assign]: Publishing assignments...')
        for task, robot in best_assignment:
            message = TaskAssignmentAction()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = "/greedy_task_assignment_node"
            message.robot_id = robot.id
            message.task_request = task.task_request

            rospy.loginfo('Info[greedy_task_assignment_node.assign]: Publishing the assignment: %s', message)
            PUBLISHER.publish(message)

    return best_assignment, best_cost


def main():
    rospy.init_node('task_assigner_node', anonymous=True)
    rospy.loginfo('Info[greedy_task_assignment_node.main]: Instantiated the task_assignment node')

    rospy.Subscriber('monitor/world_state', WorldState, assign, queue_size=1)

    rospy.loginfo('Info[greedy_task_assignment_node.main]: Spinning...')
    rospy.spin()


if __name__ == '__main__':
    main()
