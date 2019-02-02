#!/usr/bin/env python
import datetime
import json
import os
import sys

import numpy as np
import rospy

import utils
from robot import Robot, RobotType
from task import DeliveryTask, EscortTask
from task_assignment.msg import TaskAssignmentAction, WorldState

PUBLISHER = rospy.Publisher('task_assignment/task_assignment_action', TaskAssignmentAction, queue_size=1)

TASK_MAP = {
    'delivery': DeliveryTask,
    'escort': EscortTask
}

PUMPKIN = Robot('pumpkin', RobotType.TURTLEBOT)
JAKE = Robot('jake', RobotType.JACKAL)
SHLOMO = Robot('human', RobotType.HUMAN)
ROBOTS = [PUMPKIN, JAKE, SHLOMO]


def get_map():
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


def calculate_expected_assignment_cost(assignment, map):
    cost = 0

    for task, robot in assignment:
        break_probability = robot.get_break_probability(task.pickup_location, task.dropoff_location)
        time = robot.calculate_time(map, task.pickup_location, task.dropoff_location)
        cost += 1000 * break_probability + (time * (1 - break_probability))

    return cost


def find_best_assignment(tasks, assignments, map):
    best_cost = float('inf')
    best_assignment = None

    for assignment in assignments:
        exp_cost = calculate_expected_assignment_cost(assignment, map) + 1000 * (len(tasks) - len(assignment))
        
        if exp_cost < best_cost:
            best_cost = exp_cost
            best_assignment = assignment

    return best_assignment, best_cost


def assignment_to_json(assignment):
    assignment_dict = {}

    for task, robot in assignment:
        assignment_dict[robot] = str(task)

    return json.dumps(assignment_dict, indent=4)


def unpack_tasks(tasks_as_dict):
    T = []

    for task in tasks_as_dict.values():
        T.append(TASK_MAP[task['task_type']](**task))

    return T

        
def assign(message):
    tasks = unpack_tasks(json.loads(message.tasks))
    robot_status = message.robot_status

    available_robots = [robot for i, robot in enumerate(ROBOTS) if robot_status[i]]

    map = get_map()

    if available_robots:
        rospy.loginfo('Info[task_assignment_node.assign]: Generating tasks assignments...')
        assignments = generate_assignments(tasks, available_robots)

        rospy.loginfo('Info[task_assignment_node.assign]: Determining best assignments...')
        best_assignment, best_cost = find_best_assignment(tasks, assignments, map)
        
        rospy.loginfo('Info[task_assignment_node.assign]: Publishing assignments...')
        for task, robot in best_assignment:
            message = TaskAssignmentAction()
            message.robot_id = robot.get_id()
            message.task_type = task.get_type()
            message.task_data = task.pack()

            rospy.loginfo('Info[task_assignment_node.assign]: Publishing the assignment: %s', message)
            PUBLISHER.publish(message)

        rate = rospy.Rate(10)
        rate.sleep()

    return best_assignment, best_cost


def main():
    rospy.init_node('task_assigner_node', anonymous=True)
    rospy.loginfo('Info[task_assignment_node.main]: Instantiated the task_assignment node')

    rospy.Subscriber('monitor/world_state', WorldState, assign, queue_size=1)

    rospy.loginfo('Info[task_assignment_node.main]: Spinning...')
    rospy.spin()


def test():
    t1 = DeliveryTask(0,'package1','shlomoOffice','AMRL','0','3')
    t2 = DeliveryTask(1,'package2','AMRL','shlomoOffice','0','2')
    t3 = DeliveryTask(2,'package3','AMRL','mailroom','1','4')
    t4 = DeliveryTask(3,'package4','shlomoOffice','mailroom','3','4')

    T = {}

    T[1] = t1.pack()
    T[2] = t2.pack()
    T[3] = t3.pack()
    T[4] = t4.pack()

    T2 = unpack_tasks(T)
    print(T2)
    quit()

    T = [t1,t2,t3,t4]

    assignment,cost = assign(T)

    print(assignment_to_json(assignment))
    print(cost)

if __name__ == '__main__':
    main()
