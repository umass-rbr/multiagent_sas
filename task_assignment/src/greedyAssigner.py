#!/usr/bin/env python
import itertools as it
import json
from collections import namedtuple
import os
import sys

import numpy as np
import rospy

import robot
from task import DeliveryTask, EscortTask
from task_assignment.msg import TaskAssignmentAction, WorldState

pumpkin = robot.Robot(0, 0, 0)
jackal = robot.Robot(1, 1, 3)
human = robot.Robot(2, 2, 2)
Robots = [pumpkin, jackal, human]

PUBLISHER = rospy.Publisher("task_assignment/task_assignment_action", TaskAssignmentAction, queue_size=1)


def power_set(iterable):    
    """ Return the power set of any iterable (e.g., list) with set elements. """

    s = list(iterable)
    powerSet = it.chain.from_iterable(it.combinations(s, r) for r in range(len(s) + 1))
    powerSetList = [set(ele) for ele in list(powerSet)]
    return powerSetList

def generate_all_assignments(tasks,robots):
    A = list(power_set(list(it.product(tasks, robots))))
    A = [a for a in A if len(a) <= len(tasks)]

    #Clean up the powerset to remove infeasible actions
    #I.e. assigning multiple tasks to 1 robot or vice-versa
    new_A = []
    for a in A:
        t_check = [0 for t in tasks]
        r_check = [0 for r in robots]
        check = True
        for (t,r) in a:
            if t_check[tasks.index(t)] == 1:
                check = False
                break
            elif r_check[robots.index(r)] == 1:
                check = False
                break
            else:
                t_check[tasks.index(t)] = 1
                r_check[robots.index(r)] = 1
        if check is True: 
            new_A.append(a)

    return new_A

def calculate_expected_assignment_cost(assignment):
    cost = 0
    for (task, robot) in assignment:
        cost += robot.get_break_probability(task.start_location,task.end_location) * 1000
        cost += ((1 - robot.get_break_probability(task.start_location,task.end_location))
                * robot.calculate_time(task.start_location,task.end_location))
    return cost

def find_best_assignment(tasks, assignments):
    best_cost = float('inf')
    best_assignment = None

    for assignment in assignments:
        exp_cost = calculate_expected_assignment_cost(assignment) + 1000 * (len(tasks) - len(assignment))
        if exp_cost < best_cost:
            best_cost = exp_cost
            best_assignment = assignment

    return best_assignment, best_cost


def assignment_to_json(assignment):
    assignment_dict = {}
    for task,robot in assignment:
        assignment_dict[robot] = str(task)
    return json.dumps(assignment_dict, indent=4)


def unpack_tasks(tasks_as_dict):
    print(tasks_as_dict)
    T = []

    for key in tasks_as_dict.keys():
        task = tasks_as_dict[key]
        if task['task_type'] == 'delivery':
            deliveryTask = DeliveryTask(task['id'], task['package'], task['start_location'], task['end_location'], task['start_time'], task['end_time'])
            T.append(deliveryTask)
        if tasks_as_dict[key]['task_type'] == 'escort':
            escortTask = DeliveryTask(task['id'], task['person'], task['start_location'], task['end_location'], task['start_time'], task['end_time'])
            T.append(escortTask)

    return T

        
def assign_tasks(message):
    tasks = unpack_tasks(json.loads(message.tasks))
    robot_status = message.robot_status
    robots = [Robots[i] for i in range(len(robot_status)) if robot_status[i] == 1]
    if len(robots) != 0:
        rospy.loginfo("Info[task_assignment_node.assign_tasks]: Generating tasks assignments...")
        assignments = generate_all_assignments(tasks,robots)

        rospy.loginfo("Info[task_assignment_node.assign_tasks]: Determining best assignments...")
        best_assignment, best_cost = find_best_assignment(tasks, assignments)
        
        rospy.loginfo("Info[task_assignment_node.assign_tasks]: Publishing assignments...")
        for (task,robot) in best_assignment:
            assignment_msg = TaskAssignmentAction()
            assignment_msg.robot_id = str(robot.get_id())
            assignment_msg.task_type = task.get_type()
            assignment_msg.task_data = task.pack()

            rospy.loginfo("Info[task_assignment_node.assign_tasks]: Publishing the assignment: %s", assignment_msg)
            PUBLISHER.publish(assignment_msg)

        rate = rospy.Rate(10)
        rate.sleep()

    return (best_assignment,best_cost)

def main():
    rospy.init_node("task_assigner_node", anonymous=True)
    rospy.loginfo("Info[task_assignment_node.main]: Instantiated the task_assignment node")

    rospy.Subscriber("monitor/world_state", WorldState, assign_tasks, queue_size=1)

    rospy.loginfo("Info[task_assignment_node.main]: Spinning...")
    rospy.spin()


def test():

    t1 = task.DeliveryTask(0,'package1','2','1','0','3')
    t2 = task.DeliveryTask(1,'package2','1','0','0','2')
    t3 = task.DeliveryTask(2,'package3','1','2','1','4')
    t4 = task.DeliveryTask(3,'package4','2','3','3','4')

    T = [t1,t2,t3,t4]

    assignment,cost = assign_tasks(T)

    print(assignment_to_json(assignment))
    print(cost)

if __name__ == '__main__':
    main()
