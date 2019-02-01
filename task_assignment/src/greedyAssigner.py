import os
import sys
import json

import numpy as np
import itertools as it
import rospy

import task
import robot

thisFilePath = os.path.dirname(os.path.realpath(__file__))

print thisFilePath

sys.path.append(os.path.join(thisFilePath, "..", "..", "..", "multiagent_sas"))

from task_assignment.msg import TaskAssignmentAction
quit()


pumpkin = robot.Robot(0,0,0)
jackal = robot.Robot(1,1,3)
human = robot.Robot(2,2,2)
Robots = [pumpkin,jackal,human]

#PUBLISHER = rospy.Publisher("task_assignment/task_execution_action", TaskExecutionAction, queue_size=1)

def power_set(iterable):    
    """ Return the power set of any iterable (e.g., list) with set elements. """

    s = list(iterable)
    powerSet = it.chain.from_iterable(it.combinations(s, r) for r in range(len(s) + 1))
    powerSetList = [set(ele) for ele in list(powerSet)]
    return powerSetList

def generate_assignments(tasks,robots):
    A = list(power_set(list(it.product(tasks,robots))))
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

def calculate_expected_assignment_cost(tasks, assignment):
    cost = 0
    assignment_tasks = set()
    for (task,robot_id) in assignment:
        cost += Robots[robot_id].get_break_probability(task.start_location,task.end_location) * 1000
        cost += ((1-Robots[robot_id].get_break_probability(task.start_location,task.end_location))
                * Robots[robot_id].calculate_time(task.start_location,task.end_location))
        assignment_tasks.add(task)
    for task in set(tasks).difference(assignment_tasks):
        cost += 1000
    print(cost)
    return cost


def assignment_to_json(assignment):
    assignment_dict = {}
    for task,robot in assignment:
        assignment_dict[robot] = str(task)
    return json.dumps(assignment_dict, indent=4)


def assign_tasks(tasks):
    #tasks = worldState.tasks
    #robot_status = worldState.robot_status
    robot_status = [1,1,1]
    robots = [i for i in range(len(robot_status)) if robot_status[i] == 1]
    assignments = generate_assignments(tasks,robots)
    best_cost = float('inf')
    best_assignment = None
    for assignment in assignments:
        exp_cost = calculate_expected_assignment_cost(tasks,assignment)
        if exp_cost < best_cost:
            best_cost = exp_cost
            best_assignment = assignment
    return (best_assignment,best_cost)

def main():
    rospy.init_node("task_assigner_node", anonymous=True)
    rospy.loginfo("Info[task_assignment_node.main]: Instantiated the task_assignment node")

    rospy.Subscriber("monitor/delivery_mdp_state", DeliveryMdpState, delivery_mdp_state_callback, queue_size=1)
    rospy.Subscriber("monitor/escort_mdp_state", EscortMdpState, escort_mdp_state_callback, queue_size=1)
    rospy.Subscriber("task_assignment/task_assignment_action", TaskAssignmentAction, execute, queue_size=1)

    rospy.loginfo("Info[task_execution_node.main]: Spinning...")
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