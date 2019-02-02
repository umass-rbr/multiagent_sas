import json
import rospy

from msg import TaskAssignment

import problem_generator
import robot
import task
import utils

WORLDS_DIRECTORY = "worlds/"
TASK_DURATION = 1800

current_state = None

rospy.init_node('state_listener')
rospy.Subscriber('state_information', State, assign_tasks)

def assign_tasks(data):
    # TODO Ask Sadegh about node structure
    current_state = unpack(data)

    pub = rospy.Publisher('task_assigner/assignment', TaskAssignment)
    rospy.init_node('task_assigner')

    with open('mdp_info.json', 'r') as mdp_info_file:
        mdp_info = json.loads(mdp_info_file)

        # TODO Make a a map service
        with open(WORLDS_DIRECTORY + "world.json", "r") as world_file:
            world = json.load(world_file)

            while not rospy.is_shutdown():
                # TODO Query state from some source
                current_action = utils.get_action(mdp_info, current_state)

                for t, r in current_action:
                    msg = TaskAssignment()
                    msg.robot_id = r.get_id()
                    msg.problem = problem_generator.generate_escort_problem(r, t, world)

                    rospy.loginfo(msg)
                    pub.publish(msg)

                rospy.sleep(TASK_DURATION)

def unpack(data):
    time = data.time
    list_tasks_as_strings = data.tasks.split('\n')
    list_tasks_as_objects = [convert_tasks_str_obj(task) for task in list_tasks_as_strings]
    robot_status = data.robot_status
    return set(time,list_tasks_as_objects,robot_status)

def convert_tasks_str_obj(task):
    task_info = task.split(',')
    if task.get_type() == 'delivery':
        return task.DeliveryTask(int(task_info[0]),task_info[1],task_info[2]
            task_info[3],task_info[4],task_info[5])
    elif task.get_type() == 'escort':
        return task.EscortTask(int(task_info[0]),task_info[1],task_info[2]
            task_info[3],task_info[4],task_info[5])
