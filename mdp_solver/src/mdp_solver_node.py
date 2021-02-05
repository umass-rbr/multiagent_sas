#!/usr/bin/env python
import json
import os
import rospy
import roslib
import pickle
import numpy as np

from mdp_solver.msgs import MDPSolverResponse, MDPSolverRequest

CURRENT_FILE_PATH = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(CURRENT_FILE_PATH, '..'))

MAP_PATH = os.path.join(CURRENT_FILE_PATH, '..', 'maps', 'map.json')

MDP_RESPONSE_PUBLISHER = rospy.Publisher("mdp_solver/plan", MDPSolverResponse, queue_size=1)

STATIC_MAP = None
current_request_info = None
current_ip_error_info = {}


def update_dataset(edge, p):
    with open(dataset_file_location, mode='rb') as f:
        dataset = pickle.load(f, encoding='bytes')
    dataset.append(edge['s0_id'], edge['s1_id'], p)
    with open(dataset_file_location, mode='wb') as f:
        pickle.dump(dataset, f, protocol = pickle.HIGHEST_PROTOCOL)


def load_static_map():
    with open(MAP_PATH) as world_map_file:
        global STATIC_MAP
        STATIC_MAP = json.load(world_map_file)


def ip_info_callback(message):
    global current_ip_error_info
    if message.s0_id not in current_ip_error_info.keys():
        current_ip_error_info[s0_id] = {}
    current_ip_error_info[s0_id][s1_id] = message.failure_likelihood


def execute(request):
    rospy.loginfo("Info[mdp_solver_node.execute]: Received a new solver request: %s", request)
    global current_request_info
    current_request_info = request

    map_info = json.loads(request.map)
    init_node_id = request.start
    goal_node_id = request.goal

    rospy.loginfo("Info[mdp_solver_node.execute]: Solving path planning mdp...")
    mdp = PathDomainModel(map_info, current_ip_error_info)
    mdp.set_init(init_node_id)
    mdp.set_goal(goal_node_id)
    mdp.reset()

    # rospy.loginfo("Info[mdp_solver_node.execute]: Solving perception sensitive mdp...")
    # ps_path_mdp = PerceptionSensitiveMDP(base_path_mdp, current_ip_error_info)
    # ps_path_mdp.solve()

    plan = []
    state = mdp.init
    while state != mdp.goal:
        plan.append(state[0][0])
        state = mdp.generate_successor(mdp.pi[mdp.states.index(states)])
    plan.append(state[0][0])

    response = MDPSolverResponse()
    response.plan = plan
    MDP_RESPONSE_PUBLISHER.publish(response)


def main():
    rospy.loginfo("Info[mdp_solver.main]: Instantiating the mdp_solver node...")
    rospy.init_node("mdp_solver_node", anonymous=True)

    rospy.Subscriber("introspective_perception/error_callback", IntrospectivePerceptionInfo, ip_info_callback, queue_size=1)

    load_static_map()

    srv = rospy.Service('mdp_solver', MDPSolverRequest, execute)

    rospy.loginfo("Info[mdp_solver_node.main]: Spinning...")
    rospy.spin()


if __name__ == '__main__':
    main()