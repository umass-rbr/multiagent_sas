import os, sys, json
import rospy

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..', '..'))

from mdp_solver.srv import *

def mdp_solver_client(map_info):
    rospy.wait_for_service('mdp_solver')
    try:
        mdp_solver = rospy.ServiceProxy('mdp_solver', MDPSolver)
        response = mdp_solver(json.dumps(map_info), 0, 2)
        return response.plan
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    with open(os.path.join(current_file_path, '..', 'maps', 'map.json')) as f:
        map_info = json.load(f)
        print(mdp_solver_client(map_info))
