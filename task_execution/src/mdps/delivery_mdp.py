import ctypes as ct
import os
import sys

import numpy as np

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, "..", "..", "..", "..", "..", "nova", "python"))

from nova.mdp import MDP
from nova.mdp_value_function import MDPValueFunction
from nova.mdp_vi import MDPVI


# TODO Make sure nova works
# TODO Implement a pretty print function
# TODO Make sure the getters work properly
class DeliveryMDP(object):
    def __init__(self, map, start_location, end_location):
        self.map = map
        self.start_location = start_location
        self.end_location = end_location

        self.mdp = None
        self.H = 4

        self._initialize()

    def _initialize(self):
        self.states = self._compute_states()
        self.actions = self._compute_actions()

        self.mdp = MDP()
        self.mdp.n = len(self.states)
        self.mdp.ns = len(self.states)
        self.mdp.m = len(self.actions)
        self.mdp.gamma = 0.99
        self.mdp.horizon = int(self.H)
        self.mdp.epsilon = 0.001
        self.mdp.s0 = int(self.mdp.n / (self.H + 1) - 1)
        self.mdp.ng = 0

        S, self.T = self._compute_state_transitions()
        array_type_nmns_int = ct.c_int * (self.mdp.n * self.mdp.m * self.mdp.ns)
        array_type_nmns_float = ct.c_float * (self.mdp.n * self.mdp.m * self.mdp.ns)
        self.mdp.S = array_type_nmns_int(*np.array(S).flatten())
        self.mdp.T = array_type_nmns_float(*np.array(self.T).flatten())

        R = self._compute_rewards(self.T)
        array_type_nm_float = ct.c_float * (self.mdp.n * self.mdp.m)
        self.mdp.R = array_type_nm_float(*np.array(R).flatten())
        self.mdp.Rmax = float(np.array(R).max())
        self.mdp.Rmin = float(np.array(R).min())

    def _compute_states(self):
        location_states = self.map["locations"].keys()
        has_package_states = [True, False]
        return [(location, has_package) for location in location_states for has_package in has_package_states]

    def _compute_actions(self):
        return self.map["locations"].keys() + ["pickup"]

    def _compute_state_transitions(self):
        S = [[[-1 for sp in range(self.mdp.ns)] for a in range(self.mdp.m)] for s in range(self.mdp.n)]
        T = [[[0.0 for sp in range(self.mdp.ns)] for a in range(self.mdp.m)] for s in range(self.mdp.n)]

        for s, state in enumerate(self.states):
            for a, action in enumerate(self.actions):
                if state[0] == self.end_location and state[1] == True:
                    T[s][a][s] = 1.0
                    continue

                for sp, statePrime in enumerate(self.states):
                    S[s][a][sp] = sp

                    if action == "pickup":
                        if state[0] == self.start_location and state[0] == statePrime[0] and state[1] == False and statePrime[1] == True:
                            T[s][a][sp] = 1.0
                            break
                    else:
                        if self.map["paths"][state[0]][action] and action == statePrime[0]:
                            T[s][a][sp] = 1.0
                            break

        return S, T

    def _compute_rewards(self, T):
        R = [[0.0 for a in range(self.mdp.m)] for s in range(self.mdp.n)]

        for s, state in enumerate(self.states):
            for a, action in enumerate(self.actions):
                if action == "pickup":
                    if state[0] == self.start_location and state[1] == 0:
                        R[s][a] = float("inf") 
                else:
                    path_cost = self.map["paths"][state[0]][action]
                    if not path_cost:
                        R[s][a] = float("-inf") 
                    else:
                        R[s][a] = path_cost

        return R

    def _get_name(self):
        return "delivery-{}-{}-{}".format(self.map.name, self.start_location, self.end_location)

    def _get_policy(self):
        policy = {}

        algorithm = MDPVI(self.mdp)
        raw_policy = algorithm.solve()

        policy_as_string = str(raw_policy)[str(raw_policy).index("p"):]
        policy_as_string = policy_as_string[policy_as_string.index("[") + 1:policy_as_string.index("]")].strip()
        policy_as_string = policy_as_string.replace("\n", "")

        policy_as_list = policy_as_string.split(" ")
        policy_as_list = [c for c in policy_as_list if c]

        for s in range(len(self.states)):
            policy[str(s)] = policy_as_list[s]

        return policy

    def _get_state_map(self):
        state_map = {}

        for s in range(len(self.states)):
            key = str(self.states[s])
            state_map[key] = str(s)
        
        return state_map

    def _get_action_map(self):
        action_map = {}

        for a in range(len(self.actions)):
            key = str(self.actions[a])
            action_map[key] = str(a)
            
        return action_map

    def solve(self):
        return {
            "state_map": self._get_state_map(),
            "action_map": self._get_action_map(),
            "policy": self._get_policy(), 
        }
