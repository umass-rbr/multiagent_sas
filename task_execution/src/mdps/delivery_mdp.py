import ctypes
import os
import sys

import numpy as np

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, "..", "..", "..", "..", "..", "nova", "python"))

from nova.mdp import MDP
from nova.mdp_value_function import MDPValueFunction
from nova.mdp_vi import MDPVI

from simulate import simulate


# TODO Implement a pretty print function
class DeliveryMDP(object):
    def __init__(self, map, pickup_location, dropoff_location):
        self.map = map
        self.pickup_location = pickup_location
        self.dropoff_location = dropoff_location

        self.name = "delivery-mdp-{}-{}-{}".format(self.map["name"], self.pickup_location, self.dropoff_location)
        
        self.state_map = None
        self.action_map = None
        self.policy = None

        self._initialize()

    def _initialize(self):
        self.states = self._compute_states()
        self.actions = self._compute_actions()

        self.mdp = MDP()
        self.mdp.n = len(self.states)
        self.mdp.ns = len(self.states)
        self.mdp.m = len(self.actions)
        self.mdp.gamma = 0.99
        self.mdp.horizon = 4
        self.mdp.epsilon = 0.001
        self.mdp.s0 = int(self.mdp.n / (self.mdp.horizon + 1) - 1)
        self.mdp.ng = 0

        S, T = self._compute_state_transitions()
        array_type_nmns_int = ctypes.c_int * (self.mdp.n * self.mdp.m * self.mdp.ns)
        array_type_nmns_float = ctypes.c_float * (self.mdp.n * self.mdp.m * self.mdp.ns)
        self.mdp.S = array_type_nmns_int(*np.array(S).flatten())
        self.mdp.T = array_type_nmns_float(*np.array(T).flatten())

        R = self._compute_rewards()
        array_type_nm_float = ctypes.c_float * (self.mdp.n * self.mdp.m)
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
                if state[0] == self.dropoff_location and state[1] == True:
                    T[s][a][s] = 1.0
                    continue

                for sp, statePrime in enumerate(self.states):
                    S[s][a][sp] = sp

                    if action == "pickup":
                        if state[0] == self.pickup_location and state[0] == statePrime[0] and state[1] == False and statePrime[1] == True:
                            T[s][a][sp] = 1.0
                            break
                        else:
                            continue

                    if self.map["paths"][state[0]].has_key(action):
                        if statePrime[0] == action and state[1] == statePrime[1]:
                            T[s][a][sp] = 1.0
                            break

        return S, T

    def _compute_rewards(self):
        R = [[0.0 for a in range(self.mdp.m)] for s in range(self.mdp.n)]

        for s, state in enumerate(self.states):
            for a, action in enumerate(self.actions):

                if action is not "pickup":
                    if not self.map["paths"][state[0]].has_key(action):
                        R[s][a] = float("-inf")
                    else:
                        R[s][a] -= self.map["paths"][state[0]][action]['cost']

        return R

    def _generate_state_map(self):
        self.state_map = {state: s for s, state in enumerate(self.states)}
        return self.state_map

    def _generate_action_map(self):
        self.action_map = {a: action for a, action in enumerate(self.actions)}
        return self.action_map

    def _generate_policy(self):
        policy = {}

        algorithm = MDPVI(self.mdp)
        raw_policy = algorithm.solve()

        policy_as_string = str(raw_policy)[str(raw_policy).index("p"):]
        policy_as_string = policy_as_string[policy_as_string.index("[") + 1:policy_as_string.index("]")].strip()
        policy_as_string = policy_as_string.replace("\n", "")

        policy_as_list = policy_as_string.split(" ")
        policy_as_list = [c for c in policy_as_list if c]

        for index in range(len(self.states)):
            policy[index] = int(policy_as_list[index])

        self.policy = policy
        return self.policy

    def solve(self):
        if self.policy is not None:
            return {
                "state_map": self.state_map,
                "action_map": self.action_map,
                "policy": self.policy
            }

        return {
            "state_map": self._generate_state_map(),
            "action_map": self._generate_action_map(),
            "policy": self._generate_policy()
        }

    def is_goal(self, current_state):
        if current_state[0] == self.dropoff_location and current_state[1] == True:
            return True
        return False

def main():
    with open('../tmp/lgrc.json') as f:
        map = json.load(f)
        delivery-mdp = DeliveryMDP(map, 'shlomoOffice', 'AMRL')
        simulate(delivery-mdp, ('mailroom', False))

if __name__ == '__main__':
    main()