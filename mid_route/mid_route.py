""" The MIT License (MIT)

    Copyright (c) 2018 Kyle Hollins Wray, University of Massachusetts

    Permission is hereby granted, free of charge, to any person obtaining a copy of
    this software and associated documentation files (the "Software"), to deal in
    the Software without restriction, including without limitation the rights to
    use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
    the Software, and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
    FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
    COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import os
import sys
import time

thisFilePath = os.path.dirname(os.path.realpath(__file__))

sys.path.append(os.path.join(thisFilePath, "..", "..", "nova", "python"))
from nova.mdp import MDP
from nova.mdp_vi import MDPVI
from nova.mdp_value_function import MDPValueFunction

import campus_map

# import rospy

import itertools as it
import ctypes as ct
import numpy as np


def power_set(iterable):
    """ Return the power set of any iterable (e.g., list) with set elements. """

    s = list(iterable)
    powerSet = it.chain.from_iterable(it.combinations(s, r) for r in range(len(s) + 1))
    powerSetList = [set(ele) for ele in list(powerSet)]
    return powerSetList


class RouteMDP(object):
    """ The high-level route MDP that decides path for delivery. """

    def __init__(self, initialNode, goalNode):
        """ The constructor for the RouteMDP object. """

        self.initialNode = initialNode
        self.goalNode = goalNode
        self.map = campus_map.generate_map()  #The map of campus as a dictionary
        # self.failedTransitions = [0, 1, 2, 3, 4, 5]
        self.obstacles = ["none", "crosswalk"]
        # self.teleoperation = [True, False]

        self.mdp = None
        self.policy = None

        self.currentState = 0

    def __str__(self):
        """ Make a pretty print of the MDP and its policy.

            Return:
                A pretty string.
        """

        result = "Route MDP:\n\n"
        if self.mdp is not None:
            result += str(self.mdp) + "\n\n"
        else:
            result += "Not yet defined.\n\n"

        result += "Route MDP Policy:\n\n"
        if self.policy is not None:
            result += str(self.policy)
        else:
            result += "Not yet defined.\n\n"

        return result

    def _compute_states(self):
        """ Compute the set of states from the state factores for the RouteMDP.
            Returns:
                S -- the set of states.
                s in S := ([campus_map], [failed_transitions], [obstacles], [teleoperation])
        """

        # using nodes of map graph in map.keys()
        S = list(
	            it.product(
	            	list(self.map.keys()),
	            	# self.failedTransitions,
	            	self.obstacles
	            	# self.teleoperation
        		)
            )

        return S

    def _compute_actions(self):
        """ Compute the set of all actions for the RouteMDP.
            Returns:
                A -- the list of actions.
        """

        '''
        actions include:
            observe -- staying in current state to observe
            move -- moving to next state
            call -- calling for human help

        if failed_transition = 5 then call for human help

        '''

        A = list(self.map.keys())
        A.append("call")

        return A

    def _compute_state_transitions(self):
        """ Compute the state transitions for the RouteMDP.

            This assumes states, actions, n, m, and ns are set.

            Returns:
                S   --  The n-m-ns lists of state indices.
                T   --  The n-m-ns lists of state transition probabilities.
        """

        S = [[[int(-1) for sp in range(self.mdp.ns)] for a in range(self.mdp.m)] for s in range(self.mdp.n)]
        T = [[[float(0.0) for sp in range(self.mdp.ns)] for a in range(self.mdp.m)] for s in range(self.mdp.n)]

        for s, state in enumerate(self.states):
            for a, action in enumerate(self.actions):
                for sp, statePrime in enumerate(self.states):
                    S[s][a][sp] = sp
                    T[s][a][sp] = 0.0

                    # # first check for when failedTransitions is at max
                    # if state[1] == self.failed_transitions[-1] and action is "call":
                    # 	# go to any state that has teleoperation = True

                    # check for goal node
                    if state[0] == self.goalNode and action == self.goalNode and state[1] is "none":
                        # can only transition to itself
                        if statePrime[0] == self.goalNode and statePrime[1] is "none":
                            T[s][a][sp] = 1.0
                    # going from crosswalk state to goal node
                    elif state[0] == 4 and action == self.goalNode and state[1] is "crosswalk":
                        # moves from crosswalk to goal node
                        if statePrime[0] == self.goalNode and statePrime[1] is "none":
                                T[s][a][sp] = 1.0
                    # going from crosswalk node to crosswalk state
                    elif state[0] == 4 and action == "call" and state[1] is not "crosswalk":
                        # transitions to crosswalk state with same graph node
                        if statePrime[0] == 4 and statePrime[1] is "crosswalk":
                            T[s][a][sp] = 1.0
                    # going around the map on nodes
                    elif state[0] != 4 and state[0] != self.goalNode and action != self.goalNode and state[1] is "none":
                        # calculate valid moves on the map
                        validMoves = self.map[state[0]]
                        for destination in validMoves:
                            if action == destination[0] and statePrime[1] is "none" and statePrime[0] == destination[0]:
                                T[s][a][sp] = 1#/len(validMoves)
                    # # check for action call
                    # elif action is "call":
                    # 	# for now, you just get approved to move. duplicated code.
                    # 	nextNode = statePrime[0]
                    # 	for destination in validMoves:
                    # 		if destination is nextNode:
                    # 			T[s][a][sp] = 1.0
                    # check for valid moves on the map

                # TODO: Check if T sums to 1.
                check = 0.0
                for sp, statePrime in enumerate(self.states):
                   check += T[s][a][sp]
                print("State:", state, " ***** Action:", action, " ***** Check:", check)

        return S, T

    def _compute_rewards(self):
        """ Compute the rewards for the TaskMDP.
        
            This assumes states, actions, n, and m are set.

            Returns:
                R   --  The n-m lists of rewards.
        """

        R = [[0.0 for a in range(self.mdp.m)] for s in range(self.mdp.n)]

        for s, state in enumerate(self.states):
            for a, action in enumerate(self.actions):
				                
                # goal node stays in its own state
                if state[0] == self.goalNode and action == self.goalNode and state[1] is "none":
                    # becomes the terminal state in a way with no cost
                    R[s][a] = 1
                # cost for calling in crosswalk state
                elif state[0] == 4 and action is "call" and state[1] is "none":
                    R[s][a] -= 10
                # normal move has edge cost
                elif action is not "call":
                	# compute valid moves on map given state
                    validMoves = self.map[state[0]]
                    for destination in validMoves:
                        if destination[0] == action:
                            R[s][a] -= destination[1]
                else:
                    R[s][a] -= 200
        return R

    def initialize(self):
        """ Initialize the nova MDP using the map from 'snap' Cartographer. """

        self.states = self._compute_states()
        self.actions = self._compute_actions()

        self.mdp = MDP()
        self.mdp.n = int(len(self.states))
        self.mdp.ns = int(len(self.states))
        self.mdp.m = int(len(self.actions))
        self.mdp.gamma = float(0.99)
        self.mdp.horizon = int(1000)
        self.mdp.epsilon = float(0.001)
        self.mdp.s0 = int(0)
        self.mdp.ng = int(0)

        S, T = self._compute_state_transitions()
        array_type_nmns_int = ct.c_int * (self.mdp.n * self.mdp.m * self.mdp.ns)
        array_type_nmns_float = ct.c_float * (self.mdp.n * self.mdp.m * self.mdp.ns)
        self.mdp.S = array_type_nmns_int(*np.array(S).flatten())
        self.mdp.T = array_type_nmns_float(*np.array(T).flatten())

        R = self._compute_rewards()
        array_type_nm_float = ct.c_float * (self.mdp.n * self.mdp.m)
        self.mdp.R = array_type_nm_float(*np.array(R).flatten())
        self.mdp.Rmax = float(np.array(R).max())
        self.mdp.Rmin = float(np.array(R).min())

    def solve(self):
        """ Use the nova MDP to compute the policy. """

        # rospy.loginfo("Info[RouteMDP.solve]: Solving the RouteMDP...")

        algorithm = MDPVI(self.mdp)

        timing = time.time()
        self.policy = algorithm.solve()
        timing = time.time() - timing

        print(self.mdp)
        print(self.policy)

        # rospy.loginfo("Info[RouteMDP.solve]: Completed in %.3f seconds!" % (timing))

    def save_policy(self, filename="route_mdp.policy"):
        """ Save the stored policy to a file.

            Parameters:
                filename    --  Optionally, a custom name of the file to save.
        """

        if self.policy is None:
            #rospy.logerror("Error[RouteMDP.save_policy]: Policy is not yet defined.")
            raise Exception()

        #rospy.loginfo("Info[RouteMDP.save_policy]: Saving the RouteMDP policy.")

        absoluteName = os.path.join(thisFilePath, filename)
        self.policy.save(absoluteName)

    def load_policy(self, filename="route_mdp.policy"):
        """ Load any stored policy from a file.

            Parameters:
                filename    --  Optionally, a custom name of the file to load.
        """

        #rospy.loginfo("Info[RouteMDP.load_policy]: Loading the RouteMDP policy.")

        absoluteName = os.path.join(thisFilePath, filename)
        self.policy = MDPValueFunction()
        self.policy.load(absoluteName)

    def execute_reset(self):
        """ During execution, reset the RouteMDP's state to its initial state. """

        # Randomly pick an issue.
        #self.currentState = rnd.randint(1, len(self.states) - 1)

        # Always initalize to the active tasks: Clean and Medicate.
        self.currentState = self.states.index((self.initialNode, "none"))

    def execute_get_state(self):
        """ During execution, get the current state.

            Returns:
                The current state of the robot from self.states.
        """

        return self.states[self.currentState]

    def execute_get_action(self):
        """ During execution, get the action for the current state.

            Returns:
                The action to execute on the robot from self.actions.
        """

        a = self.policy.action(self.currentState)
        action = self.actions[a]

        return action

    def execute_take_action(self,action):
        rand = np.random.uniform()
        thresh = 0
        for sp in range(self.mdp.n):
            thresh += self.mdp.T[self.currentState * (self.mdp.m* self.mdp.ns) + action * (self.mdp.ns) + sp]
            if rand <= thresh:
                self.currentState = sp
                break
        return

    def execute_update_state(self, successor):
        """ During execution, update the current state, but here we are removing (and adding) issues.

            Parameters:
                successor   --  The observed new state from self.states.
        """

        self.currentState = self.states.index(successor)

    def simulate(self):
        """ Simulate a trial starting at initial state.
            Parameters:
            Returns:
                The reward/cost of a stochastic simulation following the optimal policy.
        """
        reward = 0
        R = self._compute_rewards()
        S, T = self._compute_state_transitions()
        self.execute_reset()
        while self.execute_get_state()[0] < self.mdp.horizon :
            s = self.currentState
            a = self.policy.action(self.currentState)
            self.execute_take_action(a)
            reward += R[s][a]

            a_out = "[ " + str(self.actions[a]) + " , " + " ], "

            print("*******\n" + 
                  "s: " + str(s) + " | " + str(self.states[s]) + "\n" + 
                  "a: " + str(a) + " | " + a_out + "\n" +
                  "sp: " + str(self.currentState) + " | " + str(self.execute_get_state()) + "\n" +
                  "T[s][a][sp]: " + str(T[s][a][self.currentState]) + "\n" +
                  "Reward: " + str(reward) + "\n" +
                  "*******\n")

            if s == self.currentState or len(self.execute_get_state()[1]) == 0: break

        return (self.currentState, reward)

if __name__ == "__main__":
    goalNode = 7
    initialNode = 0
    route = RouteMDP(initialNode, goalNode)
    route.initialize()
    route.solve()
    route.simulate()