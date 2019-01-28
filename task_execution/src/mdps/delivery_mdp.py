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
import json

thisFilePath = os.path.dirname(os.path.realpath(__file__))

sys.path.append(os.path.join(thisFilePath, "..", "..", "nova", "python"))
from nova.mdp import MDP
from nova.mdp_vi import MDPVI
from nova.mdp_value_function import MDPValueFunction

#import rospy
import itertools as it
import ctypes as ct
import numpy as np

import robot
import task
import worldMap

def power_set(iterable):    
    """ Return the power set of any iterable (e.g., list) with set elements. """

    s = list(iterable)
    powerSet = it.chain.from_iterable(it.combinations(s, r) for r in range(len(s) + 1))
    powerSetList = [set(ele) for ele in list(powerSet)]
    return powerSetList


class DeliveryMDP(object):
    """ The high-level task MDP that decides which tasks to complete. """


    def __init__(self, task):
        """ The constructor for the TaskMDP object. """
        self.mdp = None
        self.policy = None
        self.task = task
        self.world = json.load(open(worldMap,'r'))

        #self.currentState = (init_pos, False)

    def __str__(self):
        """ Make a pretty print of the MDP and its policy.

            Return:
                A pretty string.
        """

        result = "Delivery MDP:\n\n"
        if self.mdp is not None:
            result += str(self.mdp) + "\n\n"
        else:
            result += "Not yet defined.\n\n"

        result += "Delivery MDP Policy:\n\n"
        if self.policy is not None:
            result += str(self.policy)
        else:
            result += "Not yet defined.\n\n"

        return result

    def _compute_states(self):
        """ Compute the set of states from the state factores for the RTAMDP.

            Returns:
                S -- the list of states.
                s in S := (location, has_package_bool)
        """
        return list(it.product(self.world['locations'].keys(),[0,1]))

        
    def _compute_actions(self):
        """ Compute the set of all actions for the RTAMDP.

            Returns:
                A -- the list of actions.
        """
        return self.world['locations'].keys().append('pickup')

    def _compute_state_transitions(self):
        """ Compute the state transitions for the TaskMDP.

            This assumes states, actions, n, m, and ns are set.

            Returns:
                S   --  The n-m-ns lists of state indices.
                T   --  The n-m-ns lists of state transition probabilities.
        """

        S = [[[int(-1) for sp in range(self.mdp.ns)] for a in range(self.mdp.m)] for s in range(self.mdp.n)]
        T = [[[float(0.0) for sp in range(self.mdp.ns)] for a in range(self.mdp.m)] for s in range(self.mdp.n)]

        for s, state in enumerate(self.states):

            for a, action in enumerate(self.actions):
                if state[0] == self.task.end_location and state[1] == 1:
                    T[s][a][s] = 1.0
                    continue

                for sp, statePrime in enumerate(self.states):
                    S[s][a][sp] = sp

                    if action == 'pickup':
                        if state[1] == 0 and statePrime[1] == 1 and self.task.start_location == state[0] and state[0] == statePrime[0]:
                            T[s][a][sp] = 1.0
                            break

                    else: # Then action must be a location
                        if self.world['paths'][state[0]][action] and action == statePrime[0]:
                            T[s][a][sp] = 1.0
                            break


                # Uncomment to check if T sums to 1.
                # check = 0.0
                # for sp, statePrime in enumerate(self.states):
                #     check += T[s][a][sp]
                # print(check)
                # if round(check,3) != 1.0:
                #     print(check)
                #     print(state)
                #     print(tasks)
                #     print([r.get_id() for r in robots])
                #     quit()
        return S, T

    def _compute_rewards(self,T):
        """ Compute the rewards for the TaskMDP.
        
            This assumes states, actions, n, and m are set.

            Returns:
                R -- The n-m lists of rewards.
        """

        R = [[0.0 for a in range(self.mdp.m)] for s in range(self.mdp.n)]

        for s, state in enumerate(self.states):

            for a, action in enumerate(self.actions):
                if action == 'pickup':
                    if state[0] == self.task.start_location and state[1] == 0:
                        R[s][a] += 10
                else:
                    path_cost = self.world['paths'][state[0]][action]
                    if not path_cost:
                        R[s][a] = float("-inf")
                    else:
                        R[s][a] -= path_cost
        
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
        self.mdp.horizon = int(self.H)
        self.mdp.epsilon = float(0.001)
        self.mdp.s0 = int(self.mdp.n/(self.H+1)-1)
        self.mdp.ng = int(0)

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

    def solve(self):
        """ Use the nova MDP to compute the policy. """

        #rospy.loginfo("Info[TaskMDP.solve]: Solving the TaskMDP...")

        algorithm = MDPVI(self.mdp)

        timing = time.time()
        self.policy = algorithm.solve()
        timing = time.time() - timing

        print(self.mdp)
        print(self.policy)

        rospy.loginfo("Info[TaskMDP.solve]: Completed in %.3f seconds!" % (timing))

    def get_policy(self,):
        pi = {}
        policy_as_string = str(self.policy)[str(self.policy).index('p'):]
        policy_as_string = policy_as_string[policy_as_string.index('[')+1:policy_as_string.index(']')].strip()
        policy_as_string = policy_as_string.replace("\n","")
        policy_as_list = policy_as_string.split(" ")
        policy_as_list = [c for c in policy_as_list if c]
        for s in range(len(self.states)):
            pi[str(s)] = policy_as_list[s]
        return pi

    def save_policy(self, filename="task_mdp.policy"):
        """ Save the stored policy to a file.

            Parameters:
                filename    --  Optionally, a custom name of the file to save.
        """

        if self.policy is None:
            rospy.logerror("Error[TaskMDP.save_policy]: Policy is not yet defined.")
            raise Exception()

        rospy.loginfo("Info[TaskMDP.save_policy]: Saving the TaskMDP policy.")

        absoluteName = os.path.join(thisFilePath, filename)
        self.policy.save(absoluteName)

    def load_policy(self, filename="task_mdp.policy"):
        """ Load any stored policy from a file.

            Parameters:
                filename    --  Optionally, a custom name of the file to load.
        """

        rospy.loginfo("Info[TaskMDP.load_policy]: Loading the TaskMDP policy.")

        absoluteName = os.path.join(thisFilePath, filename)
        self.policy = MDPValueFunction()
        self.policy.load(absoluteName)

    def execute_reset(self):
        """ During execution, reset the TaskMDP's state to its initial state. """

        # Randomly pick an issue.
        #self.currentState = rnd.randint(1, len(self.states) - 1)

        # Always initalize to the active tasks: Clean and Medicate.
        self.currentState = self.mdp.s0

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
        """ During execution, execute the action and update the current state accordinly.

            Returns:
                True if action taken successfully.
                False if action was unsuccessful for any reason.
        """
        rand = np.random.uniform()
        thresh = 0
        for sp in range(self.mdp.n):
            thresh += self.mdp.T[self.currentState * (self.mdp.m* self.mdp.ns) + action * (self.mdp.ns) + sp]
            if rand <= thresh:
                self.currentState = sp
                return True
        return False

    def execute_update_state(self, successor):
        """ During execution, update the current state, but here we are removing (and adding) issues.

            Parameters:
                successor   --  The observed new state from self.states.
        """

        self.currentState = self.states.index(successor)

    def simulate(self):
        """ Simulate a trial starting at initial state. This will output a trace of the policy execution
            during runtime. 

            Returns:
                The reward/cost of a stochastic simulation following the optimal policy.
        """

        reward = 0
        self.execute_reset()
        while self.execute_get_state()[0] < self.mdp.horizon :
            s = self.currentState
            a = self.policy.action(self.currentState)

            if not (self.execute_take_action(a)):
                print("Error in executing action " + str(a))

            reward += self.R_full[s][a][self.currentState]

            a_out = ''
            for (task,robot) in self.actions[a]: #Reading action information into a readable string.
                a_out += "[ " + str(task) + " , " + str(robot.get_id()) + " ], "
            a_out = a_out[:-1] #Cropping the hanging comma.

            print("*******\n" + 
                  "s: " + str(s) + " | " + str(self.states[s]) + "\n" + 
                  "a: " + str(a) + " | " + a_out + "\n" +
                  "sp: " + str(self.currentState) + " | " + str(self.execute_get_state()) + "\n" +
                  "T[s][a][sp]: " + str(self.T[s][a][self.currentState]) + "\n" +
                  "Reward: " + str(reward) + "\n" +
                  "*******\n")

            #If there are no tasks left to perform, end the simulation.
            if len(self.execute_get_state()[1]) == 0: 
                break

        return (self.currentState, reward)

    def log_mdp_information(self, filename = "mdp_info.json"):
        """ Log the following information in a JSON file for future online use:
                - States with key as the state tuple and value being the state index.
                - Actions with key being the action index and value being the task-robot tuple list
                - Policy pi as is
        """

        mdp_info = {'states' : {}, 'actions' : {}, 'pi' : {}}

        for s in range(len(self.states)):
            state_as_set = self.states[s]
            mdp_info['states'][str(state_as_set)] = str(s)

        for a in range(len(self.actions)):
            mdp_info['actions'][a] = {}
            for (task,robot) in self.actions[a]:
                mdp_info['actions'][a][str(robot.get_id())] = str(task)

        policy_as_string = str(self.policy)[str(self.policy).index('p'):]
        policy_as_string = policy_as_string[policy_as_string.index('[')+1:policy_as_string.index(']')].strip()
        policy_as_string = policy_as_string.replace("\n","")
        policy_as_list = policy_as_string.split(" ")
        policy_as_list = [c for c in policy_as_list if c]
        for s in range(len(self.states)):
            mdp_info['pi'][str(s)] = policy_as_list[s]

        mdp_info = json.dumps(mdp_info, indent=4, sort_keys=True)
        absoluteName = os.path.join(thisFilePath, filename)
        ofile = open(absoluteName,'w+')
        ofile.write(mdp_info)
        ofile.close()

