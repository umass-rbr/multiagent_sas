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

import rospy

import itertools as it
import ctypes as ct
import numpy as np


def power_set(iterable):
    """ Return the power set of any iterable (e.g., list) with set elements. """

    s = list(iterable)
    powerSet = it.chain.from_iterable(it.combinations(s, r) for r in range(len(s) + 1))
    powerSetList = [set(ele) for ele in list(powerSet)]
    return powerSetList


class RTAMDP(object):
    """ The high-level task MDP that decides which tasks to complete. """

    def __init__(self,tasks,robots,H=4,delta=0.5):
        """ The constructor for the TaskMDP object. """

        self.tasks = tasks
        self.robots = robots
        self.H = H
        self.delta = delta

        self.mdp = None
        self.policy = None

        self.currentState = 0

    def __str__(self):
        """ Make a pretty print of the MDP and its policy.

            Return:
                A pretty string.
        """

        result = "RTA MDP:\n\n"
        if self.mdp is not None:
            result += str(self.mdp) + "\n\n"
        else:
            result += "Not yet defined.\n\n"

        result += "RTA MDP Policy:\n\n"
        if self.policy is not None:
            result += str(self.policy)
        else:
            result += "Not yet defined.\n\n"

        return result

    def _compute_states(self):
        """ Compute the set of states from the state factores for the RTAMDP.

            Returns:
                S -- the list of states.
                s in S := (time, [current-tasks], [robot-conditions], [task-robot-assignments])
        """

        matchings = list(itertools.product(self.tasks,self.robots))
        times = [i for i in range(self.H)]

        S = list(itertools.product(times,list(power_set(self.tasks))))
        S = list(itertools.product(S,list(itertools.product([0,1],repeat=len(self.robots)))))
        S = list(itertools.product(S,list(power_set(matchings))))

        return S

    def _compute_actions(self):
        """ Compute the set of all actions for the RTAMDP.

            Returns:
                A -- the list of actions.
        """
        A = list(power_set(list(itertools.product(self.T,self.R))))
        A = [a for a in A if len(a) <= len(self.tasks)]

        new_A = []
        for a in A:
            t_check = [0 for t in self.Tasks]
            r_check = [0 for r in self.Robots]
            check = True
            for (t,r) in p:
                if t_check[t] == 1:
                    check = False
                    break
                elif r_check[r] == 1:
                    check = False
                    break
                else:
                    t_check[t] = 1
                    r_check[r] = 1
            if check is True: new_A.append(a)

        return new_A

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
                for sp, statePrime in enumerate(self.states):
                    S[s][a][sp] = sp

                    if statePrime[0] == state[0] + 1
                        T[s][a][sp] = 1.0

                    for (task,robot) in zip(a):
                        if (statePrime[0] <= task.start() or statePrime[0] > task.end()) and task not in statePrime[1]:
                            T[s][a][sp] = 0.0
                            continue
                        if task not in state[1]:
                            T[s][a][sp] = 0.0
                            continue
                        if state[2][robot.id()] > 0 and statePrime[2][robot.id()] > 0:
                            if task not in statePrime[1]: T[s][a][sp] = 0.0
                            continue
                        elif state[2][robot.id()] > 0 and statePrime[2][robot.id()] == 0:
                            T[s][a][sp] = 0.0
                            continue
                        elif state[2][robot.id()] == 0 and statePrime[2][robot.id()] > 0:
                            if task in statePrime[1]: T[s][a][sp] *= robot.get_break_probability()
                            else: T[s][a][sp] = 0.0 
                        else:
                            if task not in statePrime[1]: T[s][a][sp] *= (1.0 - robot.get_break_probability())
                            else: T[s][a][sp] = 0.0

                # TODO: Check if T sums to 1.
                check = 0.0
                for sp, statePrime in enumerate(self.states):
                   check += T[s][a][sp]
                print(check)

        return S, T

    def _compute_rewards(self):
        """ Compute the rewards for the TaskMDP.
        
            This assumes states, actions, n, and m are set.

            Returns:
                R   --  The n-m-ns lists of rewards.
        """

        R = [[[0.0 for sp in range(self.mdp.n)] for a in range(self.mdp.m)] for s in range(self.mdp.n)]

        for s, state in enumerate(self.states):
            for a, action in enumerate(self.actions):
                for sp, statePrime in enumerate(self.states):
                    for (task,robot) in zip(a):
                        if task.end() < statePrime[0]:
                            R[s][a][sp] -= self.delta
                        elif task in statePrime[1]:
                            R[s][a][sp] -= self.delta
                            if state[2][robot.id()] == 0 and statePrime[2][robot.id()] > 0:
                                R[s][a][sp] -= robot.get_break_cost(statePrime[2][robot.id()])
                        else:
                            R[s][a][sp] -= robot.calculate_path_cost(task.pickup(),task.dropoff())

        return R

    def initialize(self):
        """ Initialize the nova MDP using the map from 'snap' Cartographer. """

        self.states = _compute_states(self)
        self.actions = _compute_actions(self)

        self.mdp = MDP()
        self.mdp.n = int(len(self.states))
        self.mdp.ns = int(len(self.states))
        self.mdp.m = int(len(self.actions))
        self.mdp.gamma = float(0.99)
        self.mdp.horizon = int(self.H)
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

        rospy.loginfo("Info[TaskMDP.solve]: Solving the TaskMDP...")

        algorithm = MDPVI(self.mdp)

        timing = time.time()
        self.policy = algorithm.solve()
        timing = time.time() - timing

        print(self.mdp)
        print(self.policy)

        rospy.loginfo("Info[TaskMDP.solve]: Completed in %.3f seconds!" % (timing))

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
        self.currentState = self.states.index({"Clean", "Medicate"})

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

    def execute_update_state(self, successor):
        """ During execution, update the current state, but here we are removing (and adding) issues.

            Parameters:
                successor   --  The observed new state from self.states.
        """

        self.currentState = self.states.index(successor)