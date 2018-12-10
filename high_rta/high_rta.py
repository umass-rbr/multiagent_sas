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

import Robot

thisFilePath = os.path.dirname(os.path.realpath(__file__))

sys.path.append(os.path.join(thisFilePath, "..", "..", "nova", "python"))
from nova.mdp import MDP
from nova.mdp_vi import MDPVI
from nova.mdp_value_function import MDPValueFunction

#import rospy

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

    def __init__(self,tasks,robots,H=4,delta=30):
        """ The constructor for the TaskMDP object. """

        self.tasks = tasks #Tasks are tuples: <start_time, end_time, pickup, dropoff>
        self.robots = robots #Robots are objects implemented in Robot.py
        self.H = H #H is the Horizon
        self.delta = delta #delta is the length of each 'action'

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
                s in S := (time, {current-tasks}, [robot-conditions])
        """

        #matchings = list(it.product(self.tasks,self.robots))
        times = [i for i in range(self.H)]
        robot_list = [list(r) for r in list(it.product([0,1],repeat=len(self.robots)-1))]
        for r in robot_list: r.append(1)
        S = list(it.product(times,list(power_set(self.tasks)),robot_list))
        return S
        
    def _compute_actions(self):
        """ Compute the set of all actions for the RTAMDP.

            Returns:
                A -- the list of actions.
        """
        A = list(power_set(list(it.product(self.tasks,self.robots))))
        A = [a for a in A if len(a) <= len(self.tasks)]

        new_A = []
        for a in A:
            t_check = [0 for t in self.tasks]
            r_check = [0 for r in self.robots]
            check = True
            for (t,r) in a:
                if t_check[self.tasks.index(t)] == 1:
                    check = False
                    break
                elif r_check[self.robots.index(r)] == 1:
                    check = False
                    break
                else:
                    t_check[self.tasks.index(t)] = 1
                    r_check[self.robots.index(r)] = 1
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
            if state[0] == self.H -1:
                T[s][a][s] = 1.0
                continue
            for a, action in enumerate(self.actions):
                tasks = []
                #robots = []
                for (task,robot) in action:
                    tasks.append(task)
                    #robots.append(robot)
                for sp, statePrime in enumerate(self.states):
                    S[s][a][sp] = sp

                    #Can only go to states at next time step.
                    if statePrime[0] != state[0] + 1:
                        T[s][a][sp] = 0.0
                        continue

                    #If the action set is empty or there are no tasks then only one transition is possible.
                    if len(action) == 0 or len(state[1]) == 0:
                        if statePrime[1] == state[1] and statePrime[2] == state[2]:
                            T[s][a][sp] = 1.0
                            break
                        else:
                            continue

                    #Second check that state's tasks contains all tasks in statePrime U action.tasks
                    if not state[1].issuperset(statePrime[1].union(tasks)):
                        T[s][a][sp] = 0.0
                        continue

                    #Next make sure that we don't drop any tasks from the state.
                    if len(statePrime[1].union(tasks)) < len(state[1]):
                        T[s][a][sp] = 0.0
                        continue

                    for (task,robot) in action:
                        #Make sure a broken robot is not completing a task 
                        if state[2][robot.get_id()] == 0 and task not in statePrime[1]:
                            T[s][a][sp] = 0.0
                            break
                        #If robot breaks
                        if state[2][robot.get_id()] == 1 and statePrime[2][robot.get_id()] == 0:
                            if task in state[1] and task in statePrime[1]: #And the task was not erroneously completed
                                if T[s][a][sp] == 0:
                                    T[s][a][sp] = robot.get_break_probability(task[2],task[3])
                                else:
                                    T[s][a][sp] = T[s][a][sp]*robot.get_break_probability(task[2],task[3])
                            else: 
                                T[s][a][sp] = 0.0
                                break
                        #If robot doesn't break
                        if state[2][robot.get_id()] == 1 and statePrime[2][robot.get_id()] == 1:
                            if task in state[1] and task not in statePrime[1]: #And task was completed
                                if T[s][a][sp] == 0:
                                    T[s][a][sp] = (1-robot.get_break_probability(task[2],task[3]))
                                else:
                                    T[s][a][sp] = T[s][a][sp]*(1-robot.get_break_probability(task[2],task[3]))
                            else:
                                T[s][a][sp] = 0.0
                                break

                    #Basically make sure that the robot status list is consistent
                    for i in range(len(state[2])):
                        if state[2][i] == 0 and statePrime[2][i] == 1:
                            T[s][a][sp] = 0.0
                        if state[2][i] == 1 and statePrime[2][i] == 0:
                            r_check = False
                            for (task,robot) in action:
                                if robot.get_id() == i: r_check = True
                            if not r_check: T[s][a][sp] = 0.0

                if np.sum(T[s][a]) == 0.0:
                    for sp,statePrime in enumerate(self.states):
                        if statePrime[0] == (state[0]+1) and len(statePrime[1]) == 0 and statePrime[2] == state[2]:
                            T[s][a][sp] = 1.0
                            break
                # TODO: Check if T sums to 1.
                # check = 0.0
                # for sp, statePrime in enumerate(self.states):
                #     check += T[s][a][sp]
                # print(check)
                # if round(check,3) != 1.0:
                #     print(state)
                #     print(tasks)
                #     print([r.get_id() for r in robots])
                #     #print(T[s][a])
                #     quit()
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
                    for task in statePrime[1]:
                        R[s][a][sp] -= self.delta
                    for (task,robot) in action:
                        if task in statePrime[1]:
                            if state[2][robot.get_id()] == 1 and statePrime[2][robot.get_id()] == 0:
                                #R[s][a][sp] -= robot.get_break_cost(statePrime[2][robot.get_id()])
                                R[s][a][sp] -= 100
                        else:
                            R[s][a][sp] -= robot.calculate_time(task[2],task[3])
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
        self.mdp.s0 = int(len(self.states)/self.H)-1
        self.mdp.ng = int(0)

        S, T = self._compute_state_transitions()
        array_type_nmns_int = ct.c_int * (self.mdp.n * self.mdp.m * self.mdp.ns)
        array_type_nmns_float = ct.c_float * (self.mdp.n * self.mdp.m * self.mdp.ns)
        self.mdp.S = array_type_nmns_int(*np.array(S).flatten())
        self.mdp.T = array_type_nmns_float(*np.array(T).flatten())

        R = self._compute_rewards()
        #array_type_nms_float = ct.c_float * (self.mdp.n * self.mdp.m * self.mdp.ns)
        self.mdp.R = array_type_nmns_float(*np.array(R).flatten())
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

        #rospy.loginfo("Info[TaskMDP.solve]: Completed in %.3f seconds!" % (timing))

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

def main():
    T = [ (0,3,0,1),
          (0,2,1,0),
          (1,4,1,2),
          (3,4,2,1),
          (3,4,3,2) ]
    R = [Robot.robot(0,0,0), Robot.robot(1,1,3), Robot.robot(2,2,2)]
    rta = RTAMDP(T,R)
    rta.initialize()
    rta.solve()

main()