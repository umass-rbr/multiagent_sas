import os, sys, time, random, pickle

import numpy as np
import pandas as pd
import itertools as it

from IPython import embed

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..', '..'))

from CDB_ps.path_domain_model import PathDomainModel

class PerceptionSensitiveMDP():
    def __init__(self, MDP, error_info):
        self.MDP = MDP
        self.error_info = error_info
        self.r = [0,1]              # r == 1: RELIABLE, r == 0: UNRELIABLE
        self._states, self._actions = self.generate_states(self.MDP), self.generate_actions(self.MDP)
        self.eta, self.phi = self.load_eta(), self.load_phi()
        self._init, self._goal = (self.mdp.init, 1), (self.mdp.goal, 1)
        self._transitions, self._costs = self.compute_transitions(), self.compute_costs()

        self.pi, self.V, self.Q, self.state_map = None, None, None, None


    def generate_states(self):
        return list(it.product(self.MDP.states, self.r)) + ['FAILURE']


    @property
    def states(self):
        return self._states
    

    def set_init(self):
        return (self.MDP.init, 1)


    @property
    def init(self):
        return self._init
    

    def set_goal(self):
        return (self.MDP.goal, 1)


    def set_random_task(self):
        self.MDP.set_random_task()
        self.set_init()
        self.set_goal()
        self.compute_transitions()
        self.compute_costs()


    def reset(self):
        self.compute_transitions()
        self.compute_costs()


    @property
    def goal(self):
        return self._goal
    

    def generate_actions(self):
        return self.MDP.actions + ['QUERY']


    @property
    def actions(self):
        return self._actions


    def compute_transitions(self):
        T = np.ones((len(self.states), len(self.actions), len(self.states)))

        for s, state in enumerate(self.MDP.states):
            if state == 'FAILURE':
                T[s][:][s] = 1.
                continue
            for r1 in [1, 0]:
                s_tilde = s + r1
                for a, action in enumerate(self.MDP.actions):
                    if action == 'QUERY':
                        if r1 == 1:
                            T[s_tilde][a][s_tilde] = 1.
                            continue
                        else:
                            T[s_tilde][a][s_tilde - 1] = 1.
                            continue
                    else:
                        for sp, statePrime in enumerate(self.MDP.states):
                            if statePrime == 'FAILURE':
                                T[s_tilde][a][sp] = self.phi((state, r1), a)
                            else:
                                for r2 in [1, 0]:
                                    sp_tilde = sp + r2
                                    if r2 == 1:
                                        T[s_tilde][a][sp_tilde] = (self.MDP.transitions[s][a][sp] 
                                            * (1 - self.phi[s_tilde][a]) * (1 - self.eta[s_tilde][a][sp]))
                                    if r2 == 0:
                                        T[s_tilde][a][sp_tilde] = (self.MDP.transitions[s][a][sp] 
                                            * (1 - self.phi[s_tilde][a]) * (self.eta[s_tilde][a][sp]))
        self._transitions = T


    @property
    def transitions(self):
        return self._transitions


    def compute_costs(self):
        C = np.zeros((len(self.states), len(self.actions)))
        for s, state in enumerate(self.MDP.states):
            if state == 'FAILURE':
                C[s] = 100.0
                continue
            for a, action in enumerate(self.MDP.actions):
                for r in [0,1]:
                    if action == 'QUERY':
                        C[s+r][a] = 5.0
                    else:
                        C[s+r][a] = self.MDP.costs[s][a]
        self._costs = C


    @property
    def costs(self):
        return self._costs
    

    def load_eta(self):
        # with open(ETA_FILE_PATH, mode='rb') as f:
        #     eta = pickle.load(f, encoding='bytes')
        _eta = np.zeros((len(self.states), len(self.actions), int((len(self.states)-1)/len(self.r))))
        return _eta
        # for s, state in enumerate(self.states):
        #     for a, action in enumerate(self.actions):
        #         for sp, statePrime in enumerate(self.states):
        #             if state in eta.keys() and action in eta[state].keys() and statePrime in eta[state][action].keys():
        #                 _eta[s][a][sp] = eta[state][action][statePrime]

        # return _eta


    def load_phi(self):
        # with open(PHI_FILE_PATH, mode='rb') as f:
        #     phi = pickle.load(f, encoding='bytes')
        _phi = np.zeros((len(self.states), len(self.actions)))

        for s, state in enumerate(self.states):
            for a, action in enumerate(self.actions):
                if state[0][0] in self.error_info.keys() and action[0][1] in self.error_info[state].keys():
                    _phi[s][a] = self.error_info[state[0][0]][action[0][1]]

        return _phi


    def solve(self):
        start_time = time.time()
        mdp_info = FVI(self)
        end_time = time.time()

        self.pi = mdp_info['pi']
        self.state_map = mdp_info['state map']
        self.V = mdp_info['V']
        self.Q = mdp_info['Q']


    def generate_successor(self, state, action):
        """
            params:
                state - The state we are generating a successor for.
                action - The action we are generating a successor for.

            returns:
                successor - The successor state that the agent arrives in
                            when taking 'action' in 'state', as determined
                            by the transition function. 
        """
        s = self.states.index(state)
        a = self.actions.index(action)

        rand = np.random.uniform()
        thresh = 0.0

        for sp in range(len(self.states)):
            thresh += self.transitions[s][a][sp]
            if rand <= thresh:
                return self.states[sp]