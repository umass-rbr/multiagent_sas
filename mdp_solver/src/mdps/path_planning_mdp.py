import os, sys, time, json, pickle, random

from collections import defaultdict
from IPython import embed
import numpy as np
import pandas as pd
import itertools as it

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..', '..'))

# from CDB_ma.domain_helper import Helper
from scripts.utils import FVI

DOMAIN_PATH = os.path.join(current_file_path, '..', '..', '..', 'domains', 'CDB_ps')
MAP_PATH = os.path.join(DOMAIN_PATH, 'maps')

DIRECTIONS = [(1,0),(0,-1),(-1,0),(0,1)]
MAPPING = {(-1,0): 'NORTH', (0,1) : 'EAST', (1,0) : 'SOUTH', (0,-1): 'WEST'}
REV_MAPPING = {'NORTH': (-1,0), 'EAST': (0,1), 'SOUTH': (1,0), 'WEST': (0,-1)}

def dist(x1, y1, x2, y2):
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def list_to_dic(list):
    node_dic = {}
    for ele in list:
        node_dic[ele['id']] =  {'loc': ele['loc']}

def point_to_line_dist(p1, p2, p3):
    p1, p2, p3 = np.array(p1), np.array(p2), np.array(p3)
    d=np.cross(p2-p1,p3-p1)/norm(p2-p1)


class PathDomainModel():
    def __init__(self, map_info, error_info, gamma=1.0):
        self.gamma = gamma
        self.error_info = error_info
        self.vertices, self.edges = list_to_dic(map_info['nodes']), map_info['edges']
        self._states, self._actions = self.generate_states(), self.generate_actions()
        self._init, self._goal = None, None
        self._transitions, self._costs = None, None
        # self.check_validity()

        self.pi, self.V, self.Q, self.state_map = None, None, None, None


    def generate_states(self):
        states = []
        for k,vertex in self.vertices.items():
            states.append((k, vertex['loc']['x'], vertex['loc'['y']]))
        return states + ['FAILURE']


    @property
    def states(self):
        return self._states
    

    def generate_actions(self):
        actions = []
        for edge in self.edges.values():
            actions.append((edge['s0_id'], edge['s1_id']))
        return actions


    @property
    def actions(self):
        return self._actions
    

    def set_init(self, node_id):
        self.init = tuple(node_id, self.vertices[node_id]['loc']['x'], self.vertices[node_id]['loc']['y'])


    @property
    def init(self):
        return self._init
    

    def set_goal(self, node_id):
        self.goal = tuple(node_id, self.vertices[node_id]['loc']['x'], self.vertices[node_id]['loc']['y'])


    @property
    def goal(self):
        return self._goal


    def set_random_task(self):
        self.set_init(np.random.choice(np.array(self.vertices.keys())))
        self.set_goal(np.random.choice(np.array(self.vertices.keys())))
        while self.goal == self.init:
            self.set_goal(np.random.choice(np.array(self.vertices.keys())))
        self.compute_transitions()
        self.compute_costs()
    

    def compute_transitions(self):
        T = np.array([[[0.0 for sp in range(len(self.states))]
                            for a in range(len(self.actions))]
                            for s in range(len(self.states))])

        for s, state in enumerate(self.states):
            if state == self.goal or state == 'FAILURE':
                T[s][:,s] = 1.0
                continue
            for a, action in enumerate(self.actions):
                if state[0] != action[0]:
                    T[s][a][s] = 1.0
                    continue
                else:
                    statePrime = (action[1], self.nodes[action[1]]['loc']['x'], self.nodes[action[1]]['loc']['y'])
                    sp = self.states.index(statePrime)
                    T[s][a][sp] = 1.0
                    if state[0] in self.error_info.keys() and action[1] in self.error_info[state[0]].keys():
                        T[s][a][sp] *= self.error_info[state[0]][action[1]]
                        T[s][a][len(self.states)-1] = self.error_info[state[0]][action[1]]
                if np.sum(T[s][a]) == 0.0:
                    T[s][a][s] = 1.0

        self._transitions = T


    @property
    def transition(self):
        return self._transition
    

    def compute_costs(self):
        C = np.array([[1.0 for a in range(len(self.actions))] 
                           for s in range(len(self.states))])

        for s, state in enumerate(self.states):
            for e, edge in enumerate(self.edges):
                if edge['s0_id'] == state[0]:
                    action = (edge['s0_id'], edge['s1_id'])
                    a = self.actions.index(action)
                    C[s][a] = (dist(self.V[str(action[0])]['loc']['x'], self.V[str(action[0])]['loc']['y'],
                            self.V[str(action[1])]['loc']['x'], self.V[str(action[1])]['loc']['x'])/ edge['max_speed'])

        C[self.states.index(self.goal)] *= 0.0

        self._costs = C


    @property
    def costs(self):
        return self._costs


    def reset(self):
        self.compute_transitions()
        self.compute_costs()
    

    def check_validity(self):
        for s in range(len(self.states)):
            for a in range(len(self.actions)):

                if round(np.sum(self.transitions[s][a]),3) != 1.0:
                    print("Error @ state " + str(self.states[s]) + " and action " + str(self.actions[a]))
                    embed()
                    quit()


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