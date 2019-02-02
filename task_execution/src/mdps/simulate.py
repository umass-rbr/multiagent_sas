#!/usr/bin/env python

import delivery_mdp
import escort_mdp

def simulate(self, mdp, init):
    """ Simulate a trial starting at initial state. This will output a trace of the policy execution
        during runtime. 

        Returns:
            The reward/cost of a stochastic simulation following the optimal policy.
    """

    print(print("------------------------ Solving MDP Now ...."))
    mdp.solve()

    reward = 0
    s = mdp.state_map[init]

    print("------------------------Beginning simulation trace....")

    while not mdp.is_goal(s):
        a = mdp.policy[s]

        reward += mdp.R[s][a]

        print("*******\n" + 
              "s: " + str(s) + " | " + str(mdp.states[s]) + "\n" + 
              "a: " + str(a) + " | " + str(mdp.action_map[a]) + "\n" +
              "Reward: " + str(reward) + "\n" +
              "*******\n")

        # print("*******\n" + 
        #       "s: " + str(s) + " | " + str(mdp.states[s]) + "\n" + 
        #       "a: " + str(a) + " | " + str(mdp.action_map[a]) + "\n" +
        #       "sp: " + str(self.currentState) + " | " + str(self.execute_get_state()) + "\n" +
        #       "T[s][a][sp]: " + str(self.T[s][a][self.currentState]) + "\n" +
        #       "Reward: " + str(reward) + "\n" +

	print("------------------------Ending simulation trace....")

	print(str("Final state and cost: " + str(mdp.states[s])) + " | " + str(reward))