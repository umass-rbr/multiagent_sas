import numpy as np


# TODO Fix this function
def simulate(mdp, initial_state):
    print("Solving the MDP...")
    solution = mdp.solve()

    current_state = initial_state
    current_cumulative_reward = 0

    print("Starting the simulation...")
    while not mdp.is_goal(current_state):
        current_state_index = solution["state_map"][current_state]
        current_action_index = solution["policy"][current_state_index]

        current_action = solution["action_map"][current_action_index]
        current_cumulative_reward += mdp.R[current_state_index][current_action_index]

        action_probability = np.random.uniform()
        threshold = 0
        for successor_state_index in range(mdp.mdp.n):
            threshold += mdp.T[current_state_index * (mdp.mdp.m * mdp.mdp.ns) + current_action_index * (mdp.mdp.ns) + successor_state_index]
            if action_probability <= threshold:
                current_state = solution["state_map"][successor_state_index]

        print({
            'current_state_index': current_state_index,
            'current_state': current_state,
            'current_action_index': current_action_index,
            'current_action': current_action,
            'current_cumulative_reward': current_cumulative_reward
        })

    print("Ending the simulation...")