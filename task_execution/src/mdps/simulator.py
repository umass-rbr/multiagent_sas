import numpy as np


# TODO Fix this function
def simulate(mdp, initial_state):
    print("Solving the MDP...")
    solution = mdp.solve()

    current_state = initial_state
    current_cumulative_reward = 0

    states = mdp.states

    print("Starting the simulation...")
    while not mdp.is_goal(current_state):
        current_state_index = solution["state_map"][current_state]
        current_action_index = solution["policy"][current_state_index]

        current_action = solution["action_map"][current_action_index]
        current_cumulative_reward += mdp.R[current_state_index][current_action_index]

        print(
            '********************************\n'
            + 'current_state_index:' + str(current_state_index) + "\n"
            + 'current_state:' + str(current_state) + "\n"
            + 'current_action_index:' + str(current_action_index) + "\n"
            + 'current_action:' + str(current_action) + "\n"
            + 'current_cumulative_reward:' + str(current_cumulative_reward) + "\n"
            + '********************************\n'
        )

        action_probability = np.random.uniform()
        threshold = 0
        for successor_state_index in range(mdp.mdp.n):
            threshold += mdp.T[current_state_index][current_action_index][successor_state_index]
            if action_probability <= threshold:
                current_state = states[successor_state_index]
                break

    print("Ending the simulation...")