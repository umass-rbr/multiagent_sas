import json

import problem_generator
import robot
import rospy
import task

WORLDS_DIRECTORY = "worlds/"

def main():
    with open('mdp_info.json','r') as f:
        mdp_info = json.loads(f)

        state_map = mdp_info['states']
        action_map = mdp_info['actions']
        policy = mdp_info['pi']

        # TODO Query state from some source
        current_state = status.get_current_state()

        recommended_action = action_map[policy[state_map[current_state]]]

        with open(WORLDS_DIRECTORY + "world.json", "r") as world_file:
            for t, r in recommended_action:
                # TODO Either need to make it into a task object here, or change high_rta to 
                # function task objects. The second option is probably better design.
                problem_file = open((str(r.get_id()) + "_assignment.pddl"), "w+")
                problem_file.write(problem_generator.generate_escort_problem(r, t, json.load(world_file)))


if __name__ == "__main__":
    main()
