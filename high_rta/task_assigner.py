import json
import rospy

from msg import Problem

import problem_generator
import robot
import task
import utils

WORLDS_DIRECTORY = "worlds/"

def main():
    pub = rospy.Publisher('Task Assigner', Problem)

    with open('mdp_info.json', 'r') as f:
        mdp_info = json.loads(f)

        # TODO Query state from some source
        current_state = status.get_current_state()
        current_action = utils.get_action(mdp_info, current_state)

        with open(WORLDS_DIRECTORY + "world.json", "r") as world_file:
            for t, r in current_action:
                # TODO Either need to make it into a task object here, or change high_rta to 
                # function task objects. The second option is probably better design.
                rospy.init_node('Assigner')
                r = rospy.Rate(10)
                msg = Problem()
                msg.problem = problem_generator.generate_escort_problem(r,t,json.load(world_file))

                while not rospy.is_shutdown():
                    rospy.log_info(msg)
                    pub.publish(msg)
                    r.sleep()


if __name__ == "__main__":
    main()
