import json
import rospy

from msg import TaskAssignment

import problem_generator
import robot
import task
import utils

WORLDS_DIRECTORY = "worlds/"
TASK_DURATION = 900

def main():
    # TODO Ask Sadegh about convention
    # TODO Ask Sadegh about node structure
    pub = rospy.Publisher('task_assignment', TaskAssignment)
    rospy.init_node('TaskAssigner')

    with open('mdp_info.json', 'r') as mdp_info_file:
        mdp_info = json.loads(mdp_info_file)

        with open(WORLDS_DIRECTORY + "world.json", "r") as world_file:
            world = json.load(world_file)

            while not rospy.is_shutdown():
                # TODO Query state from some source
                current_state = status.get_current_state()
                current_action = utils.get_action(mdp_info, current_state)

                for t, r in current_action:
                    msg = TaskAssignment()
                    msg.robot_id = r.get_id()
                    msg.problem = problem_generator.generate_escort_problem(r, t, world)

                    rospy.loginfo(msg)
                    pub.publish(msg)

                rospy.sleep(TASK_DURATION)


if __name__ == "__main__":
    main()
