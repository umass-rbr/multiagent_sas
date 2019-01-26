import ros
import utils

def excecute(mdp_info, state_estimator, task_type):
    current_state = state_estimator.get_state(task_type)
    action = utils.get_action(mdp_info, current_state)

    pub = rospy.Publisher('mdp_executor/action', Action)
    rospy.init_node('mdp_executor')

    msg = Action()
    msg.action = action
    rospy.loginfo(msg)
    rospy.publish