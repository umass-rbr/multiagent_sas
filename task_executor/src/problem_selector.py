import mdps
import rospy

DELIVERY_PROBLEM_TYPE = "delivery"
ESCORT_PROBLEM_TYPE = "escort"

# TODO Make sure task_assignment.problem lines up with the parameters of the MDP
def get(task_assignment):
    if task_assignment.task_type == DELIVERY_PROBLEM_TYPE:
        return mdps.delivery_mdp(task_assignment.problem)

    if task_assignment.task_type == ESCORT_PROBLEM_TYPE:
        return mdps.escort_mdp(task_assignment.problem)

    rospy.loginfo("Info[")
    return False
