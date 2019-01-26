import execution_selector
import rospy
from std_msgs.msg import String, TaskAssignment

ROBOT_ID = "pumpkin"

def task_handler(task_assignment):
    # TODO Parameterize the model
    # TODO Send details to the execute function
    if task_assignment.robot_id == ROBOT_ID:
        executor, model = execution_selector.select(task_assignment.task_type)

        model = 
        model.initialize()
        policy = model.solve()

        executor.execute(policy, goal_state)


def listener():
    rospy.init_node("task_listener", anonymous=True)
    rospy.Subscriber("task_assigner/task_assignment", TaskAssignment, task_handler)
    rospy.spin()


if __name__ == '__main__':
    listener()
