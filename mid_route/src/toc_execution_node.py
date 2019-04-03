#!/usr/bin/env python
import rospy
import os

current_file_path = os.path.dirname(os.path.realpath(__file__))

from mid_route.msg import TocMdpState, TocMdpAction
from mid_route.src.mdps import RouteMDP


toc_mdp_state_message = None

TOC_ACTION_PUBLISHER = rospy.Publisher("toc/action", TocMdpAction, queue_size=1)

def toc_mdp_state_callback(message):
    global toc_mdp_state_message
    toc_mdp_state_message = message

def execute(currentState, goalState, mapName):
	""" Publishes the action to take towards the goal based on current state to the toc/action state """

	# initialize the MDP
    route = RouteMDP(currentState, goalState, mapName)
    route.initialize()
    route.solve()

    # get the action for the current state
    action = route.get_action(currentState)

    # initialize the message to publish
    actionMessage = TocMdpAction()
    actionMessage.location = action[0]
    actionMessage.control = action[1] 

    # publish message to topic
    TOC_ACTION_PUBLISHER.publish(actionMessage)


def main():

    rospy.loginfo("Info[toc_execution_node.main]: Instantiating the toc_execution node...")
    rospy.init_node('toc_execution_node', anonymous=True)

    rospy.Subscriber('monitor/toc_mdp_state', TocMdpState, toc_mdp_state_callback)

    rospy.loginfo("Info[toc_execution_node.main]: Spinning...")
    rospy.spin()


if __name__ == '__main__':
    main()
