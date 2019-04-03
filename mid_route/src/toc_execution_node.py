#!/usr/bin/env python
import rospy
import os

current_file_path = os.path.dirname(os.path.realpath(__file__))

from mid_route.msg import TocMdpState, TocMdpAction
from mid_route.src.mdps import RouteMDP


toc_mdp_state_message = None

TOC_ACTION_PUBLISHER = rospy.Publisher("orb_nav/goal", TocMdpAction, queue_size=1)

def toc_mdp_state_callback(message):
    global toc_mdp_state_message
    toc_mdp_state_message = message

def execute(currentState, goalState, mapName):

    route = RouteMDP(currentState, goalState, mapName)
    route.initialize()
    route.solve()
    action = route.get_action(currentState)

    actionMessage = TocMdpAction()
    actionMessage.location = action[0]
    actionMessage.control = action[1] 

    TOC_ACTION_PUBLISHER.publish(actionMessage)


def main():

    rospy.loginfo("Info[toc_execution_node.main]: Instantiating the toc_execution node...")
    rospy.init_node('toc_execution_node', anonymous=True)

    rospy.Subscriber('monitor/toc_mdp_state', String, callback)

    rospy.loginfo("Info[toc_execution_node.main]: Spinning...")
    rospy.spin()


if __name__ == '__main__':
    main()
