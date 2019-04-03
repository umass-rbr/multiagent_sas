#!/usr/bin/env python
import rospy
import os

currentFilePath = os.path.dirname(os.path.realpath(__file__))

from mid_route.msg import TocMdpState, TocMdpAction
from mid_route.src.mdps import RouteMDP


tocMdpStateMessage = None

TOC_ACTION_PUBLISHER = rospy.Publisher("toc/action", TocMdpAction, queue_size=1)

def toc_mdp_state_callback(message):
    global tocMdpStateMessage
    tocMdpStateMessage = message

def load_json_map():
    """ Loads a map from json file to a dictionary for input into MDP """

    with open(currentFilePath + '/tmp/map.json') as mapFile:
    	data = json.load(mapFile)

    for attribute in data:
        if attribute == "paths":
            mapPaths = data[attribute]

    loadedMap = dict()

    for location in mapPaths.keys():

        locationEdges = mapPaths[location]

        # edge connections include the destination, cost, and obstruction
        edgeConnections = []

        for destination in locationEdges.keys():

            destinationName = destination     

            for destinationAttribute in locationEdges[destination]:

                if destinationAttribute == "cost":
                    destinationCost = locationEdges[destination][destinationAttribute]

                elif destinationAttribute == "obstruction":
                    destinationObstruction = locationEdges[destination][destinationAttribute]

            edgeConnections.append( (destinationName, destinationCost, destinationObstruction) )

        loadedMap[location] = edgeConnections

    return loadedMap

def execute(currentState, goalState):
	""" Publishes the action to take towards the goal based on current state to the toc/action state """

	# load map file
	campusMap = load_json_map()

	# initialize the MDP
    route = RouteMDP(currentState, goalState, campusMap)
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
