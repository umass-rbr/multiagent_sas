import sys
import os
import json

import rospy

import robot
import task
import problem_generator

#TO DO: Implement the server capabilities.
# - Should receieve 'current state of world' from something
# - Should send out each problem file to the respective robot.


WORLDS_DIRECTORY = "worlds/"

with open('mdp_info.json','r') as f:
    mdp_info = json.loads(f)
state_map = mdp_info['states']
action_map = mdp_info['actions']
pi = mdp_info['pi']

current_state = status.get_current_state() # There needs to be something to call to get the current state
                                           # Alternatively we can have this passed in as a parameter?

action_to_take = actions[policy[states[currentState]]]

with open(WORLDS_DIRECTORY + "world.json", "r") as world_file:
    for (t,r) in action_to_take:
        # TO DO: Either need to make t into a task object here,
        # or change high_rta to function task objects. This second thing is
        # probably better design.
        problem_file = open( (str(r.get_ID()) + "_assignment.pddl") , "w+")
       problem_file.write(problem_generator.generate_escort_problem(r,t,json.load(world_file)))