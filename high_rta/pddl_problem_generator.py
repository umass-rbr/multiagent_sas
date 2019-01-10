import sys
import os
import json

import robot
import mdp_info # JSON file storing mdp information (states, actions, policy)

#This is a temporary placeholder. Need something which will return the current state of the world
import status

if __name__ == '__main__':
	'''
	This file is used to generate PDDL Problem Files for the various robots in working condition.
	This file should have access to the current state of the world:
		- The curernt time t.
		- The set of uncompleted tasks within the horizon.
		- The current list of functioning robots.
	It will then access the precomputed policy for this problem and find the action to take for the current state.
	For every (task,robot) in action_to_take, generate a PDDL problem file defining that task as a problem
	for the robot in question. Naming of problem file will be standardized to <robot_id>_assignment.pddl
	so that each problem's assigned robot can be easily identified as the first character of the string.

	In the future we will want to generate multiple types of tasks.
	TO DO:
		- Implement a task.py class that has a task_type value which will be helpful for parsing.
		- We will offload the task time and break calculations to the task class, which will access
			and use Jason's route planner and keep that independent from the HighRTA planner itself. 
	'''	

	# Load in the JSON file to access MDP problem information.
	with open('mdp_info.json','r') as f:
		mdp_info = json.load(f)
	state_map = mdp_info['states']
	action_map = mdp_info['actions']
	pi = mdp_info['pi']


	current_state = status.get_current_state() # There needs to be something to call to get the current state
											   # Alternatively we can have this passed in as a parameter?

	# Get the action from the policy pi for the current state. 
	action_to_take = actions[policy[states[currentState]]]

	for (t,r) in actionToTake:
		header_string = "(define (problem <package-dropoff>)\n(:domain world)" # Need to decide on the domain name.
		object_string = "(:objects \n"
		init_string = "(:init "
		goal_string = ""

		problem_file = open( (str(t.get_ID()) + "_assignment.pddl") , "w+")

		problem_file.write(header_string)
		problem_file.write(object_string)
		problem_file.write(init_string)
		problem_file.write(goal_string + "\n)")

		problem_file.close()

