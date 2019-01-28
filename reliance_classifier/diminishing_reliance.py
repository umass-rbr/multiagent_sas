import sys
import os
import numpy as np

from beta_counter import BetaCounter

class DiminishingRelianceObject(object):
	"""docstring for DiminishingRelianceObject"""
	def __init__(self, action_schemas, abstract_features):
		self.action_schemas = action_schemas
		self.abstract_features = abstract_features

		self.reliance_tracker = {action : {} for action in action_schemas}

	def get_approval_probability(self,a,s):
		temp_dic = self.reliance_tracker[a]
		index = 0	
		exists = True
		while index < len(s)-1:
			try:
				temp_dic = temp_dic[s[index]]
				index += 1
			except Exception:
				exists = False
				temp_dic[s[index]] = {}
				temp_dic = temp_dic[s[index]]
				index += 1
		if not exists or s[len(s)-1] not in temp_dic.keys():
			beta_counter = BetaCounter()
			print(beta_counter.get_lower_bound_likelihood())
			temp_dic[s[len(s)-1]] = beta_counter
		return temp_dic[s[len(s)-1]].get_lower_bound_likelihood()

	def update_approval_probability(self,a,s,y):
		temp_dic = self.reliance_tracker[a]
		index = 0
		exists = True
		while index < len(s)-1:
			try:
				temp_dic = temp_dic[s[index]]
				index += 1
			except Exception:
				exists = False
				temp_dic[s[index]] = {}
				temp_dic = temp_dic[s[index]]
				index += 1
		if not exists or s[len(s)-1] not in temp_dic.keys():
			temp_dic[s[len(s)-1]] = BetaCounter()
		if y == 1:
			temp_dic[s[len(s)-1]].increment_positive()
		else:
			temp_dic[s[len(s)-1]].increment_negative()

if __name__ == '__main__':
	action_schemas = ['move','wait','open']
	abstract_features = ['obstacle','visibility','surface_condition']
	obstacles = ['crosswalk','door','person','none']
	visibility = ['light','dark','fog']
	surface_condition = ['wet','rough']
	DRO = DiminishingRelianceObject(action_schemas,abstract_features)

	states = []
	for i in range(1000):
		print("Make state " + str(i) + "...")
		state = [obstacles[np.random.randint(len(obstacles))],
					visibility[np.random.randint(len(visibility))],
					surface_condition[np.random.randint(len(surface_condition))]]
		states.append(state)

	for state in states:
		action = action_schemas[np.random.randint(len(action_schemas))]

		approval_probability = DRO.get_approval_probability(action,state)

		print("--------------- State: " + str(state) + " ---------------\n"
			+ "Probability of approval: " + str(approval_probability) +"\n")
		if approval_probability <= 0.9:
			print("Calling for approval....")
			if np.random.uniform() >= 0.25:
				print("Received Approval!")
				DRO.update_approval_probability(action,state,1)
			else:
				print("Approval Denied!")
				DRO.update_approval_probability(action,state,0)