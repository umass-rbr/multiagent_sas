import numpy as np
import itertools as it

import campus_map

class robot():
	def __init__(self, r_id, r_type, pos):
		self.id = r_id
		self.r_type = r_type
		self.pos = pos

	def get_break_probability(self, l1, l2):
		if self.r_type == 0: #Inside Robot
			if campus_map.get_building(l1) != campus_map.get_building(l2):
				return .2
			else: return .05
		elif self.r_type == 1: #Outside Robot
			if campus_map.get_building(l1) != campus_map.get_building(l2):
				return .05
			else: return .2
		else: #Human
			return 0.0

	def calculate_time(self, l1, l2):
		if self.r_type == 0: #Inside robot
			if campus_map.get_building(l1) != campus_map.get_building(l2):
				return 2*campus_map.distance(l1,l2)
			else:
				return campus_map.distance(l1,l2)
		elif self.r_type == 1: #Outside robot
			if campus_map.get_building(l1) != campus_map.get_building(l2):
				return campus_map.distance(l1,l2)
			else:
				return 2*campus_map.distance(l1,l2)
		else: #Human
			return 4*campus_map.distance(l1,l2)
			
	def get_id(self):
		return self.id
	def get_type(self):
		return self.r_type
