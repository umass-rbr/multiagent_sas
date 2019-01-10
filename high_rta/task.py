import numpy as np
import itertools as it

import campus_map
import robot

class EscortTask():
	"""docstring for ClassName"""
	def __init__(self, person, pickup, dropoff, start, end):
		self.person = person
		self.pickup = pickup
		self.dropoff = dropoff
		self.start = start
		self.end = end
		

class DeliveryTask(object):
	"""docstring for DeliveryTask"""
	def __init__(self, object, start, end):
		