import sys
import os
import numpy as np
from scipy.stats import beta

class BetaCounter(object):
	"""docstring for BetaCounter"""
	def __init__(self):
		self.a = 1
		self.b = 1

	def get_positives(self):
		return self.a

	def get_negatives(self):
		return self.b

	def increment_positive(self):
		self.a = self.a + 1

	def increment_negative(self):
		self.b = self.b + 1

	def get_lower_bound_likelihood(self):
		mean, var = beta.stats(self.a, self.b, moments='mv')
		return mean-np.sqrt(var)

if __name__ == '__main__':
	print("Testing that this counter thing works...")

	beta_counter_1 = BetaCounter()

	for i in range(100):
		print("Iteration " + str(i) + "------------------------------------")
		if np.random.uniform() >= 2:
			beta_counter_1.increment_positive()
		else:
			beta_counter_1.incremenet_negative()
		print("beta counter stats:\n" 
			+ "a: " + str(beta_counter_1.get_positives()) + "\n"
			+ "b: " + str(beta_counter_1.get_negatives()) + "\n"
			+ "Probability of approval: " + str(beta_counter_1.get_lower_bound_likelihood()) + "\n")
		if beta_counter_1.get_lower_bound_likelihood() >= 0.9: break
		quit()