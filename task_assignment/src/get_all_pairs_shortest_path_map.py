#!/usr/bin/env python

import sys
import json

from math import inf
from itertools import product

from IPython import embed

def floyd_warshall(input_map):
	_map = json.load( open(input_map, mode='r+') )

	V = list(_map["locations"].keys())
	E = {key: {} for key in V}

	n = len(V)

	for start in _map["paths"].keys():
		for end in _map["paths"][start].keys():
			E[start][end] = _map["paths"][start][end]["cost"]

	dist = {key: {key2: float("inf") for key2 in V} for key in V}

	for u in E.keys():
		for v in E[u].keys():
			dist[u][v] = E[u][v]

	for k in V:
		for i in V:
			for j in V:
				if dist[i][j] > dist[i][k] + dist[k][j]:
					dist[i][j] = dist[i][k] + dist[k][j]

	for u in V:
		dist[u][u] = 0

	all_pairs_shortest_path_map = json.dumps(dist, indent=4)

	ofile = open("all_pairs_shortest_path_"+input_map, "w+")
	ofile.write(all_pairs_shortest_path_map)
	ofile.close()

if __name__ == '__main__':
	input_map = sys.argv[1]
	floyd_warshall(input_map)