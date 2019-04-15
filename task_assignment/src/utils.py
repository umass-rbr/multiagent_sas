import itertools as it
import json
import sys


def get_cartesian_product(A, B):
    return [(a, b) for a in A for b in B]


def get_power_set(A):
    return list(it.chain.from_iterable(it.combinations(A, size) for size in range(len(A) + 1)))


def get_distance_map(world_map):
    V = list(world_map["locations"].keys())
    E = {key: {} for key in V}

    n = len(V)

    for start in world_map["paths"].keys():
        for end in world_map["paths"][start].keys():
            E[start][end] = world_map["paths"][start][end]["cost"]

    distances = {key: {key2: float("inf") for key2 in V} for key in V}

    for u in E.keys():
        for v in E[u].keys():
            distances[u][v] = E[u][v]

    for k in V:
        for i in V:
            for j in V:
                if distances[i][j] > distances[i][k] + distances[k][j]:
                    distances[i][j] = distances[i][k] + distances[k][j]

    for u in V:
        distances[u][u] = 0

    return distances
