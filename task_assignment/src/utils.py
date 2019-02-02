import itertools as it

def get_cartesian_product(A, B):
    return [(a, b) for a in A for b in B]


def get_power_set(A):
    return list(it.chain.from_iterable(it.combinations(A, size) for size in range(len(A) + 1)))
