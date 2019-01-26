from executors import mdp_executor
from mdps import delivery_mdp

EXECUTION_MAP = {
    "delivery": (mdp_executor, delivery_mdp)
}


def select(task_type):
    return EXECUTION_MAP[task_type]
