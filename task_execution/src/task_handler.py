from mdps.delivery_mdp import DeliveryMDP
from mdps.escort_mdp import EscortMDP


class DeliveryTaskHandler(object):
    def get_problem(self, world_map, task_data):
        return DeliveryMDP(world_map, task_data["pickup_location"], task_data["dropoff_location"])

    def get_solution(self, problem):
        return problem.solve()

    def get_name(self, problem):
        return problem.name

    def get_state(self, message):
        return message.location, message.has_package

    def is_goal(self, state, task_data):
        if not state:
            return False
        return state[0] == task_data["dropoff_location"] and state[1]


class EscortTaskHandler(object):
    def get_problem(self, world_map, task_data):
        return EscortMDP(world_map, task_data["pickup_location"], task_data["dropoff_location"])

    def get_solution(self, problem):
        return problem.solve()

    def get_name(self, problem):
        return problem.name

    def get_state(self, message):
        return message.location, message.with_person

    def is_goal(self, state, task_data):
        if not state:
            return False
        return state[0] == task_data["dropoff_location"] and state[1]
