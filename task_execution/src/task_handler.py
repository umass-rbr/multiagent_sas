from mdps.delivery_mdp import DeliveryMDP
from mdps.escort_mdp import EscortMDP

class DeliveryTaskHandler(object):
    def get_problem(self, map, task_data):
        return DeliveryMDP(map, task_data["start_location"], task_data["end_location"])

    def get_solution(self, problem):
        return problem.solve()

    def get_name(self, problem):
        return problem.name

    def get_state(self, message):
        return message.location, message.has_package

    def is_goal(self, state, task_data):
        if not state:
            return False
        print(state[0])
        print(task_data["end_location"])
        return state[0] is task_data["end_location"]

class EscortTaskHandler(object):
    def get_problem(self, map, task_data):
        return EscortMDP(map, task_data["start_location"], task_data["end_location"])

    def get_solution(self, problem):
        return problem.solve()
    
    def get_name(self, problem):
        return problem.name

    def get_state(self, message):
        return message.location, message.with_person

    def is_goal(self, state, task_data):
        if not state:
            return False
        return state[0] is task_data["end_location"]
