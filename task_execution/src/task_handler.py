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

class EscortTaskHandler(object):
    def get_problem(self, map, task_data):
        return EscortMDP(map, task_data["start_location"], task_data["end_location"])

    def get_solution(self, problem):
        return problem.solve()
    
    def get_name(self, problem):
        return problem.name

    def get_state(self, message):
        return message.location, message.with_person
