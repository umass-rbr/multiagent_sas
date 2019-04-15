from enum import Enum


class RobotType(Enum):
    TURTLEBOT = 0
    JACKAL = 1
    HUMAN = 2


class Robot(object):
    def __init__(self, name, type):
        self.name = name
        self.type = type

    def get_name(self):
        return self.name

    def get_cost(self, distance_map, trajectory):
        cost = 0

        for i in range(len(trajectory) - 1):
            cost += distance_map[trajectory[i]][trajectory[i + 1]]

        if self.type == RobotType.TURTLEBOT:
            return 2 * cost

        return cost
