from enum import Enum


class RobotType(Enum):
    TURTLEBOT = 0
    JACKAL = 1
    HUMAN = 2


class Robot(object):
    def __init__(self, id, type):
        self.id = id
        self.type = type

    def get_id(self):
        return self.id

    def get_break_probability(self, start_location, end_location):
        if self.type == RobotType.TURTLEBOT:
            return 0.2 if start_location != end_location else 0.05

        if self.type == RobotType.JACKAL:
            return 0.05 if start_location != end_location else 0.2

        return 0

    def get_time_duration(self, map, start_location, end_location):
        cost = map['paths'][start_location][end_location]['cost']

        if self.type == RobotType.TURTLEBOT:
            return 2 * cost if start_location != end_location else cost

        if self.type == RobotType.JACKAL:
            return cost if start_location != end_location else 2 * cost

        return 4 * cost
