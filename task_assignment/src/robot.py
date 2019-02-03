from enum import Enum 


class RobotType(Enum):
    TURTLEBOT = 0
    JACKAL = 1
    HUMAN = 2


class Robot():
    def __init__(self, id, type):
        self.id = id
        self.type = type

    def get_id(self):
        return self.id

    def get_break_probability(self, l1, l2):
        if self.type == RobotType.TURTLEBOT:
            return 0.2 if l1 != l2 else 0.05

        if self.type == RobotType.JACKAL:
            return 0.05 if l1 != l2 else 0.2

        return 0

    def get_time_duration(self, map, l1, l2):
        cost = map['paths'][l1][l2]['cost']
        
        if self.type == RobotType.TURTLEBOT:
            return 2 * cost if l1 != l2 else cost

        if self.type == RobotType.JACKAL:
            return cost if l1 != l2 else 2 * cost

        return 4 * cost
