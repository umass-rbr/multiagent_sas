import campus_map

class Robot():
    def __init__(self, r_id, r_type, pos):
        self.id = r_id
        self.r_type = r_type
        self.pos = pos

    def get_break_probability(self, l1, l2):
        # Inside Robot
        if self.r_type == 0:
            if campus_map.get_building(l1) != campus_map.get_building(l2):
                return 0.2
            return 0.05

        # Outside Robot
        if self.r_type == 1:
            if campus_map.get_building(l1) != campus_map.get_building(l2):
                return 0.05
            return 0.2

        # Human
        return 0

    def calculate_time(self, l1, l2):
        l1 = int(l1)
        l2 = int(l2)

        # Inside robot
        if self.r_type == 0:
            if campus_map.get_building(l1) != campus_map.get_building(l2):
                return 2 * campus_map.distance(l1, l2)
            return campus_map.distance(l1, l2)

        # Outside robot
        if self.r_type == 1:
            if campus_map.get_building(l1) != campus_map.get_building(l2):
                return campus_map.distance(l1, l2)
            return 2 * campus_map.distance(l1, l2)

        # Human
        return 4 * campus_map.distance(l1, l2)

    def get_id(self):
        return self.id

    def get_type(self):
        return self.r_type
