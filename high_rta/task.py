class DeliveryTask():
    def __init__(self, package, start_location, end_location, start_time, end_time):
        self.package = package
        self.start_location = start_location
        self.end_location = end_location
        self.start_time = start_time
        self.end_time = end_time

class EscortTask():
    def __init__(self, person, start_location, end_location, start_time, end_time):
        self.person = person
        self.start_location = start_location
        self.end_location = end_location
        self.start_time = start_time
        self.end_time = end_time
