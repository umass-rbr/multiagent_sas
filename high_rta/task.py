# TODO Can we make an "abstract" task that has an id, start_time, end_time?
class DeliveryTask():
    def __init__(self, id, package, start_location, end_location, start_time, end_time):
        self.id = id
        self.package = package
        self.start_location = start_location
        self.end_location = end_location
        self.start_time = start_time
        self.end_time = end_time

class EscortTask():
    def __init__(self, id, person, start_location, end_location, start_time, end_time):
        self.id = id
        self.person = person
        self.start_location = start_location
        self.end_location = end_location
        self.start_time = start_time
        self.end_time = end_time
