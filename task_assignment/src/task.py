import json

class Task(object):
    def __init__(self, id, task_type, start_time, end_time):
        self.id = id
        self.task_type = task_type
        self.start_time = start_time
        self.end_time = end_time

    def pack(self):
        return json.dumps(self.__dict__, indent=4)

    def get_id(self):
        return self.id

    def get_type(self):
        return self.task_type

class DeliveryTask(Task):
    def __init__(self, id, package, pickup_location, dropoff_location, start_time, end_time):
        super(DeliveryTask, self).__init__(id, 'delivery', start_time, end_time)
        self.package = package
        self.pickup_location = pickup_location
        self.dropoff_location = dropoff_location

    def __str__(self):
        return "[" + str(self.start_time) + ", " + str(self.end_time) + ", " + str(self.pickup_location) + ", " + str(self.dropoff_location) + "]"

class EscortTask(Task):
    def __init__(self, id, person, pickup_location, dropoff_location, start_time, end_time):
        super(EscortTask, self).__init__(id, 'escort', start_time, end_time)
        self.person = person
        self.pickup_location = pickup_location
        self.dropoff_location = dropoff_location

    def __str__(self):
        return "[" + str(self.start_time) + ", " + str(self.end_time) + ", " + str(self.pickup_location) + ", " + str(self.dropoff_location) + "]"
