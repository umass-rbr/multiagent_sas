import json

class Task(object):
    def __init__(self, id, type, start_time, end_time):
        self.id = id
        self.type = type
        self.start_time = start_time
        self.end_time = end_time

    def pack(self):
        return json.dumps(self.__dict__, indent=4)

    def get_id(self):
        return self.id

    def get_type(self):
        return self.type

    def __str__(self):
        return json.dumps(self.__dict__, indent=4)

class DeliveryTask(Task):
    def __init__(self, id, package, pickup_location, dropoff_location, start_time, end_time, task_type='delivery'):
        super(DeliveryTask, self).__init__(id, task_type, start_time, end_time)
        self.package = package
        self.pickup_location = pickup_location
        self.dropoff_location = dropoff_location

class EscortTask(Task):
    def __init__(self, id, person, pickup_location, dropoff_location, start_time, end_time, task_type='escort'):
        super(EscortTask, self).__init__(id, task_type, start_time, end_time)
        self.person = person
        self.pickup_location = pickup_location
        self.dropoff_location = dropoff_location
