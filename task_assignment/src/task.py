import json

class Task(object):
    def __init__(self, id, type, start_time, end_time, task_request):
        self.id = id
        self.type = type
        self.start_time = start_time
        self.end_time = end_time
        self.task_request = task_request

    def get_task_request(self):
        return self.task_request

class DeliveryTask(Task):
    def __init__(self, id, start_time, end_time, task_request, package, pickup_location, dropoff_location, task_type='delivery'):
        super(DeliveryTask, self).__init__(id, task_type, start_time, end_time, task_request)
        self.package = package
        self.pickup_location = pickup_location
        self.dropoff_location = dropoff_location

class EscortTask(Task):
    def __init__(self, id,  start_time, end_time, task_request, person, pickup_location, dropoff_location, task_type='escort'):
        super(EscortTask, self).__init__(id, task_type, start_time, end_time, task_request)
        self.person = person
        self.pickup_location = pickup_location
        self.dropoff_location = dropoff_location
