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
        return id

    def get_type(self):
        return self.task_type


# TODO Can we make an "abstract" task that has an id, start_time, end_time?
class DeliveryTask(Task):
    def __init__(self, id, package, start_location, end_location, start_time, end_time):
        super(DeliveryTask, self).__init__(id,'delivery',start_time,end_time)
        self.package = package
        self.start_location = start_location
        self.end_location = end_location

    def __str__(self):
        to_output = "[" + str(self.start_time) + ", " + str(self.end_time) + ", " + str(self.start_location) + ", " + str(self.end_location) + "]"
        return to_output

class EscortTask():
    def __init__(self, id, person, start_location, end_location, start_time, end_time):
        super(EscortTask, self).__init__(id,'escort',start_time,end_time)
        self.person = person
        self.start_location = start_location
        self.end_location = end_location

    def __str__(self):
        to_output = "[" + str(self.start_time) + ", " + str(self.end_time) + ", " + str(self.start_location) + ", " + str(self.end_location) + "]"
        return to_output
