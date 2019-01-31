import json

class Task(object):
    def __init__(self, id, start_time, end_time):
        self.id = id
        self.start_time = start_time
        self.end_time = end_time

    def pack(self):
        return json.dumps(self.__dict__, indent=4)


# TODO Can we make an "abstract" task that has an id, start_time, end_time?
class DeliveryTask(Task):
    def __init__(self, id, package, start_location, end_location, start_time, end_time):
        super().__init__(id,start_time,end_time)
        self.package = package
        self.start_location = start_location
        self.end_location = end_location

    def get_type(self):
        return 'delivery'

    def __str__(self):
        to_output = "[" + str(self.start_time) + ", " + str(self.end_time) + ", " + str(self.start_location) + ", " + str(self.end_location) + "]"
        return to_output

class EscortTask():
    def __init__(self, id, person, start_location, end_location, start_time, end_time):
        super().__init__(id,start_time,end_time)
        self.person = person
        self.start_location = start_location
        self.end_location = end_location

    def get_type(self):
        return 'escort'

    def __str__(self):
        to_output = "[" + str(self.start_time) + ", " + str(self.end_time) + ", " + str(self.start_location) + ", " + str(self.end_location) + "]"
        return to_output

#t1 = DeliveryTask(0,'package1','2','1','0','3')
#print(t1.pack())