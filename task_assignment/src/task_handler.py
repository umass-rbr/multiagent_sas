from task import DeliveryTask, EscortTask


class DeliveryTaskHandler(object):
    def get_task(self, id, start_time, end_time, task_request, data):
        return DeliveryTask(id, start_time, end_time, task_request, data['package'], data['pickup_location'], data['dropoff_location'])


class EscortTaskHandler(object):
    def get_task(self, id, start_time, end_time, task_request, data):
        return EscortTask(id, start_time, end_time, task_request, data['person'], data['pickup_location'], data['dropoff_location'])
