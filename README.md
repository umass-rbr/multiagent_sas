# multiagent_sas
Multiagent semi-autonomous systems (SAS) for campus delivery robots.

## TODO
* Modify the TaskAssignment object
* Should we have different messages for each task?

## Important Commands
* rostopic pub /task_assignment/task_assignment_action task_assignment/TaskAssignmentAction '{robot_id: "pumpkin", task_type: "delivery", task_data: "{\"start_location\": \"5\", \"end_location\": \"10\"}" }'

* rostopic pub /monitor/world_state task_assignment/WorldState '{tasks: "{ \"0\": { \"task_type\": \"delivery\", \"package\": \"package1\", \"start_time\": \"0\", \"end_time\": \"3\", \"start_location\": \"2\", \"end_location\": \"1\", \"id\": 0 } }", robot_status: [1, 0, 0]}' 