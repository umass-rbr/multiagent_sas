# multiagent_sas
Multiagent semi-autonomous systems (SAS) for campus delivery robots.

## TODO
* Modify the TaskAssignment object
* Should we have different messages for each task?
* Implement getters using best practices
* Apply PEP8 or Google style guidelines throughout the code
* Do I like how I'm passing around the TaskRequest object?
* Is there a better name for WorldState?
* Why is the expected cost tied to the implementation of a task?
* Refactor project to fit with Kyle's suggestions
* Make sure you address all TODOs throughout the code

## Important Commands
* rostopic pub /task_assignment/task_assignment_action task_assignment/TaskAssignmentAction '{robot_id: "pumpkin", task_type: "delivery", task_data: "{\"start_location\": \"5\", \"end_location\": \"10\"}" }'

* rostopic pub /monitor/world_state task_assignment/WorldState '{tasks: "{ \"0\": { \"task_type\": \"delivery\", \"package\": \"package1\", \"start_time\": \"0\", \"end_time\": \"3\", \"start_location\": \"2\", \"end_location\": \"1\", \"id\": 0 } }", robot_status: [1, 0, 0]}' 