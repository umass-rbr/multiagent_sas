# multiagent_sas
Multiagent semi-autonomous systems (SAS) for campus delivery robots.

## Questions
* Should we have different messages for each task?
* Should I implement getters and setters as properties?
* Do I like how I'm passing around the TaskRequest object?
* Is there a better name than WorldState?
* Why does the function calculate_expected_cost() rely on the implementation of a task?
* Should we refactor the code to fit with Kyle's structure?
* Create an action execution map

## Important Commands
* rostopic pub /task_assignment/task_assignment_action task_assignment/TaskAssignmentAction '{robot_id: "pumpkin", task_type: "delivery", task_data: "{\"start_location\": \"5\", \"end_location\": \"10\"}" }'

* rostopic pub /monitor/world_state task_assignment/WorldState '{task_requests: [{id: 0, type: "delivery", start_time: 0, end_time: 0, data: "{\"package\": \"package1\", \"pickup_location\": \"shlomoOffice\", \"dropoff_location\": \"amrl\"}"}], robot_status: [1, 0, 0]}'