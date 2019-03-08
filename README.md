# multiagent-sas
Multiagent semi-autonomous systems (SAS) for campus delivery robots.

## Tasks
* Should we have different messages for each task?
* Should I implement getters and setters as properties?
* Do I like how I'm passing around the TaskRequest object?
* Is there a better name than WorldState?
* Why does the function calculate_expected_cost() rely on the implementation of a task?
* Should we refactor the code to fit with Kyle's structure?
* Clean up all of the pylint errors
* Rename everything across all files to be consistent
* Create an action execution map
* Connect to the map manager

## Important Commands
* rostopic pub /task_assignment/task_assignment_action task_assignment/TaskAssignmentAction '{robot_id: "pumpkin", task_request: {id: 0, type: "delivery", start_time: 0, end_time: 100, data: "{\"pickup_location\": \"RBR\", \"dropoff_location\": \"OfficeJoydeep\"}"}}'