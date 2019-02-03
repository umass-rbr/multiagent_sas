cmake_minimum_required(VERSION 2.8.3)
project(task_assignment)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  TaskAssignmentAction.msg
  TaskRequest.msg
  WorldState.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  DEPENDS rospy std_msgs
  CATKIN_DEPENDS message_runtime
)