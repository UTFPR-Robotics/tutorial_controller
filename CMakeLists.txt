cmake_minimum_required(VERSION 2.8.3)
project(tutorial_controller)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  roslaunch
)

roslaunch_add_file_check(launch)

add_action_files(
   DIRECTORY action
   FILES Tutorial.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs   std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(TutorialServer src/ControllerServer.cpp)
target_link_libraries(TutorialServer ${catkin_LIBRARIES})
add_dependencies(TutorialServer ${tutorial_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(TutorialClient src/ControllerClient.cpp)
target_link_libraries(TutorialClient ${catkin_LIBRARIES})
add_dependencies(TutorialClient ${tutorial_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
