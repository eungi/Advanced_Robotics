cmake_minimum_required(VERSION 2.8.3)
project(robot)

find_package(catkin REQUIRED COMPONENTS
  urdf
  sensor_msgs
  pcl_ros
)

find_package(PCL REQUIRED COMPONENTS common io)

generate_messages(DEPENDENCIES sensor_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(RRrobot_control_node src/RR-robot.cpp)
add_dependencies(RRrobot_control_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(RRrobot_control_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(RProbot_control_node src/RP-robot.cpp)
add_dependencies(RProbot_control_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(RProbot_control_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(computer_problem_6_node src/RR-robot_CP6.cpp)
add_dependencies(computer_problem_6_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(computer_problem_6_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
