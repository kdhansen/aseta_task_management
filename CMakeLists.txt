cmake_minimum_required(VERSION 2.8.3)
project(aseta_task_management)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs actionlib geometry_msgs sensor_msgs roscpp rostest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 

add_action_files(
	DIRECTORY action
	FILES PhotoWaypoint.action
)
add_service_files(
	DIRECTORY srv
	FILES PhotographArea.srv
)

generate_messages(
	DEPENDENCIES
	actionlib_msgs
	geometry_msgs
	sensor_msgs
)

catkin_package(
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(decomposer src/decomposer.cpp)
target_link_libraries(decomposer ${catkin_LIBRARIES})

add_executable(task_manager src/task_manager.cpp)
target_link_libraries(task_manager ${catkin_LIBRARIES} decomposer)
add_dependencies(task_manager ${PROJECT_NAME}_gencpp)

add_executable(register_task src/register_task.cpp)
target_link_libraries(register_task ${catkin_LIBRARIES})
add_dependencies(register_task task_manager ${PROJECT_NAME}_gencpp)

add_executable(dummy_drone src/dummy_drone.cpp)
target_link_libraries(dummy_drone ${catkin_LIBRARIES})
add_dependencies(dummy_drone ${PROJECT_NAME}_gencpp)

###############
#### TESTS ####
###############

#catkin_add_gtest(unit_test test/unit_test.cpp)

catkin_add_gtest(test_dummy_drone test/test_dummy_drone.cpp)
target_link_libraries(test_dummy_drone ${catkin_LIBRARIES})
add_dependencies(test_dummy_drone dummy_drone ${PROJECT_NAME}_gencpp)

catkin_add_gtest(test_decomposer test/test_decomposer.cpp)
target_link_libraries(test_decomposer ${catkin_LIBRARIES} decomposer)
add_dependencies(test_decomposer decomposer)