cmake_minimum_required(VERSION 2.8.3)
project(aseta_task_management)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs actionlib geometry_msgs sensor_msgs roscpp rostest)
find_package(Boost REQUIRED COMPONENTS system)

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
	${Boost_INCLUDE_DIRS}
)

add_executable(task_manager src/TaskManager.cpp)
target_link_libraries(task_manager ${catkin_LIBRARIES})
add_dependencies(task_manager ${PROJECT_NAME}_gencpp)

add_executable(dummy_drone src/dummy_drone.cpp)
target_link_libraries(dummy_drone ${catkin_LIBRARIES})
add_dependencies(dummy_drone ${PROJECT_NAME}_gencpp)

catkin_add_gtest(unit_test test/unit_test.cpp)

catkin_add_gtest(test_dummy_drone test/test_dummy_drone.cpp)
target_link_libraries(test_dummy_drone ${catkin_LIBRARIES})
add_dependencies(test_dummy_drone ${PROJECT_NAME}_gencpp)