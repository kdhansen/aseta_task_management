cmake_minimum_required(VERSION 2.8.3)
project(aseta_task_management)

find_package(catkin REQUIRED COMPONENTS genmsg actionlib_msgs geometry_msgs actionlib roscpp)

add_action_files(
	DIRECTORY action
	FILES PhotographArea.action
)
add_service_files(
	DIRECTORY srv
	FILES PhotographArea.srv
)

generate_messages(
	DEPENDENCIES
	actionlib_msgs
	geometry_msgs
)

catkin_package(
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(task_manager src/TaskManager.cpp)
target_link_libraries(task_manager ${catkin_LIBRARIES})
add_dependencies(task_manager ${PROJECT_NAME}_gencpp)

catkin_add_gtest(unit_test test/unit_test.cpp)