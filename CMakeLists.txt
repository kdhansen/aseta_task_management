cmake_minimum_required(VERSION 2.8.3)
project(aseta_task_management)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs actionlib geometry_msgs sensor_msgs roscpp rostest)
find_package(Boost REQUIRED COMPONENTS system thread)
find_library(Clippoly "libclippoly.so")
if(${Clippoly} STREQUAL "Clippoly-NOTFOUND")
	message("Could not find libclippoly")
	return()
endif(${Clippoly} STREQUAL "Clippoly-NOTFOUND")

# If the compiler is GCC we must enable c++11/c++0x support
if(CMAKE_COMPILER_IS_GNUCXX)
   SET(ENABLE_CXX11 "-std=c++11")

   EXECUTE_PROCESS(COMMAND "${CMAKE_CXX_COMPILER} -dumpversion" OUTPUT_VARIABLE GCC_VERSION)
   if (GCC_VERSION VERSION_LESS 4.7)
      SET(ENABLE_CXX11 "-std=c++0x")
   endif()

   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ENABLE_CXX11}")
endif()

##########################
### Message Generation ###
##########################

add_action_files(
	DIRECTORY action
	FILES PhotoWaypoint.action
)
add_service_files(
	DIRECTORY srv
	FILES 
	PhotographArea.srv
	Goal.srv
)

generate_messages(
	DEPENDENCIES
	actionlib_msgs
	geometry_msgs
	sensor_msgs
)

catkin_package(
)

###############
### Sources ###
###############

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(libgatsp 
	src/gatsp/Problem.cpp
    src/gatsp/Waypoint.cpp
    src/gatsp/Genome.cpp
    src/gatsp/GeneticAlgorithm.cpp
)
target_link_libraries(libgatsp ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(decomposer src/decomposer.cpp)
target_link_libraries(decomposer ${catkin_LIBRARIES} ${Clippoly})

add_executable(task_manager src/task_manager.cpp)
target_link_libraries(task_manager ${catkin_LIBRARIES} ${Boost_LIBRARIES} decomposer libgatsp)
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