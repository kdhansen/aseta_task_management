/*
Test for ASETA Dummy Drone
Integration testing of the dummy drone for the ASETA system.
Copyright (C) 2013 Karl D. Hansen (kdh@es.aau.dk)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <actionlib/client/simple_action_client.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "aseta_task_management/PhotoWaypointAction.h"

// Declare a test
TEST(DummyDrone, executeTask)
{
	actionlib::SimpleActionClient<aseta_task_management::PhotoWaypointAction> ac("dummy_drone/task_server", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	aseta_task_management::PhotoWaypointGoal goal;
	goal.pose.position.x = 10;
	goal.pose.position.y = 10;
	goal.pose.position.z = 10;
	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
		ROS_INFO("Action did not finish before the time out.");

	EXPECT_TRUE(finished_before_timeout);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "test_dummy_drone");
	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}