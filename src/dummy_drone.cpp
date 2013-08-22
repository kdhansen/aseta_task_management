/*
ASETA Dummy Drone
For testing the output of the ASETA system's planner.
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

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "aseta_task_management/PhotoWaypointAction.h"

namespace aseta
{
	class DummyDrone
	{
	public:
		DummyDrone() :
			_priv_nh("~"),
			_as(_priv_nh, "task_server", boost::bind(&DummyDrone::executeCB, this, _1), false)
		{
			_as.start();
		}

		~DummyDrone(){}

		void executeCB(const aseta_task_management::PhotoWaypointGoalConstPtr &goal_)
		{
			ROS_INFO("Executing task.");

			ros::Rate r(1);
			bool succes = true;
			for (int t = 3; t > 0; --t)
			{
				// Check for preemption.
				if (_as.isPreemptRequested() || !ros::ok())
				{
					ROS_INFO("Task preempted.");
					_as.setPreempted();
					succes = false;
					break;
				}
				// If task was not preempted go towards goal.
				ROS_INFO_STREAM("Time to target: " << t << " seconds.");
				aseta_task_management::PhotoWaypointFeedback _feedback;
				_feedback.time_to_target = t;
				_as.publishFeedback(_feedback);
				r.sleep();
			}
			if (succes)
			{
				aseta_task_management::PhotoWaypointResult _result;
				ros::Time now = ros::Time::now();
				_result.pose_stamped.pose = goal_->pose;
				_result.pose_stamped.header.stamp = now;
				_result.image.height = 1;
				_result.image.width = 1;
				_result.image.encoding = "rgb";
				_result.image.data.push_back(100);
				_result.image.header.stamp = now;
				ROS_INFO("Image taken.");
				_as.setSucceeded(_result);
			}
		}

	private:
		ros::NodeHandle _priv_nh; // not priv as in private member (although it is) but in the private ROS namespace "~".
		actionlib::SimpleActionServer<aseta_task_management::PhotoWaypointAction> _as;
	};


}
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "dummy_drone");
	aseta::DummyDrone dd;
	ros::spin();
	return 0;
}