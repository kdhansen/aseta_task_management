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

#include <vector>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include "aseta_task_management/PhotoWaypointAction.h"

namespace aseta
{
	class DummyDrone
	{
	public:
		DummyDrone();
		~DummyDrone();
	private:
		void executeCB(const aseta_task_management::PhotoWaypointGoalConstPtr &);
		void update(const ros::TimerEvent& event);
		void publishPath();
		void setTaskCompleted();
		
		ros::NodeHandle priv_nh; // not priv as in private member (although it is) but in the private ROS namespace "~".
		actionlib::SimpleActionServer<aseta_task_management::PhotoWaypointAction> as;
		std::string reference_frame;
		geometry_msgs::Pose current_pose;
		geometry_msgs::Pose goal_pose;
		std::vector<geometry_msgs::Pose> flewn_path;

		ros::Timer update_timer;
		double _update_rate;

		ros::Publisher path_publisher;
		double speed;

		boost::mutex lock;
		bool is_moving;
	};

	DummyDrone::DummyDrone() :
		priv_nh("~"),
		as(priv_nh, "task_server", boost::bind(&DummyDrone::executeCB, this, _1), false),
		_update_rate(0.1)
	{
		// Set the speed of the vehicle
		priv_nh.param<double>("speed", speed, 1.0);
		ROS_INFO_STREAM("Vehicle speed is set to " << speed);

		// Set the reference frame
		priv_nh.param<std::string>("reference_frame", reference_frame, "map");
		ROS_INFO_STREAM("reference frame is set to " << reference_frame);

		// Show the flewn path
		path_publisher = priv_nh.advertise<nav_msgs::Path>("sim", 1);

		// Update the state of the drone periodically.
    	update_timer = priv_nh.createTimer(ros::Duration(_update_rate), &DummyDrone::update, this);

		as.start();
	}

	DummyDrone::~DummyDrone(){}

	void DummyDrone::executeCB(const aseta_task_management::PhotoWaypointGoalConstPtr &goal)
	{
		ROS_INFO("Setting new task.");

		goal_pose = goal->pose;
		lock.lock();
		is_moving = true;
		lock.unlock();
		bool preempted = false;
		ros::Rate wait = ros::Rate(1.0);
		while (is_moving)
		{
			// Check for preemption.
			if (as.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("Task preempted.");
				as.setPreempted();
				lock.lock();
				is_moving = false;
				lock.unlock();
			}
			wait.sleep();
		}
		if (! preempted)
		{
			aseta_task_management::PhotoWaypointResult _result;
			ros::Time now = ros::Time::now();
			_result.pose_stamped.pose = goal_pose;
			_result.pose_stamped.header.stamp = now;
			_result.image.height = 1;
			_result.image.width = 1;
			_result.image.encoding = "rgb";
			_result.image.data.push_back(100);
			_result.image.header.stamp = now;
			ROS_INFO("Image taken.");
			as.setSucceeded(_result);
		}

	}

	/// Updates the vehicles position along the path.
	///
	void DummyDrone::update(const ros::TimerEvent& event)
	{
		boost::lock_guard<boost::mutex> guard(lock);
		if (is_moving) // If task was not preempted go towards goal.
		{
			// Compute new position
			double x_diff = goal_pose.position.x - current_pose.position.x;
			double y_diff = goal_pose.position.y - current_pose.position.y;
			double z_diff = goal_pose.position.z - current_pose.position.z;
			double segment_length = sqrt(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff);
			double ratio_to_consume = speed * _update_rate / segment_length;

			if (1 <= ratio_to_consume) // We made the goal
			{
				current_pose.position.x = goal_pose.position.x;
				current_pose.position.y = goal_pose.position.y;
				current_pose.position.z = goal_pose.position.z;
				is_moving = false;
			}
			else
			{
				current_pose.position.x = current_pose.position.x + ratio_to_consume * x_diff;
				current_pose.position.y = current_pose.position.y + ratio_to_consume * y_diff;
				current_pose.position.z = current_pose.position.z + ratio_to_consume * z_diff;
				// t = segment_length / speed;
				// ROS_INFO_STREAM("Time to target: " << t << " seconds.");
				// aseta_task_management::PhotoWaypointFeedback _feedback;
				// _feedback.time_to_target = t;
				// as.publishFeedback(_feedback);
			}
			flewn_path.push_back(current_pose);
		}

		// Publish the new path
		publishPath();
	}

	/// Publishes a path for rviz between the waypoints in a problem.
	///
	void DummyDrone::publishPath()
	{
	    if (ros::ok())
	    {
	        // Path
	        nav_msgs::Path path;
	        path.header.frame_id = reference_frame;
	        path.header.stamp = ros::Time();

	        // Fill the lists
	        unsigned int seq = 0;
	        for (auto& my_pose : flewn_path)
	        {
	            geometry_msgs::PoseStamped ps;
	            ps.pose = my_pose;
	            ps.header.frame_id = reference_frame;
	            ps.header.stamp = ros::Time();
	            ps.header.seq = seq++;
	            path.poses.push_back(ps);
	        }

	        path_publisher.publish(path);
	    }
	}

	/// Sets the task as completed.
	///
	void DummyDrone::setTaskCompleted()
	{

	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "dummy_drone");
	aseta::DummyDrone dd;
	ros::spin();
	return 0;
}