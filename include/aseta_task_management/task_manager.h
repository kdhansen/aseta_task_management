/*
ASETA Task Manager
Keeps track of outstanding and finished tasks for the ASETA system.
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

#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <string>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>
#include "aseta_task_management/PhotographArea.h"
#include "aseta_task_management/PhotoWaypointAction.h"
#include "aseta_task_management/Goal.h"
#include "gatsp/GeneticAlgorithm.h"

namespace aseta
{

	typedef actionlib::SimpleActionClient<aseta_task_management::PhotoWaypointAction> PhotoWaypointActionClient;

	class TaskManager
	{
	public:
		TaskManager();
		~TaskManager();

	private:
		ros::NodeHandle priv_nh;
		ros::ServiceServer register_task_service;
		ros::ServiceServer next_goal_service;

		PhotoWaypointActionClient * drone_action_client;
		std::string drone_action_server;
		bool task_assigned;

		ros::Timer manager_timer;
		std::string reference_frame;
		
		gatsp::GeneticAlgorithm tspga;
		boost::thread ga_thread;

		geometry_msgs::Polygon field;
		ros::Publisher field_pub;
		ros::Timer field_timer;

		std::vector<geometry_msgs::Point> waypoints;
		ros::Publisher waypoint_pub;
		ros::Timer waypoint_timer;

		ros::Publisher path_pub;
		ros::Timer path_timer;

		int camera_sensor_width, camera_sensor_height;
		double camera_focal_length_x, camera_focal_length_y;

		void updateTaskManager(const ros::TimerEvent&);

		bool registerTaskCb(aseta_task_management::PhotographArea::Request &,
			                aseta_task_management::PhotographArea::Response &);

		bool nextGoalCb(aseta_task_management::Goal::Request &,
			            aseta_task_management::Goal::Response &);

		void publishField(const ros::TimerEvent&);
		void publishWaypoints(const ros::TimerEvent&);
		void publishPath(const ros::TimerEvent& event);
	};
};

#endif
