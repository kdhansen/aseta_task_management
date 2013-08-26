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

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include "aseta_task_management/PhotographArea.h"

namespace aseta
{
	class TaskManager
	{
	public:
		TaskManager();
		~TaskManager();

	private:
		ros::NodeHandle priv_nh;
		ros::ServiceServer register_task_service;

		std::string reference_frame;

		geometry_msgs::Polygon field;
		ros::Publisher field_pub;
		ros::Timer field_timer;

		std::vector<geometry_msgs::Point> waypoints;
		ros::Publisher waypoint_pub;
		ros::Timer waypoint_timer;

		bool registerTaskCb(aseta_task_management::PhotographArea::Request &,
			                  aseta_task_management::PhotographArea::Response &);

		void publishField(const ros::TimerEvent&);

		void publishWaypoints(const ros::TimerEvent&);
	};
};

#endif
