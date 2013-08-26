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

#include <geometry_msgs/PoseArray.h>
#include "aseta_task_management/task_manager.h"
#include "aseta_task_management/decomposer.h"

namespace aseta
{
    TaskManager::TaskManager() :
        priv_nh("~")
    {
        priv_nh.param<std::string>("reference_frame", reference_frame, "map");
        priv_nh.param("camera_sensor_width", camera_sensor_width, 640);
        priv_nh.param("camera_sensor_height", camera_sensor_height, 480);
        priv_nh.param("camera_focal_length_x", camera_focal_length_x, camera_sensor_width*1.1);
        priv_nh.param("camera_focal_length_y", camera_focal_length_y, camera_sensor_height*1.1);
        register_task_service = 
            priv_nh.advertiseService("register_task",
                                     &TaskManager::registerTaskCb,
                                     this);
        field_pub = priv_nh.advertise<geometry_msgs::PolygonStamped>("field", 1);
        field_timer = priv_nh.createTimer(ros::Duration(1.0), &TaskManager::publishField, this);
        waypoint_pub = priv_nh.advertise<geometry_msgs::PoseArray>("waypoints", 1);
        waypoint_timer = priv_nh.createTimer(ros::Duration(0.5), &TaskManager::publishWaypoints, this);
    }

    TaskManager::~TaskManager()
    {}

    /// Callback funtion to respond to service requests for doing a task.
    ///
    /// @param req The request containing an area to photograph at a specified resolution.
    /// @param[out] res A response when the task has been registered. This is empty for now.
    bool TaskManager::registerTaskCb(
        aseta_task_management::PhotographArea::Request &req,
        aseta_task_management::PhotographArea::Response &res)
    {
        ROS_INFO("Registering task.");

        // If this is the first request, we will assume that this
        // is the field.
        if (field.points.size() == 0)
        {
            field = req.area;
        }

        // Decompose the task and put the waypoints into the list.
        aseta::Decomposer d(camera_focal_length_x, camera_focal_length_y, req.m_pr_px, camera_sensor_height, camera_sensor_width);
        d.decompose(req.area, waypoints);

        return true;
    }

    /// Callback to periodically publish the field we are working with.
    ///
    void TaskManager::publishField(const ros::TimerEvent&)
    {
        if (field.points.size() > 0)
        {
            geometry_msgs::PolygonStamped f;
            f.polygon = field;
            f.header.stamp = ros::Time::now();
            f.header.frame_id = reference_frame;
            field_pub.publish(f);
        }
    }

    /// Callback to periodically publish the unvisited waypoints.
    ///
    void TaskManager::publishWaypoints(const ros::TimerEvent&)
    {
        // if (waypoints.size() > 0)
        // {
            geometry_msgs::PoseArray pa;
            pa.header.stamp = ros::Time::now();
            pa.header.frame_id = reference_frame;
            for (auto wp : waypoints)
            {
                geometry_msgs::Pose p;
                p.position = wp;
                pa.poses.push_back(p);
            }
            waypoint_pub.publish(pa);
        // }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task_manager");
    aseta::TaskManager tm;
    ros::spin();
    return 0;
}
