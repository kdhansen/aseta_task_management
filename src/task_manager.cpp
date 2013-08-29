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
#include <nav_msgs/Path.h>
#include "aseta_task_management/task_manager.h"
#include "aseta_task_management/decomposer.h"

namespace aseta
{
    TaskManager::TaskManager() :
        priv_nh("~")
    {
        // Get parameters
        priv_nh.param<std::string>("reference_frame", reference_frame, "map");
        priv_nh.param("camera_sensor_width", camera_sensor_width, 640);
        priv_nh.param("camera_sensor_height", camera_sensor_height, 480);
        priv_nh.param("camera_focal_length_x", camera_focal_length_x, 500.0);
        priv_nh.param("camera_focal_length_y", camera_focal_length_y, 500.0);

        // The register task service lets users (or ros nodes) supply the manager
        // with areas that must be photographed.
        register_task_service = 
            priv_nh.advertiseService("register_task", &TaskManager::registerTaskCb, this);

        // Serve next goal in line
        next_goal_service = priv_nh.advertiseService("next_goal", &TaskManager::nextGoalCb, this);

        // The genetic algorithm will run in a tread of its own.
        tspga.setTerminator(&gatsp::GeneticAlgorithm::terminateInfinite);
        ga_thread = boost::thread(&gatsp::GeneticAlgorithm::evolve, &(this->tspga));

        // The field and waypoint publishers, shows the state of the mission
        // to rviz.
        field_pub = priv_nh.advertise<geometry_msgs::PolygonStamped>("field", 1);
        field_timer = priv_nh.createTimer(ros::Duration(1.0), &TaskManager::publishField, this);
        waypoint_pub = priv_nh.advertise<geometry_msgs::PoseArray>("waypoints", 1);
        waypoint_timer = priv_nh.createTimer(ros::Duration(0.5), &TaskManager::publishWaypoints, this);

        path_pub = priv_nh.advertise<nav_msgs::Path>("path", 1);
        path_timer = priv_nh.createTimer(ros::Duration(0.5), &TaskManager::publishPath, this);
    }

    TaskManager::~TaskManager()
    {
        ga_thread.interrupt();
    }

    /// Callback funtion to respond to service requests for doing a task.
    ///
    /// @param req The request containing an area to photograph at a specified resolution.
    /// @param[out] res A response when the task has been registered. This is empty for now.
    bool TaskManager::registerTaskCb( aseta_task_management::PhotographArea::Request &req,
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
        std::vector<geometry_msgs::Point> new_waypoints;
        d.decompose(req.area, new_waypoints);

        // Put the new waypoints into the list and genetic algorithm.
        waypoints.insert(waypoints.end(), new_waypoints.begin(), new_waypoints.end());
        for(auto wp : new_waypoints)
        {
            gatsp::Waypoint gwp(gatsp::Point(wp.x, wp.y, wp.z), gatsp::Quarternion());
            tspga.addWaypoint(gwp);
        }

        return true;
    }

    /// Callback function to retreive the next waypoint in line from the genetic algorithm.
    ///
    /// @param req The request. Empty for now.
    /// @param[out] res The goal specified as a pose in space.
    /// @returns False if there is no waypoint in line.
    bool TaskManager::nextGoalCb(aseta_task_management::Goal::Request&,
                                 aseta_task_management::Goal::Response& res)
    {
        try
        {
            gatsp::Waypoint wp = tspga.popWaypoint();
            res.goal.position.x = wp.x();
            res.goal.position.y = wp.y();
            res.goal.position.z = wp.z();
        }
        catch (gatsp::EmptySolution)
        {
            return false;
        }

        return true;
    }

    /// Callback to periodically publish the field we are working with.
    ///
    void TaskManager::publishField(const ros::TimerEvent&)
    {
            geometry_msgs::PolygonStamped f;
            f.polygon = field;
            f.header.stamp = ros::Time::now();
            f.header.frame_id = reference_frame;
            field_pub.publish(f);
    }

    /// Callback to periodically publish the unvisited waypoints.
    ///
    void TaskManager::publishWaypoints(const ros::TimerEvent&)
    {
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
    }

    /// Publishes a path for rviz between the waypoints in a problem.
    ///
    void TaskManager::publishPath(const ros::TimerEvent& event)
    {
        // Path
        nav_msgs::Path path;
        path.header.frame_id = reference_frame;
        path.header.stamp = ros::Time();

        // Fill the lists
        unsigned int seq = 0;
        gatsp::Problem prob = tspga.bestIndividual();
        for (auto& wp : prob.orderedWaypoints())
        {
            geometry_msgs::PoseStamped ps;
            geometry_msgs::Point point;
            point.x = wp.x();
            point.y = wp.y();
            point.z = wp.z();
            ps.pose.position = point;
            geometry_msgs::Quaternion quaternion;
            ps.pose.orientation = quaternion;
            ps.header.frame_id = reference_frame;
            ps.header.stamp = ros::Time();
            ps.header.seq = seq++;
            path.poses.push_back(ps);
        }

        // Send it off.
        path_pub.publish(path);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task_manager");
    aseta::TaskManager tm;
    ros::spin();
    return 0;
}
