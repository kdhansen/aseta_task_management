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

#include "aseta_task_management/task_manager.h"

namespace aseta
{
    TaskManager::TaskManager() :
        priv_nh("~")
    {
        register_task_service = 
            priv_nh.advertiseService("register_task",
                                     &TaskManager::registerTaskCb,
                                     this);
        field_pub = priv_nh.advertise<geometry_msgs::PolygonStamped>("field", 1);
        field_timer = priv_nh.createTimer(ros::Duration(1.0), &TaskManager::publishField, this);
    }

    TaskManager::~TaskManager()
    {}

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
            for (auto p : field.points)
                ROS_INFO_STREAM("Poly point: [" << p.x << " " << p.y << "]");
        }
        return true;
    }

    void TaskManager::publishField(const ros::TimerEvent&)
    {
        if (field.points.size() > 0)
        {
            geometry_msgs::PolygonStamped f;
            f.polygon = field;
            f.header.stamp = ros::Time::now();
            f.header.frame_id = "map";
            field_pub.publish(f);
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task_manager");
    aseta::TaskManager tm;
    ros::spin();
    return 0;
}
