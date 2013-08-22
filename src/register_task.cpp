/*
ASETA Register Task Util.
Register a Photograph area with the ASETA Task Manager.
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
#include <geometry_msgs/Polygon.h>
#include "aseta_task_management/PhotographArea.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "register_task");

    geometry_msgs::Polygon poly;
    float x[] = {1, 2, 2, 1};
    float y[] = {1, 1, 2, 2};
    for (int i = 0; i < 4; ++i)
    {
        geometry_msgs::Point32 p;
        p.x = x[i];
        p.y = y[i];
        p.z = 0;
        poly.points.push_back(p);
    }

    aseta_task_management::PhotographArea pa;
    pa.request.area = poly;
    pa.request.mm_pr_px = 0.1;

    if (ros::service::call("/task_manager/register_task", pa))
    {
        ROS_INFO("Service: /task_manager/register_task called.");
    }
    else
    {
        ROS_INFO("Service: /task_manager/register_task failed.");
    }
    return 0;
}
