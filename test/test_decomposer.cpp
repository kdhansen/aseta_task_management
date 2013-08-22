/*
Test for ASETA decomposer
Checks whether the decomposer decomposes correctly.
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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "aseta_task_management/PhotoWaypointAction.h"
#include "aseta_task_management/decomposer.h"

// Declare a test
TEST(Decomposer, rightHeigth)
{
	aseta::Decomposer d(100,100,0.01,10,10);
	
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

	std::vector<geometry_msgs::Point> waypoints;

	d.decompose(poly, waypoints);

	ASSERT_GT(waypoints.size(), 0);

	EXPECT_LT(1-waypoints[0].z, 0.001);
}

int main(int argc, char *argv[])
{
	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}