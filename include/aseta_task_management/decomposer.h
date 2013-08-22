/*
ASETA Decomposer
Splits an area into smaller partitions that can be photographed.
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

#ifndef DECOMPOSER_H
#define DECOMPOSER_H

#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>

namespace aseta
{
	class Decomposer
	{
	public:
		Decomposer(float, float, float, int, int);
		~Decomposer();

		void decompose(geometry_msgs::Polygon const &, std::vector<geometry_msgs::Point> &);

	private:
		float footprint_height;
        float footprint_width;
		float flying_height;
	};
};

#endif
