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

#include <algorithm>
#include <cmath>
#include <limits>
#include <boost/bind.hpp>
#include <clippoly/poly.h>
#include <clippoly/nclip.h>
#include "aseta_task_management/decomposer.h"

namespace aseta
{
    /// Generates waypoints to photograph an area.
    ///
    /// @param focal_length_x The focal length in pixel-relation in the x-direction.
    /// @param focal_length_y The focal length in pixel-relation in the y-direction.
    /// @param m_pr_px Desired resolution of the photograph. Expressed in metres per pixel.
    /// @param image_height Height of the image sensor in pixels.
    /// @param image_width Width of the image sensor in pixels.
    Decomposer::Decomposer(float focal_length_x,
                           float focal_length_y, 
                           float m_pr_px,
                           int image_height,
                           int image_width)
    {
        // The footprint is the images projection on the ground.
        footprint_height = image_height * m_pr_px;
        footprint_width = image_width * m_pr_px;
        // If the pixels in the camera is not square, we will have different
        // focal lengths for x and y, and thus two different heights for the
        // waypoint, we'll pick the lowest, then we meet the criterion for both
        // directions.
        float desired_height_h = footprint_height * focal_length_y / image_height;
        float desired_height_w = footprint_width * focal_length_x / image_width;
        if (desired_height_h < desired_height_w)
        {
            flying_height = desired_height_h;
            footprint_width = image_width * flying_height / focal_length_x;
        }
        else
        {
            flying_height = desired_height_w;
            footprint_height = image_height * flying_height / focal_length_y;
        }
        ROS_DEBUG_STREAM("Footprint: " << footprint_width << " x " << footprint_height);
        ROS_DEBUG_STREAM("Flying_height: " << flying_height);
    }

    Decomposer::~Decomposer()
    {}

    /// Decompose an area into waypoints.
    ///
    /// Given the camera information given in the constructor. The area specified
    /// is partitioned into a checkerboard-pattern of photos.
    ///
    /// @param area The area to photograph
    /// @param[out] waypoints The waypoints generated are appended to this vector.
    void Decomposer::decompose(geometry_msgs::Polygon const &area, std::vector<geometry_msgs::Point> &waypoints)
    {
        // Make a polygon from the input
        Poly field(Point(area.points[0].x, area.points[0].y));
        for (int i = 1; i < area.points.size(); ++i)
        {
            field.add(Point(area.points[i].x, area.points[i].y));
        }

        // Make a checkerboard that covers it completely
            // Find the right/left/top/bottom-most coordinates.
        float min_x = std::numeric_limits<float>::infinity();
        float min_y = min_x;
        float max_x = - std::numeric_limits<float>::infinity();
        float max_y = max_x;
        for (auto point : area.points)
        {
            if (point.x > max_x) max_x = point.x;
            if (point.y > max_y) max_y = point.y;
            if (point.x < min_x) min_x = point.x;
            if (point.y < min_y) min_y = point.y;
        }
        ROS_DEBUG_STREAM("Bounding box: [" << min_x << " " << min_y << "][" << max_x << " " << max_y << "]");
            // Find out how many footprints that fits.
        int num_vertical_footprints = ceil((max_x - min_x) / footprint_width);
        int num_horisontal_footprints = ceil((max_y - min_y) / footprint_height);
        ROS_DEBUG_STREAM("Number of horisontal images: " << num_horisontal_footprints);
        ROS_DEBUG_STREAM("Number of vertical images: " << num_vertical_footprints);
            // Since we ceil the amount, we get a margin.
        float margin_x = ((num_vertical_footprints * footprint_width) - (max_x - min_x)) / 2;
        ROS_DEBUG_STREAM("Margin x: " << margin_x);
        float margin_y = ((num_horisontal_footprints * footprint_height) - (max_y - min_y)) / 2;
        ROS_DEBUG_STREAM("Margin y: " << margin_y);
        float initial_x = min_x - margin_x;
        float initial_y = min_y - margin_y;
        ROS_DEBUG_STREAM("Initial: " << initial_x << " " << initial_y);
            // Make a vector of footprints.
        std::vector<Poly> footprint_list;
        for (int h = 0; h < num_horisontal_footprints; ++h)
        {
            for (int v = 0; v < num_vertical_footprints; ++v)
            {
                float x = initial_x + v*footprint_width;
                float y = initial_y + h*footprint_height;
                Poly fp(Point(x, y));
                fp.add(Point(x + footprint_width, y));
                fp.add(Point(x + footprint_width, y + footprint_height));
                fp.add(Point(x, y + footprint_height));
                footprint_list.push_back(fp);
            }
        }

        // Return the waypoints of the footprints that overlap withe field
        // The position of the way point is assumed to be straight over the
        // middle of the footprint, i.e. the drone is not tilting.
        for (auto fp : footprint_list)
        {
            PolyPList   a_min_b, b_min_a, overlap;
            clip_poly(field, fp, a_min_b, b_min_a, overlap);
            if (!overlap.empty())
            {
                geometry_msgs::Point p;
                p.x = fp.firstpoint().x() + footprint_width/2;
                p.y = fp.firstpoint().y() + footprint_height/2;
                p.z = flying_height;
                waypoints.push_back(p);
            }
        }

        return;
    }
}
