// Header file for the Waypoint class of the GATSP project.
// 
// This file is part of the GATSP project.
// Copyright 2012-2013 Karl Damkjær Hansen - Aalborg University

#ifndef GATSP_WAYPOINT_H
#define GATSP_WAYPOINT_H

#include <ostream>
#include "gatsp/Point.h"
#include "gatsp/Quarternion.h"

namespace gatsp
{
    class Waypoint
    {
    public:
        Waypoint();
        Waypoint(Point, Quarternion);

        Point point() const;
        void point(Point);

        double x() const;
        void x(double);

        double y() const;
        void y(double);

        double z() const;
        void z(double);

        Quarternion quarternion() const;
        void quarternion(Quarternion);

    private:
        Point _point;
        Quarternion _quarternion;
    };
};

std::ostream& operator<<(std::ostream&, gatsp::Waypoint&);

#endif // GATSP_WAYPOINT_H