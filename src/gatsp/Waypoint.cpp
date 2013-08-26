// Waypoint.cpp
// Implements the waypoint class. 
// Declarations are in Waypoint.h
//
// This file is part of the GATSP project.
// Copyright 2012-2013 Karl Damkjær Hansen, Aalborg University

#include "gatsp/Waypoint.h"

namespace gatsp
{
    /// Default constructor
    Waypoint::Waypoint() 
    {
    }

    /// Construct a Waypoint with an initial set of coordinates.
    ///
    /// @param point Coordinates of the Waypoint in free space
    /// @param quarternion the pose of the waypoint at the coordinates.
    Waypoint::Waypoint(Point point, Quarternion quarternion) :
        _point(point),
        _quarternion(quarternion)
    {
    }

    /// Get the point of the waypoint in free space.
    ///
    /// Functions for the individual coordinates x/y/z exist.
    ///
    /// @returns Point in free space
    Point Waypoint::point() const
    {
        return _point;
    }

    /// Set the point of the waypoint in free space.
    ///
    /// Functions for setting the individual x/y/z coordinate exists.
    ///
    /// @param p Point in free space.
    void Waypoint::point(Point p)
    {
        _point = p;
    }

    /// Get the x-coordinate.
    ///
    /// @returns x-coordinate.
    double Waypoint::x() const
    {
        return _point.x;
    }

    /// Set the x-coordinate.
    ///
    /// @param x The new x-coordinate.
    void Waypoint::x(double x)
    {
        _point.x = x;
    }

    /// Get the y-coordinate.
    ///
    /// @returns y-coordinate.
    double Waypoint::y() const
    {
        return _point.y;
    }

    /// Set the y-coordinate.
    ///
    /// @param y The new y-coordinate.
    void Waypoint::y(double y)
    {
        _point.y = y;
    }

    /// Get the z-coordinate.
    ///
    /// @returns z-coordinate.
    double Waypoint::z() const
    {
        return _point.z;
    }

    /// Set the z-coordinate.
    ///
    /// @param z The new y-coordinate.
    void Waypoint::z(double z)
    {
        _point.z = z;
    }

    /// Get the quarternion/pose of the waypoint
    ///
    /// @returns The quarternion.
    Quarternion Waypoint::quarternion() const
    {
        return _quarternion;
    }

    /// Set the quarternion
    ///
    /// @param quarternion The new quarternion.
    void Waypoint::quarternion(Quarternion quarternion)
    {
        _quarternion = quarternion;
    }
}

std::ostream& operator<<(std::ostream& out, gatsp::Waypoint& wp)
{
    auto q = wp.quarternion();
    out << "(Point:[" << wp.x() << ", " << wp.y() << ", " << wp.z() << "], Heading:[" << q.x << ", " << q.y << ", " << q.z << ", " << q.w <<"])";
    return out;
}

