// Problem.cpp
// Implements the problem class. 
// Definitions are in Problem.h
//
// This file is part of the GATSP project.
// Copyright 2012-2013 Karl Damkjær Hansen, Aalborg University

#include <algorithm>
#include <regex>
#include <sstream>
#include "gatsp/Problem.h"

namespace gatsp
{

    /// Default constructor.
    Problem::Problem() :
        _fixed_first_wp(false)
    {
    }

    /// Constructor initializing with a set of waypoints. Further Waypoints can
    /// be added later.
    ///
    /// @param wps Initial set of waypoints for the problem.
    Problem::Problem(const std::vector<Waypoint>& wps) : 
        _fixed_first_wp(false)
    {
        waypoints(wps);
    }

    /// Destructor.
    Problem::~Problem()
    {
    }

    /// Get the name of the Problem.
    ///
    /// @returns The name of the problem, if it is set.
    const std::string Problem::name() const 
    {
        return _name;
    }

    /// Set the name of the Problem.
    ///
    /// @param name New name of the Problem, usually given in a TSPLIB file.
    void Problem::name(std::string name)
    {
        _name = name;
    }

    /// Get the waypoints in the problem.
    ///
    /// @returns A copy of the problems' waypoints in a vector.
    const std::vector<Waypoint> Problem::waypoints() const
    {
        std::vector<Waypoint> v(_waypoints.begin(), _waypoints.end());
        v.push_back(_endpoint);
        return v;
    }

    /// Get the waypoints in the problem ordered according to the solution.
    ///
    /// @returns A copy of the problems' ordered waypoints in a vector.
    const std::vector<Waypoint> Problem::orderedWaypoints() const
    {
        std::vector<Waypoint> v;
        for (auto s = _solution.begin(); s != _solution.end(); ++s)
        {
            v.push_back(_waypoints[*s]);
        }
        v.push_back(_endpoint);
        return v;
    }

    /// Set the waypoints of the problem.
    void Problem::waypoints(const std::vector<Waypoint>& wps)
    {
        _waypoints = wps;
        for (size_t i = 0; i < _waypoints.size(); ++i)
        {
            _solution.push_back(i);
        }
        _endpoint = *(wps.rbegin());
    }

    /// Add a Waypoint to the Problem.
    ///
    /// @param newWaypoint Waypoint that will be added to the set of waypoints.
    void Problem::addWaypoint(const Waypoint& newWaypoint)
    {
        _waypoints.push_back(newWaypoint);
        size_t num_wps = _solution.size();
        _solution.push_back(num_wps);
        if (0 == num_wps)
        {
            _endpoint = newWaypoint;
        }
    }

    /// Remove a waypoint.
    ///
    /// @param n index to remove.
    void Problem::removeWaypoint(size_t n)
    {
        auto remove_idx = std::find(_solution.begin(), _solution.end(), n);
        _solution.erase(remove_idx);
        for (auto& idx : _solution)
        {
            if (n < idx)
            {
                idx -= 1;
            }
        }
        _waypoints.erase(_waypoints.begin()+n);
    }

    /// Reserve space for a (large) number of waypoints. This should only
    /// be used when you expect to add a large number of waypoints. It is 
    /// not necessary to reserve space, but it improves efficiency for large
    /// problems.
    ///
    /// @param n Number of waypoints to reserve space for.
    void Problem::reserve(size_t n)
    {
        _waypoints.reserve(n);
    }

    /// Set whether the first waypoint should be fixed.
    /// That is if it should always remain the first waypoint
    /// in the sequence.
    ///
    /// @param setFixed Should the first waypoint be fixed..
    void Problem::fixedFirstWaypoint(bool setFixed)
    {
        _fixed_first_wp = setFixed;
    }

    /// See whether the first waypoint is fixed.
    ///
    bool Problem::fixedFirstWaypoint()
    {
        return _fixed_first_wp;
    }

    /// Get the solution vector.
    const std::vector<size_t>& Problem::solution() const
    {
        return _solution;
    }

    // On pause until I get regex to work
    // void Problem::parseTsplibSolution(std::istream& tspStream)
    // {
    //     std::string myLine;
    //     // Index for dimension
    //     size_t dim = 0;
    //     // Read line by line to get the parameters
    //     while (std::getline(tspStream, myLine)) // Hopefully we will not reach EOF but the TOUR_SECTION instead
    //     {
    //         std::smatch m;
    //         // See if line is a parameter
    //         if ( std::regex_match(myLine, m, std::regex("(\\w*)[ ]?:[ ]?(\\w*)")) )
    //         {
    //             if (m[1] == "DIMENSION" && m.size() == 3)
    //             {
    //                 // Converting string to an index
    //                 std::stringstream ss;
    //                 ss << m[2];
    //                 ss >> dim;
    //                 // Allocating space for the tour
    //                 _solution.reserve(dim);
    //             }
    //             else if (m[1] == "TYPE" && m.size() == 3)
    //             {
    //                 if (m[2] != "TOUR")
    //                 {
    //                     throw gatsp::TspParserError("Solution file formatted wrongly.");
    //                 }
    //             }
    //         }
    //         // Check to see if we reached the tour section
    //         else if ( std::regex_match(myLine, std::regex("TOUR_SECTION")) )
    //         {
    //             break; // Continue below to read all indexes
    //         }
    //     }

    //     // Read indexes
    //     for (size_t i = 0; i < dim; ++i)
    //     {
    //         std::getline(tspStream, myLine);
    //         if (myLine == "EOF" || myLine == "-1") // The file holds a explicit "EOF" in the end and a -1 to stop the tour.
    //         {
    //             break;
    //         }
    //         else
    //         {
    //             std::stringstream ss;
    //             ss << myLine;
    //             size_t idx;
    //             ss >> idx;
    //             _solution.push_back(idx - 1); // TSPLIB is index base 1, hence the -1
    //         }
    //     }

    //     return;
    // }
}

std::ostream& operator<< (std::ostream& out, gatsp::Problem& prob)
{
    out << "--- Problem \"" << prob.name() << "\" ---" << std::endl;
    out << "Waypoints:" << std::endl;
    auto wps = prob.waypoints();
    for(auto it = wps.begin(); it != wps.end(); ++it)
    {
        out << *it << std::endl;
    }
    out << "Solution:" << std::endl;
    out << "[ ";
    auto sol = prob.solution();
    for (auto it = sol.begin(); it != sol.end(); ++it)
    {
        out << *it << " ";
    }
    out << "]";
    return out;
}
