// Header file for the Problem class of the GATSP project.
// 
// This file is part of the GATSP project.
// Copyright 2012-2013 Karl Damkjær Hansen, Aalborg University

#ifndef GATSP_PROBLEM_H
#define GATSP_PROBLEM_H

//#include <istream>
#include <ostream>
#include <string>
#include <vector>
#include "gatsp/Waypoint.h"
//#include "gatsp/errors.h"


namespace gatsp
{
    /// Represents a TSP problem.
    class Problem
    {
    public:
        Problem();
        Problem(const std::vector<Waypoint>&);
        virtual ~Problem();
        
        const std::string name() const;
        void name(const std::string);

        const std::vector<Waypoint> waypoints() const;
        const std::vector<Waypoint> orderedWaypoints() const;
        void waypoints(const std::vector<Waypoint>&);

        virtual void addWaypoint(const Waypoint&);
        virtual void removeWaypoint(size_t);
        void reserve(size_t);

        void fixedFirstWaypoint(bool);
        bool fixedFirstWaypoint();

        const std::vector<size_t>& solution() const;
        //virtual void parseTsplibSolution(std::istream&); // On pause until I get regex working.

    protected:
        std::string _name;
        std::vector<Waypoint> _waypoints;
        Waypoint _endpoint;
        std::vector<size_t> _solution;
        bool _fixed_first_wp;
    };
};

std::ostream& operator<< (std::ostream&, gatsp::Problem&);

#endif // GATSP_PROBLEM_H
