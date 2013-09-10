/*
Problem Base
Base class for the GATSP Problem
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

#ifndef PROBLEM_BASE_H
#define PROBLEM_BASE_H

#include <memory>
#include <vector>
#include "gatsp/Waypoint.h"
#include "gatsp/SolutionBase.h"

namespace gatsp
{
    class ProblemBase
    {
    public:
        ProblemBase() = default;
        virtual ~ProblemBase() = default;

        virtual std::shared_ptr<SolutionBase> makeSolution() = 0;
        virtual std::vector<Waypoint> route(const SolutionBase&) = 0;
        virtual Waypoint firstWaypoint(const SolutionBase&) = 0;
        virtual void popWaypointFront(const SolutionBase&) = 0;
        virtual void addWaypoint(const Waypoint&) = 0;

        virtual double solutionCost(const SolutionBase&) const = 0;
        virtual bool isSolutionValid(const SolutionBase&) const = 0;
        virtual bool repairSolution(const SolutionBase&) const = 0;
    };
}

#endif // PROBLEM_BASE_H