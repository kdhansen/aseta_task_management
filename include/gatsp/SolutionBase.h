/*
Solution Base
Base class for the GATSP Solution
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

#ifndef SOLUTION_H
#define SOLUTION_H

#include <exception>

namespace gatsp
{
    struct SolutionEntryBase
    {
        SolutionEntryBase() = default;
        SolutionEntryBase(size_t i) {index = i;}
        size_t index;
    };

    typedef std::vector<SolutionEntryBase> SolutionBase;

    class InvalidSolution : public std::exception
    {
        virtual const char* what() const throw()
        {
            return "Solution invalid. Maybe try gatsp::Problem::repairSolution().";
        }
    };
}

#endif // SOLUTION_H