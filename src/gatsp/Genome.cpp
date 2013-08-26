// Source file for the Genome class of the gatsp project.
//
// Declarations are in Genome.h
//
// This file is part of the GATSP project.
// Copyright 2013 Karl Damkjær Hansen, Aalborg University
#include <deque>
#include "gatsp/Genome.h"

namespace gatsp
{
	/// "Default" constructor.
	/// The genome is initialized with the random number generator
	/// of the genetic algorithm.
	///
	/// @param generator A Mersenne Twister 19937 64 bit generator..
	Genome::Genome(std::mt19937_64& generator) :
		_generator(generator),
        _evaluated(false)
	{}

	/// Construct a genome based on an existing Problem.
	///
	/// @param problem The original Problem.
	/// @param generator A Mersenne Twister 19937 64 bit generator.
	Genome::Genome(const Problem& problem, std::mt19937_64& generator) :
		Problem(problem),
		_generator(generator),
        _evaluated(false)
	{}

	/// Destructor
	///
	Genome::~Genome()
	{}

    /// Assignment Operator
    ///
    Genome& Genome::operator=(const Genome& rhs)
    {
        Problem::operator=(rhs);
        _generator = rhs._generator;
        _evaluated = false;
        return *this;
    }

	/// Randomizes the solution.
	/// It used in the initialization process of the Population.
	///
	void Genome::randomize()
	{
		_solution.clear();
        std::deque<size_t> sequential_indexes;
        for (size_t i = 0; i < _waypoints.size(); ++i)
        {
            sequential_indexes.push_back(i);
        }
        while (sequential_indexes.size() > 0)
        {
    		std::uniform_int_distribution<size_t> rand_int(0, sequential_indexes.size()-1);   	
            size_t rand_idx = rand_int(_generator);
            _solution.push_back(sequential_indexes[rand_idx]);
            sequential_indexes.erase(sequential_indexes.begin() + rand_idx);
        }
		return;
	}

	/// Mutates the solution of the problem. Thre different mutations are implemented:
    /// The displacement, exchange and inversion mutations.
	///
	void Genome::mutate()
	{
        if (2 < _waypoints.size())
        {
            // Choose mutation.
            std::uniform_int_distribution<unsigned int> which_mutation(0, 2);
            switch (which_mutation(_generator))
            {
                case 0:
                    (*this).displacementMutation();
                    break;
                case 1:
                    (*this).exchangeMutation();
                    break;
                case 2:
                    (*this).inversionMutation();
                    break;
            }
        }

        // Set flag to trigger evaluation.
        _evaluated = false;
		return;
	}

	/// Evaluates the solution quality based on the Euclidean distance.
	///
	/// @returns Length of solution.
	double Genome::evaluate()
	{
        if (!_evaluated)
        {
            if (2 >_waypoints.size()) // Cannot compute distance between less than two wps.
            {
                _score = 0.0;
            }
            else
            {
                double dist = 0.0;
                for (auto s = _solution.begin(); s!= _solution.end()-1; ++s)
                {
                    auto from = _waypoints[*s];
                    auto to = _waypoints[*(s+1)];
                    double dx = to.x() - from.x();
                    double dy = to.y() - from.y();
                    double dz = to.z() - from.z();
                    dist += sqrt(dx*dx + dy*dy + dz*dz);
                }

                // Goto endpoint
                auto from = _waypoints[*(_solution.end()-1)];
                double dx = _endpoint.x() - from.x();
                double dy = _endpoint.y() - from.y();
                double dz = _endpoint.z() - from.z();
                dist += sqrt(dx*dx + dy*dy + dz*dz);

                _score = dist;
            }
        }

        return _score;
	}

    /// Add a Waypoint to the Genome.
    ///
    /// @param newWaypoint Waypoint that will be added to the set of waypoints.
    void Genome::addWaypoint(const Waypoint& newWaypoint)
    {
        _evaluated = false;
        Problem::addWaypoint(newWaypoint);
    }

    /// Remove a waypoint.
    ///
    /// @param n index to remove.
    void Genome::removeWaypoint(size_t n)
    {
        _evaluated = false;
        Problem::removeWaypoint(n);
    }

	/// Displacement mutation. Selects a range of indices and "slides" them
    /// back or forth in the solution. One special case is where the 
    /// insertion_point is where the range of indices already are located.
    /// In that case, the vector is not mutated.
    ///
    void Genome::displacementMutation()
    {
        size_t solution_length = _solution.size();
        size_t first_idx = 0;
        if (_fixed_first_wp)
            first_idx = 1;
        std::uniform_int_distribution<size_t> dist_subtour_index(_fixed_first_wp, solution_length-1);
        size_t subtour_index = dist_subtour_index(_generator);
        std::uniform_int_distribution<size_t> dist_subtour_length(1, solution_length - subtour_index);
        size_t subtour_length = dist_subtour_length(_generator);
        std::uniform_int_distribution<size_t> dist_insertion_point(first_idx, solution_length - subtour_length);
        size_t insertion_point = dist_insertion_point(_generator);

        auto subtour_begin = _solution.begin() + subtour_index;
        auto subtour_end = subtour_begin + subtour_length;      

        if (insertion_point > subtour_index)
        // Here the subtour is picked out in a temporary vector while the
        // affected region is moved.
        {
            std::vector<size_t> subtour(subtour_begin, subtour_end);
            auto affected_portion_begin = subtour_end;
            auto affected_portion_end =  _solution.begin() 
                                       + insertion_point
                                       + subtour_length;
            auto to = subtour_begin;
            for (auto from = affected_portion_begin;
                 from != affected_portion_end;
                 ++from)
            {
                *to = *from;
                ++to;
            }
            for (auto from = subtour.begin();
                 from != subtour.end();
                 ++from)
            {
                *to = *from;
                ++to;
            }
        } 
        else if (insertion_point < subtour_index)
        // Here the affected region is picked out instead.
        {
            auto affected_portion_begin = _solution.begin() + insertion_point;
            auto affected_portion_end = subtour_begin;
            std::vector<size_t> affected_portion(affected_portion_begin,
                                                 affected_portion_end);

            auto to = affected_portion_begin;
            for (auto from = subtour_begin;
                 from != subtour_end;
                 ++from)
            {
                *to = *from;
                ++to;
            }
            for (auto from = affected_portion.begin();
                 from != affected_portion.end();
                 ++from)
            {
                *to = *from;
                ++to;
            }
        }

        return;
    }

    /// Exchange mutation. This mutation chooses two indices and swaps the
    /// content.
    ///
    void Genome::exchangeMutation()
    {
        size_t solution_length = _solution.size();
        size_t first_idx = 0;
        if (_fixed_first_wp)
            first_idx = 1;
        std::uniform_int_distribution<size_t> dist(first_idx, solution_length-1);
        size_t index1 = dist(_generator);
        size_t index2 = dist(_generator);
        while (index1 == index2)
        {
            index2 = dist(_generator);
        }
        auto section_begin = _solution.begin() + index1;
        auto section_end = _solution.begin() + index2;
        std::swap(*section_begin, *section_end);

    }

    /// Inversion mutation. This mutation chooses a range in the list and
    /// reverses the sequence in that range.
    ///
    /// @param solution Vector to mutate.
    void Genome::inversionMutation()
    {
        size_t solution_length = _solution.size();
        size_t first_idx = 0;
        if (_fixed_first_wp)
            first_idx = 1;
        std::uniform_int_distribution<size_t> dist(first_idx, solution_length-1);
        size_t index1 = dist(_generator);
        size_t index2 = dist(_generator);
        while (index1 == index2)
        {
            index2 = dist(_generator);
        }
        if (index1 > index2)
        {
            std::swap(index1, index2);
        }
        auto section_begin = _solution.begin() + index1;
        auto section_end = _solution.begin() + index2;
        size_t num_swaps = (index2 - index1) / 2 + 1;
        for (size_t i = 0; i < num_swaps; ++i)
        {
            std::swap(*section_begin, *section_end);
            ++section_begin;
            --section_end;
        }
    }
}
