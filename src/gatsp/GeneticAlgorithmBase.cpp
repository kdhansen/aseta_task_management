/*
Genetic Algorithm Base
Base class for the GATSP Genetic Algorithm
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

#include <limits>
#include "gatsp/GeneticAlgorithmBase.h"

namespace gatsp
{
    // Constructor
    GeneticAlgorithmBase::GeneticAlgorithmBase(
        std::shared_ptr<ProblemBase> problem,
        int num_individuals = 100,
        double mutate_rate = 1.0,
        double crossover_rate = 0.0,
        unsigned int seed = 0
    ) :
        _problem(problem),
        _num_individuals(num_individuals),
        _mutate_rate(mutate_rate),
        _crossover_rate(crossover_rate),
        _random_generator(std::mt19937(seed)),
        _best_cost(std::numeric_limits<double>::infinity())
    {
        // Generate the population.
        _individuals.reserve(_num_individuals);
        for (size_t i = 0; i < _num_individuals; ++i)
        {
        	_individuals.push_back(problem->makeSolution());
        }
        evaluateIndividuals();
    }

    // Evolve the problem until the solution criterion is satisfied.
    //
    // Simply step()s until the termination criterion returns true.
    // Should be ideal for threading (using boost::thread).
    void GeneticAlgorithmBase::evolve()
    {
        while (! terminate() && ! isRunning())
        {
            try
            {
                step();
                boost::this_thread::interruption_point();
            }	
            catch (const boost::thread_interrupted&)
            {
                stop();
                break;
            }
        }

        // If we exited the while loop because the termination criterion
        // returned true, let's just note that we are done.
        if (terminate())
        {
            boost::lock_guard<boost::mutex> guard(_lock);
            _isDone = true;
        }
    }

    /// Stop the algorithm.
    /// May be used both from the outside and inside of the algorithm.
    void GeneticAlgorithmBase::stop()
    {
        boost::lock_guard<boost::mutex> guard(_lock);
        _isRunning = false;
        return;
    }

    /// Test if the algorithm is running.
    ///
    /// @returns true if the algorithm is running.
    bool GeneticAlgorithmBase::isRunning()
    {
        boost::lock_guard<boost::mutex> guard(_lock);
        return _isRunning;
    }

    /// Test whether the algorithm was stopped by the termination criterion.
    ///
    /// @returns true if the algorithm was stopped by the termination criterion.
    bool GeneticAlgorithmBase::isDone()
    {
        boost::lock_guard<boost::mutex> guard(_lock);
        return _isDone;
    }

    /// Advance the algorithm one generation.
    ///
    void GeneticAlgorithmBase::step()
    {
        // - Make babies. (crossover)
        // - Mutate the babies.
        crossover();
        mutate();
        evaluateIndividuals();
        return;
    }

    /// Get the best solution.
    ///
    SolutionBase GeneticAlgorithmBase::bestSolution()
    {
        return _best_individual;
    }

    /// Evaluate the individuals and update the all-time high.
    ///
    void GeneticAlgorithmBase::evaluateIndividuals()
    {
        std::vector<double> new_costs;
        for (auto ind : _individuals)
        {
            double cost = _problem->solutionCost(ind);
            new_costs.push_back(cost);
            if (cost < _best_cost)
            {
                _best_cost = cost;
                _best_individual = ind;
            }
        }
        _costs.swap(new_costs);
        return;
    }
}