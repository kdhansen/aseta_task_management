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

#ifndef GENETIC_ALGORITHM_BASE
#define GENETIC_ALGORITHM_BASE

#include <memory>
#include <random>
#include <boost/thread.hpp>
#include "gatsp/ProblemBase.h"
#include "gatsp/SolutionBase.h"

namespace gatsp
{
	class GeneticAlgorithmBase
	{
	public:
		GeneticAlgorithmBase(
			std::shared_ptr<ProblemBase> problem,
		    int num_individuals, 
		    double mutate_rate, 
		    double crossover_rate, 
		    unsigned int seed
		);
		virtual ~GeneticAlgorithmBase() = default;

		void evolve();
		void stop();
		bool isRunning();
		bool isDone();
		virtual void step();
		SolutionBase bestSolution();

	private:
		virtual void mutate() = 0;
		virtual void crossover() = 0;
		virtual bool terminate() = 0;

		void evaluateIndividuals();

	protected:
		std::shared_ptr<ProblemBase> _problem;
		
		int	_num_individuals;
		std::vector<SolutionBase> _individuals;
		std::vector<double> _costs;
		SolutionBase _best_individual;
		double _best_cost;

		double _mutate_rate;
		double _crossover_rate;

		std::mt19937 _random_generator;

		boost::mutex _lock;
		bool _isRunning;
		bool _isDone;
	};
}

#endif // GENETIC_ALGORITHM_BASE