// Source file for the GeneticAlgorithm class of the gatsp project.
//
// Declarations are in GeneticAlgorithm.h
//
// This file is part of the GATSP project.
// Copyright 2013 Karl Damkjær Hansen, Aalborg University

#include <chrono>
#include <iostream>
#include <limits>
#include "gatsp/GeneticAlgorithm.h"

namespace gatsp
{	
	/// Default constructor
	GeneticAlgorithm::GeneticAlgorithm() :
		_terminator(&GeneticAlgorithm::terminateGenerations),
		_num_generations(0),
		_max_generations(100),
		_best_individual_idx(0)
	{
		// Set the seed to the clock
		unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
		_generator = std::mt19937_64(seed);

		// Create the population with an empty problem.
		Problem prototype;
		size_t population_size = 50;
		_population = std::vector<Genome>(population_size, Genome(prototype, _generator));
	}

	/// Constructs the genetic algorithm from a specified prototype.
	///
	/// @param prototype Problem to spark the evolution.
	/// @param population_size How many genomes should the population consist of.
	/// @param seed Seed for the randomization engine. If 0 is given, the seed is taken of the system clock.
	GeneticAlgorithm::GeneticAlgorithm(const Problem& prototype, size_t population_size, unsigned int seed) :
		_terminator(&GeneticAlgorithm::terminateGenerations),
		_num_generations(0),
		_max_generations(100),
		_best_individual_idx(0)
	{
		// Set the seed to the clock if the user supplies 0 for seed
		if (seed == 0)
		{
			seed = std::chrono::system_clock::now().time_since_epoch().count();
		}
		_generator = std::mt19937_64(seed);

		// Create the population and randomize them.
		_population = std::vector<Genome>(population_size, Genome(prototype, _generator));
		for(auto& g : _population)
		{
			g.randomize();
		}
	}

	/// Destructor
	///
	GeneticAlgorithm::~GeneticAlgorithm()
	{
	}

	/// Evolves the population one generation.
	/// This is essentially the core of the genetic algorithm.
	///
	void GeneticAlgorithm::step()
	{
		// Select individuals for breeding. 
		// - Make a roulette wheel 
		//   (remember we are minimizing the length, thus 1.0/x)
		//   This conveniently also evaluates the individuals.
		std::vector<double> roulette_wheel;
		roulette_wheel.reserve(_population.size());
		double score = 0.0;
		for (auto& individual : _population)
		{
			score += 1.0/individual.evaluate();
			roulette_wheel.push_back(score);
		}
		double total_score = score;

		// Make children
		std::vector<Genome> children;
		children.reserve(_population.size());
		for (size_t idx_child = 0; idx_child < _population.size(); ++idx_child)
		{
			std::uniform_real_distribution<double> roulette_ball(0, total_score);
			double ball_value = roulette_ball(_generator);
			size_t idx_parent = 0;
			for (double r : roulette_wheel)
			{
				if (r >= ball_value)
				{
					break;
				}
				else
				{
					++idx_parent;
				}
			}
			Genome g(_population[idx_parent]);
			children.push_back(g);
		}

		// Mutate the children
		for (auto& child : children)
		{
			child.mutate();
		}

		// Elitism is any of the parents better than all of the children,
		// then he is going to replace the worst of the children.
		double best_parent_score = _population[_best_individual_idx].evaluate();
		size_t best_parent_idx = _best_individual_idx;
		double worst_child_score = 0.0;
		double best_child_score = std::numeric_limits<double>::infinity();
		size_t worst_child_idx = 0;
		size_t child_idx = 0;
		for (auto& child : children)
		{
			if (child.evaluate() < best_child_score)
			{
				best_child_score = child.evaluate();
			}
			if (child.evaluate() > worst_child_score)
			{
				worst_child_score = child.evaluate();
				worst_child_idx = child_idx;
			}
			++child_idx;
		}
		if (best_parent_score < best_child_score)
		{
			children[worst_child_idx] = _population[best_parent_idx];
		}

		// Swap the children and the parents.
		_population.swap(children);

		// Update best individual
		findBestIndividual();

		_num_generations += 1;
	}

	/// Continues to evolve the population until the termination criterion has been reached.
	///
	void GeneticAlgorithm::evolve()
	{
		while (!done())
		{
			try
			{
				_lock.lock();
				step();
				_lock.unlock();
				boost::this_thread::interruption_point();
			}
			catch (const boost::thread_interrupted&)
			{
				std::cout << "GA interrupted..." << std::endl;
				break;
			}
		}
		std::cout << "GA done..." << std::endl;
		return;
	}

	/// Reports how many generations the algorithm has evolved.
	///
	/// @returns Number of evolved generations.
	unsigned int GeneticAlgorithm::generation() const
	{
		boost::lock_guard<boost::mutex> guard(_lock);
		return _num_generations;
	}

	/// Set the maximal number of generations to evolve.
	///
	/// @param generations Maximum number of generations.
	void GeneticAlgorithm::maxGenerations(unsigned int generations)
	{
		boost::lock_guard<boost::mutex> guard(_lock);
		_max_generations = generations;
	}

	/// Checks whether the the termination criterion has been fulfilled.
	///
	/// @returns True if the algorithm should terminate.
	bool GeneticAlgorithm::done()
	{
		return _terminator(*this);
	}

	/// Set the terminator to something new.
	///
	/// @param terminator_function A function indicating wheter to terminate
	///                            the algorithm.
	void GeneticAlgorithm::setTerminator(std::function<bool(GeneticAlgorithm&)> terminator_function)
	{
		boost::lock_guard<boost::mutex> guard(_lock);
		_terminator = terminator_function;
	}

	/// Termination criterion that terminates upon reaching max generations.
	///
	bool GeneticAlgorithm::terminateGenerations()
	{
		boost::lock_guard<boost::mutex> guard(_lock);
		bool do_terminate = false;
		if (_num_generations >= _max_generations)
		{
			do_terminate = true;
		}
		return do_terminate;
	}

	/// Termination criterion that adaptively stops the algorithm.
	///
	bool GeneticAlgorithm::terminateAdaptive()
	{
		return true;
	}

	/// Termination criterion that never stops the algorithm.
	///
	bool GeneticAlgorithm::terminateInfinite()
	{
		return false;
	}

	/// Add a waypoint to all individuals.
	///
	/// @param newWaypoint The Waypoint to add.
	void GeneticAlgorithm::addWaypoint(const Waypoint& newWaypoint)
	{
		boost::lock_guard<boost::mutex> guard(_lock);
		for (auto& individual : _population)
		{
			individual.addWaypoint(newWaypoint);
		}
		findBestIndividual();
		
		return;
	}

	/// Remove a specific waypoint from all individuals.
	///
	/// The indices are the same across individuals. So the id:
	/// id = Problem.solution()[n];
	/// will refer to the same waypoint in other individuals,
	/// eventhough the n may be different.
	///
	/// @param n index of waypoint to remove.
	void GeneticAlgorithm::removeWaypoint(size_t n)
	{
		boost::lock_guard<boost::mutex> guard(_lock);
		for (auto& individual : _population)
		{
			individual.removeWaypoint(n);
		}
		findBestIndividual();

		return;
	}

	/// Pop a waypoint of the front of the solution, i.e. the next waypoint.
	///
	/// Essentially a convenience function instead of letting the client
	/// find, copy and remove the waypoint by himself.
	///
	/// @returns The next waypoint in the solution.
	const Waypoint GeneticAlgorithm::popWaypoint() throw(EmptySolution)
	{
		const Problem& best_individual = bestIndividual();
		auto solution = best_individual.solution();
		if (0 == solution.size())
		{
			throw(EmptySolution());
		}
		size_t current_goal_idx = solution[0];
		Waypoint wp(best_individual.waypoints()[current_goal_idx]);
    	removeWaypoint(current_goal_idx);
    	return wp;
	}

	void GeneticAlgorithm::setNumFixedWaypoints(unsigned int n)
	{
		_num_fixed_waypoints = n;
	}

	unsigned int GeneticAlgorithm::numFixedWaypoints()
	{
		return _num_fixed_waypoints;
	}

	/// Get the population of genomes that the algorithm is working on.
	///
	/// @returns The underlying Population.
	const std::vector<Genome>& GeneticAlgorithm::population() const
	{
		boost::lock_guard<boost::mutex> guard(_lock);
		return _population;
	}

	const Genome& GeneticAlgorithm::bestIndividual() const
	{
		boost::lock_guard<boost::mutex> guard(_lock);
		return _population[_best_individual_idx];
	}

	void GeneticAlgorithm::findBestIndividual()
	{
		double best_score = _population[_best_individual_idx].evaluate();
		size_t idx = 0;
		for (auto& individual : _population)
		{
			if (individual.evaluate() < best_score)
			{
				best_score = individual.evaluate();
				_best_individual_idx = idx;
			}
			++idx;
		}
	}
}
