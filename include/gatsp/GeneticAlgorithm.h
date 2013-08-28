// Header file for the GeneticAlgorithm class of the gatsp project.
//
// This file is part of the GATSP project.
// Copyright 2013 Karl Damkjær Hansen, Aalborg University

#ifndef GATSP_GENETIC_ALGORITHM_H
#define GATSP_GENETIC_ALGORITHM_H

#include <functional>
#include <random>
#include <vector>
#include <boost/thread.hpp>
#include "gatsp/Problem.h"
#include "gatsp/Genome.h"

namespace gatsp
{
    class GeneticAlgorithm
    {
    public:
        GeneticAlgorithm();
        GeneticAlgorithm(const Problem&, size_t = 50, unsigned int = 0);
        virtual ~GeneticAlgorithm();

        void step();
        void evolve();

        unsigned int generation() const;

        bool done();
        void setTerminator(std::function<bool(GeneticAlgorithm&)>);
        bool terminateGenerations();
        bool terminateAdaptive();
        bool terminateInfinite();
        void maxGenerations(unsigned int);

        void addWaypoint(const Waypoint&);
        void removeWaypoint(size_t);

        const std::vector<Genome>& population() const;
        const Genome& bestIndividual() const;

    private:
        void findBestIndividual();

        mutable boost::mutex _lock;

    	unsigned int _max_generations;
    	unsigned int _num_generations;
        std::mt19937_64 _generator;
    	std::function<bool(GeneticAlgorithm&)> _terminator;
        std::vector<Genome> _population;
        size_t _best_individual_idx;
    };
}

#endif // GATSP_GENETIC_ALGORITHM_H
