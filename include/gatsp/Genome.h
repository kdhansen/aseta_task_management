// Header file for the Genome class of the GATSP project.
// 
// This file is part of the GATSP project.
// Copyright 2013 Karl Damkjær Hansen, Aalborg University

#ifndef GATSP_GENOME_H
#define GATSP_GENOME_H

#include <iostream>
#include <random>
#include "gatsp/Problem.h"

namespace gatsp
{
	class Genome : public Problem
	{
	public:
	 	explicit Genome(std::mt19937_64&);
		Genome(const Problem&, std::mt19937_64&);
		virtual ~Genome();

		Genome& operator=(const Genome&);
		
		void randomize();
		void mutate();
		double evaluate();

		virtual void addWaypoint(const Waypoint&);
        virtual void removeWaypoint(size_t);

	private:
		std::mt19937_64& _generator;
		bool _evaluated;
		double _score;

		void displacementMutation();
        void exchangeMutation();
        void inversionMutation();
	};
};

#endif
