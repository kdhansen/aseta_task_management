// Header file for the Quarternion structure of the GATSP project.
// 
// This file is part of the GATSP project.
// Copyright 2012-2013 Karl Damkjær Hansen - Aalborg University

#ifndef GATSP_QUARTERION_H
#define GATSP_QUARTERION_H

namespace gatsp
{
	struct Quarternion
	{
		Quarternion()
			: x(0), y(0), z(0), w(0)
		{
		}

		Quarternion(double x, double y, double z, double w)
			: x(x), y(y), z(z), w(w)
		{
		}
		
		double x;
		double y;
		double z;
		double w;
	};
}

#endif // GATSP_QUARTERION_H