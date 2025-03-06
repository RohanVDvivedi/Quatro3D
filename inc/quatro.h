#ifndef QUATRO_H
#define QUATRO_H

#include<math.h>

#ifndef float_number
	#define float_number float
#endif

typedef struct vector vector;
struct vector
{
	float_number xi;
	float_number yj;
	float_number zk;
};

typedef struct quaternion quaternion;
struct quaternion
{
	// scalar component
	float_number sc;

	// i, j, and k component
	float_number xi;
	float_number yj;
	float_number zk;
};

#endif