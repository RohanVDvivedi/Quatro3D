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

extern const vector zero_vector;
extern const vector unit_vector_x_axis;
extern const vector unit_vector_y_axis;
extern const vector unit_vector_z_axis;

#endif