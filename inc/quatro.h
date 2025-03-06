#ifndef QUATRO_H
#define QUATRO_H

#include<math.h>

#ifndef float_number
	#define float_number float
#endif

// --------------------------------------------------------------------------------

// VECTOR 3D

typedef struct vector vector;
struct vector
{
	float_number xi;
	float_number yj;
	float_number zk;
};

extern const vector zero_vector;
extern const vector unit_vector_x_axis;
extern const vector unit_vector_y_axis;
extern const vector unit_vector_z_axis;

// if al the 3 components of vector are 0
int is_zero_vector(const vector* A);

// C = A + B
void vector_sum(vector* C, const vector* A, const vector* B);

// C = A - B
void vector_sub(vector* C, const vector* A, const vector* B);

// c = A * sc
void vector_mul_scalar(vector* C, const vector* A, float_number sc);

// C = A X B
void vector_cross_prod(vector* C, const vector* A, const vector* B);

// return value A.B
float_number vector_dot_prod(const vector* A, const vector* B);

// returns value A.A
float_number vector_magnitude_squared(const vector* A);

// returns value sqrt(A.A)
float_number vector_magnitude(const vector* A);

// get a unit_vector in direction of A in unitResult
// returns magnitude in return value, unitResult is not correct if magnitude returned is 0
float_number vector_unit_dir(vector* unitResult, const vector* A);

// C = component of A parallel to B
// does not work with B as zero_vector, as then B would not have a direction
void vector_parallel_component(vector* C, const vector* A, const vector* B);

// C = component of A perpendicular to B
// does not work with B as zero_vector, as then B would not have a direction
void vector_perpendicular_component(vector* C, const vector* A, const vector* B);

// --------------------------------------------------------------------------------

// QUATERNION 3D (for rotation)

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