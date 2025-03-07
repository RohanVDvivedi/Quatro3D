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

// if all the 3 components of vector are 0
int is_zero_vector(const vector* A);

// |A| == 1.0
int is_unit_vector(const vector* A);

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
float_number vector_unit_dir(vector* unit_A, const vector* A);

// C = component of A parallel to unit_dir
// unit_dir must be a unit vector
void vector_parallel_component(vector* C, const vector* A, const vector* unit_dir);

// C = component of A perpendicular to B
// unit_dir must be a unit vector
// a side effect is to also poduce the parallel_component, this can be NULL if you don't need it
void vector_perpendicular_component(vector* C, vector* parallel_component, const vector* A, const vector* unit_dir);

// figures out the angle of rotation about unit_axis, require to move unit vector from Ai to Af
// if a solution could not be found then NAN is returned
float_number angle_between_2_vectors(const vector& unit_axis, const vector* unit_Ai, const vector* unit_Af);

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

// this is identity quaternion it represents no rotation
extern const quaternion identity_quaternion;

// compose a quaternion, representing an angle of rotation about the given unit_axis (which must be a unit_vector)
void compose_quaternion(quaternion* Q, float angle, const vector* unit_axis);

// decomposes a auaternion into its angle (return value) and the axis of rotation
float_number decompose_quaternion(vector* axis, const quaternion* Q);

// sum of squares of components of the quaternion
float_number quaternion_magnitude_squared(const quaternion* Q);

// sqrt(output of above function)
float_number quaternion_magnitude(const quaternion* Q);

// quaternion_magnitude(A) == 1.0
int is_unit_quaternion(const quaternion* Q);

// at the end of this function res is same as Q, but with the xi, yj and zk with opposite sign
// res = [Q->sc, -Q->xi, -Q->yj, -Q->zk]
void quaternion_conjugate(quaternion* res, const quaternion* Q);

// this is reciprocal of Q, hamiltonian product of quaternion and its reciprocal is identity_quaternion
// res = quaternion_conjugate / quaternion_magnitude
// for a unit_quaternion, reciprocal and conjugate are the same thing
// in rotation terms they both represent the same rotation but in opposite direction
void quaternion_reciprocal(quaternion* res, const quaternion* Q);

// hamiltonian product of 2 quaternions A and B to give C
// C = A x B
// also A x B = C * D, then (C^-1) x A x B = D and A x B x (D^-1) = C
void quaternion_hamilton_prod(quaternion* C, quaternion* A, quaternion* B);

#endif