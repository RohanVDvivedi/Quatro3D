#ifndef QUATRO_H
#define QUATRO_H

#include<math.h>

#ifndef float_number
	#define float_number float
#endif

// --------------------------------------------------------------------------------

// NOTE :: parameter names containing *unit* in their names must be unit vectors

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

// chec if A == B
int are_equal_vectors(const vector* A, const vector* B);

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

// convert vector to unit vector in place
// returns magnitude of the vector
float_number make_unit_vector(vector* A);

// C = component of A parallel to unit_dir
// unit_dir must be a unit vector
void vector_parallel_component(vector* C, const vector* A, const vector* unit_dir);

// C = component of A perpendicular to B
// unit_dir must be a unit vector
// a side effect is to also poduce the parallel_component, this can be NULL if you don't need it
void vector_perpendicular_component(vector* C, vector* parallel_component, const vector* A, const vector* unit_dir);

// figures out the angle of rotation about unit_axis, require to move unit vector from Ai to Af
// if a solution could not be found then NAN is returned
float_number angle_between_2_vectors(const vector* unit_axis, const vector* unit_Ai, const vector* unit_Af);

// a single change in vector from Ai to Af can happen due to rotation about infinite number of axis-s
// but if you have 2 such changes from 2 non collinear vectors then we can find the unit axis in which the plane was rotated
// this function could error and may return nan, if any of the A or B vectors are parallel to either each other (which is not allowed and must be checked) OR are paralle with the actual axis of rotation (which can not be checked unless the function results correctly)
// this function also returns a nan if the absolute orientation says that there is no rotation what so ever
void axis_of_rotation_for_2_vectors(vector* unit_axis, const vector* unit_Ai, const vector* unit_Af, const vector* unit_Bi, const vector* unit_Bf);

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
void quaternion_hamilton_prod(quaternion* C, const quaternion* A, const quaternion* B);

// get result vector after rotating Ai by Quaternion Q
// same as Q x Ai x (Q^-1)
void rotate_by_quaternion(vector* Af, const quaternion* Q, const vector* Ai);

#endif

/*
	Some tips for your next quadcopter project

	Gryroscope data
		We will be integrating this data so lets assume the current quaternion rotation w.r.t global axis is Q
		let gyroscope data be Wrel = vector(Wx, Wy, Wx) (Note:: which is angular rotation w.r.t. your local axis)
		first we need to get Wabs which is rate of angular rotation w.r.t. global axis
		Wabs = rotate_by_quaternion(reciprocal(Q), Wrel);
		Now to get a change in quaternion by this Wabs we can do
		q_change = compose_quaternion(|Wabs|, unit_axis_in_direction_of(Wabs));
		then your final Qnew = quaternion_hamilton_prod(q_change, Q);
		Voila and you are done.
		if you expand this equation, you will realize that it is very conducive to Kalman Filtering (which is system being of the form => Xk+1 = A * Xk, all of these are martices).

	Accelerometer data alone can not give you quaternion
		let's talk about absolute_roll (Euler angles are shit and outdated), this quantity is the absolute rotation needed about +X axis to make Y axis parallel to horizon
		Similarly define absolute_pitch as, the absolute rotation needed about +Y axis to make X axis parallel to horizon
		Now these 2 quantities are not interrelated so they are not Euler angles in any sense
		let A_initial be unit vector in the initial Acceleration, and A_current be the unit vector in current acceleration
		absolute_pitch = angle_between_2_vectors([0,1,0], A_current, A_initial); -> the parameters are reversed, because here local axis has rotated but not the acceleration vector itself

	Magnetometer
		Now getting a quaternion from only magnetometer is not possible, but if you accelerometer with it then the things change
		get the initial and final direction of these vector (get unit vecotrs in their directions), Ai, Af, and Mi and Mf
		Now the axis of rotation for the quaternion about the global axis is
		unit_axis = axis_of_rotation_for_2_vectors(Af, Ai, Mf, Mi); -> the parameters are reversed, because here local axis has rotated but not the acceleration/magnetometer vector itself
		angle = angle_between_2_vectors(unit_axis, Af, Ai); OR like wise using magnetometer readings -> again the note the reverse parameters
		now you can compute the quaternion using the unit_axis and the angle
		NOTE : here this logic can not be used if for some reason (like closer to poles), when both the sensors are pointing in collinear direction OR when the direction of atleast one of the vectors change with time or position
*/