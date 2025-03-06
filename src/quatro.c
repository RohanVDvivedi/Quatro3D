#include<quatro.h>

#ifndef M_PI
	#define M_PI ((float_number)(3.141592653589793238L))
#endif

// below 6 macros are chat-gpt generated for language agnostic version of the trigonometric functions

#define sine(x)  _Generic((x), float: sinf, double: sin, long double: sinl)(x)
#define cosine(x) _Generic((x), float: cosf, double: cos, long double: cosl)(x)
#define tangent(x) _Generic((x), float: tanf, double: tan, long double: tanl)(x)

#define arcsine(x)  _Generic((x), float: asinf, double: asin, long double: asinl)(x)
#define arccosine(x) _Generic((x), float: acosf, double: acos, long double: acosl)(x)
#define arctangent(x) _Generic((x), float: atanf, double: atan, long double: atanl)(x)

const vector zero_vector = {.xi = 0.0, .yj = 0.0, .zk = 0.0};
const vector unit_vector_x_axis = {.xi = 1.0, .yj = 0.0, .zk = 0.0};
const vector unit_vector_y_axis = {.xi = 0.0, .yj = 1.0, .zk = 0.0};
const vector unit_vector_z_axis = {.xi = 0.0, .yj = 0.0, .zk = 1.0};

int is_zero_vector(const vector* A)
{
	return (A->xi == 0.0) && (A->yj == 0.0) && (A->zk == 0.0);
}

void vector_sum(vector* C, const vector* A, const vector* B)
{
	C->xi = A->xi + B->xi;
	C->yj = A->yj + B->yj;
	C->zk = A->zk + B->zk;
}

void vector_sub(vector* C, const vector* A, const vector* B)
{
	C->xi = A->xi - B->xi;
	C->yj = A->yj - B->yj;
	C->zk = A->zk - B->zk;
}

void vector_mul_scalar(vector* C, const vector* A, float_number sc)
{
	C->xi = A->xi * sc;
	C->yj = A->yj * sc;
	C->zk = A->zk * sc;
}

void vector_cross_prod(vector* C, const vector* A, const vector* B)
{
	C->xi = (A->yj * B->zk) - (A->zk * B->yj);
	C->yj = (A->zk * B->xi) - (A->xi * B->zk);
	C->zk = (A->xi * B->yj) - (A->yj * B->xi);
}

float_number vector_dot_prod(const vector* A, const vector* B)
{
	return (A->xi * B->xi) + (A->yj * B->yj) + (A->zk * B->zk);
}

float_number vector_magnitude_squared(const vector* A)
{
	return vector_dot_prod(A, A);
}

float_number vector_magnitude(const vector* A)
{
	return sqrt(vector_magnitude_squared(A));
}

float_number vector_unit_dir(vector* unitResult, const vector* A)
{
	float_number magnitude = vector_magnitude(A);
	if(magnitude != 0.0)
		vector_mul_scalar(unitResult, A, 1.0 / magnitude);
	return magnitude;
}

void vector_parallel_component(vector* C, const vector* A, const vector* B)
{
	// get unit vector in direction of B
	vector unit_B = {};
	vector_unit_dir(&unit_B, B);

	// multiply magnitude and the direction, magnitude of this new vector is A.unitB
	vector_mul_scalar(C, &unit_B, vector_dot_prod(A, &unit_B));
}

void vector_perpendicular_component(vector* C, const vector* A, const vector* B)
{
	// here, we get parallel componnet of A
	vector parallel_component = {};
	vector_parallel_component(&parallel_component, A, B);

	// C = A - component of A parallel to B = component of A perpendicular to B
	vector_sub(C, A, &parallel_component);
}

float_number angle_between_vectors(const vector* A, const vector* B)
{
	// get unit vectos in both the directions
	vector unit_A;
	vector_unit_dir(&unitA, A);

	vector unit_B;
	vector_unit_dir(&unitB, B);

	// find out angle sines and cosines
	vector A_cross_B = {};
	vector_cross_prod(&A_cross_B, unit_A, unit_B);
	float_number angle_sine = vector_magnitude(&A_cross_B);

	float_number angle_cosine = vector_dot_prod(unit_A, unit_B);

	// find the actual angle using the angle_sine and angle_cosine
	float_number angle = arcsine(angle_sine);
	if(angle_cosine < 0)
	{
		if(angle_sine > 0)
			angle = M_PI - angle;
		else if(angle_sine < 0)
			angle = -M_PI - angle;
	}

	return angle;
}