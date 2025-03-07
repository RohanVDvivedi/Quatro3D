#include<quatro.h>

#ifndef M_PI
	#define M_PI ((float_number)(3.141592653589793238L))
#endif

#define UNIT_VALUE ((float_number)(1.0L))
#define EQUALITY_TOLERANCE ((float_number)(0.00001L))

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

int is_unit_vector(const vector* A)
{
	float_number magnitude = vector_magnitude(A);
	return ((UNIT_VALUE - EQUALITY_TOLERANCE) <= magnitude) && (magnitude <= (UNIT_VALUE + EQUALITY_TOLERANCE));
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

float_number vector_unit_dir(vector* unit_A, const vector* A)
{
	float_number magnitude = vector_magnitude(A);
	vector_mul_scalar(unit_A, A, UNIT_VALUE / magnitude);
	return magnitude;
}

void vector_parallel_component(vector* C, const vector* A, const vector* unit_dir)
{
	// multiply magnitude and the direction, magnitude of this new vector is A.unitB
	vector_mul_scalar(C, unit_dir, vector_dot_prod(A, unit_dir));
}

void vector_perpendicular_component(vector* C, vector* parallel_component, const vector* A, const vector* unit_dir)
{
	// allocate stack memory for parallel_component if the di not provide this to us
	vector parallel_component_temp;
	if(parallel_component == (void*)(0))
		parallel_component = &parallel_component_temp;

	// here, we get parallel component of A
	vector_parallel_component(parallel_component, A, unit_dir);

	// C = A - component of A parallel to unit_dir = component of A perpendicular to unit_dir
	vector_sub(C, A, parallel_component);
}

void compose_quaternion(quaternion* Q, float angle, const vector* axis)
{
	float_number sine_by_2 = sine(angle / 2);
	float_number cosine_by_2 = cosine(angle / 2);

	vector unit_axis;
	vector_unit_dir(&unit_axis, axis);

	Q->sc = cosine_by_2;
	Q->xi = sine_by_2 * unit_axis.xi;
	Q->yj = sine_by_2 * unit_axis.yj;
	Q->zk = sine_by_2 * unit_axis.zk;
}

float_number decompose_quaternion(vector* axis, const quaternion* Q)
{
	// get axis
	axis->xi = Q->xi;
	axis->yj = Q->yj;
	axis->zk = Q->zk;

	// convert axis to unit vector
	{
		vector temp = (*axis);
		vector_unit_dir(axis, &temp);
	}

	// return the angle
	return 2 * arccosine(Q->sc);
}