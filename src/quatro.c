#include<quatro3d/quatro.h>

#ifndef M_PI
	#define M_PI ((float_number)(3.141592653589793238L))
#endif

#define UNIT_VALUE ((float_number)(1.0L))
#define EQUALITY_TOLERANCE ((float_number)(0.00001L))
#define are_float_numbers_equal(a, b) ((((a) - EQUALITY_TOLERANCE) <= (b)) && ((b) <= ((a) + EQUALITY_TOLERANCE)))

// below 7 macros are chat-gpt generated for language agnostic version of the trigonometric functions

#define sine(x)  _Generic((x), float: sinf, double: sin, long double: sinl)(x)
#define cosine(x) _Generic((x), float: cosf, double: cos, long double: cosl)(x)
#define tangent(x) _Generic((x), float: tanf, double: tan, long double: tanl)(x)

#define arcsine(x)  _Generic((x), float: asinf, double: asin, long double: asinl)(x)
#define arccosine(x) _Generic((x), float: acosf, double: acos, long double: acosl)(x)
#define arctangent(x) _Generic((x), float: atanf, double: atan, long double: atanl)(x)

#define sqroot(x) _Generic((x), float: sqrtf, double: sqrt, long double: sqrtl)(x)

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
	return are_float_numbers_equal(UNIT_VALUE, magnitude);
}

int are_equal_vectors(const vector* A, const vector* B)
{
	return are_float_numbers_equal(A->xi, B->xi) && are_float_numbers_equal(A->yj, B->yj) && are_float_numbers_equal(A->zk, B->zk);
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
	return sqroot(vector_magnitude_squared(A));
}

float_number vector_unit_dir(vector* unit_A, const vector* A)
{
	float_number magnitude = vector_magnitude(A);
	vector_mul_scalar(unit_A, A, UNIT_VALUE / magnitude);
	return magnitude;
}

float_number make_unit_vector(vector* A)
{
	float_number magnitude = vector_magnitude(A);
	// below line of code represents an element by element division so it is just safe to do it, kindly avoid this in user space code
	vector_mul_scalar(A, A, UNIT_VALUE / magnitude);
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

float_number angle_between_2_vectors(const vector* unit_axis, const vector* unit_Ai, const vector* unit_Af)
{
	// find perpendicular and parallel components in unit_axis direction -> subscript p for paralle and pp for perpendicular
	vector unit_Ai_p;
	vector unit_Ai_pp;
	vector_perpendicular_component(&unit_Ai_pp, &unit_Ai_p, unit_Ai, unit_axis);
	vector unit_Af_p;
	vector unit_Af_pp;
	vector_perpendicular_component(&unit_Af_pp, &unit_Af_p, unit_Af, unit_axis);

	// vector remains unchanges in parallel to the unit_axis direction, so magnitudes must match
	{
		float_number aip_mag = vector_magnitude(&unit_Ai_p);
		float_number afp_mag = vector_magnitude(&unit_Af_p);
		if(!are_float_numbers_equal(aip_mag, afp_mag))
			return NAN;
	}

	// now just forget about the parallel components
	// all we need to do is find angle between perpedicular components

	float_number A_dot = vector_dot_prod(&unit_Ai_pp, &unit_Af_pp);
	// convert dot product into cosine of the angle
	A_dot /= vector_magnitude_squared(&unit_Ai_pp);
	vector A_cross;
	vector_cross_prod(&A_cross, &unit_Ai_pp, &unit_Af_pp);
	// make A_cross unit vector
	make_unit_vector(&A_cross);

	float_number angle = arccosine(A_dot);
	if(!are_equal_vectors(&A_cross, unit_axis))
		angle = -angle;

	return angle;
}

void axis_of_rotation_for_2_vectors(vector* unit_axis, const vector* unit_Ai, const vector* unit_Af, const vector* unit_Bi, const vector* unit_Bf)
{
	// find out change of both A and B
	vector A_diff;
	vector_sub(&A_diff, unit_Af, unit_Ai);
	vector B_diff;
	vector_sub(&B_diff, unit_Bf, unit_Bi);

	// now we know that, Ai dot unit_axis = Af dit unit_axis -> components of A in the direction of unit_axis will not change due to rotation
	// so this gives A_diff dot unit_axis = 0 and B_diff dot unit_axis = 0
	float_number Kx = (A_diff.yj * B_diff.zk) - (B_diff.yj * A_diff.zk); // unit_axis->xi = Kx * K
	float_number Ky = (A_diff.zk * B_diff.xi) - (B_diff.zk * A_diff.xi); // unit_axis->yj = Ky * K
	float_number Kz = (A_diff.xi * B_diff.yj) - (B_diff.xi * A_diff.yj); // unit_axis->zk = Kz * K

	// we also have information that unit_aixs has magnitude of 1
	float_number K = UNIT_VALUE / sqroot(Kx * Kx + Ky * Ky + Kz * Kz);
	unit_axis->xi = Kx * K;
	unit_axis->yj = Ky * K;
	unit_axis->zk = Kz * K;
}

const quaternion identity_quaternion = {.sc = 1.0, .xi = 0.0, .yj = 0.0, .zk = 0.0};

void compose_quaternion(quaternion* Q, float angle, const vector* unit_axis)
{
	float_number sine_by_2 = sine(angle / 2);
	float_number cosine_by_2 = cosine(angle / 2);

	Q->sc = cosine_by_2;
	Q->xi = sine_by_2 * unit_axis->xi;
	Q->yj = sine_by_2 * unit_axis->yj;
	Q->zk = sine_by_2 * unit_axis->zk;
}

float_number decompose_quaternion(vector* axis, const quaternion* Q)
{
	// get axis
	axis->xi = Q->xi;
	axis->yj = Q->yj;
	axis->zk = Q->zk;

	// convert axis to unit vector
	if(is_zero_vector(axis)) // an identity_quaternion will have axis as a zero vector, so give it a random unit vector
		(*axis) = unit_vector_x_axis;
	else
		make_unit_vector(axis);

	// return the angle
	return 2 * arccosine(Q->sc);
}

float_number quaternion_magnitude_squared(const quaternion* Q)
{
	return (Q->sc * Q->sc) + (Q->xi * Q->xi) + (Q->yj * Q->yj) + (Q->zk * Q->zk);
}

float_number quaternion_magnitude(const quaternion* Q)
{
	return sqroot(quaternion_magnitude_squared(Q));
}

int is_unit_quaternion(const quaternion* Q)
{
	float_number magnitude = quaternion_magnitude(Q);
	return are_float_numbers_equal(UNIT_VALUE, magnitude);
}

void quaternion_conjugate(quaternion* res, const quaternion* Q)
{
	res->sc = +Q->sc;
	res->xi = -Q->xi;
	res->yj = -Q->yj;
	res->zk = -Q->zk;
}

void quaternion_reciprocal(quaternion* res, const quaternion* Q)
{
	// calculate magnitude
	float_number magnitude_squared = quaternion_magnitude_squared(Q);

	res->sc = +Q->sc / magnitude_squared;
	res->xi = -Q->xi / magnitude_squared;
	res->yj = -Q->yj / magnitude_squared;
	res->zk = -Q->zk / magnitude_squared;
}

void quaternion_hamilton_prod(quaternion* C, const quaternion* A, const quaternion* B)
{
	C->sc = (A->sc * B->sc) - (A->xi * B->xi) - (A->yj * B->yj) - (A->zk * B->zk);
	C->xi = (A->sc * B->xi) + (A->xi * B->sc) + (A->yj * B->zk) - (A->zk * B->yj);
	C->yj = (A->sc * B->yj) - (A->xi * B->zk) + (A->yj * B->sc) + (A->zk * B->xi);
	C->zk = (A->sc * B->zk) + (A->xi * B->yj) - (A->yj * B->xi) + (A->zk * B->sc);
}

void rotate_by_quaternion(vector* Af, const quaternion* Q, const vector* Ai)
{
	//quaternion Q_reciprocal;
	//quaternion_reciprocal(Q_reciprocal, Q);
	// since we know that the Q is a unit quaternion, we can use the conjuagte instead
	quaternion Q_conjugate;
	quaternion_conjugate(&Q_conjugate, Q);

	quaternion AI = {.sc = 0, .xi = Ai->xi, .yj = Ai->yj, .zk = Ai->zk};
	quaternion AF;

	// Q * Ai * (Q^-1)
	{
		quaternion t1;
		quaternion_hamilton_prod(&t1, Q, &AI);
		quaternion_hamilton_prod(&AF, &t1, &Q_conjugate);
	}

	(*Af) = (vector){.xi = AF.xi, .yj = AF.yj, .zk = AF.zk};
}