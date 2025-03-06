#include<quatro.h>

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
	// skip completely if B is zero vector
	if(is_zero_vector(B))
	{
		(*C) = zero_vector;
		return;
	}

	// get unit vector in direction of B
	vector unit_B = {};
	vector_unit_dir(&unit_B, B);

	// multiply magnitude and the direction, magnitude of this new vector is A.unitB
	vector_mul_scalar(C, &unit_B, vector_dot_prod(A, &unit_B));
}

void vector_perpendicular_component(vector* C, const vector* A, const vector* B)
{
	// skip completely if B is zero vector
	if(is_zero_vector(B))
	{
		(*C) = zero_vector;
		return;
	}

	// here, we get parallel componnet of A
	vector parallel_component = {};
	vector_parallel_component(&parallel_component, A, B);

	// C = A - component of A parallel to B = component of A perpendicular to B
	vector_sub(C, A, &parallel_component);
}