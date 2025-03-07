#include<stdio.h>
#include<stdlib.h>

#include<quatro.h>

void print_vector(vector v)
{
	printf("(%f %f %f)", v.xi, v.yj, v.zk);
}

void print_quaternion(quaternion q)
{
	printf("(%f %f %f %f)", q.sc, q.xi, q.yj, q.zk);
}

void test_quaternion_compose_decompose(float angle, vector v)
{
	printf("TEST : quaternion compose decompose\n");

	print_vector(v); printf("is v unit %d\n", is_unit_vector(&v));

	vector axis;
	vector_unit_dir(&axis, &v);

	printf("is axis unit %d\n", is_unit_vector(&axis));

	print_vector(axis);printf(" is axis unit %d ", is_unit_vector(&axis));
	printf(", angle = %f\n", angle);

	quaternion q;
	compose_quaternion(&q, angle, &axis);

	print_quaternion(q);printf(" is unit = %d\n", is_unit_quaternion(&q));

	angle = 0;
	axis = (vector){};

	print_vector(axis);
	printf("%f\n", angle);

	angle = decompose_quaternion(&axis, &q);

	print_vector(axis);
	printf("%f\n", angle);

	printf("\n");
}

void test_vector_components(vector v, vector dir_for_v)
{
	printf("TEST : vector components\n");

	vector dir;
	vector_unit_dir(&dir, &dir_for_v);

	vector parallel;
	vector perpendicular;
	vector_perpendicular_component(&perpendicular, &parallel, &v, &dir);

	printf("vec : "); print_vector(v); printf(" is_unit = %d\n", is_unit_vector(&v));
	printf("dov : "); print_vector(dir_for_v); printf(" is_unit = %d\n", is_unit_vector(&dir_for_v));
	printf("dir : "); print_vector(dir); printf(" is_unit = %d\n", is_unit_vector(&dir));
	printf("||| : "); print_vector(parallel); printf(" is_unit = %d\n", is_unit_vector(&parallel));
	printf("_|_ : "); print_vector(perpendicular); printf(" is_unit = %d\n", is_unit_vector(&perpendicular));

	printf("\n");
}

void test_conjugate_and_reciprocal(quaternion q)
{
	printf("q : "); print_quaternion(q); printf("\n");

	quaternion q_c;
	quaternion_conjugate(&q_c, &q);
	printf("q_c : "); print_quaternion(q_c); printf("\n");

	quaternion q_r;
	quaternion_reciprocal(&q_r, &q);
	printf("q_r : "); print_quaternion(q_r); printf("\n");

	quaternion q_times_q_c;
	quaternion_hamilton_prod(&q_times_q_c, &q, &q_c);
	printf("q * q_c : "); print_quaternion(q_times_q_c); printf("\n");

	quaternion q_c_times_q;
	quaternion_hamilton_prod(&q_c_times_q, &q_c, &q);
	printf("q_c * q : "); print_quaternion(q_c_times_q); printf("\n");

	quaternion q_times_q_r;
	quaternion_hamilton_prod(&q_times_q_r, &q, &q_r);
	printf("q * q_r : "); print_quaternion(q_times_q_r); printf("\n");

	quaternion q_r_times_q;
	quaternion_hamilton_prod(&q_r_times_q, &q_r, &q);
	printf("q_r * q : "); print_quaternion(q_r_times_q); printf("\n");

	printf("\n");
}

int main()
{
	test_vector_components((vector){3.9, 1.9, -1.6}, (vector){2.0, 2.5, -2.6});
	test_vector_components((vector){-3.0, 2.5, 2.6}, unit_vector_y_axis);
	test_vector_components((vector){-3.0, 2.5, 2.6}, (vector){9.3, 1.3, -7.3});

	test_quaternion_compose_decompose(-2.5, (vector){3.0, 1.5, -7.6});
	test_quaternion_compose_decompose(-1.5, (vector){2.0, -2.5, 3.6});

	test_conjugate_and_reciprocal((quaternion){-1.9, -3.11, 5.13, 7.15});
	test_conjugate_and_reciprocal((quaternion){1.9, -3.11, 5.13, -7.15});

	return 0;
}