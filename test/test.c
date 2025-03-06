#include<stdio.h>
#include<stdlib.h>

#include<quatro.h>

void print_vector(vector v)
{
	printf("(%f %f %f)", v.xi, v.yj, v.zk);
}

void test_quaternion_compose_decompose(float angle, vector v)
{
	printf("TEST : quaternion compose decompose\n");

	vector axis;
	vector_unit_dir(&axis, &v);

	print_vector(axis);
	printf("%f\n", angle);

	quaternion q;
	compose_quaternion(&q, angle, &axis);

	angle = 0;
	axis = (vector){};

	print_vector(axis);
	printf("%f\n", angle);

	angle = decompose_quaternion(&axis, &q);

	print_vector(axis);
	printf("%f\n", angle);

	printf("\n");
}

void test_vector_components(vector v, vector dir)
{
	printf("TEST : vector components\n");
	vector parallel;
	vector_parallel_component(&parallel, &v, &dir);

	vector perpendicular;
	vector_perpendicular_component(&perpendicular, &v, &dir);

	printf("vec : "); print_vector(v); printf("\n");
	printf("dir : "); print_vector(dir); printf("\n");
	printf("||| : "); print_vector(parallel); printf("\n");
	printf("_|_ : "); print_vector(perpendicular); printf("\n");

	printf("\n");
}

int main()
{
	test_vector_components((vector){3.9, 1.9, -1.6}, (vector){2.0, 2.5, -2.6});
	test_vector_components((vector){-3.0, 2.5, 2.6}, unit_vector_y_axis);
	test_vector_components((vector){-3.0, 2.5, 2.6}, (vector){9.3, 1.3, -7.3});
	test_quaternion_compose_decompose(-2.5, (vector){3.0, 1.5, -7.6});
	return 0;
}