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

int main()
{
	test_quaternion_compose_decompose(-2.5, (vector){3.0, 1.5, -7.6});
	return 0;
}