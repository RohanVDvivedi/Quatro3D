#include<stdio.h>
#include<stdlib.h>

#include<quatro.h>

void print_vector(vector v)
{
	printf("(%f %f %f)", v.xi, v.yj, v.zk);
}

int main()
{
	vector v1 = {1,0,0};
	vector v2 = {0,-1,0};
	print_vector(v1);
	print_vector(v2);
	printf("->%f\n", angle_between_vectors(&v1, &v2) * 180 / M_PI);


	return 0;
}