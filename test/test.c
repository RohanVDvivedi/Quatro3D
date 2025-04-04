#include<stdio.h>
#include<stdlib.h>

#include<quatro3d/quatro.h>

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

void test_hamiltonian_product_rules(quaternion A, quaternion B, quaternion C)
{
	quaternion A_times_B;
	quaternion_hamilton_prod(&A_times_B, &A, &B);

	quaternion C_1;
	quaternion_reciprocal(&C_1, &C);

	quaternion D;
	quaternion_hamilton_prod(&D, &C_1, &A_times_B);

	quaternion D_1;
	quaternion_reciprocal(&D_1, &D);

	quaternion C_times_D;
	quaternion_hamilton_prod(&C_times_D, &C, &D);

	printf("A : "); print_quaternion(A); printf("\n");
	printf("B : "); print_quaternion(B); printf("\n");
	printf("C : "); print_quaternion(C); printf("\n");
	printf("D : "); print_quaternion(D); printf("\n");
	printf("D = (C^-1) * A * B, then below 2 must be equal\n");
	printf("A * B : "); print_quaternion(A_times_B); printf("\n");
	printf("C * D : "); print_quaternion(C_times_D); printf("\n");

	printf("also then A * B * (D^-1) = C, lhs is given below\n");
	quaternion lhs;
	quaternion_hamilton_prod(&lhs, &A_times_B, &D_1);
	printf("lhs from above = "); print_quaternion(lhs); printf("\n");

	printf("\n");
}

void test_rotations(vector axis, float angle, vector vi)
{
	printf("vi : "); print_vector(vi); printf(" mag = %f \n", vector_magnitude(&vi));

	// generate quaternion
	make_unit_vector(&axis);
	printf("axis : "); print_vector(axis); printf(" angle = %f\n", angle);
	quaternion q;
	compose_quaternion(&q, angle, &axis);
	printf("q : "); print_quaternion(q); printf("\n");

	vector vf;
	rotate_by_quaternion(&vf, &q, &vi);
	printf("vf : "); print_vector(vf); printf(" mag = %f \n", vector_magnitude(&vf));

	// make vi and vf unit vectors
	make_unit_vector(&vi);
	make_unit_vector(&vf);

	printf("calculate angle from axis and vi and vf => %f\n", angle_between_2_vectors(&axis, &vi, &vf));

	printf("\n");
}

void test_2_vector_rotation_diffs(vector A, vector B, vector axis, float angle)
{
	make_unit_vector(&axis);
	vector Ai = A;
	make_unit_vector(&Ai);
	vector Bi = B;
	make_unit_vector(&Bi);

	printf("Ai : "); print_vector(Ai); printf(" is unit %d\n", is_unit_vector(&Ai));
	printf("Bi : "); print_vector(Bi); printf(" is unit %d\n", is_unit_vector(&Bi));
	printf("axis : "); print_vector(axis); printf(" angle = %f\n", angle);

	// construct actual quaternion
	quaternion q;
	compose_quaternion(&q, angle, &axis);
	printf("q : "); print_quaternion(q); printf("\n");

	// perform rotations
	vector Af;
	rotate_by_quaternion(&Af, &q, &Ai);
	vector Bf;
	rotate_by_quaternion(&Bf, &q, &Bi);

	// print outputs after rotation
	printf("Af : "); print_vector(Af); printf(" is unit %d\n", is_unit_vector(&Af));
	printf("Bf : "); print_vector(Bf); printf(" is unit %d\n", is_unit_vector(&Bf));

	vector axis_calculated;
	axis_of_rotation_for_2_vectors(&axis_calculated, &Ai, &Af, &Bi, &Bf);
	printf("axis_calculated : "); print_vector(axis_calculated); printf("\n");

	printf("angle of rotation from A = %f\n", angle_between_2_vectors(&axis_calculated, &Ai, &Af));
	printf("angle of rotation from B = %f\n", angle_between_2_vectors(&axis_calculated, &Bi, &Bf));

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

	test_hamiltonian_product_rules((quaternion){1.9, -3.11, 5.13, -7.15}, (quaternion){13, 11, 9, -7}, (quaternion){1, -15, 5, -35});

	test_rotations((vector){1.0, 2.0, 3.1}, 1.5, (vector){1.0, 0.4, 3.9});
	test_rotations((vector){2.0, 2.8, 3.2}, -1.5, (vector){1.0, 0.4, 3.9});
	test_rotations((vector){1.0, 2.8, 3.3}, 0.5, (vector){1.0, 0.4, 3.9});
	test_rotations((vector){2.0, 2.06, 3.4}, -0.5, (vector){1.0, 0.4, 3.9});
	test_rotations((vector){3.0, 2.02, 3.3}, 2.5, (vector){1.0, 0.4, 3.9});
	test_rotations((vector){2.0, 2.01, 3.2}, -2.5, (vector){1.0, 0.4, 3.9});

	printf("-------------------NOW WORKING IN ONLY X-Y PLANE---------------------------\n\n");
	for(float f = -M_PI; f <= M_PI + 0.1; f += M_PI/6)
		test_rotations((vector){0, 0, 1}, f, (vector){1, 0, 0});

	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){5,-2,3}, 2.5);

	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){5,-2,3}, -1.5);

	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){-5,-1,3}, 1.5);

	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){-5,6,-3}, -2.9);

	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){-4.3,-6.2,-3.9}, -3.1);

	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){4.3,-6.2,-3.9}, +3.1);

	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){1,0,0}, 0.5);
	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){0,1,0}, 0.5);
	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){0,0,1}, 0.5);

	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){1,1,0}, 0.5);
	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){0,1,1}, 0.5);
	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){1,0,1}, 0.5);

	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){2,0,0}, -1.5);
	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){0,3,0}, -1.5);
	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){0,0,4}, -1.5);

	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){2,3,0}, -1.5);
	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){0,3,2}, -1.5);
	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){4,0,5}, -1.5);

	// tests when there is no rotation
	// this will also give a nan result
	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){2,1,4}, 0);
	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){1,3,4}, 0);
	test_2_vector_rotation_diffs((vector){3,7,1}, (vector){1,2,3}, (vector){4,1,4}, 0);

	// test for when axis is parallel to one of the vectors, OR when both the vector are parallel
	// this will give an errored result or a nan
	test_2_vector_rotation_diffs((vector){6,-7,1}, (vector){1,2,3}, (vector){6,-7,1}, -2.0);
	test_2_vector_rotation_diffs((vector){6,-7,1}, (vector){3,-3.5,0.5}, (vector){6,7,1}, -2.0);

	return 0;
}