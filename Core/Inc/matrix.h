#ifndef MATRIX_H
#define MATRIX_H


void matrix_mult_3x4_4x1(float A[3][4], float B[4], float C[3]);

void matrix_transpose_3x4( float A[3][4], float At[4][3]);

void matrix_mult_3x4_4x3( float A[3][4],  float B[4][3], float C[3][3]);

bool matrix_invert_3x3( float A[3][3], float A_inv[3][3]);
												
void matrix_mult_4x3_3x3( float A[4][3],  float B[3][3], float C[4][3]);

void matrix_mult_4x3_3x1( float A[4][3],  float v[3], float result[4]);

void matrix_mult_3x3_3x1( float A[3][3], float v[3], float result[3]);
												
void this_is_a_link_test(void);
#endif 