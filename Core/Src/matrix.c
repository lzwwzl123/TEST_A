#include "matrix.h"


// =============================================================================
// 【【【 新增的静态矩阵运算辅助函数 】】】
// =============================================================================



void matrix_mult_3x4_4x1(float A[3][4], float B[4], float C[3]) {
    for (int i = 0; i < 3; i++) {
        C[i] = 0.0f;
        for (int j = 0; j < 4; j++) {
            C[i] += A[i][j] * B[j];
        }
    }
}


void matrix_transpose_3x4( float A[3][4], float At[4][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            At[j][i] = A[i][j];
        }
    }
}

void matrix_mult_3x4_4x3( float A[3][4],  float B[4][3], float C[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

bool matrix_invert_3x3( float A[3][3], float A_inv[3][3]) {
    float det = A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) -
                A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    if (fabsf(det) < 1e-10f) {
        return false; // 矩阵奇异
    }

    float inv_det = 1.0f / det;
    A_inv[0][0] = (A[1][1] * A[2][2] - A[2][1] * A[1][2]) * inv_det;
    A_inv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;
    A_inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;
    A_inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * inv_det;
    A_inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
    A_inv[1][2] = (A[1][0] * A[0][2] - A[0][0] * A[1][2]) * inv_det;
    A_inv[2][0] = (A[1][0] * A[2][1] - A[2][0] * A[1][1]) * inv_det;
    A_inv[2][1] = (A[2][0] * A[0][1] - A[0][0] * A[2][1]) * inv_det;
    A_inv[2][2] = (A[0][0] * A[1][1] - A[1][0] * A[0][1]) * inv_det;
    return true;
}

void matrix_mult_4x3_3x3( float A[4][3],  float B[3][3], float C[4][3]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            C[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void matrix_mult_4x3_3x1( float A[4][3],  float v[3], float result[4]) {
    for (int i = 0; i < 4; i++) {
        result[i] = 0;
        for (int j = 0; j < 3; j++) {
            result[i] += A[i][j] * v[j];
        }
    }
}

void matrix_mult_3x3_3x1( float A[3][3], float v[3], float result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = 0;
        for (int j = 0; j < 3; j++) {
            result[i] += A[i][j] * v[j];
        }
    }
}

