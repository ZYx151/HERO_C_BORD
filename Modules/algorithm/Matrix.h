#ifndef __MATRIX_H
#define __MATRIX_H

#include "RMLibHead.h"
#include "math.h"
#include "arm_math.h"

#define matr arm_matrix_instance_f32
//#define Matrix_Init arm_mat_init_f32
//#define Matrix_Add arm_mat_add_f32
//#define Matrix_Subtract arm_mat_sub_f32
//#define Matrix_Multiply arm_mat_mult_f32
//#define Matrix_Transpose arm_mat_trans_f32
//#define Matrix_Inverse arm_mat_inverse_f32

typedef struct {
    matr arm_mat_;  // The arm math instance
	  // Data buffer
	uint8_t _rows;
	uint8_t _cols;
    float data_[6]; // 用于初始化矩阵 用于之后的计算
}Matrixf;

void Matrixf_Init(Matrixf *matrixf, uint8_t rows, uint8_t cols, float *data);
Matrixf Matrixf_Add(Matrixf *matrixf, Matrixf *matrixf1);
Matrixf Matrixf_Subtract(Matrixf *matrixf, Matrixf *matrixf1);
Matrixf Matrixf_Scale(Matrixf *matrixf, const float32_t val);
Matrixf Matrixf_Multiply(Matrixf *matrixf, Matrixf *matrixf1, Matrixf *matrixf2);

/* 矩阵设置 */
Matrixf Matrixf_eye(Matrixf *matrixf, uint8_t _rows, uint8_t _cols, float delta);
Matrixf Matrixf_ones(Matrixf *matrixf, uint8_t _rows, uint8_t _cols);
Matrixf Matrixf_zeros(Matrixf *matrixf, uint8_t _rows, uint8_t _cols);

/* 矩阵计算 */
Matrixf Matrixf_Operator_add(const Matrixf *matrixf, const Matrixf *mat, Matrixf *Final);
Matrixf Matrixf_Operator_sub(const Matrixf *matrixf, const Matrixf *mat, Matrixf *Final);
Matrixf Matrixf_Operator_mult( Matrixf *matrixf,  Matrixf *mat2, Matrixf *Final);
Matrixf Matrixf_Operator_divide(const Matrixf *matrixf, const float val, Matrixf *Final);
Matrixf Matrixf_Operator_scalar(const Matrixf *matrixf, const float val, Matrixf *Final);
Matrixf Matrixf_Operator_trans(const Matrixf *matrixf, Matrixf *Final);
#endif
