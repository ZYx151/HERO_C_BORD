#include "Matrix.h"

/**	
 * @brief 矩阵创建
 * @param data  存储数据的二维数组缓冲区  A 2-D array buffer that stores the data
**/
void Matrixf_Init(Matrixf *matrixf, uint8_t rows, uint8_t cols, float *data)
{
	matrixf->_rows = rows;
    matrixf->_cols = cols;
//    matrixf->data_ = (float* )RMLIB_MALLOC(sizeof(float) * cols * rows);
	memcpy(matrixf->data_, data, sizeof(float) * cols * rows);
	arm_mat_init_f32(&matrixf->arm_mat_, rows, cols, matrixf->data_);
}

/**
 * @brief 两个矩阵的附加运算符  Additional operator of two matrices (row * size)
 * @param matrixf  右边的矩阵 The matrix on the right hand side
 * @note  函数返回自身作为结果 This function returns itself as the result
 * @return  两个矩阵的和
 */
Matrixf Matrixf_Add(Matrixf *matrixf, Matrixf *matrixf1)
{
    arm_mat_add_f32(&matrixf->arm_mat_, &matrixf1->arm_mat_, &matrixf->arm_mat_);
    return *matrixf;
}


/**
 * @brief 两个矩阵的减法运算 Substraction operator of two matrices(row * size)
 * @param mat 左边的矩阵 The matrix on the left hand side
 * @note  This function returns itself as the result
 * @return  两个矩阵的差
 */
Matrixf Matrixf_Subtract(Matrixf *matrixf, Matrixf *matrixf1)
{
	arm_mat_sub_f32(&matrixf->arm_mat_, &matrixf1->arm_mat_, &matrixf->arm_mat_);
    return *matrixf;
}

/**
 * @brief 矩阵的标量算子和比例因子 Scalar operator of the matrix and a scaling factor
 * @param val 比例因子 The scaling factor
 * @note  这个函数返回自己作为结果 This function returns itself as the result
 * @return  缩放矩阵  THe scaled matrix
 */
Matrixf Matrixf_Scale(Matrixf *matrixf, const float32_t val)
{
	arm_mat_scale_f32(&matrixf->arm_mat_, val, &matrixf->arm_mat_);
    return *matrixf;
}

/**
 * @brief 矩阵乘法  The matrix multiplication
 * @param mat1 LHS上的矩阵  the matrix on the LHS
 * @param mat2 RHS上的矩阵  the matrix on the RHS
 * @return 乘法结果 The multiplication result
**/
Matrixf Matrixf_Multiply(Matrixf *matrixf, Matrixf *matrixf1, Matrixf *matrixf2)
{
	arm_mat_mult_f32(&matrixf1->arm_mat_, &matrixf2->arm_mat_, &matrixf->arm_mat_); 
    return *matrixf;
}

/**
 * @brief 返回一个乘以delta后的单位矩阵
 *       _rows * delta矩阵    Returns a _rows * delta  matrix
 * @tparam _rows 行大小 The row size
 * @tparam _cols 列大小 The column size
 * @retval The identity matrix
**/
Matrixf Matrixf_eye(Matrixf *matrixf, uint8_t _rows, uint8_t _cols, float delta)
{
	float *data = (float* )RMLIB_MALLOC(sizeof(float) * _cols * _rows);;
	memset(&data, 0, sizeof(sizeof(float) * _rows * _cols));
	for (uint8_t i = 0; i < fmin(_rows, _cols); i++)
	    data[i * _cols + i] = 1 * delta;
    Matrixf_Init(matrixf, _rows, _cols, data);
	return *matrixf;
}

/**
 * @brief Returns a _rows x _cols one matrix
 * @tparam _rows The row size
 * @tparam _cols The column size
 * @retval The one matrix
 */
Matrixf Matrixf_ones(Matrixf *matrixf, uint8_t _rows, uint8_t _cols)
{
	float *data = (float* )RMLIB_MALLOC(sizeof(float) * _cols * _rows);;
	memset(&data, 0, sizeof(sizeof(float) * _rows * _cols));
	for (uint8_t i; i < _rows * _cols ; i++)
	     data[i] = 1;
	Matrixf_Init(matrixf, _rows, _cols, data);
    return *matrixf;
}
/**
 * @brief  生成一个零矩阵 Returns a _rows x _cols zero matrix
 * @tparam _rows The row size
 * @tparam _cols The column size
 * @retval The zero matrix
**/
Matrixf Matrixf_zeros(Matrixf *matrixf, uint8_t _rows, uint8_t _cols)
{
	float *data = (float* )RMLIB_MALLOC(sizeof(float) * _cols * _rows);;
	memset(&data, 0, sizeof(sizeof(float) * _rows * _cols));
	Matrixf_Init(matrixf, _rows, _cols, data);
    return *matrixf;
}
Matrixf Matrixf_Rest(Matrixf *matrixf)
{
	float *data = (float* )RMLIB_MALLOC(sizeof(float) * matrixf->_rows * matrixf->_cols);;
	memset(&data, 0, sizeof(sizeof(float) * matrixf->_rows * matrixf->_cols));
    return *matrixf;
}

/**
 * @brief 两个矩阵相加 Additional operator of two matrices(row * size)
 * @param mat  The matrix on the right hand side
 * @note 这个函数返回自己作为结果
 * @return     The sum of two matrices
**/
Matrixf Matrixf_Operator_add(const Matrixf *matrixf, const Matrixf *mat, Matrixf *Final)
{
	arm_mat_add_f32(&matrixf->arm_mat_, &mat->arm_mat_, &Final->arm_mat_);
    return *Final;
}

/**
 * @brief 两个矩阵的减法运算 Substraction operator of two matrices(row * size)
 * @param mat 左边的矩阵 The matrix on the left hand side
 * @note  This function returns itself as the result
 * @return  两个矩阵的差
 */
Matrixf Matrixf_Operator_sub(const Matrixf *matrixf, const Matrixf *mat, Matrixf *Final)
{
	arm_mat_sub_f32(&matrixf->arm_mat_, &mat->arm_mat_, &Final->arm_mat_);
    return *Final;
}

/**
 * @brief 两个矩阵相乘 Scalar operator of the matrix and a scaling factor
 * @param val The scaling factor
 * @note  This function returns itself as the result
 * @return    THe scaled matrix
**/
Matrixf Matrixf_Operator_mult( Matrixf *matrixf,  Matrixf *mat2, Matrixf *Final)
{
	arm_mat_mult_f32(&matrixf->arm_mat_, &mat2->arm_mat_, &Final->arm_mat_);
	return *Final;
}

/**  矩阵与常数相乘  **/
Matrixf Matrixf_Operator_scalar(const Matrixf *matrixf, const float val, Matrixf *Final)
{
	arm_mat_scale_f32(&matrixf->arm_mat_, val, &Final->arm_mat_);
	return *Final;
}

/**
 * @brief 矩阵与常数相除 Scalar operator of the matrix and a division factor
 * @param val The division factor
 * @note  This function returns itself as the result
 * @retval    matrix / val
 * @return    The scaled matrix
**/
Matrixf Matrixf_Operator_divide(const Matrixf *matrixf, const float val, Matrixf *Final)
{
  	arm_mat_scale_f32(&matrixf->arm_mat_, 1.0f / val, &Final->arm_mat_);
	return *Final;
}

/**
 * @brief 得到矩阵的转置 Get the transpose of the matrix
 * @param
 * @retval 转置矩阵 the transposed matrix
 */
Matrixf Matrixf_Operator_trans(const Matrixf *matrixf, Matrixf *Final)
{
	arm_mat_trans_f32(&matrixf->arm_mat_, &Final->arm_mat_);
	return *Final;
}


/**
 * @brief  返回一个_rows x _cols对角矩阵 Returns a _rows x _cols diagonal matrix
 * @tparam _rows The row size
 * @tparam _cols The column size
 * @param  vec The diagnoal entries
 */
Matrixf Matrixf_dig(const Matrixf *matrixf, const Matrixf *vec, Matrixf *Final)
{
    if(Final->_cols != matrixf->_cols || Final->_rows != vec->_rows)
	     Matrixf_zeros(Final, vec->_rows, matrixf->_cols);
	else 
		 Matrixf_Rest(Final);

        for (int i = 0; i < fmin(matrixf->_rows, matrixf->_cols); i++)
        {
	       Final->data_[i * matrixf->_rows + 1] = 1 * vec->data_[i];
        }
        return *Final;
}
/**
 * @brief 得到矩阵的逆 Get the inverse of the matrix
 * @param
 * @retval The inverse of the matrix
 */
Matrixf Matrixf_inv(const Matrixf *matrixf, Matrixf *Final)
{
	if (matrixf->_cols != matrixf->_rows)
	    return *Final;

	arm_status status = arm_mat_inverse_f32(&matrixf->arm_mat_, &Final->arm_mat_);

	if (status == ARM_MATH_SINGULAR)
		return Matrixf_zeros(Final, matrixf->_rows, matrixf->_cols);
	return *Final;
}
	
	