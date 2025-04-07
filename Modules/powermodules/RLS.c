#include "RLS.h"

/**
* @brief Reset the RLS module
* @retval None
**/
void  RLS_Reset(RLS_s *rls)
{
	Matrixf_eye(&rls->transMatrix, rls->dimension, rls->dimension, rls->delta);// 转移矩阵实例
	Matrixf_zeros(&rls->gainVector, rls->dimension, 1);     // 参数更新的增益矢量
	Matrixf_zeros(&rls->paramsVector, rls->dimension, 1); // 参数向量  
	Matrixf_zeros(&rls->defaultParamsVector, rls->dimension, 1); // 参数向量  
}

/**
 * @brief 构造函数 The constructor
 * @param delta_ 传递矩阵的初始化非奇异值 初始化协方差矩阵对角值  The intialized non-singular value of the transfer matrix
 * @param lambda_ 遗忘因子 0.95 ~ 0.9999 越大收敛越慢越稳定
 */
void RLS_Init(RLS_s *rls, uint32_t dim, float delta_, float lambda_)
{
	rls->lastUpdate = 0;
	rls->updateCnt = 0;
	rls->dimension = dim;
	rls->delta = delta_;
	rls->lambda = lambda_;
	Matrixf_eye(&rls->transMatrix, dim, dim, delta_);// 转移矩阵实例
	Matrixf_zeros(&rls->gainVector, dim, 1);     // 参数更新的增益矢量
	Matrixf_zeros(&rls->paramsVector, dim, 1); // 参数向量
	Matrixf_zeros(&rls->defaultParamsVector, rls->dimension, 1); // 参数向量  
}

/**
 * @brief  处理一个RLS更新周期 Proccess a cycle of RLS update
 * @param  sampleVector 新的样本输入以n × 1 维向量形式表示  The new samples input expressed in n x 1 dimensionasl vector form
 * @param  actualOutput 实际反馈实际输出  The actual feedback real output
 * @retval paramsVector
**/
Matrixf RLS_Update(RLS_s *rls, Matrixf *sampleVector, float actualOutput)
{
   	float float_paramsVector = 0.0f;
   	
	if(rls->updateCnt == 0) 
	{
		Matrixf_zeros(&rls->trans_sample, sampleVector->_cols, sampleVector->_rows);
		Matrixf_zeros(&rls->mult_transMatrix, rls->transMatrix._rows, sampleVector->_cols);
		Matrixf_zeros(&rls->mult_gainVector, sampleVector->_cols, rls->transMatrix._cols);            
		Matrixf_zeros(&rls->divide_transMatrix, sampleVector->_cols, sampleVector->_cols);
		Matrixf_zeros(&rls->mult_paramsVector, sampleVector->_cols, rls->paramsVector._cols);

		Matrixf_zeros(&rls->add_transVector, rls->paramsVector._rows, rls->paramsVector._cols);
		Matrixf_zeros(&rls->transMatrix_mult, rls->gainVector._rows, sampleVector->_rows);                                            
		Matrixf_zeros(&rls->mid_transMatrix, rls->gainVector._rows, rls->transMatrix._cols);
		Matrixf_zeros(&rls->sub_transMatrix, rls->transMatrix._rows, rls->transMatrix._cols);
	}
	// 矩阵求逆
	arm_mat_trans_f32(&sampleVector->arm_mat_, &rls->trans_sample.arm_mat_);
    // 计算增益矢量 Get gain vector
    arm_mat_mult_f32(&rls->transMatrix.arm_mat_, &sampleVector->arm_mat_, &rls->mult_transMatrix.arm_mat_);
    arm_mat_mult_f32(&rls->trans_sample.arm_mat_, &rls->transMatrix.arm_mat_, &rls->mult_gainVector.arm_mat_);
	arm_mat_mult_f32(&rls->mult_gainVector.arm_mat_, &sampleVector->arm_mat_, &rls->divide_transMatrix.arm_mat_);
   	arm_mat_scale_f32(&rls->mult_transMatrix.arm_mat_ , 1 / (1.0f + rls->divide_transMatrix.arm_mat_.pData[0] / rls->lambda), &rls->mult_transMatrix.arm_mat_);
	arm_mat_scale_f32(&rls->mult_transMatrix.arm_mat_ , 1.0f / rls->lambda , &rls->gainVector.arm_mat_);  
//   	arm_mat_scale_f32(&mult_transMatrix.arm_mat_ , rls->lambda + divide_transMatrix.arm_mat_.pData[0] , &rls->gainVector.arm_mat_);
	
	//获取参数向量 Get params vector               
	arm_mat_mult_f32(&rls->trans_sample.arm_mat_, &rls->paramsVector.arm_mat_, &rls->mult_paramsVector.arm_mat_);
   	float_paramsVector = actualOutput - rls->mult_paramsVector.arm_mat_.pData[0];
	arm_mat_scale_f32(&rls->gainVector.arm_mat_, float_paramsVector, &rls->add_transVector.arm_mat_);
	arm_mat_add_f32(&rls->paramsVector.arm_mat_, &rls->add_transVector.arm_mat_, &rls->paramsVector.arm_mat_);
	
	// 得到转移矩阵 Get transferred matrix
	arm_mat_mult_f32(&rls->gainVector.arm_mat_, &rls->trans_sample.arm_mat_ , &rls->transMatrix_mult.arm_mat_);
   	arm_mat_mult_f32(&rls->transMatrix_mult.arm_mat_, &rls->transMatrix.arm_mat_, &rls->mid_transMatrix.arm_mat_);
    arm_mat_sub_f32(&rls->transMatrix.arm_mat_, &rls->mid_transMatrix.arm_mat_, &rls->sub_transMatrix.arm_mat_);
    arm_mat_scale_f32(&rls->sub_transMatrix.arm_mat_, 1 / rls->lambda, &rls->transMatrix.arm_mat_); 

   	rls->updateCnt++;
   	rls->lastUpdate = xTaskGetTickCount();
   	return rls->paramsVector;
}

/**
 * @brief Set the default regression parameters
 * @param updatedParams
 * @retval None
 */
void RLS_setParamVector(RLS_s *rls, float *initParams)
{                                                                                                                  
	memcpy(rls->paramsVector.data_ , initParams, sizeof(float)*(rls->paramsVector._cols)*(rls->paramsVector._rows));
	memcpy(rls->defaultParamsVector.data_ , initParams, sizeof(float)*(rls->paramsVector._cols)*(rls->paramsVector._rows));
}

Matrixf RLS_GetParamsVector(RLS_s *rls) { return rls->paramsVector; }

