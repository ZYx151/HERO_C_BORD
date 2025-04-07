#ifndef __RLS_H
#define __RLS_H

#include "Matrix.h"
#include "QuaternionEKF.h"


typedef struct {
    uint32_t dimension;  // RLS空间的维度 Dimension of the RLS space
    float lambda;        // 遗忘指数 The forget index
    float delta;         // 转移矩阵的初始值 Intialized value of the transferred matrix
    
    TickType_t lastUpdate;  // Last update tick
    uint32_t updateCnt;     // Total update Count
    
    /*RLS relvant matrix*/
    Matrixf transMatrix;  	     // <dim, dim> 转移矩阵实例 Transfer matrix instance
    Matrixf gainVector;          // <dim, 1>   参数更新的增益矢量 Gain vector for params update
    Matrixf paramsVector;        // <dim, 1>   参数向量 Params vector
    Matrixf defaultParamsVector; // <dim, 1>   初始的参数矩阵s
	
	
	Matrixf trans_sample;	// 计算中间变量
	Matrixf mult_transMatrix;
    Matrixf mult_gainVector;
	Matrixf divide_transMatrix;
    Matrixf add_transVector;
	Matrixf mult_paramsVector;
 	Matrixf mid_transMatrix;
	Matrixf transMatrix_mult;
	Matrixf sub_transMatrix ;


}RLS_s;

void  RLS_Reset(RLS_s *rls);
void RLS_Init(RLS_s *rls, uint32_t dim, float delta_, float lambda_);
Matrixf RLS_Update(RLS_s *rls, Matrixf *sampleVector, float actualOutput);

void RLS_setParamVector(RLS_s *rls, float *initParams);
Matrixf RLS_GetParamsVector(RLS_s *rls);
#endif
