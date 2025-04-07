/**
 * @file    PID.c
 * @author  yao
 * @date    1-May-2020
 * @brief   PID模块
 **/

#include "PID.h"
#include "bsp_dwt.h"

float PID_Control(float current, float expected, PID *parameter) {
	parameter->ref = expected;
	parameter->measure = current;
	
    parameter->dt = DWT_GetDeltaT((void *)&parameter->DWT_CNT); // 获取两次pid计算的时间间隔,用于积分和微分
    parameter->error_now = expected - current;

    if(fabs(parameter->error_now) < parameter->DeadBand) {
        parameter->error_now = 0.0f;                   
        parameter->error_last = parameter->error_now;
	    parameter->last_measure = parameter->measure; 
        parameter->ITerm = 0;
		parameter->Iout *= 0.8f;
		return parameter->pid_out;
	}

	// 抗饱和积分  (P->PreError+P->LastError)/2*(P->PreError - P->I_L)/(P->I_U - P->I_L);
	if (fabs(parameter->error_now) < fabs(parameter->inter_threLow))
	{
	// 梯形积分 
		if(parameter->error_now <= 0)
			parameter->ITerm = (parameter->error_now + parameter->error_last + parameter->DeadBand) / 2;
		else
			parameter->ITerm = (parameter->error_now + parameter->error_last - parameter->DeadBand) / 2;
	}
	else if(fabs(parameter->error_now) < fabs(parameter->inter_threUp))
	{
		if(parameter->error_now <= 0)
			parameter->ITerm = (parameter->error_now + parameter->error_last + parameter->DeadBand) / 2 * (parameter->error_now - parameter->inter_threLow)/(parameter->inter_threUp - parameter->inter_threLow);
		else
			parameter->ITerm = (parameter->error_now + parameter->error_last - parameter->DeadBand) / 2 * (parameter->error_now - parameter->inter_threLow)/(parameter->inter_threUp - parameter->inter_threLow);
	}
	else
		 parameter->ITerm = 0;
	// 比例计算
	parameter->Pout = parameter->Kp * parameter->error_now;
	//  微分计算
	parameter->Dout = parameter->Kd * (parameter->last_measure = parameter->measure);
	// 积分限幅
	parameter->Iout += parameter->ITerm * parameter->Ki * parameter->dt;
	limit(parameter->Iout, parameter->interlimit, -parameter->interlimit);
	// 输出计算
	parameter->pid_out = parameter->Pout + parameter->Iout + parameter->Dout;
	limit(parameter->pid_out, parameter->outlimit, -parameter->outlimit);
	parameter->error_last = parameter->error_now;
	
	parameter->last_measure = parameter->measure; 
	return parameter->pid_out;
}

float PID_Control_Smis(float current, float expected, PID_Smis *parameter, float speed) {
    parameter->dt = DWT_GetDeltaT((void *)&parameter->DWT_CNT); // 获取两次pid计算的时间间隔,用于积分和微分

    parameter->error_now = expected - current;

    if(fabs(parameter->error_now) < parameter->DeadBand){
        parameter->error_last = parameter->error_now;
		parameter->error_now = 0.0f;
//        parameter->Iout = 0;
        parameter->ITerm = 0;
		return parameter->pid_out;
	}
	
	// 抗饱和积分  (P->PreError+P->LastError)/2*(P->PreError - P->I_L)/(P->I_U - P->I_L);
	if (fabs(parameter->error_now) < fabs(parameter->inter_threLow))
	{
	// 梯形积分 
		if(parameter->error_now <= 0)
			parameter->ITerm = (parameter->error_now + parameter->error_last + parameter->DeadBand) / 2;
		else
			parameter->ITerm = (parameter->error_now + parameter->error_last - parameter->DeadBand) / 2;
	}
	else if(fabs(parameter->error_now) < fabs(parameter->inter_threUp))
	{
		if(parameter->error_now <= 0)
			parameter->ITerm = (parameter->error_now + parameter->error_last + parameter->DeadBand) / 2 * (parameter->error_now - parameter->inter_threLow)/(parameter->inter_threUp - parameter->inter_threLow) ;
		else
			parameter->ITerm = (parameter->error_now + parameter->error_last - parameter->DeadBand) / 2 * (parameter->error_now - parameter->inter_threLow)/(parameter->inter_threUp - parameter->inter_threLow);
	}
	else 
		 parameter->ITerm = 0;
		
	// 比例计算
	parameter->Pout = parameter->Kp * parameter->error_now;
	//  微分先行
	parameter->Dout = parameter->Kd * speed;
	// 积分限幅
	parameter->Iout += parameter->ITerm * parameter->Ki * parameter->dt;
	limit(parameter->Iout, parameter->interlimit, -parameter->interlimit);
	// 输出计算
	parameter->pid_out = parameter->Pout + parameter->Iout + parameter->Dout;
	// 输出滤波
////		parameter->pid_out = parameter->last_pid_out * ( 1 - parameter->Output_LPF_RC ) + parameter->pid_out;
	// 输出限幅
	limit(parameter->pid_out, parameter->outlimit, -parameter->outlimit);
	parameter->error_last = parameter->error_now;
	parameter->last_pid_out = parameter->pid_out;
	return parameter->pid_out;
}

float PID_Increment(float current, float expect, PID_ADD *parameter) {
    parameter->error_now = expect - current;

    parameter->increament =
            parameter->Kp * (parameter->error_now - parameter->error_next) + parameter->Ki * (parameter->error_now) + 
            parameter->Kd * (parameter->error_now - 2 * parameter->error_next + parameter->error_last);
    
    parameter->error_last = parameter->error_next;
    parameter->error_next = parameter->error_now;
    
    return parameter->increament;
}

float FeedForward_Calc(FeedForward_Typedef *FF, float In) {
	FF->dt = DWT_GetDeltaT((void *)&FF->DWT_CNT);
    FF->Now_DeltIn = In;
    
    FF->Ref_dot  = (FF->Now_DeltIn - FF->Last_DeltIn) / FF->dt;
	FF->Ref_ddot = (FF->Ref_dot - FF->Last_dout) / FF->dt;
	
	FF->Out = FF->Now_DeltIn * FF->K1 + FF->Ref_dot * FF->K2 + FF->Ref_ddot * FF->K3;
	
	FF->Last_DeltIn = FF->Now_DeltIn;
    FF->Last_dout = FF->Ref_dot;
    limit(FF->Out,FF->OutMax,-FF->OutMax);

    return FF->Out;
}

void PID_IoutReset(PID *parameter)
{
	parameter->Iout = 0;
	parameter->pid_out = 0;
}