/**
  ******************************************************************************
  * @file	 controller.h
  * @author  Wang Hongxi
  * @version V1.1.3
  * @date    2021/7/3
  * @brief   
  ******************************************************************************
  * @attention 
  *
  ******************************************************************************
  */
#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "RMLibHead.h"
#include "bsp_dwt.h"
#include <math.h>
#include "arm_math.h"

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif
/**@brief 限幅函数 */
#define limit(IN, MAX, MIN) \
    if (IN < MIN)           \
        IN = MIN;           \
    if (IN > MAX)           \
        IN = MAX

	
/******************************** FUZZY PID **********************************/
#define NB -3
#define NM -2
#define NS -1
#define ZE 0
#define PS 1
#define PM 2
#define PB 3

#pragma pack(1)
typedef  struct
{
    uint16_t Order;
    uint32_t Count;

    float *x;
    float *y;

    float k;
    float b;

    float StandardDeviation;

    float t[4];
} Ordinary_Least_Squares_t;


typedef struct  // 模糊PID结构体
{
    float KpFuzzy;
    float KiFuzzy;
    float KdFuzzy;


    float (*FuzzyRuleKp)[7];
    float (*FuzzyRuleKi)[7];
    float (*FuzzyRuleKd)[7];

    float KpRatio;
    float KiRatio;
    float KdRatio;

    float eStep;
    float ecStep;

    float e;
    float ec;
    float eLast;

    uint32_t DWT_CNT;
    float dt;
} FuzzyRule_t;

void Fuzzy_Rule_Init(FuzzyRule_t *fuzzyRule, 
	                 float kpRatio, float kiRatio, float kdRatio,
                     float eStep, float ecStep);
void Fuzzy_Rule_Implementation(FuzzyRule_t *fuzzyRule, float measure, float ref);
#pragma pack()
/******************************* PID CONTROL *********************************/
#pragma pack(1)
typedef enum pid_Improvement_e
{
    NONE = 0X00,                        //0000 0000  
    Integral_Limit = 0x01,              //0000 0001  积分限幅
    Derivative_On_Measurement = 0x02,   //0000 0010  微分先行
    Trapezoid_Intergral = 0x04,         //0000 0100  梯形积分
    Proportional_On_Measurement = 0x08, //0000 1000  
    OutputFilter = 0x10,                //0001 0000  输出滤波
    ChangingIntegrationRate = 0x20,     //0010 0000  变积分
    DerivativeFilter = 0x40,            //0100 0000   衍生滤波
    ErrorHandle = 0x80,                 //1000 0000   异常处理
} PID_Improvement_e;

typedef enum errorType_e  // 异常枚举 电机堵转保护
{
    PID_ERROR_NONE = 0x00U,
    Motor_Blocked = 0x01U
} ErrorType_e;

typedef  struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;
#pragma pack()

#pragma pack(1)
/* 用于PID初始化的结构体*/
typedef struct // config parameter
{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // 输出限幅
    float DeadBand; // 死区

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit; // 积分限幅
    float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;
	uint16_t OLS_Order;
} PID_Init_Config_s;

typedef struct pid_t
{
	PID_Init_Config_s pid_config;
    float Ref;         // 期望
    
    float Measure;      // 测量值
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;   // 上次积分结果 PID_COMP_POSITION

    float Pout;
    float Iout;   //Iout = ITerm_0 + ITerm_1 +....+ ITerm_n  多次积分之和
    float Dout;
    float ITerm; // 积分结果 ITerm = Err * Ki

    float Output;
    float Last_Output;
    float Last_Dout;
    float Next_Dout;
//    float ControlPeriod;
    Ordinary_Least_Squares_t OLS; // OLS用于提取信号微分

    uint32_t DWT_CNT;            // DWT定时器用于计算控制周期
    float dt;
//    FuzzyRule_t FuzzyRule;  // 模糊PID
//    PID_ErrorHandler_t ERRORHandler;
//    void (*User_Func1_f)(struct pid_t *pid);
//    void (*User_Func2_f)(struct pid_t *pid);
} PID_t;
#pragma pack()
void PID_Init(
    PID_Init_Config_s *pid,
    float max_out,
    float intergral_limit,
    float deadband,  //死去

    float kp,
    float ki,
    float kd,

    float A,
    float B,

    float output_lpf_rc,
    float derivative_lpf_rc,

    uint16_t ols_order,

    uint8_t improve);

void PID_Create(PID_Init_Config_s *config, PID_t *pid);
float PID_Calculate(PID_t *pid, float measure, float ref,float Derivative_Measure);
void Remove_Iout(PID_t *pid);

/*************************** FEEDFORWARD CONTROL *****************************/
#pragma pack(1)
typedef struct
{
    float c[3]; // G(s) = 1/(c2s^2 + c1s + c0)  // 需要对云台进行系统辨别得到传递函数

    float Ref;
    float Last_Ref;

    float DeadBand;

    uint32_t DWT_CNT;
    float dt;

    float LPF_RC; // RC = 1/omegac

    float Ref_dot;
    float Ref_ddot;
    float Last_Ref_dot;

    uint16_t Ref_dot_OLS_Order;  //  最小二乘法
    Ordinary_Least_Squares_t Ref_dot_OLS;
    uint16_t Ref_ddot_OLS_Order;
    Ordinary_Least_Squares_t Ref_ddot_OLS;

    float Output;
    float MaxOut;

} Feedforward_t;

void Feedforward_Init(
    Feedforward_t *ffc,
    float max_out,
    float *c,
    float lpf_rc,
    uint16_t ref_dot_ols_order,
    uint16_t ref_ddot_ols_order);

float Feedforward_Calculate(Feedforward_t *ffc, float ref);

/************************* LINEAR DISTURBANCE OBSERVER *************************/
typedef  struct
{
    float c[3]; // G(s) = 1/(c2s^2 + c1s + c0)

    float Measure;
    float Last_Measure;

    float u; // system input

    float DeadBand;

    uint32_t DWT_CNT;
    float dt;

    float LPF_RC; // RC = 1/omegac

    float Measure_dot;
    float Measure_ddot;
    float Last_Measure_dot;

    uint16_t Measure_dot_OLS_Order;
    Ordinary_Least_Squares_t Measure_dot_OLS;
    uint16_t Measure_ddot_OLS_Order;
    Ordinary_Least_Squares_t Measure_ddot_OLS;

    float Disturbance;
    float Output;
    float Last_Disturbance;
    float Max_Disturbance;
} LDOB_t;

void LDOB_Init(
    LDOB_t *ldob,
    float max_d,
    float deadband,
    float *c,
    float lpf_rc,
    uint16_t measure_dot_ols_order,
    uint16_t measure_ddot_ols_order);

float LDOB_Calculate(LDOB_t *ldob, float measure, float u);
#pragma pack()
/*************************** Tracking Differentiator ***************************/
#pragma pack(1)
typedef  struct
{
    float Input;

    float h0;
    float r;

    float x;
    float dx;
    float ddx;

    float last_dx;
    float last_ddx;

    uint32_t DWT_CNT;
    float dt;
} TD_t;

void TD_Init(TD_t *td, float r, float h0);
float TD_Calculate(TD_t *td, float input);
#pragma pack()

/*************************** sliding mode control 滑膜控制 ***************************/

typedef float ( *ReachingLaw)(float data);

#pragma pack(1)

typedef struct Smc_config_t {
    float kp;                 // 常规滑动模糊面系数
  	float kd;  
	float kmax;               // 扰动增益系数
	ReachingLaw reaching_law; // 趋近
	float outputMax;          // 最大输出限幅
}Smc_config;

typedef struct Smc_t {
	Smc_config config;
	float error[2];
	float ref;
	float fdb;
	float output;
} Smc;
#pragma pack()

void SMC_Init(Smc* smc,Smc_config* config);
void SMC_Calc(Smc* smc, float ref, float fbd);
void SMC_SetConfig(Smc_config* config, float kp, float kd, float kmax, ReachingLaw reaching_law, float outputMax) ;

// 趋近律
float ReachingLaw_sgn(float error);
float ReachingLaw_sqrt(float error);
float ReachingLaw_square(float error);

int sgn(int x);
int fsgn(float x);
float sgn_like(float x, float d);

//  最小二乘法
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS);
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS);

#endif
