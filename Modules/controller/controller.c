#include "controller.h"
#include "user_lib.h"

int sgn(int x){
    return x == 0 ? 0 : x > 0 ? 1 : -1;
}

int fsgn(float x) {
    return (x != 0.0f ? (x < 0.0f ? -1 : 1) : 0);
}

float sgn_like(float x, float d) {
    if (fabs(x) >= d)
        return fsgn(x);
    else
        return x / d;
}

/******************************** FUZZY PID **********************************/
 float FuzzyRuleKpRAW[7][7] = {
    PB, PB, PM, PM, PS, ZE, ZE,
    PB, PB, PM, PS, PS, ZE, PS,
    PM, PM, PM, PS, ZE, PS, PS,
    PM, PM, PS, ZE, PS, PM, PM,
    PS, PS, ZE, PS, PS, PM, PM,
    PS, ZE, PS, PM, PM, PM, PB,
    ZE, ZE, PM, PM, PM, PB, PB};

 float FuzzyRuleKiRAW[7][7] = {
    PB, PB, PM, PM, PS, ZE, ZE,
    PB, PB, PM, PS, PS, ZE, ZE,
    PB, PM, PM, PS, ZE, PS, PS,
    PM, PM, PS, ZE, PS, PM, PM,
    PS, PS, ZE, PS, PS, PM, PB,
    ZE, ZE, PS, PS, PM, PB, PB,
    ZE, ZE, PS, PM, PM, PB, PB};

 float FuzzyRuleKdRAW[7][7] = {
    PS, PS, PB, PB, PB, PM, PS,
    PS, PS, PB, PM, PM, PS, ZE,
    ZE, PS, PM, PM, PS, PS, ZE,
    ZE, PS, PS, PS, PS, PS, ZE,
    ZE, ZE, ZE, ZE, ZE, ZE, ZE,
    PB, PS, PS, PS, PS, PS, PB,
    PB, PM, PM, PM, PS, PS, PB};
void Fuzzy_Rule_Init(FuzzyRule_t *fuzzyRule,
                     float kpRatio, float kiRatio, float kdRatio,
                     float eStep, float ecStep)
{
    fuzzyRule->KpRatio = kpRatio;
    fuzzyRule->KiRatio = kiRatio;
    fuzzyRule->KdRatio = kdRatio;

    if (eStep < 0.00001f)
        eStep = 1;
    if (ecStep < 0.00001f)
        ecStep = 1;
    fuzzyRule->eStep = eStep;
    fuzzyRule->ecStep = ecStep;
}
void Fuzzy_Rule_Implementation(FuzzyRule_t *fuzzyRule, float measure, float ref)
{
    float eLeftTemp, ecLeftTemp;
    float eRightTemp, ecRightTemp;
    int eLeftIndex, ecLeftIndex;
    int eRightIndex, ecRightIndex;

    fuzzyRule->dt = 0.001;

    fuzzyRule->e = ref - measure;
    fuzzyRule->ec = (fuzzyRule->e - fuzzyRule->eLast) / fuzzyRule->dt;
    fuzzyRule->eLast = fuzzyRule->e;

    //隶属区间
    eLeftIndex =   (int)(fuzzyRule->e >= 3 * fuzzyRule->eStep ? 6 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e >= 0 ? ((int)(fuzzyRule->e / fuzzyRule->eStep) + 3) : ((int)(fuzzyRule->e / fuzzyRule->eStep) + 2))));
    eRightIndex =  (int)(fuzzyRule->e >= 3 * fuzzyRule->eStep ? 6 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e >= 0 ? ((int)(fuzzyRule->e / fuzzyRule->eStep) + 4) : ((int)(fuzzyRule->e / fuzzyRule->eStep) + 3))));
    ecLeftIndex =  (int)(fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 6 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec >= 0 ? ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 3) : ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 2))));
    ecRightIndex = (int)(fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 6 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec >= 0 ? ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 4) : ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 3))));

    //隶属度
    eLeftTemp =   (fuzzyRule->e >= 3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 1 : (eRightIndex - fuzzyRule->e / fuzzyRule->eStep - 3)));
    eRightTemp =  (fuzzyRule->e >= 3 * fuzzyRule->eStep ? 1 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e / fuzzyRule->eStep - eLeftIndex + 3)));
    ecLeftTemp =  (fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 1 : (ecRightIndex - fuzzyRule->ec / fuzzyRule->ecStep - 3)));
    ecRightTemp = (fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 1 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec / fuzzyRule->ecStep - ecLeftIndex + 3)));

	limit(eLeftIndex,6,0);
	limit(eRightIndex,6,0);
	limit(ecRightIndex,6,0);
	limit(ecLeftIndex,6,0);
	
    fuzzyRule->KpFuzzy = eLeftTemp * ecLeftTemp * FuzzyRuleKpRAW[eLeftIndex][ecLeftIndex]
	
                         + eLeftTemp * ecRightTemp * FuzzyRuleKpRAW[eRightIndex][ecLeftIndex] 
	
	                     + eRightTemp * ecLeftTemp * FuzzyRuleKpRAW[eLeftIndex][ecRightIndex] 
	
	                     + eRightTemp * ecRightTemp * FuzzyRuleKpRAW[eRightIndex][ecRightIndex];

    fuzzyRule->KiFuzzy = eLeftTemp * ecLeftTemp * FuzzyRuleKiRAW[eLeftIndex][ecLeftIndex]  
	
	                     + eLeftTemp * ecRightTemp * FuzzyRuleKiRAW[eRightIndex][ecLeftIndex] 
	
	                     + eRightTemp * ecLeftTemp * FuzzyRuleKiRAW[eLeftIndex][ecRightIndex] 
	
	                     + eRightTemp * ecRightTemp * FuzzyRuleKiRAW[eRightIndex][ecRightIndex];
						 

    fuzzyRule->KdFuzzy = ( eLeftTemp * ecLeftTemp * FuzzyRuleKdRAW[eLeftIndex][ecLeftIndex] 
	
	                    + eLeftTemp * ecRightTemp * FuzzyRuleKdRAW[eRightIndex][ecLeftIndex] 
						
						+ eRightTemp * ecLeftTemp * FuzzyRuleKdRAW[eLeftIndex][ecRightIndex] 
						
						+ eRightTemp * ecRightTemp * FuzzyRuleKdRAW[eRightIndex][ecRightIndex]) *0.3f;
}

/******************************* PID CONTROL *********************************/
// PID优化环节函数声明
static void f_Trapezoid_Intergral(PID_t *pid);
static void f_Integral_Limit(PID_t *pid);
static void f_Derivative_On_Measurement(PID_t *pid,float Derivative_Measure);
static void f_Changing_Integration_Rate(PID_t *pid);
static void f_Output_Filter(PID_t *pid);
static void f_Derivative_Filter(PID_t *pid);
static void f_Output_Limit(PID_t *pid);
static void f_Proportion_Limit(PID_t *pid);
//static void f_PID_ErrorHandle(PID_t *pid);


static void f_Trapezoid_Intergral(PID_t *pid)
{
//    if (pid->FuzzyRule.dt == NULL)
        pid->ITerm = pid->pid_config.Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
//    else
//        pid->ITerm = (pid->Ki + pid->FuzzyRule.KiFuzzy) * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

static void f_Changing_Integration_Rate(PID_t *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        if (abs(pid->Err) <= pid->pid_config.CoefA)
            return; // Full integral
        if (abs(pid->Err) <= (pid->pid_config.CoefA + pid->pid_config.CoefB))
            pid->ITerm *= (pid->pid_config.CoefA - abs(pid->Err) + pid->pid_config.CoefB) / pid->pid_config.CoefA;
        else
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PID_t *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (abs(temp_Output) > pid->pid_config.MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
            // 积分呈累积趋势
            // Integral still increasing
            pid->ITerm = 0;
        }
    }
    
    if (temp_Iout > pid->pid_config.IntegralLimit)  // 积分限幅 
    {
        pid->ITerm = 0;
        pid->Iout = pid->pid_config.IntegralLimit;
    }
    if (temp_Iout < -pid->pid_config.IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->pid_config.IntegralLimit;
    }
}

static void f_Derivative_On_Measurement(PID_t *pid,float Derivative_Measure)
{
        if (pid->pid_config.OLS_Order > 2)
            pid->Dout = pid->pid_config.Kd * OLS_Derivative(&pid->OLS, pid->dt, -pid->Measure);
        else
            pid->Dout = pid->pid_config.Kd * Derivative_Measure;
}

static void f_Derivative_Filter(PID_t *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->pid_config.Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->pid_config.Derivative_LPF_RC / (pid->pid_config.Derivative_LPF_RC + pid->dt);
}

static void f_Output_Filter(PID_t *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->pid_config.Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->pid_config.Output_LPF_RC / (pid->pid_config.Output_LPF_RC + pid->dt);
}

static void f_Output_Limit(PID_t *pid)
{
    if (pid->Output > pid->pid_config.MaxOut)
    {
        pid->Output = pid->pid_config.MaxOut;
    }
    if (pid->Output < -(pid->pid_config.MaxOut))
    {
        pid->Output = -(pid->pid_config.MaxOut);
    }
}

static void f_Proportion_Limit(PID_t *pid)
{
    if (pid->Pout > pid->pid_config.MaxOut)
    {
        pid->Pout = pid->pid_config.MaxOut;
    }
    if (pid->Pout < -(pid->pid_config.MaxOut))
    {
        pid->Pout = -(pid->pid_config.MaxOut);
    }
}

// PID ERRORHandle Function
//static void f_PID_ErrorHandle(PID_t *pid)
//{
//    /*Motor Blocked Handle*/
//    if (pid->Output < pid->pid_config.MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
//        return;

//    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
//    {
//        // Motor blocked counting
//        pid->ERRORHandler.ERRORCount++;
//    }
//    else
//    {
//        pid->ERRORHandler.ERRORCount = 0;
//    }

//    if (pid->ERRORHandler.ERRORCount > 500)
//    {
//        // Motor blocked over 1000times
//        pid->ERRORHandler.ERRORType = Motor_Blocked;
//    }
//}

/**
 * @brief          PID初始化   PID initialize
 * @param[in]      PID结构体   PID structure
 * @param[in]      
 * @retval         返回空      null
 */
void PID_Init(
    PID_Init_Config_s *pid,
    float max_out,
    float intergral_limit,
    float deadband,
    
    float kp,
    float Ki,
    float Kd,
    
    float A,
    float B,
    
    float output_lpf_rc,
    float derivative_lpf_rc,

    uint16_t ols_order,
    
    uint8_t improve/*使能优化PID参数*/)
{
    pid->DeadBand = deadband;   // 死区赋值
    pid->IntegralLimit = intergral_limit; // 积分限幅
    pid->MaxOut = max_out;       // 输出值限度

    pid->Kp = kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    // 变速积分参数
    // coefficient of changing integration rate
    pid->CoefA = A;
    pid->CoefB = B;

    pid->Output_LPF_RC = output_lpf_rc;   // 输出 滤波遥控器

    pid->Derivative_LPF_RC = derivative_lpf_rc;  // 衍生滤波 遥控

    // 最小二乘提取信号微分初始化
    // differential signal is distilled by OLS
    pid->OLS_Order = ols_order;
    // 设置PID优化环节
    pid->Improve = improve;

}

void PID_Create(PID_Init_Config_s *config, PID_t *pid)
{
    pid->pid_config = *config;
	pid->DWT_CNT = 0;
	OLS_Init(&pid->OLS, config->OLS_Order);	
}
/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PID_Calculate(PID_t *pid, float measure, float ref,float Derivative_Measure)
{
//    if (pid->pid_config.Improve & ErrorHandle)
//        f_PID_ErrorHandle(pid);
	
    pid->dt = DWT_GetDeltaT((void *)&pid->DWT_CNT); // 获取两次pid计算的时间间隔,用于积分和微分
    
	pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;
    
	/** 自定义指针函数 **/
//    if (pid->User_Func1_f != NULL)
//        pid->User_Func1_f(pid);
	
    if ( pid->pid_config.Improve & ErrorHandle) {
		pid->Output = pid->Ref;
		return pid->Ref;
	}
	
    if (abs(pid->Err) > pid->pid_config.DeadBand)
    {
            pid->Pout = pid->pid_config.Kp * pid->Err;
            pid->ITerm = pid->pid_config.Ki * pid->Err * pid->dt;
            if (pid->pid_config.OLS_Order > 2) // 使能最小二乘法
                pid->Dout = pid->pid_config.Kd * OLS_Derivative(&pid->OLS, pid->dt, pid->Err);
            else
                pid->Dout = pid->pid_config.Kd * (pid->Err - 2 * pid->Next_Dout + pid->Last_Err);
//        if (pid->User_Func2_f != NULL)
//            pid->User_Func2_f(pid);
         
        // 梯形积分
        if (pid->pid_config.Improve & Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->pid_config.Improve & ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 微分先行
        if (pid->pid_config.Improve & Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid, Derivative_Measure);
        // 微分滤波器
        if (pid->pid_config.Improve & DerivativeFilter)
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->pid_config.Improve & Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;

        pid->Output = pid->Pout + pid->Iout + pid->Dout;

        // 输出滤波
        if (pid->pid_config.Improve & OutputFilter)
            f_Output_Filter(pid); 
		// 输出限幅
        f_Output_Limit(pid);
		
	} else {
		pid->Iout = 0;
		pid->Output = 0;
		return 0;
	}
	pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Next_Dout = pid->Last_Dout;
	pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;

    return pid->Output;
}

void Remove_Iout(PID_t *pid)
{
	pid->Iout = 0;
}

/*************************** FEEDFORWARD CONTROL *****************************/
/**
 * @brief          前馈控制初始化
 * @param[in]      前馈控制结构体
 * @param[in]      略
 * @retval         返回空
 */
void Feedforward_Init(
    Feedforward_t *ffc,
    float max_out,
    float *c,      // G(s) = 1/(c2s^2 + c1s + c0)  // 需要对云台进行系统辨别得到传递函数
    float lpf_rc,
    uint16_t ref_dot_ols_order,
    uint16_t ref_ddot_ols_order)
{
    ffc->MaxOut = max_out;

    // 设置前馈控制器参数 详见前馈控制结构体定义
    // set parameters of feed-forward controller (see struct definition)
    if (c != NULL && ffc != NULL)
    {
        ffc->c[0] = c[0];
        ffc->c[1] = c[1];
        ffc->c[2] = c[2];
    }
    else
    {
        ffc->c[0] = 0;
        ffc->c[1] = 0;
        ffc->c[2] = 0;
        ffc->MaxOut = 0;
    }

    ffc->LPF_RC = lpf_rc;

    // 最小二乘提取信号微分初始化
    // differential signal is distilled by OLS
    ffc->Ref_dot_OLS_Order = ref_dot_ols_order;
    ffc->Ref_ddot_OLS_Order = ref_ddot_ols_order;
    if (ref_dot_ols_order > 2)
        OLS_Init(&ffc->Ref_dot_OLS, ref_dot_ols_order);
    if (ref_ddot_ols_order > 2)
        OLS_Init(&ffc->Ref_ddot_OLS, ref_ddot_ols_order);

    ffc->DWT_CNT = 0;

    ffc->Output = 0;
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float Feedforward_Calculate(Feedforward_t *ffc, float ref)
{
    ffc->dt = DWT_GetDeltaT((void *)&ffc->DWT_CNT);

    ffc->Ref = ref * ffc->dt / (ffc->LPF_RC + ffc->dt) +
               ffc->Ref * ffc->LPF_RC / (ffc->LPF_RC + ffc->dt);
    // 计算一阶导数
    // calculate first derivative
    if (ffc->Ref_dot_OLS_Order > 2)
        ffc->Ref_dot = OLS_Derivative(&ffc->Ref_dot_OLS, ffc->dt, ffc->Ref);
    else
        ffc->Ref_dot = (ffc->Ref - ffc->Last_Ref) / ffc->dt;

    // 计算二阶导数
    // calculate second derivative
    if (ffc->Ref_ddot_OLS_Order > 2)
        ffc->Ref_ddot = OLS_Derivative(&ffc->Ref_ddot_OLS, ffc->dt, ffc->Ref_dot);
    else
        ffc->Ref_ddot = (ffc->Ref_dot - ffc->Last_Ref_dot) / ffc->dt;

    // 计算前馈控制输出
    // calculate feed-forward controller output
    ffc->Output = ffc->c[0] * ffc->Ref + ffc->c[1] * ffc->Ref_dot + ffc->c[2] * ffc->Ref_ddot;

    ffc->Output = float_constrain(ffc->Output, -ffc->MaxOut, ffc->MaxOut);

    ffc->Last_Ref = ffc->Ref;
    ffc->Last_Ref_dot = ffc->Ref_dot;

    return ffc->Output;
}

/*************************LINEAR DISTURBANCE OBSERVER *************************/
void LDOB_Init(
    LDOB_t *ldob,
    float max_d,
    float deadband,
    float *c,
    float lpf_rc,
    uint16_t measure_dot_ols_order,
    uint16_t measure_ddot_ols_order)
{
    ldob->Max_Disturbance = max_d;

    ldob->DeadBand = deadband;

    // 设置线性扰动观测器参数 详见LDOB结构体定义
    // set parameters of linear disturbance observer (see struct definition)
    if (c != NULL && ldob != NULL)
    {
        ldob->c[0] = c[0];
        ldob->c[1] = c[1];
        ldob->c[2] = c[2];
    }
    else
    {
        ldob->c[0] = 0;
        ldob->c[1] = 0;
        ldob->c[2] = 0;
        ldob->Max_Disturbance = 0;
    }

    // 设置Q(s)带宽  Q(s)选用一阶惯性环节
    // set bandwidth of Q(s)    Q(s) is chosen as a first-order low-pass form
    ldob->LPF_RC = lpf_rc;

    // 最小二乘提取信号微分初始化
    // differential signal is distilled by OLS
    ldob->Measure_dot_OLS_Order = measure_dot_ols_order;
    ldob->Measure_ddot_OLS_Order = measure_ddot_ols_order;
    if (measure_dot_ols_order > 2)
        OLS_Init(&ldob->Measure_dot_OLS, measure_dot_ols_order);
    if (measure_ddot_ols_order > 2)
        OLS_Init(&ldob->Measure_ddot_OLS, measure_ddot_ols_order);

    ldob->DWT_CNT = 0;

    ldob->Disturbance = 0;
}

float LDOB_Calculate(LDOB_t *ldob, float measure, float u)
{
    ldob->dt = DWT_GetDeltaT((void *)&ldob->DWT_CNT);

    ldob->Measure = measure;

    ldob->u = u;

    // 计算一阶导数
    // calculate first derivative
    if (ldob->Measure_dot_OLS_Order > 2)
        ldob->Measure_dot = OLS_Derivative(&ldob->Measure_dot_OLS, ldob->dt, ldob->Measure);
    else
        ldob->Measure_dot = (ldob->Measure - ldob->Last_Measure) / ldob->dt;

    // 计算二阶导数
    // calculate second derivative
    if (ldob->Measure_ddot_OLS_Order > 2)
        ldob->Measure_ddot = OLS_Derivative(&ldob->Measure_ddot_OLS, ldob->dt, ldob->Measure_dot);
    else
        ldob->Measure_ddot = (ldob->Measure_dot - ldob->Last_Measure_dot) / ldob->dt;

    // 估计总扰动
    // estimate external disturbances and internal disturbances caused by model uncertainties
    ldob->Disturbance = ldob->c[0] * ldob->Measure + ldob->c[1] * ldob->Measure_dot + ldob->c[2] * ldob->Measure_ddot - ldob->u;
    ldob->Disturbance = ldob->Disturbance * ldob->dt / (ldob->LPF_RC + ldob->dt) +
                        ldob->Last_Disturbance * ldob->LPF_RC / (ldob->LPF_RC + ldob->dt);

    ldob->Disturbance = float_constrain(ldob->Disturbance, -ldob->Max_Disturbance, ldob->Max_Disturbance);

    // 扰动输出死区
    // deadband of disturbance output
    if (abs(ldob->Disturbance) > ldob->DeadBand * ldob->Max_Disturbance)
        ldob->Output = ldob->Disturbance;
    else
        ldob->Output = 0;

    ldob->Last_Measure = ldob->Measure;
    ldob->Last_Measure_dot = ldob->Measure_dot;
    ldob->Last_Disturbance = ldob->Disturbance;

    return ldob->Output;
}

/*************************** Tracking Differentiator ***************************/
void TD_Init(TD_t *td, float r, float h0)
{
    td->r = r;
    td->h0 = h0;

    td->x = 0;
    td->dx = 0;
    td->ddx = 0;
    td->last_dx = 0;
    td->last_ddx = 0;
}
float TD_Calculate(TD_t *td, float input)
{
    static float d, a0, y, a1, a2, a, fhan;

    td->dt = DWT_GetDeltaT((void *)&td->DWT_CNT);

    if (td->dt > 0.5f)
        return 0;

    td->Input = input;

    d = td->r * td->h0 * td->h0;
    a0 = td->dx * td->h0;
    y = td->x - td->Input + a0;
    a1 = sqrt(d * (d + 8 * abs(y)));
    a2 = a0 + sign(y) * (a1 - d) / 2;
    a = (a0 + y) * (sign(y + d) - sign(y - d)) / 2 + a2 * (1 - (sign(y + d) - sign(y - d)) / 2);
    fhan = -td->r * a / d * (sign(a + d) - sign(a - d)) / 2 -
           td->r * sign(a) * (1 - (sign(a + d) - sign(a - d)) / 2);

    td->ddx = fhan;
    td->dx += (td->ddx + td->last_ddx) * td->dt / 2;
    td->x += (td->dx + td->last_dx) * td->dt / 2;

    td->last_ddx = td->ddx;
    td->last_dx = td->dx;

    return td->x;
}

/*************************** sliding mode control 滑模控制 ***************************/

void SMC_SetConfig(Smc_config* config, float kp, float kd, float kmax, ReachingLaw reaching_law, float outputMax)
{
    config->kp = kp;
    config->kd = kd;
    config->kmax = kmax;
    config->reaching_law = reaching_law;
    config->outputMax = outputMax;
}

void SMC_Init(Smc* smc, Smc_config* config)
{
    smc->config = *config;
}
 
void SMC_Calc(Smc* smc, float ref, float fdb) {
	smc->ref = ref;
    smc->fdb = fdb;
	smc->error[1] = smc->error[0];
	smc->error[0] = smc->ref - smc->fdb;
	
	float u;
	u = smc->config.reaching_law(smc->error[0]);
	
    smc->output = smc->config.kp * smc->error[0]                        // 经典滑膜面选择
	               + smc->config.kd * (smc->error[0] - smc->error[1])   // kd
	               + smc->config.kmax * u;
 	if( smc->output < -smc->config.outputMax ) smc->output =  -smc->config.outputMax;
	if( smc->output > +smc->config.outputMax ) smc->output =  +smc->config.outputMax;
		
}

// 最简单的趋近律
// 易产生震动
float ReachingLaw_sgn(float error) { return fsgn(error); }

// 根号趋近
// 根号趋近
// 能有效减小稳态误差
float ReachingLaw_sqrt(float error) {
    float u;
    if (error > 0) {
        arm_sqrt_f32(fabs(error), &u);
    } else if (error < 0) {
        arm_sqrt_f32(fabs(error), &u);
        u *= -1;
    } else
        u = 0;

    return u;
}

float ReachingLaw_square(float error) { return (error * error + 0.1f) * fsgn(error); }

/**
  * @brief          最小二乘法初始化
  * @param[in]      最小二乘法结构体
  * @param[in]      样本数
  * @retval         返回空
  */
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order)
{
    OLS->Order = order;
    OLS->Count = 0;
    OLS->x = (float *)user_malloc(sizeof(float) * order);
    OLS->y = (float *)user_malloc(sizeof(float) * order);
    OLS->k = 0;
    OLS->b = 0;
    memset((void *)OLS->x, 0, sizeof(float) * order);
    memset((void *)OLS->y, 0, sizeof(float) * order);
    memset((void *)OLS->t, 0, sizeof(float) * 4);
}

/**
  * @brief          最小二乘法拟合
  * @param[in]      最小二乘法结构体
  * @param[in]      信号新样本距上一个样本时间间隔
  * @param[in]      信号值
  */
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }
    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;
}

/**
  * @brief          最小二乘法提取信号微分
  * @param[in]      最小二乘法结构体
  * @param[in]      信号新样本距上一个样本时间间隔
  * @param[in]      信号值
  * @retval         返回斜率k
  */
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k;
}

/**
  * @brief          获取最小二乘法提取信号微分
  * @param[in]      最小二乘法结构体
  * @retval         返回斜率k
  */
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k;
}

/**
  * @brief          最小二乘法平滑信号
  * @param[in]      最小二乘法结构体
  * @param[in]      信号新样本距上一个样本时间间隔
  * @param[in]      信号值
  * @retval         返回平滑输出
  */
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

/**
  * @brief          获取最小二乘法平滑信号
  * @param[in]      最小二乘法结构体
  * @retval         返回平滑输出
  */
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}
