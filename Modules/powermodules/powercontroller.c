/**
 * @file PowerController.c
 * @version 2.0
 * @note 估计功率公式 The estimated power formula: P = τΩ + k1|Ω| + k2τ^2
 */
#include "powercontroller.h"
#include "robot_def.h"
#include "pub_sub.h"
#include "Task_Init.h"
#include "SuperCAP.h"
#include "can_comm.h"
#include "user_lib.h"
#include "PID.h"
							 
							 
#define USE_SUPER_CAPACITOR 1
// 缓冲功率置信度
#define POWER_PD_KP 20.0f
#define POWER_PD_KD 0.55f
#define clamp(x, min, max) ((x < min) ? min : (x > max) ? max : x)

static RM_Status  isInitialized; // 初始化标志位 检测初始化成功
/* 功率控制主要变量 */
static Chassis_Manager_s Chassis_Manager;
// 限制的最大功率
static float maxPower = 0;
static float AllocatablePower;
/**  超级电容变量  **/
static CANCommInstance *SuperCAP_can_comm; // 超级电容通信can
       Supercap_TypeDef SuperCap_Rx;       // 超级电容通信信息
static uint16_t  SUPERCAP_CNT = 0;
/** RLS功率参数更新 **/
static RLS_update_t chassis_power_get;    // 发送功率数据结构体
static Subscriber_t *power_get_sub;       // 裁判系统反馈消息发布者

static uint8_t LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = 1;  // 机器人等级记录防止裁判系统通信丢失
static RM_Status isCapEnergyOut           = 0;         // 使用超电能量标志位
static float MIN_MAXPOWER_CONFIGURED      = 45.0f;     // 最低控制功率 随等级提升而提升

/* 超级电容在线的缓冲能量 */
const static float CAP_FULL_BUFFSET     = 100.0f;
const static float CAP_BASE_BUFFSET     = 40.0f;

/* 裁判系统缓冲能量 */
const float REFEREE_FULL_BUFFSET        = 60.0f;  // 裁判系统最大缓冲能量
const float REFEREE_BASE_BUFFSET        = 35.0f;  // 功率限制使用的基础缓冲能量  作为PID的期望使用缓冲功率
/* 功率重分配阈值(期望速度误差) */
const float error_powerDistribution_set = 25.0f;  // 功率分配在限制功率基础上增加
const float prop_powerDistribution_set  = 5.0f;  // 功率分配在限制功率基础上增加

/* 尝试超级电容动态功率参数 */
const float CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD = 43.0f;  // 为连接裁判系统时测试用 使能能量最低阈值
const float CAP_OFFLINE_ENERGY_TARGET_POWER           = 35.0f;  // 超电目标能量
static float MAX_CAP_POWER_OUT                        = 50.0f;  // 随超电反馈最大功率而变化

/* 部分系统不在线时限制功率缩小值 */
float REFEREE_GG_COE                            = 0.95f;
float CAP_REFEREE_BOTH_GG_COE                   = 0.85f;

/* 最大限制功率计算 */
static PID powerPD_base = {.Kp = POWER_PD_KP, .Ki = 0.0f, .Kd = POWER_PD_KD,      .interlimit = 100, .outlimit = 120, .DeadBand = 0.1f, .inter_threLow = 5, .inter_threUp = 20},
           powerPD_full = {.Kp = POWER_PD_KP, .Ki = 0.0f, .Kd = POWER_PD_KD,      .interlimit = 100, .outlimit = 120, .DeadBand = 0.1f, .inter_threLow = 5, .inter_threUp = 20},
		   powerPD_buff = {.Kp = 15,          .Ki = 0.0f, .Kd = POWER_PD_KD+0.2f, .interlimit = 100, .outlimit = 40,  .DeadBand = 0.1f, .inter_threLow = 5, .inter_threUp = 20};

/**
 * @brief 按兵种类型和级别划分的功率限制和最大HP枚举 
 * @note  Copy from RM2024 Official Rule Manual
 * @attention The infantry data list only suits for standard infantry, but not balanced infantry
 * @attention if the pilot changes the chassis type before the game officially start, and simultaneously the referee system is disconnected before
 *       chassis type changed, there will be problem of distinguishing the chassis type, so we choose HP_FIRST chassis type conservatively, except for
 *       sentry
**/
static uint8_t HeroPower[maxLevel]            = {55U, 60U, 65, 70U, 75U, 80U, 85U, 90U, 100U, 120U};
static uint8_t InfantryChassisPower[maxLevel] = {45U, 50U, 55U, 60U, 65U, 70U, 75U, 80U, 90U, 100U};
static uint8_t SentryChassisPowerLimit        = 100U;
void Receive_SuperCAP(CANInstance *_instance);

extern Referee_data_t Referee_SendData;              // 接收到裁判系统任务反馈

static inline RM_Status floatEqual(float a, float b) { return fabs(a - b) < 1e-5f; }

static inline float rpm2av(float rpm) { return rpm * M_PI / 30.0f; }

static inline float av2rpm(float av) { return av * 30.0f / (float)M_PI; }

static inline void setErrorFlag(uint8_t *curFlag, ErrorFlags_e setFlag) { *curFlag |= (uint8_t)(setFlag); }

static inline void clearErrorFlag(uint8_t *curFlag, ErrorFlags_e clearFlag) { *curFlag &= (~(uint8_t)(clearFlag)); }

static inline RM_Status isFlagged(uint8_t *curFlag, ErrorFlags_e flag) { return (*curFlag & (uint8_t)(flag)) != 0; }

static inline RM_Status isALLMotorConnected()
{
	for (uint8_t i = 0; i < 4; i++)
	  if (DJIMotor_detect(Chassis_Manager.chassis_motor[i]) == RM_ERROR)
		  return RM_ERROR;
	return RM_SUCCESS;
}

void Manager_Init(DJIMotor_Instance *motor[4], const Division_e division_, RLSEnabled_e rlsEnabled_, const float k1_, 
    	const float k2_, const float k3_)
{
	memset(&Chassis_Manager , 0, sizeof(Chassis_Manager_s));
	Chassis_Manager.Robot_Division = division_;
	Chassis_Manager.RLSEnableflag = rlsEnabled_;
	
	LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = division_ == SENTRY ? 10u : 1u;
	MIN_MAXPOWER_CONFIGURED = 30;
	Chassis_Manager.powerUpperLimit = CAP_OFFLINE_ENERGY_TARGET_POWER + MAX_CAP_POWER_OUT;
	
	Chassis_Manager.k1 = k1_;
	Chassis_Manager.k2 = k2_;
	Chassis_Manager.k3 = k3_ * 4.0f;
	Chassis_Manager.chassis_motor[0] = motor[0];
	Chassis_Manager.chassis_motor[1] = motor[1];
	Chassis_Manager.chassis_motor[2] = motor[2];
	Chassis_Manager.chassis_motor[3] = motor[3];
	// RLS初始化
	RLS_Init(&Chassis_Manager.rls, 2, 1e-5f, 0.998f);
	
	float initParams[2] = {k1_, k2_ };
//	Matrixf upfateParams;
//	Matrixf_Init(&upfateParams, 2, 1, initParams);
	
    // 初始化RLS参数矩阵
	RLS_setParamVector(&Chassis_Manager.rls, initParams);
	
	Chassis_Manager.torqueConst = TORQUECONST;

	/* 超电初始化 */
	CANComm_Init_Config_s Super_config = {
		.can_config = {
			.can_handle = &hcan2,
			.tx_id = 0x210,
			.rx_id = 0x211,
			.can_module_callback = Receive_SuperCAP,
		},
		.recv_data_len = sizeof(Supercap_TypeDef),
		.send_data_len = sizeof(uint16_t), // 发送使用SuperCAP.h文件里的SuperCAP_Send
		.dog_config = {
			.Max_num = 6,
			.dog_name = "SuperCAP",
		},
	};
	SuperCAP_can_comm = CANCommInit(Super_config);

#ifdef CHASSIS_BOARD
	xTaskCreate((TaskFunction_t)Task_PowerController,     "Task_PowerController", 128*8, NULL, 8, &Task_PowerController_Handle);
#endif
}

void setMaxPowerConfigured(Chassis_Manager_s *manager, float maxPower)
{
    manager->userConfiguredMaxPower = clamp(manager->powerUpperLimit, manager->fullMaxPower, manager->baseMaxPower);
}

void registerPowerCallbackFunc(Chassis_Manager_s *manager, float (*callback)(void))  { manager->callback = callback; }

void setRLSEnabled(Chassis_Manager_s *manager,uint8_t enable) { manager->RLSEnableflag = enable; }

//const volatile PowerStatus_s getPowerStatus(Chassis_Manager_s *manager) { return manager->powerStatus; }

float getLatestFeedbackJudgePowerLimit(Chassis_Manager_s *manager) { return manager->refereeMaxPower; }

/** 
 * @brief 功率重分配 添加在悬空轮子处理之后
 *
 **/
float newCmdPower_sum;
float *getControlledOutput(PowerObj_s *objs[4])
{
	// 电流输出缩放比例
	const float k0 = Chassis_Manager.torqueConst;
    // 返回输出
    static float newTorqueCurrent[6];
    // 计算总功率
    float sumCmdPower = 0.0f;
    float sumError = 0.0f;
	// 设置最大限制功率
	maxPower = (int16_t)Chassis_Manager.userConfiguredMaxPower;
  
	float allocatablePower = maxPower;   // 最终分配功率
    float sumPowerRequired = 0.0f;  

#if USER_POWER_DEBUG
    static float newCmdPower[4];
#endif
	for (uint8_t i = 0; i < 4; i++ )
	{
	    PowerObj_s *p = objs[i];
		if(DJIMotor_detect(Chassis_Manager.chassis_motor[i]) == RM_SUCCESS) 
		{   // 电机在线  PID计算后预测功率
			//                           Ti   *    Wi      +      |Wi|  *    |Wi|    *   k1                                Ti *         Ti        *        k2           +    k3     
            p->cmdPower = p->pidOutput * k0 * p->curAv + fabs(p->curAv) * Chassis_Manager.k1 + p->pidOutput * k0 * p->pidOutput * k0 * Chassis_Manager.k2  + Chassis_Manager.k3 / 4.0f;
			sumCmdPower += p->cmdPower;  // 先计算总功率包括负功
			if(floatEqual(p->cmdPower, 0.0f) || p->cmdPower < 0.0f)
			{
				allocatablePower += -p->cmdPower;   // 对零功率或负功处理
			}
			else 
			{
				sumError += p->errorAv; // 正功率处理
				sumPowerRequired += p->cmdPower;
		    }
		}
		else
		{  // 无法连接到电机
			p->cmdPower = 0.0f;
			p->errorAv = 0.0f;
		}
	}
	// update power status
	AllocatablePower = allocatablePower;
	Chassis_Manager.user_maxPower = maxPower;      // 限制最大功率
	Chassis_Manager.sum_Power = sumPowerRequired;  // 整体功率
    float Power_K[4]={0};
	// 功率超出最大功率 进行功率分配
	if(sumCmdPower > maxPower) // 比较 sumPowerRequired  与  sumCmdPower那个更适合
	{
		float errorConfidence; // 功率补偿
		if (sumError > error_powerDistribution_set)  // 超出规定功率阈值
		   errorConfidence = 1.0f;                                                                                                                                     
		else if (sumError > prop_powerDistribution_set) // 未超出阈值但是超出限制功率
			errorConfidence = clamp((sumError - prop_powerDistribution_set)
		                          / (error_powerDistribution_set - prop_powerDistribution_set), 0.0f, 1.0f);
		else
			errorConfidence = 0.0f;
		for (uint8_t i = 0; i < 4 ; i++)
		{
			if (DJIMotor_detect(Chassis_Manager.chassis_motor[i]) == RM_SUCCESS) {
				PowerObj_s *po = objs[i];
                if (floatEqual(po->cmdPower, 0.01f) || po->cmdPower < 0.0f)
				{
				  newTorqueCurrent[i] = po->pidOutput; // 零功率或负功正常输出即可
				  continue;
				}
				/* 功率重分配 */
				float powerWeight_Error = po->errorAv / sumError;
				float powerWetght_Prop  = po->cmdPower / sumPowerRequired;
				float powerWeight       = errorConfidence * powerWeight_Error + (1.0f - errorConfidence) * powerWetght_Prop;
				float delta             = po->curAv * po->curAv - 4.0f * Chassis_Manager.k2 * (Chassis_Manager.k1 * fabs(po->curAv) + Chassis_Manager.k3 / 4.0f - powerWeight * allocatablePower);
				if (floatEqual(delta, 0.0f))   // 重复根 repeat roots
					newTorqueCurrent[i] = -po->curAv / (2.0f * Chassis_Manager.k2 ) / k0;                                  
				else if (delta > 0.0f)   // 不同的根：distinct roots
					newTorqueCurrent[i] = po->pidOutput > 0.0f ?  (-po->curAv + sqrtf(delta)) / (2.0f * Chassis_Manager.k2 ) / k0
					                                           :  (-po->curAv - sqrtf(delta)) / (2.0f * Chassis_Manager.k2 ) / k0;
				else 
					newTorqueCurrent[i] = -po->curAv / (2.0f * Chassis_Manager.k2 ) / k0; // imaginary roots  虚解 （或可以直接赋值为0）
				newTorqueCurrent[i] = clamp(newTorqueCurrent[i], -po->pidMaxOutput, po->pidMaxOutput); //限制最大输出
			} else {
				newTorqueCurrent[i] = 0;
			}				
		}
	}
    else
	{
		for(uint8_t i = 0 ; i < 4; i++)
		{
			if ( DJIMotor_detect(Chassis_Manager.chassis_motor[i]) == RM_SUCCESS )
				newTorqueCurrent[i] = objs[i]->pidOutput;
			else
				newTorqueCurrent[i] = 0.0f;
		}
	}
#if USER_POWER_DEBUG
	newCmdPower_sum = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        PowerObj_s *p = objs[i];
        newCmdPower[i] = newTorqueCurrent[i] * k0 * p->curAv + fabs(p->curAv) * Chassis_Manager.k1 
		               + newTorqueCurrent[i] * k0 * newTorqueCurrent[i] * k0 * Chassis_Manager.k2  + Chassis_Manager.k3 / 4.0f;
        if(newCmdPower[i] > 0)
		    newCmdPower_sum += newCmdPower[i];
    }
	limit(newCmdPower_sum , 150, -150);
	newTorqueCurrent[4] = sumCmdPower;
	newTorqueCurrent[5] = newCmdPower_sum;
#endif
	return newTorqueCurrent;  // 返回四个电机PID输出值
}

static  void changeErrorFlag(Chassis_Manager_s *manager)
{
    /*Judge the error status*/
#if USE_SUPER_CAPACITOR
    if( SuperCAP_can_comm->Dog->state != Dog_Online )  // 超级电容不在线
        setErrorFlag(&manager->error, CAPDisConnect);
    else
        clearErrorFlag(&manager->error, CAPDisConnect);
#else
    setErrorFlag(&manager->error, CAPDisConnect);
#endif
	if(chassis_power_get.refree_status != Device_Online ) // 裁判系统不在线
      setErrorFlag(&manager->error, RefereeDisConnect);
    else
	  clearErrorFlag(&manager->error, RefereeDisConnect);	
	if ( isALLMotorConnected())
      setErrorFlag(&manager->error, MotorDisconnect);
    else 
      clearErrorFlag(&manager->error, MotorDisconnect);
}

/**
 * 功率RLS解算任务
 *
**/
float delta_tim; // 超级电容计算消耗总能量
extern int16_t Can2Send_Chassis[4];
void Task_PowerController(void *pvParameters)
{
	static Matrixf samples;   // 记录数据矩阵 观测矩阵
	static Matrixf params;   // 参数矩阵
	static float effectivePower = 0;
	static float super_energy_flag;
	super_energy_flag = HAL_GetTick();

	Matrixf_zeros(&samples, 2, 1);
	Matrixf_zeros(&params , 2, 1);
	
	isInitialized = RM_SUCCESS;
	
    power_get_sub = SubRegister("rlssend", sizeof(RLS_update_t));
	static portTickType currentTime;
	
	for (;;) 
	{
		currentTime = xTaskGetTickCount();
		/**  获取裁判系统信息  **/
		SubGetMessage(power_get_sub, (void *)&chassis_power_get);
	    changeErrorFlag(&Chassis_Manager); // 离线检测
		
        // model
        // estimate the cap energy if cap disconnect
        // estimated cap energy = cap energy feedback when cap is connected
#if USE_SUPER_CAPACITOR == 1
		if( !isFlagged(&Chassis_Manager.error, CAPDisConnect)) 
		{			// 超电在线
			MAX_CAP_POWER_OUT = SuperCap_Rx.cap_power;
			isCapEnergyOut = TRUE;
		    Chassis_Manager.RLSEnableflag = RLS_Enable;
			/* RLS更新参数不再需要裁判系统返回功率，不再判断裁判系统连接状态 */
//			if( !isFlagged(&Chassis_Manager.error, RefereeDisConnect)) 
//		     {                                                   // 裁判系统在线
//				isCapEnergyOut = TRUE;
//				Chassis_Manager.RLSEnableflag = RLS_Enable;
//			} 
//			else 
//			{
//				isCapEnergyOut = FALSE;    // 裁判系统离线
//				Chassis_Manager.RLSEnableflag = RLS_Disable; // 取消RLS更新
//			}
		}
		else 
		{
			MAX_CAP_POWER_OUT = 0;
            isCapEnergyOut = FALSE;  // 超级电容与电路断开，禁用rls更新
			Chassis_Manager.RLSEnableflag = RLS_Disable;
		}		
#else    
		if (!isFlagged(&Chassis_Manager.error, RefereeDisConnect)) 
           Chassis_Manager.RLSEnableflag = RLS_Disable;  // 裁判系统在线
		else 
			Chassis_Manager.RLSEnableflag = RLS_Disable;
				
		isCapEnergyOut = FALSE; // 超电不使能
#endif
// 根据当前状态设置缓冲功率和基于当前状态的Buff设置  Set the power buff and buff set based on the current state
// 以裁判系统的功率缓冲功率作为反馈  take the referee system's power buffer as feedback
// 如果裁判系统断开，那么我们需要关闭能量回路并保守处理电源回路  If referee system is disconnected, then we need to disable the energy loop and treat power loop conservatively
// 当cap和裁判都断开时，我们禁用能量循环，因此不需要更新powerBuff和buffSet   When both cap and referee are disconnected, we disable the energy loop and therefore no need to update the powerBuff and buffSet
        // 获取裁判系统缓冲功率
#if USE_SUPER_CAPACITOR == 1 // 使用超电
	    if( !isFlagged(&Chassis_Manager.error, CAPDisConnect) ) {
			isCapEnergyOut = TRUE;
			Chassis_Manager.powerBuff = SuperCap_Rx.cap_power;
		    
			Chassis_Manager.fullBuffSet = CAP_FULL_BUFFSET;
		    Chassis_Manager.baseBuffSet = CAP_BASE_BUFFSET;
		} else if ( !isFlagged(&Chassis_Manager.error, RefereeDisConnect) ) {
			isCapEnergyOut = FALSE;
			Chassis_Manager.powerBuff = chassis_power_get.send_power.power_buffer; // 裁判系统在线获得缓冲能量
	       	
			Chassis_Manager.fullBuffSet = REFEREE_FULL_BUFFSET;
			Chassis_Manager.baseBuffSet = REFEREE_BASE_BUFFSET;		
		} else
			Chassis_Manager.powerBuff = 0;
#else  
       if ( !isFlagged(&Chassis_Manager.error, RefereeDisConnect) )
			Chassis_Manager.powerBuff = chassis_power_get.send_power.power_buffer; // 裁判系统在线获得缓冲能量
		else
			Chassis_Manager.powerBuff = 0;
		
		Chassis_Manager.fullBuffSet = REFEREE_FULL_BUFFSET;
		Chassis_Manager.baseBuffSet = REFEREE_BASE_BUFFSET;
#endif		
//  Set the energy target based on the current error status

        // Update the referee maximum power limit and user configured power limit
        // If disconnected, then restore the last robot level and find corresponding chassis power limit
        if ( !isFlagged(&Chassis_Manager.error, RefereeDisConnect)) 
		{
			//  裁判系统在线 得到机器人等级 设置能量限制
		  Chassis_Manager.refereeMaxPower = fmax(chassis_power_get.send_power.power_limit, CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD);
		    //  机器人等级计算错误
		  if(chassis_power_get.robot_level > 10u)  
			  LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = 1u;
		  else 
			  LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = fmax(1u, chassis_power_get.robot_level);
			 /**  设置最大限制功率  **/
#if USE_SUPER_CAPACITOR == 1 // 使用超电
		  if ( !isFlagged(&Chassis_Manager.error, CAPDisConnect) && SuperCap_Rx.cap_power >= 65.0f)  // 超电在线使用缓冲能量
			  chassis_power_get.shitf_flag == 1 ? ( Chassis_Manager.powerUpperLimit = Chassis_Manager.refereeMaxPower + MAX_CAP_POWER_OUT - 0.6f * PID_Control(sqrtf(chassis_power_get.send_power.power_buffer), sqrtf(REFEREE_BASE_BUFFSET), &powerPD_buff))
		  : ( Chassis_Manager.powerUpperLimit = Chassis_Manager.refereeMaxPower + MAX_CAP_POWER_OUT - 0.6f * PID_Control(sqrtf(chassis_power_get.send_power.power_buffer), sqrtf(REFEREE_BASE_BUFFSET), &powerPD_buff));
		  else // 超电离线保守控制
			  Chassis_Manager.powerUpperLimit = Chassis_Manager.refereeMaxPower - PID_Control(sqrtf(chassis_power_get.send_power.power_buffer), sqrtf(REFEREE_BASE_BUFFSET), &powerPD_buff);
#else
		  Chassis_Manager.powerUpperLimit = Chassis_Manager.refereeMaxPower - PID_Control(sqrtf(chassis_power_get.send_power.power_buffer), sqrtf(REFEREE_BASE_BUFFSET), &powerPD_buff);
#endif
		  // 裁判系统不在线 使用离线前记录的等级限制功
		} else {      // 裁判系统离线后处理
		    switch (Chassis_Manager.Robot_Division) 
		    {
			case HERO:
				Chassis_Manager.refereeMaxPower = HeroPower[LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL - 1];
			    break;
			case INFANTRY:
				Chassis_Manager.refereeMaxPower = InfantryChassisPower[LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL - 1];
			    break;
			case SENTRY:
				Chassis_Manager.refereeMaxPower = SentryChassisPowerLimit;
			    break;
		 	default: 
				Chassis_Manager.refereeMaxPower = CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD;
			    break;
		    }
// Since we have less available feedback, we constrain the power conservatively
#if USE_SUPER_CAPACITOR == 1
		  if (!isFlagged(&Chassis_Manager.error, CAPDisConnect)) 
			Chassis_Manager.powerUpperLimit = Chassis_Manager.refereeMaxPower + MAX_CAP_POWER_OUT;
		   else
			Chassis_Manager.powerUpperLimit = Chassis_Manager.refereeMaxPower * CAP_REFEREE_BOTH_GG_COE; // 超电离线
#else
		  Chassis_Manager.powerUpperLimit  = Chassis_Manager.refereeMaxPower * CAP_REFEREE_BOTH_GG_COE ; // 超电和裁判系统离线 将最大能量 * 0.85F
#endif
		}
		
		// 更新最低控制功率
		MIN_MAXPOWER_CONFIGURED = Chassis_Manager.refereeMaxPower * 0.85f;
		
		//   能量循环
		/** 如果超电和裁判系统同时离线, 将最大功率设置为最新功率限制* 0.85f,并禁用能量循环
			如果裁判系统离线, 将最大功率设置为最新功率限制 * 0.95f,当超电能量低时使能能量循环 **/
		if(isFlagged(&Chassis_Manager.error, CAPDisConnect) && isFlagged(&Chassis_Manager.error, RefereeDisConnect)) {
			// 裁判系统和超级电容同时离线
		   Chassis_Manager.baseMaxPower = Chassis_Manager.fullMaxPower = Chassis_Manager.powerUpperLimit * CAP_REFEREE_BOTH_GG_COE;
		   PID_IoutReset(&powerPD_base);
		   PID_IoutReset(&powerPD_full);
		} else{ //  裁判系统和超级电容同时在线 使用PID计算充分使用缓冲功率
		       Chassis_Manager.baseMaxPower =
		   	      fmax(Chassis_Manager.refereeMaxPower - PID_Control(sqrtf(Chassis_Manager.powerBuff), sqrtf(Chassis_Manager.baseBuffSet), &powerPD_base), MIN_MAXPOWER_CONFIGURED);
		       Chassis_Manager.fullMaxPower =
		          fmax(Chassis_Manager.refereeMaxPower - PID_Control(sqrtf(Chassis_Manager.powerBuff) ,sqrtf(Chassis_Manager.fullBuffSet), &powerPD_full), MIN_MAXPOWER_CONFIGURED);
	           if(chassis_power_get.shitf_flag)
				   Chassis_Manager.baseMaxPower += 40;
		}

		//  自定义功率曲线（ 暂未使用 ）
		if (Chassis_Manager.callback != NULL)
			setMaxPowerConfigured(&Chassis_Manager, Chassis_Manager.callback());
		else
           setMaxPowerConfigured(&Chassis_Manager, 300);
		
		// 计算预估功率
		// Estimate the power based on the current model
		effectivePower = 0;
		samples.arm_mat_.pData[0] = 0;
		samples.arm_mat_.pData[1] = 0;
		for(uint8_t i = 0; i < 4; i++)
		{
		  if(DJIMotor_detect(Chassis_Manager.chassis_motor[i]))                                                                                                           
		  {
			effectivePower  += Chassis_Manager.torqueConst * Chassis_Manager.chassis_motor[i]->measure.CurrentFilter * (Chassis_Manager.chassis_motor[i]->measure.SpeedFilter*PI/30.0f);
			samples.arm_mat_.pData[0] += fabs(Chassis_Manager.chassis_motor[i]->measure.SpeedFilter/30.0f*PI);
			samples.arm_mat_.pData[1] += Chassis_Manager.chassis_motor[i]->measure.TorqueCurrent * Chassis_Manager.chassis_motor[i]->measure.TorqueCurrent * Chassis_Manager.torqueConst * Chassis_Manager.torqueConst;
		  }
		}
        Chassis_Manager.estimatedPower = Chassis_Manager.k1 * samples.arm_mat_.pData[0] + Chassis_Manager.k2 * samples.arm_mat_.pData[1] + effectivePower + Chassis_Manager.k3 ;
		  // 根据在线情况选取RLS拟合数据
		  // If cap is disconnected, get measured power from referee feedback  todo: 从超级电容中获取功率
		  // Otherwise, set it to estimated power	
		 /* 2025赛季裁判系统不再返回底盘消耗功率，以后使用超级电容返回功率作为测量值（而且超级电容返回功率更加准确） */
		if(!isFlagged(&Chassis_Manager.error, CAPDisConnect)) {
			Chassis_Manager.measuredPower = SuperCap_Rx.out_p; // 超电在线使用超电返回功率
		} else  Chassis_Manager.measuredPower = Chassis_Manager.estimatedPower; // 超级电容离线  使用预测功率(不进行RLS更新参数)
        
        /* 更新功率状态  update power status 测试用*/
//		 Chassis_Manager.powerStatus.userConfiguredMaxPower = Chassis_Manager.userConfiguredMaxPower;                          // 用户配置最大功率
//		 Chassis_Manager.powerStatus.effectivePower         = effectivePower;                                                  // 有效功率
//		 Chassis_Manager.powerStatus.powerLoss              = Chassis_Manager.measuredPower - effectivePower;                  // 无用功率
//		 Chassis_Manager.powerStatus.efficiency             = clamp(effectivePower/Chassis_Manager.measuredPower, 0.0f, 1.0f); // 有&效功率比例
//		 Chassis_Manager.powerStatus.estimatedCapEnergy     = Chassis_Manager.estimatedCapEnergy = clamp( chassis_power_get.send_power.power_limit- 2 + 0.25f*chassis_power_get.send_power.power_buffer, 43, 120);// 给超级电容发送期望功率 
//		 Chassis_Manager.powerStatus.Errorflags             = (ErrorFlags_e )Chassis_Manager.error;
        
		 // 更新RLS参数 添加死区 因为裁判系统无法检测到负功率，导致实际测量失败 所以用估计的功率来评估这种情况
//		if (Chassis_Manager.RLSEnableflag == RLS_Enable && Chassis_Manager.measuredPower > 5.0f && effectivePower < 0) //  
//		{
//			params = RLS_Update(&Chassis_Manager.rls, &samples, Chassis_Manager.measuredPower - effectivePower - Chassis_Manager.k3);
//			Chassis_Manager.k1 = fmax(params.arm_mat_.pData[0], 1e-5f);  // In case the k1 diverge to negative number
//			Chassis_Manager.k2 = fmax(params.arm_mat_.pData[1], 1e-5f);  // In case the k2 diverge to negative number
//		}
		
	   /** 向超级电容更新功率和缓冲能量 **/
		if(SuperCap_Rx.cap_percent == 0 && SuperCap_Rx.cap_v != 0)
		{    // 超电离线重启
			 if(SUPERCAP_CNT++ == 0)
				 SuperCAP_Force2Restart(&hcan2, 0x210);
			 if(SUPERCAP_CNT >= 2000)
				 SUPERCAP_CNT = 0;
		}
		else
		{
			SUPERCAP_CNT = 0;
            if(isCapEnergyOut == TRUE)
               SuperCAP_Send(&hcan2, 0x210, chassis_power_get.send_power.power_limit - 5, chassis_power_get.send_power.power_buffer); // 超电正常运行
	    }
		
        vTaskDelayUntil(&currentTime, 2);
	}
}

/**
 * @brief 超电消息接收
 *
**/
void Receive_SuperCAP(CANInstance *_instance)
{
	CANCommInstance *comm = (CANCommInstance *)_instance->id;
	SuperCAP_Receive(&SuperCap_Rx, _instance->rx_buff);
	comm->recv_state = 0;
	comm->cur_recv_len = 0;
	comm->update_flag = 1;
	Feed_Dog(comm->Dog);
}