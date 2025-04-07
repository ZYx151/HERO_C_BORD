#ifndef __POWERCONTROLLER_H
#define __POWERCONTROLLER_H

#include "DJIMotor.h"
#include "Referee_unpack.h"
#include "Matrix.h"
#include "RLS.h"
#include "user_lib.h"

#define M_PI	3.1415926f
#define maxLevel 10
#define USER_POWER_DEBUG 1

//m3508转矩电流(-16384~16384)转为成电机输出转矩(N.m)的比例
#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.000366211f // 转矩电流转化为转矩（未加减速比）

#define K_A 0.0156223f  // ( 0.3f * 187.0f / 3591.0f )  // 把3508反馈电流(A)转化为转矩 = c * 3508减速比   c = 0.3为3508转矩电流常数(N.m/A)
#define I_T 0.0011880717595f  // ( 20.0f / 16384.0f)  发送C620的转矩电流信号转化为电流 (-20A ~ 20A)
#define TORQUECONST 1.90703e-5// 发送C620的转矩电流信号转化为扭矩 转矩（N.m）  (20/16384)*(0.3)*(187/3591)

typedef enum {                                                   
	lf = 0,
	lb = 1,
	rb = 2,
	rf = 3,
}chassis_wheel_e;

typedef enum 
{
    INFANTRY = 0,
    HERO,
    SENTRY
}Division_e;

typedef enum {
	RLS_Disable = 0,
	RLS_Enable  = 1
} RLSEnabled_e;

typedef enum {
	MotorDisconnect   = 1U,
	RefereeDisConnect = 2U,
	CAPDisConnect     = 4U
}ErrorFlags_e;

typedef struct {
    float pidOutput;     // torque current command, [-maxOutput, maxOutput], no unit
    float curAv;         // 测量角速度 电机返回速度  Measured angular velocity, [-maxAv, maxAv], rad/s
    float setAv;         // 目标角速度 target angular velocity, [-maxAv, maxAv], rad/s
    float cmdPower;      // 预测电机消耗功率
	float pidMaxOutput;  // pid max output
	float errorAv;
	uint8_t motor_state;  // 电机状态
}PowerObj_s;

/**
 * @brief 存储底盘的功率状态
 */
typedef struct {
    float userConfiguredMaxPower;
    float maxPowerLimited;
    float sumPowerCmd_before_clamp;
    float effectivePower;
    float powerLoss;
    float efficiency;
    uint8_t estimatedCapEnergy;
	ErrorFlags_e Errorflags;    // Chassis_Manager中的Errorflags
}PowerStatus_s;

typedef struct {
	Division_e   Robot_Division;
    RLSEnabled_e RLSEnableflag;
	ErrorFlags_e Errorflags;
	
//	PowerObj_s powerObj;
//	PowerStatus_s powerStatus;
	float user_maxPower;
	float sum_Power;
	/** 底盘电机 **/
    DJIMotor_Instance *chassis_motor[4];
	
	/** 参数 **/
    uint8_t error;
    /**
     * @brief The constructor of the power manager object
     * @param division_             The type of robot
     * @param rlsEnabled_           Enable or disable the RLS adaptive param mode
     * @param torqueConst_          The torque const (KA) of the motor, measured by (N.m / A)
     * @param k1_                   The frequency-dissipate params on the power estimation motor
     * @param k2_                   The current-dissipate's square params on the power estimation motor
     * @param k3_                   The constant power loss
     * @param lambda_               The RLS update forgetting factor
     */
	float powerBuff;
	float fullBuffSet;
	float baseBuffSet;
	float fullMaxPower;
	float baseMaxPower; 
	
	float powerUpperLimit;   // 功率最大限制
	float refereeMaxPower;   // 裁判系统返回最大功率
	
	float userConfiguredMaxPower; // 配置最大功率
	float (*callback)(void);
	
    float measuredPower;      // 实际测量功率
    float estimatedPower;     // 预估功率
    float estimatedCapEnergy; // 预估超电能量
	
	float torqueConst; // 力矩配置
	
	float k1;  // P = tau_i * omega_i + k_1 * omega_i
	float k2;  // + k_2 * tau_i^2 + 
	float k3;  // + k_3    车体无输出时的底盘总功率 由裁判系统返回 k_3
	
	RLS_s rls;       // dim = 2
} Chassis_Manager_s;

// Chassis_Manager构造函数
//Chassis_Manager_s* Manager_Init(DJIMotor_Instance *motor[4], const Division_e division_, RLSEnabled_e rlsEnabled_, const float k1_, const float k2_, const float k3_, const float lambda_);
void Manager_Init(DJIMotor_Instance *motor[4], const Division_e division_, RLSEnabled_e rlsEnabled_, const float k1_, const float k2_, const float k3_);

// return the latest feedback referee power limit(before referee disconnected), according to the robot level
float getLatestFeedbackJudgePowerLimit(Chassis_Manager_s *manager);

/**
 * @brief Get the controlled output torque current based on current model
 * @param objs The collections of power objects from four wheels, recording the necessary data from the PID controller
 * @retval The controlled output torque current
 */
float *getControlledOutput( PowerObj_s *objs[4]);
/**
 * @brief return the power status of the chassis
 * @retval The power status object
 */
const volatile PowerStatus_s getPowerStatus(Chassis_Manager_s *manager);

/**
 * @brief The power controller module initialization function
 * @param manager The manager object
 * @note This function should be called before the scheduler starts
 */
//void init(const Chassis_Manager_s manager);

/**
 * @brief set the user configured max power
 * @param maxPower The max power value
 * @note The max power configured by this function will compete with the basic energy limitation, to ensure system does not die
 */
void setMaxPowerConfigured( Chassis_Manager_s *manager, float maxPower);

/**
 * @brief set the user power mode
 */
void setMode(uint8_t mode);

/**
 * @brief set the user manage callback
 */
void registerPowerCallbackFunc(Chassis_Manager_s *manager, float (*callback)(void));

/**
 * @brief Enable for disable the automatically parameters update process
 * @param isUpdate disable with 0, enable with 1
 * @note  The system will automatically disable the update when both referee system and cap is disconnect from the power module
 * @retval None
 */
void setRLSEnabled(Chassis_Manager_s *manager,uint8_t enable);

/**
 * @brief 功率重分配任务
 */
 void Task_PowerController(void *pvParameters);

#endif

