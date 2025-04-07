#ifndef __DMMOTOR_H
#define __DMMOTOR_H
/**
  * 1 传统MIT模式
  * 2 达妙电机一拖四模式(仿大疆电机)
 **/
#define DMMOTORCTRLMODE 2

#include "motor_def.h" 
#include "user_lib.h"

/* 电机最大值 */
#define DM_MOTOR_CNT 4
/* 达妙电机参数设置 */
#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f
// 一圈编码器刻度
#define Encoder_Num_Per_Round 8192
// 电流到输出的转化系数
#define Current_To_Out (16384.0f/10.261194f)
// 理论最大输出电流
#define Theoretical_Output_Current_Max 10.261194f

/* 滤波系数设置为1的时候即关闭滤波 */  
#define DMMOTOR_SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define DMMOTOR_CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DM 0.0439453125f      // (360/8192),将编码器值转化为角度制

/**
 * @brief 达妙电机控制状态, 传统模式有效
 *
 */
typedef enum 
{
    Motor_DM_Control_Status_DISABLE = 0x0,
    Motor_DM_Control_Status_ENABLE,
    Motor_DM_Control_Status_OVERVOLTAGE = 0x8,
    Motor_DM_Control_Status_UNDERVOLTAGE,
    Motor_DM_Control_Status_OVERCURRENT,
    Motor_DM_Control_Status_MOS_OVERTEMPERATURE,
    Motor_DM_Control_Status_ROTOR_OVERTEMPERATURE, // 电机过热
    Motor_DM_Control_Status_LOSE_CONNECTION,       // 连接断开
    Motor_DM_Control_Status_MOS_OVERLOAD,          // 电机过载
}DM_Control_Status_Normal;

/**
* @brief 达妙电机控制方式
*/
typedef enum 
{
   Motor_DM_Control_Method_NORMAL_MIT = 0,
   Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,
   Motor_DM_Control_Method_NORMAL_OMEGA,
   Motor_DM_Control_Method_NORMAL_EMIT,
   Motor_DM_Control_Method_1_TO_4,
}DM_Control_Method_e;

/**
* @brief 达妙电机控制帧发送
*/
typedef struct
{
	// 角度, rad, 目标角度
    float Control_Angle;
    // 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
    float Control_Omega;
    // 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
    float Control_Torque;
    // 电流, A, EMIT模式是限幅, 其余模式无效
    float Control_Current;
    // K_P, 0~500, MIT模式有效
    float K_P;
    // K_D, 0~5, MIT模式有效
    float K_D;
}DMMotor_Send_s;

/**
* @brief 达妙电机指令 传统模式使用
*/
typedef enum {
	DM_CMD_MOTOR_MODE = 0xfc, //使能，会响应指令
	DM_CMD_RESET_MODE = 0xfd, //停止
	DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
	DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;


/**
 * @brief 达妙电机常规源数据, 位置速度控制报文
 *
 */
typedef struct
{
    float Control_Angle;
    float Control_Omega;
} DM_Tx_Data_Normal_Angle_Omega;

/**
 * @brief 达妙电机常规源数据, 速度控制报文
 *
 */
typedef struct
{
    float Control_Omega;
} DM_Tx_Data_Normal_Omega;

/**
 * @brief 达妙电机常规源数据, EMIT控制报文
 *
 */
typedef struct {
    float Control_Angle;
    // 限定速度用, rad/s的100倍
    uint16_t Control_Omega;
    // 限定电流用, 电流最大值的10000倍
    uint16_t Control_Current;
} DM_Tx_Data_Normal_EMIT;

#pragma pack(1)
/** 电机接收数据 传统模式 **/
typedef struct {  
	DM_Control_Status_Normal state;  // 当前电机状态
    uint8_t id;
    float Velocity;
    float Position;
    float Torque;
    float Mos_Temp;
    float Rotor_Temp;

	float Last_Position;
    int32_t Total_Position;
    int32_t Total_round;
	uint16_t Pre_encoder;
	uint8_t flag;
} DMMotor_Measure_Normal;

typedef struct {
	DMMotor_Measure_Normal measure;
	DM_Control_Method_e motor_mode;
	DMMotor_Send_s motor_set;
   	WatchDog_TypeDef* watchdog;        // 电机看门狗
    CANInstance* motor_can_instance;   // 电机CAN实例

   	Controller_s *motor_controller;
   	Motor_Working_Type_e stop_flag;  // 电机启停标志
	
    /* 分组发送设置 */
    uint8_t message_num;
    float *other_angle_feedback_ptr;   // 角度反馈数据指针,注意电机使用total_angle
    float *other_speed_feedback_ptr;   //速度反馈数据指针,单位为angle per sec
}DMMotor_Instance_Normal;

typedef struct {
    uint16_t MchanicalAngle;   // 机械角度
    float Speed;               // 速率
    float TorqueCurrent; 	   // 力矩电流
    int16_t Mos_temperature;   // 线圈温度
	int16_t ERROR_Handle;      // 电机错误状态
	// 处理后数据
    uint16_t LastMchanicalAngle; // 上次位置
	float SpeedFilter;         // 滤波后转速 /rpm
    float CurrentFilter;       // 滤波后转矩电流
    int16_t Angle;             // 连续化角度
    int16_t LastAngle;         // 上次连续化角度
    float Angle_DEG;           // 连续化角度制角度

    int16_t round;             // 圈数
	uint16_t MotorCenter;
	uint8_t flag;
} DMMotor_Measure_1_4;

typedef struct {
	DMMotor_Measure_1_4 measure;
   	WatchDog_TypeDef* watchdog;        // 电机看门狗
    CANInstance* motor_can_instance;   // 电机CAN实例

   	Controller_s *motor_controller;
   	Motor_Working_Type_e stop_flag;  // 电机启停标志
	
    /* 分组发送设置 */
    uint8_t sender_group;
    uint8_t message_num;
    float *other_angle_feedback_ptr;   // 角度反馈数据指针,注意电机使用total_angle
    float *other_speed_feedback_ptr;   //速度反馈数据指针,单位为angle per sec
}DMMotor_Instance_1_4;
#pragma pack()

DMMotor_Instance_Normal *DMMotorInit_Normal(Motor_Init_config_s *_config,DM_Control_Method_e motor_mode);

DMMotor_Instance_1_4 *DMMotorInit_1_4(Motor_Init_config_s *_config);

void DMMotorSetRef(DMMotor_Instance_1_4*motor, float ref);
/* 电流只能通过电机自带传感器监测*/
void DMMotorChangeFeed(DMMotor_Instance_1_4 *motor, Feedback_Source_e type);
void DMMotorOuterLoop(DMMotor_Instance_1_4 *motor, Closeloop_Type_e outer_loop);
void DMMotorEnable(DMMotor_Instance_1_4 *motor);
void DMMotorStop(void *id);
void DMMotorReceive(DMMotor_Instance_1_4 *motor, uint8_t buf[8]);
//void DMMotorCailEncoder(DMMotor_Instance *motor);
//void DMMotorControlInit();
void DMMmotor_Update();

#endif
