#ifndef _MOTOR_DEF_H_
#define _MOTOR_DEF_H_

#include "Controller.h"
#include "bsp_can.h"
#include "WatchDog.h"
#include "PID.h"
#include "motor.h"

// rpm换算到rad/s
#define RPM_TO_RADPS (2.0f * PI / 60.0f)
// deg换算到rad
#define DEG_TO_RAD (PI / 180.0f)
// 摄氏度换算到开氏度
#define CELSIUS_TO_KELVIN (273.15f)

/**
 * @brief 闭环类型,如果需要多个闭环,则使用或运算
 *        例如需要速度环和电流环: CURRENT_LOOP|SPEED_LOOP
 */
typedef enum
{
    NONE_LOOP = 0,     // 单环
	CURRENT_LOOP = 1,  // 电流环
    SPEED_LOOP = 2,    // 速度环
    ANGLE_LOOP = 3,    // 位置环
}Closeloop_Type_e;

/* 反馈来源设定,若设为OTHER_FEED则需要指定数据来源指针(如陀螺仪角度) */
typedef enum
{
    MOTOR_FEED = 0,
    OTHER_FEED,
} Feedback_Source_e;

typedef enum
{
    NONE_FEEDFORWARD= 0x00,       // 不使能前馈
    SPEED_FEEDFORWARD = 0x01,    // 速度环前馈
    ANGLE_FEEDFORWARD = 0x10,      // 角度环前馈
} Feedfoward_Type_e;

/* 使能外环 */
typedef enum
{
    LOOP_DISABLE = 0,
    LOOP_ENABLE,
} Motor_LoopFlag_e;

/* 电机正反转标志 */
typedef enum
{
    MOTOR_NORMAL = 0,
    MOTOR_REVERSE = 1
} Motor_Reverse_Flag_e;

/* 反馈量正反标志 */
typedef enum
{
    FEEDBACK_NORMAL = 0,
    FEEDBACK_REVERSE = 1
} Feedback_Reverse_Flag_e;

typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_ENALBED = 1,
	MOTOR_CLOSE = 2,
} Motor_Working_Type_e;

typedef enum
{ 
    PID_MODEL = 0, 
    MRAC_MODEL, 
    ADRC_MODEL, 
    SMC_MODEL
} controller_type_e;

/* 电机类型枚举 */
typedef enum {
    MOTOR_TYPE_NONE = 0,
    GM6020,
    RM3508,
    M2006,
    LK9025,
    DM4310,
} Motor_Typed_e;
#pragma pack(1)
/* 电机控制设置,包括闭环类型,反转标志和反馈来源 */
typedef struct {
    Motor_Reverse_Flag_e motor_reverse_flag;       // 电机是否反转
    Feedback_Reverse_Flag_e feedback_reverse_flag; // 反馈是否反向
    Feedback_Source_e angle_feedback_source;       // 角度反馈类型
	Feedback_Source_e speed_feedback_source;       // 速度反馈类型
	controller_type_e control_mode;                // 控制模式	
    Closeloop_Type_e close_loop_type;              // 使用闭环类型 默认位置环位最外环
	Feedfoward_Type_e feedfoward_type;             // 使用前馈类型
} Control_Setting_s;
#pragma pack()

#pragma pack(1)

/* 电机控制器,包括其他来源的反馈数据指针,3环控制器和电机的参考输入*/
typedef struct {
	Control_Setting_s setting_config;        // 控制设置
	uint16_t MotorCenter;
} controller_Init_config_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
   	Control_Setting_s motor_setting;        // 控制设置
	float output;
    float ref_speed;
    float ref_current;
    float ref_position;
    float fdb_current;
    float fdb_speed;
    float fdb_position;
	// 将会作为每个环的输入和输出顺次通过串级闭环
} Controller_s;
#pragma pack()

#pragma pack(1)
/* 用于初始化CAN电机的结构体,各类电机通用 */
typedef struct
{
    Motor_Typed_e motor_type;
	controller_Init_config_t contruller_config;    // 电机控制器
	CAN_Init_Config_s can_init_config;             // CAN结构体 用于创建CANInstance
    WatchDog_Init_config dog_init_config;          
	
    float *other_angle_feedback_ptr;   // 角度反馈数据指针
    float *other_speed_feedback_ptr;   // 速度反馈数据指针
} Motor_Init_config_s;
#pragma pack()

/**
 * @brief  创建一个控制结构体
 *
 * @param  _config 控制初始化结构体
 * @return Controller_s*
 */
Controller_s* create_controller(controller_Init_config_t* _config);

/**
 * @brief  控制结构体计算
 */
float controller_clc(Controller_s *obj);


#endif

