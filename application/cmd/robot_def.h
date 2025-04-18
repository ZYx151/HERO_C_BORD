#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "stdint.h"
#include "stdlib.h"

/* 开发板类型定义 */
//#define ONE_BOARD     // 单片机控整车
//#define CHASSIS_BOARD // 底盘板
#define GIMBAL_BOARD  // 云台板

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

#define CHASSIS_MOVE 1
#define GIMBAL_MOVE  1
#define SHOOT_MOVE   1
/* 陀螺仪选择 */
#define BMI088_INS
//#define WT931_IMU

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define Yaw_Mid_Front   3800       //!< @brief Yaw轴电机中值（机器人前方）
/* 根据机器人正前方Yaw轴机械角度算出机器人剩下三个方向Yaw轴机械角度（6020正装，反装将左右相反） */
#if (Yaw_Mid_Front + 2048) > 8191 
#define Yaw_Mid_Left Yaw_Mid_Front - 6143
#else
#define Yaw_Mid_Left Yaw_Mid_Front + 2048
#endif

#if (Yaw_Mid_Left + 2048) > 8191
#define Yaw_Mid_Back Yaw_Mid_Left - 6143
#else
#define Yaw_Mid_Back Yaw_Mid_Left + 2048
#endif

#if (Yaw_Mid_Back + 2048) > 8191
#define Yaw_Mid_Right Yaw_Mid_Back - 6143
#else
#define Yaw_Mid_Right Yaw_Mid_Back + 2048
#endif
//!< @brief Pitch轴电机归中机械角度
#define Pitch_Mid        2301
//!< @brief Pitch轴电机上限位与水平位差值
#define P_ADD_limit      0
//!< @brief Pitch轴电机下限位与水平位差值
#define P_LOSE_limit     0
//!< @brief IMU上限位（Pitch轴）
#define IMU_UP_limit     16.0f
//!< @brief IMU下限位（Pitch轴）
#define IMU_DOWN_limit   -40.0f
//!< @brief IMU上限位（Pitch轴）
#define MCH_UP_limit     30.0f
//!< @brief IMU下限位（Pitch轴）
#define MCH_DOWN_limit   -30.0f

// 发射参数
//!< @brief 摩擦轮电机速度环PID的期望值  /5800 3508电机Speed_Max: 
#define SHOOT_SPEED            5350
//!< @brief 拨盘一圈的装载量
#define SHOOT_NUM_PER_CIRCLE   5
//!< @brief 拨弹盘电机连发时速度环PID的期望值 /19.0 (RPM) SHOOT_LOADER_MOTOR_ONE * 弹频
//#define PLUCK_SPEED         1500 
//!< @brief  摩擦轮半径(mm)
//#define SHOOT_RADIUS         0.30f
//!< @brief 摩擦轮周长(mm)
// #define SHOOT_PERIMETER       SHOOT_RADIUS * 2 * PI
//!< @brief 一发弹丸拨弹盘电机转过的机械角度
#define SHOOT_LOADER_MOTOR_ONE 360.0 * SHOOT_MOTOR_DECELE_RATIO / SHOOT_NUM_PER_CIRCLE
//!< @brief 每发射一颗小弹增加的热量
#define SHOOT_UNIT_HEAT_17MM   5
//!< @brief 每发射一颗大弹丸增加的热量
#define SHOOT_UNIT_HEAT_42MM   100

/** @brief 机器人底盘修改的参数,单位位 m  **/
#define WHEEL_BASE  0.360           // 纵向轴距(前后方向)
#define TRACK_WIDTH 0.300           // 横向轮距(左右方向)
#define RADIUS_WHEEL 0.150          // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.20f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换 3508为19.2f
/** @brief 电机参考速度 */
// 电压到输出的转化系数
#define VOLTAGE2OUT  (25000.0f/24.0f)
// 电流到输出的转化系数
#define CURRENT2OUT  (16384.0f/3.0f)
//#define STD_Speed 2380    // 标准速度1m/s
//#define STD_Omega 6142    // 标准速度1rpm/s
//#define STD_Angle 0.36f   // 角度制1rpm/s
//#define STD_MAngle 8.192f // 机械角度制1rpm/s
    // DR16摇杆对云台yaw灵敏度系数(0.002 * PI表示yaw速度最大时为1rpm)
#define DR16_Rocker_Yaw_Resolution ( 0.002f * PI)
    // DR16摇杆对云台pitch灵敏度系数(0.002 * PI表示pitch速度最大时为1rpm)
#define DR16_Rocker_Pitch_Resolution (0.0006f * PI)
    // DR16鼠标云台yaw灵敏度系数
#define DR16_Mouse_Yaw_Angle_Resolution 1.0f
    // DR16鼠标云台pitch灵敏度系数
#define DR16_Mouse_Pitch_Angle_Resolution 0.6f

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

/**@brief 去除遥控器摇杆死区 */
#define DEADLINE_LIMIT(value, dealine)                     \
	{                                                      \
		if ((value) <= (dealine) && (value) >= -(dealine)) \
			value = 0;                                     \
	}

/**
 * @brief 限幅宏函数
 * @param IN 限幅变量
 * @param MAX 最大值
 * @param MIN 最小值
 */
#define limit(IN, MAX, MIN) \
    if (IN < MIN)           \
        IN = MIN;           \
    if (IN > MAX)           \
        IN = MAX
	
#pragma pack(1) // 压缩结构体,取消字节对齐
/* -------------- 基本控制模式和数据类型定义 -------------- */

// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
	Robot_RUNNING,
} Robot_Status_e;

// 对模块状态定义
typedef enum
{
	Device_Offline = 0,         //!< @brief 设备离线
	Device_Online  = 1,	        //!< @brief 设备在线
	Device_Error   = 2	        //!< @brief 设备错误        
} DeviceState_e;

// 底盘模式控制
typedef enum
{
    CHASSIS_ZERO_FORCE = 0, //!< @brief  电流零输入
    CHASSIS_FOLLOW,         //!< @brief  跟随模式，底盘叠加角度环控制
    CHASSIS_SPIN,           //!< @brief  小陀螺模式
    CHASSIS_NORMAL,         //!< @brief  不跟随，允许全向平移
    CHASSIS_NAVIGATION,     //!< @brief  导航模式
} Chassis_mode_e;

// 底盘调度模式
typedef enum Chassis_dispatch_mode_e {
    chassis_dispatch_mild = 0,           // 限制加速度 不消耗电容
    chassis_dispatch_without_acc_limit,  // 无加速度限制 不消耗电容
    chassis_dispatch_shift,              // 无加速度限制 耗电容
    chassis_dispatch_climb,              // 爬坡
    chassis_dispatch_fly                 // 飞坡
} Chassis_dispatch_mode;

// 云台模式设置
typedef enum{
    GIMBAL_ZERO_FORCE = 0,  //!< @brief 电流零输入
	GIMBAL_MECH_MODE,       //!< @brief 机械,云台跟随底盘
	GIMBAL_GYRO_MODE,       //!< @brief 陀螺仪,底盘跟随云台
	GIMBAL_MIDDLE,          //!< @brief 云台归中
	GIMBAL_SYSTEM_MODE,     //!< @brief 云台系统辨识模式
} Gimbal_mode_e;

// 发射模式设置
typedef enum{
    SHOOT_ZERO_FORCE = 0,     //!< @brief  电流零输入
	SHOOT_AIM        = 1,     //!< @brief  自瞄控制发射
	SHOOT_STOP       = 2,     //!< @brief  停止发射（摩擦轮、拨弹盘停止）
	SHOOT_READY      = 3,     //!< @brief  准备发射（摩擦轮启动）
    SHOOT_STUCKING   = 4,     //!< @brief  卡弹退弹中
	SHOOT_NOMAL      = 5,     //!< @brief  进入发射模式
	SHOOT_CHEAKOUT   = 6,     //!< @brief  检录模式
} Shoot_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} Lid_mode_e;

/* 云台归中位置 */
typedef enum{
	FRONT = 0,       //!< @brief   前方
	BACK  = 1,       //!< @brief   后方
} eMidMode;


// 发射类型运行模式
typedef enum  {
    BULLET_HOLDON   = 0, //!< @brief 拨弹盘停止
//    BULLET_REVERSE,    //!< @brief 反转，卡弹处理
    BULLET_SINGLE,     //!< @brief 单发
    BULLET_DOUBLE,     //!< @brief 双发
    BULLET_TRIBLE,     //!< @brief 三发
    BULLET_CONTINUE,   //!< @brief 连发
} Bullet_mode_e;

// 自瞄模式
typedef enum  {
    auto_aim_off = 0,     // 关闭自瞄
	auto_aim_distant,     // 吊射
    auto_aim_normal,      // 正常自瞄
} AutoAim_mode_e;

typedef enum {
	AIM_STOP = 0,     //!< @brief   关闭自瞄
	AIM_AID  = 1,     //!< @brief   自瞄不自动射击
	AIM_AUTO = 2,     //!< @brief   自瞄+自动射击
}Aim_Action_e;

/**
 * @brief 云台角度结构体
 */
typedef struct {
    float Pitch;               //!< @brief Pitch轴角度
    float Yaw;                 //!< @brief Yaw轴角度
} PTZAngle_Ref_t;


/** 机器人模块控制量定义 **/
// 对底盘速度的控制量
/* vx vy rotate传入时以offset系（一般为云台系）为基准， 经过解算后映射到底盘
 * 小陀螺模式下，rotate参数失效，旋转速度由模块内部设定
 */
typedef struct  {
    float vx;            // 单位 基准速度的倍率（基准速度由底盘模块根据功率自动计算）
    float vy;            // 单位 基准速度的倍率
    float rotate;        // 单位 旋转速度度每秒
} Cmd_chassis_speed_t;

// 对底盘功率的控制量
typedef struct  {
    uint8_t if_consume_supercap;          // 是否消耗电容
    uint16_t power_limit;                 // 功率限制
    float power_now;                      // 当前功率
    uint16_t power_buffer;                // 缓冲功率
} Cmd_chassis_power_t;

// 对底盘模块的控制量（总）
typedef struct  {
    Chassis_mode_e mode;              // 底盘模式
    float vx;                         // 单位 m/s
    float vy;            			  // 单位 m/s
    float rotate;                     // 单位 旋转速度度每秒
	uint8_t Close_flag;				  // 底盘关闭标志位
    uint8_t soft_reset_flag;               // Shift跑路
    Chassis_dispatch_mode dispatch_mode;  // 底盘运行模式
} Chassis_ctrl_cmd_t;

// 对发射机构的控制量
typedef struct  {
    Shoot_mode_e mode;
    Bullet_mode_e bullet_mode;    // 发射模式
    Lid_mode_e mag_mode;         // 弹仓盖
    uint16_t bullet_speed;      // 弹速
    float fire_rate;            // 射频（发/秒）
    int16_t heat_limit_remain;  // 剩余热量，cooling_limit-cooling_heat
    float bullet_speed_fdb;     // 实时弹速
} Shoot_ctrl_cmd_t;

// 对云台的控制量
typedef struct {
    Gimbal_mode_e mode;
    Gimbal_mode_e last_mode;
	Chassis_mode_e chassis_mode;
	Shoot_mode_e shoot_mode;
    PTZAngle_Ref_t Gyro_Ref;
    PTZAngle_Ref_t Mech_Ref;
	eMidMode       Mid_mode;     // 归中角度
	PTZAngle_Ref_t Feedback_Speed; // 云台前馈角速度
    float rotate_feedforward;  // 小陀螺前馈 云台跟随底盘用
} Gimbal_ctrl_cmd_t;


typedef enum {
	Gimbal_offline = 0,    
	Gimbal_online  = 1,     
}Gimbal_status_e;

typedef enum {
	shoot_offline = 0,     
	shoot_online  = 1,      
}shoot_status_e;

typedef enum {
	stop = 0,    
	normal  = 1,   
    rotate = 2,
    fly = 3,
    grasp = 4
} move_status_e;

typedef enum {
	vision_offline = 0,   
	vision_online  = 1,    
}vision_status_e;


typedef enum {
	shoot_mode_stop = 0,   
	shoot_mode_ready  = 1,   
    shoot_mode_fire = 2,
    shoot_mode_follow = 3,
}shoot_mode_e;

typedef struct  {
    Gimbal_status_e Pitch: 1;
    Gimbal_status_e Yaw : 1;
} Gimbal_status_t;

typedef struct  {
	uint16_t vision_distance;           
	int16_t Pitch_angle;
	int16_t Yaw_angle;
	int16_t Offset_Angle;
} Gimbal_data_t;

typedef struct{
    Gimbal_status_t Gimbal_status;         
    shoot_status_e shoot_status : 1;          
    move_status_e move_status : 4;     
    vision_status_e vision_status : 1;          
    shoot_mode_e shoot_mode;				           
	uint8_t Key; 
	uint8_t vision_number; 
}Gimbal_action_t;

// 云台回传cmd的数据
typedef struct {
	Gimbal_mode_e mode;
	uint16_t yaw_encorder;     // YAW反馈机械角度 用于求角度补偿
	int16_t yaw_round;
	float pitch_encorder;      // PITCH反馈机械角度
    float rotate_speed;        // 底盘旋转速度 
	DeviceState_e gimbal_status;
	DeviceState_e pitch_status;
} Gimbal_upload_t;

// 底盘模块回传cmd的数据
typedef struct {
	DeviceState_e chassis_status;
	Chassis_mode_e mode;
    float chassis_supercap_percent;  // 超级电容回传容量剩余
    float chassis_battery_voltage;   // 电池电压ACD电池采样
} Chassis_upload_t;

typedef struct {
	DeviceState_e shoot_status;
    Shoot_mode_e mode;
    Bullet_mode_e bullet_mode;    // 发射模式
    float real_bullet_speed;
} Shoot_upload_t;

// RLS计算
typedef struct {
	DeviceState_e refree_status;
	DeviceState_e superCAP_status;
	Cmd_chassis_power_t send_power;
    uint8_t robot_level;
} RLS_update_t;

/***  板间通信定义  ***/
// 云台->底盘数据包
typedef struct  {
    int16_t vx;            // 单位 基准速度的倍率（基准速度由底盘模块根据功率自动计算）
    int16_t vy;            // 单位 基准速度的倍率
    int16_t rotate;        // 单位 旋转速度度每秒
	uint8_t Close_flag;		//!< @brief 底盘关闭标志位
    uint8_t soft_reset_flag;	//!< @brief 软件重启标志
} Gimbal_board_send_t;
// 云台->底盘功能包
typedef struct  {
    uint8_t now_robot_mode;            // 遥控器在云台主控 包含stop模式与云台重要模块掉线
    uint8_t chassis_dispatch_mode;     // 底盘功率模式
	uint8_t autoaim_mode;              // UI所需自瞄数据
    Chassis_mode_e chassis_mode;              // UI所需底盘模式
	uint8_t pc_online;                 // UI所需PC是否在线
    uint8_t gimbal_mode;               // UI所需云台数据
    uint8_t shoot_mode;                // UI所需摩擦轮数据
    uint8_t bullet_mode;               // UI所需拨弹盘数据
    uint8_t vision_has_target;         // 自瞄是否检测到目标
    uint8_t soft_reset_flag;           // 软重启标志位
} Gimbal_board_send_UI_t;
// PC->云台接收数据包
typedef struct {
	Gimbal_board_send_t Robot_CmdSpeed;
} Usb_ReceiveData_t;

// 云台<-底盘数据包
typedef struct  {
    int16_t chassis_gyro;         // 将底盘主控的imu数据发到云台
//    uint8_t chassis_board_status;  // 同步底盘是否有重要模块掉线
//    uint8_t robot_id;
    struct {
		uint8_t  robot_color;       // 机器人颜色
		uint8_t  robot_level;       // 机器人等级
        uint16_t heat_limit_remain; // 剩余热量
        uint16_t bullet_speed_now;  // 实时弹速
    } shoot_referee_data;
} Chassis_board_send_t;

// 裁判系统与底板通信数据
typedef struct{
	DeviceState_e refree_status;
    uint8_t  robot_color;                      //!< @brief 机器人颜色
    uint8_t  robot_level;                      //!< @brief 等级
    uint16_t heat_now_remain;                //!< @brief 实时枪口热量
    uint16_t shooter_barrel_heat_limit;        //!< @brief 枪口热量上限
    uint16_t bullet_speed_now;                 //!< @brief 弹速
	uint16_t buffer_energy;                    //!< @brief 缓冲能量（单位：J）
	float    chassis_power;                    //!< @brief 底盘功率（单位：J）
	uint16_t chassis_power_limit;              //!< @brief 底盘功率上限（单位：J）
    uint16_t robot_HP;                         //!< @brief 机器人血量
    uint8_t  chassis_output;                   //!< @brief 底盘电流输出
    uint8_t  Ammo_remain; 
    uint16_t Max_HP;
    uint16_t Ammo_consume;
    float level_gain;
} Referee_data_t;

/** 自瞄消息处理后数据 **/
typedef struct{
	float yaw;
	float pitch;
	float yaw_vel;
	float pitch_vel;
	
	DeviceState_e usb_state;
    Aim_Action_e aim_action;
	AutoAim_mode_e auto_mode;
	uint8_t fire_on;    
	uint8_t fire_flag;
	
	uint16_t horizontal;
	uint8_t number;
}Aim_Data_t;
extern Aim_Data_t Aim_Ref; 

#pragma pack()


#endif

