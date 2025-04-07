#ifndef _DJIMOTOR_H_
#define _DJIMOTOR_H_

#include "stdint.h"
#include "motor_def.h"
#include "RMQueue.h"

/* 使用DJI系列电机 */
#define DJI_MOTOR_CNT 12     /* 使能大疆电机总数 */ 

/* 滤波系数设置为1的时候即关闭滤波 */  
// TODO 给不同的电机设置不同的低通滤波器惯性系数而不是统一使用宏
#define SPEED_SMOOTH_COEF 0.90f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.95f     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.0439453125f // (360/8192),将编码器值转化为角度制

#define RAD_2_DEGREE 57.2957795f    // 180/pi
#define DEGREE_2_RAD 0.01745329252f // pi/180

#define RPM_2_ANGLE_PER_SEC 6.0f       // ×360°/60sec
#define RPM_2_RAD_PER_SEC 0.104719755f // ×2pi/60sec

#pragma pack(1)
/**
* @brief 目前使用的大疆电机can反馈基本相同 使用相同结构体接收
*/
typedef struct {
    uint16_t MchanicalAngle;    //!<@brief 机械角度
    uint16_t LastMchanicalAngle;//!<@brief 上一次的机械角度
    uint8_t flag;
    int16_t Speed;              //!<@brief 转速 /rpm
    int16_t TorqueCurrent;      //!<@brief 转矩电流
    uint8_t temp;               //!<@brief 温度
    int16_t round;              //!<@brief 圈数
    
	float SpeedFilter;        //!<@brief 转速 /rpm
    float CurrentFilter;      //!<@brief 转矩电流
    int32_t Angle;              //!<@brief 连续化机械角度
	int32_t LastAngle;          //!<@brief 上一次连续化机械角度
    float   Angle_DEG;          //!<@brief 连续化角度制角度
	uint16_t MotorCenter;
} DJIMotor_Measure_s;

/**
* @brief  电机使能总结构体
*/
typedef struct{
   	DJIMotor_Measure_s measure;        // 电机测量值
    CANInstance* motor_can_instance;   // 电机CAN实例
   	WatchDog_TypeDef* watchdog;        // 电机看门狗
	
   	Controller_s motor_controller;
   	Motor_Working_Type_e stop_flag;  // 电机启停标志
    
    /* 分组发送设置 */
    uint8_t sender_group;
    uint8_t message_num;
	/*  电机计算 */
	Motor_Typed_e DJI_motor;
    float *other_angle_feedback_ptr;   // 角度反馈数据指针,注意电机使用total_angle
    float *other_speed_feedback_ptr;   //速度反馈数据指针,单位为angle per sec
} DJIMotor_Instance;
extern CANInstance sender_assignment[6];
#pragma pack()

//    float line_speed;           //!<@brief 线速度（m/s，根据角速度算出）
//    float velocity;             //!<@brief用电机编码器计算出来的角速度（单位：度每秒）
//    struct PowerCOF_s {
//	float ss;               //!<@brief 速度平方项系数
//	float sc;               //!<@brief 速度,转矩电流乘积项系数
//	float cc;               //!<@brief 转矩电流平方项系数
//	float constant;         //!<@brief 常量
//    }   PowerCOF;                 //!<@brief 计算功率所用的系数,由MATLAB拟合
//    float Power;                //!<@brief 功率

//    RMQueue_Handle *Position_Queue; // 计算角速度的循环队列
//    float position_sum;              //队列中所有值的和

/**
 * @brief 调用此函数注册一个DJI电机,需要传递较多的初始化参数,请在application初始化的时候调用此函数
 *        推荐传参时像标准库一样构造initStructure然后传入此函数.
 *        recommend: type xxxinitStructure = {.member1=xx,
 *                                            .member2=xx,
 *                                             ....};
 *        请注意不要在一条总线上挂载过多的电机(超过6个),若一定要这么做,请降低每个电机的反馈频率(设为500Hz),
 *        并减小DJIMotorControl()任务的运行频率.
 *
 * @attention M3508和M2006的反馈报文都是0x200+id,而GM6020的反馈是0x204+id,请注意前两者和后者的id不要冲突.
 *            如果产生冲突,在初始化电机的时候会进入IDcrash_Handler(),可以通过debug来判断是否出现冲突.
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return DJIMotorInstance*
 */
DJIMotor_Instance *DJI_Motor_create(Motor_Init_config_s *config);

/**
 * @brief 停止电机,注意不是将设定值设为零,而是直接给电机发送的电流值置零
 */
void DJIMotorStop(void *id);

/**
 * @brief 启动电机,此时电机会响应设定值
 *        初始化时不需要此函数,因为stop_flag的默认值为0
 */
void DJIMotorEnable(DJIMotor_Instance *motor);

/**
 * @brief 关闭电机 仅有同一个can通道上的电机同时close时生效
 */
void DJIMotorClose(DJIMotor_Instance *motor);


/**
 * @brief 该函数被motor_task调用运行在rtos上,motor_stask内通过osDelay()确定控制频率
 */
void DJIMotorControl(void );
float DJIMotor_Ctrl(DJIMotor_Instance *motor);
void  DJIMotorReceive(DJIMotor_Instance *motor, uint8_t *rxbuff);
/**
 * @brief 该函数用于修改控制反馈
 */
void DJIMotorChangeFeed(DJIMotor_Instance *motor, Feedback_Source_e type);

/**
 * @brief 修改电机的实际闭环对象
 */
void DJIMotorOuterLoop(DJIMotor_Instance *motor, Closeloop_Type_e outer_loop);

/**
 * @briecf 修改电机期望
 */
void DJIMotorSetRef(DJIMotor_Instance *motor, float ref);
/**  获取电机信息  **/
int16_t DJIMotor_GetAngle(DJIMotor_Instance *motor);
int16_t DJIMotor_GetTorque(DJIMotor_Instance *motor);
int16_t DJIMotor_GetRPMSpeed(DJIMotor_Instance *motor);
RM_Status DJIMotor_detect(DJIMotor_Instance *motor);
HAL_StatusTypeDef DJIMotor_Transmit(CAN_HandleTypeDef *hcan,uint32_t StdId, int16_t *Data);

#endif

