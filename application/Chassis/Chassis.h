#ifndef __CHASSIS_H
#define __CHASSIS_H

/**
 * @brief 底盘各个轮子电机位置
 *
 *           ^ X
 * 0         |        3
 *           |
 *           |
 * ----------|----------> Y
 *           |
 *           |
 * 1         |        2
 * 旋转速度roate为逆时针旋转速度 
 *
**/


#define USE_MECANUM_CHASSIS 1

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define RPM_TO_ANGULAR_SPEED_DIV  (30.0f / PI)  // 转化系数 9.55f
//#define chassisWheelToCentreLength   (HALF_WHEEL_BASE^HALF_WHEEL_BASE+HALF_TRACK_WIDTH^HALF_TRACK_WIDTH) //每个电机接触地面和底盘中心之间的距离 m 
#define SQRT2 1.41421356237f  
#define MOTOR_DECELE_RATIO  19.203f
#define LF_CENTER ((HALF_TRACK_WIDTH+HALF_WHEEL_BASE)*2)
#define RF_CENTER ((HALF_TRACK_WIDTH+HALF_WHEEL_BASE)*2)
#define LB_CENTER ((HALF_TRACK_WIDTH+HALF_WHEEL_BASE)*2)
#define RB_CENTER ((HALF_TRACK_WIDTH+HALF_WHEEL_BASE)*2)

#define CHASSSIWHEEL_TO_CENTER sqrt(HALF_TRACK_WIDTH*HALF_TRACK_WIDTH+HALF_WHEEL_BASE*HALF_WHEEL_BASE)
/* 电机功率参数(数据暂未完全拟合，由Matlab拟合) */
#define CC  2.022e-8             //!< @brief功率环参数(扭矩电流*扭矩电流(ma))
#define SC  2.248e-6             //!< @brief功率环参数(转速RPM*扭矩电流)

#define CCI 1.23e-07            //!< @brief功率环参数(发送电流*发送电流（pid.out）)
#define SCI 1.99688994e-6f      //!< @brief功率环参数(转速RPM*发送电流)

#define SS  1.453e-07          //!< @brief功率环参数(转速RPM*转速RPM)
#define Constant 0.85          //!< @brief功率环参数(空载时功率)

#define ANGLE2RADIAN (2.0f*PI/360.0f)
typedef struct {
	float X;
	float Y;
	float Omega;
}Chassis_speed_s;
/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void Chassis_Init(void);

/**
 * @brief 云台任务
 * 
 */
void Chassis_Upeadt(void);

#endif