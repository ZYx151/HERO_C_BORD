#ifndef __SHOOT_H
#define __SHOOT_H

#include "DJIMotor.h"
#include "pub_sub.h"
#include "robot_def.h"

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void Shoot_Init(void);

/**
 * @brief 云台任务
 * 
 */
void Shoot_Upeadt(void);




#endif