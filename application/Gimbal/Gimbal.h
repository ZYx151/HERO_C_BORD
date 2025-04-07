#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "robot_def.h"

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void Gimbal_Init(void)
;

/**
 * @brief 云台任务
 * 
 */
void Gimbal_Task(void);

#endif
