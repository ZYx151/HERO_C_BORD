#ifndef __TASK_INIT_H
#define __TASK_INIT_H

#include "FreeRTOS.h"
#include "task.h"
#include "RMLibHead.h"

extern TaskHandle_t Task_Robot_Handle;
extern TaskHandle_t Task_Protect_Handle;
extern TaskHandle_t Task_Shoot_Handle;
extern TaskHandle_t Task_Gimbal_Handle;
extern TaskHandle_t Task_INS_Handle;
extern TaskHandle_t Task_Remote_Handle;
extern TaskHandle_t Task_PowerController_Handle;
extern TaskHandle_t Task_Vision_Handle;
extern TaskHandle_t Task_Usb_Handle;
extern TaskHandle_t Task_Music_Handle;
/**
 * @brief 看门狗保护任务
 */
void Task_Protect(void *pvParameters);
/**
 * @brief 任务CMD函数
 */
void Task_Robot(void *pvParameters);
/**
* @brief 发射任务
 */
void Task_Shoot(void *pvParameters);
/**
 * @brief 底盘任务
 */
void Task_Chassis(void *pvParameters);
/**
 * @brief 云台函数
 */
void Task_Gimbal(void *pvParameters);
/**
 * @brief 虚拟串口接收
 */
void Task_Usb(void *pvParameters);
/**
 * @brief 陀螺仪任务
 */
void Task_INS(void *pvParameters);
/**
 * @brief 视觉函数
 */
void Task_Vision(void *pvParameters);
/**
 * @brief 任务CMD函数
 */
void Task_Referee_UI(void *pvParameters);
/**
 * @brief 任务CMD函数
 */
void Task_Referee(void *pvParameters);
/**
 * @brief 蜂鸣器任务
 */
void Task_Music(void *pvParameters);
						  
#endif
