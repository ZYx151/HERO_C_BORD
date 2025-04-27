#include "Task_Init.h"

#include "bsp_dwt.h"
#include "usart.h"
#include "remote.h"
#include "robot_def.h"
#include "ins_task.h"
#include "tim.h"

TaskHandle_t Task_Robot_Handle;
TaskHandle_t Task_Protect_Handle;
TaskHandle_t Task_Music_Handle;
#ifdef BMI088_INS
TaskHandle_t Task_INS_Handle;
#endif
#ifdef GIMBAL_BOARD
TaskHandle_t Task_Shoot_Handle;
TaskHandle_t Task_Gimbal_Handle;
TaskHandle_t Task_Vision_Handle;
TaskHandle_t Task_Usb_Handle;
#endif
#ifdef CHASSIS_BOARD
TaskHandle_t Task_Chassis_Handle;
TaskHandle_t Task_PowerController_Handle;
TaskHandle_t Task_Referee_UI_Handle;
TaskHandle_t Task_Referee_Handle;
#endif

void Task_Init()
{
    taskENTER_CRITICAL(); // 进入临界区
	 /* 外设初始化 */
	DWT_Init(168);

/* 创建任务 */
#ifdef WT931_IMU
#elif defined(BMI088_INS)
	INS_Init();
	xTaskCreate((TaskFunction_t)Task_INS,               "Task_INS",              128*8, NULL, 8, &Task_INS_Handle);
#endif                                                                                                               
	// 启动蜂鸣器
	xTaskCreate((TaskFunction_t)Task_Music,             "Task_Music",            128,   NULL, 5, &Task_Music_Handle);
#ifdef GIMBAL_BOARD
	/* 串口1初始化 DT7遥控器 */
	Remote_Init();
	xTaskCreate((TaskFunction_t)Task_Robot,             "Task_Robot",             128*4, NULL, 7, &Task_Robot_Handle);
	xTaskCreate((TaskFunction_t)Task_Gimbal,            "Task_Gimbal",            128*8, NULL, 6, &Task_Gimbal_Handle);
	xTaskCreate((TaskFunction_t)Task_Shoot,             "Task_Shoot",             128*8, NULL, 5, &Task_Shoot_Handle);
//	xTaskCreate((TaskFunction_t)Task_Vision,            "Task_Vision",            128*8, NULL, 6, &Task_Vision_Handle);
//	xTaskCreate((TaskFunction_t)Task_Usb,               "Task_Usb",               128*2, NULL, 7, &Task_Usb_Handle);
#endif
#ifdef CHASSIS_BOARD
	xTaskCreate((TaskFunction_t)Task_Chassis,           "Task_Chassis",          128*8, NULL, 5, &Task_Chassis_Handle);
	xTaskCreate((TaskFunction_t)Task_Robot,             "Task_Robot",            128*4, NULL, 6, &Task_Robot_Handle);
	xTaskCreate((TaskFunction_t)Task_Referee_UI,        "Task_Referee_UI",       128*4, NULL, 5, &Task_Referee_UI_Handle);
	xTaskCreate((TaskFunction_t)Task_Referee,           "Task_Referee",          128*4, NULL, 7, &Task_Referee_Handle);
#endif
	xTaskCreate((TaskFunction_t)Task_Protect,           "Task_Protect",          128*2, NULL, 6, &Task_Protect_Handle);

    HAL_TIM_Base_Start(&htim5);
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 2280);  //400  关    600开
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 1400);  //400  关    600开

//    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
//    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1, 5000);  //400  关    600开
//    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
//    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2, 2000);  //400  关    600开

    taskEXIT_CRITICAL(); // 退出临界区
    vTaskDelete(NULL);   // 删除开始空闲任务
}




