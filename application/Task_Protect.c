#include "Task_Protect.h"
#include "Task_Init.h"
#include "WatchDog.h"
#include "bsp_pwm.h"
#include "bsp_dwt.h"

#define LOOK_STACK 0
#if LOOK_STACK
UBaseType_t uxHighWaterMark[6];//观察任务堆栈使用情况
char taskListBuffer[30*6];//保存任务运行时间信息 分别是：任务名 任务状态 优先级 剩余栈 任务序号
char taskStateBuffer[30*6];//保存任务运行时间信息 分别是：任务名 运行计数  使用率
#endif
/* 设置LED颜色 */
void aRGB_led_show(uint32_t aRGB)
{
    static uint8_t alpha;
    static uint16_t red,green,blue;

    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;
	
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}

/* 断控保护任务 */
void Task_Protect(void *pvParameters)
{
    static portTickType currentTime;
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	for(;;)
	{
	    currentTime = xTaskGetTickCount();  // 	获取当前系统时间
		WatchDog_Polling(); // 软件看门狗轮询
        aRGB_led_show(0x7F123456);

//		HAL_IWDG_Refresh(&hiwdg);  // 硬件看门狗
#if LOOK_STACK
        //获得任务名 任务状态 优先级 剩余栈 任务序号
        memset(taskListBuffer, 0, 30*6);
        vTaskList((char *)&taskListBuffer); 
        //获取剩余Stack大小,堆栈不够会进入硬件错误
        uxHighWaterMark[0] = uxTaskGetStackHighWaterMark( Task_Robot_Handle );
        uxHighWaterMark[1] = uxTaskGetStackHighWaterMark( Task_Protect_Handle );
        uxHighWaterMark[2] = uxTaskGetStackHighWaterMark( Task_Shoot_Handle );
        uxHighWaterMark[3] = uxTaskGetStackHighWaterMark( Task_Gimbal_Handle );
        uxHighWaterMark[4] = uxTaskGetStackHighWaterMark( Task_Vision_Handle );
        //获取任务名 运行计数  使用率
        memset(taskStateBuffer, 0, 30*6);
        vTaskGetRunTimeStats((char *)&taskStateBuffer);
#endif
		
       vTaskDelayUntil(&currentTime, 10);
	}
}
