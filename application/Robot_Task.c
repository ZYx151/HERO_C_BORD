#include "Robot_Task.h"

#include "robot_def.h"

#ifdef GIMBAL_BOARD
#include "Gimbal_board_cmd.h"
#include "bsp_dwt.h"
#include "Gimbal.h"
#include "Shoot.h"
#include "DMmotor.h"
#include "remote.h"
#endif
#ifdef CHASSIS_BOARD
#include "Chassis_board_cmd.h"
#include "Chassis.h"
#include "Referee.h"
#endif

void RobotInit()
{
#ifdef GIMBAL_BOARD
    Gimbal_Cmd_Init();
	Gimbal_Init();
	Shoot_Init();
#endif
#ifdef CHASSIS_BOARD
    Chassis_Cmd_Init();
	Chassis_Init();
	Referee_Init();
#endif
}

void Task_Robot(void *pvParameters)
{
    static portTickType currentTime_robot;
	RobotInit();
	for(;;)
	{
	    currentTime_robot= xTaskGetTickCount();
#ifdef GIMBAL_BOARD
		Gimbal_board_CMD_Update();
#endif
#ifdef CHASSIS_BOARD
        Chassis_board_CMD_Update();
		Referee_Update();
#endif
		vTaskDelayUntil(&currentTime_robot, 1);
	}
}

#ifdef GIMBAL_BOARD
void Task_Shoot(void *pvParameters)
{
    static portTickType currentTime_robot;

   	for(;;)
	{
	    currentTime_robot= xTaskGetTickCount();
	    Shoot_Upeadt();
		vTaskDelayUntil(&currentTime_robot, 1);
	}
}

void Task_Gimbal(void *pvParameters)
{
    static portTickType currentTime_robot;
  		
	for(;;)
	{
	    currentTime_robot= xTaskGetTickCount();
		Gimbal_Task();
		vTaskDelayUntil(&currentTime_robot, 1);
	}
}
#endif

#ifdef CHASSIS_BOARD
void Task_Chassis(void *pvParameters)
{
	static portTickType currentTime_Chassis;
	for (;;)
	{
		currentTime_Chassis = xTaskGetTickCount();
		Chassis_Upeadt();
		vTaskDelayUntil(&currentTime_Chassis, 2);
	}
}
#endif


