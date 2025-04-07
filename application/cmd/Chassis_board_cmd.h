#ifndef _CHASSIS_BOARD_CMD_H_
#define _CHASSIS_BOARD_CMD_H_

#include "can_comm.h"

/** @brief 电机参考速度 */
#define STD_Speed 2380  // 标准速度1m/s
#define STD_Omega 6142  // 标准速度1rpm/s
#define STD_Angle 0.36f			   // 角度制1rpm/s
#define STD_MAngle 8.192f		   // 机械角度制1rpm/s


/** 底盘CMD结构体初始化  **/
void Chassis_Cmd_Init(void);

/** 底盘CDM任务函数 **/
void Chassis_board_CMD_Update(void) ;

#endif


