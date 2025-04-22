#ifndef _FSM_H
#define _FSM_H

#include "user_lib.h"

#define STATE_MAX (10)
/**
 * @bried 状态所处的阶段
 *
 **/
typedef enum
{
	State_Stage_DISABLE = 0,
	State_Stage_ENABLE,
}Enum_Status_State;

/**
 * @brief 状态结构体
**/
typedef struct 
{
	Enum_Status_State Status_Stage;
	uint32_t Count_Time;
}Struct_Status;

/**
 * @brief Reusable, 有限自动机核心, 一般有时间需求的则采用有限自动机
 * 使用时请继承->声明友元后使用
 *
 */
typedef struct{
	Struct_Status Status[STATE_MAX];
	
	// 有限状态机触发回调函数
    void (*fsm_module_callback)(void);
    
	
}FSM_S;
/* 初始化结构体 */
void FSM_Init(FSM_S *fsm, uint8_t status_number, uint8_t init_status);

#endif