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
	Status_Stage_DISABLE = 0,
	Status_Stage_ENABLE,
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
    // 状态数量
	uint8_t Status_Number;
    // FSM当前状态
    uint8_t Now_Status_Serial;
	// 有限状态机触发回调函数
    void (*fsm_module_callback)(void);
    // 软件定时器处理函数(放在任务中调用)
	void (*Calculate_PeriodElapsedCallback)(void);
	
}FSM_S;
/* 初始化结构体 */
void FSM_Init(FSM_S *fsm, uint8_t status_number, uint8_t init_status);

/* 更新FSM模式 */
void Set_FSM_Status(FSM_S *fsm, uint8_t Next_Status_serial);
/* 获取当前FSM状态 */
uint8_t Get_FSM_Status(FSM_S *fsm);
#endif