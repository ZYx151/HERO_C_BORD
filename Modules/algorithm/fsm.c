#include "fsm.h"

/** 
 * @brief 初始化FSM结构体
 *
**/
void FSM_Init(FSM_S *fsm, uint8_t _Status_Number, uint8_t __Now_Status_Serial)
{
	fsm->Status_Number = _Status_Number;
	fsm->Now_Status_Serial = __Now_Status_Serial;
    // 所有状态全刷0
    for (uint8_t i = 0; i < STATE_MAX; i++)
    {
        fsm->Status[i].Status_Stage = Status_Stage_DISABLE;
        fsm->Status[i].Count_Time = 0;
    }
	
    // 使能初始状态
    fsm->Status[__Now_Status_Serial].Status_Stage = Status_Stage_ENABLE;
	
}




