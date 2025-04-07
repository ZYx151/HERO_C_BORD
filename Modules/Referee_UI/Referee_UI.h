#ifndef __TASK_REFEREE_UI_H
#define __TASK_REFEREE_UI_H

#include "RM_Cilent_Ul.h"

enum{
    INIT = 0,
    INITING = 1,
    MOVEING = 2,
}UI_STATE;

/**
 * @brief UI��ʼ��
 */
void UI_Init(void);

/**
 * @brief ��̬UI
 */
void UI_Move(void);

/**
 * @brief UIˢ��
 */
void UI_Refresh(void);

#endif