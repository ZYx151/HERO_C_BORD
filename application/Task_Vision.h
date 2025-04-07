#ifndef __TASK_VISION_H
#define __TASK_VISION_H

void Aim_Init(void);        	      //!< @brief  PC通信初始化
void Aim_Control(void);            //!< @brief  自瞄主控函数
void Aim_Shoot();

enum ARMOR_NUM
{
    ARMOR_NUM_BALANCE = 2,
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
};

#endif
