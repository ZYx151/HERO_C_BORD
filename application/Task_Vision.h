#ifndef __TASK_VISION_H
#define __TASK_VISION_H

void Aim_Init(void);    //!< @brief  PC通信初始化
void Aim_Control(void); //!< @brief  自瞄主控函数
void Aim_Shoot(void);   //!< @brief  自瞄发射控制
void Aim_Offset(float roll,float pitch,float yaw, float bullet_speed);  //!< @brief  自瞄角度补偿计算

enum ARMOR_NUM
{
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
};

enum ARMOR_ID
{
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7
};


#endif
