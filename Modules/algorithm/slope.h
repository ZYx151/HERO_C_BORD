#ifndef __SLOPE_H
#define __SLOPE_H
/** 斜坡函数, 用于速度规划等 **/
#include "math.h"

/**
 * @brief 规划优先类型, 分为目标值优先和真实值优先
 * 目标值优先, 即硬规划
 * 真实值优先, 即当前真实值夹在当前规划值和目标值之间, 当前规划值转为当前真实值
 *
 */
typedef enum 
{
    SLOPE_FIRST_REAL = 0,
    SLOPE_FIRST_TARGET,
}Enum_Slope_First;

/* 斜坡函数结构体 */
typedef struct{
    // 输出值
    float Out;
    // 规划优先类型
    Enum_Slope_First Slope_First;
    // 当前规划值
    float Now_Planning;
    // 当前真实值
    float Now_Real;

    // 绝对值增量, 一次计算周期改变值
    float Increase_Value;
    // 绝对值减量, 一次计算周期改变值
    float Decrease_Value;
    // 目标值
    float Target;
}Slope_s;

void Slope_Init(Slope_s *slope, float __Increase_Value, float __Decrease_Value, Enum_Slope_First __Slope_First);
float Slope_Calc(Slope_s *slope, float target, float real);


#endif

