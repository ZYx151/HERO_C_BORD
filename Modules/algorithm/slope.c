#include "slope.h"

/**
 * @brief 初始化
 *
 * @param __Increase_Value 增长最大幅度
 * @param __Decrease_Value 降低最大幅度
 */
void Slope_Init(Slope_s *slope, float __Increase_Value, float __Decrease_Value, Enum_Slope_First __Slope_First)
{
    slope->Increase_Value = __Increase_Value;
    slope->Decrease_Value = __Decrease_Value;
    slope->Slope_First = __Slope_First;
}


/**
 * @brief 斜坡函数计算值
 *
 */
float Slope_Calc(Slope_s *slope, float target, float real)
{
	slope->Target = target;
	slope->Now_Real = real;
	
    // 规划为当前真实值优先的额外逻辑
    if(slope->Slope_First == SLOPE_FIRST_REAL)
	{
	    if((target >= real && real >= slope->Now_Planning) || target <= real && slope->Now_Real <= slope->Now_Planning)
	     slope->Out = real;
	}
	
	if(slope->Now_Planning > 0.0f)
	{
		if( target > slope->Now_Planning)
		{
			// 此时处于正值加速阶段
			if(fabs( slope->Now_Planning - target) > slope->Increase_Value )
				slope->Out += slope->Increase_Value ;
			else
				slope->Out = target;
		}
		else if ( target < slope->Now_Planning)
		{
			// 正值减速
			if( fabs( slope->Now_Planning - target) > slope->Decrease_Value)
				slope->Out -= slope->Decrease_Value ;
			else
				slope->Out = target;
		}
	}
	else if(slope->Now_Planning < 0.0f)
	{
		if(target < slope->Now_Planning)
		{
			// 负值加速
			if(fabs(slope->Now_Planning - target) > slope->Increase_Value)
				slope->Out -= slope->Increase_Value;
			else
				slope->Out = target;
		}
		else if(target > slope->Now_Planning)
		{
			// 负值减速
			if(fabs(slope->Now_Planning - target) > slope->Decrease_Value)
				slope->Out += slope->Decrease_Value;
			else
				slope->Out = target;

		}
	}
	else
	{
		if(target > slope->Now_Planning)
		{
			// 0值正加速
			if( fabs(slope->Now_Planning - target) > slope->Increase_Value)
				slope->Out += slope->Increase_Value;
			else
				slope->Out = target;
		}
		else if (target < slope->Now_Planning)
		{
            // 0值负加速
            if (fabs(slope->Now_Planning - target) > slope->Decrease_Value)
                slope->Out -= slope->Decrease_Value;
            else
                slope->Out = target;
		}
	}
	
	slope->Now_Planning = slope->Out;
	return slope->Out;
}


