#include "bsp_pwm.h"
#include "stdlib.h"

// 配合中断以及初始化
static uint8_t idx;
static PWMInstance *pwm_instance[PWM_DEVICE_CNT] = {NULL}; // 所有的pwm instance保存于此,用于callback时判断中断来源
static uint32_t PWMSelectTclk(TIM_HandleTypeDef *htim );

/**
 * @brief pwm dma传输回调函数
 * 
 * @param htim 发生中断的定时器句柄
**/
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    for (uint8_t i = 0; i < idx; i++)
    { // 来自同一个定时器的中断且通道相同
        if (pwm_instance[i]->htim == htim && htim->Channel == (1<<(pwm_instance[i]->channel/4)))
        {
            if (pwm_instance[i]->callback) // 如果有回调函数
                pwm_instance[i]->callback(pwm_instance[i]);
            return; // 一次只能有一个通道的中断,所以直接返回
        }
    }
}

/**
 * @brief 返回一个PWM实例
 *
**/
PWMInstance *PWMRegister(PWM_Init_Config_s *config)
{
	 if(idx >= PWM_DEVICE_CNT)
		 return 0;
	 PWMInstance *PWM = (PWMInstance *)RMLIB_MALLOC(sizeof(PWMInstance));
	 memset(PWM, 0, sizeof(PWMInstance));
	 
	 PWM->htim = config->htim;
	 PWM->channel = config->channel;
	 PWM->period = config->period;
	 if(config->callback)
	    PWM->callback = config->callback;
	 PWM->id = config->id;
     PWM->tclk = PWMSelectTclk(PWM->htim);

    // 启动PWM
    HAL_TIM_PWM_Start(PWM->htim, PWM->channel);
    PWMSetPeriod(PWM, PWM->period);
    PWMSetDutyRatio(PWM, PWM->dutyratio);
	 pwm_instance[idx++] = PWM;
	 return PWM;
}

/* 只是对HAL的函数进行了形式上的封装 */
void PWMStart(PWMInstance *pwm)
{
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
}

/* 只是对HAL的函数进行了形式上的封装 */
void PWMStop(PWMInstance *pwm)
{
    HAL_TIM_PWM_Stop(pwm->htim, pwm->channel);
}

/*
 * @brief 设置pwm周期
 *
 * @param pwm pwm实例
 * @param period 周期 单位 s
 */
void PWMSetPeriod(PWMInstance *pwm, float period)
{
    __HAL_TIM_SetAutoreload(pwm->htim, period*((pwm->tclk)/(pwm->htim->Init.Prescaler+1)));
}

/*
    * @brief 设置pwm占空比
    *
    * @param pwm pwm实例
    * @param dutyratio 占空比 0~1
*/
void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio)
{
    __HAL_TIM_SetCompare(pwm->htim, pwm->channel, dutyratio * (pwm->htim->Instance->ARR));
}

/* 只是对HAL的函数进行了形式上的封装 */
void PWMStartDMA(PWMInstance *pwm, uint32_t *pData, uint32_t Size)
{
    HAL_TIM_PWM_Start_DMA(pwm->htim, pwm->channel, pData, Size);
}

// 设置pwm对应定时器时钟源频率
//tim2~7,12~14:APB1  tim1,8~11:APB2
static uint32_t PWMSelectTclk(TIM_HandleTypeDef *htim)
{
    uintptr_t tclk_temp  = ((uintptr_t)((htim)->Instance));
    if (
            (tclk_temp <= (APB1PERIPH_BASE + 0x2000UL)) &&
            (tclk_temp >= (APB1PERIPH_BASE + 0x0000UL)))
    {
        return (HAL_RCC_GetPCLK1Freq() * (APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos] == 0 ? 1 : 2));
    }
    else if (
            ((tclk_temp <= (APB2PERIPH_BASE + 0x0400UL)) &&
             (tclk_temp >= (APB2PERIPH_BASE + 0x0000UL))) ||
            ((tclk_temp <= (APB2PERIPH_BASE + 0x4800UL)) &&
             (tclk_temp >= (APB2PERIPH_BASE + 0x4000UL))))
    {
        return (HAL_RCC_GetPCLK2Freq() * (APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos] == 0 ? 1 : 2));
    }
    return 0;
}
