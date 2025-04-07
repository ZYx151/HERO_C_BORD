#include "bsp_buzzer.h"
#include "main.h"
extern TIM_HandleTypeDef htim4;
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
void buzzer_note(uint16_t note,float volume)
{
    if(volume > 1.0f)
    {
        volume = 1.0f;
    }else if(volume < 0.0f)
    {
        volume = 0.0f;
    }
    // ���ö�ʱ��
    __HAL_TIM_DISABLE(&htim4);

    // ���ö�ʱ��������
    htim4.Instance->CNT = 0;
    
    // �����Զ���װ�ؼĴ�����ARR�����Կ���PWM�źŵ�Ƶ��
    htim4.Instance->ARR = (8*21000 / note - 1) * 1u;
    
    // ���ñȽϼĴ�����CCR3�����Կ���PWM�źŵ�ռ�ձ�
    htim4.Instance->CCR3 = (8*10500 / note - 1) * volume * 1u;
    
    // �������ö�ʱ��
    __HAL_TIM_ENABLE(&htim4);
    
    // ����PWM�ź�
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
