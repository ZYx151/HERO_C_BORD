#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "gpio.h"
#include "stdio.h"

#define GPIO_MX_DEVICE_NUM 8


/**
 * @define 用于判断中断来源
**/
typedef enum
{
	GPIO_EXTI_MODE_RISING,
	GPIO_EXTI_MODE_FALLING,
	GPIO_ECTI_MODE_RISING_FALLING,
	GPIO_EXTI_MODE_NONE,
}GPIO_EXTI_MODE_e;

/* GPIO实例结构体定义 */
typedef struct tmpgpio
{
	GPIO_TypeDef *GPIOx;         // GPIOA, GPIOC, GPIOB
    GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_PinState pin_state;    // 引脚状态
	uint16_t GPIO_Pin;          // GPIO引脚号 GPIO_PIN_0 GPIO_PIN_1 
    uint8_t gpio_mode; // GPIO模式
	
	GPIO_EXTI_MODE_e exti_mode; // 外部中断模式
	void (*gpio_model_callback)(struct tmpgpio *); // 中断回调函数
	void *id;                  // 区分不同的GPIO实例
} GPIOInstance;

/**
 * @brief GPIO初始化配置结构体定义
**/
typedef struct
{
    GPIO_TypeDef *GPIOx;        // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state;    // 引脚状态,Set,Reset not frequently used
    GPIO_EXTI_MODE_e exti_mode; // 外部中断模式 not frequently used
    uint16_t GPIO_Pin;          // 引脚号
    void (*gpio_model_callback)(GPIOInstance *); // exti中断回调函数
    void *id;                                    // 区分不同的GPIO实例

} GPIO_Init_Config_s;

/**
 * @brief 注册GPIO实例
 *
 * @param GPIO_config
 * @return GPIOInstance*
 */
GPIOInstance *GPIORegister(GPIO_Init_Config_s *GPIO_config);
/**
 * @brief GPIO API,切换GPIO电平
 *
 * @param _instance
 */
void GPIO_Toggel(GPIOInstance *_instance);
void BSP_GPIO_Toggle(uint8_t gpio_index);

/**
 * @brief 设置GPIO电平
 *
 * @param _instance
 */
void GPIO_Set(GPIOInstance *_instance, uint8_t set);
void BSP_GPIO_Set(uint8_t gpio_index, uint8_t status);
/**
 *  @brief 读取GPIO电平
**/
GPIO_PinState GPIO_Read(GPIOInstance *_instance);
void BSP_GPIO_Read(uint8_t gpio_index, uint8_t* data);

/**
 *  @brief 设置GPIO中断
**/
void BSP_GPIO_Reinit(uint8_t gpio_index, uint8_t mode);

#endif
