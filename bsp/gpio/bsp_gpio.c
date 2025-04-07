#include "bsp_gpio.h"
#include "RMLibHead.h"
#include "bsp_def.h"

static uint8_t idx;
static GPIOInstance gpio_instance[GPIO_MX_DEVICE_NUM] = {NULL};

void BSP_GPIO_Init()
{  // 初始化固定功能的GPIO
	gpio_instance[0].GPIOx    = GPIO_0_BASE;
	gpio_instance[0].GPIO_Pin = GPIO_0_PIN;
	gpio_instance[0].gpio_mode = GPIO_0_MODE;
	
	gpio_instance[1].GPIOx    = GPIO_1_BASE;
	gpio_instance[1].GPIO_Pin = GPIO_1_PIN;
	gpio_instance[1].gpio_mode = GPIO_1_MODE;
    
	gpio_instance[2].GPIOx    = GPIO_2_BASE;
	gpio_instance[2].GPIO_Pin = GPIO_2_PIN;
	gpio_instance[2].gpio_mode = GPIO_2_MODE;
    
	gpio_instance[3].GPIOx    = GPIO_3_BASE;
	gpio_instance[3].GPIO_Pin = GPIO_3_PIN;
	gpio_instance[3].gpio_mode = GPIO_3_MODE;

	gpio_instance[4].GPIOx    = GPIO_4_BASE;
	gpio_instance[4].GPIO_Pin = GPIO_4_PIN;
	gpio_instance[4].gpio_mode = GPIO_4_MODE;

	gpio_instance[5].GPIOx    = GPIO_5_BASE;
	gpio_instance[5].GPIO_Pin = GPIO_5_PIN;
	gpio_instance[5].gpio_mode = GPIO_5_MODE;
	
	idx = 6;
}

/**
 * @brief EXTI中断回调函数,根据GPIO_Pin找到对应的GPIOInstance,并调用模块回调函数(如果有)
 * @note 如何判断具体是哪一个GPIO的引脚连接到这个EXTI中断线上?
 *       一个EXTI中断线只能连接一个GPIO引脚,因此可以通过GPIO_Pin来判断,PinX对应EXTIX
 *       一个Pin号只会对应一个EXTI,详情见gpio.md
 * @param GPIO_Pin 发生中断的GPIO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 如有必要,可以根据pinstate和HAL_GPIO_ReadPin来判断是上升沿还是下降沿/rise&fall等
    GPIOInstance *gpio;
    for (size_t i = 0; i < idx; i++)
    {
        gpio = &gpio_instance[i];
        if (gpio->GPIO_Pin == GPIO_Pin && gpio->gpio_model_callback != NULL)
        {
            gpio->gpio_model_callback(gpio);
            return;
        }
    }
}

GPIOInstance *GPIORegister(GPIO_Init_Config_s *GPIO_config)
{
	if(idx == 0)
		BSP_GPIO_Init();
	if(idx >= GPIO_MX_DEVICE_NUM)
		return 0;
	
    for ( uint8_t i = 0; i < idx; i ++)
	{
		if(GPIO_config->GPIOx == gpio_instance[i].GPIOx && GPIO_config->GPIO_Pin == gpio_instance[i].GPIO_Pin)
		{
			gpio_instance[i].id = GPIO_config->id;
			gpio_instance[i].gpio_model_callback = GPIO_config->gpio_model_callback;
			return &gpio_instance[i];
		}
	}
    
    gpio_instance[idx].GPIOx = GPIO_config->GPIOx;
    gpio_instance[idx].GPIO_Pin = GPIO_config->GPIO_Pin;
    gpio_instance[idx].pin_state = GPIO_config->pin_state;
    gpio_instance[idx].id = GPIO_config->id;
    gpio_instance[idx].gpio_model_callback = GPIO_config->gpio_model_callback;
    idx++;
    return &gpio_instance[idx];
}

// ----------------- GPIO API -----------------
void GPIO_Toggel(GPIOInstance *_instance)
{
   HAL_GPIO_TogglePin(_instance->GPIOx, _instance->GPIO_Pin);
}

void GPIO_Set(GPIOInstance *_instance, uint8_t set)
{
    HAL_GPIO_WritePin(_instance->GPIOx, _instance->GPIO_Pin, set);
}

GPIO_PinState GPIO_Read(GPIOInstance *_instance)
{
    _instance->pin_state = HAL_GPIO_ReadPin(_instance->GPIOx, _instance->GPIO_Pin);
    return _instance->pin_state ;
}

void BSP_GPIO_Set(uint8_t gpio_index, uint8_t status) {
    if (gpio_instance[gpio_index].gpio_mode == GPIO_OUTPUT_MODE) {
        HAL_GPIO_WritePin(gpio_instance[gpio_index].GPIOx, gpio_instance[gpio_index].GPIO_Pin, status);
    }
}

void BSP_GPIO_Reinit(uint8_t gpio_index, uint8_t mode) {
    gpio_instance[gpio_index].GPIO_InitStruct.Pin = gpio_instance[gpio_index].GPIO_Pin;
    if(mode){
        gpio_instance[gpio_index].GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        gpio_instance[gpio_index].gpio_mode = GPIO_OUTPUT_MODE;
    }
    else{
        gpio_instance[gpio_index].GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        gpio_instance[gpio_index].gpio_mode = GPIO_INPUT_MODE;
    }
    gpio_instance[gpio_index].GPIO_InitStruct.Pull = GPIO_NOPULL;
    gpio_instance[gpio_index].GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(gpio_instance[gpio_index].GPIOx,&gpio_instance[gpio_index].GPIO_InitStruct);
}

void BSP_GPIO_Toggle(uint8_t gpio_index) {
    if (gpio_instance[gpio_index].gpio_mode == GPIO_OUTPUT_MODE) {
        HAL_GPIO_TogglePin(gpio_instance[gpio_index].GPIOx, gpio_instance[gpio_index].GPIO_Pin);
    }
}

void BSP_GPIO_Read(uint8_t gpio_index, uint8_t* data) {
    if (gpio_instance[gpio_index].gpio_mode == GPIO_INPUT_MODE) {
        (*data) = HAL_GPIO_ReadPin(gpio_instance[gpio_index].GPIOx, gpio_instance[gpio_index].GPIO_Pin);
    }
}