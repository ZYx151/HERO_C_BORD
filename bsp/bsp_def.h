#ifndef _BSP_DEF_H_
#define _BSP_DEF_H_

/*--------------------------------------------------bsp_gpio--------------------------------------------------*/
/**
 * @brief      :定义GPIO的输入与输出模式
 * @attention  :固定值，禁止更改
 */
#define GPIO_INPUT_MODE 0
#define GPIO_OUTPUT_MODE 1

/**
 * @brief      :主控用到的GPIO口的数目
 * @attention  :用到几个就填几个，如果更改的话需要在bsp_gpio.c中的函数里面加上额外的gpio配置，并添加如下的宏定义
 */
#define DEVICE_GPIO_CNT 9

/**
 * @brief      :gpio_ports[x]的宏定义配置，分别为引脚定义、输入还是输出模式
 * @attention  :gpio数目一定要和DEVICE_GPIO_CNT一致
 */
#define GPIO_0_BASE GPIOA
#define GPIO_0_PIN GPIO_PIN_4
#define GPIO_0_MODE GPIO_OUTPUT_MODE

#define GPIO_1_BASE GPIOB
#define GPIO_1_PIN GPIO_PIN_0
#define GPIO_1_MODE GPIO_OUTPUT_MODE

#define GPIO_2_BASE GPIOC
#define GPIO_2_PIN GPIO_PIN_8
#define GPIO_2_MODE GPIO_OUTPUT_MODE

#define GPIO_3_BASE GPIOH
#define GPIO_3_PIN GPIO_PIN_10
#define GPIO_3_MODE GPIO_OUTPUT_MODE

#define GPIO_4_BASE GPIOH
#define GPIO_4_PIN GPIO_PIN_11
#define GPIO_4_MODE GPIO_OUTPUT_MODE

#define GPIO_5_BASE GPIOH
#define GPIO_5_PIN GPIO_PIN_12
#define GPIO_5_MODE GPIO_OUTPUT_MODE

//#define GPIO_6_BASE GPIOI
//#define GPIO_6_PIN GPIO_PIN_6
//#define GPIO_6_MODE GPIO_OUTPUT_MODE

//#define GPIO_7_BASE GPIOI
//#define GPIO_7_PIN GPIO_PIN_7
//#define GPIO_7_MODE GPIO_OUTPUT_MODE

//#define GPIO_8_BASE GPIOC
//#define GPIO_8_PIN GPIO_PIN_6
//#define GPIO_8_MODE GPIO_OUTPUT_MODE

/**
 * @brief      :APP层和HAL层会调用的GPIO口的宏定义
 * @attention  :0代表gpio_ports[0]，1代表gpio_ports[1]，以此类推
 */
#define GPIO_BMI088_ACCEL_NS 0
#define GPIO_BMI088_GYRO_NS 1
#define GPIO_5V_OUTPUT 2
#define GPIO_LED1 3
#define GPIO_LED2 4
#define GPIO_LED3 5
//#define GPIO_SOFT_I2C_SCL 6
//#define GPIO_SOFT_I2C_SDA 7
//#define GPIO_VL53L0x_ENABLE 8

/*--------------------------------------------------bsp_pwm--------------------------------------------------*/
/**
 * @brief      :主控用到的PWM的数目
 * @attention  :用到几个就填几个，如果更改的话需要在bsp_pwm.c中的函数里面加上额外的pwm配置，并添加如下的宏定义
 */
#define DEVICE_PWM_CNT 3

/**
 * @brief      :用到DMA的PWM输出定时器口
 * @attention  :开哪个就填那个
 */
#define PWM_DMA_1 hdma_tim1_ch1

/**
 * @brief      :pwm_ports[0]的宏定义配置，分别为定时器索引、通道数
 * @attention  :pwm数目一定要和DEVICE_PWM_CNT一致
 *              0代表pwm_ports[0]，1代表pwm_ports[1]，以此类推
 */
#define PWM_0_BASE &htim10
#define PWM_0_CHANNEL TIM_CHANNEL_1
#define PWM_1_BASE &htim4
#define PWM_1_CHANNEL TIM_CHANNEL_3
#define PWM_2_BASE &htim1
#define PWM_2_CHANNEL TIM_CHANNEL_1
#define PWM_3_BASE &htim1
#define PWM_3_CHANNEL TIM_CHANNEL_2
#define PWM_4_BASE &htim1
#define PWM_4_CHANNEL TIM_CHANNEL_3
#define PWM_5_BASE &htim1
#define PWM_5_CHANNEL TIM_CHANNEL_4
#define PWM_6_BASE &htim8
#define PWM_6_CHANNEL TIM_CHANNEL_1
#define PWM_7_BASE &htim8
#define PWM_7_CHANNEL TIM_CHANNEL_2
#define PWM_8_BASE &htim8
#define PWM_8_CHANNEL TIM_CHANNEL_3

/**
 * @brief      :APP层和HAL层会调用的PWM口的宏定义
 */
#define PWM_BMI088_HEAT_PORT 0
#define PWM_BUZZER_PORT 1
#define PWM_SERVO_1_PORT 2  // 从远离DBUS端开始数
#define PWM_SERVO_2_PORT 3
#define PWM_SERVO_3_PORT 4
#define PWM_SERVO_4_PORT 5
#define PWM_SERVO_5_PORT 6
#define PWM_SERVO_6_PORT 7
#define PWM_SERVO_7_PORT 8


#endif
