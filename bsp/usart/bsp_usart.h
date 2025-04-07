#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "main.h"
#include "usart.h"
#include "WatchDog.h"

#define DEVICE_USART_CNT   3     // C板至多分配3个串口
#define DEVICE_USART_DEPTH 2     // 串口接收数据深度
#define USART_RXBUFF_LIMIT 64 // 如果协议需要更大的buff,请修改这里

/* 串口模块回调函数 用于串口解析 */
typedef void(*usart_module_callback)(uint8_t *id);  // 推荐把模块解包放入任务中
/* 发送模式枚举 */
typedef enum {
	UASRT_TRANSFER_NONE = 0,
	USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
}USART_TRANSFER_MODE;

/* uasrt 初始化配置结构体 */
typedef struct
{
	UART_HandleTypeDef *usart_handle;      // 实例对应的uart_handle
	usart_module_callback module_callback; // 解析函数
	uint16_t data_len;   // 单包数据长度
} USART_Init_Config_s;

// 串口实例结构体,每个module都要包含一个实例.
// 由于串口是独占的点对点通信,所以不需要考虑多个module同时使用一个串口的情况,因此不用加入id;当然也可以选择加入,这样在bsp层可以访问到module的其他信息
#pragma pack(1)
typedef struct 
{
	uint8_t *recv_buff[DEVICE_USART_DEPTH];
	UART_HandleTypeDef *usart_handle;      // 实例对应的uart_handle
	uint8_t temp_depth;
	usart_module_callback module_callback; // 解析函数
	uint16_t data_len;   // 单包数据长度
}USARTInstance;
#pragma pack()
/**
 *
 * @bief  注册一个串口实例,返回串口实例
 *
 * @param _config 传入串口初始化结构体
 *
**/
USARTInstance *USARTRegister(USART_Init_Config_s *_config);

/**
 *
 * @bief  启动串口服务,需要传入一个串口实例
 *
 * @param init_ocnfig 传入串口初始化结构体
 *
**/
void USARTServiceInit(USARTInstance *_instance);

void USART_Transmit(USARTInstance *instance, uint8_t *send_buff, uint16_t data_len, uint8_t mode);

/**
 * @brief 串口空闲中断（中断回调）函数
 * @param 串口号
 * @retval None
 * @note  放在"stm32f4xx_it.c"里形如"void USART2_IRQHandler(void)"类的函数中，只要用了DMA接收的串口都放
 * 具体位置：系统调用的HAL_UART_IRQHandler函数下面，"USER CODE BEGIN USART1_IRQn 1"和"USER CODE END USART1_IRQn 1"两行注释之间
 */
void BSP_UART_IRQHandler(UART_HandleTypeDef *huart);



#endif