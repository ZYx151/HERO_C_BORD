#include "bsp_usart.h"

/* usart实例,所有注册了usart的模块信息会被保存在这里 */
static uint8_t idx;
USARTInstance *usart_instance[DEVICE_USART_CNT];

/**
 *
 * @brief 启动串口服务,会在每个实例注册之后自动启动接收 (仅DMA)
 *
 * @param _instance 模块的串口实例
 *
**/
void USARTSerxiceInit(USARTInstance *_instance)
{
	// HAL库b 的BUG处理，对于DMA需要先DeInit再Init，不然GG
	HAL_DMA_DeInit(_instance->usart_handle->hdmatx);
	HAL_DMA_Init(_instance->usart_handle->hdmatx);
	HAL_UART_DMAStop(_instance->usart_handle);
	//使能串口空闲中断
	__HAL_UART_ENABLE_IT(_instance->usart_handle, UART_IT_IDLE);  //使能串口空闲中断
	//开启DMA接收
	HAL_UART_Receive_DMA(_instance->usart_handle, _instance->recv_buff[0],_instance->data_len);
}

USARTInstance *USARTRegister(USART_Init_Config_s *_config)
{
   	USARTInstance *instance = (USARTInstance *)RMLIB_MALLOC(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));
   	/* 对新键的队列进行初始化 */
    instance->data_len = _config->data_len;
	if (_config->module_callback != 0)
		 instance->module_callback = _config->module_callback;
    for (int i = 0; i < DEVICE_USART_DEPTH; i++) {
        instance->recv_buff[i] = RMLIB_MALLOC(_config->data_len);
        if (instance->recv_buff[i] == NULL) {
            for (int j = 0; j < i; j++)
                RMLIB_FREE(instance);
            return 0;
        }
    }
     
   	instance->usart_handle = _config->usart_handle;
    usart_instance[idx++] = instance;
	USARTSerxiceInit(instance);
	return instance;
}

/* 串口发送 仅进行了简单包装 */
void USART_Transmit(USARTInstance *instance, uint8_t *send_buff, uint16_t data_len, uint8_t mode)
{
	switch (mode) {
		case USART_TRANSFER_DMA :
			HAL_UART_Transmit_DMA(instance->usart_handle, send_buff, data_len);
			break;
		case USART_TRANSFER_IT :
			HAL_UART_Transmit_IT(instance->usart_handle, send_buff, data_len);
			break;
		case USART_TRANSFER_BLOCKING :
			HAL_UART_Transmit(instance->usart_handle, send_buff, data_len, 100);
			break;
		default: break;;
	}
}

/**
 * @brief 每次dma/idle中断发生时，都会调用此函数.对于每个uart实例会调用对应的回调进行进一步的处理
 *
 * @note  通过__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
 *        这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
 *        我们只希望处理，因此直接关闭DMA半传输中断第一种和第三种情况
 *
**/
void BSP_UART_IDLECallback(uint8_t uart_index, UART_HandleTypeDef *huart)
{
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
  {
      __HAL_UART_CLEAR_IDLEFLAG(huart);  //清除空闲中断标志（否则会一直不断进入中断）  尝试 HAL_UARTEx_ReceiveToIdle_DMA 接收
	  HAL_UART_DMAStop(huart);
	  if (usart_instance[uart_index]->temp_depth == 0)
       {
          usart_instance[uart_index]->temp_depth = 1;
		  if(usart_instance[uart_index]->module_callback != 0)
			  usart_instance[uart_index]->module_callback(usart_instance[uart_index]->recv_buff[0]);
	      HAL_UART_Receive_DMA(huart, usart_instance[uart_index]->recv_buff[1],usart_instance[uart_index]->data_len);
	   }
	  else
	  {
         usart_instance[uart_index]->temp_depth = 0;
		 if(usart_instance[uart_index]->module_callback != 0)
  		      usart_instance[uart_index]->module_callback(usart_instance[uart_index]->recv_buff[1]);
	     HAL_UART_Receive_DMA(huart, usart_instance[uart_index]->recv_buff[0],usart_instance[uart_index]->data_len);
	  }
  
	  //防止串口热插拔导致ORE错误卡死在中断里
	__HAL_DMA_DISABLE_IT( usart_instance[uart_index]->usart_handle->hdmarx, DMA_IT_HT);
	if((READ_REG(huart->Instance->SR) & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE))){
	  (void)huart->Instance->SR;
	  (void)huart->Instance->DR;
	}
  }
}

void BSP_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	for(uint8_t i = 0; i < idx ; i++) {
		if (huart == usart_instance[i]->usart_handle) {
			BSP_UART_IDLECallback(i, huart);
		}
    }
}


/**
 * @brief 当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
 *
 * @note  最常见的错误:奇偶校验/溢出/帧错误
 *
 * @param huart 发生错误的串口
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff[usart_instance[i]->temp_depth], usart_instance[i]->data_len);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            return;
        }
    }
}

