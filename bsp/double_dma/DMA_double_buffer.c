/** 
 * @file    DMA_double_buffer.c
 * @brief   DMA循环双缓冲模式启动程序
 * @author  Wzx
 * @date    2024-2-20
 * @detail  由于Hal库中没有直接用于启动DMA双缓冲模式的函数，此文件仿照Hal库实现此功能
 * @note    与Hal库一样，直接调用HAL_UART_Receive_DMA_double（）即可启动DMA循环双缓冲模式
            触发接收完成中断回调进入弱函数HAL_UART_RxCpltCallback(huart)里可以让CPU处理数据
            通过if((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)可判断DMA当前用的缓冲区
 */
#include "DMA_double_buffer.h"
#include "robot_def.h"

static void UART_EndTxTransfer(UART_HandleTypeDef *huart);
static void UART_EndRxTransfer(UART_HandleTypeDef *huart);
//static void DMA_MultiBufferSetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
static void UART_DMAReceiveCplt_M0(DMA_HandleTypeDef *hdma);
static void UART_DMAReceiveCplt_M1(DMA_HandleTypeDef *hdma);
static void UART_DMAError(DMA_HandleTypeDef *hdma);

HAL_StatusTypeDef HAL_UART_Receive_DMA_double(UART_HandleTypeDef *huart, uint8_t *pData, uint8_t *pData1 ,uint16_t Size)
{
  /* Check that a Rx process is not already ongoing */
  if (huart->RxState == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return HAL_ERROR;
    }
    
    /* Process Locked */
    __HAL_LOCK(huart);
    
    /* Set Reception type to Standard reception */
    huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
    
    return (UART_Start_Receive_DMA_double(huart, pData, pData1, Size));
  }
  else
  {
    return HAL_BUSY;
  }
}

HAL_StatusTypeDef UART_Start_Receive_DMA_double(UART_HandleTypeDef *huart, uint8_t *pData, uint8_t *pData1, uint16_t Size)
{
  uint32_t *tmp;
  uint32_t *tmp1;

  huart->pRxBuffPtr = pData;
  huart->RxXferSize = Size;

  huart->ErrorCode = HAL_UART_ERROR_NONE;
  huart->RxState = HAL_UART_STATE_BUSY_RX;

  /* Set the UART DMA transfer complete callback */
  huart->hdmarx->XferCpltCallback   = UART_DMAReceiveCplt_M0;
  huart->hdmarx->XferM1CpltCallback = UART_DMAReceiveCplt_M1;
  /* Set the UART DMA Half transfer complete callback */
  huart->hdmarx->XferHalfCpltCallback = NULL;

  /* Set the DMA error callback */
  huart->hdmarx->XferErrorCallback = UART_DMAError;

  /* Set the DMA abort callback */
  huart->hdmarx->XferAbortCallback = NULL;

  /* Enable the DMA stream */
  tmp  = (uint32_t *)&pData;
  tmp1 = (uint32_t *)&pData1;
  
  // 注册双缓冲区
  HAL_DMAEx_MultiBufferStart_IT(huart->hdmarx, (uint32_t)&huart->Instance->DR, *(uint32_t *)tmp,*(uint32_t *)tmp1 ,Size);
  
  /* Clear the Overrun flag just before enabling the DMA Rx request: can be mandatory for the second transfer */
  __HAL_UART_CLEAR_OREFLAG(huart);
  
  /* Process Unlocked */
  __HAL_UNLOCK(huart);

  if (huart->Init.Parity != UART_PARITY_NONE)
  {
    /* Enable the UART Parity Error Interrupt */
    ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);
  }

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

  /* Enable the DMA transfer for the receiver request by setting the DMAR bit
  in the UART CR3 register */
  ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  return HAL_OK;
}

/**
  * @brief  End ongoing Tx transfer on UART peripheral (following error detection or Transmit completion).
  * @param  huart UART handle.
  * @retval None
  */
static void UART_EndTxTransfer(UART_HandleTypeDef *huart)
{
  /* Disable TXEIE and TCIE interrupts */
  ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));

  /* At end of Tx process, restore huart->gState to Ready */
  huart->gState = HAL_UART_STATE_READY;
}

/**
  * @brief  End ongoing Rx transfer on UART peripheral (following error detection or Reception completion).
  * @param  huart UART handle.
  * @retval None
  */
static void UART_EndRxTransfer(UART_HandleTypeDef *huart)
{
  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
  ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

  /* In case of reception waiting for IDLE event, disable also the IDLE IE interrupt source */
  if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
  }
   
  /* At end of Rx process, restore huart->RxState to Ready */
  huart->RxState = HAL_UART_STATE_READY;
  huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
}


/**
  * @brief  Set the DMA Transfer parameter.
  * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval HAL status
  */
//static void DMA_MultiBufferSetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
//{
//  /* Configure DMA Stream data length */
//  hdma->Instance->NDTR = DataLength;
//  
//  /* Peripheral to Memory */
//  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
//  {
//    /* Configure DMA Stream destination address */
//    hdma->Instance->PAR = DstAddress;
//    
//    /* Configure DMA Stream source address */
//    hdma->Instance->M0AR = SrcAddress;
//  }
//  /* Memory to Peripheral */
//  else
//  {
//    /* Configure DMA Stream source address */
//    hdma->Instance->PAR = SrcAddress;
//    
//    /* Configure DMA Stream destination address */
//    hdma->Instance->M0AR = DstAddress;
//  }
//}

/**
  * @brief  DMA memory 0 UART receive process complete callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void UART_DMAReceiveCplt_M0(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  /* DMA Normal mode*/
  if ((hdma->Instance->CR & DMA_SxCR_CIRC) == 0U)
  {
    huart->RxXferCount = 0U;

    /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
    ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_PEIE);
    ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

    /* Disable the DMA transfer for the receiver request by setting the DMAR bit
       in the UART CR3 register */
    ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* At end of Rx process, restore huart->RxState to Ready */
    huart->RxState = HAL_UART_STATE_READY;

    /* If Reception till IDLE event has been selected, Disable IDLE Interrupt */
    if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
    {
      ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
    }
  }

  /* Check current reception Mode :
     If Reception till IDLE event has been selected : use Rx Event callback */
  if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
  {
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    /*Call registered Rx Event callback*/
    huart->RxEventCallback(huart, huart->RxXferSize);
#else
    /*Call legacy weak Rx Event callback*/
    HAL_UARTEx_RxEventCallback(huart, huart->RxXferSize);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
  }
  else
  {
    /* In other cases : use Rx Complete callback */
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    /*Call registered Rx complete callback*/
    huart->RxCpltCallback(huart);
#else
    /*Call legacy weak Rx complete callback*/
    HAL_UART_RxCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  DMA memory 1 UART receive process complete callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void UART_DMAReceiveCplt_M1(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  /* DMA Normal mode*/
  if ((hdma->Instance->CR & DMA_SxCR_CIRC) == 0U)
  {
    huart->RxXferCount = 0U;

    /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
    ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_PEIE);
    ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

    /* Disable the DMA transfer for the receiver request by setting the DMAR bit
       in the UART CR3 register */
    ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* At end of Rx process, restore huart->RxState to Ready */
    huart->RxState = HAL_UART_STATE_READY;

    /* If Reception till IDLE event has been selected, Disable IDLE Interrupt */
    if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
    {
      ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
    }
  }

  /* Check current reception Mode :
     If Reception till IDLE event has been selected : use Rx Event callback */
  if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
  {
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    /*Call registered Rx Event callback*/
    huart->RxEventCallback(huart, huart->RxXferSize);
#else
    /*Call legacy weak Rx Event callback*/
    HAL_UARTEx_RxEventCallback(huart, huart->RxXferSize);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
  }
  else
  {
    /* In other cases : use Rx Complete callback */
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    /*Call registered Rx complete callback*/
    huart->RxCpltCallback(huart);
#else
    /*Call legacy weak Rx complete callback*/
    HAL_UART_RxCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
  }
}

static void UART_DMAError(DMA_HandleTypeDef *hdma)
{
  uint32_t dmarequest = 0x00U;
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Stop UART DMA Tx request if ongoing */
  dmarequest = HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAT);
  if ((huart->gState == HAL_UART_STATE_BUSY_TX) && dmarequest)
  {
    huart->TxXferCount = 0x00U;
    UART_EndTxTransfer(huart);
  }

  /* Stop UART DMA Rx request if ongoing */
  dmarequest = HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR);
  if ((huart->RxState == HAL_UART_STATE_BUSY_RX) && dmarequest)
  {
    huart->RxXferCount = 0x00U;
    UART_EndRxTransfer(huart);
  }

  huart->ErrorCode |= HAL_UART_ERROR_DMA;
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  /*Call registered error callback*/
  huart->ErrorCallback(huart);
#else
  /*Call legacy weak error callback*/
  HAL_UART_ErrorCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
}

