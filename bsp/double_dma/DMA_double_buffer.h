#ifndef __DMA_DOUBLE_BUFFER_H
#define __DMA_DOUBLE_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__ARMCC_VERSION) || defined(__GNUC__)
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
    defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F410Tx) || defined(STM32F410Cx) || \
    defined(STM32F410Rx) || defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F469xx) || \
    defined(STM32F479xx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F412Rx) || \
    defined(STM32F412Vx) || defined(STM32F413xx) || defined(STM32F423xx)
#include <stm32f4xx.h>
#elif defined(STM32F030x6)|| defined(STM32F030x8)|| defined(STM32F031x6)|| defined(STM32F038xx)|| \
      defined(STM32F042x6)|| defined(STM32F048xx)|| defined(STM32F051x8)|| defined(STM32F058xx)|| \
      defined(STM32F070x6)|| defined(STM32F070xB)|| defined(STM32F071xB)|| defined(STM32F072xB)|| \
      defined(STM32F078xx)|| defined(STM32F091xC)|| defined(STM32F098xx)|| defined(STM32F030xC)
#include <stm32f0xx.h>
#elif defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101x6) || defined(STM32F101xB) || \
      defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102x6) || defined(STM32F102xB) || \
      defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
      defined(STM32F105xC) || defined(STM32F107xC)
#include <stm32f1xx.h>
#elif defined(STM32F301x8) || defined(STM32F302x8) || defined(STM32F302xC) || defined(STM32F302xE) || \
      defined(STM32F303x8) || defined(STM32F303xC) || defined(STM32F303xE) || defined(STM32F373xC) || \
      defined(STM32F334x8) || defined(STM32F318xx) || defined(STM32F328xx) || defined(STM32F358xx) || \
      defined(STM32F378xx) || defined(STM32F398xx)
#include <stm32f3xx.h>
#elif defined(STM32H743xx) || defined(STM32H753xx) || defined(STM32H750xx) || defined(STM32H742xx) || \
      defined(STM32H745xx) || defined(STM32H755xx) || defined(STM32H747xx) || defined(STM32H757xx) || \
      defined(STM32H7B0xx) || defined(STM32H7B0xxQ)|| defined(STM32H7A3xx) || defined(STM32H7B3xx) || \
      defined(STM32H7A3xxQ)|| defined(STM32H7B3xxQ)|| defined(STM32H735xx) || defined(STM32H733xx) || \
      defined(STM32H730xx) || defined(STM32H730xxQ)|| defined(STM32H725xx) || defined(STM32H723xx)
#include <stm32h7xx.h>
#endif
#endif
/**
  * @brief  启用DMA循环双缓冲模式接收数据
  * @note   无论DMA初始化是否为循环模式，当启用双缓冲模式时，将强制更改为循环模式
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer_M0 (u8 or u16 data elements).
  * @param  pData Pointer to data buffer_M1 (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @note   When the UART parity is enabled (PCE = 1) the received data contains the parity bit.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_Receive_DMA_double(UART_HandleTypeDef *huart, uint8_t *pData, uint8_t *pData1 ,uint16_t Size);

/**
  * @brief  Start Receive operation in DMA mode.
  * @note   This function could be called by all HAL UART API providing reception in DMA mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         UART Handle is assumed as Locked.
  * @param  huart UART handle.
  * @param  pData Pointer to data buffer_M0 (u8 or u16 data elements).
  * @param  pData Pointer to data buffer_M1 (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef UART_Start_Receive_DMA_double(UART_HandleTypeDef *huart, uint8_t *pData, uint8_t *pData1, uint16_t Size);

#ifdef __cplusplus
}
#endif
#endif
