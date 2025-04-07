/**
 * @file    SuperCAP.h
 * @author  yinzexu
 * @date    23-12-2024
 * @brief   用于主控板与超级电容控制板之间的通信
 */

/*
1.首先将SuperCAP.c和SuperCAP.h导入407/405主控板的工程文件中，之后用Supercap_TypeDef创建一个超级电容结构体，
  在CAN回调中标准帧ID为0x211的位置上调用SuperCAP_Receive函数。
  创建一个任务，以合适的频率调用SuperCAP_Send函数，标准帧ID为0x210。

2.超级电容命令数据包发送函数的参数三power_set的值应为当前底盘最大功率限制(按需可减去10瓦)，
  由于不同等级或模式下底盘最大功率限制不同，需从裁判系统中读取该数值然后传入此函数。

3.超级电容反馈数据包处理函数的标准帧ID为0x211。

4.缓冲能量计算函数需要每Xms调用一次,裁判系统的底盘功率结算频率为10Hz,因此建议为100ms。

5.机器人发生撞击后机器人底盘的pidout可能会发生突变，使得电路中产生一个极大的电流，若超过超电DCDC承受范围则会导致DCDC自动关闭。

6.机器人底盘调试若遇超调问题(直观感受为底盘功率曲线毛刺过多)，可减小pid的kp，同时增大速度期望值，可能会有很大的改观。

7.407或405如接收到电容百分比为0，则说明DCDC控制板已关闭，可通过SuperCAP_Force2Restart尝试重启，每次上电有7次机会，若超出则认定DCDC出现故障不再允许重启。

8.机器人死亡后需立即调用SuperCAP_Stop函数。可以通过读取裁判系统数据判断机器人是否死亡。

9.机器人存活时必须保持上下板与超电之间的通信，否则DCDC会停止工作。
*/

#ifndef _SuperCAP_H_
#define _SuperCAP_H_

#if defined(__ARMCC_VERSION) || defined(__GNUC__)
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
    defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F410Tx) || defined(STM32F410Cx) || \
    defined(STM32F410Rx) || defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F469xx) || \
    defined(STM32F479xx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F412Rx) || \
    defined(STM32F412Vx) || defined(STM32F413xx) || defined(STM32F423xx)
#include <stm32f4xx.h>
#elif defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F031x6) || defined(STM32F038xx) || \
    defined(STM32F042x6) || defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) ||   \
    defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) ||   \
    defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx) || defined(STM32F030xC)
#include <stm32f0xx.h>
#elif defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101x6) || defined(STM32F101xB) || \
    defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102x6) || defined(STM32F102xB) ||   \
    defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) ||   \
    defined(STM32F105xC) || defined(STM32F107xC)
#include <stm32f1xx.h>
#elif defined(STM32F301x8) || defined(STM32F302x8) || defined(STM32F302xC) || defined(STM32F302xE) || \
    defined(STM32F303x8) || defined(STM32F303xC) || defined(STM32F303xE) || defined(STM32F373xC) ||   \
    defined(STM32F334x8) || defined(STM32F318xx) || defined(STM32F328xx) || defined(STM32F358xx) ||   \
    defined(STM32F378xx) || defined(STM32F398xx)
#include <stm32f3xx.h>
#elif defined(STM32H743xx) || defined(STM32H753xx) || defined(STM32H750xx) || defined(STM32H742xx) || \
    defined(STM32H745xx) || defined(STM32H755xx) || defined(STM32H747xx) || defined(STM32H757xx) ||   \
    defined(STM32H7B0xx) || defined(STM32H7B0xxQ) || defined(STM32H7A3xx) || defined(STM32H7B3xx) ||  \
    defined(STM32H7A3xxQ) || defined(STM32H7B3xxQ) || defined(STM32H735xx) || defined(STM32H733xx) || \
    defined(STM32H730xx) || defined(STM32H730xxQ) || defined(STM32H725xx) || defined(STM32H723xx)
#include <stm32h7xx.h>
#endif
#endif

#ifdef HAL_CAN_MODULE_ENABLED

#include "can.h"

/**
 * @brief 超级电容结构体
 */
typedef struct
{
    float in_v;          // 输入电压
    float cap_v;         // 电容电压
    float in_c;          // 输入电流
    float out_c;         // 输出电流
    float out_p;         // 输出功率
    uint8_t cap_percent; // 电容百分比
	float cap_power;    // 最大可提供功率
	float sum_energy;    // 整体使用能量
	float dt;
	uint32_t DWT_CNT; // 用于DWT计时
} Supercap_TypeDef;

/**
 * @brief 超级电容反馈数据包处理函数
 * @param[in,out] Sdef 超级电容反馈数据包结构体指针
 * @param[in,out] Data CAN数据帧指针
 * @return void
 */
void SuperCAP_Receive(Supercap_TypeDef *Sdef, uint8_t *Data);

/**
 * @brief 超级电容命令数据包发送函数
 * @param[in,out] hcan CAN句柄
 * @param[in] StdId 标准帧ID
 * @param[in] power_set 功率设置(单位:瓦,精度:0.01)
 * @return HAL Status structures definition
 */
HAL_StatusTypeDef SuperCAP_Send(CAN_HandleTypeDef *hcan, uint32_t StdId, double power_set, double buf_tmp);

/**
 * @brief 超级电容DCDC停止工作命令发送函数(在机器人死亡时循环调用)
 * @param[in,out] hcan CAN句柄
 * @param[in] StdId 标准帧ID
 * @return HAL Status structures definition
 */
HAL_StatusTypeDef SuperCAP_Stop(CAN_HandleTypeDef *hcan, uint32_t StdId);

/**
 * @brief 超级电容重启命令(每次上电有7次机会，超出后则认为DCDC模块已损坏不再允许重启)
 * @param[in,out] hcan CAN句柄
 * @param[in] StdId 标准帧ID
 * @return HAL Status structures definition
 */
HAL_StatusTypeDef SuperCAP_Force2Restart(CAN_HandleTypeDef *hcan, uint32_t StdId);

/**
 * @brief 获取超级电容最大可提供的功率
 * @param[in,out] Sdef 超级电容反馈数据包结构体指针
 * @return 最大可提供的功率(若电容容量为0，返回0)
 */
float SuperCAP_GetDCDCMaxPower(Supercap_TypeDef *Sdef);

/**
 * @brief 获取模拟缓冲能量值(需要每Xms调用一次,裁判系统的底盘功率结算频率为10Hz,因此建议为100ms)
 * @param[in,out] Sdef 超级电容反馈数据包结构体指针
 * @param[in] X 周期(单位: ms)
 * @param[in] PowerMargin 功率裕度
 * @return 缓冲能量
 */
//float SuperCAP_GetBufferEnergy(Supercap_TypeDef *Sdef, float X, float PowerMargin);

/**
 * @brief 获取底盘消耗过的能量
 * @param[in,out] Sdef 超级电容反馈数据包结构体指针
 * @param[in] X 周期(单位: ms)
 * @return 底盘消耗的总能量
 */
float SuperCAP_GetEnergyConsumed(Supercap_TypeDef *Sdef, float X);

/**
 * @brief 将消耗过的能量数值置零
 */
void SuperCAP_ResetEnergyConsumed();

#endif // HAL_CAN_MODULE_ENABLED

#endif //_SuperCAP_H_
