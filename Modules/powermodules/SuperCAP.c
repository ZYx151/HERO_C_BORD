/**
 * @file    SuperCAP.c
 * @author  yinzexu
 * @date    23-12-2024
 * @brief   用于主控板与超级电容控制板之间的通信
 */

#include "SuperCAP.h"
#include "bsp_dwt.h"

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

#ifdef HAL_CAN_MODULE_ENABLED

static float uint_to_float(int x_int, float x_min, float x_max, int bits);

static float ConsumedEnergy = 0.f;

/**
 * @brief 超级电容反馈数据包处理函数
 * @param[in,out] Sdef 超级电容反馈数据包结构体指针
 * @param[in,out] Data CAN数据帧指针
 * @return void
 */
void SuperCAP_Receive(Supercap_TypeDef *Sdef, uint8_t *Data)
{
	Sdef->in_v = ((float)((uint16_t)(Data[1] << 8U | Data[0]))) / 100.0f;
	Sdef->cap_v = ((float)((uint16_t)(Data[3] << 8U | Data[2]))) / 100.0f;
	Sdef->in_c = ((float)((uint16_t)(Data[5] << 8U | Data[4]))) / 100.0f;
	Sdef->cap_percent = Data[6];
	Sdef->out_c = uint_to_float(Data[7], -3, 9, 8);
	Sdef->out_p = Sdef->out_c * Sdef->in_v;
	Sdef->cap_power = SuperCAP_GetDCDCMaxPower(Sdef);
	Sdef->dt = DWT_GetDeltaT((void *)&Sdef->DWT_CNT)*1000;
	Sdef->sum_energy = SuperCAP_GetEnergyConsumed(Sdef, Sdef->dt);
}

/**
 * @brief 超级电容命令数据包发送函数
 * @param[in,out] hcan CAN句柄
 * @param[in] StdId 标准帧ID
 * @param[in] power_set 功率设置(单位:瓦,精度:0.01)
 * @param[in] buf_tmp 缓冲能量
 * @return HAL Status structures definition
 */
HAL_StatusTypeDef SuperCAP_Send(CAN_HandleTypeDef *hcan, uint32_t StdId, double power_set, double buf_tmp)
{
	CAN_TxHeaderTypeDef CAN_Tx = {
		.StdId = StdId,
		.ExtId = 0,
		.IDE = CAN_ID_STD,
		.RTR = CAN_RTR_DATA,
		.DLC = 8,
		.TransmitGlobalTime = DISABLE,
	};

	uint32_t TxMailbox = 0;

	uint8_t msg[8] = {0};

	uint16_t temp = (uint16_t)(power_set * 100 + 0.5);
	uint16_t buff_energy = (uint16_t)(buf_tmp * 100);

	msg[0] = (uint8_t)(temp >> 8U);
	msg[1] = (uint8_t)(temp & 0xff);
	msg[2] = (uint8_t)(buff_energy >> 8U);
	msg[3] = (uint8_t)(buff_energy & 0xff);

	return HAL_CAN_AddTxMessage(hcan, &CAN_Tx, msg, &TxMailbox);
}
/**
 * @brief 超级电容DCDC停止工作命令发送函数(在机器人死亡时循环调用)
 * @param[in,out] hcan CAN句柄
 * @param[in] StdId 标准帧ID
 * @return HAL Status structures definition
 */
HAL_StatusTypeDef SuperCAP_Stop(CAN_HandleTypeDef *hcan, uint32_t StdId)
{
	CAN_TxHeaderTypeDef CAN_Tx = {
		.StdId = StdId,
		.ExtId = 0,
		.IDE = CAN_ID_STD,
		.RTR = CAN_RTR_DATA,
		.DLC = 8,
		.TransmitGlobalTime = DISABLE,
	};

	uint32_t TxMailbox = 0;

	uint8_t msg[8] = {0x5a, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0xaf};

	return HAL_CAN_AddTxMessage(hcan, &CAN_Tx, msg, &TxMailbox);
}

/**
 * @brief 超级电容重启命令(每次上电有7次机会，超出后则认为DCDC模块已损坏不再允许重启)
 * @param[in,out] hcan CAN句柄
 * @param[in] StdId 标准帧ID
 * @return HAL Status structures definition
 */
HAL_StatusTypeDef SuperCAP_Force2Restart(CAN_HandleTypeDef *hcan, uint32_t StdId)
{
	CAN_TxHeaderTypeDef CAN_Tx = {
		.StdId = StdId,
		.ExtId = 0,
		.IDE = CAN_ID_STD,
		.RTR = CAN_RTR_DATA,
		.DLC = 8,
		.TransmitGlobalTime = DISABLE,
	};

	uint32_t TxMailbox = 0;

	uint8_t msg[8] = {0x1a, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f};

	return HAL_CAN_AddTxMessage(hcan, &CAN_Tx, msg, &TxMailbox);
}

/**
 * @brief 获取超级电容最大可提供的功率
 * @param[in,out] Sdef 超级电容反馈数据包结构体指针
 * @return 最大可提供的功率(若电容容量为0，返回0)
 */
float SuperCAP_GetDCDCMaxPower(Supercap_TypeDef *Sdef)
{
	if (Sdef->cap_percent)
		return (float)((Sdef->cap_v * 5.0f) > Sdef->in_v)
				   ? (float)(Sdef->cap_v * 5.0f)
				   : Sdef->in_v;
	else
		return 0;
}

/**
 * @brief 获取模拟缓冲能量值(需要每Xms调用一次,裁判系统的底盘功率结算频率为10Hz,因此建议为100ms)
 * @param[in,out] Sdef 超级电容反馈数据包结构体指针
 * @param[in] X 周期(单位: ms)
 * @param[in] PowerMargin 功率裕度
 * @return 缓冲能量
 */
//float SuperCAP_GetBufferEnergy(Supercap_TypeDef *Sdef, float X, float PowerMargin)
//{
//	static float Energy = 60.0f;

//	Energy += X * 0.001f * (-Sdef->in_c * Sdef->in_v + Sdef->set_power + PowerMargin);

//	if (Energy < 0)
//	{
//		Energy = 0;
//	}
//	else if (Energy > 60)
//		Energy = 60;

//	return Energy;
//}

/**
 * @brief 获取底盘消耗过的能量
 * @param[in,out] Sdef 超级电容反馈数据包结构体指针
 * @param[in] X 周期(单位: ms) 100
 * @return 底盘消耗的总能量
 */
float SuperCAP_GetEnergyConsumed(Supercap_TypeDef *Sdef, float X)
{
	ConsumedEnergy += X * 0.001f * Sdef->in_c * Sdef->in_v;

	return ConsumedEnergy;
}

/**
 * @brief 将消耗过的能量数值置零
 */
void SuperCAP_ResetEnergyConsumed()
{
	ConsumedEnergy = 0.f;
}

/**
 * @brief:       uint_to_float: 无符号整数转换为浮点数函数
 * @param[in]:   x_int: 待转换的无符号整数
 * @param[in]:   x_min: 范围最小值
 * @param[in]:   x_max: 范围最大值
 * @param[in]:   bits:  无符号整数的位数
 * @details:     将给定的无符号整数 x_int·在指定范围 [x min，x max] 内进行线性映射,映射结果为一个浮点数
 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

#endif // HAL_CAN_MODULE_ENABLED
