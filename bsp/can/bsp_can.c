#include "bsp_can.h"
#include "bsp_dwt.h"
#include "main.h"
#include "can.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// 在CAN产生接收中断会遍历数组,选出hcan和rxid与发生中断的实例相同的那个,调用其回调函数
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx, can1_idx, can2_idx; // 全局CAN实例索引,每次有新的模块注册会自增

/* ----------------two static function called by CANRegister()-------------------- */

/**
 * @brief 添加过滤器以实现对特定id的报文的接收,会被CANRegister()调用
 *        给CAN添加过滤器后,BxCAN会根据接收到的报文的id进行消息过滤,符合规则的id会被填入FIFO触发中断
 *
 * @note f408的bxCAN有28个过滤器,这里将其配置为前14个过滤器给CAN1使用,后14个被CAN2使用
 *       初始化时,can1会被分配到FIFO1,can2会被分配到FIFO1
 *       注册到CA12的模块使用过滤器0-13,CAN2使用过滤器14-27
 *
 * @param _instance can instance owned by specific module
 */
static void CANAddFilter(CANInstance *_instance)
{
    CAN_FilterTypeDef can_filter_conf;
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用  // CAN_FILTERMODE_IDMASK 
    
    can_filter_conf.FilterMode =  CAN_FILTERMODE_IDLIST;                                                      // 使用id 列表list模式,即只有将rxid添加到过滤器中才会接收到,其他报文会被过滤
    can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;                                                      // 使用16位id模式,即只有低16位有效
    can_filter_conf.FilterFIFOAssignment = (idx & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;                           // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
    can_filter_conf.SlaveStartFilterBank = 14;                                                                // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机)
    can_filter_conf.FilterIdLow = _instance->rx_id << 5;                                                      // 过滤器寄存器的低16位,因为使用STDID,所以只有低11位有效,高5位要填0
    can_filter_conf.FilterBank = _instance->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++); // 根据can_handle判断是CAN1还是CAN2,然后自增
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;                                                     // 启用过滤器
    
	HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf);
}

/**
 * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
 *
 * @note 此函数会启动CAN2和CAN2,开启CAN1和CAN2的FIFO0 or FIFO1溢出通知
 *
 */
static void CANServiceInit()
{
	HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}
/* ----------------------- two extern callable function -----------------------*/

CANInstance *CANRegister(CAN_Init_Config_s config)
{
    // 检测重复ID

    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance)); // 分配空间
    memset(instance, 0, sizeof(CANInstance));                           // 分配的空间未必是0,所以要先清空
	
    // 进行发送报文的配置
    instance->txconf.StdId = config.tx_id;    // 发送id
    instance->txconf.IDE = CAN_ID_STD;        // 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    instance->txconf.RTR = CAN_RTR_DATA;      // 发送数据帧
    instance->txconf.DLC = 0x008;             // 默认发送长度为8
    instance->txconf.TransmitGlobalTime = DISABLE;
	// 设置回调函数和接收发送id
    instance->can_handle = config.can_handle;
	instance->tx_id = config.tx_id;
    instance->rx_id = config.rx_id;
    instance->can_module_callback = config.can_module_callback;
    instance->id = config.id;
	if( instance->rx_len == 0 ) instance->rx_len = 8;
	config.can_handle == &hcan1 ? can1_idx : can2_idx++; // 将实例保存到can_instance中
	
	can_instance[idx] = instance;
    if( idx++ == 0)
        CANServiceInit(); // 第一次注册,先进行硬件初始化
   CANAddFilter(instance);         // 添加CAN过滤器规则
     
    
    return instance; // 返回can实例指针
}

/**
 * @brief CAN发送消息函数
 */
HAL_StatusTypeDef CANTransmit(CANInstance *_instance, float timeout, uint8_t *tx_buff)
{
    float can_tx_start_time = DWT_GetTimeline_ms();
    while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0) { // 等待邮箱
        float can_tx_now_time = DWT_GetTimeline_ms();
		if(can_tx_now_time > can_tx_start_time + timeout) { /* 等待1ms如果超过则终止发送 */
            HAL_CAN_AbortTxRequest(_instance->can_handle,CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
		    return HAL_ERROR;
		}
	}
    uint32_t txmailbox = 0;
	HAL_StatusTypeDef err = HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, tx_buff, &txmailbox);

	return err;
}

/**
 * @brief 修改CAN发送帧长度
 */
RM_Status CANSetDLC(CANInstance *_instance, uint8_t length)
{
    // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
    if (length > 8 || length == 0){ // 安全检查
         _instance->txconf.DLC = 8;
         return RM_ERROR;
	} else 
	    _instance->txconf.DLC = length;
	return RM_SUCCESS;
}

/* -----------------------belows are callback definitions--------------------------*/
/**
 * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _hcan
 */
static void CNAFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf;
	uint8_t rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) {  // FIFO不为空,有可能在其他中断时有多帧数据进入
	     HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, rx_buff);  // 从FIFO中获取数据
	     for(uint8_t i = 0; i < idx; i ++) {  // 判断ID是否存在对应CAN实例
			if (_hcan == can_instance[i]->can_handle && rxconf.StdId == can_instance[i]->rx_id ) {
				can_instance[i]->rx_len = rxconf.DLC;                      // 保存接收到的数据长度
				memcpy(can_instance[i]->rx_buff, rx_buff, rxconf.DLC);     // 消息拷贝到对应实例
				if(can_instance[i]->can_module_callback != NULL) 
				    can_instance[i]->can_module_callback(can_instance[i]);  //触发回调进行数据解析和处理
  			    return;
			}
		}
	}
}

/**
 * @brief 注意,STM32的两个CAN设备共享两个FIFOic
 * 下面两个函数是HAL库中的回调函数,他们被HAL声明为__weak,这里对他们进行重载(重写)
 * 当FIFO0或FIFO1溢出时会调用这两个函数
 */
 /*下面的函数会调用CANFIFOxCallback()来进一步处理来自特定CAN设备的消息 */
/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CNAFIFOxCallback(hcan, CAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CNAFIFOxCallback(hcan, CAN_RX_FIFO1);
}

