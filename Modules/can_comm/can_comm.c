#include "can_comm.h"
#include "stdlib.h"
#include "CRC.h"
#include "bsp_dwt.h"

/**
 * @brief 重置CAN comm的接收状态和buffer
 *
 * @param ins 需要重置的实例
 */
static void CANCommResetRx(CANCommInstance *ins)
{
	/* 当前已经收到的buffer清零 */
	memset(ins->raw_recvbuf, 0, ins->cur_recv_len);
	ins->recv_state = 0;    // 接收状态重置
	ins->cur_recv_len = 0;  // 当前已经收到的长度重置
}

/**
 * @brief cancomm的接收回调函数
 *
 * @param _instance
*/
static void CANCommRxCallback(CANInstance *_instance)
{
	CANCommInstance *comm = (CANCommInstance *)_instance->id;

    /* 当前接收状态判断 */
    if (_instance->rx_buff[0] == CAN_COMM_HEADER && comm->recv_state == 0)  { // 之前尚未开始接收且此次包里第一个位置是帧头
        if (_instance->rx_buff[1] == comm->recv_data_len) { // 如果这一包里的datalen也等于我们设定接收长度(这是因为暂时不支持动态包长)
            comm->recv_state = 1;  // 设置接收状态为1,说明已经开始接收
		}
        else
            return; // 直接跳过即可
    }
	
	if (comm->recv_state)
	  { // 已经收到过帧头
		// 如果已经接收到的长度加上当前一包的长度大于总buf_len, 说明接收错误
		if (comm->cur_recv_len + _instance->rx_len > comm->recv_buf_len) {
			CANCommResetRx(comm);
			return;
	  }
		// 直接把当前接收到的数据接到buffer后面
		memcpy(comm->raw_recvbuf + comm->cur_recv_len, _instance->rx_buff, _instance->rx_len);
		comm->cur_recv_len += _instance->rx_len;
		
		// 收完这一包以后刚好等于buff_len, 说明已经收完了
		if (comm->cur_recv_len == comm->recv_buf_len)
		  {
			// 通过校验,复制数据到
			if (comm->raw_recvbuf[comm->recv_buf_len - 1] == CAN_COMM_TAIL)
			{
					memcpy(comm->unpacked_recv_data, comm->raw_recvbuf + 2, comm->recv_data_len);
					comm->recv_state = 0;
					comm->cur_recv_len = 0;
					comm->update_flag = 1;
					Feed_Dog(comm->Dog);
			}
			CANCommResetRx(comm);
			return; // 重置状态然后返回
	     }
	}
}

static void CANCommLostCallback(void *cancomm)
{
    CANCommInstance *comm = (CANCommInstance *)cancomm;
    CANCommResetRx(comm);
}

CANCommInstance *CANCommInit(CANComm_Init_Config_s comm_config)
{
	CANCommInstance *ins = (CANCommInstance *)malloc(sizeof(CANCommInstance) );
	memset(ins, 0, sizeof(CANCommInstance));
	
	/* 给数组分配内存 */
	ins->recv_data_len = comm_config.recv_data_len;
	ins->recv_buf_len  = comm_config.recv_data_len + CAN_COMM_OFFSET_BYTES;
	ins->raw_recvbuf   = (uint8_t * )RMLIB_MALLOC(comm_config.recv_data_len + CAN_COMM_OFFSET_BYTES);
    
    ins->send_data_len = comm_config.send_data_len;
	ins->raw_sendbuf   = (uint8_t * )RMLIB_MALLOC(comm_config.send_data_len + CAN_COMM_OFFSET_BYTES);
	ins->send_buf_len  = comm_config.send_data_len + CAN_COMM_OFFSET_BYTES;
	ins->unpacked_recv_data = (uint8_t * )RMLIB_MALLOC(comm_config.recv_data_len);
	
	ins->raw_sendbuf[0] = CAN_COMM_HEADER; // 发送头部直接设置避免每次发送都要重新赋值
	ins->raw_sendbuf[1] = comm_config.send_data_len; // datalen
	ins->raw_sendbuf[comm_config.send_data_len + CAN_COMM_OFFSET_BYTES - 1] = CAN_COMM_TAIL; // 尾部
	// can instance 的设置
	comm_config.can_config.id = (void *)ins;
	if (comm_config.can_config.can_module_callback == 0) 
		comm_config.can_config.can_module_callback = CANCommRxCallback;
	ins->can_ins = CANRegister(comm_config.can_config);
	
	// watch_dog
	comm_config.dog_config.watch_callback = CANCommLostCallback;
	comm_config.dog_config.owner_id = (void *)ins;
	
	ins->Dog = WatchDog_Init(comm_config.dog_config);
	return ins ;
}

/**
 * @brief 单片机通信发送函数 （已弃用crc8）
*/
HAL_StatusTypeDef CANCommSend(CANCommInstance *instance, uint8_t *data)
{
	static uint8_t crc8, send_len;
	HAL_StatusTypeDef state;
	uint8_t tx_buff[8];
    
    // 将data copy到raw_sendbuf中,计算crc8
    memcpy(instance->raw_sendbuf + 2, data, instance->send_data_len);
//    crc8 = Verify_CRC8_Check_Sum(instance->raw_sendbuf + 2, instance->send_data_len);
//    instance->raw_sendbuf[2 + instance->send_data_len] = crc8;

    // CAN单次发送最大为8字节,如果超过8字节,需要分包发送
    for (size_t i = 0; i < instance->send_buf_len; i += 8)
    { // 如果是最后一包,send len将会小于8,要修改CAN的txconf中的DLC位,调用bsp_can提供的接口即可
        send_len = instance->send_buf_len - i >= 8 ? 8 : instance->send_buf_len - i;
        CANSetDLC(instance->can_ins, send_len);
        memcpy(tx_buff, instance->raw_sendbuf + i, send_len);
        state = CANTransmit(instance->can_ins, 0.5f, tx_buff);
		
    }
	return state;
}

void *CANCommGet(CANCommInstance *instance)
{
	instance->update_flag = 0;  // 读取后将更新flag为0
	
	return instance->unpacked_recv_data;
}


