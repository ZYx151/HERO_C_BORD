#include "bsp_iic.h"
#include "RMLibHead.h"

static uint8_t idx = 0;
static IICInstance *iic_instance[IIC_DEVICE_CNT] = {NULL};

IICInstance* IICRegister(IIC_Init_Config_s *config)
{
	if(idx > MX_IIC_SLAVE_CNT)  // 超过最大数目
		return 0;
	IICInstance *instance = (IICInstance *)RMLIB_MALLOC(sizeof(IICInstance));
    memset(instance, 0, sizeof(IICInstance));
	
	instance->dev_address = config->dev_address << 1; // 地址左移一位
	instance->callback = config->callback;
	instance->work_mode = config->work_mode;
	instance->handle = config->handle;
	instance->id = config->id;
	
	iic_instance[idx++] = instance;
	return instance;
}

void IICSetMode(IICInstance *iic, IIC_Work_Mode_e mode)
{// HAL自带重入保护,不需要手动终止或等待传输完成
    if (iic->work_mode != mode)
    {
        iic->work_mode = mode; // 如果不同才需要修改
    }
}

void IICTransmit(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e seq_mode)
{
    if (seq_mode != IIC_SEQ_RELEASE && seq_mode != IIC_SEQ_HOLDON)
        while (1)
            ; // 未知传输模式, 程序停止
    
		// 根据不同的工作模式进行不同的传输
    switch (iic->work_mode)
    {
    case IIC_BLOCK_MODE:
        if (seq_mode != IIC_SEQ_RELEASE)
            while (1)
                ;                                                                // 阻塞模式下不支持HOLD ON模式!!!只能传输完成后立刻释放总线
        HAL_I2C_Master_Transmit(iic->handle, iic->dev_address, data, size, 100); // 默认超时时间100ms
        break;
    case IIC_IT_MODE:
        if (seq_mode == IIC_SEQ_RELEASE)
            HAL_I2C_Master_Seq_Transmit_IT(iic->handle, iic->dev_address, data, size, I2C_OTHER_AND_LAST_FRAME);
        else if (seq_mode == IIC_SEQ_HOLDON)
            HAL_I2C_Master_Seq_Transmit_IT(iic->handle, iic->dev_address, data, size, I2C_OTHER_FRAME);
        break;
    case IIC_DMA_MODE:
        if (seq_mode == IIC_SEQ_RELEASE)
            HAL_I2C_Master_Seq_Transmit_DMA(iic->handle, iic->dev_address, data, size, I2C_OTHER_AND_LAST_FRAME);
        else if (seq_mode == IIC_SEQ_HOLDON)
            HAL_I2C_Master_Seq_Transmit_DMA(iic->handle, iic->dev_address, data, size, I2C_OTHER_FRAME);
        break;
    default:
        while (1)
            ; // 未知传输模式, 程序停止
    }
}

void IICReceive(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e seq_mode)
{
    if (seq_mode != IIC_SEQ_RELEASE && seq_mode != IIC_SEQ_HOLDON)
        while (1)
            ; // 未知传输模式, 程序停止,请检查指针越界
	iic->rx_buff = data;
	iic->rx_len = size;
	
	switch(iic->work_mode) {
		case IIC_BLOCK_MODE :
			if (seq_mode != IIC_SEQ_HOLDON)
				break;
			HAL_I2C_Master_Receive(iic->handle, iic->dev_address, data, size, 100); // 默认超时时间为100ms
			break;
		case IIC_IT_MODE :
			if(seq_mode == IIC_SEQ_RELEASE)
				HAL_I2C_Master_Seq_Receive_IT(iic->handle, iic->dev_address, data, size, I2C_OTHER_AND_LAST_FRAME);
			else if(seq_mode == IIC_SEQ_HOLDON)
            HAL_I2C_Master_Seq_Receive_IT(iic->handle, iic->dev_address, data, size, I2C_OTHER_FRAME);
        break;
    case IIC_DMA_MODE:
        if (seq_mode == IIC_SEQ_RELEASE)
            HAL_I2C_Master_Seq_Receive_DMA(iic->handle, iic->dev_address, data, size, I2C_OTHER_AND_LAST_FRAME);
        else if (seq_mode == IIC_SEQ_HOLDON)
            HAL_I2C_Master_Seq_Receive_DMA(iic->handle, iic->dev_address, data, size, I2C_OTHER_FRAME);
        break;
		default :
			while(1)
				; // 未知传输模式
	}
}

void IICAccessMem(IICInstance *iic, uint16_t mem_addr, uint8_t *data, uint16_t size, IIC_Mem_Mode_e mem_mode, uint8_t mem8bit_flag)
{
    uint16_t bit_flag = mem8bit_flag ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;
    if (mem_mode == IIC_WRITE_MEM)
    {
        HAL_I2C_Mem_Write(iic->handle, iic->dev_address, mem_addr, bit_flag, data, size, 100);
    }
    else if (mem_mode == IIC_READ_MEM)
    {
        HAL_I2C_Mem_Read(iic->handle, iic->dev_address, mem_addr, bit_flag, data, size, 100);
    }
    else
    {
        while (1)
            ; // 未知模式, 程序停止
    }
}

/**
 * @brief IIC接收完成回调函数
 *
 * @param hi2c handle
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // 如果是当前i2c硬件发出的complete,且dev_address和之前发起接收的地址相同,同时回到函数不为空, 则调用回调函数
    for (uint8_t i = 0; i < idx; i++)
    {
        if (iic_instance[i]->handle == hi2c && hi2c->Devaddress == iic_instance[i]->dev_address)
        {
            if (iic_instance[i]->callback != NULL) // 回调函数不为空
                iic_instance[i]->callback(iic_instance[i]);
            return;
        }
    }
}

/**
 * @brief 内存访问回调函数,仅做形式上的封装,仍然使用HAL_I2C_MasterRxCpltCallback
 *
 * @param hi2c handle
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_MasterRxCpltCallback(hi2c);
}
