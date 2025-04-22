#include "Referee.h"
#include "usart.h"
#include "robot_def.h"
#include "DMA_double_buffer.h"
#include "pub_sub.h"

#include "Referee_unpack.h"
#include "bsp_usart.h"
#include "WatchDog.h"

/** 裁判系统  **/
static uint8_t Usart6_Referee_buffer[2][REFEREE_BUFFER_LEN];// 裁判系统数据接收缓冲区
Referee_data_t Referee_SendData;
static Publisher_t *Referee_Feedback_Pub;           // 裁判系统反馈消息发布者
static RMQueue_Handle Referee_queue;                // 裁判系统解算数据包
       WatchDog_TypeDef *Referee_Dog;               // 裁判系统看门狗
static USARTInstance *Referee_Uart6;  // USARTSerxiceInit(Referee_Uart6);

void Task_Referee(void *pvParameters)
{
    static portTickType currentTime;
    for (;;)
    {
        currentTime = xTaskGetTickCount();
        unpack_referee_system_data(&Referee_queue);//裁判系统解包
        vTaskDelayUntil(&currentTime, 3);
    }
}

// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void Referee_LostCallback(void *id)
{
	// 裁判系统丢失将重新启动串口
    HAL_NVIC_DisableIRQ(USART6_IRQn);
    HAL_UART_Receive_DMA_double(&huart6, Usart6_Referee_buffer[0] , Usart6_Referee_buffer[1], REFEREE_BUFFER_LEN);
}

void Referee_Init()
{
    /* 串口6初始化(裁判系统通信)*/
	USART_Init_Config_s referee_config = {
			.data_len = sizeof(uint8_t),
			.usart_handle = &huart6,
		    .module_callback = 0,
	};
	Referee_Uart6 = (USARTInstance *)RMLIB_MALLOC(sizeof(USARTInstance));
    
    RMQueueInit(&Referee_queue, REFEREE_BUFFER_LEN, RM_QUEUE_LEN);
    HAL_NVIC_DisableIRQ(USART6_IRQn);	// DMA发送需在对应DMA中断中清除状态
    HAL_UART_Receive_DMA_double(&huart6, Usart6_Referee_buffer[0] , Usart6_Referee_buffer[1], REFEREE_BUFFER_LEN);
    
    Referee_Feedback_Pub = PubRegister("referee", sizeof(Referee_data_t) );
	WatchDog_Init_config Referee_Config = {
		.watch_callback = Referee_LostCallback,
//		.feed_callback = 0,
		.dog_name = "RefereeDog",
		.owner_id = Referee_Uart6,
		.Max_num = 20,
	};
	Referee_Dog = WatchDog_Init(Referee_Config);
}

uint8_t shoot_flag = 0;
void Referee_Update()
{
    // unpack_referee_system_data(&Referee_queue);//裁判系统解包
	if(!Damage_status.HP_deduction_reason) {
		Damage_status.HP_deduction_reason = 1;
	}
	/* 等级（底盘增益，小陀螺增益），枪口热量(弹频限制)，弹丸速度（自调节或手动调），颜色（视觉识别），冷却增益（单频） */
	if (Referee_Dog->state == Dog_Online){
		shoot_flag = Real_time_shooting.bullet_type;
		
		Referee_SendData.refree_status = Device_Online;
		if(Robot_state.robot_id < 100)
			Referee_SendData.robot_color = 1;//红1
		else
			Referee_SendData.robot_color = 0;//蓝0
		Referee_SendData.robot_level                 = Robot_state.robot_level;
		Referee_SendData.shooter_barrel_heat_limit   = Robot_state.shooter_barrel_heat_limit; // 热量上限
		Referee_SendData.heat_now_remain             = Real_time_chassis_power_and_shoot_heat.shooter_42mm_barrel_heat; // 实时热量
		Referee_SendData.buffer_energy               = Real_time_chassis_power_and_shoot_heat.buffer_energy;   // 缓冲能量
		Referee_SendData.bullet_speed_now            = Real_time_shooting.initial_speed;   // 实时弹速
		Referee_SendData.chassis_power               = Real_time_chassis_power_and_shoot_heat.chassis_power; // 底盘功率
		Referee_SendData.chassis_power_limit         = Robot_state.chassis_power_limit;    // 功率上限
        Referee_SendData.robot_HP                    = Robot_state.current_HP;             // 机器人当前血量
        Referee_SendData.chassis_output              = Robot_state.power_management.chassis_output; //chassis 口输出：0 为无输出，1 为 24V 输出
        if (Referee_SendData.Ammo_remain  < Ammo_amount.projectile_allowance_42mm)
		{	
			Referee_SendData.Ammo_add                = Ammo_amount.projectile_allowance_42mm - Referee_SendData.Ammo_remain;
			Referee_SendData.Ammo_remain             = Ammo_amount.projectile_allowance_42mm;
		}
        Referee_SendData.Max_HP                      = Robot_state.maximum_HP;
        Referee_SendData.Ammo_consume                = Referee_SendData.Ammo_add + Referee_SendData.Ammo_remain - Ammo_amount.projectile_allowance_17mm;
		if (Match_status.game_progress != 4)
		{
			Referee_SendData.Ammo_remain             = 0;
			Referee_SendData.Ammo_add                = 0;
			Referee_SendData.Ammo_consume            = 0;
		}
	} else Referee_SendData.refree_status = Device_Offline;
	 //  消息发布
    PubPushMessage(Referee_Feedback_Pub, (void *)&Referee_SendData);
}

#ifdef CHASSIS_BOARD
/* DMA接收完成回调函数 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6) {
       void *endPtr = RMQueueGetEndPtr(&Referee_queue);
        if (endPtr) {
            (huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET ?  
                memcpy(endPtr, Usart6_Referee_buffer[1], REFEREE_BUFFER_LEN)
			      : memcpy(endPtr, Usart6_Referee_buffer[0], REFEREE_BUFFER_LEN);
           RMQueuePushEndPtr(&Referee_queue);
        } else  {
           unpack_referee_system_data(&Referee_queue);//队列满了没处理
           RMQueuePushEndPtr(&Referee_queue);//队列被锁定
        }
        Feed_Dog(Referee_Dog);
	}
}
#endif

