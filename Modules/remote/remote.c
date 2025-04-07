#include "remote.h"

#include "PID.h"
//uint8_t Remote_flag = 0;
//uint8_t Usart1_Remote_Dma[2][RC_FRAME_LENGTH + 1];

RC_Ctl_t RC_CtrlData;
USARTInstance *remote_uart;
TaskHandle_t Task_Remote_Handle;

/**
* @brief  遥控器数据接收回调函数
*/
__weak void STOPControlProcess(void) 
{
	memset(&RC_CtrlData.key, 0, sizeof(RC_CtrlData.key) );
	memset(&RC_CtrlData.key_count, 0, sizeof(RC_CtrlData.key_count) );
    return;
}

/**
 * @brief 遥控器模式控制回调函数
 */
void RemoteControlProcess(void)
{
	RC_CtrlData.RemoteMode = REMOTE_INPUT;
	
	RC_CtrlData.Key_CH[0]  = (float )(RC_CtrlData.rc.ch0 - REMOTE_CONTROLLER_STICK_OFFSET)/ 660;
	RC_CtrlData.Key_CH[1]  = (float )(RC_CtrlData.rc.ch1 - REMOTE_CONTROLLER_STICK_OFFSET)/ 660;
	RC_CtrlData.Key_CH[2]  = (float )(RC_CtrlData.rc.ch2 - REMOTE_CONTROLLER_STICK_OFFSET)/ 660;
	RC_CtrlData.Key_CH[3]  = (float )(RC_CtrlData.rc.ch3 - REMOTE_CONTROLLER_STICK_OFFSET)/ 660;
	
	deadline_limit(RC_CtrlData.Key_CH[0] , 0.1f);
	deadline_limit(RC_CtrlData.Key_CH[1] , 0.1f);
	deadline_limit(RC_CtrlData.Key_CH[2] , 0.1f);
	deadline_limit(RC_CtrlData.Key_CH[3] , 0.1f);
}

/**
 * @brief 键鼠模式控制回调函数
 */
void MouseKeyControlProcess(void) 
{
	RC_CtrlData.RemoteMode=KEY_MOUSE_INPUT;
	
	limit (RC_CtrlData.mouse. x,660,-660);
	limit (RC_CtrlData.mouse. y,660,-660);
	limit (RC_CtrlData.mouse. z,660,-660);
	
	RC_CtrlData.Mouse_Ch[0]=(float)(RC_CtrlData.mouse. x)/660;
	RC_CtrlData.Mouse_Ch[1]=(float)(RC_CtrlData.mouse. y)/660;
	RC_CtrlData.Mouse_Ch[2]=(float)(RC_CtrlData.mouse. z)/660;

	deadline_limit(RC_CtrlData.Mouse_Ch[0],0.01f);
	deadline_limit(RC_CtrlData.Mouse_Ch[1],0.01f);
	deadline_limit(RC_CtrlData.Mouse_Ch[2],0.01f);
}

void Remote_Rx(uint8_t *RxMsg) {
    RC_CtrlData.rc.ch0 = (RxMsg[0] | (RxMsg[1] << 8)) & 0x07FF ;
    RC_CtrlData.rc.ch1 = ((RxMsg[1] >> 3) | (RxMsg[2] << 5)) & 0x07FF ;
    RC_CtrlData.rc.ch2 = ((RxMsg[2] >> 6) | (RxMsg[3] << 2) | (RxMsg[4] << 10)) & 0x07FF ;
    RC_CtrlData.rc.ch3 = ((RxMsg[4] >> 1) | (RxMsg[5] << 7)) & 0x07FF ;

    RC_CtrlData.rc.s1 = (RxMsg[5] >> 4 & 0x000C) >> 2;
    RC_CtrlData.rc.s2 = (RxMsg[5] >> 4 & 0x0003);

    RC_CtrlData.mouse.x = (int16_t)(RxMsg[6] | (RxMsg[7] << 8));
    RC_CtrlData.mouse.y = (int16_t)(RxMsg[8] | (RxMsg[9] << 8));
    RC_CtrlData.mouse.z = (int16_t)(RxMsg[10] | (RxMsg[11] << 8));

    RC_CtrlData.mouse.press_l = RxMsg[12];
    RC_CtrlData.mouse.press_r = RxMsg[13];
    
    *(uint16_t * ) & (RC_CtrlData.key[KEY_PRESS] ) = (uint16_t)(RxMsg[14] | RxMsg[15] << 8 );

    if (RC_CtrlData.key[KEY_PRESS].Ctrl) // ctrl键按下
        RC_CtrlData.key[KEY_PRESS_WITH_CTRL] = RC_CtrlData.key[KEY_PRESS];
    else
        memset(&RC_CtrlData.key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (RC_CtrlData.key[KEY_PRESS].Shift) // shift键按下
        RC_CtrlData.key[KEY_PRESS_WITH_SHIFT] = RC_CtrlData.key[KEY_PRESS];
    else
        memset(&RC_CtrlData.key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));
	
    uint16_t key_now = RC_CtrlData.key[KEY_PRESS].keys,                     // 当前按键是否按下
        key_last = RC_CtrlData.Lastkey[KEY_PRESS].keys,                     // 上一次按键是否按下
        key_with_ctrl = RC_CtrlData.key[KEY_PRESS_WITH_CTRL].keys,          // 当前ctrl组合键是否按下
        key_with_shift = RC_CtrlData.key[KEY_PRESS_WITH_SHIFT].keys,        //  当前shift组合键是否按下
        key_last_with_ctrl = RC_CtrlData.Lastkey[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = RC_CtrlData.Lastkey[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下
    
    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            RC_CtrlData.key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            RC_CtrlData.key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            RC_CtrlData.key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }
     
     memcpy(&RC_CtrlData.Lastkey, &RC_CtrlData.key, sizeof(RC_CtrlData.key));
	
    switch (RC_CtrlData.rc.s2) {
        case REMOTE_INPUT:
            //遥控器控制模式
            RemoteControlProcess();
            break;

        case KEY_MOUSE_INPUT:
            //键鼠控制模式
		    MouseKeyControlProcess();
            break;
        case STOP:
			RC_CtrlData.RemoteMode = STOP;
            STOPControlProcess();
            break;
    }
    RC_CtrlData.mouse.last_press_l = RC_CtrlData.mouse.press_l;
    RC_CtrlData.mouse.last_press_r = RC_CtrlData.mouse.press_r;
	Feed_Dog(RC_CtrlData.Remote_dog);
}

/**
* @brief  获取遥控器指针
*/
RC_Ctl_t *get_remote_control_point(void)
{
    return &RC_CtrlData;
}

/**
* @brief  看门狗丢失函数
*/
void RCLostCallback(void * id)
{
	memset(&RC_CtrlData.key, 0, sizeof(RC_CtrlData.key) );
	memset(&RC_CtrlData.key_count, 0, sizeof(RC_CtrlData.key_count) );
}

/**@brief 遥控器数据乱码检测 */
uint8_t REMOTE_IfDataError( void *id)
{
	if ( (RC_CtrlData.rc.s1 != 1 && RC_CtrlData.rc.s1 != 3 && RC_CtrlData.rc.s1 != 2)
	|| (RC_CtrlData.rc.s2 != 1 && RC_CtrlData.rc.s2 != 3 && RC_CtrlData.rc.s2 != 2)
	|| (RC_CtrlData.rc.ch0 > 1684 || RC_CtrlData.rc.ch0 < 364)
	|| (RC_CtrlData.rc.ch1 > 1684 || RC_CtrlData.rc.ch1 < 364)
	|| (RC_CtrlData.rc.ch2 > 1684 || RC_CtrlData.rc.ch2 < 364)
	|| (RC_CtrlData.rc.ch3 > 1684 || RC_CtrlData.rc.ch3 < 364) )
		return 0;
	else
		return 1;
}

/**@brief 遥控器初始化 */
void Remote_Init() {
	WatchDog_Init_config RC_Dog = {
		.watch_callback = RCLostCallback,
		.feed_callback = REMOTE_IfDataError,
		.dog_name = "RemoteDog",
		.owner_id = &RC_CtrlData,
		.Max_num = 15,
	};
   RC_CtrlData.RemoteMode = STOP;
   RC_CtrlData.Remote_dog = WatchDog_Init(RC_Dog);
    
	USART_Init_Config_s remote_config = {
			.data_len = (RC_FRAME_LENGTH),
			.usart_handle = &huart3,
		    .module_callback = Remote_Rx,
	};
	remote_uart = USARTRegister(&remote_config);
//    xTaskCreate((TaskFunction_t)Task_Remote,            "Task_Remote",            128*2, NULL, 7, &Task_Remote_Handle);
}

//void Task_Remote(void *pvParameters)
//{
//	for(;;)
//	{
//        if(osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever))
//            remote_uart->temp_depth ? Remote_Rx(remote_uart->recv_buff[0]) : Remote_Rx(remote_uart->recv_buff[1]);
//	}
//}

