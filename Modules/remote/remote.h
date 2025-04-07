/**
 * @file    remote.h
 * @author  yao
 * @date    1-May-2020
 * @brief   遥控器模块头文件
 */

#ifndef __remote_H
#define __remote_H

#include "RMLibHead.h"
#include "stdlib.h"
#include "WatchDog.h"
#include "bsp_usart.h"

RMLIB_CPP_BEGIN

extern USARTInstance *remote_uart;

#define RC_FRAME_LENGTH 18u
#define REMOTE_CONTROLLER_STICK_OFFSET 1024

// 获取按键操作
#define KEY_PRESS 0
#define KEY_STATE 1
#define KEY_PRESS_WITH_CTRL 1
#define KEY_PRESS_WITH_SHIFT 2

/**@brief 去除遥控器摇杆死区 */
#define deadline_limit(value, dealine)                     \
	{                                                      \
		if ((value) <= (dealine) && (value) >= -(dealine)) \
			value = 0;                                     \
	}

/**
 * @brief 摇杆数据结构体
 */
typedef struct {
    int16_t ch0;  //!<@brief 摇杆通道0数据
    int16_t ch1;  //!<@brief 摇杆通道1数据
    int16_t ch2;  //!<@brief 摇杆通道2数据
    int16_t ch3;  //!<@brief 摇杆通道3数据
    int8_t s1;    //!<@brief 开关1状态
    int8_t s2;    //!<@brief 开关2状态
} Remote;

/**
 * @brief 鼠标数据结构体
 */
typedef struct {
    int16_t x;              //!<@brief 鼠标x轴速度
    int16_t y;              //!<@brief 鼠标y轴速度
    int16_t z;              //!<@brief 鼠标z轴速度
    uint8_t last_press_l;   //!<@brief 上一次左键状态
    uint8_t last_press_r;   //!<@brief 上一次右键状态
    uint8_t press_l;        //!<@brief 左键状态
    uint8_t press_r;        //!<@brief 右键状态
} Mouse;

/**
 * @brief 键盘数据结构体
 * @details 每一个成员对应其按键
 */
typedef union
{
    struct // 用于访问键盘状态
    {
		uint16_t W: 1;
		uint16_t S: 1;
		uint16_t A: 1;
		uint16_t D: 1;
		uint16_t Shift: 1;
		uint16_t Ctrl: 1;
		uint16_t Q: 1;
		uint16_t E: 1;
		uint16_t R: 1;
		uint16_t F: 1;
		uint16_t G: 1;
		uint16_t Z: 1;
		uint16_t X: 1;
		uint16_t C: 1;
		uint16_t V: 1;
		uint16_t B: 1;
    };
    uint16_t keys; // 用于memcpy而不需要进行强制类型转换
} Key_t;

#define Key_W 0
#define Key_S 1
#define Key_D 2
#define Key_A 3
#define Key_Shift 4
#define Key_Ctrl 5
#define Key_Q 6
#define Key_E 7
#define Key_R 8
#define Key_F 9
#define Key_G 10
#define Key_Z 11
#define Key_X 12
#define Key_C 13
#define Key_V 14
#define Key_B 15

/**
 * @brief 输入模式枚举
 */
typedef enum {
    REMOTE_INPUT = 1,        //!<@brief 遥控器输入
    KEY_MOUSE_INPUT = 3,     //!<@brief 键盘输入
    STOP = 2,                //!<@brief 急停模式
} InputMode_e;

/**
 * @brief 遥控器整体数据结构体
 */
#pragma pack(1)

typedef struct {
    float Key_CH[4];    //!<@brief 遥控器CH遥感数据
	float Mouse_Ch[3];  // !<@brief 鼠标数据
	InputMode_e RemoteMode;
    Key_t key[3];        //!<@brief 键盘数据
    Key_t Lastkey[3];    //!<@brief 上一帧键盘数据
    uint8_t key_count[3][16];
	Remote rc;        //!<@brief 遥控器数据
    Mouse mouse;      //!<@brief 鼠标数据
    WatchDog_TypeDef *Remote_dog;
} RC_Ctl_t;

#pragma pack()

/**
 * @brief 遥控器数据接收函数
 * @param[in] RxMsg 遥控器串口原始数据
 */
void Remote_Rx(uint8_t *RxMsg);

/**
 * @brief 遥控器数据归零
 */
void RemoteClear(void);

/**
 * @brief 停止模式控制回调函数
 */
void STOPControlProcess(void);

/**
* @brief  获取遥控器指针
*/
RC_Ctl_t *get_remote_control_point(void);

/**
* @brief  遥控器初始化函数 因为只有一个遥控器 不需要返回指针
*/
void Remote_Init(void);
/**
 * @brief 遥控器接收任务
 */
void Task_Remote(void *pvParameters);


RMLIB_CPP_END

#endif
