/**
 * @file    WatchDog.h
 * @author  yao
 * @date    1-May-2020
 * @brief   软件看门狗模块头文件
 */

#ifndef _WATCH_DOG_H_
#define _WATCH_DOG_H_

#define WatchDoglength 20

#if defined(WatchDoglength) && WatchDoglength > 0

#include "RMLibHead.h"

RMLIB_CPP_BEGIN

typedef void (*WatchDog_CallBack )(void *);
typedef uint8_t (*FeedDog_CallBack )(void *);

typedef enum {
	Dog_Offline = 0,        //!< @brief 设备离线
	Dog_Online  = 1,	    //!< @brief 设备在线
	Dog_Error   = 2	        //!< @brief 设备错误        
}Dog_State_e;	

/**
 * @brief 看门狗结构体
 */
typedef struct {
	char name[10]; //看门狗名字
    uint8_t ID;     //!<@brief 看门狗ID
	
	/**  看门狗点灯  **/
	GPIO_TypeDef *GPIOx;         // GPIOA, GPIOC, GPIOB
	GPIO_PinState pin_state;    // 引脚状态
	uint16_t GPIO_Pin;          // GPIO引脚号 GPIO_PIN_0 GPIO_PIN_1 
	
	Dog_State_e state;
    WatchDog_CallBack   watch_callback; // callback needs an instance to tell among registered ones
    FeedDog_CallBack   feed_callback; // callback needs an instance to tell among registered ones
    
    void *owner_id;
    uint32_t Life;  //!<@brief 当前离线计数
    uint32_t Max;   //!<@brief 最大离线计数
} WatchDog_TypeDef;

typedef struct {
    WatchDog_CallBack  watch_callback;
    FeedDog_CallBack   feed_callback;
	/**  看门狗点灯  **/
	GPIO_TypeDef *GPIOx;         // GPIOA, GPIOC, GPIOB
	uint16_t GPIO_Pin;          // GPIO引脚号 GPIO_PIN_0 GPIO_PIN_1 
    
	void *owner_id;
	char dog_name[10];
    uint32_t Max_num;   //!<@brief 最大离线计数
} WatchDog_Init_config;

/**
 * @brief 看门狗轮询函数，需要以一定周期调用
 * @note 在无系统时可以使用重写HAL_IncTick函数的方式轮询  
 *       例：  
 *       {@code 
 *       void HAL_IncTick(void) {
 *           uwTick += uwTickFreq;
 *           WatchDog_Polling();
 *       }}
 *       在实时系统下则需要创建一个任务周期性调用该函数
 */
void WatchDog_Polling(void);

/**
 * @brief 判断是否为同一个看门狗
 * @param handle 看门狗结构体指针
 * @param Dog 要判断的看门狗
 */
#define IS_Dog(handle, Dog) ((handle) == &(Dog))

/**
 * @brief 初始化看门狗
 * @param[out] handle 看门狗结构体指针
 * @param[in] Life
 */
WatchDog_TypeDef *WatchDog_Init(WatchDog_Init_config handle);

/**
 * @brief 喂狗函数
 * @param[in] handle 看门狗结构体指针
 */
void Feed_Dog(WatchDog_TypeDef* handle);

RMLIB_CPP_END

#endif

#endif
