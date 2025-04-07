/**
 * @file    WatchDog.c
 * @author  yao
 * @date    1-May-2020
 * @brief   看门狗模块
 * @details 要启用看门狗需要添加全局宏定义WatchDoglength并赋值
 *  需要的最大看门狗数量
 */

#include <WatchDog.h>

#if defined(WatchDoglength) && WatchDoglength > 0

/*!@brief 看门狗列表*/
static WatchDog_TypeDef *List[WatchDoglength];

/*!@brief 看门狗长度*/
static uint16_t Len = 0;

void WatchDog_Polling(void) {
    for (uint8_t i = 0; i < Len; ++i) {
        List[i]->Life++;
        if (List[i]->Life > List[i]->Max) {
			 List[i]->state = Dog_Offline;
			
			if (List[i]->GPIOx != 0) 
			      HAL_GPIO_WritePin(List[i]->GPIOx, List[i]->GPIO_Pin, GPIO_PIN_RESET);
			if ( List[i]->owner_id && List[i]->watch_callback)
			      List[i]->watch_callback(List[i]->owner_id);
//            WatchDog_CallBack(List[i])
        }
    }
}

WatchDog_TypeDef *WatchDog_Init(WatchDog_Init_config config) {
    if (Len >= WatchDoglength)
        return 0;
    WatchDog_TypeDef *instance = (WatchDog_TypeDef *)RMLIB_MALLOC(sizeof(WatchDog_TypeDef) );
    memset(instance, 0, sizeof(WatchDog_TypeDef) );
	instance->state = Dog_Offline;
    instance->Max = config.Max_num == 0 ? 20 : config.Max_num;
	instance->watch_callback = config.watch_callback;
	instance->feed_callback = config.feed_callback;
	instance->GPIOx = config.GPIOx;
	instance->GPIO_Pin = config.GPIO_Pin;
	
	strcpy(instance->name, config.dog_name);
	instance->owner_id = config.owner_id;
	instance->ID = Len + 1;
    List[Len++] = instance;
	return instance;
}

void Feed_Dog(WatchDog_TypeDef* handle) {
    handle->Life = 0;
    
	handle->state = Dog_Online;
	handle->owner_id && handle->feed_callback ? handle->feed_callback(handle->owner_id) ?
   	      ( handle->state = Dog_Online ) : ( handle->state = Dog_Error ) : 0 ;
	if(handle->GPIOx != 0)
        HAL_GPIO_WritePin(handle->GPIOx, handle->GPIO_Pin, GPIO_PIN_SET);
}

//void FeedDog_CallBack(WatchDogp handle) {
//    UNUSED(handle);
//}

//__weak void WatchDog_CallBack(WatchDogp handle) {
//    UNUSED(handle);
//}

#endif
