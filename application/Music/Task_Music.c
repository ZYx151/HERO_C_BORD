#include "Task_Music.h"

#include "bsp_buzzer.h"
#include "cmsis_os.h"
#include "music.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t music_high_water;
#endif

#define STEP_INIT 1
#define STEP_NORMAL 2

#define is_play_cali()                                                                 \
    (is_play == CALI_BEGIN || is_play == CALI_MIDDLE_TIME || is_play == CALI_GIMBAL || \
     is_play == CALI_IMU || is_play == CALI_CHASSIS)
	 
#define cali_buzzer_begin() buzzer_on(50, 10000)   // ???????????
#define cali_buzzer_middle() buzzer_on(20, 10000)  // ???????????
#define cali_buzzer_gimbal() buzzer_on(30, 19999)  // ??????,???????????
#define cali_buzzer_imu() buzzer_on(60, 19999)  // ?imu???,???????????
#define cali_buzzer_chassis() buzzer_on(100, 19999)  // ??????,???????????
#define cali_buzzer_off() buzzer_off()               // buzzer off,?????

extern uint32_t play_id;       // Index of the note to be played
void Task_Music(void *pvParameters)
{
	static int16_t tim = 0;
    // 
    vTaskDelay(MUSIC_TASK_INIT_TIME);
    // 
    MusicStartInit();
    static portTickType currentTime;
    for(;;)
	{
		currentTime = xTaskGetTickCount();		
		tim++;			 
		MusicStartPlay();
		
		if(tim >= 900)
		{
		  buzzer_off();
		  osThreadSuspend(Task_Music_Handle);			 
		}
		
	    vTaskDelayUntil(&currentTime, 1);
	}
}

