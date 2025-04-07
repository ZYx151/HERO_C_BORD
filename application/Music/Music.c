#include "Music.h"

#include "stm32f4xx_hal.h"
#include "bsp_buzzer.h"


//定义低音  
#define A1  131
#define A2  147
#define A3  165
#define A4  175
#define A5  196
#define A6  220
#define A7  247
  
//定义中音  
#define B1  262
#define B2  296
#define B3  330
#define B4  349
#define B5  392
#define B6  440
#define B7  494
  
//定义高音  
#define C1  523
#define C2  587
#define C3  659
#define C4  698
#define C4p 741
#define C5  784
#define C6  880
#define C7  988
  
//定义高二度  
#define D1  1047
#define D2  1175
#define D3  1319
#define D4  1397
#define D5  1568
#define D6  1760
#define D7  1976

//定义节拍  
#define OneBeat   200//一拍子两个1beat 
#define HalfBeat  100

#define NOTE_NUM 10
static Note Notes[NOTE_NUM];  // Array of notes

static uint32_t last_note_id = 0;  // Index of the last note
static uint32_t write_id = 1;      // Index of the note to be written
uint32_t play_id = 1;       // Index of the note to be played

static uint32_t start_time = 0;  // Start time of the music
static uint32_t now = 0;

         /* Music_Start */
static void WriteNote(int note, float Long)
{
    Notes[write_id].note = note;
    Notes[write_id].Long = Long;
    Notes[write_id].end = Notes[write_id - 1].end + Long;
    write_id++;
}

/**
 * @brief 播放音乐
 * @param  none
 * @return 结束1 未结束0
 */
bool MusicStartPlay(void ){
	now = HAL_GetTick();
	bool end = false;
	if(now - start_time >= Notes[play_id].end){
	play_id++;
		if(play_id > last_note_id){
			end = true;
			play_id = 1;
			start_time = now; 
		}
		buzzer_note(Notes[play_id].note,0.1);
	}
	return end;
}

void MusicStartInit(void){
	WriteNote(0,2);
	WriteNote(B1,HalfBeat*2);
	WriteNote(0,3);
	WriteNote(C3,HalfBeat*2);
	WriteNote(0,3);
	WriteNote(D5,HalfBeat*2);
	WriteNote(0,5);
	WriteNote(D1,HalfBeat*5);
	
	last_note_id = write_id -1;
	write_id = 1;
}
