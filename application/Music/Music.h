#ifndef __MUSIC_H
#define __MUSIC_H

#include "struct_typedef.h"

#define MUSIC_TASK_INIT_TIME 10
#define MUSIC_TASK_TIME_MS 1
typedef struct __Note{
	int note;
	float Long;
	uint32_t end;
}Note;

bool MusicStartPlay();
void MusicStartInit();
#endif
