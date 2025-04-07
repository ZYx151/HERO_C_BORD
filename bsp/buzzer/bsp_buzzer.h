#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "struct_typedef.h"

void buzzer_on(uint16_t psc, uint16_t pwm);
void buzzer_off(void);
void buzzer_note(uint16_t note,float volume);
#endif
