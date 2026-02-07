#pragma once
/*********************************PCNT*************************/
#include "control_1.7.h"
#include "driver/pulse_cnt.h"

#define NUM_ENCODERS NUM_MOTORS

extern QueueHandle_t encorderQue;
extern int32_t PCNT_Count[NUM_ENCODERS];
extern pcnt_unit_handle_t pcnt_unit[NUM_ENCODERS];
void PCNT_app_main(void);

