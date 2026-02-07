#pragma once

#include "control_1.2.h" 

#define NUM_GATES 4

#define gate_pos_A 28000//5088
#define gate_pos_B 28000//5088

#define PHASE_A_DONE_BIT   (1 << 0)
#define PHASE_B_DONE_BIT   (1 << 1)

extern EventGroupHandle_t crawl_event_group;

extern uint8_t phaseA_remaining ;
extern uint8_t phaseB_remaining ;

typedef enum {
    MODE_IDLE = 0,
    MODE_CRAWL,
    MODE_STANDBY,
    MODE_TURTLE
} SequenceMode;

typedef struct {
    bool phaseA;
    bool phaseB;
    bool phaseC;
    bool phaseD;
} WaitFlags;

void mot_command_app_main(void);
