/*********************************I2C****************************/
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

/*********************************PCNT*************************/
#include <stdio.h>
#include "driver/pulse_cnt.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#define NUM_ENCODERS 8

extern int32_t PCNT_Count[NUM_ENCODERS];
void PCNT_app_main(void);

extern portMUX_TYPE buf_mux;
extern volatile int32_t pcnt_buf_A[NUM_ENCODERS];
extern volatile int32_t pcnt_buf_B[NUM_ENCODERS];

extern volatile int32_t *front_buf;   // readers use this
extern volatile int32_t *back_buf ;