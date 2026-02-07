#pragma once

//====================== Standard + ESP-IDF headers ==============================//
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include <stdbool.h>

// ESP-IDF Core
#include "esp_system.h"
#include "esp_log.h"
#include "sdkconfig.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

// ESP-IDF Drivers
#include "driver/gpio.h"
#include "driver/i2c_slave.h"
#include "esp_timer.h"
#include "nvs_flash.h"


//====================== Project-Wide Constants ==============================//
#define NUM_MOTORS  8    // Used by PID, Motor Control, PWM, BLE

extern portMUX_TYPE encoder_mux;
extern volatile int32_t encorder_val[NUM_MOTORS];

int safe_read_encoder(int index);



