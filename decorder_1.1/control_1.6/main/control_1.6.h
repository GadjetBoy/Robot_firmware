#pragma once

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
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#define NUM_MOTORS 8
#define NUM_ENCODERS NUM_MOTORS

extern TaskHandle_t i2c_send_task;

