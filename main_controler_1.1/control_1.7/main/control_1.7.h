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
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/uart.h"


//====================== Project-Wide Constants ==============================//
#define NUM_MOTORS  8    // Used by PID, Motor Control, PWM, BLE

typedef struct __attribute__((packed)) {
    float target_pwm_val[NUM_MOTORS];
} pwm_packet_t;

extern volatile float motor_cmd_buffer[2][NUM_MOTORS];
extern volatile uint8_t current_write_index;  // PID reads from this
extern portMUX_TYPE pid_buffer_swap_mux;

extern QueueHandle_t log_queue;
extern TaskHandle_t blinkTaskHandle;
