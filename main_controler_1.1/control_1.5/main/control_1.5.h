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

typedef struct __attribute__((packed)) {
    int32_t encoders[NUM_MOTORS];
} encoder_packet_t;

typedef struct {
    i2c_slave_dev_handle_t slave_handle;
    QueueHandle_t recieve_queue;
    bool error;
} i2c_slave_context_t;

extern i2c_slave_context_t context;

int safe_read_encoder(int index);




