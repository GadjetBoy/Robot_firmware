#pragma once

#include "driver/uart.h"
#include "driver/gpio.h"
#include "PID.h"

#define UART_PORT_NUM      UART_NUM_0
#define UART_BAUD_RATE     115200
#define UART_TX_PIN        GPIO_NUM_1   // default TX for UART0
#define UART_RX_PIN        GPIO_NUM_3   // default RX for UART0
#define UART_BUF_SIZE      1024

extern Motor update_motors[NUM_MOTORS];
extern portMUX_TYPE UART_mux;
extern SemaphoreHandle_t uart_mutex;

void UART_app_main(void);
void uart_send_data(const char *data);