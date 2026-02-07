#pragma once
#include "control_1.8.h"
#include "driver/uart.h"
#include "cpg.h"

//============================================UART0=========================================================

#define UART0_PORT_NUM      UART_NUM_0
#define UART0_BAUD_RATE     115200
#define UART0_TX_PIN        GPIO_NUM_1   // default TX for UART0
#define UART0_RX_PIN        GPIO_NUM_3   // default RX for UART0
#define UART0_BUF_SIZE      4096

extern portMUX_TYPE UART_mux;
extern SemaphoreHandle_t uart_mutex;

void UART_app_main(void);
void uart0_send_data(const char *data);

//==================================================UART1====================================================

#define UART_PORT_NUM       UART_NUM_1
#define TXD_PIN             13
#define RXD_PIN             15
#define UART_BAUD_RATE      2000000
#define UART_BUF_SIZE       2048

#define UART_UPDATE_RATE_HZ 5000
#define UART_DT (1.0f / UART_UPDATE_RATE_HZ)
#define UART_UPDATE_RATE_US (UART_DT * 1000000.0f)

#define PACKET_HEADER_1 0xAA
#define PACKET_HEADER_2 0x55
#define PACKET_SIZE (2 + NUM_MOTORS * sizeof(float)) // 34 bytes


extern TaskHandle_t uart_send_task;

void uart_init(void);
void uart_send_task_func(void *param);
void uart_event_task(void *pvParameters);
void uart_timer_callback(void *arg);