#ifndef TASK_SERVICE_H__
#define TASK_SERVICE_H__
#include "nrf_delay.h"
#include "boards.h"

#define UART_TX_BUF_SIZE 256       //串口发送缓存大小（字节数）
#define UART_RX_BUF_SIZE 256       //串口接收缓存大小（字节数）

void uart_config(void);
void mpu_run(void);

#endif

