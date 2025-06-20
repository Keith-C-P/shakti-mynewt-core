/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */
// #ifndef HAL_UART_H_ <-- REEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// #define HAL_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include "hal/hal_uart.h"
#include "bsp/bsp.h"
#include "mcu/vajra_hal.h"

#define STS_RX_THRESHOLD 0x1 << 8
#define BREAK_ERROR 1 << 7
#define FRAME_ERROR 1 << 6
#define OVERRUN 1 << 5
#define PARITY_ERROR 1 << 4
#define STS_RX_FULL 1 << 3
#define STS_RX_NOT_EMPTY 1 << 2
#define STS_TX_FULL 1 << 1
#define STS_TX_EMPTY 1 << 0

/*! UART Interrupt Enable bits description */
#define ENABLE_RX_THRESHOLD 1 << 8
#define ENABLE_BREAK_ERROR 1 << 7
#define ENABLE_FRAME_ERROR 1 << 6
#define ENABLE_OVERRUN 1 << 5
#define ENABLE_PARITY_ERROR 1 << 4
#define ENABLE_RX_FULL 1 << 3
#define ENABLE_RX_NOT_EMPTY 1 << 2
#define ENABLE_TX_FULL 1 << 1
#define ENABLE_TX_EMPTY 1 << 0
#define UARTX_BUFFER_SIZE 10000

/* UART control register */
#define STOP_BITS(x)                                                           \
  ((x & 3) << 1) /*! 00 - 1 stop bits, 01 - 1.5 stop bits; 10 - 2 stop bits;   \
                    11 unused */
#define PARITY(x)                                                              \
  ((x & 3) << 3) /*! 00 --- No parity; 01 -Odd Parity; 10 - Even Parity;  11 - Unused */ // 11000
#define UART_TX_RX_LEN(x)                                                      \
  ((x & 0x1F) << 5) /*! Maximum length 32 bits */ // 1111100000

// extern uart_struct *uart_instance[MAX_UART_COUNT];
unsigned char uart0_complete;
unsigned char uart1_complete;
unsigned char uart2_complete;
unsigned int uart0_tx_isr_count;
unsigned int uart0_rcv_isr_count;
unsigned int uart1_tx_isr_count;
unsigned int uart1_rcv_isr_count;
unsigned int uart2_tx_isr_count;
unsigned int uart2_rcv_isr_count;

typedef struct 
{
  volatile unsigned short baud; /*! Baud rate configuration Register -- 16 bits*/
  unsigned short rsv0;
  volatile unsigned int tx_reg;  /*! Transmit register -- the value that needs to be
                           tranmitted needs to be written here-32 bits*/
  volatile unsigned int rcv_reg; /*! Receive register -- the value that received from
                           uart can be read from here --32 bits*/
  volatile unsigned char status; /*! Status register -- Reads various transmit and
                           receive status - 8 bits*/
  unsigned char rsv1;
  unsigned short rsv2;
  volatile unsigned short delay; /*! Delays the transmit with specified clock - 16bits*/
  unsigned short rsv3;
  volatile unsigned short control; /*! Control Register -- Configures the no. of bits
                             used, stop bits, parity enabled or not - 16bits*/
  unsigned short rsv5;
  volatile unsigned char ien; /*! Enables the required interrupts - 8 bits*/
  unsigned char rsv6;
  unsigned short rsv7;
  volatile unsigned char iqcycles; /*! 8-bit register that indicates number of input
                             qualification cycles - 8 bits*/
  unsigned char rsv8;
  unsigned short rsv9;
#ifdef USE_RX_THRESHOLD /*! This is to be used only when support is there. */
  unsigned char rx_threshold; /*! RX FIFO size configuration register - 8 bits*/
  unsigned char rsv10;
  unsigned short rsv11;
#endif
} uart_struct;

hal_uart_rx_char test;

typedef struct
{
  uint8_t u_open : 1;
  uint8_t u_rx_stall : 1;
  uint8_t u_tx_started : 1;
  uint8_t u_rx_buf;
  hal_uart_rx_char u_rx_func;
  hal_uart_tx_char u_tx_func; // func pointer that returns the data to be sent?
  hal_uart_tx_done u_tx_done;
  void *u_func_arg;
  // uint32_t u_baudrate;
} hal_uart;

static hal_uart *hal_uart_instances[MAX_UART_COUNT];

static uart_struct *uart_instances[MAX_UART_COUNT];

int
hal_uart_init_cbs(int port, hal_uart_tx_char tx_func, hal_uart_tx_done tx_done,
  hal_uart_rx_char rx_func, void *arg)
{
    if (port > MAX_UART_COUNT) return -1;

    hal_uart *u = hal_uart_instances[port];

    if (u->u_open) return -1;

    u->u_rx_func = rx_func;
    u->u_tx_func = tx_func;
    u->u_tx_done = tx_done;
    u->u_func_arg = arg;
    return 0;
}

int hal_uart_init(int uart, __attribute__((unused)) void *cfg) {
  if (uart > MAX_UART_COUNT) return -1;

	uart_instances[uart] = (uart_struct*) (UART0_START + uart * UART_OFFSET);
  return 0;
}

int hal_uart_config(int port, int32_t speed, uint8_t databits, uint8_t stopbits,
                    enum hal_uart_parity parity, __attribute((unused)) enum hal_uart_flow_ctl flow_ctl) { // recheck flow ctl
  if (port > MAX_UART_COUNT)
    return -1;
  if (databits > 32)
    return -1;
  uart_struct *uart = uart_instances[port];
  
  //Setting Baud Rate Divisor
  unsigned int baud_count = 0;
  baud_count = CLOCK_FREQUENCY / (16 * speed);
  uart->baud = baud_count;

  //Setting Control Register
  uart->control = UART_TX_RX_LEN(databits) | PARITY(parity) | STOP_BITS(stopbits);

  //Setting Interrupt Register
  // uart->ien = (0x1 << 8) - 1; // 1 << 8 = 0x100000000 - 1 = 0x11111111
  uart->ien = 0;

  hal_uart_instances[port]->u_open = 1;
  hal_uart_instances[port]->u_rx_stall = 0;
  hal_uart_instances[port]->u_tx_started = 0;
  return 0;
}

int hal_uart_close(int port) { 
  if (port > MAX_UART_COUNT)
    return -1;
  if (!hal_uart_instances[port]->u_open)
    return 1;
  
  hal_uart_instances[port]->u_open = 0; //Close the UART
  return 0;
}

static int
vajra_hal_uart_tx_fill_fifo(int port)
{
    if (port > MAX_UART_COUNT){
      return -1;
    }    
    int data = 0; 
    hal_uart *hal_uart = hal_uart_instances[port];
    uart_struct *uart = uart_instances[port];

    while ((int32_t) uart ->tx_reg >= 0) { // As long as there is something in tx_reg
        data = hal_uart->u_tx_func(hal_uart->u_func_arg); // Grab data from OS
        if (data <= 0) {
            if (hal_uart->u_tx_done) {
                hal_uart->u_tx_done(hal_uart->u_func_arg);
            }
            /* No more interrupts for TX */
            hal_uart->u_tx_started = 0;
            break;
        } else {
            uart->tx_reg = data;
        }
    }
    return data;
}

void hal_uart_start_tx(int port) { 
  if (port > MAX_UART_COUNT)
    return;

  if (!hal_uart_instances[port]->u_open)
    return;
  
  hal_uart *hal_uart = hal_uart_instances[port];
  // __HAL_DISABLE_INTERRUPTS(sr); // TODO: IMPLEMENT __HAL_DISABLE_INTERRUPTS
  if (hal_uart->u_tx_started == 0) { 
      // UART0_REG(UART_REG_TXCTRL) |= UART_TXEN;
      vajra_hal_uart_tx_fill_fifo(port);
  }
  // __HAL_ENABLE_INTERRUPTS(sr);
}

void hal_uart_start_rx(int port) {
  if (port > MAX_UART_COUNT)
    return;

  if (!hal_uart_instances[port]->u_open)
    return;

  hal_uart *hal_uart = hal_uart_instances[port];
  uart_struct *uart = uart_instances[port];

  if (hal_uart->u_rx_stall) {
      // __HAL_DISABLE_INTERRUPTS(sr);
      int rc = hal_uart->u_rx_func(hal_uart->u_func_arg, uart->rcv_reg);
      if (rc == 0) {
          hal_uart->u_rx_stall = 0;
          // UART0_REG(UART_REG_IE) |= UART_IP_RXWM;
      }
      // __HAL_ENABLE_INTERRUPTS(sr);
  }
}

void hal_uart_blocking_tx(int uart, uint8_t byte) {
  if (uart > MAX_UART_COUNT)
    return;
  
  uart_struct *instance = uart_instances[uart];
  int8_t interrupt_state = instance->ien; // Save previous enabled interrupts
  instance->ien = 0; // Disable interrupts
  instance->tx_reg = byte;
  instance->ien = interrupt_state; // Enable interrupts

  //TODO Disable Interrupts
}

#ifdef __cplusplus
}
#endif

// #endif /* H_HAL_UART_H_ */

/**
 *   @} HALUart
 * @} HAL
 */
