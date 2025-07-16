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
#include "uart.h"
#include "mcu/parashu_hal.h"

struct hal_uart
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
};

unsigned char uart0_complete;
unsigned char uart1_complete;
unsigned char uart2_complete;
unsigned int uart0_tx_isr_count;
unsigned int uart0_rcv_isr_count;
unsigned int uart1_tx_isr_count;
unsigned int uart1_rcv_isr_count;
unsigned int uart2_tx_isr_count;
unsigned int uart2_rcv_isr_count;

struct uart_dev os_bsp_uart0;

static struct hal_uart volatile hal_uart_instances[MAX_UART_COUNT];
static struct uart_struct *volatile uart_instances[MAX_UART_COUNT];

void
uart_init(void)
{
  for (int i = 0; i < MAX_UART_COUNT; i++)
  {
    uart_instances[i] = (struct uart_struct *)(UART0_START + i * UART_OFFSET);
  }
}

/*
* Creating UART for the HAL itself
*/
void
parashu_periph_create_uart(void)
{
    int rc;

    rc = os_dev_create((struct os_dev *)&os_bsp_uart0, "uart0", OS_DEV_INIT_PRIMARY, 0, uart_hal_init, NULL);
    assert(rc == 0);
}

int
hal_uart_init_cbs(int port, hal_uart_tx_char tx_func, hal_uart_tx_done tx_done,
  hal_uart_rx_char rx_func, void *arg)
{
    if (port > MAX_UART_COUNT && port < 0) return -1;

    hal_uart_instances[port].u_rx_func = rx_func;
    hal_uart_instances[port].u_tx_func = tx_func;
    hal_uart_instances[port].u_tx_done = tx_done;
    hal_uart_instances[port].u_func_arg = arg;

    return 0;
}

int hal_uart_init(int port, __attribute__((unused)) void *cfg) {
  if (port > MAX_UART_COUNT || port < 0)
    return -1;

  uart_instances[port] = (struct uart_struct*) (UART0_START + port * UART_OFFSET);
  return 0;
}

int hal_uart_config(int port, int32_t speed, uint8_t databits, uint8_t stopbits,
                    enum hal_uart_parity parity, __attribute((unused)) enum hal_uart_flow_ctl flow_ctl) { // recheck flow ctl
  if (port > MAX_UART_COUNT || port < 0)
    return -1;
  if (databits > 32)
    return -1;
  volatile struct uart_struct *uart = uart_instances[0];
  
  //Setting Baud Rate Divisor
  uint16_t baud_count = CLOCK_FREQUENCY / (16 * speed);
  uart->baud = baud_count;

  //Setting Control Register
  uart->control = UART_TX_RX_LEN(databits) | PARITY(parity) | STOP_BITS(stopbits);

  //Setting Interrupt Register
  // uart->ien = (0x1 << 8) - 1; // 1 << 8 = 0x100000000 - 1 = 0x11111111
  uart->ien = 0;

  hal_uart_instances[port].u_open = 1;
  hal_uart_instances[port].u_rx_stall = 0;
  hal_uart_instances[port].u_tx_started = 0;
  return 0;
}

int hal_uart_close(int port) {
  if (port > MAX_UART_COUNT || port < 0)
    return -1;
  if (!hal_uart_instances[port].u_open)
    return 1;
  
  hal_uart_instances[port].u_open = 0; //Close the UART
  return 0;
}

static int
parashu_hal_uart_tx_fill_fifo(int port)
{
  if (port > MAX_UART_COUNT || port < 0)
    return -1;
  if (!uart_instances[port]) return -1;

  int data = 0;
  volatile struct uart_struct *uart = uart_instances[port];

  while ((uint32_t)uart_instances[port]->control && STS_TX_EMPTY >= 0)
  {                                                   // As long as there is something in tx_reg
    data = hal_uart_instances[port].u_tx_func(hal_uart_instances[port].u_func_arg); // Grab data from OS
    if (data <= 0)
    {
      if (hal_uart_instances[port].u_tx_done)
      {
        hal_uart_instances[port].u_tx_done(hal_uart_instances[port].u_func_arg);
      }
      /* No more interrupts for TX */
      hal_uart_instances[port].u_tx_started = 0;
      break;
    }
    else
    {
      uart->tx_reg = data;
    }
  }
  return data;
}

void hal_uart_start_tx(int port) {
  if (port > MAX_UART_COUNT || port < 0)
    return;

  if (!hal_uart_instances[port].u_open)
    return;
  
  // __HAL_DISABLE_INTERRUPTS(sr); // TODO: IMPLEMENT __HAL_DISABLE_INTERRUPTS
  if (hal_uart_instances[port].u_tx_started == 0) { 
      // UART0_REG(UART_REG_TXCTRL) |= UART_TXEN;
      hal_uart_instances[port].u_tx_started = 1;
      parashu_hal_uart_tx_fill_fifo(port);
  }
  // __HAL_ENABLE_INTERRUPTS(sr);
}

void hal_uart_start_rx(int port) {
  if (port > MAX_UART_COUNT || port < 0)
    return;

  if (!hal_uart_instances[port].u_open)
    return;

  volatile struct hal_uart hal_uart = hal_uart_instances[port];
  volatile struct uart_struct *uart = uart_instances[port];

  if (hal_uart.u_rx_stall) {
      // __HAL_DISABLE_INTERRUPTS(sr);
      int rc = hal_uart.u_rx_func(hal_uart.u_func_arg, uart->rcv_reg);
      if (rc == 0) {
          hal_uart.u_rx_stall = 0;
          // UART0_REG(UART_REG_IE) |= UART_IP_RXWM;
      }
      // __HAL_ENABLE_INTERRUPTS(sr);
  }
}

void hal_uart_blocking_tx(int port , uint8_t byte) {
  if (port > MAX_UART_COUNT || port < 0)
    return;
  
  volatile struct uart_struct *instance = uart_instances[port];
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
