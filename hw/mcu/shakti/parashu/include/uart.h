#ifndef UART_H
#define UART_H
#include <stdint.h>
#include "bsp/bsp.h"
#include "uart/uart.h"
#include "uart_hal/uart_hal.h"
/* Struct to access UART registers as 32 bit registers */
struct uart_struct
{
    uint16_t baud;        /*! Baud rate configuration Register -- 16 bits*/
    uint16_t reserv0;     /*! reserved */
    uint32_t tx_reg;        /*! Transmit register -- the value that needs to be tranmitted needs to be written here-32 bits*/
    uint32_t rcv_reg;       /*! Receive register -- the value that received from uart can be read from here --32 bits*/
    uint8_t status;       /*! Status register -- Reads various transmit and receive status - 8 bits*/
    uint8_t reserv1;      /*! reserved */
    uint16_t reserv2;     /*! reserved */
    uint16_t delay;       /*! Delays the transmit with specified clock - 16bits*/
    uint16_t reserv3;     /*! reserved */
    uint16_t control;     /*! Control Register -- Configures the no. of bits used, stop bits, parity enabled or not - 16bits*/
    uint16_t reserv5;     /*! reserved */
    uint8_t ien;          /*! Enables the required interrupts - 8 bits*/
    uint8_t reserv6;      /*! reserved */
    uint16_t reserv7;     /*! reserved */
    uint8_t iqcycles;     /*! 8-bit register that indicates number of input qualification cycles - 8 bits*/
    uint8_t reserv8;      /*! reserved */
    uint16_t reserv9;     /*! reserved */
#ifdef USE_RX_THRESHOLD         /*! This is to be used only when support is there. */
    uint8_t rx_threshold; /*! RX FIFO size configuration register - 8 bits*/
    uint8_t reserv10;     /*! reserved */
    uint16_t reserv11;    /*! reserved */
#endif
};

#define USE_INTERRUPT

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
#define UARTX_BUFFER_SIZE 100

/* UART control register */
#define STOP_BITS(x) ((x & 3) << 1)         /*! 00 - 1 stop bits, 01 - 1.5 stop bits; 10 - 2 stop bits; 11 unused */
#define PARITY(x) ((x & 3) << 3)            /*! 00 --- No parity; 01 -Odd Parity; 10 - Even Parity;  11 - Unused */
#define UART_TX_RX_LEN(x) ((x & 0x1F) << 5) /*! Maximum length 32 bits */

extern unsigned char uart0_complete;
extern unsigned char uart1_complete;
extern unsigned char uart2_complete;
extern unsigned int uart0_tx_isr_count;
extern unsigned int uart0_rcv_isr_count;
extern unsigned int uart1_tx_isr_count;
extern unsigned int uart1_rcv_isr_count;
extern unsigned int uart2_tx_isr_count;
extern unsigned int uart2_rcv_isr_count;

extern struct uart_dev os_bsp_uart0;

void uart_init(void);
void parashu_periph_create_uart(void);
#endif