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

#include <string.h>
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

#include "os/mynewt.h"
#include <hal/hal_flash_int.h>
#include <hal/hal_flash.h>
#include <hal/hal_bsp.h>
#include <mcu/vajra_hal.h>

#define SPI0_OFFSET 0x00000000
#define SPI1_OFFSET 0x00000100
#define SPI2_OFFSET 0x00000200

#define SPI_CR1	     0x00020000
#define SPI_CR2	     0x00020004
#define SPI_SR       0x00020008
#define SPI_DR1	     0x0002000C
#define SPI_DR2	     0x00020010
#define SPI_DR3	     0x00020014
#define SPI_DR4	     0x00020018
#define SPI_DR5      0x0002001C
#define SPI_CRCPR    0x00020020
#define SPI_RXCRCR   0x00020024
#define SPI_TXCRCR   0x00020028

// defining SPI_CR1 register
#define SPI_CPHA	      (1 << 0)
#define SPI_CPOL	      (1 << 1)
#define SPI_MSTR	      (1 << 2)
#define SPI_BR(x)	      (x << 3)
#define SPI_SPE		      (1 << 6)
#define SPI_LSBFIRST	      (1 << 7)
#define SPI_SSI		      (1 << 8)
#define SPI_SSM		      (1 << 9)
#define SPI_RXONLY	      (1 << 10)
#define SPI_CRCL	      (1 << 11)
#define SPI_CCRCNEXT	      (1 << 12)
#define SPI_CRCEN	      (1 << 13)
#define SPI_BIDIOE	      (1 << 14)
#define SPI_BIDIMODE	      (1 << 15)
#define SPI_TOTAL_BITS_TX(x)  (x << 16)
#define SPI_TOTAL_BITS_RX(x)  (x << 24)

// defining SPI_CR2 register
#define SPI_RX_IMM_START   (1 << 16)
#define SPI_RX_START	   (1 << 15)
#define SPI_LDMA_TX	   (1 << 14)
#define SPI_LDMA_RX	   (1 << 13)
#define SPI_FRXTH	   (1 << 12)
#define SPI_DS(x)	   (x << 8)
#define SPI_TXEIE	   (1 << 7)
#define SPI_RXNEIE	   (1 << 6)
#define SPI_ERRIE	   (1 << 5)
#define SPI_FRF		   (1 << 4)
#define SPI_NSSP	   (1 << 3)
#define SPI_SSOE	   (1 << 2)
#define SPI_TXDMAEN	   (1 << 1)
#define SPI_RXDMAEN	   (1 << 0)

//defining SR register
#define SPI_FTLVL(x)	(x << 11)
#define SPI_FRLVL(x)	(x << 9)
#define SPI_FRE		(1 << 8)
#define SPI_OVR		(1 << 6)
#define SPI_MODF	(1 << 5)
#define SPI_CRCERR	(1 << 4)
#define TXE		(1 << 1)
#define RXNE		(1 << 0)

static int vajra_flash_read(const struct hal_flash *dev, uint32_t address,
        void *dst, uint32_t num_bytes);
static int vajra_flash_write(const struct hal_flash *dev, uint32_t address,
        const void *src, uint32_t num_bytes);
static int vajra_flash_erase_sector(const struct hal_flash *dev,
        uint32_t sector_address);
static int vajra_flash_sector_info(const struct hal_flash *dev, int idx,
        uint32_t *address, uint32_t *sz);
static int vajra_flash_init(const struct hal_flash *dev);

static const struct hal_flash_funcs vajra_flash_funcs = {
    .hff_read = vajra_flash_read,
    .hff_write = vajra_flash_write,
    .hff_erase_sector = vajra_flash_erase_sector,
    .hff_sector_info = vajra_flash_sector_info,
    .hff_init = vajra_flash_init
};

const struct hal_flash vajra_flash_dev = {
    .hf_itf = &vajra_flash_funcs,
    .hf_base_addr = 0x0,
    .hf_size = 0x1000,  /* XXX read from factory info? */
    .hf_sector_cnt = 0x100,       /* XXX read from factory info? */
    .hf_align = 1,
    .hf_erased_val = 0xFF,
};

// #define FLASH_CMD_READ_STATUS_REGISTER 0x05
// #define FLASH_CMD_WRITE_ENABLE 0x06
// #define FLASH_CMD_PAGE_PROGRAM 0x02
// #define FLASH_CMD_SECTOR_ERASE 0x20
//
// #define FLASH_STATUS_BUSY 0x01
// #define FLASH_STATUS_WEN  0x02

// XXX All flash functions are stubbed

static int
vajra_flash_read(const struct hal_flash *dev, uint32_t address, void *dst,
        uint32_t num_bytes){
    memset(dst, 0xFF, num_bytes);
    return 0;
}

// static int __attribute((section(".ram_text.vajra_flash_transmit")))
// vajra_flash_transmit(uint8_t out_byte){
//   return 0;
// }
//
// static int __attribute((section(".ram_text.vajra_flash_fifo_put")))
// vajra_flash_fifo_put(uint8_t out_byte){
//     return 0;
// }
//
// static int __attribute((section(".ram_text.vajra_flash_fifo_write")))
// vajra_flash_fifo_write(const uint8_t *ptr, int count){
//     return 0;
// }
//
// static int __attribute((section(".ram_text.vajra_flash_wait_till_ready")))
// vajra_flash_wait_till_ready(void){
//     return 0;
// }
//
// static int __attribute((section(".ram_text.vajra_flash_write_enable")))
// vajra_flash_write_enable(void){
//     return 0;
// }
//
// static int  __attribute((section(".ram_text.vajra_flash_write_page"))) __attribute((noinline))
// vajra_flash_write_page(const struct hal_flash *dev, uint32_t address,
//                       const void *src, uint32_t num_bytes){
//    return 0;
// }

static int
vajra_flash_write(const struct hal_flash *dev, uint32_t address,
        const void *src, uint32_t num_bytes){
   return 0;
}

static int __attribute((section(".ram_text.vajra_flash_erase_sector"))) __attribute((noinline))
vajra_flash_erase_sector(const struct hal_flash *dev, uint32_t sector_address){
    return 0;
}

static int
vajra_flash_sector_info(const struct hal_flash *dev, int idx,
        uint32_t *address, uint32_t *sz){
   return 0;
}

static int
vajra_flash_init(const struct hal_flash *dev){
    return 0;
}
