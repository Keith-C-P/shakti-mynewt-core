/***************************************************************************
* Project                     : shakti devt board
* Name of the file	      : clint_driver.c
* Brief Description of file   : source file for clint.
* Name of Author    	      : Sathya Narayanan N
* Email ID                    : sathya281@gmail.com

 Copyright (C) 2019  IIT Madras. All rights reserved.

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses/>.
***************************************************************************/
/**
@file clint_driver.c
@brief source file for clint.
@detail This file is a driver file for clint. The file contains the clint 
interrupt handler, configure the counter and support for e and c class clint timers. 
*/

#include <inttypes.h>
#include "clint.h"
#include "bsp/bsp.h"

/** @fn void configure_counter( uint64_t value)
 * @brief sets up the timer
 * @details sets the mtimecmp to current mtime + delta
 * @param unsigned 64bit int (delta value after which interrupt happens)
 */
// void configure_counter( uint64_t value)
// {
// 	// log_trace("\nconfigure_counter entered\n");
//
// 	*mtimecmp = *mtime + value;
//
// 	// log_debug("mtimecmp value = %d\n", *mtimecmp);
// 	// log_debug("mtime value = %d\n", *mtime);
//
// 	// log_trace("\nconfigure_counter exited\n");
// }

/** @fn void mach_clint_handler(uintptr_t int_id, uintptr_t epc)
 * @brief handler for machine timer interrupt
 * @details handler for machine timer interrupt. This handles the timer interrupt and sets mtimecmp to clear timer interrupt.
 * @param unsigned int ptr int_id
 * @param unsigned int ptr epc
 */
void mach_clint_handler( __attribute__((unused)) uintptr_t int_id,  __attribute__((unused)) uintptr_t epc)
{
	// log_trace("\nmach_clint_handler entered\n");

	//set mtimecmp to some value. On appln reqt basis handle timer interrupt
	CLINT_REG(MTIMECMP + 4) = -1;
	CLINT_REG(MTIMECMP) = -1;

	// log_info("Timer interrupt handled \n");

	// log_trace("mach_clint_handler exited\n");
}
