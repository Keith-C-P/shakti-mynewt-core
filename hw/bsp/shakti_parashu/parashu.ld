/***************************************************************************
 * Project           		     : shakti devt board
 * Name of the file	     	   : link.ld
 * Brief description of file : linker file
 * Name of Author    	       : Sathya Narayanan N & Abhinav Ramnath
 * Email-ID                  : sathya281@gmail.com

 Copyright (C) 2020  IIT Madras. All rights reserved.

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
OUTPUT_ARCH( "riscv" )
ENTRY(_start)

MEMORY
{
  /* flash (rxai!w) : ORIGIN = 0x20000000, LENGTH = 32K */
  ram (wxa!ri) : ORIGIN = 0x80000000, LENGTH = 128K
}

SECTIONS
{
  .text.init : {
    *(.text.init) 
  } >ram

  .text : {
    *(.text) 
  } >ram

  _ram_start=ORIGIN(ram);

  .rodata : 
  {
    __rodata_start = .;
    *(.rodata)
    *(.rodata.*)
    *(.gnu.linkonce.r.*)
    __rodata_end = .;
  } >ram

  .sdata : 
  {
    __global_pointer$ = . + 0x800;
    *(.srodata.cst16) *(.srodata.cst8) *(.srodata.cst4) *(.srodata.cst2) *(.srodata*)
    *(.sdata .sdata.* .gnu.linkonce.s.*)
  } >ram

  .sbss : 
  {
    __sbss_start = .;
    *(.sbss)
    *(.sbss.*)
    *(.gnu.linkonce.sb.*)
    __sbss_end = .;
  } >ram

  .data : 
  {
    . = ALIGN(4);
    __data_start = .;
    *(.data)
    *(.data.*)
    *(.gnu.linkonce.d.*)
    __data_end = .;
  }  >ram

  .bss : 
  {
    . = ALIGN(4);
    __bss_start = .;
    *(.bss)
    *(.bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end = .;
  } >ram

  .tdata :
  {
    _tls_data = .;
    *(.tdata.begin)
    *(.tdata)
    *(.tdata.end)
    _tls_end = .;
  } >ram

  .tbss :
  {
    *(.tbss)
    *(.tbss.end)
  } >ram
  
  . = ALIGN(4);
  _end = .;
  . = _ram_start + LENGTH(ram) - 0x400;
  _free_space = . - _end;
  _STACK_SIZE = (_free_space * 50) / 100 ; 
  _HEAP_SIZE = _free_space - _STACK_SIZE;

  .stack : 
  {
    _stack_end = .;
    __stack_pointer$ = .;
    _sp = .;
    _stack = . - _STACK_SIZE;
  } >ram

  . = _stack;
  .heap : {
    _heap = . - _HEAP_SIZE;
    _heap_end = .;
  } >ram
  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >ram

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
    KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >ram

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
    KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >ram
}
