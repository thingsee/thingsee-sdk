/****************************************************************************
 * configs/haltian-tsone/scripts/stm32l162ve.ld
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <nuttx/config.h>

/* The STM32L162VE has 512KB of FLASH beginning at address 0x0800:0000 and
 * 80Kb of SRAM at address 0x20000000.
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0800:0000 address range.
 */

#define _FLASH_BASE 0x08000000

#ifndef CONFIG_NUTTX_DFU_IMAGE_FLASHING_ADDRESS
#  define CONFIG_NUTTX_DFU_IMAGE_FLASHING_ADDRESS _FLASH_BASE
#endif

#define _IMAGE_ADDR  CONFIG_NUTTX_DFU_IMAGE_FLASHING_ADDRESS
#define _IMAGE_SIZE  (512K - (_IMAGE_ADDR - _FLASH_BASE))

MEMORY
{
    flash (rx) : ORIGIN = _IMAGE_ADDR, LENGTH = _IMAGE_SIZE
    sram (rwx) : ORIGIN = 0x20000000, LENGTH = 80K
}

_application_flash_size = _application_flash_end - _application_flash_start;

OUTPUT_ARCH(arm)
ENTRY(_stext)
SECTIONS
{
	.text : {
		_application_flash_start = ABSOLUTE(.);
		_stext = ABSOLUTE(.);
		KEEP(*(.vectors))
		KEEP(*(.application_flash_size .application_flash_size.*))
		*(.text .text.*)
		KEEP(*(.text .unusedtext))
		*(.fixup)
		*(.gnu.warning)
		*(.rodata .rodata.*)
		*(.gnu.linkonce.t.*)
		*(.glue_7)
		*(.glue_7t)
		*(.got)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)
		_etext = ABSOLUTE(.);
	} > flash

	.init_section : {
		_sinit = ABSOLUTE(.);
		*(.init_array .init_array.*)
		_einit = ABSOLUTE(.);
	} > flash

	.ARM.extab : {
		*(.ARM.extab*)
	} > flash

	__exidx_start = ABSOLUTE(.);
	.ARM.exidx : {
		*(.ARM.exidx*)
	} > flash
	__exidx_end = ABSOLUTE(.);

	_eronly = ABSOLUTE(.);

	.data : {
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		_edata = ABSOLUTE(.);
	} > sram AT > flash

	.nuttx.randomdata : {
		*(.nuttx.randomdata.*)
	} > sram

	.padding : {
		_spadding = ABSOLUTE(.);
		FILL(0xEEEEEEEE);
		. = ALIGN(4);
		QUAD(0x004e4149544c4148);
		_epadding = ABSOLUTE(.);
	} >flash

	.bss : {
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		_ebss = ABSOLUTE(.);
	} > sram

#ifdef CONFIG_BOARD_ENABLE_FILL_FLASH
	.fill LOADADDR(.padding) + (_epadding - _spadding) : {
		FILL(0x00000000);
#ifdef CONFIG_OCIMAGE
		. = ORIGIN(flash) + LENGTH(flash) - 20 - 1;
#else
		. = ORIGIN(flash) + LENGTH(flash) - 1;
#endif
		BYTE(0x00)
	} >flash
#endif

	.app_end : {
		_application_flash_end = ABSOLUTE(.);
	} > flash

	/* Stabs debugging sections. */
	.stab 0 : { *(.stab) }
	.stabstr 0 : { *(.stabstr) }
	.stab.excl 0 : { *(.stab.excl) }
	.stab.exclstr 0 : { *(.stab.exclstr) }
	.stab.index 0 : { *(.stab.index) }
	.stab.indexstr 0 : { *(.stab.indexstr) }
	.comment 0 : { *(.comment) }
	.debug_abbrev 0 : { *(.debug_abbrev) }
	.debug_info 0 : { *(.debug_info) }
	.debug_line 0 : { *(.debug_line) }
	.debug_pubnames 0 : { *(.debug_pubnames) }
	.debug_aranges 0 : { *(.debug_aranges) }
}
