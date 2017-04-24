/*
 * bottloader_asm.s
 *
 *  Created on: 9 Oca 2016
 *      Author: LENOVO
 */

.syntax unified

	.align 2
	.section .bootloader_func.__dsb
	.global __dsb
    .func
	.thumb_func
	.type	__dsb, %function

__dsb:
     dsb

	.endfunc
