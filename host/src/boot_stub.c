/*
 * Copyright (C) 2019 Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <types.h>
#include <rtl.h>
#include <uart.h>
#include <ld_sym.h>
#include <test_printf.h>

void asm_assert(__unused int line, __unused const char *file, __unused const char *txt)
{
}

void init_primary_pcpu(void)
{
	(void)memset(&ld_bss_start, 0U, (size_t)(&ld_bss_end - &ld_bss_start));
	uart_init();

	while (1) {
		uint64_t time_count = 0x10000000UL;

		test_printf("platfrom boot: %s\r\n", __func__);
		while (time_count--) {
		}
	}
}
