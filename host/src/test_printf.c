/*
 * Copyright (C) 2018 Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <types.h>
#include <rtl.h>
#include <util.h>
#include <sprintf.h>
#include <uart.h>
#include <cpu.h>
#include <spinlock.h>

static spinlock_t printf_lock = { .head = 0U, .tail = 0U };
static void test_char_out(size_t cmd, const char *s_arg, uint32_t sz_arg,
		struct snprint_param *param)
{
	const char *s = s_arg;
	uint32_t sz = sz_arg;
	/* pointer to an integer to store the number of characters */
	size_t nchars = param->wrtn;
	/* working pointer */
	const char *p = s;
	size_t len;

	/* copy mode ? */
	if (cmd == PRINT_CMD_COPY) {
		if (sz > 0U) { /* copy 'sz' characters */
			len = uart_puts(s, sz);
			s += len;
		}

		nchars += (s - p);
	} else {
		/* fill mode */
		nchars += sz;
		while (sz != 0U) {
			uart_putc(*s);
			sz--;
		}
	}
	param->wrtn = nchars;
}

void test_vprintf(const char *fmt, va_list args)
{
	/* struct to store all necessary parameters */
	struct print_param param;
	struct snprint_param snparam;

	/* initialize parameters */
	(void)memset(&snparam, 0U, sizeof(snparam));
	(void)memset(&param, 0U, sizeof(param));
	param.emit = test_char_out;
	param.data = &snparam;

	/* execute the printf() */
	do_print(fmt, &param, args);
}

void test_printf(const char *fmt, ...)
{
	/* variable argument list needed for do_print() */
	va_list args;
	uint64_t rflags;

	spinlock_irqsave_obtain(&printf_lock, &rflags);
	va_start(args, fmt);

	/* execute the printf() */
	test_vprintf(fmt, args);

	/* destroy parameter list */
	va_end(args);
	spinlock_irqrestore_release(&printf_lock, rflags);
}
