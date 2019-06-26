/*
 * Copyright (C) 2019 Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <types.h>
#include <spinlock.h>
#include <uart.h>
#include <io.h>
#include <pci.h>
#include <pgtable.h>

//#if defined(CONFIG_SERIAL_PIO_BASE)
static bool serial_port_mapped = true;
static uint64_t uart_base_address = 0x3F8;
//static uint64_t uart_base_address = CONFIG_SERIAL_PIO_BASE;
//#elif defined(CONFIG_SERIAL_PCI_BDF)
//static bool serial_port_mapped;
//static uint64_t uart_base_address = 0xFC000000UL;
//#endif

typedef uint32_t uart_reg_t;

static spinlock_t uart_rx_lock;
static spinlock_t uart_tx_lock;

static inline uint32_t uart_read_reg(uint64_t base, uint16_t reg_idx)
{
	if (serial_port_mapped) {
		return pio_read8((uint16_t)base + reg_idx);
	} else {
		return mmio_read32((void *)((uint32_t *)hpa2hva(base) + reg_idx));
	}
}

static inline void uart_write_reg(uint64_t base, uint32_t val, uint16_t reg_idx)
{
	if (serial_port_mapped) {
		pio_write8((uint8_t)val, (uint16_t)base + reg_idx);
	} else {
		mmio_write32(val, (void *)((uint32_t *)hpa2hva(base) + reg_idx));
	}
}

static void uart_calc_baud_div(uint32_t ref_freq, uint32_t *baud_div_ptr, uint32_t baud_rate_arg)
{
	uint32_t baud_rate = baud_rate_arg;
	uint32_t baud_multiplier = baud_rate < BAUD_460800 ? 16U : 13U;

	if (baud_rate == 0U) {
		baud_rate = BAUD_115200;
	}
	*baud_div_ptr = ref_freq / (baud_multiplier * baud_rate);
}

static void uart_set_baud_rate(uint32_t baud_rate)
{
	uint32_t baud_div, duart_clock = UART_CLOCK_RATE;
	uart_reg_t temp_reg;

	/* Calculate baud divisor */
	uart_calc_baud_div(duart_clock, &baud_div, baud_rate);

	/* Enable DLL and DLM registers for setting the Divisor */
	temp_reg = uart_read_reg(uart_base_address, UART16550_LCR);
	temp_reg |= LCR_DLAB;
	uart_write_reg(uart_base_address, temp_reg, UART16550_LCR);

	/* Write the appropriate divisor value */
	uart_write_reg(uart_base_address, ((baud_div >> 8U) & 0xFFU), UART16550_DLM);
	uart_write_reg(uart_base_address, (baud_div & 0xFFU), UART16550_DLL);

	/* Disable DLL and DLM registers */
	temp_reg &= ~LCR_DLAB;
	uart_write_reg(uart_base_address, temp_reg, UART16550_LCR);
}

void uart_init(void)
{

	spinlock_init(&uart_rx_lock);
	spinlock_init(&uart_tx_lock);
	/* Enable TX and RX FIFOs */
	uart_write_reg(uart_base_address, FCR_FIFOE | FCR_RFR | FCR_TFR, UART16550_FCR);

	/* Set-up data bits / parity / stop bits. */
	uart_write_reg(uart_base_address, (LCR_WL8 | LCR_NB_STOP_BITS_1 | LCR_PARITY_NONE), UART16550_LCR);

	/* Disable interrupts (we use polling) */
	uart_write_reg(uart_base_address, UART_IER_DISABLE_ALL, UART16550_IER);

	/* Set baud rate */
	uart_set_baud_rate(BAUD_115200);

	/* Data terminal ready + Request to send */
	uart_write_reg(uart_base_address, MCR_RTS | MCR_DTR, UART16550_MCR);
}

char uart_getc(void)
{
	char ret = -1;

	spinlock_obtain(&uart_rx_lock);

	/* If a character has been received, read it */
	if ((uart_read_reg(uart_base_address, UART16550_LSR) & LSR_DR) == LSR_DR) {
		/* Read a character */
		ret = uart_read_reg(uart_base_address, UART16550_RBR);

	}
	spinlock_release(&uart_rx_lock);
	return ret;
}

void uart_putc(char c)
{
	uint8_t temp;
	uint32_t reg;

	/* Ensure there are no further Transmit buffer write requests */
	do {
		reg = uart_read_reg(uart_base_address, UART16550_LSR);
	} while ((reg & LSR_THRE) == 0U || (reg & LSR_TEMT) == 0U);

	temp = (uint8_t)c;
	/* Transmit the character. */
	uart_write_reg(uart_base_address, (uint32_t)temp, UART16550_THR);
}

size_t uart_puts(const char *buf, uint32_t len)
{
	uint32_t i;

	spinlock_obtain(&uart_tx_lock);
	for (i = 0U; i < len; i++) {
		/* Transmit character */
		uart_putc(*buf);
		if (*buf == '\n') {
			/* Append '\r', no need change the len */
			uart_putc('\r');
		}
		buf++;
	}
	spinlock_release(&uart_tx_lock);
	return len;
}
