/*
 * Tests vector instruction support
 *
 * Copyright 2018 IBM Corp.
 *
 * Authors:
 *    Janosch Frank <frankja@de.ibm.com>
 *
 * This code is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Library General Public License version 2.
 */
#include <libcflat.h>
#include <asm/page.h>
#include <asm/facility.h>
#include <asm/interrupt.h>
#include <asm-generic/barrier.h>

static uint8_t pagebuf[PAGE_SIZE] __attribute__((aligned(PAGE_SIZE)));

/* Fills all vector registers with data from addr */
static inline void vlm_all(unsigned long *addr)
{
	asm volatile(" .machine z13\n"
		     " vlm 0, 15, %[a]\n"
		     : : [a]  "Q" (*addr)
		     :	"v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8",
			"v9", "v10", "v11", "v12", "v13", "v14", "v15");
	asm volatile(" .machine z13\n"
		     " vlm 16, 31, %[a]\n"
		     : : [a]  "Q" (*(addr+256/8))
		     :	"v16", "v17", "v18", "v19", "v20", "v21", "v22",
			"v23", "v24", "v25", "v26", "v27", "v28", "v29",
			"v30", "v31");
}

static void test_add(void)
{
	static struct prm {
		__uint128_t a,b,c;
	} prm __attribute__((aligned(16)));

	prm.a = prm.b = prm.c = 21;

	asm volatile(" .machine z13\n"
		     " vl 0, %[v1]\n"
		     " vl 1, %[v2]\n"
		     " va 2, 0, 1, 4\n"
		     " vst 2, %[v3]\n"
		     : [v3]  "=Q" (prm.c)
		     : [v1]  "Q" (prm.a), [v2]  "Q" (prm.b)
		     : "v0", "v1", "v2", "memory");
	report("adding 21", prm.c == 42);
}

/* z14 vector extension test */
static void test_ext1_nand(void)
{
	bool has_vext = test_facility(134);
	static struct prm {
		__uint128_t a,b,c;
	} prm __attribute__((aligned(16)));

	report_xfail("Vector extensions 1 available", !has_vext, has_vext);
	if (!has_vext)
		return;

	memset(&prm, 0xff, sizeof(prm));

	asm volatile(" .machine z13\n"
		     " vl 0, %[v1]\n"
		     " vl 1, %[v2]\n"
		     " .byte 0xe7, 0x20, 0x10, 0x00, 0x00, 0x6e\n" /* vnn */
		     " vst 2, %[v3]\n"
		     : [v3]  "=Q" (prm.c)
		     : [v1]  "Q" (prm.a), [v2]  "Q" (prm.b)
		     : "v0", "v1", "v2", "memory");
	report("nand ff", !prm.c);
}

/* z14 bcd extension test */
static void test_bcd_add(void)
{
	bool has_bcd = test_facility(135);
	static struct prm {
		__uint128_t a,b,c;
	} prm __attribute__((aligned(16)));

	report_xfail("Vector BCD extensions available", !has_bcd, has_bcd);
	if (!has_bcd)
		return;

	prm.c = 0;
	prm.a = prm.b = 0b001000011100;

	asm volatile(" .machine z13\n"
		     " vl 0, %[v1]\n"
		     " vl 1, %[v2]\n"
		     " .byte 0xe6, 0x20, 0x10, 0x01, 0x00, 0x71\n" /* vap */
		     " vst 2, %[v3]\n"
		     : [v3]  "=Q" (prm.c)
		     : [v1]  "Q" (prm.a), [v2]  "Q" (prm.b)
		     : "v0", "v1", "v2", "memory");
	report("bcd add 21", prm.c == 0x42c);
}

static void init(void)
{
	/* Enable vector instructions */
	ctl_set_bit(0, 17);

	/* Preset vector registers to 0xff */
	memset(pagebuf, 0xff, PAGE_SIZE);
	vlm_all((u64*)pagebuf);
}

int main(void)
{
	bool has_vregs = test_facility(129);

	report_prefix_push("vector");
	report_xfail("Basic vector facility available", !has_vregs, has_vregs);
	if (!has_vregs)
		goto done;

	init();
	test_add();
	test_ext1_nand();
	test_bcd_add();

done:
	report_prefix_pop();
	return report_summary();
}
