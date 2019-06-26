#include <types.h>
#include <cpu.h>
#include <irq.h>
#include <vlapic.h>
#include <per_cpu.h>
#include <init.h>
#include <uart.h>
#include <vm_reset.h>
#include <platform_acpi_info.h>
#include <default_acpi_info.h>
#include <test_printf.h>

void uart16550_init(__unused bool eraly_boot)
{
}

void asm_assert(__unused int line, __unused const char *file, __unused const char *txt)
{
}

void do_logmsg(__unused uint32_t severity, __unused const char *fmt, ...)
{
}

void ptdev_init(void)
{
}

void printf(__unused const char *fmt, ...)
{
}

void dump_exception(__unused struct intr_excp_ctx *ctx, __unused uint16_t pcpu_id)
{
}

void TRACE_2L(__unused uint32_t evid, __unused uint64_t e, __unused uint64_t f)
{
}

bool is_pci_dbg_uart(__unused union pci_bdf bdf_value)
{
	return true;
}

bool handle_dbg_cmd(__unused const char *cmd, __unused int32_t len)
{
	return false;
}

void vlapic_set_intr(__unused struct acrn_vcpu *vcpu, __unused uint32_t vector, __unused bool level)
{
}

uint64_t sos_vm_hpa2gpa(uint64_t hpa)
{
	return hpa;
}


void vlapic_set_apicv_ops(void)
{
}

void vlapic_restore(__unused struct acrn_vlapic *vlapic, __unused const struct lapic_regs *regs)
{
}

void set_vcpu_regs(__unused struct acrn_vcpu *vcpu, __unused struct acrn_vcpu_regs *vcpu_regs)
{
}

void *gpa2hva(__unused struct acrn_vm *vm, uint64_t x)
{
	return (void *)x;
}

bool is_sos_vm(__unused const struct acrn_vm *vm)
{
	return true;
}

int32_t sbl_init_vm_boot_info(__unused struct acrn_vm *vm)
{
	return 0;
}

int32_t uefi_init_vm_boot_info(__unused struct acrn_vm *vm)
{
	return 0;
}

/* Push sp magic to top of stack for call trace */
#define SWITCH_TO(rsp, to)                                              \
{                                                                       \
	asm volatile ("movq %0, %%rsp\n"                                \
			"pushq %1\n"                                    \
			"jmpq *%2\n"                                    \
			:                                              \
			: "r"(rsp), "rm"(SP_BOTTOM_MAGIC), "a"(to));   \
}


void disable_smep(void)
{
	uint64_t val64 = 0UL;

	/* disable CR4.SMEP*/
	CPU_CR_READ(cr4, &val64);
	CPU_CR_WRITE(cr4, val64 & (~CR4_SMEP));
}

void disable_smap(void)
{
	uint64_t val64 = 0UL;

	/* disable CR4.SMAP*/
	CPU_CR_READ(cr4, &val64);
	CPU_CR_WRITE(cr4, val64 & (~CR4_SMAP));
}

static void init_primary_pcpu_post(void)
{
	uart_init();

	//init_debug_pre();
	init_pcpu_post(BOOT_CPU_ID);
	//init_seed();

	disable_smep();
	disable_smap();

	while (1) {
		test_printf("hw management: %s pcpu_id = %d\r\n", __func__, get_pcpu_id());
		udelay(1000*1000);
	}

	//init_debug_post(BOOT_CPU_ID);

	//enter_guest_mode(BOOT_CPU_ID);
}

/* NOTE: this function is using temp stack, and after SWITCH_TO(runtime_sp, to)
 * it will switch to runtime stack.
 */
void init_primary_pcpu(void)
{
	uint64_t rsp;

	init_pcpu_pre(true);

	/* Switch to run-time stack */
	rsp = (uint64_t)(&get_cpu_var(stack)[CONFIG_STACK_SIZE - 1]);
	rsp &= ~(CPU_STACK_ALIGN - 1UL);
	SWITCH_TO(rsp, init_primary_pcpu_post);
}

void init_secondary_pcpu(void)
{
	uint16_t pcpu_id;

	init_pcpu_pre(false);

	pcpu_id = get_pcpu_id();

	init_pcpu_post(pcpu_id);
	disable_smep();
	disable_smap();
	while (1) {
		test_printf("hw management: %s pcpu_id = %d\r\n", __func__, get_pcpu_id());
		udelay(1000*1000);
	}

	//	init_debug_post(pcpu_id);

	//	enter_guest_mode(pcpu_id);
}
