#include <types.h>
#include <sprintf.h>
#include <cpu.h>
#include <per_cpu.h>
#include <mmu.h>
#include <vmx.h>
#include <ept.h>
#include <vm.h>
#include <reloc.h>
#include <init.h>
#include <board.h>
#include <vtd.h>
#include <vm_reset.h>
#include <guest_pm.h>
#include <vboot_info.h>
#include <console.h>
#include <shell.h>
#include <logmsg.h>

/* Push sp magic to top of stack for call trace */
#define SWITCH_TO(rsp, to)                                              \
{                                                                       \
	asm volatile ("movq %0, %%rsp\n"                                \
			"pushq %1\n"                                    \
			"jmpq *%2\n"                                    \
			:                                              \
			: "r"(rsp), "rm"(SP_BOTTOM_MAGIC), "a"(to));   \
}

void destroy_secure_world(__unused struct acrn_vm *vm,__unused bool need_clr_mem)
{
}

void append_seed_arg(__unused char *cmd_dst, __unused bool vm_is_sos)
{
}

bool handle_dbg_cmd(__unused const char *cmd, __unused int32_t len)
{
	return false;
}

int32_t vmcall_vmexit_handler(__unused struct acrn_vcpu *vcpu)
{
	return 0;
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

/*TODO: move into debug module */
static void init_debug_pre_stub(void)
{
	/* Initialize console */
	console_init();

	/* Enable logging */
	init_logmsg(CONFIG_LOG_DESTINATION);
}

/*TODO: move into debug module */
static void init_debug_post_stub(uint16_t pcpu_id)
{
	if (pcpu_id == BOOT_CPU_ID) {
		/* Initialize the shell */
		shell_init();
		console_setup_timer();
	}

	profiling_setup();
}

/*TODO: move into guest-vcpu module */
static void enter_guest_mode(uint16_t pcpu_id)
{
	vmx_on();

	(void)launch_vms(pcpu_id);
	switch_to_idle(default_idle);
	/* Control should not come here */
	cpu_dead();
}
static void init_primary_pcpu_post(void)
{
	init_debug_pre_stub();
	init_pcpu_post(BOOT_CPU_ID);

	disable_smep();
	disable_smap();

	init_debug_post_stub(BOOT_CPU_ID);

	enter_guest_mode(BOOT_CPU_ID);
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

	init_debug_post_stub(pcpu_id);

	enter_guest_mode(pcpu_id);
}
