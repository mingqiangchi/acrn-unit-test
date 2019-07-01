#include <types.h>
#include <util.h>
#include <sprintf.h>
#include <cpu.h>
#include <vmx.h>
#include <mmu.h>
#include <timer.h>
#include <per_cpu.h>
#include <vtd.h>
#include <board.h>
#include <uart.h>
#include <ept.h>
#include <init.h>
#include <vcpu.h>
#include <vm.h>
#include <reloc.h>
#include <vm_reset.h>
#include <vboot_info.h>
#include <platform_acpi_info.h>
#include <default_acpi_info.h>
#include <test_printf.h>

static struct e820_entry sos_ve820_stub[E820_MAX_ENTRIES];
/* host reset register defined in ACPI */
static struct acpi_reset_reg host_reset_reg = {
	.reg = {
		.space_id = RESET_REGISTER_SPACE_ID,
		.bit_width = RESET_REGISTER_BIT_WIDTH,
		.bit_offset = RESET_REGISTER_BIT_OFFSET,
		.access_size = RESET_REGISTER_ACCESS_SIZE,
		.address = RESET_REGISTER_ADDRESS,
	},
	.val = RESET_REGISTER_VALUE
};

struct acpi_reset_reg *get_host_reset_reg_data(void)
{
	return &host_reset_reg;
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

void do_logmsg(__unused uint32_t severity, __unused const char *fmt, ...)
{
}

void asm_assert(__unused int line, __unused const char *file, __unused const char *txt)
{
}

void uart16550_init(__unused bool eraly_boot)
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

void TRACE_4I(__unused uint32_t evid, __unused uint32_t a, __unused uint32_t b,
		__unused uint32_t c, __unused uint32_t d)
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

void vlapic_set_apicv_ops(void)
{
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

int32_t vlapic_create(__unused struct acrn_vcpu *vcpu)
{
	return 0;
}

bool is_x2apic_enabled(__unused const struct acrn_vlapic *vlapic)
{
	return false;
}

void register_pio_default_emulation_handler(__unused struct acrn_vm *vm)
{
}

void register_mmio_default_emulation_handler(__unused struct acrn_vm *vm)
{
}

uint64_t vlapic_get_tsc_deadline_msr(__unused const struct acrn_vlapic *vlapic)
{
	return 0;
}

struct acrn_vpic *vm_pic(const struct acrn_vm *vm)
{
	return (struct acrn_vpic *)&(vm->arch_vm.vpic);
}

void vpic_pending_intr(__unused struct acrn_vpic *vpic, __unused uint32_t *vecptr)
{
}

void vpic_intr_accepted(__unused struct acrn_vpic *vpic, __unused uint32_t vector)
{
}

bool vlapic_has_pending_delivery_intr(__unused struct acrn_vcpu *vcpu)
{
	return false;
}

bool vlapic_inject_intr(__unused struct acrn_vlapic *vlapic,__unused bool guest_irq_enabled,
		__unused bool injected)
{
	return false;
}

void profiling_vmenter_handler(__unused struct acrn_vcpu *vcpu)
{
}

void profiling_pre_vmexit_handler(__unused struct acrn_vcpu *vcpu)
{
}

void profiling_post_vmexit_handler(__unused struct acrn_vcpu *vcpu)
{
}

void append_seed_arg(__unused char *cmd_dst, __unused bool vm_is_sos)
{
}

void vlapic_set_intr(__unused struct acrn_vcpu *vcpu, __unused uint32_t vector,
		__unused bool level)
{
}

void vlapic_restore(__unused struct acrn_vlapic *vlapic, __unused const struct lapic_regs *regs)
{
}

int32_t vmcall_vmexit_handler(__unused struct acrn_vcpu *vcpu)
{
	return -1;
}

int32_t pio_instr_vmexit_handler(__unused struct acrn_vcpu *vcpu)
{
	return -1;
}

int32_t apic_access_vmexit_handler(__unused struct acrn_vcpu *vcpu)
{
	return -1;
}

int32_t tpr_below_threshold_vmexit_handler(__unused struct acrn_vcpu *vcpu)
{
	return 0;
}

int32_t veoi_vmexit_handler(__unused struct acrn_vcpu *vcpu)
{
	return 0;
}

int32_t ept_violation_vmexit_handler(__unused struct acrn_vcpu *vcpu)
{
	return -1;
}

int32_t apic_write_vmexit_handler(__unused struct acrn_vcpu *vcpu)
{
	return -1;
}

void triple_fault_shutdown_vm(__unused struct acrn_vcpu *vcpu)
{
}

/*
 * @pre vlapic != NULL
 */
uint32_t
vlapic_get_apicid(const struct acrn_vlapic *vlapic)
{
	uint32_t apicid;
	if (is_x2apic_enabled(vlapic)) {
		apicid = vlapic->apic_page.id.v;
	} else {
		apicid = (vlapic->apic_page.id.v) >> APIC_ID_SHIFT;
	}

	return apicid;
}

/**
 *APIC-v: Get the HPA to APIC-access page
 * **/
uint64_t
vlapic_apicv_get_apic_access_addr(void)
{
	/*APIC-v APIC-access address */
	static uint8_t apicv_apic_access_addr[PAGE_SIZE] __aligned(PAGE_SIZE);

	return hva2hpa(apicv_apic_access_addr);
}

/**
 *APIC-v: Get the HPA to virtualized APIC registers page
 * **/
uint64_t
vlapic_apicv_get_apic_page_addr(struct acrn_vlapic *vlapic)
{
	return hva2hpa(&(vlapic->apic_page));
}

uint64_t apicv_get_pir_desc_paddr(struct acrn_vcpu *vcpu)
{
	struct acrn_vlapic *vlapic;

	vlapic = &vcpu->arch.vlapic;
	return hva2hpa(&(vlapic->pir_desc));
}

uint64_t vlapic_get_apicbase(const struct acrn_vlapic *vlapic)
{
	return vlapic->msr_apicbase;
}

int32_t vlapic_x2apic_read(__unused struct acrn_vcpu *vcpu, __unused uint32_t msr,
		__unused uint64_t *val)
{
	return 0;
}

int32_t vlapic_x2apic_write(__unused struct acrn_vcpu *vcpu, __unused uint32_t msr, __unused uint64_t val)
{
	return 0;
}

void vlapic_set_tsc_deadline_msr(__unused struct acrn_vlapic *vlapic, __unused uint64_t val_arg)
{
}

int32_t validate_pstate(__unused const struct acrn_vm *vm, __unused uint64_t perf_ctl)
{
	return 0;
}

int32_t vlapic_set_apicbase(__unused struct acrn_vlapic *vlapic, __unused uint64_t new)
{
	return 0;
}

void shutdown_vm_from_idle(uint16_t pcpu_id)
{
	struct acrn_vm *vm = get_vm_from_vmid(per_cpu(shutdown_vm_id, pcpu_id));
	const struct acrn_vcpu *vcpu = vcpu_from_vid(vm, BOOT_CPU_ID);

	if (vcpu->pcpu_id == pcpu_id) {
		(void)shutdown_vm(vm);
	}
}

/**
 * @pre vm_config != NULL
 */
uint16_t get_pcpu_id_of_vm_bsp(const struct acrn_vm_config *vm_config)
{
	uint16_t cpu_id = INVALID_CPU_ID;

	cpu_id = ffs64(vm_config->pcpu_bitmap);

	return (cpu_id < get_pcpu_nums()) ? cpu_id : INVALID_CPU_ID;
}

/**
 * @brief Initialize the I/O bitmap for \p vm
 *
 * @param vm The VM whose I/O bitmap is to be initialized
 */
static void setup_io_bitmap_stub(struct acrn_vm *vm)
{
	if (is_sos_vm(vm)) {
		(void)memset(vm->arch_vm.io_bitmap, 0x00U, PAGE_SIZE * 2U);
	} else {
		/* block all IO port access from Guest */
		(void)memset(vm->arch_vm.io_bitmap, 0xFFU, PAGE_SIZE * 2U);
	}
}
/* Add EPT mapping of EPC reource for the VM */
static void prepare_epc_vm_memmap_stub(struct acrn_vm *vm)
{
	struct epc_map* vm_epc_maps;
	uint32_t i;

	if (is_vsgx_supported(vm->vm_id)) {
		vm_epc_maps = get_epc_mapping(vm->vm_id);
		for (i = 0U; (i < MAX_EPC_SECTIONS) && (vm_epc_maps[i].size != 0UL); i++) {
			ept_add_mr(vm, (uint64_t *)vm->arch_vm.nworld_eptp, vm_epc_maps[i].hpa,
				vm_epc_maps[i].gpa, vm_epc_maps[i].size, EPT_RWX | EPT_WB);
		}
	}
}
/**
 * @param[inout] vm pointer to a vm descriptor
 *
 * @retval 0 on success
 *
 * @pre vm != NULL
 * @pre is_sos_vm(vm) == true
 */
static void prepare_sos_vm_memmap_stub(struct acrn_vm *vm)
{
	uint16_t vm_id;
	uint32_t i;
	uint64_t attr_uc = (EPT_RWX | EPT_UNCACHED);
	uint64_t hv_hpa;
	struct acrn_vm_config *vm_config;
	uint64_t *pml4_page = (uint64_t *)vm->arch_vm.nworld_eptp;
	struct epc_section* epc_secs;

	const struct e820_entry *entry;
	uint32_t entries_count = vm->e820_entry_num;
	const struct e820_entry *p_e820 = vm->e820_entries;
	const struct e820_mem_params *p_e820_mem_info = get_e820_mem_info();

	pr_dbg("sos_vm: bottom memory - 0x%llx, top memory - 0x%llx\n",
		p_e820_mem_info->mem_bottom, p_e820_mem_info->mem_top);

	if (p_e820_mem_info->mem_top > EPT_ADDRESS_SPACE(CONFIG_SOS_RAM_SIZE)) {
		panic("Please configure SOS_VM_ADDRESS_SPACE correctly!\n");
	}

	/* create real ept map for all ranges with UC */
	ept_add_mr(vm, pml4_page, p_e820_mem_info->mem_bottom, p_e820_mem_info->mem_bottom,
			(p_e820_mem_info->mem_top - p_e820_mem_info->mem_bottom), attr_uc);

	/* update ram entries to WB attr */
	for (i = 0U; i < entries_count; i++) {
		entry = p_e820 + i;
		if (entry->type == E820_TYPE_RAM) {
			ept_modify_mr(vm, pml4_page, entry->baseaddr, entry->length, EPT_WB, EPT_MT_MASK);
		}
	}

	pr_dbg("SOS_VM e820 layout:\n");
	for (i = 0U; i < entries_count; i++) {
		entry = p_e820 + i;
		pr_dbg("e820 table: %d type: 0x%x", i, entry->type);
		pr_dbg("BaseAddress: 0x%016llx length: 0x%016llx\n", entry->baseaddr, entry->length);
	}

	/* Unmap all platform EPC resource from SOS.
	 * This part has already been marked as reserved by BIOS in E820
	 * will cause EPT violation if sos accesses EPC resource.
	 */
	epc_secs = get_phys_epc();
	for (i = 0U; (i < MAX_EPC_SECTIONS) && (epc_secs[i].size != 0UL); i++) {
		ept_del_mr(vm, pml4_page, epc_secs[i].base, epc_secs[i].size);
	}

	/* unmap hypervisor itself for safety
	 * will cause EPT violation if sos accesses hv memory
	 */
	hv_hpa = hva2hpa((void *)(get_hv_image_base()));
	ept_del_mr(vm, pml4_page, hv_hpa, CONFIG_HV_RAM_SIZE);
	/* unmap prelaunch VM memory */
	for (vm_id = 0U; vm_id < CONFIG_MAX_VM_NUM; vm_id++) {
		vm_config = get_vm_config(vm_id);
		if (vm_config->load_order == PRE_LAUNCHED_VM) {
			ept_del_mr(vm, pml4_page, vm_config->memory.start_hpa, vm_config->memory.size);
		}
	}
}

/**
 * @pre vm != NULL && vm_config != NULL
 */
static void prepare_prelaunched_vm_memmap_stub(struct acrn_vm *vm, const struct acrn_vm_config *vm_config)
{
	uint64_t base_hpa = vm_config->memory.start_hpa;
	uint32_t i;

	for (i = 0U; i < vm->e820_entry_num; i++) {
		const struct e820_entry *entry = &(vm->e820_entries[i]);

		if (entry->length == 0UL) {
			break;
		}

		/* Do EPT mapping for GPAs that are backed by physical memory */
		if (entry->type == E820_TYPE_RAM) {
			ept_add_mr(vm, (uint64_t *)vm->arch_vm.nworld_eptp, base_hpa, entry->baseaddr,
				entry->length, EPT_RWX | EPT_WB);

			base_hpa += entry->length;
		}

		/* GPAs under 1MB are always backed by physical memory */
		if ((entry->type != E820_TYPE_RAM) && (entry->baseaddr < (uint64_t)MEM_1M)) {
			ept_add_mr(vm, (uint64_t *)vm->arch_vm.nworld_eptp, base_hpa, entry->baseaddr,
				entry->length, EPT_RWX | EPT_UNCACHED);

			base_hpa += entry->length;
		}
	}
}

static void filter_mem_from_sos_e820_stub(struct acrn_vm *vm, uint64_t start_pa, uint64_t end_pa)
{
	uint32_t i;
	uint64_t entry_start;
	uint64_t entry_end;
	uint32_t entries_count = vm->e820_entry_num;
	struct e820_entry *entry, new_entry = {0};

	for (i = 0U; i < entries_count; i++) {
		entry = &sos_ve820_stub[i];
		entry_start = entry->baseaddr;
		entry_end = entry->baseaddr + entry->length;

		/* No need handle in these cases*/
		if ((entry->type != E820_TYPE_RAM) || (entry_end <= start_pa) || (entry_start >= end_pa)) {
			continue;
		}

		/* filter out the specific memory and adjust length of this entry*/
		if ((entry_start < start_pa) && (entry_end <= end_pa)) {
			entry->length = start_pa - entry_start;
			continue;
		}

		/* filter out the specific memory and need to create a new entry*/
		if ((entry_start < start_pa) && (entry_end > end_pa)) {
			entry->length = start_pa - entry_start;
			new_entry.baseaddr = end_pa;
			new_entry.length = entry_end - end_pa;
			new_entry.type = E820_TYPE_RAM;
			continue;
		}

		/* This entry is within the range of specific memory
		 * change to E820_TYPE_RESERVED
		 */
		if ((entry_start >= start_pa) && (entry_end <= end_pa)) {
			entry->type = E820_TYPE_RESERVED;
			continue;
		}

		if ((entry_start >= start_pa) && (entry_start < end_pa) && (entry_end > end_pa)) {
			entry->baseaddr = end_pa;
			entry->length = entry_end - end_pa;
			continue;
		}
	}

	if (new_entry.length > 0UL) {
		entries_count++;
		ASSERT(entries_count <= E820_MAX_ENTRIES, "e820 entry overflow");
		entry = &sos_ve820_stub[entries_count - 1U];
		entry->baseaddr = new_entry.baseaddr;
		entry->length = new_entry.length;
		entry->type = new_entry.type;
		vm->e820_entry_num = entries_count;
	}

}

/**
 * before boot sos_vm(service OS), call it to hide HV and prelaunched VM memory in e820 table from sos_vm
 *
 * @pre vm != NULL
 */
static void create_sos_vm_e820_stub(struct acrn_vm *vm)
{
	uint16_t vm_id;
	uint64_t hv_start_pa = hva2hpa((void *)(get_hv_image_base()));
	uint64_t hv_end_pa  = hv_start_pa + CONFIG_HV_RAM_SIZE;
	uint32_t entries_count = get_e820_entries_count();
	const struct e820_mem_params *p_e820_mem_info = get_e820_mem_info();
	struct acrn_vm_config *sos_vm_config = get_vm_config(vm->vm_id);

	(void)memcpy_s((void *)sos_ve820_stub, entries_count * sizeof(struct e820_entry),
			(const void *)get_e820_entry(), entries_count * sizeof(struct e820_entry));

	vm->e820_entry_num = entries_count;
	vm->e820_entries = sos_ve820_stub;
	/* filter out hv memory from e820 table */
	filter_mem_from_sos_e820_stub(vm, hv_start_pa, hv_end_pa);
	sos_vm_config->memory.size = p_e820_mem_info->total_mem_size - CONFIG_HV_RAM_SIZE;

	/* filter out prelaunched vm memory from e820 table */
	for (vm_id = 0U; vm_id < CONFIG_MAX_VM_NUM; vm_id++) {
		struct acrn_vm_config *vm_config = get_vm_config(vm_id);

		if (vm_config->load_order == PRE_LAUNCHED_VM) {
			filter_mem_from_sos_e820_stub(vm, vm_config->memory.start_hpa,
					vm_config->memory.start_hpa + vm_config->memory.size);
			sos_vm_config->memory.size -= vm_config->memory.size;
		}
	}
}
/**
 * @pre vm_id < CONFIG_MAX_VM_NUM && vm_config != NULL && rtn_vm != NULL
 * @pre vm->state == VM_POWERED_OFF
 */
int32_t create_vm_stub(uint16_t vm_id, struct acrn_vm_config *vm_config, struct acrn_vm **rtn_vm)
{
	struct acrn_vm *vm = NULL;
	int32_t status = 0;
	bool need_cleanup = false;

	/* Allocate memory for virtual machine */
	vm = get_vm_from_vmid(vm_id);
	(void)memset((void *)vm, 0U, sizeof(struct acrn_vm));
	vm->vm_id = vm_id;
	vm->hw.created_vcpus = 0U;
	vm->emul_mmio_regions = 0U;

	init_ept_mem_ops(vm);
	vm->arch_vm.nworld_eptp = vm->arch_vm.ept_mem_ops.get_pml4_page(vm->arch_vm.ept_mem_ops.info);
	sanitize_pte((uint64_t *)vm->arch_vm.nworld_eptp);

	/* Register default handlers for PIO & MMIO if it is SOS VM or Pre-launched VM */
	if ((vm_config->load_order == SOS_VM) || (vm_config->load_order == PRE_LAUNCHED_VM)) {
		register_pio_default_emulation_handler(vm);
		register_mmio_default_emulation_handler(vm);
	}

	(void)memcpy_s(&vm->uuid[0], sizeof(vm->uuid),
		&vm_config->uuid[0], sizeof(vm_config->uuid));

	if (is_sos_vm(vm)) {
		/* Only for SOS_VM */
		create_sos_vm_e820_stub(vm);
		prepare_sos_vm_memmap_stub(vm);

		status = init_vm_boot_info(vm);
		if (status != 0) {
			need_cleanup = true;
		}

	} else {
		/* For PRE_LAUNCHED_VM and POST_LAUNCHED_VM */
		if ((vm_config->guest_flags & GUEST_FLAG_SECURE_WORLD_ENABLED) != 0U) {
			vm->sworld_control.flag.supported = 1U;
		}
		if (vm->sworld_control.flag.supported != 0UL) {
			struct memory_ops *ept_mem_ops = &vm->arch_vm.ept_mem_ops;

			ept_add_mr(vm, (uint64_t *)vm->arch_vm.nworld_eptp,
				hva2hpa(ept_mem_ops->get_sworld_memory_base(ept_mem_ops->info)),
				TRUSTY_EPT_REBASE_GPA, TRUSTY_RAM_SIZE, EPT_WB | EPT_RWX);
		}
		if (vm_config->name[0] == '\0') {
			/* if VM name is not configured, specify with VM ID */
			snprintf(vm_config->name, 16, "ACRN VM_%d", vm_id);
		}

		 if (vm_config->load_order == PRE_LAUNCHED_VM) {
			create_prelaunched_vm_e820(vm);
			prepare_prelaunched_vm_memmap_stub(vm, vm_config);
			status = init_vm_boot_info(vm);
		 }
	}

	if (status == 0) {
		prepare_epc_vm_memmap_stub(vm);

		INIT_LIST_HEAD(&vm->softirq_dev_entry_list);
		spinlock_init(&vm->softirq_dev_lock);
		spinlock_init(&vm->vm_lock);

		vm->arch_vm.vlapic_state = VM_VLAPIC_XAPIC;
		vm->intr_inject_delay_delta = 0UL;

		/* Set up IO bit-mask such that VM exit occurs on
		 * selected IO ranges
		 */
		setup_io_bitmap_stub(vm);

		enable_iommu();

		/* Populate return VM handle */
		*rtn_vm = vm;
		vm->sw.io_shared_page = NULL;
		if ((vm_config->load_order == POST_LAUNCHED_VM) && ((vm_config->guest_flags & GUEST_FLAG_IO_COMPLETION_POLLING) != 0U)) {
			/* enable IO completion polling mode per its guest flags in vm_config. */
			vm->sw.is_completion_polling = true;
		}
		status = set_vcpuid_entries(vm);
		if (status == 0) {
			vm->state = VM_CREATED;
		} else {
			need_cleanup = true;
		}
	}

	if (need_cleanup) {
		if (vm->arch_vm.nworld_eptp != NULL) {
			(void)memset(vm->arch_vm.nworld_eptp, 0U, PAGE_SIZE);
		}
	}
	return status;
}


/**
 * Prepare to create vm/vcpu for vm
 *
 * @pre vm_id < CONFIG_MAX_VM_NUM && vm_config != NULL
 */
void prepare_vm_stub(uint16_t vm_id, struct acrn_vm_config *vm_config)
{
	int32_t err = 0;
	uint16_t i;
	struct acrn_vm *vm = NULL;

	err = create_vm_stub(vm_id, vm_config, &vm);

	if (err == 0) {
		for (i = 0U; i < get_pcpu_nums(); i++) {
			if (bitmap_test(i, &vm_config->pcpu_bitmap)) {
				err = prepare_vcpu(vm, i);
				if (err != 0) {
					break;
				}
			}
		}
	}

	if (err == 0) {
		if (is_prelaunched_vm(vm)) {
			(void)mptable_build(vm);
		}

	}
}

/**
 * @pre vm_config != NULL
 */
void launch_vms_stub(uint16_t pcpu_id)
{
	uint16_t vm_id, bsp_id;
	struct acrn_vm_config *vm_config;

	for (vm_id = 0U; vm_id < CONFIG_MAX_VM_NUM; vm_id++) {
		vm_config = get_vm_config(vm_id);
		if ((vm_config->load_order == SOS_VM) || (vm_config->load_order == PRE_LAUNCHED_VM)) {
			if (vm_config->load_order == SOS_VM) {
			//	struct acrn_vm *p_sos_vm = get_sos_vm();
			//	sos_vm_ptr = get_vm_from_vmid(vm_id);
			}

			bsp_id = get_pcpu_id_of_vm_bsp(vm_config);
			if (pcpu_id == bsp_id) {
				prepare_vm_stub(vm_id, vm_config);
			}
		}
	}
}
/*TODO: move into guest-vcpu module */
static void enter_test_thread(uint16_t pcpu_id)
{
	vmx_on();

	(void)launch_vms_stub(pcpu_id);
	while (1) {
		test_printf("%s pcpu_id = %d\r\n", __func__, pcpu_id);
		udelay(1000*1000);
	}

}
static void init_primary_pcpu_post(void)
{
	uart_init();

	init_pcpu_post(BOOT_CPU_ID);

	disable_smep();
	disable_smap();

	enter_test_thread(BOOT_CPU_ID);
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

	enter_test_thread(pcpu_id);
}
