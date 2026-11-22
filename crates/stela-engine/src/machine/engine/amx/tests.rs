use super::super::tests::{BASE, machine_on};
use super::*;
use crate::machine::system::Register as SystemRegister;
use crate::machine::{
    amx::{AMX_CTL_EL1, AMX_STATE_T, AmxView},
    exception::syndrome,
};
use stela_compiler::TargetCpu;
use stela_contracts::Chip;

fn machine(words: &[u32]) -> Machine<super::super::tests::FakeHost, super::super::NoDevices> {
    machine_on(Chip::AppleM2, words)
}

const LDX: u32 = 0x0020_1000;
const STZ: u32 = 0x0020_10a0;
const FMA32: u32 = 0x0020_1180;
const SET: u32 = 0x0020_1220;
const CLR: u32 = 0x0020_1221;
const DATA: u64 = BASE + 0x8000;

fn msr(encoding: stela_compiler::aarch64::system::registers::Encoding, rt: u32) -> u32 {
    0xd510_0000
        | u32::from(encoding.op0() & 1) << 19
        | u32::from(encoding.op1()) << 16
        | u32::from(encoding.crn()) << 12
        | u32::from(encoding.crm()) << 8
        | u32::from(encoding.op2()) << 5
        | rt
}

fn mrs(encoding: stela_compiler::aarch64::system::registers::Encoding, rt: u32) -> u32 {
    msr(encoding, rt) | 1 << 21
}

#[test]
fn disabled_amx_raises_the_implementation_defined_class() {
    for chip in [Chip::AppleM1, Chip::AppleM2, Chip::AppleM3, Chip::AppleM4] {
        let mut machine = machine_on(chip, &[]);
        machine
            .cpu
            .registers
            .set(SystemRegister::VBAR_EL1, 0, BASE + 0x4000);
        machine.amx(LDX).unwrap();
        assert_eq!(machine.cpu.read(SystemRegister::ESR_EL1), 0xfe00_0003);
        assert_eq!(
            machine.cpu.read(SystemRegister::ESR_EL1),
            syndrome(Class::ImplementationDefined, DISABLED_ISS)
        );
        assert_eq!(
            machine.cpu.read(SystemRegister::ELR_EL1),
            BASE,
            "the PC stays on the word"
        );
        assert_eq!(machine.cpu.pc, BASE + 0x4200);
        assert!(
            machine.amx(0x0020_12e0).is_err(),
            "unallocated while disabled"
        );
    }
}

#[test]
fn kernel_style_enable_set_compute_save_and_clear() {
    let mut machine = machine(&[]);
    machine.cpu.state.x[1] = 1 << 63 | 0x100;
    machine.system_instruction(msr(AMX_CTL_EL1, 1)).unwrap();
    assert!(machine.cpu.amx.enabled(AmxView::El1));
    machine.system_instruction(mrs(AMX_STATE_T, 2)).unwrap();
    assert_eq!(machine.cpu.state.x[2], 0);

    machine.amx(SET).unwrap();
    assert!(machine.cpu.amx.live());
    assert!(machine.amx(SET).is_err(), "set while live is refused");
    machine.system_instruction(mrs(AMX_STATE_T, 2)).unwrap();
    assert_eq!(machine.cpu.state.x[2], 1);

    // X0 = 1.0f32 lanes at DATA, loaded into X[0] and Y[0].
    let ones: Vec<u8> = (0..16).flat_map(|_| 1.0f32.to_le_bytes()).collect();
    assert!(machine.host.write_ram(DATA, &ones));
    machine.cpu.state.x[3] = DATA;
    machine.amx(LDX | 3).unwrap();
    machine.amx(0x0020_1020 | 3).unwrap(); // ldy
    machine.cpu.state.x[4] = 0;
    machine.amx(FMA32 | 4).unwrap();
    // stz row 0 to DATA + 0x100 crossing no page; then a store across one.
    machine.cpu.state.x[5] = DATA + 0x100;
    machine.amx(STZ | 5).unwrap();
    let mut row = [0u8; 64];
    assert!(machine.host.read_ram(DATA + 0x100, &mut row));
    assert_eq!(&row[..4], &1.0f32.to_le_bytes());
    machine.cpu.state.x[5] = DATA + 0x1000 - 32;
    machine.amx(STZ | 5).unwrap();
    assert!(machine.host.read_ram(DATA + 0x1000 - 32, &mut row));
    assert_eq!(&row[..4], &1.0f32.to_le_bytes());
    // msr, mrs, set, mrs, ldx, ldy, fma32, stz, stz.
    assert_eq!(machine.cpu.pc, BASE + 9 * 4, "each completed word advances");

    machine.amx(CLR).unwrap();
    assert!(!machine.cpu.amx.live());
    assert!(machine.amx(CLR).is_err(), "clr without live state");
    assert!(machine.amx(LDX | 3).is_err(), "no live state");
}

#[test]
fn chip_generation_drives_id_and_transfer_semantics() {
    for (chip, id, rows) in [
        (Chip::AppleM1, 1, 2),
        (Chip::AppleM2, 2, 4),
        (Chip::AppleM3, 4, 4),
        (Chip::AppleM4, 8, 4),
    ] {
        let mut machine = machine_on(chip, &[]);
        machine
            .system_instruction(mrs(crate::machine::amx::AMXIDR_EL1, 2))
            .unwrap();
        assert_eq!(machine.cpu.state.x[2], id);
        machine.cpu.state.x[1] = (1 << 63) | 0x100;
        machine.system_instruction(msr(AMX_CTL_EL1, 1)).unwrap();
        machine.amx(SET).unwrap();
        for row in 0..4 {
            assert!(
                machine
                    .host
                    .write_ram(DATA + row * 64, &[row as u8 + 1; 64])
            );
        }
        // Bit60 selects four rows on M2–M4; M3/M4 bit61 distributes
        // them every other row. All destinations wrap after X7.
        machine.cpu.state.x[3] = (1 << 62) | (1 << 61) | (1 << 60) | (7 << 56) | DATA;
        machine.amx(LDX | 3).unwrap();
        let registers = machine.cpu.amx.registers.as_ref().unwrap();
        assert_eq!(registers.x[7], [1; 64]);
        if matches!(chip, Chip::AppleM3 | Chip::AppleM4) {
            assert_eq!(registers.x[1], [2; 64]);
            assert_eq!(registers.x[3], [3; 64]);
            assert_eq!(registers.x[5], [4; 64]);
            assert_eq!(registers.x[0], [0; 64]);
        } else {
            assert_eq!(registers.x[0], [2; 64]);
            assert_eq!(registers.x[1], [if rows == 4 { 3 } else { 0 }; 64]);
            assert_eq!(registers.x[2], [if rows == 4 { 4 } else { 0 }; 64]);
        }
        machine.amx(CLR).unwrap();
        assert!(!machine.cpu.amx.live());
    }
}

#[test]
fn unqualified_generation_refuses_before_enablement_or_state_effects() {
    for chip in [Chip::AppleM5, Chip::AppleM6] {
        let mut machine = machine_on(chip, &[]);
        let ram = machine.host.ram.clone();
        let esr = machine.cpu.read(SystemRegister::ESR_EL1);
        for word in [SET, LDX, FMA32, CLR] {
            let error = machine.amx(word).unwrap_err();
            assert!(format!("{error:?}").contains(amx::UNQUALIFIED_GENERATION));
            assert_eq!(machine.cpu.pc, BASE);
            assert_eq!(machine.cpu.read(SystemRegister::ESR_EL1), esr);
            assert!(!machine.cpu.amx.live());
            assert_eq!(machine.host.ram, ram);
        }
    }
}

#[test]
fn a_vhe_host_kernel_is_governed_by_the_el2_copy() {
    let mut machine = machine(&[]);
    machine.cpu.pstate = crate::machine::exception::Pstate::reset(2);
    *machine.cpu.amx.control(AmxView::El1) = 1 << 63;
    assert!(machine.amx(SET).is_ok());
    assert!(!machine.cpu.amx.live(), "the EL1 copy does not enable");
    machine.cpu.pc = BASE;
    machine.cpu.state.x[1] = 1 << 63;
    machine.system_instruction(msr(AMX_CTL_EL1, 1)).unwrap();
    machine.amx(SET).unwrap();
    assert!(machine.cpu.amx.live());
}

#[test]
fn unallocated_operations_are_undefined_when_enabled() {
    let mut machine = machine(&[]);
    *machine.cpu.amx.control(AmxView::El1) = 1 << 63;
    machine.amx(0x0020_13ff).unwrap();
    assert_eq!(
        machine.cpu.read(SystemRegister::ESR_EL1),
        syndrome(Class::Unknown, 0)
    );
}

#[test]
fn translated_transfers_cross_pages_and_fault_without_partial_effects() {
    for chip in [Chip::AppleM1, Chip::AppleM2, Chip::AppleM3, Chip::AppleM4] {
        let mut machine = machine_on(chip, &[]);
        // 4 KiB granule, 39-bit low half: L1, L2 and L3 tables in RAM. VA
        // page 0 is writable, page 1 read-only and page 2 unmapped.
        let (l1, l2, l3) = (BASE + 0xa000, BASE + 0xb000, BASE + 0xc000);
        let page =
            |pa: u64, read_only: bool| pa | 0b11 | 1 << 10 | 1 << 2 | u64::from(read_only) << 7;
        for (slot, descriptor) in [
            (l1, l2 | 0b11),
            (l2, l3 | 0b11),
            (l3, page(BASE, false)),
            (l3 + 8, page(BASE + 0x1000, true)),
        ] {
            assert!(machine.host.write_ram(slot, &descriptor.to_le_bytes()));
        }
        for (register, value) in [
            (SystemRegister::SCTLR_EL1, mmu::SCTLR_M),
            // T0SZ 25, EPD1, 48-bit IPS.
            (SystemRegister::TCR_EL1, 25 | 1 << 23 | 5 << 32),
            (SystemRegister::TTBR0_EL1, l1),
            (SystemRegister::MAIR_EL1, 0xff00),
        ] {
            machine.cpu.registers.set(register, 0, value);
        }
        machine.cpu.registers.set(SystemRegister::VBAR_EL1, 0, 0x0);
        let bytes: Vec<u8> = (0..512).map(|i| (i * 7) as u8).collect();
        assert!(machine.host.write_ram(BASE + 0xf00, &bytes));
        *machine.cpu.amx.control(AmxView::El1) = 1 << 63;
        machine.amx(SET).unwrap();

        machine.cpu.state.x[3] = 0xfe0;
        machine.amx(LDX | 3).unwrap();
        let registers = machine.cpu.amx.registers.as_deref().unwrap();
        assert_eq!(registers.x[0][..], bytes[0xe0..0xe0 + 64]);

        // A store into page 1 is a permission fault at its first byte there.
        let mut before = [0u8; 64];
        assert!(machine.host.read_ram(BASE + 0xfe0, &mut before));
        machine.cpu.pc = 0x40;
        machine.amx(STZ | 3).unwrap();
        assert_eq!(
            machine.cpu.read(SystemRegister::ESR_EL1),
            syndrome(Class::DataAbortSame, 0b00_1111 | 1 << 6)
        );
        assert_eq!(machine.cpu.read(SystemRegister::FAR_EL1), 0x1000);
        assert_eq!(machine.cpu.read(SystemRegister::ELR_EL1), 0x40);
        let mut after = [0u8; 64];
        assert!(machine.host.read_ram(BASE + 0xfe0, &mut after));
        assert_eq!(after, before, "nothing written");

        // A load reaching page 2 is a level 3 translation fault.
        machine.cpu.state.x[3] = 0x1fe0;
        machine.amx(LDX | 3).unwrap();
        assert_eq!(
            machine.cpu.read(SystemRegister::ESR_EL1),
            syndrome(Class::DataAbortSame, 0b00_0111)
        );
        assert_eq!(machine.cpu.read(SystemRegister::FAR_EL1), 0x2000);
        let registers = machine.cpu.amx.registers.as_deref().unwrap();
        assert_eq!(registers.x[0][..], bytes[0xe0..0xe0 + 64], "unchanged");
    }
}

#[test]
fn m1_pair_alignment_reports_native_iss_and_far_before_translation() {
    for el in [0, 1] {
        for operation in [0, 1, 2, 3, 4, 5] {
            for offset in [1, 63, 64, 127] {
                let mut machine = machine_on(Chip::AppleM1, &[]);
                *machine.cpu.amx.control(AmxView::El1) = 1 << 63;
                machine.amx(SET).unwrap();
                machine.cpu.amx.registers.as_mut().unwrap().x[7] = [0x37; 64];
                machine.cpu.amx.registers.as_mut().unwrap().z[63] = [0xb1; 64];
                let before = machine.cpu.amx.registers.clone();
                let ram = machine.host.ram.clone();
                machine.cpu.pc = BASE;
                machine.cpu.pstate = crate::machine::exception::Pstate::reset(el);
                // No translation table exists at TTBR0=0: a walk would
                // fail. Alignment must win without any memory service.
                machine
                    .cpu
                    .registers
                    .set(SystemRegister::SCTLR_EL1, 0, mmu::SCTLR_M);
                machine
                    .cpu
                    .registers
                    .set(SystemRegister::TCR_EL1, 0, 25 | 5 << 32);
                machine.cpu.registers.set(SystemRegister::TTBR0_EL1, 0, 0);
                machine
                    .cpu
                    .registers
                    .set(SystemRegister::VBAR_EL1, 0, BASE + 0x4000);
                let address = 0x100_0000 | offset;
                machine.cpu.state.x[3] = (0xff << 56) | address;
                let write = matches!(operation, 2 | 3 | 5);
                machine
                    .completing(BASE, |machine| machine.amx(LDX | (operation << 5) | 3))
                    .unwrap();
                let class = if el == 0 {
                    Class::DataAbortLower
                } else {
                    Class::DataAbortSame
                };
                assert_eq!(
                    machine.cpu.read(SystemRegister::ESR_EL1),
                    syndrome(class, 0x21 | u32::from(write) << 6),
                    "ISV is zero, including the 128-byte transfer width"
                );
                assert_eq!(machine.cpu.read(SystemRegister::FAR_EL1), address);
                assert_eq!(machine.cpu.read(SystemRegister::ELR_EL1), BASE);
                assert_eq!(machine.cpu.retired, 0, "faulting word never retires");
                assert!(machine.cpu.amx.registers == before);
                assert_eq!(machine.host.ram, ram);
            }
        }
    }
}

#[test]
fn alignment_qualification_does_not_change_live_or_hacr_admission() {
    let mut machine = machine_on(Chip::AppleM1, &[]);
    machine.cpu.state.x[3] = (1 << 62) | (DATA + 64);
    machine.amx(LDX | 3).unwrap();
    assert_eq!(machine.cpu.read(SystemRegister::ESR_EL1), 0xfe00_0003);

    *machine.cpu.amx.control(AmxView::El1) = 1 << 63;
    machine.cpu.pc = BASE;
    assert!(
        machine.amx(LDX | 3).is_err(),
        "no live state remains unqualified"
    );
    machine.amx(SET).unwrap();
    machine.cpu.pc = BASE;
    let before = machine.cpu.amx.registers.clone();
    machine
        .cpu
        .registers
        .set(SystemRegister::HACR_EL2, 0, 1 << 10);
    let error = machine.amx(LDX | 3).unwrap_err();
    assert!(format!("{error:?}").contains("bit 10"));
    assert_eq!(machine.cpu.pc, BASE);
    assert!(machine.cpu.amx.registers == before);
}

#[test]
fn later_chips_keep_refusing_unqualified_pair_alignment() {
    for chip in [Chip::AppleM2, Chip::AppleM3, Chip::AppleM4] {
        let mut machine = machine_on(chip, &[]);
        *machine.cpu.amx.control(AmxView::El1) = 1 << 63;
        machine.amx(SET).unwrap();
        machine.cpu.pc = BASE;
        machine.cpu.state.x[3] = (1 << 62) | (DATA + 64);
        let before = machine.cpu.amx.registers.clone();
        let esr = machine.cpu.read(SystemRegister::ESR_EL1);
        for word in [LDX, STZ] {
            assert!(machine.amx(word | 3).is_err());
            assert_eq!(machine.cpu.pc, BASE);
            assert_eq!(machine.cpu.read(SystemRegister::ESR_EL1), esr);
            assert!(machine.cpu.amx.registers == before);
        }
    }
}

#[test]
fn transfers_outside_ram_stop_without_effects() {
    let mut machine = machine(&[]);
    *machine.cpu.amx.control(AmxView::El1) = 1 << 63;
    machine.amx(SET).unwrap();
    machine.cpu.state.x[3] = 1 << 40;
    let stop = machine.amx(LDX | 3).unwrap_err();
    assert!(format!("{stop:?}").contains("Device"), "{stop:?}");
    assert_eq!(
        machine.cpu.amx.registers.as_deref(),
        Some(&stela_compiler::amx::AmxRegisters::zeroed())
    );
}

/// The service completes covered arithmetic on the host's native tier
/// (`amx::native`) with the state the model gives on a baseline target.
#[test]
fn native_arithmetic_tier_matches_the_model_through_the_service() {
    let host = TargetCpu::detect();
    let native = Native::select(host);
    if native == Native::MODEL {
        std::eprintln!("AMX native service test skipped: host has no FMA3 tier");
        return;
    }
    let mut machines = [machine(&[]), machine(&[])];
    machines[1].target = host;
    let mut seed = 0x2545_f491_4f6c_dd1d_u64;
    let mut next = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        seed
    };
    let mut fill = stela_compiler::amx::AmxRegisters::zeroed();
    for byte in fill.as_bytes_mut() {
        *byte = next() as u8;
    }
    for machine in &mut machines {
        machine.cpu.state.x[1] = (1 << 63) | 0x100;
        machine.system_instruction(msr(AMX_CTL_EL1, 1)).unwrap();
        machine.amx(SET).unwrap();
        **machine.cpu.amx.registers.as_mut().unwrap() = fill.clone();
    }
    // fma32, fms32, fma64, fms64 with every field the native tier reads,
    // including bits 29:27 it leaves to the model, and f16 vecfp.
    for step in 0..500u64 {
        let word =
            [FMA32, FMA32 + 0x20, 0x0020_1140, 0x0020_1160, 0x0020_1260][step as usize % 5] | 6;
        let operand = if step % 5 == 4 {
            // f16 type 2, mode 0/1, broadcast Y lane, no shuffle or index.
            next() & !(0x7 << 54 | 1 << 53 | 0x3e << 47 | 0xf << 42 | 7 << 38 | 1 << 31 | 0xf << 27)
                | 2 << 42
                | (step & 8) << 35
        } else {
            next() & !(6 << 27) | (step % 3) << 28
        };
        for machine in &mut machines {
            machine.cpu.state.x[6] = operand;
            machine.amx(word).unwrap();
        }
        assert!(
            machines[0].cpu.amx.registers == machines[1].cpu.amx.registers,
            "step {step} word {word:#x} operand {operand:#x}"
        );
    }
    assert_eq!(machines[0].cpu.pc, machines[1].cpu.pc);
}

/// Native moves find the register file only where the service would run a
/// move on it: live state with AMX enabled for the governing EL. The service
/// counts the moves it completes.
#[test]
fn native_moves_find_the_file_only_where_moves_run() {
    let mut machine = machine(&[]);
    assert!(
        machine.native_system().amx,
        "an AMX chip's blocks lower moves"
    );
    assert_eq!(machine.amx_registers(), 0, "disabled, no state");
    machine.cpu.state.x[1] = 1 << 63;
    machine.system_instruction(msr(AMX_CTL_EL1, 1)).unwrap();
    assert_eq!(machine.amx_registers(), 0, "no live state");
    machine.amx(SET).unwrap();
    let file = machine.cpu.amx.registers.as_deref().unwrap() as *const _ as u64;
    assert_eq!(machine.amx_registers(), file);
    machine.enter_amx_cell();
    let cell = machine.cpu.amx.native.cell;
    assert_eq!(cell.registers, file);
    assert_eq!(cell.generation, Some(stela_compiler::amx::Generation::M2));
    assert_eq!(
        cell.routine,
        stela_compiler::x86_64::amx_routine::routine as *const () as usize as u64
    );
    assert_eq!(
        machine.anchors()[stela_compiler::Anchor::Amx],
        &machine.cpu.amx.native as *const _ as u64,
        "the anchor names the cell the engine refreshes on entry"
    );
    machine.cpu.state.x[1] = 0;
    machine.system_instruction(msr(AMX_CTL_EL1, 1)).unwrap();
    assert_eq!(machine.amx_registers(), 0, "disabled with live state");
    machine.cpu.state.x[1] = 1 << 63;
    machine.system_instruction(msr(AMX_CTL_EL1, 1)).unwrap();
    machine.cpu.state.x[3] = DATA;
    machine.amx(LDX | 3).unwrap();
    machine.amx(FMA32 | 3).unwrap();
    assert_eq!(machine.statistics.amx_moves, [0, 1]);
    assert_eq!(machine.statistics.amx_operations, [0, 1]);
    machine.cpu.amx.native.cell.moves = 3;
    machine.cpu.amx.native.cell.calls = 5;
    machine.cpu.amx.native.cell.kernels[12] = [4, 1];
    machine.leave_amx_cell();
    machine.fold_amx_kernels();
    assert_eq!(machine.statistics.amx_kernels[12], [4, 1]);
    assert_eq!(machine.cpu.amx.native.cell.kernels[12], [0, 0]);
    assert_eq!(machine.statistics.amx_moves, [3, 1]);
    assert_eq!(machine.statistics.amx_operations, [5, 1]);
    machine.amx(CLR).unwrap();
    assert_eq!(machine.amx_registers(), 0, "cleared");
    let mut other = machine_on(Chip::AppleM5, &[]);
    assert!(
        !other.native_system().amx,
        "no qualified generation, no lowered moves"
    );
    assert_eq!(other.amx_registers(), 0);
}

/// Under step timing (`--profile`) blocks call the routine that counts each
/// register operation it completes by form and path, beside the compiler's
/// own count by number, and the service counts the forms it completes;
/// folding moves the forms to the statistics, and `amx_counts` reports them
/// with what the cell holds yet. Without step timing blocks call the
/// compiler's routine itself and nothing is counted by form.
#[test]
fn profiled_register_operations_count_by_form_and_path() {
    use super::forms::{self, AmxForms};
    use stela_compiler::amx::{AmxOperation, Generation};
    const MATFP: u32 = 0x0020_12a0;
    let host = TargetCpu::detect();
    let mut machine = machine(&[]);
    machine.target = host;
    machine.cpu.state.x[1] = 1 << 63;
    machine.system_instruction(msr(AMX_CTL_EL1, 1)).unwrap();
    machine.amx(SET).unwrap();
    let compiler = stela_compiler::x86_64::amx_routine::routine as *const () as usize as u64;
    let counting = forms::routine as *const () as usize as u64;
    machine.enter_amx_cell();
    assert_eq!(machine.cpu.amx.native.cell.routine, compiler);
    machine.set_step_timing(true);
    machine.enter_amx_cell();
    assert_eq!(machine.cpu.amx.native.cell.routine, counting);
    let fused = forms::key(AmxOperation::Fma32, 0, Generation::M2).unwrap();
    let matfp = forms::key(AmxOperation::MatrixFloat, 4 << 42, Generation::M2).unwrap();
    // A matrix fma32 runs natively wherever the host has a tier; matfp on
    // the model.
    let fused_path = usize::from(Native::select(host) == Native::MODEL);
    let cell = &raw mut machine.cpu.amx.native.cell;
    for (number, operand) in [(12, 0), (12, 0), (21, 4 << 42)] {
        // SAFETY: the cell names the PE's live register file, which nothing
        // else touches during the call.
        assert_eq!(unsafe { forms::routine(cell, number, operand) }, 1);
    }
    let amx = machine.cpu.amx.native;
    let mut expected = [[0; 2]; 2];
    expected[0][fused_path] = 2;
    expected[1][1] = 1;
    assert_eq!([amx.forms.counts[fused], amx.forms.counts[matfp]], expected);
    assert_eq!(amx.cell.kernels[12][fused_path], 2, "the compiler's count");
    assert_eq!(
        amx.forms.sampled[fused][fused_path][1], 1,
        "a form's first call is timed"
    );
    let counts = machine.amx_counts();
    assert_eq!(counts.operations, [3, 0]);
    assert_eq!(counts.forms, amx.forms);
    machine.fold_amx_kernels();
    assert_eq!(machine.cpu.amx.native.forms, AmxForms::ZERO);
    assert_eq!(machine.statistics.amx_forms.counts[matfp], [0, 1]);
    assert_eq!(machine.amx_counts(), counts, "folding moves counts");
    // The service counts the register operations it completes.
    machine.cpu.state.x[3] = 4 << 42;
    machine.amx(MATFP | 3).unwrap();
    assert_eq!(machine.statistics.amx_forms.counts[matfp], [0, 2]);
    assert_eq!(machine.amx_counts().operations, [3, 1]);
    machine.set_step_timing(false);
    machine.amx(MATFP | 3).unwrap();
    assert_eq!(
        machine.statistics.amx_forms.counts[matfp],
        [0, 2],
        "only under step timing"
    );
    machine.enter_amx_cell();
    assert_eq!(machine.cpu.amx.native.cell.routine, compiler);
}

/// Register operations reach no memory.
struct NoMemory;

impl stela_compiler::amx::model::AmxMemory for NoMemory {
    fn read(
        &mut self,
        _: u64,
        _: &mut [u8],
    ) -> Result<(), stela_compiler::amx::model::MemoryFault> {
        panic!("arithmetic must not read guest memory")
    }
    fn write(&mut self, _: u64, _: &[u8]) -> Result<(), stela_compiler::amx::model::MemoryFault> {
        panic!("arithmetic must not write guest memory")
    }
}

/// With native AMX off (`--amx-native off`) blocks find a routine in their
/// cell that completes every register operation on the model alone, and
/// the service runs each on the model too, with and without step timing:
/// copies and constants (bits 29:27 of 3, 5, 6 and 7) of fma32, fma64 and
/// fma16 included, which every host path, the model tier's too, otherwise
/// completes itself. On again, the compiler's routine and the host's
/// paths, which complete those copies natively.
#[test]
fn native_amx_off_leaves_every_register_operation_to_the_model() {
    use super::forms::{self, AmxForms};
    use stela_compiler::amx::{AmxOperation, AmxRegisters, Generation, model};
    const FMA64: u32 = 0x0020_1140;
    const FMA16: u32 = 0x0020_11e0;
    const VECTOR: u64 = 1 << 63;
    let host = TargetCpu::detect();
    let native = Native::select(host);
    let compiler = stela_compiler::x86_64::amx_routine::routine as *const () as usize as u64;
    let modelled = super::routines::routine as *const () as usize as u64;
    let counting = forms::routine as *const () as usize as u64;
    let mut machine = machine(&[]);
    machine.target = host;
    machine.cpu.state.x[1] = 1 << 63;
    machine.system_instruction(msr(AMX_CTL_EL1, 1)).unwrap();
    machine.amx(SET).unwrap();
    let mut seed = 0x6a09_e667_f3bc_c908_u64;
    for byte in machine.cpu.amx.registers.as_mut().unwrap().as_bytes_mut() {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        *byte = seed as u8;
    }
    machine.enter_amx_cell();
    assert_eq!(machine.cpu.amx.native.cell.routine, compiler);
    assert_eq!(machine.cpu.amx.native.cell.native, native);
    machine.set_amx_native(false);
    // Modes 3, 5, 6 and 7, vector and matrix, with lane enables and
    // offsets, none widening.
    let operands: Vec<u64> = [3u64, 5, 6, 7]
        .into_iter()
        .flat_map(|mode| {
            [
                mode << 27 | 5 << 20 | 0x48 << 10 | 0x31,
                VECTOR | mode << 27 | 9 << 20 | 2 << 46 | 3 << 41 | 0x1c0 << 10 | 0x7,
                mode << 27 | 4 << 37 | 5 << 32 | 1 << 20,
            ]
        })
        .collect();
    let operations = [
        (FMA32, AmxOperation::Fma32),
        (FMA64, AmxOperation::Fma64),
        (FMA16, AmxOperation::Fma16),
    ];
    for timing in [false, true] {
        machine.set_step_timing(timing);
        machine.enter_amx_cell();
        let amx = machine.cpu.amx.native;
        assert_eq!(amx.cell.routine, if timing { counting } else { modelled });
        assert_eq!(amx.cell.native, Native::MODEL);
        assert_eq!(amx.model, u32::MAX);
        let service = machine.statistics.amx_operations[1];
        for (word, operation) in operations {
            let number = operation.encoding().0;
            for &operand in &operands {
                // In a block, through the routine the cell names.
                let mut expected: AmxRegisters =
                    (**machine.cpu.amx.registers.as_ref().unwrap()).clone();
                model::execute_for(
                    Generation::M2,
                    &mut expected,
                    operation,
                    operand,
                    &mut NoMemory,
                )
                .unwrap();
                let cell = &raw mut machine.cpu.amx.native.cell;
                // SAFETY: the cell names the PE's live register file, which
                // nothing else touches during the call, and is an AmxCell's.
                let done = unsafe {
                    if timing {
                        forms::routine(cell, number, operand)
                    } else {
                        super::routines::routine(cell, number, operand)
                    }
                };
                assert_eq!(done, 1);
                assert!(
                    **machine.cpu.amx.registers.as_ref().unwrap() == expected,
                    "{operation:?} operand {operand:#x} in a block"
                );
                // Through the service.
                model::execute_for(
                    Generation::M2,
                    &mut expected,
                    operation,
                    operand,
                    &mut NoMemory,
                )
                .unwrap();
                machine.cpu.state.x[3] = operand;
                machine.amx(word | 3).unwrap();
                assert!(
                    **machine.cpu.amx.registers.as_ref().unwrap() == expected,
                    "{operation:?} operand {operand:#x} in the service"
                );
            }
            let kernels = machine.cpu.amx.native.cell.kernels[number as usize];
            assert_eq!(kernels, [0, operands.len() as u64], "{operation:?}");
            if timing {
                for &operand in &operands {
                    let form = forms::key(operation, operand, Generation::M2).unwrap();
                    assert_eq!(machine.cpu.amx.native.forms.counts[form][0], 0);
                    assert_eq!(machine.statistics.amx_forms.counts[form][0], 0);
                    assert_ne!(machine.statistics.amx_forms.counts[form][1], 0);
                }
            }
            machine.fold_amx_kernels();
        }
        assert_eq!(
            machine.statistics.amx_operations[1] - service,
            (operations.len() * operands.len()) as u64,
            "the service counts what it completes"
        );
    }
    assert_eq!(machine.statistics.amx_kernels[12][0], 0);
    // On again: the compiler's routine, and a copy completes natively in a
    // block and in the service, on every host.
    machine.set_step_timing(false);
    machine.set_amx_native(true);
    machine.enter_amx_cell();
    let amx = machine.cpu.amx.native;
    assert_eq!(
        (amx.cell.routine, amx.cell.native, amx.model),
        (compiler, native, 0)
    );
    let cell = &raw mut machine.cpu.amx.native.cell;
    // SAFETY: as above.
    let done = unsafe { stela_compiler::x86_64::amx_routine::routine(cell, 12, operands[0]) };
    assert_eq!(done, 1);
    assert_eq!(machine.cpu.amx.native.cell.kernels[12], [1, 0]);
    machine.set_step_timing(true);
    machine.statistics.amx_forms = AmxForms::ZERO;
    let service = machine.statistics.amx_operations[1];
    machine.cpu.state.x[3] = operands[0];
    machine.amx(FMA32 | 3).unwrap();
    let form = forms::key(AmxOperation::Fma32, operands[0], Generation::M2).unwrap();
    assert_eq!(machine.statistics.amx_forms.counts[form], [1, 0]);
    assert_eq!(machine.statistics.amx_operations[1], service + 1);
}

/// Every PE publishes its AMX counts at each run slice: the forms only
/// under step timing, which alone counts them.
#[test]
fn published_amx_counts_hold_forms_only_under_step_timing() {
    let mut machine = machine(&[]);
    machine.statistics.amx_forms.counts[3] = [4, 5];
    machine.cpu.amx.native.forms.counts[3] = [1, 0];
    machine.statistics.amx_operations = [7, 2];
    let counts = machine.amx_counts();
    assert_eq!(counts.operations, [7, 2]);
    assert_eq!(counts.forms, super::forms::AmxForms::ZERO);
    machine.set_step_timing(true);
    assert_eq!(machine.amx_counts().forms.counts[3], [5, 5]);
}
