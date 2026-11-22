//! What every host path leaves to the model whatever its operand: the
//! operations no path covers and the model tier's arithmetic.
use crate::harness::tiers;
use stela_compiler::{
    TargetCpu,
    amx::{
        AmxOperation, AmxRegisters, Generation,
        native::{Native, Tier},
    },
};

#[test]
fn the_model_tier_and_other_operations_are_left_to_the_model() {
    let mut registers = AmxRegisters::zeroed();
    assert_eq!(Native::select(TargetCpu::Sse42), Native::MODEL);
    assert!(!Native::MODEL.execute(Generation::M2, &mut registers, AmxOperation::Fma32, 0));
    for paths in tiers() {
        let native = paths.native;
        for operation in [AmxOperation::Mac16, AmxOperation::LoadX] {
            assert!(!native.execute(Generation::M2, &mut registers, operation, 0));
        }
        // fma16 arithmetic needs F16C.
        assert_eq!(
            native.execute(Generation::M2, &mut registers, AmxOperation::Fma16, 0),
            paths.f16c
        );
        assert!(!native.execute(Generation::M2, &mut registers, AmxOperation::Fma16, 1 << 27));
        // fma16 bit 62 with bits 29:27 = 7: the Z clear Espresso runs,
        // native on every tier.
        assert!(native.execute(
            Generation::M2,
            &mut registers,
            AmxOperation::Fma16,
            0x4000_0000_3800_0000
        ));
    }
    assert_eq!(
        Native::select(TargetCpu::Sse42.with_fma().with_f16c()).tier(),
        Tier::Fma
    );
}
