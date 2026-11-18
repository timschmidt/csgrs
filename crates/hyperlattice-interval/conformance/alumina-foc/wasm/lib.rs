//! Allocator-free Wasm consumer exercising both implementations at runtime.
#![no_std]
use alumina_foc::{AlphaBeta, FocError, Q30, Q30Interval};
use interval_foc_conformance::{self as shared, samples};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn conformance_probe(index: u32, action: u32) -> u64 {
    let sample = samples::sample(index);
    let action = samples::ACTIONS[action as usize % samples::ACTIONS.len()];
    let native = samples::native(action, sample).unwrap();
    let shared = samples::shared(action, sample).unwrap();
    assert_eq!(shared, native);
    shared.digest(0xcbf29ce484222325)
}

#[unsafe(no_mangle)]
pub extern "C" fn range_probe(alpha: i32, beta: i32, rotation: u32) -> i32 {
    let value = AlphaBeta {
        alpha: Q30Interval::point(Q30::from_bits(alpha)),
        beta: Q30Interval::point(Q30::from_bits(beta)),
    };
    let r = samples::sample(rotation % 5).rotation;
    let native = alumina_foc::park(value, r);
    let shared = shared::park(value, r);
    assert_eq!(shared, native);
    match shared {
        Ok(_) => 0,
        Err(FocError::Range) => 1,
        Err(_) => panic!("unexpected error"),
    }
}

/// Words 0 through 2 contain lower and upper signed endpoint bits, lower
/// first in the low half. Word 3 is the output tag. The selector repeats.
/// An unexpected firmware error returns the reversed pair (i32::MAX, i32::MIN).
#[unsafe(no_mangle)]
pub extern "C" fn output_word(index: u32, action: u32, word: u32) -> u64 {
    let sample = samples::sample(index);
    let action = samples::ACTIONS[action as usize % samples::ACTIONS.len()];
    let Ok(output) = samples::native(action, sample) else {
        return (u64::from(i32::MIN as u32) << 32) | u64::from(i32::MAX as u32);
    };
    let (tag, lanes) = output.tagged_lanes();
    match lanes.get((word % 4) as usize) {
        Some(lane) => {
            u64::from(lane.lower().bits() as u32) | (u64::from(lane.upper().bits() as u32) << 32)
        }
        None => u64::from(tag),
    }
}
