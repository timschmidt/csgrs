//! Allocator-free Wasm consumer exercising both implementations at runtime.
//!
//! Exports select bounded fixtures or point-valued Park inputs. Sample indices
//! use the fixture's modulo-33/modulo-five domain; action, rotation and word
//! selectors wrap rather than reject out-of-range values. This is a conformance
//! ABI: it does not export general typed FOC errors or certainty receipts.
//! Each call recomputes its result. JavaScript receives u64 returns as signed
//! BigInt values at the i64 boundary; decode endpoint halves or reinterpret bits
//! explicitly. The panic handler spins, so the host must contain failed assertions.
#![no_std]
use alumina_foc::{AlphaBeta, FocError, Q30, Q30Interval};
use interval_foc_conformance::{self as shared, samples};

#[panic_handler]
/// Ignore panic diagnostics and spin forever without allocating, unwinding or returning a status.
/// The manifest's abort setting does not replace this no_std handler with a useful error result.
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[unsafe(no_mangle)]
/// Execute one fixture action through both providers, assert full equality, and return its digest.
///
/// Reduce action modulo five in ACTIONS order and construct sample(index). Run
/// native before shared and unwrap both results. On success, hash all tagged
/// endpoint bytes from the shared result with the FNV-1a offset basis. The digest
/// is not an injective encoding or an independent arithmetic proof. An unexpected
/// provider error or mismatch enters the nonreturning panic handler.
pub extern "C" fn conformance_probe(index: u32, action: u32) -> u64 {
    let sample = samples::sample(index);
    let action = samples::ACTIONS[action as usize % samples::ACTIONS.len()];
    let native = samples::native(action, sample).unwrap();
    let shared = samples::shared(action, sample).unwrap();
    assert_eq!(shared, native);
    shared.digest(0xcbf29ce484222325)
}

#[unsafe(no_mangle)]
/// Compare point-valued Park results and report zero for success or one for Range.
///
/// alpha/beta are signed raw Q30 numerators. Reduce rotation modulo five to choose
/// the fixture's cardinal or fifths rotation. Both providers run and their complete
/// Results must agree before classification. This reports intermediate overflow too,
/// not just final representability. A mismatch or another error kind spins on panic;
/// successful endpoints and the underlying error object are not returned here.
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
///
/// Construct sample(index), reduce action modulo five, and evaluate only the native
/// provider; conformance_probe separately checks shared-provider agreement. Reduce
/// word modulo four: select a packed lane for 0..2 or tag 1/2/3 for word 3. Two-axis
/// outputs have an exact zero third lane. Sign-extend each decoded 32-bit half
/// independently; the returned u64 bits can appear as a negative BigInt in JavaScript.
///
/// Any native Err returns 0x800000007fffffff before word selection, including a tag
/// request. This fallback loses the error kind; it is not a valid ordered interval
/// or output tag. The admitted fixture corpus is expected to succeed. Constructor
/// panics still spin and are not converted to this sentinel; no result is cached.
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
