//! Allocating dyadic bridge consumer; the scalar float-only module stays no_std.
#[path = "../../../tests/support/dyadic_egress_cases.rs"]
mod cases;

/// Run independent exact bit expectations; any mismatch traps the WASM instance.
#[unsafe(no_mangle)]
pub extern "C" fn verify_dyadic_egress() -> u32 {
    cases::verify()
}
