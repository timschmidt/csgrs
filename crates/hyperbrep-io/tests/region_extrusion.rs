#[path = "support/region_extrusion.rs"]
mod acceptance;

use hypercurve::OffsetCap;

#[test]
fn butt_region_completes_exact_step_and_native_continuation() {
    acceptance::round_trip(OffsetCap::Butt, false).unwrap();
}

#[test]
fn reversed_butt_region_completes_exact_step_and_native_continuation() {
    acceptance::round_trip(OffsetCap::Butt, true).unwrap();
}

#[test]
fn round_region_completes_exact_step_and_native_continuation() {
    acceptance::round_trip(OffsetCap::Round, false).unwrap();
}

#[test]
fn reversed_round_region_completes_exact_step_and_native_continuation() {
    acceptance::round_trip(OffsetCap::Round, true).unwrap();
}
