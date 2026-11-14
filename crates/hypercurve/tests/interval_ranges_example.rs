#![cfg(feature = "interval-ranges")]
#[path = "../examples/interval_ranges.rs"]
mod consumer;
#[test]
fn actual_native_curve_consumer() {
    consumer::run().unwrap();
}
