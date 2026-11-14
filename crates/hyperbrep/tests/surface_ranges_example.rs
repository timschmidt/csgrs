#![cfg(feature = "interval-ranges")]
#[path = "../examples/surface_ranges.rs"]
mod consumer;
#[test]
fn actual_native_surface_range_consumer() {
    consumer::run().unwrap();
}
