#![cfg(all(feature = "flow-noise", feature = "dispatch-trace"))]
#[path = "../examples/flow_noise.rs"]
mod shipped_example;

#[test]
fn manual_affine_noise_chain_preserves_numeric_dispatch_counts() {
    use hyperreal::dispatch_trace;
    dispatch_trace::reset();
    dispatch_trace::with_recording(|| shipped_example::run().unwrap());
    let actual = format!("{:?}", dispatch_trace::take_trace());
    println!("MANUAL_TRACE={actual}");
    assert_eq!(actual, include_str!("fixtures/flow_noise_trace.txt").trim());
}
