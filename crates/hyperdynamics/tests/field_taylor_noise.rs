#![cfg(feature = "taylor-noise")]
#[path = "../examples/taylor_noise.rs"]
mod consumer;
#[test]
fn field_noise_observation_and_subsequent_image() {
    consumer::run().unwrap();
}
