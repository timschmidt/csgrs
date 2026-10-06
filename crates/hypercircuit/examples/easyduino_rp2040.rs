//! Native HyperCircuit rendition of the Easyduino Raspberry Pi Pico RP2040 board.

mod easyduino_native;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    easyduino_native::run(
        "rp2040",
        include_str!("../tests/fixtures/easyduino/native/rp2040.hypercircuit.json"),
    )
}
