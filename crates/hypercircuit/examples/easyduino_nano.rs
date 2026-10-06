//! Native HyperCircuit rendition of the Easyduino Arduino Nano board.

mod easyduino_native;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    easyduino_native::run(
        "nano",
        include_str!("../tests/fixtures/easyduino/native/nano.hypercircuit.json"),
    )
}
