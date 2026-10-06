//! Native HyperCircuit rendition of the Easyduino Arduino Uno board.

mod easyduino_native;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    easyduino_native::run(
        "uno",
        include_str!("../tests/fixtures/easyduino/native/uno.hypercircuit.json"),
    )
}
