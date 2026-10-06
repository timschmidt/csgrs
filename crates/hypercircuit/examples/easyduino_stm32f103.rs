//! Native HyperCircuit rendition of the Easyduino STM32F103 Blue Pill board.

mod easyduino_native;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    easyduino_native::run(
        "stm32f103",
        include_str!("../tests/fixtures/easyduino/native/stm32f103.hypercircuit.json"),
    )
}
