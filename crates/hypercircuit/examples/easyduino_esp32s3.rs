//! Native HyperCircuit rendition of the Easyduino ESP32-S3 board.

mod easyduino_native;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    easyduino_native::run(
        "esp32s3",
        include_str!("../tests/fixtures/easyduino/native/esp32s3.hypercircuit.json"),
    )
}
