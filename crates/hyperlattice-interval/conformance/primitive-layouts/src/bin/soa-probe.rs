//! Separate-endpoint-array version of the shared command-line chain probe.
#[path = "../probe.rs"]
mod probe;
/// Select SoA storage at compile time and delegate seed parsing and diagnostic output.
fn main() {
    probe::run::<true>();
}
