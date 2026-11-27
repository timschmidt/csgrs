//! Native Cartesian-storage version of the shared command-line chain probe.
#[path = "../probe.rs"]
mod probe;
/// Select native storage at compile time and delegate seed parsing and diagnostic output.
fn main() {
    probe::run::<false>();
}
