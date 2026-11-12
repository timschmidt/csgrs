#[path = "../probe.rs"]
mod probe;
fn main() {
    probe::run::<true>();
}
