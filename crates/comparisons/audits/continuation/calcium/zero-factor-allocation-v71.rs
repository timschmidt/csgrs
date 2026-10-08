#[allow(dead_code)]
mod instrumentation {
    include!("power-sums-allocation.rs");
    pub fn snapshot(reset: bool) -> [usize; 4] {
        allocation::snapshot(reset)
    }
}
include!("zero-factor-cost-v71.rs");
fn main() {
    benchmark::run_cost(Some(instrumentation::snapshot));
}
