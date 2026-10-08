mod instrumentation {
    // Reuse the separately qualified counting allocator without its unrelated
    // public harness. Kept as a mechanically checked prefix in the build origin.
    include!("twelfth-counting-allocator-v78.rs");
    pub fn snapshot(reset: bool) -> [usize; 4] {
        allocation::snapshot(reset)
    }
}
include!("twelfth-cost-v78.rs");
fn main() {
    run(Some(instrumentation::snapshot));
}
