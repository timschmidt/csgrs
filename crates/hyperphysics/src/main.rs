fn main() {
    // Structural note: future physics kernels should carry body/shape metadata
    // such as convexity, axis alignment, transform kind, and mass-matrix shape
    // so adapters can choose faster exact setup or explicit lossy exports
    // without adding primitive-float predicates to the core hyper stack.
    println!("Hello, world!");
}
