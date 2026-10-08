use hyperreal::Real;
use hypersolve::{IsolatedRootInterval, sign_at_selected_root};
fn main() {
    for (label, coefficient, intercept) in [("pi", Real::pi as fn() -> Real, -6), ("exp", || (Real::one() / Real::from(3)).unwrap().exp().unwrap(), -2)] {
    for precision in [-64, -600, -1200] {
        let root = coefficient();
        let [lower, upper] = root.certified_dyadic_interval(precision).unwrap();
        let interval = IsolatedRootInterval { lower: Real::new(lower), upper: Real::new(upper), exact_root: None, distinct_root_count: 1 };
        let defining = [-coefficient(), Real::one()];
        let query = [Real::from(intercept), Real::from(2)];
        println!("kind={label} precision={precision} sign={:?}", sign_at_selected_root(&defining, &query, &interval));
    }
}
}
