#![cfg(feature = "reachability")]

#[path = "support/reachability_policy.rs"]
mod oracle;

#[test]
fn analytic_partitions_and_prefixes_at_policy_boundaries() {
    for shape in 0..6 {
        for mode in 0..5 {
            for duration in [0, 3, 6] {
                oracle::check([252, 5, shape, 6 - shape, shape, 254, duration, mode]);
            }
        }
    }
}

#[test]
fn affine_time_fields_reach_the_exact_refinement_threshold() {
    for levels in 0..3 {
        for slope in 0..6 {
            for mode in [128, 129] {
                oracle::check([250, 1, slope, slope, levels, 254, slope, mode]);
            }
        }
    }
}
