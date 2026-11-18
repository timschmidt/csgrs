#[path = "support/krawczyk_oracle.rs"]
mod oracle;

#[test]
fn factored_roots_and_mixed_rows_certify_the_same_original_system() {
    for shear in [1, 2, 4] {
        for gap in [0, 1, 4, 15, 255] {
            for widths in [0, 1, 2, 3, 0x04, 0x10, 0x21, 0x88, 255] {
                for mode in [8, 12, 40, 104, 136] {
                    for offset in [0, 2, 4] {
                        oracle::check([3, 251, shear, 2, offset, gap, widths, mode]);
                    }
                }
            }
        }
    }
}

#[test]
fn retained_solver_inputs_replay_under_both_policies() {
    for input in [
        include_bytes!("fixtures/krawczyk/certified-coupled.bin"),
        include_bytes!("fixtures/krawczyk/one-root.bin"),
        include_bytes!("fixtures/krawczyk/replay-cost-1.bin"),
        include_bytes!("fixtures/krawczyk/replay-cost-2.bin"),
        include_bytes!("fixtures/krawczyk/second-pivot.bin"),
        include_bytes!("fixtures/krawczyk/singular.bin"),
        include_bytes!("fixtures/krawczyk/two-roots.bin"),
    ] {
        oracle::check(*input);
    }
}
