#[path = "support/schedule_equivalence.rs"]
mod oracle;

#[test]
fn caller_caps_corner_limits_and_reachability_survive_route_reversal() {
    for orientation in 0..4 {
        for turns in 0..16 {
            for edit in [0x00, 0x15, 0x2f, 0x39] {
                let mut data = [edit, 0x27, 3, 0, 0x5d, 3, 0x6f, 1, 0];
                data[0] = (edit & !3) | orientation;
                data[5] |= (turns & 3) << 6;
                data[7] |= (turns >> 2) << 6;
                oracle::check(&data);
            }
        }
    }
}

#[test]
fn retained_equivalence_corpus() {
    let seeds: &[&[u8]] = &[
        include_bytes!("../fuzz/seeds/schedule_equivalence/interior_stop"),
        include_bytes!("../fuzz/seeds/schedule_equivalence/opposing_bounds"),
        include_bytes!("../fuzz/seeds/schedule_equivalence/radius_limited_turn"),
        include_bytes!("../fuzz/seeds/schedule_equivalence/rolling_reversal"),
        include_bytes!("../fuzz/seeds/schedule_equivalence/single_braking_span"),
        include_bytes!("../fuzz/seeds/schedule_equivalence/six_mixed_spans"),
    ];
    for seed in seeds {
        oracle::check(seed);
    }
}
