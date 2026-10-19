#[path = "../dev/parameter_encodings.rs"]
mod parameter_encodings;
#[path = "../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;

#[test]
fn all_scalar_pairs_replay_bounded_parameters_into_geodesy() {
    for first in 0..22 {
        for second in 0..22 {
            for flags in [0, 7] {
                let data = [first, second, 2, 2, 0, flags];
                parameter_encodings::exercise(data)
                    .unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
            }
        }
    }
}

#[test]
fn encoded_endpoints_and_near_boundaries_retain_local_receipts() {
    for family in 0..22 {
        for mode in 0..4 {
            for flags in [0, 7] {
                for endpoint in 0..3 {
                    let data = [family, (family + 7) % 22, 127, endpoint, mode, flags];
                    parameter_encodings::exercise(data)
                        .unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
                }
            }
        }
    }
}
