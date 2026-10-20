#[path = "../dev/elliptic.rs"]
mod elliptic;
#[path = "../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;

#[test]
fn every_scalar_family_crosses_four_elliptic_regimes_cold_and_warm() {
    for family in 0..22 {
        for mode in 0..4 {
            for flags in [0, 4] {
                let data = [family, (family + 7) % 22, 127, 39, mode, flags];
                elliptic::exercise(data).unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
            }
        }
    }
}

#[test]
fn all_484_scalar_pairs_compose_elliptic_outputs_into_geodesy() {
    for first in 0..22 {
        for second in 0..22 {
            let data = [first, second, 2, 2, 0, 0];
            elliptic::exercise(data).unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
        }
    }
}
