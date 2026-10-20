#[path = "../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;
#[path = "../dev/transverse_mercator.rs"]
mod transverse;

#[test]
fn every_scalar_certificate_crosses_each_transverse_regime_cold_and_warm() {
    for family in 0..22 {
        for mode in 0..4 {
            for flags in [0, 4] {
                let data = [family, (family + 7) % 22, 127, 39, mode, flags];
                transverse::exercise(data).unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
            }
        }
    }
}

#[test]
fn all_484_scalar_pairs_complete_fresh_transverse_inverse_and_utm_selection() {
    for first in 0..22 {
        for second in 0..22 {
            let data = [first, second, 128, 2, 1, 0];
            transverse::exercise(data).unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
        }
    }
}
