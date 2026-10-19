#[path = "../dev/complex_atanh.rs"]
mod complex_atanh;
#[path = "../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;

#[test]
fn all_scalar_pairs_consume_atanh_components_in_geodesy() {
    for first in 0..22 {
        for second in 0..22 {
            let data = [first, second, 2, 2, 0, 0];
            complex_atanh::exercise(data).unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
        }
    }
}

#[test]
fn all_reciprocal_pairs_consume_atanh_components() {
    for first in 0..22 {
        for second in 0..22 {
            let data = [first, second, 2, 2, 1, 0];
            complex_atanh::exercise(data).unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
        }
    }
}

#[test]
fn atanh_crosses_all_families_regimes_and_warm_states() {
    for family in 0..22 {
        for mode in 0..4 {
            for flags in [0, 7] {
                let data = [family, (family + 7) % 22, 127, 39, mode, flags];
                complex_atanh::exercise(data).unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
            }
        }
    }
}
