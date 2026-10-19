#[path = "../dev/inverse_legendre.rs"]
mod inverse_legendre;
#[path = "../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;

#[test]
fn inverse_integrals_cross_all_families_regimes_and_warm_states() {
    for family in 0..22 {
        for mode in 0..4 {
            for flags in [0, 5] {
                let data = [family, (family + 7) % 22, 127, 39, mode, flags];
                inverse_legendre::exercise(data)
                    .unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
            }
        }
    }
}

#[test]
fn all_scalar_pairs_produce_consumable_inverse_integrals() {
    for first in 0..22 {
        for second in 0..22 {
            let data = [first, second, 2, 2, 0, 0];
            inverse_legendre::exercise(data).unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
        }
    }
}
