#[path = "../dev/jacobi_epsilon.rs"]
mod jacobi_epsilon;
#[path = "../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;

#[test]
fn all_scalar_pairs_consume_epsilon_components_in_geodesy() {
    for first in 0..22 {
        for second in 0..22 {
            let data = [first, second, 2, 2, 0, 0];
            jacobi_epsilon::exercise(data).unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
        }
    }
}

#[test]
fn all_negative_parameter_pairs_consume_epsilon_components() {
    for first in 0..22 {
        for second in 0..22 {
            let data = [first, second, 2, 2, 1, 0];
            jacobi_epsilon::exercise(data).unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
        }
    }
}

#[test]
fn epsilon_crosses_all_families_regimes_and_warm_states() {
    for family in 0..22 {
        for mode in 0..4 {
            for flags in [0, 7] {
                let data = [family, (family + 7) % 22, 127, 39, mode, flags];
                jacobi_epsilon::exercise(data)
                    .unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
            }
        }
    }
}
