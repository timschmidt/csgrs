#[path = "../dev/complex_jacobi.rs"]
mod complex_jacobi;
#[path = "../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;

#[test]
fn complex_jacobi_crosses_all_families_regimes_and_warm_states() {
    for family in 0..22 {
        for mode in 0..4 {
            for flags in [0, 5] {
                let data = [family, (family + 7) % 22, 127, 39, mode, flags];
                complex_jacobi::exercise(data)
                    .unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
            }
        }
    }
}

#[test]
fn all_scalar_pairs_produce_consumable_complex_jacobi_values() {
    for first in 0..22 {
        for second in 0..22 {
            let data = [first, second, 2, 2, 0, 0];
            complex_jacobi::exercise(data).unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
        }
    }
}

#[test]
fn all_scalar_pairs_produce_consumable_negative_parameter_values() {
    for first in 0..22 {
        for second in 0..22 {
            let data = [first, second, 2, 2, 1, 0];
            complex_jacobi::exercise(data).unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
        }
    }
}

#[test]
fn const_product_sinpi_pair_remains_consumable() {
    complex_jacobi::exercise([7, 19, 2, 2, 0, 0]).unwrap();
}

#[test]
fn sanitizer_timeout_and_slow_unit_remain_consumable() {
    for data in [
        [21, 6, 127, 39, 1, 4],
        [21, 6, 0, 0, 1, 4], // The period bytes are unused in this regime.
        [12, 12, 66, 201, 7, 1],
    ] {
        complex_jacobi::exercise(data).unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
    }
}

#[test]
fn directed_negative_quadratic_parameter_remains_consumable() {
    for data in [[0, 120, 113, 108, 1, 0], [0, 10, 0, 0, 1, 0]] {
        complex_jacobi::exercise(data).unwrap_or_else(|error| panic!("{data:?}: {error:?}"));
    }
}
