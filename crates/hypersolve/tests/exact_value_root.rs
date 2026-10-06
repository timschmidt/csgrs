use hyperreal::{Real, ZeroKnowledge};
use hypersolve::{
    AlgebraicRootRepresentation, AlgebraicRootValidationStatus, PredicatePolicy,
    validate_algebraic_root_representation,
};

#[test]
fn exact_values_retain_small_polynomials_and_the_selected_quadratic_sheet() {
    let mixed = Real::from(2).sqrt().unwrap() + Real::from(3).sqrt().unwrap();
    let radical = ((&mixed * &mixed - Real::from(5)) / Real::from(2)).unwrap();
    for sign in [-1, 1] {
        let value = Real::from(7) + Real::from(sign) * &radical;
        let root = AlgebraicRootRepresentation::from_exact_value(&value);
        assert_eq!(
            root.polynomial_coefficients,
            vec![Real::from(43), Real::from(-14), Real::one()],
        );
        assert_eq!(
            (root.exact_point_witness().unwrap() - &value).zero_status(),
            ZeroKnowledge::Zero,
        );
        assert_eq!(
            validate_algebraic_root_representation(&root, PredicatePolicy::STRICT).status,
            AlgebraicRootValidationStatus::Valid,
        );
        let mut wrong_sheet = root.clone();
        wrong_sheet.interval.exact_root = Some(Real::from(14) - root.interval.lower);
        assert_eq!(
            validate_algebraic_root_representation(&wrong_sheet, PredicatePolicy::STRICT).status,
            AlgebraicRootValidationStatus::WitnessOutsideInterval,
        );
    }
}

#[test]
fn exact_value_roots_keep_the_general_coefficient_field() {
    for value in [Real::from(-3), Real::pi(), Real::e().sin()] {
        let root = AlgebraicRootRepresentation::from_exact_value(&value);
        assert_eq!(
            root.polynomial_coefficients,
            vec![-value.clone(), Real::one()]
        );
        assert_eq!(root.exact_point_witness(), Some(&value));
        assert_eq!(
            validate_algebraic_root_representation(&root, PredicatePolicy::STRICT).status,
            AlgebraicRootValidationStatus::Valid,
        );
    }
}
