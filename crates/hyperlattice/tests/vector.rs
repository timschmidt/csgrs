mod common;

use common::{abort_signal, frac, r, unknown_zero};
use hyperlattice::{
    Axis2, Problem, RationalStorageClass, RealExactSetDenominatorKind,
    RealExactSetDyadicExponentClass, RealExactSetSignPattern, RealSymbolicDependencyMask,
    SharedScaleVec, SignedAxis2, Vector2, Vector3, Vector4, Vector4HomogeneousKind, ZeroStatus,
    one, pi, zero,
};

#[test]
fn vector_dot_and_normalize() {
    let v = Vector3::new([r(3), r(4), r(0)]);
    assert_eq!(v.dot(&v), r(25));

    let normalized = v.normalize().unwrap();
    assert_eq!(normalized.dot(&normalized), one());

    let signal = abort_signal();
    assert_eq!(v.dot_with_abort(&v, &signal), r(25));
    assert_eq!(v.magnitude_with_abort(&signal).unwrap(), r(5));
    assert_eq!(
        v.normalize_checked_with_abort(&signal)
            .unwrap()
            .dot(&normalized),
        one()
    );
}

#[test]
fn vector2_structural_facts_expose_zero_mask_without_topology() {
    let x_axis = Vector2::new([r(5), r(0)]);
    let zero_vector: Vector2 = Vector2::zero();

    let x_axis_facts = x_axis.structural_facts();
    assert_eq!(
        x_axis_facts.component_zero,
        [ZeroStatus::NonZero, ZeroStatus::Zero]
    );
    assert_eq!(Axis2::X.index(), 0);
    assert_eq!(Axis2::Y.bit(), 0b10);
    assert_eq!(x_axis_facts.component_zero(Axis2::X), ZeroStatus::NonZero);
    assert_eq!(x_axis_facts.known_axis, Some(Axis2::X));
    assert_eq!(x_axis_facts.known_signed_axis, None);
    assert!(!x_axis_facts.is_signed_unit_axis());
    assert_eq!(x_axis_facts.known_zero_mask(), 0b10);
    assert_eq!(x_axis_facts.known_nonzero_mask(), 0b01);
    assert_eq!(x_axis_facts.unknown_zero_mask(), 0);
    assert_eq!(x_axis_facts.known_zero_count(), 1);
    assert_eq!(x_axis_facts.known_nonzero_count(), 1);
    assert_eq!(x_axis_facts.unknown_zero_count(), 0);
    assert!(x_axis_facts.symbolic_dependencies.is_empty());
    assert_eq!(x_axis_facts.squared_norm_zero_status(), ZeroStatus::NonZero);
    assert!(!x_axis_facts.has_unknown_zero());
    assert!(!x_axis_facts.known_zero);
    assert!(x_axis_facts.exact.is_nonempty_exact_rational());
    assert!(x_axis_facts.exact.has_dyadic_schedule());

    let zero_facts = zero_vector.structural_facts();
    assert_eq!(
        zero_facts.component_zero,
        [ZeroStatus::Zero, ZeroStatus::Zero]
    );
    assert_eq!(zero_facts.known_axis, None);
    assert_eq!(zero_facts.known_signed_axis, None);
    assert_eq!(zero_facts.known_zero_mask(), 0b11);
    assert_eq!(zero_facts.known_nonzero_mask(), 0);
    assert_eq!(zero_facts.known_zero_count(), 2);
    assert_eq!(zero_facts.squared_norm_zero_status(), ZeroStatus::Zero);
    assert!(zero_facts.known_zero);
}

#[test]
fn vector_structural_facts_summarize_symbolic_dependencies() {
    let vector2 = Vector2::new([pi(), frac(1, 5) * pi().sin()]);
    let facts2 = vector2.structural_facts();
    assert!(
        facts2
            .symbolic_dependencies
            .contains(RealSymbolicDependencyMask::PI)
    );
    assert!(
        !facts2
            .symbolic_dependencies
            .contains(RealSymbolicDependencyMask::LOG)
    );

    let log_two = r(2).ln().unwrap();
    let vector3 = Vector3::new([log_two, zero(), r(7)]);
    let facts3 = vector3.structural_facts();
    assert!(
        facts3
            .symbolic_dependencies
            .contains(RealSymbolicDependencyMask::LOG)
    );
    assert!(
        !facts3
            .symbolic_dependencies
            .contains(RealSymbolicDependencyMask::TRIG)
    );

    let trig = (frac(1, 5) * pi()).sin();
    let vector4 = Vector4::new([trig, zero(), r(1), zero()]);
    let facts4 = vector4.structural_facts();
    assert!(
        facts4
            .symbolic_dependencies
            .contains(RealSymbolicDependencyMask::TRIG)
    );
    assert!(
        facts4
            .symbolic_dependencies
            .contains(RealSymbolicDependencyMask::PI)
    );
}

#[test]
fn vector_facts_classify_squared_norm_zero_status_without_self_dot() {
    let vector3 = Vector3::new([unknown_zero(), r(4), r(0)]);
    let vector3_facts = vector3.structural_facts();
    assert_eq!(vector3_facts.known_nonzero_count(), 1);
    assert_eq!(vector3_facts.unknown_zero_count(), 1);
    assert_eq!(
        vector3_facts.squared_norm_zero_status(),
        ZeroStatus::NonZero
    );
    assert!(vector3.normalize_checked().is_ok());

    let unknown_vector = Vector4::new([unknown_zero(), r(0), r(0), r(0)]);
    let unknown_facts = unknown_vector.structural_facts();
    assert_eq!(
        unknown_facts.squared_norm_zero_status(),
        ZeroStatus::Unknown
    );
    assert_eq!(
        unknown_vector.normalize_checked(),
        Err(Problem::UnknownZero)
    );

    let zero_vector = Vector4::zero();
    assert_eq!(
        zero_vector.structural_facts().squared_norm_zero_status(),
        ZeroStatus::Zero
    );
    assert_eq!(zero_vector.normalize_checked(), Err(Problem::DivideByZero));
}

#[test]
fn vector2_structural_facts_certify_signed_unit_axes() {
    let pos_x = Vector2::new([one(), zero()]);
    let neg_y = Vector2::new([zero(), -one()]);
    let scaled_axis = Vector2::new([r(5), zero()]);

    let pos_x_facts = pos_x.structural_facts();
    assert_eq!(pos_x_facts.known_axis, Some(Axis2::X));
    assert_eq!(pos_x_facts.known_signed_axis, Some(SignedAxis2::PosX));
    assert!(pos_x_facts.is_signed_unit_axis());
    assert_eq!(pos_x_facts.known_signed_axis.unwrap().axis(), Axis2::X);
    assert!(!pos_x_facts.known_signed_axis.unwrap().is_negative());
    assert_eq!(pos_x_facts.known_signed_axis.unwrap().sign_real(), one());

    let neg_y_facts = neg_y.structural_facts();
    assert_eq!(neg_y_facts.known_axis, Some(Axis2::Y));
    assert_eq!(neg_y_facts.known_signed_axis, Some(SignedAxis2::NegY));
    assert_eq!(neg_y_facts.known_signed_axis.unwrap().axis(), Axis2::Y);
    assert!(neg_y_facts.known_signed_axis.unwrap().is_negative());
    assert_eq!(neg_y_facts.known_signed_axis.unwrap().sign_real(), -one());

    let scaled_facts = scaled_axis.structural_facts();
    assert_eq!(scaled_facts.known_axis, Some(Axis2::X));
    assert_eq!(scaled_facts.known_signed_axis, None);
}

#[test]
fn vector_exact_facts_carry_common_scale_without_denominator_access() {
    let vector2 = Vector2::new([frac(1, 3), frac(2, 3)]);
    let vector2_facts = vector2.structural_facts().exact;
    assert!(vector2_facts.is_nonempty_exact_rational());
    assert!(!vector2_facts.has_dyadic_schedule());
    assert!(vector2_facts.has_shared_denominator_schedule());
    assert!(!vector2_facts.has_integer_grid_schedule());
    assert_eq!(vector2_facts.known_positive_count, 2);
    assert_eq!(
        vector2_facts.sign_pattern(),
        RealExactSetSignPattern::AllPositive
    );

    let vector3 = Vector3::new([frac(1, 4), frac(3, 4), frac(-5, 4)]);
    let vector3_facts = vector3.exact_facts();
    assert_eq!(vector3_facts.len, 3);
    assert!(vector3_facts.has_dyadic_schedule());
    assert!(vector3_facts.has_shared_denominator_schedule());
    assert_eq!(vector3_facts.exact_integer_count, 0);
    assert_eq!(
        vector3_facts.sign_pattern(),
        RealExactSetSignPattern::MixedKnown
    );

    let symbolic = Vector4::new([unknown_zero(), r(1), r(2), r(3)]);
    let symbolic_facts = symbolic.exact_facts();
    assert_eq!(symbolic_facts.exact_rational_count, 3);
    assert_eq!(symbolic_facts.exact_integer_count, 3);
    assert_eq!(symbolic_facts.unknown_zero_count, 1);
    assert_eq!(
        symbolic_facts.sign_pattern(),
        RealExactSetSignPattern::Unknown
    );
    assert!(!symbolic_facts.all_exact_rational);
    assert!(!symbolic_facts.has_shared_denominator_schedule());
}

#[test]
fn vector3_and_vector4_structural_facts_expose_sparse_masks_and_homogeneous_kind() {
    let vector3 = Vector3::new([zero(), r(5), zero()]);
    let facts3 = vector3.structural_facts();
    assert_eq!(
        facts3.component_zero,
        [ZeroStatus::Zero, ZeroStatus::NonZero, ZeroStatus::Zero]
    );
    assert_eq!(facts3.known_zero_mask, 0b101);
    assert_eq!(facts3.known_nonzero_mask, 0b010);
    assert_eq!(facts3.unknown_zero_mask, 0);
    assert_eq!(facts3.one_mask, 0);
    assert_eq!(facts3.known_axis_index, Some(1));
    assert_eq!(facts3.known_zero_count(), 2);
    assert_eq!(facts3.known_nonzero_count(), 1);
    assert_eq!(facts3.unknown_zero_count(), 0);
    assert!(!facts3.known_zero);
    assert!(facts3.exact.has_integer_grid_schedule());

    let point = Vector4::new([r(3), zero(), r(4), one()]);
    let point_facts = point.structural_facts();
    assert_eq!(point_facts.homogeneous, Vector4HomogeneousKind::Point);
    assert_eq!(point_facts.known_zero_mask, 0b0010);
    assert_eq!(point_facts.one_mask, 0b1000);
    assert_eq!(point_facts.known_axis_index, None);
    assert_eq!(point_facts.known_zero_count(), 1);

    let direction = Vector4::new([zero(), one(), zero(), zero()]);
    let direction_facts = direction.structural_facts();
    assert_eq!(
        direction_facts.homogeneous,
        Vector4HomogeneousKind::Direction
    );
    assert_eq!(direction_facts.known_axis_index, Some(1));
    assert_eq!(direction_facts.known_zero_mask, 0b1101);
    assert_eq!(direction_facts.one_mask, 0b0010);

    let negative_weight = Vector4::new([zero(), zero(), zero(), -one()]);
    let negative_facts = negative_weight.structural_facts();
    assert_eq!(negative_facts.homogeneous, Vector4HomogeneousKind::Unknown);
    assert_eq!(negative_facts.known_axis_index, Some(3));
    assert_eq!(negative_facts.one_mask, 0);
}

#[test]
fn vector_shared_scale_views_preserve_borrowed_common_scale_facts() {
    let vector2 = Vector2::new([frac(1, 3), frac(2, 3)]);
    let view2 = vector2
        .shared_scale_view()
        .expect("thirds share a denominator");
    assert_eq!(view2.len(), 2);
    assert!(!view2.is_empty());
    assert_eq!(view2.exact.len, 2);
    assert!(view2.exact.has_shared_denominator_schedule());
    assert_eq!(
        view2.exact.shared_denominator_kind(),
        Some(RealExactSetDenominatorKind::SharedNonDyadic)
    );
    assert_eq!(
        view2.exact.max_rational_storage,
        Some(RationalStorageClass::WordSized)
    );
    assert_eq!(view2.exact.max_dyadic_exponent_class, None);
    assert!(!view2.exact.has_integer_grid_schedule());
    assert_eq!(
        view2.exact.sign_pattern(),
        RealExactSetSignPattern::AllPositive
    );
    assert_eq!(view2.known_zero_mask, 0);
    assert_eq!(view2.known_nonzero_mask, 0b11);
    assert_eq!(view2.unknown_zero_mask, 0);
    assert_eq!(view2.known_zero_count(), 0);
    assert_eq!(view2.known_nonzero_count(), 2);
    assert_eq!(view2.unknown_zero_count(), 0);
    assert_eq!(view2.components()[0], &frac(1, 3));

    let zero_vector = Vector2::zero();
    let zero_view = zero_vector
        .shared_scale_view()
        .expect("zero coordinates share the integer denominator");
    assert_eq!(zero_view.known_zero_mask, 0b11);
    assert_eq!(zero_view.known_nonzero_mask, 0);
    assert_eq!(zero_view.known_zero_count(), 2);
    assert_eq!(zero_view.known_nonzero_count(), 0);
    assert!(zero_view.exact.has_integer_grid_schedule());
    assert!(zero_view.exact.has_signed_unit_schedule());
    assert_eq!(
        zero_view.exact.sign_pattern(),
        RealExactSetSignPattern::AllZero
    );
    assert!(zero_view.is_known_zero());

    let vector3 = Vector3::new([frac(1, 5), frac(-2, 5), frac(3, 5)]);
    let view3 = vector3
        .shared_scale_view()
        .expect("fifths share a denominator");
    assert_eq!(view3.known_zero_mask, 0);
    assert_eq!(view3.known_nonzero_mask, 0b111);
    assert_eq!(view3.known_zero_count(), 0);
    assert_eq!(view3.known_nonzero_count(), 3);
    assert_eq!(view3.exact.known_negative_count, 1);
    assert_eq!(
        view3.exact.sign_pattern(),
        RealExactSetSignPattern::MixedKnown
    );
    assert!(view3.is_known_dense());

    let dyadic = Vector3::new([frac(1, 4), frac(-3, 4), frac(5, 4)]);
    let dyadic_view = dyadic
        .shared_scale_view()
        .expect("quarters share a dyadic denominator");
    assert_eq!(
        dyadic_view.exact.max_dyadic_exponent_class,
        Some(RealExactSetDyadicExponentClass::Small)
    );

    let vector4 = Vector4::new([unknown_zero(), frac(1, 7), frac(2, 7), frac(3, 7)]);
    assert!(vector4.shared_scale_view().is_none());

    let mixed_denominators = Vector3::new([frac(1, 2), frac(1, 3), frac(1, 6)]);
    assert!(mixed_denominators.shared_scale_view().is_none());
}

#[test]
fn owned_shared_scale_vectors_preserve_common_scale_across_lifetimes() {
    let owned = SharedScaleVec::from_components([frac(1, 9), frac(-2, 9), frac(4, 9)])
        .expect("ninths share a reduced denominator");
    assert_eq!(owned.len(), 3);
    assert!(!owned.is_empty());
    assert_eq!(owned.known_zero_count(), 0);
    assert_eq!(owned.known_nonzero_count(), 3);
    assert_eq!(owned.unknown_zero_count(), 0);
    assert!(owned.exact.has_shared_denominator_schedule());
    assert!(!owned.exact.has_integer_grid_schedule());
    assert_eq!(
        owned.exact.shared_denominator_kind(),
        Some(RealExactSetDenominatorKind::SharedNonDyadic)
    );
    assert_eq!(
        owned.exact.sign_pattern(),
        RealExactSetSignPattern::MixedKnown
    );
    assert_eq!(owned.components()[0], frac(1, 9));

    let view = owned.as_view();
    assert_eq!(view.known_zero_mask, 0);
    assert_eq!(view.known_nonzero_mask, 0b111);
    assert_eq!(view.components()[1], &frac(-2, 9));

    let from_vector = Vector3::new([frac(1, 4), frac(3, 4), frac(-5, 4)])
        .into_shared_scale()
        .expect("quarters share a reduced denominator");
    assert!(from_vector.exact.has_dyadic_schedule());
    assert_eq!(
        from_vector.exact.max_dyadic_exponent_class,
        Some(RealExactSetDyadicExponentClass::Small)
    );
    assert_eq!(
        from_vector.into_components(),
        [frac(1, 4), frac(3, 4), frac(-5, 4)]
    );

    let zero_units = Vector4::new([zero(), one(), -one(), zero()])
        .into_shared_scale()
        .expect("signed unit coordinates are integer-grid shared scale");
    assert!(zero_units.exact.has_integer_grid_schedule());
    assert!(zero_units.exact.has_signed_unit_schedule());

    assert!(SharedScaleVec::from_components([frac(1, 2), frac(1, 3)]).is_none());
    assert!(
        Vector4::new([unknown_zero(), frac(1, 7), frac(2, 7), frac(3, 7)])
            .into_shared_scale()
            .is_none()
    );
}

#[test]
fn shared_scale_vectors_use_known_exact_dot_products() {
    let left = SharedScaleVec::from_components([frac(1, 6), frac(-5, 6), frac(5, 6)])
        .expect("sixths share a reduced denominator");
    let right = SharedScaleVec::from_components([frac(7, 10), frac(3, 10), frac(-3, 10)])
        .expect("tenths share a reduced denominator");

    let expected =
        Vector3::new(left.components().clone()).dot(&Vector3::new(right.components().clone()));
    assert_eq!(left.dot(&right), expected);
    assert_eq!(left.as_view().dot(right.as_view()), -frac(23, 60));
    assert_eq!(left.squared_norm(), frac(51, 36));
    assert_eq!(left.as_view().squared_norm(), frac(51, 36));

    let zeros = SharedScaleVec::from_components([zero(), zero()]).expect("zero grid is shared");
    assert_eq!(zeros.dot(&zeros), zero());
    assert!(zeros.exact.has_signed_unit_schedule());

    let dyadic_left =
        SharedScaleVec::from_components([frac(1, 8), frac(3, 8), frac(-5, 8), frac(7, 8)])
            .expect("eighths share a dyadic denominator");
    let dyadic_right =
        SharedScaleVec::from_components([frac(3, 8), frac(-5, 8), frac(7, 8), frac(-1, 8)])
            .expect("eighths share a dyadic denominator");
    assert!(dyadic_left.exact.has_dyadic_schedule());
    assert_eq!(
        dyadic_left.dot(&dyadic_right),
        Vector4::new(dyadic_left.components().clone())
            .dot(&Vector4::new(dyadic_right.components().clone()))
    );
}

#[test]
fn shared_scale_vectors_use_known_exact_wedge_products() {
    let left = SharedScaleVec::from_components([frac(2, 9), frac(5, 9)])
        .expect("ninths share a reduced denominator");
    let right = SharedScaleVec::from_components([frac(-4, 15), frac(7, 15)])
        .expect("fifteenths share a reduced denominator");

    let expected =
        Vector2::new(left.components().clone()).wedge(&Vector2::new(right.components().clone()));
    assert_eq!(left.wedge(&right), expected);
    assert_eq!(left.as_view().wedge(right.as_view()), frac(34, 135));

    let parallel_left = SharedScaleVec::from_components([frac(3, 11), frac(6, 11)])
        .expect("elevenths share a reduced denominator");
    let parallel_right = SharedScaleVec::from_components([frac(-5, 13), frac(-10, 13)])
        .expect("thirteenths share a reduced denominator");
    assert_eq!(parallel_left.wedge(&parallel_right), zero());

    let dyadic_left = SharedScaleVec::from_components([frac(3, 8), frac(-5, 8)])
        .expect("eighths share a dyadic denominator");
    let dyadic_right = SharedScaleVec::from_components([frac(7, 8), frac(1, 8)])
        .expect("eighths share a dyadic denominator");
    assert!(dyadic_left.exact.has_dyadic_schedule());
    assert_eq!(
        dyadic_left.wedge(&dyadic_right),
        Vector2::new(dyadic_left.components().clone())
            .wedge(&Vector2::new(dyadic_right.components().clone()))
    );
}

#[test]
fn shared_scale_vectors_use_known_exact_cross_products() {
    let left = SharedScaleVec::from_components([frac(1, 6), frac(5, 6), frac(-7, 6)])
        .expect("sixths share a reduced denominator");
    let right = SharedScaleVec::from_components([frac(3, 10), frac(-7, 10), frac(9, 10)])
        .expect("tenths share a reduced denominator");

    let expected =
        Vector3::new(left.components().clone()).cross(&Vector3::new(right.components().clone()));
    assert_eq!(left.cross(&right), expected);
    assert_eq!(
        left.as_view().cross(right.as_view()),
        Vector3::new([-frac(1, 15), -frac(1, 2), -frac(11, 30)])
    );

    let parallel_left = SharedScaleVec::from_components([frac(2, 9), frac(4, 9), frac(8, 9)])
        .expect("ninths share a reduced denominator");
    let parallel_right =
        SharedScaleVec::from_components([frac(-3, 11), frac(-6, 11), frac(-12, 11)])
            .expect("elevenths share a reduced denominator");
    assert_eq!(parallel_left.cross(&parallel_right), Vector3::zero());
    assert!(
        parallel_left
            .cross(&parallel_right)
            .into_shared_scale()
            .is_some()
    );

    let axis_left = SharedScaleVec::from_components([one(), zero(), zero()])
        .expect("integer axis vector has shared scale");
    let axis_right = SharedScaleVec::from_components([zero(), one(), zero()])
        .expect("integer axis vector has shared scale");
    let axis_cross = axis_left.cross(&axis_right);
    assert_eq!(axis_cross, Vector3::new([zero(), zero(), one()]));
    assert!(axis_cross.into_shared_scale().is_some());
}

#[test]
fn vector_scalar_add_and_subtract_are_componentwise() {
    let vector = Vector4::new([r(1), r(2), r(3), r(4)]);

    assert_eq!(
        vector.clone() + r(10),
        Vector4::new([r(11), r(12), r(13), r(14)])
    );
    assert_eq!(vector - r(1), Vector4::new([r(0), r(1), r(2), r(3)]));
}

#[test]
fn vector_display_forwards_real_formatting() {
    let vector = Vector3::new([frac(1, 2), r(2), frac(3, 4)]);

    assert_eq!(format!("{vector}"), "[1/2, 2, 3/4]");
    assert_eq!(format!("{vector:#}"), "[0.5, 2, 0.75]");
}

#[test]
fn checked_vector_operations_reject_zero_divisors() {
    let vector = Vector3::new([r(1), r(2), r(3)]);
    let zero_vector: Vector3 = Vector3::zero();

    assert_eq!(zero_vector.clone().normalize(), Err(Problem::DivideByZero));
    assert_eq!(vector.clone() / zero(), Err(Problem::DivideByZero));
    assert_eq!(zero_vector.normalize_checked(), Err(Problem::DivideByZero));
    assert_eq!(
        vector.clone().div_scalar_checked(zero()),
        Err(Problem::DivideByZero)
    );
    assert_eq!(
        vector.div_scalar_checked(r(2)).unwrap(),
        Vector3::new([frac(1, 2), r(1), frac(3, 2)])
    );
}

#[test]
fn checked_vector_operations_reject_unknown_zero_divisors() {
    let vector = Vector3::new([r(1), r(2), r(3)]);
    let signal = abort_signal();

    assert_eq!(
        vector.clone().div_scalar_checked(unknown_zero()),
        Err(Problem::UnknownZero)
    );
    assert_eq!(
        vector.div_scalar_checked_with_abort(unknown_zero(), &signal),
        Err(Problem::UnknownZero)
    );

    let unknown_vector = Vector3::new([unknown_zero(), r(0), r(0)]);
    assert_eq!(
        unknown_vector.normalize_checked_with_abort(&signal),
        Err(Problem::UnknownZero)
    );
}
