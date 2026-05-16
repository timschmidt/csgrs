use hyperlimit::orient::orient2d_with_policy;
use hyperlimit::{
    DeterminantScheduleHint, ExactPredicateKernel, LineSide, Plane3, PlaneSide, Point2, Point3,
    PredicateOutcome, PredicatePolicy, RationalStorageClass, RealExactSetDenominatorKind,
    RealExactSetDyadicExponentClass, RealExactSetSignPattern, RealSymbolicDependencyMask, Sign,
    classify_point_line, classify_point_oriented_plane, classify_point_plane, incircle2d, orient2d,
    orient2d_batch, orient3d,
};

type Real = hyperreal::Real;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("finite test scalar")
}

fn p2(x: f64, y: f64) -> Point2 {
    Point2::new(real(x), real(y))
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(real(x), real(y), real(z))
}

fn decided<T: Copy>(outcome: PredicateOutcome<T>) -> T {
    outcome.value().expect("adversarial case should decide")
}

fn rational(numerator: i64, denominator: u64) -> Real {
    hyperreal::Rational::fraction(numerator, denominator)
        .expect("valid test rational")
        .into()
}

fn unknown_zero() -> Real {
    let one = Real::from(1);
    one.clone().sin() - one.sin()
}

#[test]
fn point_shared_scale_views_preserve_coordinate_facts_without_storage_leaks() {
    let point2 = Point2::new(rational(1, 5), rational(-2, 5));
    let view2 = point2
        .shared_scale_view()
        .expect("fifths should share a denominator");
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
    assert_eq!(view2.exact.exact_integer_count, 0);
    assert!(!view2.exact.has_integer_grid_schedule());
    assert_eq!(view2.exact.known_positive_count, 1);
    assert_eq!(view2.exact.known_negative_count, 1);
    assert_eq!(
        view2.exact.sign_pattern(),
        RealExactSetSignPattern::MixedKnown
    );
    assert_eq!(view2.known_zero_mask, 0);
    assert_eq!(view2.known_nonzero_mask, 0b11);
    assert_eq!(view2.unknown_zero_mask, 0);
    assert_eq!(view2.known_zero_count(), 0);
    assert_eq!(view2.known_nonzero_count(), 2);
    assert_eq!(view2.unknown_zero_count(), 0);
    assert!(view2.is_known_dense());
    assert_eq!(view2.coordinates()[0], &rational(1, 5));

    let zero = Point2::new(Real::from(0), Real::from(0));
    let zero_view = zero
        .shared_scale_view()
        .expect("integer zeros share a denominator");
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

    let point3 = Point3::new(rational(1, 7), rational(2, 7), rational(3, 7));
    let view3 = point3
        .shared_scale_view()
        .expect("sevenths should share a denominator");
    assert_eq!(view3.known_nonzero_mask, 0b111);
    assert_eq!(view3.known_nonzero_count(), 3);
    assert_eq!(view3.known_zero_count(), 0);
    assert!(view3.exact.has_shared_denominator_schedule());
    assert!(!view3.exact.has_integer_grid_schedule());
    assert_eq!(
        view3.exact.sign_pattern(),
        RealExactSetSignPattern::AllPositive
    );

    let dyadic = Point3::new(rational(1, 8), rational(-3, 8), rational(5, 8));
    let dyadic_view = dyadic
        .shared_scale_view()
        .expect("eighths should share a dyadic denominator");
    assert_eq!(
        dyadic_view.exact.max_dyadic_exponent_class,
        Some(RealExactSetDyadicExponentClass::Small)
    );

    let mixed = Point3::new(rational(1, 2), rational(1, 3), rational(1, 6));
    assert!(mixed.shared_scale_view().is_none());

    let symbolic = Point2::new(Real::pi(), rational(1, 5));
    assert!(symbolic.shared_scale_view().is_none());
}

#[test]
fn point_structural_facts_preserve_sparse_coordinate_metadata() {
    let point2 = Point2::new(Real::from(0), Real::from(1));
    let facts2 = point2.structural_facts();
    assert_eq!(facts2.known_zero_mask, 0b01);
    assert_eq!(facts2.known_nonzero_mask, 0b10);
    assert_eq!(facts2.unknown_zero_mask, 0);
    assert_eq!(facts2.one_mask, 0b10);
    assert_eq!(facts2.known_axis_index, Some(1));
    assert_eq!(facts2.known_zero_count(), 1);
    assert_eq!(facts2.known_nonzero_count(), 1);
    assert_eq!(facts2.unknown_zero_count(), 0);
    assert!(!facts2.has_unknown_zero());
    assert!(facts2.is_one_hot());
    assert!(facts2.has_sparse_support());
    assert!(!facts2.known_zero);
    assert!(facts2.exact.has_integer_grid_schedule());
    assert!(facts2.exact.has_signed_unit_schedule());
    assert!(facts2.symbolic_dependencies.is_empty());

    let point3 = Point3::new(Real::from(0), Real::from(-1), Real::from(0));
    let facts3 = point3.structural_facts();
    assert_eq!(facts3.known_zero_mask, 0b101);
    assert_eq!(facts3.known_nonzero_mask, 0b010);
    assert_eq!(facts3.unknown_zero_mask, 0);
    assert_eq!(facts3.one_mask, 0);
    assert_eq!(facts3.known_axis_index, Some(1));
    assert_eq!(facts3.known_zero_count(), 2);
    assert_eq!(facts3.known_nonzero_count(), 1);
    assert_eq!(facts3.unknown_zero_count(), 0);
    assert!(!facts3.has_unknown_zero());
    assert!(facts3.is_one_hot());
    assert!(facts3.has_sparse_support());
    assert!(!facts3.known_zero);
    assert!(facts3.exact.has_signed_unit_schedule());
    assert!(facts3.symbolic_dependencies.is_empty());

    let symbolic = Point2::new(unknown_zero(), Real::from(0));
    let symbolic_facts = symbolic.structural_facts();
    assert_eq!(symbolic_facts.known_zero_mask, 0b10);
    assert_eq!(symbolic_facts.unknown_zero_mask, 0b01);
    assert_eq!(symbolic_facts.known_axis_index, None);
    assert!(symbolic_facts.has_unknown_zero());
    assert!(!symbolic_facts.is_one_hot());
    assert!(!symbolic_facts.has_sparse_support());
    assert!(!symbolic_facts.exact.has_shared_denominator_schedule());
}

#[test]
fn point_structural_facts_summarize_symbolic_dependencies() {
    let point2 = Point2::new(Real::pi(), Real::from(2).ln().unwrap());
    let facts2 = point2.structural_facts();
    assert!(
        facts2
            .symbolic_dependencies
            .contains(RealSymbolicDependencyMask::PI)
    );
    assert!(
        facts2
            .symbolic_dependencies
            .contains(RealSymbolicDependencyMask::LOG)
    );
    assert!(
        !facts2
            .symbolic_dependencies
            .contains(RealSymbolicDependencyMask::TRIG)
    );

    let trig = (rational(1, 5) * Real::pi()).sin();
    let point3 = Point3::new(trig, Real::e(), Real::from(0));
    let facts3 = point3.structural_facts();
    assert!(
        facts3
            .symbolic_dependencies
            .contains(RealSymbolicDependencyMask::TRIG)
    );
    assert!(
        facts3
            .symbolic_dependencies
            .contains(RealSymbolicDependencyMask::PI)
    );
    assert!(
        facts3
            .symbolic_dependencies
            .contains(RealSymbolicDependencyMask::EXP)
    );
}

#[test]
fn prepared_predicate_facts_preserve_point_local_shared_scales() {
    let a = Point2::new(rational(1, 3), rational(2, 3));
    let b = Point2::new(rational(1, 5), rational(2, 5));
    let prepared = hyperlimit::PreparedLine2::new(&a, &b);
    let facts = prepared.facts();

    assert!(facts.fixed_coordinates_exact_rational);
    assert!(!facts.fixed_coordinates_dyadic);
    assert!(!facts.fixed_coordinates_shared_denominator);
    assert_eq!(facts.fixed_point_shared_scale_mask, 0b11);
    assert_eq!(facts.fixed_point_origin_mask, 0);
    assert_eq!(facts.fixed_point_one_hot_mask, 0);
    assert_eq!(facts.fixed_point_unknown_zero_mask, 0);
    assert_eq!(facts.fixed_point_shared_scale_count(), 2);
    assert_eq!(facts.fixed_point_origin_count(), 0);
    assert_eq!(facts.fixed_point_one_hot_count(), 0);
    assert_eq!(facts.fixed_point_unknown_zero_count(), 0);
    assert!(!facts.has_fixed_point_unknown_zero());
    assert_eq!(facts.fixed_point_sparse_support_mask(), 0);
    assert!(facts.fixed_symbolic_dependencies.is_empty());

    let c = Point2::new(Real::pi(), rational(3, 7));
    let incircle = hyperlimit::PreparedIncircle2::new(&a, &b, &c);
    let facts = incircle.facts();
    assert!(!facts.fixed_coordinates_exact_rational);
    assert_eq!(facts.fixed_point_shared_scale_mask, 0b011);
    assert_eq!(facts.fixed_point_unknown_zero_mask, 0);
    assert!(
        facts
            .fixed_symbolic_dependencies
            .contains(RealSymbolicDependencyMask::PI)
    );
    assert!(
        !facts
            .fixed_symbolic_dependencies
            .contains(RealSymbolicDependencyMask::LOG)
    );

    let trig = (rational(1, 5) * Real::pi()).sin();
    let symbolic3 = Point3::new(trig, Real::e(), Real::from(0));
    let origin3_symbolic = Point3::new(Real::from(0), Real::from(0), Real::from(0));
    let x_axis3_symbolic = Point3::new(Real::from(1), Real::from(0), Real::from(0));
    let y_axis3_symbolic = Point3::new(Real::from(0), Real::from(1), Real::from(0));
    let symbolic_insphere = hyperlimit::PreparedInsphere3::new(
        &origin3_symbolic,
        &x_axis3_symbolic,
        &y_axis3_symbolic,
        &symbolic3,
    );
    let facts = symbolic_insphere.facts();
    assert!(
        facts
            .fixed_symbolic_dependencies
            .contains(RealSymbolicDependencyMask::TRIG)
    );
    assert!(
        facts
            .fixed_symbolic_dependencies
            .contains(RealSymbolicDependencyMask::PI)
    );
    assert!(
        facts
            .fixed_symbolic_dependencies
            .contains(RealSymbolicDependencyMask::EXP)
    );

    let origin = Point2::new(Real::from(0), Real::from(0));
    let y_axis = Point2::new(Real::from(0), Real::from(5));
    let unknown = Point2::new(unknown_zero(), Real::from(0));
    let prepared = hyperlimit::PreparedIncircle2::new(&origin, &y_axis, &unknown);
    let facts = prepared.facts();
    assert_eq!(facts.fixed_point_origin_mask, 0b001);
    assert_eq!(facts.fixed_point_one_hot_mask, 0b010);
    assert_eq!(facts.fixed_point_unknown_zero_mask, 0b100);
    assert_eq!(facts.fixed_point_origin_count(), 1);
    assert_eq!(facts.fixed_point_one_hot_count(), 1);
    assert_eq!(facts.fixed_point_unknown_zero_count(), 1);
    assert!(facts.has_fixed_point_unknown_zero());
    assert_eq!(facts.fixed_point_sparse_support_mask(), 0b011);

    let origin3 = Point3::new(Real::from(0), Real::from(0), Real::from(0));
    let x_axis3 = Point3::new(Real::from(7), Real::from(0), Real::from(0));
    let y_axis3 = Point3::new(Real::from(0), Real::from(-1), Real::from(0));
    let unknown3 = Point3::new(Real::from(0), unknown_zero(), Real::from(0));
    let prepared = hyperlimit::PreparedInsphere3::new(&origin3, &x_axis3, &y_axis3, &unknown3);
    let facts = prepared.facts();
    assert_eq!(facts.fixed_point_origin_mask, 0b0001);
    assert_eq!(facts.fixed_point_one_hot_mask, 0b0110);
    assert_eq!(facts.fixed_point_unknown_zero_mask, 0b1000);
    assert_eq!(facts.fixed_point_origin_count(), 1);
    assert_eq!(facts.fixed_point_one_hot_count(), 2);
    assert_eq!(facts.fixed_point_unknown_zero_count(), 1);
    assert_eq!(facts.fixed_point_sparse_support_mask(), 0b0111);
}

#[test]
fn prepared_predicate_facts_select_advisory_determinant_schedule_hints() {
    let origin = Point2::new(Real::from(0), Real::from(0));
    let axis = Point2::new(Real::from(3), Real::from(0));
    let sparse = hyperlimit::PreparedLine2::new(&origin, &axis);
    assert_eq!(
        sparse.facts().determinant_schedule_hint(),
        DeterminantScheduleHint::SparseSupportCandidate {
            kernel: ExactPredicateKernel::Orient2dRationalDet2,
            fixed_sparse_points: 2,
        }
    );

    let fifth_a = Point2::new(rational(1, 5), rational(2, 5));
    let fifth_b = Point2::new(rational(3, 5), rational(4, 5));
    let shared = hyperlimit::PreparedLine2::new(&fifth_a, &fifth_b);
    assert_eq!(
        shared.facts().determinant_schedule_hint(),
        DeterminantScheduleHint::SharedDenominatorCandidate {
            kernel: ExactPredicateKernel::Orient2dRationalDet2,
        }
    );

    let dyadic_a = Point2::new(rational(1, 2), rational(1, 4));
    let dyadic_b = Point2::new(rational(3, 8), rational(5, 16));
    let dyadic = hyperlimit::PreparedLine2::new(&dyadic_a, &dyadic_b);
    assert_eq!(
        dyadic.facts().determinant_schedule_hint(),
        DeterminantScheduleHint::DyadicCandidate {
            kernel: ExactPredicateKernel::Orient2dRationalDet2,
        }
    );

    let mixed_a = Point2::new(rational(1, 3), rational(2, 5));
    let mixed_b = Point2::new(rational(3, 7), rational(4, 11));
    let exact = hyperlimit::PreparedLine2::new(&mixed_a, &mixed_b);
    assert_eq!(
        exact.facts().determinant_schedule_hint(),
        DeterminantScheduleHint::ExactRationalKernel {
            kernel: ExactPredicateKernel::Orient2dRationalDet2,
        }
    );

    let symbolic = Point2::new(Real::pi(), rational(1, 7));
    let fallback = hyperlimit::PreparedLine2::new(&mixed_a, &symbolic);
    assert_eq!(
        fallback.facts().determinant_schedule_hint(),
        DeterminantScheduleHint::GenericRealFallback
    );
}

#[test]
fn orient2d_handles_exact_degenerate_and_near_degenerate_cases() {
    let a = p2(0.0, 0.0);
    let b = p2(1.0, 0.0);

    assert_eq!(decided(orient2d(&a, &b, &p2(0.0, 1.0))), Sign::Positive);
    assert_eq!(decided(orient2d(&a, &b, &p2(0.0, -1.0))), Sign::Negative);
    assert_eq!(decided(orient2d(&a, &b, &p2(0.5, 0.0))), Sign::Zero);
    assert_eq!(
        decided(orient2d_with_policy(
            &a,
            &b,
            &Point2::new(real(0.5), real(f64::from_bits(1))),
            PredicatePolicy::STRICT,
        )),
        Sign::Positive
    );
}

#[test]
fn orientation_sign_changes_under_coordinate_permutation() {
    let a = p2(-3.0, 2.0);
    let b = p2(5.0, -7.0);
    let c = p2(11.0, 13.0);

    assert_eq!(decided(orient2d(&a, &b, &c)), decided(orient2d(&b, &c, &a)));
    assert_eq!(
        decided(orient2d(&a, &b, &c)).reversed(),
        decided(orient2d(&a, &c, &b))
    );
    assert_eq!(decided(classify_point_line(&a, &b, &c)), LineSide::Left);
}

#[test]
fn batch_predicates_match_scalar_predicates_on_hostile_rows() {
    let cases = vec![
        (p2(0.0, 0.0), p2(1.0, 1.0), p2(0.5, 0.5 + 1.0e-15)),
        (
            p2(1.0e150, 1.0e150),
            p2(1.0e150 + 1.0, 1.0e150),
            p2(1.0e150, 1.0e150 + 1.0),
        ),
        (p2(-1.0e-150, 0.0), p2(0.0, 1.0e-150), p2(1.0e-150, 0.0)),
    ];

    assert_eq!(
        orient2d_batch(&cases),
        cases
            .iter()
            .map(|(a, b, c)| orient2d(a, b, c))
            .collect::<Vec<_>>()
    );
}

#[test]
fn orient3d_and_plane_classification_agree_on_sides_and_degeneracy() {
    let a = p3(0.0, 0.0, 0.0);
    let b = p3(1.0, 0.0, 0.0);
    let c = p3(0.0, 1.0, 0.0);

    assert_eq!(
        decided(orient3d(&a, &b, &c, &p3(0.0, 0.0, 1.0))),
        Sign::Negative
    );
    assert_eq!(
        decided(orient3d(&a, &b, &c, &p3(0.0, 0.0, -1.0))),
        Sign::Positive
    );
    assert_eq!(
        decided(orient3d(&a, &b, &c, &p3(0.25, 0.25, 0.0))),
        Sign::Zero
    );

    assert_eq!(
        decided(classify_point_oriented_plane(
            &a,
            &b,
            &c,
            &p3(0.0, 0.0, 1.0)
        )),
        PlaneSide::Below
    );
    assert_eq!(
        decided(classify_point_plane(
            &p3(0.0, 0.0, 3.0),
            &Plane3::new(p3(0.0, 0.0, 1.0), real(-2.0)),
        )),
        PlaneSide::Above
    );
}

#[test]
fn incircle_detects_inside_outside_and_on_circle_cases() {
    let a = p2(1.0, 0.0);
    let b = p2(0.0, 1.0);
    let c = p2(-1.0, 0.0);

    assert_eq!(
        decided(incircle2d(&a, &b, &c, &p2(0.0, 0.0))),
        Sign::Positive
    );
    assert_eq!(
        decided(incircle2d(&a, &b, &c, &p2(0.0, 2.0))),
        Sign::Negative
    );
    assert_eq!(decided(incircle2d(&a, &b, &c, &p2(0.0, -1.0))), Sign::Zero);
}
