#[cfg(feature = "predicates")]
use hypercurve::Similarity2;
use hypercurve::{
    BezierSubcurve2, Curve2, CurveContext, CurveError, CurveFamily2, CurveOperation2,
    CurveParameterSide2, ExactCurveError, NurbsCurve2, Point2, Real, SplinePeriodicity2,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn quadratic_nurbs() -> NurbsCurve2 {
    NurbsCurve2::try_new(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(1), r(2), r(4), r(1)],
        vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value()
}

#[cfg(feature = "predicates")]
fn terminal_nurbs() -> (NurbsCurve2, Real) {
    let half = q(1, 2);
    let symbolic_half = &half + ((Real::pi() + Real::e()) - (Real::e() + Real::pi()));
    let curve = NurbsCurve2::try_new(
        2,
        vec![p(0, 0), p(1, 2), p(2, 0), p(3, -2), p(4, 0)],
        vec![Real::one(); 5],
        vec![
            r(0),
            r(0),
            r(0),
            half,
            symbolic_half.clone(),
            r(1),
            r(1),
            r(1),
        ],
        &CurveContext::APPROXIMATE_512,
    )
    .expect("the terminal policy must retain the symbolically repeated knot")
    .into_value();
    (curve, symbolic_half)
}

#[cfg(feature = "predicates")]
#[test]
fn nurbs_construction_obeys_terminal_policy_without_replacing_knots() {
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let symbolic_end = r(1) + undecidable_zero.clone();
    let controls = vec![p(0, 0), p(2, 0)];
    let weights = vec![Real::one(), Real::one()];
    let knots = vec![r(0), r(0), symbolic_end.clone(), r(1)];

    assert!(matches!(
        NurbsCurve2::try_new(
            1,
            controls.clone(),
            weights.clone(),
            knots.clone(),
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Construction
                && blocker.reason() == hypercurve::UncertaintyReason::Ordering
    ));

    let constructed = NurbsCurve2::try_new(
        1,
        controls.clone(),
        weights.clone(),
        knots.clone(),
        &CurveContext::APPROXIMATE_512,
    )
    .expect("the terminal policy must validate the symbolic clamped knot");
    assert_eq!(
        constructed.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(constructed.value.knots(), knots);
    assert_eq!(constructed.value.parameter_domain().1, &symbolic_end);

    let half = q(1, 2);
    let symbolic_half = &half + undecidable_zero;
    let evaluation_curve = NurbsCurve2::try_new(
        2,
        vec![p(0, 0), p(1, 2), p(2, 0), p(3, -2), p(4, 0)],
        vec![Real::one(); 5],
        vec![
            r(0),
            r(0),
            r(0),
            half,
            symbolic_half.clone(),
            r(1),
            r(1),
            r(1),
        ],
        &CurveContext::APPROXIMATE_512,
    )
    .expect("the terminal policy must retain the symbolically repeated interior knot")
    .into_value();
    assert!(matches!(
        evaluation_curve.bezier_decomposition(&CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::BezierDecomposition
                && blocker.reason() == hypercurve::UncertaintyReason::Ordering
    ));
    let decomposition = evaluation_curve
        .bezier_decomposition(&CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must decompose the exact symbolic carrier");
    assert_eq!(
        decomposition.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert!(
        decomposition
            .value
            .refined_knots()
            .iter()
            .any(|knot| knot == &symbolic_half)
    );

    let spans = evaluation_curve
        .bezier_spans(&CurveContext::APPROXIMATE_512)
        .expect("the retained decomposition must preserve approximate ownership");
    assert_eq!(
        spans.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(spans.into_value().len(), 2);
    let native = evaluation_curve
        .native_subcurves(&CurveContext::APPROXIMATE_512)
        .expect("native promotion must use the same terminal policy");
    assert_eq!(
        native.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(native.value.len(), 2);
    let native_spans = evaluation_curve
        .native_spans(&CurveContext::APPROXIMATE_512)
        .expect("native span views must preserve terminal consumption");
    assert_eq!(
        native_spans.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(native_spans.into_value().len(), 2);

    assert!(matches!(
        evaluation_curve.point_at(&symbolic_half, &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Evaluation
                && blocker.reason() == hypercurve::UncertaintyReason::Ordering
    ));
    let point = evaluation_curve
        .point_at(&symbolic_half, &CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must evaluate the exact symbolic knot");
    assert_eq!(
        point.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(point.value, p(2, 0));
    let derivative = evaluation_curve
        .derivative_at_side(
            &symbolic_half,
            CurveParameterSide2::Left,
            &CurveContext::APPROXIMATE_512,
        )
        .expect("the terminal policy must evaluate the exact symbolic derivative");
    assert_eq!(
        derivative.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(derivative.value.dx(), &r(4));
    assert_eq!(derivative.value.dy(), &r(-8));
    let derivatives = evaluation_curve
        .derivatives_at_side(
            &symbolic_half,
            2,
            CurveParameterSide2::Left,
            &CurveContext::APPROXIMATE_512,
        )
        .expect("higher derivatives must use the selected terminal policy");
    assert_eq!(
        derivatives.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(derivatives.value.len(), 2);
    assert_eq!(derivatives.value[0], derivative.value);

    let evaluation_top_level = Curve2::from(evaluation_curve.clone());
    let top_level_point = evaluation_top_level
        .as_view()
        .point_at(&symbolic_half, &CurveContext::APPROXIMATE_512)
        .expect("CurveView2 must preserve NURBS evaluation certainty");
    assert_eq!(
        top_level_point.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(top_level_point.value, p(2, 0));
    assert!(matches!(
        evaluation_top_level.point_at(&symbolic_half, &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Evaluation
                && blocker.reason() == hypercurve::UncertaintyReason::Ordering
    ));

    assert!(matches!(
        evaluation_curve.bezier_decomposition(&CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::BezierDecomposition
                && blocker.reason() == hypercurve::UncertaintyReason::Ordering
    ));
    assert!(matches!(
        evaluation_curve.native_subcurves(&CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::NativeTopology
                && blocker.reason() == hypercurve::UncertaintyReason::Ordering
    ));

    let top_level = Curve2::try_nurbs(
        1,
        controls,
        weights,
        vec![r(0), r(0), symbolic_end, r(1)],
        &CurveContext::APPROXIMATE_512,
    )
    .expect("Curve2 must preserve construction certainty");
    assert_eq!(
        top_level.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
}

#[cfg(feature = "predicates")]
#[test]
fn nurbs_subdivision_reconstruction_obeys_terminal_policy() {
    let curve = quadratic_nurbs();
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let parameter = r(1) + undecidable_zero;

    let strict = curve
        .split_at(parameter.clone(), &CurveContext::STRICT)
        .unwrap_err();
    assert!(
        matches!(
            &strict,
            ExactCurveError::Blocked(blocker)
                if blocker.operation() == CurveOperation2::Subdivision
                    && blocker.reason() == hypercurve::UncertaintyReason::Ordering
        ),
        "{strict:?}"
    );

    let split = curve
        .split_at(parameter.clone(), &CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must resolve the symbolically equal knot");
    assert_eq!(
        split.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    let (left, right) = split.into_value();
    assert_eq!(left.parameter_domain(), (&r(0), &parameter));
    assert_eq!(right.parameter_domain(), (&parameter, &r(2)));
    assert_eq!(left.end(), right.start());

    let subcurve = curve
        .subcurve(r(0), parameter.clone(), &CurveContext::APPROXIMATE_512)
        .expect("terminal policy must propagate through reconstructed subcurves");
    assert_eq!(
        subcurve.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(subcurve.value.parameter_domain(), (&r(0), &parameter));

    let top_level = Curve2::from(curve.clone());
    let top_level_split = top_level
        .split_at(parameter.clone(), &CurveContext::APPROXIMATE_512)
        .expect("Curve2 must propagate the selected policy into its NURBS carrier");
    assert_eq!(
        top_level_split.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert!(
        top_level_split
            .value
            .0
            .parameter_domain()
            .end()
            .eq(&parameter)
    );

    assert!(matches!(
        curve.split_at(parameter, &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Subdivision
                && blocker.reason() == hypercurve::UncertaintyReason::Ordering
    ));
}

#[cfg(feature = "predicates")]
#[test]
fn nurbs_exact_edits_isolate_terminal_policy_and_replay_retained_proofs() {
    let (curve, symbolic_half) = terminal_nurbs();
    let knot = q(3, 4);

    let strict_insertion = curve
        .insert_knot(knot.clone(), &CurveContext::STRICT)
        .unwrap_err();
    assert!(matches!(
        strict_insertion,
        ExactCurveError::Blocked(blocker)
            if blocker.operation() == CurveOperation2::KnotInsertion
    ));
    let inserted = curve
        .insert_knot(knot.clone(), &CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must refine the exact symbolic carrier");
    assert_eq!(
        inserted.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(inserted.value.control_points().len(), 6);
    assert!(
        inserted
            .value
            .knots()
            .iter()
            .any(|value| value == &symbolic_half)
    );
    let inserted_replay = curve
        .insert_knot(knot.clone(), &CurveContext::APPROXIMATE_512)
        .expect("the retained terminal refinement must replay");
    assert_eq!(
        inserted_replay.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert!(std::ptr::eq(
        inserted.value.control_points(),
        inserted_replay.value.control_points()
    ));
    assert_eq!(
        curve
            .insert_knot(knot.clone(), &CurveContext::STRICT)
            .unwrap_err(),
        strict_insertion
    );
    let approximate_first_knot = q(7, 8);
    let approximate_first = curve
        .insert_knot(
            approximate_first_knot.clone(),
            &CurveContext::APPROXIMATE_512,
        )
        .expect("an approximate-first cache entry must preserve its strict blocker");
    assert_eq!(
        approximate_first.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert!(matches!(
        curve.insert_knot(approximate_first_knot, &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::KnotInsertion
    ));

    let strict_removal = inserted
        .value
        .remove_knot(knot.clone(), &CurveContext::STRICT)
        .unwrap_err();
    let removed = inserted
        .value
        .remove_knot(knot, &CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must certify inverse knot removal");
    assert_eq!(
        removed.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    let removed_curve = removed
        .value
        .as_ref()
        .expect("the inserted knot must remain exactly removable");
    assert_eq!(removed_curve.degree(), curve.degree());
    assert_eq!(removed_curve.knots(), curve.knots());
    assert_eq!(removed_curve.start(), curve.start());
    assert_eq!(removed_curve.end(), curve.end());
    assert_eq!(
        inserted
            .value
            .remove_knot(q(3, 4), &CurveContext::STRICT)
            .unwrap_err(),
        strict_removal
    );

    let strict_spans = curve
        .degree_elevation(3, &CurveContext::STRICT)
        .unwrap_err();
    let spans = curve
        .degree_elevation(3, &CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must elevate every exact symbolic span");
    assert_eq!(
        spans.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    let span_replay = curve
        .degree_elevation(3, &CurveContext::APPROXIMATE_512)
        .expect("the retained span elevation must replay");
    assert!(std::ptr::eq(
        spans.value.spans().as_ptr(),
        span_replay.value.spans().as_ptr()
    ));
    assert_eq!(
        curve
            .degree_elevation(3, &CurveContext::STRICT)
            .unwrap_err(),
        strict_spans
    );

    let strict_carrier = curve
        .elevated_to_degree(3, &CurveContext::STRICT)
        .unwrap_err();
    let elevated = curve
        .elevated_to_degree(3, &CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must reconstruct the elevated carrier");
    assert_eq!(
        elevated.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(elevated.value.degree(), 3);
    let elevated_replay = curve
        .elevated_to_degree(3, &CurveContext::APPROXIMATE_512)
        .expect("the retained elevated carrier must replay");
    assert!(std::ptr::eq(
        elevated.value.control_points(),
        elevated_replay.value.control_points()
    ));
    assert_eq!(
        curve
            .elevated_to_degree(3, &CurveContext::STRICT)
            .unwrap_err(),
        strict_carrier
    );

    let reversed = curve
        .reversed(&CurveContext::APPROXIMATE_512)
        .expect("reversal must validate exact reflected symbolic knots");
    assert_eq!(
        reversed.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert!(matches!(
        curve.reversed(&CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Reversal
    ));

    let transform = Similarity2::try_from_real_affine(
        Real::one(),
        Real::zero(),
        Real::zero(),
        Real::one(),
        r(2),
        r(3),
    )
    .unwrap();
    let transformed = Curve2::from(curve.clone())
        .transform_similarity(&transform, &CurveContext::APPROXIMATE_512)
        .expect("Curve2 transformation must preserve the terminal policy");
    assert_eq!(
        transformed.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert!(matches!(
        Curve2::from(curve).transform_similarity(&transform, &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Transformation
    ));
}

#[test]
fn linear_nurbs_evaluates_and_promotes_with_source_provenance() {
    let curve = NurbsCurve2::try_new(
        1,
        vec![p(0, 0), p(4, 0)],
        vec![r(1), r(3)],
        vec![r(0), r(0), r(1), r(1)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let half = (r(1) / r(2)).unwrap();

    assert_eq!(curve.degree(), 1);
    assert_eq!(curve.parameter_domain(), (&r(0), &r(1)));
    assert_eq!(
        curve
            .point_at(&half, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(3, 0)
    );
    let derivative = curve
        .derivative_at(&half, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(derivative.dx(), &r(3));
    assert_eq!(derivative.dy(), &r(0));
    assert_eq!(
        curve
            .derivative_at(&half, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        derivative
    );
    let spans = curve
        .native_spans(&CurveContext::STRICT)
        .unwrap()
        .into_value()
        .collect::<Vec<_>>();
    assert_eq!(spans.len(), 1);
    assert_eq!(spans[0].source_span().knot_interval(), (&r(0), &r(1)));
    assert!(matches!(
        spans[0].curve(),
        BezierSubcurve2::RationalQuadratic(_)
    ));

    let top_level = Curve2::from(curve);
    let fragments = top_level
        .native_bezier_fragments(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(fragments.len(), 1);
    assert_eq!(fragments[0].parameter_range(), (&r(0), &r(1)));
}

#[test]
fn nurbs_derivative_uses_authored_knot_parameter_and_shared_span_evaluators() {
    let curve = NurbsCurve2::try_new(
        1,
        vec![p(0, 0), p(4, 8)],
        vec![r(1), r(1)],
        vec![r(2), r(2), r(6), r(6)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let clone = curve.clone();

    assert_eq!(curve.parameter_domain(), (&r(2), &r(6)));
    let derivative = curve
        .derivative_at(&r(3), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(derivative.dx(), &r(1));
    assert_eq!(derivative.dy(), &r(2));
    assert_eq!(
        clone
            .derivative_at(&r(5), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        derivative
    );
}

#[test]
fn nurbs_higher_derivatives_use_each_authored_parameter_chain_power() {
    let curve = NurbsCurve2::try_new(
        1,
        vec![p(0, 0), p(4, 0)],
        vec![r(1), r(3)],
        vec![r(2), r(2), r(6), r(6)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    let derivatives = curve
        .derivatives_at(&r(4), 3, &CurveContext::STRICT)
        .unwrap()
        .into_value();

    assert_eq!(derivatives.len(), 3);
    assert_eq!(
        (derivatives[0].dx(), derivatives[0].dy()),
        (&q(3, 4), &r(0))
    );
    assert_eq!(
        (derivatives[1].dx(), derivatives[1].dy()),
        (&q(-3, 8), &r(0))
    );
    assert_eq!(
        (derivatives[2].dx(), derivatives[2].dy()),
        (&q(9, 32), &r(0))
    );
}

#[test]
fn nurbs_internal_corner_requires_explicit_derivative_side() {
    let curve = NurbsCurve2::try_new(
        1,
        vec![p(0, 0), p(1, 0), p(1, 1)],
        vec![r(1), r(1), r(1)],
        vec![r(0), r(0), r(1), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    let error = curve
        .derivative_at(&r(1), &CurveContext::STRICT)
        .unwrap_err();
    assert!(matches!(
        error,
        ExactCurveError::Blocked(blocker)
            if blocker.reason() == hypercurve::UncertaintyReason::Boundary
    ));
    let left = curve
        .derivative_at_side(&r(1), CurveParameterSide2::Left, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let right = curve
        .derivative_at_side(&r(1), CurveParameterSide2::Right, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!((left.dx(), left.dy()), (&r(1), &r(0)));
    assert_eq!((right.dx(), right.dy()), (&r(0), &r(1)));

    let top_level = Curve2::from(curve);
    assert!(
        top_level
            .derivative_at(&r(1), &CurveContext::STRICT)
            .is_err()
    );
    assert_eq!(
        top_level
            .derivative_at_side(&r(1), CurveParameterSide2::Right, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        right
    );
}

#[test]
fn discontinuous_nurbs_knot_requires_explicit_point_side() {
    let curve = NurbsCurve2::try_new(
        2,
        vec![p(0, 0), p(1, 1), p(2, 0), p(10, 0), p(11, 1), p(12, 0)],
        vec![r(1); 6],
        vec![r(0), r(0), r(0), r(1), r(1), r(1), r(2), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    assert!(matches!(
        curve.point_at(&r(1), &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::Boundary
    ));
    assert_eq!(
        curve
            .point_at_side(&r(1), CurveParameterSide2::Left, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(2, 0)
    );
    assert_eq!(
        curve
            .point_at_side(&r(1), CurveParameterSide2::Right, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(10, 0)
    );

    let (left, right) = curve
        .split_at(r(1), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(left.parameter_domain(), (&r(0), &r(1)));
    assert_eq!(right.parameter_domain(), (&r(1), &r(2)));
    assert_eq!(left.end(), &p(2, 0));
    assert_eq!(right.start(), &p(10, 0));

    let top_level = Curve2::from(curve);
    assert_eq!(
        top_level
            .as_view()
            .point_at_side(&r(1), CurveParameterSide2::Right, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(10, 0)
    );
}

#[test]
fn nurbs_knot_insertion_preserves_exact_image_source_and_full_multiplicity_cache() {
    let curve = NurbsCurve2::try_new(
        2,
        vec![p(0, 0), p(2, 4), p(4, 0)],
        vec![r(1), r(2), r(1)],
        vec![r(0), r(0), r(0), r(2), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let samples = [r(0), (r(1) / r(2)).unwrap(), r(1), r(2)];
    let expected = samples
        .iter()
        .map(|parameter| {
            curve
                .point_at(parameter, &CurveContext::STRICT)
                .unwrap()
                .into_value()
        })
        .collect::<Vec<_>>();

    let once = curve
        .insert_knot(r(1), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let twice = once
        .insert_knot(r(1), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(
        once.control_points().len(),
        curve.control_points().len() + 1
    );
    assert_eq!(
        twice.control_points().len(),
        curve.control_points().len() + 2
    );
    assert_eq!(
        twice.knots().iter().filter(|knot| **knot == r(1)).count(),
        2
    );
    assert_eq!(
        samples
            .iter()
            .map(|parameter| twice
                .point_at(parameter, &CurveContext::STRICT)
                .unwrap()
                .into_value())
            .collect::<Vec<_>>(),
        expected
    );

    let cached = twice
        .bezier_decomposition(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    let no_op = twice
        .insert_knot(r(1), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert!(std::ptr::eq(
        cached,
        no_op
            .bezier_decomposition(&CurveContext::STRICT)
            .unwrap()
            .into_value()
    ));
    assert_eq!(no_op, twice);
}

#[test]
fn nurbs_batch_knot_refinement_projects_once_and_reuses_clone_shared_result() {
    let curve = NurbsCurve2::try_new(
        2,
        vec![p(0, 0), p(2, 4), p(4, 0)],
        vec![r(1), r(2), r(1)],
        vec![r(0), r(0), r(0), r(2), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let clone = curve.clone();
    let request = vec![r(1), r(1)];

    let batch = curve
        .insert_knots(request.clone(), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let sequential = curve
        .insert_knot(r(1), &CurveContext::STRICT)
        .unwrap()
        .into_value()
        .insert_knot(r(1), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(batch, sequential);
    for parameter in [r(0), q(1, 2), r(1), q(3, 2), r(2)] {
        assert_eq!(
            batch.point_at(&parameter, &CurveContext::STRICT),
            curve.point_at(&parameter, &CurveContext::STRICT)
        );
    }

    let retained = batch
        .bezier_decomposition(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    let replay = clone
        .insert_knots(request, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert!(std::ptr::eq(
        retained,
        replay
            .bezier_decomposition(&CurveContext::STRICT)
            .unwrap()
            .into_value()
    ));
}

#[test]
fn nurbs_batch_knot_refinement_retains_contextual_failure_without_mutating_source() {
    let curve = quadratic_nurbs();
    let source_control_count = curve.control_points().len();
    let request = vec![r(1), r(3)];

    let first = curve
        .insert_knots(request.clone(), &CurveContext::STRICT)
        .unwrap_err();
    assert_eq!(first.operation(), CurveOperation2::KnotInsertion);
    assert_eq!(first.family(), CurveFamily2::Nurbs);
    assert_eq!(
        curve
            .insert_knots(request, &CurveContext::STRICT)
            .unwrap_err(),
        first
    );
    assert_eq!(curve.control_points().len(), source_control_count);
}

#[test]
fn nurbs_knot_removal_exactly_inverts_insertion_and_reuses_clone_shared_proof() {
    let curve = NurbsCurve2::try_new(
        3,
        vec![p(0, 0), p(1, 4), p(4, 3), p(6, 0)],
        vec![r(1), r(2), r(5), r(3)],
        vec![r(0), r(0), r(0), r(0), r(2), r(2), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let knot = q(3, 4);
    let refined = curve
        .insert_knot(knot.clone(), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let clone = refined.clone();

    let removed = refined
        .remove_knot(knot.clone(), &CurveContext::STRICT)
        .unwrap()
        .into_value()
        .unwrap();
    assert_eq!(removed.degree(), curve.degree());
    assert_eq!(removed.knots(), curve.knots());
    assert_eq!(removed.control_points(), curve.control_points());
    assert_eq!(removed.weights(), curve.weights());
    for parameter in [r(0), q(1, 4), q(3, 4), q(3, 2), r(2)] {
        assert_eq!(
            removed.point_at(&parameter, &CurveContext::STRICT),
            curve.point_at(&parameter, &CurveContext::STRICT)
        );
    }

    let retained = removed
        .bezier_decomposition(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    let replay = clone
        .remove_knot(knot, &CurveContext::STRICT)
        .unwrap()
        .into_value()
        .unwrap();
    assert!(std::ptr::eq(
        retained,
        replay
            .bezier_decomposition(&CurveContext::STRICT)
            .unwrap()
            .into_value()
    ));
}

#[test]
fn nurbs_knot_removal_retains_exact_negative_result_and_contextual_domain_errors() {
    let curve = quadratic_nurbs();
    let clone = curve.clone();
    assert!(
        curve
            .remove_knot(r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value()
            .is_none()
    );
    assert!(
        clone
            .remove_knot(r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value()
            .is_none()
    );

    for knot in [r(-1), r(0), r(2), r(3)] {
        let error = curve.remove_knot(knot, &CurveContext::STRICT).unwrap_err();
        assert_eq!(error.operation(), CurveOperation2::KnotRemoval);
        assert_eq!(error.family(), CurveFamily2::Nurbs);
        assert!(matches!(
            error,
            ExactCurveError::Invalid {
                cause: CurveError::InvalidCurveParameter,
                ..
            }
        ));
    }
}

#[test]
fn periodic_nurbs_knot_removal_preserves_period_and_wrapped_image() {
    let curve = NurbsCurve2::try_new_periodic(
        2,
        vec![p(0, 0), p(3, 0), p(4, 2), p(2, 5), p(-1, 2)],
        vec![r(1), r(2), r(5), r(3), r(4)],
        vec![r(0), r(1), r(1), r(3), r(5), r(8)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let knot = q(5, 2);
    let refined = curve
        .insert_knot(knot.clone(), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let removed = refined
        .remove_knot(knot, &CurveContext::STRICT)
        .unwrap()
        .into_value()
        .unwrap();

    assert_eq!(removed.period(), curve.period());
    assert_eq!(removed.start(), removed.end());
    for parameter in [r(-3), r(0), q(5, 2), r(7), r(13)] {
        assert_eq!(
            removed.point_at_wrapped(&parameter, &CurveContext::STRICT),
            curve.point_at_wrapped(&parameter, &CurveContext::STRICT)
        );
    }
}

#[test]
fn nurbs_degree_elevation_retains_exact_span_image_intervals_and_source() {
    let curve = NurbsCurve2::try_new(
        2,
        vec![p(0, 0), p(1, 3), p(3, 3), p(4, 0)],
        vec![r(1), r(2), r(3), r(4)],
        vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let clone = curve.clone();

    let elevation = curve
        .degree_elevation(4, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(elevation.source_degree(), 2);
    assert_eq!(elevation.target_degree(), 4);
    assert_eq!(elevation.spans().len(), 2);
    for span in elevation.spans() {
        assert_eq!(span.curve().degree(), 4);
        let (start, end) = span.parameter_interval();
        for local in [r(0), q(1, 2), r(1)] {
            let source_parameter = start + &local * (end - start);
            assert_eq!(
                span.curve()
                    .point_at(&local, &CurveContext::STRICT)
                    .unwrap(),
                curve
                    .point_at_side(
                        &source_parameter,
                        if local == r(0) {
                            CurveParameterSide2::Right
                        } else {
                            CurveParameterSide2::Left
                        },
                        &CurveContext::STRICT,
                    )
                    .unwrap()
                    .into_value()
            );
        }
    }
    let replay = clone
        .degree_elevation(4, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert!(std::ptr::eq(
        elevation.spans().as_ptr(),
        replay.spans().as_ptr()
    ));
}

#[test]
fn nurbs_elevated_carrier_preserves_image_source_and_source_continuity() {
    let curve = NurbsCurve2::try_new(
        2,
        vec![p(0, 0), p(1, 3), p(3, 3), p(4, 0)],
        vec![r(1), r(2), r(3), r(4)],
        vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let clone = curve.clone();

    let elevated = curve
        .elevated_to_degree(4, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(elevated.degree(), 4);
    assert_eq!(elevated.parameter_domain(), curve.parameter_domain());
    assert_eq!(
        elevated
            .knots()
            .iter()
            .filter(|knot| **knot == r(1))
            .count(),
        3
    );
    for parameter in [r(0), q(1, 4), q(3, 4), r(1), q(3, 2), r(2)] {
        assert_eq!(
            elevated.point_at(&parameter, &CurveContext::STRICT),
            curve.point_at(&parameter, &CurveContext::STRICT)
        );
    }
    for parameter in [q(1, 2), r(1), q(3, 2)] {
        assert_eq!(
            elevated.derivative_at(&parameter, &CurveContext::STRICT),
            curve.derivative_at(&parameter, &CurveContext::STRICT)
        );
    }

    let retained = elevated
        .bezier_decomposition(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    let replay = clone
        .elevated_to_degree(4, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert!(std::ptr::eq(
        retained,
        replay
            .bezier_decomposition(&CurveContext::STRICT)
            .unwrap()
            .into_value()
    ));
}

#[test]
fn nurbs_elevated_carrier_preserves_discontinuous_knot_sides() {
    let curve = NurbsCurve2::try_new(
        2,
        vec![p(0, 0), p(1, 1), p(2, 0), p(10, 0), p(11, 1), p(12, 0)],
        vec![r(1); 6],
        vec![r(0), r(0), r(0), r(1), r(1), r(1), r(2), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let elevated = curve
        .elevated_to_degree(4, &CurveContext::STRICT)
        .unwrap()
        .into_value();

    assert_eq!(elevated.degree(), 4);
    assert_eq!(
        elevated
            .knots()
            .iter()
            .filter(|knot| **knot == r(1))
            .count(),
        5
    );
    assert_eq!(
        elevated.point_at_side(&r(1), CurveParameterSide2::Left, &CurveContext::STRICT),
        curve.point_at_side(&r(1), CurveParameterSide2::Left, &CurveContext::STRICT)
    );
    assert_eq!(
        elevated.point_at_side(&r(1), CurveParameterSide2::Right, &CurveContext::STRICT),
        curve.point_at_side(&r(1), CurveParameterSide2::Right, &CurveContext::STRICT)
    );
    assert!(matches!(
        elevated.point_at(&r(1), &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::Boundary
    ));
}

#[test]
fn periodic_nurbs_elevated_carrier_preserves_wrapped_points_and_derivatives() {
    let curve = NurbsCurve2::try_new_periodic(
        2,
        vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)],
        vec![r(1), r(2), r(3), r(4)],
        (0..=4).map(r).collect(),
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let elevated = curve
        .elevated_to_degree(3, &CurveContext::STRICT)
        .unwrap()
        .into_value();

    assert_eq!(elevated.degree(), 3);
    assert_eq!(elevated.period(), curve.period());
    assert_eq!(elevated.start(), elevated.end());
    for parameter in [r(-3), q(1, 2), q(7, 2), r(4), q(17, 2)] {
        assert_eq!(
            elevated.point_at_wrapped(&parameter, &CurveContext::STRICT),
            curve.point_at_wrapped(&parameter, &CurveContext::STRICT)
        );
        assert_eq!(
            elevated.derivative_at_wrapped(&parameter, &CurveContext::STRICT),
            curve.derivative_at_wrapped(&parameter, &CurveContext::STRICT)
        );
    }
}

#[test]
fn nurbs_degree_elevation_retains_contextual_invalid_target_and_projective_blocker() {
    let curve = quadratic_nurbs();
    let invalid = curve
        .degree_elevation(1, &CurveContext::STRICT)
        .unwrap_err();
    assert_eq!(invalid.operation(), CurveOperation2::DegreeElevation);
    assert_eq!(invalid.family(), CurveFamily2::Nurbs);

    let singular = NurbsCurve2::try_new(
        1,
        vec![p(0, 0), p(2, 0)],
        vec![r(1), r(-1)],
        vec![r(0), r(0), r(1), r(1)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let blocked = singular
        .degree_elevation(2, &CurveContext::STRICT)
        .unwrap_err();
    assert_eq!(blocked.operation(), CurveOperation2::DegreeElevation);
    assert_eq!(blocked.family(), CurveFamily2::Nurbs);
    assert_eq!(
        singular
            .degree_elevation(2, &CurveContext::STRICT)
            .unwrap_err(),
        blocked
    );
}

#[test]
fn out_of_domain_nurbs_knot_insertion_has_contextual_error() {
    let curve = quadratic_nurbs();
    let error = curve.insert_knot(r(3), &CurveContext::STRICT).unwrap_err();

    assert_eq!(error.operation(), CurveOperation2::KnotInsertion);
    assert_eq!(error.family(), CurveFamily2::Nurbs);
}

#[test]
fn nurbs_split_and_subcurve_preserve_authored_parameters_and_exact_image() {
    let curve = NurbsCurve2::try_new(
        2,
        vec![p(0, 0), p(2, 4), p(4, 0)],
        vec![r(1), r(2), r(1)],
        vec![r(0), r(0), r(0), r(2), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    let (left, right) = curve
        .split_at(r(1), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(left.parameter_domain(), (&r(0), &r(1)));
    assert_eq!(right.parameter_domain(), (&r(1), &r(2)));
    assert_eq!(left.end(), right.start());
    assert_eq!(
        left.end(),
        &curve
            .point_at(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
    assert_eq!(
        left.point_at(&q(1, 2), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .point_at(&q(1, 2), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
    assert_eq!(
        right
            .point_at(&q(3, 2), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .point_at(&q(3, 2), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );

    let middle = curve
        .subcurve(q(1, 2), q(3, 2), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(middle.parameter_domain(), (&q(1, 2), &q(3, 2)));
    assert_eq!(
        middle.start(),
        &curve
            .point_at(&q(1, 2), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
    assert_eq!(
        middle.end(),
        &curve
            .point_at(&q(3, 2), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
    assert_eq!(
        middle
            .point_at(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .point_at(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
}

#[test]
fn nurbs_reversal_preserves_domain_source_and_exact_parameter_mapping() {
    let curve = quadratic_nurbs();
    let reversed = curve.reversed(&CurveContext::STRICT).unwrap().into_value();

    assert_eq!(reversed.parameter_domain(), curve.parameter_domain());
    assert_eq!(reversed.start(), curve.end());
    assert_eq!(reversed.end(), curve.start());
    assert_eq!(
        reversed
            .point_at(&q(1, 2), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .point_at(&q(3, 2), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
    let forward_derivative = curve
        .derivative_at(&q(3, 2), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let reverse_derivative = reversed
        .derivative_at(&q(1, 2), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(reverse_derivative.dx(), &(-forward_derivative.dx()));
    assert_eq!(reverse_derivative.dy(), &(-forward_derivative.dy()));
    assert_eq!(
        reversed
            .reversed(&CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
    );
}

#[test]
fn invalid_nurbs_split_and_trim_ranges_evidence_subdivision_context() {
    let curve = quadratic_nurbs();
    for error in [
        curve.split_at(r(0), &CurveContext::STRICT).unwrap_err(),
        curve
            .subcurve(r(1), r(1), &CurveContext::STRICT)
            .unwrap_err(),
        curve
            .subcurve(r(-1), r(1), &CurveContext::STRICT)
            .unwrap_err(),
    ] {
        assert_eq!(error.operation(), CurveOperation2::Subdivision);
        assert_eq!(error.family(), CurveFamily2::Nurbs);
    }
}

#[test]
fn top_level_nurbs_retains_source_and_exact_geometry_under_explicit_policy() {
    let curve = quadratic_nurbs();

    assert_eq!(curve.degree(), 2);
    assert_eq!(curve.start(), &p(0, 0));
    assert_eq!(curve.end(), &p(6, 0));
    assert_eq!(curve.control_points().len(), 4);
    assert_eq!(curve.weights(), &[r(1), r(2), r(4), r(1)]);
}

#[test]
fn nurbs_clones_share_one_retained_bezier_decomposition() {
    let curve = quadratic_nurbs();
    let clone = curve.clone();

    let first = curve
        .bezier_decomposition(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    let second = clone
        .bezier_decomposition(&CurveContext::STRICT)
        .unwrap()
        .into_value();

    assert!(std::ptr::eq(first, second));
    assert_eq!(first.spans().len(), 2);
    assert_eq!(first.inserted_knot_count(), 1);
}

#[test]
fn native_nurbs_spans_are_cached_and_borrowed() {
    let curve = quadratic_nurbs();

    let first = curve
        .native_subcurves(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    let first_ptr = first.as_ptr();
    let second = curve
        .native_subcurves(&CurveContext::STRICT)
        .unwrap()
        .into_value();

    assert_eq!(first_ptr, second.as_ptr());
    assert_eq!(first.len(), 2);
    assert!(
        first
            .iter()
            .all(|span| matches!(span, BezierSubcurve2::RationalQuadratic(_)))
    );

    let retained = curve
        .bezier_spans(&CurveContext::STRICT)
        .unwrap()
        .into_value()
        .collect::<Vec<_>>();
    let promoted = curve
        .native_spans(&CurveContext::STRICT)
        .unwrap()
        .into_value()
        .collect::<Vec<_>>();
    assert_eq!(retained.len(), 2);
    assert_eq!(retained[0].span_index(), 0);
    assert_eq!(retained[1].span_index(), 1);
    assert_eq!(retained[0].knot_interval(), (&r(0), &r(1)));
    assert_eq!(retained[1].knot_interval(), (&r(1), &r(2)));
    assert!(std::ptr::eq(
        retained[0].retained_span(),
        curve
            .bezier_decomposition(&CurveContext::STRICT)
            .unwrap()
            .into_value()
            .spans()
            .first()
            .unwrap()
    ));
    assert_eq!(promoted[1].source_span().span_index(), 1);
    assert!(std::ptr::eq(promoted[0].curve(), &first[0]));
}

#[test]
fn nurbs_evaluation_reuses_decomposition_and_preserves_exact_coordinates() {
    let curve = quadratic_nurbs();

    assert_eq!(
        curve
            .point_at(&r(0), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(0, 0)
    );
    let join = curve
        .point_at(&r(1), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(join.x(), &(Real::from(10) / Real::from(3)).unwrap());
    assert_eq!(join.y(), &r(4));
    assert_eq!(
        curve
            .point_at(&r(2), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(6, 0)
    );
    assert_eq!(
        curve
            .point_at(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        join
    );
}

#[test]
fn out_of_domain_nurbs_evaluation_has_contextual_error() {
    let curve = quadratic_nurbs();

    let error = curve.point_at(&r(3), &CurveContext::STRICT).unwrap_err();

    assert_eq!(error.operation(), CurveOperation2::Evaluation);
    assert_eq!(error.family(), CurveFamily2::Nurbs);
    assert!(matches!(
        error,
        ExactCurveError::Invalid {
            cause: hypercurve::CurveError::InvalidCurveParameter,
            ..
        }
    ));
}

#[test]
fn unequal_weight_cubic_nurbs_promotes_once_with_provenance() {
    let curve = NurbsCurve2::try_new(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(1), r(2), r(4), r(8), r(16)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    let first = curve
        .native_subcurves(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    let first_pointer = first.as_ptr();
    let second = curve
        .native_subcurves(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(first_pointer, second.as_ptr());
    assert_eq!(first.len(), 2);
    assert!(
        first
            .iter()
            .all(|span| matches!(span, BezierSubcurve2::Rational(_)))
    );

    let spans = curve
        .native_spans(&CurveContext::STRICT)
        .unwrap()
        .into_value()
        .collect::<Vec<_>>();
    assert_eq!(spans.len(), 2);
}

#[test]
fn higher_degree_nurbs_promotes_evaluates_and_splits_exactly() {
    let curve = NurbsCurve2::try_new(
        4,
        vec![p(0, 0), p(1, 4), p(2, 0), p(3, 4), p(4, 0)],
        vec![r(1); 5],
        [vec![r(0); 5], vec![r(1); 5]].concat(),
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    assert_eq!(curve.degree(), 4);
    assert_eq!(
        curve
            .point_at(&q(1, 2), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(2, 2)
    );
    let spans = curve
        .native_spans(&CurveContext::STRICT)
        .unwrap()
        .into_value()
        .collect::<Vec<_>>();
    assert_eq!(spans.len(), 1);
    assert_eq!(spans[0].source_span().degree(), 4);
    assert!(matches!(spans[0].curve(), BezierSubcurve2::Rational(_)));

    let (left, right) = curve
        .split_at(q(1, 2), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(left.end(), &p(2, 2));
    assert_eq!(right.start(), &p(2, 2));
}

#[test]
fn unclamped_nurbs_retains_active_endpoints_and_exact_editing() {
    let curve = NurbsCurve2::try_new(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(1); 4],
        (0..=6).map(r).collect(),
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    assert_eq!(curve.parameter_domain(), (&r(2), &r(4)));
    assert_eq!(curve.start(), &Point2::new(r(1), r(2)));
    assert_eq!(curve.end(), &Point2::new(r(5), r(2)));
    assert_eq!(
        curve
            .point_at(&r(3), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(3, 4)
    );

    let inserted = curve
        .insert_knot(r(3), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(inserted.start(), curve.start());
    assert_eq!(inserted.end(), curve.end());
    assert_eq!(
        inserted
            .point_at(&r(3), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(3, 4)
    );

    let (left, right) = curve
        .split_at(r(3), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(left.parameter_domain(), (&r(2), &r(3)));
    assert_eq!(right.parameter_domain(), (&r(3), &r(4)));
    assert_eq!(left.start(), curve.start());
    assert_eq!(left.end(), &p(3, 4));
    assert_eq!(right.start(), &p(3, 4));
    assert_eq!(right.end(), curve.end());

    let reversed = curve.reversed(&CurveContext::STRICT).unwrap().into_value();
    assert_eq!(reversed.start(), curve.end());
    assert_eq!(reversed.end(), curve.start());
    assert_eq!(
        reversed
            .point_at(&r(3), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .point_at(&r(3), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
}

#[test]
fn unclamped_weighted_nurbs_projects_homogeneous_endpoint_evidence() {
    let curve = NurbsCurve2::try_new(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(1), r(2), r(3), r(4)],
        (0..=6).map(r).collect(),
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    assert_eq!(curve.start(), &Point2::new(q(4, 3), q(8, 3)));
    assert_eq!(curve.end(), &Point2::new(q(36, 7), q(12, 7)));
    assert_eq!(
        curve
            .point_at(&r(2), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve.start().clone()
    );
    assert_eq!(
        curve
            .point_at(&r(4), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve.end().clone()
    );
    assert!(
        curve
            .bezier_decomposition(&CurveContext::STRICT)
            .unwrap()
            .into_value()
            .refined_weights()
            .iter()
            .all(|weight| weight.zero_status() == hyperreal::ZeroKnowledge::NonZero)
    );
}

#[test]
fn invalid_nurbs_construction_returns_contextual_error() {
    let error = NurbsCurve2::try_new(
        1,
        vec![p(0, 0), p(1, 1)],
        vec![r(1), r(1)],
        vec![r(0), r(0), r(1)],
        &CurveContext::STRICT,
    )
    .unwrap_err();

    assert_eq!(error.operation(), CurveOperation2::Construction);
    assert_eq!(error.family(), CurveFamily2::Nurbs);
    assert!(matches!(
        error,
        ExactCurveError::Invalid {
            cause: hypercurve::CurveError::InvalidBSpline,
            ..
        }
    ));
}

#[test]
fn periodic_nurbs_wraps_exact_points_derivatives_and_retains_source() {
    let curve = NurbsCurve2::try_new_periodic(
        2,
        vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)],
        vec![r(1), r(1), r(1), r(1)],
        (0..=4).map(r).collect(),
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    assert_eq!(curve.period(), Some(&r(4)));
    assert!(matches!(
        curve.periodicity(),
        SplinePeriodicity2::Periodic { .. }
    ));
    assert_eq!(curve.parameter_domain(), (&r(0), &r(4)));
    assert_eq!(curve.control_points().len(), 6);
    assert_eq!(curve.knots().len(), 9);
    assert_eq!(curve.start(), curve.end());
    assert_eq!(
        curve
            .point_at(&r(0), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(1, 0)
    );
    assert_eq!(
        curve
            .point_at_wrapped(&r(-1), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(0, 1)
    );
    assert_eq!(
        curve
            .point_at_wrapped(&r(5), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(2, 1)
    );
    assert_eq!(
        curve
            .point_at_wrapped(&r(9), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(2, 1)
    );
    assert_eq!(
        curve
            .derivatives_at_wrapped(&q(11, 2), 3, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .derivatives_at(&q(3, 2), 3, &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
    assert_eq!(
        curve
            .derivatives_at_wrapped(&r(4), 1, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .derivatives_at(&r(0), 1, &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
}

#[cfg(feature = "predicates")]
#[test]
fn periodic_nurbs_wrapping_obeys_terminal_policy() {
    let curve = NurbsCurve2::try_new_periodic(
        2,
        vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)],
        vec![r(1), r(2), r(3), r(4)],
        (0..=4).map(r).collect(),
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let wrapped_seam = r(4) + undecidable_zero;

    assert!(matches!(
        curve.point_at_wrapped(&wrapped_seam, &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Evaluation
                && blocker.reason() == hypercurve::UncertaintyReason::Ordering
    ));
    let point = curve
        .point_at_wrapped(&wrapped_seam, &CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must resolve an undecidable NURBS seam");
    assert_eq!(
        point.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(point.value, curve.start().clone());

    let derivatives = curve
        .derivatives_at_wrapped_side(
            &wrapped_seam,
            2,
            CurveParameterSide2::Right,
            &CurveContext::APPROXIMATE_512,
        )
        .expect("wrapped NURBS derivatives must share the terminal policy");
    assert_eq!(
        derivatives.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(
        derivatives.value,
        curve
            .derivatives_at_side(&r(0), 2, CurveParameterSide2::Right, &CurveContext::STRICT,)
            .unwrap()
            .into_value()
    );

    let top_level = Curve2::from(curve);
    let top_level_point = top_level
        .as_view()
        .point_at_wrapped(&wrapped_seam, &CurveContext::APPROXIMATE_512)
        .expect("CurveView2 must preserve wrapped NURBS certainty");
    assert_eq!(
        top_level_point.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(top_level_point.value, top_level.start().clone());
}

#[test]
fn periodic_nurbs_editing_preserves_period_only_for_whole_curve_operations() {
    let curve = NurbsCurve2::try_new_periodic(
        2,
        vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)],
        vec![r(1), r(2), r(3), r(4)],
        (0..=4).map(r).collect(),
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    let inserted = curve
        .insert_knots(vec![q(1, 2), q(3, 2)], &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(inserted.period(), curve.period());
    assert_eq!(
        inserted
            .point_at_wrapped(&r(5), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .point_at_wrapped(&r(5), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );

    let elevated = curve
        .degree_elevation(3, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(elevated.source_degree(), 2);
    assert_eq!(elevated.target_degree(), 3);
    assert_eq!(
        elevated.spans().first().unwrap().parameter_interval().0,
        &r(0)
    );
    assert_eq!(
        elevated.spans().last().unwrap().parameter_interval().1,
        &r(4)
    );
    assert_eq!(
        elevated
            .spans()
            .first()
            .unwrap()
            .curve()
            .point_at(&Real::zero(), &CurveContext::STRICT)
            .unwrap(),
        curve.start().clone()
    );
    assert_eq!(
        elevated
            .spans()
            .last()
            .unwrap()
            .curve()
            .point_at(&Real::one(), &CurveContext::STRICT)
            .unwrap(),
        curve.end().clone()
    );

    let reversed = curve.reversed(&CurveContext::STRICT).unwrap().into_value();
    assert_eq!(reversed.period(), curve.period());
    assert_eq!(reversed.start(), reversed.end());
    assert_eq!(
        reversed
            .point_at_wrapped(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .point_at_wrapped(&r(3), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );

    let (left, right) = curve
        .split_at(r(2), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(left.period(), None);
    assert_eq!(right.period(), None);
    assert_ne!(left.start(), left.end());
    assert_ne!(right.start(), right.end());

    let clamped = curve
        .clamped_subcurve(r(0), r(1), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(clamped.period(), None);
    assert_eq!(clamped.parameter_domain(), (&r(0), &r(1)));
    assert_eq!(clamped.knots(), &[r(0), r(0), r(0), r(1), r(1), r(1)]);
    assert_eq!(
        clamped.start(),
        &curve
            .point_at(&r(0), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
    assert_eq!(
        clamped.end(),
        &curve
            .point_at(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
}

#[test]
fn nonuniform_weighted_periodic_nurbs_supports_repeated_interior_knots() {
    let curve = NurbsCurve2::try_new_periodic(
        2,
        vec![p(0, 0), p(3, 0), p(4, 2), p(2, 5), p(-1, 2)],
        vec![r(1), r(2), r(5), r(3), r(4)],
        vec![r(0), r(1), r(1), r(3), r(5), r(8)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let parameter = q(5, 2);
    let shifted = q(21, 2);

    assert_eq!(curve.period(), Some(&r(8)));
    assert_eq!(curve.start(), curve.end());
    assert_eq!(
        curve
            .point_at_wrapped(&shifted, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .point_at(&parameter, &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
    assert_eq!(
        curve
            .derivative_at_wrapped(&shifted, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .derivative_at(&parameter, &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
    assert!(
        curve
            .bezier_spans(&CurveContext::STRICT)
            .unwrap()
            .into_value()
            .len()
            >= 4
    );
}

#[test]
fn periodic_nurbs_rejects_invalid_layout_and_nonperiodic_wrapping() {
    let invalid = NurbsCurve2::try_new_periodic(
        2,
        vec![p(0, 0), p(1, 0), p(1, 1)],
        vec![r(1), r(1)],
        (0..=3).map(r).collect(),
        &CurveContext::STRICT,
    )
    .unwrap_err();
    assert!(matches!(
        invalid,
        ExactCurveError::Invalid {
            cause: CurveError::InvalidPeriodicSpline,
            ..
        }
    ));

    let open = quadratic_nurbs();
    let error = open
        .point_at_wrapped(&r(3), &CurveContext::STRICT)
        .unwrap_err();
    assert_eq!(error.operation(), CurveOperation2::Evaluation);
    assert!(matches!(
        error,
        ExactCurveError::Invalid {
            cause: CurveError::CurveIsNotPeriodic,
            ..
        }
    ));
}
