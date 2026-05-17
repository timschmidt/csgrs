use hypercurve::{
    Axis2, BezierAreaMomentPrefixSums2, BezierAreaPrefixSums2, BezierCurveRelation,
    BezierCuspClassification, BezierDegree, BezierEndpoint, BezierFlatteningOptions,
    BezierInflectionClassification, BezierLineFitRelation, BezierLineImageFitRelation,
    BezierLineRelation, Classification, CubicBezier2, CurvePolicy, IntersectionKind,
    LineLineIntersection, LineSeg2, Point2, QuadraticBezier2, RationalQuadraticBezier2,
    RationalQuadraticConicKind, Real, SymbolicDependencyMask, UncertaintyReason, ZeroStatus,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn half() -> Real {
    (Real::one() / Real::from(2_i8)).unwrap()
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

#[test]
fn quadratic_bezier_facts_preserve_exact_control_polygon_shape() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let facts = curve.structural_facts();

    assert_eq!(facts.degree, BezierDegree::Quadratic);
    assert_eq!(facts.degree.control_point_count(), 3);
    assert!(facts.all_exact_rational());
    assert!(facts.has_shared_denominator_schedule());
    assert_eq!(facts.endpoint_coincidence, ZeroStatus::NonZero);
    assert_eq!(facts.endpoint_delta_known_zero_mask, 0b10);
    assert_eq!(facts.derivative_known_zero_mask, 0b1010);
    assert!(facts.derivative_y_components_known_zero());
    assert_eq!(facts.second_difference_known_zero_mask, 0b11);
    assert_eq!(facts.curvature_known_zero_mask, 0b1);
}

#[test]
fn cubic_bezier_facts_retain_inflection_candidates_without_float_predicates() {
    let curve = CubicBezier2::new(point(0, 0), point(1, 2), point(2, -2), point(3, 0));
    let facts = curve.structural_facts();

    assert_eq!(facts.degree, BezierDegree::Cubic);
    assert_eq!(facts.degree.control_point_count(), 4);
    assert!(facts.all_exact_rational());
    assert_eq!(facts.endpoint_coincidence, ZeroStatus::NonZero);
    assert_eq!(facts.endpoint_delta_known_zero_mask, 0b10);
    assert_ne!(
        facts.curvature_known_zero_mask, 0,
        "parallel derivative-edge pairs should be retained as exact zero witnesses"
    );
    assert_ne!(facts.curvature_known_nonzero_mask, 0);
}

#[test]
fn symbolic_bezier_control_points_are_reported_as_object_facts() {
    let curve = QuadraticBezier2::new(
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(Real::pi(), Real::one()),
        Point2::new(Real::from(2_i8), Real::zero()),
    );
    let facts = curve.structural_facts();

    assert!(!facts.all_exact_rational());
    assert!(
        facts
            .symbolic_dependencies
            .contains(SymbolicDependencyMask::PI)
    );
    assert_eq!(facts.endpoint_coincidence, ZeroStatus::NonZero);
}

#[test]
fn degenerate_loop_bezier_reports_coincident_endpoints_but_preserves_controls() {
    let curve = CubicBezier2::new(point(0, 0), point(5, 1), point(-3, 1), point(0, 0));
    let facts = curve.structural_facts();

    assert_eq!(curve.endpoints_coincident_status(), ZeroStatus::Zero);
    assert!(facts.endpoints_known_coincident());
    assert_eq!(facts.endpoint_delta_known_zero_mask, 0b11);
    assert_ne!(
        facts.derivative_known_nonzero_mask, 0,
        "coincident endpoints must not erase the nonzero interior control polygon"
    );
}

#[test]
fn bezier_point_at_uses_exact_de_casteljau_midpoints() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    assert_eq!(quadratic.point_at(half()), point(2, 2));

    let cubic = CubicBezier2::new(point(0, 0), point(0, 4), point(4, 4), point(4, 0));
    assert_eq!(cubic.point_at(half()), point(2, 3));
}

#[test]
fn endpoint_tangent_classification_uses_exact_derivative_vectors() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(3, 4), point(8, 4));
    let start = quadratic.endpoint_tangent(BezierEndpoint::Start);
    let end = quadratic.endpoint_tangent(BezierEndpoint::End);

    assert_eq!(start.dx(), &r(6));
    assert_eq!(start.dy(), &r(8));
    assert_eq!(start.zero_status(), ZeroStatus::NonZero);
    assert_eq!(end.dx(), &r(10));
    assert_eq!(end.dy(), &r(0));
    assert_eq!(end.zero_status(), ZeroStatus::NonZero);

    let degenerate = CubicBezier2::new(point(0, 0), point(0, 0), point(2, 0), point(5, 0));
    let start = degenerate.endpoint_tangent(BezierEndpoint::Start);
    let end = degenerate.endpoint_tangent(BezierEndpoint::End);

    assert_eq!(start.zero_status(), ZeroStatus::Zero);
    assert_eq!(end.dx(), &r(9));
    assert_eq!(end.dy(), &r(0));
    assert_eq!(end.zero_status(), ZeroStatus::NonZero);
}

#[test]
fn parameterized_point_on_bezier_is_certified_exactly() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    assert_eq!(
        quadratic.contains_point_at_parameter(&point(2, 2), half(), &policy()),
        Classification::Decided(true)
    );
    assert_eq!(
        quadratic.contains_point_at_parameter(&point(2, 3), half(), &policy()),
        Classification::Decided(false)
    );

    let cubic = CubicBezier2::new(point(0, 0), point(0, 4), point(4, 4), point(4, 0));
    assert_eq!(
        cubic.contains_point_at_parameter(&point(2, 3), half(), &policy()),
        Classification::Decided(true)
    );
    assert_eq!(
        cubic.contains_point_at_parameter(&point(2, 2), half(), &policy()),
        Classification::Decided(false)
    );
}

#[test]
fn quadratic_point_on_curve_solves_exact_parameters() {
    let arch = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let Classification::Decided(parameters) = arch.parameters_for_point(&point(2, 2), &policy())
    else {
        panic!("integer midpoint on quadratic should have certified parameters");
    };
    assert!(
        !parameters.is_empty(),
        "quadratic point solver should retain at least one exact parameter"
    );
    for parameter in parameters {
        assert_eq!(
            arch.contains_point_at_parameter(&point(2, 2), parameter, &policy()),
            Classification::Decided(true)
        );
    }
    assert_eq!(
        arch.contains_point(&point(2, 2), &policy()),
        Classification::Decided(true)
    );
    assert_eq!(
        arch.parameters_for_point(&point(2, 3), &policy()),
        Classification::Decided(Vec::new())
    );
    assert_eq!(
        arch.contains_point(&point(2, 3), &policy()),
        Classification::Decided(false)
    );

    let constant = QuadraticBezier2::new(point(5, 5), point(5, 5), point(5, 5));
    assert_eq!(
        constant.parameters_for_point(&point(5, 5), &policy()),
        Classification::Decided(vec![Real::zero()])
    );
    assert_eq!(
        constant.contains_point(&point(6, 5), &policy()),
        Classification::Decided(false)
    );
}

#[test]
fn control_hull_box_contains_all_control_points() {
    let curve = CubicBezier2::new(point(-3, 4), point(2, -5), point(7, 8), point(1, -2));
    let Classification::Decided(bbox) = curve.control_hull_box(&policy()) else {
        panic!("integer control hull should have decided ordering");
    };

    assert_eq!(bbox.min(), &point(-3, -5));
    assert_eq!(bbox.max(), &point(7, 8));
    for control in curve.control_points() {
        assert_eq!(
            bbox.contains_point(control, &policy()),
            Classification::Decided(true)
        );
    }
}

#[test]
fn quadratic_monotone_spans_and_bounds_use_exact_derivative_roots() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));

    assert_eq!(
        curve.axis_monotone_parameters(Axis2::Y, &policy()),
        Classification::Decided(vec![half()])
    );
    let Classification::Decided(spans) = curve.monotone_spans(&policy()) else {
        panic!("integer quadratic monotone roots should be decided");
    };
    assert_eq!(spans.len(), 2);
    assert_eq!(spans[0].start(), &Real::zero());
    assert_eq!(spans[0].end(), &half());
    assert_eq!(spans[1].start(), &half());
    assert_eq!(spans[1].end(), &Real::one());

    let Classification::Decided(bounds) = curve.certified_bounds(&policy()) else {
        panic!("integer quadratic bounds should be decided");
    };
    assert_eq!(bounds.min(), &point(0, 0));
    assert_eq!(bounds.max(), &point(4, 2));
}

#[test]
fn quadratic_line_relation_solves_certified_parameters() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let baseline = LineSeg2::try_new(point(-1, 0), point(5, 0)).unwrap();
    let above = LineSeg2::try_new(point(-1, 5), point(5, 5)).unwrap();

    assert_eq!(
        curve.relation_to_line(&baseline, &policy()),
        Classification::Decided(BezierLineRelation::Intersects {
            parameters: vec![Real::zero(), Real::one()]
        })
    );
    assert_eq!(
        curve.relation_to_line(&above, &policy()),
        Classification::Decided(BezierLineRelation::ControlHullDisjoint {
            side: hypercurve::LineSide::Right
        })
    );
}

#[test]
fn quadratic_cusp_classifier_handles_flat_derivative_component() {
    let cusp = QuadraticBezier2::new(point(0, 0), point(1, 0), point(0, 0));
    assert_eq!(
        cusp.cusp_classification(&policy()),
        Classification::Decided(BezierCuspClassification::Cusps {
            parameters: vec![half()]
        })
    );
    assert_eq!(
        cusp.inflection_classification(),
        BezierInflectionClassification::NotApplicable
    );
}

#[test]
fn cubic_inflection_and_coarse_curve_relation_are_certified() {
    let cubic = CubicBezier2::new(point(0, 0), point(1, 2), point(2, -2), point(3, 0));
    let Classification::Decided(BezierInflectionClassification::Inflections { parameters }) =
        cubic.inflection_classification(&policy())
    else {
        panic!("symmetric integer cubic should expose a certified inflection");
    };
    assert!(parameters.contains(&half()));

    let far = CubicBezier2::new(point(10, 10), point(11, 12), point(12, 8), point(13, 10));
    assert_eq!(
        cubic.relation_to_cubic(&far, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
    assert_eq!(
        cubic.relation_to_cubic(&cubic, &policy()),
        Classification::Decided(BezierCurveRelation::SameControlPolygon)
    );
}

#[test]
fn bezier_length_bounds_use_exact_chord_and_control_polygon_interval() {
    let line_quadratic = QuadraticBezier2::new(point(0, 0), point(3, 4), point(6, 8));
    let line_bounds = line_quadratic.length_bounds().unwrap();
    assert_eq!(line_bounds.lower(), &Real::from(10_i8));
    assert_eq!(line_bounds.upper(), &Real::from(10_i8));
    assert_eq!(line_bounds.width(), Real::zero());
    assert!(line_bounds.is_exact());

    let loop_quadratic = QuadraticBezier2::new(point(0, 0), point(3, 4), point(0, 0));
    let loop_bounds = loop_quadratic.length_bounds().unwrap();
    assert_eq!(loop_bounds.lower(), &Real::zero());
    assert_eq!(loop_bounds.upper(), &Real::from(10_i8));
    assert!(!loop_bounds.is_exact());

    let line_cubic = CubicBezier2::new(point(0, 0), point(2, 0), point(4, 0), point(6, 0));
    let cubic_bounds = line_cubic.length_bounds().unwrap();
    assert_eq!(cubic_bounds.lower(), &Real::from(6_i8));
    assert_eq!(cubic_bounds.upper(), &Real::from(6_i8));
    assert!(cubic_bounds.is_exact());
}

#[test]
fn refined_bezier_length_bounds_tighten_by_exact_subdivision() {
    let curve = QuadraticBezier2::new(point(0, 0), point(3, 4), point(6, 0));
    let base = curve.length_bounds().unwrap();
    let refined = curve.refined_length_bounds(1).unwrap();
    let two = Real::from(2_i8);
    let sqrt_thirteen = Real::from(13_i8).sqrt().unwrap();

    assert_eq!(base.lower(), &Real::from(6_i8));
    assert_eq!(base.upper(), &Real::from(10_i8));
    assert_eq!(refined.lower(), &(&two * sqrt_thirteen));
    assert_eq!(refined.upper(), &Real::from(8_i8));
    assert_ne!(refined, base);
    assert_eq!(curve.refined_length_bounds(0).unwrap(), base);
}

#[test]
fn prefix_bezier_length_bounds_split_exactly_at_parameter() {
    let curve = QuadraticBezier2::new(point(0, 0), point(3, 4), point(6, 0));
    let prefix = curve
        .prefix_length_bounds(half(), &policy())
        .unwrap()
        .unwrap_decided_for_test();
    let refined_prefix = curve
        .refined_prefix_length_bounds(half(), 0, &policy())
        .unwrap()
        .unwrap_decided_for_test();
    let five_halves = (Real::from(5_i8) / Real::from(2_i8)).unwrap();
    let three_halves = (Real::from(3_i8) / Real::from(2_i8)).unwrap();
    let sqrt_thirteen = Real::from(13_i8).sqrt().unwrap();

    assert_eq!(prefix.lower(), &sqrt_thirteen);
    assert_eq!(prefix.upper(), &(&five_halves + &three_halves));
    assert_eq!(refined_prefix, prefix);
    assert_eq!(
        curve
            .prefix_length_bounds(Real::from(2_i8), &policy())
            .unwrap_err(),
        hypercurve::CurveError::InvalidBezierParameter
    );
}

#[test]
fn inverse_length_parameter_region_is_certified_not_sampled() {
    let line = CubicBezier2::new(point(0, 0), point(2, 0), point(4, 0), point(6, 0));
    let region = line
        .inverse_length_parameter_region(Real::from(3_i8), 8, 2, &policy())
        .unwrap()
        .unwrap_decided_for_test();

    assert_eq!(region.target_length(), &Real::from(3_i8));
    assert_eq!(region.parameter_span().start(), &half());
    assert_eq!(region.parameter_span().end(), &half());
    assert_eq!(
        region.prefix_bounds_at_span_end().lower(),
        &Real::from(3_i8)
    );
    assert_eq!(
        region.prefix_bounds_at_span_end().upper(),
        &Real::from(3_i8)
    );

    assert_eq!(
        line.inverse_length_parameter_region(Real::from(-1_i8), 8, 2, &policy())
            .unwrap_err(),
        hypercurve::CurveError::InvalidBezierArcLengthTarget
    );
    assert_eq!(
        line.inverse_length_parameter_region(Real::from(7_i8), 8, 2, &policy())
            .unwrap_err(),
        hypercurve::CurveError::InvalidBezierArcLengthTarget
    );
}

#[test]
fn bezier_signed_area_contribution_integrates_green_boundary_exactly() {
    let line_quadratic = QuadraticBezier2::new(point(1, 1), point(3, 2), point(5, 3));
    assert_eq!(
        line_quadratic.signed_area_contribution().unwrap(),
        Real::from(-1_i8)
    );

    let arch = QuadraticBezier2::new(point(0, 0), point(3, 4), point(6, 0));
    assert_eq!(arch.signed_area_contribution().unwrap(), Real::from(-8_i8));
    assert_eq!(
        arch.prefix_signed_area_contribution(half(), &policy())
            .unwrap()
            .unwrap_decided_for_test(),
        Real::from(-1_i8)
    );
    assert_eq!(
        arch.prefix_signed_area_contribution(Real::from(-1_i8), &policy())
            .unwrap_err(),
        hypercurve::CurveError::InvalidBezierParameter
    );

    let cubic_line = CubicBezier2::new(point(1, 1), point(2, 1), point(4, 1), point(5, 1));
    assert_eq!(
        cubic_line.signed_area_contribution().unwrap(),
        Real::from(-2_i8)
    );
}

#[test]
fn bezier_area_prefix_sums_answer_range_queries_without_reintegration() {
    let first = QuadraticBezier2::new(point(0, 0), point(3, 4), point(6, 0));
    let second = QuadraticBezier2::new(point(6, 0), point(7, 0), point(8, 0));
    let third = QuadraticBezier2::new(point(8, 0), point(8, 2), point(8, 4));
    let table = BezierAreaPrefixSums2::from_quadratics([&first, &second, &third]).unwrap();

    assert_eq!(table.segment_count(), 3);
    assert_eq!(table.prefixes()[0], Real::zero());
    assert_eq!(table.range_contribution(0..1).unwrap(), Real::from(-8_i8));
    assert_eq!(table.range_contribution(1..1).unwrap(), Real::zero());
    assert_eq!(table.range_contribution(1..3).unwrap(), Real::from(16_i8));
    assert_eq!(table.total(), &Real::from(8_i8));
    assert_eq!(
        table.range_contribution(2..4).unwrap_err(),
        hypercurve::CurveError::InvalidBezierRange
    );

    let contributions = vec![Real::from(3_i8), Real::from(-5_i8), Real::from(7_i8)];
    let direct = BezierAreaPrefixSums2::from_contributions(contributions);
    assert_eq!(direct.range_contribution(0..3).unwrap(), Real::from(5_i8));
    assert_eq!(direct.range_contribution(1..3).unwrap(), Real::from(2_i8));
}

#[test]
fn bezier_area_moments_integrate_green_boundary_identities_exactly() {
    let horizontal = CubicBezier2::new(point(1, 2), point(2, 2), point(4, 2), point(5, 2));
    let moments = horizontal.area_moments_contribution().unwrap();
    assert_eq!(moments.signed_area(), &Real::from(-4_i8));
    assert_eq!(moments.x_moment(), &Real::zero());
    assert_eq!(moments.y_moment(), &Real::from(-8_i8));

    let vertical = QuadraticBezier2::new(point(3, 1), point(3, 3), point(3, 5));
    let moments = vertical.area_moments_contribution().unwrap();
    assert_eq!(moments.signed_area(), &Real::from(6_i8));
    assert_eq!(moments.x_moment(), &Real::from(18_i8));
    assert_eq!(moments.y_moment(), &Real::zero());

    let prefix = horizontal
        .prefix_area_moments_contribution(half(), &policy())
        .unwrap()
        .unwrap_decided_for_test();
    assert_eq!(prefix.signed_area(), &Real::from(-2_i8));
    assert_eq!(prefix.x_moment(), &Real::zero());
    assert_eq!(prefix.y_moment(), &Real::from(-4_i8));
    assert_eq!(
        horizontal
            .prefix_area_moments_contribution(Real::from(2_i8), &policy())
            .unwrap_err(),
        hypercurve::CurveError::InvalidBezierParameter
    );
}

#[test]
fn bezier_area_moment_prefix_sums_answer_range_queries_without_reintegration() {
    let first = CubicBezier2::new(point(1, 2), point(2, 2), point(4, 2), point(5, 2));
    let second = CubicBezier2::new(point(5, 2), point(5, 3), point(5, 4), point(5, 6));
    let third = CubicBezier2::new(point(5, 6), point(3, 6), point(2, 6), point(1, 6));
    let table = BezierAreaMomentPrefixSums2::from_cubics([&first, &second, &third]).unwrap();

    assert_eq!(table.segment_count(), 3);
    assert_eq!(table.prefixes()[0].signed_area(), &Real::zero());
    assert_eq!(
        table.range_contribution(0..1).unwrap(),
        first.area_moments_contribution().unwrap()
    );
    let empty = table.range_contribution(1..1).unwrap();
    assert_eq!(empty.signed_area(), &Real::zero());
    assert_eq!(
        table.range_contribution(2..4).unwrap_err(),
        hypercurve::CurveError::InvalidBezierRange
    );

    let full = table.range_contribution(0..3).unwrap();
    assert_eq!(table.total(), &full);
}

#[test]
fn mixed_polynomial_bezier_curve_relation_certifies_disjoint_and_shared_endpoint() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(1, 2), point(2, 0));
    let far_cubic = CubicBezier2::new(point(10, 10), point(11, 12), point(12, 8), point(13, 10));
    let touching_cubic = CubicBezier2::new(point(2, 0), point(3, 2), point(4, 2), point(5, 0));

    assert_eq!(
        quadratic.relation_to_cubic(&far_cubic, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
    assert_eq!(
        far_cubic.relation_to_quadratic(&quadratic, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
    assert_eq!(
        quadratic.relation_to_cubic(&touching_cubic, &policy()),
        Classification::Decided(BezierCurveRelation::SharedEndpoint)
    );
    assert_eq!(
        touching_cubic.relation_to_quadratic(&quadratic, &policy()),
        Classification::Decided(BezierCurveRelation::SharedEndpoint)
    );
}

#[test]
fn polynomial_bezier_curve_relation_certifies_endpoint_on_quadratic_interior() {
    let arch = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let endpoint_on_arch = CubicBezier2::new(point(2, 2), point(3, 5), point(5, 5), point(6, 4));

    let relation = endpoint_on_arch.relation_to_quadratic(&arch, &policy());
    let Classification::Decided(BezierCurveRelation::EndpointIntersections { points }) = relation
    else {
        panic!(
            "endpoint on quadratic interior should be certified before subdivision: {relation:?}"
        );
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &point(2, 2));

    assert_eq!(
        arch.relation_to_cubic(&endpoint_on_arch, &policy()),
        Classification::Decided(BezierCurveRelation::EndpointIntersections { points })
    );
}

#[test]
fn polynomial_bezier_curve_relation_solves_same_parameter_quadratic_crossings() {
    let arch = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let crossing = QuadraticBezier2::new(point(0, 2), point(2, -2), point(4, 2));

    let relation = arch.relation_to_quadratic(&crossing, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!("same-parameter quadratic crossings should be solved algebraically: {relation:?}");
    };

    assert_eq!(points.len(), 2);
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        crossing.relation_to_quadratic(&arch, &policy())
    else {
        panic!("same-parameter quadratic crossing relation should be symmetric");
    };
    assert_eq!(points.len(), 2);
}

#[test]
fn polynomial_bezier_curve_relation_certifies_same_axis_monotone_no_hit() {
    let arch = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let raised = QuadraticBezier2::new(point(0, 1), point(2, 5), point(4, 1));

    assert_eq!(
        arch.relation_to_quadratic(&raised, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );
    assert_eq!(
        raised.relation_to_quadratic(&arch, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );
}

#[test]
fn polynomial_bezier_curve_relation_degree_normalizes_same_axis_no_hit() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(3, 6), point(6, 0));
    let elevated_raised = CubicBezier2::new(point(0, 1), point(2, 5), point(4, 5), point(6, 1));

    assert_eq!(
        quadratic.relation_to_cubic(&elevated_raised, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );
    assert_eq!(
        elevated_raised.relation_to_quadratic(&quadratic, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );
}

#[test]
fn polynomial_bezier_curve_relation_certifies_degree_elevated_same_image() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(3, 6), point(6, 0));
    let elevated = CubicBezier2::new(point(0, 0), point(2, 4), point(4, 4), point(6, 0));

    assert_eq!(
        quadratic.relation_to_cubic(&elevated, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
    assert_eq!(
        elevated.relation_to_quadratic(&quadratic, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
}

#[test]
fn polynomial_bezier_curve_relation_certifies_mixed_degree_midpoint_hit() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(3, 6), point(6, 0));
    let cubic = CubicBezier2::new(point(0, 12), point(2, 0), point(4, 0), point(6, 12));

    let relation = quadratic.relation_to_cubic(&cubic, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!("mixed-degree same-parameter midpoint hit should be promoted exactly: {relation:?}");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &point(3, 3));

    assert_eq!(
        cubic.relation_to_quadratic(&quadratic, &policy()),
        Classification::Decided(BezierCurveRelation::IntersectionPoints { points })
    );
}

#[test]
fn polynomial_bezier_curve_relation_certifies_mixed_degree_quarter_hit() {
    let middle_y = (Real::from(62_i8) / Real::from(3_i8)).unwrap();
    let quadratic = QuadraticBezier2::new(
        point(0, 0),
        Point2::new(Real::from(3_i8), middle_y),
        point(6, 0),
    );
    let cubic = CubicBezier2::new(point(0, 16), point(2, 0), point(4, 0), point(6, 64));

    let relation = quadratic.relation_to_cubic(&cubic, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!("mixed-degree same-parameter quarter hit should be promoted exactly: {relation:?}");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(
        points[0].point(),
        &quadratic.point_at((Real::one() / Real::from(4_i8)).unwrap())
    );
}

#[test]
fn polynomial_bezier_curve_relation_certifies_cubic_quarter_hit() {
    let first = CubicBezier2::new(point(0, 0), point(2, 4), point(4, 4), point(6, 0));
    let second = CubicBezier2::new(point(0, 4), point(2, 0), point(4, 0), point(6, 36));

    let quarter = (Real::one() / Real::from(4_i8)).unwrap();
    let relation = first.relation_to_cubic(&second, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!("cubic same-parameter quarter hit should be promoted exactly: {relation:?}");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &first.point_at(quarter));

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        second.relation_to_cubic(&first, &policy())
    else {
        panic!("cubic same-parameter quarter hit should be symmetric");
    };
    assert_eq!(points.len(), 1);
}

#[test]
fn cubic_dyadic_point_solver_promotes_endpoint_on_cubic() {
    let cubic = CubicBezier2::new(point(0, 0), point(2, 4), point(4, 4), point(6, 0));
    let quarter = (Real::one() / Real::from(4_i8)).unwrap();
    let cubic_point = cubic.point_at(quarter.clone());
    let probe = QuadraticBezier2::new(
        Point2::new(cubic_point.x().clone(), cubic_point.y() + Real::from(20_i8)),
        Point2::new(
            cubic_point.x() + Real::one(),
            cubic_point.y() + Real::from(10_i8),
        ),
        cubic_point.clone(),
    );

    assert_eq!(
        cubic.dyadic_parameters_for_point(&cubic_point, &policy()),
        Classification::Decided(vec![quarter])
    );
    assert_eq!(
        cubic.dyadic_parameters_for_point(&point(3, 4), &policy()),
        Classification::Decided(Vec::new())
    );

    let relation = probe.relation_to_cubic(&cubic, &policy());
    let Classification::Decided(BezierCurveRelation::EndpointIntersections { points }) = relation
    else {
        panic!("dyadic endpoint-on-cubic hit should be certified before subdivision: {relation:?}");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &cubic_point);
}

#[test]
fn polynomial_bezier_curve_relation_rejects_prefix_only_shared_axis_proof() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(1, 10), point(2, 0));
    let prefix_matching_cubic =
        CubicBezier2::new(point(0, 1), point(1, 11), point(2, 1), point(100, 1));

    assert_ne!(
        quadratic.relation_to_cubic(&prefix_matching_cubic, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );
    assert_ne!(
        quadratic.relation_to_cubic(&prefix_matching_cubic, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
}

#[test]
fn polynomial_bezier_line_images_use_exact_line_intersection() {
    let horizontal = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let vertical = CubicBezier2::new(point(2, -2), point(2, -1), point(2, 1), point(2, 2));
    let Classification::Decided(BezierCurveRelation::LineSegmentIntersection { intersection }) =
        horizontal.relation_to_cubic(&vertical, &policy())
    else {
        panic!("certified Bezier line images should use native line intersection");
    };
    assert_eq!(
        intersection,
        LineLineIntersection::Point {
            point: point(2, 0),
            a_param: half(),
            b_param: half(),
            kind: IntersectionKind::Crossing,
        }
    );

    let overlap = QuadraticBezier2::new(point(2, 0), point(3, 0), point(4, 0));
    let Classification::Decided(BezierCurveRelation::LineSegmentIntersection {
        intersection: LineLineIntersection::Overlap { segment, .. },
    }) = horizontal.relation_to_quadratic(&overlap, &policy())
    else {
        panic!("collinear Bezier line images should report native overlap");
    };
    assert_eq!(segment.start(), &point(2, 0));
    assert_eq!(segment.end(), &point(4, 0));
}

#[test]
fn polynomial_bezier_line_image_against_curve_uses_supporting_line_roots() {
    let tangent_line = QuadraticBezier2::new(point(0, 2), point(2, 2), point(4, 2));
    let arch = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        tangent_line.relation_to_quadratic(&arch, &policy())
    else {
        panic!("line-image versus quadratic should use exact supporting-line roots");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &point(2, 2));

    assert_eq!(
        arch.relation_to_quadratic(&tangent_line, &policy()),
        Classification::Decided(BezierCurveRelation::IntersectionPoints { points })
    );

    let clipped_line = CubicBezier2::new(point(3, 2), point(3, 2), point(4, 2), point(4, 2));
    assert_eq!(
        clipped_line.relation_to_quadratic(&arch, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );
}

#[test]
fn cubic_line_relation_reports_exact_and_isolated_roots() {
    let line = LineSeg2::try_new(point(-1, 0), point(4, 0)).unwrap();
    let exact_roots = CubicBezier2::new(point(0, 0), point(1, 2), point(2, -2), point(3, 0));

    assert_eq!(
        exact_roots.relation_to_line(&line, &policy()),
        Classification::Decided(BezierLineRelation::Intersects {
            parameters: vec![Real::zero(), half(), Real::one()]
        })
    );

    let isolated_root = CubicBezier2::new(point(0, -1), point(1, 1), point(2, 1), point(3, 1));
    let Classification::Decided(BezierLineRelation::IsolatedIntersections { spans }) =
        isolated_root.relation_to_line(&line, &policy())
    else {
        panic!("non-rational cubic line root should be retained as an isolating span");
    };

    assert_eq!(spans.len(), 1);
    assert_ne!(spans[0].start(), &Real::zero());
    assert_ne!(spans[0].end(), &Real::one());
    assert_ne!(
        spans[0].start(),
        spans[0].end(),
        "non-represented cubic root should remain a nonzero bracket"
    );
}

#[test]
fn rational_quadratic_evaluates_homogeneous_conic_points() {
    let polynomial = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::one(),
    )
    .unwrap();
    assert_eq!(
        polynomial.point_at(half(), &policy()),
        Classification::Decided(point(2, 2))
    );

    let weighted = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let eight_thirds = (Real::from(8_i8) / Real::from(3_i8)).unwrap();
    assert_eq!(
        weighted.point_at(half(), &policy()),
        Classification::Decided(Point2::new(Real::from(2_i8), eight_thirds))
    );
}

#[test]
fn rational_quadratic_classifies_conic_weight_families_and_facts() {
    let ellipse = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(1, 1),
        point(2, 0),
        half(),
    )
    .unwrap();
    let parabola = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(1, 1),
        point(2, 0),
        Real::one(),
    )
    .unwrap();
    let hyperbola = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(1, 1),
        point(2, 0),
        Real::from(2_i8),
    )
    .unwrap();

    assert_eq!(
        ellipse.conic_kind(&policy()),
        Classification::Decided(RationalQuadraticConicKind::EllipseLike)
    );
    assert_eq!(
        parabola.conic_kind(&policy()),
        Classification::Decided(RationalQuadraticConicKind::Parabola)
    );
    assert_eq!(
        hyperbola.conic_kind(&policy()),
        Classification::Decided(RationalQuadraticConicKind::HyperbolaLike)
    );

    let facts = ellipse.structural_facts();
    assert!(facts.all_exact_rational());
    assert_eq!(facts.weight_known_nonzero_mask, 0b111);
    assert_eq!(facts.conic_discriminant_zero_status, ZeroStatus::NonZero);
    assert_eq!(
        RationalQuadraticBezier2::try_unit_end_weights(
            point(0, 0),
            point(1, 1),
            point(2, 0),
            Real::zero()
        ),
        Err(hypercurve::CurveError::ZeroRationalBezierWeight)
    );
}

#[test]
fn rational_quadratic_monotone_spans_and_bounds_use_quotient_derivative() {
    let curve = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::one(),
    )
    .unwrap();

    assert_eq!(
        curve.axis_monotone_parameters(Axis2::Y, &policy()),
        Classification::Decided(vec![half()])
    );
    let Classification::Decided(spans) = curve.monotone_spans(&policy()) else {
        panic!("equal-weight rational quadratic should expose polynomial monotone spans");
    };
    assert_eq!(spans.len(), 2);
    assert_eq!(spans[0].start(), &Real::zero());
    assert_eq!(spans[0].end(), &half());
    assert_eq!(spans[1].start(), &half());
    assert_eq!(spans[1].end(), &Real::one());

    let Classification::Decided(bounds) = curve.certified_bounds(&policy()) else {
        panic!("equal-weight rational quadratic bounds should be decided");
    };
    assert_eq!(bounds.min(), &point(0, 0));
    assert_eq!(bounds.max(), &point(4, 2));
}

#[test]
fn rational_quadratic_certifies_parameterized_points_and_line_relation() {
    let curve = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let midpoint = Point2::new(
        Real::from(2_i8),
        (Real::from(8_i8) / Real::from(3_i8)).unwrap(),
    );
    assert_eq!(
        curve.contains_point_at_parameter(&midpoint, half(), &policy()),
        Classification::Decided(true)
    );
    assert_eq!(
        curve.contains_point_at_parameter(&point(2, 2), half(), &policy()),
        Classification::Decided(false)
    );

    let baseline = LineSeg2::try_new(point(-1, 0), point(5, 0)).unwrap();
    assert_eq!(
        curve.relation_to_line(&baseline, &policy()),
        Classification::Decided(BezierLineRelation::Intersects {
            parameters: vec![Real::zero(), Real::one()]
        })
    );

    let above = LineSeg2::try_new(point(-1, 5), point(5, 5)).unwrap();
    assert_eq!(
        curve.relation_to_line(&above, &policy()),
        Classification::Decided(BezierLineRelation::ControlHullDisjoint {
            side: hypercurve::LineSide::Right
        })
    );
}

#[test]
fn rational_quadratic_point_on_curve_solves_exact_parameters() {
    let curve = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 3),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();

    let Classification::Decided(parameters) = curve.parameters_for_point(&point(2, 2), &policy())
    else {
        panic!("integer point on positive-weight conic should have certified parameters");
    };
    assert!(
        !parameters.is_empty(),
        "rational point solver should retain at least one exact parameter"
    );
    for parameter in parameters {
        assert_eq!(
            curve.contains_point_at_parameter(&point(2, 2), parameter, &policy()),
            Classification::Decided(true)
        );
    }
    assert_eq!(
        curve.contains_point(&point(2, 2), &policy()),
        Classification::Decided(true)
    );
    assert_eq!(
        curve.parameters_for_point(&point(2, 3), &policy()),
        Classification::Decided(Vec::new())
    );
    assert_eq!(
        curve.contains_point(&point(2, 3), &policy()),
        Classification::Decided(false)
    );

    let constant = RationalQuadraticBezier2::try_unit_end_weights(
        point(5, 5),
        point(5, 5),
        point(5, 5),
        Real::from(2_i8),
    )
    .unwrap();
    assert_eq!(
        constant.parameters_for_point(&point(5, 5), &policy()),
        Classification::Decided(vec![Real::zero()])
    );
}

#[test]
fn rational_quadratic_reports_projective_denominator_boundary() {
    let curve = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::one(),
        -Real::one(),
        Real::one(),
    )
    .unwrap();

    assert_eq!(
        curve.point_at(half(), &policy()),
        Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
    );
    assert_eq!(
        curve.contains_point_at_parameter(&point(2, 2), half(), &policy()),
        Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
    );
}

#[test]
fn rational_quadratic_curve_relation_certifies_identity_disjoint_and_shared_endpoint() {
    let first = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(1, 2),
        point(2, 0),
        half(),
    )
    .unwrap();
    let far = RationalQuadraticBezier2::try_unit_end_weights(
        point(10, 10),
        point(11, 12),
        point(12, 10),
        half(),
    )
    .unwrap();
    let touching = RationalQuadraticBezier2::try_unit_end_weights(
        point(2, 0),
        point(3, 2),
        point(4, 0),
        half(),
    )
    .unwrap();

    assert_eq!(
        first.relation_to_rational_quadratic(&first, &policy()),
        Classification::Decided(BezierCurveRelation::SameControlPolygon)
    );
    assert_eq!(
        first.relation_to_rational_quadratic(&far, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
    assert_eq!(
        first.relation_to_rational_quadratic(&touching, &policy()),
        Classification::Decided(BezierCurveRelation::SharedEndpoint)
    );
}

#[test]
fn rational_quadratic_curve_relation_certifies_projective_weight_identity() {
    let first = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(1_i8),
        Real::from(2_i8),
        Real::from(3_i8),
    )
    .unwrap();
    let scaled = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(5_i8),
        Real::from(10_i8),
        Real::from(15_i8),
    )
    .unwrap();
    let non_proportional = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(1_i8),
        Real::from(2_i8),
        Real::from(4_i8),
    )
    .unwrap();

    assert_eq!(
        first.relation_to_rational_quadratic(&scaled, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
    assert_eq!(
        scaled.relation_to_rational_quadratic(&first, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
    assert_ne!(
        first.relation_to_rational_quadratic(&non_proportional, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
}

#[test]
fn rational_curve_relations_isolate_overlapping_regions() {
    let first = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        half(),
    )
    .unwrap();
    let second = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 2),
        point(2, -2),
        point(4, 2),
        half(),
    )
    .unwrap();
    let polynomial = QuadraticBezier2::new(point(0, 2), point(2, -2), point(4, 2));

    let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) =
        first.relation_to_rational_quadratic(&second, &policy())
    else {
        panic!("overlapping positive-weight conics should retain subdivision regions");
    };
    assert!(!regions.is_empty());

    let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) =
        first.relation_to_quadratic(&polynomial, &policy())
    else {
        panic!("overlapping rational/polynomial curves should retain subdivision regions");
    };
    assert!(!regions.is_empty());
    assert_eq!(
        polynomial.relation_to_rational_quadratic(&first, &policy()),
        first.relation_to_quadratic(&polynomial, &policy())
    );
}

#[test]
fn equal_weight_rational_relations_reuse_polynomial_quadratic_dispatch() {
    let rational_arch = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::one(),
    )
    .unwrap();
    let rational_crossing = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 2),
        point(2, -2),
        point(4, 2),
        Real::one(),
    )
    .unwrap();
    let polynomial_crossing = QuadraticBezier2::new(point(0, 2), point(2, -2), point(4, 2));

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        rational_arch.relation_to_rational_quadratic(&rational_crossing, &policy())
    else {
        panic!("equal-weight rational quadratics should reuse polynomial intersection dispatch");
    };
    assert_eq!(points.len(), 2);

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        rational_arch.relation_to_quadratic(&polynomial_crossing, &policy())
    else {
        panic!("equal-weight rational/polynomial relation should reuse polynomial dispatch");
    };
    assert_eq!(points.len(), 2);
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        polynomial_crossing.relation_to_rational_quadratic(&rational_arch, &policy())
    else {
        panic!("polynomial/rational equal-weight relation should be symmetric");
    };
    assert_eq!(points.len(), 2);
}

#[test]
fn rational_relations_certify_endpoint_on_conic_interior() {
    let rational_arch = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 3),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let polynomial_probe = CubicBezier2::new(point(2, 2), point(3, 5), point(5, 5), point(6, 4));
    let rational_probe = RationalQuadraticBezier2::try_unit_end_weights(
        point(2, 2),
        point(3, 5),
        point(6, 4),
        Real::from(2_i8),
    )
    .unwrap();

    let relation = polynomial_probe.relation_to_rational_quadratic(&rational_arch, &policy());
    let Classification::Decided(BezierCurveRelation::EndpointIntersections { points }) = relation
    else {
        panic!("polynomial endpoint on rational conic interior should be certified: {relation:?}");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &point(2, 2));

    assert_eq!(
        rational_arch.relation_to_cubic(&polynomial_probe, &policy()),
        Classification::Decided(BezierCurveRelation::EndpointIntersections { points })
    );

    let relation = rational_probe.relation_to_rational_quadratic(&rational_arch, &policy());
    let Classification::Decided(BezierCurveRelation::EndpointIntersections { points }) = relation
    else {
        panic!("rational endpoint on rational conic interior should be certified: {relation:?}");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &point(2, 2));
}

#[test]
fn rational_line_image_relations_use_exact_line_intersection() {
    let horizontal = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 0),
        point(4, 0),
        half(),
    )
    .unwrap();
    let vertical = RationalQuadraticBezier2::try_unit_end_weights(
        point(2, -2),
        point(2, 0),
        point(2, 2),
        Real::from(2_i8),
    )
    .unwrap();
    let Classification::Decided(BezierCurveRelation::LineSegmentIntersection { intersection }) =
        horizontal.relation_to_rational_quadratic(&vertical, &policy())
    else {
        panic!("certified rational line images should use native line intersection");
    };
    assert_eq!(
        intersection,
        LineLineIntersection::Point {
            point: point(2, 0),
            a_param: half(),
            b_param: half(),
            kind: IntersectionKind::Crossing,
        }
    );

    let polynomial_vertical =
        CubicBezier2::new(point(2, -2), point(2, -1), point(2, 1), point(2, 2));
    assert_eq!(
        horizontal.relation_to_cubic(&polynomial_vertical, &policy()),
        Classification::Decided(BezierCurveRelation::LineSegmentIntersection {
            intersection: LineLineIntersection::Point {
                point: point(2, 0),
                a_param: half(),
                b_param: half(),
                kind: IntersectionKind::Crossing,
            }
        })
    );
    assert_eq!(
        polynomial_vertical.relation_to_rational_quadratic(&horizontal, &policy()),
        horizontal.relation_to_cubic(&polynomial_vertical, &policy())
    );
}

#[test]
fn rational_polynomial_line_image_relations_use_supporting_line_roots() {
    let rational_arch = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let baseline = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        baseline.relation_to_rational_quadratic(&rational_arch, &policy())
    else {
        panic!("polynomial line image should use rational supporting-line roots");
    };
    assert_eq!(points.len(), 2);
    assert!(points.iter().any(|hit| hit.point() == &point(0, 0)));
    assert!(points.iter().any(|hit| hit.point() == &point(4, 0)));

    let clipped = QuadraticBezier2::new(point(1, 0), point(2, 0), point(3, 0));
    assert_eq!(
        clipped.relation_to_rational_quadratic(&rational_arch, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );

    let rational_line = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 2),
        point(2, 2),
        point(4, 2),
        Real::from(2_i8),
    )
    .unwrap();
    let polynomial_arch = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        rational_line.relation_to_quadratic(&polynomial_arch, &policy())
    else {
        panic!("rational line image should use polynomial quadratic line roots");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &point(2, 2));
}

#[test]
fn rational_polynomial_curve_relation_is_symmetric_for_identity_disjoint_and_endpoint() {
    let rational_polynomial = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::one(),
    )
    .unwrap();
    let quadratic = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let far_quadratic = QuadraticBezier2::new(point(50, 50), point(52, 54), point(54, 50));
    let touching_cubic = CubicBezier2::new(point(4, 0), point(5, 2), point(7, 2), point(8, 0));
    let far_cubic = CubicBezier2::new(
        point(-80, -80),
        point(-79, -77),
        point(-75, -77),
        point(-72, -80),
    );

    assert_eq!(
        rational_polynomial.relation_to_quadratic(&quadratic, &policy()),
        Classification::Decided(BezierCurveRelation::SameControlPolygon)
    );
    assert_eq!(
        quadratic.relation_to_rational_quadratic(&rational_polynomial, &policy()),
        Classification::Decided(BezierCurveRelation::SameControlPolygon)
    );
    assert_eq!(
        rational_polynomial.relation_to_quadratic(&far_quadratic, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
    assert_eq!(
        far_quadratic.relation_to_rational_quadratic(&rational_polynomial, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
    assert_eq!(
        rational_polynomial.relation_to_cubic(&touching_cubic, &policy()),
        Classification::Decided(BezierCurveRelation::SharedEndpoint)
    );
    assert_eq!(
        touching_cubic.relation_to_rational_quadratic(&rational_polynomial, &policy()),
        Classification::Decided(BezierCurveRelation::SharedEndpoint)
    );
    assert_eq!(
        rational_polynomial.relation_to_cubic(&far_cubic, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
    assert_eq!(
        far_cubic.relation_to_rational_quadratic(&rational_polynomial, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
}

#[test]
fn certified_bezier_flattening_emits_only_when_flatness_is_proven() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let options = BezierFlatteningOptions::try_new(Real::one(), 12, &policy()).unwrap();
    let Classification::Decided(polyline) = curve.flatten_certified(&options, &policy()) else {
        panic!("integer quadratic should flatten under a generous certified depth");
    };

    assert_eq!(polyline.points().first(), Some(&point(0, 0)));
    assert_eq!(polyline.points().last(), Some(&point(4, 0)));
    assert_eq!(
        polyline.certificate().segment_count(),
        polyline.points().len() - 1
    );
    assert!(
        polyline.points().len() > 2,
        "curved quadratic should be split before certification"
    );

    let tiny_error = (Real::one() / Real::from(10_i8)).unwrap();
    let shallow = BezierFlatteningOptions::try_new(tiny_error, 1, &policy()).unwrap();
    assert_eq!(
        curve.flatten_certified(&shallow, &policy()),
        Classification::Uncertain(hypercurve::UncertaintyReason::Unsupported)
    );
    assert_eq!(
        BezierFlatteningOptions::try_new(Real::zero(), 12, &policy()),
        Err(hypercurve::CurveError::InvalidFlatteningOptions)
    );
}

#[test]
fn certified_flattening_simplifies_only_exactly_collinear_vertices() {
    let line_like = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let options = BezierFlatteningOptions::try_new(Real::one(), 12, &policy()).unwrap();
    let polyline = line_like
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    let simplified = polyline
        .simplify_exact_collinear(&policy())
        .unwrap_decided_for_test();

    assert_eq!(simplified.points(), &[point(0, 0), point(4, 0)]);
    assert_eq!(simplified.certificate().segment_count(), 1);
    assert_eq!(
        simplified.certificate().max_error(),
        polyline.certificate().max_error()
    );

    let curved = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let curved_polyline = curved
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    let curved_simplified = curved_polyline
        .simplify_exact_collinear(&policy())
        .unwrap_decided_for_test();
    assert_eq!(curved_simplified.points(), curved_polyline.points());
}

#[test]
fn display_offset_left_offsets_certified_flattened_chords_without_topology_claims() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let options = BezierFlatteningOptions::try_new(Real::one(), 12, &policy()).unwrap();
    let polyline = curve
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test()
        .simplify_exact_collinear(&policy())
        .unwrap_decided_for_test();
    let offset = polyline.display_offset_left(Real::one()).unwrap();

    assert_eq!(offset.segments().len(), 1);
    assert_eq!(offset.segments()[0].start(), &point(0, 1));
    assert_eq!(offset.segments()[0].end(), &point(4, 1));
    assert_eq!(
        offset.source_certificate().max_error(),
        polyline.certificate().max_error()
    );
}

#[test]
fn certified_flattened_polyline_fits_exact_line_only_for_zero_error_cases() {
    let line_like = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let curved = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let options = BezierFlatteningOptions::try_new(Real::one(), 12, &policy()).unwrap();

    let line_polyline = line_like
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    let Classification::Decided(BezierLineFitRelation::Fit(fit)) =
        line_polyline.fit_exact_line(&policy()).unwrap()
    else {
        panic!("exact collinear flattened Bezier should fit a line");
    };
    assert_eq!(fit.line().start(), &point(0, 0));
    assert_eq!(fit.line().end(), &point(4, 0));
    assert_eq!(
        fit.source_certificate().max_error(),
        line_polyline.certificate().max_error()
    );

    let curved_polyline = curved
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    assert_eq!(
        curved_polyline.fit_exact_line(&policy()).unwrap(),
        Classification::Decided(BezierLineFitRelation::NotLine)
    );
}

#[test]
fn polynomial_bezier_line_image_fits_without_flattening() {
    let quadratic = QuadraticBezier2::new(point(0, 3), point(2, 3), point(6, 3));
    let fit = quadratic
        .fit_exact_line_image(&policy())
        .unwrap()
        .unwrap_decided_for_test();
    let BezierLineImageFitRelation::Fit(fit) = fit else {
        panic!("collinear control polygon should fit its endpoint line image");
    };
    assert_eq!(fit.control_point_count(), 3);
    assert_eq!(fit.line().start(), &point(0, 3));
    assert_eq!(fit.line().end(), &point(6, 3));
    let offset = fit.offset_left_exact(Real::from(2_i8)).unwrap();
    assert_eq!(offset.line().start(), &point(0, 5));
    assert_eq!(offset.line().end(), &point(6, 5));
    assert_eq!(offset.control_point_count(), 3);
    assert_eq!(offset.distance(), &Real::from(2_i8));

    let off_line = CubicBezier2::new(point(0, 0), point(1, 2), point(3, 2), point(4, 0));
    assert_eq!(
        off_line
            .fit_exact_line_image(&policy())
            .unwrap()
            .unwrap_decided_for_test(),
        BezierLineImageFitRelation::NotLine
    );
}

#[test]
fn rational_bezier_line_image_fit_requires_positive_weights() {
    let rational_line = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 3),
        point(2, 3),
        point(6, 3),
        Real::from(2_i8),
    )
    .unwrap();
    let fit = rational_line
        .fit_exact_line_image(&policy())
        .unwrap()
        .unwrap_decided_for_test();
    let BezierLineImageFitRelation::Fit(fit) = fit else {
        panic!("positive-weight collinear conic should fit its endpoint line image");
    };
    assert_eq!(fit.control_point_count(), 3);
    assert_eq!(fit.line().start(), &point(0, 3));
    assert_eq!(fit.line().end(), &point(6, 3));
    let offset = fit.offset_left_exact(Real::from(2_i8)).unwrap();
    assert_eq!(offset.line().start(), &point(0, 5));

    let curved = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    assert_eq!(
        curved
            .fit_exact_line_image(&policy())
            .unwrap()
            .unwrap_decided_for_test(),
        BezierLineImageFitRelation::NotLine
    );

    let negative_weight = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 3),
        point(2, 3),
        point(6, 3),
        Real::from(-1_i8),
    )
    .unwrap();
    assert_eq!(
        negative_weight.fit_exact_line_image(&policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn certified_line_fit_offsets_exactly_as_line_primitive() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let options = BezierFlatteningOptions::try_new(Real::one(), 12, &policy()).unwrap();
    let polyline = curve
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    let Classification::Decided(BezierLineFitRelation::Fit(fit)) =
        polyline.fit_exact_line(&policy()).unwrap()
    else {
        panic!("collinear Bezier should produce an exact line fit");
    };
    let offset = fit.offset_left_exact(Real::one()).unwrap();

    assert_eq!(offset.line().start(), &point(0, 1));
    assert_eq!(offset.line().end(), &point(4, 1));
    assert_eq!(offset.distance(), &Real::one());
    assert_eq!(
        offset.source_certificate().max_error(),
        fit.source_certificate().max_error()
    );
}

proptest! {
    #[test]
    fn quadratic_endpoint_evaluation_is_exact(
        ax in -32_i32..32,
        ay in -32_i32..32,
        bx in -32_i32..32,
        by in -32_i32..32,
        cx in -32_i32..32,
        cy in -32_i32..32,
    ) {
        let start = point(ax, ay);
        let control = point(bx, by);
        let end = point(cx, cy);
        let curve = QuadraticBezier2::new(start.clone(), control, end.clone());

        prop_assert_eq!(curve.point_at(Real::zero()), start.clone());
        prop_assert_eq!(curve.point_at(Real::one()), end.clone());
        prop_assert_eq!(
            curve.contains_point_at_parameter(&start, Real::zero(), &policy()),
            Classification::Decided(true)
        );
        prop_assert_eq!(
            curve.contains_point_at_parameter(&end, Real::one(), &policy()),
            Classification::Decided(true)
        );
        prop_assert!(curve.structural_facts().all_exact_rational());
    }

    #[test]
    fn quadratic_point_solver_accepts_generated_arch_midpoints(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let mid_y = ay + height;
        let arch = QuadraticBezier2::new(
            point(ax, ay),
            point(mid_x, ay + 2 * height),
            point(end_x, ay),
        );
        let midpoint = point(mid_x, mid_y);

        prop_assert_eq!(
            arch.contains_point(&midpoint, &policy()),
            Classification::Decided(true)
        );
        let Classification::Decided(parameters) = arch.parameters_for_point(&midpoint, &policy())
        else {
            panic!("generated quadratic midpoint should have certified parameters");
        };
        prop_assert!(!parameters.is_empty());
        for parameter in parameters {
            prop_assert_eq!(
                arch.contains_point_at_parameter(&midpoint, parameter, &policy()),
                Classification::Decided(true)
            );
        }
    }

    #[test]
    fn cubic_midpoint_stays_inside_control_hull_for_integer_controls(
        ax in -8_i32..8,
        ay in -8_i32..8,
        bx in -8_i32..8,
        by in -8_i32..8,
        cx in -8_i32..8,
        cy in -8_i32..8,
        dx in -8_i32..8,
        dy in -8_i32..8,
    ) {
        let curve = CubicBezier2::new(point(ax, ay), point(bx, by), point(cx, cy), point(dx, dy));
        let midpoint = curve.point_at(half());
        let bbox = match curve.control_hull_box(&policy()) {
            Classification::Decided(bbox) => bbox,
            Classification::Uncertain(reason) => panic!("integer hull ordering should be decided: {reason:?}"),
        };

        prop_assert_eq!(
            bbox.contains_point(&midpoint, &policy()),
            Classification::Decided(true)
        );
        prop_assert!(curve.structural_facts().all_exact_rational());
    }

    #[test]
    fn quadratic_certified_bounds_are_inside_control_hull_for_integer_controls(
        ax in -16_i32..16,
        ay in -16_i32..16,
        bx in -16_i32..16,
        by in -16_i32..16,
        cx in -16_i32..16,
        cy in -16_i32..16,
    ) {
        let curve = QuadraticBezier2::new(point(ax, ay), point(bx, by), point(cx, cy));
        let midpoint = curve.point_at(half());
        let bounds = match curve.certified_bounds(&policy()) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => panic!("integer bounds should be decided: {reason:?}"),
        };

        prop_assert_eq!(
            bounds.contains_point(&midpoint, &policy()),
            Classification::Decided(true)
        );
        prop_assert!(curve.monotone_spans(&policy()).map(|spans| !spans.is_empty()) == Classification::Decided(true));
    }

    #[test]
    fn rational_quadratic_positive_integer_weights_evaluate_midpoint(
        ax in -8_i32..8,
        ay in -8_i32..8,
        bx in -8_i32..8,
        by in -8_i32..8,
        cx in -8_i32..8,
        cy in -8_i32..8,
        w0 in 1_i32..8,
        w1 in 1_i32..8,
        w2 in 1_i32..8,
    ) {
        let curve = RationalQuadraticBezier2::try_new(
            point(ax, ay),
            point(bx, by),
            point(cx, cy),
            Real::from(w0),
            Real::from(w1),
            Real::from(w2),
        )
        .unwrap();

        let Classification::Decided(_) = curve.point_at(half(), &policy()) else {
            panic!("positive integer weights should not hit the projective denominator boundary");
        };
        prop_assert_eq!(
            curve.point_at(Real::zero(), &policy()).unwrap_decided_for_test(),
            curve.start().clone()
        );
        prop_assert_eq!(
            curve.point_at(Real::one(), &policy()).unwrap_decided_for_test(),
            curve.end().clone()
        );
        let baseline = LineSeg2::try_new(point(-100, ay), point(100, ay)).unwrap();
        let relation = curve.relation_to_line(&baseline, &policy());
        prop_assert!(
            matches!(
                relation,
                Classification::Decided(BezierLineRelation::Intersects { .. })
                    | Classification::Decided(BezierLineRelation::OnSupportingLine)
                    | Classification::Decided(BezierLineRelation::Unresolved)
                    | Classification::Decided(BezierLineRelation::ControlHullDisjoint { .. })
            ),
            "positive integer rational conic line relation should decide or return an explicit unresolved relation, got {relation:?}"
        );
        prop_assert_eq!(
            curve.relation_to_rational_quadratic(&curve, &policy()),
            Classification::Decided(BezierCurveRelation::SameControlPolygon)
        );
        prop_assert!(curve.structural_facts().all_exact_rational());
    }

    #[test]
    fn rational_projective_weight_scaling_preserves_generated_conic_images(
        ax in -8_i32..8,
        ay in -8_i32..8,
        bx in -8_i32..8,
        by in -8_i32..8,
        cx in -8_i32..8,
        cy in -8_i32..8,
        w0 in 1_i32..8,
        w1 in 1_i32..8,
        w2 in 1_i32..8,
        scale in 2_i32..8,
    ) {
        let curve = RationalQuadraticBezier2::try_new(
            point(ax, ay),
            point(bx, by),
            point(cx, cy),
            Real::from(w0),
            Real::from(w1),
            Real::from(w2),
        )
        .unwrap();
        let scaled = RationalQuadraticBezier2::try_new(
            point(ax, ay),
            point(bx, by),
            point(cx, cy),
            Real::from(w0 * scale),
            Real::from(w1 * scale),
            Real::from(w2 * scale),
        )
        .unwrap();

        prop_assert_eq!(
            curve.relation_to_rational_quadratic(&scaled, &policy()),
            Classification::Decided(BezierCurveRelation::SameCurveImage)
        );
        prop_assert_eq!(
            scaled.relation_to_rational_quadratic(&curve, &policy()),
            Classification::Decided(BezierCurveRelation::SameCurveImage)
        );
    }

    #[test]
    fn rational_point_solver_accepts_generated_positive_weight_midpoints(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let mid_y = ay + 2 * height;
        let curve = RationalQuadraticBezier2::try_unit_end_weights(
            point(ax, ay),
            point(mid_x, ay + 3 * height),
            point(end_x, ay),
            Real::from(2_i8),
        )
        .unwrap();
        let midpoint = point(mid_x, mid_y);

        prop_assert_eq!(
            curve.contains_point(&midpoint, &policy()),
            Classification::Decided(true)
        );
        let Classification::Decided(parameters) = curve.parameters_for_point(&midpoint, &policy())
        else {
            panic!("generated rational midpoint should have certified parameters");
        };
        prop_assert!(!parameters.is_empty());
        for parameter in parameters {
            prop_assert_eq!(
                curve.contains_point_at_parameter(&midpoint, parameter, &policy()),
                Classification::Decided(true)
            );
        }
    }

    #[test]
    fn rational_quadratic_positive_integer_weight_bounds_contain_midpoint(
        ax in -8_i32..8,
        ay in -8_i32..8,
        bx in -8_i32..8,
        by in -8_i32..8,
        cx in -8_i32..8,
        cy in -8_i32..8,
        w0 in 1_i32..8,
        w1 in 1_i32..8,
        w2 in 1_i32..8,
    ) {
        let curve = RationalQuadraticBezier2::try_new(
            point(ax, ay),
            point(bx, by),
            point(cx, cy),
            Real::from(w0),
            Real::from(w1),
            Real::from(w2),
        )
        .unwrap();
        let midpoint = curve.point_at(half(), &policy()).unwrap_decided_for_test();
        match curve.certified_bounds(&policy()) {
            Classification::Decided(bounds) => {
                prop_assert_eq!(
                    bounds.contains_point(&midpoint, &policy()),
                    Classification::Decided(true)
                );
            }
            Classification::Uncertain(reason) => {
                prop_assert!(
                    matches!(
                        reason,
                        hypercurve::UncertaintyReason::Ordering
                            | hypercurve::UncertaintyReason::RealSign
                    ),
                    "positive weights should only leave extrema ordering/sign uncertainty, got {reason:?}"
                );
            }
        }
        match curve.monotone_spans(&policy()) {
            Classification::Decided(spans) => prop_assert!(!spans.is_empty()),
            Classification::Uncertain(reason) => prop_assert!(
                matches!(
                    reason,
                    hypercurve::UncertaintyReason::Ordering
                        | hypercurve::UncertaintyReason::RealSign
                ),
                "positive weights should only leave monotone ordering/sign uncertainty, got {reason:?}"
            ),
        }
    }

    #[test]
    fn mixed_polynomial_bezier_far_boxes_are_decided(
        ax in -16_i32..16,
        ay in -16_i32..16,
        bx in -16_i32..16,
        by in -16_i32..16,
        cx in -16_i32..16,
        cy in -16_i32..16,
        offset in 100_i32..200,
    ) {
        let quadratic = QuadraticBezier2::new(point(ax, ay), point(bx, by), point(cx, cy));
        let cubic = CubicBezier2::new(
            point(ax + offset, ay + offset),
            point(bx + offset, by + offset),
            point(cx + offset, cy + offset),
            point(cx + offset + 1, cy + offset),
        );

        prop_assert_eq!(
            quadratic.relation_to_cubic(&cubic, &policy()),
            Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
        );
    }

    #[test]
    fn rational_polynomial_positive_weight_far_boxes_are_decided(
        ax in -16_i32..16,
        ay in -16_i32..16,
        bx in -16_i32..16,
        by in -16_i32..16,
        cx in -16_i32..16,
        cy in -16_i32..16,
        w in 1_i32..8,
        offset in 100_i32..200,
    ) {
        let rational = RationalQuadraticBezier2::try_unit_end_weights(
            point(ax, ay),
            point(bx, by),
            point(cx, cy),
            Real::from(w),
        )
        .unwrap();
        let quadratic = QuadraticBezier2::new(
            point(ax + offset, ay + offset),
            point(bx + offset, by + offset),
            point(cx + offset, cy + offset),
        );
        let cubic = CubicBezier2::new(
            point(ax - offset, ay - offset),
            point(bx - offset, by - offset),
            point(cx - offset, cy - offset),
            point(cx - offset - 1, cy - offset),
        );

        prop_assert_eq!(
            rational.relation_to_quadratic(&quadratic, &policy()),
            Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
        );
        prop_assert_eq!(
            quadratic.relation_to_rational_quadratic(&rational, &policy()),
            Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
        );
        prop_assert_eq!(
            rational.relation_to_cubic(&cubic, &policy()),
            Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
        );
        prop_assert_eq!(
            cubic.relation_to_rational_quadratic(&rational, &policy()),
            Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
        );
    }

    #[test]
    fn large_error_flattening_keeps_generated_bezier_endpoints(
        ax in -16_i32..16,
        ay in -16_i32..16,
        bx in -16_i32..16,
        by in -16_i32..16,
        cx in -16_i32..16,
        cy in -16_i32..16,
        dx in -16_i32..16,
        dy in -16_i32..16,
    ) {
        let quadratic = QuadraticBezier2::new(point(ax, ay), point(bx, by), point(cx, cy));
        let cubic = CubicBezier2::new(point(ax, ay), point(bx, by), point(cx, cy), point(dx, dy));
        let options = BezierFlatteningOptions::try_new(Real::from(1000_i32), 4, &policy()).unwrap();
        let quadratic_polyline = quadratic.flatten_certified(&options, &policy()).unwrap_decided_for_test();
        let cubic_polyline = cubic.flatten_certified(&options, &policy()).unwrap_decided_for_test();

        prop_assert_eq!(quadratic_polyline.points(), &[point(ax, ay), point(cx, cy)]);
        prop_assert_eq!(quadratic_polyline.certificate().segment_count(), 1);
        prop_assert_eq!(cubic_polyline.points(), &[point(ax, ay), point(dx, dy)]);
        prop_assert_eq!(cubic_polyline.certificate().segment_count(), 1);
    }

    #[test]
    fn exact_collinear_simplification_preserves_generated_endpoints(
        ax in -16_i32..16,
        ay in -16_i32..16,
        bx in -16_i32..16,
        cx in -16_i32..16,
    ) {
        let curve = QuadraticBezier2::new(point(ax, ay), point(bx, ay), point(cx, ay));
        let options = BezierFlatteningOptions::try_new(Real::from(1000_i32), 4, &policy()).unwrap();
        let polyline = curve.flatten_certified(&options, &policy()).unwrap_decided_for_test();
        let simplified = polyline.simplify_exact_collinear(&policy()).unwrap_decided_for_test();

        prop_assert_eq!(simplified.points().first(), Some(&point(ax, ay)));
        prop_assert_eq!(simplified.points().last(), Some(&point(cx, ay)));
        prop_assert!(simplified.points().len() <= polyline.points().len());
        prop_assert_eq!(simplified.certificate().max_error(), polyline.certificate().max_error());
    }

    #[test]
    fn exact_line_fit_preserves_generated_collinear_endpoints(
        ax in -16_i32..16,
        ay in -16_i32..16,
        bx in -16_i32..16,
        cx in -16_i32..16,
    ) {
        prop_assume!(ax != cx);
        let curve = QuadraticBezier2::new(point(ax, ay), point(bx, ay), point(cx, ay));
        let options = BezierFlatteningOptions::try_new(Real::from(1000_i32), 4, &policy()).unwrap();
        let polyline = curve.flatten_certified(&options, &policy()).unwrap_decided_for_test();
        let fit = polyline.fit_exact_line(&policy()).unwrap();

        let Classification::Decided(BezierLineFitRelation::Fit(fit)) = fit else {
            panic!("generated collinear quadratic should fit one exact line");
        };
        prop_assert_eq!(fit.line().start(), &point(ax, ay));
        prop_assert_eq!(fit.line().end(), &point(cx, ay));
        prop_assert_eq!(fit.source_certificate().max_error(), polyline.certificate().max_error());
    }

    #[test]
    fn exact_line_image_fit_preserves_generated_collinear_controls(
        ax in -16_i32..16,
        ay in -16_i32..16,
        ab in 1_i32..16,
        bc in 0_i32..16,
        cd in 0_i32..16,
    ) {
        let bx = ax + ab;
        let cx = bx + bc;
        let dx = cx + cd;
        let curve = CubicBezier2::new(point(ax, ay), point(bx, ay), point(cx, ay), point(dx, ay));
        let fit = curve
            .fit_exact_line_image(&policy())
            .unwrap()
            .unwrap_decided_for_test();
        let BezierLineImageFitRelation::Fit(fit) = fit else {
            panic!("generated horizontal control polygon should be a line image");
        };

        prop_assert_eq!(fit.control_point_count(), 4);
        prop_assert_eq!(fit.line().start(), &point(ax, ay));
        prop_assert_eq!(fit.line().end(), &point(dx, ay));
        let offset = fit.offset_left_exact(Real::one()).unwrap();
        prop_assert_eq!(offset.line().start(), &point(ax, ay + 1));
    }

    #[test]
    fn rational_line_image_fit_preserves_generated_positive_weight_lines(
        ax in -16_i32..16,
        ay in -16_i32..16,
        ab in 1_i32..16,
        bc in 0_i32..16,
        weight in 1_i32..8,
    ) {
        let bx = ax + ab;
        let cx = bx + bc;
        let curve = RationalQuadraticBezier2::try_unit_end_weights(
            point(ax, ay),
            point(bx, ay),
            point(cx, ay),
            Real::from(weight),
        )
        .unwrap();
        let fit = curve
            .fit_exact_line_image(&policy())
            .unwrap()
            .unwrap_decided_for_test();
        let BezierLineImageFitRelation::Fit(fit) = fit else {
            panic!("generated positive-weight horizontal rational should be a line image");
        };

        prop_assert_eq!(fit.control_point_count(), 3);
        prop_assert_eq!(fit.line().start(), &point(ax, ay));
        prop_assert_eq!(fit.line().end(), &point(cx, ay));
    }

    #[test]
    fn line_image_curve_relation_finds_generated_quadratic_tangencies(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let tangent_y = ay + height;
        let arch = QuadraticBezier2::new(
            point(ax, ay),
            point(mid_x, ay + 2 * height),
            point(end_x, ay),
        );
        let tangent = QuadraticBezier2::new(
            point(ax, tangent_y),
            point(mid_x, tangent_y),
            point(end_x, tangent_y),
        );

        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            tangent.relation_to_quadratic(&arch, &policy())
        else {
            panic!("generated tangent line image should return an exact point");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &point(mid_x, tangent_y));
    }

    #[test]
    fn endpoint_on_quadratic_relation_handles_generated_midpoints(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let mid_y = ay + height;
        let arch = QuadraticBezier2::new(
            point(ax, ay),
            point(mid_x, ay + 2 * height),
            point(end_x, ay),
        );
        let probe = CubicBezier2::new(
            point(mid_x, mid_y),
            point(mid_x + 1, mid_y + 3),
            point(end_x + 1, mid_y + 3),
            point(end_x + 2, mid_y + 2),
        );

        let relation = probe.relation_to_quadratic(&arch, &policy());
        let Classification::Decided(BezierCurveRelation::EndpointIntersections { points }) = relation
        else {
            panic!("generated endpoint on quadratic midpoint should be certified: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &point(mid_x, mid_y));
    }

    #[test]
    fn same_parameter_quadratic_crossings_are_generated_exact_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let arch = QuadraticBezier2::new(
            point(ax, ay),
            point(mid_x, ay + 2 * height),
            point(end_x, ay),
        );
        let crossing = QuadraticBezier2::new(
            point(ax, ay + height),
            point(mid_x, ay - height),
            point(end_x, ay + height),
        );

        let relation = arch.relation_to_quadratic(&crossing, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated same-parameter quadratic crossings should be exact points: {relation:?}");
        };
        prop_assert_eq!(points.len(), 2);
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            crossing.relation_to_quadratic(&arch, &policy())
        else {
            panic!("generated same-parameter quadratic crossing relation should be symmetric");
        };
        prop_assert_eq!(points.len(), 2);
    }

    #[test]
    fn same_axis_monotone_quadratics_with_positive_gap_are_generated_no_hits(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
        gap in 1_i32..16,
    ) {
        prop_assume!(gap <= height);
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let arch = QuadraticBezier2::new(
            point(ax, ay),
            point(mid_x, ay + 2 * height),
            point(end_x, ay),
        );
        let raised = QuadraticBezier2::new(
            point(ax, ay + gap),
            point(mid_x, ay + 2 * height + gap),
            point(end_x, ay + gap),
        );

        prop_assert_eq!(
            arch.relation_to_quadratic(&raised, &policy()),
            Classification::Decided(BezierCurveRelation::NoIntersection)
        );
        prop_assert_eq!(
            raised.relation_to_quadratic(&arch, &policy()),
            Classification::Decided(BezierCurveRelation::NoIntersection)
        );
    }

    #[test]
    fn degree_normalized_same_axis_quadratic_cubic_gaps_are_generated_no_hits(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
        gap in 1_i32..16,
    ) {
        prop_assume!(gap <= height);
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + 3 * width, ay + 3 * height),
            point(ax + 6 * width, ay),
        );
        let elevated_raised = CubicBezier2::new(
            point(ax, ay + gap),
            point(ax + 2 * width, ay + 2 * height + gap),
            point(ax + 4 * width, ay + 2 * height + gap),
            point(ax + 6 * width, ay + gap),
        );

        prop_assert_eq!(
            quadratic.relation_to_cubic(&elevated_raised, &policy()),
            Classification::Decided(BezierCurveRelation::NoIntersection)
        );
        prop_assert_eq!(
            elevated_raised.relation_to_quadratic(&quadratic, &policy()),
            Classification::Decided(BezierCurveRelation::NoIntersection)
        );
    }

    #[test]
    fn degree_elevated_quadratic_cubic_pairs_are_generated_same_images(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in -16_i32..16,
    ) {
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + 3 * width, ay + 3 * height),
            point(ax + 6 * width, ay),
        );
        let elevated = CubicBezier2::new(
            point(ax, ay),
            point(ax + 2 * width, ay + 2 * height),
            point(ax + 4 * width, ay + 2 * height),
            point(ax + 6 * width, ay),
        );

        prop_assert_eq!(
            quadratic.relation_to_cubic(&elevated, &policy()),
            Classification::Decided(BezierCurveRelation::SameCurveImage)
        );
        prop_assert_eq!(
            elevated.relation_to_quadratic(&quadratic, &policy()),
            Classification::Decided(BezierCurveRelation::SameCurveImage)
        );
    }

    #[test]
    fn mixed_degree_midpoint_hits_are_generated_exact_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + 3 * width, ay + 3 * height),
            point(ax + 6 * width, ay),
        );
        let cubic = CubicBezier2::new(
            point(ax, ay + 6 * height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay + 6 * height),
        );

        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            quadratic.relation_to_cubic(&cubic, &policy())
        else {
            panic!("generated mixed-degree midpoint hit should be promoted exactly");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &quadratic.point_at(half()));
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            cubic.relation_to_quadratic(&quadratic, &policy())
        else {
            panic!("generated mixed-degree midpoint hit should be symmetric");
        };
        prop_assert_eq!(points.len(), 1);
    }

    #[test]
    fn mixed_degree_quarter_hits_are_generated_exact_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + 3 * width, ay + 62 * height),
            point(ax + 6 * width, ay),
        );
        let cubic = CubicBezier2::new(
            point(ax, ay + 48 * height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay + 192 * height),
        );

        let relation = quadratic.relation_to_cubic(&cubic, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated mixed-degree quarter hit should be promoted exactly: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(
            points[0].point(),
            &quadratic.point_at((Real::one() / Real::from(4_i8)).unwrap())
        );
    }

    #[test]
    fn cubic_quarter_hits_are_generated_exact_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let first = CubicBezier2::new(
            point(ax, ay),
            point(ax + 2 * width, ay + 4 * height),
            point(ax + 4 * width, ay + 4 * height),
            point(ax + 6 * width, ay),
        );
        let second = CubicBezier2::new(
            point(ax, ay + 4 * height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay + 36 * height),
        );

        let quarter = (Real::one() / Real::from(4_i8)).unwrap();
        let relation = first.relation_to_cubic(&second, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated cubic quarter hit should be promoted exactly: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &first.point_at(quarter));

        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            second.relation_to_cubic(&first, &policy())
        else {
            panic!("generated cubic quarter hit should be symmetric");
        };
        prop_assert_eq!(points.len(), 1);
    }

    #[test]
    fn endpoint_on_cubic_relation_handles_generated_quarter_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let cubic = CubicBezier2::new(
            point(ax, ay),
            point(ax + 2 * width, ay + 4 * height),
            point(ax + 4 * width, ay + 4 * height),
            point(ax + 6 * width, ay),
        );
        let quarter = (Real::one() / Real::from(4_i8)).unwrap();
        let cubic_point = cubic.point_at(quarter.clone());
        let probe = QuadraticBezier2::new(
            Point2::new(
                cubic_point.x().clone(),
                cubic_point.y() + Real::from(20 * height),
            ),
            Point2::new(
                cubic_point.x() + Real::from(width),
                cubic_point.y() + Real::from(10 * height),
            ),
            cubic_point.clone(),
        );

        prop_assert_eq!(
            cubic.dyadic_parameters_for_point(&cubic_point, &policy()),
            Classification::Decided(vec![quarter])
        );
        let relation = probe.relation_to_cubic(&cubic, &policy());
        let Classification::Decided(BezierCurveRelation::EndpointIntersections { points }) = relation
        else {
            panic!("generated endpoint on cubic quarter point should be certified: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &cubic_point);
    }

    #[test]
    fn equal_weight_rational_crossings_follow_generated_polynomial_dispatch(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let rational_arch = RationalQuadraticBezier2::try_unit_end_weights(
            point(ax, ay),
            point(mid_x, ay + 2 * height),
            point(end_x, ay),
            Real::one(),
        )
        .unwrap();
        let rational_crossing = RationalQuadraticBezier2::try_unit_end_weights(
            point(ax, ay + height),
            point(mid_x, ay - height),
            point(end_x, ay + height),
            Real::one(),
        )
        .unwrap();

        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            rational_arch.relation_to_rational_quadratic(&rational_crossing, &policy())
        else {
            panic!("generated equal-weight rational crossings should reuse polynomial dispatch");
        };
        prop_assert_eq!(points.len(), 2);
    }

    #[test]
    fn exact_length_bounds_collapse_for_generated_axis_collinear_beziers(
        ax in -16_i32..16,
        ay in -16_i32..16,
        ab in 0_i32..16,
        bc in 0_i32..16,
        cd in 0_i32..16,
    ) {
        let bx = ax + ab;
        let cx = bx + bc;
        let dx = cx + cd;
        let quadratic = QuadraticBezier2::new(point(ax, ay), point(bx, ay), point(dx, ay));
        let cubic = CubicBezier2::new(point(ax, ay), point(bx, ay), point(cx, ay), point(dx, ay));
        let expected = Real::from(dx - ax);

        let quadratic_bounds = quadratic.length_bounds().unwrap();
        let cubic_bounds = cubic.length_bounds().unwrap();
        let refined_quadratic = quadratic.refined_length_bounds(3).unwrap();
        let refined_cubic = cubic.refined_length_bounds(3).unwrap();
        let quadratic_prefix = quadratic
            .refined_prefix_length_bounds(half(), 2, &policy())
            .unwrap()
            .unwrap_decided_for_test();
        let cubic_prefix = cubic
            .refined_prefix_length_bounds(half(), 2, &policy())
            .unwrap()
            .unwrap_decided_for_test();
        let inverse_zero = cubic
            .inverse_length_parameter_region(Real::zero(), 4, 2, &policy())
            .unwrap()
            .unwrap_decided_for_test();

        prop_assert_eq!(quadratic_bounds.lower(), &expected);
        prop_assert_eq!(quadratic_bounds.upper(), &expected);
        prop_assert!(quadratic_bounds.is_exact());
        prop_assert_eq!(cubic_bounds.lower(), &expected);
        prop_assert_eq!(cubic_bounds.upper(), &expected);
        prop_assert!(cubic_bounds.is_exact());
        prop_assert_eq!(refined_quadratic, quadratic_bounds);
        prop_assert_eq!(refined_cubic, cubic_bounds);
        prop_assert!(quadratic_prefix.is_exact());
        prop_assert!(cubic_prefix.is_exact());
        prop_assert_eq!(inverse_zero.parameter_span().start(), &Real::zero());
        prop_assert_eq!(inverse_zero.parameter_span().end(), &Real::zero());
    }

    #[test]
    fn signed_area_contribution_matches_generated_horizontal_line_images(
        ax in -16_i32..16,
        ay in -16_i32..16,
        ab in 0_i32..16,
        bc in 0_i32..16,
        cd in 0_i32..16,
    ) {
        let bx = ax + ab;
        let cx = bx + bc;
        let dx = cx + cd;
        let quadratic = QuadraticBezier2::new(point(ax, ay), point(bx, ay), point(dx, ay));
        let cubic = CubicBezier2::new(point(ax, ay), point(bx, ay), point(cx, ay), point(dx, ay));
        let expected = ((Real::from(-ay) * Real::from(dx - ax)) / Real::from(2_i8)).unwrap();

        prop_assert_eq!(quadratic.signed_area_contribution().unwrap(), expected.clone());
        prop_assert_eq!(cubic.signed_area_contribution().unwrap(), expected.clone());
        prop_assert_eq!(
            quadratic
                .prefix_signed_area_contribution(Real::zero(), &policy())
                .unwrap()
                .unwrap_decided_for_test(),
            Real::zero()
        );
        prop_assert_eq!(
            cubic
                .prefix_signed_area_contribution(Real::one(), &policy())
                .unwrap()
                .unwrap_decided_for_test(),
            expected
        );
    }

    #[test]
    fn area_prefix_sums_match_generated_horizontal_line_image_totals(
        ax in -16_i32..16,
        ay in -16_i32..16,
        ab in 0_i32..16,
        bc in 0_i32..16,
        cd in 0_i32..16,
    ) {
        let bx = ax + ab;
        let cx = bx + bc;
        let dx = cx + cd;
        let first = QuadraticBezier2::new(point(ax, ay), point(bx, ay), point(cx, ay));
        let second = QuadraticBezier2::new(point(cx, ay), point(cx + bc, ay), point(dx, ay));
        let table = BezierAreaPrefixSums2::from_quadratics([&first, &second]).unwrap();
        let expected = ((Real::from(-ay) * Real::from(dx - ax)) / Real::from(2_i8)).unwrap();

        prop_assert_eq!(table.segment_count(), 2);
        prop_assert_eq!(table.total(), &expected);
        prop_assert_eq!(table.range_contribution(0..2).unwrap(), expected);
        prop_assert_eq!(table.range_contribution(1..1).unwrap(), Real::zero());
    }

    #[test]
    fn area_moments_match_generated_axis_aligned_line_images(
        ax in -16_i32..16,
        ay in -16_i32..16,
        bx in -16_i32..16,
        by in -16_i32..16,
    ) {
        let horizontal = CubicBezier2::new(point(ax, ay), point(ax, ay), point(bx, ay), point(bx, ay));
        let horizontal_moments = horizontal.area_moments_contribution().unwrap();
        let dx = Real::from(bx - ax);
        let horizontal_area = ((Real::from(-ay) * dx.clone()) / Real::from(2_i8)).unwrap();
        let horizontal_y_moment =
            ((Real::from(-ay * ay) * dx.clone()) / Real::from(2_i8)).unwrap();

        prop_assert_eq!(horizontal_moments.signed_area(), &horizontal_area);
        prop_assert_eq!(horizontal_moments.x_moment(), &Real::zero());
        prop_assert_eq!(horizontal_moments.y_moment(), &horizontal_y_moment);

        let vertical = QuadraticBezier2::new(point(ax, ay), point(ax, by), point(ax, by));
        let vertical_moments = vertical.area_moments_contribution().unwrap();
        let dy = Real::from(by - ay);
        let vertical_area = ((Real::from(ax) * dy.clone()) / Real::from(2_i8)).unwrap();
        let vertical_x_moment =
            ((Real::from(ax * ax) * dy.clone()) / Real::from(2_i8)).unwrap();

        prop_assert_eq!(vertical_moments.signed_area(), &vertical_area);
        prop_assert_eq!(vertical_moments.x_moment(), &vertical_x_moment);
        prop_assert_eq!(vertical_moments.y_moment(), &Real::zero());
    }

    #[test]
    fn area_moment_prefix_sums_match_generated_horizontal_line_image_totals(
        ax in -16_i32..16,
        ay in -16_i32..16,
        ab in 0_i32..16,
        bc in 0_i32..16,
        cd in 0_i32..16,
    ) {
        let bx = ax + ab;
        let cx = bx + bc;
        let dx = cx + cd;
        let first = CubicBezier2::new(point(ax, ay), point(ax, ay), point(bx, ay), point(cx, ay));
        let second = CubicBezier2::new(point(cx, ay), point(cx, ay), point(cx, ay), point(dx, ay));
        let table = BezierAreaMomentPrefixSums2::from_cubics([&first, &second]).unwrap();
        let total_dx = Real::from(dx - ax);
        let expected_area = ((Real::from(-ay) * total_dx.clone()) / Real::from(2_i8)).unwrap();
        let expected_y_moment =
            ((Real::from(-ay * ay) * total_dx) / Real::from(2_i8)).unwrap();

        prop_assert_eq!(table.segment_count(), 2);
        prop_assert_eq!(table.total().signed_area(), &expected_area);
        prop_assert_eq!(table.total().x_moment(), &Real::zero());
        prop_assert_eq!(table.total().y_moment(), &expected_y_moment);
        let empty = table.range_contribution(1..1).unwrap();
        prop_assert_eq!(empty.signed_area(), &Real::zero());
    }

    #[test]
    fn display_offset_left_preserves_generated_chord_count_for_nonzero_lines(
        ax in -16_i32..16,
        ay in -16_i32..16,
        cx in -16_i32..16,
    ) {
        prop_assume!(ax != cx);
        let curve = QuadraticBezier2::new(point(ax, ay), point((ax + cx) / 2, ay), point(cx, ay));
        let options = BezierFlatteningOptions::try_new(Real::from(1000_i32), 4, &policy()).unwrap();
        let polyline = curve
            .flatten_certified(&options, &policy())
            .unwrap_decided_for_test()
            .simplify_exact_collinear(&policy())
            .unwrap_decided_for_test();
        let offset = polyline.display_offset_left(Real::one()).unwrap();

        prop_assert_eq!(offset.segments().len(), polyline.points().len() - 1);
        prop_assert_eq!(offset.source_certificate().segment_count(), polyline.certificate().segment_count());
    }
}

trait ClassificationTestExt<T> {
    fn unwrap_decided_for_test(self) -> T;
}

impl<T> ClassificationTestExt<T> for Classification<T> {
    fn unwrap_decided_for_test(self) -> T {
        match self {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                panic!("expected decided classification: {reason:?}")
            }
        }
    }
}
