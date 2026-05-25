use hypercurve::{
    BezierAreaMomentPrefixSums2, BezierAreaPrefixSums2, BezierLineImageFitRelation,
    BezierOffsetCandidate2, Classification, CurveError, CurvePolicy, Point2, QuadraticBezier2,
    RationalQuadraticBezier2, Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

#[test]
fn quadratic_line_image_fit_offsets_as_exact_line() {
    let bezier = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0));

    let fit = bezier.fit_exact_line_image(&policy()).unwrap();
    let Classification::Decided(BezierLineImageFitRelation::Fit(fit)) = fit else {
        panic!("collinear quadratic should be a certified line image");
    };
    assert_eq!(fit.line().start(), &p(0, 0));
    assert_eq!(fit.line().end(), &p(2, 0));

    let offset = bezier.offset_left_staged(r(1), &policy()).unwrap();
    let Classification::Decided(BezierOffsetCandidate2::ExactLineImage { offset, preflight }) =
        offset
    else {
        panic!("line-image quadratic should offset as an exact primitive");
    };
    assert!(preflight.is_clear());
    assert_eq!(offset.line().start(), &p(0, 1));
    assert_eq!(offset.line().end(), &p(2, 1));
}

#[test]
fn rational_quadratic_conic_line_image_fit_offsets_as_exact_line() {
    let conic =
        RationalQuadraticBezier2::try_new(p(0, 0), p(1, 0), p(2, 0), r(1), r(2), r(1)).unwrap();

    let fit = conic.fit_exact_line_image(&policy()).unwrap();
    let Classification::Decided(BezierLineImageFitRelation::Fit(fit)) = fit else {
        panic!("same-sign collinear rational quadratic should be a certified line image");
    };
    assert_eq!(fit.control_point_count(), 3);
    assert_eq!(fit.line().start(), &p(0, 0));
    assert_eq!(fit.line().end(), &p(2, 0));

    let offset = conic.offset_left_staged(r(1), &policy()).unwrap();
    let Classification::Decided(BezierOffsetCandidate2::ExactLineImage { offset, preflight }) =
        offset
    else {
        panic!("line-image rational quadratic should offset as an exact primitive");
    };
    assert!(preflight.is_clear());
    assert_eq!(offset.line().start(), &p(0, 1));
    assert_eq!(offset.line().end(), &p(2, 1));
}

#[test]
fn bezier_area_prefix_sums_answer_exact_ranges() {
    let first = QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0));
    let second = QuadraticBezier2::new(p(2, 0), p(3, -1), p(4, 0));
    let curves = [first, second];

    let area_prefixes = BezierAreaPrefixSums2::from_quadratics(curves.iter()).unwrap();
    assert_eq!(area_prefixes.segment_count(), 2);
    assert_eq!(
        area_prefixes.range_contribution(0..1).unwrap(),
        curves[0].signed_area_contribution().unwrap()
    );
    assert_eq!(
        area_prefixes.range_contribution(1..2).unwrap(),
        curves[1].signed_area_contribution().unwrap()
    );
    assert_eq!(
        area_prefixes.range_contribution(0..2).unwrap(),
        area_prefixes.total().clone()
    );
    let reversed_start = 2;
    let reversed_end = 1;
    assert_eq!(
        area_prefixes.range_contribution(reversed_start..reversed_end),
        Err(CurveError::InvalidBezierRange)
    );

    let moment_prefixes = BezierAreaMomentPrefixSums2::from_quadratics(curves.iter()).unwrap();
    assert_eq!(moment_prefixes.segment_count(), 2);
    assert_eq!(
        moment_prefixes.range_contribution(0..2).unwrap(),
        moment_prefixes.total().clone()
    );
    assert_eq!(
        moment_prefixes.range_contribution(0..1).unwrap(),
        curves[0].area_moments_contribution().unwrap()
    );
}
