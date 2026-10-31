//! Every retained-point probe direction, replayed one at a time.
//!
//! Axis probes and direct ray classification usually decide these targets
//! before an oblique probe runs, so whole-query tests cannot show what one
//! oblique direction does. The original wrong answers came from oblique probes
//! meeting uncertified contacts: for y^2 <= x <= 1/4, a line through the branch
//! junction at the origin; below y = x^3 + 2x, the line y = 2x, which crosses
//! the graph in an odd-order contact at the origin. Skipping such a contact
//! drops a crossing. Each candidate may refuse, but none may misclassify.
use super::fragment_location::FORCED_PROBE_CANDIDATE;
use super::{
    CurveRegionBooleanContext, CurveRegionBooleanOperand2, RetainedPointProbeClassification,
};
use crate::{
    Classification, Curve2, CurveContext, CurveParameterRange2, CurvePath2, CurveRegion2, FillRule,
    LineSeg2, Point2, PolynomialCurveSet2, Real, RegionPointLocation,
};
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_topology::{BivariateCurveDomain, decompose_bivariate_curve};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn branches(equation: BivariatePolynomial, y: [i32; 2]) -> PolynomialCurveSet2 {
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [y[0].into(), y[1].into()],
        },
        Default::default(),
    )
    .unwrap();
    PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap()
}

fn line(start: Point2, end: Point2) -> Curve2 {
    LineSeg2::try_new(start, end).unwrap().into()
}

/// y^2 <= x <= 1/4.
fn parabola_path() -> CurvePath2 {
    let source = branches(
        BivariatePolynomial::new(vec![vec![0.into(), 0.into(), 1.into()], vec![(-1).into()]]),
        [-2, 2],
    );
    let range = CurveParameterRange2::try_new(Real::zero().into(), q(1, 4).into()).unwrap();
    CurvePath2::try_new(vec![
        source.curves()[0].restrict_source(range.clone()).unwrap(),
        line(
            Point2::new(q(1, 4), q(-1, 2)),
            Point2::new(q(1, 4), q(1, 2)),
        ),
        source.curves()[1]
            .restrict_source(range)
            .unwrap()
            .reversed()
            .unwrap(),
    ])
    .unwrap()
}

/// Below y = x^3 + 2x on [-1, 1], closed through (1, -3) and (-1, -3).
fn cubic_path(y: [i32; 2]) -> CurvePath2 {
    let source = branches(
        BivariatePolynomial::new(vec![
            vec![0.into(), 1.into()],
            vec![(-2).into()],
            vec![],
            vec![(-1).into()],
        ]),
        y,
    );
    CurvePath2::try_new(vec![
        source.curves()[0].clone(),
        line(Point2::from_values(1, 3), Point2::from_values(1, -3)),
        line(Point2::from_values(1, -3), Point2::from_values(-1, -3)),
    ])
    .unwrap()
}

/// Classifies `point` using only probe candidate `index`.
fn probe_once(
    path: &CurvePath2,
    point: &Point2,
    index: usize,
) -> Classification<RegionPointLocation> {
    let region =
        CurveRegion2::try_from_boundary_paths(std::slice::from_ref(path), FillRule::NonZero)
            .unwrap();
    let policy = CurveContext::STRICT;
    let context = CurveRegionBooleanContext::try_new_unary(&region, &policy).unwrap();
    FORCED_PROBE_CANDIDATE.with(|forced| forced.set(Some(index)));
    let location = context.classify_retained_point_off_boundary_by_probe_once(
        0,
        point.clone().into(),
        CurveRegionBooleanOperand2::First,
        RetainedPointProbeClassification::FilledRegion,
    );
    FORCED_PROBE_CANDIDATE.with(|forced| forced.set(None));
    location.unwrap()
}

#[test]
fn no_single_probe_direction_misclassifies_polynomial_interiors() {
    // Independently: (3/16)^2 = 9/256 < 1/8 < 1/4, and
    // -3 < 1 < (1/2)^3 + 2(1/2) = 9/8 with -1 < 1/2 < 1.
    let parabola_inside = Point2::new(q(1, 8), q(3, 16));
    let cubic_inside = Point2::new(q(1, 2), 1.into());
    let cases = [
        ("parabola", parabola_path(), parabola_inside.clone()),
        (
            "reversed parabola",
            parabola_path().reversed().unwrap(),
            parabola_inside,
        ),
        ("tight cubic", cubic_path([-3, 3]), cubic_inside.clone()),
        ("loose cubic", cubic_path([-4, 4]), cubic_inside),
    ];
    for (label, path, inside) in cases {
        let mut decided = 0;
        // Each path has three carriers: 2*3+1 fallbacks, four corners and
        // four axis probes. Indices past the last candidate are empty.
        for index in 0..16 {
            match probe_once(&path, &inside, index) {
                Classification::Decided(RegionPointLocation::Inside) => decided += 1,
                Classification::Decided(location) => {
                    panic!("{label}: probe candidate {index} classified {location:?}")
                }
                Classification::Uncertain(_) => {}
            }
        }
        assert!(decided > 0, "{label}: some probe direction must decide");
    }
}

#[test]
fn original_oblique_cubic_probe_refuses_its_odd_order_contact() {
    // Corner 0 of the tight enclosure is (-2, -4), so this probe is y = 2x.
    // Its one contact with the graph is uncertified; ignoring it would report
    // no crossing and an outside point.
    assert!(matches!(
        probe_once(&cubic_path([-3, 3]), &Point2::new(q(1, 2), 1.into()), 0),
        Classification::Uncertain(_)
    ));
}
