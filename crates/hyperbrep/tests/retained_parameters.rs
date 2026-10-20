use std::cmp::Ordering;

use hyperbrep::{
    Curve3, CurveParameterLocation, GeometryError, OperationContext, Parameter, Point3, Real,
    WorkBudget, WorkKind,
};

fn rational(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn quintic_controls() -> Vec<Real> {
    // Bernstein controls of 32t^5 - 2t - 1. Its unique unit-interval root
    // lies strictly between 1/2 and 3/4, and has no stored Real view.
    [-5, -7, -9, -11, -13, 145].map(|n| rational(n, 5)).to_vec()
}

fn parameters(curve: &Curve3, query: &Point3) -> Vec<Parameter> {
    let CurveParameterLocation::Parameters(values) = curve.parameters_of(query).unwrap() else {
        panic!("expected a finite nonempty set of exact parameters");
    };
    values
}

fn assert_between(parameter: &Parameter, lower: Real, upper: Real) {
    assert_eq!(
        parameter.compare(&Parameter::from_real(lower)).unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        parameter.compare(&Parameter::from_real(upper)).unwrap(),
        Ordering::Less
    );
}

#[test]
fn spatial_queries_retain_selected_roots_in_each_projection() {
    for axis in 0..3 {
        let controls = quintic_controls()
            .into_iter()
            .map(|value| {
                let mut coordinates = [Real::zero(), Real::zero(), Real::zero()];
                coordinates[axis] = value;
                let [x, y, z] = coordinates;
                Point3::new(x, y, z)
            })
            .collect();
        let curve = Curve3::rational_bezier(controls, vec![Real::one(); 6]).unwrap();
        let first = parameters(&curve, &Point3::origin());
        assert_eq!(first.len(), 1);
        assert!(first[0].try_real().is_none());
        assert_between(&first[0], rational(1, 2), rational(3, 4));
        let repeated = parameters(&curve, &Point3::origin());
        assert_eq!(first[0].compare(&repeated[0]).unwrap(), Ordering::Equal);
        assert_eq!(
            first[0].compare(&first[0].clone()).unwrap(),
            Ordering::Equal
        );
        let reversed = parameters(&curve.reversed().unwrap(), &Point3::origin());
        assert_eq!(reversed.len(), 1);
        assert!(reversed[0].try_real().is_none());
        assert_between(&reversed[0], rational(1, 4), rational(1, 2));
    }
}

#[test]
fn selected_root_incidence_replays_the_remaining_spatial_coordinate() {
    for weights in [vec![Real::one(); 6], (1..=6).map(Real::from).collect()] {
        for shift in [Real::zero(), Real::one(), "1e-1000".parse().unwrap()] {
            let controls = quintic_controls()
                .into_iter()
                .zip(&weights)
                .map(|(coefficient, weight)| {
                    let value = (coefficient / weight).unwrap();
                    Point3::new(value.clone(), Real::zero(), Real::from(2) * value + &shift)
                })
                .collect();
            let curve = Curve3::rational_bezier(controls, weights.clone()).unwrap();
            if shift == Real::zero() {
                let values = parameters(&curve, &Point3::origin());
                assert_eq!(values.len(), 1);
                assert!(values[0].try_real().is_none());
                assert_between(&values[0], rational(1, 2), rational(3, 4));
            } else {
                // The x/y projection still has the same selected root. Its z
                // image differs, including at a separation far below binary64.
                assert!(matches!(
                    curve.parameters_of(&Point3::origin()).unwrap(),
                    CurveParameterLocation::None
                ));
            }
        }
    }
}

#[test]
fn repeated_spatial_visits_retain_distinct_algebraic_parameters() {
    // (t² - 1/8)(t² - 1/2) has two distinct positive unit-interval roots.
    let curve = Curve3::rational_bezier(
        [3, 3, -2, -12, 21]
            .map(|n| Point3::new(rational(n, 48), Real::zero(), Real::zero()))
            .to_vec(),
        vec![Real::one(); 5],
    )
    .unwrap();
    let values = parameters(&curve, &Point3::origin());
    assert_eq!(values.len(), 2);
    assert!(
        values
            .iter()
            .all(|parameter| parameter.try_real().is_none())
    );
    assert_between(&values[0], Real::zero(), rational(1, 2));
    assert_between(&values[1], rational(1, 2), Real::one());
    assert_eq!(values[0].compare(&values[1]).unwrap(), Ordering::Less);
}

#[test]
fn nurbs_queries_preserve_nonunit_charts_and_deduplicate_seams() {
    let scalar_controls = quintic_controls();
    let controls = scalar_controls
        .iter()
        .chain(scalar_controls.iter().rev().skip(1))
        .map(|x| Point3::new(x.clone(), Real::zero(), Real::zero()))
        .collect();
    let knots = std::iter::repeat_n(Real::from(2), 6)
        .chain(std::iter::repeat_n(Real::from(5), 5))
        .chain(std::iter::repeat_n(Real::from(9), 6))
        .collect();
    let curve = Curve3::nurbs(5, controls, vec![Real::one(); 11], knots).unwrap();
    let values = parameters(&curve, &Point3::origin());
    assert_eq!(values.len(), 2);
    assert!(
        values
            .iter()
            .all(|parameter| parameter.try_real().is_none())
    );
    assert_between(&values[0], rational(7, 2), rational(17, 4));
    assert_between(&values[1], Real::from(6), Real::from(7));
    assert_eq!(values[0].compare(&values[1]).unwrap(), Ordering::Less);

    let seam = parameters(
        &curve,
        &Point3::new(Real::from(29), Real::zero(), Real::zero()),
    );
    assert_eq!(seam.len(), 1);
    assert_eq!(seam[0].try_real(), Some(&Real::from(5)));
}

#[test]
fn partly_constant_nurbs_preimages_remain_a_typed_blocker() {
    let curve = Curve3::nurbs(
        1,
        vec![
            Point3::origin(),
            Point3::origin(),
            Point3::new(Real::one(), Real::zero(), Real::zero()),
        ],
        vec![Real::one(); 3],
        [0, 0, 1, 2, 2].map(Real::from).to_vec(),
    )
    .unwrap();
    assert_eq!(
        curve.parameters_of(&Point3::origin()).unwrap_err(),
        GeometryError::UnsupportedParameterLocation
    );
}

#[test]
fn retained_inverse_queries_retry_after_a_denied_refinement() {
    let curve = Curve3::rational_bezier(
        quintic_controls()
            .into_iter()
            .map(|x| Point3::new(x, Real::zero(), Real::zero()))
            .collect(),
        vec![Real::one(); 6],
    )
    .unwrap();
    let denied = OperationContext::strict(WorkBudget::ZERO)
        .evaluate(|| curve.parameters_of(&Point3::origin()));
    assert_eq!(denied.result.unwrap_err().kind, WorkKind::RootRefinement);
    let retry = OperationContext::strict(WorkBudget {
        refinements: 512,
        promotions: 0,
    })
    .evaluate(|| parameters(&curve, &Point3::origin()));
    let values = retry.result.unwrap();
    assert_eq!(values.len(), 1);
    assert!(values[0].try_real().is_none());
    assert!(retry.usage.refinements > 0);
}

#[test]
fn parameter_comparison_rejects_ambient_approximate_equality() {
    let tiny: Real = "1e-2000".parse().unwrap();
    let difference =
        (Real::pi() + Real::from(2) * &tiny).exp().unwrap() - (Real::pi() + tiny).exp().unwrap();
    let first = Parameter::from_real(Real::one());
    let second = Parameter::from_real(Real::one() + difference);
    assert!(matches!(
        first.compare(&second),
        Err(GeometryError::PredicateUnresolved { .. })
    ));
    let provisional = hypercurve::provisional(|| first.compare(&second));
    assert!(matches!(
        provisional.into_unverified(),
        Err(GeometryError::PredicateUnresolved { .. })
    ));
}

#[test]
fn topology_transfer_reports_unmaterialized_cuts_without_changing_the_source() {
    use hyperbrep::{TopologyEditError, Vector3, builder};
    use hypercurve::{Curve2, CurvePath2, LineSeg2, Point2, RationalBezier2};

    let p = Point2::from_values;
    let outer = CurvePath2::try_new(vec![
        Curve2::from(
            RationalBezier2::try_new(vec![p(0, 0), p(0, 0), p(1, 0)], vec![Real::one(); 3])
                .unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(p(1, 0), p(1, 1)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(1, 1), p(0, 1)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, 1), p(0, 0)).unwrap()),
    ])
    .unwrap();
    let sheet =
        builder::planar_face(&outer, &[], Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
    let source_counts = sheet.model().counts();
    let cut = Curve3::line(
        Point3::new(rational(1, 2), Real::zero(), Real::zero()),
        Point3::new(rational(1, 2), Real::one(), Real::zero()),
    )
    .unwrap();
    let error = sheet
        .faces()
        .next()
        .unwrap()
        .split_by_curve(&cut)
        .unwrap_err();
    assert_eq!(
        error,
        TopologyEditError::Geometry(GeometryError::UnrepresentableParameter)
    );
    assert_eq!(sheet.model().counts(), source_counts);
}
