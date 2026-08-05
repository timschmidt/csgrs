#![cfg(any(feature = "earcut", feature = "cdt", feature = "nd"))]

#[cfg(any(feature = "earcut", feature = "cdt"))]
use hypertri::Point2;
use hypertri::{Error, PredicatePolicy, Real, TriangulationCertainty, TriangulationContext};

const STRICT: TriangulationContext = TriangulationContext::new(PredicatePolicy::STRICT);
const APPROX: TriangulationContext = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

fn terminal_equality() -> (Real, Real) {
    (Real::pi() + Real::e(), Real::e() + Real::pi())
}

#[cfg(feature = "earcut")]
#[test]
fn earcut_reports_terminal_policy_consumption() {
    let (left, right) = terminal_equality();
    let vertices = vec![
        Point2::new(left.clone(), Real::zero()),
        Point2::new(&left + &Real::one(), Real::zero()),
        Point2::new(&left + &Real::one(), Real::one()),
        Point2::new(left, Real::one()),
        Point2::new(right, Real::zero()),
    ];

    assert!(matches!(
        hypertri::earcut(&STRICT, &vertices, &[]),
        Err(Error::PredicateUndecided { .. })
    ));
    let outcome = hypertri::earcut(&APPROX, &vertices, &[]).unwrap();
    assert_eq!(outcome.value.len(), 6);
    assert_eq!(
        outcome.certainty,
        TriangulationCertainty::Approximate512Consumed
    );
}

#[cfg(feature = "cdt")]
#[test]
fn delaunay_reports_terminal_policy_consumption() {
    let (left, right) = terminal_equality();
    let points = vec![
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(Real::one(), Real::one()),
        Point2::new(left, right),
    ];

    assert!(matches!(
        hypertri::cdt::delaunay(&STRICT, &points),
        Err(Error::PredicateUndecided { .. })
    ));
    let outcome = hypertri::cdt::delaunay(&APPROX, &points).unwrap();
    assert!(outcome.value.triangles().is_empty());
    assert_eq!(
        outcome.certainty,
        TriangulationCertainty::Approximate512Consumed
    );
}

#[cfg(feature = "cdt")]
#[test]
fn constrained_topology_reports_terminal_policy_consumption() {
    let (left, right) = terminal_equality();
    let points = vec![
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(Real::one(), Real::one()),
        Point2::new(left, right),
        Point2::new(Real::zero(), Real::one()),
    ];

    assert!(matches!(
        hypertri::cdt::constrained_triangulation_convex_hull(&STRICT, &points, &[]),
        Err(Error::PredicateUndecided { .. })
    ));
    let outcome =
        hypertri::cdt::constrained_triangulation_convex_hull(&APPROX, &points, &[]).unwrap();
    assert_eq!(outcome.value.len(), 2);
    assert_eq!(
        outcome.certainty,
        TriangulationCertainty::Approximate512Consumed
    );
}

#[cfg(feature = "nd")]
#[test]
fn nd_oracle_reports_terminal_policy_consumption() {
    let (left, right) = terminal_equality();
    let points = vec![
        hypertri::PointD::new(vec![Real::zero(), Real::zero()]),
        hypertri::PointD::new(vec![Real::one(), Real::one()]),
        hypertri::PointD::new(vec![left, right]),
    ];

    assert!(matches!(
        hypertri::nd::delaunay_complex(&STRICT, &points),
        Err(Error::PredicateUndecided { .. })
    ));
    let outcome = hypertri::nd::delaunay_complex(&APPROX, &points).unwrap();
    assert!(outcome.value.cells().is_empty());
    assert_eq!(
        outcome.certainty,
        TriangulationCertainty::Approximate512Consumed
    );
}

#[test]
fn exact_rational_work_stays_certified_under_both_policies() {
    let contexts = [STRICT, APPROX];
    for context in contexts {
        #[cfg(feature = "earcut")]
        {
            let triangle = [
                Point2::new(Real::zero(), Real::zero()),
                Point2::new(Real::one(), Real::zero()),
                Point2::new(Real::zero(), Real::one()),
            ];
            assert_eq!(
                hypertri::earcut(&context, &triangle, &[])
                    .unwrap()
                    .certainty,
                TriangulationCertainty::Certified
            );
        }

        #[cfg(feature = "cdt")]
        {
            let triangle = [
                Point2::new(Real::zero(), Real::zero()),
                Point2::new(Real::one(), Real::zero()),
                Point2::new(Real::zero(), Real::one()),
            ];
            assert_eq!(
                hypertri::cdt::delaunay(&context, &triangle)
                    .unwrap()
                    .certainty,
                TriangulationCertainty::Certified
            );
            assert_eq!(
                hypertri::cdt::constrained_triangulation_convex_hull(&context, &triangle, &[])
                    .unwrap()
                    .certainty,
                TriangulationCertainty::Certified
            );
        }

        #[cfg(feature = "nd")]
        {
            let triangle = [
                hypertri::PointD::new(vec![Real::zero(), Real::zero()]),
                hypertri::PointD::new(vec![Real::one(), Real::zero()]),
                hypertri::PointD::new(vec![Real::zero(), Real::one()]),
            ];
            assert_eq!(
                hypertri::nd::delaunay_complex(&context, &triangle)
                    .unwrap()
                    .certainty,
                TriangulationCertainty::Certified
            );
        }
    }
}
