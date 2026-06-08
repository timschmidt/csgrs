#![cfg(feature = "triangulation")]

use hypercurve::{
    BulgeVertex2, Contour2, CurveError, CurvePolicy, FiniteProjectionOptions, Point2, Real,
    Region2, triangulate_finite_rings,
};

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn rectangle(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(min_x, min_y), r(0)),
        BulgeVertex2::new(p(max_x, min_y), r(0)),
        BulgeVertex2::new(p(max_x, max_y), r(0)),
        BulgeVertex2::new(p(min_x, max_y), r(0)),
    ])
    .unwrap()
}

fn signed_area(triangles: &[[[f64; 2]; 3]]) -> f64 {
    triangles
        .iter()
        .map(|tri| {
            let [a, b, c] = *tri;
            0.5 * ((a[0] * b[1] - b[0] * a[1])
                + (b[0] * c[1] - c[0] * b[1])
                + (c[0] * a[1] - a[0] * c[1]))
        })
        .sum()
}

#[test]
fn triangulate_finite_rings_normalizes_repeated_closing_vertex() {
    let outer = [[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0], [0.0, 0.0]];

    let triangles = triangulate_finite_rings(&outer, &[]).unwrap();

    assert_eq!(triangles.len(), 2);
    assert!((signed_area(&triangles).abs() - 12.0).abs() < 1.0e-9);
}

#[test]
fn triangulate_finite_rings_normalizes_adjacent_duplicate_vertices() {
    let outer = [
        [0.0, 0.0],
        [0.0, 0.0],
        [4.0, 0.0],
        [4.0, 3.0],
        [0.0, 3.0],
        [0.0, 0.0],
    ];

    let triangles = triangulate_finite_rings(&outer, &[]).unwrap();

    assert_eq!(triangles.len(), 2);
    assert!((signed_area(&triangles).abs() - 12.0).abs() < 1.0e-9);
}

#[test]
fn triangulate_finite_rings_rejects_nonfinite_before_normalization() {
    let outer = [[0.0, 0.0], [f64::NAN, 0.0], [1.0, 1.0]];

    assert_eq!(
        triangulate_finite_rings(&outer, &[]).unwrap_err(),
        CurveError::NonFiniteProjectionPoint
    );
}

#[test]
fn triangulate_finite_rings_ignores_all_duplicate_rings() {
    let outer = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]];

    assert!(triangulate_finite_rings(&outer, &[]).unwrap().is_empty());
}

#[test]
fn triangulate_finite_rings_rejects_nonadjacent_repeated_vertices() {
    let repeated_material = [[0.0, 0.0], [4.0, 0.0], [0.0, 0.0], [0.0, 4.0]];
    assert!(matches!(
        triangulate_finite_rings(&repeated_material, &[]),
        Err(CurveError::Topology(_))
    ));

    let material = [[0.0, 0.0], [6.0, 0.0], [6.0, 6.0], [0.0, 6.0]];
    let repeated_hole = [[1.0, 1.0], [2.0, 1.0], [1.0, 1.0], [1.0, 2.0]];
    assert!(matches!(
        triangulate_finite_rings(&material, &[&repeated_hole]),
        Err(CurveError::Topology(_))
    ));
}

#[test]
fn finite_region_profile_triangulates_material_with_owned_hole() {
    let region = Region2::new(vec![rectangle(0, 0, 6, 6)], vec![rectangle(2, 2, 4, 4)]);
    let profiles = region
        .project_to_finite_profiles(
            &FiniteProjectionOptions::try_new(1.0e-3).unwrap(),
            &CurvePolicy::certified(),
        )
        .unwrap()
        .expect_decided("rectangle hole ownership should be decided");

    assert_eq!(profiles.len(), 1);
    assert_eq!(profiles[0].holes().len(), 1);

    let triangles = profiles[0].triangulate().unwrap();
    assert!(!triangles.is_empty());
    assert!((signed_area(&triangles).abs() - 32.0).abs() < 1.0e-9);
}

trait ExpectDecided<T> {
    fn expect_decided(self, message: &str) -> T;
}

impl<T> ExpectDecided<T> for hypercurve::Classification<T> {
    fn expect_decided(self, message: &str) -> T {
        match self {
            Self::Decided(value) => value,
            Self::Uncertain(reason) => panic!("{message}: {reason:?}"),
        }
    }
}
