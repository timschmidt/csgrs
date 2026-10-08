use std::collections::BTreeSet;

use exactcore_hyper_comparison::delaunay_dt4;
use hypertri::{ExactPoint, Point2, PointD, PredicatePolicy, Real, TriangulationContext};
use proptest::prelude::*;

const CONTEXT: TriangulationContext = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

fn exact_point(point: [i64; 2]) -> ExactPoint {
    Point2::new(Real::from(point[0]), Real::from(point[1]))
}

fn point_d(point: [i64; 2]) -> PointD {
    PointD::new(vec![Real::from(point[0]), Real::from(point[1])])
}

fn canonical_core(points: &[[i64; 2]]) -> BTreeSet<[usize; 3]> {
    delaunay_dt4(points)
        .unwrap()
        .into_iter()
        .map(|triangle| {
            let mut triangle = triangle.map(|index| index as usize);
            triangle.sort_unstable();
            triangle
        })
        .collect()
}

fn canonical_hyper_complex(points: &[[i64; 2]]) -> BTreeSet<[usize; 3]> {
    let points = points.iter().copied().map(point_d).collect::<Vec<_>>();
    hypertri::nd::delaunay_complex(&CONTEXT, &points)
        .unwrap()
        .value
        .cells()
        .iter()
        .map(|simplex| {
            let mut triangle: [usize; 3] = simplex.indices().try_into().unwrap();
            triangle.sort_unstable();
            triangle
        })
        .collect()
}

fn canonical_hyper_triangulation(points: &[[i64; 2]]) -> BTreeSet<[usize; 3]> {
    let points = points.iter().copied().map(exact_point).collect::<Vec<_>>();
    let result = hypertri::cdt::delaunay(&CONTEXT, &points).unwrap().value;
    result.validate(&CONTEXT).unwrap();
    result
        .triangles()
        .iter()
        .map(|triangle| {
            let mut triangle = *triangle;
            triangle.sort_unstable();
            triangle
        })
        .collect()
}

#[test]
fn exhaustive_empty_circle_delaunay_cells_match() {
    let cases: &[&[[i64; 2]]] = &[
        &[[0, 0], [4, 0], [0, 3]],
        &[[0, 0], [5, 0], [1, 4], [2, 1]],
        &[[0, 0], [6, 0], [7, 4], [3, 7], [-2, 4], [2, 3]],
        &[[0, 0], [4, 0], [4, 4], [0, 4]],
        &[[0, 0], [8, 0], [0, 8], [2, 2]],
        &[[0, 0], [1, 0], [2, 0], [3, 0]],
    ];

    for points in cases {
        assert_eq!(
            canonical_core(points),
            canonical_hyper_complex(points),
            "Delaunay complex mismatch for {points:?}",
        );
    }
}

#[test]
fn ordinary_hypertri_delaunay_equals_core_when_the_complex_is_simplicial() {
    let cases: &[&[[i64; 2]]] = &[
        &[[0, 0], [4, 0], [0, 3]],
        &[[0, 0], [5, 0], [1, 4], [2, 1]],
        &[[0, 0], [6, 0], [7, 4], [3, 7], [-2, 4], [2, 3]],
        &[[0, 0], [8, 0], [0, 8], [2, 2]],
    ];

    for points in cases {
        assert_eq!(
            canonical_core(points),
            canonical_hyper_triangulation(points),
            "unique triangulation mismatch for {points:?}",
        );
    }
}

#[test]
fn cocircular_core_complex_contains_hypertri_tie_broken_triangulation() {
    let points = [[0, 0], [4, 0], [4, 4], [0, 4]];
    let core = canonical_core(&points);
    let complex = canonical_hyper_complex(&points);
    let triangulation = canonical_hyper_triangulation(&points);

    assert_eq!(core, complex);
    assert_eq!(core.len(), 4, "every square triple is an empty-circle cell");
    assert_eq!(triangulation.len(), 2);
    assert!(triangulation.is_subset(&core));
}

#[test]
fn collinear_inputs_have_no_two_dimensional_cells() {
    for points in [
        vec![[0, 0], [1, 0], [2, 0]],
        vec![[-3, -5], [-1, -1], [1, 3], [3, 7], [5, 11]],
    ] {
        assert!(canonical_core(&points).is_empty());
        assert!(canonical_hyper_complex(&points).is_empty());
    }
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 96,
        failure_persistence: None,
        ..ProptestConfig::default()
    })]

    #[test]
    fn integer_point_delaunay_complexes_match(
        points in prop::collection::vec((-12_i64..=12, -12_i64..=12), 3..9),
    ) {
        let points = points
            .into_iter()
            .map(|(x, y)| [x, y])
            .collect::<Vec<_>>();
        let unique = points.iter().copied().collect::<BTreeSet<_>>();
        prop_assume!(unique.len() == points.len());

        prop_assert_eq!(canonical_core(&points), canonical_hyper_complex(&points));
    }
}

#[test]
#[ignore = "the exactCore-backed adapter accepts duplicate coordinates while hypertri rejects them explicitly"]
fn duplicate_point_preconditions_match() {
    let points = [[0, 0], [4, 0], [0, 4], [0, 0]];
    let hyper_points = points.iter().copied().map(point_d).collect::<Vec<_>>();
    let core = delaunay_dt4(&points);
    let hyper = hypertri::nd::delaunay_complex(&CONTEXT, &hyper_points);
    assert_eq!(core.is_err(), hyper.is_err());
}
