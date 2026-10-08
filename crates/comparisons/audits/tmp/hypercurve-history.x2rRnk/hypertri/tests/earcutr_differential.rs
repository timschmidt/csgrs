#![cfg(feature = "earcut")]

use hypertri::{ExactPoint, Point2, PredicatePolicy, Real, TriangulationContext};
use proptest::prelude::*;

const APPROX: TriangulationContext = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

/// Lift ordinary finite `f64` fixtures into exact dyadic coordinates.
///
/// This keeps the comparison at the intended abstraction boundary: `earcutr`
/// remains a dev-only reference for non-degenerate runtime-shaped inputs, while
/// `hypertri` makes topology decisions through exact `Real` predicates.
fn lift_vertices(vertices: &[[f64; 2]]) -> Vec<ExactPoint> {
    vertices
        .iter()
        .map(|point| {
            Point2::new(
                Real::try_from(point[0]).unwrap(),
                Real::try_from(point[1]).unwrap(),
            )
        })
        .collect()
}

fn flatten_vertices(vertices: &[[f64; 2]]) -> Vec<f64> {
    vertices
        .iter()
        .flat_map(|point| [point[0], point[1]])
        .collect()
}

fn ring_ranges(vertex_count: usize, hole_indices: &[usize]) -> Vec<(usize, usize)> {
    let mut starts = Vec::with_capacity(hole_indices.len() + 1);
    starts.push(0);
    starts.extend_from_slice(hole_indices);

    starts
        .iter()
        .enumerate()
        .map(|(ring, &start)| {
            let end = starts.get(ring + 1).copied().unwrap_or(vertex_count);
            (start, end)
        })
        .collect()
}

fn signed_ring_area(vertices: &[[f64; 2]], start: usize, end: usize) -> f64 {
    let mut doubled = 0.0;
    for index in start..end {
        let next = if index + 1 == end { start } else { index + 1 };
        doubled += vertices[index][0] * vertices[next][1] - vertices[next][0] * vertices[index][1];
    }
    doubled * 0.5
}

fn polygon_area(vertices: &[[f64; 2]], hole_indices: &[usize]) -> f64 {
    let mut ranges = ring_ranges(vertices.len(), hole_indices).into_iter();
    let Some((outer_start, outer_end)) = ranges.next() else {
        return 0.0;
    };

    let mut area = signed_ring_area(vertices, outer_start, outer_end).abs();
    for (start, end) in ranges {
        area -= signed_ring_area(vertices, start, end).abs();
    }
    area
}

fn triangle_area_sum(vertices: &[[f64; 2]], triangles: &[usize]) -> f64 {
    triangles
        .chunks_exact(3)
        .map(|triangle| {
            let a = vertices[triangle[0]];
            let b = vertices[triangle[1]];
            let c = vertices[triangle[2]];
            ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])).abs() * 0.5
        })
        .sum()
}

fn assert_indices_in_bounds(triangles: &[usize], vertex_count: usize) {
    assert_eq!(
        triangles.len() % 3,
        0,
        "triangle index buffer is flat triples"
    );
    for &index in triangles {
        assert!(
            index < vertex_count,
            "triangle index {index} exceeded vertex count {vertex_count}"
        );
    }
}

fn assert_ordinary_case_matches_earcutr(vertices: &[[f64; 2]], hole_indices: &[usize]) {
    let exact_vertices = lift_vertices(vertices);
    let hypertri_triangles = hypertri::earcut(&APPROX, &exact_vertices, hole_indices)
        .unwrap()
        .value;

    let flat = flatten_vertices(vertices);
    let earcutr_triangles = earcutr::earcut(&flat, hole_indices, 2).unwrap();

    assert_indices_in_bounds(&hypertri_triangles, vertices.len());
    assert_indices_in_bounds(&earcutr_triangles, vertices.len());
    assert_eq!(
        hypertri_triangles.len(),
        earcutr_triangles.len(),
        "ordinary input should produce the same triangle count as earcutr"
    );

    let expected_area = polygon_area(vertices, hole_indices);
    let hypertri_area = triangle_area_sum(vertices, &hypertri_triangles);
    let earcutr_area = triangle_area_sum(vertices, &earcutr_triangles);

    assert!(
        expected_area > 0.0,
        "differential fixtures must be ordinary non-degenerate polygons"
    );
    assert!(
        (hypertri_area - expected_area).abs() <= 1.0e-9,
        "hypertri area {hypertri_area} did not match polygon area {expected_area}"
    );
    assert!(
        (earcutr_area - expected_area).abs() <= 1.0e-9,
        "earcutr area {earcutr_area} did not match polygon area {expected_area}"
    );
}

#[test]
fn ordinary_convex_polygons_match_earcutr_topology_and_area() {
    assert_ordinary_case_matches_earcutr(
        &[[0.0, 0.0], [4.0, 0.0], [5.0, 2.0], [2.0, 4.0], [-1.0, 2.0]],
        &[],
    );
}

#[test]
fn ordinary_concave_polygons_match_earcutr_topology_and_area() {
    assert_ordinary_case_matches_earcutr(
        &[
            [0.0, 0.0],
            [4.0, 0.0],
            [4.0, 1.0],
            [1.5, 1.0],
            [1.5, 3.5],
            [0.0, 3.5],
        ],
        &[],
    );
}

#[test]
fn ordinary_holed_polygons_match_earcutr_topology_and_area() {
    assert_ordinary_case_matches_earcutr(
        &[
            [0.0, 0.0],
            [6.0, 0.0],
            [6.0, 6.0],
            [0.0, 6.0],
            [2.0, 2.0],
            [2.0, 4.0],
            [4.0, 4.0],
            [4.0, 2.0],
        ],
        &[4],
    );
}

#[test]
fn ordinary_reversed_outer_ring_matches_earcutr_topology_and_area() {
    assert_ordinary_case_matches_earcutr(
        &[[-1.0, 2.0], [2.0, 4.0], [5.0, 2.0], [4.0, 0.0], [0.0, 0.0]],
        &[],
    );
}

proptest! {
    #[test]
    fn ordinary_axis_aligned_rectangles_match_earcutr(
        min_x in -64_i32..63,
        min_y in -64_i32..63,
        width in 1_i32..64,
        height in 1_i32..64,
    ) {
        let max_x = min_x + width;
        let max_y = min_y + height;
        let vertices = [
            [f64::from(min_x), f64::from(min_y)],
            [f64::from(max_x), f64::from(min_y)],
            [f64::from(max_x), f64::from(max_y)],
            [f64::from(min_x), f64::from(max_y)],
        ];

        assert_ordinary_case_matches_earcutr(&vertices, &[]);
    }

    #[test]
    fn ordinary_rectangular_holes_match_earcutr(
        origin_x in -32_i32..32,
        origin_y in -32_i32..32,
        outer_width in 8_i32..48,
        outer_height in 8_i32..48,
        inset in 1_i32..4,
    ) {
        prop_assume!(inset * 2 < outer_width);
        prop_assume!(inset * 2 < outer_height);

        let left = origin_x;
        let bottom = origin_y;
        let right = origin_x + outer_width;
        let top = origin_y + outer_height;
        let hole_left = left + inset;
        let hole_bottom = bottom + inset;
        let hole_right = right - inset;
        let hole_top = top - inset;
        let vertices = [
            [f64::from(left), f64::from(bottom)],
            [f64::from(right), f64::from(bottom)],
            [f64::from(right), f64::from(top)],
            [f64::from(left), f64::from(top)],
            [f64::from(hole_left), f64::from(hole_bottom)],
            [f64::from(hole_left), f64::from(hole_top)],
            [f64::from(hole_right), f64::from(hole_top)],
            [f64::from(hole_right), f64::from(hole_bottom)],
        ];

        assert_ordinary_case_matches_earcutr(&vertices, &[4]);
    }
}
