#![cfg(all(feature = "earcut", feature = "cdt"))]

use hypertri::{Constraint, ExactPoint, Point2, Real};
use proptest::prelude::*;

fn p(x: i32, y: i32) -> ExactPoint {
    Point2::new(Real::from(x), Real::from(y))
}

fn selected_triangle_edges(mask: u8) -> Vec<Constraint> {
    let mut constraints = Vec::new();
    if mask & 0b001 != 0 {
        constraints.push(Constraint::new(0, 1));
    }
    if mask & 0b010 != 0 {
        constraints.push(Constraint::new(1, 2));
    }
    if mask & 0b100 != 0 {
        constraints.push(Constraint::new(2, 0));
    }
    constraints
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 256,
        failure_persistence: None,
        ..ProptestConfig::default()
    })]

    #[test]
    fn fuzz_exact_earcut_l_shapes_keep_index_topology(
        x in -100i32..100,
        y in -100i32..100,
        width in 3i32..100,
        height in 3i32..100,
        notch_x in 1i32..99,
        notch_y in 1i32..99,
    ) {
        prop_assume!(notch_x < width);
        prop_assume!(notch_y < height);

        let vertices = vec![
            p(x, y),
            p(x + width, y),
            p(x + width, y + height),
            p(x + notch_x, y + height),
            p(x + notch_x, y + notch_y),
            p(x, y + notch_y),
        ];

        let triangles = hypertri::earcut(&vertices, &[]).unwrap();
        let facts = hypertri::PolygonInput::new(vertices.clone(), vec![]).facts().clone();

        prop_assert_eq!(facts.rings[0].signed_area, Some(hypertri::Sign::Positive));
        prop_assert_eq!(facts.rings[0].convexity, hypertri::RingConvexity::MixedTurns);
        prop_assert!(facts.all_ring_orientations_certified());
        prop_assert_eq!(triangles.len(), 12);
        for triangle in triangles.chunks_exact(3) {
            prop_assert!(triangle.iter().all(|&index| index < vertices.len()));
            prop_assert_ne!(triangle[0], triangle[1]);
            prop_assert_ne!(triangle[1], triangle[2]);
            prop_assert_ne!(triangle[2], triangle[0]);
        }
    }

    #[test]
    fn fuzz_cdt_accepts_constraints_that_are_already_delaunay_edges(
        x in -100i32..100,
        y in -100i32..100,
        width in 1i32..100,
        height in 1i32..100,
        mask in 1u8..8,
    ) {
        let points = vec![
            p(x, y),
            p(x + width, y),
            p(x, y + height),
        ];
        let constraints = selected_triangle_edges(mask);

        let triangulation = hypertri::cdt::constrained_delaunay(&points, &constraints).unwrap();

        triangulation.validate().unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay()
            .unwrap();
        prop_assert_eq!(triangulation.constraints(), constraints.as_slice());
        prop_assert_eq!(triangulation.triangles().len(), 1);
        for constraint in &constraints {
            prop_assert!(
                triangulation
                    .triangles()
                    .iter()
                    .any(|triangle| triangle.contains(&constraint.from)
                        && triangle.contains(&constraint.to))
            );
        }
    }

    #[test]
    fn fuzz_cdt_recovers_opposite_rectangle_diagonal_by_flips(
        x in -100i32..100,
        y in -100i32..100,
        width in 1i32..100,
        height in 1i32..100,
    ) {
        let points = vec![
            p(x, y),
            p(x + width, y),
            p(x + width, y + height),
            p(x, y + height),
        ];
        let constraints = vec![Constraint::new(1, 3)];
        let facts = hypertri::PolygonInput::new(points.clone(), vec![]).facts().clone();

        prop_assert_eq!(facts.rings[0].signed_area, Some(hypertri::Sign::Positive));
        prop_assert_eq!(facts.rings[0].convexity, hypertri::RingConvexity::LocallyConvex);

        let triangulation = hypertri::cdt::constrained_delaunay(&points, &constraints).unwrap();

        triangulation.validate().unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay()
            .unwrap();
        prop_assert_eq!(triangulation.constraints(), constraints.as_slice());
        prop_assert_eq!(triangulation.triangles().len(), 2);
        prop_assert!(
            triangulation
                .triangles()
                .iter()
                .any(|triangle| triangle.contains(&1) && triangle.contains(&3))
        );
    }

    #[test]
    fn fuzz_cdt_splits_constraint_through_existing_vertex(
        x in -100i32..100,
        y in -100i32..100,
        width in 2i32..100,
        height in 1i32..100,
        split in 1i32..99,
    ) {
        prop_assume!(split < width);
        let points = vec![
            p(x, y),
            p(x + width, y),
            p(x + split, y),
            p(x, y + height),
        ];
        let constraints = vec![Constraint::new(0, 1)];

        let triangulation = hypertri::cdt::constrained_delaunay(&points, &constraints).unwrap();

        triangulation.validate().unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay()
            .unwrap();
        prop_assert_eq!(triangulation.constraints(), constraints.as_slice());
        prop_assert_eq!(
            triangulation.constraint_edges(),
            &[Constraint::new(0, 2), Constraint::new(2, 1)]
        );
        for (from, to) in [(0, 2), (2, 1)] {
            prop_assert!(
                triangulation
                    .triangles()
                    .iter()
                    .any(|triangle| triangle.contains(&from) && triangle.contains(&to))
            );
        }
    }

    #[test]
    fn fuzz_cdt_splits_crossing_constraints_at_inserted_vertex(
        x in -100i32..100,
        y in -100i32..100,
        width in 1i32..100,
        height in 1i32..100,
    ) {
        let points = vec![
            p(x, y),
            p(x + width, y + height),
            p(x, y + height),
            p(x + width, y),
        ];
        let constraints = vec![Constraint::new(0, 1), Constraint::new(2, 3)];

        let triangulation = hypertri::cdt::constrained_delaunay(&points, &constraints).unwrap();

        triangulation.validate().unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay()
            .unwrap();
        prop_assert_eq!(triangulation.constraints(), constraints.as_slice());
        prop_assert_eq!(triangulation.points().len(), 5);
        prop_assert_eq!(
            triangulation.constraint_edges(),
            &[
                Constraint::new(0, 4),
                Constraint::new(4, 1),
                Constraint::new(2, 4),
                Constraint::new(4, 3),
            ]
        );
        for (from, to) in [(0, 4), (4, 1), (2, 4), (4, 3)] {
            prop_assert!(
                triangulation
                    .triangles()
                    .iter()
                    .any(|triangle| triangle.contains(&from) && triangle.contains(&to))
            );
        }
    }
}
