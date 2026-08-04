#![cfg(all(feature = "earcut", feature = "cdt"))]

#[cfg(feature = "nd")]
use hypertri::Rational;
use hypertri::{Constraint, ExactPoint, Point2, PredicatePolicy, Real, TriangulationContext};
use proptest::prelude::*;

const APPROX: TriangulationContext = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

fn p(x: i32, y: i32) -> ExactPoint {
    Point2::new(Real::from(x), Real::from(y))
}

#[cfg(feature = "nd")]
fn rational_point_d(xn: i64, xd: u64, yn: i64, yd: u64) -> hypertri::PointD {
    hypertri::PointD::new(vec![
        Real::from(Rational::fraction(xn, xd).unwrap()),
        Real::from(Rational::fraction(yn, yd).unwrap()),
    ])
}

#[cfg(feature = "nd")]
fn canonical_cells(complex: &hypertri::DelaunayComplex) -> Vec<Vec<usize>> {
    let mut cells = complex
        .cells()
        .iter()
        .map(|cell| {
            let mut indices = cell.indices().to_vec();
            indices.sort_unstable();
            indices
        })
        .collect::<Vec<_>>();
    cells.sort();
    cells
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

fn has_undirected_edge(edges: &[Constraint], expected: Constraint) -> bool {
    edges.iter().any(|edge| {
        (edge.from == expected.from && edge.to == expected.to)
            || (edge.from == expected.to && edge.to == expected.from)
    })
}

fn assert_certified_convex_hull_cdt(points: &[ExactPoint], constraints: &[Constraint]) {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let context = TriangulationContext::new(policy);
        let outcome = hypertri::cdt::constrained_delaunay(&context, points, constraints).unwrap();

        assert_eq!(
            outcome.certainty,
            hypertri::TriangulationCertainty::Certified
        );
        outcome.value.validate(&context).unwrap();
        outcome
            .value
            .validate_unconstrained_edges_are_delaunay(&context)
            .unwrap();
    }
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 256,
        failure_persistence: None,
        ..ProptestConfig::default()
    })]

    #[test]
    fn fuzz_spatial_delaunay_remains_exact_and_index_preserving(
        coordinates in prop::collection::vec((-30i32..30, -30i32..30), 5..20),
    ) {
        let mut unique = coordinates.clone();
        unique.sort_unstable();
        unique.dedup();
        prop_assume!(unique.len() == coordinates.len());

        let points = coordinates
            .into_iter()
            .map(|(x, y)| p(x, y))
            .collect::<Vec<_>>();
        let triangulation = hypertri::cdt::delaunay_spatial(&APPROX, &points)
            .unwrap()
            .value;

        triangulation.validate(&APPROX).unwrap();
        prop_assert_eq!(triangulation.points(), points.as_slice());
        prop_assert!(
            triangulation
                .triangles()
                .iter()
                .flatten()
                .all(|&index| index < points.len())
        );
    }

    #[test]
    fn fuzz_cdt_recovers_two_finite_constraints(
        coordinates in prop::collection::vec((-100i32..100, -100i32..100), 5..16),
        selectors in prop::collection::vec((any::<u16>(), any::<u16>()), 2..3),
    ) {
        let mut unique = coordinates.clone();
        unique.sort_unstable();
        unique.dedup();
        prop_assume!(unique.len() == coordinates.len());
        let (a, b) = (coordinates[0], coordinates[1]);
        prop_assume!(coordinates[2..].iter().any(|&(x, y)| {
            let ab = (i64::from(b.0 - a.0), i64::from(b.1 - a.1));
            let ap = (i64::from(x - a.0), i64::from(y - a.1));
            ab.0 * ap.1 - ab.1 * ap.0 != 0
        }));

        let mut constraints = selectors
            .into_iter()
            .map(|(from, to)| {
                let from = usize::from(from) % coordinates.len();
                let to = (from + 1 + usize::from(to) % (coordinates.len() - 1))
                    % coordinates.len();
                Constraint::new(from, to)
            })
            .collect::<Vec<_>>();
        let first = (
            constraints[0].from.min(constraints[0].to),
            constraints[0].from.max(constraints[0].to),
        );
        if (
            constraints[1].from.min(constraints[1].to),
            constraints[1].from.max(constraints[1].to),
        ) == first
        {
            constraints[1].to = (0..coordinates.len())
                .find(|&to| {
                    to != constraints[1].from
                        && (
                            constraints[1].from.min(to),
                            constraints[1].from.max(to),
                        ) != first
                })
                .unwrap();
        }
        let points = coordinates
            .into_iter()
            .map(|(x, y)| p(x, y))
            .collect::<Vec<_>>();

        assert_certified_convex_hull_cdt(&points, &constraints);
    }

    #[test]
    fn fuzz_cdt_recovers_three_edge_fans(
        coordinates in prop::collection::vec((-100i32..100, -100i32..100), 5..16),
        anchor_selector in any::<u16>(),
        target_selectors in prop::collection::vec(any::<u16>(), 3..4),
    ) {
        let mut unique = coordinates.clone();
        unique.sort_unstable();
        unique.dedup();
        prop_assume!(unique.len() == coordinates.len());
        let (a, b) = (coordinates[0], coordinates[1]);
        prop_assume!(coordinates[2..].iter().any(|&(x, y)| {
            let ab = (i64::from(b.0 - a.0), i64::from(b.1 - a.1));
            let ap = (i64::from(x - a.0), i64::from(y - a.1));
            ab.0 * ap.1 - ab.1 * ap.0 != 0
        }));

        let anchor = usize::from(anchor_selector) % coordinates.len();
        let mut used = vec![anchor];
        let constraints = target_selectors
            .into_iter()
            .map(|selector| {
                let mut target = usize::from(selector) % coordinates.len();
                while used.contains(&target) {
                    target = (target + 1) % coordinates.len();
                }
                used.push(target);
                Constraint::new(anchor, target)
            })
            .collect::<Vec<_>>();
        let points = coordinates
            .into_iter()
            .map(|(x, y)| p(x, y))
            .collect::<Vec<_>>();

        assert_certified_convex_hull_cdt(&points, &constraints);
    }

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

        let triangles = hypertri::earcut(&APPROX, &vertices, &[]).unwrap().value;
        let report = hypertri::earcut_report(&APPROX, &vertices, &[])
            .unwrap()
            .value;
        let facts = hypertri::PolygonInput::new(vertices.clone(), vec![]).facts().clone();

        prop_assert_eq!(&report.triangles, &triangles);
        prop_assert!(
            report.diagnostics.containment_convex_rejects > 0,
            "L-shape fuzz cases should exercise exact reflex/convex containment pruning"
        );
        prop_assert!(
            report.diagnostics.containment_convex_rejects
                + report.diagnostics.containment_bbox_rejects
                + report.diagnostics.containment_tests
                <= report.diagnostics.containment_candidates,
            "containment diagnostic stages must account for no more than the candidates scanned"
        );
        prop_assert_eq!(
            report.diagnostics.containment_prepared_reflex_lookups,
            report.diagnostics.containment_candidates,
            "every scanned containment candidate should use the prepared reflex/convex table"
        );
        prop_assert_eq!(facts.rings[0].known_axis_aligned_edges, 6);
        prop_assert_eq!(facts.rings[0].unknown_edge_zero_status, 0);
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

        let triangulation =
            hypertri::cdt::constrained_delaunay(&APPROX, &points, &constraints)
                .unwrap()
                .value;

        triangulation.validate(&APPROX).unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay(&APPROX)
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

        prop_assert_eq!(facts.rings[0].known_axis_aligned_edges, 4);
        prop_assert_eq!(facts.rings[0].unknown_edge_zero_status, 0);

        let triangulation =
            hypertri::cdt::constrained_delaunay(&APPROX, &points, &constraints)
                .unwrap()
                .value;

        triangulation.validate(&APPROX).unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay(&APPROX)
            .unwrap();
        prop_assert_eq!(triangulation.constraints(), constraints.as_slice());
        prop_assert_eq!(triangulation.triangles().len(), 2);
        prop_assert!(
            triangulation
                .triangles()
                .iter()
                .any(|triangle| triangle.contains(&1) && triangle.contains(&3))
        );

        let topology = hypertri::cdt::constrained_triangulation_convex_hull(
            &APPROX,
            &points,
            &constraints,
        )
        .unwrap()
        .value;
        topology.validate(&APPROX).unwrap();
        prop_assert_eq!(topology.constraints(), constraints.as_slice());
        prop_assert!(
            topology
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

        let triangulation =
            hypertri::cdt::constrained_delaunay(&APPROX, &points, &constraints)
                .unwrap()
                .value;

        triangulation.validate(&APPROX).unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay(&APPROX)
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

        let triangulation =
            hypertri::cdt::constrained_delaunay(&APPROX, &points, &constraints)
                .unwrap()
                .value;

        triangulation.validate(&APPROX).unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay(&APPROX)
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

    #[test]
    fn fuzz_cdt_recovers_separated_closed_cycles_as_general_pslg(
        x in -100i32..100,
        y in -100i32..100,
        first_width in 1i32..40,
        first_height in 1i32..40,
        gap in 1i32..40,
        second_width in 1i32..40,
        second_height in 1i32..40,
    ) {
        let second_x = x + first_width + gap;
        let points = vec![
            p(x, y),
            p(x + first_width, y),
            p(x + first_width, y + first_height),
            p(x, y + first_height),
            p(second_x, y),
            p(second_x + second_width, y),
            p(second_x + second_width, y + second_height),
            p(second_x, y + second_height),
        ];
        let constraints = vec![
            Constraint::new(0, 1),
            Constraint::new(1, 2),
            Constraint::new(2, 3),
            Constraint::new(3, 0),
            Constraint::new(4, 5),
            Constraint::new(5, 6),
            Constraint::new(6, 7),
            Constraint::new(7, 4),
        ];

        let triangulation =
            hypertri::cdt::constrained_delaunay(&APPROX, &points, &constraints)
                .unwrap()
                .value;

        triangulation.validate(&APPROX).unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay(&APPROX)
            .unwrap();
        prop_assert_eq!(triangulation.constraints(), constraints.as_slice());
        for constraint in &constraints {
            prop_assert!(has_undirected_edge(
                triangulation.constraint_edges(),
                *constraint
            ));
            prop_assert!(
                triangulation
                    .triangles()
                    .iter()
                    .any(|triangle| triangle.contains(&constraint.from)
                        && triangle.contains(&constraint.to))
            );
        }

        let topology = hypertri::cdt::constrained_triangulation_convex_hull(
            &APPROX,
            &points,
            &constraints,
        )
        .unwrap()
        .value;
        topology.validate(&APPROX).unwrap();
        for constraint in &constraints {
            prop_assert!(has_undirected_edge(
                topology.constraint_edges(),
                *constraint
            ));
        }
    }

    #[test]
    #[cfg(feature = "nd")]
    fn fuzz_nd_bistellar_flip_round_trips_on_cospherical_rectangles(
        x in -50i32..50,
        y in -50i32..50,
        width in 1i32..50,
        height in 1i32..50,
        denominator in 1u64..32,
    ) {
        let points = vec![
            rational_point_d(x as i64, denominator, y as i64, denominator),
            rational_point_d((x + width) as i64, denominator, y as i64, denominator),
            rational_point_d((x + width) as i64, denominator, (y + height) as i64, denominator),
            rational_point_d(x as i64, denominator, (y + height) as i64, denominator),
        ];
        let original = hypertri::DelaunayComplex::from_parts(
            2,
            points,
            vec![
                hypertri::Simplex::new(vec![0, 1, 2]),
                hypertri::Simplex::new(vec![0, 2, 3]),
            ],
        );
        original.validate(&APPROX).unwrap();

        let forward = hypertri::BistellarFlipD::new(vec![0, 1, 2, 3], vec![1, 3]);
        let flipped = original.flip_oracle(&APPROX, &forward).unwrap().value;
        prop_assert!(flipped.validation().is_valid());
        prop_assert_eq!(
            canonical_cells(flipped.result()),
            vec![vec![0, 1, 3], vec![1, 2, 3]]
        );

        let reverse = hypertri::BistellarFlipD::new(vec![0, 1, 2, 3], vec![0, 2]);
        let round_trip = flipped
            .result()
            .flip_oracle(&APPROX, &reverse)
            .unwrap()
            .value;
        prop_assert!(round_trip.validation().is_valid());
        prop_assert_eq!(canonical_cells(round_trip.result()), canonical_cells(&original));
    }
}
