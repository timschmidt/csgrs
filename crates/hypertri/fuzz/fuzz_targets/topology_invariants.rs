//! Fuzz exact triangulation topology over generated rational inputs.
//!
//! The harness stays on `hyperreal::Real` coordinates through `hypertri`'s
//! public APIs. It checks topology invariants and CDT validators rather than
//! comparing against a primitive-float implementation: generated data may be
//! adversarial, but irreversible decisions still belong to exact predicates.

#![no_main]

use arbitrary::Arbitrary;
use hypertri::{
    Constraint, ExactPoint, Point2, PredicatePolicy, Rational, Real, TriangleIndices,
    TriangulationContext,
};
use libfuzzer_sys::fuzz_target;

const APPROX: TriangulationContext =
    TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

#[derive(Clone, Copy, Debug, Arbitrary)]
struct RawInput {
    origin_x: i16,
    origin_y: i16,
    width: u8,
    height: u8,
    notch_x: u8,
    notch_y: u8,
    gap: u8,
    second_width: u8,
    second_height: u8,
    tiny_den: u8,
}

fuzz_target!(|input: RawInput| {
    exercise_l_shape_earcut(input);
    exercise_crossing_constraint_cdt(input);
    exercise_separated_cycle_cdt(input);
    exercise_exact_nd_complex(input);
});

fn exercise_l_shape_earcut(input: RawInput) {
    let width = positive_i32(input.width, 3, 66);
    let height = positive_i32(input.height, 3, 66);
    let notch_x = bounded_i32(input.notch_x, 1, width - 1);
    let notch_y = bounded_i32(input.notch_y, 1, height - 1);
    let x = i32::from(input.origin_x);
    let y = i32::from(input.origin_y);

    let vertices = vec![
        p(x, y),
        p(x + width, y),
        p(x + width, y + height),
        p(x + notch_x, y + height),
        p(x + notch_x, y + notch_y),
        p(x, y + notch_y),
    ];

    let Ok(outcome) = hypertri::earcut(&APPROX, &vertices, &[]) else {
        return;
    };
    let triangles = outcome.value;
    assert_triangle_indices(&triangles, vertices.len());
}

fn exercise_crossing_constraint_cdt(input: RawInput) {
    let width = positive_i32(input.width, 2, 66);
    let height = positive_i32(input.height, 2, 66);
    let den = u64::from(input.tiny_den).saturating_add(1);
    let x = i32::from(input.origin_x);
    let y = i32::from(input.origin_y);

    let points = vec![
        p(x, y),
        point(q(i64::from(x + width), 1), q(i64::from(y + height), den)),
        point(q(i64::from(x), 1), q(i64::from(y + height), den)),
        p(x + width, y),
    ];
    let constraints = [Constraint::new(0, 1), Constraint::new(2, 3)];

    let Ok(outcome) =
        hypertri::cdt::constrained_delaunay(&APPROX, &points, &constraints)
    else {
        return;
    };
    let triangulation = outcome.value;
    triangulation
        .validate(&APPROX)
        .expect("CDT topology must validate");
    triangulation
        .validate_unconstrained_edges_are_delaunay(&APPROX)
        .expect("unprotected CDT edges must be locally Delaunay");
    assert_eq!(triangulation.constraints(), constraints);
    assert!(triangulation.points().len() >= points.len());
}

fn exercise_separated_cycle_cdt(input: RawInput) {
    let first_width = positive_i32(input.width, 1, 40);
    let first_height = positive_i32(input.height, 1, 40);
    let gap = positive_i32(input.gap, 1, 40);
    let second_width = positive_i32(input.second_width, 1, 40);
    let second_height = positive_i32(input.second_height, 1, 40);
    let x = i32::from(input.origin_x);
    let y = i32::from(input.origin_y);
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
    let constraints = [
        Constraint::new(0, 1),
        Constraint::new(1, 2),
        Constraint::new(2, 3),
        Constraint::new(3, 0),
        Constraint::new(4, 5),
        Constraint::new(5, 6),
        Constraint::new(6, 7),
        Constraint::new(7, 4),
    ];

    let Ok(outcome) =
        hypertri::cdt::constrained_delaunay(&APPROX, &points, &constraints)
    else {
        return;
    };
    let triangulation = outcome.value;
    triangulation
        .validate(&APPROX)
        .expect("CDT topology must validate");
    triangulation
        .validate_unconstrained_edges_are_delaunay(&APPROX)
        .expect("unprotected CDT edges must be locally Delaunay");
    for constraint in constraints {
        assert!(
            triangulation
                .triangles()
                .iter()
                .any(|triangle| triangle.contains(&constraint.from) && triangle.contains(&constraint.to)),
            "every generated protected edge must appear in CDT triangles"
        );
    }
}

fn exercise_exact_nd_complex(input: RawInput) {
    let width = positive_i32(input.width, 2, 40);
    let height = positive_i32(input.height, 2, 40);
    let depth = positive_i32(input.second_height, 2, 40);
    let den = u64::from(input.tiny_den).saturating_add(2);
    let x = i32::from(input.origin_x);
    let y = i32::from(input.origin_y);

    let points = vec![
        point_d(&[r(i64::from(x)), r(i64::from(y)), r(0)]),
        point_d(&[r(i64::from(x + width)), r(i64::from(y)), r(0)]),
        point_d(&[r(i64::from(x)), r(i64::from(y + height)), r(0)]),
        point_d(&[r(i64::from(x)), r(i64::from(y)), r(i64::from(depth))]),
        point_d(&[
            q(i64::from(x * 4 + width), 4),
            q(i64::from(y * 4 + height), 4),
            q(i64::from(depth), den),
        ]),
    ];

    let Ok(outcome) = hypertri::nd::delaunay_complex(&APPROX, &points) else {
        return;
    };
    let complex = outcome.value;
    complex
        .validate(&APPROX)
        .expect("exact ND Delaunay complex must validate");
    for cell in complex.cells() {
        assert_eq!(cell.indices().len(), complex.dimension() + 1);
        assert!(cell.indices().iter().all(|&index| index < complex.points().len()));
    }
}

fn assert_triangle_indices(indices: &TriangleIndices, vertex_count: usize) {
    assert_eq!(indices.len() % 3, 0);
    for triangle in indices.chunks_exact(3) {
        assert!(triangle.iter().all(|&index| index < vertex_count));
        assert_ne!(triangle[0], triangle[1]);
        assert_ne!(triangle[1], triangle[2]);
        assert_ne!(triangle[2], triangle[0]);
    }
}

fn positive_i32(value: u8, min: i32, span: i32) -> i32 {
    min + i32::from(value) % span
}

fn bounded_i32(value: u8, min: i32, max_inclusive: i32) -> i32 {
    min + i32::from(value) % (max_inclusive - min + 1)
}

fn p(x: i32, y: i32) -> ExactPoint {
    point(Real::from(x), Real::from(y))
}

fn point(x: Real, y: Real) -> ExactPoint {
    Point2::new(x, y)
}

fn q(numerator: i64, denominator: u64) -> Real {
    Real::from(Rational::fraction(numerator, denominator).expect("positive denominator"))
}

fn r(value: i64) -> Real {
    Real::from(value)
}

fn point_d(coordinates: &[Real]) -> hypertri::nd::PointD {
    hypertri::nd::PointD::new(coordinates.to_vec())
}
