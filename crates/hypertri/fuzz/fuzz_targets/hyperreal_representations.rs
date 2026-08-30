//! Cross every finite optimized Hyperreal certificate through Hypertri.

#![no_main]

use hyperreal::{Rational, Real, StructuralKind};
use hypertri::{
    Constraint, Point2, PointD, PolygonInput, PredicatePolicy, TriangulationContext,
    TriangulationOptions,
};
use libfuzzer_sys::fuzz_target;

const APPROX: TriangulationContext = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

#[derive(Clone, Copy, Debug)]
struct Input {
    width: u8,
    height: u8,
    interior_x: u8,
    interior_y: u8,
    representation_stride: u8,
    graph_depth: u8,
    graph_opcode: u8,
}

fuzz_target!(|data: &[u8]| {
    // Default missing bytes to zero so even libFuzzer's empty/minimal corpus
    // executes the complete finite representation inventory instead of being
    // rejected by a fixed-size `Arbitrary` decoder.
    let byte = |index| data.get(index).copied().unwrap_or(0);
    let input = Input {
        width: byte(0),
        height: byte(1),
        interior_x: byte(2),
        interior_y: byte(3),
        representation_stride: byte(4),
        graph_depth: byte(5),
        graph_opcode: byte(6),
    };
    let width = i64::from(input.width % 29) + 3;
    let height = i64::from(input.height % 29) + 3;
    let interior_x = i64::from(input.interior_x) % (width - 1) + 1;
    let interior_y = i64::from(input.interior_y) % (height - 1) + 1;
    let mut values = representative_values();
    values.extend(opaque_graph_values(input.graph_depth, input.graph_opcode));
    let stride = usize::from(input.representation_stride) % values.len();

    for (index, tx) in values.iter().enumerate() {
        // Every execution visits all 22 finite classes. Input bytes rotate the
        // second coordinate so libFuzzer reaches every ordered cross-class
        // pairing without an O(n^2) loop on each execution.
        let ty = &values[(index + stride) % values.len()];
        exercise_earcut(tx, ty, width, height);
        exercise_delaunay(tx, ty, width, height, interior_x, interior_y);
        exercise_crossing_constraint_cdt(tx, ty, width, height);
        exercise_nd_delaunay(tx, ty, width, height);
        exercise_runtime_selection(tx, ty, width, height);
    }
});

fn exercise_earcut(tx: &Real, ty: &Real, width: i64, height: i64) {
    let vertices = rectangle(tx, ty, width, height);
    let outcome = hypertri::earcut(&APPROX, &vertices, &[]).expect("translated rectangle earcut");
    assert_triangle_indices(&outcome.value, vertices.len(), 2);
}

fn exercise_delaunay(
    tx: &Real,
    ty: &Real,
    width: i64,
    height: i64,
    interior_x: i64,
    interior_y: i64,
) {
    let points = vec![
        point(tx, ty, 0, 0),
        point(tx, ty, width, 0),
        point(tx, ty, width, height),
        point(tx, ty, 0, height),
        point(tx, ty, interior_x, interior_y),
    ];
    for outcome in [
        hypertri::cdt::delaunay(&APPROX, &points),
        hypertri::cdt::delaunay_spatial(&APPROX, &points),
    ] {
        let triangulation = outcome.expect("translated exact Delaunay").value;
        assert_eq!(triangulation.points().len(), points.len());
        assert_eq!(triangulation.triangles().len(), 4);
        triangulation
            .validate(&APPROX)
            .expect("valid translated exact Delaunay");
    }
}

fn exercise_crossing_constraint_cdt(tx: &Real, ty: &Real, width: i64, height: i64) {
    let points = vec![
        point(tx, ty, 0, 0),
        point(tx, ty, width, height),
        point(tx, ty, 0, height),
        point(tx, ty, width, 0),
    ];
    let constraints = [Constraint::new(0, 1), Constraint::new(2, 3)];
    let triangulation = hypertri::cdt::constrained_delaunay(&APPROX, &points, &constraints)
        .expect("translated crossing constraints")
        .value;
    assert_eq!(triangulation.points().len(), 5);
    assert_eq!(triangulation.constraint_edges().len(), 4);
    triangulation
        .validate(&APPROX)
        .expect("valid translated constrained topology");
    triangulation
        .validate_unconstrained_edges_are_delaunay(&APPROX)
        .expect("valid translated constrained Delaunay edges");
}

fn exercise_nd_delaunay(tx: &Real, ty: &Real, width: i64, height: i64) {
    let tz = tx + ty;
    let point_d = |x, y, z| {
        PointD::new(vec![
            tx + Real::from(x),
            ty + Real::from(y),
            &tz + Real::from(z),
        ])
    };
    let points = vec![
        point_d(0, 0, 0),
        point_d(width, 0, 0),
        point_d(0, height, 0),
        point_d(0, 0, width + height),
        PointD::new(vec![
            tx + rational(width, 4),
            ty + rational(height, 4),
            &tz + rational(width + height, 4),
        ]),
    ];
    let complex = hypertri::nd::delaunay_complex(&APPROX, &points)
        .expect("translated exact N-D Delaunay")
        .value;
    assert_eq!(complex.dimension(), 3);
    assert_eq!(complex.points().len(), points.len());
    assert!(!complex.cells().is_empty());
    complex
        .validate(&APPROX)
        .expect("valid translated exact N-D topology");
}

fn exercise_runtime_selection(tx: &Real, ty: &Real, width: i64, height: i64) {
    let input = PolygonInput::new(rectangle(tx, ty, width, height), Vec::new());
    let outcome = hypertri::triangulate_polygon(&APPROX, &input, TriangulationOptions::default())
        .expect("translated runtime-selected triangulation");
    assert_triangle_indices(&outcome.value, input.vertices().len(), 2);
}

fn rectangle(tx: &Real, ty: &Real, width: i64, height: i64) -> Vec<Point2> {
    vec![
        point(tx, ty, 0, 0),
        point(tx, ty, width, 0),
        point(tx, ty, width, height),
        point(tx, ty, 0, height),
    ]
}

fn point(tx: &Real, ty: &Real, x: i64, y: i64) -> Point2 {
    Point2::new(tx + Real::from(x), ty + Real::from(y))
}

fn rational(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

fn assert_triangle_indices(indices: &[usize], vertex_count: usize, expected_triangles: usize) {
    assert_eq!(indices.len(), expected_triangles * 3);
    for triangle in indices.chunks_exact(3) {
        assert!(triangle.iter().all(|&index| index < vertex_count));
        assert_ne!(triangle[0], triangle[1]);
        assert_ne!(triangle[1], triangle[2]);
        assert_ne!(triangle[2], triangle[0]);
    }
}

fn representative_values() -> Vec<Real> {
    let pi = Real::pi();
    let e = Real::e();
    let pi_squared = &pi * &pi;
    let sqrt_two = Real::from(2).sqrt().expect("positive radicand");
    let ln_two = Real::from(2).ln().expect("positive logarithm input");
    let ln_three = Real::from(3).ln().expect("positive logarithm input");
    let values = vec![
        rational(3, 2),
        pi.clone(),
        pi_squared.clone(),
        pi.clone().inverse().expect("pi is nonzero"),
        &pi * &e,
        (&e / &pi).expect("pi is nonzero"),
        &pi * &sqrt_two,
        &pi_squared * &e,
        &pi - Real::from(3),
        &(&pi_squared * &e) * &sqrt_two,
        sqrt_two,
        Real::from(2).exp().expect("finite exponential"),
        ln_three.clone(),
        (Real::from(2) * &e).ln().expect("positive logarithm input"),
        &ln_two * &ln_three,
        Real::from(2).log10().expect("positive logarithm input"),
        Real::from(3).log2().expect("positive logarithm input"),
        rational(1, 7)
            .exp10()
            .expect("finite rational base-ten power"),
        rational(1, 7)
            .exp2()
            .expect("finite rational base-two power"),
        rational(1, 5).sin_pi(),
        rational(1, 5)
            .tan_pi()
            .expect("one fifth of a turn is not a tangent pole"),
        Real::one().sin(),
    ];
    assert_eq!(values.len(), 22, "update the private Real class corpus");
    assert_eq!(
        values
            .iter()
            .map(|value| value.detailed_facts().symbolic.kind)
            .collect::<Vec<_>>(),
        vec![
            StructuralKind::ExactRational,
            StructuralKind::PiLike,
            StructuralKind::PiLike,
            StructuralKind::PiLike,
            StructuralKind::ExpLike,
            StructuralKind::ExpLike,
            StructuralKind::SqrtLike,
            StructuralKind::ProductConstant,
            StructuralKind::ProductConstant,
            StructuralKind::ProductConstant,
            StructuralKind::SqrtLike,
            StructuralKind::ExpLike,
            StructuralKind::LogLike,
            StructuralKind::LogLike,
            StructuralKind::LogLike,
            StructuralKind::LogLike,
            StructuralKind::LogLike,
            StructuralKind::ExpLike,
            StructuralKind::ExpLike,
            StructuralKind::TrigExact,
            StructuralKind::TrigExact,
            StructuralKind::ComputableOpaque,
        ]
    );
    values
}

fn opaque_graph_values(depth_seed: u8, opcode_seed: u8) -> Vec<Real> {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let identity_residual = &sine * &sine + &cosine * &cosine - Real::one();
    let mut recursive = &sine + &cosine;

    // Irrational graphs have no finite exhaustive inventory. Vary a bounded
    // DAG so the fuzzer explores depth, node sharing, unary kernels, and
    // binary composition in addition to the finite 22-class corpus.
    for level in 0..=usize::from(depth_seed % 8) {
        recursive = match (usize::from(opcode_seed) + level) % 6 {
            0 => recursive.sin(),
            1 => recursive.cos(),
            2 => recursive.exp().expect("bounded finite graph exponential"),
            3 => &recursive * &recursive + &sine,
            4 => &recursive + &cosine,
            5 => (&recursive * &sine) - &cosine,
            _ => unreachable!(),
        };
    }

    let values = vec![sine, cosine, identity_residual, recursive];
    assert!(
        values.iter().all(|value| {
            value.detailed_facts().symbolic.kind == StructuralKind::ComputableOpaque
        })
    );
    values
}
