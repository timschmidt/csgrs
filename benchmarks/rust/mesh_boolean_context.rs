//! Predicate-policy comparison for subtracting a sampled drill cylinder.

use std::{hint::black_box, time::Instant};

use csgrs::{
    GeometryCertainty, GeometryContext, Real, TriangleMesh,
    solid::{self, SolidExt},
};
use hypermesh::HypermeshError;

fn measure(
    body: &TriangleMesh,
    drill: &TriangleMesh,
    segments: usize,
    context: GeometryContext,
    iterations: usize,
) {
    let mut elapsed_us = Vec::with_capacity(iterations);
    let mut completed = 0;
    let mut approximate = 0;
    let mut undecided = 0;
    let mut output_triangles = 0;
    for _ in 0..iterations {
        let start = Instant::now();
        let result =
            black_box(body).try_difference_with_context(black_box(drill), black_box(&context));
        elapsed_us.push(start.elapsed().as_micros());
        match result {
            Ok(outcome) => {
                completed += 1;
                approximate += usize::from(
                    outcome.certainty == GeometryCertainty::Approximate512Consumed,
                );
                output_triangles = outcome.value.triangles.len();
                black_box(outcome);
            },
            Err(HypermeshError::PredicateUndecided { .. }) => undecided += 1,
            Err(error) => panic!("unexpected drill Boolean error: {error}"),
        }
    }
    // The first sample includes initial predicate evaluation. Later samples
    // reuse the same immutable input meshes and their retained kernel facts.
    let first_us = elapsed_us[0];
    elapsed_us.sort_unstable();
    let median_us = elapsed_us[iterations / 2];
    println!(
        "drill_boolean segments={segments} policy={:?} iterations={iterations} completed={completed} approximate={approximate} undecided={undecided} input_triangles={} output_triangles={output_triangles} first_us={first_us} median_us={median_us}",
        context.predicate_policy(),
        body.triangles.len() + drill.triangles.len(),
    );
    if context == GeometryContext::STRICT {
        assert_eq!(undecided, iterations);
    } else {
        assert_eq!(completed, iterations);
        assert_eq!(approximate, iterations);
    }
}

fn main() {
    let iterations = std::env::var("CSGRS_BOOLEAN_CONTEXT_BENCH_ITERS")
        .map(|value| value.parse::<usize>().expect("iterations must be an integer"))
        .unwrap_or(5);
    assert!(iterations > 0, "iterations must be positive");
    for segments in [16, 32] {
        for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
            // Start each policy with fresh mesh storage so the strict run
            // cannot pre-populate the approximate run's mesh facts.
            let body = solid::cuboid(Real::from(12), Real::from(12), Real::from(4));
            let body = TriangleMesh::new(body.positions.to_vec(), body.triangles.to_vec());
            let drill = solid::cylinder(Real::from(2), Real::from(6), segments).translated(
                Real::from(6),
                Real::from(6),
                Real::from(-1),
            );
            let drill = TriangleMesh::new(drill.positions.to_vec(), drill.triangles.to_vec());
            measure(&body, &drill, segments, context, iterations);
        }
    }
}
