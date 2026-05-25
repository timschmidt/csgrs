#![no_main]

use hypercurve::{
    BezierArrangementGraph2, BezierParameter2, BezierRetainedOverlapReport2, Classification,
    CubicBezier2, CurvePolicy, Point2, QuadraticBezier2, Real,
};
use libfuzzer_sys::fuzz_target;

fn real_from_byte(byte: u8) -> Real {
    Real::from(byte as i32 - 128)
}

fn unit_from_byte(byte: u8) -> Real {
    (Real::from((byte % 17) as i32) / Real::from(16_i32)).unwrap()
}

fn point(x: u8, y: u8) -> Point2 {
    Point2::new(real_from_byte(x), real_from_byte(y))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }

    let policy = CurvePolicy::certified();
    let mut materializations = Vec::new();
    for chunk in data.chunks(8).take(8) {
        if chunk.len() < 8 {
            break;
        }
        let curve = QuadraticBezier2::new(
            point(chunk[0], chunk[1]),
            point(chunk[2], chunk[3]),
            point(chunk[4], chunk[5]),
        );
        let mut parameters = Vec::new();
        if let Ok(Classification::Decided(parameter)) =
            BezierParameter2::exact(unit_from_byte(chunk[6]), &policy)
        {
            parameters.push(parameter);
        }
        if let Ok(Classification::Decided(materialization)) =
            curve.split_at_parameters(&parameters, &policy)
        {
            materializations.push(materialization);
        }
    }

    let graph = BezierArrangementGraph2::from_split_materializations(&materializations);
    let _ = graph.traverse_branch_free(&policy);
    let _ = graph.traverse_with_tangent_order(&policy);
    let _ = graph.traverse_retained_with_tangent_order(&policy);
    let _ = graph.traverse_retained_deduplicating_materialized_overlaps(&policy);
    let _ = graph
        .split_retained_linear_overlaps(&policy)
        .map(|refinement| {
            for overlap in refinement.resolved_overlaps() {
                let _ = overlap.first_refined_fragment_index();
                let _ = overlap.second_refined_fragment_index();
                let _ = overlap.orientation();
            }
        });
    let _ = graph.traverse_retained_splitting_linear_overlaps(&policy);
    let _ = BezierRetainedOverlapReport2::from_graph(&graph, &policy).map(|report| {
        let _ = report.line_overlap_splits(&policy);
        let _ = report.linear_bezier_overlap_splits(&graph, &policy);
    });

    let same_tangent_curves = [
        QuadraticBezier2::new(point(128, 128), point(129, 128), point(130, 128)),
        QuadraticBezier2::new(point(130, 128), point(131, 129), point(132, 128)),
        QuadraticBezier2::new(point(130, 128), point(132, 130), point(133, 128)),
    ];
    let mut same_tangent_materializations = Vec::new();
    for curve in same_tangent_curves {
        if let Ok(Classification::Decided(materialization)) =
            curve.split_at_parameters(&[], &policy)
        {
            same_tangent_materializations.push(materialization);
        }
    }
    let same_tangent_graph =
        BezierArrangementGraph2::from_split_materializations(&same_tangent_materializations);
    let _ = same_tangent_graph.traverse_with_tangent_order(&policy);
    let _ = same_tangent_graph.traverse_retained_with_tangent_order(&policy);

    let cubic_same_tangent_curves = [
        CubicBezier2::new(
            point(130, 128),
            point(131, 128),
            point(132, 128),
            point(133, 129),
        ),
        CubicBezier2::new(
            point(130, 128),
            point(131, 128),
            point(132, 128),
            point(133, 127),
        ),
    ];
    let mut cubic_materializations = same_tangent_materializations[..1].to_vec();
    for curve in cubic_same_tangent_curves {
        if let Ok(Classification::Decided(materialization)) =
            curve.split_at_parameters(&[], &policy)
        {
            cubic_materializations.push(materialization);
        }
    }
    let cubic_same_tangent_graph =
        BezierArrangementGraph2::from_split_materializations(&cubic_materializations);
    let _ = cubic_same_tangent_graph.traverse_with_tangent_order(&policy);
    let _ = cubic_same_tangent_graph.traverse_retained_with_tangent_order(&policy);
});
