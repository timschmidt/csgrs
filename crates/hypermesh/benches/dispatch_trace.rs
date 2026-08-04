mod common;
#[path = "../competitive/support.rs"]
#[allow(dead_code)]
mod competitive_support;

use hypermesh::clip::clip_polygon;
use hypermesh::{
    BooleanExpression, BooleanOp, BooleanProgram, ExactBvh, HypermeshResult, MeshContext, Plane,
    Point3, PredicatePolicy, Real, TriangleMeshRef, boolean, convex_hull,
    convex_hull_with_coplanar_groups, convex_hull_with_retained_facts, convex_triangle,
    intersect_polygons, polygon_soup,
};

const CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::APPROXIMATE_512);

fn trace_workload<T>(name: &str, workload: impl FnOnce() -> HypermeshResult<T>) -> T {
    hyperreal::dispatch_trace::reset();
    let result = hyperreal::dispatch_trace::with_recording(workload)
        .unwrap_or_else(|error| panic!("{name} trace workload must remain certified: {error}"));
    let trace = hyperreal::dispatch_trace::take_trace();
    let correlation = trace.correlation_summary();
    assert!(
        correlation.dispatch_events > 0 || correlation.rational_temporaries > 0,
        "{name} did not emit an exact-computation path trace"
    );
    println!("{name}: correlation={correlation:?}");
    for summary in &trace.dispatch {
        println!(
            "  {}/{}/{}/{}",
            summary.layer, summary.operation, summary.path, summary.count
        );
    }
    result
}

fn main() {
    if competitive_support::yeahright_enabled() {
        let yeahright_case = competitive_support::yeahright_boolean_case();
        let yeahright_inputs = competitive_support::prepare_yeahright(&yeahright_case);
        hyperreal::dispatch_trace::reset();
        let yeahright_output = hyperreal::dispatch_trace::with_recording(|| {
            competitive_support::run_hypermesh_batch(
                &yeahright_inputs.hypermesh,
                competitive_support::Operation::Union,
            )
        });
        let trace = hyperreal::dispatch_trace::take_trace();
        println!(
            "{}/Union: triangles={}, correlation={:?}",
            yeahright_case.name,
            yeahright_output.results[0].triangles.len(),
            trace.correlation_summary(),
        );
        for summary in &trace.dispatch {
            println!(
                "  {}/{}/{}/{}",
                summary.layer, summary.operation, summary.path, summary.count
            );
        }
    }

    for (name, meshes) in [
        ("cubes", common::cube_pair()),
        ("nested_cubes", common::nested_cube_pair()),
        ("octahedra", common::octahedron_pair()),
    ] {
        for op in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::SymmetricDifference,
        ] {
            hyperreal::dispatch_trace::reset();
            let result = hyperreal::dispatch_trace::with_recording(|| {
                boolean(
                    &CONTEXT,
                    &[meshes[0].as_ref(), meshes[1].as_ref()],
                    BooleanProgram::Operation(op),
                )
            });
            let output = result
                .expect("trace workload must remain certified")
                .into_value();
            let trace = hyperreal::dispatch_trace::take_trace();
            let correlation = trace.correlation_summary();
            assert!(
                correlation.dispatch_events > 0 || correlation.rational_temporaries > 0,
                "{name}/{op:?} did not emit an exact-computation path trace"
            );
            println!(
                "{name}/{op:?}: triangles={}, correlation={:?}",
                output.results[0].triangles.len(),
                correlation
            );
            for summary in &trace.dispatch {
                println!(
                    "  {}/{}/{}/{}",
                    summary.layer, summary.operation, summary.path, summary.count
                );
            }
        }
    }

    let nested_tools = common::nested_tool_cubes();
    let nested_tool_refs = nested_tools
        .iter()
        .map(|mesh| mesh.as_ref())
        .collect::<Vec<_>>();
    hyperreal::dispatch_trace::reset();
    let nested_tool_result = hyperreal::dispatch_trace::with_recording(|| {
        boolean(
            &CONTEXT,
            &nested_tool_refs,
            BooleanProgram::Operation(BooleanOp::Difference),
        )
    })
    .expect("trace variadic difference must remain certified")
    .into_value();
    let trace = hyperreal::dispatch_trace::take_trace();
    let correlation = trace.correlation_summary();
    assert!(
        correlation.dispatch_events > 0 || correlation.rational_temporaries > 0,
        "nested_tools_5/Difference did not emit an exact-computation path trace"
    );
    println!(
        "nested_tools_5/Difference: triangles={}, correlation={:?}",
        nested_tool_result.results[0].triangles.len(),
        correlation
    );
    for summary in &trace.dispatch {
        println!(
            "  {}/{}/{}/{}",
            summary.layer, summary.operation, summary.path, summary.count
        );
    }

    let subdivided_cubes = common::subdivided_cube_pair(2);
    hyperreal::dispatch_trace::reset();
    let subdivided_result = hyperreal::dispatch_trace::with_recording(|| {
        boolean(
            &CONTEXT,
            &[subdivided_cubes[0].as_ref(), subdivided_cubes[1].as_ref()],
            BooleanProgram::Operation(BooleanOp::Union),
        )
    })
    .expect("subdivided cube union must remain certified")
    .into_value();
    let trace = hyperreal::dispatch_trace::take_trace();
    let correlation = trace.correlation_summary();
    assert!(
        correlation.dispatch_events > 0 || correlation.rational_temporaries > 0,
        "subdivided_cubes_192/Union did not emit an exact-computation path trace"
    );
    println!(
        "subdivided_cubes_192/Union: triangles={}, correlation={:?}",
        subdivided_result.results[0].triangles.len(),
        correlation
    );
    for summary in &trace.dispatch {
        println!(
            "  {}/{}/{}/{}",
            summary.layer, summary.operation, summary.path, summary.count
        );
    }

    let hull_points = (-8..=8)
        .flat_map(|x| {
            (-8..=8).flat_map(move |y| {
                (-8..=8).map(move |z| Point3::new(Real::from(x), Real::from(y), Real::from(z)))
            })
        })
        .collect::<Vec<_>>();
    hyperreal::dispatch_trace::reset();
    let hull = hyperreal::dispatch_trace::with_recording(|| convex_hull(&CONTEXT, &hull_points))
        .expect("trace point set must span 3D")
        .into_value();
    let trace = hyperreal::dispatch_trace::take_trace();
    let correlation = trace.correlation_summary();
    assert!(
        correlation.dispatch_events > 0 || correlation.rational_temporaries > 0,
        "convex_hull/grid_4913 did not emit an exact-computation path trace"
    );
    println!(
        "convex_hull/grid_4913: vertices={}, triangles={}, correlation={:?}",
        hull.positions.len(),
        hull.triangles.len(),
        correlation
    );
    for summary in &trace.dispatch {
        println!(
            "  {}/{}/{}/{}",
            summary.layer, summary.operation, summary.path, summary.count
        );
    }

    let cube_pair = common::cube_pair();
    let cube_refs = [
        TriangleMeshRef::new(&cube_pair[0].positions, &cube_pair[0].triangles),
        TriangleMeshRef::new(&cube_pair[1].positions, &cube_pair[1].triangles),
    ];
    let native_cube_refs = [cube_pair[0].as_ref(), cube_pair[1].as_ref()];
    let soup = trace_workload("mesh_build_polygon_soup", || {
        Ok(polygon_soup(&CONTEXT, &cube_refs)?.into_value())
    });
    assert_eq!(soup.num_meshes, 2);
    assert!(!soup.polygons.is_empty());

    trace_workload("surface_arrangement_single_result", || {
        let result = boolean(
            &CONTEXT,
            &native_cube_refs,
            BooleanProgram::Operation(BooleanOp::Union),
        )?
        .into_value();
        Ok(result.results[0].triangles.len())
    });
    let nodes = [
        BooleanExpression::Operation(BooleanOp::Union),
        BooleanExpression::Operation(BooleanOp::Intersection),
        BooleanExpression::Operation(BooleanOp::Difference),
        BooleanExpression::Operation(BooleanOp::SymmetricDifference),
    ];
    let roots = [0, 1, 2, 3];
    trace_workload("surface_arrangement_shared_four_results", || {
        let batch = boolean(
            &CONTEXT,
            &native_cube_refs,
            BooleanProgram::Expressions {
                nodes: &nodes,
                roots: &roots,
            },
        )?
        .into_value();
        Ok(batch
            .results
            .iter()
            .map(|result| result.triangles.len())
            .sum::<usize>())
    });

    let p = |x, y, z| Point3::new(Real::from(x), Real::from(y), Real::from(z));
    let host = convex_triangle(&CONTEXT, &p(0, 0, 0), &p(4, 0, 0), &p(0, 4, 0), 0, 0)
        .expect("host triangle must be valid")
        .into_value();
    let cutter = convex_triangle(&CONTEXT, &p(2, -1, -1), &p(2, 5, -1), &p(2, 2, 1), 1, 0)
        .expect("cutter triangle must be valid")
        .into_value();
    trace_workload("polygon_clip_intersection_bvh", || {
        let clipped =
            clip_polygon(&CONTEXT, &host, &Plane::axis_aligned(0, Real::from(1)))?.into_value();
        assert!(
            clipped.left.is_valid(&CONTEXT)?.into_value()
                || clipped.right.is_valid(&CONTEXT)?.into_value()
        );

        let intersection = intersect_polygons(&CONTEXT, &host, &cutter, 1)?.into_value();

        let left = ExactBvh::build(&CONTEXT, std::slice::from_ref(&host))?.into_value();
        let right = ExactBvh::build(&CONTEXT, std::slice::from_ref(&cutter))?.into_value();
        let mut pair_count = 0;
        left.intersect_pairs(&CONTEXT, &right, |_, _| pair_count += 1)?;
        assert_eq!(pair_count, 1);
        Ok((
            matches!(
                intersection,
                hypermesh::PairwiseIntersection::NonCoplanarSegment(_)
            ),
            pair_count,
        ))
    });

    let retained_points = vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0), p(0, 0, 2)];
    let coordinate_ids = vec![
        [0, 1, 2, 10, 20],
        [3, 4, 5, 10, 21],
        [6, 7, 8, 10, 22],
        [9, 10, 11, 10, 23],
    ];
    trace_workload("convex_hull_public_variants", || {
        let grouped =
            convex_hull_with_coplanar_groups(&CONTEXT, &retained_points, &[])?.into_value();
        let retained =
            convex_hull_with_retained_facts(&CONTEXT, &retained_points, &[], &coordinate_ids)?
                .into_value();
        Ok((grouped.triangles.len(), retained.triangles.len()))
    });
}
