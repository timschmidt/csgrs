mod common;

use hypermesh::{BooleanOp, EmberConfig, Point3, Real, boolean_operation, convex_hull};

fn main() {
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
                boolean_operation(
                    &[meshes[0].as_ref(), meshes[1].as_ref()],
                    op,
                    EmberConfig::default(),
                )
            });
            let output = result.expect("trace workload must remain certified");
            let trace = hyperreal::dispatch_trace::take_trace();
            println!(
                "{name}/{op:?}: polygons={}, correlation={:?}",
                output.classifications().len(),
                trace.correlation_summary()
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
        boolean_operation(
            &nested_tool_refs,
            BooleanOp::Difference,
            EmberConfig::default(),
        )
    })
    .expect("trace variadic difference must remain certified");
    let trace = hyperreal::dispatch_trace::take_trace();
    println!(
        "nested_tools_5/Difference: polygons={}, correlation={:?}",
        nested_tool_result.classifications().len(),
        trace.correlation_summary()
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
        boolean_operation(
            &[subdivided_cubes[0].as_ref(), subdivided_cubes[1].as_ref()],
            BooleanOp::Union,
            EmberConfig::default(),
        )
    })
    .expect("subdivided cube union must remain certified");
    let trace = hyperreal::dispatch_trace::take_trace();
    println!(
        "subdivided_cubes_192/Union: polygons={}, correlation={:?}",
        subdivided_result.classifications().len(),
        trace.correlation_summary()
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
    let hull = hyperreal::dispatch_trace::with_recording(|| convex_hull(&hull_points))
        .expect("trace point set must span 3D");
    let trace = hyperreal::dispatch_trace::take_trace();
    println!(
        "convex_hull/grid_4913: vertices={}, triangles={}, correlation={:?}",
        hull.positions.len(),
        hull.triangles.len(),
        trace.correlation_summary()
    );
    for summary in &trace.dispatch {
        println!(
            "  {}/{}/{}/{}",
            summary.layer, summary.operation, summary.path, summary.count
        );
    }
}
