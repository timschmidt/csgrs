mod common;

use hypermesh::{BooleanOp, EmberConfig, boolean_operation};

fn main() {
    for (name, meshes) in [
        ("cubes", common::cube_pair()),
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
}
