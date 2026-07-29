#[path = "../benchmarks/support/competitive.rs"]
#[allow(dead_code)]
mod support;

use csgrs::solid;
use support::{
    LARGE_TRIANGLES_PER_MESH, Operation, assert_close, assert_output, corpus,
    large_boolean_case, prepare, run_boolmesh, run_csgrs, run_manifold, summarize,
    validate_with_tri_mesh,
};

#[test]
fn public_csgrs_booleans_match_boolmesh_and_manifold_on_shared_corpus() {
    for case in corpus() {
        let inputs = prepare(&case);
        let left_volume = summarize(&case.left).volume;
        let right_volume = summarize(&case.right).volume;
        let mut volumes = [[0.0; 3]; 3];
        let mut areas = [[0.0; 3]; 3];

        for operation in Operation::ALL {
            let outputs = [
                ("csgrs", run_csgrs(&inputs.csgrs, operation)),
                ("boolmesh", run_boolmesh(&inputs.boolmesh, operation)),
                ("manifold-rust", run_manifold(&inputs.manifold, operation)),
            ];
            for (engine_index, (engine, output)) in outputs.into_iter().enumerate() {
                let summary = summarize(&output);
                assert_output(engine, &case, operation, &summary);
                volumes[engine_index][operation_index(operation)] = summary.volume;
                areas[engine_index][operation_index(operation)] = summary.area;
            }
        }

        for (engine_index, (engine, volumes)) in ["csgrs", "boolmesh", "manifold-rust"]
            .into_iter()
            .zip(volumes)
            .enumerate()
        {
            assert_close(
                volumes[0] + volumes[1],
                left_volume + right_volume,
                &format!("{engine} {} union/intersection identity", case.name),
            );
            assert_close(
                volumes[2] + volumes[1],
                left_volume,
                &format!("{engine} {} difference/intersection identity", case.name),
            );
            for operation in Operation::ALL {
                assert_close(
                    areas[engine_index][operation_index(operation)],
                    areas[0][operation_index(operation)],
                    &format!("{engine} {} {} surface area", case.name, operation.name()),
                );
            }
        }
    }
}

#[test]
fn public_csgrs_outputs_round_trip_through_tri_mesh_topology() {
    for case in corpus() {
        let inputs = prepare(&case);
        for operation in Operation::ALL {
            let output = run_csgrs(&inputs.csgrs, operation);
            let summary = summarize(&output);
            assert_output("csgrs", &case, operation, &summary);
            let context = format!("{} {}", case.name, operation.name());
            let (vertices, faces) = validate_with_tri_mesh(&output, &context);
            assert_eq!(
                vertices,
                summary.vertices,
                "vertex count differs for {} {}",
                case.name,
                operation.name()
            );
            assert_eq!(
                faces,
                summary.triangles,
                "face count differs for {} {}",
                case.name,
                operation.name()
            );
        }
    }
}

#[test]
fn competitor_inputs_preserve_public_csgrs_volume_and_manifoldness() {
    for case in corpus() {
        let prepared = prepare(&case);
        for (side, raw, mesh, manifold, boolmesh) in [
            (
                "left",
                &case.left,
                &prepared.csgrs[0],
                &prepared.manifold[0],
                &prepared.boolmesh[0],
            ),
            (
                "right",
                &case.right,
                &prepared.csgrs[1],
                &prepared.manifold[1],
                &prepared.boolmesh[1],
            ),
        ] {
            let expected = summarize(raw);
            assert!(solid::is_closed_manifold(mesh), "{} {side}", case.name);
            assert!(boolmesh.is_manifold(), "{} {side}", case.name);
            assert_close(
                manifold.volume(),
                expected.volume,
                &format!("Manifold {} {side} volume", case.name),
            );
        }
    }
}

#[test]
fn large_boolean_benchmark_inputs_are_closed_and_keep_the_intended_scale() {
    let case = large_boolean_case();
    assert_eq!(case.left.triangles.len(), LARGE_TRIANGLES_PER_MESH);
    assert_eq!(case.right.triangles.len(), LARGE_TRIANGLES_PER_MESH);
    for (side, mesh) in [("left", &case.left), ("right", &case.right)] {
        let summary = summarize(mesh);
        assert!(summary.closed, "{side} large fixture is open");
        assert!(summary.nondegenerate, "{side} large fixture is degenerate");
        assert_eq!(summary.triangles, LARGE_TRIANGLES_PER_MESH);
    }
    let prepared = prepare(&case);
    assert!(prepared.csgrs.iter().all(solid::is_closed_manifold));
    assert!(prepared.boolmesh.iter().all(|mesh| mesh.is_manifold()));
    assert!(
        prepared
            .manifold
            .iter()
            .all(|mesh| mesh.num_tri() == LARGE_TRIANGLES_PER_MESH)
    );
}

fn operation_index(operation: Operation) -> usize {
    match operation {
        Operation::Union => 0,
        Operation::Intersection => 1,
        Operation::Difference => 2,
    }
}
