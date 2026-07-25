#[path = "../competitive/support.rs"]
mod support;

use hypermesh::{Plane, Point3, triangle_soup_closure_evidence};
use support::{
    LARGE_TRIANGLES_PER_MESH, Operation, YEAHRIGHT_TRIANGLES, assert_close, assert_summary, corpus,
    large_boolean_case, prepare, prepare_yeahright, raw_from_hypermesh, run_boolmesh,
    run_hypermesh, run_hypermesh_exact, run_manifold, summarize, validate_with_tri_mesh,
    yeahright_boolean_case,
};

#[test]
fn boolmesh_and_manifold_match_hypermesh_on_shared_boolean_corpus() {
    for case in corpus() {
        let inputs = prepare(&case);
        let mut volumes = [[0.0; 3]; 3];
        let mut areas = [[0.0; 3]; 3];

        for operation in Operation::ALL {
            let outputs = [
                ("hypermesh", run_hypermesh(&inputs.hypermesh, operation)),
                ("boolmesh", run_boolmesh(&inputs.boolmesh, operation)),
                ("manifold", run_manifold(&inputs.manifold, operation)),
            ];
            for (engine_index, (engine, output)) in outputs.into_iter().enumerate() {
                let summary = summarize(&output);
                assert_summary(engine, &case, operation, &summary);
                volumes[engine_index][operation_index(operation)] = summary.volume;
                areas[engine_index][operation_index(operation)] = summary.surface_area;
            }
        }

        for (engine_index, (engine, volume)) in ["hypermesh", "boolmesh", "manifold"]
            .into_iter()
            .zip(volumes)
            .enumerate()
        {
            let left = summarize(&case.left).volume;
            let right = summarize(&case.right).volume;
            assert_close(
                volume[0] + volume[1],
                left + right,
                &format!("{engine} {} union/intersection identity", case.name),
            );
            assert_close(
                volume[2] + volume[1],
                left,
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
fn hypermesh_boolean_outputs_are_valid_tri_mesh_half_edge_inputs() {
    for case in corpus() {
        let inputs = prepare(&case);
        for operation in Operation::ALL {
            let output = run_hypermesh(&inputs.hypermesh, operation);
            let summary = summarize(&output);
            let (vertices, faces, components) = validate_with_tri_mesh(&output);
            assert_eq!(
                vertices,
                summary.vertices,
                "tri-mesh vertex count differs for {} {}",
                case.name,
                operation.name()
            );
            assert_eq!(
                faces,
                summary.triangles,
                "tri-mesh face count differs for {} {}",
                case.name,
                operation.name()
            );
            assert_eq!(
                components,
                summary.components,
                "tri-mesh component count differs for {} {}",
                case.name,
                operation.name()
            );
        }
    }
}

#[test]
fn competitor_input_adapters_preserve_fixture_geometry() {
    for case in corpus() {
        for (side, input) in [("left", &case.left), ("right", &case.right)] {
            let expected = summarize(input);
            let (_, faces, components) = validate_with_tri_mesh(input);
            assert_eq!(faces, expected.triangles, "{} {side}", case.name);
            assert_eq!(components, expected.components, "{} {side}", case.name);

            let prepared = prepare(&case);
            let boolmesh_input = if side == "left" {
                &prepared.boolmesh[0]
            } else {
                &prepared.boolmesh[1]
            };
            assert!(boolmesh_input.is_manifold(), "{} {side}", case.name);

            let manifold_input = if side == "left" {
                &prepared.manifold[0]
            } else {
                &prepared.manifold[1]
            };
            assert_close(
                manifold_input.volume(),
                expected.volume,
                &format!("Manifold {} {side} input volume", case.name),
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
    assert!(prepared.boolmesh.iter().all(|mesh| mesh.is_manifold()));
    assert!(
        prepared
            .manifold
            .iter()
            .all(|mesh| mesh.num_tri() == LARGE_TRIANGLES_PER_MESH)
    );
}

#[test]
fn yeahright_benchmark_inputs_reach_every_competitor() {
    let case = yeahright_boolean_case();
    assert_eq!(case.name, "yeahright_hull_4512_box");
    assert_eq!(case.left.triangles.len(), YEAHRIGHT_TRIANGLES);
    assert_eq!(case.right.triangles.len(), 12);
    for (side, mesh) in [("hull", &case.left), ("box", &case.right)] {
        let summary = summarize(mesh);
        assert!(summary.closed, "{side} fixture is open");
        assert!(summary.finite, "{side} fixture is non-finite");
        assert!(summary.nondegenerate, "{side} fixture is degenerate");
        let (vertices, faces, components) = validate_with_tri_mesh(mesh);
        assert_eq!(vertices, summary.vertices, "tri-mesh {side} vertex count");
        assert_eq!(faces, summary.triangles, "tri-mesh {side} face count");
        assert_eq!(
            components, summary.components,
            "tri-mesh {side} component count"
        );
    }

    let prepared = prepare_yeahright(&case);
    assert!(prepared.boolmesh.iter().all(|mesh| mesh.is_manifold()));
    assert_eq!(
        prepared.manifold[0].num_tri(),
        YEAHRIGHT_TRIANGLES,
        "Manifold did not receive the subdivided YeahRight hull"
    );
    assert_eq!(
        prepared.hypermesh[0].triangles.len(),
        YEAHRIGHT_TRIANGLES,
        "HyperMesh did not receive the subdivided YeahRight hull"
    );
}

#[test]
fn yeahright_exact_hypermesh_outputs_remain_boundaryless_for_every_operation() {
    let case = yeahright_boolean_case();
    let prepared = prepare_yeahright(&case);
    for operation in Operation::ALL {
        let exact = run_hypermesh_exact(&prepared.hypermesh, operation);
        let closure = triangle_soup_closure_evidence(&exact);
        assert!(
            closure.has_no_boundary(),
            "HyperMesh {} exact output has a boundary: {closure:?}",
            operation.name()
        );
        let degenerate_triangles = exact
            .triangles
            .iter()
            .filter(|triangle| {
                let [a, b, c] = triangle.map(|index| {
                    let vertex = &exact.vertices[index];
                    Point3::new(vertex.x.clone(), vertex.y.clone(), vertex.z.clone())
                });
                !Plane::points_are_nondegenerate(&a, &b, &c)
            })
            .count();
        assert_eq!(
            degenerate_triangles,
            0,
            "HyperMesh {} exact output contains degenerate triangles",
            operation.name()
        );
        let output = raw_from_hypermesh(&exact);
        let summary = summarize(&output);
        assert!(
            summary.closed,
            "HyperMesh {} output is open: {summary:?}",
            operation.name(),
        );
        assert!(
            summary.finite,
            "HyperMesh {} output is non-finite",
            operation.name()
        );
    }
}

fn operation_index(operation: Operation) -> usize {
    match operation {
        Operation::Union => 0,
        Operation::Intersection => 1,
        Operation::Difference => 2,
    }
}
