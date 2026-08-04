#[path = "../competitive/support.rs"]
#[allow(dead_code)]
mod support;

use std::collections::BTreeMap;

use hypermesh::{
    BooleanExpression, BooleanMeshResult, BooleanOp, BooleanProgram, MeshContext, Plane,
    PredicatePolicy, boolean, boolean_mesh_closure_evidence, certify_convex_mesh, polygon_soup,
};
use support::{
    APPROXIMATE_CONTEXT, LARGE_TRIANGLES_PER_MESH, Operation, YEAHRIGHT_CONTROL_TRIANGLES,
    YEAHRIGHT_CONTROL_VERTICES, YEAHRIGHT_STRESS_SUBDIVISIONS, assert_close, assert_summary,
    corpus, large_boolean_case, lower_dimensional_contact_corpus, prepare, prepare_yeahright,
    raw_from_hypermesh_batch, run_boolmesh, run_hypermesh, run_hypermesh_all, run_hypermesh_batch,
    run_manifold, summarize, to_hypermesh, validate_with_tri_mesh, yeahright_boolean_case,
    yeahright_boolean_case_with_subdivisions, yeahright_control_mesh,
};

const HYPERMESH_OPERATIONS: [(&str, BooleanOp); 4] = [
    ("union", BooleanOp::Union),
    ("intersection", BooleanOp::Intersection),
    ("difference", BooleanOp::Difference),
    ("symmetric difference", BooleanOp::SymmetricDifference),
];

fn predicate_contexts() -> [(&'static str, MeshContext); 2] {
    [
        ("STRICT", MeshContext::new(PredicatePolicy::STRICT)),
        (
            "APPROXIMATE_512",
            MeshContext::new(PredicatePolicy::APPROXIMATE_512),
        ),
    ]
}

fn boundary_is_balanced(result: &BooleanMeshResult) -> bool {
    let mut edges = BTreeMap::<[u32; 2], [usize; 2]>::new();
    for triangle in &result.triangles {
        for [start, end] in [
            [triangle[0], triangle[1]],
            [triangle[1], triangle[2]],
            [triangle[2], triangle[0]],
        ] {
            let edge = if start < end {
                [start, end]
            } else {
                [end, start]
            };
            edges.entry(edge).or_default()[usize::from(start > end)] += 1;
        }
    }
    edges.values().all(|uses| uses[0] == uses[1])
}

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
fn shared_arrangement_matches_all_four_cgal_boolean_outputs_under_both_policies() {
    for case in corpus() {
        let inputs = prepare(&case);
        for (policy, context) in predicate_contexts() {
            let batch = run_hypermesh_all(&context, &inputs.hypermesh);
            assert_eq!(batch.results.len(), 4, "{} {policy}", case.name);
            for (output, expected_volume) in [
                case.expected_volumes[0],
                case.expected_volumes[1],
                case.expected_volumes[2],
                summarize(&case.right).volume - case.expected_volumes[1],
            ]
            .into_iter()
            .enumerate()
            {
                assert!(
                    boundary_is_balanced(&batch.results[output]),
                    "{} {policy} output {output} is not closed",
                    case.name
                );
                let summary = summarize(&raw_from_hypermesh_batch(&batch, output));
                assert!(summary.finite, "{} {policy} output {output}", case.name);
                assert_close(
                    summary.volume,
                    expected_volume,
                    &format!("HyperMesh {} {policy} output {output}", case.name),
                );
            }
        }
    }
}

#[test]
fn lower_dimensional_closed_pwn_contacts_are_total_under_both_policies() {
    let nodes = [
        BooleanExpression::Operation(BooleanOp::Union),
        BooleanExpression::Operation(BooleanOp::Intersection),
        BooleanExpression::Operation(BooleanOp::Difference),
        BooleanExpression::Operand(0),
        BooleanExpression::Operand(1),
        BooleanExpression::Not(3),
        BooleanExpression::And([4, 5]),
    ];
    let roots = [0_u32, 1, 2, 6];
    for case in lower_dimensional_contact_corpus() {
        let inputs = [to_hypermesh(&case.left), to_hypermesh(&case.right)];
        let right_volume = summarize(&case.right).volume;
        let expected_volumes = [
            case.expected_volumes[0],
            case.expected_volumes[1],
            case.expected_volumes[2],
            right_volume - case.expected_volumes[1],
        ];
        for (policy, context) in predicate_contexts() {
            let outcome = boolean(
                &context,
                &[inputs[0].as_ref(), inputs[1].as_ref()],
                BooleanProgram::Expressions {
                    nodes: &nodes,
                    roots: &roots,
                },
            )
            .unwrap_or_else(|error| panic!("{} {policy} failed: {error}", case.name));
            assert_eq!(outcome.certainty, hypermesh::MeshCertainty::Certified);
            assert_eq!(outcome.value.results.len(), roots.len());
            for (output_index, (output, expected_volume)) in outcome
                .value
                .results
                .iter()
                .zip(expected_volumes)
                .enumerate()
            {
                assert!(
                    boundary_is_balanced(output),
                    "{} {policy} output {output_index} is directionally unbalanced",
                    case.name
                );
                let raw = raw_from_hypermesh_batch(&outcome.value, output_index);
                let summary = summarize(&raw);
                assert!(
                    summary.finite,
                    "{} {policy} output {output_index}",
                    case.name
                );
                assert!(
                    summary.nondegenerate,
                    "{} {policy} output {output_index}",
                    case.name
                );
                assert_close(
                    summary.volume,
                    expected_volume,
                    &format!(
                        "Hypermesh {} {policy} output {output_index} volume",
                        case.name
                    ),
                );
            }
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
#[ignore = "requires the opt-in external benchmark fixture (YEAHRIGHT_BENCH=1)"]
fn yeahright_benchmark_inputs_reach_every_competitor() {
    let case = yeahright_boolean_case();
    assert_eq!(case.name, "yeahright_control_hull_subdivided_box");
    assert_eq!(
        certify_convex_mesh(&APPROXIMATE_CONTEXT, to_hypermesh(&case.left).as_ref())
            .expect("the dyadic benchmark subdivision must remain exactly convex")
            .certainty,
        hypermesh::MeshCertainty::Certified
    );
    let triangle_count = case.left.triangles.len();
    assert!(triangle_count > 12);
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
        triangle_count,
        "Manifold did not receive the subdivided YeahRight hull"
    );
    assert_eq!(
        prepared.hypermesh[0].triangles.len(),
        triangle_count,
        "HyperMesh did not receive the subdivided YeahRight hull"
    );
}

#[test]
#[ignore = "requires the opt-in external benchmark fixture (YEAHRIGHT_BENCH=1)"]
fn yeahright_exact_hypermesh_outputs_remain_boundaryless_for_every_operation() {
    let case = yeahright_boolean_case();
    let prepared = prepare_yeahright(&case);
    let mut strict_outputs = Vec::with_capacity(HYPERMESH_OPERATIONS.len());
    for (policy, context) in predicate_contexts() {
        for (operation_index, (operation, boolean_op)) in
            HYPERMESH_OPERATIONS.into_iter().enumerate()
        {
            let exact = boolean(
                &context,
                &[
                    prepared.hypermesh[0].as_ref(),
                    prepared.hypermesh[1].as_ref(),
                ],
                BooleanProgram::Operation(boolean_op),
            )
            .unwrap_or_else(|error| panic!("HyperMesh {policy} {operation} failed: {error}"))
            .into_value();
            if policy == "STRICT" {
                strict_outputs.push(exact.clone());
            } else {
                assert_eq!(
                    exact, strict_outputs[operation_index],
                    "HyperMesh policy outputs differ for {operation}",
                );
            }
            assert!(
                boundary_is_balanced(&exact.results[0]),
                "HyperMesh {policy} {operation} exact output has a boundary",
            );
            let degenerate_triangles = exact.results[0]
                .triangles
                .iter()
                .filter(|triangle| {
                    let [a, b, c] = triangle.map(|index| exact.vertices[index as usize].clone());
                    !Plane::points_are_nondegenerate(&context, &a, &b, &c)
                        .expect("YeahRight output triangle predicate must decide")
                        .into_value()
                })
                .count();
            assert_eq!(
                degenerate_triangles, 0,
                "HyperMesh {policy} {operation} exact output contains degenerate triangles",
            );
            let output = raw_from_hypermesh_batch(&exact, 0);
            let summary = summarize(&output);
            // The competitive summary merges vertices through a quantized
            // binary64 key. Symmetric difference can retain exact distinct
            // crossings that share that key, so its exact closure evidence
            // above is the authoritative topology check.
            if boolean_op != BooleanOp::SymmetricDifference {
                assert!(
                    summary.closed,
                    "HyperMesh {policy} {operation} output is open: {summary:?}",
                );
            }
            assert!(
                summary.finite,
                "HyperMesh {policy} {operation} output is non-finite",
            );
        }
    }
}

#[test]
#[ignore = "requires the opt-in external benchmark fixture (YEAHRIGHT_BENCH=1)"]
fn yeahright_single_and_multi_expression_results_remain_consistent() {
    let case = yeahright_boolean_case();
    let prepared = prepare_yeahright(&case);
    for (policy, context) in predicate_contexts() {
        let views = [
            prepared.hypermesh[0].as_ref(),
            prepared.hypermesh[1].as_ref(),
        ];
        let nodes =
            HYPERMESH_OPERATIONS.map(|(_, operation)| BooleanExpression::Operation(operation));
        let roots = [0, 1, 2, 3];
        let batch = boolean(
            &context,
            &views,
            BooleanProgram::Expressions {
                nodes: &nodes,
                roots: &roots,
            },
        )
        .unwrap_or_else(|error| panic!("HyperMesh batch {policy} failed: {error}"))
        .into_value();
        for (output, (operation, boolean_op)) in HYPERMESH_OPERATIONS.into_iter().enumerate() {
            let single = boolean(&context, &views, BooleanProgram::Operation(boolean_op))
                .unwrap_or_else(|error| {
                    panic!("HyperMesh single {policy} {operation} failed: {error}")
                })
                .into_value();
            assert!(boundary_is_balanced(&batch.results[output]));
            let batch_summary = summarize(&raw_from_hypermesh_batch(&batch, output));
            let single_summary = summarize(&raw_from_hypermesh_batch(&single, 0));
            if boolean_op != BooleanOp::SymmetricDifference {
                assert!(batch_summary.closed);
            }
            assert!(batch_summary.finite);
            assert_close(
                batch_summary.volume,
                single_summary.volume,
                &format!("HyperMesh batch {policy} {operation} volume"),
            );
            assert_close(
                batch_summary.surface_area,
                single_summary.surface_area,
                &format!("HyperMesh batch {policy} {operation} area"),
            );
        }
    }
}

#[test]
#[ignore = "requires the opt-in external benchmark fixture (YEAHRIGHT_BENCH=1)"]
fn full_resolution_yeahright_reaches_and_validates_in_hypermesh() {
    let raw = yeahright_control_mesh();
    let summary = summarize(&raw);
    assert_eq!(raw.positions.len(), YEAHRIGHT_CONTROL_VERTICES);
    assert_eq!(summary.triangles, YEAHRIGHT_CONTROL_TRIANGLES);
    assert!(summary.closed);
    assert!(summary.finite);
    assert!(summary.nondegenerate);

    let exact = to_hypermesh(&raw);
    assert_eq!(exact.positions.len(), YEAHRIGHT_CONTROL_VERTICES);
    assert_eq!(exact.triangles.len(), YEAHRIGHT_CONTROL_TRIANGLES);
    polygon_soup(&APPROXIMATE_CONTEXT, &[exact.as_ref()])
        .expect("full-resolution YeahRight must satisfy Hypermesh's closed-PWN input contract");
}

#[test]
#[ignore = "manual 11,894-by-11,894 certified-empty memory-ceiling test"]
fn full_resolution_yeahright_rotated_intersection_certifies_empty() {
    let source = yeahright_control_mesh();
    let rotated = support::RawMesh {
        positions: source
            .positions
            .iter()
            .map(|[x, y, z]| [z + 1.0, y + 12.0, 1.0 - x])
            .collect(),
        triangles: source.triangles.clone(),
    };
    let left = to_hypermesh(&source);
    let right = to_hypermesh(&rotated);
    let outcome = boolean(
        &APPROXIMATE_CONTEXT,
        &[left.as_ref(), right.as_ref()],
        BooleanProgram::Operation(BooleanOp::Intersection),
    )
    .expect("full-resolution YeahRight intersection must remain valid");
    assert_eq!(outcome.certainty, hypermesh::MeshCertainty::Certified);
    assert!(outcome.value.vertices.is_empty());
    assert!(outcome.value.results[0].triangles.is_empty());
}

#[test]
#[ignore = "manual 3,360/13,440-triangle memory-pressure stress"]
fn larger_yeahright_fixtures_expose_memory_pressure() {
    let cases = YEAHRIGHT_STRESS_SUBDIVISIONS.map(yeahright_boolean_case_with_subdivisions);
    let prepared = cases.iter().map(prepare_yeahright).collect::<Vec<_>>();

    for ((case, inputs), _subdivisions) in cases
        .iter()
        .zip(&prepared)
        .zip(YEAHRIGHT_STRESS_SUBDIVISIONS)
    {
        let triangle_count = case.left.triangles.len();
        assert_eq!(case.left.triangles.len(), triangle_count);
        let summary = summarize(&case.left);
        assert!(summary.closed, "{} is open", case.name);
        assert!(summary.finite, "{} is non-finite", case.name);
        assert!(summary.nondegenerate, "{} is degenerate", case.name);
        assert_eq!(inputs.hypermesh[0].triangles.len(), triangle_count);
        assert!(inputs.boolmesh[0].is_manifold());
        assert_eq!(inputs.manifold[0].num_tri(), triangle_count);

        let exact = run_hypermesh_batch(&inputs.hypermesh, Operation::Union);
        assert!(
            boolean_mesh_closure_evidence(&exact.results[0]).has_no_boundary(),
            "{} exact union has a boundary",
            case.name
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
