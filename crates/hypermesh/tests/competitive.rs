#[path = "../competitive/support.rs"]
#[allow(dead_code)]
mod support;

use std::collections::BTreeMap;

use hypermesh::{
    BooleanExpression, BooleanMeshResult, BooleanOp, BooleanProgram, MeshContext, Plane,
    PredicatePolicy, Real, boolean, boolean_mesh_closure_evidence, certify_convex_mesh,
    polygon_soup,
};
use hyperreal::Rational;
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

const ALL_BOOLEAN_NODES: [BooleanExpression; 8] = [
    BooleanExpression::Operation(BooleanOp::Union),
    BooleanExpression::Operation(BooleanOp::Intersection),
    BooleanExpression::Operation(BooleanOp::Difference),
    BooleanExpression::Operation(BooleanOp::SymmetricDifference),
    BooleanExpression::Operand(0),
    BooleanExpression::Operand(1),
    BooleanExpression::Not(4),
    BooleanExpression::And([5, 6]),
];
const ALL_BOOLEAN_ROOTS: [u32; 5] = [0, 1, 2, 7, 3];

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

fn exact_six_volume(batch: &hypermesh::BooleanMeshBatch, output: usize) -> Rational {
    let mut total = Real::zero();
    for triangle in &batch.results[output].triangles {
        let [a, b, c] = triangle.map(|vertex| &batch.vertices[vertex as usize]);
        let cross_x = &b.y * &c.z - &b.z * &c.y;
        let cross_y = &b.z * &c.x - &b.x * &c.z;
        let cross_z = &b.x * &c.y - &b.y * &c.x;
        total += &a.x * cross_x + &a.y * cross_y + &a.z * cross_z;
    }
    total
        .abs()
        .exact_rational()
        .expect("exact-rational mesh has an exact-rational volume")
}

fn canonical_oriented_triangle([a, b, c]: [u32; 3]) -> [u32; 3] {
    if a <= b && a <= c {
        [a, b, c]
    } else if b <= c {
        [b, c, a]
    } else {
        [c, a, b]
    }
}

fn assert_translated_batch_equivalent(
    context: &MeshContext,
    actual: &hypermesh::BooleanMeshBatch,
    offsets: &[Real; 3],
    expected: &hypermesh::BooleanMeshBatch,
    label: &str,
) {
    assert_eq!(actual.results.len(), expected.results.len(), "{label}");
    assert_eq!(actual.vertices.len(), expected.vertices.len(), "{label}");

    let mut vertex_map = Vec::with_capacity(actual.vertices.len());
    for point in &actual.vertices {
        let translated = [
            &point.x - &offsets[0],
            &point.y - &offsets[1],
            &point.z - &offsets[2],
        ];
        let matches = expected
            .vertices
            .iter()
            .enumerate()
            .filter_map(|(index, candidate)| {
                let equal = translated
                    .iter()
                    .zip([&candidate.x, &candidate.y, &candidate.z])
                    .all(|(coordinate, candidate)| {
                        hypermesh::geometry::compare_real(context, coordinate, candidate)
                            .is_ok_and(|comparison| comparison.value.is_eq())
                    });
                equal.then_some(index as u32)
            })
            .collect::<Vec<_>>();
        assert_eq!(matches.len(), 1, "{label} translated vertex identity");
        vertex_map.push(matches[0]);
    }

    for (output, (actual, expected)) in actual.results.iter().zip(&expected.results).enumerate() {
        assert_eq!(
            actual.exterior_inside, expected.exterior_inside,
            "{label} output {output}"
        );
        let mut actual_facets = actual
            .triangles
            .iter()
            .zip(&actual.sources)
            .map(|(triangle, source)| {
                (
                    canonical_oriented_triangle(triangle.map(|index| vertex_map[index as usize])),
                    *source,
                )
            })
            .collect::<Vec<_>>();
        let mut expected_facets = expected
            .triangles
            .iter()
            .zip(&expected.sources)
            .map(|(&triangle, &source)| (canonical_oriented_triangle(triangle), source))
            .collect::<Vec<_>>();
        actual_facets.sort_unstable();
        expected_facets.sort_unstable();
        assert_eq!(actual_facets, expected_facets, "{label} output {output}");
    }
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
fn sparse_multishell_arrangement_scales_componentwise_under_both_policies() {
    for shell_count in support::SPARSE_MULTISHELL_COUNTS {
        let case = support::sparse_multishell_tetrahedra_case(shell_count);
        let inputs = [to_hypermesh(&case.left), to_hypermesh(&case.right)];
        let expected_triangles = [16, 4, 12, 8, 20].map(|count| count * shell_count);
        let expected_six_volumes = [127, 1, 63, 63, 126].map(|value| value * shell_count);
        let mut strict = None;

        for (policy, context) in predicate_contexts() {
            let outcome = boolean(
                &context,
                &[inputs[0].as_ref(), inputs[1].as_ref()],
                BooleanProgram::Expressions {
                    nodes: &ALL_BOOLEAN_NODES,
                    roots: &ALL_BOOLEAN_ROOTS,
                },
            )
            .unwrap_or_else(|error| panic!("{} {policy}: {error}", case.name));
            assert_eq!(outcome.certainty, hypermesh::MeshCertainty::Certified);
            if let Some(strict) = &strict {
                assert_eq!(outcome.value, *strict, "policy outputs differ");
            } else {
                strict = Some(outcome.value.clone());
            }
            assert_eq!(outcome.value.vertices.len(), shell_count * 11);
            for output in 0..ALL_BOOLEAN_ROOTS.len() {
                assert_eq!(
                    outcome.value.results[output].triangles.len(),
                    expected_triangles[output],
                    "{policy} output {output} triangle count"
                );
                assert!(boundary_is_balanced(&outcome.value.results[output]));
                assert_eq!(
                    exact_six_volume(&outcome.value, output),
                    Rational::new(
                        i64::try_from(expected_six_volumes[output])
                            .expect("sparse fixture volume fits i64"),
                    ),
                    "{policy} output {output} exact volume"
                );
            }
        }
    }
}

#[test]
fn transverse_self_pwn_clusters_scale_under_both_policies() {
    for cluster_count in support::SELF_PWN_CLUSTER_COUNTS {
        let case = support::transverse_self_pwn_cluster_case(cluster_count);
        let inputs = [to_hypermesh(&case.left), to_hypermesh(&case.right)];
        let expected_triangles = [
            cluster_count * 16 + 4,
            0,
            cluster_count * 16,
            4,
            cluster_count * 16 + 4,
        ];
        let expected_six_volumes = [
            cluster_count * 127 + 64,
            0,
            cluster_count * 127,
            64,
            cluster_count * 127 + 64,
        ];
        let mut strict = None;

        for (policy, context) in predicate_contexts() {
            let outcome = boolean(
                &context,
                &[inputs[0].as_ref(), inputs[1].as_ref()],
                BooleanProgram::Expressions {
                    nodes: &ALL_BOOLEAN_NODES,
                    roots: &ALL_BOOLEAN_ROOTS,
                },
            )
            .unwrap_or_else(|error| panic!("{} {policy}: {error}", case.name));
            assert_eq!(outcome.certainty, hypermesh::MeshCertainty::Certified);
            if let Some(strict) = &strict {
                assert_eq!(outcome.value, *strict, "policy outputs differ");
            } else {
                strict = Some(outcome.value.clone());
            }
            assert_eq!(outcome.value.vertices.len(), cluster_count * 10 + 4);
            for output in 0..ALL_BOOLEAN_ROOTS.len() {
                assert_eq!(
                    outcome.value.results[output].triangles.len(),
                    expected_triangles[output],
                    "{policy} output {output} triangle count"
                );
                assert!(boundary_is_balanced(&outcome.value.results[output]));
                assert_eq!(
                    exact_six_volume(&outcome.value, output),
                    Rational::new(
                        i64::try_from(expected_six_volumes[output])
                            .expect("self-PWN fixture volume fits i64"),
                    ),
                    "{policy} output {output} exact volume"
                );
            }
        }
    }
}

#[test]
fn dense_crossing_grid_exhausts_the_public_arrangement_under_both_policies() {
    assert_dense_crossing_grid(&support::DENSE_CROSSING_GRID_LINE_COUNTS[..2]);
}

#[test]
#[ignore = "manual 1,572-triangle / 16,900-crossing exact policy-equality gate"]
fn dense_crossing_grid_large_policy_outputs_are_exactly_equal() {
    assert_dense_crossing_grid(&support::DENSE_CROSSING_GRID_LINE_COUNTS[2..]);
}

fn assert_dense_crossing_grid(line_counts: &[usize]) {
    for &line_count in line_counts {
        let case = support::dense_crossing_grid_case(line_count);
        let inputs = [to_hypermesh(&case.left), to_hypermesh(&case.right)];
        let expected_six_volume = Rational::new(
            i64::try_from(6 * (3 * line_count * line_count + 2 * line_count))
                .expect("crossing-grid six-volume fits i64"),
        );
        let mut strict = None;

        for (policy, context) in predicate_contexts() {
            let outcome = boolean(
                &context,
                &[inputs[0].as_ref(), inputs[1].as_ref()],
                BooleanProgram::Operation(BooleanOp::Intersection),
            )
            .unwrap_or_else(|error| panic!("{} {policy}: {error}", case.name));
            assert_eq!(outcome.certainty, hypermesh::MeshCertainty::Certified);
            if let Some(strict) = &strict {
                assert_eq!(&outcome.value, strict, "policy outputs differ");
            } else {
                strict = Some(outcome.value.clone());
            }
            assert!(boundary_is_balanced(&outcome.value.results[0]));
            assert_eq!(exact_six_volume(&outcome.value, 0), expected_six_volume);
        }
    }
}

#[test]
fn opposite_diagonal_coplanar_overlay_is_exact_under_both_policies() {
    let case = support::dense_coplanar_box_case(4);
    let inputs = [to_hypermesh(&case.left), to_hypermesh(&case.right)];
    let expected_triangles = [384, 384, 0, 0, 0];
    let expected_volumes = [64.0, 64.0, 0.0, 0.0, 0.0];
    let mut strict = None;

    for (policy, context) in predicate_contexts() {
        let outcome = boolean(
            &context,
            &[inputs[0].as_ref(), inputs[1].as_ref()],
            BooleanProgram::Expressions {
                nodes: &ALL_BOOLEAN_NODES,
                roots: &ALL_BOOLEAN_ROOTS,
            },
        )
        .unwrap_or_else(|error| panic!("{} {policy} failed: {error}", case.name));
        assert_eq!(outcome.certainty, hypermesh::MeshCertainty::Certified);
        if let Some(strict) = &strict {
            assert_eq!(outcome.value, *strict, "policy outputs differ");
        } else {
            strict = Some(outcome.value.clone());
        }

        for (output_index, ((output, expected_triangles), expected_volume)) in outcome
            .value
            .results
            .iter()
            .zip(expected_triangles)
            .zip(expected_volumes)
            .enumerate()
        {
            assert_eq!(
                output.triangles.len(),
                expected_triangles,
                "{policy} output {output_index} triangle count",
            );
            assert!(
                boundary_is_balanced(output),
                "{policy} output {output_index} is directionally unbalanced",
            );
            let summary = summarize(&raw_from_hypermesh_batch(&outcome.value, output_index));
            assert!(summary.finite, "{policy} output {output_index}");
            assert!(summary.nondegenerate, "{policy} output {output_index}");
            assert_close(
                summary.volume,
                expected_volume,
                &format!("{policy} output {output_index} volume"),
            );
        }
    }
}

#[test]
fn wide_rational_similarity_preserves_every_boolean_under_both_policies() {
    let normalized_six_volumes = [504, 72, 312, 120, 432];
    let mut reference = None;

    for shift in support::WIDE_RATIONAL_SHIFTS {
        let case = support::wide_rational_overlapping_box_case(2, shift);
        let inputs = [case.left, case.right];
        let scale = support::wide_rational_scale(shift)
            .exact_rational()
            .expect("fixture scale is exact rational");
        let scale_cubed = (&scale * &scale) * &scale;
        let mut strict = None;

        for (policy, context) in predicate_contexts() {
            let outcome = boolean(
                &context,
                &[inputs[0].as_ref(), inputs[1].as_ref()],
                BooleanProgram::Expressions {
                    nodes: &ALL_BOOLEAN_NODES,
                    roots: &ALL_BOOLEAN_ROOTS,
                },
            )
            .unwrap_or_else(|error| panic!("wide-rational shift {shift} {policy}: {error}"));
            assert_eq!(outcome.certainty, hypermesh::MeshCertainty::Certified);
            if let Some(strict) = &strict {
                assert_eq!(outcome.value, *strict, "shift {shift} policy output");
            } else {
                strict = Some(outcome.value.clone());
            }

            for (output, normalized_six_volume) in normalized_six_volumes.into_iter().enumerate() {
                assert!(boundary_is_balanced(&outcome.value.results[output]));
                assert_eq!(
                    exact_six_volume(&outcome.value, output),
                    Rational::new(normalized_six_volume) * &scale_cubed,
                    "shift {shift} {policy} output {output} volume",
                );
            }

            if policy == "STRICT" {
                let normalized_vertices = outcome
                    .value
                    .vertices
                    .iter()
                    .map(|point| {
                        [&point.x, &point.y, &point.z].map(|coordinate| {
                            coordinate
                                .exact_rational()
                                .expect("wide-rational output coordinate")
                                / &scale
                        })
                    })
                    .collect::<Vec<_>>();
                let topology = outcome
                    .value
                    .results
                    .iter()
                    .map(|result| result.triangles.clone())
                    .collect::<Vec<_>>();
                if let Some((reference_vertices, reference_topology)) = &reference {
                    assert_eq!(&normalized_vertices, reference_vertices);
                    assert_eq!(&topology, reference_topology);
                } else {
                    reference = Some((normalized_vertices, topology));
                }
            }
        }
    }
}

#[test]
fn thin_dyadic_affine_embedding_preserves_every_boolean_under_both_policies() {
    let normalized_six_volumes = [504, 72, 312, 120, 432];
    let mut reference = None;

    for shift in support::THIN_DYADIC_SHIFTS {
        let case = support::thin_dyadic_overlapping_box_case(2, shift);
        let inputs = [case.left, case.right];
        let scale = support::thin_dyadic_scale(shift)
            .exact_rational()
            .expect("fixture scale is exact dyadic");
        let mut strict = None;

        for (policy, context) in predicate_contexts() {
            let outcome = boolean(
                &context,
                &[inputs[0].as_ref(), inputs[1].as_ref()],
                BooleanProgram::Expressions {
                    nodes: &ALL_BOOLEAN_NODES,
                    roots: &ALL_BOOLEAN_ROOTS,
                },
            )
            .unwrap_or_else(|error| panic!("thin-dyadic shift {shift} {policy}: {error}"));
            assert_eq!(outcome.certainty, hypermesh::MeshCertainty::Certified);
            if let Some(strict) = &strict {
                assert_eq!(outcome.value, *strict, "shift {shift} policy output");
            } else {
                strict = Some(outcome.value.clone());
            }

            for (output, normalized_six_volume) in normalized_six_volumes.into_iter().enumerate() {
                assert!(boundary_is_balanced(&outcome.value.results[output]));
                assert_eq!(
                    exact_six_volume(&outcome.value, output),
                    Rational::new(normalized_six_volume) * &scale,
                    "shift {shift} {policy} output {output} volume",
                );
            }

            if policy == "STRICT" {
                let normalized_vertices = outcome
                    .value
                    .vertices
                    .iter()
                    .map(|point| {
                        let x = point.x.exact_rational().expect("thin-dyadic x");
                        let y = point.y.exact_rational().expect("thin-dyadic y");
                        let z = point.z.exact_rational().expect("thin-dyadic z") / &scale;
                        [x - &z, y, z]
                    })
                    .collect::<Vec<_>>();
                let topology = outcome
                    .value
                    .results
                    .iter()
                    .map(|result| result.triangles.clone())
                    .collect::<Vec<_>>();
                if let Some((reference_vertices, reference_topology)) = &reference {
                    assert_eq!(&normalized_vertices, reference_vertices);
                    assert_eq!(&topology, reference_topology);
                } else {
                    reference = Some((normalized_vertices, topology));
                }
            }
        }
    }
}

#[test]
fn deep_symbolic_translation_obeys_policy_at_every_depth() {
    let base = support::exact_mesh_pair(
        corpus()
            .into_iter()
            .find(|case| case.name == "overlapping_boxes")
            .expect("base exact box case"),
    );
    let reference = boolean(
        &MeshContext::new(PredicatePolicy::STRICT),
        &[base.left.as_ref(), base.right.as_ref()],
        BooleanProgram::Expressions {
            nodes: &ALL_BOOLEAN_NODES,
            roots: &ALL_BOOLEAN_ROOTS,
        },
    )
    .expect("rational reference arrangement")
    .into_value();
    for depth in support::DEEP_SYMBOLIC_TRANSLATION_DEPTHS {
        let (case, offsets) = support::deep_symbolic_translated_box_case(depth);
        assert!(
            offsets
                .iter()
                .all(|coordinate| coordinate.exact_rational_ref().is_none()),
            "depth {depth} must remain genuinely non-rational"
        );
        let strict_context = MeshContext::new(PredicatePolicy::STRICT);
        let strict_input =
            polygon_soup(&strict_context, &[case.left.as_ref(), case.right.as_ref()])
                .unwrap_or_else(|error| panic!("{} STRICT input: {error}", case.name));
        assert_eq!(strict_input.certainty, hypermesh::MeshCertainty::Certified);
        let strict = boolean(
            &strict_context,
            &[case.left.as_ref(), case.right.as_ref()],
            BooleanProgram::Expressions {
                nodes: &ALL_BOOLEAN_NODES,
                roots: &ALL_BOOLEAN_ROOTS,
            },
        );
        if depth == 1 {
            let strict = strict.unwrap_or_else(|error| panic!("{} STRICT: {error}", case.name));
            assert_eq!(strict.certainty, hypermesh::MeshCertainty::Certified);
            assert_translated_batch_equivalent(
                &strict_context,
                &strict.value,
                &offsets,
                &reference,
                case.name,
            );
        } else {
            assert!(
                matches!(
                    strict,
                    Err(hypermesh::HypermeshError::PredicateUndecided { .. })
                ),
                "{} STRICT must preserve an unresolved exact predicate",
                case.name
            );
        }

        let context = MeshContext::new(PredicatePolicy::APPROXIMATE_512);
        let input = polygon_soup(&context, &[case.left.as_ref(), case.right.as_ref()])
            .unwrap_or_else(|error| panic!("{} APPROXIMATE_512 input: {error}", case.name));
        assert_eq!(input.certainty, hypermesh::MeshCertainty::Certified);
        let outcome = boolean(
            &context,
            &[case.left.as_ref(), case.right.as_ref()],
            BooleanProgram::Expressions {
                nodes: &ALL_BOOLEAN_NODES,
                roots: &ALL_BOOLEAN_ROOTS,
            },
        )
        .unwrap_or_else(|error| panic!("{} APPROXIMATE_512: {error}", case.name));
        assert_eq!(
            outcome.certainty,
            if depth == 1 {
                hypermesh::MeshCertainty::Certified
            } else {
                hypermesh::MeshCertainty::Approximate512Consumed
            }
        );
        assert_translated_batch_equivalent(
            &context,
            &outcome.value,
            &offsets,
            &reference,
            case.name,
        );
    }
}

#[test]
fn lower_dimensional_closed_pwn_contacts_are_total_under_both_policies() {
    for case in lower_dimensional_contact_corpus() {
        let inputs = [to_hypermesh(&case.left), to_hypermesh(&case.right)];
        let right_volume = summarize(&case.right).volume;
        let expected_volumes = [
            case.expected_volumes[0],
            case.expected_volumes[1],
            case.expected_volumes[2],
            right_volume - case.expected_volumes[1],
            case.expected_volumes[0] - case.expected_volumes[1],
        ];
        for (policy, context) in predicate_contexts() {
            let outcome = boolean(
                &context,
                &[inputs[0].as_ref(), inputs[1].as_ref()],
                BooleanProgram::Expressions {
                    nodes: &ALL_BOOLEAN_NODES,
                    roots: &ALL_BOOLEAN_ROOTS,
                },
            )
            .unwrap_or_else(|error| panic!("{} {policy} failed: {error}", case.name));
            assert_eq!(outcome.certainty, hypermesh::MeshCertainty::Certified);
            assert_eq!(outcome.value.results.len(), ALL_BOOLEAN_ROOTS.len());
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
