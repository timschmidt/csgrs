# Blocker regression coverage

Every committed change that claims to resolve, remove, or safely discharge an
exact-topology blocker must retain a named regression. The regression must
exercise the decision that formerly blocked; a test that only checks blocker
provenance does not satisfy this gate.

The current history audit covers blocker-resolution commits whose resolved
paths remain reachable in the current API, plus the all-family `CurveRegion2`
milestone. Removed report-only algebraic handoff types are outside this gate
because neither their blocker nor their resolver remains executable.

| Resolved blocker | Regression |
| --- | --- |
| Complete mixed-family `CurveRegion2` workload: every curve family, every `Real` representation fixture, and all four exact Boolean operations | `full_pathological_native_workload_decides_all_268_exact_booleans`; `pathological_pi_weight_conic_decides_native_booleans_without_projection`; `retired_exact_curve_region_boolean_failures_remain_in_the_corpus` |
| Algebraic polyline region crossings and all four Boolean operations (`2491124`) | `retired_exact_curve_region_boolean_failures_remain_in_the_corpus` (`AlgebraicPolylineContacts`); `shared_demo_conic_cubic_contacts_are_complete`; `shared_demo_cubic_pair_contacts_are_complete` |
| Mixed-family rational-quadratic `RealSign` selection and complex curved Booleans (`2860f1d`) | `pathological_cell_reaches_curved_intersections_and_decidable_polygon_booleans`; `implicit_conic_route_replays_degree_elevated_line_contact_in_both_orders`; `algebraic_tangent_order_handles_distinct_generators_with_disjoint_enclosures` |
| Real-coefficient conic/cubic parameter images for transcendental weights | `pathological_pi_weight_conic_decides_native_booleans_without_projection`; `rational_point_image_retains_real_coefficient_root_expression` |
| Uniform-weight general rational-Bezier region orientation and explicit nonuniform rational interior-side evidence | `uniform_weight_general_rational_bezier_uses_exact_polynomial_area`; `retired_exact_curve_region_boolean_failures_remain_in_the_corpus` (`UniformWeightGeneralRationalArea`); `explicit_loop_topology_supports_reversed_nonuniform_rational_regions` |
| Rational/cubic contact discarded by an unsound algebraic candidate-image interval accelerator | persisted proptest seed `df5adb6252abf2023ed022a724fde67811356d8f24b1b7897472929fa32b8c82`; `retired_candidate_interval_pruning_completes`; `retired_exact_curve_region_boolean_failures_remain_in_the_corpus` (`CandidateImageIntervalPruning`) |
| Degree-twelve rational intersection parameter mapped through a cubic rational coordinate | Hypersolve `rational_image_supports_degree_twelve_source_with_cubic_map`; `retired_exact_curve_region_boolean_failures_remain_in_the_corpus` (`RationalImageDegreeBound`) |
| Finite exact line-image contacts hidden by degree-elevated projective base factors, including algebraic contact parameters | persisted proptest seed `523b6319840126b06ce95d28a203f63f83762651332045afdb62d6f3858f577d`; `implicit_conic_route_replays_quadratic_line_contact`; `exact_line_image_route_replays_algebraic_conic_contact`; `retired_exact_curve_region_boolean_failures_remain_in_the_corpus` (`FiniteLineImageContactReplay`) |
| XOR traversal at a shared endpoint with four incident carriers selected one outgoing half-edge twice | persisted proptest seed `692f0be287f7802dfea1328438bbba68a15a180b672b3f3f8296b26caf991a10`; `shared_endpoint_xor_completes`; `retired_exact_curve_region_boolean_failures_remain_in_the_corpus` (`SharedEndpointXorTraversal`) |
| An algebraic conic chart proved an extension contact outside `[0, 1]` but dropped that absence proof before replaying an interior rational-quadratic/cubic contact | persisted proptest seed `3b1daff23b05a6bee1a9c78a2c902941e1dc4c7b767f5b7903b90bbc165b410f`; `implicit_conic_route_retains_an_interior_rational_quadratic_cubic_contact`; `retired_exact_curve_region_boolean_failures_remain_in_the_corpus` (`ConicChartAbsencePropagation`) |
| Polynomial-graph replay rejected a valid general-rational/cubic contact when the two resultant projections had different candidate counts | `polynomial_graph_replay_accepts_unequal_resultant_projection_counts`; `retired_exact_curve_region_boolean_failures_remain_in_the_corpus` (`PolynomialGraphProjectionReplay`) |
| Specializing a rational-curve resultant at an integer sample canceled its leading eliminated coefficient, so interpolation used a lower-degree Sylvester determinant and shifted the retained roots | `resultant_replay_retains_an_interior_nonuniform_rational_cubic_contact`; Hypersolve `rational_resultant_skips_specialized_degree_drop_samples`; `retired_exact_curve_region_boolean_failures_remain_in_the_corpus` (`RationalResultantDegreeDropSampling`) |
| Independently degree-elevated line images reached a `RealSign` resultant blocker before their exact partial overlap was replayed | persisted proptest seed `1b148efd4f31020f4f09d713a65b897d5475a442fc3de8cdb03dad07a7cae5c5`; `retired_exact_curve_region_boolean_failures_remain_in_the_corpus` (`DegreeElevatedLineImageOverlap`) |
| Endpoint deflation changed the retained carrier of a remaining algebraic conic root, but shared simple-root classification still required the original pre-deflation polynomial | persisted proptest seed `b3f6311940f997e36caddf509424ef76e40326cffea4c73b4b38f296769c16ca`; `simple_root_certificate_accepts_an_endpoint_deflated_algebraic_carrier`; `conic_endpoint_root_isolation_completes`; `retired_exact_curve_region_boolean_failures_remain_in_the_corpus` (`ConicEndpointRootIsolation`) |
| Simultaneous exact straight-skeleton line events (`ec8dda9`) | `non_general_position_l_shape_materializes_terminal_vertex_event`; `non_general_position_line_fixtures_complete_exactly` |
| Exact line-Boolean branch vertices (`d9d009b`) | `boundary_chain_assembly_orders_branch_points_by_tangent`; `boundary_chain_assembly_rejects_equal_tangent_branch_points` |
| Certified removal of unresolved opposite boundary pairs (`f1a7562`) | `unresolved_boundaries_require_opposite_fragment_pair_evidence`; `unresolved_boundaries_retain_certified_opposite_fragment_pairs` |
| Clone-shared additive cancellation below the sign-refinement floor | `shared_cancellation_resolves_rational_weight_monotonicity_blocker`; `shared_cancellation_resolves_rational_evaluation_and_bounds_blockers`; `shared_cancellation_resolves_disjoint_rational_contact_blocker`; Hyperreal `add_cancels_structurally_shared_term_across_nested_sum`, `atan2_shared_cancellation_resolves_positive_y_below_refinement_floor`, and the two `computable_atan2_shared_cancellation_*` regressions |
| Exact quadratic-surd equality at a convex erosion collapse | `unified_region_convex_erosion_keeps_symbolic_diagonal_offsets_and_collapse_exact`; Hyperreal `exact_sign_reduces_quadratic_surd_field_identities`, `exact_sign_orders_nonzero_quadratic_surds`, and `opposite_sign_quadratic_surd_is_certified_nonzero` |
| Retained overlap orientation, indices, spans, and traversal materialization (`92af76a`, `561fc1f`, `047caba`, `c6d3d1c`) | `retained_linear_overlap_split_graph_rejects_forged_orientation`; `retained_resolved_overlap_constructor_rejects_unordered_indices`; `resolved_linear_overlap_traversal_materializes_native_and_retained_regions`; `retained_linear_overlap_refinement_evidence_reversed_span_orientation` |
| Simple Bezier arrangement branches (`516c7c0`) | `tangent_ordered_traversal_resolves_simple_branch_vertex`; `tangent_ordered_traversal_uses_second_order_for_equal_outgoing_tangents`; `tangent_ordered_traversal_rejects_equal_second_order_outgoing_tangents` |

The retained Sturm-chain optimization is additionally guarded by
`retained_sturm_certificate_classifies_mixed_root_multiplicity`, which proves
that both simple and repeated isolated roots remain classified from the
certificate produced during root isolation.

The exact curved-region Boolean property test writes minimized failing seeds to
`tests/hypercurve_curve_region_boolean_fuzz.proptest-regressions` and replays
them before generating new cases. Seeds remain after their fix. Every retired
failure category must also have a named geometry in
`retired_failure_corpus`; the corpus test compares its IDs against
`RetiredFailure::ALL`, so deleting the last reproducer for a retired category
fails the regression gate.

Run the complete gate with:

```bash
cargo test --all-features --all-targets
cargo test --no-default-features --lib --tests
```
