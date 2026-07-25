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
| Algebraic polyline region crossings and all four Boolean operations (`2491124`) | `shared_demo_algebraic_polyline_blocker_resolves_all_boolean_modes`; `shared_demo_conic_cubic_contacts_are_complete`; `shared_demo_cubic_pair_contacts_are_complete` |
| Mixed-family rational-quadratic `RealSign` selection and complex curved Booleans (`2860f1d`) | `pathological_cell_reaches_curved_intersections_and_decidable_polygon_booleans`; `implicit_conic_route_replays_degree_elevated_line_contact_in_both_orders`; `algebraic_tangent_order_handles_distinct_generators_with_disjoint_enclosures` |
| Real-coefficient conic/cubic parameter images for transcendental weights | `pathological_pi_weight_conic_decides_native_booleans_without_projection`; `rational_point_image_retains_real_coefficient_root_expression` |
| Simultaneous exact straight-skeleton line events (`ec8dda9`) | `non_general_position_l_shape_materializes_terminal_vertex_event`; `non_general_position_line_fixtures_complete_exactly` |
| Exact line-Boolean branch vertices (`d9d009b`) | `boundary_chain_assembly_orders_branch_points_by_tangent`; `boundary_chain_assembly_rejects_equal_tangent_branch_points` |
| Certified removal of unresolved opposite boundary pairs (`f1a7562`) | `unresolved_boundaries_require_opposite_fragment_pair_evidence`; `unresolved_boundaries_retain_certified_opposite_fragment_pairs` |
| Clone-shared additive cancellation below the sign-refinement floor | `shared_cancellation_resolves_rational_weight_monotonicity_blocker`; `shared_cancellation_resolves_rational_evaluation_and_bounds_blockers`; `shared_cancellation_resolves_disjoint_rational_contact_blocker`; Hyperreal `add_cancels_structurally_shared_term_across_nested_sum`, `atan2_shared_cancellation_resolves_positive_y_below_refinement_floor`, and the two `computable_atan2_shared_cancellation_*` regressions |
| Retained overlap orientation, indices, spans, and traversal materialization (`92af76a`, `561fc1f`, `047caba`, `c6d3d1c`) | `retained_linear_overlap_split_graph_rejects_forged_orientation`; `retained_resolved_overlap_constructor_rejects_unordered_indices`; `resolved_linear_overlap_traversal_materializes_native_and_retained_regions`; `retained_linear_overlap_refinement_evidence_reversed_span_orientation` |
| Simple Bezier arrangement branches (`516c7c0`) | `tangent_ordered_traversal_resolves_simple_branch_vertex`; `tangent_ordered_traversal_uses_second_order_for_equal_outgoing_tangents`; `tangent_ordered_traversal_rejects_equal_second_order_outgoing_tangents` |

The retained Sturm-chain optimization is additionally guarded by
`retained_sturm_certificate_classifies_mixed_root_multiplicity`, which proves
that both simple and repeated isolated roots remain classified from the
certificate produced during root isolation.

Run the complete gate with:

```bash
cargo test --all-features --all-targets
cargo test --no-default-features --lib --tests
```
