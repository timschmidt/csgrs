# Probe incidence continuation

The broader Hypercurve goal remains active. Current focused progress is recorded in probe-incidence-20260923-proof.md and retained-fragment-witness-20260923-next-steps.md.

TERMINAL/REAPED owned process: session 61279, qualify-probe-incidence-20260923.py, prefix probe-incidence-20260923-final1. The owned source freeze has ended. No owned build/test/probe process remains active. Both all-target feature checks pass. The complete 1,217-case library run is terminal against d00fa18's bound baseline (1,192 passes, six ignored, nine assertions, nine 75-second limits). Current production binary SHA256: 85dd2d619f2098d2e13fe9a62304d640403e2e58dccba08bdec477959d68ecb7.

Main production candidate SHA256:
- src/bezier_offset.rs: 5dd4e977dd884b6bc39a41d7019f2721e9dcf922e8702af78937c64ba9a187c7
- src/curve_region_boolean.rs: a1d9266a0ecd061aa3b07f9f01bc278818d391a0334d370e3fa99c8340a31979

The normalization file remains separately unqualified: src/bezier_region.rs SHA256 1bc26463270fbe520873708bc6909db19a7af36356367f8bbc366b92696ed720. The production qualification uses committed region bytes d2d5ddee1f0120d01a76bdc54a858794b13904ba51112bd3a442ee5691b9b46c.

If the full comparison has no new or changed failure, bind the exact staged/HEAD bytes and commit only the Boolean and offset files. Inspect actual results first; a timeout or assertion is never silently accepted as a pass. Keep the full goal active. Hyperreal's other-session working changes are untouched and unused; dependencies are immutable snapshots.

Next normalization work: migrate callers to the final regularized set. Six old failures are tied to boundary/traversal assumptions:
- selected_circle_direct_line_fillet_cut currently scans only boundary 0 and rejects a reversed retained source circle. Find the source cut across all boundaries in the original support chart, preserving its exact parameter and ownership; do not infer traversal from normalized output ordering.
- selected_circle_fillet_owns_exact_smooth_run_seam_endpoint and selected_circle_fillet_crosses_an_independently_reframed_run compare complete fragment structural equality in boundary 0. Preserve their exact seam/range ownership assertion while accepting normalized traversal reversal and multiple boundary components.
- independent_selected_circle_pair_fillets_extend_over_both_full_supports and selected_circle_pair_fillets_extend_over_both_full_supports count selected circles only on boundary 0. Check surviving exact support evidence across boundaries, including retained radial authority.
- nonrepresented_chord_line_corner_fillets_through_shared_carriers and nonrepresented_chord_parallel_corner_fillets_without_reintersection inspect adjacencies only in boundary 0. Traverse each individual loop for adjacency, aggregating witness counts across loops; preserve tangent and endpoint-only proofs. Never concatenate loops when interpreting neighbors.

These are proposed migrations, not proof that every failure is merely a caller assumption. Requalify exact semantics and membership; do not remove assertions to match output or assume every inserted fillet must survive set regularization.

The normalized major-arc blocker now passes all 16 cases in 33.25 seconds. The independently framed smooth-seam chamfer passes after d00fa18. The previously recorded five additional normalized 75-second limits and the changed old oblique-chord assertion still need investigation with the new candidate; no performance conclusion from obsolete timings.

Committed 4763c7ca97567a7fdfd14fac982019c390a248ce. Qualification: 1,193 passes, six ignored, nine unchanged assertions and nine unchanged 75-second limits; both all-target checks pass, no new/changed nonpass. The proposed commit instructions above have been completed. Post-commit bytes match the qualified candidate, and only src/bezier_region.rs remains dirty. No owned session remains active. Proceed with the normalization callers and their exact semantics, then performance work; do not repeat the completed endpoint-incidence diagnosis.
