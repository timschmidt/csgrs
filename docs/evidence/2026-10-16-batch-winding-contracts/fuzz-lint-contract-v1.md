# Fixed-size triangle iteration in existing fuzz harnesses

The complete qualification reaches the existing Hypertri fuzz manifests under
Rust 1.98.1 and Clippy with warnings denied. The check build succeeds, but Clippy
reports `chunks_exact_to_as_chunks` at the two `assert_triangle_indices` loops
in `hyperreal_representations.rs` and `topology_invariants.rs`. Both source files
are byte-identical to `f74a7848`; the finding is reproduced on this revision,
not inferred from an earlier assessment. The failed command stays retained.

The owning layer is the fuzz harness, under P35–P37. The domain is a flat
triangle-index array whose length is already asserted to be a multiple of
three. Replace only `indices.chunks_exact(3)` with `indices.as_chunks::<3>().0`.
Both iterate exactly the complete triples; retain the existing length,
index-range and distinct-vertex assertions. Do not add a warning allowance,
remove a target, change geometry or weaken predicate policy.

Wait for the frozen broad phase to finish before editing. Then run root and
fuzz-manifest formatting checks plus the affected locked fuzz check and Clippy
commands in a fresh private target. Record the two before/after hashes. The
production implementation and all non-fuzz regression sources must remain
identical to their broad qualification snapshots. The final resource probe
continues to use the same qualified production libraries; this harness-only
repair does not justify a new performance baseline or require changing the
frozen probe. A successful target build is not a fuzzing campaign.
