# Resource replay after complete qualification

Keep the original 24 pre-edit children in `baseline-probes-v1/measurements`
and their source/library pins. The unchanged `resource_probe_v1.rs` remains the
only measured Rust driver. In addition to that original baseline, collect a
contemporaneous replay of the preserved old libraries alongside the final
candidate. This additional baseline must not replace or conceal the first one.

For each old/new variant and each of the three completed controls (overlap,
coincident and touching), run three timing children and one separate allocation
child at each of the cold/warm conditions defined in `cost-contract-v1.md`.
Run the same children for the candidate's full26 and merged26 cases. This is
64 additional children: 48 timing and 16 allocation rows. Together with the
original baseline, retain all 88 rows, all failures, process resource reports,
serialized artifacts and executable hashes. Run them after compilation finishes,
with no simultaneous qualification build from this task.

The three controls must retain their exact ordered artifact bytes across the
original baseline, old-library replay and final candidate. Full26 and merged26
have no successful batch baseline; report their certified completion and costs
without converting an earlier error into a speedup claim. The fixed driver's
input-construction boundary, including its initial cube construction in the
new-only cases, is retained exactly.

Check matching library feature/profile/compiler/target/flag metadata and the
source/dependency locks. Record host information, selected library sizes and
probe executable sizes. State that host scheduling is not pinned and these are
small focused samples. Compare timing rows separately from allocator-instrumented
rows; retain the unfavorable results from both baseline epochs. Describe any
observed latency or memory increase as part of the completeness repair.

Each child has the same external 120-second wall limit and inherited 8-GiB
address-space containment. Verify its executable hash before and after use.
These checks do not establish internal work, cancellation or heap-byte bounds.
The final admission still requires all affected local/downstream checks, immutable
artifact preservation and exact source pins; this measurement plan establishes
none of those results by itself.
