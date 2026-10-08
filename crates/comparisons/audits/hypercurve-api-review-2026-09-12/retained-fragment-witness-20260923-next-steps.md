# Retained fragment witness — endpoint-incidence candidate

The previous goal turn made progress: d00fa18 was committed and three diagnostic runs isolated the normalized major-arc endpoint comparison. The full goal remains active.

The small independent radial fixture is terminal/reaped (session 57260). All eight cases pass on committed d00fa18: rational and irrational parameters, both unshifted and independently irrational translated centers. It does not reproduce the major-arc failure and is not a failing regression. Artifacts use circle-endpoint-witness-20260923-baseline1; executable SHA256 d926df664d0e44f424a89f4a088aeba04beced8abbbb663de7e6d60b7a322de9. No production source changed for that probe.

Trace 9 is terminal/reaped (session 11879), executable 14887b3bb780029f0dba3674e3c5e9bfd6d2492f6da6e54d11735c45aa207d62. Bounded scalar serialization stopped at chord_end_x: its expanded JSON serialization alone had 5,621,356 bytes, exceeding the 2 MB capture bound. No capture JSON file was written. This is evidence of expression expansion in serialization, not a measurement of DAG memory size or geometry runtime. The inactive Sympy parser was not executed and provides no oracle result. Never enable full geometry Debug tracing.

Main now additionally changes src/curve_region_boolean.rs: regularized_fragment_geometric_decision removes local_circular_curve sampling and uses the original carrier chart for the representative, derivative and source_parameter passed to winding classification. The separate derivative_follows_boundary flag disappears. No new point variant or synthetic root wrapper is introduced. This candidate has not yet been qualified. The mandatory-normalization file remains separately unqualified and unchanged.

Session 19997 is terminal/reaped: run-retained-fragment-witness-20260923.py. Clean root /tmp/hypercurve-retained-fragment-witness-2026-09-23 contains d00fa18 plus the current Boolean file and mandatory-normalization file. Prefix retained-fragment-witness-20260923-focused1. It builds a fresh all-feature release libtest and runs:
- major_retained_rational_arc_and_general_chord_share_the_fillet_kernel (90 seconds)
- selected_circle_corner_candidates_publish_normalized_single_loops (75 seconds)
- selected_circle_chamfer_crosses_one_sided_smooth_run_seam (75 seconds)
- exact_radial_chord_replays_a_retained_circle_endpoint_contact (60 seconds)

The focused1 source freeze ended after session 19997 was reaped. Audit Markdown and inactive runners are the only edit exceptions. Hyperreal's other-session work remains untouched and unused; dependency snapshots are pinned.

Source bindings:
- src/curve_region_boolean.rs: 34920c72eb031bf759f0c7e62edf6d17349769ce6441713c4559bde5a595784b
- src/bezier_region.rs: 1bc26463270fbe520873708bc6909db19a7af36356367f8bbc366b92696ed720

After reaping, inspect the actual focused results before preserving or extending the candidate. If this removes the blocker, qualify the witness simplification against its parent and continue the remaining all-boundary/traversal caller migrations and normalization performance checks. Keep all existing finite-domain and topology proofs. Do not infer correctness from this four-case run, nor mark the broader goal complete.

The prior exact endpoint diagnosis and rejected dispatch-only trials are recorded in circle-contact-replay-20260923-next-steps.md. Its assertion that main only has the normalization file is superseded by this active candidate.

## Continuation: focused2

The original sampling-chart simplification alone failed the major-arc case (Ordering, 2.50 seconds); the other three focused1 cases passed. Executable SHA256 b4f73ee59392b72b3e2588d464747924f917da6eaabb4b7c739d4769a8859971.

Main now retains a probe endpoint incidence in the private chord-pair context. A strict nonzero derivative cross proves a simple root. The chord/rational kernel consumes the known scalar factor using the existing recursive projective deflation route, solves all residual contacts, and republishes the known endpoint. No new public representation is added. This candidate is unqualified.

Session 30170 is terminal/reaped, run-retained-fragment-witness2-20260923.py. Same clean root and four focused cases as focused1. Prefix retained-fragment-witness-20260923-focused2. Source hashes are in its candidate.json and sources.json. All owned production/test/probe inputs are frozen until this exact session is terminal and reaped. Hyperreal remains the pinned snapshot; the other session's live source is not an input.

## Continuation: focused3

Focused2 executable 87d1cbaa58f600c144066a70f7d7e8f6dba766bf5f3a8f537de419768b69d0f7 clears all eight STRICT major-arc combinations. It then fails the major-arc certainty assertion in the APPROXIMATE_512 cases (11.21 seconds), while the three other focused cases pass. The same binary was rerun with bounded existing approximation-site logging in session 99513, terminal/reaped; both consumed comparisons originate at bezier_parameter.rs:2185. Records: retained-fragment-witness-20260923-approximate1*.

Main now attempts the complete strict fragment-side classifier, including retained boundary probes, before permitting a local approximate terminal. A new cubic regression checks all residual transverse contacts, rational and irrational known endpoint parameters, finite clipping, both traversals and policies, and exact endpoint identity. Redundant seed-box cloning and unrelated rustfmt churn in bezier_offset.rs were removed.

Session 33442 is terminal/reaped, run-retained-fragment-witness3-20260923.py; source hashes in retained-fragment-witness-20260923-focused3-candidate.json and sources.json. The same immutable dependency root is used. All owned source inputs are frozen until the exact handle is terminal and reaped. Five cases (the previous four plus the cubic regression) run in this candidate; all remain unqualified until results and broader checks complete.


Focused3 passes all five tests; the major-arc test completes all 16 combinations in 33.25 seconds including process startup, with no approximation consumption reported. The cubic regression completes in 0.15 seconds including startup. Session 61279 is terminal/reaped: the separate production qualification runner qualify-probe-incidence-20260923.py, prefix probe-incidence-20260923-final1. Its clean snapshot excludes normalization and changes only Boolean and offset files from d00fa18. All owned source inputs are frozen until this runner is terminal/reaped. See probe-incidence-20260923-proof.md for invariants and pending qualification.

Endpoint-incidence correction is now committed as 4763c7ca97567a7fdfd14fac982019c390a248ce. All owned processes are reaped, no source freeze remains, and final qualification has no new or changed nonpass. See probe-incidence-20260923-next-steps.md for current work; earlier ACTIVE/source-freeze paragraphs are historical.
