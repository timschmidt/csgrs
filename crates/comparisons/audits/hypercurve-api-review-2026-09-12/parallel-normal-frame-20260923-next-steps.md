Candidate adds a shared recursive ParallelNormal circle frame, retaining its selected source parameter and positive speed radical. All previous processes are terminal/reaped. Main Hypercurve changes only src/bezier_offset.rs. Hyperreal live worktree is excluded; all dependency symlink targets are pinned /tmp snapshots.

Next: focused1 compilation and five bounded tests; freeze all owned source inputs while runner is active.

ACTIVE runner session 19845, run-parallel-normal-frame-20260923.py, prefix parallel-normal-frame-20260923-focused1, root /tmp/hypercurve-parallel-normal-frame-2026-09-23. All owned production/test/probe source inputs frozen until this exact handle is terminal/reaped. Audit notes and inactive runners may change.

Session 19845 terminal/reaped; compilation caught a moved-value error in the new test only. Corrected with half.clone().sqrt(). Launch focused2 on the updated bound snapshot.

ACTIVE session 68688, focused2. All owned sources frozen until terminal/reaped.

Session 68688 terminal/reaped. Focused2 binary acd3e7ff0b54b946343b3fe7b0229b88814d0536914d38beeea8b17e58ee2f42. New geometric frame regression passes (0.03s); selected-circle/promoted-line normalization now passes (6.52s, formerly >300s); concentric regression passes. Analytic full-support extension reaches an Inside/Outside assertion previously hidden by its 75s timeout; companion case still >75s. All source inputs editable. Next independently examine full-support candidate memberships before altering the assertion.

ACTIVE broad1 session 8905 (1220 cases, then all-target checks); ACTIVE diagnostic trace1 session 88857 (snapshot-only straight-extension candidate/endpoint/center bounds and membership logs). All owned source inputs frozen until BOTH handles terminal/reaped. Diagnostic snapshot /tmp/hypercurve-extended-line-membership-2026-09-23 must never be copied into main.

Diagnostic session 88857 terminal/reaped with compile errors: endpoint helper returns Option, and radius to_f64_lossy returns Option. Repair snapshot-only patterns and radius formatting after broad 8905 is terminal; all source inputs still frozen while broad runs.

Broad session 8905 terminal/reaped: 1220 attempted, 1192 pass, 6 ignored, 10 assertions, 12 limits. No previously passing test regresses; two former timeouts reach assertions and target circle/line normalization improves to pass. Both all-target configurations pass. All main source hashes unchanged. Diagnostic compile patterns corrected after all owned processes were reaped; trace2 is next.

ACTIVE diagnostic session 39607, trace2, run-extended-line-membership-20260923.py. All owned source inputs frozen until this exact handle is terminal/reaped. Main remains qualified focused2/broad1 candidate.

Diagnostic session 39607 terminal/reaped. Trace2 candidate 0 has two loops: the first is the complete original clockwise lower semicircle and two closing straight sides (traversed in reverse after normalization); the second is a small loop left of x=-0.49. Origin nevertheless classifies Outside, whereas the other two candidates classify Inside. The Inside assertion is retained: investigate winding/loop role/nesting instead of weakening it. All source inputs editable. Main shared-frame improvement is qualified and ready for an incremental commit; full semantic closure remains open.

CURRENT: committed 160401ebf39ec7e38a3b83363747f304ed8ba556; Hypercurve clean. All owned processes terminal/reaped. Continue the exposed origin-membership defect with bounded diagnostics; no user permission needed for the authorized implementation.

ACTIVE diagnostic session 35142, trace3: compares native/retained origin classification, individual loops, stored/computed roles, and nesting samples. Main clean at 160401e. All owned source inputs frozen until terminal/reaped. No production edit after commit.

Session 35142 terminal/reaped. Trace3 identifies the defect below topology roles: roles are Material/Material, native line/arc lowering is Unsupported, but retained loop 1 (the tiny left bubble) reports origin Inside. Loop 0 also reports Inside, so default parity cancels them. Loop samples correctly classify outside the other loop. Next trace individual rays and fragment winding contributions; especially the left ray through shared (-1/2,0). Main clean at 160401e. All owned sources editable.

ACTIVE session 1525, trace4. Main now contains only a new minimal regression in bezier_region.rs (no production change): three quarters of the unit circle centered at (-1,0), closed by a selected analytic diagonal chord, queried at (1,0). Entire boundary x<=0 independently proves Outside. Diagnostic snapshot runs that test against committed production, then prints per-ray and per-fragment winding for the original tiny bubble. All owned source inputs frozen until terminal/reaped. Hypothesis: procedural selected parallels use traversal half-open endpoints while selected circles use spatial positive-side ownership.

Trace4 session 1525 terminal/reaped. The minimal regression fails on committed production with winding 1 instead of 0. The original bubble has winding 1 only on the left ray: circle contributions 0,0,+1 and selected chord 0 (should -1). All other nine rays give zero. Implemented one shared spatial contact contribution for procedural, retained polynomial/rational, and native Bezier paths, consuming crossing direction or retained tangent-side evidence at finite endpoints. Removed traversal-end exclusion and duplicate containment/endpoint helpers. Original Inside assertion remains; Boolean replay migrates to arbitrary normalized loop counts. Next focused1.
