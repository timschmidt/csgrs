

## 2026-09-23 — low-degree witness endpoint ownership

Committed Hypersolve `383f3e96599c975e82023149dd821ff57e7ec404`. Low-degree witness selection now honors the root isolator's `(lower, upper]` convention; an explicit point witness or singleton owns its exact endpoint. This prevents a foreign root at the lower bound from hiding the selected upper root, and prevents accepting a lower-only root. The existing repeated/endpoint regression now covers both cases, linear roots, and explicit point ownership.

All 506 library tests and both all-target feature checks pass without warnings. The expanded endpoint regression fails on parent `1bf4c8e`. All 185 Hypersolve input files match the immutable candidate snapshot, with staged and committed hashes verified. Records: `witness-ownership-20260923-{qualification,staged,post-commit,terminal}`. All owned processes are terminal/reaped. Hyperreal's other-session inputs remain untouched and pinned. The separate Hypercurve frame candidate remains unqualified, and the full goal remains active.
