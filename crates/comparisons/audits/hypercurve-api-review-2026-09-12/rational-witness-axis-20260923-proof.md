

## 2026-09-23 — eliminate genuinely rational selected roots

Committed Hypersolve `d7cd3d9ffe33e6ef26c0c409dda986a88f16f7ef`. After STRICT validation against the source polynomial, a witness with a genuine rational payload now retains its monic linear defining equation. Arbitrary exact nonrational witnesses keep their algebraic equations. Root selection, half-open ownership, provenance indices and point identity are unchanged. This removes rational variables during existing quotient reduction, without another Hypercurve scheduling or normalization layer.

The new tensor regression covers zero, signed rational roots, a denominator of 1009 and nonrational coefficient gauges. It requires the selected equation to eliminate its axis exactly; the parent retains dimension two and fails. All 507 Hypersolve library tests and both all-target feature checks pass without warnings. Seven focused Hypercurve tests pass against the final source. In particular, `represented_recursive_contact_centers_handle_exact_tangency` returns to 31.22 seconds after the prior full sweep hit its 75-second limit (earlier committed baseline 31.69 seconds).

The earlier full Hypercurve candidate attempted all 1,225 tests: 1,197 passed, six ignored, 22 nonpasses. Its only new failure was that tangency timeout; the existing selected-circle fillet timeout instead reached its fragment-layout assertion. The Hypercurve source remains uncommitted pending a new full sweep against this fix. Exact source/executable/staged/HEAD bindings are in `rational-witness-axis-20260923-{qualification,staged,post-commit}`. All owned processes were terminal/reaped before commit. The full goal remains active.
