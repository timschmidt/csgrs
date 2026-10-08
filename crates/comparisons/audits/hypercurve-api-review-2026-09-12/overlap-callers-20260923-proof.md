

## 2026-09-23 — overlap callers use exact correspondence, not map family

Committed Hypercurve `50f2a03e6335df1d4a9e248b60ee5f96c1477e93`. Two existing tests no longer require an analytic circle overlap to use the internal rational-map variant. They assert the expected Same orientation and continue through all existing exact obligations: nonrational cut forward/inverse mapping, authored clipping, reversed orientation and complement, chamfer order comparisons, retained point authority, strong-owner counts, shared evidence and weak-cache expiry.

Both previously failing tests pass on the exact committed production, under both policies (1.10s and 0.29s). No production code or mathematical assertions were removed. A fresh release executable and all 368 Hypercurve source inputs are bound in `overlap-callers-20260923-{focused1,qualification,staged,post-commit}`. The full suite is not repeated for two test-only caller changes. All owned processes were terminal/reaped before commit. The selected-circle fillet layout still requires diagnosis; the full goal remains active.
