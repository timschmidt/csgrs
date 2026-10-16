# Exact path construction failure contract

Baseline revision: `958d2fa76`. The previous goal turn made concrete progress:
strict drill composition and typed mesh Boolean adapters were committed and
verified. No source change for this extension has been made yet.

The owning layer is the CSGRS construction grammar in `src/curve/native.rs`.
P6/P16/P18/P25/P30 require construction failure to remain distinct from a
successful region or path, and P8/P9 require the selected policy and consumed
certainty to survive the shared native construction path. The native
Hypercurve carriers and their predicates remain authoritative.

The inspected APIs are `bezier_path`, `bezier_region`, `bspline_path` and their
checked/context forms. The first returns `Option`, the second converts any
checked failure to empty geometry, and the B-spline wrapper discards the
checked error. Checked region/spline methods additionally stringify native
errors, losing the existing `ValidationError::ExactCurve` classification and
cause. Reproduce these observations before editing production code.

The intended migration is explicit `Result<_, ValidationError>` at the primary
constructors, one policy-taking implementation for each construction, and
retained native errors through existing conversions. Preserve successful
native control points, curve families, exact evaluation, display-resolution
arguments, input domain and subsequent operations. Adding a context form for
the Bezier path lets closed-region construction consume its certainty without
silently forcing another policy. Existing policy arguments and fixtures must
not be weakened to make tests pass.

Acceptance must distinguish malformed controls, invalid display counts,
invalid spline degrees, undecided closure and genuine successful geometry.
Use independently derived polynomial evaluations and area/membership where
supported, then consume the native outputs in subsequent operations. Exercise
strict/approximate/strict construction with a reproduced unresolved fixture;
selecting approximate policy alone must not qualify a certified construction.
This is operation-local certainty; full input provenance remains an open stack
requirement rather than a claim established by changing a wrapper.

Known callers are root tests, `examples/readme_renders.rs`, the feature and
profile benchmark workloads, and public README signatures. Current C-family
and WASM wrappers expose no Bezier/B-spline constructor, so their consumer
checks cannot substitute for these public Rust paths. Run affected tests,
example/benchmark target checks, docs and Clippy after the final edits.

Hilbert infill still has a lossy default wrapper and a midpoint-only clipping
implementation requiring its own semantic acceptance. Legacy sampled gears,
airfoil, TrueType text and other text-only adapters also remain open; this
family's migration does not discharge them or narrow the properties goal.

No new total-query work, heap-byte or cancellation bound is claimed. Keep
external command/address-space containment and use private storage under
`/run/user/1000` or a verified copy-on-write target: `/tmp` has reached its user
quota. Preserve E13 and all inherited artifact expectations, the partial
quota-failed copy, and the mounted E13 overlay. Update only `current_work` in
the status ledger and keep the 35 pending native files separately unadmitted.
