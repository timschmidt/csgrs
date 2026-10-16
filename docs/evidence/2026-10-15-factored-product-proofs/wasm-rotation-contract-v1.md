# Real WASM rotation acceptance

`qualification-v5` passes its 13 Rust commands; its binding command stops at
the CLI-version guard. `wasm-tools-v1` installs locked CLI 0.2.126 into its own
target. `wasm-bindings-v1` builds the real module and reproduces a stale Node
assertion at the original opposite-rotation curve query `(0.5,0)`: the strict
query now returns certified boundary instead of throwing.

Owner: binding acceptance of native Hypercurve/Hyperreal facts, P4/P8/P9/P30/
P35/P37. No production change is proposed. Keep the original unit square and
13/-13-degree rotations. Opposite rotations compose to identity mathematically;
the origin-axis boundary additionally permits direct cancellation of opposite
sin/cos products. Require certified boundary at the original point under both
policies and after reuse, with independently known interior/exterior points.

`wasm-rotation-v1` queries the preserved generated module. It confirms the
original curve point is certified under strict/approximate/strict, while
`(1,0.5)` remains Uncertain/qualified Boundary/Uncertain. The latter requires
an additional unit-norm relation at a nonzero coordinate. Retain this as added
terminal-policy coverage and require the legacy strict API's typed undecided
exception there. The original mesh rotation query remains Unknown/qualified
false/Unknown, so its existing assertions stay unchanged. The probe also
records a three-rotation case; that diagnostic does not expand the admitted
completion domain or remove any hard fixture.

Only the JavaScript consumer test changes after the completed Rust matrix.
Rerun the actual Node/WASM and TypeScript script on a fresh target with
independent copied inodes. Verify every other frozen source hash equals the
Rust qualification snapshot. Preserve both failed binding runs and report the
final acceptance as an aggregate of executed checks, never by rewriting an old
failed phase as passed.
