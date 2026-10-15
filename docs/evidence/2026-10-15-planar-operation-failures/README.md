# Planar operation failures and matrix meaning

Apply `properties.md` and `docs/admission.md`, chiefly P1, P4, P6, P8–P9,
P17–P20, P25–P30 and P35–P39. CSGRS owns public modeling failure semantics
and admission of a homogeneous matrix into a planar affine operation.
Hypercurve owns affine topology and classification. Reuse its native kernel;
do not duplicate determinant/topology machinery in the adapter.

Reproduce errors that become empty transforms, undecided queries that become
boundary results, stringified native blockers, and ignored homogeneous or
out-of-plane matrix components. Supported planar affine matrices must preserve
homogeneous meaning, including constant nonunit/negative weights. Perspective
and out-of-plane maps require an explicit unsupported outcome; constant zero
weight is unbounded. Empty geometry must not bypass invalid argument checks.

Tests retain independent exact coordinate/area/volume expectations, manifold
validation and downstream composition. Preserve original examples and benchmark
inputs/resolutions. Stronger admissible cached proof may strengthen Unknown;
qualified matrix-admission evidence must not become Certified at an identity
kernel shortcut. Operation-local certainty is separate from source provenance.

Previous goal turn made progress in `1916037c7`, `fcb269765`, `98bda8a48`.
This slice starts from the actual worktree at `98bda8a48`. The 35 pending native
files remain byte-identical and require separate admission. Trials are immutable
and use fresh targets; all failures and raw measurements remain retained.
Internal work, heap-byte and cancellation bounds remain open; adversarial work
uses external wall/address-space containment. Full plan and release validation
remain open. No push or CI changes.

## Reproduction

`baseline-v1` retains a direct-rustc harness failure: `cfg(test)` was omitted
from the explicit check-cfg declarations. `baseline-v2` compiles the unchanged
13 regression tests and reports 3 pass/10 fail. `baseline-v3` adds four adapter
regressions and reports 3 pass/14 fail, still against the preserved library
from `98bda8a48` and the unchanged native worktree.

The scalar adapter also converts typed errors to strings and ignores Z
translation and X/Y rotation. Its non-context translate/scale ignore non-finite
third components altogether. These APIs reach the C ABI, C++, Python and Go,
so the repair must use the same matrix-admission implementation and exercise
real consumers. A plane-preserving X half-turn is a reflection, not identity;
a quarter-turn that leaves the XY plane is unsupported for this carrier.

The valid controls establish exact boundary/inside/outside classification,
empty-transform composition, and area 18/volume 36 with separate manifold
validation for reflected, translated and rotated regions with holes. No test
fixture or policy was reduced to obtain these results. Implementation and
final qualification remain pending.
