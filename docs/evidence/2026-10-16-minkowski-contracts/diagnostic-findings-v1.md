# Retained failures and scope changes

The pre-edit `baseline-v1` program exits successfully as a diagnostic, but records
incorrect old results: separated boxes are convexified (volume 24 instead of 16
and the gap filled), an unused position changes the solid (volume 314/6 instead
of 8), and malformed input paired with empty is concealed as success. Point-set
hull failures also lose their native cause. These rows are not certified-success
baseline workloads merely because the diagnostic executable exits zero.

`candidate-v1` preserved the complete hollow fixture and exposed a winding
propagation error in a single final union. `union-baseline-v1` reproduces this
with 26 independently constructed boxes on the old libraries. Both native
multi-input and merged-shell forms fail; binary accumulation certifies six-volume
744 and closed-PWN topology. The implementation now composes native binary unions,
as recorded in `implementation-contract-v2.md`. The batch-arrangement defect is
still open and its full 26-box diagnostic is retained.

`candidate-v2` retains the original README input and full 16×8 sphere example.
The sphere example fails, also reproduced on old libraries in
`convexity-baseline-v1/v2`. The source sphere validates as closed PWN, while 64
convexity predicates are undecided. The first captured determinant needs the
built-in root-square identity documented in `root-square-contract-v1.md`.

`candidate-v3` advances the full example to a later hull predicate failure.
Its new scalar test helper has a denominator type mismatch and fails compilation;
the same error fails Clippy. `hull-proof-v1` fixes that helper and passes all three
root-square tests, but retains the later full-example failure and the exact four
points/expression. Their determinant needs the eighth-angle identity documented
in `eighth-angle-contract-v1.md`. Temporary capture instrumentation is absent
from the final implementation.

`candidate-v4` stops during runner preparation because its seed configuration was
missing. No validation command or target build starts; its traceback is retained.
`candidate-v5` passes all 18 existing geometry-context tests, all nine Minkowski
contracts including the full sphere, and both native Minkowski tests. A separate
numerator type mismatch in the new eighth-angle unit-test helper fails scalar
compilation and Clippy. The final source fixes that helper, strengthens the sphere
with independent membership witnesses and starts fresh broad qualification.

No failed phase, original fixture, policy, assertion requiring certified success,
or build directory was removed or overwritten. The final qualification result,
not this diagnostic narrative, determines acceptance. A successful local result
does not establish inherited provenance, complete internal resource bounds, a
published distribution, full CI or completion of the properties plan.

The broad `qualification-v1` benchmark command reused a helper rooted in the
previous evidence directory. Its target-path assertion rejected the old binary
before any write, leaving prior evidence intact. The final repair phase uses
the identical helper content in this evidence directory and the new build log.

After the real WASM consumer passed its existing tests,
`count-boundary-baseline-v1` added a missing public-boundary acceptance check.
All 60 invalid count cases failed the intended InvalidInput contract: 30 empty
operations and 10 cube operations succeeded, while 20 cube operations incorrectly
reported ResourceExhausted. The unadmitted wrapper's u32 ABI had already coerced
the JavaScript number. `wasm-count-contract-v1.md` assigns validation to this
boundary before conversion; native usize limits and the solid algorithm remain
unchanged. The final affected checks also cover 96 invalid-number cases and the
valid zero/upper endpoints under both policies.
