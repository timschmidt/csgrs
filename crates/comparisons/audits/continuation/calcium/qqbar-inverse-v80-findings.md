# Checkpoint 80 — exact inverse-angle recognition

The pinned current FLINT misses an exact rational inverse angle: for the public
algebraic input `x = tan(pi/3360)`, `qqbar_atan_pi` returns zero, although the
principal result is exactly `1/3360`. Its approximate search proposes `1/3359`,
correctly rejects that fraction, then stops. This is a documented recognition
completeness failure, **not an unsound accepted equality**. No production change
is retained from this pass; the seven continuation retentions remain unchanged.

## Source coverage and architecture

Read independently at Calcium `8dbb16fc4fe92eaf3ebbc7478d629e994d39f944`
and FLINT `e269d38061d7a42070ddcffe6eb114466ed4aa7e`.
Paths below are relative to `qqbar/` and `src/qqbar/` respectively. Each stated
file was read from its first through last line, not inferred from similarity.

| File | Calcium lines | FLINT lines | New credit |
| --- | ---: | ---: | --- |
| `asin_pi.c` | 152 | 151 | Both |
| `acos_pi.c` | 41 | 42 | Both |
| `atan_pi.c` | 237 | 239 | Both |
| `acot_pi.c` | 120 | 119 | Both |
| `log_pi_i.c` | 33 | 33 | Neither: previously covered |
| `test/t-asin_pi.c` | 64 | 56 | Both |
| `test/t-acos_pi.c` | 64 | 56 | Both |
| `test/t-atan_pi.c` | 69 | 61 | Both |
| `test/t-acot_pi.c` | 68 | 60 | Both |
| `test/t-log_pi_i.c` | 63 | 55 | Both |
| `impl.h` | — | 21 | FLINT declarations only |

Nineteen newly completed files / 1,738 new lines: archived 878, current 860.
The twenty-file queue contained 1,783 lines, but 66 were already covered; the
additional private header adds 21. Both manual sections were reread, with no
duplicate credit. Cumulative continuation coverage is 1,466 complete files,
20 partial files, 187,355 uniquely read lines. This is not whole-ecosystem
completion. Exact ranges and per-file snapshot hashes are bound through
[read records](qqbar-inverse-read-records-v80.json),
[origin](qqbar-inverse-origin-v80.json) and the inventory.

The donor's successful fast paths use rational/quadratic polynomial tables and
the selected real embedding. Generic asin/atan propose a rational angle from a
64-bit approximation, filter by degree, then reconstruct and compare exactly.
`best_rational_fast` is a bounded Stern–Brocot mediant walk with fixed tolerance,
not an accelerated continued-fraction procedure. Exact final validation is
essential; exhausting this one-candidate search is not a proof of nonexistence.

`acos` composes the asin result with an exactly reduced complement. The acot
branch is `(-pi/2, pi/2]`, with zero mapped to `+pi/2` and negative inputs to
negative angles; do not substitute the other common acot convention. Logarithm
phase is `(-1,1]` in pi units, retaining `+1` at -1. Only root-of-unity recognition
explicitly permits null fraction outputs; these inverse wrappers do not. No
failed output, null-pointer experiment or invalid internal enclosure is used.

The complete upstream test sources check small randomized round trips, reduced
fractions and principal ranges. They were read, not newly executed. The archived
tests use a larger default iteration multiplier. Neither selected test set
contains the negative-recognition/cache-history corpus added here.

## Independent native qualification

The deterministic collector emits 1,081 cases in two enclosure-cache states:
2,162 rows plus terminal. Full minimal polynomials and both input components are
checked using exact rational/biquadratic/quadratic arithmetic, Galois products,
cyclotomic polynomials and independently isolated cubic roots. Enclosure widths
must be at most `2^-96`; overlap alone is insufficient.

- 2,114 valid input rows: 2,042 recognized, 72 not recognized; 48 poles.
- 25,216 mathematical checks pass; fifteen deliberate corruptions are rejected.
- All sixteen near-match rows propose the expected angle and overlap at the
  approximate stage, but exact recognition rejects the `+/-2^-100` perturbation.
  This establishes rejection of that candidate, not a general transcendence proof.
- Initial versus explicitly 256-bit cached input states agree on recognition.
- Native and Memcheck output are byte-identical: 593,765 bytes. Memcheck reports
  zero errors and no live blocks; 1,492,610 allocations/frees and 74,269,892 total
  requested bytes cover the entire collector, not per-inverse cost or peak memory.

The separate denominator-3360 probe is capped at 1 GiB address space, 30 CPU
seconds and 45 wall seconds. It terminates successfully without approaching
word-size denominators. The raw record contains all 385 coefficients of the
degree-384 polynomial, exact dyadic endpoints, recognition zero and proposal
`1/3359` with no overlap.

Independent proof:

1. Construct `F(X) = Im((1+iX)^3360)` with exact binomial coefficients. The input
   polynomial divides F with zero remainder and an integer quotient.
2. Exact endpoint signs bracket a root within `(1/1200, 1/1000)`, with zero
   imaginary component. Both normalized-rational and homogeneous-Horner
   evaluators verify the bracket; the latter also passes 420 differential cases.
3. From `3 < pi < 22/7`, `sin(t) <= t`, `cos(t) >= 1-t^2/2` and monotonicity,
   this interval contains exactly `tan(pi/3360)` among the real roots of F.
4. Therefore the principal inverse is `1/3360`. The fixed `1e-7` search tolerance
   accepts `1/3359` first; exact rational inequalities separate this from the
   preceding `1/3358` candidate. The public routine does not seek another fraction.

Eight corruptions are rejected by the primary proof checker. The second replay
preserves the same mathematical conclusion without using FLINT as its oracle.
Only current FLINT is executed; the matching archived algorithm is source evidence,
not a claim that the archived binary was run. See the
[raw public probe](results/qqbar-inverse-counterexample-native-v80.stdout),
[primary proof](results/qqbar-inverse-counterexample-check-confirmed-v80.stdout) and
[independent replay](results/qqbar-inverse-counterexample-replay-v80.stdout).

## Hyper comparison and transfer decision

The unchanged 957-file live source map is rechecked. Hyper keeps compact rational
angle forms through sin_pi/tan_pi and inverse special forms. Equivalent explicit
radicals can lose those proof routes. The current read covers
`hyperreal/src/real/arithmetic/elementary_functions.rs` 2835–3240, with a later
3030–3245 reread. There is no new production proof helper or representation change.

The small audit-only Hyper executable produces 1,848 rows plus terminal in each
of debug/release, byte-identical including full certificates (363,958 bytes).
Independent membership, target-fraction and semantic checks report:

| Outcome | Rows per profile |
| --- | ---: |
| Equal | 584 |
| NotEqual | 896 |
| Unknown | 312 |
| Pole | 48 |
| Explicit rational domain error | 8 |

No observed equality or inequality contradicts the independent oracle. All
ordinary `+1/1024` targets are proven unequal. Eight tiny perturbations remain
Unknown at -64 and become NotEqual at -256. The eight signed denominator-3360
identity rows are structurally Equal, including a `2^1025` exact period offset;
the eight corresponding unequal controls are NotEqual.

Remaining Unknowns include explicit twelfth-/twenty-fourth-turn radicals, golden
quadratic inputs and a **derived** acot comparison. Acot is implemented here as
atan(1/x), with the donor's explicit zero branch; this is not a Hyper acot API.
The derived reciprocal may itself lose angle structure. Cubic Hyper inputs are
cos_pi expressions, not imported generic algebraic polynomials. Logarithm is not
included in the Hyper probe. Repeated floors, periods and forms are not independent
defects. Eleven corrupted streams/records are rejected.

Formatting, warning-denied all-target/all-feature Clippy, locked offline metadata
and both execution profiles pass. The full resolved graph has 21 packages/nodes
and exactly one live Hyper dependency, Hyperreal. This is public capability
qualification, not a fresh whole-stack regression, WASM runtime or benchmark.

Keep Hyper's retained proof certificates and explicit Unknown fallback. Donor
exact reconstruction is a useful acceptance safeguard, but a failed heuristic
search must not become negative proof. Additional cheap exact inverse tables may
help completeness; this pass does not establish their cost or justify importing
the bounded mediant search. The rejected checkpoint-79 field evaluator remains
isolated. No new performance, peak-memory, linked-size or code-size win is claimed.

## Evidence, storage and scope

Existing native libraries and the shared Rust target are reused. Only two small
C executables are added to `/tmp/calcium-qqbar-inverse-v80.uoobbg` (36,720 bytes).
Unique debug/release audit executables in the shared target add 5,596,264 bytes;
four bound executables total 5,632,984 bytes. These are storage accounting, not
representative product-size measurements. Post-build available `/tmp` space is
13,910,597,632 bytes; no broad source copy, library rebuild or cleanup occurs.

Two sandboxed checker captures exited zero with empty required output. They are
unusable, not evidence of success, and remain preserved. Approved confirmations
provide the actual result records. The first combined summary also counted the
14 fields of its source-summary object instead of the 957 verified source files.
The final wrapper corrects only that reporting field and preserves the original
script/capture; source checks, mathematical results and raw observations are
unchanged. Combined original evidence passes 2026-09-12T01:53:04.898Z; final
bookkeeping-corrected evidence and sealing are separate required gates.

Reproduce from this directory:

```sh
node verify-qqbar-inverse-v80.mjs
```

The verifier checks frozen source/artifact hashes, captured commands, outcomes,
coverage additions, both independent oracles and complete Hyper certificates.
This checkpoint does not close qqbar, Calcium/FLINT or the ecosystem inventory.
Remaining inverse/root/recognition support, formal/symbolic/historical references,
unresolved worthwhile transfers and inventory reconciliation remain required.
No donor/live edit, external report, new retention, commit or push.
