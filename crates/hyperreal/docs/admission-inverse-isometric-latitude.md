# Inverse isometric latitude admission

Status: native implementation and the checks below completed on branch
`hyperspatial`, starting at `33dd55ccc`. This is the scalar prerequisite for E2 ellipsoidal
Mercator, not completion of that projection or the full Hyperspatial plan.

The owner is Hyperreal (P1–P7, P17–P20, P26–P27, P35–P39): only it can construct
a new exact, refinable scalar. The missing acceptance test is an independently
constructed oblate/prolate inverse with subsequent trigonometry, strict rational
enclosures, and domain rejection. The input is finite isometric latitude `psi`
and a positive polar/equatorial axis ratio `b/a`; the output is the unique
geodetic latitude in the open interval `(-pi/2, pi/2)`. Undefined ratios return
`Problem`; unresolved domain proofs keep the scalar's existing typed failure.
No float conversion, fixed iteration answer, or external production dependency
is admitted.

[PROJ's Mercator definition](https://proj.org/en/stable/operations/projections/merc.html)
(consulted 2026-10-18) supplies the forward equation. The following enclosure
algorithm and bounds are derived here, not copied from its floating-point solver.
Set `r=(b/a)^2`, `s=asinh(tan(phi))`, and `u=tanh(s)`. Define

```
F(s,r) = s - sqrt(1-r) atanh(sqrt(1-r) u),  0 < r < 1
       = s,                              r = 1
       = s + sqrt(r-1) atan(sqrt(r-1) u),  r > 1.
```

Its derivative with respect to `s` is `r/(1+(r-1)tanh(s)^2)`, between
`min(1,r)` and `max(1,r)`. Consequently `F` is onto and strictly increasing,
with one root of `F(s,r)-psi`. For fixed positive `s`, `F` increases with `r`:
differentiating its integral gives the positive integrand
`sech(v)^2/(1+(r-1)tanh(v)^2)^2`. For negative `s` the order reverses.
Rational bounds on `r` and native elementary-function enclosures thus bound
the residual without deciding whether a nonrational `r` equals one.

At a rational midpoint, divide the residual interval by the positive derivative
interval to enclose the root, intersect with the previous bracket, and retain
the sign-based bisection bound when it is certified. Adapt evaluation precision
until the residual enclosure is narrow enough relative to the derivative lower
bound and requested root width. This handles an exactly zero residual without
trying to decide equality.

For requested binary precision `p <= 0`, use the clipped root on `[-K,K]`,
`K=4-p`. Clipping is monotone, so clipping each new root interval preserves
containment. The final latitude map `g(s)=atan(sinh(s))` is 1-Lipschitz and
`pi/2-g(K) < 2 exp(-K) < 2^(1-K)`. Clipping contributes less than one eighth
of a requested ulp even for inputs beyond binary64. Refine the root bracket
to width at most `2^(p-4)` and evaluate the endpoint-stable identity
`g(s)=sign(s)(pi/2-2 atan(exp(-abs(s))))` at `p-3`, then round to `p`.
The total error is below one ulp (clipping 1/8, root midpoint 1/32,
elementary evaluation 1/8, rounding 1/2). For `p>=1`, zero is already within
one ulp of every possible latitude.

Only the requested enclosure uses a clipped root. The node stores the original
equation, and `K` grows with requested precision; no clipped midpoint replaces
the exact value. Outward dyadic rounding of each isolating bracket prevents
denominator growth across iterations while preserving its enclosure.

The private node stores `psi` and `ln(b/a)`, so its ratio is positive by
construction even after decoding a different finite logarithm. Appending its
enum variant preserves prior binary indices. Existing restrictions on malformed
non-leaf Computable graphs still apply; this is not general graph validation.
The graph retains the equation and unique branch; numeric caches are only
accelerators. Abort paths must not publish truncated approximations as facts.

The existing scalar precision representation and cooperative
abort mechanism apply. There is no new total-work or heap-byte bound. External
timeouts contain adversarial tests; huge axis ratios or extreme precision can
still exceed practical resources. Formal machine-checked proof, arbitrary
hostile graph validation and release packaging remain open.

## Executable acceptance and retained evidence

[`inverse_isometric_latitude.rs`](../tests/inverse_isometric_latitude.rs) contains
nine integration tests (eight without serde). Independent identities use
`sin(phi)=3/5`, `tan(phi)=3/4`, `asinh(tan(phi))=ln(2)` with rational oblate
eccentricity `4/5` and its prolate continuation. Tests check 512-bit absolute
errors, subsequent sine/cosine, odd symmetry, equatorial domain validation,
`1e-80` and `1e320` inputs, axis ratios `1e-40` and `1e40`, and represented
perturbations of a spherical axis ratio. The latter tests retain both signs
of the perturbation rather than treating a small difference as zero.

A separate MPFR solver bisects directly in geodetic latitude at 768 working
bits for 260 steps. It compares a 35-case grid (including ratios 1/100 and 100)
and all 484 ordered pairs of the existing 22 public scalar recipes. This is a
development oracle: no MPFR value enters a production result. The oracle's
adapter initially parsed Hyperreal's mixed-fraction Display text incorrectly;
`1 2/3` became `12/3`. The retained
[failure](../../hyperspatial/verification/2026-10-18-e2-isometric/oracle-adapter-failure.log)
and regression show why numerator, denominator and sign are now transferred
explicitly. No mathematical fixture or error threshold was relaxed.

JSON/CBOR tests check the new node tag, deterministic cold/warm serialization,
retained equation, restored evaluation and trigonometric composition. The full
private inventory now contains 61 Computable variants. Known operand signs
propagate structurally; a decoded opaque argument such as `pi-2` may still be
Unknown at zero-bit refinement. Fine refinement recovers the positive sign,
and a definite cold answer cannot change. Cancellation, recovery, concurrent
refinement, maximum coarse precision and preexisting aborts are exercised.

Hyperspatial's shared development workflow tests the inverse followed by a
validated latitude, ellipsoid, independent Cartesian normal calculation, and
ENU/NED local coordinates. Its deterministic test uses all 22 families twice;
directed/fuzz inputs also vary both families, axis ratio, height and meridian.
The README consumer executes an independent oblate identity through ECEF and
local frames using only the Hyperspatial source dependency.

The [verification directory](../../hyperspatial/verification/2026-10-18-e2-isometric/)
retains actual logs, coverage denominators, traces, sampled timings and campaign
configuration. Completed checks on stable Rust 1.99.0 include:

- `cargo test --locked -p hyperreal --lib --tests --all-features --no-fail-fast`:
  1,079 tests across 33 test groups, all passing.
- Hyperreal's five representation configurations: 63 test executions, including
  the new inverse in every cache configuration and the private serde inventory.
- Hyperspatial's four representation configurations: 188 integration-test
  executions (47 per configuration), all passing; all-target Clippy in each.
- `crates/hyperspatial/scripts/checks.sh` on stable and Rust 1.95.0: default and
  all-feature tests, benches in test invocation, docs, source consumer, native
  dependency boundary, fuzz-target compilation and Clippy `-D warnings`.
- Hyperreal all-target/all-feature Clippy `-D warnings`; native WASM library
  Clippy `-D warnings`; workspace and fuzz formatting; `zizmor --pedantic
  --strict-collection .` with no findings (offline audits).

The broad run also reproduced a preexisting empty-product panic on the untouched
base revision. Commit `456586bce` repairs it at the Rational owner; the unchanged
failing test, added zero-factor cases and native downstream results are in its
[separate record](../verification/2026-10-18-empty-products/README.md).

Coverage is measured by the extended Hyperspatial gate. Hyperspatial retains
1,073/1,091 executable lines (98.3501%), 1,468/1,567 regions (93.6822%) and
126/126 functions. The two new scalar implementation files separately cover
136/140 physical executable production lines (97.1429%), excluding the inline
unit-test module. Each line denominator has a 95% CI floor. Branch coverage is
unmeasured, and no whole-Hyperreal coverage claim follows from this subset.
Both the native-owner tests and the downstream workflow contribute executions.

## Cost and campaign scope

The native benchmark completes the independently checked inverse/ECEF/local
workflow, including scalar-recipe construction and receipts. The six-byte
fixture is `[0,7,5,1,3,0]`; it is a symbolic mathematical ellipsoid/normal
fixture, not a measurement of an ordinary WGS84 application.
Five optimized trials of 20 workflows took 0.804–1.257 seconds under concurrent
validation. A fresh-process warmup plus one workflow used 16,128 KiB peak RSS
and a 2,782,672-byte executable. The
[resource record](../../hyperspatial/benchmarks/2026-10-18-isometric-resources.json)
retains compiler, command, source/binary digests, output and timing boundaries.
These are cost observations, not allocation counts or native resource bounds.

The GMP/MPFR API benchmark now includes evaluated inverse output at 128 bits.
Both sides consume the same represented binary64 inputs `psi=0.5`, `b/a=0.6`;
the latter is the represented binary64 value, not the exact rational `3/5`.
Hyperreal retains an exact equation and returns a certified rational interval;
the independent MPFR bisection uses 256 working bits and returns a rounded
128-bit value. Those certainty contracts differ, so the timings are not an
exact-contract speed comparison. The initial short run measured about 18.7 ms
and 3.74 ms respectively, and reported an insufficient requested sample time;
its raw samples and warning are retained alongside a three-second follow-up.
That follow-up measured about 16.7 ms and 3.10 ms respectively, with ten samples
per row. The implementation did not change between these runs; Criterion's
automatic improvement labels reflect run-to-run conditions, not an optimization.
The native inverse was slower on this numeric-output probe. No speedup claim
is made, and the retained-equation/certainty contracts still differ.

The directed campaign completed 176/176 checked workflows in 22 independent
family buckets, with two generations of four candidates and 40 distinct
selected seed files. Observed dispatch counts ranged from 7,688 to 308,073;
the Context counted 23 orchestration steps and receipt depth five. Those step
counts do not measure scalar work. It retained all raw
native paths, lexicase selection scores, inputs, failures (none), compiler,
policy, timeout and source digests. The digests match the implemented native
sources; `source_revision` identifies the base checkout of the measured working
tree, not an assertion that uncommitted files were absent.

The initial ASan pass completed the three existing 60-second mutation campaigns.
For the new inverse target, its 84 seed files required 146 executions and 302
seconds solely for initialization. LibFuzzer's 60-second total-time flag did
not prevent this seed replay, and there were no mutations. That run is retained
as initialization evidence, not credited as a completed mutation campaign.
The driver now gives this target 900 seconds, preserves every seed, and retains
the 10-second per-input timeout, 2048 MiB RSS limit and six-byte input domain.
Its outer process timeout also covers compilation and initialization; it is
not a native work bound. CI executes all four targets and retains failure inputs
and logs. The extended inverse run completed 861 executions in 904 seconds,
including coverage-increasing mutations after initialization: 10,201 counters,
35,407 features, a live corpus of 330 inputs, and 839 MiB reported peak RSS.
There was no crash, timeout or sanitizer diagnostic and no new failure artifact
to minimize. These counters describe the instrumented fuzz binary, not
production line/branch coverage. The before/after bytes and full log are in
[the extended report](../../hyperspatial/verification/2026-10-18-e2-isometric/asan-mutations/report.json).

The initial runs of the existing targets also completed with mutations and no
new failure:

| Target | Seed files | Executions / elapsed | Counters / features | ASan peak RSS |
| --- | --- | --- | --- | --- |
| `forward_local` | 918 | 1,790 / 61 s | 8,316 / 23,950 | 617 MiB |
| `normal_offset` | 44 | 558 / 61 s | 9,497 / 26,144 | 663 MiB |
| `mercator` | 44 | 1,972 / 61 s | 9,850 / 28,970 | 604 MiB |

The earlier empty LeakSanitizer sandbox artifact remains explicitly preexisting
in the first target's inventory. It is not a new geometry failure. The extended
campaign used nightly Rust 1.101.0 (`8006091eb`, 2026-10-17), cargo-fuzz 0.13.2,
ASan and seed `20261018`. Hosted execution and artifact upload remain untested.

Ellipsoidal Mercator's prepared projection API is still the next E2 change.
It must retain the inverse equation's open latitude range as construction
evidence: re-proving that range numerically near a pole can be Unknown at a
fixed predicate budget even though the scalar is well-defined. The huge-input
scalar enclosure tests do not claim strict completion of every subsequent
geospatial predicate. Transverse Mercator/UTM, geodesics, E3–E4 and A1–A5 remain
active work. Formal proofs, 100% coverage, scalar work/heap budgets, hostile
graph validation, real binding consumers and release packaging remain open.
No push or publication occurred; hosted artifact upload was not run locally.
