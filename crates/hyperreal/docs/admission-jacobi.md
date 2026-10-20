# Real Jacobi functions

Owner: Hyperreal, P1–P7 and P17–P20. The missing acceptance cases on
`d0aa0d38a` are a shared `sn`/`cn`/`dn` construction, structural unit and
elliptic norm relations, positive `dn`, and consumption of those outputs by
strict spatial operations. These primitives precede complex continuation and
the Thompson/Lee ellipsoidal Transverse Mercator mapping.

## Contract and representation

Admit real arguments and real parameters `m < 1`, including negative m.
The API takes the parameter, not the modulus. Validate `ln(1-m)` before
zero or other identities; invalid parameters return `NotANumber`, and an
unresolved domain proof returns `Exhausted`. The endpoint m=1 and complex
arguments remain outside this increment.

The shared, continuous amplitude `phi = am(u|m)` retains its defining
integral and unwrapped periods. Three appended private nodes store
`[phi, ln(1-m)]`, representing `sin(phi)`, `cos(phi)`, and the positive
square root of `cos(phi)^2 + exp(ln(1-m))*sin(phi)^2`. Every finite payload
has this meaning, including after deserialization. This intrinsically
validates the positive branch without trusting imported cache hints.

Evaluation first obtains an integer n with strict `|2*phi/pi - n| < 1` by
refining that exact quotient. The residual `phi - n*pi/2` lies strictly
between -pi/2 and pi/2; n modulo four selects the sine/cosine signs and
permutation. Native trigonometric and positive-root kernels then evaluate
the exact reduced expressions, with their existing error contracts.
The three outputs share the amplitude node and its refinement
cache. Exact quarter-period endpoints use existing exact scalar carriers.
No finite amplitude estimate replaces a represented value, and no foreign
runtime backend is introduced. The existing scalar precision and cancellation
contracts apply; total work, heap bytes and wall time have no new bound.

Bounded structural recognition admits direct scaled square sums for matching
amplitude and parameter identities. In particular, `sn² + cn² = 1`,
`cn² + (1-m) sn² = dn²`, `dn² + m sn² = 1`, and
`dn² - m cn² = 1-m`. A mismatch or recognition-budget decline retains
the exact original expression. Individual squares keep their stable
evaluation, rather than being rewritten as cancellation-prone differences.
This is not a general trigonometric theorem prover.
Square-term recognition has depth eight per operand. Coefficient equality
uses the existing bounded rational-polynomial recognizer; neither budget
adds a bound on the entire scalar operation or structural DAG comparisons.

Recovery of `exp(ln(c))` from native log reductions is optional and bounded:
64 visits, binary power shifts at most four, integer powers at most 16,
and 4096 retained rational coefficient bits. A single bounded power may
temporarily use up to 65536 coefficient bits. Generic log arguments require
an existing positive-domain proof or a cached residual enclosure; inspection
never requests digits. After import, warming may supply that proof. The
unit norm and dn positivity need no such recovery. Unknown is never false,
and all outputs retain their exact expressions when recognition declines.

The huge-period acceptance test also reproduced an existing generic range
reduction defect: dividing numerator and divisor approximations at precision
-4 loses an unbounded number of periods as the numerator grows. The shared
near-integer helper now refines their exact quotient to precision zero.
Its positive shared divisors are pi, pi/2 and ln(2); existing residual
corrections remain in place. The direct 1100-bit signed quotient regression
fails on the previous implementation. Optional exponential range reduction
declines when its binary scale cannot fit i32, preserving the existing lazy
exact exponential instead of failing construction. This is a correctness change, with
no performance claim about ordinary argument construction.
Existing serialized node indices remain unchanged; the three Jacobi variants
are appended. The corrected quotient may select a different equivalent
reduced expression when constructing ordinary exponentials/trigonometric
values. This does not promise byte equality with expressions constructed by
an older revision. Imported expressions retain their mathematical meaning.

The definitions, positive branch and relations follow
[DLMF amplitude](https://dlmf.nist.gov/22.16),
[elementary identities](https://dlmf.nist.gov/22.6), and
[real periods](https://dlmf.nist.gov/22.4).
For negative parameters, the same real definitions follow directly from the
strictly positive integrand `1/sqrt(cos²(phi)+(1-m)sin²(phi))`.

## Required evidence

Fresh MPFR integral-series inverses and independently sampled amplitudes,
exact endpoints, negative parameters, huge periods, tiny offsets, all 22
scalar families, mismatched relations, serialization, cancellation recovery,
precision order and concurrency. Exercise strict spatial consumption and
qualification retention. Extend the existing independent coverage gates,
representation campaigns, sanitizer workloads, benchmarks and consumers.
Record executed results separately; this contract is not validation evidence.

Executed commands, diagnostic failures, completed checks and remaining limits
are recorded in the [Jacobi verification record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperspatial/verification/2026-10-19-e2-jacobi/README.md).

Complex Jacobi continuation, ellipsoidal Transverse Mercator/UTM, geodesics,
and the remaining E1–E4/A1–A5 requirements stay open.

## Shared epsilon output

`Real::jacobi_sncndn_epsilon` extends the same admitted real domain with
E(am(u|m)|m). It owns the shared amplitude and retains the existing separate
APIs. The [epsilon admission](../../hyperlattice/docs/admission-jacobi-epsilon.md)
records the failing sanitizer workload, serialized-equivalence requirement,
cancellation and exact downstream relations. No precision or campaign budget
is relaxed. The [epsilon verification record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperspatial/verification/2026-10-19-e2-jacobi-epsilon/README.md)
records passing stable/MSRV tests, independent coverage floors, all four cache
configurations, exact serialized equivalence and actual sanitizer mutations.
