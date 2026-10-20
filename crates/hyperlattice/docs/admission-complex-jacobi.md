# Complex Jacobi continuation

Baseline: `5261beba6`. Owner: Hyperlattice for complex assembly over native
Hyperreal scalars; Hyperreal owns any additional scalar relation proof.
Relevant properties: P1–P7 and P17–P20. Missing acceptance: a complex
`jacobi_sncndn` API, finite quarter-period continuation, explicit pole failure,
and reuse of its exact relations through ordinary complex arithmetic and
strict spatial consumers.

## Domain and construction

Accept `z = x + i*y` with finite Real components and an admitted real parameter
`m < 1`, including negative m. The API returns the ordered `(sn, cn, dn)` tuple.
Complex parameters and m=1 remain outside this increment. Validate the
parameter before zero/real-axis shortcuts. Known invalid parameters return
`NotANumber`; an unresolved parameter branch returns `Exhausted`. Known poles
return `DivideByZero`; an unresolved denominator returns `UnknownZero`.
The existing scalar precision and cancellation contracts apply. No new native
total-work, retained-heap or wall-time bound is claimed.

Use the existing real triplet at x. For 0<m<1, use the complementary triplet
at y and form the rectangular addition formulas with one shared inverse of
`Q = C² + m*s²*S²`. Never divide separately by C: C=0 and s≠0 is finite.
For m<0, set h=1-m and obtain the complementary real functions by the
reciprocal-modulus transformation at `sqrt(h)*y` with parameter `1/h`.
Its stable denominator is `Q = C² + (-m/h)*c²*S²`, a nonnegative sum.
The real-axis path preserves the native real triplet; m=0 uses the exact
circular/hyperbolic continuation. Unlike the real-axis dn, complex dn is
not asserted positive and can be zero at finite points.

These constructions follow [DLMF addition formulas](https://dlmf.nist.gov/22.8),
[imaginary transformation](https://dlmf.nist.gov/22.6.iv), and
[reciprocal modulus](https://dlmf.nist.gov/22.17). The implementation uses
parameter m=k² rather than DLMF's modulus k.

Keep the component expressions and their native amplitude/domain identities.
Any additional rational-function relation recognition is an optional bounded
proof at the scalar owner, after the existing proofs. It must preserve original
expressions when it declines, validate every inverse denominator, avoid new
numerical refinement, and never turn a budget decline into a negative fact.
Do not replace individual stable squares with cancellation-prone evaluation.

## Required acceptance

Use an independent high-precision development oracle, not a second spelling
of the rectangular construction. Check real, imaginary and general arguments;
positive, negative and zero parameters; both kinds of finite quarter-period
points; poles, near-pole offsets, huge periods and tiny arguments. Require
exact certificates for unit/elliptic relations and reject mismatched inputs.
Exercise all 22 scalar families, cold/warm and primitive-cache configurations,
precision order, cancellation/recovery, concurrent readers, and component
serialization. Feed outputs into further complex arithmetic and strict
spatial operations while retaining input qualification.

Component serialization omits scalar caches. Replay imported component values
and inverse domains independently before querying the exact norm relations;
structural subtraction of equal carriers alone would bypass that replay.
The existing wire format is unchanged. The
[spatial testing contract](../../hyperspatial/docs/admission/e2-complex-jacobi.md)
records the full downstream matrix, independent ODE oracle and added gates.

Extend independent coverage floors, API/benchmark evidence, directed case
selection, sanitizer campaigns and independent source consumers. Record actual
commands and completed results separately. This admission is a contract, not
evidence that the implementation satisfies it. Thompson/Lee mapping, its
certified inverse and the remaining Earth/sky plan stay open.

Executed acceptance and remaining limits are recorded in the
[complex Jacobi verification](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperspatial/verification/2026-10-19-e2-complex-jacobi/README.md),
including independent enclosures, strict spatial compositions, both compilers,
all cache configurations, coverage and the completed sanitizer campaign.
