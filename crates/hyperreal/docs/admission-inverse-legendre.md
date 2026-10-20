# Native inverse Legendre integrals

Baseline `0171a2a48`. Hyperreal owns this extension (P1–P7, P17–P20);
Hyperspatial E2 is its downstream consumer. The baseline has forward F/E but
no native Jacobi amplitude or inverse E. The missing acceptance test is
`tests/inverse_legendre.rs`, initially expected to fail compilation.

`u.jacobi_amplitude(m)` is the unique real phi with F(phi|m)=u.
`u.inverse_elliptic_e(m)` is the unique real phi with E(phi|m)=u.
Both accept every real u and real parameter m<1, including negative m. They
are continuous, unwrapped inverses; m=1 and complex continuation are excluded.
Validate the parameter before zero, circular or composition shortcuts.
Invalid parameters return `NotANumber`; an unresolved parameter proof returns
`Exhausted`. Ordinary arithmetic and typed failures consume the resulting Real.
See [Jacobi amplitude](https://dlmf.nist.gov/22.16) for the first-kind inverse
and its quasi-periods; here m is the squared modulus used in that reference.
The real quadrature defines negative m without requiring a real modulus k.

## Representation, periods and proof

Append two boxed `[value, ln(1-m)]` nodes. Every finite serialized payload is
in-domain. The inverse's sign equals the value's sign. Matching forward and
inverse nodes with structurally identical parameter payloads cancel by their
defining equations, including after serialization. Recognized integer
multiples of a complete integral recover exact multiples of pi/2; no numerical
rounding decides such an identity. Reconstructed rational and pi values retain
their public exact carriers; other preimages retain their defining Computable
DAG. Structural equality is not a general value-equality theorem.

Let Q be the corresponding complete integral. A strict-error approximation
of u/Q at precision zero selects an integer n with |u/Q-n|<1. After subtracting
nQ, use the original complement c=1-m for even n. For odd n use complement
1/c and divide the residual by 1/sqrt(c) for F or sqrt(c) for E. The local
target is strictly between the negative and positive local complete integrals.
The inverse amplitude is n*pi/2 plus the unique local root in (-pi/2,pi/2).
Aborted reduction may never supply a phase witness.

The local solver maintains an exact rational interval for a clipped root.
An inner rational endpoint k<pi/2 comes from a certified enclosure. Its gap
to pi/2 is budgeted in the output error, so roots arbitrarily close to a pole
do not require an exact endpoint comparison. At each midpoint, enclose the
exact principal integral minus the target. The derivative is sqrt(Y) for E
or 1/sqrt(Y) for F, where Y=cos²(phi)+c*sin²(phi)>0. On this chart each
derivative is monotone in |phi|; enclosures at the smallest and largest
absolute bracket endpoints therefore bound it throughout the bracket.

Interval Newton uses these positive derivative bounds, outward dyadic
rounding and intersection with the current bracket. When the residual has
a certified sign, bisection also narrows the bracket. If the true root lies
beyond an inner endpoint, monotonicity ensures the clamped Newton interval
still contains that endpoint: apply the derivative bound only between the
midpoint and the endpoint. Residual uncertainty shrinks below 1/16 of the
current bracket width times the minimum derivative. Thus an undecided
residual sign still yields a strictly narrower Newton interval.

For requested precision p<4, root width is at most 2^(p-4), clipping error
is below 2^(p-7), and the final phase-plus-midpoint is approximated at p-3
before rounding. Their combined error is strictly below 2^p. For p>=4,
omitting the bounded local root only in the approximation, with a guarded
phase approximation, also meets that error bound. The represented value
always retains its defining equation, never a finite root estimate.

The existing rational interval helpers move to a shared native file. The
inverse-isometric coverage denominator retains those moved lines. Existing
scalar precision, allocation and abort limits apply; this extension adds no
total-work, heap-byte or wall-time bound. Check cancellation throughout
refinement, and never cache an interrupted estimate as an exact inverse.

## Acceptance

Require independent MPFR constructions, fresh inputs rather than only paired
round trips, circular/zero/quarter-period identities, negative parameters,
huge and tiny arguments, near-pole roots, all 22 scalar families, precision
order, cache modes, cancellation recovery, concurrency and JSON/CBOR replay.
Wrong-parameter and wrong-kind nodes must never cancel. Spatial consumers
must retain qualification when using the new outputs. Extend completion
benchmarks, sanitizer/representation buckets, source consumers and independent
coverage gates without weakening their earlier denominators.

This implements inverse primitives; sn/cn/dn, complex Jacobi continuation,
the Thompson/Lee mapping and the later Earth/sky stages remain required work.

Executed validation and its limitations are recorded in the
[inverse Legendre evidence bundle](../../hyperspatial/verification/2026-10-19-e2-inverse-legendre/README.md).
