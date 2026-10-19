# Principal complex inverse hyperbolic tangent

Baseline `7042b1ef2`, branch `hyperspatial`, after the epsilon increment.
Hyperlattice owns complex assembly, with existing Hyperreal primitives.
P1–P7 and P17–P20 apply. The missing API is Complex::atanh for the principal
analytic branch on the complex plane excluding real cuts (-infinity,-1] and
[1,infinity). The real interval (-1,1) reuses Real::atanh; poles at +/-1 retain
its Infinity error and other real cut inputs return NotANumber. An unresolved
cut-membership decision returns Exhausted. There is no signed-zero convention
or silently selected bank of the cut. This is a prerequisite for reconstructing
the isometric coordinate from a later native Thompson continuation.

The [DLMF principal-value/logarithm definitions](https://dlmf.nist.gov/4.37)
give, for z=x+i*y off the cuts:

    Re atanh(z) = log(((1+x)^2+y^2)/((1-x)^2+y^2))/4
    Im atanh(z) = atan2(2*y, (1-x)*(1+x)-y^2)/2.

When the sign of x is already available, use a nonnegative log1p argument:
4*x/((1-x)^2+y^2) for x>=0, or -4*x/((1+x)^2+y^2) and negate the result
for x<0. This avoids constructing a ratio near one or a log1p argument near
minus one. Keep the exact logarithmic quotient fallback when x's sign is
unknown; that is not a domain failure. The imaginary axis uses i*atan(y).
Prove cut exclusion from either nonzero y or -1<x<1; do not silently classify
an unknown zero or infer domain membership from display values.

Acceptance starts with a missing-method compile failure. Add an independent
MPFR interval power-series oracle with its whole-tail bound, reciprocal
continuation for large nonreal inputs and explicit coverage of its restricted
comparison domain. Exercise both axes, exact endpoints and cuts, all scalar
families, parity/conjugation, huge and tiny values, near-pole offsets on both
sides, genuinely unresolved domains, precision order, cancellation, concurrency
and serialized component replay. Feed outputs into subsequent native arithmetic
and spatial consumers with qualification preserved. Keep existing complex,
epsilon and scalar tests. Add independently gated production coverage, completed
benchmark rows, directed cases, sanitizer evidence and a source example.

No new scalar node or wire tag is proposed. This API alone does not implement
Thompson/Lee inversion or establish any physical observational accuracy. Native
scalar precision/cancellation behavior applies; no total-work or heap bound is
claimed. Record executed evidence separately before retaining the increment.

The missing-method failure was reproduced on `7042b1ef2` before integration.
The candidate's 732 frozen runtime, test, README and tooling files match
the integrated tree byte for byte. Complete stable/MSRV checks, native suites,
all four cache configurations, directed/ASan campaigns, twelve independent
coverage floors and all thirteen root integration checks pass. The
[verification record](../../hyperspatial/verification/2026-10-19-e2-complex-atanh/README.md)
records commands, failures, hashes, benchmark scope and remaining limits.
