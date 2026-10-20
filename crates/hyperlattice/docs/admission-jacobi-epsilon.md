# Jacobi epsilon for rectification

Baseline `5bd072da7`, branch `hyperspatial`. This increment builds on the native
complex Jacobi continuation. Hyperreal owns
the real scalar composition; Hyperlattice owns complex assembly. Properties
P1–P7, P17–P20 and P35–P39 apply. The missing acceptance is an epsilon API
whose argument is the Jacobi argument, rather than Legendre's amplitude, and
whose real defining composition and complex quasi-periods survive subsequent
native operations. Hyperspatial consumes the values with retained input
qualification. Thompson/Lee projection and its selected inverses remain open.

The domain is finite real/complex arguments and admitted real parameter m<1,
including negative m. Epsilon is the meromorphic primitive of dn² with value
zero at the origin. Its continuation has poles, but no path-dependent branch
value: the residues of dn² vanish. Real inputs use E(am(u|m)|m), with the
existing continuous unwrapped amplitude. There is no extra scalar node or wire
tag. Validate m before zero shortcuts; preserve NotANumber, Exhausted,
DivideByZero and UnknownZero at their existing domain boundaries. Existing
precision and cancellation contracts apply; no new total-work or heap bound
is claimed.

## Rectangular construction

For 0<m<1 let (s,c,d) be the real triplet at x|m and (S,C,D) the triplet at
y|h, h=1−m. Set Q=C²+m*s²*S². The
[epsilon addition formula](https://dlmf.nist.gov/22.16.E27) gives

    Re epsilon(x+i*y|m) = epsilon(x|m) + m*s*c*d*S²/Q
    Im epsilon(x+i*y|m) = y−epsilon(y|h) + d²*S*C*D/Q.

In the imaginary transformation, epsilon(i*y|m)=i*(y−epsilon(y|h)+S*D/C).
Substitution in the addition formula cancels the apparent C divisor using
Q−m*s²=d²*C². The final form remains finite at C=0 when s is nonzero.
Only Q requires a nonzero certificate.

For m<0 use r=sqrt(h), parameter 1/h, argument r*y and its real triplet
(ss,cc,dd). Complementary functions are (ss/r,dd,cc), and the nonnegative
denominator is cc²+(−m/h)*c²*ss². Integration of cn² gives
epsilon(y|h)=r*epsilon(r*y|1/h)+(1−h)*y, so the vertical term is
r*(r*y−epsilon(r*y|1/h)). These transformations use the same complementary
data and denominator as the complex triplet. For m=0, epsilon(z|0)=z.
Definitions and quasi-periods follow [DLMF 22.16](https://dlmf.nist.gov/22.16).

## Acceptance

Extend the independent MPFR Taylor-ODE oracle with epsilon'=dn² and
epsilon(0)=0. Coefficient n is a convolution of n dn coefficients divided
by n, so its magnitude is at most A^(n−1)<=A^n for A=max(1,|m|). The existing
q^(N+1)/(1−q) tail therefore also bounds epsilon in the local oracle domain.
The oracle does not evaluate rectangular epsilon or E(am).

Check both parameter signs, zero, all scalar representation families, real
and imaginary axes, parity, conjugation, the quasi-addition law, real and
imaginary quasi-periods, huge translations, tiny offsets, finite quarter
periods and true poles. Retain the real inverse-E/forward-F composition as
an exact certificate. Exercise serialized components, precision order,
cancellation/recovery and concurrent readers. Downstream geodetic consumers
must retain input qualification and exact arithmetic through composition.

Extend the independently gated real-Jacobi, complex-Jacobi and forward-Legendre
production coverage subsets, retaining every existing file and floor. Add
completed benchmark rows, source consumer examples, directed selection and
ASan fixtures. Record actual final
source hashes and commands separately. A development mpmath quadrature probe
checks signs at 25 points; it is exploratory evidence, not certified acceptance
or production code.

The initial epsilon quarter-period test reproduces a missing exact scalar
certificate: E(am(K(m)|m)|m) has the correct numerical value but cannot certify
equality with complete E(m). The forward Legendre constructor owns this
P2/P18/P19 gap, for both F and E at integer multiples of pi/2. The integrand is
even and pi-periodic, and each quarter integral is K or E, hence the complete
value times that integer is exact. Retain this relation in the common scalar
constructor after ordinary inverse cancellation; preserve domain validation,
cancellation and unwrapped multiples. Keep near-quarter offsets distinct.
Newly constructed expressions may now serialize as the equivalent complete
formula; existing wire tags and the meaning of previously serialized nodes
remain unchanged. This is a representation change, not a claim of bitwise
identity with earlier constructed expressions.

## Shared real amplitudes after the initial sanitizer run

The initial full 215-file seed replay completes in 1912 seconds with both
INITED and DONE at run 310, so no mutation occurs.
Its retained slow input is `[3,10,127,39,1,0]`, already in the full deterministic
family/regime matrix. A fixed-input replay completes in 12.537 seconds; this
replay does not enforce the mutation campaign's per-input timeout. The original
binary and logs remain diagnostic evidence. Do not reduce the seed corpus,
per-input limit, output materialization or spatial composition assertions.

Hyperreal owns reuse of the continuous real amplitude (P2/P4/P5/P18/P20/P36).
The existing complex epsilon construction builds the amplitude once for each
triplet and again for each real epsilon. Add a shared native
`jacobi_sncndn_epsilon` evaluation with the same four mathematical outputs and
parameter/zero behavior. Construct every output from the same owned amplitude
and parameter nodes. Preserve the separate existing APIs. Verify serialized
output equivalence with separate calls, low/high/low refinement and downstream
exact identities before measuring the complete original workload. Complex
continuation must request epsilon only when needed, and reuse the same
complementary/reciprocal amplitude. No proof, precision or campaign budget is
relaxed; validate an actual mutation campaign after the change.

The fuzz driver now requires a completed run count greater than INITED, in
addition to a successful process exit. The actual seed-only epsilon log is a
deterministic regression for this acceptance gate; malformed, partial and
fixed-input logs must also fail. This does not change any campaign budget.

## Final verification

The [verification record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperspatial/verification/2026-10-19-e2-jacobi-epsilon/README.md)
retains all commands, process exits and frozen source hashes. Stable/MSRV,
native/downstream suites, all four primitive-cache configurations, independent
coverage gates, source consumers, WASM compilation and benchmarks pass.
The final ASan campaigns advance from INITED 330 to DONE 519 for epsilon and
322 to 680 for complex Jacobi under the unchanged limits. No new crash or
timeout is reported; both campaigns retain new slow inputs, including an
eleven-second complex case despite the configured ten-second input limit.
Native work/heap/wall-time bounds and the Thompson/Lee projection remain open.
