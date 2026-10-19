# Jacobi relations in rational compositions

Owner: Hyperreal. Baseline `5261beba6`, with the pending native Hyperlattice
complex continuation. Properties P1–P7 and P17–P20. The consumer test
`complex_jacobi_relations_survive_ordinary_arithmetic` reproduces a missing
exact certificate: `sn(z|m)²+cn(z|m)²` has no rational normal form at
z=1/4+i/8, m=1/2. The quarter-period and invalid-parameter tests pass.

The symbolic negative-parameter case also reproduced loss of an already
proved logarithm domain: `logarithm_argument_recovery_is_bounded_and_domain_checked`
recovers a positive argument from a cached residual enclosure, but its returned
expression has no positive fact for a subsequent inverse. Retain that exact
positive-domain certificate at the existing logarithm replay owner (P4/P19),
without requesting new digits or assuming an imported log tag is valid.

Add an optional proof after the existing polynomial, quotient and built-in
root proofs. Preserve those attempts and their limits. The new attempt admits
native Jacobi cn²=1−sn² and dn²=1−(1−h)sn², where h=exp(log(1−m)),
using matching amplitude/parameter identities. A query may instead use cn
as its basis, with sn²=1−cn² and dn²=h+(1−h)cn²; a positive cached sign
of log(h) selects this basis for the stable negative-parameter denominator.
Either basis is valid for every admitted parameter. Choose once per
structurally equal log(h) in the query, so differently warmed copies cannot
introduce opposite rewrite rules. Replay a logarithm only when its positive
domain evidence is available. Native square roots contribute r²=radicand
only with a nonnegative radicand certificate. Matching positive exponentials
may contribute b=2^k/a only when the existing exact log-expression recognizer
proves log(a)+log(b)=k log(2). No new refinement is permitted by this proof.

Relations form a triangular extension: dependencies enter the atom table
before a dependent atom. A rational relation a^p=N/H (p=1 or 2) has a proved
nonzero H. Substitution multiplies both the numerator and denominator by
the same power of H before replacing each p-tuple of a factors. This
preserves the quotient, and avoids assuming that a formal nonzero polynomial
is nonzero at the actual input. All original inverse operands remain
domain-checked. Inverting a nonzero admitted root may use 1/r=rH/N:
N≠0 follows from r≠0 and H≠0. Exact polynomial division cancels factors
only from admitted denominators. The additional divider maintains the same
graded monomial order as the original divider; it has independent product
and remainder comparisons against that implementation.

The pair `[0,10,2,2,0,0]` (rational and sqrt(2) families) reproduced a
degree-bound decline in the elliptic norm. Its quadratic inverse coefficients
were carried into different common denominators. When an admitted root has
r²=q with rational q, a nonzero rational norm a²−b²q permits
1/(a+b*r)=(a−b*r)/(a²−b²q). Check that norm and every coefficient bound
before using this proof. A vanishing norm declines even if a+b*r itself
is nonzero, as at 1+sqrt(1); the ordinary inverse path remains available.
This compaction keeps the existing degree-12 limit and all original inverse
guards. The minimized scalar quadratic-parameter regression accompanies the
full 484-pair corpus.

To keep parameter coefficients compact, a bounded prepass can retain a
quotient already present in a native log residual as one opaque atom. Before
hiding its arithmetic, check every visible inverse for a known nonzero sign.
Later copies must match structurally. Products of opaque scalar coefficients
can likewise stay opaque only when they hide no visible inverse or Jacobi
component. Neither compression assumes algebraic independence or numerical
values for its atoms.

A formal assignment sn=0 and other atoms=1 proposes a rational candidate;
it is never a certificate. Accept that candidate only after the full residual
N−candidate*D reduces to exact zero under admitted relations. This zero-only
proof can omit normalization of D, which already has a nonzero witness, and
remove a common monomial as a sufficient proof: if the remaining factor is
zero, its product is zero even if the omitted monomial vanishes. No rescaled
nonzero result or inverse certificate may escape the zero-only proof. A full
quotient reduction and a zero-numerator attempt remain bounded fallbacks.

The separate attempt allows degree 12, needed by products involving a
symbolic parameter and two Jacobi triplets. It otherwise retains 512 node
visits, depth 64, 32 atoms, 64 monomials, 4096 coefficient bits, and 8192
work charges. The parameter prepass, derived arithmetic parsing, polynomial
substitutions, power construction and scans share these budgets. There are
at most 32 basis choices and 32 compressed parameter identities. Log-residual
scanning and scalar-product classification have depth 8. Existing logarithm
replay has its separate 64-visit limit; each exponential-pair comparison has
its separate 128-visit, 16-term and 1024-bit log-coefficient limits. These
helper limits are additional to the outer work charges, not included in a
claim of 8192 total CPU operations. Exponential binary shifts must have
absolute value below 4096 and their resulting rational must fit the ordinary
coefficient check.

Temporary owned atoms live only for the query; memoization and prepass pointer
keys remain restricted to the borrowed authoritative input graph. No pointer
to a temporary replay expression is retained. These bounds do not bound the
existing structural matcher, native expression construction, total heap bytes
or the entire caller operation. A decline leaves the authoritative expressions
intact and supplies no negative fact. Valid positive-domain facts recovered
from logarithms may survive a larger proof's decline; they certify their own
arguments, not the incomplete outer claim.

Downstream admission reproduced another missing composition certificate:
after proving a complex norm n=1, the equatorial expression
(n+2)²/hypot(n+2,0)+n has no rational certificate for 4. The minimized scalar
test `proved_norm_remains_consumable_as_an_ellipsoid_axis` and the spatial
cases `[0,0,2,2,0,0]` and `[0,7,127,39,0,0]` fail on the pending implementation.
Retain a completed rational proof in the existing exact quadratic-tower cache
as a rational tower, alongside its sign. Later native-relation parsing can
consume that certificate as a checked constant without re-expanding its DAG.
Publish only after the whole query and cancellation check succeed. This reuses
the existing synchronized, per-node cache, adds no cache field or wire data,
and keeps the authoritative expression unchanged. Its lifetime is the owning
node's lifetime; numeric approximation entries cannot evict this exact proof.
The retained rational has at most 4096 coefficient bits; total retained heap
bytes across live nodes remain unbounded. Serde omits this cache like other
scalar caches; imported expressions replay their domains before reuse.

The full ECEF case also exposes a separate endpoint gap: a proved zero
imaginary norm is consumed as latitude, but generic sin/cos only recognize
rational-leaf zero. The regression
`proved_zero_is_consumed_by_trigonometric_endpoints_without_refinement`
fails despite a retained exact-zero sign. Consume that already-proved zero
at the generic sine/cosine owner, retaining its abort signal. This adds no
refinement, treats Unknown as inconclusive, and does not change the narrow
`Real::definitely_zero` structural-carrier query.

The full pair `[7,19,2,2,0,0]` (pi²*e and sin(pi/5)) then reproduces a
stack overflow in generic logarithm construction. An inexact magnitude hint
is used as a range certificate; after binary reduction the recursive call
selects the same stale hint with a zero shift. The Hyperreal logarithm owner
must use certified magnitude information for these recursive rewrites
(P3/P4/P5/P20). The domain is every positive logarithm operand, including
legal inexact planning hints and cold/warm symbolic quotients. The minimized
`logarithm_ignores_inexact_magnitude_hints` test also overflows before the
fix; it requires MPFR-checked output at low/high/low precision. Retain the
original symbolic parameter and full spatial case alongside it. The fallback
must obtain a fresh approximation when only a hint is available; this is not
a new total-work bound.

Acceptance must include positive/negative/zero and symbolic parameters,
ordinary Complex arithmetic, retained division obligations, mismatched
arguments/parameters, unsupported logarithm/root evidence, budget declines,
cancellation and recovery, and serialized components. Require exact rational
certificates without approximation, then run the existing scalar normalizer
and downstream consumer checks. This record specifies work; it does not
establish successful implementation or measured coverage.

The initial focused coverage probe exposes missing owned-coefficient replay
cases: linear combinations, square/offset wrappers, retained rational proofs,
and inverses whose domain is proved only by rational reduction. Add exact
value/domain regressions for those representations, including hidden zero
divisors, depth and coefficient limits, and parameter-basis exhaustion. Keep
the production denominator and existing 95% floor unchanged. The helper
tests complement the full consumer matrix; they do not replace it.

The initial complex ASan campaign times out at ten seconds on the retained
seed `[21,6,127,39,1,4]`. A fixed-input replay completes in 13,736 ms; that
mode does not enforce the mutation loop's timeout, so it is diagnostic evidence
only. The stack is in Carlson RF evaluation during a negative-parameter
complex component's inverse-amplitude refinement. Owner: Hyperreal's existing
Carlson dyadic rounding helper (P5/P20/P35/P36). Domain: nonnegative integer
numerator, positive denominator and the helper's signed binary shift. Before
retaining an optimization, require the same floor/ceiling integers, independent
elliptic/complex enclosures, unchanged downstream exact relations and the
original ten-second sanitizer campaign. The existing directed-quotient test
covers exact division, remainders and signed shifts. First evaluate replacing
separate quotient/remainder divisions with one `div_rem`; retain only measured
changes that preserve the complete output contract. Keep both timeout and
slow-unit artifacts and the complete seed corpus.

Directed case `[0,120,113,108,1,0]` (equivalently `[0,10,0,0,1,0]`)
exposes another missing unit-norm proof with a negative quadratic parameter.
The saved pre-Carlson binary reproduces the same assertion, so the new RF
enclosure is not its cause. Hyperreal owns the bounded rational proof
(P2/P18/P19/P20/P35). Extend the existing positive-quadratic regression to
both signs and retain both spatial encodings. Preserve degree, work and
inverse-domain limits; correct representation reuse or avoid redundant proof
work rather than accepting an unknown norm or enlarging the proof budget.
The failure is a degree-13 intermediate at multiplication of degrees 4 and 9.
Inverting an admitted square root r²=N/H already permits r*H/N. When N is
itself quadratic, apply the existing nonzero rational-norm reciprocal to N
before repeated component squares expand that denominator. This reuses the
same guarded quadratic proof and keeps the degree-12 limit. The unit regression
requires all three relations at both signs. Add all 484 negative-parameter
ordered pairs to the spatial matrix alongside the existing positive pairs.

The [verification record](../../hyperspatial/verification/2026-10-19-e2-complex-jacobi/README.md)
records executed acceptance, unchanged proof budgets, all four cache layouts,
the independent relation-helper coverage gate and retained diagnostic failures.
