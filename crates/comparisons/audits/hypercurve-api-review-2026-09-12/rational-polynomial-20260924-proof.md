# Exact rational polynomial accumulation

Parent Hyperreal: 28eaac35e8c24006c532b25a393b7d430d03823c.
Parent Hypercurve: 04b524382cdf80457c06f02c2bd2d0fa222d21b8.
All prior owned processes were reaped before this edit. No other repository is edited.

The V1 candidate replaces repeated rational Horner products and sums only for
structurally exact rational arguments and coefficients. The private Rational
kernel canonicalizes lazy inputs, trims leading zero coefficients, and clears
coefficient denominators with the existing positive common-denominator helper.
For D the common denominator, C_i=D*c_i, x=p/q, the partial invariant is
V=N/(D*Q). Each step sets N=N*p+C_i*Q*q and Q=Q*q. A zero suffix may reset
Q to one before inserting C_i because its exact value has no denominator
dependence. One final fraction normalization preserves the full numeric value,
including sign and scale. Empty, zero and constant polynomials remain exact.
No field, root, or geometric identity is changed. Mixed Real coefficients and
nonrational arguments retain their existing paths. There is no new public API.

The independent GMP power-sum test remains. A further GMP oracle checks wide
positive and negative rational arguments, many cancelling suffixes, negative
rational coefficient scales, trailing zero coefficients, and lazily unreduced
inputs. Existing symbolic, quadratic-certificate and ill-conditioned nonroot
tests must pass. All Hyperreal tests and both all-target Clippy configurations
are required before any dependent qualification.

The separately frozen probe compares the candidate with the literal prior
rational Horner kernel using the same compiled arithmetic library and inputs.
It checks exact equality before timing and records medians of five batches,
with input construction outside timing and ordering alternated across cases.
This is an internal arithmetic comparison, not a full-workload speedup claim.
Small native and wide rational workloads, dyadic and odd denominators, degrees
one through sixty-four, and repeated cancellation are included. The candidate
is not qualified until these results and dependent closure tests are assessed.

V1 Clippy stopped before tests on two pre-existing Rust 1.98 fixed-chunk lints in the quadratic-tower tests. The driver was reaped, then both loops were migrated to as_chunks::<2>().0.iter(). V2 uses a new immutable snapshot with that mechanical test-only cleanup; no arithmetic candidate change.

V2 passes all 834 Hyperreal tests, both all-target Clippy configurations and cargo
fmt. The first probe warmed the old arithmetic caches and showed substantial
repeat-evaluation regressions. A separately frozen fresh-input probe confirms
that wide degree-32/64 evaluations improve by 5–47x, while low-degree dyadic
cases can regress. All V2 build and probe processes are reaped before V3.

V3 keeps the original rational Horner path through degree four, where repeated
native/cached evaluation wins in the probe. Higher-degree integer accumulation
publishes through the existing lazy internally unreduced Rational representation.
Its full numerator and positive denominator retain the exact value; sign/zero
queries avoid a final GCD, while canonical numeric queries perform and retain
that reduction on demand. No new storage or cache is introduced. This is intended
to address the sampled root-refinement sign workload without creating a second
public polynomial interface. The same independent numeric oracles must still
pass, and fresh/repeated measurements and downstream qualification are pending.

V3 again passes all 834 Hyperreal tests and both Clippy configurations. Its
construction probe restores the short-polynomial timings and removes the eager
final GCD from long outputs. Repeated wide long-polynomial calls still regress
against the old warmed arithmetic chain (roughly 2–200 microseconds versus
0.1–1 microsecond). These measurements are construction-only and do not claim
canonical-value performance: the independent equality oracle verifies full
numeric meaning before each timed workload. The result is not yet qualified;
the dependent driver is testing actual retained-root and geometry workloads.

The V3 dependent run passes all 252 Hyperlimit tests and its Clippy configurations, then stops before Hypersolve tests on eight existing Clippy findings. All processes are reaped. V4 changes only those mechanical Hypersolve expressions and creates a fresh full snapshot. A separate frozen fresh-input probe distinguishes sign demand from full canonical numerator demand; both compare exact results first.

V4 passes the scalar/Hyperlimit suites and demand probes, then Hypersolve Clippy
reaches one more pre-existing integration-test vec lint. All V4 processes are
reaped. V5 replaces that fixed vec with an array. It also consolidates the existing
lazy-ratio canonicalizer onto the standard fraction-reduction kernel, removing
its duplicate generic GCD/division implementation and recovering the existing
power-of-two reduction optimization. The GMP power-sum oracle now checks signs
and arithmetic reentry before its canonical numeric comparison, so deferred
ratio publication is exercised before the comparison forces normalization.
Fresh wide degree-8/32/64 sign queries in V4 improve 43–1372x; forcing canonical
numerators improves 1.3–47x in those samples. Repeated-wide construction remains
slower than the old retained arithmetic chain. No end-to-end claim is made yet.

V5 passes all 834 Hyperreal, 252 Hyperlimit and 507 Hypersolve library tests,
and both all-target Clippy configurations for all four affected stack crates.
Formatting and diff checks pass. The source-sharing regression passes; the
selected chamfer passes in 4.86 library seconds and the recursive selected-radial
chamfer in 61.11 seconds (about 61 seconds before this scalar change).
The previously 75-second extended-fillet timeout now completes in 37.98 library
seconds, including its inside/outside queries for both policies and directions.
The third-generation fillet still exceeds 75 seconds. All focused processes
are reaped; public scalar integrations and all Hypercurve library/public cases
now qualify the same frozen V5 sources before any commit.

All V5 numerical public integration targets pass: fourteen Hyperreal targets and
all nine Hypersolve targets. Hyperreal's separate GMP API inventory test fails
because four existing exact certificate APIs have no inventory classification.
No new public API was added by this candidate. The outer driver stops before
full geometry qualification; all V5 processes are reaped.

V6 changes only hyperreal/tests/gmp_api_coverage.rs, classifying the four existing
selected radical-field certificate/branch APIs as having no direct GMP/MPFR
counterpart. The input bridge verifies that every other one of 2,044 source,
test and fixture bytes is unchanged from V5. The repaired inventory test and
all-target Hyperreal Clippy are rebuilt fresh from V6. Completed V5 numerical,
root and focused geometry results are reused with their original executable
hashes and immutable snapshots, because the modified integration-inventory
source is not part of those executable inputs. Full Hypercurve qualification
uses the same hash-bound V5 library-test executable and newly built public
integration executables from the unchanged production source in V6.

The initial V6 copy exceeded the /tmp quota before publishing its manifest or
running a build. With all processes reaped, only the task-owned rebuildable
Cargo debug cache was cleared (about 5 GiB). The incomplete, unpublished V6
copy was resumed; all prior immutable snapshots, logs and executable evidence
remain untouched. The completed V6 copy is now immutable.

V6 full qualification completed: 1,340 passes, six ignored, thirteen nonpasses
across 1,359 cases. Two previous timeouts pass, but two previously passing
fillet/chamfer cases time out under the full concurrent schedule. V6 is not
qualified for commit. All owned processes are reaped before V7.

A public polynomial-reentry counterexample additionally proves that a lazy
4/6 payload plus one becomes 10/6 marked reduced. Numeric equality holds,
but canonical queries and equal-value hashes fail. The terminal receipt
is rational-polynomial-canonical-reentry-20260924-parent-terminal.json.
V7 fixes the shared Rational arithmetic boundaries, including direct unit
addition/subtraction, optimized means, and power-of-two division. These
kernels use cached canonical inputs before applying coprimality theorems;
zero/sign queries and delayed geometry aggregate reducers keep their routes.
An independent num::BigRational oracle tests canonical parts, structural
facts, equality and hash consistency across native and wide lazy ratios,
both operand positions, unary and binary arithmetic, and aggregate reentry.
The polynomial cancellation test also checks canonical parts before GMP
conversion can hide a lost lazy flag. Scalar and downstream checks remain
pending; no success or performance claim is inferred from the fix alone.

V7 passes both all-target Clippy configurations, the canonicality regression,
and the polynomial tests. Its full suite has 834 passes and one obsolete
scheduling assertion: lazy dyadics now normalize by shifts and enter the
wide-dyadic subtraction kernel without a GCD, rather than declining it.
All V7 processes are reaped. V8 changes only that dispatch expectation,
retaining the exact-value oracle and requiring one lazy normalization,
one optimized subtraction, and zero GCDs.

V8 passes all 835 Hyperreal library tests and both Clippy configurations.
The frozen public counterexample now reports numeric_equal=true,
canonical_parts=true, equal_hashes=true against the V8 library. A first
standalone compiler invocation used the release directory instead of its
deps subdirectory and failed before running the probe; the corrected
invocation succeeds with unchanged production and probe sources. The
before/after terminal receipts retain exact source and executable hashes.
Hyperlattice is explicitly included in the V8 downstream sweep because
ordinary Rational arithmetic is shared by its scalar and geometry kernels.

V8 dependent library suites pass: 19 Hyperlattice, 252 Hyperlimit and 507
Hypersolve tests. Both all-target Clippy feature configurations pass for
all five stack crates including Hyperreal and Hypercurve. The seven focused
Hypercurve cases retain the existing third-generation fillet timeout; six
pass. Recursive chamfer: 61.23 library seconds; nonrepresented-chord/selected-
circle fillet: 147.20; extended companion fillet: 38.09; selected-parallel
companion fillet: 0.35. The latter two were earlier bounded-workload timeouts.
The former two match their previous isolated timings, resolving the suspected
V6 regression at isolated scheduling. No concurrent throughput improvement is
claimed. The full V8 sweep will reuse these exact frozen executable receipts
and run the remaining cases with two workers, keeping all deadlines unchanged.

Fresh wide degree-8/32/64 sign demand improves 43.4–1368.9x in the V8 probe;
canonical-value demand ranges from 0.95x to 47.9x. Identical warmed long
polynomials can still favor the old retained arithmetic chain substantially.
These bounded microbenchmarks do not imply universal workload improvements.
All dependent and demand-probe processes are reaped before public integration
validation begins. The candidate remains uncommitted pending that full sweep.

Arithmetic proof boundary: for reduced a/b and c/d, let g=gcd(b,d).
The sum numerator a(d/g)+c(b/g) is coprime to b/g and d/g, so only g
can cancel from lcm(b,d); the mean permits the additional factor two.
Similarly gcd(a±b,b)=1 and division by a power of two needs only binary
cancellation if a/b is already reduced. These are exactly the theorems
invalidated by an unmarked common factor. V8 obtains the cached canonical
view before those kernels and leaves valid lazy returns on zero shortcuts
flagged. The polynomial accumulator publishes through the existing lazy
constructor rather than claiming those unreduced integer parts are canonical.

Qualified and committed: hyperreal a1a8637b59bf036d9d68dbd34a0279df4626ab49, hypersolve e6d6cf56f7c8d27aaa75699bf095feeadcca6aa0. All owned processes reaped; all thirty repositories clean; no push. Full goal remains active.
