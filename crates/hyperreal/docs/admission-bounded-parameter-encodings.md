# Bounded parameter encoding admission

The following entries retain the investigation and failed variants. Final
source-frozen validation is recorded at the end; intermediate successes do
not substitute for that evidence.

2026-10-19. Research only, before any new production node. Hyperreal owns exact
algebraic and inverse-trigonometric composition; P1/P2/P3/P6/P18/P19 apply.
A candidate parameter carrier must be mathematically finite for every decoded
field without trusting a serialized chart assertion. At the public encoder,
exact source values should survive the bounded-parameter reconstruction.

For the inverse meridian use an algebraic open-interval parameter rather than
tanh(atanh(v)): for -1<v<1, a=v/sqrt(1-v²), and decode v=a/sqrt(1+a²).
Every finite decoded a gives a strict -1<v<1. Positive root branches determine
the same sign and exact value. This avoids requiring an unrelated missing
hyperbolic inverse certificate. The meridian coordinate is xi=E(m)*v.

For 0<=m<1, r=sqrt(m/(1-m)); decode h=1/(1+r²),m=1-h. For closed normalized
time u∈[-1,1], tau=asin(u); decode u=sin(tau), so exact chart endpoints remain
available. The actual flow times are u/9 forward and u/6 reverse.

Missing acceptance: across all 22 scalar representation families and both
signs for v/u, prove zero exact residuals after these encodings. Include zero
and closed-time endpoints. Test the existing implementation before choosing
this payload. A mathematical identity is not a claim that the current scalar
certificate normalizer can already replay it. If a certificate is missing,
record the failure at its scalar owner; do not replace exact assertions with
numerical closeness or assume that an unverified metadata flag proves it.

The baseline probe fails the Pi parameter round trip. A diagnostic of all 22
families finds direct rational reconstruction succeeds throughout, and every
sin(asin) round trip succeeds. Generic square-root squaring and the derived
parameter/meridian reconstruction lack certificates in all 21 nonrational
families. Native ring recognition currently compresses a product of square
roots as one opaque scalar atom, hiding the existing individual root relations.
First investigate preserving those roots as separate relation atoms. Retain
the existing nonnegative-radicand and denominator guards, degree, atom and work
limits. This is a proof-completion experiment, not an accepted optimization.

The root-atom experiment restores all 22 parameter encodings, but all 42
nonrational meridian cases still lack a certificate. A diagnostic at Pi exposes
a residual (1+Pi)*(r*s-1), where r,s are admitted positive roots and their
square relations prove (r*s)^2=1. Squaring alone cannot choose +1 over -1.

Investigate one additional sufficient zero proof after ordinary relation
reduction: split the residual into P-Q using only terms with retained exact
signs, so P,Q are nonnegative; prove P²-Q²=0 with the existing admitted ring
relations. Injectivity of squaring on nonnegative reals then proves P-Q=0.
No division by P+Q is required, including P=Q=0. This is a single extra attempt
under the remaining operation, atom, term, degree and cancellation budgets;
its recursive reduction must not invoke another square-sign attempt. Unknown
term signs decline, and all original inverse and root-domain guards remain.
Negative/opposite signs, zero, invalid/unknown radicands, active cancellation,
work exhaustion and perturbed nonidentities need private guard acceptance.

The single sign-square attempt restores 42 of 44 meridian cases, and six
private branch/domain/resource guard tests pass. PiSqrt still declines before
that attempt: a traced multiplication tries degrees 12+2 while a repeated
sqrt(2) factor could already collapse to its admitted rational square. There
are 424 node visits and 4,448 operations remaining, so this is a degree cap,
not exhaustion of either work counter. Exposing constant-root factors alone
does not resolve it.

Investigate reducing admitted rational root squares while merging the two
sorted monomials. Never construct a retained monomial above degree 12; charge
every input-factor visit and coefficient product under the same remaining
8,192-operation budget. Check the original coefficient product and each
reduction against the existing 4,096-bit cap, even when a zero root would
later erase it. Only already-admitted square-root relations with constant
numerator and unit denominator qualify. Unknown/negative radicands and
nonconstant relations retain their existing behavior. This moves an existing
exact identity earlier, rather than raising the degree or work limit.

Early rational-root reduction admits the complete PiSqrt expansion but its
20-term residual still exceeds degree 12 when squared. Its positive and
negative sides share the same nonconstant polynomial factor. Before the one
square attempt, try the existing exact polynomial divider on those two sides.
A completed P=R*Q makes R=1 sufficient even if Q vanishes; this does not invert
Q or claim it nonzero. Re-partition the smaller residual with retained signs,
then make the same single bounded square attempt. Keep both failed experiments
and their degree traces; neither partial improvement is final acceptance.

The unchanged 22-family encoding acceptance now passes with the sign/factor
and early-root reductions. The first complete scalar library run passes 954
tests and fails the new rational-root degree test. An initial hypothesis that
constant N/D radicands needed an extra rational quotient was tested and still
failed. Inspection shows that Computable::rational leaves noninteger Ratio
sign caches empty; the test constructed a raw root before certifying its
radicand, so its presumed root relation was never admitted. Remove the
unnecessary quotient variant. Have the fixture establish its exact rational
sign and assert relation admission before testing early products. The separate
unknown-radicand rejection fixtures remain unchanged. Keep both failed runs
and the discarded variant. A new replay probe initially fails to compile
because it used to_f64 instead of the actual to_f64_lossy display API; that
diagnostic is separate from mathematical replay acceptance.

With the fixture's domain precondition explicit, all 955 scalar library tests
pass, including eight new private guards. The unchanged public family encoding
test also passes. A stronger replay acceptance remains failing: deserializing
all five fields loses the input construction's sign evidence. Across 44 signed
cases, cold parameter and meridian certificates succeed only for two rational
cases; structural-fact inspection restores 42 (ConstOffset = pi-3 remains
unresolved), and display or certified -128-bit refinement restores all 44.
All 44 sin(asin) time certificates survive cold replay. The nine zero/near-endpoint
replay cases pass. Keep the cold failure; no display tolerance or warmed result
substitutes for that missing proof. Decide the explicit domain-evidence replay
contract before accepting a production payload. Full downstream regression
checks on this frozen experiment are a separate obligation and do not close
that replay gap.

The first downstream regression run passes Hyperreal (1,215/41 suites),
Hyperlattice and Hyperlimit, then fails two existing spatial complex-Jacobi
matrix entries: [0,6,2,2,0,0] and [0,6,2,2,1,0] lose their unit-norm rational
certificate. Preserve both matrices and the failed frozen-source archive.
The new decomposition must not preempt the established native proof strategy.
Run the original native strategy unchanged first; only if it declines, clear
strategy-local atoms/memo tables and try root completion with the SAME remaining
node and operation counters. Do not allocate a fresh proof budget. Add direct
counter/cancellation checks for that handoff and rerun both old matrices and
the new encoding acceptance. Cold serialized replay remains a separate gap.

The preserved-strategy variant passes the unchanged encoding acceptance and
all six complex-Jacobi spatial tests, including both complete pair matrices.
Cold replay still fails at Pi on this target. Investigate non-numerical domain
replay only within the optional root-completion strategy: exact leaf/sign
syntax, signs of admitted polynomial quotients, and the existing fixed rational
intervals of shared constants can re-prove inverse and root obligations.
Keep these proofs query-local, bounded by the same remaining node/work counters.
A completed nonnegative root-domain proof may determine its principal-root
sign without a numerical root cache. Unknown/opposite domains still decline.
Do not change the decoder, wire format, numeric refinement policy or original
native strategy. Original constants' fixed intervals are not new tolerances.


The query-local domain variant passes all three unchanged external acceptance
tests, including cold and display-warmed replay across every scalar family and
nine exact near-boundary cases. Its first full library run passes 954 tests and
fails a newly added fixture that incorrectly assumed cold sqrt(2) and sqrt(1/2)
had unprovable domains. Exact rational syntax now proves those domains. Preserve
that failed run and replace the fixture's unknown-domain assumption with raw
sin(1/3), a deliberately opaque atom in this non-numerical recognizer. It must
decline before domain certification, recover afterward, and never numerically
refine either root. Keep a separate positive cold-rational-domain test. The
other unknown-root and inverse guard tests are unchanged.

Fourteen private guards now pass: the original branch, zero, coefficient,
degree and domain cases plus fixed-constant interval separation, conservative
trigonometric syntax, domain replay resource limits and the preserved-strategy
handoff. Fixed bounds touching zero do not prove a strict sign. Both attempts
share the original query's counters and controller; exhausted or cancelled
queries cannot acquire a fresh budget. Only the completed top-level exact
reduction publishes a persistent certificate. The public family/replay probe
is now copied into the native Hyperreal integration suite with serialization
tests gated by its existing optional serde feature. This remains a candidate
pending complete downstream and testing-posture validation.


The first broad run of these guards exposes a test isolation error: the raw
constant's cached() accessor reads a process-global shared constant cache, so
parallel tests may warm it. Its interval/sign assertions all pass. Inspect the
query's own node cache and facts instead, retaining the numerical-cache checks
on every nonconstant root and opaque-domain fixture. This does not disable or
clear shared caches or weaken a mathematical acceptance assertion. Preserve
the failed 960-pass/one-fail log and freeze before rerunning the full matrix.


The source-frozen replay candidate now passes the complete affected regression
matrix: Hyperreal 1,224 executions / 42 suites, Hyperlattice 239 / 23,
Hyperlimit 366 / 19, Hyperspatial 100 / 24, and Hypercurve 68 / 3. Strict
all-target/all-feature Clippy, workspace formatting and diff checks also pass.
The cold-replay research archive retains all 736 frozen source paths and every
executed result. This precedes the broader testing-posture extension, so it is
not a substitute for final validation of that extension.

The integrated candidate adds four more private guards after a focused
133/157-line domain coverage result: cold arithmetic signs, unknown mixed
nonconstant bounds, coefficient growth, zero root/inverse separation and cold
linear-form replay. All eighteen private guards and four public integration
tests now pass. The latter include JSON/CBOR, concurrent readers, low/high/low
precision order and unchanged authoritative serialized values. Independent
spatial consumers already pass 1,496 workflows in an external probe. The new
coverage gates, benchmark, cache/compiler/source consumers and sanitizer
campaign remain pending final validation.

## Final candidate validation

The final 741-path source freeze passes Hyperreal's 965 library tests and
1,229 executions across 42 suites, Hyperlattice 239/23, Hyperlimit 366/19 and
the affected Hypercurve suites 68/3. Stable and Rust 1.95.0 each pass 206
spatial executions in 51 suites, eight independent README consumers, strict
Clippy and benchmarks. All four primitive-cache configurations pass 640
executions in 128 suites. Native documentation and WASM check/Clippy pass.
The unchanged source is required at target integration; the
[verification record](../../hyperspatial/verification/2026-10-19-e2-parameter-encodings/README.md)
records command exits, hashes and subsequent root checks.

The new domain helper measures 156/157 executable lines (99.36%); the complete
changed shared-ring functions measure 101/103 (98.06%). All fourteen independent
coverage floors pass. LLVM's combined-object export emits missing-profile
warnings; the verification record retains their audit and limits rather than
claiming warning-free instrumentation or whole-crate coverage.

The directed campaign completes 176 cases in 22 independent family buckets.
ASan advances INITED 267 to DONE 11,268, with no new failure artifacts. The
completion-checked native benchmark accepts all 2,200 evaluations in 220 rows
on both compilers and in a separate process measurement. It measures cold
payload reconstruction and retained proof reuse as distinct workloads; no
cross-library speed ratio is claimed.

This retains the original proof strategy before optional root completion and
does not add a scalar wire tag, change authoritative values or widen proof
budgets. Native total-work/heap bounds, connected formal verification and the
Thompson map/inverse remain outside this increment. The spatial admission
records preservation of qualified provenance through subsequent operations.
