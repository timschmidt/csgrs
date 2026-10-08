# Retained Hyper arctangent range-dispatch repair

Disposition: RETAIN, uncommitted. Exactness/completeness outweigh the measured
ordinary-path costs below. The numbers audit and broader ecosystem goal remain
OPEN; adjacent asin/atanh failures are recorded separately, not claimed fixed.

## Why the change is necessary

The unchanged source at Hyperreal bd92d87 plus the previously qualified Plume
Display edit has three reproducible arctangent failures:

1. Cold atan(4*sin(100)) chooses PrescaledAtan for a large negative argument.
   Sampling only an upper signed threshold is not an absolute-domain test.
   The original 18-request history corpus has four cold caps and fourteen
   directed-MPFR passes. Each capped expression's warm partner passes.
2. `sqrt(5)/8 + sum(32 copies of sqrt(7)/64)` is about1.60, yet repeated Add
   nodes retain a planning magnitude below-1. Old atan dispatch treats that
   estimate as a small-domain certificate and diverges. A separate public
   five-second baseline cap is recorded in atan-hint-before.log.
3. A valid exact magnitude certificate may leave sign unknown. The old large
   shortcut then applies pi/2-atan(1/x) to a negative input, giving the wrong
   sign. The synthetic metadata regression uses128*sin(100); outward MPFR
   verifies -65<x<-64 before installing its exact magnitude6/sign-unknown fact.

All three dispatch regressions fail promptly on old code; the first two assert
the invalid dispatch without actually executing the divergent series.
`atan-regressions-before.log` records0passed/3failed, exit101. Frozen baseline
benchmark executable `.audit-numbers.rjcbha/atan-before` SHA-256:
`98e5ed482cfc8f0bfba37baa311e03bb1cea9c49b9c0cc790c48bd293c443ff9`.

## Repair and local argument

- Structural series shortcuts require `BoundInfo::known_msd`, not the
  potentially unbounded-error planning estimate. Exact magnitude<-1 proves
  absolute argument<1/2.
- The structural large reciprocal shortcut also requires a positive sign.
- Otherwise an approximation a at precision-4 satisfies |16x-a|<=1. Only
  |a|<=7 selects the small kernel, proving |x|<=1/2, including its endpoint.
  The kernel's comment now includes that endpoint explicitly.
- Remaining a<0 has |a|>=8, hence x<=-7/16<0. Odd symmetry reaches the positive
  reducer without an undecidable equality/sign search. Its separated cached
  sample also certifies the sign on subsequent queries.
- Positive intermediate and reciprocal reductions retain their existing
  identities. No new node, field, public API, dependency, or representation.

This is a dispatcher repair, not a new approximation series. The five retained
tests include valid coarse-cache boundary variants (601 allowed cache samples,
2404 MPFR enclosure checks), the three dispatch regressions and aborted-cache
protection. One initial test compile needed explicit integer types; both failed
compile logs remain, followed by successful final full gates.

## Correctness and integration evidence

- Full default debug Hyperreal:728passes including19doctests.
- Full release/all-features Hyperreal:833passes including24doctests.
- Both builds pass6456 independent public controls:5544 trig-offset/history,
  336 estimated-sum/history and576 tiny perturbations around half boundaries.
  Every comparison encloses an outward4096-bit MPFR interval. Four input-cache
  histories and six requested precisions are exercised through512bits; tiny
  perturbations extend to2^-1024.
- Both builds pass all18 formerly mixed cold/warm cases,1184 earlier elementary/
  history comparisons,219 exact sign/truncation/history controls and43 oracle
  self-checks. `run-atan-validation.mjs` has21 successful isolated requests/build.
- The release6456-check grid under Memcheck has0errors. Leak checking was
  disabled; no leak-freedom or peak-residency claim.
- fmt check and all-feature library Clippy with-Dwarnings pass.
- Downstream library runs: Hyperlattice19, Hyperlimit242, Hypertri3,
  Hypersolve432, Hypercurve893passed/1ignored (581.51s, exit0):1589passes total.
  These qualify sources as compiled during the run, not concurrent edits made
  later. User-owned Hypercurve work was not changed by this audit.

## Performance, allocation and size

Frozen before/after executables run alternating fresh processes pinned toCPU6,
with256 independently preconstructed inputs/process and32/128-bit demands.
Input construction,4096-bit oracle work and output validation are outside timing.
Every output is validated after timing, not merely checksummed. Global constant
caches may warm within a process; both variants use the same input order.
144 process observations total,128after excluding each family's round0 warmup.
Paired cases have8post-warmup pairs/family, with20000 paired bootstrap resamples.
Pilot observations are separate and excluded. Final hashes, exact counts and
all rows are checked by analyze-atan-bench.mjs.

| Family | Paired before/after CPU ratio,95% interval | Allocation consequence |
| --- | --- | --- |
| Rational |1.00015 [0.97903,1.01093]|unchanged|
| Known small radical |0.95879 [0.94820,0.98404]|unchanged|
| Known negative radical |0.97255 [0.96654,1.01642]|unchanged|
| Small opaque |1.00002 [0.98423,1.01013]|unchanged|
| Positive opaque |0.83783 [0.83348,0.84796]|calls39097→45333; bytes1795424→2071824|
| Warm negative opaque |0.86628 [0.86096,0.88651]|calls34498→39174; bytes1878224→2131712|
| Small estimated sum |0.90670 [0.88920,0.91122]|unchanged|

Ratios below1 mean slower: about4.3% for known-small,19.4% for positive opaque,
15.4% for warm-negative and10.3% for the small estimated sum in these workloads.
This is explicitly not a general speed improvement. Allocator counters are
enabled in timings; byte counts are cumulative requested allocation, not peak
live memory. The correctness repair is retained under the user's priority order,
not because these costs were hidden or timed-out work was called fast.

The repaired formerly nonterminating families complete and pass MPFR: median
4,356,062.5 CPU ns/256 for negative opaque and20,735,884.5 for the large estimated
sum. No ratio against the capped baseline is claimed.

Audit executable before→after: file2209000→2208936bytes; text1661651→1661583;
data219048unchanged; bss4456→4520; loaded total1885155→1885151. Effectively
unchanged size, not a library-wide size claim. After executable hash:
`bfd68b49b7b38f3831233f634b9b26c8ff2e61b2d2bc54936ef382075c050255`.
Numerical benchmark source SHA:
`8c21ed59ba739380dd3e7ae16cef20151f9049f1a20c4e1b92e2a9ea984462b2`.

## Open adjacent findings

Review of the other planning-magnitude consumers distinguishes identities valid
over their full domain from the preconditions of the kernels implementing them.
exp's evaluator rechecks its range and log's reduction rechecks its scaled
argument. asin and atanh feed unchecked tiny-series kernels. asinh/acosh use
globally valid mathematical log identities, but ln_1p directly constructs an
unchecked PrescaledLn; its small-domain assumption requires separate checking
and is NOT considered cleared by the algebraic identity.

Public input `sqrt(5)/64 + sum(740 copies of sqrt(7)/2048)` is approximately
0.99092292259, strictly inside both real domains. At32bits, asin is about10.40
output units low and atanh about3.59units low. Directed MPFR proves both errors.
The frozen80-request/build neighbor corpus is separate: debug79finite with
37numerical failures and1exception; release80finite with38failures;42passes/build,
no caps. These methods have NOT been repaired in this checkpoint. Next turn
should inspect the precise exception, repair the series-domain gates and qualify
those changes independently. Do not close numbers or the broad audit yet.

The precise debug exception is asin at746terms/160bits: i32 coefficient
multiplication overflows in inverse_trig.rs after the invalid small-domain
dispatch. The input is about0.9986741471, still strictly within the real domain.

Separate unchecked-ln_1p baseline: log_boundary.rs checks77 valid asinh/acosh
outputs against outward MPFR. Eight coarse precision0 results violate the
one-unit approximation contract: asinh(±2), asinh(4), asinh(6), asinh(sqrt2),
asinh(sqrt5), acosh(sqrt5), acosh(sqrt17). Each returns zero through PrescaledLn's
coarse shortcut although the true magnitude exceeds1. All69 other sampled
precision/input combinations pass. This additional failure is OPEN; globally
valid log identities alone do not satisfy their chosen kernel's precondition.
Frozen release log-boundary executable SHA:
`9ba171a5ef78127394ed82300d43a503742800dcca30e918540f2f7f3d0a4a11`.
Debug repeats have identical77output rows and the same8failures. All current
evidence revalidates with analyze.mjs and analyze-atan.mjs; immutable snapshots
of the retained atan production/test files are in .audit-numbers.rjcbha.
