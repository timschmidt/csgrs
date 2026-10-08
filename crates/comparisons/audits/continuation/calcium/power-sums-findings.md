# Checkpoint 52 — power sums and a shared point-image completeness gap

The isolated power-sum candidate passes the exact kernel oracle, but is **not
retained or performance-qualified**. Public-query qualification exposed a
pre-existing completeness gap, identical in the retained baseline and candidate.
Timing and allocation campaigns are paused in favor of that higher-priority work.

## Candidate and mathematical evidence

The candidate uses integer-scaled roots, Newton power sums, binomial convolution
for addition/subtraction and pointwise power sums for multiplication/division.
It reconstructs a primitive integer annihilator with exact divisibility checks.
Division by a carrier with an unused zero root falls back to the original sampled
resultant path. Signed leading-coefficient orientation, degree-nine admission,
source validation, image construction and public refinement are unchanged.
Only one existing source file changes; one private module is added. No live
production or donor file changes. The 956-file frozen retained baseline is reused.

An independent polynomial-ring Sylvester determinant (subset/Leibniz expansion,
not sampling, interpolation, Newton sums or a donor backend) checks 4,840 signed
carrier/operation cases: baseline and candidate agree with all expected full
polynomials, including multiplicities, zero resultants and 136 fallback cases.
There are 1,210 independently constructed determinants. All 19 focused debug
tests and 805 default-feature debug library/integration tests pass. This is not
a fresh matched baseline, all-feature, release, Clippy, WASM or full-CI run.

The public collector emits 6,441 records per variant: 6,400 arithmetic cases,
40 cost-case controls and a terminal record. Full paired records are identical.
The independent BigInt rational polynomial/Sturm oracle performs 43,934 checks:
43,109 pass and 825 fail. All 4,301 returned roots pass annihilator-root-set,
unique arithmetic-image root, selected-root isolation and containment checks;
all 969 supplied exact witnesses pass vanishing and containment checks.

| Public outcome | Cases | Independent disposition |
| --- | ---: | --- |
| Transformed | 4,301 | Every returned result passes |
| InvalidTransformedEvidence | 825 | Valid single-root images rejected; gate fails |
| DenominatorMayContainZero | 241 | Nonzero guard preserved |
| NonIsolatingImageInterval | 65 | Multiple/no-root image controls preserved |
| UnsupportedDegree | 968 | Existing degree admission preserved |
| Undecided | 40 | Zero-resultant controls preserved, separate open issue |

The failed mathematical gate remains exit 1. The original oracle, failed capture
and full output are preserved; evidence-integrity success must not relabel this
as mathematical success. The Sturm oracle checks closed intervals; this corpus
does not establish all partition-owned half-open endpoint behavior.

## Root cause and next transfer

Every one of the 825 failures contains the message `a collapsed refinement
interval requires an exact witness`. For example, exact point inputs 1 and 1
under addition produce a valid singleton image at 2, but no result is returned.
`hypersolve/src/algebraic_binary.rs` constructs every binary image interval with
`exact_root: None`, including collapsed intervals. The refinement layer correctly
rejects such an interval before polynomial replay (root_isolation.rs 693–840).
This demonstrates lost completeness, not a wrong returned value or a regression
introduced by the power-sum candidate.

Next: a separate isolated candidate off the retained baseline should preserve a
certified collapsed image as a witness while keeping containment, polynomial
vanishing, uniqueness, degree and policy replay. Approximate overlap is not
equality. Nonrational endpoints, Unknown/false equality, half-open ownership,
zero guards, repeated/reducible carriers and consumer behavior need explicit
qualification. Do not combine this repair with the still-unqualified power-sum
optimization. The 40 zero-resultant controls remain a separate open transfer.

## Resources and limits

Native baseline, candidate and candidate Memcheck stdout are byte-identical,
4,332,578 bytes each (12,997,734 workspace bytes together). Candidate Memcheck
reports zero errors and zero definite/indirect/possible lost bytes, but
**1,364,464 bytes in 11,605 blocks remain reachable**. It records 2,634,556
allocations, 2,622,951 frees and 156,378,772 cumulative requested bytes including
collection/setup/output. Some stacks show Sturm-cache allocation; ownership and
boundedness of every reachable block have not been established. This is not a
zero-live, peak-memory, RSS or performance result, nor a baseline memory comparison.

Five dedicated executables occupy 34,189,432 bytes in the existing bounded
`/tmp/calcium-power-sums.2CGTky` directory; the shared Cargo cache is reused and
its growth is separate. The isolated source copy has 45,450,206 logical bytes
before edits, reflinked where supported. No new donor build or broad copy,
cleanup/deletion, commit, push or external report. Unstripped driver sizes and
collection elapsed times are not application-size or benchmark evidence.

All ten build/test/check/memory captures are terminal. There are no timed CPU or
allocation observations yet. No new donor lines are credited: continuation
coverage stays 1,415 complete files, 20 partial files and 183,653 unique read
lines. All five retained continuation transfers remain unchanged. The complete
original ecosystem audit remains in progress.
