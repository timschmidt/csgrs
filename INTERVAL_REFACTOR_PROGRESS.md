# Interval refactor progress

Contract: [implementation plan](docs/INTERVAL_REFACTOR_PLAN.md). The full
15-phase objective remains active. No phase is claimed complete.

## Initial state and isolation

- CSGRS HEAD: `1671894b4a896b091416503910e3720ecf233c82`.
- Initial branch: `main`, zero ahead / zero behind the locally recorded
  `origin/main`; this is not a claim about a fresh remote fetch.
- Initial dirty paths: only untracked `docs/INTERVAL_REFACTOR_PLAN.md`,
  user-owned and preserved unchanged.
- Work branch: `interval-refactor`. No concurrent worktree was listed.
- Alumina Interface cloned clean at `ef94d9e60e0004f3ec894d174fa49c00d547c63f`;
  firmware cloned clean at `bdee397d0b80841f16bf17e7bcd029b4feb2c212`.
  Both started on `main`, zero ahead/behind fetched `origin/main`, and now
  have `interval-refactor` branches. Neither contains AGENTS.md.
- User explicitly authorized cloning and managing the referenced repositories.
- Evidence archive cloned sparsely at `b20000e377626a41b6ff8e6f87aee2b20ebd7865`.
  Its default main is historical; existing newer ledger rows will not be replaced.
- `interval-assessment.md` was absent in the initial CSGRS and local sibling
  search. It is also absent in the two fresh clones; GitHub code search returned
  zero matches for that filename under timschmidt. This required reference
  remains unavailable. Implementation follows the explicit plan and current
  source; no unseen assessment claim is treated as reproduced evidence.
- Raw local evidence staging: `/tmp/interval-refactor/baseline`. Archive and
  digest registration for completed foundation checks is now recorded below;
  still-running checks remain in staging and are not claimed complete.

## Admission before production edits

Owner: new Hyperinterval scalar set layer. Relevant properties: P1, P3, P5,
P6, P8–P11, P19, P31–P33, P35–P39. Initial input domain: explicit empty and
finite ordered primitive endpoint sets; typed domain and exhaustion outcomes.
Missing witnesses: checked construction rejects reversed endpoints, empty is
not a sentinel, interval signs cannot be consumed as scalar equality, and
firmware feature builds have no allocation or Hyperreal dependency.
No existing arithmetic or predicate dispatch has been changed.

## Phase status

| Phase | State | Evidence still required |
| --- | --- | --- |
| 0 inventory/baseline | in progress | audited source map, sibling states, tests, cost/size/coverage/fuzz, hashes |
| 1 skeleton/laws | local acceptance passed | hosted OS jobs remain unexecuted |
| 2 integer/fixed | local correctness/cost probes passed | firmware cycles/HIL and coordinated baseline gaps remain |
| 3 rational/dyadic/ball | carriers and local acceptance implemented | solver kernel migration, full coverage and consumer cost gates |
| 4 proposals/float | proposal wrappers and strict integer IEEE core implemented; local x86_64 qualification passed | full surface/coverage, further targets and shipped consumers |
| 5 elementary functions | abs/min/max, rational signed powers and rational/dyadic/fixed/IEEE square roots and rational/dyadic exp/log implemented | remaining powers/functions, domain/extrema/pole corpus and full coverage |
| 6 lattice | not started | component oracles and allocation/cost evidence |
| 7 solver | qualification started; production migration rejected | satisfy size/performance gates with unchanged statuses, certificates and dispatch |
| 8 interface | not started | unchanged canonical bytes and profiles |
| 9 firmware | not started | exact differential, no_alloc, HIL/cycles/stack/size |
| 10 predicate filters | not started | proof/cost/trace admission |
| 11 path dynamics | not started | domain-neutral extraction and exact field equivalence |
| 12 ODE/reachability | not started | certified tubes, bounded refinement, analytic oracles |
| 13 Taylor/adoption | not started | consumer evidence and remainder/wrapping proofs |
| 14 release | not started | coordinated CI, packaging and clean consumers |

## Executed checks

- Initial `cargo test --locked -p hypersolve --all-features --lib`: failed
  before compilation because the sandbox could not resolve static.crates.io
  for crossbeam-epoch 0.9.21. Original log retained.
- Authorized network retry: passed, 623 library tests. Full Hypersolve
  integration-test command also passed; individual suite outcomes retained.
- Hyperpath all-feature baseline passed: 444 tests across four suites.
- Firmware Rust 1.88 baseline passed: 83 FOC and 5 clock tests. The preinstalled
  1.88 toolchain was incomplete; repaired with rustup before measuring.
- Interface baseline first failed on duplicate Hyper packages using sibling
  aliases. Normalized its dependency paths to this monorepo, regenerating the
  lockfile. Compilation then reproduced two existing API incompatibilities:
  fallible CSGRS construction and the new implicit-curve family tag.
  Propagated `ValidationError` through `SceneError::Solid`, preserved tags 1–9
  and assigned tag 10 to implicit polynomial curves. With these explicit
  compatibility fixes, 199 library tests passed. This is a normalized baseline,
  not a claim that the untouched upstream revision passed.
- Hypersolve interval certification focused Criterion baseline passed (10
  samples, 1 s warmup, 2 s measurement per row); raw samples retained. These
  are focused probes, not cross-library performance or regression claims.
- Hypersolve four-iteration allocation profile passed; raw per-representation
  rows retained. Full production coverage baseline passed: 49,022 / 55,093
  executable lines (88.98%), above the unchanged 85% gate. The 1800-second
  external containment deadline was not reached; this is not an internal
  work-bound claim.
- Hyperinterval: 22 stable/MSRV feature-matrix commands passed; nine conformance
  tests and three compile-fail authority tests passed. Six Rust 1.88 no_std
  builds passed (default, integer/fixed, integer/fixed/serde on WASM and
  thumbv7em-none-eabihf). These are representative builds, not Xtensa/HIL proof.
- Hyperinterval strict Clippy, rustdoc, examples/benches checks and workspace
  formatting passed. Whole-repository `zizmor --pedantic --strict-collection .`
  passed with no findings. Hosted macOS/Windows jobs are configured, not run.
- Initial Hyperinterval LLVM coverage: 195 production executable lines, 186
  covered (95.38%); 285 regions, 273 covered (95.79%); 46/46 functions. Tests
  and dependencies excluded. LLVM reports no branch counters in this mode;
  this is not a 100% or branch-coverage claim. Raw report retained.
- `cargo metadata --no-deps --format-version 1 --locked`: see captured exit
  status in baseline/features.txt. This records declared features, not a
  resolved dependency-cycle or firmware-closure proof.

## Immutable foundation evidence

- CSGRS inventory/API checkpoint: `6b72f0c7d`; phase-1 source: `8e151ca5c`.
- Interface normalized baseline checkpoint: `ee6cdb1` (199 tests passed).
- Evidence repository local commit: `ac24a78d847762b6ca63e9a16cd8fead30296b9d`.
- Record: `docs/evidence/2026-11-12-interval-foundation`, 206 files,
  756,992 bytes, SHA-256
  `f408b880769a135e8247970f6b116548a4995f7592680f98eeb9c1f711eeb425`.
- Generated and verified using the existing `scripts/evidence_archive.py`
  manifest/verification functions scoped to this new record. Existing archive
  rows were preserved; this is not a verification of unavailable later records.
- Bounded ASan smoke passed: 10,000 runs, seed 20261112, max length 1024,
  max time 30 s. Fuzz build and strict Clippy passed. Corpus retained in the
  evidence record, outside the source tree. No repository has been pushed.

## Immutable phase-2 evidence

- Source checkpoint: `8d6daa7d7`.
- Evidence local commit: `8fa48073c8ced67a4dd7cd1460f8a32e8e79c0df`.
- Record: `docs/evidence/2026-11-12-interval-fixed`.
- Manifest row: `docs/evidence/2026-11-12-interval-fixed	275	8102925	6545215bd945264a9e8c9cd01966f87ecdd0a805f94b40474bb4bda0585cc328`.
- Both favorable and rejected cost samples, original failed attempts, final
  sanitizer corpora, Q30 differential workspaces and completed Hypersolve
  coverage are retained. Registration uses the same scoped archive manifest
  and verification functions; no prior evidence row is rewritten.

## Phase 2 admission and current evidence

Owner: Hyperinterval, P1/P3/P5/P6/P31/P35/P36. Input domains are primitive
integer ranges and i32/i64 fixed intervals with valid scales. Integer products
return the containing integer hull (gaps need not be attained); fixed products
and quotients round outward. Integer reciprocal/division is not silently
reinterpreted as truncated real arithmetic. Fixed strict division rejects a
zero-containing denominator even for an empty numerator.

Missing witnesses before implementation were exhaustive reduced grids,
overflow edges, named midpoint rounding, exact widening/coarsening and Q30
field/error equivalence. These tests now exist. Iterative power operations
are bounded by a u32 exponent; no ambient precision or refinement loop exists.
Q30 differential checks use the actual unchanged firmware kernel: 20,736
boundary combinations, 100,000 deterministic random rows plus ratios and 81
extreme ratios passed with no mismatch. This does not establish timing, HIL,
firmware migration or all-phase completion.

Final local evidence: 20 conformance/arithmetic tests and three compile-fail
checks; stable and 1.88 feature matrices; WASM/ARM no_std builds; strict lint,
rustdoc and actual example execution passed. Q31.32 extreme cross-product
oracles and exact storage/rescale/discrete-to-continuous tests are included.
ASan arithmetic smoke completed 100,000 runs, seed 20261112, max length 16,
max time 30 s; constructor smoke completed 10,000 runs. Original harness-CWD,
missing corpus-directory and sandbox LeakSanitizer failures are retained.
A fuzz-only collapsible-if lint was fixed without changing evaluation order.

The initial cost probe exposed a real constant-specialization regression and
is retained as rejected. Inlining two private quotient helpers preserved exact
outputs and improved the corrected paired probe; see
[performance record](crates/hyperinterval/PERFORMANCE.md) for raw medians,
code-size attribution and limits. No domain consumer has been migrated and no
firmware cycle-time equivalence is claimed. The shared carrier's explicit empty
state remains larger than the legacy nonempty wrapper.

Final production Hyperinterval LLVM report: 470 / 496 lines (94.76%),
900 / 959 regions (93.85%), 83 / 83 functions. Dependencies and tests excluded;
no branch coverage was measured. The universal 100% coverage target remains
open; these finite generators/fuzz campaigns are not a universal closure proof.

## Phase 3 admission before edits

Owner: Hyperinterval scalar sets and exact-point bridge, P1/P3/P5/P6/P19.
Inputs: ordered finite rational endpoints, common-scale arbitrary integer
mantissas, explicit binary-grid exponents, exact Real centers and nonnegative
rational radii. Missing witnesses are all sign-partition products against
independent four-corner rationals, directed scale conversion, bounded huge-shift
rejection/retry, and independent MPFR controls on Real-derived enclosures.

The primitive build must retain zero production dependencies. Host Rational
uses Hyperreal's optional dependency and its Rust 1.95 requirement; standalone
BigInt/dyadic code stays allocation-enabled no_std. No Hyperreal source, layout
or cache is changed. Solver reports and symbolic RealInterval remain in
Hypersolve. Arithmetic extraction must preserve predicate/sign order; existing
consumer migration is gated separately by exact outputs, traces and cost.

## Phase 3 local acceptance and remaining work

Implemented RationalInterval, common-scale DyadicInterval and initial rational-
radius RealBall. All public ingress validates order/radius; dyadic imports
validate the claimed grid. The private endpoint relation kernel also handles
Hyperreal Rational's exact PartialOrd interface; no arbitrary endpoint family
is publicly admitted. Hyperreal source, representation and caches are unchanged.
The existing Hypersolve product remains in production pending consumer migration;
its copied candidate has independent exact and dispatch qualification here.

Final checks after production edits:

- 51 compiler/feature/test/doc/lint/cross-target commands passed on stable,
  Rust 1.95 host and Rust 1.88 primitive/dyadic subsets. Eight cross-target
  checks cover WASM and ARM. The initial matrix's two benchmark-only Clippy
  failures were fixed with a function-pointer type alias; original logs remain.
- 32 conformance/arithmetic tests and three compile-fail documentation checks
  pass with all features. Rational products cover 784 ordered interval pairs
  against an integer numerator oracle. Independent GMP tests cover larger
  fractions; directed 512-bit MPFR bounds exercise pi, sqrt(2), rational points,
  signed rational scaling, ball radii, coarse/fine/warm and concurrent use.
- Product dispatch/reducer snapshots match in 450 windows from independent
  fresh processes. Earlier attempts sharing per-value or interned Rational
  cache state correctly failed and are retained; counts were not relaxed.
- Directed dyadic division covers 50,274 reduced-grid scalar cases; exact
  rational replay, outward coarsening, extreme exponent reservations, typed
  work/size failures and fresh-budget retry pass. The bridge's conversion
  budget does not bound Hyperreal evaluation or arbitrary rational arithmetic.
- Final ASan campaigns passed 10,000 constructor, 100,000 fixed arithmetic and
  100,000 dyadic arithmetic runs (seed 20261112; 30-second campaign caps).
  Fuzz lint/format, workspace formatting and whole-repository Zizmor passed.
- Production LLVM coverage: 1,161 / 1,203 lines (96.51%), 2,155 / 2,269 regions
  (94.98%), 179 / 181 functions. Branch instrumentation was not enabled.
  The plan's full coverage gate is still open.
- Actual Q30 differential client passes unchanged. Fixed-only normal dependency
  closure remains exactly Hyperinterval. The final comparison executable has
  346,782 text / 16,208 data / 296 BSS bytes, identical to the phase-2 artifact.
  Host-only error variants are feature-gated after the initial build revealed
  a 384-byte primitive text increase. Paired fixed cost ratios remain about
  0.906 add/subtract, 0.594 multiply, 0.757 divide, 0.798 square versus legacy.
- The host product cost probe retains both initial and inlined samples; see
  the crate performance ledger. Final wrapper symbols are 1,085 bytes shared
  and 1,098 legacy; this is not a full backend or solver binary comparison.
  Timing differences around one percent do not establish the consumer gate.

The first independent GMP/MPFR attempt exposed an oracle adapter error: parsing
Hyperreal's mixed-number Display as a plain fraction. The corrected adapter
imports exact numerator, denominator and sign; all original fixtures remain.
MSVC cannot build GMP, so its configured jobs retain the portable integer and
dispatch tests while directed GMP/MPFR tests run on Unix. Hosted Windows/macOS
jobs, firmware hardware timing and full downstream migration remain unexecuted.
No repository has been pushed and the full 15-phase goal remains active.

## Immutable phase-3 carrier evidence

- Source checkpoint: `caa8adb9f`.
- Evidence local commit: `58e3c747cfa0d9906bcdd39a80f633b93d8d50da`.
- Manifest row: `docs/evidence/2026-11-12-interval-host	502	21558032	1c159f57964f1805b30df2310135cb66c775e9ab12f97a405de4c2d9afb6a1ca`.
- Retains final and failed checks, final sanitizer corpora, instrumented coverage,
  fixed/host cost samples, hardware/compiler metadata and reconstruction patches
  for rejected production candidates. Build trees are excluded.
- Verified all three interval records together with the existing scoped archive
  implementation. Previously registered evidence rows remain unchanged. Local
  archival does not claim hosted publication or full-plan completion.

## Rational Horner migration admission

Owner: Hyperinterval arithmetic; Hypersolve retains polynomial/root proofs and
policy/report ownership. Properties P1/P4/P6/P10/P19/P35/P36. Input domain:
finite Real coefficient lists whose consumed coefficients have exact rational
representations, over ordered rational endpoint pairs. Empty lists, reversed
bounds and a nonrational coefficient preserve their existing `None` fallback.

Targeted paths are algebraic rational polynomial enclosure and the polynomial/
derivative zero-exclusion filters in root_isolation. Each already checks input
order exactly once. That comparison can construct the shared carrier without
adding a decision. The loop must keep coefficient extraction, product, addition,
and final sign checks in their current order. Symbolic Real adapters and their
policy-owned endpoint comparisons remain unchanged in this increment.

Missing witnesses before edits: differential original/new Horner results and
cold/warm dispatch for both successful and declined rational paths; identical
public root reports under STRICT and APPROXIMATE_512; matched owner/downstream
cost and allocation/size evidence. Baseline checkout is detached at `8f5c14723`
in `/mnt/development/csgrs-interval-baseline`; raw staging is
`/tmp/interval-refactor/horner`. No migration is accepted from source sharing
alone; a failed gate requires a fix or rejection of the candidate.

## Rational Horner migration experiment: rejected

The three candidate paths reproduced the original results on 702 rational,
signed, reversed, degenerate and symbolic-fallback inputs. Independent process
runs matched 4,212 cold/warm dispatch windows, including rational reducer
counters. The full 11-workload Hypersolve dispatch report was byte-identical
(26,002 bytes). The public client matched report fields, native exact endpoint
JSON and allocation measurements under STRICT and APPROXIMATE_512 (13,360
output bytes). The first candidate's complete all-target/all-feature test run
also passed; that does not qualify its cost.

The migration failed the size gate. Experiments included general intervals,
nonempty owned/borrowed forms, scalar translation, a shared Horner stream and
inlining alternatives. A same-path, same-dependency, production-mode control
still grew from 2,069,474 to 2,073,622 summed function bytes, and from 3,960,744
to 3,969,944 executable bytes. Initial separate-checkout figures and all adverse
variants are retained. Focused production timing was mixed: the degree-16 row
was about 0.918x baseline, while the exact-rational-point row measured about
1.154x. These finite samples do not justify a universal timing claim; the
independent size failure is sufficient to reject retention.

Restored the original production routines and Hyperinterval API. The nonempty
variants and experimental shared Horner API are not shipped. Hyperinterval is
only a Hypersolve dev-dependency for the retained candidate tests. Kept the
shared generated corpus, frozen baseline, exact/dispatch candidate comparisons,
and an optional public allocation/native-report probe reusing the existing
allocator. Four inventory rows now have body/caller review and an evidence-
based rejected disposition; the other lexical rows remain unaudited.

No tests, dispatch counts, corpus resolution or gates were relaxed. The next
kernel migration needs a wider cost-qualified boundary or a lower-overhead
implementation; moving a loop through an owning interval object is not yet
accepted. Phase 3 kernel relocation and phase 7 production migration remain
open. The baseline successive-operations executable was built, but paired
successive-operation timing was not run after rejection; no downstream
performance gate is claimed passed for the rejected implementation.

After restoration, the same-path production client again has exactly 2,069,474
function bytes and 3,960,744 executable bytes, with identical report/allocation
output. Executable SHA-256 differs, so byte-identical binaries are not claimed.
The normal Hypersolve dependency tree contains no Hyperinterval edge; its new
edge is test-only. Retained candidate tests pass with and without features and
on Rust 1.95. Strict all-target Clippy, the existing fuzz-target build, default
allocation probe, workspace formatting and whole-repository Zizmor pass. The
production functions were also compared byte-for-byte with the starting source;
only cfg(test) module declarations are appended to those files.

Final retained revision: `cargo test --locked -p hypersolve --all-features --lib`
passes all 627 tests. The final 702-case assertion and dispatch differentials
pass, as does `cargo clippy --locked -p hypersolve --all-features --all-targets
-- -D warnings`. No production solver migration is accepted.

Raw evidence: `docs/evidence/2026-11-12-interval-horner`, evidence commit `90cb9af26b9fcc14f869b330a34ae500ff1ad322`; 280 files, 6333332 bytes,
record SHA-256 `eb13db0fbfcad7f4a7b192e861d5df66bd85bbf53d47a8a307309af21993919a`. Retained implementation/test source
is commit `d70c037b8`. The archive includes rejected patches and adverse samples.

## Implementation and open gates

- [API/feature proposal](docs/research/interval-api-design.md) preceded the
  skeleton. `interval-inventory.tsv` contains 1498 lexical candidate rows,
  with provisional disposition owners; these are not 1498 audited bodies.
- Hyperinterval phase 1 implements sealed primitive carriers, checked ingress,
  explicit emptiness, named relations, sign sets, domain/error types and serde.
  No existing scalar arithmetic, cache or predicate sequence was modified.
- `IntegerRange` is discrete. `FixedInterval` is continuous with dyadic lattice
  endpoints. Generic machinery is private and cannot certify arbitrary
  downstream endpoint types.
- The explicit-empty fixed carrier is larger than Alumina's current nonempty
  wrapper (12 vs 8 bytes for i32 on this host). Preserve the domain layout when
  evaluating delegation; do not silently accept that cost in firmware.
- Full source-map body/consumer audit, wider baseline fuzz, matched whole-stack
  binary and WASM/firmware size measurements, HIL and downstream performance
  gates remain open.
  No phase-0 completion or downstream migration is claimed.

## Dyadic measurements and unary images: admission

Owner: Hyperinterval, plan §3.5/§3.9; P1, P3, P5, P6, P18–P20.
At source `f69ee758c`, DyadicInterval has no width, midpoint, radius,
pointwise square, reciprocal or point-containment APIs. Existing exact add,
subtract, product and directed division remain unchanged. Input domain: empty
or finite normalized dyadic intervals with signed 32-bit stored exponent;
queries and new images share the caller-owned DyadicBudget. Width, midpoint
and radius return explicit singleton dyadic intervals (empty for empty input),
without implicitly selecting a point from an enclosure. Midpoint and radius
are exact; a value below the minimum representable exponent fails typed.
Square preserves the dependence of repeated operands and is nonnegative across
zero. Reciprocal uses caller-selected grid spacing and rejects zero.

Missing witnesses before implementation: independent integer expectations for
all small ordered intervals and several scales; zero-crossing square vs.
independent-product distinction; minimum-exponent exact halves and unrepresentable
halves; normalization at exponent limits; budget exhaustion followed by retry;
consumption through existing arithmetic, set relations, bridges and native replay.
This is a new core capability, not a production consumer refactor. No existing
solver predicate order or policy behavior is to change.

## Dyadic measurements and unary images: local acceptance

Implemented exact singleton width/midpoint/radius, endpoint access and point
containment, zero-aware square, and caller-grid reciprocal. New exact results
normalize temporary exponents across storage boundaries when representable;
required mantissa expansion is charged before allocation. Nonrepresentable
halves and squares fail with Overflow, not a rounded substitute. Existing
arithmetic and solver dispatch bodies are unchanged.

Independent small-integer expectations cover 1,710 intervals (190 endpoint
pairs × 9 scales), with 35,910 finer-grid containment cases and 5,670 reciprocal
images. A 512-bit case uses a binomial identity for the square. Measurements
reconstruct their original intervals using existing arithmetic. Rational
bridges and checked serialization consume generated results. Small-budget
cases include exhaustion after successful products but before normalization,
then successful retry with fresh budgets.

Final all-feature run: 36 tests plus 3 compile-fail doc tests pass. Strict
all-target Clippy passes. The recorded matrix has 13 passing commands across
Rust 1.88/1.95, feature subsets, docs, fuzz-target compilation, workspace format,
Hypersolve's retained migration corpus, and no_std ARM/WASM checks. Final
Rust 1.88 dyadic tests and format pass after the last test addition. Expanded
ASan dyadic campaign: 100,000 runs, seed 20261112, no failures. An initial
missing-PATH invocation and a test type-inference compile failure are retained;
both corrected runs passed.

LLVM coverage reports 479/483 dyadic lines (99.17%); no branch counters were
emitted. The whole-crate 100% gate remains open. No speedup, allocation-byte
bound, universal completion or downstream migration claim is made. Integer
powers and other §3.5 gaps, the rejected solver migration, broader cost gates,
and all later plan phases remain work to do.

Raw dyadic acceptance evidence: `docs/evidence/2026-11-12-interval-dyadic-measures`, evidence commit
`79ddc1c4c6f6a61f90e197b34ab831567bfe69af`; 136 files, 25363884 bytes,
SHA-256 `67dd6f9fbaef47c17ea5a15a049854f00d95d8841c954c7689f62066e924db62`.

## Dyadic nonnegative integer powers: admission

Owner: Hyperinterval; plan §3.5/§3.9, P1/P3/P5/P6/P18–P20.
At `054342332`, the dyadic carrier lacks integer powers beyond square. Add
`checked_pow(u32, &mut DyadicBudget)` consistently with existing primitive
carriers. Exponent zero maps every nonempty set to one (including zero);
empty maps to empty. Odd powers are monotone, even powers have a zero minimum
when the input contains zero. Negative powers remain explicit composition
with caller-grid reciprocal. No new rational-power API is included: Hyperreal's
existing eager power has its own size failure which must not be silently
relabelled or bypassed by a future adapter.

Before editing, missing witnesses are exact integer endpoint/critical-point
images, extrema and unit/zero cases at u32::MAX, exhaustion before BigInt growth,
minimum/maximum temporary exponents, and consuming powered output through
regrid, reciprocal and exact replay. Existing square and product remain native
specializations; no production consumer or predicate dispatch changes.

## Dyadic nonnegative integer powers: local acceptance

Implemented exact dyadic `checked_pow` with endpoint parity and zero-aware
bounds, reusing square for exponent two. Nontrivial endpoint powers reserve
`input_bits * exponent` bits and a conservative bounded number of quadratic
stages before BigInt exponentiation. Unit/zero mantissas have an explicit
constant-work path; the inspected BigInt primitive-power implementation does
not supply that shortcut itself. The caller's budget also covers correction
of temporary exponents. Exhaustion never publishes a partial interval.

Independent integer expectations cover 6,370 images (91 endpoint pairs × 7
scales × 10 exponents), followed by outward regridding and reciprocal/product
consumption. Additional witnesses cover u32::MAX, exact unit and zero results,
size/work exhaustion and fresh-budget retry, normalization across both exponent
limits, and serialization of generated seventh powers. Full-feature tests pass
38 cases plus 3 compile-fail examples. All 13 recorded compiler/feature/doc/lint/
format/downstream/ARM/WASM checks pass after correcting the parity-style lint;
the original failing Clippy logs are retained. A final ASan campaign completes
100,000 runs with seed 20261113 and no failures. Dyadic line coverage is 522/526
(99.24%); whole-crate line coverage is 1305/1347 (96.88%). Branch counters remain
unavailable and the 100% coverage gate stays open.

No performance speedup is claimed. This adds only nonnegative powers;
negative-power composition requires the intermediate positive power to fit its
representation/resource contract. A direct negative-power enclosure and the
rational-power API are still missing. This does not complete phase 3 or any
consumer migration.

Raw power evidence: `docs/evidence/2026-11-12-interval-dyadic-powers`, evidence commit
`66472def3c8be87872c46633cdd64d777c81536a`; 167 files, 45423439 bytes,
SHA-256 `2f6c6073c3eda9ea08139238bdf070de4fda93f8608eea78c0299d7189bd8f2a`.

## Primitive proposal layer: admission

Owner: Hyperinterval, plan §3.11 and phase 4; P3/P5/P6/P8/P9/P19.
At `6bd4b882c` no proposal or strict-float module exists. Add allocation-free,
no_std F32ProposalInterval/F64ProposalInterval behind `float-proposal` with
finite ordered endpoint estimates and explicit empty state. Primitive arithmetic
may propose new estimates, but cannot establish containment, exclusion, scalar
sign, or any certified status. No conversion from proposals to an authoritative
interval is provided. Exact source replay remains the only admission path.

Ingress validates finite bit patterns and numeric order with integer bit keys,
canonicalizing signed zero as the same set endpoint. NaN/infinity input fails
with NonFiniteEndpoint; an overflowing estimate fails with Overflow; strict
proposal division rejects a zero-spanning denominator even with an empty
numerator. Arithmetic is ordinary native floating point, without an outward
rounding claim. The method returning a midpoint estimate may be affected by
underflow and is not an implicitly selected exact point.

Missing acceptance witnesses before implementation: finite/special-value input,
separate zero/empty/error states, the inability to pass proposal values into
certified APIs or call containment/sign methods, checked replay, arithmetic
composition staying in proposal types, no dependencies/allocation, and Rust
1.88 host/ARM/WASM builds. Shared private Bounds storage is reused. A private
IEEE ordering helper is needed without Hyperreal: its existing verified decoder
is crate-private and the proposal subset must not depend on that host crate.
No Hyperreal code, scalar cache or consumer dispatch is changed.

## Primitive proposal layer: local acceptance

Implemented the two separate proposal wrappers with finite bit-validated
construction, canonical zero, explicit empty state, native sum/difference/
product/quotient/affine estimates, reflected estimates and an explicitly
approximate midpoint. Equality compares stored canonical bits. No containment,
exclusion, sign-set or certification conversion API was introduced. Native
bit-pair serialization reconstructs through checked ingress. No default feature,
Hyperreal source/layout/cache or consumer operation changed.

Final checks: all 46 tests and 9 compile-fail examples pass with all features.
The 17-command final matrix passes Rust 1.88/1.95 feature combinations, strict
linting, docs, fuzz-target check, formatting, retained Hypersolve migration
witnesses, ARM/WASM no_std builds and repository-wide pedantic zizmor. Separate
proposal-only dependency output contains just Hyperinterval. The final ASan
campaign completes 100,000 cases with seed 20261113 and no failures. Native
IEEE ingress tests cover 20,000 generated pairs plus named special values;
typed errors distinguish nonfinite ingress, reversed estimates, zero-spanning
denominators and arithmetic overflow. A known rounded sum intentionally
excludes the exact result, making the lack of certification concrete.

On this x86_64 host, proposal layouts are 12 bytes (f32) and 24 bytes (f64).
The standalone release probe completes 30 samples of 16,384 add/affine steps
per width and consumes both final estimates. Observed medians are 83.85 ns/step
(f32) and 67.73 ns/step (f64); these are workload observations without a speedup
or cross-target performance claim. The recorded executable includes its host
benchmark harness, not just kernel code. LLVM line coverage is 103/104 for
proposal.rs and 15/15 for ieee.rs; branch counters remain unavailable. The
whole-crate coverage gate remains open. Strict finite float enclosures are not
implemented or certified by this change.

Raw proposal evidence: `docs/evidence/2026-11-12-interval-proposals`, evidence commit
`102f91c01a32fe0123aa3571ed96efaad32a6251`; 158 files, 23812789 bytes,
SHA-256 `a742c2dbb1ee6d0f977758fb768fe235c8acaa5eb6977f2b15cdf4790fe49669`.

## Strict finite float layer: admission

Owner: Hyperinterval; plan §3.12/phase 4, P1/P3/P5/P6/P8–P11/P19.
At source `385006d68`, the strict float types and arithmetic do not exist.
The algorithm and missing witnesses are recorded in
`docs/research/interval-float-backend.md`. Implement an allocation-free integer
IEEE backend behind independent `float-enclosure`, initially admitting
nonempty public inputs only on x86_64. Other architectures must reject with
UnqualifiedEnvironment; cross-compilation alone will not qualify them.
Native float arithmetic remains confined to the separate proposal module.

Domain: canonical empty or closed finite f32/f64 endpoint sets; exact finite
IEEE ingress, explicit NaN/infinity rejection, signed-zero canonicalization,
integer-based order and outward basic arithmetic. Overflow cannot publish
saturated finite bounds. Denominator validation precedes empty-numerator
shortcuts. Primitive work is statically bounded; integer powers use at most
32 squaring and 32 product stages per source endpoint, each evaluating both
directed bounds. No ambient modes, caches, hidden
refinement, standard transcendental or proposal promotion are admitted.

Missing witnesses include reduced-format exhaustive exact oracles, real IEEE
edge and random exact-rational oracles, cancellation and underflow, operation
chains/replay, compile-fail authority tests, isolated rounding-mode/FTZ/DAZ
checks, emitted-code inspection, feature/MSRV/lint/sanitizer/target checks,
and measured completed workload costs. No existing consumer/predicate path
is to change, and this record is not evidence of completed implementation.

### Implemented core and qualification

`float-enclosure` now provides independent allocation-free F32/F64 enclosures,
with exact IEEE ingress, canonical empty, set relations, directed basic
arithmetic, fused affine images, nonnegative powers, expansion, validated bit
replay and optional exact rational/dyadic bridges. Only x86_64 admits nonempty
inputs. Integer basic arithmetic and affine images are tight; powers may widen
and encounter intermediate overflow. The affine kernel computes the product
before rounding, so `MAX * 2 - MAX` succeeds. Its width/tail proof is in the
backend design; no native transcendental or proposal promotion is introduced.

Qualification reproduced a release-only failure: LLVM folded an inlined
integer zero check into `ucomisd`; DAZ then made a nonzero subnormal compare as
zero. The original failing release test, disassembly, source patch and snapshot
are retained. Keeping the shared finite validator out of line fixes the
reproduced case. This corrects proposal ingress too; the earlier proposal
record's normal-environment results did not establish DAZ independence.

The final 27-command matrix passes on x86_64 Linux: Rust 1.88/1.95/stable
feature tests, 67 all-feature unit/integration tests and 11 doctests, release
float tests, all-target checks, strict Clippy, docs, workspace formatting,
whole-repository zizmor, repository budget, fuzz builds, four downstream
Hypersolve value/dispatch witnesses, dependency isolation, and ARM/WASM no_std
compilation. No other consumer kernel or dispatch path changed. The independent
integer oracle exhausts 57,121 reduced-format pairs for three operations and
166,375 fused affine triples. GMP/MPFR oracles cover named IEEE boundaries,
20,000 random pair candidates per width, interval corner extrema, fused affine
images, and exact-power containment. Invalid, overflow and zero-denominator
outcomes are checked separately from successful bounds.

Isolated child processes exercise all 16 combinations of rounding, FTZ and
DAZ for both widths in debug and release, including exact endpoint bridges.
Fat-LTO release tests also pass. Emitted-code scans find no native floating
arithmetic/comparison in the selected kernel/client functions: 55 symbols on
1.88, 56 on 1.95, 55 on stable 1.99, and 39 with fat LTO. The scan excludes
runtime/panic code and is compiler-specific evidence. These checks do not
qualify arbitrary flags, fast-math, GPU execution, macOS/Windows runtime, or
ARM/WASM execution. CI contains the affected hosted checks; none were pushed.

The ASan strict-float campaign completes 100,000 runs, seed 20261114, maximum
input 24 bytes, 30-second limit (12 seconds used), with no failure. The old
two-input corpus remains intact; the new corpus seeds its third operand with
zero. All-feature production coverage is 1,913/1,960 lines (97.60%) and
3,611/3,770 regions (95.78%); float.rs is 239/242 lines and ieee.rs 265/266.
Dependencies, tests, conformance probes and Hyperreal are excluded. LLVM reports
no branch denominator. Its one profile mismatch is identified as Rug's excluded
`OptRational::parts`, not a production Hyperinterval function. The 100% gate
remains open.

The completed strict chain materializes [4/5,1] outward after affine, square,
add and reciprocal. Thirty samples of 16,384 chains have median 574.681 ns
(f32) and 659.598 ns (f64); layouts are 12 and 24 bytes. The correctness barrier
has an adverse cost: three alternating process rounds of the unchanged proposal
probe, 30 samples/round, measure 83.475→95.786 ns (f32) and 67.681→99.816 ns
(f64), with identical output checksums. This is a retained correctness fix,
not a no-regression refactor. Raw samples, build flags and executable hashes
are retained; these focused host measurements are not firmware or general
performance guarantees. See the crate PERFORMANCE record for binary sizes.

Implementation commit: `8b5bddf7633820cac4302b2eaa2c87b019091a0b`. Raw evidence
is archived as `2026-11-12-interval-strict-float` in evidence commit
`3ca69e4a571a032b05dfb3a34dd608c015819ab1`: 601 files, 162,500,126 bytes,
SHA-256 `42f82b7130db558e55076bf0b2d43a581134e3d4c85208feca6fc606a55ed8fd`.
Scoped digest verification passes; all earlier manifest rows are preserved.
Both commits remain local. This checkpoint does not complete phase 4 or the
plan: direct negative powers,
remaining common measurements/conversions, full coverage, further targets,
consumer migrations and distribution qualification remain open. Existing
solver migration remains rejected under its unchanged size gate.

## Elementary absolute value and order images: admission

Owner: Hyperinterval; plan §3.5/§3.13, P1/P3/P5/P6/P19/P35/P36. At
`ab528b090`, no finite carrier exposes `abs`, `min_image` or `max_image`.
The domain is canonical empty or valid finite integer/fixed/rational/dyadic/
qualified-IEEE intervals. Absolute value is a unary point image; min/max are
binary images over independent operand sets, so either empty operand yields
empty (unlike hull). These functions are total on real scalar values and
require no domain clipping. Overflow remains explicit when a signed primitive
minimum's absolute value cannot fit. Dyadic alignment and retained results
must share the caller's existing budget, with no partial output on exhaustion.

Reuse private Bounds ordering and the existing checked primitive reflection,
IEEE sign/ordering, and dyadic alignment/accounting. Add missing independent
enumeration and exact-rational image witnesses, extreme endpoints, unequal
dyadic grids, budget exhaustion/retry, output-as-input/replay and altered-mode
IEEE checks. Do not reinterpret uncertain symbolic balls or proposals as exact
intervals. Existing arithmetic dispatch and consumer kernels are unchanged.
Rational/signed-power completion, square roots and the rest of phase 5 remain
required; this is the first ordered elementary-function extension.

### Implemented images and evidence

All finite carriers now expose these three operations. For nonempty
`[a,b]` and `[c,d]`, monotonicity gives
`min_image = [min(a,c),min(b,d)]` and
`max_image = [max(a,c),max(b,d)]`. Continuity over the real rectangle establishes
attainment of the intermediate values; discrete integer ranges also attain
every integer between these endpoints. Absolute value reflects a negative
interval, retains a nonnegative interval, and maps a zero-crossing interval to
`[0,max(-a,b)]`. Fixed/IEEE endpoint reflection is exact when representable;
dyadic common-grid alignment uses a positive power-of-two factor and does not
round. Shared Bounds helpers own min/max endpoint selection. No existing
arithmetic or predicate call sequence was edited.

The new witness initially fails to compile against `ab528b090` because the
methods are absent. The completed witness enumerates 46 interval states
(including empty), 2,116 operand pairs per instantiated family, and every
integer point pair in those small intervals. It tests all six instantiated
families (integer, fixed, rational, dyadic, f32 and f64), generated results as
operands, exact squares after absolute value, and deterministic serde replay.
Additional cases cover all signed/unsigned primitive widths, fixed scale
extremes, signed-minimum overflow, finite IEEE extremes/subnormals, 2,000
mixed dyadic-grid cases, exponent extremes, reservation failure and fresh-budget
retry. There is no clipping or approximate completion in this corpus.

The final 32-command matrix passes: 77 unit/integration tests, 11 doctests,
Rust 1.88/1.95/stable feature combinations, strict Clippy, docs, workspace
formatting, fuzz builds, repository budget, zizmor, dependency isolation,
ARM/WASM no_std builds, release float mode tests and the four unchanged
Hypersolve value/dispatch witnesses. Three additional release order-image
commands pass on the same compilers. The library stays unsafe-free. The
existing isolated IEEE mode tests now include point and interval absolute
images and min/max images; all rounding/FTZ/DAZ combinations pass, including
fat LTO. The code scanner includes emitted Bounds functions as well as
float/IEEE functions: 63, 64, 61 and 45 selected symbols on Rust 1.88, 1.95,
stable 1.99 and fat LTO, respectively, with no native float arithmetic or
comparison found in that scope. Other runtime targets remain unqualified.

Fixed, dyadic and strict-float ASan campaigns each complete 100,000 runs with
no failure. Seeds are 20261112/20261112/20261114, input caps 16/14/24 bytes,
30-second limits, actual durations 1/13/14 seconds. Existing assertions and
prior corpora are preserved; the strict-float campaign starts from its prior
corpus. Production coverage is 2,052/2,097 lines (97.85%) and 3,891/4,054
regions (95.98%). Dependencies, tests and cost probes are excluded; LLVM has
no branch denominator. Its excluded Rug profile mismatch remains identified
in the diagnostic dump. Rust 1.99's normal report/export succeeded, but its
optional `llvm-cov --dump` diagnostic hit an internal memory-corruption error
and was terminated after emitting the failure. A separately instrumented Rust
1.95 build passes report, export and diagnostic dump, reproducing exactly the
same production line/region counts and identifying only excluded
`Rug::OptRational::parts` as the mismatch. Both tool outputs are retained;
the LLVM diagnostic defect is not reported as a successful check. The 100%
target remains open.

The completed `abs → min_image → max_image` probe uses preconstructed inputs,
4,096 warmup calls, 30 samples of 16,384 chains per family, and owns/drops the
materialized result in the timed region. Median ns/chain are 8.666 (integer
i64), 6.384 (fixed i64/16), 153.348 (rational), 676.350 (dyadic), 27.993 (f32)
and 21.078 (f64). Family grids/layouts differ; these host observations are not
a cross-family speedup, baseline-regression or firmware timing claim. Raw
samples, compiler/hardware/build flags and executable identity are retained.
Source distribution, broader consumer migrations, rational/signed powers and
all subsequent elementary functions remain unfinished.

Implementation: `435be1e85b2ffa884b08e50da6ac2d1f66edfd91`. Evidence record
`2026-11-12-interval-elementary-order` is archived at
`6e447153d623d62cddfd52717f2e9a148d07520b`: 477 files, 102,465,598 bytes,
SHA-256 `a5b6b9d2b1acb8f209179afcf948f6ef38ecb7bc3df81bb7e79ef13c4fad896e`.
Scoped verification passes and every prior manifest row is preserved. Both
commits are local; no push occurred.

## Rational integer powers: admission

Owner: Hyperinterval set images over Hyperreal rational arithmetic; plan
§3.5/§3.13, P1/P3/P5/P6/P19/P35/P36. At `d45d4358c`, RationalInterval lacks
integer powers. Add `checked_pow(i64)` for exact signed-integer images: empty
maps to empty, exponent zero maps nonempty input to one, positive even powers
retain the zero critical point, and negative powers reject any zero-containing
input. Other negative-power images use monotonicity on the admitted nonzero
interval. No pointwise dependency is replaced by repeated interval products.

Reuse Hyperreal's public Rational::powi kernel and existing interval square/
reciprocal specializations. Preserve kernel errors in a rational-feature
IntervalError variant with the original Problem available through source().
Do not convert Exhausted into empty, approximate geometry, unsupported or
undecided. Hyperreal's current eager-power guard is inherited: exponents up
to magnitude 255 have bounded amplification relative to the input; larger
nontrivial powers are admitted only when their estimated combined numerator/
denominator size does not exceed 65,536 bits. This is a fixed scalar growth
policy, not a new caller-selected work,
heap-byte or cancellation budget. Existing input normalization and rational
allocation contracts remain in force. Stress tests must be externally bounded.

Missing witnesses: positive/negative parity and zero-domain images against
independent exact expectations, i64 exponent extremes on zero/unit/empty sets,
the actual native guard boundary and retry, preserved typed source errors,
generated-output composition/replay, feature isolation and affected downstream
value/dispatch checks. Do not change Hyperreal's guard or existing consumers.

### Implemented signed powers and qualification

`RationalInterval::checked_pow(i64)` now returns exact connected images or a
typed failure. It reuses square/reciprocal for exponents 2/-1, handles 0/1
directly, and evaluates a singleton only once. Positive odd powers increase;
positive even powers decrease on the negative half-line and increase on the
positive half-line. Negative odd powers decrease on either admitted half-line;
negative even powers increase only on the negative half-line. These facts
order the generated endpoints without an output comparison. Zero is the sole
interior critical point for the admitted positive powers.

Review reproduced an initial completion failure: `[-1/3,1]^(i64::MAX-1)` hit
the scalar guard while powering an endpoint that cannot determine either
output bound. The corrected positive-even zero-crossing path selects the
largest absolute input endpoint before exponentiation, then returns zero as
the minimum. Both mirrored unit-range cases now complete as [0,1]; cases
requiring a nontrivial enormous upper bound still return the native exhaustion.
The original failing test, source snapshot and patch are retained.

`IntervalError::Scalar(Problem)` retains the original typed cause and exposes
it through Error::source; no scalar failure is flattened into false or empty.
The variant is gated by the existing rational feature, so it adds no Hyperreal
dependency to primitive builds. The GMP conversion helper was moved unchanged
from the Real-bridge test to shared test support; all original assertions remain.

The final 26-command matrix passes, followed by the strengthened guard fixture's
full tests, release tests and Clippy: 84 unit/integration tests and 11 doctests;
Rust 1.95/stable rational combinations and release powers; Rust 1.88 primitive
feature tests; strict lint, docs, formatting, fuzz harnesses, whole-repository
zizmor, budget/dependency checks, ARM/WASM no_std compilation, and the four
unchanged Hypersolve value/dispatch witnesses. The guard-boundary fixture also
passes inside a 60-second external limit. No production Hyperreal or consumer
kernel was changed, and no hosted OS or distribution claim follows.

Independent GMP checks enumerate 364 small rational intervals and exponents
-9 through 9, plus mixed denominators and coefficients beyond primitive width.
They check exact endpoint extrema and domain failures, not floating estimates.
Additional witnesses cover empty and i64 exponent extremes, generated chains,
native replay, outward dyadic conversion followed by further powers, and retry
after refusal. For exact 2, ±21,845 completes and ±21,846 reaches the native
guard. A separate 2^400 seed raised to 255 completes a 102,001-bit numerator,
confirming that the existing modest-exponent amplification rule is not falsely
documented as a universal 65,536-bit output cap. Failed second endpoints publish
no partial interval and do not poison a later successful operation.

The final ASan campaign completes 100,000 runs in 14 seconds, seed 20261115,
10-byte cap and 30-second limit. It retains the earlier 100,000-run campaign,
its corpus, and directed seeds for large even exponents. Ordinary fuzz powers
are limited to magnitude 32; huge-exponent oracle cases use exact zero/unit
facts and the native guard instead of asking GMP to allocate enormous values.
The host-only rational fuzz feature is separate from the primitive harness.

Rust 1.95 instrumentation reports 2,101/2,145 production lines (97.95%) and
3,981/4,142 regions (96.11%). Dependencies, tests and cost probes are excluded;
LLVM reports no branch denominator. Report/export and diagnostic dump complete,
identifying the known excluded Rug OptRational::parts mismatch. Full coverage
and a cumulative rational work/heap/cancellation budget remain open.

The exact `power(3) → power(-2)` cost probe uses preconstructed positive,
negative and singleton inputs, 4,096 warmups, and 30 samples of 8,192 completed
chains per case. It consumes owned exact endpoints and checks independently
derived rational expectations before timing. Final median ns/chain are 561.891,
548.480 and 316.054 respectively on Rust 1.99 x86_64 Linux. Initial samples,
final samples, layouts, executable hashes and whole-executable sizes are
retained. This is a host completion probe, not a speedup or firmware claim.

Source commit: `0a5a732635019abbf4fee1dfcec6e19edba87aa8`. The separate
`2026-11-12-interval-rational-powers` record is committed in the evidence
repository at `6c27df18cfffd259fabd676662cd8d6d756f4ae4`: 357 files,
111,203,749 bytes, SHA-256
`ec666d5527171ad4dbea70f60501277d16a43b49e05d23dd48095e360698820d`.
The scoped manifest was verified against that committed revision and registered
in `docs/evidence-archive.tsv`; prior records are preserved. Raw staging remains
at `/tmp/interval-refactor/rational-powers`. Remaining signed-power surfaces,
square root and subsequent elementary functions, consumer migrations, full
coverage and release qualification are still required.

## Square-root admission (2026-11-12)

Owner: Hyperinterval, P1/P3/P5/P6 and P19. Target revision before edits:
`79fa33f6f`. The missing `sqrt` witness was reproduced as a compile failure.
Domain: finite rational or dyadic closed real intervals, canonical empty,
all signed i32 output-grid exponents, principal nonnegative root. Rational
inputs need rational+dyadic features. Output is a `RangeEnclosure` of a dyadic
interval, with Complete coverage for nonnegative/empty sources, Partial when
negative source members are excluded, and EmptyDomain for a strictly negative
nonempty input. No existing consumer dispatch or scalar kernel was changed.

For positive n/d after exact binary scaling, let q=floor(n/d). Then
floor(sqrt(n/d))=floor(sqrt(q)); the upper endpoint is the same exactly when
n/d is integral and q is a square, otherwise it is one grid unit higher.
Monotonicity on the clipped nonnegative domain selects endpoint images. The
shared existing scaled-quotient helper and num-bigint integer square root
supply these operations. No rounded source interval substitutes for the input.
Num-bigint may use a float proposal for a Newton seed when its std feature is
unified; integer iteration establishes the result. That proposal is not an
accepted endpoint certificate.

Reservations precede operand cloning/scaling and root extraction. Root stages
charge `(quotient_bits+2)^2 * (2*bit_length(quotient_bits+2)+4)` deterministic
work units, in addition to existing linear ingress and scaled-division costs.
This is the existing accounting contract, not a CPU-cycle, peak-byte or
cancellation guarantee. Domain checks and caller-owned input construction are
outside that accounting. Exhaustion publishes no partial interval, and fresh
budgets permit retry. Zero and empty avoid scale-sized allocations; nonzero
extreme grids can exhaust the declared size limit.

Final qualification in `/tmp/interval-refactor/square-roots` passes all 26
commands: 90 all-feature unit/integration tests and 11 doctests; the affected
Rust 1.88/1.95/stable feature/release/strict-lint matrix; docs, formatting,
fuzz checks, whole-repository zizmor and retention budget; unchanged solver
value/dispatch witnesses; primitive dependency closure; and ARM/WASM no_std
compilation. The first matrix exposed an unused rational-only test import in
a dyadic-only build; feature-gating that import preserves every assertion.
Initial compile diagnostics and all matrix results are retained. No hosted
Windows/macOS or release-distribution completion is inferred.

The first ASan campaign completed 100,000 cases in 28 seconds. After reserving
the dyadic denominator constant explicitly, the final-source campaign completed
92,929 cases in 31 seconds at its 30-second wall-time stopping condition, with
no failure (seed 20261116, 10-byte cap). Both corpora and logs are retained;
the final run is not reported as 100,000. Each generated case checks rational
and dyadic tightness, domain coverage, serialization and further root/square
composition using a shared independent oracle.

Rust 1.95 instrumentation reports 2,186/2,230 production lines (98.03%) and
4,123/4,289 regions (96.13%), excluding dependencies, tests and probes. No
branch denominator is reported. Report/export and diagnostic dump succeeded;
both mismatches identify excluded Rug OptRational::parts records, not a
Hyperinterval function. Full coverage remains open.

All 640 cost samples complete on Rust 1.99 x86_64 Linux: four input widths,
four grid exponents and both backends, each with 1,024 warmups followed by
20 samples of 1,024 owned outputs. Inputs and independent GMP/MPFR checks are
outside the timed loop; budget creation and output destruction are inside.
The full median table is in the crate performance ledger. No comparative
speedup, allocation-byte, firmware or consumer claim follows.
Independent exact GMP adjacent-grid inequalities and directed MPFR bounds cover
small rational/dyadic domains, large/near-square coefficients, subnormal-sized
values, partial domains, full exponent tags, resource refusal/retry, refinement,
validated serialization, repeated roots/powers and reciprocal composition.
Source commit: `0b9798d02c2df2591b13d055c599ff13d33d7c7d`. The separate
`2026-11-12-interval-square-roots` evidence record is committed at
`c068701b4f8f7bc02b7ed3f25ca1c33c2207cedd`: 718 files,
52,639,790 bytes, SHA-256
`25e0fcb80f98959c97140cf1ce97c51fde53fd95165bde892dfd66941f5994b6`.
The scoped manifest was verified against that committed archive revision and
registered in `docs/evidence-archive.tsv`; all prior records are preserved.
Remaining elementary functions, primitive square-root backends, consumer
migrations, full coverage and release qualification remain open.

## Primitive square-root admission (2026-11-12)

Owner: Hyperinterval primitive scalar-set layer, P1/P3/P5/P6/P19. Baseline
`225daa5f4`; missing fixed/IEEE `sqrt` methods reproduced by compile failure.
Domain: every valid i32/i64 fixed interval at each supported scale, and finite
binary32/binary64 intervals under the existing x86_64 backend qualification;
canonical empty is accepted. Principal root clips negative source members with
Partial coverage, rejects strictly negative nonempty inputs with EmptyDomain,
and maps empty to empty with Complete coverage. Returned carriers remain
ordinary arithmetic, set-operation and validated replay operands. The domain
receipt remains explicit even when two ranges happen to be identical.

Both kernels share one primitive `u128::isqrt` floor/ceiling helper. For fixed
m/2^F, output-grid mantissas enclose sqrt(m*2^F); the radicand has at most 126
bits. A rounded upper mantissa can exceed signed storage at maximal F, which
returns Overflow rather than saturation. For IEEE m*2^e, h=floor(log2(m)) and
output-grid exponent g=floor((e+h)/2)-F, where F is the fraction width. The
scaled radicand m*2^(e-2g) has at most 2F+2 <= 106 bits. Every nonzero IEEE
root is normal, even for subnormal source values. Integer square testing
selects the exact ceiling; existing encoding authority builds the endpoints.
No new float comparison, arithmetic, allocation, arbitrary-precision
production dependency or ambient-mode mutation is introduced.

Work and storage are bounded by primitive widths and at most two integer root
calls; no configurable precision budget is needed. This is not a measured
firmware-cycle or stack-byte bound. Existing predicate order and consumer
kernels are untouched. The existing opaque finite-validation boundary remains
in place. Codegen qualification now requires six client symbols, including
both root widths, and scans emitted integer-root helpers as well as the
existing IEEE/Bounds and wide-division paths. No old exclusion or assertion
was weakened.

Qualification at `/tmp/interval-refactor/primitive-roots` passes the final
33-command matrix: 96 all-feature unit/integration tests and 11 doctests;
Rust 1.88/1.95/stable feature, release, strict lint and documentation checks;
formatting, fuzz harnesses, whole-repository zizmor and retention budget;
unchanged downstream solver value/dispatch witnesses; primitive dependency
closure; and ARM/WASM compilation. No hosted OS, HIL or release-distribution
claim follows. A final test-only cfg prevents unused imports on non-x86
float-only builds.

The acceptance corpus covers
all 96 fixed endpoint/scale combinations, exact GMP adjacent-grid witnesses,
all finite IEEE exponent classes, deterministic random images, directed MPFR
and exact GMP endpoint checks, maximal scales, zero/partial/empty domains,
replay and successive operation tests. Release MXCSR tests exercise all 16
combinations before comparing to independent expectations prepared in the
normal environment. The ASan campaign completed 100,000 cases in 12 seconds, seed 20261117,
16-byte cap, 30-second wall limit, with no failure. It validates eight fixed
endpoint/scale configurations, both IEEE widths, ingress failures, tightness,
domain qualification, serialization and subsequent squaring. Expected
fixed-scale Overflow and EmptyDomain are checked separately from completion.

Codegen scans passed on Rust 1.88, 1.95 and stable and under stable fat LTO;
the corresponding release mode tests pass for both IEEE widths and every
MXCSR combination. Coverage reports 2,240/2,284 production lines (98.07%) and
4,222/4,396 regions (96.04%). Tests, dependencies and probes are excluded;
there is no reported branch denominator. Diagnostic dump identifies three
excluded Rug OptRational::parts profile mismatches. Report/export/dump all
complete; full coverage remains open.

The cost probe completed all 960 samples across four source widths and eight
endpoint configurations, each with 4,096 warmups then 30 samples of 8,192
owned images. Independent exact checks precede timing. The crate performance
ledger contains the complete median table. No comparative improvement or
firmware cost is inferred.

Source commit: `af060436e8cbe3d1f47c52a62f4846ae07eebe70`. The separate
`2026-11-12-interval-primitive-roots` evidence record is committed at
`d1601f96173528144cfafd73770060f2b458dd70`: 273 files,
81,619,721 bytes, SHA-256
`9590b874c9e448e2ec4c3e0c7726294a51535d9b7f81ccea97ab479b8a9f3db7`.
The scoped manifest was verified against that committed archive revision and
registered in `docs/evidence-archive.tsv`; all prior records are preserved.

Later elementary functions, remaining signed-power surfaces, symbolic-ball
function adapters, consumer migrations, full coverage and release
qualification remain open. No phase completion is inferred from this addition.

## Exponential admission (2026-11-12)

Owner: Hyperinterval's interval evaluation layer, P1/P3/P5/P6/P19. Baseline
`97f0d1d0c`; the missing `exp` witness was reproduced before implementation.
Domain: finite rational/dyadic closed intervals, canonical empty, signed i32
output grids. Output is a Complete-domain dyadic range with tight outward
endpoint bounds. Size/work exhaustion or intermediate exponent-storage
Overflow is typed before publishing an image. No consumer kernel is changed.

Hyperreal's existing computable exponential has precision and abort controls,
but no cumulative work contract that a Hyperinterval wrapper can enforce.
The new interval algorithm has a distinct bounded-work contract and reuses
existing DyadicInterval arithmetic, scaled quotient, normalization and output
validation. It does not change or duplicate the computable graph machinery.
Rational exact numerators/denominators remain the authoritative source at each
refinement. A shared test-only exact GMP dyadic conversion was extracted from
the square-root oracle without dropping assertions.

For r in [0,1/2], each successive positive Taylor term ratio is at most 1/2;
after S_n the tail is at most twice t_(n+1). Directed arithmetic encloses the
argument, every term, the sum and this tail. Binary argument reduction and
repeated directed squaring reconstruct exp(abs(x)); reciprocal reconstructs
negative arguments. Success requires each endpoint enclosure, after output
regridding, to have adjacent grid bounds. Exp(0) is treated exactly. For
nonzero rational input exp(x) is not dyadic, so convergent refinement can
separate the grid boundaries, subject to resource/storage limits. No uncertain
predicate is treated as a successful bound.

Every attempt and nested arithmetic call shares the caller's DyadicBudget.
No refinement counter alone claims total work control; intermediate size and
cumulative deterministic work units are separately limited. There is no
cache or retained expression graph. These units are not heap bytes, CPU
instructions or cancellation support; those broader controls remain open.
For p<=0 and x<=p, exp(x)<=2^p; for p>=0 and x<0, exp(x)<1<=2^p. Exact
bit-length comparisons establish these negative tails without materializing
huge magnitudes. Very small nonzero arguments retain the correct side of one.

Two completion regressions were reproduced during review: retaining the unused
half of a point enclosure during interval assembly forced scale-sized growth,
and aligning a selected zero unnecessarily exhausted the budget at the minimum
grid exponent. Assembly now selects only the monotone extrema and constructs
zero-based results directly through the existing validated constructor. Both
failed logs and pre-fix source snapshots are retained alongside the original
extreme-scale witnesses.

The 26-command qualification matrix at `/tmp/interval-refactor/exponential`
passes: 101 all-feature unit/integration tests and 11 doctests, affected Rust
1.88/1.95/stable features/release/strict lint/docs, formatting, fuzz harnesses,
whole-repository zizmor and retention budget, unchanged solver value/dispatch
witnesses, primitive dependency closure and ARM/WASM compilation. Strengthened
large-rational and coarse-tail fixtures also pass final release, dyadic-only
MSRV and strict-lint checks. No hosted OS or distribution claim follows.

Independent directed MPFR grid bounds cover rational sources, values through
±1024, precision through -2048, coefficients beyond 1024 bits, tiny inputs,
empty/zero, small budgets and retries, refinement, serialization and further
operations. The maintained final ASan campaign completes 10,000 cases in
13 seconds, seed 20261118, 8-byte cap and 30-second limit, with no failure.
Its precision generator includes every grid exponent from -72 through zero;
earlier campaigns restricted that band to -72 through -8 and are retained.

Fresh Rust 1.95 coverage reports 2,388/2,432 production lines (98.19%) and
4,583/4,790 regions (95.68%). Tests, dependencies and probes are excluded;
LLVM supplies no branch denominator. Report/export/dump all succeed and all
four mismatch records identify excluded Rug OptRational::parts. Earlier
profiles remain separate, not mixed into the final report. Full coverage is
still open.

All 480 cost samples complete: four source widths, three grid precisions,
and both backends; 128 warmups and 20 samples of 128 owned images per group.
The crate performance ledger has the full median table. Raw samples also
retain deterministic work units, which are distinct from measured CPU time.
No speedup or consumer/firmware claim follows.

Source commit: `a72659abf47e8d04c54f3c77e4e4391cef7ec519`. The separate
`2026-11-12-interval-exponential` evidence record is committed at
`ab6efa286f8e86caa08d8331452a00b926987404`: 829 files,
142,585,251 bytes, SHA-256
`2df892c946bd524dac776446ec960ea214b97cb5a1c16c69e9e989d2f735df37`.
The scoped manifest was verified against that committed archive revision and
registered in `docs/evidence-archive.tsv`; all prior records are preserved.

Later elementary functions, primitive exponential backends, symbolic-ball
adapters, consumer migrations, actual memory/cancellation bounds, full coverage
and release qualification remain open.

## Logarithm and extended-carrier admission (2026-11-12)

Owner: Hyperinterval real-set and interval-evaluation layer, P1/P3/P5/P6/P19.
Baseline `23d0ac3d6`. Missing extended carrier and `log` methods were reproduced
before edits. Domain: finite rational/dyadic source intervals, canonical empty,
and signed i32 output grids. Positive sources have Complete coverage; a positive
upper endpoint with nonpositive lower endpoint gives Partial coverage and an
explicit lower-unbounded image. A nonempty source with no positive member
returns EmptyDomain. Internal exponent-storage failure and size/work exhaustion
remain typed. No source-domain clipping is silent.

`ExtendedInterval<I>` stores empty, nonempty finite, below/above singleton, or
entire states. Its backend trait is sealed to finite real interval families;
discrete integer sets and proposal carriers cannot confer authority. Ray
constructors reject non-singletons, including empty bounds. Replay rebuilds
through those constructors, canonicalizing finite empty. Infinity is a bound
direction, not a member. Copy is conditional on the finite backend being Copy.
Dyadic consumers provide exact negation, hull, intersection, inclusion and point
membership, checked finite extraction, and outward exponential images. Other
extended arithmetic and IntervalSet2 remain open.

A new bounded dyadic point-order helper first uses signs and top exponents,
then aligns only equal-top mantissas. It charges size/work before arithmetic
and avoids irrelevant enormous exponent gaps; existing finite-carrier predicate
order is unchanged. Extended finite construction delegates to existing validated
DyadicInterval constructors/arithmetic and retains canonical zero handling.

For a positive exact source x=m*2^k with 1<=m<2, log(x)=log(m)+k*log(2).
The positive atanh series uses t=(m-1)/(m+1)<=1/3; directed argument rounding
keeps t<=1/2. Successive true term ratios are at most 1/4, so twice the next
term bounds the omitted tail. All terms, sums and scale reconstruction are
outward intervals with shared DyadicBudget accounting. The exact sign of x-1
is transported through logarithm monotonicity to avoid rediscovering it after
cancellation around one. Log(1) remains exact zero. Nonunit rational logarithms
are not dyadic, so convergent refinement separates grid boundaries subject to
resource/storage limits. Success requires adjacent output-grid endpoint bounds.
No generic computable graph, hidden cache or new Hyperreal dispatch is added.

The returned extended image is consumed by later exponentiation and set
operations without clipping its unbounded end. Exponentiation of a lower ray
includes zero as the infimum of its closed enclosure; it does not assert that
zero was an exponential value. Domain coverage is operation-local, so the
original Partial receipt remains necessary after a later Complete operation.

The 34-command qualification matrix in `/tmp/interval-refactor/logarithm`
passes after a cost-probe type annotation repair. All five initial compile/lint
failures and their successful reruns are retained. All-feature tests pass 113
unit/integration tests and 12 doctests. Independent endpoint-order laws,
malformed ray replay, extreme scales, directed MPFR logarithm bounds,
1024-bit near-one and large/small rational inputs, resource refusal/retry,
log/exp composition and conditional primitive Copy are covered. The small
resource fixture completes near-one cases with 4096 reserved bits and 10^9
deterministic work units. ASan fuzzing completes 10,000 inputs in 13 seconds
(seed 20261119), with independent MPFR/GMP checks, replay and continued use.

The matrix also passes strict Clippy/docs/format, fuzz-target builds, all
repository Zizmor checks, ARM/WASM no_std builds, primitive dependency closure,
and four unchanged Hypersolve value/dispatch regressions. Shared-error changes
are checked against all 16 native rounding/FTZ/DAZ modes and strict-IEEE emitted
code scans under Rust 1.88, 1.95 and stable. Existing finite dispatch remains
unchanged; this does not qualify a consumer migration or a hosted platform.

Rust 1.95 instrumented all-feature coverage is 2811/2871 lines (97.91%) and
5483/5791 regions (94.68%). LLVM reports no branch denominator; tests,
conformance probes, Hyperreal, dependencies and test-only IEEE code are excluded.
All five profile mismatches are diagnosed as excluded Rug OptRational::parts.
The 600-sample cost probe completes all rational/dyadic, finite/partial and
near-one cases, with independent bounds checked before timing. Output and
receipts are materialized inside timing; raw work units and host metadata are
retained. The measured dyadic carrier is 72 bytes and its extended carrier
80 bytes, excluding heap storage. See the crate's PERFORMANCE.md for all
latencies; no baseline speedup or full memory bound is claimed.

Source and archive registration are pending. Actual heap-byte limits,
cancellation, remaining elementary backends/functions, consumer migrations,
full coverage and release qualification remain open.

## Priority order and non-negotiable rules (verbatim from plan)


Retain changes only when they improve, in strict order:

1. exactness;
2. completeness;
3. performance;
4. memory use;
5. binary size;
6. source size.

Every lower-priority improvement is gated by all higher-priority contracts.

## 1.1 Mathematical authority

Use this ownership rule throughout the stack:

> **Hyperinterval encloses. Hyperlattice carries set-valued linear structure. Hyperlimit decides. Hypersolve certifies. Hyperdynamics evolves. Domain crates assign meaning.**

Consequences:

- An interval is a set, not a fuzzy point.
- A midpoint is never implicitly substituted for an interval.
- A float proposal is never promoted to proof merely because it is widened heuristically.
- A certified enclosure may prove exclusion or containment, but Hyperlimit/Hypersolve owns the resulting decision status.
- Approximate or primitive code may propose roots, boxes, steps, preconditioners, split axes, or schedules. Exact/certified replay accepts or rejects them.
- No interval that merely contains zero may be reported as exact zero.
- No refinement budget exhaustion may be converted into a guessed sign, equality, or topology.

## 1.2 Point-valued `Real` remains separate

Do not put interval state into every `hyperreal::Real`.

- `Real` continues to denote one exact point value.
- `hyperreal` must not depend on `hyperinterval`.
- `hyperinterval` may optionally depend on `hyperreal` for `Rational` and `Real` bridges.
- Do not make arbitrary `[Real, Real]` endpoints the main interval representation. Endpoint order can itself require a bounded exact decision and would create a dependency cycle if the core interval crate depended on Hyperlimit.
- Preserve `hypersolve::real_interval::RealInterval` temporarily as a proof-layer adapter where exact `Real` endpoints are genuinely useful. Do not mistake it for the new general core.
- Prefer exact rational endpoints, exact dyadic endpoints, fixed-point lattice endpoints, or an exact `Real` center with a totally ordered exact radius.

## 1.3 No ambient numerical state

Do not use:

- process-global precision;
- process-global rounding modes;
- mutable global epsilon;
- hidden fast-math assumptions;
- mutable history-sensitive interval iterators;
- reversed endpoints as an error sentinel;
- NaN as an ordinary interval state;
- finite integer sentinels for infinity.

All budgets, rounding policies, precision requests, and terminal behaviors must be passed explicitly or fixed in a type.

## 1.4 Bounded work and explicit failure

Every iterative/refinement API must have caller-owned bounds and a typed exhausted result.

- No unbounded precision doubling.
- No unbounded subdivision.
- No indefinite retry on a zero-width or insensitive domain.
- Progress must be monotone and testable.
- The final newly produced enclosure/certificate must be checked before termination.
- Unsupported domains, partial domains, poles, overflow, and environment limitations remain explicit.

## 1.5 Preserve semantic wrappers

Generalize arithmetic, not meaning.

Do not erase distinctions among:

- exact design intent;
- measured uncertainty;
- configured uncertain facts;
- canonical machine lattice values;
- lossy display values;
- solver candidate balls;
- algebraic isolating intervals;
- firmware current intervals;
- clock-correlation intervals.

`BoundedMeasurement<U>`, machine-profile facts, root isolators, `Q30Interval`, servo intervals, and clock estimates may delegate to shared kernels while retaining their domain-specific types, invariants, provenance, and reports.

## 1.6 Preserve user work

Before any edit:

- record every repository HEAD, branch, dirty path, untracked path, and ahead/behind state;
- treat all pre-existing edits as user-owned;
- use isolated branches/worktrees where concurrent work exists;
- never reset, clean, delete, reformat, or rewrite unrelated files;
- attribute only agent-owned changes to this project.

