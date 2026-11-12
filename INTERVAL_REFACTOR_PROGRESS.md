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
| 4 proposals/float | proposal wrappers implemented; strict backend pending | environment qualification and rational oracle |
| 5 elementary functions | not started | MPFR domain/extrema/pole corpus |
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

