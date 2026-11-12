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
| 5 elementary functions | abs/min/max, integer powers, square roots, exp/log/atan/sin/cos, split division, rational powers and tan/cot implemented on documented backends | final function/backend coverage, resource and consumer qualification |
| 6 lattice | Cartesian/affine carriers, widths, derived midpoint/radius, norms/measures/dominance and contraction evidence implemented | independent center/radius forms, Jacobian carriers, specialization and full consumer/performance gates |
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

Implementation is committed as `c349a30ec7472fbaacd69a04ab164c1f8c6641f3`.
The `2026-11-12-interval-logarithm` evidence record is committed at
`52ac436ea8400b4ceb4243dfdaa044f477932525` with 386 files,
77,664,890 bytes and record SHA-256
`be624b23ddaea7651dc5d6cf70ba3f694b608ec4be9b015395fee9dd11637c94`.
The committed archive digest is verified and registered in
`docs/evidence-archive.tsv`. Actual heap-byte limits, cancellation, remaining
elementary backends/functions, consumer migrations, full coverage and release
qualification remain open.

## Arctangent admission (2026-11-12)

Owner: Hyperinterval elementary enclosure layer, P1/P3/P5/P6/P19. Baseline
`5d1e9e455`; the missing DyadicInterval::atan witness is reproduced before
production edits in `/tmp/interval-refactor/arctangent/missing-witness.log`.
Domain: finite rational/dyadic real intervals and explicit extended dyadic
sets, including empty, with i32 output grids and shared DyadicBudget limits.
Atan is monotone on all reals; extended ends use its +/-pi/2 limits in a closed
enclosure. Coverage is locally Complete, preserving earlier Partial receipts.
Acceptance requires independent directed MPFR bounds, exact zero and symmetry,
range-reduction boundaries, huge/tiny inputs, resource refusal/retry, replay,
and output-as-input operations. Hyperreal has a Machin/atan scalar evaluator,
but it does not provide this cumulative DyadicBudget contract; its production
code, caches and dispatch remain unchanged.

The implementation maps sign by odd symmetry and maps large positive x through
atan(x)=pi/2-atan(1/x). Around one it uses the exact quarter-circle identity
with (1-y)/(1+y), reducing every series argument to [0,1/2]. Alternating odd
powers decrease; the next term bounds the remainder with its known sign.
Every power, sum and remainder is an outward dyadic interval. Machin's identity
pi/4=4 atan(1/5)-atan(1/239) supplies budgeted constants without a cache. Exact
zero and 0<atan(x)<min(x,2) provide structural shortcuts. Nonzero rational
arctangents and pi/2 are not dyadic; convergent refinement separates their grid
boundaries subject to resource/exponent storage limits. Only adjacent-grid
endpoint enclosures are published. Unbounded ends use certified limits, never
finite stand-ins. Assembly reuses the existing validated extended-bound layer.

The 26-command local feature/release/lint/docs/fuzz-build/workflow-security/
portable-build matrix passes. One copied test filter initially selected zero
tests; `resource-contained.log` records the corrected explicit test passing
under a 60-second external limit, and the reproduction script is corrected.
All-feature tests pass 118 unit/integration tests and 12 doctests. Supplemental
independent tests pass through 2048-bit output precision, 1024-bit rational
coefficients near 1/2, 1 and 2, both i32 dyadic scale extremes, signed symmetry,
refinement, serde and continued exp/atan/log use. The initial test-only missing
type annotation and its correction are retained. No production regression was
found by the independent corpus. Four unchanged Hypersolve value/dispatch tests
pass; ARM/WASM no_std and primitive dependency closure remain qualified locally.

ASan fuzzing completes 10,000 cases in 9 seconds (seed 20261120), retaining
independent MPFR/GMP checks, extended limits, replay and continued operations.
Instrumented Rust 1.95 all-feature coverage is 3000/3063 lines (97.94%) and
5981/6364 regions (93.98%); LLVM supplies no branch denominator. Tests, probes,
Hyperreal, dependencies and test-only IEEE code are excluded. All six profile
mismatches are diagnosed as excluded Rug OptRational::parts. This is not full
coverage. All 660 timing samples complete, including unbounded sources and
huge/tiny finite inputs. The crate PERFORMANCE.md reports all group medians;
raw work units, host/compiler metadata and whole-probe size remain in evidence.

Implementation is committed as `dc953d7bb9d24ff1a3ec1f05a337830a2fd16786`.
The `2026-11-12-interval-arctangent` evidence record is committed at
`7c97c5ad4db227316b4b05066871939b2f4522f3` with 219 files,
63,298,632 bytes and record SHA-256
`5c51eb4dbf70ed02877bedad259749fa21c778a400240c5b83c92693e24533f9`.
The committed archive digest is verified and registered in
`docs/evidence-archive.tsv`. Actual heap-byte bounds, cancellation, remaining
elementary backends/functions, split images, consumer migrations, full coverage
and release qualification remain open.

## Periodic trigonometry admission (2026-11-12)

Owner: Hyperinterval elementary enclosure layer, P1/P3/P5/P6/P19. Baseline
`9bbeb1012`. Missing sin/cos methods and an interior-extremum acceptance witness
are reproduced in `/tmp/interval-refactor/trigonometry/missing-witness.log`.
The required domain is finite rational/dyadic intervals and extended dyadic
sets, with exact sources, outward dyadic results and local Complete coverage.
Acceptance includes certified phase reduction and interior extrema, huge and
tiny arguments, resource refusal/retry, independent MPFR bounds and continued
operations. No endpoint-only trigonometric range evaluation is admitted.

A baseline review also reproduces a feature-gate regression: the arctangent
module insertion detached the cfg attribute from the private bounds module.
No-feature strict Clippy fails with dead-code errors. Restore the original
gate placement and add no-feature strict Clippy to CI; the earlier successful
featureful lint runs did not qualify this configuration. Evidence is retained,
not replaced by the repaired outcome.

The implementation reuses the existing budgeted Machin pi/4 enclosure with
unchanged arithmetic and predicate order. For each nonzero rational endpoint,
adjacent integer bounds on abs(x)/(pi/2) certify its quadrant: this quotient
cannot be an integer. Odd/even symmetry transports negative sources without
rediscovering a tiny sign after cancellation. The retained integer phase and
residual select sine/cosine and its sign. Residuals lie in [0,pi/2], within
[0,2]; alternating factorial terms decrease beyond the initial cosine term,
and the signed next term encloses the remainder. Directed interval operations
retain source uncertainty and every rounding error. Exact zero is handled
structurally; nonzero rational sine/cosine values separate dyadic grid
boundaries under convergent refinement, subject to work/size/storage limits.

Interior extrema are selected from the certified endpoint phase indices and
integer residues, not sampled membership. Every crossed critical point is
accounted for; an endpoint at zero retains its exact function value. Width at
least 8 proves a whole period before phase materialization (2*pi<8); four
crossed quarter periods also prove both extrema. Nonempty unbounded sources
reach both extrema directly. Full-range and exact-zero results preserve fine
grids without unnecessary expansion. A narrow source at exponent i32::MAX
reaches an actual declared coefficient reservation limit instead of being
reported as a completed whole-range approximation. No source is rounded into
a replacement model; each retry consumes the original exact endpoints.

The final 29-command qualification matrix passes: 123 unit/integration tests,
12 doctests, strict Clippy (including no-feature builds on 1.88/1.95/stable),
docs/format, fuzz builds, all repository Zizmor checks, no_std ARM/WASM builds,
primitive dependency closure and four unchanged Hypersolve value/dispatch
regressions. Initial type-inference failures and two Clippy failures on the
quadrant parity expression are retained; the final expression explicitly
matches quadrants 0/2. The complete matrix was rerun after that final source
edit. Existing predicate order and the scalar evaluator are unchanged.

The independent oracle uses directed MPFR at an exactly represented rounded
point plus the global unit Lipschitz bound for any exact rational input error;
this avoids assuming sine/cosine monotonicity while rounding the source.
Directed pi bounds and exact GMP arithmetic independently select crossed
extrema. The corpus covers brackets of width 3*2^-256 around periodic
boundaries, phase magnitudes through 2^512, output precision through -1024,
subnormal-scale values, i32 scale extremes with explicit completion/refusal,
odd/even symmetry, refinement, serde, and log/sin/cos continued use.

The first ASan campaign reaches 8199 inputs before its 30-second stop (31 s
reported). A 60-second campaign completes 10,000 in 39 seconds, and the final
post-lint source repeats 10,000 in 39 seconds (seed 20261121). All logs and the
retained corpus remain in evidence; CI keeps its bounded 30-second campaign.
Final Rust 1.95 coverage is 3298/3374 lines (97.75%) and 6612/7090 regions
(93.26%); no branch denominator is reported. Exclusions are tests, probes,
Hyperreal, dependencies and test-only IEEE code. All seven profile mismatches
are diagnosed as excluded Rug OptRational::parts; initial and final profiles
are kept separate. Full coverage remains open.

All 1560 cost samples complete, including finite and unbounded sources and
all refinement work. PERFORMANCE.md contains every group median; raw work
units, compiler/host metadata, executable hash and whole-probe size are
retained. No baseline speedup, actual heap-byte limit or consumer migration
is inferred. Implementation is committed as
`52eaa40d2f66a482747c42322d7c065d62f7b9bb`.
The `2026-11-12-interval-trigonometry` evidence record is committed at
`aa98e3c00c32852d217e4bb0d58f8734bcb58826` with 595 files,
110,663,701 bytes and record SHA-256
`de993c4af15dcdc1d908c3e1266f23db89f379b36a4fb29278979c72583a5542`.
The committed archive digest is verified and registered in
`docs/evidence-archive.tsv`. Remaining elementary backends, rational powers,
poles/split images, cancellation, full consumer migrations and release
qualification remain open.

## Split images and extended division admission (2026-11-12)

Owner: Hyperinterval scalar-set layer, P1/P3/P5/P6/P19. Baseline `726f6da83`.
The missing IntervalSet2 and extended reciprocal witness is reproduced in
`/tmp/interval-refactor/split-division/missing-witness.log`. This is the plan's
explicit prerequisite for negative rational powers crossing zero: do not turn
the disconnected image into an undeclared hull. Existing finite strict division
and its zero-rejection order remain unchanged.

Domain: connected extended dyadic intervals, with closed outward division
images containing at most two components on a requested grid. Zero-only
denominators return EmptyDomain before empty-numerator shortcuts; other empty
Cartesian sources return successful empty with Complete coverage. A nonempty
source whose denominator includes zero has Partial coverage. Bounds at infinity
are limits, never real members; closure may include an unattained zero. Split
normalization merges touching/overlapping closed enclosures and preserves every
real gap. Admission requires independent exact rational expectations, extreme
scales, malformed canonical replay, small-budget refusal/retry, and continued
negation, membership, subset, explicit hull and exponential use. More general
set operations that can create more than two pieces require a separate carrier
or an explicit capacity contract; no universal union closure is claimed.
The sealed generic carrier stores zero/one/two inline components; two-piece
construction, canonical replay and numerical operations currently use the dyadic
backend. Primitive one-piece carriers remain Copy. Exact comparison normalizes
order and merges touching/overlapping represented enclosures; a real gap is
never filled implicitly. Native replay rejects reversed, touching, overlapping
or empty two-piece encodings and revalidates each extended bound. Explicit hull
is the named operation that fills a gap. Reflection transports order proof by
reversing components; membership and inclusion inspect the union itself.

Division splits the denominator by sign. On a positive branch, numerator sign
selects the correct denominator extreme for each quotient endpoint; the negative
branch is its exact reflection. Finite endpoints are rounded from original
quotients, not from rounded reciprocals. Pole and infinite-end limits are
explicit. Rounding may close a gap, in which case normalization correctly
reports the represented closed union. Zero and unit relations and strict
sub-grid magnitude bounds avoid irrelevant scale expansion.

A maintained regression found [6,9]/3 and other exact dyadic quotients failing
at grid i32::MIN through avoidable expansion. The pre-fix source and failure
remain in evidence. The repair reserves deterministic GCD/reduction work before
calling the shared integer GCD backend, recognizes reduced power-of-two
denominators, and retains compact exact coordinates. Non-dyadic fine-grid
results still require their real coefficient growth. A 1024-bit ratio fixture
checks exact near-unit bounds, explicit work exhaustion and successful retry.

The final 29-command feature/release/lint/docs/security/portable-build matrix
passes, including 131 unique unit/integration tests and 12 doctests. The replay
test additionally succeeds in a fresh child process; its extra one-test result
is not double-counted. Four unchanged Hypersolve value/dispatch witnesses,
no-feature strict Clippy on all three compilers, primitive dependency closure,
ARM/WASM no_std builds and all repository Zizmor checks pass. Two initial
Clippy failures on a nested condition are retained; the syntax repair keeps
the same condition order. All affected checks were rerun after the final
production change. The large-coefficient fixture was added and passed afterward;
final instrumented tests include it. No existing strict operation was changed.

The independent GMP oracle enumerates all four finite/limit corners for each
denominator sign branch and normalizes rounded results independently. Its
23-state connected-source corpus covers every pair at three grids. ASan fuzzing
completes 100,000 cases in 19 seconds initially and 100,000 in 20 seconds on final
source (seed 20261122), including replay and continued set operations. Final
Rust 1.95 coverage is 3570/3643 lines (98.00%) and 7070/7571 regions (93.38%);
no branch denominator is reported. Tests, probes, Hyperreal, dependencies and
test-only IEEE code are excluded. All eight profile mismatches are diagnosed
as excluded Rug OptRational::parts; earlier profiles are not merged with final.

All 540 cost samples complete with full image/receipt materialization and
shared work accounting. The crate PERFORMANCE.md records all medians. Source
commit `d42537ac1779409cad572d47f7fff52289aa5416`; evidence commit
`143c567661e52959fafd09650e0d21a30d1316dc`, record
`docs/evidence/2026-11-12-interval-split-division`: 644 files, 144380703 bytes,
SHA-256 `c2b2b26adeeff29a96419af9a79ab82235309524cce61e2ee2434678a4aaecc8`.
The committed record was independently regenerated and verified before registration.
Rational powers themselves, additional
split operations/backends, general union closure, actual heap-byte bounds,
cancellation, consumer migrations and release qualification remain open.

## Rational exponent admission (2026-11-12)

Owner: Hyperinterval scalar image layer, P1/P3/P5/P6/P19, baseline `593d5c95e`.
The missing `pow_rational` acceptance witness is reproduced in
`/tmp/interval-refactor/fractional-power/missing-witness.log` before implementation.
The domain is connected rational, dyadic and extended dyadic source intervals
and an arbitrary-size integer numerator/denominator exponent. The denominator
must be nonzero even for an empty source. Normalize its sign and common factors
before determining the real branch: odd reduced denominators admit negative
sources; even denominators admit nonnegative sources. Negative powers exclude
zero and retain disconnected images. The zero exponent maps every real source
member, including zero, to one, consistently with existing integer powers.
Empty maps to empty; no defined member returns EmptyDomain; excluded members
produce Partial. The output is a normalized closed IntervalSet2 on the requested
dyadic grid; infinity denotes a limit, and closure may include unattained zero.

Use exact integer root/power relations and monotonicity of each sign branch.
Retain perfect rational roots before expanding a fine requested grid. Share
normalization, exact quotient rounding, extended bounds and work accounting
with existing owners; do not introduce a rounded reciprocal intermediate.
Small-budget refusal/retry, independent GMP/MPFR expectations, extreme scales,
replay, and consumption by subsequent supported operations are acceptance gates.
DyadicBudget's deterministic size/work accounting applies across nested calls;
actual heap bytes, cancellation and consumer migration are not established by
this new entry point.

The shared implementation now serves all three adapters. It strips binary
factors, preserves perfect rational roots before powering, and applies signed
powers to the exact source fraction before grid rounding/root isolation.
For nonnegative z, floor(root_q(z)) = floor(root_q(floor(z))); bisection keeps
lo^q <= floor(z) < hi^q and an exact power comparison distinguishes equality.
The upper grid bound increments exactly when either relation is nonintegral.
Reduced numerator parity and denominator parity select admitted sign branches;
negative exponents reverse magnitude order. Split normalization acts only after
outward bounds are certified. No approximate scalar or rounded reciprocal is
used. Hyperreal's existing perfect_nth_root is an unbudgeted u32-degree helper;
this arbitrary-degree no_std dyadic implementation does not change its dispatch
or add a Hyperreal dependency. The primitive power helper and its accounting
also remain unchanged.

Independent GMP tests cover 23 connected source states, 11 numerators, nine
denominators (including zero and negative), and four grids: 9108 combinations.
An additional 80 point cases use directed MPFR, with an exact GMP relation for
singletons that a rounded intermediate MPFR root can straddle. Rational tests
include perfect roots, partial domains, fine grids and 1024-bit near-one ratios.
Extreme exponents/scales exercise compact results, explicit reservation
exhaustion and retry. Fresh-process replay consumes split output in another
power; further exponentials retain operation-local and inherited coverage.

The initial constant-one shortcut ignored coarse grids. The failing regression
and pre-fix source are retained in zero-grid-failure.log and pre-zero-grid.rs;
the final zero-exponent image rounds outward on the requested grid. Early
oracle compile/lint failures and an initial module-order formatting failure
remain in the record. These are distinct from successful geometric outcomes.
The final 29-command feature/release/lint/docs/security/portable-build matrix
passes, including 140 unique unit/integration tests and 12 doctests. The two
fresh-process child executions are not double-counted; test-list.log records
the unique count. Four unchanged Hypersolve value/dispatch witnesses, strict
Clippy on supported compiler/feature combinations, primitive dependency closure,
ARM/WASM no_std builds and repository Zizmor checks pass. No push was attempted.

Final Rust 1.95 coverage: 3878/3959 production lines (97.95%), 7709/8259 regions
(93.34%), all 12 new-module functions executed. No branch denominator is reported.
Tests, probes, Hyperreal, dependencies and test-only IEEE code are excluded;
eight profile mismatches are diagnosed as excluded Rug OptRational::parts.
Initial and final profiles remain separate. ASan fuzzing completes the published
10,000-case campaign; raw logs/corpus are retained. All 540 completed cost
samples, compiler/host metadata, executable hash and size are retained;
PERFORMANCE.md records medians and methodological limits.

Source commit `9093a43aba16b452ca8d6be3a658324c2b90815a`; evidence commit
`3f61bdac98d1ff366169d18853729629f2f3a2eb`, record
`docs/evidence/2026-11-12-interval-fractional-power`: 529 files, 142059988 bytes,
SHA-256 `bb2419ef4ed44f53d213b188dfb6c6b2ae96c9a894e73fd050915ce8e969d62d`.
The committed record was regenerated and independently verified before registration.
Tangent/pole functions, remaining
Phase 5 qualifications, broader consumer migrations, allocation/cancellation,
coordinated release and the earlier incomplete plan phases remain open.

## Tangent and cotangent admission (2026-11-12)

Owner: Hyperinterval scalar image layer, P1/P3/P5/P6/P19; baseline `0ef6cc8ec`.
The missing tan/cot witness is reproduced before implementation in
`/tmp/interval-refactor/tangent/missing-witness.log`. The input domain is
connected rational, dyadic and extended dyadic source intervals. Every finite
rational tangent endpoint is defined; cotangent excludes zero and its irrational
nonzero poles. A zero-only cotangent source returns EmptyDomain, empty sources
return empty/Complete, and any excluded pole gives Partial coverage. The image
is a normalized IntervalSet2 of closed extended dyadic components, rounded
outward on the requested grid. A full branch or nonempty unbounded source gives
the entire line/Partial, proved from source topology rather than uncertainty.

Reuse certified half-pi phase reduction, Taylor bounds and extrema detection;
keep existing sine/cosine call order and accounting unchanged. Prove monotonic
branch extrema and pole incidence before assembling finite intervals or rays.
Never infer a whole-range success from a denominator enclosure containing zero:
refine or exhaust the shared deterministic size/work budget. Exact cotangent
endpoint poles require one-sided limits and domain qualification. No rounded
source conversion supplies the authoritative phase. Independent directed MPFR
point bounds and GMP pole indexing, near-pole sources, huge/tiny arguments,
partial/repeated operations, fresh-process replay and budget refusal/retry are
acceptance gates. Actual heap-byte limits, cancellation, wider consumer migration
and coordinated release remain open.

The implementation shares the existing reducer, alternating Taylor bounds and
extremum scan. Sine extrema mark tangent poles; cosine extrema mark cotangent
poles. Nonzero rational endpoints cannot equal nonzero rational multiples of pi.
Cotangent's exact zero endpoint is excluded from the interior scan and supplied
as a one-sided infinite limit. Two interior poles, or an interior pole plus an
endpoint pole, prove a full branch. With no pole, monotonicity selects finite
extrema; with one interior pole, it selects two rays. Tangent increases and
cotangent decreases on each branch. Normalization merges only touching or
overlapping represented enclosures, including gaps closed by outward rounding.

Endpoint evaluation selects the denominator first from certified sine/cosine
residual enclosures. If zero cannot yet be excluded, refinement continues. The
original rational source remains authoritative at every attempt. Signed quotient
bounds must reach adjacent requested-grid endpoints before publication; no
rounded-source pole decision or exhausted partial result escapes. Full-line
unbounded-source results need no numerical allocation or budget work.

The source audit in shared-trig-body-check.json proves the old trigonometry
module is identical after removing only crate visibility, whitespace and one
optional signature comma. Existing call order/accounting and Hyperreal dispatch
remain unchanged; no numerical-body refactor was performed. Shared test helpers
now supply MPFR pi bounds and exact GMP phase indices to both trig corpora.

The final 29-command feature/release/lint/docs/security/portable-build matrix
passes. It includes the existing sine/cosine corpus, four unchanged Hypersolve
value/dispatch witnesses, primitive dependency closure, no-feature strict Clippy,
ARM/WASM no_std builds and repository-wide Zizmor. Two initial Clippy failures
on the same parity spelling are retained with pre-lint source; the repair keeps
condition order. One additional adapter test was then added and passed focused
strict Clippy and the complete instrumented suite. The final unique count is
148 unit/integration tests plus 12 doctests, recorded by test-list.log; the three
fresh-process child executions are not additional unique tests. No push occurred.

The independent oracle combines directed MPFR endpoint bounds with GMP branch
indices, a different closed-form calculation from the production residue scan.
Its ordinary dyadic corpus has 504 function/source/grid combinations, plus
explicit pole-crossing cases. Rational near-pole fixtures include points and
intervals within 2^-128 of periodic boundaries, and huge phases through 2^512.
Tiny cases include both functions at 2^-1074 with grids 2^-64 and 2^-1074.
A near-pole point exhausts small work budgets without being mislabeled a pole;
a larger retry returns the same Complete finite enclosure. Output undergoes
fresh-process replay, another pole operation, arctangent and exponential use.
Initial ASan fuzzing completed 10,000 cases in 55 seconds; final-source fuzzing
completed 10,000 in 61 seconds, seed 20261124, within the published 120-second cap.

Final Rust 1.95 production coverage is 4121/4208 lines (97.93%) and 8190/8772
regions (93.37%); all 16 new-module functions execute. No branch denominator is
available. Tests, probes, Hyperreal, dependencies and test-only IEEE code are
excluded. LLVM's initial extra diagnostic run aborted; the failure/backtrace is
retained. Running diagnostics with one merge thread succeeded and its report
table matched the successful ordinary report. All nine profile mismatches are
excluded Rug OptRational::parts. Final profiles are fresh and separate from the
earlier adapter-incomplete run; single-thread reporting changes no exclusions.

All 1560 cost samples completed, across rational/dyadic functions and extended
entire-line inputs, three grids and six finite source shapes. PERFORMANCE.md
records medians, deterministic work and the expensive subnormal cotangent case;
raw samples retain compiler/host metadata, executable hash and whole-probe size.
No allocation/cancellation, embedded-cycle or speedup claim is made.
Source commit `9cc7829e966c4b5b3f7a4f427fe03c48c5128cf2`; evidence commit
`d012804d5d964371d343f2d5189aa7bd05fa043e`, record
`docs/evidence/2026-11-12-interval-tangent`: 619 files, 124164420 bytes,
SHA-256 `1b2ebd6f32b5d317c8c98d6b8af8428cadac9203c616e3bfe9a26b0010d46cf6`.
The committed record was regenerated and independently verified before registration.
Phase 5 qualification gaps and all
remaining plan phases require their own completion evidence.

## Phase 6 admission: Cartesian interval algebra and dependency boundary

Before production edits, revision `a76acf1ac83c6532e72279ced05305dd29181e5d`
reproduces absent `IntervalVector`/`IntervalMatrix` imports and fails Rust 1.88
`cargo check -p hyperlattice --no-default-features --lib`: both Hyperlattice
and its unconditional Hyperreal dependency require Rust 1.95. The normal
no-default dependency tree includes Hyperreal and allocating numeric crates.
Hypersolve and fuzz consumers already use Hyperlattice with defaults disabled;
removing their existing Real API behind a new default feature would break them.
This is the dependency evidence required by plan §2.3 to permit a production
split. A Rust 1.88, no_std `hyperlattice-interval` companion will hold the single
interval-linear implementation, re-exported additively from Hyperlattice. The
legacy scalar/linear implementation and its feature behavior stay intact.

Owner: Hyperlattice for Cartesian structure; Hyperinterval for all scalar
arithmetic, set relations, and backend qualification. Relevant contracts:
P1, P3, P6, P17–P20, P25, P26, P35–P39. Domain: fixed-size Cartesian products
of validated finite real intervals (fixed i32/i64, rational, dyadic, qualified
strict floats), including canonical empty and zero-dimensional unit products.
An empty lane canonicalizes the entire product to empty. No scalar midpoint
or point equality is inferred from an interval. Matrix/vector products and
component hulls return explicit enclosures: their coordinates need not be
independent in the source image. A coordinate affine enclosure does not satisfy
the separate planned exact correlated affine-image carrier requirement.

Missing acceptance witnesses: independent exact corner oracles for dot and
matrix products, fixed outward rounding and overflow, empty/unit distinctions,
serde validation for arbitrary const dimensions, consumed output chains,
shared dyadic budget exhaustion and retry, primitive no-allocation/MSRV and
embedded build, unchanged legacy tests/source/dispatch. Dyadic scalar work and
size accounting must span every lane and nested product; it is not a wall-time,
heap-byte or cancellation bound. Rational arithmetic remains explicitly
unbounded. Primitive work is bounded by dimensions and fixed-width kernels;
stack storage grows with dimensions. No partial numerical result is published
on scalar failure. Explicit cloning/serialization are caller-owned work.

Raw staging: `/tmp/interval-refactor/lattice-core`. The first legacy baseline
was contaminated by a temporary compile-fail witness and is retained separately;
it is not correctness evidence. The clean rerun on unchanged legacy source
passed all 240 tests; the additive API run passed those 240 plus its new surface
witness. Phase 6 remains open for its full advertised operation,
measurement, solver/firmware consumer and exact-affine obligations.

## Phase 6 Cartesian foundation: executed local qualification

Implemented one sealed adapter to existing Hyperinterval scalar operations and
inline `IntervalVector`, `IntervalMatrix`, and `BoxN` carriers. Construction,
component relations, addition/subtraction, dot and matrix products, affine
**enclosures**, transpose, projections, and arbitrary-dimension serde share the
same authority through Hyperlattice's additive re-exports. No legacy numerical
source or scalar kernel changed: a byte comparison covers 47 source files,
subtracting only the new re-export block from Hyperlattice's lib.rs.

The independent exact corner oracle exhausts 1,296 two-lane interval input
combinations and 2,916 point-matrix/box combinations per scalar backend (i32,
i64, rational, dyadic, strict f32 and strict f64). It checks dot, 1x1 and 2x2
matrix products, affine enclosures, and subsequent products. Another 50,625
fixed fractional cases check outward bounds against exact integer numerators.
The 12 maintained tests also cover empty/unit products, invalid scales and
indices, rounding/overflow, shared budgets and retry, owned-copy accounting,
malformed replay, dimensions above 32, and allocation-free primitive chains.
The final deterministic suite passes ASan with leak detection; this is not a
claim of a directed fuzz campaign or universal proof.

Final feature checks pass on Rust 1.88 and stable, plus the all-feature suite
on 1.95; no-default and all-enabled Clippy, release tests, docs with warnings
as errors, fmt and pedantic strict-collection zizmor pass. Hyperlattice's
all-feature tests pass (241), its all-target Clippy and no-default API build
pass, and all four Hypersolve interval migration differential/dispatch tests
pass. ARM thumbv7em and wasm32 subsets check under both 1.88 and stable.
Primitive normal dependency closure contains exactly the companion and
Hyperinterval. A clean Rust 1.88 no_std Wasm cdylib links with no global
allocator and no imports; Wasmtime 50.0.1 verifies 2,049 calls covering 513
unique clamped inputs. Package listing includes the public source, licenses,
README and runnable fixed example; registry publication is not established.

Final owner-only instrumentation reports 472/483 executable lines (97.72%),
605/639 regions (94.68%) and 101/101 functions. There is no measured branch
denominator. Dependencies, tests, conformance and toolchain code are excluded;
initial reports/profiles are preserved alongside the final report. The 100%
coverage and full qualification targets remain open.

A paired scalar-loop/control probe records 200 raw samples: 20 samples per
path across five completed 2x2 product cases, each with 128 materialized and
dropped outputs after 64 warmups. Both primitive cases allocate zero; rational
and dyadic cases match their controls' allocation counts. Median Cartesian
latencies are approximately 158 ns (fixed Q16), 1.86 us (strict f64), 1.57 us
(small rational), 1.15 us (97-bit rational numerators), and 6.02 us (dyadic).
Small/wide rational rows are slower than controls in this run; no speedup,
firmware cycle, HIL, stack bound, or legacy Real benchmark claim is made.
The probe ran on a shared host with concurrent validation, not an isolated
performance machine. This additive capability is retained for its contract
extension; it is not a behavior-preserving numerical refactor.

Outstanding Phase 6 work includes exact correlated affine carriers, widths and
widest axes, midpoint/radius forms, norms and matrix measures, contraction and
Krawczyk helpers, measured specializations/facts and full consumer acceptance.
Legacy Real benchmark/monomorphized-size and successive-operation comparisons
remain part of the full phase gate. No solver or firmware production migration
has occurred in this checkpoint. No phase or overall goal is complete, and no
push was attempted. Source commit:
`b03b3a4c98b0ee8731423816fa0666ecb7bbe50f`. Immutable evidence commit:
`3f240cc3526422d0107381b9f655e1d02ecda1fe`, record `docs/evidence/2026-11-12-interval-lattice-core`,
189 files, 42,314,675 bytes, SHA-256
`d73ad3b7683a0f1fc42f4b9e4cce275d48b49a59881bdbad3f8cbfb5d1779a03`. The committed record was regenerated and
verified before registration; all prior archive rows are preserved.

## Phase 6 admission: exact affine definitions and composition

Previous goal turn: progress (Cartesian source and immutable evidence committed).
Revalidated clean source `8a6fd2f2bbdf4b60419d8dced25fbc328f96f502` and the
12-test Cartesian baseline. A missing-API witness fails on this revision:
`AffineMap` is absent. Its intended result maps one parameter to `(x,x)`, then
subtracts those coordinates; the exact image is `{0}` while materializing the
intermediate Cartesian box permits `[-2,2]` for `x` in `[-1,1]`.

Owner: Hyperlattice exact affine structure; Hyperinterval scalar arithmetic.
Relevant properties: P1–P3, P5–P6, P17–P20, P25–P27, P35–P39. Domain: finite
compile-time-dimensional closed boxes and finite chains of affine maps whose
matrix and translation entries are exact singleton finite intervals. Admit
singular maps, arbitrary rectangular dimensions, zero-dimensional unit products,
canonical empty source domains, and every existing arithmetic backend. Reject
empty/non-singleton map coefficients even when a source domain is empty.

The exact object retains one source parameter box and its affine definitions,
with no intermediate Cartesian substitution and no coefficient materialization
required at construction. Subsequent affine maps consume that definition;
parameter restrictions intersect the original parameter domain (not the output
coordinates). Structural equality and deterministic replay are separate from
equality of geometric images. There is no general image-membership or Boolean
claim for this numerical carrier.

Enclosure uses `B(Ax+b)+c=(BA)x+(Bb+c)`, pulling coefficient bounds to the
original source before interval evaluation. Rational/dyadic arithmetic yields
the exact coordinate hull on successful completion; fixed/strict-float scalar
rounding can widen it. A rounded enclosure remains an explicit projection and
never replaces the retained exact definition. Primitive overflow, shared dyadic
size/work exhaustion and other scalar failures remain typed. Failed queries or
restrictions must leave the original reusable; owned restriction copies must
be charged through the scalar backend. Loops are dimension-bounded, nesting is
finite in the type, and storage is inline. Stack use grows with dimensions/depth;
there is no claimed heap-byte/CPU-time/cancellation bound for arbitrary-precision
arithmetic, validation or serde. Explicit caller clones remain caller-owned.

Missing acceptance witnesses: correlated cancellation and inverse compositions,
exact independent corner expectations, off-grid primitive definitions surviving
further operations, rank collapse and empty/unit maps, malformed fresh-process
replay, restrictions in parameter space, nested exhaustion/retry and no-allocation
primitive chains. Existing Cartesian operations must keep their source and
observed outputs. Raw staging: `/tmp/interval-refactor/affine-images`.
The full Phase 6 and 15-phase objective remain active.

## Phase 6 retained affine images: local qualification

Implemented `AffineMap`, `AffineImage`, `AffineBox` and the sealed `AffineSource`
contract, re-exported by Hyperlattice. Point coefficient validation precedes
image construction. Further maps retain one parameter source; restriction acts
in that source's coordinates and preserves every map. Enclosure pulls coefficient
bounds to the original domain, sharing one scalar context through all operations.
No existing scalar/Cartesian numerical code changed: 49 source files and the
array/dot helpers, existing serde implementations and legacy lib body (apart
from the re-export list) were compared to the baseline.

The 26 maintained tests plus one doctest pass across the declared feature/MSRV
matrix. New exact corner oracles cover all 6,561 pairs of 2x2 matrices with
entries in `{-1,0,1}` on each of six backends, with independent point evaluation
at four source-box corners. Additional witnesses cover correlated cancellation,
inverse maps with large rational parameters and fractional coefficients,
rank loss, rectangular/zero-dimensional maps, invalid coefficients, empty sets,
parameter restrictions, exact work exhaustion/retry and charged owned copies.
A dyadic image whose intermediate projection exceeds 256 bits can be consumed
by its inverse and recover its original box within that size budget. Fixed and
IEEE values below their output grids retain definitions and recover through a
later map. Fresh-process replay verifies this off-grid composition; malformed
nested maps fail even with empty sources. All six scalar families exercise
validated affine serialization when serde is enabled.

All 44 selected checks passed: Rust 1.88/stable feature isolation, Rust 1.95
all features, release tests, no-default/all-feature Clippy, docs, fmt, two
Hyperlattice facade tests, four solver differential/dispatch tests, and ARM/Wasm
subsets on 1.88 and stable. Final ASan tests with leak detection pass (including
the fresh replay child). Owner coverage is 600/610 lines (98.36%), 767/805 regions
(95.28%) and 126/126 functions; `affine.rs` has 110/110 executable lines and
121/126 regions covered. No branch denominator was emitted. Tests, conformance,
dependencies and toolchain code are excluded. The 100% and directed-fuzz targets
remain open; this ASan run is the deterministic qualification suite.

A separate Rust 1.88 no_std Wasm consumer links without a global allocator or
imports and passes 2,051 calls per export (513 unique clamped inputs), checking
both the old Cartesian and new correlated path. The runtime imposes an external
one-million-fuel limit per call. The artifact is 18,161 bytes, SHA-256
`4e49a7683a89ab534ab98bb61fe58f5a5b653d7a417b03ecebb538aaa74ee626`.
This is a development path consumer, not registry/HIL/firmware-cycle proof.

The final affine probe records 480 samples: four backends, depths 1/2/4,
enclosure and owned restriction, 20 samples of 128 completed operations after
64 warmups. Primitive paths allocate zero. Four-map inline footprints are
312 bytes (fixed Q16), 624 (strict f64), 488 (rational) and 1,872 (dyadic);
these exclude heap storage and are not peak stack measurements. The original
probe's `retained_bytes` label was corrected to `inline_bytes`; both runs are
retained. Timing varies substantially on this shared host, so no speedup is
claimed. Baseline/current Cartesian probe outputs and allocation counts agree;
its ELF text section grew 292 bytes and its combined text/data/bss grew 4 bytes.
This is an additive capability, not a numerical refactor or solver migration.

Definitions currently use finite nesting in Rust types. Chains with lengths
chosen at runtime, broader client qualification, width/midpoint/radius/norm and
measure operations, specialized kernels and full legacy performance/size gates
remain work toward the unchanged complete plan. No phase is complete and no
push was attempted. Source commit:
`2d92066ae55632a99b298ff7c359b01728038b05`. Immutable evidence commit:
`0aa57313971c2c49bb84930ad42b5267712ca763`, record `docs/evidence/2026-11-12-interval-affine-images`,
115 files, 19,983,041 bytes, SHA-256
`0671e133ed1c8a4a44d351962580d86ba825db7274ff7818c7398ca715b6a99f`. The committed record was regenerated and
verified before registration; all prior archive rows are preserved.

## Phase 6 admission: runtime affine chains

Previous goal turn made progress: retained affine images and evidence committed.
Baseline source `948aed757ecfe417c7f82e280099d578740dcb6c`; all four managed
repositories are clean on `interval-refactor`, with exact states recorded in
`/tmp/interval-refactor/affine-chains/initial-state.json`. No nested owning
AGENTS instructions were found. Reuse search found the retained image pullback,
budgeted map copying and checked serde visitors; no bounded affine chain exists.

Owner: Hyperlattice set-valued linear structure, delegating all scalar work to
Hyperinterval. Properties P1–P3, P5–P6, P17–P20, P25–P27, P35–P39; plan §5.2.
Missing witness: a runtime-selected number of retained point maps must compose
without changing the Rust type, losing shared parameters or projecting between
steps. Baseline API has only statically nested images. Add bounded inline chains
of square point maps over any existing exact affine source. Rectangular maps
remain available before/after the chain. Capacity is a declared storage limit,
not a mathematical completeness bound. Capacity zero must retain the source,
including fixed scales unable to encode +1; do not synthesize a rounded identity.

Keep source/map definitions, parameter restrictions, structural identity and
checked native replay. A failed append leaves the chain unchanged; its moved map
is consumed (as documented). Owned restriction copies use the scalar context.
Enclosure traverses maps backwards with the existing matrix product/translation
operations and shared context; it never replaces retained maps with coefficient
bounds. Primitive work is dimension/depth bounded and allocation-free. Dyadic
queries share work/size accounting; rational heap/time, serde, explicit clones,
and general cancellation remain without an internal total bound. No caches.

Acceptance: independent integer-corner expectations across runtime depths and
backends; off-grid recovery, empty and zero-dimensional sources, zero capacity,
capacity exhaustion/rebuild, mixed static/runtime chains, original-parameter
restriction, budget failure/retry, fresh-process replay with malformed maps and
lengths, primitive allocation and portable build checks. Existing scalar,
Cartesian and static affine arithmetic must remain unchanged. Full Phase 6 and
the 15-phase goal remain open. Raw staging is the directory named above.

## Phase 6 runtime affine chains: local qualification

`AffineChain<I,S,N,CAP>` now retains a runtime-selected number of exact square
maps over any admitted affine source, including static rectangular images and
nested runtime chains. Zero maps retain the source without scalar identity
coefficients. Append exhausts only at its declared capacity and preserves the
chain; the moved rejected map is dropped as documented. `into_parts` permits
moving the original source/maps into a larger-capacity chain without cloning.
Restrictions intersect original parameters and charge owned scalar copies.
Enclosure iterates backwards, composing temporary coefficient bounds before
source evaluation. No intermediate Cartesian box replaces the exact source.
Serialization stores source plus actual maps, validates every map, and checks
the receiving capacity; no unused slots/count/cache flags are trusted.

The final owner suite passes 35 tests and two doctests. Six scalar backends
complete all 781 words of lengths 0–4 over five noncommuting/invertible/singular
maps against independent integer corner calculations. Mixed rectangular/static/
runtime chains run through depths 0–8; tests cover original-domain restriction,
empty/unit products, fixed scales unable to represent +1, off-grid recovery,
overflow followed by a valid collapse, capacity failure/rebuild, and shared
dyadic work/size exhaustion with exact-work retry and charged copies. A fresh
process reloads a 33-map off-grid chain into capacity 40 and feeds it into
runtime and static further operations. Malformed maps at every position fail
even with empty sources, as do capacity/shape/scale errors.

All 44 selected checks pass: Rust 1.88/stable feature isolation, Rust 1.95 all
features, release tests, no-default/all-feature Clippy, documentation with
warnings denied, formatting, three Hyperlattice facade tests, four unchanged
solver migration/dispatch tests, package file listing/dependency closure, and
ARM/Wasm subsets on 1.88 and stable. ASan with leak detection passes the owner
suite including both fresh-process replay children. Owner coverage is 779/789
lines (98.73%), 1008/1055 regions (95.55%) and 157/157 functions. The new chain
implementation has 116/116 lines and 152/156 regions covered. No branch
denominator was emitted. Tests, conformance, dependencies and toolchains are
excluded; the 100% and directed-fuzz targets remain open. This ASan run is the
deterministic suite, not a directed fuzz campaign.

The separate Rust 1.88 no_std Wasm consumer links without an allocator/imports.
It passes 73,836 runtime-chain calls (33 effective depths, 513 unique clamped
inputs), plus 2,051 calls to each existing export, with a one-million-fuel limit
per call. Its three-export artifact is 28,300 bytes, SHA-256
`ee696ee92cef75463c7db6e704d05b905b2ca6e30597e363f97ec5babcb7a1c6`.
The extra export increases this consumer by 10,139 bytes relative to the prior
two-export artifact; these are different completed workloads, not a same-contract
size comparison. No registry release, production firmware migration or HIL claim.

The runtime cost probe records 800 samples per run: four backends, map counts
0/1/4/16/32, enclosure/restriction, 20 samples of 128 completed operations after
64 warmups. Primitive Q16 and strict-f64 paths allocate zero. Capacity-32 inline
footprints are 2,336/4,664/3,632/13,976 bytes for Q16/f64/rational/dyadic,
respectively, independent of active count. They exclude heap and are not peak
stack measurements. Initial and final runs are retained; initial timings overlap
other validation on the shared host. No speedup or universal latency bound is
claimed. Query scratch is independent of this chain's active count by iterative
construction; nested sources still add stack frames. Rational operations,
validation/serde, explicit cloning and general cancellation remain without an
internal total-resource contract.

This additive carrier leaves 49 tracked scalar/Cartesian/legacy source files
byte-identical to baseline. Original affine map/image arithmetic, pullbacks,
serde implementations and dot/array helpers are preserved; the main facade
only adds the shared re-export. No numerical refactor or solver/firmware
production migration occurred. Full legacy benchmarks, successive_operations,
monomorphized-size gates and broader consumer qualification remain open with
width/midpoint/radius/norm/measure work. No phase is complete; no push attempted.
Source commit: `ba91b35d57278475d5b4e8f1960f371955d5dd69`.
Immutable evidence commit: `8e82e99b277564f434d34d38b065720020731b6e`, record
`docs/evidence/2026-11-12-interval-affine-chains`, 116 files,
24,482,321 bytes, SHA-256
`7567ad6632e2f40051e98822d916b49a083b19209f29fae6dbb2743a7c592643`. This committed record was regenerated
and verified before registration; all prior archive rows are preserved.

## Phase 6 admission: norms, matrix measures and dominance margins

Previous goal turn made progress: bounded runtime affine chains and evidence
committed. Revalidated source `5336b85d2c23bb71c593e3e521435471a0ed4123`;
all four managed repositories are clean on interval-refactor. Exact state and
commands are staged under `/tmp/interval-refactor/lattice-norms`. No nested
owning AGENTS instructions. Reuse inspection found scalar absolute value,
maximum image, sum and difference for all six arithmetic backends; no Cartesian
one/infinity norms, logarithmic matrix measures or dominance-margin helpers.

Owner: Hyperlattice Cartesian linear structure, with all scalar arithmetic and
resource accounting delegated to Hyperinterval. Properties P1–P3, P5–P6,
P8–P11, P17–P20, P25–P27, P35–P39; plan §5.2. Domain: finite Cartesian vectors
and rectangular matrix sets, including canonical empties and zero-dimensional
unit products. Norm outputs enclose the full scalar norm range, not merely an
upper bound. Matrix one/infinity norms are maximum absolute column/row sums.
For square positive dimension, logarithmic measures are the maximum of signed
diagonal plus absolute off-diagonal column/row sums. Diagonal-dominance margins
are absolute diagonal minus absolute off-diagonal sums, as interval vectors.

Rational/dyadic successful results are exact ranges because the independent
Cartesian coordinates attain each term's extrema simultaneously; max reductions
use disjoint rows/columns. Primitive sums may round outward or overflow. Empty
input has empty image; unit products have zero norm. For nonempty 0×0 input,
the norm-limit definition of a logarithmic measure tends to negative infinity;
the finite output API reports scalar UnboundedImage, never a false zero. Zero
size dominance vectors retain unit-product semantics. Negative measures must
not be clamped by initializing their maximum to zero.

No solver certificate/status, policy choice or implicit midpoint enters this
layer. Outputs remain consumable as scalar/vector enclosures and retain typed
errors. All dyadic scalar work uses one caller context, including owned copies;
failed queries must leave inputs reusable. Loops are dimension bounded; primitive
paths must allocate nothing. Rational heap/time, general cancellation and
constructor/serde validation retain their existing disclosed limitations.

Missing witnesses: independent exact minima/maxima of absolute values (including
interior zero), signed measures on all-negative rows, nonsquare norm orientation,
zero-sized/empty distinctions, dominance margin ranges, fractional/extreme inputs,
shared exhaustion/retry, consumption of affine projection outputs, checked native
replay and portable no-allocation clients. Existing arithmetic/predicate order and
serializations must remain unchanged. Full Phase 6 and the 15-phase goal stay open.

## Phase 6 norms and matrix measures: local qualification

Vector and matrix `norm_one`/`norm_infinity`, square-matrix `measure_one`/
`measure_infinity`, and `row_dominance_margins`/`column_dominance_margins` now
return scalar/vector interval evidence through the shared Hyperlattice facade.
The backend adds only pass-throughs for existing scalar absolute/max images.
All prior scalar/Cartesian/affine arithmetic, serde and predicate order remain
unchanged: 52 tracked source files are byte-identical, and the two existing
source files edited contain additions only. No scalar algorithm was duplicated.

The final owner suite passes 45 tests and three doctests. For all six backends,
125 three-coordinate interval cases and 625 two-by-two interval cases compare
full norm/measure/margin ranges against independent integer point enumeration.
The intervals have integer endpoints and the absolute-value breakpoints are
zero, so those witnesses attain the continuous piecewise-linear extrema;
interior minima are checked, not just corner maxima. A nonsquare 2×3 case,
negative measures, empty/unit products, all zero-dimension orientations, exact
fractional rational answers, fixed extrema, IEEE subnormals and outward-rounded
sums extend the corpus. Primitive overflow is explicit, including absolute-value
and sum overflow; larger exact backends retain their own scalar contract.

All eight reductions share dyadic work/size budgets. One-unit-short work limits
fail, exact measured work succeeds, and retries preserve the original matrix.
Margin-output then norm-input uses the same budget without reset. A fresh child
replays matrix inputs, recomputes measures/margins, and replays/reuses output
ranges. A retained affine `(x,1-x)` construction remains separate from its box:
the projection norm is `[0,2]`, while further exact summation is `{1}`. No norm
of a projected box is mislabeled as the exact correlated image's norm.

All 44 selected checks pass: Rust 1.88/stable feature isolation and portable
ARM/Wasm subsets, Rust 1.95 all features, release tests, Clippy with warnings
denied, docs, fmt, four facade tests, four unchanged solver migration/dispatch
tests, package listing and primitive dependency closure. ASan with leak detection
passes the owner suite and fresh-process replays. LLVM reports 920/935 owner
lines (98.40%), 1234/1300 regions (94.92%), and 182/182 functions; the new norms
file has 117/122 lines and 193/212 regions. No branch denominator. Test,
conformance, dependency and toolchain code are excluded; the annotated norm
report and raw profiles are retained. The 100% and directed-fuzz targets remain
open; this ASan run is the deterministic suite. Initial Clippy rejected a range
index loop; direct row/column iteration fixed it without relaxing any lint.

A standalone Rust 1.88 no_std Wasm consumer links with no imports/allocator.
Its new norm/measure export passes 2,051 calls (255 distinct effective inputs),
alongside all 73,836 runtime-chain and 4,102 existing static-export calls, each
with an external one-million-fuel limit. Four-export artifact: 39,592 bytes,
SHA-256 `d41badc0780d71a287e3522d8f90f3e14c7c022088ce50a518f2aeb7e1163df6`.
Adding the export grows the previous three-export consumer by 11,292 bytes;
these are different workloads, not a same-contract binary-size comparison.

The cost probe records 640 samples: four backends, eight queries on dimension-4
inputs, 20 samples of 128 completed operations after 64 warmups. Inputs are
constructed outside timing and all returned scalar/vector outputs are consumed.
Independent exact fixture expectations pass. Q16 and strict-f64 queries allocate
zero. Dimension-4 vector/matrix inline bytes are 48/192 (Q16), 96/384 (f64),
72/264 (rational), and 288/1152 (dyadic). These exclude heap and do not measure
peak stack. Raw times, allocations, compiler/CPU, native probe size and hashes
are retained; no speedup, uniform latency or full legacy-size claim.

No production solver/firmware migration, implicit midpoint or policy decision
was introduced. Width/midpoint/radius, contraction/Jacobian helpers, specialized
kernels, measured client adoption, full legacy benchmarks/successive_operations/
monomorphized-size gates and the remaining plan phases stay open. General
cancellation and arbitrary rational heap/time bounds remain unimplemented;
dimension and dyadic scalar budgets are not mislabeled as those bounds.
No phase is complete and no push was attempted. Source commit:
`a5b7a4963fa98e765ccbcc32999a7327534ccec4`. Immutable evidence commit:
`fdd4e1c0691546d52c0735a4fc5b8352416642b4`, record
`docs/evidence/2026-11-12-interval-lattice-norms`, 122 files,
27,524,231 bytes, SHA-256
`6f9d27889d8f0115360e0f052fc31d645d932e0b85ef63b2b3d22c86a85e1a2e`. The committed record was regenerated
and verified before registration; all prior archive rows are preserved.

## Phase 6 admission: exact box widths and widest-axis selection

Previous goal turn made progress: norms/measures/margins and evidence committed.
Revalidated source `6768c26ae0f995973834e50be761e1fe2969c242`, with all four
managed repositories clean on interval-refactor. Exact states and raw evidence
are staged at `/tmp/interval-refactor/lattice-widths`. No nested owning AGENTS
instructions. Reuse search found fixed width_ulps, rational/dyadic exact width,
the shared integer IEEE decoder and budgeted dyadic_order::compare. No box
width/widest-axis API exists, and strict float endpoints cannot represent every
exact width. Full fixed signed-range widths also exceed signed endpoint storage.

Owners: Hyperinterval exact scalar measurements/order; Hyperlattice coordinate
selection. Properties P1–P6, P8–P11, P17–P20, P25–P27, P35–P39; plan §5.2.
Domain: every admitted finite Cartesian box, including empty/unit products,
singletons, full endpoint ranges and arbitrary exact rational/dyadic data.
Fixed widths use a scale-tagged u64 numerator. Float widths retain their checked
endpoint difference, with mathematical value ordering independent of defining
interval position. Compare two widths by the exact signed sum b+c-a-d using
bounded integer limbs and the existing IEEE decoder; no native float decisions,
rounded width comparison, allocation or new scalar kernel in an adapter.
Rational/dyadic width values remain exact singleton intervals; dyadic point
comparison reuses existing budgeted ordering without expanding irrelevant gaps.

Box widths return None for empty and Some([]) for the zero-dimensional unit;
widest-axis returns None for either case. Nonempty positive dimension selects
the lowest-index maximum, including index zero when every width is zero.
Every dyadic construction/comparison uses one caller context and failure yields
no partial selection. Primitive work/storage is dimension/encoding bounded and
allocation-free. Float ingress keeps its existing target qualification. Exact
widths remain usable when optional projection into endpoint storage overflows.
Replay validates definitions/scales, not cached flags; equality of width values
is separate from byte identity of different defining float intervals.

Missing witnesses: full-range fixed widths, float overflow/subnormal/tiny width
differences and exact ties invisible to rounded subtraction, independent exact
integer/Rug comparisons, mixed exponent dyadic selection and budget retry,
empty/unit/singleton/tie behavior, output-as-input and fresh-process replay,
primitive no-allocation and portable builds. Existing arithmetic and predicate
order must remain unchanged. Width facts supply scheduling evidence; no solver
status or implicit midpoint is introduced. General cancellation/rational heap
bounds and the remaining Phase 6/full-plan requirements stay open.

## Phase 6 exact widths: local qualification

Implemented exact coordinate widths and deterministic widest-axis selection for
all six backends. `FixedWidth<F>` retains an unsigned numerator and binary scale;
`F32Width`/`F64Width` retain defining endpoints and compare mathematical distances
with bounded integer sums. Explicit projection can widen or overflow without
losing the exact width. Rational/dyadic widths are singleton intervals. Dyadic
singleton comparison shares the existing order kernel and caller budget.
`IntervalVector::widths` and `widest_axis` preserve empty/unit distinctions and
lowest-index ties. No solver status, midpoint, native float subtraction or
replacement of the retained source is introduced. Float nonempty ingress remains
qualified only on x86_64; this work does not qualify another float target.

Acceptance: 3,375 independently expected boxes per backend; 12,025 GMP rational
width comparisons per IEEE format; full fixed ranges, translated equal widths,
rounded-projection ties, projection overflow, subnormals, invalid replay,
shared dyadic work/size failures and retry, extreme exponent gaps, fresh-process
save/reload and further consumption. All 16 x86 rounding/FTZ/DAZ combinations
pass, including optimized builds on Rust 1.88, 1.95 and stable. The final lattice
suite has 52 tests and four doctests; the scalar all-feature suite also passes.
All 56 selected local/downstream commands pass: feature isolation, Clippy with
warnings denied, docs, formatting, release tests, five main-facade tests, four
existing Hypersolve migration/dispatch tests, packaging/dependency checks, and
ARM/Wasm builds. This is not the complete pre-push workflow set. No push.

Sanitizer evidence is deliberately scoped. Scalar width/float tests (24) and
all lattice integration tests (52) pass ASan with leak detection. A broader
scalar run stops after five successful arctangent tests with 1,576 leaked bytes
in three MPFR cache allocations. The same failure reproduces on unchanged
baseline `6768c26`; symbolized frames identify `mpfr_cache`, `mpfr_atan` and
Rug's pi/atan oracle paths. Test-oracle cache cleanup remains open; the broad
sanitizer run is not a pass. A cross-worktree shared-target stale artifact caused
one focused compile failure; targeted package-cache cleanup and rebuild resolved
it, with both records retained. The initial baseline attempt without an explicit
sanitizer target also failed in proc-macro compilation; the explicit-target
baseline reproduction is the controlling result.

Instrumented owner coverage reports 5,230/5,331 lines (98.11%), 9,717/10,360
regions (93.79%), and 650/652 functions. The new lattice widths file has 31/31
lines and 49/50 regions. LLVM reports ten mismatches, all traced separately to
the same external Rug `OptRational::parts` function (hash zero), which is outside
the declared report denominator. Raw warnings and per-object diagnostics remain;
this is owner coverage with a dependency warning, not a warning-free aggregate
or 100% claim. Directed fuzz and broader qualification remain open.

Completed-work cost probe: 160 samples across four backends and two queries,
128 calls/sample after 64 warmups, dimension four. Fixed Q16 and strict f64
allocate zero; measured rational small integers also allocate zero, which does
not bound general rational data. Dyadic widths/selection use 6/7 allocation
calls and 48/56 bytes per measured call. Inline source/width-array bytes are
48/32 fixed, 96/64 f64, 72/64 rational, 288/288 dyadic; these are not peak stack
or retained-heap measurements. No speedup or full legacy benchmark claim.
The five-export no_std Wasm consumer has no imports/allocator and passes 2,051
width calls over 256 effective inputs, alongside all previous probes, under a
one-million-fuel per-call cap. It is 44,726 bytes, SHA-256
`a2aa1b70e2a0528852146afd52d2aea03f97cbe55b96881ea7af7858d2c57324`.
Its extra export changes the workload; the 5,134-byte increase is not a
same-contract comparison. No firmware HIL or registry-release claim.

Source preservation records show 47 existing implementation files byte-identical
and every original function body/signature in the six edited implementation
units retained, including the IEEE finite-ingress inline barrier. New paths are
additive; existing dispatch order, Real APIs, geometry certainty and production
consumers are unchanged. Full legacy geometry/performance/monomorphized-size
qualification remains open rather than inferred from source identity.

Source commit `e529f241f120a388e439699b371d32b67e697350`; immutable evidence
commit `6d7ef14df566df76396b5386f21fa64b8eef2d3a`, record
`docs/evidence/2026-11-12-interval-lattice-widths`: 284 files,
85,234,615 bytes, SHA-256
`67402bc6d0d36a9990b87857c8bf1e64112fa862db3d51d37696c0d02e6bc138`. The committed record was regenerated
and verified before registration; all previous archive rows remain unchanged. Phase 6 still needs
midpoint/radius forms, contraction/Jacobian/Krawczyk helpers, specialized consumer
kernels and further qualification. No phase or full-plan completion is claimed.

## Validation admission: MPFR oracle thread-cache lifecycle

The preceding goal turn made progress: exact widths and evidence were committed.
Current baseline is `1b62459197944b4ebd3356054a560d1e0af97f07`; all managed
repositories are clean. States and new command logs are staged under
`/tmp/interval-refactor/mpfr-cleanup`. No nested owning AGENTS instructions.
The broader scalar ASan failure from the width checkpoint is being reproduced
on this revision before any test code changes.

Owner: Hyperinterval's independent MPFR test oracles; shared lifecycle authority
already exists in `hyperreal/tests/support/mpfr.rs`. Properties P5–P6/P20 and
plan §§14.4–14.5 require cleanup and executable sanitizer evidence. Domain:
Rug/MPFR oracle use on non-MSVC test threads, normal return and unwinding,
including concurrent threads and repeated registration. Reuse that existing
thread-local cleanup helper once per oracle executable, registering it before
each entry that starts MPFR work. Preserve cached work until thread exit; free
only Local caches, never shared caches used by other running threads. No copied
cache lifecycle implementation, unsafe code, suppression or disabled leak check.

Acceptance failure reproduced: the current arctangent process reports 1,576 bytes leaked
from MPFR thread caches; a lifecycle regression must exercise normal and panic
exit and confirm that exact oracle outputs remain valid. Audit all Float call
sites in the owner tests, retain original precision, rounding, calculations,
assertions and oracle independence. Production source/manifests/dependency graph
must remain identical. These support modules are also used by cost probes and
fuzz targets. Each affected consumer root must import the same existing helper;
oracle entry points register that owner. This avoids duplicate thread-local
cleanup owners when an executable includes several oracle modules. Validate feature isolation, MSRV, ordinary tests/lints,
ASan across both owner packages, and output/dispatch consumers. This test-only
cleanup does not establish any new geometric or arithmetic capability, speedup,
full-plan coverage, hosted CI or firmware qualification.

## MPFR oracle lifecycle: local qualification

Reproduced the width checkpoint's leak on `1b624591`: all five arctangent tests
pass, then ASan exits 1 with 1,576 bytes leaked in three MPFR cache allocations.
Reused `hyperreal/tests/support/mpfr.rs` at the roots of affected integration
tests, cost probes and fuzz executables. Every MPFR oracle entry registers its
single executable-local thread-cache owner before use. No production algorithm,
manifest, dependency, precision, rounding mode or oracle assertion changed.
Normal thread exit and unwinding release Local caches; shared caches are not
cleared while other threads may be active. A new eight-worker regression tests
repeated registration, normal/panic exit, exact oracle equality and returned
rational values consumed after their producing threads have exited.

The final full scalar/lattice ASan run passes 206 parent-suite tests across 36
executables (154 Hyperinterval, 52 lattice), including fresh-process replay,
with `ASAN_OPTIONS=detect_leaks=1`. This closes the reproducible MPFR lifecycle
blocker reported above; it does not rewrite that adverse historical evidence.
All 37 selected commands pass: Rust 1.88/1.95/stable feature isolation,
all-target tests, doctests, Clippy/docs with warnings denied, optimized owner
tests, full lattice tests, five main-facade tests, four unchanged Hypersolve
migration/dispatch tests, primitive/rational fuzz checks, fuzz Clippy/format and
workspace format. Rust 1.88 uses portable features; Hyperreal retains MSRV 1.95.

All fuzz targets build with the address sanitizer. Nine affected targets each
complete 128 executions from 32 deterministic valid-length seeds, with leak
detection, ten-second input timeout, 2,048 MiB RSS cap and 180-second external
process limit. These are bounded smoke tests, not exhaustive fuzz qualification.
Raw inputs/corpora, commands, outputs and binary digests are retained.

Two integration attempts failed and remain in the record: initial test-root
wiring omitted cost/fuzz consumers, and per-oracle helper imports produced
Clippy duplicate-module errors when several oracles shared an executable.
The final one-owner-per-executable wiring fixes both without suppressions.
Source preservation verifies 39 implementation/manifest/shared-helper files
byte-identical and every original line of 37 changed oracle/consumer files
retained in order. Production cost closures and predicate order remain intact;
no performance or binary-size improvement is claimed. No new coverage report
replaces the preceding width report's documented external Rug profile warning.
Hosted OS checks, full pre-push workflows, full coverage/fuzz, firmware HIL and
remaining arithmetic/resource/Phase 6/full-plan work remain open. No push.

Source commit `2b435bb06dcc78fae754a3577747ea8e43c831ae`; immutable evidence
commit `a8b1603d831c3f3e391d67a43ddd88a3cb0b9966`, record
`docs/evidence/2026-11-12-interval-mpfr-cache-lifecycle`: 508 files,
441,320 bytes, SHA-256
`374c190ee40b95dac399fc108a0e1bbb8efa5be2b0b58ea03427c7dfed45e459`. The committed record was regenerated
and verified before registration; all previous archive rows remain unchanged.

## Phase 6 admission: retained midpoint/radius decomposition

The preceding goal turn made progress by fixing MPFR lifecycle and restoring the
broad sanitizer suite. Baseline `9b1274ce8141d56764e3d6652540e0fe0833d68a`, all
managed repositories clean; state and evidence at
`/tmp/interval-refactor/midpoint-radius`. No nested owning AGENTS instructions.
Reuse search found fixed quotient_bounds, rational midpoint/radius, dyadic
measure/scaled_quotient and IEEE add_wide. A compile witness reproduces the
missing lattice midpoint_radius method on this revision.

Owners: Hyperinterval scalar projections; Hyperlattice retained Cartesian
forms. Properties P1–P6, P17–P20, P25–P27, P35–P39; plan §§5.1–5.2. Domain: all
admitted finite interval vectors/matrices in six backends, including empty and
zero-dimensional unit products, full signed ranges, subnormals and dyadic
minimum exponents. Each lane defines c=(a+b)/2 and r=(b-a)/2 exactly. Store the
checked source bounds once; decomposition consumes ownership, never substitutes
a rounded center/radius or expands into two independent arbitrary-precision
copies. Borrowed source and owned reconstruction retain the exact original set,
even when a numerical projection cannot fit the endpoint family. This refines
the plan's illustrative C/R type parameters into a source-parameterized carrier.

Expose midpoint_enclosure and radius_enclosure as explicit numerical projections.
Fixed uses existing quotient floor/ceil on widened endpoint sum/difference;
strict IEEE halves decoded exponents before existing add_wide, avoiding sum or
width overflow and retaining underflow direction. Rational uses existing exact
measurements. Dyadic uses exact native measurements except at i32::MIN, where
existing scaled_quotient supplies outward bounds on the minimum stored grid.
Every dyadic projection shares the caller budget; no provisional array escapes.
Projection uncertainty describes the represented center/radius point, not a
replacement for the exact retained form. Fixed radius projection may return
Overflow while exact reconstruction remains valid. No approximate solver status.

Missing acceptance: odd fixed widths, extreme IEEE sum/cancellation/subnormals,
all ambient native modes, minimum-exponent dyadics, independent exact enclosure
tightness, empty/unit and rectangular forms, exact reconstruction followed by
operations, fresh-process replay, invalid replay, shared-budget exhaustion/retry,
primitive no-allocation and portable consumers. Reuse existing array and serde
authorities. Keep all old arithmetic/predicate bodies and order unchanged.
This checkpoint supplies decomposition from bounds and explicit projections;
independently authored center/radius forms, contraction/Krawczyk/Jacobian helpers,
specialized consumers, full resource/coverage/performance gates and later plan
phases remain open. Do not claim a whole phase complete.

## Phase 6 midpoint/radius decomposition: local qualification

Implemented retained `MidpointRadiusVector<I,N>` and
`MidpointRadiusMatrix<I,R,C>` definitions for all six backends. Consuming
`midpoint_radius()` and `into_intervals()` transfer ownership without scalar
copies; borrowed `intervals()` exposes the exact checked source. Empty/unit
semantics and source validation survive native replay. The form's exact center
and radius are defined by its retained bounds; explicit midpoint_enclosure and
radius_enclosure methods return numerical enclosures and never replace that
source. Scalar projections reuse fixed directed quotients, native rational
measurements, dyadic measured halves/minimum-grid quotient rounding, and the
IEEE wide adder with exact exponent halving. Existing arithmetic bodies and
call order remain unchanged. No implicit point or solver status is introduced.

Acceptance passes: 3,375 vector and rectangular-matrix cases per backend with
independent integer expectations and exact weighted products after reconstruction;
fixed half-step tightness and full-range projection overflow; 10,081 source
pairs per IEEE format (two projections each) against independent GMP/MPFR bounds;
minimum/maximum dyadic exponents, shared matrix budgets and retry; empty/unit,
33-lane replay, malformed shapes/scales/order, and fresh-process fixed/float/
dyadic reconstruction followed by further operations. All 16 rounding/FTZ/DAZ
combinations pass on debug and optimized Rust 1.88, 1.95 and stable builds.
The original missing-method compile witness now passes unchanged.

All 59 selected commands pass after correcting the new facade test's helper
scope and rerunning its test, Clippy and formatting checks. Initial failed logs
remain. The helper reuses the two existing identical test constructors without
removing assertions. The matrix covers feature isolation, owner/downstream tests,
Clippy/docs with warnings denied, six facade tests, four unchanged Hypersolve
migration/dispatch tests, release tests, ARM/Wasm checks, package/dependency
inspection and fuzz checks. Full ASan with leak detection passes 218 parent-suite
tests across 39 executables (160 scalar, 58 lattice). Lattice has five passing
doctests. The extended strict-float fuzz target completes 10,000 executions from
52 preserved/new seeds, with timeout/RSS limits and sanitizer leak detection.
These checks do not constitute the complete pre-push workflow or full fuzz gate.

Owner coverage is 5,401/5,503 lines (98.15%), 9,967/10,615 regions (93.90%),
690/692 functions. The new retained-form module has 83/84 lines, 90/93 regions
and 21/21 functions. All 12 LLVM mismatches were individually traced to the same
external Rug OptRational::parts hash-zero function outside the report scope.
Warnings and per-object diagnostics are retained; no 100% or warning-free
all-dependency report is claimed.

The six-export allocator-free Wasm consumer passes 2,051 midpoint/radius calls
with 513 effective inputs and every previous probe under one-million-fuel caps.
Artifact: 61,716 bytes, SHA-256
`e1a8be5cf6a989fdf551cd6c4ecd3a6ece7744e0a8bf4273a3afabe64323dd6d`.
The extra export changes the workload; its 16,990-byte increase over the prior
consumer is not a same-contract regression measurement. The first replay script
had a tuple export-lookup typo; the corrected script passes and both records
remain. No new float target qualification, firmware HIL or release claim.

Completed cost probe: 480 samples, four backends, six vector/matrix actions,
128 calls/sample after 64 warmups. Ownership retention/reconstruction allocates
zero for all backends. Fixed Q16 and strict f64 projections also allocate zero;
small rational fixtures allocate zero without implying a general rational bound.
Dyadic projections allocate six times/48 bytes per four-lane measured call.
Forms retain exactly source inline size: 48/96/72/288 bytes for Q16/f64/rational/
dyadic, both length-four vectors and 2x2 matrices. These exclude retained heap
and peak stack. No speedup or full legacy benchmark/monomorphized-size claim.
Source preservation records 27 byte-identical implementation files and all
original lines retained in order in seven edited owner implementation files.

Source commit `842dc85a98bdadc50dfe63b42dd97ffade81ad7b`; immutable evidence
commit `8acb2801a9a4fd61c779ad38554a02bf0ac2305f`, record
`docs/evidence/2026-11-12-interval-midpoint-radius`: 417 files,
94,306,422 bytes, SHA-256
`551d7e0beb8ca69c9fec6c7f5152a851865c5d3ac34804ca11df870f941347a4`. The committed record was regenerated
and verified before registration; all previous archive rows remain unchanged.
Independent authored center/radius forms, contraction/Krawczyk/Jacobian helpers,
specialized consumers, full resource/coverage/performance qualification and the
remaining plan phases stay open. No phase completion or push is claimed.

## Phase 6 admission: contraction evidence and identity remainders

Baseline `be11e97da74bf8221dbe18e6c17f1c54e5ce9b6b`; all four managed
repositories were clean before edits. Initial states and absent-upstream results
are retained in `/tmp/interval-refactor/contraction-primitives`. Owner:
Hyperlattice's interval companion, with scalar constants/arithmetic delegated to
Hyperinterval. Properties P1, P3, P5, P6, P18–P20. Domain: checked finite square
Cartesian matrices over the six existing real interval backends, empty sets and
zero-dimensional units. The baseline compile witness fails for all three missing
methods: `identity_minus`, `contraction_margin_one`, and
`contraction_margin_infinity`. Existing norm reductions and scalar constructors
were found and reused. No production solver or predicate-dispatch change.

The helpers enclose `Id-A`, `1-||A||_1`, and `1-||A||_infinity`. No Jacobian,
self-mapping, root-existence or uniqueness status is synthesized. The exact one
constant must fit the backend: nonempty margins and positive-dimensional identity
subtraction report overflow for fixed i32/F31 and i64/F63. Empty stays empty;
0x0 identity subtraction needs no scalar one. Dyadic arithmetic shares one
budget, with constant construction outside accounting as for existing zero.
Rational arithmetic has no internal total-work bound. Primitive loops and inline
storage scale with dimension; partial outputs are never published.

Local qualification passes 625 independently enumerated square matrix boxes per
backend, exact identity reconstruction and transpose duality, fractional
coupled-quadratic Krawczyk images with unequal radii and further narrowing,
a two-root adversary, a nonzero residual, fresh-process replay, full-scale fixed
failures, subnormal IEEE tails, zero-containing margins, and shared dyadic
work/size exhaustion followed by exact-budget retry. The original missing-method
witness now compiles and runs unchanged. No tests or existing assertions were
removed. Primitive fixed/f32/f64 allocation checks remain zero.

All 49 selected final commands pass: Rust 1.88/stable feature isolation,
all-feature 1.95/stable/release tests, owner/facade Clippy and docs with warnings
denied, seven Hyperlattice facade tests, four unchanged Hypersolve migration/
dispatch tests, ARM/Wasm checks, formatting and package/dependency inspection.
The first feature-only runs exposed an unused new test import; its cfg fix
required import formatting. Initial warnings and the formatting failure remain
in the record. The complete selected matrix was repeated after both fixes.
ASan with leak detection passes 64 parent-suite tests across 17 executables;
all-feature doctests add five passes. Companion coverage is 1,157/1,177 lines,
1,527/1,609 regions, 232/234 functions. The new module has 39/42 lines,
63/74 regions and 5/7 functions; its two never-entered zero-dimension array
closures are included in the denominator. No 100% or complete fuzz claim.

The seven-export no-allocator Wasm consumer passes 2,051 new contraction calls
over 513 effective inputs, plus every prior probe, with one-million-fuel caps.
Artifact: 69,039 bytes, SHA-256
`621b2807b75718344902512512ac123b8e0bb8964189fc0787f9860bc8bc68a2`.
The 7,323-byte increase includes a new export/workload and is not a same-contract
size comparison. The system Python lacked wasmtime; the recorded retry with the
existing tools virtual environment passed. No additional float-target or HIL claim.

The extended norm-cost probe records 880 samples, including 240 new-operation
samples (four backends, three actions, twenty samples each; 128 completed calls
per sample after 64 warmups). Q16/f64 allocate zero. On the 4x4 fixture, rational
identity subtraction allocates 24 times/960 bytes and each margin six times/240
bytes; dyadic identity subtraction allocates 42 times/360 bytes and each margin
94 times/776 bytes. These are fixture measurements, not general bounds, latency
guarantees or comparative speed claims. The record includes timing samples,
inline sizes and executable size/digest/sections. Source preservation confirms
168 byte-identical scalar/main-lattice/solver/companion implementation files and
all original lines retained in order in the two extended companion files.
Full legacy benchmarks, successive_operations and monomorphized/binary-size
comparison gates remain open; no production migration or behavior refactor.

Source/evidence registration pending for this checkpoint. Independently authored
center/radius forms, Jacobian carriers, specialized kernels, the full consumer/
resource/performance gates and later phases remain open. No phase completion
or push is claimed.

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

