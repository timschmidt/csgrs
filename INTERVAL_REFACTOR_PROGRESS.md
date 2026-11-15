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
| 4 proposals/float | proposal wrappers and strict integer IEEE core implemented; local x86_64 and WASM qualification passed | full surface/coverage, further targets and shipped consumers |
| 5 elementary functions | abs/min/max, integer powers, square roots, exp/log/atan/sin/cos, split division, rational powers and tan/cot implemented on documented backends | final function/backend coverage, resource and consumer qualification |
| 6 lattice | Cartesian/affine carriers, widths, center/radius forms, norm/contraction evidence, source-bound Jacobian projections and FOC/Krawczyk clients implemented | broader authored-form composition, facts/specialization and full consumer/performance gates |
| 7 solver | source-bound Jacobian matrix adapter qualified; kernel migrations remain rejected | satisfy migration size/performance gates with unchanged statuses, certificates and dispatch |
| 8 interface | BoundedMeasurement constructor delegates to Hyperinterval; local qualification passed | machine-profile arithmetic, named budgets, full browser/schedule and canonical-profile gates |
| 9 firmware | clock/Q30/servo delegation retained; five-board Xtensa links and image comparisons reproduced | 4 MiB flash fit, whole-call stack (including +16-byte local changes), MCU cycles/HIL and remaining audit/release gates |
| 10 predicate filters | initial native-stage experiments and exact admission corpus qualified; two placements rejected | useful native-stage proof/cost/trace admission and representative geometry workloads |
| 11 path dynamics | complete extraction candidate implemented; retention gate open | reproduced owner timing regressions; preserve exact reports, dispatch and canonical lowering while resolving costs |
| 12 ODE/reachability | owned fields/Jacobians, Picard tubes/end enclosures, bounded native integration in retained exact state charts, rational clocks, RK proposals and finite epsilon endpoint covers implemented and locally qualified | wider reachability/wrapping, whole-resource closure and remaining platform/consumer qualification |
| 13 Taylor/adoption | exact coefficients/remainder, source-bound affine forms and nonlinear program composition, polynomial arithmetic over existing Taylor proofs, certified composition of retained Taylor models, validated trajectory Taylor steps and fixed-order whole-horizon composition, bounded automatic order/duration selection with local remainder limits, direct exact chart transport, certified QR frame construction and bounded two-chart automatic evolution with exact endpoint-diameter selection, fixed state charts/Picard and whole-horizon transport, native SDF cell/Taylor enclosures for smooth expressions, polynomial primitives and affine frames, plus bounded adaptive covers, a measured three-strategy dynamics corpus and circuit residual/Jacobian enclosures with component tolerance bindings plus continuous point-mass flow with retained force/mass evidence and sensitivity-retained noise columns for static affine field-image queries and explicit sharing of prepared trajectory orders, retained after local qualification | higher-utility automatic frame strategies, ODE fresh-noise propagation and global sensitivity, automatic-controller cross-order preparation reuse, general arithmetic across nonlinear remainder models, wider adoption and competitive completed-workload portfolio qualification remain open |
| 14 release | Interface native/browser lint and local packaging; 21 packaged crates and 26 clients on Rust 1.95/stable, clean archive reproducibility, 42 interval feature checks, dedicated Hyperdynamics workflow locally qualified | whole-stack CI/feature/platform qualification, hosted/HIL consumers, publication and remaining package gates |

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

Source commit `fb5806c4d156dd267d934b480d6f5a2aac364820`; evidence commit
`583af02f69c59579ec8fa5e7b9f9549ce09db002`, record
`docs/evidence/2026-11-12-interval-contraction-primitives`: 226 files,
43,387,673 bytes, SHA-256
`0a72167e0272dade29560b118f8d033516ca41ad905eb5ac23ccd8c006dc2914`. The committed archive was regenerated and
verified before registration; all prior archive rows remain unchanged. Independently authored
center/radius forms, Jacobian carriers, specialized kernels, the full consumer/
resource/performance gates and later phases remain open. No phase completion
or push is claimed.

## Phase 6 admission: maintained Alumina FOC client

Baseline CSGRS `a4560f1c9209396bc4409d0dc5d5f2fe92db2225`, firmware
`bdee397d0b80841f16bf17e7bcd029b4feb2c212`. Initial state records all four
managed repositories clean, their branches, heads and absent upstreams.
Owner: standalone Hyperlattice interval conformance client; Alumina retains
wrappers, rotation validation, balance policy and FocError semantics. P1, P5,
P6, P18–P20, P31–P33. Missing acceptance: Phase 6's maintained Clarke/Park
conformance and benchmark client. Existing fixed conversion, wide i64 storage,
checked scalar operations and matrix products were found and reused. No scalar
kernel or domain validator was copied into production. Raw staging:
`/tmp/interval-refactor/foc-client`.

The pre-edit witness reproduced two mismatches in naive substitutions. For
alpha=MIN, beta=0 and sine=-1/cosine=0, native Park rejects an intermediate
MIN*-1 while a signed dense matrix succeeds. For phases (1,0,-1), pre-rounded
2/3 and -1/3 coefficients widen the alpha output by one ulp on both sides while
native Clarke returns exact one. Both original witnesses remain deterministic
tests. The new client retains rounded product order, subtraction boundaries and
widened Clarke numerators until division/scaling. The public result/error types
remain Alumina's, and every arithmetic operation delegates to the shared layer.

## Phase 6 FOC client: correctness and cost qualification

Implemented the unpublished `conformance/alumina-foc` standalone package with
its own lockfile, no_std library, independent integer oracles, paired native/
shared cost and footprint binaries, and an allocator-free Wasm consumer.
The package uses four local dependencies and does not change any production
dependency graph. The exact domain is valid nonempty Q2.30 input intervals,
public validated Rotation values and all signed zero-sequence tolerances.
Constant-size loops and i128 intermediates preserve the original typed range
and balance failures. No implicit midpoint, floating demotion, policy or proof
status is introduced. The wide Clarke linear numerators have magnitude below
2^34 and fit i64; their fixed products and scaled divisions fit i128.

The endpoint and seeded transform corpus checks 399,950 full outcomes: 95,848
successful transforms, 26,366 Range outcomes and 277,736 balance rejections.
Every result and typed error agrees with firmware; successful outputs also
contain independently enumerated exact integer corners. The scalar corpus
checks 12,025 interval pairs for products and divisions, both native i32 storage
and shared narrow/widened paths, against exact rational extrema. Seed: 20261112.
Tests preserve radical-constant enclosure inequalities, full i32 endpoints,
continuous admitted rotations, balance rejection priority, original hard cases,
and 64 successful five-transform chains with each intermediate compared.
The allocation counter covers complete workloads and representative error paths;
all measured native/shared paths allocate zero.

All 24 final commands pass, including Rust 1.88/stable debug/release tests,
Clippy with -D warnings, docs with warnings denied, ARM/Wasm checks and Clippy,
both package format checks plus workspace formatting, the fixed-only companion
suite (23 tests/doctests), and all 83 original FOC tests. Initial and final
checks are retained; final repeat follows added allocation-error cases. ASan
with leak detection passes all seven client tests. Source coverage for the
client library and samples is 172/174 lines (98.85%), 382/401 regions (95.26%)
and 17/17 functions; dependencies/tests/probe binaries are outside that report.
No complete coverage, new fuzz campaign, hosted-OS or HIL claim.

The actual no-allocator Wasm module has no imports and passes 1,660 completed
transform calls across a 165-fixture period plus 405 full-range Park checks
(335 successes, 70 typed overflows). A separate Python exact-integer oracle
checks every output digest and expected overflow, in addition to native/shared
full-value assertions inside Wasm. The one-million-fuel cap was not reached;
maximum observed fuel is 28,172. Artifact: 43,086 bytes, SHA-256
`67c36ef145e556a2b7930f747c7b560bcbea273b58d3cb9c5ef6726f5bf94c5e`.
No firmware cycle, stack or additional float-target qualification is implied.

Matched completed-work cost: 200 samples per compiler, five actions, two
implementations, twenty samples per action/implementation, 512 completed calls
per sample after 64 warmups. Inputs are prebuilt; timing includes full output
materialization and digest work. All outputs/checksums match and allocations are
zero. Rust 1.88 shared/native median ratios are 3.31 Clarke, 0.71 Park, 0.71
inverse Park, 0.69 inverse Clarke, and 1.65 chain. Stable ratios are 4.26, 1.11,
1.09, 1.09 and 3.14 respectively. These are unpinned shared-host probes, not
universal speed claims. Raw samples, compiler/host metadata, executable digests
and section sizes are retained. Matching footprint probes grow by 5,752 bytes
on 1.88 and 9,928 bytes on stable; five runtime seeds yield identical checksums
across both implementations and compilers. Final footprint probes were rebuilt
after formatting the shared include file; timing binaries remain byte-identical.
Production/shared-support preservation verifies 172
byte-identical files, and the firmware checkout remains unchanged.

| Candidate | Decision | Evidence |
| --- | --- | --- |
| Maintained shared FOC conformance/cost client | kept | required Phase 6 consumer, independent exact expectations, failures and measured costs |
| Naive signed Park matrix | rejected | MIN*-1 intermediate failure changes |
| Pre-rounded Clarke coefficient matrix | rejected | exact point output widens by two ulps |
| Blanket production adoption of current shared transforms | rejected | Clarke/chain time and matched probe size regress on both measured compilers |

Source commit `3422eb3f10e0bd9fb9c0b152a71627ad4024df87`; evidence commit
`ed8d092582b4091490b83182b760cf4f0345e1a6`, record
`docs/evidence/2026-11-12-interval-foc-client`: 123 files,
735,316 bytes, SHA-256
`2182ecf72274980f523a0cf1aae81bfb1f7a9f2abc4ca3484b0fa37a5bbcceaf`. The committed record was regenerated and
verified before registration; all previous archive rows remain unchanged. Performance-specialized
fixed kernels are still needed before firmware migration; the conformance client
is evidence infrastructure, not an adopted optimization. Jacobian carriers,
independent authored center/radius, remaining specializations and later phases
stay open. No phase completion, full pre-push run or push is claimed.

## Phase 2/6 admission: fixed exact-integer division and Clarke specialization

Baseline `434b6e09a1b8edae9df7764254eb07442342dc0c`. Owner: Hyperinterval fixed
arithmetic, consumed by the unpublished FOC conformance client. P1, P5–P7,
P18–P20 and P31–P33. The missing-method witness fails on that revision and now
passes: raw i64/F63 `[-5,7] / 3` has tight grid bounds `[-2,3]`, although the
point integer three is not representable in that format.

`div_integer(i64)` covers both fixed endpoint widths, every valid fractional
scale and every signed i64 divisor. It retains the grid, normalizes negative
order in i128, and reuses `quotient_bounds` and `from_wide`. Zero is rejected
before empty propagation; nonzero division preserves empty. Signed extremes,
including i64::MIN divisors, remain exact; MIN/-1 is typed overflow where the
endpoint cannot fit. This constant-size operation allocates nothing. Existing
fixed method bodies and their ordering are byte-identical after removing the
new method and its reciprocal documentation link; 171 other previously audited
production/support files remain byte-identical. No decision policy, certainty
receipt, dispatch sequence, production dependency or firmware source changes.

The FOC client reuses the existing affine image for `2*a`, then subtracts b and
c before integer division by three. All integer linear intermediates fit i64
for Q30 i32 inputs; no intermediate rounding or new failure occurs. The balance
check remains first. Park and its original overflow/rounding witnesses remain
unchanged. This extends the fixed domain and specializes an evidence client;
it does not claim completion of firmware adoption or the general refactor gates.

Independent cross-product inequalities prove tightness for 1,673,100 small
interval/divisor cases across twelve scale instantiations, plus full endpoint/
divisor extremes and 20,000 seeded full-width cases (seed 20261113). Tests cover
point-denominator equivalence where representable, zero/empty priority, typed
overflow, subsequent division, storage widening and fresh-process checked replay.
A separate executable measures zero allocations for successful and failing
arithmetic. An initial hand-computed replay expectation was wrong by one lower
ulp; its failing log is retained. Exact inequalities corrected the expectation
and added an explicit first-stage bound assertion; no implementation or existing
fixture was weakened.

All 38 final matrix commands pass: Rust 1.88/stable debug and release tests,
Clippy with warnings denied, documentation with warnings denied, ARM/Wasm
library checks and Clippy, formatting, the fixed-only lattice suite (23 tests/
doctests), all 83 original FOC tests and dependency inspection. Scalar fixed+
serde suites each pass 33 tests/doctests. Additional fixed-only targeted tests,
the formerly failing witness, and ASan with leak detection pass. The unchanged
399,950-outcome FOC corpus and repeated operation chains pass on both compilers.
Focused scalar coverage exercises all 15 emitted div_integer instances and
27/28 distinct source regions; the remaining region is scale-validation error
propagation, unreachable through safe constructors of valid intervals. Client
coverage is 168/170 lines, 364/385 regions and 16/16 functions; this is scoped
coverage, not whole-crate completeness. No new fuzz, hosted-OS or HIL claim.

The rebuilt allocator-free Wasm module has zero imports and passes the same
independent Python integer oracle: 1,660 completed transform calls and 405
full-range Park checks (335 successes, 70 range errors). It is 40,006 bytes,
SHA-256 `28f5514ceb76f127a8dfedc8998a7b9d3798d368262d023a9bcdfbb12f0fc3e5`;
maximum observed fuel is 19,081 under a one-million-per-call cap. This measures
that artifact, not hardware cycles or a general resource bound.

Six alternating baseline/final rounds per compiler contain 4,800 samples total,
512 completed calls per sample, including materialized outputs and digest work.
All full-output prechecks, checksums and five footprint-probe seeds agree, with
zero measured allocations. Median shared/native ratios change from 3.048 to
1.588 for Clarke and 1.642 to 0.859 for the chain on Rust 1.88; stable changes
from 4.307 to 2.144 and 3.280 to 1.734. Final shared/baseline shared time is
0.441/0.504 for Clarke and 0.519/0.532 for the chain (1.88/stable). These short,
unpinned shared-host measurements do not establish universal speed or any
firmware cycle/stack, monomorphized-size or whole-workload acceptance gate.
Matched final native/shared executables are 444,776/446,168 bytes on 1.88 and
460,656/465,072 on stable: shared still grows by 1,392/4,416 bytes. Shared probe
size falls by 4,432/5,576 bytes relative to the baseline shared probes.

| Candidate | Decision | Evidence |
| --- | --- | --- |
| Exact-integer division, affine/subtract Clarke numerator, ordinary inline hint on the new method | kept | expanded divisor domain, tight exact bounds, unchanged FOC outcomes, lower measured completed cost and footprint |
| Additional inline hint on the private narrowing adapter | rejected | shared-probe and cost .text sections identical on both compilers |
| Production transform migration | still rejected | Clarke and stable chain slower than native; matched shared binaries remain larger |

Source commit `f451d7629f71fcf009524f44d4466f1f8232ccf4`; evidence commit
`1085b421abe31db731ec0c8decd22653342545a7`, record
`docs/evidence/2026-11-12-interval-fixed-integer-div`: 191 files,
4,734,853 bytes, SHA-256
`f4fc094365089543658042879f12f099f4be2978ee2351654997596c59aa69c8`. The committed archive was regenerated and verified before
registration; every previous archive row remains unchanged. Remaining Phase 6
carriers/specializations, firmware and solver adoption gates, hardware evidence,
and all later phases remain open. No full pre-push matrix or push was performed.

## Phase 8 admission: unit-bearing measurement construction

Baselines: CSGRS `efb7b2b9909e038987ec93c2b89211439b957a40`, Interface
`ee6cdb169ba1feb8d5f3e8bcda11256733c70755`. All managed repositories were clean.
The original Interface core baseline passes all 199 library tests. Owner:
Hyperinterval rational set arithmetic, with physical meaning retained by
`BoundedMeasurement<U>`. P1, P4–P10, P19 and P31–P33. Missing acceptance witness:
full old/shared measurement conformance with comparable cache states, allocation,
time and footprint, beyond the existing small integer fixtures.

The Interface constructor now delegates to the existing
`RationalInterval::from_nominal_uncertainty` and consumes its owned bounds.
It retains every finite Rational nominal and uncertainty input, physical unit,
private endpoint storage, const accessors, equality, Debug shape and successful
output. Negative uncertainty keeps the original error and message. The successful
scalar construction is necessarily nonempty; the adapter checks that invariant.
`BoundaryError::Interval` preserves any unexpected kernel diagnostic through
`Error::source`, including a nested scalar cause. This public enum addition is
recorded in Interface's `docs/INTERVAL-MIGRATION.md`; exhaustive callers need a
new arm. No new failure occurs for previously valid measurement inputs.

There is one existing scalar sign check, subtraction and addition in the same
order. Rational's owned operators delegate to the same borrowed kernels; removed
Arc clones do not replace source identities or arithmetic. No scalar/predicate
implementation is changed. An inline-hint experiment was rejected and removed.
All 116 other existing Interface files are byte-identical to the baseline,
including machine-profile, compiler, fixture and canonical-output code. Firmware
is unchanged. Machine-profile `from_scalar` needs separate proof transport:
adding the shared constructor there would introduce a sign check after earlier
configuration validation, so that path is not part of this candidate.

The maintained standalone `hyperinterval/conformance/alumina-measurements`
package checks 2,145 common-denominator cases (1,485 nonnegative-radius and 660
negative-radius cases), both unit types, exact endpoints, checked serialization
and subsequent addition. Eight representation fixtures include signed, dyadic,
word-sized, large rational and roughly 4,000-bit inputs. Both STRICT and
APPROXIMATE_512 downstream comparisons retain exact receipts. Separate fresh
processes compare complete cold and once-warmed construction traces and scalar
counters, then assert one exact-rational comparison dispatch per policy. A
separate executable compares cold/warm allocations and full outputs against the
original formula. Counts match in every fixture, including zero allocation on
rejection; cold successful work uses 0–7 allocations and once-warmed work 0–3.

The initial in-process trace test was invalid: recreating equal small rationals
can retain interned facts. Applying the exact original formula on both sides
reproduced its failure. That diagnostic and the initial failure are retained;
the fresh-process replacement compares the same cases and all counts without
relaxation. A selective release test also exposed Cargo's duplicate CSGRS
cdylib/rlib filenames when aborting probe binaries and unwinding tests shared a
build. Native probes now require an explicit `probes` feature, excluded from
release tests, and adoption artifacts use a separate target directory. Both
revisions were rebuilt with the final configuration. The failed logs remain.

All 15 final selected commands pass after those changes: five conformance tests
in debug and release, strict Clippy, documentation with warnings denied, Wasm
Clippy, Interface core tests/doctests (201), rational-owner tests/doctests (17),
formatting and dependency inspection. ASan with leak detection passes all five
conformance tests, including child processes. A preexisting stable-toolchain
`fetch_update` deprecation warning in alumina-runtime appears in baseline and
candidate Wasm builds; it is not introduced or suppressed here. No new fuzz,
full application coverage, hosted-platform, browser or full pre-push claim.

The independent Python Fraction Wasm oracle passes 4,369 calls: 3,015 successful
bounds, 1,330 negative-radius rejections and 24 invalid denominators. Endpoint
text hashes supplement full-value host tests and independent fraction bounds;
they are not the sole correctness witness. Seven existing binding imports are
present; every import traps if called and the measurement path makes zero host
calls. An unexpected-error status is explicitly rejected by the oracle. The
initial assumption of no imports and the earlier panic-based probe are retained
as failed/superseded harness evidence. Both revisions were rebuilt with the same
final probe; baseline bytes remain identical. The final raw module is 960,440
bytes versus 960,500 baseline. Actual artifact digests and fuel bounds are in the
record. This is a headless measurement consumer, not whole-UI browser coverage.

Matched native probes use Interface's release settings and Rust 1.95. The value
probe falls from 603,272 to 603,224 bytes; the cost probe from 606,760 to 606,632.
Post-LTO emitted LLVM falls from 730 definitions / 89,986 noncomment body lines
to 729 / 89,977. IR lines are a code-expansion measure, not machine bytes.
Eight full exact endpoint/error outputs agree between revisions. Six alternating
unpinned rounds contain 1,920 samples, 512 completed constructions/destructions
per sample after 32 warmups, with zero allocations in the timed scope. Successful
case median ratios are 0.651–0.737; rejection is 1.018, an adverse result retained.
Twenty further paired rounds on fixed allowed CPU 0 contain 6,400 samples:
success ratios are 0.665–0.753 and rejection is 1.000, with a bootstrap median
ratio interval of [0.99996, 1.00000]. Frequency/governor and other host activity
are not controlled. These are repeated-input local measurements, not a universal
speed claim; all raw data, commands and per-case summaries are retained.

The first generic-diagnostic Display candidate increased native footprint by
912 bytes; it was revised to a short outer message while retaining the typed
source chain. A six-byte raw Wasm increase came from the new probe's unexpected-
error panic text; changing that new probe to an explicitly rejected status and
rebuilding both revisions removed it. No production failure or assertion was
removed. The final delegation is retained with the measured native/Wasm and IR
footprint reductions, identical exact results, traces and allocations.

Interface source commit `ff668ca39d5e1ecee91b438b20eca806f3a26f29`.
CSGRS source commit `f2d4f22e9c527cc1890ad45608f2f05fcf04bc6c`; evidence commit
`fbb1bb0958d46d20ea1f53f992bcaa73241108b9`, record
`docs/evidence/2026-11-12-interval-interface-measurements`: 214 files,
1,296,345 bytes, SHA-256
`3dc3b63286523969c908fe175ffbffcba5fcea6cc606d7deadcd63866d010c53`. The committed archive was regenerated and verified before
registration; every previous archive row remains unchanged. This completes
one production constructor delegation, not Phase 8 or the full refactor.
Machine-profile intervals, positive products/quotients, named budgets, complete
canonical-profile differentials and full browser/schedule measurements remain
open. The earlier solver/firmware adoption blockers and all later phases remain.
No push was attempted.

## 2026-11-12: Phase 6 primitive layout experiment

Admission: CSGRS baseline `6c2ebd9647bd2a171881acb265084851ab3b9351`; all four
managed repositories were clean. The plan explicitly requires primitive
structure-of-arrays experiments. Hyperlattice owns Cartesian storage;
Hyperinterval retains all scalar arithmetic and validation authority
(P1, P5–P7, P18–P20, P31–P33). The missing witness was a measured, complete
operation comparison against separate lower/upper endpoint arrays. This
checkpoint adds an unpublished conformance package only; no production scalar,
Cartesian carrier, Interface, firmware, policy, or wire format changes.

The prototype retains one canonical Cartesian empty discriminator and private
endpoint arrays for i32/i64 fixed and qualified f32/f64. It imports validated
vectors and reconstructs through existing scalar constructors. Arithmetic calls
remain left-to-right; intersections evaluate later lanes after finding an empty
lane, and dot products retain multiply-then-add order. There is no unchecked
endpoint ingress, SIMD arithmetic, rational/dyadic layout change, or new replay
format. Reconstruction does perform extra scalar validation, so this is not an
accepted predicate-preserving production refactor.

Six conformance tests pass under Rust 1.88 debug/release, stable debug, and
nightly AddressSanitizer. Independent integer bounds cover 23,409 pairs of
small intervals and two-dimensional dot products with outward rounding. Tests
also cover 3,136 full-range fixed pairs across four width/scale combinations,
4,000 seeded finite IEEE bit-pattern pairs, subnormal rounding, overflow,
negative zero canonicalization, invalid families and replay, and dimensions
0, 1, 2, 3, 4, 16, 64, 256, 1024. Generated outputs feed further operations.
An injected late scalar failure verifies that empty intermediate lanes do not
hide failure. Existing owner tests/doctests pass (45). All 15 final selected
commands pass, including strict Clippy on both toolchains, docs, no_std ARM
compilation, Wasm Clippy/build, root formatting and diff checks. Initial type
inference and Clippy diagnostics remain in the evidence; no assertion was
removed. No full-coverage, new fuzz campaign, full pre-push or HIL claim.

The headless Wasm consumer passes an independent integer endpoint oracle for
612 completed pairs / 1,224 calls across both fixed families, nine dimensions,
17 seeds and both layouts. Four malformed fixture selectors trap. Both IEEE
ingress probes return `UnqualifiedEnvironment`, not fabricated success. The
module has no imports, is 297,168 bytes, and has SHA-256
`f405cd86543a80d62f86cc7e6da2510efd0f4f45bbc3bd0b64bf03bc7634bfa5`.
Maximum fuel for both endpoint calls is 9,134,806, with 100,000,000 allowed per
call. Fuel is a Wasmtime instruction metric, not wall time or MCU cycles.

Native complete-output comparisons pass for 17 fresh-process seeds and all
four families at dimensions 4 and 64. The final local cost probe records 6,912
samples (four families, nine dimensions, eight workloads, twelve alternating
samples per provider), 16 warmups, and zero allocations in every timed scope.
It runs on allowed CPU 0 without frequency or governor control. Conversion and
canonical output materialization are included for operations 0–6; operation 7
separately measures retained chains with the same final scalar output. At N=64,
SoA/native median time ratios for imported/retained chains are 0.781/0.743 for
i32, 0.696/0.660 for i64, 1.184/1.181 for f32 and 1.183/1.189 for f64. Individual
add/subtract/hull and many other paths regress. These short local probes do not
establish a general speedup or latency bound; timer-scale rows remain raw data.

At N=64 retained storage is 768→516 bytes for i32/f32 and 1,536→1,032 for
i64/f64; N=0 and N=1 do not improve. Temporary stack peaks are not measured.
The corrected single-provider native probe grows from 545,288 to 591,712 bytes;
GNU size text grows 424,966→469,166. Unique-address out-of-line named kernel
symbols sum to 72,971→113,342 bytes; this excludes inlined code and is not a
complete monomorphized-code accounting. The initial runtime-bool size harness
linked both providers and is retained as a rejected measurement. Its timing
run also overlapped root formatting; final measurements ran after checks ended.

Decision: **keep the experiment; reject a transparent production layout
replacement**. Storage savings and faster fixed chains do not overcome the
measured operation/IEEE regressions, larger executable, additional validation,
or inability to preserve borrowed `lanes()` without a migration or duplicate
storage. No production adoption or completed Phase 6 claim. Independent
center/radius, Jacobian carriers, facts, additional specialized kernels and
consumer adoption remain open. Machine-profile migration still needs proof
transport to avoid repeated positivity checks. All other phases and earlier
adoption blockers remain in scope. No push was attempted.

Source commit `94e23534708352d6de05ec7262fa722cb301646d`; evidence commit
`2bcfa39804823fa870f5e70c5887686d94164d6c`, record `docs/evidence/2026-11-12-interval-primitive-layouts`:
79 files, 1,461,024 bytes, SHA-256
`dc716354ecee5b1274cce31f6c75775ccf8773d9b07c2963bb89640282cbfed2`. The committed record was regenerated and verified;
every previous archive row remains unchanged.

## 2026-11-12: independently authored center/radius definitions

The preceding turn made progress by committing the primitive layout experiment.
Baseline `3f9679e5174f52f2e3457834fb6b7adb2652108d`; all managed repositories were
clean. Admission and commands are staged at
`/tmp/interval-refactor/authored-center-radius`. A detached checkout of this
exact revision reproduces E0432 for `CenteredVector`; the same external witness
passes against the implementation. Owner: Hyperlattice Cartesian definitions;
scalar signs, subtraction, addition and hull remain Hyperinterval authority.
Properties P1–P7, P18–P20, P25–P27, P35–P37; plan §§5.1–5.2 / Phase 6.

Implemented `CenteredVector<I,N>` and `CenteredMatrix<I,R,C>`, re-exported by
Hyperlattice, for all six `IntervalArithmetic` backends. Inputs are independently
authored exact point centers and nonnegative point radii in the same backend.
Construction validates point centers, point radii, then exact radius signs in
lane/row-major order, without endpoint arithmetic or scalar copies. Empty,
nonpoint and negative-radius components produce typed errors; an explicit empty
constructor preserves empty/unit distinctions. Borrowed components and owned
`into_parts()` retain the defining data. The existing `MidpointRadius*` bounds
decompositions, storage and wire format are unchanged. No scalar algorithm or
existing predicate body is changed; 56 existing source files across the scalar
and lattice layers are byte-identical, excluding the two export/error modules.

For each valid lane, nonnegative r proves c-r ≤ c+r. The new explicit enclosure
computes existing vector/matrix subtraction, addition and hull in that order.
Outward endpoint bounds therefore enclose the defined Cartesian set. The shared
context accounts across all operations, and only a completed carrier escapes.
An overflow or exhausted projection leaves the exact definition available for
retry or wider-family projection. The new serde format stores components and
revalidates their dimensions, scalar families, singleton status and radius
signs. No serialized validity/cache flag is accepted as proof. Existing generic
array visitors and allocation instrumentation are reused.

Independent integer expectations cover 25,704 vector/rectangular-matrix fixture
pairs across six backends, followed by affine images, intersections, products,
transpose and the existing midpoint/radius decomposition. Additional cases
cover 84 full signed-range/scale combinations, IEEE subnormal sums and outward
rounding around one, overflow with intact components, zero-sized units, negative
and nonpoint components, malformed replay and dimensions above 32. Fresh-process
replay preserves a definition whose fixed bounds overflow; promoting its retained
components to rational arithmetic recovers exact bounds and further operations.
The dyadic test rejects every work limit below the cost of a successful query,
retains the unchanged source, and succeeds at that cost and with a larger budget.

All 18 selected commands pass, including the owning package, Hyperlattice with
all features (248 tests/doctests), strict Clippy, docs, zero-feature and ARM
no_std compilation, Wasm build/Clippy and root formatting. After adding explicit
empty-matrix replay and diagnostic witnesses, seven final focused commands pass:
eight new tests on Rust 1.88, nine on Rust 1.95 debug/release and stable, Clippy,
ASan and instrumented coverage. The final complete owning suite passes 78
tests/doctests; final stable Clippy passes. The initial unused-import warning
was fixed without suppressions. LLVM reports 59/61 lines, 82/86 regions,
10/10 functions and 300/329 compiled instantiations for centered.rs; branch
instrumentation reports no denominator. This is not a 100% coverage claim.
The coverage export retains all functions mapped to this source file; digests
of the larger unfiltered exports are recorded separately.

The zero-import Wasm consumer passes 7,992 independent integer-oracle cases:
4,373 successful complete bounds, 2,455 negative-radius rejections and 1,164
projection/product overflows. It makes 18,955 returned calls plus two invalid-
selector traps, preserves zero-dimensional empty/unit semantics, and verifies
that IEEE ingress remains unqualified there. Maximum returned-call fuel is
4,688 of 1,000,000 allowed; fuel is not a timing or MCU-cycle metric. The module
is 47,864 bytes, SHA-256
`ca8965283a803506e186d5c9faa263abd7c75ee87c17f9895afff9e224c55de6`.

The maintained native probe records 960 samples with 32 warmups and 128 completed
operations/sample on allowed CPU 0, after downstream testing ended. It covers
six backends, dimensions 4/64, ownership retention, vector projection, the same
three-operation arithmetic control and rectangular matrix projection. Both
projection paths are checked against full independent expected outputs before
timing. Primitive operations allocate nothing. Retaining/reconstructing the
prepared exact components also allocates nothing for all six measured fixtures;
this is not an arbitrary-rational resource bound. At N=64, rational projection
uses 240 allocations / 9,600 bytes and dyadic 792 / 6,336, matching the explicit
arithmetic control. The form has the same inline size as its component pair
on this host; temporary stack peaks and arbitrary-precision retained heaps are
not measured. The all-backend cost executable is 1,553,688 bytes (GNU text
1,040,289). These short, sequential local probes have no governor control and
make no speedup, global no-regression or whole-stack monomorphization claim.

This is an additive retained-definition capability, not a production consumer
migration or a completed phase. General heterogeneous center/radius arithmetic,
authored-form exact affine-source integration, Jacobian carriers, fact/kernel
specializations, broader coverage and full consumer/performance qualification
remain open. Existing solver, Interface and firmware adoption blockers and the
full 15-phase goal remain unchanged. No push was attempted.

Source commit `87c0b59c4907fe9ac2403c6655b13cde45583647`; evidence commit
`449b1f1e020cfc4b69305d1fe4f48eb2784c7a42`, record `docs/evidence/2026-11-12-interval-authored-center-radius`:
77 files, 2,015,148 bytes, SHA-256
`24b6af26d27693cd5555d2f829f8ca59f2e6ef54f5456e04898cd13a62476cd8`. The committed record was regenerated and verified;
every previous archive row remains unchanged.

## 2026-11-12: exact affine sources for authored centers and radii

The preceding turn made progress through authored definitions and qualification.
Baseline `b02248ec61add81ea21414ac183a7b804283e4af`, all managed trees clean.
A fresh checkout of that revision reproduces the missing `affine_source` method;
the same external Rust 1.88 witness passes against this implementation.
Admission, raw commands and outcomes are staged at
`/tmp/interval-refactor/centered-affine`. Owner: Hyperlattice's affine source and
parameter protocol (P1–P7, P18–P20, P25–P27, P35–P37; plan Phase 6).

`CenteredVector::affine_source(context)` now converts the borrowed exact
definition into `TranslatedBox<I,N>`, retaining a point center and signed offset
parameter box `[-radius,radius]`. It never computes overflowing absolute bounds
and requires no scalar one or identity matrix. Static `AffineImage` and bounded
`AffineChain` consume this source through the existing protocol. Restriction
intersects offsets, not absolute coordinates, and retains the exact center even
when the selected domain becomes empty. Replay validates native dimensions,
families and singleton centers; a missing center is valid only for an empty
domain. Source and existing map definitions remain authoritative.

The exact identity is B(c+u)+t = Bu+(Bc+t). Pullback composes existing maps first,
then evaluates the shifted center and offset domain with the same scalar
context. Exact rational/dyadic success gives the coordinate hull; primitive
rounding may widen or fail typed, leaving the retained source reusable. No
existing scalar algorithm, affine/chain body or predicate order was changed.
The sealed arithmetic adapter gains only direct forwarding to existing scalar
negation kernels. Initial construction using `0-radius` failed the maintained
i32::MIN-exponent dyadic witness with SizeExhausted, because the old subtraction
path aligns canonical zero across the exponent gap. Exact native negation
removes that avoidable alignment in this bridge. The original failure and a
reproduction patch remain; the witness and its budget were not weakened.
General scalar zero-alignment behavior is not claimed repaired.

The five new tests cover 918 six-backend center/radius fixtures, exact source
bounds, offset restrictions, correlated duplication/difference, three-map
runtime chains, empty/unit and rectangular images, malformed/fresh-process
replay and further operation. F31/F63 tests retain overflowing [0,1] sources
and successfully apply a half-scale map although scalar one is unrepresentable.
IEEE and fresh-process fixed cases recover exact zero through a duplicated
source whose original/intermediate bounds overflow. Every insufficient dyadic
work limit below a completed bridge and a completed affine query is rejected;
the unchanged source succeeds at the measured cost and with a larger budget.
Primitive conversion/projection remains allocation-free in the shared counter.

All 15 selected commands pass: Rust 1.88 owning suite (70 tests/doctests),
Rust 1.95 debug/release and stable suites (83 each), strict Clippy, docs, ARM
no_std compilation, Hyperlattice interval surface (7), downstream library Clippy,
Wasm build/Clippy, root formatting and diff checks. The final expanded affine
budget test and strict Clippy pass; ASan passes all five new tests plus the
allocation test. The full unrelated elliptic suite was not repeated: its source
and arithmetic remain unchanged, and the prior checkpoint retains that result.
No new full-coverage/fuzz or pre-push claim; no CI files changed.

The maintained zero-import Wasm consumer now also exercises F31 half-scale
sources with exact integer expected endpoints. Across old and new cases it
passes 8,027 cases: 4,401 successes, 2,462 invalid radii and 1,164 overflow
outcomes; 19,046 returned calls plus two invalid-selector traps. The module is
48,514 bytes, SHA-256
`b91b28749e6ba7dfb01d106e17c03f2fcde0e70f6547f91dc199f1f98fa05db0`.
Maximum returned-call fuel is 4,684 under a 1,000,000 allowance. Fuel is not a
hardware cycle or timing measurement; IEEE ingress remains unqualified on Wasm.

The extended native probe records 1,440 samples after 32 warmups, 128 completed
operations/sample, six backends and dimensions 4/64 on allowed CPU 0. Separate
bridge and retained-source projection actions supplement the existing controls;
all primitive actions allocate zero. The translated source has the same inline
size as the authored component pair for these host types. At N=64, prepared
rational bridge/projection use 0/60 allocations (0/2,400 bytes), while dyadic
uses 462/210 (3,696/1,680 bytes). These are local warmed fixtures, not general
rational resource bounds. The expanded all-backend executable is 1,619,448
bytes; absolute size, raw samples and adverse costs are retained. No speedup,
whole-stack no-regression, peak-stack or monomorphization claim is made.

This completes the vector affine-source bridge, not Phase 6 or the full goal.
Heterogeneous center/radius arithmetic, matrix-form operations, Jacobian
carriers, measured facts/specializations, broader coverage and consumer adoption
remain open, together with all later phases and prior migration blockers.
No Interface/firmware source changed and no push was attempted.

Source commit `728ea5d7d63f9d98e9cf44fbeb55202d8fffa423`; evidence commit
`8d715b8c21bacef53b9528fb2b379ed96a3a5fbe`, record `docs/evidence/2026-11-12-interval-centered-affine`:
50 files, 1,286,436 bytes, SHA-256
`18cbe6601bff2737896a5f74b7946f0470492f14d8d22085288a44005e03c3fd`. The committed record was regenerated and verified;
every previous archive row remains unchanged.

## 2026-11-12: exact zero uses a compact dyadic grid

The preceding turn completed the authored affine-source bridge. Baseline
`6bd212482843261c9d514fdff3fe9b5b5bd2deac`, all four managed trees clean.
The new scalar acceptance witness reproduces SizeExhausted when adding zero to
a compact value at exponent i32::MIN. Owner: Hyperinterval's private dyadic
alignment (P1–P7, P18–P20, P35–P37; plan Phases 3 and 6). The admission and
raw results are staged at `/tmp/interval-refactor/dyadic-zero-grid`.

Kept as a resource-completion fix: common-grid operations use the nonzero
operand's exponent when the other operand is exactly zero. Alignment reserves
two work units before copying zero without a shift; its magnitude needs zero
bits on any grid. All five shared-grid sites use one private helper. Merely
zero-containing intervals keep ordinary alignment and exhaustion. Empty paths,
left/right reservation order, Bounds predicates, division preconditions and
normalization remain unchanged. There are no Hyperlimit policy or dispatch
calls in this path. Nonzero budget formulas are unchanged; the deliberately
smaller mixed-zero charges reflect work that is no longer performed. This is
not a behavior-preserving refactor or a new resource/cancellation mechanism.

Five scalar tests cover 1,320 small signed intervals across eleven exponents,
all nine affected binary operations in both orders, zero regridding, true
nonzero extreme-gap exhaustion, empty identities, 1/7/64/257-bit mantissas,
every insufficient work budget and fresh retry, independent integer arithmetic
expectations for 2,025 nonzero pairs, and fresh-process replay followed by
further operations. Downstream authored and translated zero-centered sources,
identity maps and accumulated sums complete at both exponent extremes.
The initial test compilation error (missing BigInt type annotation) is retained
alongside its correction; no failing arithmetic witness or budget was weakened.

All twelve selected checks pass: Rust 1.88/stable dyadic suites (71 tests and
doctests each), Rust 1.95 all-feature scalar debug/release suites (188 each),
the all-feature interval-lattice suite (84), Hyperlattice's interval surface
(7), strict all-target Clippy, docs, ARM and Wasm no_std compilation, root fmt
and diff checks. ASan passes five scalar and two dyadic affine tests. The
compile checks are not Wasm execution or hardware qualification. No new full
coverage, fuzz, formal proof or complete pre-push matrix claim is made.

Matched local probes preserve all 60 serialized sum/difference/hull/intersection
outputs and nonzero work charges. The primary runs use identical registry
versions/checksums from the repository lock; the initial alternate-dependency
runs are retained separately and do not supply the primary claims. Each case
has 64 warmups and 30 samples of 4,096 completed/materialized/dropped outputs,
pinned to CPU 0 without frequency control. Sixteen additional extreme-zero
cases complete only on the candidate; an old exhausted attempt is not counted
as a successful baseline. Ordinary nonzero median timing ratios range from
0.855 to 1.009; this short sequential probe is not a global speedup or
no-regression claim. For [-3,7]*2^e plus zero, e=-17/+17, work falls from
48/84 to 16 units; measured allocations remain two (16 bytes). Hull uses four
allocations (32 bytes), intersection two (16 bytes). The probe executable grows
from 524,952 to 526,112 bytes (+1,160); this lower-priority cost is retained for
new exact completion. Peak temporary memory, stack and monomorphized size are
unmeasured. Input construction is outside the timed/accounted scope.

This repairs scalar zero alignment and its tested downstream compositions.
All broader Phase 3/6 qualification and adoption gaps, the earlier solver and
firmware migration blockers, and the remaining 15-phase goal stay open.
No Interface or firmware source changed; no push was attempted.

Source commit `e58498cfbdb851cdf1c69c5839c1d3d9baf0b242`; evidence commit
`aa61d762ad69e91809edee6e085d7127cea7f648`, record `docs/evidence/2026-11-12-interval-dyadic-zero-grid`:
56 files, 575,198 bytes, SHA-256
`43fdf5e35cb750fa2b561ceded58000d896982c3ad21623a6c5a10e11a6c5cfc`. The committed record was regenerated and verified;
every previous archive row remains unchanged.

## 2026-11-12: Hyperdynamics axis extraction candidate, retention gate open

The preceding turn progressed by fixing dyadic zero alignment. Baseline CSGRS
`0d17fe178839b002b95173adf1f05137291639f3`, Interface
`ff668ca39d5e1ecee91b438b20eca806f3a26f29`, all managed trees initially clean.
Plan Phase 11's affine axis projection is now implemented as a pending
Hyperdynamics candidate. It depends only on lower scalar/predicate/solver
crates. Hyperpath keeps type aliases and deprecated one-release function
wrappers; Interface calls the new owner while preserving its Route error
boundary. The production edits remain uncommitted and unaccepted. This entry
records qualification and an adverse gate, not completion or permission to
ignore that gate.

The moved body is identical after its error/import rename and the repository's
formatter. A frozen test-only original establishes equality of every full
report and dispatch snapshot for 56 fixtures, both policies and fresh processes
through direct and legacy entry points. Independent exact minima, first-tie
selection, invalid input and unresolved symbolic cases pass. Fixed first-case
trace counts are 9 rational comparisons, 40 structural sign queries, 1,220
total events and zero GCDs. The full owner suites pass (444 baseline; 448 with
new tests), Interface passes 201 tests/doctests, strict Clippy/docs/Wasm compile
and formatting checks pass, and ASan passes the four focused tests. A separately
checked-out Interface/CSGRS baseline supplies the maintained golden planner
transcript: 41,795 bytes, SHA-256
`df893df83e90e6b53452e18586ffcaa2524a021631a395d4a6523be16263669c`.
The new caller passes that golden plus the existing exact lowering assertions.

Both successive_operations runs complete 200 certified manifold rows with
matching structural and allocation columns. The audited complete dependency
graphs are identical and contain neither Hyperpath nor Hyperdynamics. Concurrent
build activity prevents treating those elapsed times as a controlled comparison.
The completed projection probe preserves checksums and allocations; its twelve
local ratios span 0.949–1.015 with substantial drift. Its executable shrinks
3,780,856 -> 3,536,808 bytes, and the affected 17 generic formatting/drop symbols
remain 2,848 bytes. These favorable narrow observations do not clear the owner
gate.

All 194 owning Criterion cases complete, but several unchanged Specctra parsers
regress. Targeted reruns retain roughly 198–200 microseconds for 256-record
parsing versus 171–172 microseconds before extraction. The owning benchmark
binary grows 13,308,824 -> 13,319,976 bytes (+11,152; text +6,896). A candidate
build in the original absolute directory is byte-identical, ruling out that
path difference. Restoring the old implementation while retaining the new
dependency restores original timing. Thus the candidate has an unresolved
movement/code-generation/layout effect; it cannot be retained as an accepted
behavior-preserving refactor. No unrelated parser optimization, relaxed test or
favorable-only performance claim was used to bypass this finding.

The pending source and Interface patches, raw samples, locks, source-equivalence
check, baseline canonical witness, failed initial checks and controlled build
patches are archived below. The isolated `dynamics-axis-baseline` checkout now
contains the dependency-only control; it is not a clean baseline. Primary
worktrees retain the extraction candidate for continued work. Next work must
resolve the owner cost gate while completing the planned domain-neutral
feed/lookahead/jerk boundary, preserving lazy metric/join calls and their exact
receipts. All prior gaps and the full 15-phase goal remain active. No push ran.

Unaccepted candidate source is bound by CANDIDATE.json and both retained patches.
Evidence commit `865da1ad11b25edc855307a42fab454f504a84ad`, record
`docs/evidence/2026-11-12-interval-dynamics-axis-candidate`: 1793 files, 5,920,196 bytes,
SHA-256 `e628a7082d95447935348bc96e8e3a439623c17edd3c801a2768e259155d89c4`. The committed record was
regenerated and verified; all previous archive rows remain unchanged.

## 2026-11-12: Exact metric-span and jerk extraction candidate, cost gates still fail

The preceding turn implemented the axis candidate and reproduced its retention
failure. This turn extends that pending work from ledger HEAD
`26be8eeb7eb214cbf62440269a3eb11b4ad9c85a`; the earlier adverse archive remains
immutable. Phase 11 now has a geometry-independent `MetricSpan` protocol,
constant-jerk replay and two-phase monotonic planning in Hyperdynamics.
Hyperpath supplies exact lengths from retained line, explicit arc, cubic PH and
quintic PH sources. Its concrete entry points remain deprecated wrappers and
report aliases; Interface imports the new operations and preserves Route error
boundaries. All 26 legacy error variants map explicitly in both directions.
The production changes remain pending and unaccepted.

The original jerk body is identical after the documented import/error rename,
generic input and lazy length-method substitutions, and formatting. Length
queries retain their original order and repeated replay requests. Shared
positive/nonnegative checks preserve decision certainty, stage and refinement
requirements; legacy internal wrappers retain their old flattened errors. The
axis positive-limit helper reuses this authority without extra classifications.
The metric trait is an exact-input contract, not certification of arbitrary
geometry. No cumulative resource, ODE or universal closure claim is added.

A frozen original module and original guards match complete reports and full
dispatch snapshots for 43 fixtures, both policies and fresh original/direct/
compatibility processes. The first fixture fixes 35 structural sign queries,
5,287 total events and zero GCDs. The earlier 56-fixture axis differential and
its fixed counts also pass. Four new lower-layer tests independently exercise
integrated phase identities, violated continuity/caps, lazy query precedence,
failed-source retry and check metadata. Existing hard cases and assertions
remain. Final owner tests pass 453; Interface passes 201 tests/doctests and the
unchanged 41,795-byte planner transcript SHA-256
`df893df83e90e6b53452e18586ffcaa2524a021631a395d4a6523be16263669c`.
All ten selected final checks pass: strict owner/Interface Clippy, docs, Wasm
compile, owner bench smoke, successive operations, Interface tests, formatting
and diff validation. ASan was not rerun in this step; its earlier scope is not
extended by implication. No pre-push matrix or push ran.

Successive operations again completes 200 certified manifold rows with the
same structural and allocation columns as the original baseline. Its unchanged
complete dependency graph contains neither Hyperpath nor Hyperdynamics; elapsed
times are not used as a controlled comparison. Both owner Criterion runs finish
all 194 cases after builds stop, pinned to CPU 0. Broad ratios have median
0.9989 and range 0.8632–1.3062. Alternating targeted reruns retain 256-record
parser means of 171.817/171.347 us at baseline versus 182.085/186.832 us for the
candidate. Several other parser regressions persist. Broad jerk gains largely
disappear in the repeats; they cannot support a no-regression claim.

The owning executable grows 13,308,824 -> 13,379,488 bytes (+70,664), with
text +52,256. Full symbols show additional scalar/solver instantiations.
The narrow report/geometry Debug/Clone/drop subset shrinks 6,788 -> 6,705 bytes,
but that excludes broader growth and does not establish the monomorphized-code
gate. The baseline is its preserved, hash-verified original executable; the
isolated checkout remains the earlier dependency-only control and was not
rebuilt for these measurements. Binary and timing gates therefore still fail.
No test, policy or fixture was weakened to accept the extraction.

Next work must investigate cross-crate instantiation/code-generation boundaries
with isolated cost evidence while completing exact metric/join ownership.
Feed/lookahead extraction, join evidence, general functions/reachability and
all earlier plan obligations remain open. Current work is progress on a pending
candidate, not an accepted refactor or completed phase. CANDIDATE.json binds both
complete pending patches to their base revisions; initial failures, original
references, all final transcripts, raw samples and adverse measurements are kept.

Evidence commit `ab370e01cac14702b7e6c4a514361a6a36da071a`, record
`docs/evidence/2026-11-12-interval-dynamics-jerk-candidate`: 1792 files,
13,769,446 bytes, SHA-256
`316eb2b0cbc29314d3f61bf8619e3424082e6d543e3d40c1cea7b10a1a5afd4f`.
The committed record was regenerated and verified, preserving all earlier
manifest rows. Only this ledger and the archive manifest are committed here;
production code remains staged for continued qualification.

## 2026-11-12: Two dynamics code-generation experiments rejected

Previous turn: progress, with exact metric/jerk extraction and reproduced cost
failures. The primary pending candidate is unchanged from ledger HEAD
`99b810a567ad89162b5ffa23e074948311fadb3c`. A detached checkout reproduces its
owner executable byte-for-byte (13,379,488 bytes, SHA-256
`d331496d666548da7e9c90d790d2bc1ff683cd75a8c8c20460a76536c43f3c36`), providing
an isolated control without changing compiler/profile/dependencies or fixtures.

Two bounded experiments fail the existing binary gate:

- Ordinary inline availability for moved jerk helpers/shared guards produces
  13,383,312 bytes (+3,824 versus the already failing control). Rejected as a
  standalone cost change; no timing or consumer qualification is claimed.
- A private borrowed route adapter keeps lazy metric access generic while
  compiling scheduling bodies once. All 453 owner tests pass, including full
  original/direct/compatibility reports and dispatch snapshots under both
  policies. The binary grows to 13,392,472 bytes (+12,984 versus control), so
  this standalone change is also rejected. No downstream, timing, allocation,
  sanitizer or complete CI qualification is claimed for it.

Selected dynamics symbols shrink in both variants while the executable grows;
that selected sum cannot establish the binary or monomorphized-code gate.
Static caller tracing also refines the previous attribution: a new 10,107-byte
Real signed-product instance reaches unchanged geometry routines, but a larger
Rational instance disappears. The three changed signed-product families shrink
124,739 -> 123,102 named text bytes in aggregate. The new symbol alone is not
10,107 bytes of net growth. Complete causal attribution of the owner regression
remains unresolved; no extra arithmetic work is inferred from symbol counts.

Neither experiment is applied to the primary repositories. The isolated tree
is restored to its staged control and rebuilt to the same binary digest.
The primary staged patch still matches the preceding immutable archive.
Source review identifies the next coherent work: finish feed/join ownership,
preserving axis-only legacy rejection, lazy metric/classifier order, complete
geometry-owned tangent receipts, old aliases and application encoding, then
qualify the full boundary. The rejected narrow adapter is not accepted merely
because a broader design may need route access. All earlier adverse records,
remaining phase obligations and the full fifteen-phase goal remain active.
No push ran.

Evidence commit `3132007a9ca569ccc591a9e3a4f8775f3c95d487`, record
`docs/evidence/2026-11-12-interval-dynamics-boundary-cost`: 30 files,
6,132,181 bytes, SHA-256
`e5075ba88a285ac2e37b9de888fd5e16d2f9a6626f6e6d759444e5b17fb2db09`.
The record includes exact control/experiment patches, commands, build/test logs,
symbols, static call-chain reproduction and reviewed next-boundary constraints.
Its committed digest was regenerated and verified; earlier manifest rows are
unchanged. Only the ledger and archive manifest are committed here.

## 2026-11-12: Feed, join and lookahead boundary implemented as an unaccepted candidate

Previous turn: progress, rejecting two isolated cost experiments and reviewing
the remaining ownership boundary. This step extends the pending source at ledger
HEAD `fc859710c3ba1468e0ec3b0f440a960feedd3fe9`. Hyperdynamics now owns the feed
profile equations, corner constraints, forward/reverse lookahead and bounded
component jerk refinement, alongside the earlier axis/metric/jerk candidate.
`MetricPath` provides lazy aggregate length and `JoinSpan` transports the complete
source-owned join receipt. Hyperpath retains metric/tangent authority, concrete
receipt aliases and deprecated wrappers. A borrowed axis-only adapter preserves
legacy rejection of diagonal lines without copying geometry. The rejected
private type-erased route experiment is not adopted. Interface uses lower
operations directly while keeping its original error boundary and encoding.

The missing native-provider witness is reproduced before implementation. Four
independent lower tests exercise exact feed identities, reachability, one-step
halving, opaque receipt transport, query/error order and small-budget retry.
Forty-six fixtures compare complete original/native/compatibility reports and
dispatch snapshots in six fresh processes under both policies. The first fixture
retains 161 structural sign queries, 16,930 events and zero GCDs. Three original
mixed metric/tangent pairings remain diagnostics outside the same-path
precondition; three added connected arc/cubic-PH/quintic-PH plus line fixtures
cover source-consistent mixed inputs. All previous hard cases and assertions
remain. Eleven scalar helper bodies also match the original after explicitly
recorded error-name, visibility, comment and formatter transformations.

All 459 owner tests and 201 Interface tests/doctests pass. The 41,795-byte
application golden retains SHA-256
`df893df83e90e6b53452e18586ffcaa2524a021631a395d4a6523be16263669c`.
Owner and Interface strict Clippy, owner docs, stable Wasm compilation, the
standalone exact-provider example, both formatting checks and source budget pass.
ASan passes four lower tests and both feed/join migration tests, including fresh
processes. Owner benchmark smoke completes all 194 cases; successive_operations
completes 200 certified manifold rows with identical structural/allocation
columns and an unchanged graph containing neither affected crate. Those smoke
elapsed times are not a controlled performance comparison. No full pre-push CI,
hosted platform, firmware or hardware qualification is claimed.

The cost gate still fails. The final owning executable is 13,451,992 bytes,
+143,168 versus the preserved original baseline; text grows by 112,544 bytes.
All 194 timed cases complete, broad mean-ratio median 0.9993, range
0.9084–1.0689. Four targeted candidate/baseline/baseline/candidate runs retain
parser regressions: the 256-record case averages 1.0592, with baseline
171.295/172.734 us and candidate 181.896/182.511 us. Other parser means are
1.0472–1.0763. Lookahead varies materially between repeats; its favorable broad
observation does not establish a performance improvement. The selected 38
report/geometry Debug/Clone/drop symbols grow by 127 bytes; that selection is
not global monomorphized qualification. Owner allocation and complete
instantiation qualification remain open.

Final review changed two API descriptions. Five follow-up owner checks and
Interface tests/Clippy pass. The full binary-digest assertion then detected
24 bytes of symbol metadata growth; this failure and both hashes are retained.
After stripping symbols and removing only the build-ID section, the measured
and final candidate ELF files are byte-identical. The complete runtime
comparison supports reusing the measured timings; the final raw file size is
reported separately. Final fresh-process transcripts were recaptured afterward.
No source/algorithm change is hidden by this normalization.

The functional boundary is present but remains unaccepted; Phase 11 and the
fifteen-phase goal remain open. Next investigate whole-boundary code generation
with isolated controls and the full correctness corpus. Do not change unrelated
parser algorithms, compiler policies, fixtures or thresholds to mask the cost
failures. General enclosed functions and validated ODE/reachability remain open,
as do all earlier phase obligations. No push ran.

Evidence commit `64cdf13999d157c1ba57a96d6a04737d80588218`, record
`docs/evidence/2026-11-12-interval-dynamics-feed-join-candidate`: 1810 files,
26,585,712 bytes, SHA-256
`1d08bf6f2743392ba1e34ebdbbd52ab3f8996209f75dde7c9c766637b197fe62`.
The committed digest was regenerated and verified, preserving all earlier
manifest rows. It retains complete pending CSGRS/Interface patches, frozen
references, raw transcripts/samples, exact commands, initial failures and adverse
measurements. Only this ledger and the archive manifest are committed here;
production source remains pending for qualification.

## 2026-11-12: Whole-boundary inlining clears measured size gates, timing remains open

Previous turn: progress, with the full feed/join/lookahead candidate and reproduced
cost failures. This step starts at ledger HEAD
`898bd9bce0f4640f38982823a6f2635a87e87dfe`. An isolated checkout reproduces the
pending owner executable byte-for-byte before experimentation. No compiler,
profile, dependency policy, mathematical body, predicate order, fixture or
arithmetic core changes in this step.

Four attribute-only variants establish a concrete improvement over the failing
13,451,992-byte candidate. Ordinary inline availability for all 71 top-level
dynamics functions yields 13,315,312 bytes; forced inlining yields 13,337,400;
adding 13 geometry/metric/error adapter methods yields 13,315,464. Those variants
still exceed the original 13,308,824-byte baseline. Ordinary function availability
plus all 15 report methods yields 13,300,712 bytes: -151,280 versus the pending
control and -8,112 versus the original. Only this last variant is selected for
continued qualification and applied to the primary pending source. Removing
exactly its 86 ordinary inline attribute lines recovers all five prior files
byte-for-byte. The other variants remain archived experiments.

The primary and isolated selected binaries match, SHA-256
`98914d6b2b2f59989f86324d28b0c43cb2e1bfe3a2c43a6f59b0b08b27c305b7`.
GNU size's text category shrinks by 7,536 bytes versus original; the actual ELF
.text section shrinks by 7,824. A complete emitted FUNC-range inventory, merging
aliases/overlaps, shrinks 8,042,989 -> 8,035,160 bytes (-7,829), with 9,401 ->
9,392 sized ranges. Six unsized function symbols are retained separately and
whole-section size also shrinks. This addresses the earlier incomplete selected-
family size evidence for this compiled workload; it is not a universal bound
for arbitrary future providers/consumers. Data and BSS also shrink. The expanded
control's extra scalar/operator/solver instances mostly disappear, supporting
the boundary hypothesis without establishing a unique runtime cause.

All eleven final qualification commands pass: owner tests, strict owner Clippy,
docs, stable Wasm compile, owner benchmark smoke, Interface tests/strict Clippy,
successive_operations, both formatting checks and diff validation. Owner tests
pass 459 cases and Interface passes 201 tests/doctests. All existing frozen
axis/jerk/feed/join comparisons and count assertions remain intact. Six fresh
feed/join transcripts match complete reports and traces under both policies;
the first fixture retains 161 signs, 16,930 events and zero GCDs. The application
golden remains 41,795 bytes with digest
`df893df83e90e6b53452e18586ffcaa2524a021631a395d4a6523be16263669c`.
All 200 successive_operations rows retain their structural/allocation columns;
its unchanged graph contains neither affected crate. Source budget passes.
ASan was not rerun for attribute-only edits; the preceding functional record
retains its sanitizer scope. Full CI/platform/hardware and owner-wide heap/peak
memory qualification are not claimed. No push ran.

Timing still fails retention. All 194 owner cases complete against the preserved,
hash-verified original binary: broad mean-ratio median 1.0040, range
0.8876–1.1233. Four longer candidate/baseline/baseline/candidate runs for the
earlier ten cases give a 256-record parser mean ratio 1.0318 (baseline
176.676/172.579 us, candidate 180.377/179.985 us). Scheduler repeats show visible
drift; neither a favorable final repeat nor a favorable median proves equality.
Fifteen additional cases selected mechanically from the broad sweep also receive
four longer repeats. Adverse means include skew certification 1.0951, exact
trace import 1.0889, rectangular-pad clearance 1.0623, mixed symmetric jerk
1.0365, symmetric jerk 1.0356 and corner lookahead 1.0337. The supplemental
screening rule is not an acceptance tolerance. All raw samples, including
reversals of apparent broad regressions, are retained.

The skew, trace-import and multi-record-parser entry functions have unchanged
static byte/instruction counts. Their differing data/indirect-call or displayed
alias targets do not prove equal transitive code or explain timing. Further
work should resolve actual call targets or profile a fixed workload before
changing any algorithm; do not mask failures with unrelated parser/compiler
changes. The complete extraction is still unaccepted, production remains
pending, and Phase 11 plus the full fifteen-phase goal remain open. Owned
replayable fields and validated evolution remain unimplemented; all other
phase obligations and earlier adverse evidence are preserved.

Evidence commit `a014aaebaf1c0339ae015a9cb7c1d8017df42e33`, record
`docs/evidence/2026-11-12-interval-dynamics-whole-cost`: 2067 files,
59,179,330 bytes, SHA-256
`fe3ca693fe615514fd116a11fb38b7342e1a4809db00aa62f1e81c1bffac8acb`.
The committed digest was regenerated and verified, preserving earlier manifest
rows. Exact complete source patches, variant transforms, compiler/CPU/lock data,
full inventories, static listings, fresh transcripts, commands and raw timing
samples are retained. Only this ledger and the archive manifest are committed
in the primary repository; source changes remain staged and unaccepted.

## 2026-11-12: Repair symbolic derivative prerequisites for owned fields

Previous turn: progress, clearing the measured dynamics size regression while
retaining failed timing evidence. Review of plan section 10.5 on ledger HEAD
`e0eb94168cfbe8257f70827eb4775248ada9baa7` finds two concrete Hypersolve defects
before an owned field can safely reuse the symbolic layer. Their independent
correctness repair is committed as
`d499614eafd096f488ffd4e7b19c4058dbaa9329`, affecting only symbolic.rs,
jacobian.rs, the changelog and one regression-test file. The staged dynamics
and Interface patches are unchanged and remain unaccepted.

The integer power rule subtracted one from every nonzero i64 exponent. At
`i64::MIN`, debug builds panic and release builds wrap the exponent, producing
wrong repeated derivatives. Both failures are reproduced on the target source.
Only that boundary now uses `n * (x^n / x) * x'`: every stored exponent remains
representable, and the original negative power already excludes zero. Removing
the new branch recovers the entire old symbolic.rs byte-for-byte. Other branches
and predicate/evaluation sequences are unchanged. Repeated differentiation and
composed bases remain consumable without discarding the nonzero-domain guard.

The dense symbolic-Jacobian adapter also returned a guessed zero for a failed
binary64 conversion. A finite exact slope `2^2048` at `x=0` reproduced
`Ok([[0.0]])`. Plain and prepared entry points now propagate the existing typed
`DenseSolverEstimateUnavailable` after the same exact weighted-entry calculation.
These are still dense proposal APIs; the repair does not turn them into rigorous
interval Jacobians or certify other finite-difference/solver conversion paths.

Four new tests use independent i128 homogeneous-degree accounting and exact
unit-value expectations for ten boundary/ordinary exponents through three
derivatives. They cover a composed base at both unit signs, its excluded zero,
unbound input and an unrelated differentiation variable. Both symbolic-Jacobian
entry points consume the boundary derivative and reject the unavailable estimate.
No enormous rational power is materialized as a proxy for the symbolic proof,
and no old assertion, policy or hard fixture is weakened.

All eleven final local/downstream commands pass: 842 tests across the affected
Hypersolve library/smoke/regression suites; four release witnesses; strict owner
Clippy; 459 dynamics/path tests including frozen full-report/trace comparisons;
201 Interface tests/doctests; strict Interface Clippy; docs; stable Wasm compile;
successive_operations; formatting and diff validation. The application golden
remains 41,795 bytes with SHA-256
`df893df83e90e6b53452e18586ffcaa2524a021631a395d4a6523be16263669c`.
All 200 successive-operation rows preserve structural/allocation columns.
Hypersolve and Hyperpath benchmark smoke runs complete 417 and 194 cases.
Source budget passes. Initial debug/release failures, the false-zero witness and
the corrected test-only Clippy style finding remain recorded. This is affected
qualification, not the full pre-push matrix or every integration suite; no
sanitizer, hardware/platform or controlled performance claim is made. No push ran.

Section 10.5's owned/replayable field, exact interval image/Jacobian, full resource
accounting and Phase 12 validated evolution remain unimplemented. The next
interface must retain original source-domain obligations alongside derivatives,
explicit bindings and replayable identity, and must not reuse a dense point
proposal as interval evidence. Other existing proposal conversions and recursive
traversal/resource gaps are recorded for follow-up. Earlier dynamics cost results
precede this lower-layer correctness repair; newly built artifact hashes are
recorded separately and need fresh cost qualification. The extraction retention
gate and all earlier phase obligations remain open; the full goal is active.

Evidence commit `e6895420e0149991d5d8ceaab75c4671f987ffa6`, record
`docs/evidence/2026-11-12-interval-field-derivative-repair`: 43 files,
1,099,661 bytes, SHA-256
`91bceea8715cec1cc075562f8f79b3dba68e2be33a8a8d203e60f3b9a655e220`.
Its committed digest was regenerated and verified, preserving every earlier
manifest row. The exact repair and qualified pending-tree patches, source base,
commands, failures, final logs, branch-equality evidence and rebuilt artifact
identities are retained. This registration commits only the ledger and manifest;
the independent four-path source repair is the preceding commit.

## 2026-11-12: Own and replay symbolic expression programs

Previous turn: progress, repairing the symbolic power boundary and unavailable
Jacobian estimate. This increment advances section 10.5 in the symbolic owning
layer. Source commit `32dea45c8e4f7aac347a808f05095d3d6a3cb50a` adds the opt-in
Hypersolve `expression-program` feature, optional serde, a standalone example,
and its qualification corpus. Existing consumers are not migrated.

`ExpressionProgram` retains an immutable flat definition with ordered bindings,
all 17 Expr constructors, full source metadata and original domain guards.
Replay validates distinct variable identifiers, every binding, strictly backward
operand references and output indices, including unused nodes and empty output
lists. Exact evaluation follows output and operand order, reuses shared DAG
nodes and propagates typed scalar failures. Structural equality is not function
equality; serde is experimental and unversioned. The oscillator example consumes
existing symbolic derivative outputs and replays exact point/Jacobian values.

Compilation borrows recursive Expr inputs and traverses them iteratively.
Graph-work and definition-entry limits check before structural growth, observe
cooperative cancellation and permit fresh-budget retry. They do not bound scalar
cloning/arithmetic/destruction, coefficient growth, serialized ingress or heap
bytes. Caller-owned Expr destruction remains recursive. Source-bound interval
Jacobians and bounded derivative construction are not yet implemented.

The 14 focused tests cover every operator, independent exact expectations,
400 generated shared DAGs, source mutation and binding order, malformed replay,
guarded failures, error order, exhaustion/cancellation/retry, fresh-process
replay and a 20,000-deep source on a small stack. They pass on Rust 1.95, in
release mode and under AddressSanitizer. A real avoidable intermediate clone
was caught by scalar dispatch comparison and repaired with final-use moves.
The retained fresh-process warm witness preserves the complete old evaluation
trace: 11 dispatch events, zero rational temporaries, GCDs and sign queries.
Initial unmatched-cache and parallel-test interference logs remain retained;
no universal trace equivalence is claimed.

All 19 final qualification commands pass. Distinct owner coverage comprises
627 library tests, 211 smoke tests, two README tests, four previous derivative
regressions and the 14 new tests. The selected downstream configuration passes
454 Hyperdynamics/Hyperpath tests. Strict owner all-feature/all-target Clippy,
warning-free docs, stable WASM, standalone/external consumers, formatting,
diff validation and source-budget checks pass. This is affected qualification,
not a full pre-push, hosted OS or hardware matrix. No push occurred.

Twelve new completed-workload Criterion cases measure compile, replay including
cloning, and exact evaluation/output checking for degree 1/8/32/128 polynomials.
Absolute medians range from 0.389 to 47.577 microseconds; these are not speedup
or validated-field claims. The reused allocation probe completes all 28 success
and exhaustion rows with zero live bytes after each counting epoch. Degree-128
compile/replay/evaluate peak at 74,028/49,476/45,424 measured bytes. Caller inputs
and prior scalar caches are excluded. Artifact hashes, sized-function coverage,
raw samples, source/dependency/toolchain/hardware records and adverse results
are retained; no old owner/downstream cost gate is credited from these numbers.

Evidence commit `051801701b369fb796059d54d622acea9258cc52`, record
`docs/evidence/2026-11-12-interval-owned-expression-program`: 128 files,
4,190,515 bytes, SHA-256
`e1f28d147eeab09d14b3dd282550a9979b6255d7cb75828af2a73dd8b3b9e306`.
The committed digest was regenerated and verified. The pending dynamics patch
changes only its Cargo.lock blob-header identities after the independent serde
addition; every other patch byte is identical. Its new SHA-256 is
`79d7ec1dbcbed5bc368a69c43e2ad872f22e04eb2b0cff0676f9eac26ba0e377`.
Interface's pending patch and Firmware are unchanged. Dynamics extraction
retention remains open, and its prior timing qualification remains historical.

The full goal remains active: finish bounded derivative/source associations,
interval field images and Jacobians, then validated Euler/Picard evolution,
existence/uniqueness evidence, bounded halving, exact horizon, retained failed
attempts, flow tubes and end enclosures. No Phase 12 completion is claimed and
no earlier phase obligation is removed. This registration changes only the
ledger and evidence manifest.

## 2026-11-13: Enclose owned expressions without weakening source guards

Previous turn: progress, owning and replaying exact expression programs. Source
commit `edb68461ece8dc8a170799df06cd11269b25e487` advances section 10.5 with
Hyperinterval principal dyadic `asin`, `acos`, `acosh` and `atanh` images, and
Hypersolve's opt-in `interval-program` interpreter. Native monotonicity and
certified arithmetic retain complete/partial/empty domains and explicit
unbounded atanh endpoint limits. Exact special values and empty inputs survive
extreme requested grids. These are outward enclosures, not tightness promises.

`eval_dyadic` consumes all 17 Expr constructors over Cartesian input products,
retains the exact owned definition and accepts only complete operand-domain
images. It preserves source guards under zero factors and powers, including
Expr's exclusion of zero-to-zero. Native failures retain their node and source;
dependency overestimation returns `DomainNotCertified`, not a proof of invalid
geometry. Empty input products map to empty outputs after graph/binding checks;
zero-dimensional inputs remain the unit product. Shared nodes, output/operand
order, budgeted interval copies, cancellation and fresh-budget retry are covered.
Graph and dyadic work accounts remain separate. Hyperreal constant refinement,
total heap bytes and bounded derivative construction remain outside the complete
resource contract. No trajectory or source-bound Jacobian certificate is claimed.

Five shared generic-traversal refactors were rejected because the point benchmark
binary grew by 624/920/304/360/384 bytes. The final point evaluator and consumption
helper remain unchanged. The specialized interval interpreter reuses graph
validation, use counts and native arithmetic authority. Its point-only benchmark
artifact matches the accepted 7,046,936-byte size, 3,151,803-byte sized-function
range union, GNU text/data/bss sizes, and byte-identical `.text` and `.rodata`.
Whole-file identity and a speedup are not claimed. Existing point dispatch
assertions remain unchanged and pass; no old consumer refactor gate is waived.

All 16 final qualification commands pass: 27 distinct selected native tests,
866 Hypersolve owner tests, strict all-feature/all-target Clippy, strict docs,
examples, formatting/diff/source-budget checks, Rust 1.88 native and Rust 1.95
program tests, release tests, AddressSanitizer, stable Wasm builds, and 454
selected downstream Hyperdynamics/Hyperpath tests. The four new native tests
include 2,068 directed 512-bit MPFR endpoint comparisons. Eight program-image
tests cover independent affine expectations (588 boxes), all operators, source
guards, shared DAG/repeated outputs, error order, symbolic constants, existing
derivatives and resource boundaries. Existing fresh-process replay now checks
interval images too. A new test's incorrect one-cell constant-width assumption
was corrected to independent containment and exact fresh/live replay; original
assertions are unchanged. Initial compile/test failures remain in the evidence.

An external probe reuses the existing allocation helper and completes 42 warm
interval workloads (17 constructors and four polynomial degrees, each at grids
-16/-64), plus 84 work-exhaustion cases and successful retries. All allocation
epochs end with zero live bytes. Preparation is excluded; evaluation, full output
checks and destruction are included. Absolute medians span 0.402–269.271
microseconds, with a largest observed temporary peak of 57,808 bytes. Consumed
graph/arithmetic work peaks at 3,081/16,053,477 under the published entry, bit
and work budgets. These are absolute completed-image observations, not matched
speedups, total-work proofs or validated-flow measurements. Raw samples,
outputs, sources, dependencies, artifacts and adverse results are retained.

Evidence commit `943d4a68409179b1172108fbbb3523cbf88983f1`, record
`docs/evidence/2026-11-13-interval-expression-images`: 123 files, 21,371,619 bytes,
SHA-256 `e61bb904dae9d0cede8772e607a4915289a6bd4098c73fa2fccfb9736c56ec7c`.
The committed digest was regenerated and verified. Pending dynamics and
Interface patches remain byte-identical to their preceding recorded hashes;
Firmware is unchanged. Dynamics extraction retention and its earlier cost
qualification remain open. No push or full pre-push/hardware/hosted-OS claim
is made. The full goal remains active: bounded derivatives and source association,
owned field/Jacobian semantics, complete resource accounting, then validated
Euler/Picard evolution and its required downstream proofs. Earlier phase
obligations remain open. This registration changes only the ledger and manifest.

## 2026-11-13: Build source-bound symbolic Jacobians with graph budgets

Previous turn: progress, accepting guarded interval expression images. Source
commit `f69be5f322b697861a7a46179858562c693190ee` advances section 10.5 with
`ExpressionProgram::jacobian` and its private `JacobianProgram` association.
The original source definition and ordered differentiation variables determine
identity and replay; replay validates the source and regenerates derivatives.
Combined evaluation visits original outputs before column-major partials, and
`partial_output` maps their positions. Both shipped oscillator examples now use
this construction rather than independently supplied derivative trees.

Each demanded node is visited once per column. Fixed-size local templates call
the existing Expr derivative authority and substitute source references into a
shared flat graph; arbitrary source trees are never reconstructed. One source
prefix copy serves all columns. Known zero partials skip derivative work while
retaining every original output's domain checks. Construction charges graph
copies, binding/column checks, traversal, local expansion, template import and
final validation, with typed limits, cancellation and retry. Scalar payload
cloning, arithmetic/refinement and heap bytes remain outside total-work bounds.
A removable derivative singularity can still prevent formula evaluation; that
failure is not a proof of nondifferentiability or a global smoothness result.

Target-revision reproduction found a necessary correction: differentiating
`x^1` introduced `x^0`, which incorrectly rejected x=0. The identity-power rule
now retains guarded base evaluation through `0 * base + derivative(base)` and
existing domain-aware simplification. Tests cover zero/nonzero identity powers,
ln(x)^1 at a zero base, excluded logarithm inputs and genuine sqrt derivative
singularities. This is a correctness extension, not a preserving refactor. The
original minimum-exponent and unavailable-Jacobian regressions remain intact.

Seven new Jacobian tests cover the 17 constructors, extreme exponents, 81
independently expected polynomial input pairs, repeated differentiation, domain
guards, empty/invalid bindings and columns, exact work/entry boundaries,
cancellation/retry, shared DAGs, interval extrema, and fresh-process replay with
source mutation/regeneration. A 128-level shared squaring graph retains linear
node growth and computes slope 2^128 at one. A 20,000-deep graph constructs,
differentiates, evaluates and drops on a 128-KiB stack.

All 19 qualification commands pass: 874 owner tests, minimal features, Rust 1.95,
release, strict owner/Interface Clippy, examples/docs, Wasm, AddressSanitizer,
459 dynamics/path tests, 201 Interface tests, formatting/diff checks and benchmark
smokes. All 200 successive_operations rows match the earlier derivative-repair
record's structural/outcome/allocation columns; timings are not compared.
Certification/path/point smoke suites pass 417/194/12 cases. This is affected
qualification, not a full pre-push, hosted-OS or hardware campaign. No push occurred.

An external probe completes 16 constructor/replay/point/interval workloads and
12 exhaustion cases with zero live bytes per counting epoch and successful
retries. Warm absolute medians span 0.873–389.775 microseconds; maximum observed
peak is 115,256 bytes. Degree 1/8/32/128 programs grow from 5/33/129/513 source
nodes to 7/49/193/769 generated nodes and spend 71/526/2,086/8,326 constructor
graph units. Preparation is excluded; replay cloning, output checking and
returned-object destruction are included. No speedup or whole-field bound is
claimed. The point benchmark grows 832 bytes while its sized-function range
union shrinks 1,282 bytes; text/rodata change. Certification/path artifacts grow
176/104 bytes from the historical derivative-repair artifacts. These costs are
retained explicitly with the correctness extension and new capability, without
an unchanged-cost claim or acceptance of the pending extraction's cost gate.

Evidence commit `fdb00c27b2b629298213966fd9f0323914bcb98b`, record
`docs/evidence/2026-11-13-interval-program-jacobians`: 80 files, 12,401,121 bytes,
SHA-256 `f04ba69ed417f79f63b5f09ab753af70e25bb9cb46fc613f6518e457852c0850`.
The committed digest was regenerated and verified. Complete source/dependency/
compiler/hardware records, raw samples, artifact identities and the original
failing witness are retained. Pending dynamics/Interface patches remain
byte-identical, and Firmware is unchanged. A Hyperdynamics field adapter with
explicit time/state/shape semantics, whole-operation accounting and derivative/
continuity evidence remains open, followed by validated Euler/Picard evolution,
uniqueness, bounded halving, exact horizon/end enclosures and flow tubes. The
full fifteen-phase goal and all earlier unresolved obligations remain active.
This registration changes only the ledger and evidence manifest.

## 2026-11-13: Add owned expression field enclosures

Previous turn: progress, accepting source-bound symbolic Jacobians. Source commit
`afe74ec511744b48aa6229d0fdd9b3f69bcc4ad6` advances section 10.5 with an isolated
Hyperdynamics field core. `ExpressionField<N>` implements the sealed
`EnclosedVectorField<N>` interface using exactly N outputs, N distinct state
variables and one distinct time variable, with arbitrary source binding order.
Hyperdynamics owns field/time/state semantics; Hypersolve retains expression and
derivative authority, Hyperlattice supplies native Cartesian carriers, and
Hyperinterval supplies scalar arithmetic. No second mathematical kernel is added.

Immutable structural identity includes the full original source and time/state
mapping. Replay deserializes only `FieldDefinition`, then validates the source
and regenerates its Jacobian. Images evaluate the original source independently
of derivatives; Jacobians evaluate original outputs before partials and move them
into native row-major matrices. Original domain guards survive zero derivatives.
The sqrt field at zero can have a valid value image and an uncertified derivative
image. Empty products and N=0 unit products retain their distinct native meaning.
The shipped oscillator example consumes its Jacobian through native linear algebra.

One mutable `EnclosureBudget` carries explicit immutable policy and cumulative
graph/scalar counters across construction, image and Jacobian calls. Hypersolve
exposes its existing work charge and entry-admission authority for the adapter;
existing algorithms and call order are unchanged. Tests verify exact combined
image/Jacobian budgets, one-unit-short failures, entry-policy transitions,
cancellation even for empty outputs, and successful retries. Source ingress/
serde, scalar payload cloning/destruction, Hyperreal constant refinement and
heap bytes retain their existing accounting exclusions. This is not wholly
bounded field work or a continuity/existence/uniqueness certificate.

Seven field tests include 125 independently expected affine cases, reordered
bindings, nonlinear images/partials, zero-power guards, singular derivative vs
valid value behavior, zero dimension, invalid source/shape, immutable identity,
changed coordinate interpretation, fresh-process replay with sqrt(2), limits,
cancellation and retry. All seventeen qualification commands pass: all-feature,
minimal/default, Rust 1.95, release, strict all-target Clippy, docs/example, Wasm,
AddressSanitizer, 874 owner tests, 466 integrated dynamics/path tests, formatting
and diff checks. Supplementary staged-diff and retention checks pass, as do all
12 point evaluator benchmark smoke cases. This is affected qualification, not
the full pre-push matrix, hosted-OS/hardware campaign or release packaging.

The external consumer completes 35 constructor/JSON-replay/image/Jacobian/pipeline
workloads and 21 exhaustion cases, each with zero live allocated bytes after its
counting epoch. Warm absolute medians span 2.052–907.151 microseconds; maximum
observed peak is 173,660 bytes. Degree 0/1/8/32/128 constructor graph costs are
51/116/599/2,255/8,879 units. The probe includes required validation, output checks
and returned-object destruction; preparation is excluded as documented. It uses
its own lockfile and path dependencies, so this is neither a matched speedup
claim nor published-package qualification. The point artifact grows eight bytes
to 7,047,776; .text/.rodata remain byte-identical and its sized-function union
remains 3,150,521 bytes. Other section bytes differ. The field consumer is
3,934,024 bytes with a 1,992,385-byte sized-function union. Raw samples, symbols,
sections, compiler, hardware and complete source/dependency records are retained.

The core was committed from an isolated worktree. Advancing the main branch
preserved every working-file byte. The pending extraction was restaged over the
core and now has 40 paths because the two licenses are committed with the core.
Legacy algorithms, adapters, tests and the original public API body are unchanged;
package metadata/docs compose with the field core. The pending patch hash is now
`c6e2e573a168966c9f81e46120a39a5c4e2fe9da01a6f5868d6ad9c0a171b090`;
the original patch is retained. Interface patch bytes remain unchanged, Firmware
is clean, and extraction retention remains open. No push was attempted.

Evidence commit `8ddc03ef0f56eb2f6f5d3e6bc74823502101db21`, record
`docs/evidence/2026-11-13-interval-expression-field`: 132 files, 10,403,504 bytes,
SHA-256 `f648855fd472e7dff357cbe3735cb3361123350f819336a12812826a9fba7c51`.
The committed digest was regenerated and verified. Whole-field resource bounds
and derivative/continuity evidence remain open, followed by validated Euler/Picard
evolution, uniqueness, bounded halving, exact horizon/end enclosures and flow
tubes. The full fifteen-phase goal and earlier unresolved obligations remain
active. This registration changes only the ledger and evidence manifest.

## 2026-11-13: Admit rational constant costs before materialization

Previous turn: progress, accepting the owned field core. Source commit
`b3fb87d9bd6457d3f6b64ddbf219d299e1eb7a8b` closes a reproduced native resource
gap under that field adapter. On the target revision, `from_real` rejected
129-/4,097-/131,073-bit rational coefficients under a 64-bit limit only after
four allocations totaling 248/1,240/32,984 bytes per call. The same external
zero-allocation assertion now passes all three cases. The original failure,
all sizes, executable identities and allocator source are retained.

Hyperreal adds `Rational::ratio_parts`, a borrowed sign and potentially unreduced
numerator/denominator view with no normalization, allocation or cache publication.
Canonical getters and rational storage/arithmetic remain unchanged. Hyperinterval's
existing rounding kernel uses this view to reserve actual stored sizes before
coefficient copies; its existing scaled-division kernel owns arithmetic admission.
`DyadicInterval::from_rational` returns tight outward grid bounds through those
kernels. Structurally rational `from_real` inputs use it after the existing grid
check. Other symbolic inputs keep their existing supported refinement path.

This is a resource correctness extension, not a preserving refactor. Budget work
and outcomes intentionally reflect admitted stored coefficients. A large unreduced
ratio may need a larger budget even when its reduced value is small; hidden GCD
work is no longer used to bypass admission. The conversion and admission size do
not depend on canonicalization-cache warmth. Source identity and exact outward
bounds remain unchanged. General symbolic refinement, other bridge validation
costs, source ingress/serde, scalar payload lifecycle and total heap accounting
remain open; this does not establish wholly bounded field evaluation.

Six new tests cover borrowed 8,195-bit lazy components and unchanged caches,
13,243 independent signed rational/grid cases, further native set/arithmetic
consumption, large coefficients, exact/one-unit-short budgets, precision extremes,
retry, genuinely unreduced line-intersection coordinates, zero refinement/
canonicalization dispatch counts and field image/Jacobian exhaustion/retry.
Existing fresh-process replay and symbolic sqrt(2) witnesses remain intact.
An initial already-canonical complex-quotient test fixture was replaced by a truly
lazy line-intersection fixture while preserving the unreduced-storage assertion.
No existing test, policy or fixture was weakened.

All fourteen final and five supplementary qualification commands pass: 182 native
interval tests, 218 rational-owner tests, 874 Hypersolve owner checks, 467 integrated
dynamics/path tests, feature-specific MSRV checks, release, strict all-target
Clippy, Wasm, docs, AddressSanitizer, formatting/diffs and retention budgets.
All eight field tests also pass in the isolated package without the pending
extraction, and all 12 point benchmark smoke cases pass. An initial Rust 1.88
all-feature command rejected Hyperreal's documented Rust 1.95 requirement;
correct rational/Real 1.95 and dyadic-only 1.88 configurations both pass. The
initial rejection is retained; no compiler requirement was bypassed.

The unchanged external field probe compares parent/candidate at the same paths
and lockfile, completing 35 constructor/replay/image/Jacobian/pipeline workloads
and separately testing 21 exhaustion cases. CPU-0 CBBC runs initially showed
constructor/replay regressions up to 26.8%, prompting BCCB repeats. All initial
samples and adverse results remain retained. Pooled results include all 124 timing
samples per implementation per case: median candidate/control ratios are 0.9533
for construction, 0.9449 replay, 0.7885 image, 0.8172 Jacobian and 0.8809 pipeline.
Four affine construction/replay cases remain slower, up to ratio 1.03042. These
finite noisy measurements do not establish a universal or across-the-board speedup.

Every allocation epoch ends with zero live bytes. Degree-128 pipeline allocations
fall from 8,879 to 8,105; maximum observed peak remains 173,660 bytes. Graph work
is unchanged; degree-128 image/Jacobian scalar accounting falls from
4,528,715/4,599,743 to 2,280,761/2,351,789 units. The field executable grows 160
bytes to 3,934,184 and its sized-function union grows 201 bytes to 1,992,586.
The point benchmark shrinks eight bytes with identical .text/.rodata and unchanged
3,150,521-byte function union; other section bytes differ. These costs accompany
the demonstrated resource correctness gain, without an unchanged-cost claim.

Evidence commit `f95aafa6ebfe45f67815b159a63b04328b40c5c1`, record
`docs/evidence/2026-11-13-interval-rational-constant-budget`: 108 files,
8,903,429 bytes, SHA-256
`1ecfb2cd02584490820249811dbca613cda0bbfd390e85b96d1141655d3f14aa`.
The committed digest was regenerated and verified. Pending dynamics and Interface
patches remain byte-identical, Firmware remains clean, and no push occurred.
Full pre-push/hosted-OS/hardware/release qualification is not claimed. Complete
field resource and derivative/continuity contracts, validated Euler/Picard
existence/uniqueness, bounded halving, exact horizon/end enclosures and flow tubes
remain open, as do earlier unresolved phases and the extraction retention gate.
The full goal stays active. This registration changes only the ledger and manifest.

## 2026-11-13: Verify source-bound dyadic Picard steps

Previous turn: progress, admitting rational coefficient costs before copying.
Source commit `f6c48522d2784b6bbd4adf4dbeae6466758b3aec` adds a sufficient
source-domain regular Jacobian enclosure in Hypersolve, native vector scale in
Hyperlattice-interval, and a one-step Picard verifier in Hyperdynamics. Original
source continuity and strict active primitive domains establish state regularity;
time-only square roots can include zero. Conservative regularity rejection is
not proof of nondifferentiability. No serialized flag is accepted as proof.

For exact dyadic singleton times and caller-supplied X0/Y, the verifier checks
forward time, X0 subset Y, P = X0 + [0,h]F(T,Y) subset Y and h*upper(||J||inf)<1.
It checks the final endpoint contribution before certification. Immutable reports
retain source, proposal, policy, completed intermediate evidence and typed status.
Failed proposals expose no certified endpoint/tube. Empty initial sets and the
nonempty zero-dimensional unit remain distinct. Native outward integral images
include rounding; no Taylor contribution is discarded or exact point trajectory
claimed. The shipped time-dependent example consumes its first endpoint in a
second certified step with shared counters.

All 21 final qualification commands and two supplementary checks pass: 876
Hypersolve owner tests, 473 dynamics/path tests, 80 Cartesian interval tests,
feature-minimal/default-off and MSRV configurations, release, strict all-target
Clippy/docs, Wasm, AddressSanitizer, isolated field package, shipped example,
12 point benchmark smoke cases, formatting/diffs and repository budgets. Nine
added tests and existing-test extensions cover independent analytic expectations,
49 uncertain constant-field chains, a nonsymmetric Jacobian, temporal versus
state singularity, invalid/empty/unit inputs, final-contribution exhaustion,
shared budgets, cancellation/retry and fresh-process symbolic replay. No existing
assertion, policy or fixture was weakened. This is affected validation, not the
complete pre-push workflow matrix; no push occurred.

The new external probe completes 15 timing rows, with three proposal rejection
rows, 21 exhaustion-boundary rows and seven budget-16 trials reported separately.
Completed medians span 0.565–20.172 microseconds, peak measured temporary storage
is 3,312 bytes, and every allocation epoch ends with zero live-byte delta.
Verification, source replay and a two-step chain include full report materialization
and destruction. An initial probe incorrectly expected an empty step to exhaust
budget 16; the corrected probe retains that successful case and checks measured
required work minus one. The original failure and build diagnostics are archived.

Matched parent/candidate field probes complete the same 35 workloads plus 21
exhaustion cases. Allocation, peak-memory and graph/scalar counts are identical.
CBBC and reverse BCCB measurements retain every sample and adverse result:
pooled median ratios range 0.9553–1.1134, overall median 1.0198; 21 rows are slower.
This is a proof-capability extension, not a performance optimization or a claim
of regression-free timing. Existing field-consumer bytes fall 3,934,184 to
3,931,664, sized function union 1,992,586 to 1,991,490. Point benchmark .text and
.rodata are identical and its function union remains 3,150,521 bytes. The distinct
Picard probe is 4,022,968 bytes with a 2,037,790-byte function union; this is an
absolute artifact measurement, not an incremental feature-size comparison.

Evidence commit `3d0eb55d91617bb9b6253e3a45ef830562d2683b`, record
`docs/evidence/2026-11-13-interval-picard-step`: 101 files, 6,565,124 bytes,
SHA-256 `add71eb4bce245b2fb6f7d9a07fc2d42aa8973e0d8a0895f1e748d490f27be39`.
The committed digest was regenerated and verified. The 40-path pending dynamics
patch and Interface patch are byte-preserved; Firmware and the isolated core are
clean. General symbolic/resource accounting, rational time endpoints, automatic
bounded halving/proposals, horizon integration and complete flow-tube aggregation
remain open. The legacy extraction retention gate and full fifteen-phase goal
remain active. This registration changes only the ledger and evidence manifest.

## 2026-11-13: Compose bounded dyadic evolution with retained attempts

Previous turn: progress, admitting source-bound Picard steps. Source commit
`61e867f5062ee692c6805e3113ffe9c2d6b3a920` adds finite-horizon integration using
that verifier, native interval proposals, bounded candidate expansion and exact
dyadic halving. Proposals are capped by exact remaining time and the last accepted
duration. Every actual attempt retains its source, candidate origin and ordered
segment/halving/expansion indices. Each accepted endpoint becomes the next
initial box through native checked copying; no proof counters reset.

Whole-horizon completion requires strictly progressing certified segments and a
final exact end-time comparison. A failed final comparison withholds completion
even when a certified prefix already reaches that time. Stopped reports expose
that prefix separately. Zero horizon is an identity without an ODE existence
claim; successful empty evolution validates inputs and reports the requested end
time. Invalid input, native failures, cancellation and actually reached limits
remain typed, with mathematical rejection reasons retained in the step reports.
The shipped example reaches time 5/8 in three certified segments and retains all
seven attempts. No new scalar, derivative or step-proof authority was introduced.

All fifteen final qualification commands and three supplementary checks pass:
480 dynamics/path tests, 18 minimal-feature tests, default-off, all 21 field/flow
tests under Rust 1.95/release/ASan/isolated package configurations, strict Clippy
and docs, Wasm, example, external builds, formatting/diffs and repository budgets.
Seven new tests include 49 uncertain constant-flow horizons with endpoint reuse,
exact partial final steps, forced affine contraction halving, independent rational
exponential and oscillator Taylor bounds, exact Riccati controls, negative time,
minimum-exponent halving overflow, invalid/empty/unit/identity cases, complete
attempt provenance, aggregate budget boundaries, cancellation/retry and
fresh-process replay. The initial oscillator-halving assumption was corrected
using a separate q=1 affine fixture while retaining and extending the oscillator
analytic checks. No existing test, policy or assertion was weakened. Earlier
failed diagnostics and control-executable permission correction remain archived.

The external probe completes 18 integration/replay workloads and separately
records four stopped workloads and 36 exhaustion cases. Full retained reports
and destruction are included. Completed medians span 0.179–126.633 microseconds,
peak temporary memory is 26,780 bytes, and every allocation epoch ends at zero
live-byte delta. The Riccati stress run retains 149 attempts and 24 certified
segments before its actual step limit, reaching 2357295/4194304 rather than the
requested singular horizon. Its stopped workload peaks at 272,008 temporary
bytes with a 2.357-millisecond median; this is not completed-flow performance.

The matched existing Picard consumer has identical allocation/work counts,
4,022,968-byte executables, unchanged 2,037,790-byte function unions and identical
.text/.rodata. CBBC completed timing ratios span 0.9786–1.1059, median 1.0027;
eight rows are slower. All samples remain, with no speedup or regression-free
timing claim. The distinct evolution consumer is 4,084,752 bytes with a
2,065,448-byte sized function union, an absolute artifact measurement.

Evidence commit `b19c727cd6ba09022009ceb03fed3a7f32f6177d`, record
`docs/evidence/2026-11-13-interval-dyadic-evolution`: 98 files, 3,255,280 bytes,
SHA-256 `c81cd6cd5ae19d54617496bac5d22e2847a4b77c9f5dfd0c7e5e536b3b3fef65`.
The committed digest was regenerated and verified. The pending 40-path dynamics
patch and Interface patch are byte-preserved; Firmware and the isolated core are
clean. This is affected validation, not the full pre-push workflow matrix; no
push occurred. Rational time endpoints, f64 RK proposals, stronger wrapping,
end covers/thickening, total resource closure, release qualification and all other
plan obligations remain open. The full goal and legacy extraction retention gate
remain active. This registration changes only the ledger and evidence manifest.

## 2026-11-13: Retain exact rational physical clocks through evolution

Previous turn: progress, composing bounded dyadic evolution. Source commit
`6f2c110f3758c5059df7e852536769bc94bcd2b6` extends that path to exact rational
physical horizons. For a<b, retain H=b-a and t=a+H*tau; Hypersolve constructs the
source-bound DAG for H*f(a+H*tau,x). The existing field/Jacobian and Picard owners
remain the sole derivative and ODE authorities. Original source guards survive
zero output scaling, parameter expressions stay shared, and the original source,
node mapping and exact affine specification remain available.

Physical reports bind every accepted normalized proof to its original field and
exact rational time interval. Physical endpoint materialization and final horizon
equality must succeed before whole-request completion; exhaustion can withhold the
final endpoint even after normalized integration completes. All normalized attempts,
partial physical prefixes and separate mapping failures remain inspectable. Equal
time is an identity without an ODE existence claim; empty/unit/invalid outcomes
remain distinct. Options explicitly use normalized tau units. Replay validates
original source/endpoints and regenerates the transformation and proofs. The
README, crate overview and shipped example describe this path and its limits.

The new borrowed rational admission view inspects raw stored coefficient sizes
before delegating to Hyperreal, sharing the native dyadic counter. Its declared
comparison/arithmetic weights and coefficient envelopes are not bounds on GCD
steps, CPU work, temporary heap, scalar payloads or caches. Whole-resource closure
remains open. The isolated package adds the required optional production Hyperreal
dependency; external dependency versions are unchanged.

All 20 final checks and eight supplementary groups pass: 199 native interval,
1,418 Hypersolve and 487 dynamics/path tests including docs; minimal/default-off
configurations; Rust 1.95, release and isolated package runs; strict Clippy/docs;
WASM; shipped example; ASan; owning/path/point benches; 201 Interface tests; and
format/diff/budget checks. Six clock tests exercise 49 uncertain constant flows
with endpoint reuse, independent physical-time affine and Riccati controls,
300-bit-close and 512-bit-large endpoints, exact physical coverage, invalid/empty/
unit/identity cases, shared budget boundaries, partial materialization, cancellation,
retry and fresh-process replay. Native/affine tests add 14,161 independent fraction
pairs and 2,401 value/Jacobian samples. Initial compile, MSRV-command, Clippy and
documentation diagnostics remain archived; no fixture, policy or assertion weakened.

The unchanged constructor reproduces fixed full dispatch/reducer counts, work and
serialized source. All 13 complete evolution report signatures match the baseline,
including nine completed and four stopped workloads with every local proof and
failure scope. Allocation and work rows match. The 200-row successive corpus keeps
all geometry/outcome/complexity fields and allocator-call counts; concurrent smoke
run timings are not used as a speed claim.

Initial CBBC completed ratios were adverse (1.0051–1.1080, median 1.0710); reversed
ordering varied substantially. Under the same two-second scoped CPU warm-up and
balanced order, four runs per variant give retained-helper ratios 0.9205–1.0166,
median 0.9936; seven medians are slower, with overlapping per-run ranges on adverse
rows. No governor settings changed. All unfavorable timings remain, with no speedup
or regression-free timing claim. A macro-sharing experiment supplied no clear
completed-work benefit and was rejected; the final direct-helper source and binary
were restored byte-for-byte. The unchanged release consumer shrinks from 4,084,752
to 4,082,872 bytes; function union shrinks from 2,065,448 to 2,065,371 bytes.

The new clock probe completes 18 integration/replay cases with all physical
materialization and destruction, at observed medians 0.427–175.960 microseconds and
maximum completed peak allocation 15,188 bytes. Its 36 deliberate budget failures
and 12 zero-allocation large-coefficient rejections remain separate from completion.
Every allocation epoch ends at zero live bytes. The new consumer is 4,028,456 bytes,
an absolute measurement of a different workload, not a size-comparison claim.

Evidence commit `fc47eefd79f70849ec1f43e7febf9e08d5a2c317`, record
`docs/evidence/2026-11-13-interval-rational-time`: 184 files, 6,308,703 bytes,
SHA-256 `ae7faec60dee7c80b7e6c25e6b00611b13662da19392b470a79798ef9af79db9`.
The committed digest was regenerated and verified. The remaining legacy extraction
still has 40 staged paths; only the accepted dependency/README/crate overview change
its delta against the new base. All other original staged blobs and the Interface
patch are preserved. Firmware and the isolated core remain clean. No push occurred;
this is affected validation, not the complete pre-push workflow matrix. RK proposals,
stronger wrapping/end covers, total resource closure, legacy retention, platform/HIL,
release qualification and all other plan obligations remain open. The full goal
stays active. This registration changes only the ledger and evidence manifest.

## 2026-11-13: Verify source-bound primitive RK4 proposals

Previous turn: progress, accepting exact rational physical clocks. Source commit
`c7bfacdff04ca228193d4c6d0dd1a16a85c4e88c` adds the optional phase-12 RK proposal
path. Hypersolve prepares a primitive view retaining the exact DAG, ordered
bindings, demanded-node sharing, original guards and full-width integer powers.
Hyperdynamics records four RK stages, heuristic padding, the unchanged exact
request and partial failures. The candidate adopts finite IEEE coordinates exactly
and is consumed by the existing source-bound Picard verifier. Only that verifier's
report exposes a certified tube or endpoint. Native horizon integration remains
available where float projection fails. Source/request/options are rebuilt for
replay; numerical traces and serialized success flags are not certificates.

Hyperinterval owns exact finite `DyadicInterval::from_f64` ingress. It shares the
existing decoder and admission-before-allocation point construction with enclosure
conversion, without requiring strict-float platform qualification or Hyperreal.
NaN/infinity are typed invalid inputs; subnormals retain their exact values and
signed zero canonicalizes. Importing 0.1_f64 adopts its binary value, not 1/10 or
an earlier source. No exact evaluator or native integrator is replaced.

Final review reproduced a new-option validation defect in debug: DAZ admits a
negative subnormal through native `< 0.0` checks, in eight modes for each of two
padding fields. The release baseline already rejects those inputs through different
optimized lowering; that passing baseline is retained. Reusing the existing
bit-ordered `[0,padding]` constructor rejects all 32 cases in both configurations.
Negative zero remains valid and its bits remain in the trace. The failed debug
baseline and existing MXCSR helper are retained with the executable reproduction.

All 23 primary, 12 supplementary and 14 final boundary-check groups pass. The
post-layout suites report 204 Hyperinterval, 1,422 Hypersolve and 496 dynamics/path
tests including applicable docs; the isolated accepted package reports 37 and
Interface reports 201. Coverage includes minimal/default-off configurations,
Rust 1.88 minimal ingress and Rust 1.95 field/program, release, ASan, strict Clippy
and docs, WASM, no-std thumb ingress compilation, examples, owner benches and
formatting. Independent controls cover the complete owned primitive grammar,
4,096 seeded IEEE/Rational pairs, 49 uncertain constant flows, affine/exponential/
oscillator/Riccati cases, rounded-away source-domain failure, poor candidates,
stage poles, every graph cutoff, scalar exhaustion, cancellation/retry, empty/unit
and invalid inputs. Certified output feeds native evolution and rational clocks.
Five fresh-process cases rebuild source/request/options and compare retained
numerical bits, exact evidence, status and resource counts; bad references and
bindings are rejected. Full-resource and universal ODE completeness are not claimed.

All 13 old evolution report signatures agree with the accepted baseline, including
nine completed and four stopped reports with complete attempt/proof/prefix scopes.
Fixed constructor dispatch/work assertions remain unchanged; IEEE conversions have
zero Hyperreal dispatch and no predicate-policy parameter. The 200-row successive
corpus preserves geometry, certainty, complexity and allocator counts. Its timings
are smoke validation, not a performance claim.

Matched old conversion, strict-float and native-evolution executable `.text` bytes
are identical; function unions remain 484,827, 271,197 and 2,065,371 bytes. Raw file
sizes are 1,131,416 (eight fewer), 499,936 (unchanged), and 4,082,872 (unchanged).
An initial declaration layout enlarged symbol metadata; keeping existing impl
order and appending optional APIs removes the increase without changing execution
order. All measured old exact/work/allocation rows agree and live-byte deltas end
at zero. Initial balanced timing ratios span 0.99099–1.01841 for conversion,
0.99720–0.99770 for strict float, and 0.98824–1.03847 for native evolution. The last
has median 1.01097 and a nonoverlapping adverse Riccati row. A follow-up normalizes
the executable path/argv and retains both original binaries: ratios 0.98219–1.02648,
median 0.99539, all run ranges overlap, Riccati 0.99036. Both data sets remain.
Identical executable code and resource results support retention; no speedup or
regression-free timing claim is made.

Four successful RK fixtures separately measure preparation, proposal plus exact
proof, and source serialization/rebuild plus proposal/proof. Warm host medians are
0.164–0.394, 6.748–13.352 and 13.875–23.544 microseconds; maximum measured peak owned
bytes are 3,768. Rejected-proof and 15 controlled-exhaustion rows remain separate.
The new consumer is 4,084,144 file bytes and 2,034,974 function-union bytes, an
absolute measurement of a different workload. Projection/refinement, total CPU
work and whole-heap bounds remain outside the declared counters.

Evidence commit `01fd673011500a4e6fc679005ea509ceba75aca5`, record
`docs/evidence/2026-11-13-interval-rk-proposals`: 204 files, 5,883,677 bytes,
SHA-256 `24e03ab992b385d209e6ff75eb88720d8164c0868c84aacbc48e740c0b13fb5e`.
The committed digest was regenerated and verified. All 40 staged legacy paths
remain: 38 blobs unchanged, manifest/README incorporating the accepted API while
preserving legacy edits. The Interface patch is unchanged; firmware and accepted
core are clean. No push occurred; this is affected validation, not the complete
pre-push matrix. Automatic RK horizon selection, wrapping/reachability, whole-
resource closure, legacy retention, platform/HIL and the other plan obligations
remain open. The full goal stays active. This registration changes only the
ledger and evidence manifest.

## 2026-11-13: Reproduce extraction costs and isolate return overhead

Previous turn accepted source-bound RK proposals. This turn returns to the pending
phase-11 extraction at source base `b816c48cca12c922f3dbfb004c4d27586c06ad74`.
The full extraction is implemented but remains unretained. No production edit is
accepted by this investigation. The phase table above now reflects the existing
metric/join/feed/lookahead/jerk candidate and the already accepted phase-12 work;
that correction does not establish any additional implementation capability.

A fresh clean baseline and the exact 40-path candidate were built in one isolated
checkout with Rust 1.95 and locked dependencies. All 194 owner cases have identical
full result observations and allocation rows in a separate completed-result/drop
observer, with zero live-byte deltas. Debug snapshots are differential evidence,
not an independent geometry oracle or canonical serialization. The original owner
benchmark body differs only by two deprecation allowances. Its executable shrinks
13,312,600 → 13,304,584 bytes; complete defined-function union shrinks
8,044,187 → 8,036,358 bytes and `.text` 8,118,103 → 8,110,279 bytes. The main-tree
candidate executable is byte-identical to the measured isolated candidate.

Actual indirect/aliased targets are resolved, including the parser's formerly
ambiguous slot. Userspace instruction profiles of the real owner executable have
identical instruction/memory-operation counts for the selected parser, importer
and skew calls; motion calls differ slightly. The warmed complete-result/drop
observer likewise has equal parser/import/pad-clearance counts and motion count
increases up to about 0.16%. Static equality and simulated cache/branch results
are not hardware-counter evidence or an explanation of wall-clock differences.

Four complete 194-case sweeps use one executable path/argv, one working directory,
fixed CPU affinity, scoped warm-up and balanced order. Every slower case plus every
historically targeted case enters a four-run, 121-case follow-up. Forty-one cases
have candidate mean confidence ranges entirely above both baseline ranges. Examples
include parser 256 (ratio 1.05459), mixed route parsing (1.05806), circular-edge
clearance (1.06905) and arc-wire lifting (1.09998). Selection is not an acceptance
tolerance; aggregate or favorable cases do not clear this failure. A twelve-case
follow-up disables ASLR only for benchmark subprocesses: six adverse ranges remain,
including parser 256 at 1.05908 and arc-wire lifting at 1.08315. All raw unfavorable
observations remain. No governor, sysctl or compiler policy changed.

A single ordinary inline attribute removes the profiled metric-length adapter
call and lowers mixed-jerk instruction count, but leaves other overhead; its file
is 13,304,728 bytes. It was not fully timing-qualified or retained. A separate
shared-error experiment reexports DynamicsError under the old RouteCertificationError
name, preserving all variants. Its 194 observed results/allocations match and jerk
instruction counts fall below baseline, but file size rises to 13,316,864 bytes.
Removing sixteen now-identity conversions leaves 13,316,848 bytes, still above
baseline. Function union is 8,043,152 for both; unwind and symbol sections account
for file growth. Both experiments are reverted. No new public error ownership is
accepted, and no full functional qualification is claimed for rejected variants.

The restored original candidate passes all eleven affected validation groups:
496 owner tests, 201 Interface tests, strict Clippy, docs, WASM, 194 owner benchmark
smoke cases, 200 successive rows, formatting and whitespace checks. Six fresh-process
legacy/direct/compatibility transcripts match within STRICT and APPROXIMATE_512,
with 92 report/trace lines each and unchanged fixed dispatch assertions. Interface's
existing exact lowering checks preserve the 41,795-byte transcript golden digest
`df893df83e90e6b53452e18586ffcaa2524a021631a395d4a6523be16263669c`. Successive structural,
allocator-count and displayed memory columns match the retained 200-row control;
its timings are smoke validation, not a matched performance claim.

Valgrind was extracted locally without installing host packages. A linux-perf
package fetch returned HTTP 404; a direct self-process hardware-counter probe was
denied (errno 13, paranoid=3). The hardware cause of equal-instruction timing
regressions remains unresolved. Further retention work must address code generation,
return copies or layout with evidence; repeating favorable timings or changing
unrelated parser algorithms is not a fix.

Evidence commit `019ee95d1543c705ec98d1be068cf56a405354dc`, record
`docs/evidence/2026-11-13-interval-dynamics-requalification`: 5,480 files,
71,483,391 bytes, SHA-256
`ae1d5ecd798dee0c3f069af4e468daa55b55bc78c5a46ee566393d1be2524e18`.
The committed digest was regenerated and verified. All 40 main staged paths and
eight Interface edits remain byte-for-byte unchanged; isolated experiments are
reverted, accepted core and firmware remain clean. No push occurred. This is
affected validation, not the complete pre-push matrix. The full fifteen-phase goal
remains active, including the unretained extraction and all other open obligations.
This registration changes only the progress ledger and evidence manifest.

## 2026-11-13: Qualify finite epsilon endpoint covers

Previous turn reproduced phase-11 extraction timing regressions and left that
candidate unretained. This turn implements the missing finite-cover boundary in
plan 10.8. Source commit `404ba07ecdeeab8838836df0bc6b36bc78401b20` adds fifteen
paths/changes for optional Hyperdynamics reachability and exact dyadic Cartesian
subdivision/regridding. The full fifteen-phase goal remains active.

`ExpressionField::reach` returns a finite endpoint union C only after certifying
`R ⊆ C ⊆ R + [-epsilon,epsilon]^N` at the exact requested horizon. Exact initial
partitions, source identity, all attempts and native Picard evidence remain
retained. Certified native prefixes are consumed explicitly with checked outward
endpoint transport; their local step-limit statuses are not relabeled complete.
The last transfer, positive progress, horizon and diameter checks precede whole
success. Stopped reports expose no whole-request cover. Empty and identity retain
exact set semantics. Source/request/policies are reconstructed for native replay.

The original uncertain Riccati fixture, with depth 8, maximum 2048 whole attempts,
default 256 segments and epsilon 1/128, first reproduced depth, coefficient-growth
and segment-limit failures. Those observations remain archived. Exact time-cap
refinement, outward continuation and displacement-width proposals now complete
that unchanged fixture through the existing Picard authority. New proposal
provenance is independently reconstructed with rational endpoints. Other independent
controls cover exact constant flows, partition equality, an exponential Taylor sum
with a bounded tail, and Riccati fiber images `x/(1-x/2)`. Tests cover output reuse,
final contributions, multi-prefix structural/scalar budget boundaries,
cancellation/retry, invalid inputs paired with empty, and fresh-process replay.

All fourteen isolated qualification groups and eleven combined downstream groups
pass. Counts are 48 Hyperdynamics tests, 84 Hyperlattice-interval tests, 15 minimal
feature tests, 33 selected release tests and 17 AddressSanitizer tests. Strict
Clippy, warnings-denied docs, WASM, Rust 1.95, default-off checks, the example,
formatting and whitespace checks pass. Combined owner tests total 507; Interface
passes 201. All 194 path benchmark smoke cases pass. The 200 successive rows
preserve structural fields, allocation counts and displayed memory columns; their
timings remain smoke observations. Six fresh-process migration transcripts match
within policy and their previous digests (92 report/trace lines each). Three native
field traces preserve their independently measured fixed counts.

Legacy reports remain byte-identical (177,398 bytes across thirteen signatures)
and all twenty-two resource/allocation rows match. Early inlining and entry-point
experiments reproduced adverse timing or size changes and are retained. Restoring
the original temporary lifetimes and compiling the optional strategy separately
preserves byte-identical `.text` in both old consumers. Cost-observer ELF size is
4,023,072 → 4,023,032 bytes, semantic-observer size 4,160,728 → 4,160,704; function
unions remain 2,100,408 and 2,187,002 bytes. A fixed twelve-pair, alternating-order
follow-up with common path, fixed ASLR/affinity and warming retains every case.
All twenty-two paired bootstrap 95% intervals contain one; ratios range
0.996292–1.002192. This is measured absence of a detected regression, not a universal
performance claim. The pending-extraction path benchmark executable remains
byte-identical to its prior 13,304,584-byte build; its phase-11 blockers remain.

Both new-cost runs complete all six workloads with identical work/allocation rows.
The uncertain Riccati case produces 72 leaves from 143 cells and 556 whole attempts,
using 22,817 native prefix searches, 3,410,783 structural and 25,166,644,711 scalar
work units. Complete report construction/check/drop medians are 0.560740 and
0.558703 seconds. It retains 114,502,920 incremental owned heap bytes at return,
peaks at 114,504,336 and releases them on drop. Source reconstruction is measured
separately. This substantial retained evidence is a practical limitation. Cell,
depth, attempt and refinement limits do not establish whole-heap, CPU, wall-clock
or excluded scalar-refinement bounds. Zero epsilon and arbitrary fields/horizons
need not complete. Taylor models, wrapping, broader consumers, physical-clock cover
convenience, platform/HIL and release qualification remain open.

Evidence commit `7f7e64b278bbbde4687738282f88e83d01754874`, record
`docs/evidence/2026-11-13-interval-reachability`: 346 files, 85,174,134 bytes, SHA-256
`d3f5f0781211545075fb8b7f407dac85e0fad91b70613af052a12d103f4a9b99`.
The committed digest was regenerated and verified. A copied validation runner's
misdirected new log was moved out before rerunning the full downstream matrix;
all 5,480 historical record files were verified unchanged. Forty extraction paths
remain staged, with their three overlapping files incorporating accepted additions;
all thirty-seven other pending blobs and all eight Interface edits are preserved.
The isolated accepted worktree and firmware remain clean. No push occurred; this
is affected qualification, not the full pre-push matrix.

## 2026-11-13: Qualify Taylor candidate; withhold retention on pristine cost evidence

The full fifteen-phase goal remains active. Candidate commit
`8d07a1ff8b7f09ba0df3b97ff0fc37b862f4e859` is clean in
`/mnt/development/csgrs-taylor`, based on `e72c0d8b`. Its twenty source paths
implement demanded sparse normalized partials, exact-coefficient Taylor models
with certified finite remainders, and a concrete Hyperdynamics field consumer.
The candidate is not retained in main: its local installation was withdrawn after
the pristine performance repeat below. Main source and all forty staged extraction
paths were restored exactly; all eight Interface edits are unchanged.

Hypersolve owns the shared derivative graph, normalization, exact coefficients,
analytic-domain checks and multivariate integral remainder. Every next-order
contribution remains explicit. Original source outputs preserve guards, including
under zero derivatives. Models retain the original box, exact center and source;
subdomain use keeps the original global remainder and grid. The field adapter
preserves time/state bindings, original requests, native empty/unit carriers and
shared budgets. Its image can feed another field. Replay regenerates evidence
from source/request rather than trusting proof flags. This is not model arithmetic,
wrapping control, an ODE existence claim or a whole-work/heap resource bound.

Independent controls include binomial/mixed and high-factorial coefficients,
nonrational exact coefficients, polynomial identities, rational-function and sine
tails, original domain guards, stronger regularity rejection, empty/unit products,
precise final budgets, cancellation/retry and fresh-process replay. The logistic
field x(1-x) on [0,1] certifies [0,1/4], with coefficients [1/4,0,-1] and zero
remainder, versus the unchanged natural enclosure [0,1]. The reproduced empty
subdomain bug was fixed before field reuse. Authored 0^0 remains invalid under
Expr's original contract; the original failing input is preserved as a guard test.

All fifteen isolated and eleven combined downstream qualification groups pass:
1,438 Hypersolve tests plus 455 benchmark smoke cases; 53 Hyperdynamics tests;
37 selected minimal-feature tests; 29 release tests; 28 AddressSanitizer tests;
strict Clippy, warnings-denied docs, WASM, default-off configurations, example,
Rust 1.95, formatting and whitespace checks. Combined Hyperdynamics/Hyperpath tests
total 512 and Interface 201. All 194 path benchmark smoke cases pass. The 200
successive-operations rows preserve structural fields, allocator counts and
shown memory columns; these timings remain smoke observations. Its dependency
graph does not enable the changed expression-program module.

The full all-target run also reproduced an existing benchmark-report assertion
expecting six suites while the baseline manifest contains seven. The candidate
strengthens it to the seven exact names and the expression benchmark's feature/
Criterion requirements. That correction is still part of the isolated candidate.
Generated benchmark reports are archived separately from authored source.

A direct Jacobian baseline regression covers every derivative rule and consumes
results under STRICT and APPROXIMATE_512. Complete source/value serialization,
certainty receipts, work and dispatch/rational counts match in 52,947 bytes.
A fixed repository fixture asserts counts and fingerprints; complete transcripts
are archived. Existing field/evolution signatures match in 177,398 bytes. Legacy
consumer function unions/.text remain 2,100,408/2,121,815 and
2,187,002/2,209,703 bytes; both original ELF sizes decrease by 24 bytes. Text
hashes differ, so byte-identical code is not claimed. The earlier forced-inline
helper grew emitted functions by 3,066 bytes and was rejected; its evidence remains.

The first fixed twelve-pair cost run has all 22 workload confidence intervals
containing one, with identical nine resource records and 58 allocation rows.
Archive audit found that earlier `objcopy --dump-section` extraction rewrote its
input ELF container when no output ELF was supplied. A disposable-copy reproduction
matches the altered baseline hash exactly. Pristine baseline/final rebuilds match
all four original build hashes, and the accepted source hashes were restored.
The first observations are preserved alongside the correction.

A second fixed twelve-pair run using pristine binaries retains two adverse 95%
bootstrap intervals: step-limit/stopped ratio 1.0037487387, interval
[1.0012705217,1.0070406055]; halving-limit/stopped ratio 1.0033361909, interval
[1.0010526076,1.0060343072]. Allocation/resource records still match. These small
observations do not establish a universal slowdown, but they do not satisfy the
strict retention gate. No threshold, fixture or assertion was loosened. Main
retention was withdrawn; candidate code and both complete runs remain available
for code-layout/measurement investigation.

Two workspace-pinned completed-model trials agree on work/allocation records.
Construction medians are about 55 us logistic, 138 us mixed polynomial, 200 us
reciprocal and 844--850 us sine; subdomain reuse about 5.4, 31.9, 21.6 and 5.4 us.
Replay is measured separately. Incremental retained bytes over shared source are
5,272/10,360/9,400/5,952, with peaks 7,436/16,312/15,444/8,172 and zero measured
live delta after drop. Sixteen resource-failure controls and retry pass. These
are four absolute completed-workload measurements, not a general speedup,
completion envelope or whole-resource bound. The new workload binary is not
compared in size against consumers doing different work.

Evidence record `docs/evidence/2026-11-13-interval-taylor-candidate` at external
commit `102425928655792d063a3cfa8121dfcdec37a555` contains 314 files and
58,884,332 bytes, digest
`da15cf76824197c12746e9b0d6b87bc354d9e1059c44324533d4093a8a52500c`.
The committed digest was regenerated and verified. The original pending patch
remains `f5174309ce5958971b486b85c1020212c67c37049c600b8395ba3ff36bf1eecb`;
Interface remains `1dd2b902b84e07ffe6d63de0a0f007d026ef21f2391aeb598871b9de31395748`.
No push occurred. This is affected validation, not the full pre-push matrix.
This registration changes only the ledger and evidence manifest. Resolve the
stopped-path costs before retaining the candidate; wrapping, wider adoption,
competitive dynamics comparisons, the independent phase-11 extraction and all
other open phases remain in scope.

## 2026-11-13: Retain Taylor models with unchanged legacy execution artifacts

The full fifteen-phase goal remains active. Main now retains source
`5222328cf32f96ae77fb9d8226b441ceb2dae3a5`, including the initial Taylor candidate
and the qualification-history merge. The final correction restores the original
complete-column Jacobian traversal; only existing builder/helper visibility changes.
The demanded traversal composes by operand arity using those existing helpers
and Expr's derivative rules. Optional exports are appended after existing source.
No local calculus authority is duplicated, no predicate order is changed, and
legacy source-diagnostic locations remain intact.

Both legacy consumer binaries preserve every byte except twenty build-ID bytes
and ninety-one nonallocated symbol-string bytes. Code, constants, runtime data,
diagnostic locations, relocations, unwind tables, function addresses and sizes
are identical. ELF sizes remain 4,022,488 and 4,160,184 bytes; function unions and
text remain 2,100,408/2,121,815 and 2,187,002/2,209,703 bytes. Inspection reads
bytes without rewriting either binary. The full 177,398-byte semantic transcript
matches; maintained Jacobian and native evolution tests preserve exact outputs,
serialization, certainty, work and dispatch under both policies.

All fifteen final owner and eleven downstream qualification groups pass:
1,438 Hypersolve tests and 455 benchmark smoke cases; 53 Hyperdynamics tests;
37 minimal-feature tests; 29 release tests; 28 AddressSanitizer tests; strict
Clippy, warnings-denied owner docs, WASM, default-off configurations, example,
formatting and whitespace checks. Combined dynamics/path tests total 512 and
Interface 201; all 194 path smoke cases pass. The 200 successive-operations rows
preserve structural fields, allocation counts and shown memory columns. Their
timings are smoke observations. Generated owner reports remain in evidence,
separate from authored source.

All adverse measurements are retained. The intermediate two-accessor revision
restored text identity but changed nineteen diagnostic records. Its twelve-pair
short run retained an adverse halving-limit interval; a predeclared longer-batch
repeat retained adverse identity-replay and halving-limit intervals. Its identical
baseline control had no adverse intervals. These are different source revisions,
not erased or superseded measurements of the final implementation.

The final arity revision completes all twelve fixed paired runs and all 22 cases,
with identical resource/allocation rows. One pointwise 95% interval remains
adverse: unit/integrate ratio 1.0049825577, interval
[1.0000935935,1.0101950084]. Retention rests on the stronger deterministic proof
of unchanged execution artifacts, layout and work; it does not claim every timing
interval contains one or infer a speedup. No threshold, case or assertion changed.
The earlier stopped-path observations remain recorded above and in both archives.

Two final model trials preserve the earlier candidate's image, resource and
allocation records. Construction medians are about 54/136/197--198/828 us for
logistic/mixed/reciprocal/sine; reuse about 5.4/31.9/21.7/5.4 us. Replay is separate.
Retained memory and drop controls are unchanged. These are absolute completed-task
costs, not a cross-contract speedup, universal completion envelope or whole-resource
bound. Model arithmetic, wrapping, wider consumers and competitive comparisons
remain open, as do the previously documented scalar/payload/cancellation exclusions.

Evidence record `docs/evidence/2026-11-13-interval-taylor-cost-resolution` at
external commit `fe10a393c2aeac4088a928b29ff42926df19b10a` contains 333 files,
77,929,031 bytes, digest
`1ab5f27a5f959a1ba153b8f311a8d786bdeafd0221b79e292c9fac6f02bc9d73`.
Its final source/results are under `arity-qualification/`; top-level snapshots
describe the earlier accessor experiment. The prior 314-file Taylor archive
verifies unchanged. All forty pending extraction paths remain staged separately:
37 nonoverlapping blobs are identical and the three overlapping Hyperdynamics
files incorporate the retained Taylor additions. Interface's eight edits are
unchanged. Source budget passes at 4,144 files and 142,800,953 bytes before this
ledger registration. No push occurred; the complete pre-push matrix remains
required before publication. Phase 11 retention and all other open phases remain
in scope.

## 2026-11-13: Retain source-bound affine Taylor output forms

The full fifteen-phase goal remains active. Source
`dbe8d0fdfae14a7e0a311831cba293b9dec10fb6` adds opt-in `taylor-affine` support
owned by Hypersolve. Exact coefficients combine by monomial, and repeated output
maps compose against the original source and residual coordinates before interval
projection. The source certificate, domain, grid and global residual remain shared
and authoritative. This supports finite affine chains, rectangular/singular maps,
translations, nonrational coefficients, empty sources and zero-dimensional products.
The Hyperdynamics example derives x² and y from (x²+y,x²-y), then explicitly
projects to another field operation. It does not establish ODE coordinate
transport, nonlinear model arithmetic or completed wrapping support.

Ten maintained tests exercise independent polynomial coefficients, eight repeated
shear/inverse cycles, original residual identity, domain restriction, nonrational
cancellation, empty/unit composition, fresh-process replay and typed resource
failure/retry. The distinct-function residual control has equal residual bounds
but independently known difference 8/15 at 1/4, preventing invalid cancellation
from bound equality. A final audit reproduced an uncharged affine domain scan;
the failed witness and pre-fix source are retained. The corrected scan charges
before executing, and a dimension-sensitive test isolates transport from owner
work. Structural-stage and native arithmetic budgets retain the documented Real
payload/refinement, total live-entry/heap and within-scalar cancellation exclusions;
this increment does not claim a whole-resource bound.

The isolated-feature implementation before that final accounting line passed
17 owner and seven combined downstream command groups: 1,447 Hypersolve tests,
455 owner benchmark smokes, 53 Hyperdynamics tests, minimal configurations,
release/ASan, strict Clippy, warnings-denied docs, WASM, examples and format checks.
Combined dynamics/path ran 512 tests and 194 path benchmark smokes. All 200
successive-operations rows preserve structural fields, allocation counts and shown
memory columns relative to the preceding retained revision; timings are smoke
observations. After the production accounting fix, all twelve affected groups
pass: 25 focused tests, nine minimal affine and 21 original minimal Taylor tests,
53 Hyperdynamics tests, 19 release and 24 ASan tests, strict Clippy, docs, WASM,
the combined downstream consumer, formatting and whitespace. The last test-only
zero-output-chain extension subsequently passes all ten tests, strict Clippy and
formatting. Earlier broad runs are not advertised as final-code full reruns.

The first prototype passed its checks but grew an existing Taylor consumer by
10,848 ELF and 2,688 text bytes; it was not retained. The final separate feature
and nested module preserve all original bytes of the three existing implementation
files as prefixes. Both legacy evolution binaries are now byte-for-byte identical
at 4,022,952 and 4,160,560 bytes; the full 177,398-byte semantic transcript matches.
The existing Taylor consumer preserves machine code and every allocated section
except its build identifier. Its unstripped ELF grows eight bytes, from 3,919,488
to 3,919,496, in nonallocated symbol metadata. This measured additive extension
cost is explicit; this is not a size-neutral behavior-preserving refactor claim.

All twelve fixed alternating legacy timing pairs retain all 22 cases and identical
resource/allocation rows. One pointwise 95% interval remains adverse:
halving-limit/stopped ratio 1.0023400422, interval [1.0008895914,1.0041541563].
Retention uses the stronger deterministic evidence of identical complete legacy
binaries; no timing threshold, assertion or case was changed and no speedup is
claimed. Two old-model trials preserve image/work/allocation records. New affine
trials reproduce their records: coupled construction uses 145 graph/588,876 native
units and retains 2,632 incremental bytes (3,520 peak); reciprocal-copy uses
30/228,565 and 1,360 bytes (2,160 peak). Construction medians are about 52.7/13.4 us,
reuse 45.7/9.7 us, eight round trips 964/246 us, and full replay 191/83 us. All
measured operations and failure controls return live allocation delta to zero
after drop. Construction starts from an existing shared source; replay includes
its reconstruction. These are absolute completed-task costs, not competitive
results or universal completion bounds.

Evidence record `docs/evidence/2026-11-13-interval-taylor-affine` at external commit
`910978f28d0f9fa9f0b4ecae6e1354c785272224` contains 259 files, 4,644,737 bytes,
digest `048b328a2f3d95257149cfa0329cfce39466109adcce26f51ddec443447ff092`.
Final source/results are under `isolated-feature/`; its README distinguishes
superseded prototypes, the final accounting fix and later test-only checks. Both
previous immutable Taylor archives verify unchanged. All forty pending extraction
paths remain staged separately, including 37 identical nonoverlapping blobs and
three merged Hyperdynamics files. Interface's eight edits are unchanged. Source
budget passes at 4,147 files and 142,849,433 bytes before this registration. No
push occurred; the complete pre-push matrix remains required before publication.
Phase 11 retention, ODE wrapping/QR, nonlinear composition, wider adoption,
competitive comparisons and all other open plan phases remain in scope.

## 2026-11-13: Retain fixed exact state charts and Picard transport

The full fifteen-phase goal remains active. Source
`de56e383c3754f77896e9cde939b5207b784ab91` adds opt-in coordinate programs
owned by Hypersolve: prove B*A=I under STRICT, retain x=c+A*y, and construct
B*f(t,c+A*y) as a shared DAG. Source operations, domain guards, binding order,
node correspondence and inverse receipts remain available. Hyperdynamics assigns
state/time meaning and reuses the existing derivative, field and Picard owners.
A completed step transports its initial set, tube and endpoint through the native
Hyperlattice affine enclosure. A final transport failure preserves the native
report but cannot publish whole-step success. The exact original initial set is
c+A*Y; existence is not asserted for every point of its Cartesian envelope.

Fourteen maintained tests cover independent polynomial/Jacobian expectations,
nontrivial bindings, exact and undecided inverse outcomes, algebraic coefficients,
poles, eight shear/inverse round trips, empty/unit distinctions, fresh-process
replay, typed error source chains and resource/cancellation/retry behavior. The
concrete completion witness x1'=128*x2, x2'=0 rejects an original half-unit step
for contraction, but A=diag(128,1) certifies it with source endpoint
[-64,64] x [-1,1]. This establishes fixed-chart one-step support, not adaptive
QR, a complete wrapping portfolio or a whole-horizon coordinate integrator.
An initially uncharged native-carrier validation scan was reproduced and fixed;
dimensions 0, 1 and 3 now isolate its accounting. Structural work and native
arithmetic remain separate; exact Real payload/refinement, predicate internals,
total heap and intra-scalar cancellation exclusions remain explicit.

Fifteen owner command groups pass on final production code: 1,455 Hypersolve
tests and 455 benchmark smokes, 60 Hyperdynamics tests, minimal/default-off
configurations, strict Clippy, warnings-denied docs, WASM, example, release,
ASan and format checks. Combined extraction qualification passes seven groups,
including 519 dynamics/path tests and 194 path benchmark smokes. All 200
successive-operations rows preserve structural fields, allocation counts and
shown memory relative to the preceding affine stage; their timings are smoke
observations. Subsequent assertion-only strengthening passes the seven affected
tests in isolated and combined worktrees, strict Clippy and formatting. The
final source snapshot distinguishes these checks and the later fuzz package
from the initial thirteen-path broad qualification.

The unchanged full Hypersolve coverage script passes its 85% gate at 87.52%
(53,295/60,897 physical executable lines). Its existing regex includes some
Hyperinterval files; a separately retained owner-only analysis using the same
trailing-inline-test convention gives 89.47% (51,639/57,715), with 93.84% in
the new module. Dedicated cfg(test) source files/helpers may remain in that
convention. Hyperdynamics source-prefix coverage is 90.95% (1,999/2,198 lines),
with 160/180 lines in its new module; no crate-specific threshold is configured.
Neither measurement establishes branch coverage. The ASan coordinate fuzz target
uses independent polynomial/Jacobian/trajectory expectations, inverse rejection,
graph cutoffs and replay. It completed 6,060 executions in 61 seconds with no
findings, stopping at its time cap before the 10,000 ceiling. Short inputs are
skipped, so executions are not a valid-chart count. All 220 retained corpus files
also replay successfully in the combined worktree. Locked/offline fuzz builds,
strict Clippy and formatting pass.

Same-path baseline/final builds preserve both legacy binaries byte-for-byte at
4,023,080 and 4,160,888 bytes and the complete 177,398-byte semantic transcript.
Existing model/affine consumers preserve machine code and loaded sections except
build identifiers; the model ELF shrinks eight nonallocated metadata bytes and
the affine ELF size is unchanged. Twelve fixed alternating legacy timing pairs
retain all 22 cases and equal resource/allocation rows. One pointwise 95%
bootstrap interval remains adverse: exponential/integrate ratio 1.0103964844,
interval [1.0006803585,1.0280168222]. It remains in the evidence; deterministic
identical binaries support execution preservation, not a speedup claim or a
claim that every timing interval contains one.

Two coordinate cost trials reproduce records. Scaled-shear construction uses
700 graph units and retains 8,832 incremental bytes (10,016 peak); its completed
step/transport uses 380 graph and 513,467 native units. Translated-constant
construction uses 455 graph units and retains 7,152 bytes (8,328 peak), with
299/476,317 step units. Construction medians are about 24.0/7.5 us,
step/transport 34.1/34.6 us, and full replay 80.7/60.1 us. Construction starts
from an existing field; replay includes source/chart reconstruction and output
transport. Measured operations and failure controls return live allocation delta
to zero after drop. These are absolute completed-task costs, not competitive
benchmarks or uniform completion bounds.

Evidence record `docs/evidence/2026-11-13-interval-coordinates` at external commit
`b48e9742d6d35d4b52b2a47a2db2481da537a744` contains 670 files, 10,064,155 bytes,
digest `9df7d2bbf4830ac4ce5dd5876260d13aa6e5de69fb969c40483064d70a0259fd`.
All three preceding immutable Taylor stages verify unchanged. The original forty
pending extraction paths remain separate, with 37 identical nonoverlapping blobs
and three merged package files. A reproduced combined locked-build failure adds
one pending path: the new fuzz lock needs the extraction's direct Hyperlimit
edge, with no version changes. Thus 41 paths remain staged, without unstaged
changes. Interface's eight edits and clean firmware are preserved. Source budget
passes at 4,156 files and 142,931,405 bytes before this ledger registration.
No push occurred; the full pre-push matrix remains required. Phase 11 retention,
adaptive wrapping/QR, nonlinear composition, wider adoption, whole-resource
closure, HIL and coordinated release remain open.

## 2026-11-13: Reject broad and boundary-only dynamics inline experiments

The previous turn retained exact state charts. This turn returns to the open
phase-11 retention gate at `65a832b8cf108894fe0913a77f4253407f48c332` and the
exact 41-path pending extraction. Main and Interface production sources remain
unchanged. Two isolated experiments preserve method bodies, predicate order,
error identities and report fields: twenty ordinary inline attributes on trait/
error methods and compatibility wrappers, then only the five boundary methods.
The source audit also verifies that the original owner benchmark differs only
by the pending extraction's two deprecation allowances.

Same-path Rust 1.95 builds reproduce baseline/extraction ELF sizes 13,312,600 /
13,304,584 bytes and defined-function union sizes 8,044,187 / 8,036,358 bytes.
The broad variant grows to 13,319,296 ELF and 8,059,498 function-union bytes,
failing admission. Callback body expansion is visible in the symbol diagnostic;
no full timing or functional qualification is claimed for this rejected variant.
The narrow variant is 13,304,608 ELF and 8,036,427 function-union bytes, below
the retained baseline. All 194 complete Debug result observations and warmed
result/drop allocation rows match for the original extraction and both variants,
with zero live-byte delta. These are differential observations, not independent
geometry or canonical encoding proof. Dependency versions remain pinned.

Six complete cost sweeps run in balanced order baseline/extraction/narrow/narrow/
extraction/baseline, each with all 194 cases, one executable path/argv, CPU 0,
scoped warm-up and ASLR disabled only for the subprocess. There are 65 cases
for the original extraction and 109 for the narrow variant whose lower pointwise
mean-confidence bounds in both runs exceed the upper bounds of both baseline
runs. All 1,164 measured case executions and raw estimates remain. This is an
observed per-case gate failure, not a family-wise statistical claim. The narrow
variant improves parser-256 from the original extraction's ratio 1.06169 to
0.94960, but has adverse separated ranges for skew (1.04129), rectangular-pad
clearance (1.05064) and rational-quadratic evaluation (1.06147). No favorable
aggregate, case removal, threshold change or selective repeat clears the failure.
Host clock/governor and other activity are not controlled.

After timing, 21 Callgrind runs measure seven cases across baseline, extraction
and narrow variants, each with 32 warmed complete-result/drop callbacks. Parser,
importer, skew and rectangular-pad instruction counts match exactly. Narrow
inlining lowers lookahead and mixed-jerk counts relative to the original
extraction, but remains above baseline: 2,507,304 versus 2,503,560 and 2,296,200
versus 2,293,160 respectively. Symmetric jerk remains 2,053,160 versus 2,050,344.
Removing the separate 256-byte metric-length adapter therefore does not remove
all motion overhead. These are instruction-reference observations, not hardware
cycles or an explanation of unchanged-instruction wall-time differences.

Both experiments are rejected and reverted. The isolated worktree again matches
the original 41-path patch exactly. After restoration, three routing tests and
one axis-migration test pass, including fresh-process report/dispatch comparisons;
format and whitespace checks pass. Six separately captured legacy/direct/compat
transcripts under STRICT and APPROXIMATE_512 each contain 92 matching report/trace
lines within policy. No count assertion changed. This focused restored-candidate
validation does not claim a new full downstream/WASM/sanitizer/pre-push matrix.

Evidence commit `6d1a1a33c32a537edf7803e749f4246920d5597a`, record
`docs/evidence/2026-11-13-interval-dynamics-adapters`, contains 4,809 files,
59,342,303 bytes, digest
`5fe72cd01add9d288cc4b55ee5365b60d8c6f38628a758d66e9d48c75fe263d0`.
The preceding coordinate and extraction-requalification stages verify unchanged.
All 41 main staged paths, Interface's eight edits and clean firmware are preserved.
No push occurred. Blanket wrapper inlining and boundary-only hints are now
measured rejected approaches; ABI/return and code-generation retention work
remains unresolved. The complete fifteen-phase goal and every other open phase
remain active.

## 2026-11-13: Reject machine-profile uncertainty ingress variants

Phase 8 was investigated on CSGRS `3e4382664d50a7734d3fbef33b0a5da0c5ac22ef`
and Interface `ff668ca39d5e1ecee91b438b20eca806f3a26f29`, excluding the pending
extraction. Hyperreal owns fraction reduction; Hyperinterval owns closed
nominal-plus/minus sets; Interface retains units, nominal values, configuration
identity and validation. Missing shared-ingress witnesses were reproduced before
implementation. Existing signed and arbitrary-rational domains, predicate order
and typed failures were preserved on admitted configuration inputs. Rational
arithmetic remains unbudgeted; no resource-completeness claim is added.

Three experiments are rejected, with source and unfavorable evidence retained:

| Variant | Decisive size result | Disposition |
| --- | --- | --- |
| Signed fraction uncertainty ingress | Profile ELF 805,904 → 806,576 bytes | rejected |
| Typed `u64`/`NonZeroU64` scalar and interval ingress | `borrowed_ops` 6,463,088 → 6,463,232; `successive_operations` 21,930,560 → 21,930,696 bytes | rejected |
| Inline only the new typed scalar wrapper | `borrowed_ops` 6,463,088 → 6,463,176 bytes | rejected before broader qualification |

The typed variant passes 7,497 independently derived endpoint cases, full-width
scalar boundaries, a 1,200-digit nominal, consumed outputs, typed invalid-input
checks and fresh-process cold/warm trace comparisons under both policies.
Its full Interface suite passes 201 unit tests, one integration test and one
doctest; the migration trace tests run separately. Affected all-feature owner
and scalar-oracle tests, release, ASan, strict Clippy and warning-denying docs
pass. Existing measurement conformance passes all four tests and eight full
value cases. Failed harness attempts remain in the record and are not counted
as acceptance; no production or fixture assertion was weakened.

Seven profile scenarios include six successful profiles and one expected typed
invalid profile. Complete native/WASM fields and full identities match. Exact
raw allocation totals match. Same-toolchain Rust 1.95 raw-profile ELF shrinks
806,000 → 805,792 bytes, emitted function union 438,574 → 438,415, and WASM
1,064,908 → 1,064,857. WASM rejects invalid indices and consumes no host calls.
Two focused native timing campaigns each run six ABBA blocks, 31 samples of 64
complete outcomes, one executable path, CPU 0 and subprocess-only ASLR disable.
Their per-case bootstrap intervals are below one; raw-harness mean ratios span
0.94031–0.95563. Host clocks and unrelated activity are not controlled. Neither
these observations nor the profile size reductions waive the broader failures.
Scalar/successive-operation timing campaigns were not run after their size gates
failed. This is not complete benchmark, coverage, browser or pre-push acceptance.

The existing measurement binaries retain their sizes and byte-identical text;
changed build IDs and five values matching the twelve-line source shift explain
the observed section differences. All 160 completion/allocation rows match;
concurrent-build diagnostic times are not performance evidence. The typed scalar
benchmark has byte-identical text but a twenty-byte larger exception table.
Equal code length does not establish equal binary size.

All three isolated worktrees are restored cleanly. After restoration, four
rational/replay tests, 199 Interface unit tests, its integration/doctest and
workspace formatting pass. Main's 41 staged extraction paths retain patch digest
`3bebcf6370ec1ce7b72691a9699a01e3295b7a32063c4da0a52b0996b4686d48`;
Interface's eight pending paths retain
`1dd2b902b84e07ffe6d63de0a0f007d026ef21f2391aeb598871b9de31395748`.
Firmware is unchanged. No source from these experiments is retained or pushed.

Evidence commit `f1d491d7627331dcb4b3a46ef1568e92049fd60c`, record
`docs/evidence/2026-11-13-interval-machine-profile`, contains 288 files,
10,811,621 bytes, digest
`fb6ea79fcfc463911aef5f45e53e8d7e0cd0d897602a74589b242b967e7ddf0b`.
Compiler binaries and Python caches are excluded; source, locks, build attribution,
semantic transcripts, samples and section diagnostics remain. The full goal is
active. Machine-profile delegation, positive arithmetic, integer scaling, named
budgets and the other phase gates remain open; documentation is not completion.

## 2026-11-13: Diagnose reducer linkage and reject interval-only ingress variants

Phase 8 qualification continued on the same isolated CSGRS `3e4382664d` and
Interface `ff668ca39` source, excluding the pending extraction. Main `f5ba9ce9b`
differs only in ledger/index metadata. Missing API witnesses, input domains,
ownership and acceptance gates were recorded before edits. No production
source from this investigation is retained.

The freshly rebuilt `borrowed_ops` baseline exactly matches its prior hash.
A new typed scalar wrapper in a child module grows that artifact by 120 bytes;
same-module inclusion grows it by 128; delegation through public scalar ingress
grows it by 40. Disassembly and relocations establish that the child-module
variant changes an existing direct reducer call into an indirect GOT call.
Existing function bodies consequently grow despite unchanged source locations.
These are measured failures, not a rationale to waive the artifact gate.

Keeping Hyperreal unchanged avoids that scalar modification. Four interval-only
variants were evaluated against the 806,000-byte native profile baseline:

| Variant | Native profile bytes | Disposition |
| --- | ---: | --- |
| Infallible entry with formatted invariant failure | 806,440 | rejected |
| Inline only the new interval entry | 806,408 | rejected |
| Static invariant panic without scalar diagnostic formatting | 806,136 | rejected |
| Propagate the existing scalar `Problem` result | 805,904 | rejected: WASM grows 1,064,908 → 1,064,948 bytes |

The final scalar-result variant preserves valid input completion and original
application error causes. It passes three new owner tests plus four existing
rational tests in debug, release and ASan, affected-owner strict Clippy and
warning-denying rustdoc. Independent endpoint checks cover 7,497 small cases,
35 full-width boundary pairs and a 1,200-digit nominal, including subsequent
operations. Three Interface migration tests pass, including fresh-process
cold/warm dispatch checks under both policies. The full Interface run passes
201 unit tests, one integration test and one doctest. No test or policy was
weakened. Arbitrary rational arithmetic remains unbudgeted.

Complete native profile values, identities and raw allocation totals match.
The native defined-function union shrinks 438,574 → 438,468 bytes. The attributed
`successive_operations` artifact stays at 21,930,560 bytes with byte-identical
text. WASM replay preserves all corresponding transcripts, rejects invalid
indices and makes zero host calls, but its binary-size gate fails. Wasmtime
fuel rises by 132 for each of six successful profiles and 30 for the expected
typed invalid profile. Fuel is not a browser latency measurement. No final
scalar, measurement or successive-operation runtime campaign was executed after
this failure; full benchmark, coverage, MSRV, browser, release and pre-push
acceptance remain unclaimed.

All three isolated worktrees are restored cleanly. Restored rational tests,
199 Interface unit tests, its integration/doctest and both workspace formatting
checks pass. Main's 41 staged paths and Interface's eight edits retain their
previous exact patch digests. Firmware is unchanged; no push occurred.

Evidence commit `d0402e24fe173be606a4e8744c1131b6d2a0f986`, record
`docs/evidence/2026-11-13-interval-machine-profile-module`, contains 155 files,
7,560,490 bytes, digest
`2479b56c77710a018e91923799c8bd260b28cc7bd7c21e4e7d19e95e6ac0a43f`.
Compiler artifacts remain outside the record with hashes; raw disassembly and
strings retain their emitted whitespace. Scoped archive verification passes;
all seven preceding frozen stages verify unchanged. The fifteen-phase goal
and the unresolved machine-profile migration remain open.

## 2026-11-13: Retain source-bound fixed-size Jacobian matrix projections

Source `8b63fa68d0b952fdab026b4b11c609ff4a0fa319` extends Hypersolve through
opt-in `interval-lattice`. A reproduced E0599 witness established the missing
bridge at `40d337953`. `RegularJacobianEnclosure::matrix::<R,C>` returns a native
Hyperlattice matrix while retaining a borrow of the original source, domain,
selected variables and precision. It preserves exact bounds/grid and selected
column order, distinguishes empty domains from zero-dimensional unit products,
and shares graph/native copy budgets. Shape errors precede empty handling;
exhaustion/cancellation publishes no partial result and preserves retry.
Numerical matrix products/norms do not become new regularity certificates.
No derivative or matrix kernel is duplicated and no existing consumer is migrated.

Seven maintained tests exercise independent rectangular polynomial derivatives,
binding/column permutations, products/norms, shape/resource failures, source
replay and zero new scalar dispatch/reducer calls. Nonselected parameter cases
retain t in [0,4] for t^2*x and sqrt(t)*x while differentiating only in x, including
the distinction between continuity in t and C1 regularity in x. Graph work and
coefficient limits are explicit; total stack/heap and intra-copy cancellation
remain outside these counters. This adds a completed projection capability,
not whole-resource closure or an inverse/ODE algorithm.

Corrected source passes 1,461 owner tests and 455 benchmark smokes, strict
all-target/all-feature Clippy, warning-denying docs, WASM compilation, 60
isolated downstream tests and formatting. The final additional parameter test
passes seven release tests and six ASan tests (trace instrumentation excluded).
The unchanged full coverage script passes its 85% gate at 53,497/62,018 lines
(86.26%); that denominator also includes interval dependencies. The owner-prefix
subtotal is 51,700/57,777 (89.48%), and the new module is 61/61 under the script's
trailing-inline-test convention. These are not branch-coverage claims. The last
coverage configuration compiles the final seven-test corpus. The new ASan fuzz
target completes 10,000 runs in six seconds, seed 20261113, max input 16 bytes;
fuzz checks, strict Clippy and formatting pass. This is bounded smoke, not a
completed lexicase campaign.

The initial proposed combined shape diagnostic grows `ExpressionError` from
24 to 40 bytes and is rejected. Separate row/column diagnostics retain 24 bytes
without boxing or lost causes. Twelve completed cost cases compare projection,
retained evidence, native norm and output/drop against a handwritten control.
All exact values, work, allocations and peak/live memory match. For 1x1, 3x2 and
8x8 cases, sixteen calls allocate 96, 864 and 8,608 times; peak bytes range
48–1,536, 176–5,632 and 1,104–35,328 with coefficient size. Empty/unit cases
allocate nothing. Source construction is outside this measurement.

The enabled shared probe costs +288 stripped ELF bytes, +176 text bytes and
+181 sized-function-union bytes. Both six-block ABBA timing campaigns retain
unfavorable results. With CPU warmup, ordinary nonempty median differences are
about -0.7% to +2.9%; the zero-row unit case costs 59 versus 55 ns. A 1x1 large-
coefficient outlier remains adverse (mean ratio 1.0499, pointwise 95% bootstrap
interval 1.0017–1.1416). Short samples and uncontrolled host/frequency behavior
limit these comparisons. No speedup or unchanged-runtime claim is made.
Retention is for the new opt-in completeness extension, not a performance
refactor or permission to relax the rejected migration gates.

The existing feature-disabled expression benchmark preserves all machine code
and its 5,409,376-byte stripped size. Unstripped size grows eight bytes from
symbol metadata/alignment; eighteen diagnostic source locations shift by thirteen
lines. Both versions pass twelve smoke workloads. ELF bytes are not identical.
Combined qualification with the exact pending extraction passes 25 owner, 71
dynamics and 448 path tests plus strict Clippy. Main then passes eight focused
tests and formatting. The initial integration command's nonexistent test-target
error is preserved separately; the corrected command did not weaken any test.

Evidence record `docs/evidence/2026-11-13-interval-jacobian-matrix`, local archive
commit `7b7aac30903ba3a4a6e40c051cfa239f3823f74b`, contains 596 files,
22,848,827 bytes, digest
`aff2dd277d79231c31895ddea643ce2810fc8c30609d6b11755713e56206e969`.
It retains the rejected layout, final source, raw costs, full coverage and all
command outcomes. Scoped archive verification and all eight preceding frozen
stage digests pass. Four isolated worktrees are restored clean after verifying
their intended source. Main's 41 staged extraction paths and Interface's eight
edits retain their previous exact patch digests; firmware is unchanged. Source
budget passes at 4,159 files and 142,973,711 bytes before this ledger addition.
No push occurred. No phase is declared complete; field adoption, remaining
migration gates, competitive workloads, HIL and coordinated release remain open.

## 2026-11-13: Reject forced return-boundary inline and outline variants

The previous turn retained source-bound Jacobian projections. Phase-11 retention
work now reproduces the pristine and exact pending-extraction owner artifacts at
`cc056c7c36101d8ff145907563ef21b0335687af`. Both SHA-256 hashes match the prior
adapter record exactly. The isolated worktree excludes main's pending changes
until the recorded 41-path patch is explicitly applied. No production source
from the new experiments is retained.

Direct deprecated reexports were considered to remove compatibility bodies.
A Rust 1.95 compiler probe shows that `#[deprecated]` on the reexport does not
warn at a legacy call, even with `-D deprecated`; a real forwarding function
produces the expected diagnostic. Reexports would drop the required migration
notice, so that alternative is rejected before touching production entry points.

The shared native error alias is reproduced as a control. Two new variants keep
real deprecated functions: force inline only their fifteen forwarding bodies,
then additionally outline the twelve native scheduling entry points they call.
Equation constructors, metric/join methods and predicates remain untouched.
All method bodies, argument order and arithmetic remain unchanged.

| Variant | Owner ELF bytes | Complete sized-function union |
| --- | ---: | ---: |
| Pristine baseline | 13,312,600 | 8,044,187 |
| Original extraction | 13,304,584 | 8,036,358 |
| Shared error, identity conversions removed | 13,316,848 | 8,043,152 |
| Shared error and forced forwarding inlines | 13,319,192 | 8,059,155 |
| Same with native entry points outlined | 13,350,472 | 8,069,963 |

Both new variants fail the unchanged binary and code-size gates. Fewer function
ranges in the forced-inline variant do not imply less emitted code. Every artifact
passes 194 benchmark smoke workloads; those are not independent full-result,
allocation or runtime comparisons. No timing campaign or full downstream/sanitizer
qualification is claimed for a variant already rejected by size. No compiler,
linker, stripping or acceptance policy changed.

After verifying the final experimental patch, the exact original extraction is
restored. Its three routing tests and axis migration test pass, including the
fresh-process report/dispatch assertions. Six additional legacy/direct/compat
transcripts each contain 92 matching report/trace lines within STRICT and
APPROXIMATE_512; their hashes also equal the previous restored-candidate record.
Formatting and whitespace checks pass. The isolated worktree is then restored
clean at its baseline. Main's 41 staged paths, Interface's eight edits and clean
firmware retain their exact preceding states.

Evidence record `docs/evidence/2026-11-13-interval-dynamics-return` at local commit
`4ac36e669e27e462b4c21ee5bc23024ad1857014` contains 86 files and 50,707,255 bytes,
digest `c58ef69a7fade56b9e1b97d85871335d27831b2a89394d0c52c371d1ddeb2188`.
Scoped archive verification and all nine prior immutable stage digests pass.
Forced forwarding and native outlining now join the rejected ordinary-inline
approaches. Further retention work needs a distinct error/result ownership design
or demonstrated code-generation cause; favorable aggregates or unrelated parser
changes cannot resolve these gates. No push or phase-completion claim is made.
The complete fifteen-phase objective remains active.

## 2026-11-13: Provider-owned failures clear size and copy costs, fail owner timing

Previous turn rejected forced forwarding and native outlining. This experiment
starts at `8ea26dac524dacdaaab6c255a54c355f8cb9de2f`, with the same pending
41-path extraction, in an isolated tree. Hyperdynamics' unpublished metric/path/
join protocols gain associated errors that preserve native failure distinctions
and provider causes. Hyperpath chooses its unchanged legacy error enum; thirteen
whole-report identity conversions disappear. Actual deprecated functions remain.
No inline attributes, mathematical expressions, validation/predicate order,
compiler profile, dependency versions or acceptance thresholds change.

The missing provider-error test first fails against the original protocols.
Two new tests then pass under both policies: non-Clone metric/join causes retain
identity and error chains; wrapped native failures preserve exact retry selection,
one component halving, zero-budget exhaustion, receipts and subsequent replay.
Focused native, routing and axis controls plus strict all-target/all-feature owner
Clippy pass. The complete owner log records 521 passing tests and no failures;
its outer process exit status was not recovered after context compaction, so the
record distinguishes observed complete test output from a captured exit code.

The real owner ELF shrinks from 13,312,600 to 13,302,416 bytes; the complete
sized-function union shrinks from 8,044,187 to 8,035,134. All 194 full result and
allocation rows match in the separately attributed observer. Seven Callgrind
cases across three variants preserve four control instruction totals, remove
the original extraction's extra jerk report-copy instructions, and retain a
130-instruction-per-call increase in corner lookahead. These observer results
are not hardware-cycle measurements, canonical serialization proof or owner
wall-clock timings.

Six matched real-owner sweeps complete all 194 cases in balanced order, with
CPU affinity, per-process ASLR control and fixed executable path. All 67 candidate
cases flagged by separated mean-confidence intervals receive four longer
candidate/baseline/baseline/candidate repeats. Sixty-one retain adverse separation;
all six nonconfirmations remain in the record. Longer mean ratios include 1.0933
for line offsets, 1.0890 for quadratic hodographs, 1.0672 for rectangular-pad
clearance, 1.0517 for additive-bead scheduling, 1.0416 for exact trace import and
1.0219 for the 256-record parser. These are repeated per-case observations, not
a family-wise confidence claim. Favorable jerk timings and lower code size do
not clear the unchanged owner gate. The candidate is rejected.

After the final candidate edit, formatting and six fresh report/dispatch
transcripts pass. The original extraction is restored and its three routing
tests, axis migration test, formatting and six additional transcripts pass.
Each transcript has 92 report/trace lines; hashes match within both policies
across legacy/direct/compatibility modes, candidate, restoration and prior record.
The isolated tree is restored clean. Main's original 41 staged paths, Interface's
eight edits and clean firmware retain their exact starting states. No new full
downstream, successive_operations, sanitizer, fuzz, coverage, WASM, packaging or
pre-push qualification is claimed for this rejected design. No push ran.

Evidence record `docs/evidence/2026-11-13-interval-dynamics-provider-errors` at
local commit `b7c58335ea0ce9f0fad2c7ab726e174d0ae696c3` contains 5,916 files and
67,111,954 bytes, digest
`dbe1f8aebf2cf3b8e86e2d443a043be64c571b8025401557c787136b00433787`.
Scoped archive verification and all ten prior immutable-stage checks pass.
The design and its acceptance witness are archived for future qualified work;
they are not retained production support. Phase 11 and the full fifteen-phase
objective remain active, with all preceding obligations unchanged.

## 2026-11-13: Retain source-bound nonlinear program composition

Previous turn: progress, rejecting the provider-error dynamics experiment after
reproduced timing regressions. Source commit
`62b8676bcb41652d558a75743dccd0e980fe78a6` adds Hypersolve
`ExpressionProgram::compose`: independently scoped outer bindings select inner
outputs, preserving shared nodes, exact definitions and original operations.
The executable returns every inner output before the composed outputs, so
ignored inputs or an empty outer result cannot discard inner domain failures.
Unreached source nodes remain unevaluated. Original sources, selections and node
maps support diagnostics and fresh-process replay. Existing arithmetic kernels,
predicate order, dependencies and compiler profiles are unchanged.

Repeated composition, Jacobians and native Taylor projection consume the result.
An independent logistic example certifies [0,1/4] with zero remainder where
independent interval evaluation yields [0,1]. Construction uses a shared graph
work/entry budget with typed shape, resource and cancellation failures; the
existing scalar/payload/heap exclusions remain explicit. Taylor regularity still
applies to all retained outputs. This is source composition followed by native
proof construction; arithmetic on existing nonlinear remainder models, direct
field output selection, adaptive wrapping/QR and wider adoption remain open.

All final affected commands pass: 1,469 Hypersolve and 60 Hyperdynamics tests,
strict all-target/all-feature Clippy, strict docs, WASM, feature-minimal and
feature-disabled checks, and workspace formatting. Eight new acceptance tests
cover exact chain-rule values, guards, sharing, empty/unit domains, budgets,
cancellation/retry, zero construction dispatch, fresh-process replay and Taylor
projection. Main repeats pass those eight tests and 12 downstream field tests.
ASan/leak checking passes all eight; the standard-library sysroot is not
instrumented. Memcheck could not initialize because loader debug symbols needed
for strcmp redirection were absent. Original development compile failures and
the corrected retries are preserved; no assertion was weakened.

The actual 12-case owner benchmark completed baseline/candidate/candidate/
baseline measurements at a fixed path/CPU with ASLR disabled. No case had
separated slower candidate confidence intervals; degree-32 evaluation retains
an adverse mean point ratio 1.00667835 with overlapping intervals. Initial
method/module prototypes added 136 bytes and a constructor probe added 24;
they were rejected. The retained ordinary-inline method matches the baseline
6,836,472-byte executable, all section sizes and 3,167,734-byte function-address
union. Machine-code bytes differ; byte equivalence is not claimed. A separately
attributed allocation observer matches all 16 legacy result/allocation rows.
New graphs grow linearly in the measured corpus; maximum construction temporary
peak is 47,452 bytes, with zero live-byte delta after all measured successes and
late exhaustion. These are local completed-workload observations, not universal
performance or memory guarantees.

Evidence commit `709ed26a99a8926b9d7f3f522f5b08cd5bdfc7bc`, record
`docs/evidence/2026-11-13-interval-program-composition`: 309 files,
19,057,276 bytes, SHA-256
`c06f85cc6e98dfbe75aa6faa9941bac7476585c050e7bb242f358d669bfeaeb3`.
Scoped archive verification and all eleven preceding immutable-stage checks
pass. The main 41-path extraction and Interface eight-path pending patches are
byte-identical; firmware remains clean and the isolated composition tree is
clean at the retained source. Default CSGRS/Hyperpath graphs do not activate
the changed optional module. No new root successive_operations, complete
browser/firmware, release, full coverage/fuzz or pre-push qualification is
claimed. No push occurred. Phase 13 and the full fifteen-phase goal remain active.

## 2026-11-13: Adopt native whole-cell enclosures in Hypersdf

Previous turn: progress, retaining source-bound program composition. Source
`61e9c55642c6b538a3e7bab87ef41d59d2274b25` adds optional `enclosure-program`
to Hypersdf. `SdfProgram` retains the original field and translates its smooth
scalar expressions and translations into Hypersolve's existing evaluator and
Taylor proofs. No arithmetic kernel or default query body is replaced. Whole-cell
sign reports retain the original distance qualification; zero-containing ranges
remain unresolved, and empty cells differ from identically zero fields.
Primitive, piecewise and affine lowering remain explicit unsupported gaps on
this new route, with their existing APIs retained.

The missing API and old sine/cosine cell-enclosure gap were reproduced before
retention. The new route certifies sin(x)+cos(0)>0 on x in [0,1]. For
x(1-x)-3/8, the natural range [-3/8,5/8] tightens to [-3/8,-1/8] with an exact
quadratic model and zero remainder, proving the whole cell inside. Independent
integer expectations, original domain guards, source composition, native model
reuse and fresh-process reconstruction of an authored recipe are exercised.
This does not add a generic SDF serialization format or promote a range proof
to surface topology or Euclidean distance.

Final resource review reproduced an unnecessary constant-program rejection at
nine graph entries: every task reserved the largest linear expansion. Follow-up
`09f540dc8340618f587ca4c7d6282eb894d3b631` reserves each operation's actual
additional live records and charges initial binding/task construction. The
nine-entry regression and all prior exact-budget, exhaustion and retry checks
pass. Existing scalar refinement/payload, source-ingress and heap-byte exclusions
remain explicit. No resource or failure assertion was weakened.

Final qualification passes 124 owner tests plus the separate replay child,
114 default-feature tests, strict all-target/all-feature Clippy, strict docs,
WASM, formatting and diff checks. ASan/leak checking passes all nine new tests;
its standard-library sysroot is not instrumented. Main-worktree acceptance,
formatting and indexed source-budget checks also pass. All 46 default owner
benchmark smoke cases pass. Baseline/final default executables are
8,102,600/8,102,576 bytes, with identical loaded sections except the build-ID
note and identical 4,467,244-byte function-address unions. An artifact-inspection
rewrite was detected and corrected by rebuilding the baseline to its recorded
hash and restoring the identical raw final candidate; the full provenance is
retained. No new default runtime timing claim is inferred from noisy samples.

A separately attributed warmed allocation/timing harness completes polynomial
and trigonometric compilation, natural images, Taylor images, subdomain reuse
and late exhaustion. Final polynomial compile/natural/Taylor/reuse medians are
1.50/3.55/62.10/5.77 microseconds; trig medians are
1.09/90.45/701.39/16.52 microseconds. The largest measured temporary peak is
11,368 bytes; every success/drop and late-exhaustion epoch ends with zero signed
live-byte delta. Initial prototype samples, adverse differences and failed
attempts remain archived. These are local instrumented completion observations,
not an old/new speedup, universal bound or competitive validated-dynamics claim.

Evidence commit `377c832533c261dc51720e07f814ffae318bc4e0`, record
`docs/evidence/2026-11-13-interval-sdf-enclosure`: 126 files, 8,427,727 bytes,
SHA-256 `8a25e59d989f352cdcf8deb6499b1c31697fb9a37cb2ebcf043981d6a605c0c6`.
Scoped archive verification and all twelve preceding immutable-stage checks pass.
The 41 pending extraction paths retain every original hunk. Adding the optional
dependencies changed only the Cargo.lock blob-identifier line in their staged
patch; its new SHA-256 is
`3a94cc96a5288b72ca3b90087996ff8c3c3d4f06e3fe0b7a18178f255ac726dd`.
Interface's eight pending edits are unchanged; firmware and the isolated source
tree are clean. The default CSGRS graph does not enable the new feature. No new
full root/browser/firmware/hardware, coverage/fuzz, release or pre-push campaign
is claimed. No push occurred. Phase 13 and all fifteen phases remain active.

## 2026-11-13: Polynomial primitives in native SDF enclosure programs

Previous turn: progress, retaining native SDF enclosures and precise graph-entry
accounting. Source `01f8b858bc9370ff9c6bf2247b104aaa557b7639` extends the optional
route to planes, spheres and all three torus axes; follow-up
`fa51fd2e232cc92e83662d1c02129ca35032ad8d` adds independent torus Jacobian
expectations. The valid-sphere rejection was reproduced before implementation.
Native strict parameter admission retains invalid versus undecided errors,
including beneath zero multiplication. Native squared factors preserve interval
correlation; source identity, distance qualification and subsequent native
composition remain intact. No new arithmetic or predicate kernel is introduced.

Independent formulas cover 625 field values, algebraic-center spheres,
degenerate admitted parameters, empty queries, exact graph/work thresholds,
cancellation and retry. The translated-sphere cell range is [-4,-1]. For the
quartic torus cell, the natural range [-11583/256,4617/256] is unresolved;
the fourth-order Taylor range [-35/2,-2775/256] has zero remainder and certifies
the whole cell inside. Independent derivatives give (240,0,448) at (3,0,4).
Fresh-process authored-recipe reconstruction regenerates its proof; this is
not generic SDF serialization, a topology guarantee or a metric promotion.

Final qualification passes 133 owner tests plus two replay children, 114 default
tests, strict all-target/all-feature Clippy, strict rustdoc, WASM, workspace
formatting and diff checks. ASan/leak checking passes 18 enclosure tests plus
replay children; its standard-library sysroot is not instrumented. The independent
Jacobian follow-up reruns the full owner suite, strict Clippy and ASan. Main
acceptance, formatting and indexed source-budget checks pass after both commits.

The actual default owner benchmark is byte-identical to baseline: 8,102,576
bytes, SHA-256 `be7d09278a800c3623335e196ec26bb172198cfa849ec25097f1a1607b6c9afc`;
all 46 smoke cases pass. The optional legacy harness preserves all 21 exact
definition/result/allocation rows. Final controlled local timings nevertheless
show trig natural/Taylor/reuse slowdowns of 1.43%/0.81%/2.43%, with separated
batch ranges. Linear compile has a 1.67% adverse median with overlapping ranges.
The optional consumer grows 6,560 executable bytes and 3,138 function-union
bytes. These costs are retained explicitly for the new completion capability;
performance neutrality is not claimed. Both candidate variants and unfavorable
measurements remain archived. No new successive_operations timing is claimed.

New sphere/torus Taylor construction completes at local medians of
179.422/657.039 microseconds. The largest measured temporary peak is 51,700
bytes; all successful-result/drop and late-exhaustion epochs have zero signed
live-byte delta. This warmed instrumented corpus is a measured completion
envelope, not a universal bound or competitive dynamics result. Scalar
refinement/payload, source ingestion and heap-byte resource exclusions remain
explicit. No acceptance assertion was weakened.

Evidence commit `83bccd2bbbd09cf4ff5e46e0df589fc2c9fd393a`, record
`docs/evidence/2026-11-13-interval-sdf-primitives`: 162 files, 10,205,187 bytes,
SHA-256 `11ea413b4825c5b0bd7ee1906f627d38c937d265209451c6bfbdb193fd263ed9`.
Scoped archive verification and all thirteen preceding immutable-stage checks
pass. The 41-path staged extraction patch remains unchanged, SHA-256
`3a94cc96a5288b72ca3b90087996ff8c3c3d4f06e3fe0b7a18178f255ac726dd`.
Interface's eight pending edits remain unchanged; firmware and the isolated
source tree are clean. Other primitives, piecewise/affine lowering, wider
adoption, wrapping/QR and release obligations remain open. No full workflow,
browser, firmware/hardware, coverage/fuzz or pre-push campaign is claimed.
No push occurred. Phase 13 and the full fifteen-phase objective remain active.

## 2026-11-13: Preserve affine frames in native SDF enclosures

Previous turn: progress, retaining polynomial primitives and their qualification.
Source `a686a1e7a2aa88acbacf9f95f6a25fd87ca89dfe` closes the reproduced
`Unsupported("affine transform")` gap for the existing enclosure grammar. It
emits the retained inverse's three coordinate rows through the existing Linear
emitter and binds the child to their shared graph nodes. Nested/sibling frames
remain local; each frame adds 30 nodes without expanding the source tree or
recomputing an inverse. Original source identity, parameter admission and metric
qualification remain intact. Compilation follows existing inverse_point semantics;
a manually assembled public transform is not promoted to an invertibility proof.

Acceptance checks 125 independent rational sphere values, an independent
chain-rule Jacobian, algebraic coefficients, raw inverse-field semantics, mixed
nested transforms, 64-frame growth, guards, empty queries, exact entry/work
limits, cancellation and retry. The transformed quadratic fixture tightens its
natural range [-3/8,5/8] to the exact Taylor range [-3/8,-1/8] with zero remainder,
certifying the complete cell inside. Native subdomain reuse, subsequent squaring
and fresh-process authored-recipe reconstruction preserve the source proof.
Affine compilation asserts no scalar/predicate dispatch; existing trace checks
remain unchanged. No distance, topology or generic interchange claim is added.

Final qualification passes 141 owner tests plus three replay children, 114
default tests, strict all-target/all-feature Clippy, strict rustdoc, WASM,
workspace formatting and diff checks. ASan/leak checking passes 26 enclosure
tests plus three replay children; its standard-library sysroot is not instrumented.
Main acceptance, formatting and indexed source-budget checks pass. A new test's
ambiguous Into expectation and duplicate shared-helper import were corrected
without weakening assertions; failed attempts remain in the evidence.

The owning default benchmark is byte-identical to baseline (8,102,576 bytes,
SHA-256 `be7d09278a800c3623335e196ec26bb172198cfa849ec25097f1a1607b6c9afc`),
and all 46 smoke cases pass. All 21 optional legacy definition/result/allocation
rows match exactly. Controlled local timings nevertheless show polynomial,
trig and Linear compile slowdowns of 3.59%, 3.57% and 2.47%, each with separated
batch ranges. The optional executable/function union grows 776/641 bytes.
A second out-of-line emission variant did not remove the slowdown and grew more;
it is rejected, with its source and results preserved. These are explicit costs
of new completion, not a performance-neutral refactor or speedup claim.

The new transformed-field compile/natural/Taylor/reuse medians are
2.267/7.529/236.631/11.842 microseconds; the largest temporary peak is 27,212 bytes.
Success/drop and late-exhaustion epochs end with zero signed live-byte delta.
This is a local warmed instrumented completion envelope; existing scalar and
heap-byte resource exclusions remain explicit. No new successive_operations or
competitive dynamics claim is made.

Evidence commit `3dea21c54bc849cd1ce18bcc71ffc4954191212e`, record
`docs/evidence/2026-11-13-interval-sdf-affine`: 166 files, 14,771,249 bytes,
SHA-256 `7c3068484c1f24940074673b1184cee8c5f6bafb76769434d138283c14ecf4d8`. Scoped archive verification and all fourteen
preceding immutable-stage checks pass. The 41-path staged extraction patch is
unchanged, SHA-256
`3a94cc96a5288b72ca3b90087996ff8c3c3d4f06e3fe0b7a18178f255ac726dd`;
Interface's eight pending edits are unchanged, and firmware/the isolated tree
are clean. Other primitives, piecewise fields, adaptive subdivision, wider
consumers, wrapping/QR and release obligations remain open. No full workflow,
root/browser/firmware/hardware or coverage/fuzz campaign is claimed. No push
occurred. Phase 13 and the full fifteen-phase goal remain active.

## 2026-11-13: Adaptive native SDF covers with unresolved leaves

Previous turn: progress, retaining affine SDF source closure. Source
`f3f1fd56a7951c508f7b4c6877b6c0e99fdfa36e` adds `SdfProgram::subdivide` after
reproducing the missing API. It reuses Hyperlattice's exact widest-axis selection
and midpoint bisection, activated through the existing optional enclosure feature.
Every parent and child retains its native source-bound enclosure. Immutable
axis/child records establish a complete closed cover with shared facets; this
is not a disjoint-volume model or a mesh. Original source/metric qualification
and existing image/compilation paths remain intact.

Only unresolved signs refine. Requested depth and singleton stops produce
explicit unresolved leaves. Native domain, graph/arithmetic, cell-capacity and
cancellation failures abort without publishing a partial cover. Graph work and
dyadic accounting are shared across the traversal; the separate capacity bounds
all retained records, including parents. Scalar refinement/payload, allocator
spare capacity and heap-byte exclusions remain explicit. Exact 48/47 graph-entry
thresholds, one-less work/arithmetic failures, actual capacity limits, minimum-grid
bisection overflow and clean retries are maintained assertions.

The dependency fixture's natural range [-3/8,5/8] becomes six exact leaves whose
ranges are all strictly negative, proving the complete original query inside.
Its independent endpoints and eleven-record split tree preserve coverage. The
supplemental 3D witness checks all eight octants, exact image bounds and X/Y/Z
split order. Empty queries, zero fields, coarse unresolved singletons, boundary
contacts, generated affine spheres, subsequent leaf-image queries and fresh
process reconstruction retain their distinct meanings. A successful cover may
contain unresolved leaves; it is not counted as resolved geometric classification.

Final qualification passes 150 owner tests plus four replay children, 114
default tests, strict all-target/all-feature Clippy, strict rustdoc, WASM,
workspace formatting and diff checks. Focused ASan/leak checking passes 35
enclosure tests plus four replay children; its standard-library sysroot is not
instrumented. Main acceptance/formatting and indexed repository-budget checks
pass after the final 3D test edit. The external consumer's initial locked build
failure and offline lock update are retained; repository Cargo.lock is unchanged.

Default benchmark files are 8,102,576/8,102,600 bytes, with identical
4,467,244-byte function unions and identical loaded code/data sections except
the build-ID note. All 46 smoke cases pass. The original byte-identity assumption
was rejected and the exact metadata difference recorded without rewriting any
artifact. All 21 optional legacy definition/result/allocation rows match exactly.
A controlled local CBBC run has no separated adverse batch ranges; the trig
compile median ratio 1.0003 has overlapping ranges. No speedup or universal
no-regression claim follows. The optional executable/function union grows
2,712/450 bytes; all samples and dependency states remain archived.

The dependency cover completes at local median 55.358 microseconds, with eleven
records, six certified leaves, 668 graph work units and 13,576 temporary peak
bytes. The affine-sphere cover takes 456.635 microseconds, with 23 records,
12 leaves, four explicit unresolved leaves, 7,012 work units and 29,464 peak
bytes. Success/drop and separate late-exhaustion/capacity epochs end with zero
signed live-byte delta. These are warmed instrumented completion/resource
observations, not cold-cache, competitive dynamics or universal bounds.

Evidence commit `d794e74164e2b8772d31c48d2acadc2de3eb5590`, record
`docs/evidence/2026-11-13-interval-sdf-subdivision`: 131 files, 9,241,691 bytes,
SHA-256 `75802cc38a3909dbe534b662466c0d081fe6ea3eba6b264d4851a866f3fdaa0e`. Scoped verification and all fifteen preceding
immutable-stage checks pass. The 41-path staged extraction patch remains
unchanged at SHA-256
`3a94cc96a5288b72ca3b90087996ff8c3c3d4f06e3fe0b7a18178f255ac726dd`;
Interface's eight pending edits are unchanged, and firmware/the isolated source
tree are clean. Piecewise/other primitive lowering, domain/Taylor refinement
portfolios, wider adoption, wrapping/QR and release obligations remain open.
No new full workflow, root/browser/firmware/hardware, coverage/fuzz or
successive_operations campaign is claimed. No push occurred. Phase 13 and the
full fifteen-phase objective remain active.

## 2026-11-13: Correct affine SDF Lipschitz certificates

Previous turn: progress, retaining adaptive SDF covers. Review of the broader
plan reproduced an existing false certificate on revision `37eed4312`: an X
scale of 1/2 makes f=x change by 2 over world distance 1, while the API returned
bound 1. Source `265249c34ac6580f2e23bb07583d3150d067dda2` corrects transport
through the retained inverse linear part. This is a correctness repair, not a
behavior-preserving refactor or completion of the next plan phase.

Native Vector3 self/cross dots form the symmetric Gram matrix. The maximum
absolute row sum gives a proved conservative Euclidean expansion factor; an
undecided row ordering uses an independent Frobenius bound. Existing strict
max/provenance helpers preserve child certainty. Translations, unknown children
and structural constants retain their meaning and predicate prefix. No scalar
kernel, public report shape, policy, cache or resource contract is replaced.
Exact similarities and signed axis scales remain sharp; general bounds need
not be minimal. Raw inverse consistency and stronger source/replay contracts
remain outside this correction.

Maintained acceptance covers the original failure, scales/reflections/shear,
algebraic rotation, nested transforms, constants, collapsed raw inverses,
undecided norm order and unknown children. Three fields exercise 324 exact
point-pair inequalities with independently formed world distances. Fresh-process
reconstruction recomputes the certificate. Trace assertions retain 18 old AABB
comparisons plus two new maxima; constants/unknown children stay at 18 and
translation at zero. No existing assertion was weakened.

The selected source passes 158 owner tests plus five replay children, 121
no-default tests plus one replay child, strict all-feature/all-target Clippy,
strict rustdoc, WASM, formatting and diff checks. ASan/leak checking passes eight
new tests plus one replay child; its standard-library sysroot is not instrumented.
Main owner acceptance, formatting and indexed budget checks pass after transfer.
The external default-feature consumer exercises the affected public API locally;
it is not a packaged release claim.

Both correct matrix/vector implementations and all failed/adverse observations
are retained. All ten probe report/allocation rows agree between them. The
vector scale/shear median ratios are 0.7358/0.8313; the unchanged linear probe
is 1.0062 with separated short-batch ranges. All 46 owner cases complete, with
24 separated adverse candidate confidence ranges and median ratios spanning
0.9669–1.0746. No general speedup or no-regression claim is made. The old incorrect
affine rows cannot serve as a same-completed-contract performance baseline.
The selected default executable grows 14,744 bytes to 8,117,344; its read-only
ELF function union grows 10,269 bytes to 4,477,513. Exactness takes priority over
these measured costs. Every measured complete-report/drop allocation epoch
ends with zero signed live-byte delta; scale allocates zero and shear peaks at
144 bytes. These are warmed local observations, not universal resource bounds.

Evidence commit `cd7ffa124a60153eabec7b3cb10900decfd4f731`, record
`docs/evidence/2026-11-13-interval-sdf-lipschitz`: 1,589 files,
11,579,748 bytes, SHA-256
`3a563ca07af51f0ecd88de8f56dadd146a5a2515ddb0408705848c2d3be18a98`. Scoped archive
verification and all sixteen preceding immutable-stage checks pass. The original
41-path staged extraction retains SHA-256
`3a94cc96a5288b72ca3b90087996ff8c3c3d4f06e3fe0b7a18178f255ac726dd`;
Interface's eight pending edits are unchanged, and firmware/isolated source are
clean. No push, full workflow, root successive_operations, browser/firmware/
hardware, full fuzz/coverage or release campaign occurred. Wider adoption,
wrapping/QR and the full fifteen-phase goal remain open.

## 2026-11-13: Predicate filter admission frontier and rejected placements

Previous turn: progress, correcting affine SDF bounds. Phase 10 review on
`db6cab1e7` confirms existing certified primitive, exact-word and rational
orientation filters before the general resolver. The plan requires profiling
before extending that cascade. Nine independent rational, algebraic and
transcendental controls now establish an initial admission frontier, including
2^-100 near-degeneracy, exact cancellation, generated translations and collinearity.

An unconditional native dyadic coordinate filter helps separated e/pi points
but penalizes cheap and structural cases: initial strict retained-input ratios
at spacing 2^-16 range from 0.54 to 47.28. A second implemented experiment adds
a native two-term difference only after the existing signed-term callback
fails, preserving the resolver order. Its missing trace is reproduced before
implementation, and its full owner suite passes. Nevertheless, matched strict
retained/rebuilt ratios are 1.2376/1.2435 for separated e/pi points and
1.2240/1.1337 near degeneracy, with separated adverse batch ranges. Both
placements are rejected. Neither runtime code nor a new dependency is retained.

Only strict positive/negative enclosure exclusion could decide in either
experiment. Zero contact, unavailable enclosures and actual local budget failures
preserve fallback. Ordinary float proposals never decide. Native work/size
accounting excludes Hyperreal refinement; no total-work guarantee is invented.
The late-stage experiment changes the separated case's receipt from Exact/Refined
to Filtered/Filter under both policies; those observed differences are archived,
not presented as behavior-preserving equivalence.

Source `6c0819c52bf7949ecd3b464ecdb220c04f15f5d9` retains only the regression
corpus: exact result/receipt equality under both policies, 36 generated-translation
and reversal checks, admitted path/structural/refinement/terminal trace counts,
and fresh-process reconstruction. Earlier assertions remain intact. Qualification
passes 381 owner tests plus one replay child, 366 no-default tests plus one replay
child, strict all-target/all-feature Clippy, strict rustdoc, WASM, formatting and
diff checks. Main owner acceptance, formatting and indexed budget checks pass.

The matched probe uses CBBC order, CPU 0, disabled ASLR, one executable path,
recorded warmup frequency, separate tracing and five batches of 32 complete
outcomes per mode/policy. Rebuilt mode includes fixture construction; retained
mode excludes it. Neither is called cache-empty. All raw samples, adverse rows,
prototype source, failure witness and build artifacts' hashes are retained.
The initial coordinate run is exploratory, not a matched speedup claim. The
baseline external lock reconstruction is labeled explicitly. No production
performance or shipped-size improvement is claimed for this test-only retention.

Evidence commit `ae704229b3963f9ad1f6f123e88ecbb1593f6820`, record
`docs/evidence/2026-11-13-interval-predicate-filters`: 76 files,
788,273 bytes, SHA-256
`9dd0dc428ae231b2a2fd178767da44ba2d7ce9aba7dcec89c0ea70a59ebf87aa`. Scoped archive
verification and all seventeen preceding immutable-stage checks pass. The
41-path staged extraction retains SHA-256
`3a94cc96a5288b72ca3b90087996ff8c3c3d4f06e3fe0b7a18178f255ac726dd`;
Interface's eight pending paths are unchanged, and firmware/isolated source are
clean. No push occurred. Representative Hypertri/Hypercurve/Hypersolve/Hypermesh
qualification for a retained filter, broader predicate families, release and
all other open plan obligations remain. No new full workflow, sanitizer,
fuzz/coverage, root successive_operations or hardware campaign is claimed.
Phase 10 and the full fifteen-phase goal remain active.

## 2026-11-13: Reproduce dynamics controls; discard an invalid inline experiment

Previous turn: progress on predicate admission. Current baseline, extraction and
provider-error controls reproduce the prior owner artifacts and all 194 complete
result/drop observer rows. A private-helper experiment accidentally duplicated
an existing `#[inline]`. Strict Clippy rejected it; the duplicate was removed,
without suppressing warnings. It is an invalid experiment, not a new optimization
or a resolution of phase 11's cost gate. Current corner Callgrind totals remain
2,503,560 baseline versus 2,507,720 provider control for 32 calls. No fresh wall-
clock campaign or universal impossibility claim is made. Corrected control Clippy
and restored original extraction routing/axis/format checks pass. The isolated
tree is clean at `3c9a6a8e`; main and Interface were not changed by this experiment.

Evidence `90f42c67e7a64a67a9f43c925a0e291174b67e7d`, record
`docs/evidence/2026-11-13-interval-dynamics-replay-inline`: 105 files,
33,593,272 bytes, SHA-256
`518439a679bacedaa00bbd1757636d3d1ca5d162c6ad18f49ae2ec6f3e6ad57f`.
The invalid artifact, source-inspection mistake, controls and restoration are
retained explicitly. Phase 11 remains open.

## 2026-11-13: Retain whole-horizon evolution in exact state charts

Source `eac3183213830ef30c11e01352f98bff5cdac3b5` adds
`CoordinateField::integrate` after reproducing the missing API on `3c9a6a8e`.
The existing integrator runs entirely in the retained exact chart; original-
coordinate Cartesian projections never become the next step's input. The report
keeps the original/transformed source association, inverse evidence, all native
attempts and exact native failure. Completed initial/tube/endpoint transport is
atomic. A failed final transport exposes no original-coordinate result; a
stopped native run keeps only its explicitly qualified native prefix. Identity,
empty, invalid and resource outcomes remain distinct.

Eight new integration tests reuse existing fixtures and preserve every prior
assertion. Three constant-flow steps independently establish endpoints and tube
inclusion, retain the zero-width source invariant through the chart, and feed
output into another integration and field evaluation. Native report equality,
fresh-process definition/request replay, fixed dispatch counts, stopped-prefix
work preservation, empty/unit/identity/invalid inputs, cancellation, exact budget
boundaries and retry pass. Native/complete graph work is 1,554/1,657 and native
scalar work 937,228/1,684,486. A 64-step combined structural-entry boundary succeeds
at 524 and fails transport at 523 after native completion. These accounting units
exclude scalar refinement/payload lifecycle, allocator capacity and total heap.

The affine example runs four steps and checks analytic tube/endpoint inclusion.
Its first new assertion incorrectly expected reachable-set equality; the native
candidate search actually produces [-384,384] while the analytic x range is
[-128,128]. The corrected example asserts that precise computed envelope and
checks analytic inclusion separately. The original one-step assertions and native
algorithm are unchanged. Fixed charts do not eliminate native overestimation.

Nine isolated groups pass: 67 all-feature tests, 34 minimal-feature tests,
no-default check, strict all-target Clippy, warning-denied docs, WASM, the release
example, formatting and diff checks. Six combined groups pass, including 78 owner
and four downstream routing/axis tests. Fuzz package check/Clippy/format and ASan
build pass; the expanded shear/nonlinear/Jacobian/inverse/replay corpus exercises
one to four whole-horizon segments. Its fixed-seed one-minute campaign completes
4,184 libFuzzer inputs without failure (including inputs taking the length guard).
Main's combined fuzz check and strict Clippy also pass.

Four focused external consumer workloads preserve exact complete Debug rows and
independent analytic endpoints. Old/new native allocation counts agree; manual
and owned transported outputs also have identical allocation statistics and zero
remaining allocated bytes after drop. Initial unfavorable timing samples are
retained. Eight later randomized paired rounds show native median shifts of about
+0.4%/+0.6% for 16/64 segments and owned transport about +0.7%/+1.9% versus manual
assembly. Manual assembly lacks the new atomic aggregate resource admission and
is only a successful-output cost comparator. No speedup, cold-cache, competitive
or universal performance claim is made. External .text grows by 10,432 bytes;
function-address union grows by 10,208 bytes. The external lock is shared across
controls and differs explicitly from the workspace lock. No new dependency or
existing production predicate order changed.

Evidence `2845f1e399e85f62284274a0403ce1ab955b386f`, record
`docs/evidence/2026-11-13-interval-coordinate-evolution`: 482 files,
5,894,162 bytes, SHA-256
`d556d0d524bd3d45e79d90c2c5efd7d6d272c6afd5ba1ad7c33b89f810302150`.
Scoped archive verification and nineteen preceding immutable-stage checks pass.
The indexed source budget passes at 4,174 files and 143,176,954 bytes before this
ledger registration. Main's 41 staged extraction paths retain SHA-256
`3a94cc96a5288b72ca3b90087996ff8c3c3d4f06e3fe0b7a18178f255ac726dd`;
Interface's eight pending paths are unchanged. Both isolated trees are clean,
and firmware is unchanged. No push occurred. Adaptive wrapping/QR, changing
frames, whole-resource closure, wider adoption, competitive workloads, phase 11
costs, coordinated release and the remaining fifteen-phase obligations stay open.
No new full coverage, root successive_operations or hardware campaign is claimed.

## 2026-11-13 — exact clock arithmetic migration

Previous goal turn: progress (whole-horizon coordinate evolution). Retained
software migration in source `c7108d084810f175232ab087749d95edaca10f0d`
and Alumina Firmware `cb92f80c08ba13cb732dd2a3a4ad63ccdee5e1cd`.
The full fifteen-phase goal remains active; this is not Phase 9 completion.

Hyperinterval now provides checked signed i128 quotient bounds and exact scalar
images of all twelve primitive integer range families. Quotient bounds are
numerical floor/ceiling of the ratio, not a discrete set substituted for the
rational value. Zero denominators, MIN/-1 overflow, signed-minimum denominators,
empty ranges and scalar-product overflow retain explicit outcomes. The shared
positive-denominator kernel uses truncating quotient/remainder with corrections
proved not to overflow. Fixed arithmetic reuses it. Integer sum retains endpoint
addition order and uses order proved by monotonicity after checked products/sums;
public ingress validation remains unchanged. Private primitive paths perform no
Hyperreal dispatch or rational reductions, as asserted by a new trace test.

Alumina-clock delegates intersection, forward affine range arithmetic, directed
inverse division, and explicit midpoint/radius. Boot identity, causal validation,
freshness, health, deadlines, scheduling and stored scaled model stay with the
clock owner. Its new production dependency disables defaults and enables only
integer; no std, allocation or arbitrary precision enters the dependency graph.
Device heartbeat source is unchanged. Original clock tests remain, with new
fractional-offset and rejected-sample/retry regressions.

The standalone observer reproduces all 481 pristine public-observation rows,
wire bytes and typed errors over 24 analytic complete workloads; independent
forward/inverse inclusion checks pass. Estimator layout remains 208 bytes with
16-byte alignment, and all measured allocation/reallocation/byte counts are zero.
The first candidate was about 10% slower and is rejected. Recorded intermediate
inlining, outlined Option-map and affine/division investigations converge on a
final 24-pair pinned host comparison: median candidate/baseline ratio 1.000469,
bootstrap 95% interval [0.999465,1.001691]. This resolves neither a slowdown nor
a speedup. Native probe .text shrinks 267,196 -> 267,148 bytes and binary size
428,016 -> 423,712 bytes. Fixed control .text shrinks 240,028 -> 239,740 bytes;
six paired run medians show no regression on its five completed operations.
All fixed counts and checksums match; full Q30 differential coverage also passes.
These are host controls, not MCU or whole-application performance claims.

Final checks pass: 213 all-feature Hyperinterval tests, 53 Rust 1.88
integer/fixed/serde tests, strict owner all-target Clippy, warning-denied docs,
formatting, 7 clock and 76 simulator tests, strict firmware owner/simulator
Clippy, 86 Interface client tests, and Interface WASM checking. Clock no_std
checks pass for thumbv7em-none-eabihf and wasm32 with Rust 1.88. Production
checkouts repeat owner tests and firmware Clippy after transfer. Interface
Clippy 1.95 remains blocked by the same large_enum_variant in network.rs and
nonminimal_bool in worker.rs reproduced on unchanged baseline source. The
attempted Interface 1.88 check is inapplicable because its MSRV is 1.95.
No warning was suppressed and no unrelated Interface source was changed.

Evidence `d478b26d5d8703199e56a656165001cb0e770a64`, record
`docs/evidence/2026-11-13-interval-clock-migration`: 382 files,
64,547,622 bytes, SHA-256
`5f0a54ec727da2384d4b4708e2e9763fbcf65eaf94ebbc7f325a9338dfa4c253`.
Scoped archive verification and all twenty previous frozen-stage checks pass.
Indexed source budget before this ledger: 4,177 files, 143,196,608 bytes.
The 41 staged CSGRS extraction paths retain binary-patch SHA-256
`3a94cc96a5288b72ca3b90087996ff8c3c3d4f06e3fe0b7a18178f255ac726dd`;
Interface's eight pending paths retain their original hash. Firmware and the
isolated core source checkout are clean at their retained commits.

Hardware cycles, stack maxima, HIL, final MCU image and WASM code-size comparison
remain unmeasured. No new root successive_operations, whole-application cost,
full workflow, sanitizer or coverage campaign is claimed. No push occurred.
The remaining firmware/FOC/servo migration, Phase 11 cost blocker, broader
integration and coordinated release obligations remain open.

## Production Q30 interval delegation (2026-11-13)

Retained CSGRS `441c925b559005b81074f9fc45d9b47adb3298b4`, Firmware
`953ed4b7e8059288abe5146cfd68400f0961587e`, and Interface
`27086866147cb5db9fb22e6cccc4581a6fea5a02`. Hyperinterval owns exact fixed
arithmetic and ordered result reconstruction; Q30Interval retains its domain
wrapper, eight-byte layout, const surface and explicit Range/DivideByZero
outcomes. Clarke retains wide differences through the shared product before
one outward rounding and exact narrowing. Nominal point arithmetic and wide
servo carriers are unchanged. The admission record identifies the affected
properties, full integer endpoint/ratio domain and independent missing
acceptance witnesses; private ordered reconstruction is justified per caller,
while public reversed-endpoint validation remains checked.

Qualification passed: Hyperinterval 216 all-feature tests/doctests, 39
fixed/serde MSRV tests/doctests, strict Clippy, docs and formatting; Firmware
168 tests on Rust 1.88 and 1.95 plus strict MSRV Clippy; seven maintained
transform conformance tests per compiler including allocation/error cases;
three scalar Q30 differential tests; fixed-only Hyperlattice tests/Clippy;
86 Interface tests and actual-client locked native/WASM validation; and the
Cortex-M clock/FOC build. All 9,488 full observer rows remain byte-identical
across compilers and retained variants, including extreme endpoints, ratios,
errors and composed transforms. Feature inspection confirms fixed-only FOC
and integer-only clock without std, allocation, bigint or global state.
Interface Clippy reproduces its two pre-existing large_enum_variant and
nonminimal_bool findings; it is not reported as passing.

Completed transform measurements use 24 alternating pairs per compiler,
explicit warmup, pinned CPU, matching counts/checksums and zero timed
allocations. Candidate/baseline median ratios for Clarke, Park, inverse Park,
inverse Clarke and the composed chain are respectively 0.909, 0.824, 0.816,
0.877 and 0.872 on Rust 1.88; 0.972, 0.863, 0.860, 0.938 and 0.895 on 1.95.
Each seeded bootstrap 95% interval is below one. Host noise and frequency
remain uncontrolled; these are scoped host results. Native text shrank
235,644 to 235,180 bytes and 231,040 to 230,352 bytes; complete probe binaries
were 369,928 to 369,928 and 372,944 to 372,360 bytes. Scoped emitted
owner/consumer symbol unions shrank 6,949 to 6,508 and 6,515 to 5,841 bytes.
The fixed primitive control preserved counts/checksums with improved sampled
medians. Regressing intermediate candidates remain recorded as rejected.

Wasmtime 50.0.1 independent integer replay passed 1,660 complete transforms
and 405 range checks (335 success, 70 Range) on both artifacts, with the
unchanged million-fuel per-call limit and no exhaustion. Maximum observed
fuel fell 20,481 to 16,555 and WASM bytes 40,082 to 35,451. This is emulator
evidence, not hardware qualification. MCU cycles, stack/register pressure,
HIL and final firmware image remain unmeasured. No new root
successive_operations, whole-application benchmark, complete workflow,
sanitizer or coverage campaign is claimed.

The Interface commit adds only two lockfile dependency entries required by
clock/FOC. Its eight earlier pending edits remain unstaged: seven files match
their saved bytes, and removing precisely the two task entries restores the
original Cargo.lock bytes. The old patch hash is no longer an applicable
preservation check because the lockfile base changed. All 41 staged CSGRS
extraction paths retain binary-patch SHA-256
`3a94cc96a5288b72ca3b90087996ff8c3c3d4f06e3fe0b7a18178f255ac726dd`.

Evidence: `docs/evidence/2026-11-13-interval-q30-migration` at archive commit
`e98298a3209f9d45c7ebd07e34d87c33679049ef`: 717 files, 112,804,931 bytes,
manifest SHA-256
`800a92fdd0ed0b06b13885c66947d589c9526e402e5e0674cbef5640e46d8f76`.
Scoped archive verification passed; all 21 prior frozen stages matched their
registered digests before archiving. STATUS.md, CONTRACT.md and REPRODUCE.md
retain exact commands, source revisions, proof audit, raw timings, replay and
limitations. The new stage is frozen. Source budget before this ledger update
passed at 4,178 files / 143,206,836 bytes. No push occurred. Phase 9 hardware
and wide servo obligations, Phase 11 retention costs and the full 15-phase
goal remain open.

## Servo interval delegation and shared midpoint kernel (2026-11-14)

Retained CSGRS `b30763aa5a4bf5895f62462cb900eb3730e0f49b` and Firmware
`558df6d6ebf4dbca5377abf64b775c9e072aa013`. Fixed and integer intervals share
exact unsigned-distance midpoint arithmetic, with explicit rounding and
proved representable sums. Fixed widths retain the full unsigned distance.
Servo position intervals delegate midpoint/width while keeping const ingress,
endpoint methods and layout. Q30 velocity errors widen before shared i64
subtraction. Position errors retain their required i128 endpoints and exact
full-width differences. No control policy, fault order or protocol changed.

The missing full-range acceptance witnesses were added and passed on the
baseline before migration. Final qualification passed 219 core tests/doctests,
56 MSRV fixed/integer/serde tests/doctests plus the later integer-midpoint test,
171 firmware/clock/simulator tests, 28 fixed-only Hyperlattice tests, seven FOC
conformance tests, 86 Interface client tests, locked Interface WASM validation,
and Cortex-M clock/FOC checks. Core tests cover all 96 valid fixed scales,
all three rounding modes, empty/replayed outputs and all 12 primitive integer
families. Zero scalar dispatch/reduction counts remain asserted. Strict core
and firmware Clippy, core docs and affected formatting pass. The committed
production paths repeat the core 219 and firmware 171 tests and Clippy.
Interface Clippy reproduces its two existing findings and is not passing.

Complete observers match: 5,366 servo rows per compiler (Rust 1.88/1.95),
9,488 FOC rows and the maintained clock corpus. Successful reports, errors,
fault-latched states and allocation reports are preserved. Independent WASM
FOC replay completes 1,660 transforms and 405 range cases with unchanged fuel
and size. Independent position replay completes 2,090 midpoint/width calls;
max fuel falls 206 to 21 and probe size 5,233 to 3,495 bytes, without changing
the 10,000-fuel limit or exhausting it.

With identical release profiles, completed counts/checksums, zero timed
allocation, warmup and alternating CPU-0 runs, accessor/successful-service/
fault-latched-service median ratios are 0.9705/0.9909/0.9727 on 1.88 (12 pairs)
and 0.9826/0.9941/0.9724 on 1.95 (24 pairs). Servo text shrinks 247,148 to
247,116 and 244,590 to 244,462 bytes; binaries stay 389,560 and shrink 395,400
to 395,272. Clock has no resolved timing change: ratio 0.9994, bootstrap95
[0.9975,1.0012], with smaller text and unchanged binary. FOC chain ratio is
0.9836, bootstrap95 [0.98349,0.98364], with smaller text/binary. These scoped
host/emulator measurements do not establish MCU cycle, stack or HIL behavior.
Regressing intermediate candidates and initial build/test corrections remain
recorded separately from the retained artifacts.

The captured normal and successive_operations CSGRS dependency graphs exclude
Hyperinterval, so these arithmetic edits do not enter that benchmark binary.
No fresh whole-geometry benchmark, full workflow, coverage or sanitizer campaign
is claimed. Hardware cycles, stack/register pressure, HIL and final firmware
image remain open. The full 15-phase goal, Phase 11 retention costs and other
integration/release obligations remain active. No push occurred.

Evidence record `docs/evidence/2026-11-14-interval-servo-migration`, archive
`f379647b6ee0bb1986ead5ea013eec020d2cd922`: 565 files, 6,089,833 bytes,
SHA-256 `573481101d2a17940669399dc69b0cc56d75f808f85a7c32bfed568610fb9b6f`.
Scoped archive verification passed. All 22 prior frozen stages match their
registered digests. The 41 staged extraction paths retain patch SHA-256
`3a94cc96a5288b72ca3b90087996ff8c3c3d4f06e3fe0b7a18178f255ac726dd`;
Interface HEAD and its eight pending edits are unchanged. Source budget before
this ledger update: 4,180 files / 143,218,006 bytes. STATUS.md, CONTRACT.md,
REPRODUCE.md, source patches, exact observers, raw timings and artifacts retain
the qualification scope and limitations. The new evidence stage is frozen.

## Xtensa images, frame observations and board build repair (2026-11-14)

Installed a pinned Espressif compiler/linker and reproduced baseline and current
release builds through the repository's xtask. Firmware bdee397 (before all
three arithmetic migrations) and 558df6d both initially failed: main called a
TinyBee-only UART diagnostic method on every board. The same one-line TinyBee/
TinyBee-4mb feature guard normalizes both comparisons. It preserves their UART
behavior and adds no methods or hardware configuration to other boards. The
repair is retained as Firmware `b9ecf8df54723e532a42867a1d3c549f827d5f6d`;
the qualified candidate tree was checked equal to that commit. The untouched
baseline is not claimed to build.

All ten release builds pass: baseline/current MKS FOC, TinyBee 8 MiB,
TinyBee 4 MiB, T-Deck Pro and T-LoRa Pager. Toolchain identities are Rust
1.90.0-nightly abf50ae2e (1.90.0.0), GCC 14.2.0_20241119 and Xtensa LLVM
20.1.1_20250829; official espup/espflash binary hashes are retained. The two
existing target dead-code warnings occur on both revisions, so these are not
warning-free target Clippy results. Portable default members pass 601 tests/
doctests and strict all-target Clippy on Rust 1.88; target formatting passes.
All capability JSON is byte-identical, digest validation passes and boards stay
non-armable. Hyperinterval's embedded graph enables only integer/fixed.

Summed allocated file-backed section bytes and executable-byte changes are:

| Board | Baseline | Current | Executable delta |
| --- | ---: | ---: | ---: |
| MKS FOC | 4,189,417 | 4,188,057 | -1,376 |
| TinyBee 8 MiB | 4,289,065 | 4,287,793 | -1,328 |
| TinyBee 4 MiB | 4,289,069 | 4,287,789 | -1,336 |
| T-Deck Pro | 4,095,065 | 4,093,585 | -1,480 |
| T-LoRa Pager | 4,049,997 | 4,048,493 | -1,504 |

These are section measurements, not inferred flash layouts. NOBITS totals and
linker stack capacities are unchanged. espflash 4.7.0 save-image, with each
board's chip/flash capacity and the tool's default partition layout, saves
TinyBee 8 MiB (4,353,440 bytes), T-Deck Pro (4,159,232) and T-LoRa Pager
(4,114,144); all shrink from baseline. This does not validate a board-declared
partition map. Both 4 MiB configurations fail on both revisions. The current
MKS FOC app exceeds the default 4,128,768-byte partition by 59,392 bytes;
TinyBee-4mb exceeds by 159,136. Baseline excesses are 60,752 and 160,416.
These pre-existing flash-fit failures remain open; a linked ELF is not a
passing packaged-image gate. No assets or flash declarations were weakened.

Static frame observations accept only entry instructions at ELF Function-symbol
starts; interior encodings and unknown prologues are not treated as frames or
zero. Q30 multiply frames fall 368 to 96 bytes on ESP32 and to 80 on S3;
square falls 128 to 96/80. ESP32 RotorCalibration::observe grows 144 to 160,
and cosine_first_octant 80 to 96. The +16-byte local changes are explicitly
unresolved at the whole-call-stack level; smaller callees do not prove a lower
live peak. Maximal recognized local frames are unchanged (28,032 MKS FOC,
27,968 TinyBee, 27,760 S3), but do not bound nested calls, interrupts, window
spills or vendor tasks. MCU cycles, runtime stack/register-pressure and HIL
remain unmeasured. No serial fixture/probe runner was visible and no hardware
was used or flashed.

Evidence `docs/evidence/2026-11-14-interval-embedded-qualification` is verified
at archive `0abb33372171fb1a1006b38f29b02d5e023074b6`: 141 files,
122,353,043 bytes, SHA-256
`8c07771a8fb09f57a162aaf40d81f2c51de9b76b56c20d48d9dd3fe31d391b60`.
It retains original failures, normalization patch, tool identities, complete
logs, ELF/packaged artifacts, sections/symbols/disassembly and scoped frame
comparisons. All 23 prior frozen stages verify. The 41 staged extraction paths
and eight Interface pending edits are preserved. No push occurred. Whole-stack
release, flash-fit/hardware qualification, Phase 11 costs and the full goal
remain active; no phase is marked complete.

## Reproduce dynamics costs across the complete owner workload (2026-11-14)

Phase 11 remains unretained. A fresh isolated worktree at b3dcd2d3b reproduces
baseline, the exact pending extraction and the earlier associated-provider-error
control. All three owner benchmark ELF hashes equal the prior record, despite
intervening interval work. Their binary/function-union sizes remain baseline
13,312,600/8,044,187, extraction 13,304,584/8,036,358 and provider control
13,302,416/8,035,134 bytes. Six unsized symbols remain disclosed. The separate
observer reproduces identical complete results and allocation/deallocation/
reallocation/bytes/peak/live rows for all 194 owner workloads. Its entire workload
body matches the real benchmark; this is not an independent geometry oracle.

Six current real-owner sweeps complete all 194 cases, using the unchanged
Criterion settings, CPU0, disabled process ASLR, the same executable path and
baseline/extraction/provider/provider/extraction/baseline order. No compilation,
tests or instruction profiles run during timing. Adverse confidence-interval
screening flags 62 extraction and 76 provider cases; this is not simultaneous
significance or an acceptance tolerance. Before the current comparison was
available, the six adverse examples published in the old provider record were
selected for longer repeats (one-second warmup, two-second measurement, thirty
samples). All six provider cases retain adverse separation:

| Case | Provider/baseline mean ratio |
| --- | ---: |
| Axis-aligned line offset | 1.0421 |
| Quadratic hodograph | 1.0924 |
| Rectangular-pad clearance | 1.0703 |
| Additive-bead schedule | 1.0452 |
| 256-record parser | 1.0190 |
| Exact trace import | 1.0376 |

Extraction also retains adverse separation for hodograph, pad clearance and
parser in these repeats. All favorable and adverse observations remain in the
record; the other broad-screen flags did not receive longer confirmation here.
A repeat-driver preflight referenced the wrong status filename and failed before
measurement; its log is retained, and the corrected complete run passes.

The new diagnostic covers all 582 Callgrind observations: 194 cases times three
variants, 32 warmed completed-result/drop callbacks each. Four instrumentation
workers run after wall timing; their elapsed times are not performance evidence.
All compressed profiles decode to their recorded hashes and their function self
counts sum to the total. Extraction has 101 increased, 15 decreased and 78
unchanged instruction observations; provider control has 95 increased, 21
decreased and 78 unchanged. These are single observations, not universal bounds.
Crucially, all six longer-timed cases have identical instruction totals AND
per-function self counts across all variants. Instruction-count reduction alone
therefore does not explain or clear those runtime failures. Layout/cache effects
remain hypotheses; no unique hardware cause is established.

Tangent alignment changes 240,808 to 241,704 instructions per epoch, with
attribution shifting among exact-rational product-sum layers. Corner lookahead
reproduces 2,503,560 baseline, 2,508,360 extraction and 2,507,720 provider counts.
Complete attribution and 57 bounded owner-function disassemblies are retained.
Generic names can combine instantiations, and observer code generation is not
assumed equal to the timing executable. No unrelated parser/geometry mathematics,
compiler policy, predicate order, fixture or gate is changed to hide a failure.

Current provider fixtures pass two tests. The restored original extraction passes
three routing tests (including fresh-process full report/dispatch comparisons in
both policies), axis migration, workspace formatting and staged whitespace checks.
The isolated tree is then restored clean at b3dcd2d3b. No new full all-feature/
Clippy, downstream cost, successive_operations, coverage, fuzz, sanitizer, WASM,
packaging or pre-push qualification is claimed. No push occurs and no production
source is retained from this investigation. Main's 41-path index and Interface's
eight pending edits remain exact; all four authoritative states match their start
before archival. All 24 prior frozen records verify unchanged.

Evidence `docs/evidence/2026-11-14-interval-dynamics-current-cost` is verified
at archive `26216ee286e7048d3b60dedd5d1ef3c7d28ce17b`: 6,714 files,
50,033,990 bytes, SHA-256
`35a9c91d2cdca9ab3457d8a7ada0755b966de6fa907ed9a9c47e5888bac53db9`. The stage is now frozen. A further candidate must
address a demonstrated compilation/layout or result-ownership cause and pass the
unchanged full contract. Phase 11 and every remaining full-goal obligation stay open.

## Retain polynomial arithmetic over existing Taylor proofs (2026-11-14)

Phase 13 gains an opt-in `taylor-polynomial` capability, retained at
`568ddff117ee0ba873cdfa14186b1830201ede4c`. Hypersolve owns the exact sparse
coefficient arithmetic and remainder evidence; Hyperdynamics forwards the feature.
A shared original model supplies center, domain, order, bindings and source guards.
Constant/input/negation/add/subtract/multiply maps retain every truncated monomial
and P*S, Q*R, R*S cross term. Generated outputs accept further maps without fresh
differentiation. Flat shared stage history preserves source/map/node identity and
avoids recursively owning parent chains. Subdomain output reuses the existing
Cartesian carrier, including distinct empty/unit semantics. Private proofs replay
from original source/request and maps, including fresh-process validation.

The preimplementation missing API witness, empty/unit regression and misattributed
multiplication-evidence witness all fail before their fixes. The last finding
arose because a reused input shared its producer's proof: lookup now checks the
requested map node before returning a multiplication certificate. Ten core tests
cover independent exact/algebraic coefficients, all residual contributions,
multivariate cancellation, generated chains, source guards, unsupported versus
unused nodes, invalid/empty/unit cases, subdomains, final-budget cutoffs,
cancellation/retry, shared sources, replay and zero getter dispatch. The field
fixture retains field identity with reordered time/state bindings and consumes
its product in another exact polynomial map.

This is a new polynomial map grammar, not general model division/transcendentals,
direct PowI evaluation, independently centered model combination, adaptive QR,
ODE wrapping control or completion of Phase 13. Residual intervals do not establish
independence/equality; the specialized affine form remains available for its
stronger residual-coordinate composition contract. Shared graph/native budgets
apply, with existing scalar payload/refinement, total heap and within-scalar
cancellation exclusions. Stage entry checks do not claim a total memory bound.

The first ungated implementation grew the existing model consumer by 8,200 ELF
bytes and was rejected. Feature isolation, appended declarations and preserving
private range fields leave all allocated sections except build IDs identical in
three established consumers and eight owner bench/test executables. Complete sized
function layouts also agree after canonicalizing numeric LLVM disambiguators.
The field-only ELF is byte-identical. Raw file deltas are -8 bytes for the old
model consumer, zero for affine and 0/+8/+16/+40 for owner executables, attributable
to nonallocated symbol metadata. These are explicit additive-extension costs,
not a zero-size-growth refactor claim. Existing feature selections are fixed;
functional all-features checks additionally exercise the new feature. All three
consumer output/work/allocation/peak/live records and the full owner dispatch
report match. Incidental timings recorded during builds are not performance claims.

Three real maintained-benchmark trials complete all six callback workloads on
CPU 0 with ASLR disabled, one-second warmup, two-second measurement and 30 samples,
after builds/tests finish. Both paths materialize and drop their complete proof
for the same function/domain/order, with independent coefficient/extremum checks.
Seed models already exist before timing. The observed median ranges are:

| Case | Reuse (microseconds) | Fresh differentiation (microseconds) |
| --- | ---: | ---: |
| Centered logistic | 42.291–42.388 | 48.612–49.193 |
| Algebraic product | 53.913–54.207 | 42.895–43.051 |
| Reciprocal square | 73.637–73.807 | 259.477–261.305 |

The small algebraic case is slower with reuse; no blanket speedup is claimed.
Reuse allocates 10,066/12,850/9,166 bytes versus 24,950/21,984/129,116 for fresh
differentiation. Its retained and peak memory are higher in the first two cases
and lower in the reciprocal case; native work is also higher in the simple cases.
All measurements remain explicit in the archive. Warm construction/drop trials
return to zero incremental live bytes. A first cold retained-output observer
left 104 bytes while the original seed remained live; its failed assertion is
retained. Warming removes that incremental remainder. The archive narrative's
attribution to source caches is an inference from this stabilization; individual
allocation ownership was not traced. Warm measurements therefore do not establish
zero cold-cache growth, a whole-heap bound or an end-to-end seed-construction
speedup.

The final isolated revision passes 1,547 tests across 70 test/doc groups with no
failures or ignored tests, strict all-feature/all-target Clippy, warning-denied
docs, minimal/disabled feature builds, WASM and workspace formatting. Main's
combined extraction-plus-extension state additionally passes the eleven focused
core/field tests and formatting. The root default/benchmark feature graph does
not compile the changed Taylor module; no new successive_operations timing or
unified-workspace-all-features cost claim is made. Full pre-push workflows,
coverage/fuzz/sanitizer and remaining release qualification remain open. No push.

The 41-path pending extraction is preserved by an exact index-tree reversal proof
across the overlapping Hyperdynamics manifest/README; its new patch hash is
`c0b740cfaa6e6ae85aa409cd835fa5739160112cdb4a42693eaf97fc69672be5`.
Interface's eight edits and Firmware's clean state are unchanged. All 25 earlier
frozen stages verify. Evidence `docs/evidence/2026-11-14-interval-taylor-polynomial-arithmetic` at archive
`06d80b74a155c4bbec5d299a293f1b66b4121920` contains 351 files,
2,782,312 bytes, SHA-256
`084db5012b25b55b516676291510d4265bc135af872ca4d245e93c8df5dde471`.
All 351 committed blobs verify against this digest; the archive parent-tree diff
and manifest comparison preserve every older entry. A broader historical-blob scan
was stopped while fetching unrelated partial-clone objects and has no completion
claim. The stage is frozen. Phase 11 cost failures, remaining Phase 7–13 work, embedded qualification and
Phase 14 release obligations remain open under the unchanged full goal.

## Retain validated trajectory Taylor enclosures (2026-11-14)

Phase 13 gains opt-in `taylor-flow`, retained at
`bc2f1d00c0f97c6d3aa7b14036c6fe4298742ee7`. Hyperdynamics owns trajectory
construction; Hypersolve remains the derivative and analytic-domain authority.
The previously missing witness is a trajectory enclosure with a complete remainder
and an endpoint consumed by another step. Field-value Taylor models alone do not
establish that contract. The archive records the missing-API failure and the
normalized Lie-coefficient/integral-remainder derivation.

Preparation retains coefficients through order n+1, exact integer normalization,
source/derivative guards and flat shared stages. Certification first proves the
Picard tube and the field's time/state analytic domain, then encloses initial
coefficients and the next coefficient over the entire certified tube. Every term
and complete integral remainder contribute before intersection with independent
Picard endpoint/tube bounds. Order zero retains its first-order remainder. Reports
retain source identity, stages, both original proofs and all contributions; replay
reconstructs them from definitions. No foreign or deserialized certificate is
accepted. Invalid, empty, unit, rejected, exhausted and certified outcomes remain
distinct, with a final resource/cancellation check before publication.

Eleven new tests cover independent exponential/oscillator series bounds, exact
polynomial/triangular and rational Riccati flows, joint nonautonomous coefficients,
generated endpoint reuse, rational clocks and exact state charts, order zero,
empty/unit/invalid cases, rejected Picard and singular sources, time nonanalyticity,
one-unit-short budgets, cancellation/retry, zero getter dispatch and fresh-process
replay. Final isolated validation passes 79 tests in 13 groups, strict Clippy,
warning-denied docs, minimal/disabled feature builds, WASM and formatting. Main's
combined pending-extraction state passes 90 tests in 16 groups, strict Clippy,
formatting and repository budget. There are no failed, ignored or filtered tests
in these final runs. Benchmark Clippy/formatting pass after final refinements.
Package file listing checks inclusion only; it is not a release-build claim.

The old native, model and affine consumer executables are byte-for-byte identical
to same-path baselines at their existing feature selections. Their allocated
sections and canonical sized-function layouts also agree. No old-path timing
improvement or arbitrary-platform/all-feature cost invariance is claimed.

The maintained completed-workload benchmark requires x'=x, x(0)=1 on [0,1/4],
endpoint width <=1/8192, independent exact exponential-tail checks, retained proof
materialization and destruction. One order-three Taylor step and 1,024 Picard steps
both qualify. The Picard path uses the positive-linear self-map candidate and
explicit outward regridding with inclusion checks between steps. The 512-step
variant misses the width target and is recorded separately as underresolved.
Original field construction is outside both timings; Taylor preparation is
outside repeated-query timing and measured separately. Three CPU-0, ASLR-disabled
trials with one-second warmup, two-second measurement and 30 samples give:

| Completed case | Median microseconds | Allocated bytes | Retained bytes | Peak bytes |
| --- | ---: | ---: | ---: | ---: |
| Picard1024 horizon | 10,790.448–10,816.059 | 4,897,984 | 1,371,464 | 1,372,688 |
| Taylor3 query | 92.078–92.600 | 29,823 | 5,408 | 9,248 |
| Taylor order-three preparation | 22.069–22.417 | 30,296 | 9,520 | 10,320 |

Warm observers use the same operation/validation body and core dependency versions
and features as the maintained benchmark. All three construction/drop epochs
return to zero incremental live bytes. Prepared program/source storage is outside
the query epoch, with preparation shown separately. These measurements cover one
linear IVP and accuracy target, not an optimal or generally competitive solver
portfolio, zero cold-cache growth or a total heap bound. Preparation graph work is
634; Taylor query graph/scalar work is 654/1,048,685 under the recorded budgets.

The unrounded long-Picard draft exhausted at step 1,259 under the 16,384-bit native
limit. Its failed outcome and source remain archived. Explicit outward transport
addresses growth without enlarging the limit; no exhausted attempt counts as
completed geometry or a speed result. Draft compile/doc/observer-feature errors
and intermediate benchmark variants are also retained with their final fixes.

This sufficient analytic-domain verifier is not an adaptive step/order controller,
QR wrapping scheme, universal smooth-flow algorithm or whole-horizon Taylor solver.
Cartesian output drops correlations explicitly. Per-stage graph limits do not
bound total retained graph/heap; scalar payload/refinement and in-scalar cancellation
exclusions remain. Finite high orders can exhaust. General model arithmetic,
competitive portfolio qualification and broader adoption remain open.

Integration reverses this increment to recover the entire prior pending index
tree, including the overlapping manifest. Exactly 41 extraction paths remain
staged, with patch SHA-256
`fa7cae17b5b413c0e34601c118c4c4f42c1e37053ef544739e0ae161337a417f`.
Interface's eight edits and Firmware's clean state are unchanged. All 26 earlier
frozen stages verify. Evidence `docs/evidence/2026-11-14-interval-taylor-flow` at archive `c5fca8365bc6d7eed2c4f4725778e81c5007a919` contains
218 files, 1,357,970 bytes, SHA-256 `987d6643b8c9bdfecdd0ac921d7553c68e08766adf8427b4d95737def487f527`. The new committed blobs and
archive parent-tree scope verify; every older registry row is preserved. This
stage is frozen. No push or full pre-push workflow campaign occurred. Phase 11
cost failures, remaining Phase 7–13 work, embedded qualification and Phase 14
release obligations remain open under the unchanged full fifteen-phase goal.

## Retain direct exact chart transport (2026-11-14)

Phase 13 gains opt-in `coordinate-transition`, retained at
`86dbb4dd80d3997a5ca566a52c8e0c5cdff6ccaa`. Hypersolve owns the source-bound
relation between two validated charts; Hyperdynamics preserves their time/state
meaning and transports native boxes. For x=c0+A0*y=c1+A1*z, preparation forms
z=B1*A0*y+B1*(c0-c1) with the existing exact product-sum authority before any
interval projection. Both original source associations, strict inverse proofs,
exact coefficients and executable are retained. Independent structural source
reconstructions are admitted; foreign sources, variable orders and field
interpretations are rejected. This is a prerequisite for changing frames during
wrapping control; automatic frame selection and QR reduction remain open.

The transport proof owns its original input and exact map beside the Cartesian
outer enclosure. That enclosure is a sound enlarged initial set for the next
operation, not equality of the correlated image or an ODE existence claim.
Existing graph/native budgets apply with scalar payload/refinement/predicate,
whole-heap and within-scalar cancellation exclusions. Final checks precede
publication; no serialized proof flag is trusted. Empty and zero-dimensional
unit sets remain distinct. No old arithmetic or predicate sequence changes.

The missing-method witness fails on the baseline. Five core and seven field
fixtures then establish independent exact coefficient/point and width expectations,
algebraic scaling/translation, reconstruction and source rejection, empty/unit,
final work/native cutoffs, cancellation/retry, fresh-process full proof/work replay,
zero getter dispatch, and generated Picard/Taylor endpoints transported into a
changed chart and consumed by another step. Draft ownership/trace-test compile
errors and the corrected observer feature mismatch remain in the archive.

The isolated final revision passes 1,569 top-level tests in 71 test/doc groups,
with zero failures, ignored or filtered top-level tests. An existing replay child
runs one selected test separately; its parent runs all seven, and the child is
not double-counted. Strict all-feature/all-target Clippy, warning-denied docs,
minimal/disabled builds, WASM and formatting pass. Main's combined extraction
state additionally passes 97 Hyperdynamics tests, the five core transition tests,
the standalone minimal core feature, strict Hyperdynamics Clippy, formatting and
repository budget. Full pre-push/release/sanitizer/fuzz qualification is not claimed.

The old native consumer is byte-identical. The old Taylor-model, affine-model and
coordinate-evolution consumers have identical allocated sections except build IDs
and identical canonical function layouts. Their raw file deltas are 0, 0 and
-32 bytes respectively, confined to nonallocated symbol/string metadata. These
same-path, fixed-feature controls do not establish arbitrary-platform or unified
all-feature cost invariance; no old-path timing improvement is claimed.

Three maintained-benchmark trials each complete six transport workloads on CPU 0,
with ASLR disabled, one-second warmup, two-second measurement and 30 samples.
Exact charts and relations already exist before timing; every query materializes
and drops all transport proofs. Both strategies enclose the same exact affine
image, with independent expected bounds. The two-projection strategy uses the
same backend through a Cartesian source box; this is not an external solver
comparison or a claim that both output boxes are equal.

| Case | Direct median microseconds | Roundtrip median microseconds | Direct / roundtrip first-coordinate width |
| --- | ---: | ---: | ---: |
| Identity | 4.565–4.606 | 12.552–12.945 | 0 / 0 |
| Same shear | 4.521–4.594 | 13.986–14.351 | 0 / 512 |
| Changed shear | 6.204–6.352 | 13.749–13.990 | 128 / 384 |

Direct queries allocate 2,408/2,408/2,720 bytes versus 5,288/5,544/5,544 for the
round trip. Warm retained/peak heap and native work observations are explicit in
the archive; every construction/drop epoch returns to zero incremental live
bytes. Preparation separately allocates 3,432 bytes, retains 1,768 and peaks at
2,016 on each fixture. Preexisting source/chart/relation storage and stack values
are outside query heap epochs. These observations do not establish zero cold-cache
growth, a total heap/work bound, a universal width/cost improvement or an ODE
speedup. Observer operation bodies and core dependency versions/features agree
with the maintained benchmark; preparation is a separate measured supplement.

An alternate-index integration and exact reversal recover the complete original
pending tree. Exactly 41 extraction paths remain staged, SHA-256
`408c1215508fab22167f00f39d0829b8e006df5e4bfef72fea32062137134c5f`.
Interface's eight edits and Firmware's clean state remain unchanged. All 27 prior
frozen stages verify. Evidence `docs/evidence/2026-11-14-interval-coordinate-transition` at archive `ae8c65e44ced85227bd30461480d997b80ca7c70` contains 279
files, 1,172,257 bytes, SHA-256 `f2816c1d74198341387eb3e311446d6c07ffd3d04a421dacb1bca5c95156029b`; its committed blobs and parent diff scope
verify and every older registry row is preserved. The new stage is frozen.
No push occurred. Phase 11 cost failures, adaptive frame/QR and other remaining
Phase 7–13 work, embedded qualification and Phase 14 release obligations remain
open under the unchanged full fifteen-phase goal.

## Retain exact QR coordinate construction (2026-11-14)

Phase 13 gains opt-in `qr-coordinates` at `35bc68ed30cde1d267aa7af5e4776318aca2284b`.
Hypersolve constructs unpivoted exact QR from a caller-supplied finite square
basis and origin. Strict positive residual-norm decisions precede normalization;
the existing source-bound coordinate authority proves Q-transpose*Q=I, and a
separate exact reconstruction pass proves Q*R=M. Shared results retain the basis,
source/order/origin, Q, R and all strict decisions. Reflections and zero-dimensional
units are supported. Dependence, unknown, malformed input, scalar errors and
exhaustion remain distinct. Hyperdynamics consumes the admitted transformation
through its existing field/Jacobian and Picard/Taylor/transport/evolution APIs.
No old arithmetic or predicate sequence changes. Automatic basis selection and
adaptive whole-horizon QR/Lohner control remain open.

The baseline missing-API witness fails. Five core and seven field tests cover
independent rational 2D/3D and algebraic decompositions, near-dependence, unresolved
norms, invalid input, final cutoffs, cancellation/retry, strict receipts, zero
getter dispatch, fresh-process proof/work replay and QR-generated endpoints
transported into another chart and consumed by further Picard/Taylor steps.
Failed draft assertions expecting native singleton values for exact 3/5
coefficients are preserved. Tests now distinguish exact point equations from
outward native containment (width <= 2^-48), retain the tight candidate's rejection,
and certify a widened proposal. No production bypass or weakened existing test
is used. Draft compiler/observer failures and their corrections are also retained.

The isolated revision passes 1,581 top-level tests across 72 test/doc groups,
with zero failures, ignored or filtered top-level tests. An existing selected
replay child is counted separately. Strict all-feature/all-target Clippy,
warning-denied docs, minimal/disabled features, WASM and formatting pass. Main's
combined extraction state passes 104 Hyperdynamics tests, 5 focused core tests,
standalone minimal core feature, strict Hyperdynamics Clippy, formatting and
repository budget. Full pre-push/release/sanitizer/fuzz qualification is not claimed.

All five old-feature consumer controls preserve allocated sections except build
IDs and canonical function layouts. Their raw file deltas and metadata differences
are recorded in the archive. Same-path fixed-feature observations do not establish
arbitrary-platform or unified all-feature invariance; no old-path speedup is claimed.

Three maintained Criterion trials compare complete source-flow reports in identity
and QR charts, including all attempts, projections and destruction. Source initial
sets, horizons, independent exact endpoints and width <= 2^-48 acceptance are the
same; output box equality is not claimed. Preparation is measured separately.
Median microseconds across trials:

| Workload | Minimum | Maximum |
| --- | ---: | ---: |
| constant_flow/identity | 43.208 | 43.377 |
| constant_flow/prepare_qr | 12.111 | 12.300 |
| constant_flow/qr | 55.430 | 55.763 |
| rotated_equilibrium/identity | 111.140 | 111.737 |
| rotated_equilibrium/prepare_qr | 50.883 | 51.076 |
| rotated_equilibrium/qr | 55.277 | 55.701 |

Rotated equilibrium requires two identity segments/three attempts versus one QR
segment/attempt. QR reduces allocated bytes from 54,030 to 24,226, but native
work increases from 1,834,946 to 2,478,206. Constant flow uses one segment/two
attempts in both charts; QR allocation increases from 21,828 to 26,076 bytes and
native work from 619,950 to 1,434,334. QR preparation allocates 75,502/17,622 bytes
for these two fixtures. Every warm construction/drop epoch returns to zero
incremental live heap. Retained/peak storage and full graph/native work are
recorded. Exact scalar arithmetic, refinement/predicate internals, scalar payload
lifetime, whole heap and within-scalar cancellation remain outside the structural
budget contract. Prepared source/frame storage and stack are outside query heap
epochs. Zero native preparation work is a counter-scope fact, not zero arithmetic.
These measurements do not establish a universal speedup, total resource bound,
zero cold-cache growth or independent competitive-solver qualification.

Alternate-index application and exact reversal recover the entire original pending
tree. Exactly 41 extraction paths remain staged, SHA-256
`6713fe7fcebdd216a23d1ee5867701999f94ac6c772dcd50202d4648d9281e62`.
Interface's eight edits and Firmware's clean state remain unchanged. All 28 prior
frozen stages verify. Evidence `docs/evidence/2026-11-14-interval-qr-coordinates`
at archive `3c19d8596988b451f181167f9ff57b8cbf6afd39` contains 277 files,
1,207,112 bytes, SHA-256 `934a074acc5f447faf9177e943f3f7f119878880beca9f806e161943ec0df5bf`.
Committed blobs and parent diff scope verify; all older registry rows are preserved.
The new stage is frozen. No push occurred. Phase 11 cost failures, adaptive
step/order/frame control, remaining migrations and adoption, embedded qualification
and Phase 14 release obligations remain open under the full fifteen-phase goal.

## Retain whole-horizon trajectory Taylor composition (2026-11-14)

Phase 13 gains opt-in `taylor-evolution` at `d1ff299482dd25367d49cf4e3c243413c6f1d1ad`.
FlowTaylorProgram::integrate reuses the existing bounded Picard search with an
internal one-accepted-step limit and consumes its accepted proof by reference
in the existing Taylor verifier. Complete Taylor endpoints become subsequent
initial sets. No Picard predicates are repeated or entire Picard proof cloned.
Original source/order/request/options/policy, all rejected native attempts,
partial/completed analytic and remainder evidence, and accepted indices remain
retained. The internal StepLimit(1) is an explicit native prefix, not native
whole-horizon success. Taylor failure stops without fallback. Exact time equality
and final checks precede completed output. Prepared order is fixed; automatic
order/step-growth and frame control remain open.

The baseline missing-method witness fails. Eleven final focused tests establish
independent exact constant/nonautonomous and uncertain-initial flows, exponential
bounds from a rational series, tighter endpoints than Picard and actual chaining,
QR-generated field evolution followed by changed-chart transport and further
evolution, retained halvings/failures, empty/unit/identity and invalid-empty cases,
qualified prefixes, analytic failure without fallback, final graph/native cutoffs,
aggregate nested-record exhaustion, cancellation/retry, zero getter dispatch and
fresh-process complete proof/work replay. Eight tests also pass with only the
minimal new feature. Existing tests and mathematical thresholds are preserved.

The all-feature Hypersolve/Hyperdynamics run passes 1,591 top-level tests in 73
test/doc groups, with no failures, ignored or filtered top-level tests. Its
existing selected replay child is counted separately. This full run compiled
the preceding ten-test version; the final added aggregate-record regression is
covered by the 11-test focused run. Production code did not change after full
compilation. Strict all-feature/all-target Clippy, warning-denied docs,
minimal/disabled builds, WASM and formatting pass on final source. Main's combined
extraction additionally passes 115 Hyperdynamics tests, 8 minimal-feature tests,
strict Clippy, formatting and repository budget. Full pre-push/release/platform/
sanitizer/fuzz qualification is not claimed.

Three old-feature controls cover native evolution, trajectory Taylor steps and
QR evolution. Allocated sections except build IDs and canonical function layouts
match; raw metadata differences are recorded. Same-path fixed-feature controls
do not establish arbitrary-platform or unified all-feature invariance.

The maintained completed benchmark compares x'=x, x(0)=1, horizon 1/2 and endpoint
width <= 1/512, with an independent rational bracket for exp(1/2). Original Picard
uses 2,048 segments/4,096 attempts; Taylor uses two segments/two searches/four
native attempts. A constant x'=2 flow uses two segments/four attempts in both
methods and returns exact singleton one. Complete reports, all proofs and
output destruction are measured; preparation is separate. Both use the same
backend, not an external competitive solver. Three trials give these median
microsecond ranges:

| Workload | Minimum | Maximum |
| --- | ---: | ---: |
| constant/picard | 31.575 | 31.696 |
| constant/prepare_taylor3 | 10.727 | 10.873 |
| constant/taylor | 127.612 | 128.194 |
| exponential/picard | 170734.985 | 172609.394 |
| exponential/prepare_taylor3 | 21.634 | 21.669 |
| exponential/taylor | 255.594 | 257.307 |

The default scalar-size limit exhausts on the many-step Picard comparison; that
failed probe is retained and excluded from completed timing claims. Both measured
methods use precision -64, a 1,048,576-bit scalar stage limit and 10^14 native work
units with default graph limits. This changes the benchmark envelope, not source,
horizon, quality threshold or production defaults. The allocation observer's
initial num-bigint 0.4.8 resolution mismatched the maintained 0.4.6 benchmark;
original results and lock correction remain archived. Final observations use
matching versions/features and operation bodies.

Matched warm allocations are 493,194,344 versus 80,132 bytes for exponential
Picard/Taylor and 10,740 versus 56,052 for constant flow. Retained/peak heap is
123,685,072/123,732,912 versus 22,224/26,256 bytes for exponential flow; constant
flow is 4,552/5,792 versus 18,160/21,408. Every complete warm construction/drop
epoch returns to zero incremental live heap. Graph/native work and preparation
costs are fully recorded, including exact dyadic growth in the compared Picard
strategy. These are completed fixture results, not a universal speedup, optimality
claim or zero cold-cache-growth guarantee.

Aggregate stage-entry checks include nested search and accepted-index records,
but are not whole retained-graph or peak-heap reservations. The last search is
retained in a stopped report when the post-search aggregate check fails. Scalar
payloads, exact arithmetic/refinement, predicate internals, vector capacity and
within-scalar cancellation retain existing exclusions. Preexisting source and
coefficient storage and stack are outside query heap epochs. Zero native
preparation work excludes exact Real arithmetic; it is not zero computation.

Alternate-index application and exact reversal recover the complete original
pending tree. Exactly 41 extraction paths remain staged, SHA-256
`9641b4df9a65e2fcd0c3c10c305a164c8f7d3e80c0a1b73ebf5711e299e69fe5`.
Interface's eight edits and Firmware's clean state remain unchanged. All 29 prior
frozen stages verify. Evidence `docs/evidence/2026-11-14-interval-taylor-evolution`
at archive `4f9ed8b692ffd723f2f87fcddc57fca2c143fc1e` contains 265 files,
1,205,425 bytes, SHA-256 `4c7461ffea9c22ad22dc4eb0768c2f031a0281d28c8eb73c4a0b1d80157e2786`.
Committed blobs and parent diff scope verify; all previous registry rows are
preserved. The new stage is frozen. No push occurred. Phase 11 cost failures,
automatic order/frame control, remaining migrations/adoption, embedded
qualification and coordinated release remain open under the full goal.

## Qualify existing regridded reachability in the dynamics portfolio (2026-11-14)

Source `2fa15af0fc55a1183a34fe158ec10bdcd7bc402e` extends the maintained completed-flow comparison
and acceptance corpus. Current source review found that existing reachability
already controls the dyadic growth exposed by ordinary Picard. Reuse that
implementation: no production source, public API, algorithm, dependency/default
policy or predicate sequence changes. Only the benchmark, its required-feature
list and reachability tests change. Nine existing benchmark helpers remain
byte-identical, and the complete previous test file remains an unchanged prefix.
The full fifteen-phase objective remains active.

The retained hard-case witness reproduces ordinary Picard's default scalar-size
exhaustion for x'=x, x(0)=1, horizon 1/2 and cap 2^-12. Existing reachability
completes all 2,048 prefixes under default resources with one cell/evolution and
no partition/time-refinement search. Every transport inclusion, policy grid,
continuation, horizon and endpoint width is checked. Independent rational series
bracket exp(1/2); the resulting endpoint feeds a Taylor evolution to 3/4 with
another independent bracket. Default and larger-budget outputs and spent work
agree. No exhaustion is counted as geometric completion.

A finite cap probe rejects 2^-8, 2^-9 and 2^-10 at the unchanged width <=1/512
contract and completes at 2^-11 and 2^-12. A second regression retains the failed
512-segment control and successful 1,024-segment case. The benchmark uses 2^-11,
the coarsest successful cap among those probes, while preserving the 2,048-step
growth regression. This is not a globally optimal cap or automatic selector.

Final isolated qualification passes 106 Hyperdynamics tests in 14 test/doc groups
and 11 minimal-reachability tests, strict all-feature/all-target Clippy,
warning-denied docs, disabled features, WASM and formatting. Main's combined
extraction passes 117 tests, 11 minimal-feature tests, strict Clippy,
formatting and repository budget. The initial new-test assign-op Clippy finding
and copied probe's unused-helper warnings were fixed; original logs/draft remain.
No existing assertion or endpoint-quality threshold was weakened. Full pre-push,
release, platform, sanitizer/fuzz and external-solver qualification are not claimed.

The completed portfolio compares ordinary Picard, existing reachability and
Taylor on the same source/initial set/horizon/endpoint-quality contract. Reachability
also retains a single-box epsilon-cover certificate, every native prefix and
checked outward transport. Proposal padding and transport both differ from ordinary
Picard; this is not an ablation attributing all effects exclusively to rounding.
The constant flow x'=2, x(0)=0 still returns exact singleton one after horizon 1/2
in two segments/four attempts for every strategy. It preserves unfavorable overhead.
Three trials give median microseconds:

| Workload | Minimum | Maximum |
| --- | ---: | ---: |
| constant/picard | 30.888 | 31.353 |
| constant/prepare_taylor3 | 10.644 | 10.857 |
| constant/reachability | 40.856 | 41.736 |
| constant/taylor | 127.156 | 127.715 |
| exponential/picard | 113788.458 | 120606.745 |
| exponential/prepare_taylor3 | 22.111 | 23.125 |
| exponential/reachability | 23520.364 | 23565.227 |
| exponential/taylor | 253.977 | 254.515 |

Every method uses the same benchmark envelope: precision -64, scalar stage limit
1,048,576 bits, 10^14 native work and default graph limits. Reachability's default
completion is separately qualified. Complete reports, proofs, required output and
destruction are measured; common source preparation is excluded and Taylor
preparation is separate. All 24 trial workloads complete their independent checks.
The observer matches maintained operation bodies and dependency versions/features.
The six earlier warm allocation/retained/work rows reproduce exactly; this does
not assert cross-revision timing or arbitrary-platform invariance.

For the exponential fixture, reachability allocates 9,894,080 bytes, retains
5,200,760 and peaks at 5,200,768, versus ordinary Picard's 493,194,344 allocated,
123,685,072 retained and 123,732,912 peak, and Taylor's 80,132 / 22,224 / 26,256.
Reachability uses graph/native work 107,526/126,125,643 and 1,024 prefixes/2,048
native attempts. Constant-flow reachability allocates 20,180 bytes, retains
13,640 and peaks at 13,648, with work 212/140,878. Every warm construction/drop
epoch returns to zero incremental live heap. All other rows remain in the archive.
These completed fixture results do not establish general resource closure,
zero cold-cache growth, a universal speedup or external competitive performance.
Existing scalar payload/refinement/predicate, vector-capacity and within-scalar
cancellation exclusions remain. Preexisting source/coefficient storage and stack
are outside query heap epochs; zero native preparation work excludes exact Real
arithmetic. No whole nested-proof-graph or heap bound is inferred.

Alternate-index application and exact reversal recover the complete original
pending tree. Exactly 41 extraction paths remain staged, SHA-256
`676e5d602a1fbdcc3fc7ff0d15bfb62692ef03f69a5a62f957aa7c7b6697b359`.
Interface's eight edits and Firmware's clean state remain unchanged. All 30 prior
frozen stages verify. Evidence `docs/evidence/2026-11-14-interval-dynamics-portfolio-cost`
at archive `0036f24108fc524548b3e902f141d866db7b9a3a` contains 284 files,
668,093 bytes, SHA-256 `c64e0e17f50022cf08709ab58dbf6a45a8047ddc8c3265b94b2b909de817b439`.
Committed blobs and parent diff scope verify; all previous registry rows are
preserved. The new stage is frozen. No push occurred. Phase 11 cost failures,
remaining migrations, automatic order/frame selection, wider adoption, external
competitive qualification, embedded work and coordinated release remain open.

## Recheck current extraction costs and diagnostic cache models (2026-11-14)

Phase 11 remains unretained. At current revision `04286c458d15b356b305d5805bfe04be659ba246`, all three
real-owner executables (baseline, exact pending extraction, earlier provider-error
control) reproduce their previous hashes byte for byte. The separate observer's
full 194-workload body, external dependencies, completed Debug outputs and all
allocation/deallocation/reallocation/bytes/peak/live rows agree across variants.
This is not an independent geometry oracle or canonical serialization proof.
No production change is retained, and this diagnostic is not implementation
progress toward a completed phase.

Six longer CPU0/ASLR-disabled sweeps repeat the same six previously published
failures in baseline/extraction/provider/provider/extraction/baseline order,
with one-second warmup, two-second measurement and thirty samples. No concurrent
builds, tests or profiles run during timing. All six provider failures reproduce;
extraction retains adverse separation for hodograph, pad clearance and parser.
Both candidate lower mean confidence bounds exceeding both baseline upper bounds
is a screen, not family-wise significance or a permitted regression tolerance.

| Case | Extraction/baseline mean | Provider/baseline mean |
| --- | ---: | ---: |
| `axis_aligned_line_offset_exact` | 0.98653 | 1.03759 |
| `quadratic_bezier_exact_hodograph` | 1.03227 | 1.10136 |
| `rect_pad_board_edge_clearance_exact` | 1.02374 | 1.06468 |
| `rectangular_additive_bead_schedule` | 0.96811 | 1.04829 |
| `specctra_grid_route_text_parse_256` | 1.05099 | 1.02477 |
| `specctra_trace_record_exact_import` | 0.98476 | 1.03869 |

Hardware counters are unavailable (`perf_event_open` returns EACCES). Eighteen
cache/branch simulations measure 32 warmed completed/drop callbacks in the
separate observer, using explicit generic 32 KiB eight-way I1/D1 and 8 MiB
sixteen-way LL models. All six instruction totals reproduce the prior record;
decoded profile hashes and per-function event sums verify. Simulated events vary,
but do not consistently explain hardware timing: parser I1 misses remain six and
conditional branch misses decrease from 370,371 to 362,085/365,636 while wall time
increases. Offset and exact-import provider controls have fewer simulated I1
misses despite slower timing. The model omits the physical sliced cache hierarchy,
TLBs, prefetchers and execution scheduling, and has different code generation
from the real timing executable. It establishes neither a hardware cause nor a
source fix. No arbitrary layout hint or unrelated mathematical change follows.

Initial short hardware-counter attributes, an unrepresentable physical aggregate
cache setting and obsolete provider-patch changelog context failed preflight;
logs remain. Two builds accidentally launched after the patch failure repeated
extraction and are quarantined, excluded from timing/profiles. The implementation-
only provider patch was then rebuilt and its real-owner hash verified before
measurement. The corrected cache model completed all 18 profiles.

Both provider fixtures pass. Restored extraction passes three routing tests
including fresh-process two-policy reports/dispatch, one axis migration test,
workspace formatting and staged whitespace checks. The isolated tree is restored
clean. All four authoritative states match their starting HEAD/status/patch hashes
before archival; main's 41 staged paths and Interface's eight edits are preserved.
All 31 prior frozen stages verify unchanged. No new full-owner/Clippy, downstream
cost, successive_operations, sanitizer/fuzz, WASM, packaging, HIL or pre-push
qualification is claimed. No push occurred. Repeating this same diagnostic is
not a next implementation step; pursue a distinct supported hypothesis or another
open plan task while retaining this cost blocker.

Evidence `docs/evidence/2026-11-14-interval-dynamics-layout-recheck` at archive
`eed20ff24447bacc625cfb9e802e5aa59ca236ee` contains 295 files,
21,374,597 bytes, SHA-256 `a5bfb063c2b0a75207c1daa62202da0f2dcde46f17f80984a2d3cc0fe6793e23`.
Committed blobs and parent diff scope verify, all prior registry rows are preserved,
and this stage is frozen. The full fifteen-phase goal remains active.

## Certified Taylor model composition (2026-11-14)

Phase 13 gains opt-in `taylor-composition` at production commit
`55fe5251be50c341bcf70abcfc550de7a83a76a4`, with a final acceptance-only follow-up
at `fde51f4dd49735ae324788befebda526eefab261`. The owning Hypersolve layer
implements `Arc<TaylorModel>::compose` and `TaylorPolynomialModel::compose`.
The reproduced missing-consumer witness, admission contract and implementation
are retained in the evidence record. Existing defaults and raw polynomial grammar
remain unchanged.

For inner f in P+R and outer g in Q+S, prove the full input image lies inside the
outer certificate domain, propagate Q(f) through the existing polynomial engine
as T+U, then retain T+U+S. This preserves both source certificates, exact outer
coefficients and center, inner parameters/order, truncated terms, cross residuals
and product evidence. The supplied outer center need not lie inside the inner image;
result polynomial coefficients need not be the exact composite derivatives.
Domain/binding validation precedes empty success. Failed enclosure containment is
not proof that the underlying actual function is undefined. Generated results feed
further certified-model and polynomial operations, including subdomain queries.
Original definitions and requests rebuild certificates in fresh-process replay.

The full Hypersolve/Hyperdynamics run passed 1,603 top-level tests in 75 groups,
including the preceding eight-test composition corpus. Final focus passed ten new
composition tests plus ten existing polynomial tests; minimal features passed eight
applicable new tests. Tests cover independent exact coefficient expectations,
outer and nonlinear inner residuals, higher-degree truncation, mixed/unsorted
bindings, shifted centers, empty/unit/zero outputs, domain/shape rejection,
resource cutoffs, cancellation/retry, source preservation, fresh replay and
zero-dispatch proof getters. Main with the pending extraction passed 20 focused
owner and 117 downstream tests with composition explicitly enabled. Strict
all-feature/all-target Clippy, warning-denied docs, minimal/disabled feature and
WASM checks, formatting and repository budget passed. The final follow-up changes
only a test; final focused checks, test Clippy and formatting passed afterward.
No previous assertion, fixture or dispatch threshold was relaxed.

Three final CPU0/ASLR-disabled trials completed all 24 measurements against the
same function/domain/order/endpoint-quality contract, including output and proof
construction and destruction. Median ranges in microseconds:

| Case | Prepared reuse | Fresh differentiation | Inner preparation | Outer preparation |
| --- | ---: | ---: | ---: | ---: |
| nested reciprocal | 142.857–143.453 | 256.714–257.191 | 124.388–125.213 | 120.176–120.371 |
| algebraic square root | 147.599–148.425 | 186.601–189.171 | 20.982–21.082 | 186.968–190.548 |

Both paths validate exact coefficients under STRICT and APPROXIMATE_512 without
approximate certainty, analytic extrema and identical materialized ranges. This is
prepared reuse, not a one-off cold speedup. Reuse allocates fewer cumulative bytes
but retains more proof memory: reciprocal reuse/fresh retained 21,052/12,624 bytes,
and square-root reuse/fresh retained 19,956/8,712. All observed construct/drop epochs
return to zero incremental live heap. Previously retained sources/caches are excluded.

Six established polynomial workloads show no adverse separation in final ABBA
sweeps. Disabled-feature .text/.rodata, allocated section layouts and sized-function
unions match; changed allocated bytes are verified panic source-line metadata and
build ID. Raw binary size decreases 16 non-code bytes. Enabling composition adds
an observed eight allocated/retained/peak bytes per existing raw-map stage while
preserving completed ranges, allocation counts and graph/native work. This opt-in
cost buys capability; it is not presented as a cost-free refactor. Initial artifact
parser/preflight failures and earlier measurements remain in the archive.

Graph/native construction budgets are shared and checked before publication.
Scalar payload/refinement, within-scalar cancellation, vector capacities and total
live heap remain outside a universal resource claim. Independent-base arithmetic,
automatic order/step/frame/QR control, wider adoption, competitive qualification,
Phase 11's measured extraction regressions, embedded gaps and release remain open.
No full pre-push, external solver, sanitizer/fuzz, hosted platform or HIL campaign
is claimed. No push occurred; no phase is marked complete.

Alternate-index aggregate application and exact reversal preserve all 41 staged
extraction paths, patch SHA-256
`676e5d602a1fbdcc3fc7ff0d15bfb62692ef03f69a5a62f957aa7c7b6697b359`.
Interface's eight existing edits and Firmware's clean state are unchanged.
All 32 previous frozen stages verify unchanged. New frozen record:
`docs/evidence/2026-11-14-interval-taylor-model-composition`, 937 files / 1,785,163 bytes,
SHA-256 `5154a6d9d351df6bd5db024a8064fc0d488ffc671f10043875f78d5c8c9a6d34`,
archive commit `96c5c0259f27b62e8fe84ff423ff41153a252971`. Committed blob digest and
addition-only archive scope are verified. The record includes reproduction
instructions; copy frozen drivers to new mutable stages before running them.

## Circuit residual/Jacobian consumer (2026-11-14)

Phase 13 gains opt-in Hypercircuit `interval-residuals`, retained at
`83b608d92c7d5fdaec1dd1e146709948f33e1348`. A missing-consumer compiler witness
was reproduced before implementation. `CircuitResidualProgram` retains exact
Hypersolve residual definitions, ordered circuit unknowns, component parameter
nominal values/units/provenance and equation labels. Input symbols/counts and
physical identities are validated; duplicate bindings cannot silently become
independent tolerances. Repeated occurrences share one symbol. Dependent physical
parameters require shared latent-input expressions. Metadata remains an authored
association, not verification of external circuit equations or dimensional units.

`LinearMnaSystem::residual_program` snapshots its existing exact matrix/RHS and
unknown order into that same carrier. It does not restamp or solve the circuit,
change point replay, or invent original stamp provenance absent from the matrix.
The source remains unchanged if public MNA fields are later mutated. Hypersolve
owns all derivative and full-domain continuity/C1 proofs; Hyperinterval owns native
bounds; Hyperlattice owns matrix projections and subsequent arithmetic. The adapter
adds no scalar, derivative or geometry kernel.

Reports retain source, arguments, precision, circuit metadata, residuals and every
input's derivative column, including parameter sensitivities. `Excluded` proves
one residual cannot vanish anywhere in the box; `NotExcluded` proves neither root
existence nor uniqueness. Empty products, unit products, invalid inputs, guard
failures and exhausted/cancelled attempts remain distinct. Domains are explicitly
supplied dyadic products; no silent conversion of an exact physical tolerance occurs.

The final combined checkout passes 117 core circuit tests in 48 groups, including
11 new cases. Independent expectations cover resistor tolerances, all linear stamp
families, algebraic coefficients/positive-square isolation, exact corner derivatives,
source snapshots, duplicate/shape failures, shared parameters, generated derivative
sources used as subsequent inputs, downstream matrix/norm operations, empty/unit/zero
rows, nonlinear inverse-Shockley domain guards, typed causes, resource limits,
one-unit-short final work, cancellation/retry, fresh-process definition replay,
invalid source references and zero-dispatch getters. Strict all-feature/all-target
Clippy passes isolated and combined checkouts. Warning-denied docs, disabled-feature
and WASM builds, formatting and repository budget also pass. The earlier isolated
core run had 116 tests before the final algebraic-source witness was added.

An initial test incorrectly assumed a minimal quotient-derivative hull. Existing
Hypersolve generates R/R² for the V partial of V/R, enclosing [1/4,2] on R in [1,2]
although the analytical image is [1/2,1]. The final test retains the original
source/domain, asserts the conservative result, independently requires containment
of the analytical range and checks exact point derivatives. No lower-layer rule,
policy, predicate order or existing test changed. Initial logs and fixture API
corrections remain archived; no tightest-hull or exponential-node claim is made.

Three isolated CPU0/ASLR-disabled trials each complete 10,000 operations per mode
on V/R-1 with V in [3,4], R in [1,2] and precision -16. Full residual/Jacobian/source
and metadata evidence and result destruction are included. Both strategies require
the same materialized bounds and exclusion. Mean microseconds per operation:
preparation 7.245–7.262, prepared query 14.808–14.845, preparation plus query
27.287–27.305. This demonstrates prepared reuse on this contract, not a universal
or one-off cold-call speedup.

The external observer copies the maintained benchmark body with dependency versions
matched to the workspace. Preparation/query/preparation-plus-query allocate
8,985 / 3,740 / 12,725 bytes per completed operation and peak at
2,876 / 1,984 / 3,612 incremental live bytes. A retained query uses 656 bytes and
101 graph / 21,676 native work units; preparation adds 1,628 retained bytes over the
already-held original source/metadata. All measured drops return to zero incremental
live heap. Preexisting source/cache memory and allocator/platform overhead are excluded.
Graph and native budgets do not bound metadata strings, exact scalar payload/refinement,
total heap or cancellation within scalar/native operations.

The default existing MNA benchmark is byte-identical to baseline, 3,223,768 bytes,
SHA-256 `84c6acdc7de86472da7b53d31367a7d446a68c1e9a04c9afc9f4f452b0037304`.
Enabling the feature adds 21,408 executable bytes and 80 .text bytes, with unchanged
.rodata size. Five established workloads complete in six controls with no adverse
min/max separation; visible timing drift limits that screen and is not a speedup
claim. The optional feature's memory/code cost is explicit.

Alternate-index application and exact reversal recover the original pending tree.
All 41 staged extraction changes remain. Adding the optional lockfile dependency
changes only their diff blob IDs/hunk offsets: normalized patch content is identical.
The new pending SHA-256 is
`e52d18c6c53a33a3037b977bb3f718eca60ff766f474b024748897b7ffef4273`;
the original was `676e5d602a1fbdcc3fc7ff0d15bfb62692ef03f69a5a62f957aa7c7b6697b359`.
Interface's eight edits and Firmware's clean state are unchanged; all 33 prior
frozen stages verify. New frozen record `docs/evidence/2026-11-14-interval-circuit-residual-enclosures` contains
88 files / 1,485,121 bytes, SHA-256
`408947e45b5459e1cb26ffb6029fc3e02622718ca0a39e71ec3561e7d2214dbe`, archive commit
`861b707f7313f24c54b1f77c7e6703c910575217`. Committed blob digest and addition-only
archive scope are verified; reproduction instructions require copying frozen drivers.

Automatic device lowering, nonlinear root/existence/uniqueness proofs, transient
circuit reachability and other consumers remain open, alongside automatic
Taylor/frame evolution, Phase 11 extraction costs, embedded gaps and release.
No all-feature geometry integration test campaign, full pre-push matrix,
coverage/fuzz/sanitizer campaign, hosted-platform or HIL qualification is claimed.
No push occurred. The full fifteen-phase goal remains active.

## Continuous point-mass flow consumer (2026-11-14)

Phase 13 gains opt-in Hyperphysics `certified-flow`, retained at
`6fade18056dc7e63865def845aae091d4255c413`. A missing-consumer E0599 witness
was reproduced before implementation. A fixed strictly positive exact mass and
three exact forces F(t,x,v) become the six-state continuous field x'=v, v'=F/m.
The immutable model retains the original validated force graph, arbitrary ordered
seven-symbol interpretation, strict mass-sign receipt and provenance. Constant-force
construction uses the existing accumulator and snapshots each source/vector.
Original references are validated before generated nodes are appended; all source
guards remain active. Authored physical units, frames and force-law validity are
assumptions, not claims established by interval verification.

Hyperphysics assigns physical meaning; Hypersolve owns source validation and
derivatives, Hyperinterval/Hyperlattice own set arithmetic and Hyperdynamics owns
Picard/evolution verification. No new integrator or scalar kernel is introduced.
Existing Euler replay retains its original discrete meaning. Step/evolution wrappers
retain the original model and complete native policy/proposal/source/evidence, typed
failures and attempt history. Certified endpoint boxes feed subsequent steps and
whole-horizon evolution. A stopped certified prefix is separate from completed
horizon success. Empty state sets do not bypass model/time validation.

The new opaque graph-budget view forwards bounded graph operations through the
existing counter without permitting reset/replacement or configuration mutation.
Construction, field preparation and subsequent verification share one budget.
Exact scalar refinement/arithmetic, source strings/payload, total heap and cancellation
inside scalar/force-accumulation operations remain explicit exclusions. Exhausted
construction publishes no model; retry does not inherit a poisoned source/cache.

Ten new tests cover independent continuous constant-force expectations versus
velocity-first Euler, uncertain initial states, repeated steps/full horizons,
nonlinear/time/velocity forces and exact Jacobians across seven binding orders,
source-reference/shape/domain failures, algebraic and unresolved mass receipts,
empty versus invalid inputs, final one-unit-short work limits, cancellation/retry,
stopped prefixes, native exhaustion, fresh-process exact input replay and zero-dispatch
getters. The isolated owner run passes 189 tests; the combined main checkout passes
200 owner tests and 42 downstream MNA/simulation/nonlinear tests. Strict owner
all-feature/all-target Clippy, benchmark Clippy, warnings-denied docs, disabled-feature
and WASM builds, formatting and repository budget pass. Initial compiler/test and
measurement-driver mistakes remain recorded; no acceptance expectation was weakened.

Three CPU0/ASLR-disabled 10,000-operation trials with 16 warmups per mode measure
m=2, F=[4,-4,0], h=1/4, including materialized tube/end/source/mass/provenance/proof
and result destruction. Mean microseconds per operation: preparation 6.517–6.582,
prepared verification 32.785–33.050, preparation plus verification 41.468–41.575.
Reuse includes prepared source/cache benefits; it is not a universal cold-call speedup
or a comparison to discrete replay. The complete benchmark-body allocation observer
uses matching external dependency versions. Preparation/reuse/combined allocate
7,576 / 15,004 / 22,580 bytes per operation and peak at 4,182 / 7,048 / 10,974
incremental bytes. Preparation retains 3,926 bytes beyond the held source; a step
retains 328 and uses 449 graph / 316,536 scalar units. Preparation consumes 467 graph
units; scalar positivity remains outside that accounting. Every measured drop returns
to zero incremental live heap; existing source/cache and allocator overhead are excluded.

Six physics control runs preserve all sixteen fixtures/checksums. Default executable
size remains 3,935,656 bytes with unchanged .text/.rodata sizes and all 1,702 symbol
size multisets; moved symbols prevent byte identity. The limited two-run min/max
screen observes no default adverse separation, with visible drift and no statistical
no-regression claim. Enabling the feature adds 21,816 executable bytes (+10,056 GNU
size text, -3,184 .text bytes) and an unfavorable slab-conduction control:
baseline 2.005–2.206 µs, default 1.997–2.015 µs, enabled 2.314–2.350 µs.
This opt-in code cost and observed regression are explicitly accepted for the
higher-priority added continuous certification capability; no blanket enabled-feature
no-regression claim is made. Hyperdynamics' existing Taylor benchmark grows 16 raw
bytes but has byte-identical .text/.rodata/.data.rel.ro, unchanged symbol groups/sizes
and addresses. Four actual Criterion constant-flow trials and all exponential/constant
completion checks are preserved; timing drift despite identical sections limits
interpretation of sequential significance labels.

Alternate-index application and exact reversal recover the original pending tree.
All 41 staged extraction paths and their logical changes remain identical; only
blob IDs/hunk offsets change. The pending SHA-256 changes from
`e52d18c6c53a33a3037b977bb3f718eca60ff766f474b024748897b7ffef4273` to
`d0f3f56e94d54d5f7330111f7bd6051cfe2caaf7ea23b21910b9b2cfaf69f6c0`.
Interface's eight edits and Firmware's clean state are unchanged. All 34 previous
frozen stages verify unchanged. New record `docs/evidence/2026-11-14-interval-physics-flow`
contains 110 files / 1,817,565 bytes, SHA-256
`69b2698c4147a8dc33e78c618ef3d6b267929b66bb8fbdfec52d0668c1d0485a`, archive commit
`1e80e3579b838ecf0acb7b0d3ed6579b521dfac1`. Its committed blob digest and addition-only
scope are verified. Reproduction instructions require copying drivers to mutable stages.

Variable/uncertain mass, rotations, contacts/impulses, coupled bodies, automatic
physical-law lowering and wider consumer adoption remain open, alongside automatic
Taylor/order/frame evolution, Phase 11 timing regressions, embedded flash/whole-call
stack/cycles/HIL, outstanding Interface Clippy issues and coordinated release.
No full pre-push/workflow, fuzz/coverage/sanitizer or platform qualification is claimed.
No push occurred. Phase 13 and the full fifteen-phase goal remain active.

## Bounded adaptive Taylor order and duration selection (2026-11-14)

Phase 13 and plan 10.7 gain opt-in `adaptive-taylor`, retained at
`eb8261c888bcb3fdbec0e6c1ad45c88b72fcf1ca`. The absent consumer was reproduced
as E0599 before production edits. `field.adaptive_taylor(options).integrate`
tries a finite ascending order range on each original certified Picard proposal.
The first order meeting all exact coordinate limits on absolute endpoint remainder
contributions advances the state. Nonzero singleton remainders do not pass a zero
allowance merely because their interval width is zero. If every order fails quality,
bounded exact halving retries. Accepted durations may double for the next proposal,
capped by the configured maximum and exact horizon. Native existence-search and
quality-search halving limits remain separate.

The controller retains original source/options/request/policy, every native search,
ordered Taylor refinement, complete/partial proof status, magnitude enclosure and
quality decision. Native or Taylor domain/resource failures stop with their original
cause; no weaker Picard success is substituted. Prepared programs are generated once
per distinct order within a run and shared by index across refinements. The existing
coefficient generator still prepares different orders independently. Only completed,
quality-checked endpoints advance the state; stopped prefixes remain separate from
whole-horizon success. Replay rebuilds the source, request and selection options.
Local truncation limits do not bound global solution error or uncertain initial state,
and no exact reachable-set equality or optimal-order/runtime claim is made.

Existing native/Picard/fixed-order evolution files are byte-identical; the original
Taylor-flow implementation remains an unchanged prefix with an appended gated module.
Hyperdynamics owns selection/proofs, while Hypersolve/Hyperinterval/Hyperlattice retain
derivative/scalar/set authority. One budget spans preparation, native searches,
refinements, comparisons and publication. Aggregate stage record checks retain typed
exhaustion and explicit partial history. Nested proof-graph size, exact scalar payload/
refinement, vector capacity, whole heap and within-scalar cancellation remain excluded
from a whole-resource claim; these gaps are not renamed completed support.

Ten focused cases pass: independent constant flow and singleton-remainder rejection;
order escalation/program reuse; quality halvings and later regrowth; uncertain
exponential flow with independent rational bounds and endpoint continuation; oscillator
alternating-series bounds; empty/unit/identity and invalid configuration/time; quality
versus native limits and stopped prefixes; original analyticity failure after native
success; all graph-work cutoffs below completion, exact final graph/native limits,
small structural limits and cancellation/retry; fresh-process source/request/options
replay; and zero-dispatch getters. Initial compiler diagnostics remain archived without
weakened mathematical expectations, source domains, policies or limits.

The isolated owner suite passes 116 tests in 15 groups; the combined main checkout
passes 127 owner tests and nine downstream physical-flow tests. The latter command
does not enable its tenth dispatch-gated test. Strict all-feature/all-target owner
Clippy, warnings-denied docs, disabled features, WASM, formatting and repository budget
pass. No full pre-push/workflow, fuzz/coverage/sanitizer, hosted-platform, HIL, external
solver or coordinated release qualification is claimed.

The maintained completed-workload benchmark includes fresh preparation, integration,
local remainder checks, materialized source/proof/endpoints, independent expectations
and destruction. Fixed order three and automatic orders zero through three use the
same source, initial state, horizon 1/2, cap 1/4 and local limits 1/1024. End enclosures
match; exponential width must be at most 1/512 and include independently bounded
exp(1/2). Three CPU0/ASLR-disabled trials of 1,000 operations after 16 warmups give:

| Fixture | Fixed order 3, µs/op | Automatic orders 0–3, µs/op |
| --- | ---: | ---: |
| constant | 143.465–144.054 | 150.057–151.028 |
| exponential | 304.510–306.189 | 644.752–645.664 |

Automatic selection is slower on both fixtures. Its rejected-order work and retained
history are explicit costs accepted for the higher-priority new automatic selection,
halving/regrowth and certified local-quality capability. This is not a speedup or
performance-only refactor; fixed-order programs remain available. The full benchmark-
body allocation observer uses matching dependency versions and returns to zero
incremental live heap after all 32 warm operations per mode. Allocated bytes/op are
constant fixed/adaptive 74,692/63,230 and exponential 114,580/238,880. Peak bytes are
28,912/29,800 and 35,632/79,632. Separate proof-retention epochs, excluding benchmark
oracle work, retain 25,664/29,408 bytes for constant and 31,600/75,600 for exponential.
Graph/native work is 1,582/1,605,906 and 949/668,132 for constant; 2,054/3,088,363 and
4,265/5,767,196 for exponential. Common original source/caches and allocator/platform
costs are excluded; these are fixture measurements, not universal bounds.

The existing default-feature benchmark shrinks 32 raw bytes to 6,462,528. Every
allocated ELF section matches baseline bytes/addresses/sizes except the build-ID note;
all 3,256 symbol-group size/address multisets match. Enabling the feature shrinks 72
raw bytes versus baseline, with unchanged .text length and symbol size multisets but
21 moved groups. The new benchmark is 3,471,984 bytes. Initial controls show adverse
separation in default Picard/preparation/reachability and enabled Picard. A second
six-run same-executable-path campaign removes the latter separations except default
Picard: baseline 29.708–29.739 µs versus default 29.950–29.975 µs, about 0.8% slower.
This observation is also explicitly accepted as cost/uncertainty for the new capability;
no default no-regression claim or hardware-cause attribution is made. Allocated-code
identity does not erase measured samples. Both campaigns and unfavorable results
remain; no threshold was loosened or further repetitions selected for favorable output.

Alternate-index application and exact reversal recover the original pending tree.
All 41 staged extraction paths and logical changes remain identical; only blob IDs and
hunk offsets change. Pending SHA-256 changes from
`d0f3f56e94d54d5f7330111f7bd6051cfe2caaf7ea23b21910b9b2cfaf69f6c0` to
`7402c446dd98a3c94a6cca38ad4620fc4d8b8d20929ace044afd9ceda8b68b40`.
Interface's eight edits and Firmware's clean state remain unchanged. All 35 prior
frozen stages verify. New record `docs/evidence/2026-11-14-interval-adaptive-taylor`
contains 110 files / 2,070,653 bytes, SHA-256
`669de51e089c482ab534a87e03929b3673b9f4b5140da2f877c31dd368848dbc`, archive commit
`9a8116bd54b5bc0ea252119bc2d4f15a95ae9324`. Committed blob digest and addition-only
archive scope verify; reproduction requires copying frozen drivers to mutable stages.

Automatic frame/QR evolution, independent-noise retention, cross-order preparation
sharing, competitive external portfolios, remaining migrations/adoption, Phase 11
extraction regressions, embedded flash/whole-call stack/cycles/HIL, outstanding
Interface Clippy issues and coordinated release remain open. No push occurred.
The full fifteen-phase goal remains active.

## Interface validation and production packaging (2026-11-14)

Phase 8/14 downstream qualification repairs are retained in Alumina Interface at
`b4a15bd0c281cb63ecc3c1d3d451144041c85f91`. The current client reproduced two
strict Clippy failures; the browser root independently reproduced three more.
The complete bounded scan payload now uses owned indirection in `NetworkUpdate`;
all native wire fields, retained scans, transaction checks, retry identity, typed
errors, Debug and worker JSON remain intact. This intentionally changes the
unpublished Rust `Scan` payload to `Box<NetworkScanResult>` and removes `Copy`,
with deep independent `Clone`. `docs/CLIENT-MIGRATIONS.md` records that contract;
it is not a source-compatible performance-only refactor. Other variants remain
allocation-free. No lint exemption or validation-policy relaxation was added.

The snapshot Boolean simplification consumes already-evaluated bools and preserves
validation order. Three new tests cover full scan ownership after reset, invalid
response retry identity, and 4,536 phase/identity/authorization/summary cases using
an independent presence-count oracle. Baseline and candidate semantic transcripts
are byte-identical, including all snapshot rows and full scan wire/JSON/Debug.
A redundant browser Ok-pattern is collapsed after its original retry arm. Two
private helpers extract the exact capability-transfer and network-scan UI bodies;
formatted dedented body hashes match, preserving validation, same Ui/scope,
widget/action order, diagnostics and secret erasure. No mathematical predicate,
policy, certainty receipt or cache sequence changes.

All 398 workspace tests pass both in the isolated candidate and primary checkout.
Strict workspace native all-target and browser WASM Clippy, warnings-denied client
docs, formatting and the source-policy audit pass in both locations. Compilation
and body identity do not establish interactive browser execution or latency.

The release workflow required three gzip artifacts that the existing post-build
hook never emitted. The unchanged hook reproduces their absence on valid minimal
fixtures while its Brotli assets round-trip. Added deterministic `gzip -9 -n` streams
preserve optimized originals and existing Brotli outputs. Two candidate fixture
runs yield nine byte-identical assets. The actual Trunk 0.21.14 release build passes,
as do all five configured production artifact checks. All six production gzip/Brotli
files also decompress to their exact generated originals. The WASM is 10,735,302
bytes, gzip 4,582,684 and Brotli 3,520,029; output hashes are archived. The initial
Trunk invocation rejected inherited `NO_COLOR=1` before compiling; setting the
boolean environment value to `true` resolved it without source/tool bypasses.
This is a combined local-checkout build, not clean-checkout release reproducibility.

Twelve CPU0/ASLR-disabled same-executable-path runs alternate baseline/candidate/
candidate/baseline three times after all builds and installations finish. Each
case measures one million complete operations after 32 warmups, including retained
state/full payload checks, event materialization and destruction. Median ns/op:

| Case | Baseline | Candidate |
| --- | ---: | ---: |
| status accept | 170 | 147 |
| status clone | 27 | 14 |
| full scan accept | 884 | 931 |
| scan clone | 49 | 61 |

The 5.3% scan-accept and 24.5% scan-clone increases are explicit costs of the
ownership migration that restores strict validation, not a general speedup or
regression-free refactor claim. Event/Result layout shrinks 368 to 80 bytes; machine
layout remains 576. Accepted scans and scan clones each add one 368-byte allocation;
status paths add none. All measured drops return to zero incremental live heap.
Preexisting state/response objects and allocator overhead are excluded. The probe
binary grows 2,344 bytes, including 2,288 .text bytes. Browser helper latency and a
baseline/candidate production-size delta were not independently measured. Raw
samples, initial colder observations, adverse results and original failures remain.

Only seven intended Interface paths were committed. Its eight pending migration
paths remain byte-identical (`ba4ca466d9de22d1b299756ab815a6df0d9849753ce41776385b52bc9ebc605a`),
and its index is clean. CSGRS's 41 staged extraction paths remain byte-identical
(`7402c446dd98a3c94a6cca38ad4620fc4d8b8d20929ace044afd9ceda8b68b40`). Firmware is
unchanged. All 36 previous frozen stages verify. New record
`docs/evidence/2026-11-14-interval-interface-validation` contains 152 files /
3,501,621 bytes, SHA-256
`c514cfaa499ad943a26efbbb49c83c97a037a5048feae3a09548b4f2da5e4277`, archive commit
`11a05b8237190b4570e60089ee510eda6786a285`. Its committed blobs and addition-only
scope verify; reproduction requires copying frozen drivers into a mutable stage.

Machine-profile/schedule migration retention, wider browser/HIL coverage, Phase 11
performance regressions, remaining consumers and coordinated clean release remain
open. No full CSGRS pre-push, fuzz/coverage/sanitizer, hosted deployment or live device
qualification is claimed. No push occurred. The fifteen-phase goal remains active.

## 2026-11-14 automatic QR evolution: bounded selection retained with explicit costs

Source `0603beace59f538e581d14dcf31dc237a39c42c4` adds the opt-in
`qr-evolution` feature and `CoordinateField::integrate_qr`. Each finite window
proposes an exact midpoint-Jacobian Euler basis, delegates certification to
existing QR and direct chart transport, and runs both the proposed and incoming
charts through existing Picard or trajectory Taylor integration. Both branches
must finish. Exact maximum source-endpoint diameter selects the smaller result;
ties select the recentered proposal. The selected native chart endpoint becomes
the next input, while both proofs, proposals, coefficients, projections and
comparison remain inspectable. Whole-request success and provisional stages
remain separate; original failures are not hidden by a fallback.

The missing method was reproduced on the parent before implementation. Twelve
new tests cover exact flows/relations, rational-series rotation oracles,
uncertain inputs, subsequent operations, empty/unit/invalid cases, limits,
cancellation/retry, budget cutoffs, fresh-process replay, zero-dispatch getters
and a tilted equilibrium whose selected frames certify two steps instead of
four. The latter returns exact zero in both modes and is a step-capacity witness,
not a speedup. Forced Picard/QR and unconditional Taylor/QR candidates failed
unchanged width obligations and were rejected. Their sources and failures remain
archived. A defective coarse-bracket oracle was corrected with independent exact
series bounds while preserving the original input, horizon, cap and width limit.

Final isolated qualification passed128 owner tests, nine downstream physics
compatibility tests, strict all-feature/all-target Clippy, warnings-denied docs,
disabled-feature and WASM checks, and formatting. Combined main qualification
passed139 owner tests, the same nine physics tests, strict Clippy, formatting
and the repository budget. Existing QR, transport and evolution modules are
byte-identical; existing coordinate method bodies are unchanged. This is a new
strategy, not a behavior-preserving kernel refactor.

The maintained three-fixture benchmark includes per-operation preparation,
source output materialization, exact analytic/width checks and destruction:

| Fixture | Fixed / automatic median ms | Fixed / automatic allocated bytes |
| --- | ---: | ---: |
| Constant uncertain line |4.252397 /8.854451|2283256 /4808920|
| Rotating uncertain line |13.561274 /34.745927|6131872 /21441244|
| Tilted exact equilibrium |2.742795 /25.760872|1506082 /12742237|

Automatic costs are2.08x/2.56x/9.39x runtime, with identical endpoint widths on
these fixtures. Both complete branch proofs are retained. Separate fresh-process
cold/warm measurements, allocation retention/drop epochs and all adverse rows
are archived. The strategy is retained for automatic certified selection and
composition; no general narrowing, optimality or runtime advantage is claimed.

Six existing QR controls, baseline/default/enabled then reversed, showed ratios
0.9914–1.0027 without the feature and0.9853–1.00595 with it. Exact completion
transcripts agree. Default allocated ELF sections/addresses match except build ID,
while raw binary size grows24bytes. Enabling the feature grows old-control .text
6816bytes and raw size12544bytes. These are focused workload/size observations,
not universal regression or firmware-size claims.

The41 pending extraction paths are preserved by alternate-index integration and
exact reversal. Their diff changes only blob IDs/hunk offsets; pending SHA256 is
`7fb21ee90a685a7050b0336331cf4f1b2c5e40c3c2f8d41e358efa54f3b66009`.
Interface's8 pending paths and Firmware's clean state are unchanged. All37 prior
frozen evidence stages match. No push or complete pre-push/release run occurred.

Evidence: `docs/evidence/2026-11-14-interval-qr-evolution`, archive commit
`1cd0dfd4642ae5683dcf927483e9d685765544e0`,509 files/6106302 bytes, SHA256
`2355d47c8eceae40a2e92a00788050810a238f95fb320cbc7c186a4a77503a27`.
The record contains contract, source/integration patches, commands, raw timing
samples, allocator and section records, fresh-process drivers, failed candidates,
review, STATUS and reproduction instructions. Committed blobs were verified and
the archive commit adds only this record. Exact scalar payload/refinement, nested
proof graphs, total heap and within-scalar cancellation remain outside the
structural/native budget. Better useful-work frame strategies, independent-noise
retention, wider adoption, competitive qualification, remaining migrations,
Phase11 costs, embedded hardware gates and coordinated release remain open.

## 2026-11-14 — Packaged interval release admission

Retained source `2611ab263ba58c5ce928374127d4bf2cc25e2c6d` adds Hyperinterval,
Hyperlattice Interval and Hyperdynamics to coordinated packaging. The original
release command reproduced a missing-package resolution failure. The repaired
script validates 21 packages, takes quickstarts from their actual archives and
uses the selected toolchain for metadata. Scalar typed rejection, Cartesian
composition and two source-bound certified flow horizons are executable README
examples. Only five tooling/documentation files changed; production kernels,
fixtures, manifests and predicate order are unchanged.

Full release runs passed on Rust 1.95 and stable 1.99.0, then on a fresh clean
Rust 1.95 checkout without dirty/no-verify flags: 21 verified archives, 21 workspace
docs.rs configurations, archive-only resolution and 26 client checks per run.
The clients include 20 README programs, modeling user path, C/C++, Python, Go and
TypeScript compilation. Go was 1.27.4; no browser runtime or HIL result is inferred.
All 21 clean archives reproduced byte-for-byte in a second clean checkout. That
second artifact-only generation used --no-verify; mandatory first-run verification
was retained. Dirty/clean lock checksum changes were traced to actual upstream
archive hashes, preserving all other lock structure and package source bytes.

The three interval crates passed archive-derived all-feature check, strict Clippy
and documentation, plus 42 locked individual feature configurations. The fixed
scalar/lattice subset passed Rust 1.88 native programs and ARM/WASM no_std checks.
Main formatting, shell lint/syntax, diff and repository-budget checks passed.
The original failure, rejected first example, interrupted obsolete candidate and
harness mistakes remain in evidence. No performance improvement is claimed.

Evidence commit `af7e90818a8fe92f9d50f5dc4778468cae533c27`, record
`docs/evidence/2026-11-14-interval-interval-release`: 169 files, 6,423,935 bytes,
SHA-256 `45654aed4f6abf169c0e9a4a45db5fd718fe9834806f3c5af5bf44ab71cbec8b`.
`STATUS.md`, `REPRODUCE.md`, `QUALIFICATION.json` and raw logs define the scope.
All 38 earlier frozen stages remained unchanged. Integration preserved the 41
pending extraction paths and Interface's eight pending files; these do not become
accepted by this release result.

Phase 14 remains open: full coordinated workflow/feature/platform qualification,
registry/docs.rs publication, application/HIL and remaining plan gates still apply.
The optional hypervoxel/voxelis publication blocker persists. No push or publication
was performed; the pre-push validation suite has not been claimed complete.

## 2026-11-14 — Continuous Hyperdynamics feature and runtime gates

Source `c633925421f6924d8aa3be9433000eec2cce80f3` adds the missing dedicated
Hyperdynamics workflow and updates the workspace workflow's ownership comment.
The prior workflow configuration never named Hyperdynamics, and the reproduced
no-feature test run executed zero tests. Existing field, Taylor, reachability and
coordinate implementations now have explicit feature and runtime CI commands.
No Rust source, dependency manifest/lock, fixture or mathematical policy changed.

The workflow covers Rust 1.95 and stable, with push/PR triggers for all seven local
dependency owners, `.cargo`, root Cargo files and the workflow itself, plus merge
groups. Both toolchains passed all nine validation steps locally: 18 independent
feature configurations each, 128 debug and 128 release tests across all 14
integration suites, 29 Criterion smoke cases, strict Clippy at both feature
boundaries, fuzz-target check/lint and warnings-denied documentation. No-feature
and doctest commands passed but execute zero tests. All six examples compile as
test targets; their main functions are not run by this workflow. The pinned
cargo-hack 0.6.46 also builds with Rust 1.95. Stable was 1.99.0.

Actual YAML command bodies were executed, and the final command sequence is
checked against the recorded runs. The late `.cargo` trigger addition changes no
validation command. Final trigger/dependency checks and repository-wide
`zizmor --pedantic --strict-collection .` pass with no findings; Zizmor 1.32.0's
default offline mode leaves network-dependent audits unavailable. Integrated
workflow shell syntax, formatting, Zizmor, budget and diff checks pass.

Evidence commit `0a454dbc3c32312a34172ed795f8db8e30f2a1a2`, record
`docs/evidence/2026-11-14-interval-dynamics-ci`: 77 files, 1,280,262 bytes,
SHA-256 `2cd306eacb1634e3261dca0f155f59eef2730ef851c694f23c79a20487ffb121`.
`QUALIFICATION.json`, `STATUS.md`, `REPRODUCE.md` and raw command logs specify the
scope. All 39 preceding frozen stages verify unchanged. Main's 41 pending paths
retain the identical staged patch digest; Interface's eight pending files and
firmware are unchanged.

This is local qualification of the retained source, excluding the unaccepted
extraction. Hosted GitHub Actions, runtime example mains, sanitizer campaigns,
coverage instrumentation, other OS targets, full pre-push checks and wider plan
gates remain open. No timing improvement, phase completion, push or publication
is claimed.

## 2026-11-14 — Sensitivity-retained Taylor noise queries

Source `2de9c0ae569a5e40fe97d8e4d179406834c677d3` adds opt-in Hypersolve
`taylor-noise` and its Hyperdynamics field consumer. Certified contribution-width
ranking selects shared monomial columns with canonical ties and caller capacity.
Exact source definitions and coefficient associations remain authoritative.
Every omitted monomial is accounted in explicit per-output residual bounds;
original Taylor residual coordinates keep their exact affine weights through
queries. This closes a static field-image query subtask of Phase 13/plan 10.7,
not independent-noise propagation through ODE steps or global sensitivity.

The missing API was reproduced before implementation. Review then found a real
residual-correlation regression: subtracting duplicated original residuals did
not cancel. Its failing test and old source are preserved; separating original
residual coordinates from omitted-monomial boxes restores that identity. Initial
broad runs were deliberately interrupted with SIGTERM/143 and do not count as
acceptance. Final capacity tests also retain a deliberate widening case:
`[9,23]` at one column versus `[15,17]` at two. The exact source remains shared;
independent enclosure parameters do not assert exact function/image equality.

Rust 1.95.0 and stable 1.99.0 each passed all eight qualification commands:
1,637 combined all-feature owner/consumer tests in 77 suites, doctests, strict
all-target Clippy, warnings-denied docs, minimal new-feature compilation, actual
field example, ten release owner tests and one release field consumer test.
Seven clean packages verified, and a consumer built, ran and passed strict Clippy
using their unpacked archives. Three fresh processes rebuilt matching noise
proof outputs from source definitions/requests/maps/capacity; four invalid
source/map/query controls failed without successful output, including a forged
certification flag. Existing executable bodies and feature/dependency definitions
are unchanged. Main formatting, budget and focused owner/consumer tests pass.

The completed shared-quadratic observation fixture checks identical exact
`[-n,n]` bounds for every query. Thirty-sample Criterion mean 95% intervals for
16 observations are 515.706–516.324 microseconds using full affine construction,
94.512–94.668 including noise preparation, and 74.407–74.476 with prepared noise.
Original Taylor/affine source construction is excluded equally; output checking
and destruction are included. "Cold" includes noise construction, not cold
scalar caches. The separate repeated probe reports allocation calls/traffic of
5392/115840 bytes, 1141/18040 bytes, and 976/14336 bytes respectively. Six timing
rows, raw samples, exact work counts and probe sources are archived. Additional
noise buffers retain the original shared source; no retained-memory/binary
reduction or universal speedup is claimed. Whole-resource bounds, ODE noise
propagation, global sensitivity, cross-order sharing and wider competitive
portfolio qualification remain open.

Evidence commit `6dde2edd0375940bf45af997a8aeace3cf49b994`, record
`docs/evidence/2026-11-14-interval-taylor-noise`: 166 files, 1,971,643 bytes,
SHA-256 `e2ac8632134dcf9fa5043a2cdd7e73d73b39b8d9418408101ab4aa6230aeddae`.
`STATUS.md`, `DESIGN.md`, `QUALIFICATION.json` and `REPRODUCE.md` document the
proof, limits, adverse evidence and commands. All 40 previous frozen stages
verify unchanged. Integration preserved every line of the 41-path pending
extraction and proved exact reversal to the prior index tree; its patch digest
changed only because blob IDs and hunk metadata changed. Interface's eight
pending files and firmware are unchanged. The extraction remains unaccepted.
No phase completion, full pre-push qualification, push or publication is claimed.

## 2026-11-14 — Shared prepared trajectory orders

Source `5722d7eaaeb561e74e5b8163c265a2557b7c2822` adds
`FlowTaylorProgram::restrict_order`: a prepared higher polynomial order can supply
any lower order, including zero, without regenerating exact Lie coefficient
DAGs. The prefix retains its mandatory next-order remainder stage and original
source guards. Stage handles and output indices are copied under structural
budgets; exact programs and derivative requests stay shared. The ordinary step
and whole-horizon verifiers remain unchanged. Higher-than-prepared requests,
actual resource exhaustion and cancellation have distinct typed outcomes.

The missing API was reproduced as E0599 before implementation. Every existing
executable body, feature/dependency declaration and package metadata is unchanged;
the existing trajectory module only gains a module declaration. Both Rust1.95.0
and stable1.99.0 passed all eight qualification commands: 139 owner tests across
17 suites, doctests, strict Clippy at all-feature and minimal-feature boundaries,
warnings-denied docs, actual example execution, ten release tests and unchanged
legacy Taylor benchmark smoke cases. Main formatting, budget, ten new tests and
the example pass. Tests compare complete proof artifacts, exact stage/guard
associations, graph/native consumption and step-dispatch counts with independent
preparation. Restriction performs zero scalar/predicate dispatch. Independent
polynomial/rational-solution expectations, algebraic constants, empty/unit cases,
invalid domains, exact work limits, cancellation/retry, nested restrictions,
owner drop, two-step and whole-horizon use are retained.

Seven clean dependency-chain packages verified. The archive-only example builds,
runs and passes strict Clippy; nine original test bodies also pass against only
unpacked archives, including three fresh-process source/proof reconstructions.
No test assertion was dropped or weakened. Source digests, manifests and metadata
record provenance. Package qualification does not establish whole-stack release.

Three completed portfolios construct/certify orders0..3 and materialize/destroy
all four reports. Shared preparation and independent preparation give identical
proofs. Mean 95% timing intervals (microseconds) are 194.716–201.309 versus
172.477–172.710 for constant flow, 333.613–334.845 versus 307.688–308.207 for
exponential flow, and 483.214–484.114 versus 444.638–445.191 for Riccati flow.
All Taylor preparation is included; only the common original field is excluded.
Thirty samples per row and all raw Criterion JSON remain. The separate repeated
probe shows unchanged native work and lower graph work/allocation traffic for
all three fixtures. Requested output-live bytes fall 35520→30352, 44528→38976
and 54312→46392. These counts exclude common inputs and allocator overhead;
they are not RSS, whole-process memory or universal bounds. No broad speedup,
binary-size reduction or existing-path performance claim is made.

Evidence commit `33e8e027ad03322db1e31bfee94ae448f5e21139`, record
`docs/evidence/2026-11-14-interval-taylor-order-sharing`: 129 files, 1,419,835 bytes,
SHA-256 `3f4814b84bf4a42c49ab5cd383f00bd3ac7576786d96eac7e38dd0094f79e294`.
`STATUS.md`, `DESIGN.md`, `QUALIFICATION.json` and `REPRODUCE.md` define scope and
commands. All 41 preceding frozen stages verify unchanged. Integration proves
exact reversal to the previous index tree and identical pending extraction
content except diff metadata. The 41 staged extraction paths remain unaccepted;
Interface's eight pending paths and firmware are unchanged.

The automatic adaptive controller still prepares each distinct order separately.
Its lazy preparation, history, resource-failure points and cost contract need
separate qualification before it adopts reuse. ODE noise propagation, wider
wrapping/adoption, unresolved migration costs, embedded/HIL and coordinated
release gates remain open. No phase completion, push or publication is claimed.

## 2026-11-14 — Adaptive order sharing: experiment not retained

The explicit lazy shared-preparation candidate extends already demanded Taylor
coefficient prefixes while preserving the original adaptive search and proof
history. It prepares no unrequested maximum order and caches no failed extension.
The owning Hyperdynamics recurrence remains the single mathematical authority.
Both candidate implementations remain isolated; no production changes from this
experiment are integrated. Automatic-controller preparation reuse remains open.

Macro source `62ecffb2c` matched the complete baseline artifacts, but a controlled
comparison measured about 3.5% slower existing order-three preparation; the
extraction-only control measured about 3.1% slower. Typed replacement source
`c6c574909fe42990512ef76f89e39a85906a01ba` passed 150 owner tests across 19 suite
summaries, strict all-feature/all-target Clippy, formatting and the example on
Rust 1.95.0. Its complete default reports, structural/native work and dispatch
traces match the original 373,388-byte artifact exactly. Inverse source checks
preserve the original recurrence/controller statement and predicate order.
These correctness checks do not establish performance or binary preservation.

The typed four-sweep comparison used one executable path, disabled ASLR and pinned
CPU affinity. It covered 35 Criterion rows in seven existing benchmark programs,
plus four manual adaptive rows per sweep. Seven rows met the recorded adverse
separation screen, including about 3.13% slower exponential order-three
preparation. This is an observational screen, not a simultaneous statistical
proof or universal causal claim. Several existing raw executables grew by 8–72
bytes; executable text totals did not increase. Symbol-string and exception-table
differences are recorded. Both implementations fail retention gates; no existing
assertion, policy or performance requirement was weakened.

The new strategy's three complete adaptive workloads include demanded preparation,
verification, quality checks, history materialization and destruction, excluding
only common source/controller setup. Independent/shared mean 95% intervals in
microseconds were 159.741–159.904 / 164.807–165.106 for constant flow,
415.162–416.263 / 411.636–412.222 for exponential flow and
1579.907–1581.952 / 1546.819–1555.263 for Riccati flow. Constant flow is slower
and structural work rises 949→957. Exponential work falls 2390→2338 and retained
requested bytes 53568→52224; Riccati work falls 10225→9669 and retained requested
bytes 156200→148280. Native arithmetic work is unchanged within each pair.
Requested allocation bytes exclude allocator overhead and common inputs; they
are not whole-process memory bounds. No general speedup is established.

Evidence commit `41ca8e6ca5d860487642a1185b4c72625ce0ec07`, record
`docs/evidence/2026-11-14-interval-adaptive-order-sharing`: 1,830 files,
32,217,725 bytes, SHA-256
`3b127ca7dd22fd52ffce1a5f349e496ee5fefb9466a45d1ac39fdfd32dc9c168`.
The record preserves both source deltas, the extraction-only control, golden
artifacts, binary hashes/sections/symbols, all raw timing campaigns and allocation
measurements. `STATUS.md`, `QUALIFICATION.json` and `REPRODUCE.md` state limits.
All 42 earlier frozen stages verified unchanged. Main's 41 staged extraction
paths, Interface's eight pending paths and firmware are unchanged. Stable,
package-only replay, full downstream, WASM, fuzz, coverage and full pre-push
qualification were not rerun after rejection. No phase completion, push or
publication is claimed. Do not repeat these rejected transformations without a
concrete change addressing the failed gates.

## 2026-11-14 — Native rational curve interval ranges

The optional `hypercurve/interval-ranges` consumer prepares the existing native
rational Bézier source as a reusable Hypersolve expression/Jacobian program. It
returns coordinate and local-parameter derivative enclosures with C1 evidence
over a finite dyadic subdomain of the native unit interval. Native Aabb
comparisons consume exact outward bounds while borrowing that evidence. Empty,
invalid, denominator-noncertificate, exhausted and cancelled outcomes remain
distinct. This extends evaluation and exclusion; it does not establish an exact
hull, immersion, topology or general curve-operation closure.

Source `c6c43e095b245e753b53237aa4eda3b9dc0737f8` passed 12 qualified commands,
including 3261 full-release owner tests across 128 suite summaries, 12 focused
witnesses in debug and optimized builds, strict Clippy/docs, the actual example,
downstream work-budget tests, a WASM library check and a stable
all-target/all-feature compile check. Eight packages were verified and an
archive-only consumer built, ran and passed strict Clippy. Existing ignored
tests are not completed coverage. Stable full tests and the full pre-push matrix
were not rerun.

The initial candidate populated a native basis cache and changed later
high-degree point dispatch. Its differing traces and deliberately stopped owner
run are preserved. The retained adapter borrows an existing basis or owns its
conversion without filling that cache. Fresh-process witnesses preserve complete
native traces, fixed counts and certified receipts under `STRICT` and
`APPROXIMATE_512`. Existing native evaluator bodies and default dependencies are
unchanged. Earlier test-fixture and rustdoc failures are retained; no count
assertion was loosened.

Measured portfolios include preparation, sixteen coordinate/derivative reports,
bound projection, native comparisons, retained output and destruction. Common
source construction is excluded and scalar caches are warm. Timing uses 30
paired samples of 32 portfolios on one pinned CPU; unrelated host activity is
uncontrolled. Allocation results use repository-locked dependencies and five
identical observations. An earlier probe with four newer dependencies is
preserved as superseded evidence.

| Fixture | Shared speedup (paired geometric mean; 95% bootstrap) | Allocations independent → shared | Peak requested live bytes independent → shared |
| --- | --- | --- | --- |
| polynomial | 1.691×; 1.685–1.699 | 14864 → 8039 | 113936 → 22976 |
| rational | 1.491×; 1.488–1.495 | 13002 → 8592 | 94280 → 20600 |
| degree8 | 2.187×; 2.183–2.191 | 36324 → 12729 | 380112 → 47232 |

Native arithmetic work is identical within each pair; structural graph work
decreases. Requested bytes exclude allocator overhead and are not a total memory
bound. Structural/dyadic budgets do not bound scalar refinement, coefficient
growth, payload cloning or uninterrupted native basis conversion. These results
compare shared and independent preparation of the new complete workload, not the
speed of existing modeling operations.

Evidence commit `ddc31825077d35ceab55a74e81fe2f1b4914b0dc`, record
`docs/evidence/2026-11-14-interval-curve-ranges`: 219 files, 3,321,299 bytes,
SHA-256 `ef013e80643dbc91e95c577f88290caf242fba96b1729aa81df18e3817a220a5`. All
43 earlier frozen stages verified unchanged. `STATUS.md`, `CAPABILITIES.json`
and `REPRODUCE.md` delimit the demonstrated contract. Integration preserves the
41 staged extraction paths by exact reversal and pending-content comparison;
Interface changes and firmware remain unchanged.

General monotonicity, interval Newton/Krawczyk integration, certified native
subdivision scheduling, normal-parallel/implicit/surface adapters and wider
Phase 13 adoption remain open, along with other unresolved plan gates. No phase
completion, push or publication is claimed.

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


## 2026-11-14 — Native tensor surface interval ranges

The optional hyperbrep/interval-ranges consumer retains admitted positive-weight
native rational Bezier tensor surfaces and their UV domains while preparing a
shared Hypersolve coordinate/Jacobian program. It certifies three coordinate and
six local partial enclosures and feeds exact outward Aabb bounds into native
spatial exclusion. Generated algebraic subdivisions, affine/reflected sources,
reconstruction and fresh native replay feed later queries. C1 evidence does not
establish rank, injectivity, trimmed-face membership, topology or general
surface operation closure.

Implementation source `014a082252da83d8d5d566253bcb598c7a456df3` passed ten
qualification commands, including full Rust 1.95 owner and downstream release
tests/doctests. Their explicit harness summaries report 2399 passes across 147
summaries and 127 across 28, with no failures or ignored reports. These counts
are not a unique coverage denominator. Expanded acceptance adds 52 focused
passes. Strict all-feature/all-target Clippy, strict docs, disabled-feature
checks, the actual example, WASM library and stable all-target checks pass.
Stable full runtime and full pre-push workflows were not rerun.

Documentation metadata source `d625de5b8e93dcb34e481a99253b1bb3cdd77592` changes
no Rust source, workload, runtime dependency or default feature. Nine archives
verify; an archive-only unchanged native example builds/runs/passes strict
Clippy. The same archived example at the implementation revision completes three
sixteen-cell portfolios through a zero-import/default-stack debug WASM test
export. Packaged all-feature documentation and five rendered API pages pass
locally. Hosted docs.rs and public JS binding support are not claimed. Final
documentation source `8a9180461b63398d92906af4a30f5f99976d2dbf` passes affected
format, repository-budget, strict lint/docs and integrated-main private/example
checks.

Existing native evaluator bodies and default dependencies remain unchanged.
Preparation preserves the native homogeneous-control cache but can warm shared
exact scalar facts: the new complete lifecycle snapshots pin actual cold/warm
success, post-conversion exhaustion and cancellation behavior under both
policies. No old trace assertion is changed. The original mistaken
cold-transparency fixture remains in evidence. Whole expanded algebraic scalar
equality can remain undecided; an independent per-node exact quadratic oracle
verifies the maintained generated outputs without changing production
normalization. Original failed assertions, Cargo CLI and archive-comparison
premises remain visible.

Measurements compare two uses of the new API over three complete sixteen-cell
portfolios, including preparation, all coordinate/partial and source/domain
evidence, native exclusion, output retention and destruction. Warm runs use two
discarded samples and thirty alternating pairs of sixteen portfolios. Cold runs
use thirty fresh-process alternating pairs of one portfolio; source construction
and the oracle between timed query/destruction spans are excluded. CPU 8 is
pinned and no compiler/test workers run during timing; other host activity is
not controlled. These are finite same-API results, not old-kernel nonregression
or universal speedups.

| Fixture | Warm shared speedup (95% bootstrap) | Cold shared speedup (95% bootstrap) | Allocation calls independent → shared | Peak requested live bytes independent → shared |
| --- | --- | --- | --- | --- |
| polynomial | 1.720×; 1.710–1.727 | 1.815×; 1.786–1.848 | 35998 → 19213 | 229784 → 50384 |
| rational | 1.653×; 1.652–1.654 | 1.712×; 1.700–1.723 | 37828 → 21043 | 229392 → 49992 |
| degree8x3 | 3.297×; 3.296–3.299 | 3.298×; 3.277–3.316 | 208702 → 43552 | 1576616 → 183776 |

Five warmed allocation observations per row are identical and return tracked
live storage to zero after output destruction. Cold runs retain 888 bytes for
polynomial and 584 for rational sources after source/output destruction. Degree
8×3 retains 20032 bytes independently and initially 15632 with sharing; a
separate 32-query diagnostic reaches 20032 by its fourth shared query and then
plateaus. This is consistent with existing small-rational caches, not precise
attribution, leak freedom or a uniform bound. Requested payload excludes
allocator overhead/RSS. Structural graph and dyadic budgets do not bound total
scalar CPU, coefficient growth, heap or interrupt native conversion.

Evidence commit `1ae464092041d0de2667cf17a9cec05b09b1486e`, record
`docs/evidence/2026-11-14-interval-surface-ranges`: 338 files, 7,919,430 bytes,
SHA-256 `eaf0e44271d600f7453bc02088f27cf02e5d18665636a1659bfcc0bdd82c87c1`. All
44 earlier frozen stages verify unchanged. STATUS.md, CAPABILITIES.json and
REPRODUCE.md delimit the contract. Integration proves exact index reversal and
identical pending content apart from blob/hunk metadata, preserving all 41
staged extraction paths. Interface's eight pending edits, firmware and prior
evidence remain unchanged.

Other surface-family adapters, native automatic subdivision/monotonicity and
interval Newton integration, wider wrapping/adoption, unresolved migration
costs, embedded/HIL and coordinated release remain open. No phase or full-goal
completion, push or publication is claimed.


## Maintained packaged curve-range consumer: retained after release qualification

Owner: release pipeline, P36/P37, Phase 14. Source revision
`90bf46b37b73b88938090012db1d563c7eca2cd6` enables Hypercurve's
`interval-ranges` feature in the existing external consumer, copies the exact
`examples/interval_ranges.rs` from its archive, and executes it alongside all
26 previous clients. No production Rust source, example, repository manifest,
version, release eligibility or workflow file changed. Documentation now
explicitly identifies the existing documentation gate as a workspace feature
check, not archive-only docs.rs verification.

The missing-feature witness was reproduced before production edits. Its
initial minimal consumer also lacked dependent solver features; all original
diagnostics remain archived. A focused control matches the solver features
already enabled by Hyperdynamics field, isolates the missing Hypercurve
feature, then compiles successfully when it is enabled. These negative controls
use workspace library sources; archive qualification comes from the full runs.

On the clean committed candidate, both commands passed:

- `bash scripts/release-check.sh --toolchain 1.95.0`
- `bash scripts/release-check.sh --toolchain stable` (Rust 1.99.0)

Each verified 21 packages, completed 21 workspace documentation builds with
`-D warnings`, audited all 21 external dependencies as archive sources, and
passed all 27 clients. The unchanged new example was separately rerun from
each built binary: four parameter cells, retained source/tangent evidence,
native bounds and native nonoverlap consumption. Both captured consumer
manifests preserve every prior dependency declaration except the new Hypercurve
feature. Syntax, ShellCheck, formatting, diff and repository budget checks
passed before and after integration. The full pre-push matrix was not run.

The release runs took 1266.905 s and
1182.195 s respectively and ran concurrently;
these are validation observations, not comparative performance evidence.
QUALIFICATION.json records absolute debug executable sizes and hashes.
Integration preserves all 41 staged extraction paths and their patch bytes
exactly; both Alumina repository states are unchanged.

Evidence: `docs/evidence/2026-11-14-interval-release-curve-consumer`,
archive commit `ffc69225bdba97e570f7af423ca21bbcdde8e71a`,
112 files / 571,939 bytes, SHA-256
`d87b876e2fe9cadcb089cf31b6862d763be0bc8239099cc37736b368aa9cdf71`.
All 45 earlier frozen stages verify unchanged. COMPLETION.md and
REMAINING-GAPS.md delimit the result. Archive-only documentation, dependency
resolution reproducibility, the optional-voxel publication blocker, HyperBrep
release, wider feature/platform/HIL coverage and the remaining implementation
goal stay open. No phase completion, push or publication is claimed.

## Correlated trajectory endpoints: opt-in flow-noise retained

Owner: Hyperdynamics trajectory/proof composition, Hypersolve finite parameter
Taylor/affine/noise authority, Hyperlattice/Hyperinterval range arithmetic.
P1, P5–P10, P17–P20 and P36/P37; Phases 12/13. Final source
`ee9699cacc902c1363db112910f560ba14f9f573` adds `FlowNoiseState` and proposed-step
`certify_noise_step`. It retains shared initial columns, complete parameter and
time remainders, distinct fresh residual inputs and immutable field/proof history.
The entire relaxed affine image supplies the next Picard check. An independently
narrower endpoint box cannot silently restrict parameter combinations. This is
a certified outer endpoint set, not equality with the exact reachable set.

The pre-edit witness reproduces Cartesian loss under forward/reverse shears;
its adverse [-3/2,3/2] x bound remains tested. Fourteen focused tests cover
repeated generated input, exact sqrt(2) coefficients, omitted-column accounting,
nonlinear residuals, Riccati/oscillator controls, full-image admission, guards,
empty/invalid/N=0 cases, exhaustion/cancellation/retry, final publication,
512-state iterative destruction with a shared intermediate state, the actual
example and fresh-process replay. No old production function body, predicate
order, default feature, dependency version or existing assertion changes.

The clean isolated owner suite reports 153 passes in 19 harness summaries and
Hyperphysics 83 in 16. Strict all-feature/all-target Clippy, docs, disabled
features, the example, formatting and WASM library compilation pass. After
integration, the pending-extraction worktree reports 164 owner passes in 22
summaries and the same 83 downstream passes; strict Clippy, format, diff and
repository budget also pass. These are harness counts, not universal coverage
or acceptance of the previously staged extraction's costs.

Seven archives verify at implementation revision
`d97e1a968a37eca11d3666d4cecb7f96fb43c8f2`. Rust 1.95 and stable 1.99 each pass the
unchanged archived example, final fourteen-test client including fresh-process
replay, archive-only owner all-feature documentation and archive path audit.
Later changes add only the algebraic acceptance test and remove one accidental
README blank line; measured/packaged production Rust, manifests, dependencies,
example bytes and substantive README text are unchanged. This is a seven-crate
qualification, not a new full 21-crate coordinated release or hosted-docs claim.

Costs compare complete certified identity endpoints at x-width limits 2+1/M,
with y width 2. The box baseline partitions initial y into M cells; the new
path retains two columns. Both keep every proof, materialize output and include
output destruction. Thirty alternating pairs per case/preparation mode run on
CPU 8 without known compiler/test workers. Each process retains its first run
separately and summarizes five subsequent rounds. Cold includes Taylor-program
preparation, not fresh scalar caches in those subsequent rounds. Common field
ingestion is excluded; other host activity is uncontrolled.

| Workload | Cold noise/box time | Warm noise/box time | Warm peak tracked bytes: box → noise |
| --- | --- | --- | --- |
| Constant identity, no correlation benefit | 5.691 | 6.526 | 17656 → 95392 |
| Shear identity, M=8 | 0.959 | 0.965 | 151760 → 118200 |
| Shear identity, M=32 | 0.250 | 0.245 | 598160 → 118200 |
| Shear identity, M=128 | 0.063 | 0.062 | 2383760 → 118200 |

The near-break-even coarse case, slower constant control and all raw samples
remain. The linked probe grows 3,554,816 → 3,813,408 bytes (+7.27%); ELF text
grows 2,681,158 → 2,889,406 (+7.77%). Named linked symbol totals are separate,
not universal monomorphized-code bounds. These results support an optional
correlation strategy; they do not support replacing the existing flow API.

Twenty repeated allocation workloads show no growth after the first, but
post-source requested retention is 0, 104, 208 or 312 bytes depending on mode.
Existing small-rational caches are consistent with this result, not precise
attribution or leak-freedom proof. The original failed zero-retention assertion
is preserved. Tracking headers, allocator overhead, RSS and stack are excluded.
Structural/native budgets still exclude scalar refinement/CPU, coefficient
payload, total heap and within-scalar cancellation.

Integration proves exact reverse application and preserves all 41 pending
extraction paths apart from blob IDs/hunk metadata. The initial alternate-index
conflict was caused by a stray README blank line; no main state changed before
its isolated correction. Both Alumina repositories retain their original states.
All 46 previous frozen stages verify unchanged. Evidence record
`docs/evidence/2026-11-14-interval-flow-noise`, commit `078e775834cf1881b96878c5a9860d1a88f52b65`:
626 files / 4,072,543 bytes, SHA-256
`218186d773da3af3d2ca99f07b9c66bb355cd195382de0e6bd817d5cba82107f`.

STATUS.md, CAPABILITIES.json and REMAINING-GAPS.md delimit the result. Automatic
noise-aware horizon control, wider nonlinear/deep-chain and platform costs,
resource exclusions, cost-rejected migrations, staged extraction, embedded/HIL
and coordinated release remain open. No phase/full-goal completion, full
pre-push matrix, push or publication is claimed.

## Positive transmission interval chain: rejected at native and WASM size gates

Owner: Hyperinterval checked rational arithmetic/strict positivity; Hyperreal's
scalar reducer is unchanged. Alumina Interface retains units, nominal values,
configuration identity and typed physical-fact errors. P1, P3–P6, P8–P10,
P17–P20, P27, P31–P33, P36/P37; Phase 8. Targets are clean committed CSGRS
`9ee4375ed568e434bc81838199fd9fda765bead4` and Interface
`b4a15bd0c281cb63ecc3c1d3d451144041c85f91`, excluding their pending edits.

The distinct hypothesis combines unsigned-radius ingress with a retained
nonempty positive interval proof, the complete four-factor transmission product
and its positive quotient. It preserves five original positivity checks, nominal
calculation and lower-before-upper arithmetic order, with no new product/quotient
predicates. Zero factors give [1,1]; one factor copies its bounds. The public
Interface interval layout and general empty/signed interval semantics stay fixed.
No scalar implementation, dependency version or compiler profile is changed.

Before production edits, the missing owner API failed compilation on the target.
The final candidate reports eleven passing tests across three owner harnesses,
including 625 independent integer-corner chains, a large-rational GMP oracle,
generated output consumed again, invalid refinement and fresh-process original/new
small/large cold/warm dispatch equality. Strict all-feature/all-target Clippy
passes after renaming the new consuming method to `quotient`. The original
negative source subsequently grew; the archive preserves the original diagnostics
and final source without claiming they reproduce identical diagnostic counts.

Seven full native profiles and identities, including one expected invalid input,
and all seven warmed allocation/deallocation/reallocation/byte/peak/live rows
match baseline. All seven WASM transcript hashes match native, invalid selectors
are rejected and the module makes no host calls. This consumer agreement is not
an independent mathematical proof or a completed migration.

| Artifact | Baseline bytes | Final candidate bytes | Change |
| --- | ---: | ---: | ---: |
| Native profile executable | 810576 | 811456 | +880 |
| WASM profile module | 1069062 | 1070027 | +965 |

Both required size gates fail. GNU size's text/read-only-data column grows by
824 bytes; its aggregate dec falls by eight because other sections change.
That aggregate does not waive executable or code growth. Initial and final lint-
corrected candidate binaries are byte-identical. No matched timing campaign,
monomorphized function attribution, whole-profile both-policy dispatch comparison,
full schedule/browser suite or full pre-push matrix follows this early rejection.
Build durations and WASM fuel are not performance claims.

The isolated candidate is removed completely. Restored owner tests (three reports),
strict Clippy, both worktrees' formatting and diff checks pass. Fresh native/WASM
rebuilds match baseline artifact hashes byte for byte; complete native output also
matches. Both isolated worktrees are clean. CSGRS's 41 staged extraction paths,
Interface's eight pending paths and firmware's clean revision remain unchanged.
The candidate's full patches, source, locks, initial harness failures and outputs
are retained only as rejected evidence. No production API is integrated.

All 47 prior frozen stages verify unchanged. Evidence record
`docs/evidence/2026-11-14-interval-profile-positive-products`, local commit `8c43d0fcec53718bb7ec46ada0753111fcb61bae`:
85 files / 2,657,197 bytes, SHA-256
`7dee7993e0f04d606012d1598ec5dbaa14aab221b89efbb2ede78671c58c1cf8`. COMMANDS.json distinguishes execution-time argv from
reconstructed reproduction recipes; raw logs establish the reported outcomes.
Phase 8 and the overall fifteen-phase goal remain open. No push or publication.

## Release documentation argument transport: retained

Owner: repository release tooling; P36/P37, Phase 14. Source
`f8641a319fa943642b7d8c62551cc454a5eaa778` repairs a reproduced omission in the actual release
loop: docs.rs feature settings were applied but `rustdoc-args` were discarded,
including HyperDRC's `--cfg docsrs`. Before edits, the old loop fails an offline
Rust fixture whose documentation requires that cfg.

The maintained helper forwards per-package arguments through Cargo's encoded
rustdoc flags, preserves whitespace and keeps `-D warnings`. Each package gets
its own environment; ambient rustdoc flags cannot override this check. Missing
packages, unsupported metadata keys and invalid argument values fail explicitly.
The complete selection is validated before documentation begins. Supported keys
are `all-features`, `no-default-features`, `features` and `rustdoc-args`; this is
local workspace configuration validation, not hosted docs.rs emulation.

A compiled two-crate fixture checks actual cfg consumption, all/selected features,
default-feature suppression, a header filename containing spaces, rendered HTML,
flag isolation, warning rejection and preflight failure. It passes on Rust 1.95
and stable 1.99, in both full release runs and in the combined main worktree.
The initial shellcheck invocation without `-x` reports only the existing source
include; corrected source-following lint passes. No production Rust, manifest,
lockfile, kernel predicate, canonical format or CI configuration changes.

Both complete release checks pass from clean source without `--allow-dirty` or
`--no-verify`: 21 verified archives, 21 local documentation configurations and
27 client checks per compiler. These include 20 archived README programs, the
modeling user path, packaged curve ranges, C/C++, Python, Go and TypeScript.
TypeScript is compilation, not browser runtime. All released dependencies resolve
from unpacked archives, with no workspace source paths. Captured documentation
commands show HyperDRC receiving its declared cfg and every crate retaining the
warning gate. Archive digests and clean VCS revisions are retained; no new
byte-reproducibility or runtime performance campaign is claimed.

Exact patch reversal and integration preserve all 41 staged extraction paths
byte for byte, Interface's eight pending paths and firmware's clean revision.
The integrated fixture, shell lint/syntax, formatting, diff and repository-budget
checks all pass. All 48 previous frozen stages verify unchanged. Evidence record
`docs/evidence/2026-11-14-interval-release-docs-arguments`, local commit `ff166be4931ef6081139483a016c5c708178857b`:
116 files / 606,907 bytes, SHA-256
`28d1c14b9280e084168bddd0a976cad794d6aa1d0f816837f0bc84a048bd8ab8`.

Hosted docs.rs nightly/target/environment qualification, archive-only documentation,
wider platform/HIL and release gates, the optional Hypervoxel publication blocker,
rejected migrations and pending extraction remain open. No phase/full-goal
completion, full pre-push matrix, push or publication is claimed.

## Symbolic endpoint adapter: source audit and migration boundary tests

Owner: Hypersolve's strict symbolic endpoint adapter; Hyperinterval owns generic
scalar interval arithmetic. P1/P4/P6/P8–P10/P17–P20/P27/P36/P37, Phases 0/7.
Source `2480e611ab3e00913eb9cef9354f9db3b3aecdaf` retains tests and a scoped ownership audit,
with no production migration. All 490 lines and 19 declarations in
`real_interval.rs` were read. The audit records exact source hashes, selected
complete caller bodies and one partial call context separately. Exhaustive
caller and whole-inventory review remain open.

Three lexical inventory rows are updated and sixteen missed declarations added;
all unrelated rows, including earlier rejected dispositions, remain byte-identical.
`docs/research/interval-real-adapter-audit.md` explains the retained strict
comparison/predicate order, public endpoint preconditions, optional failure,
polynomial proof ownership and resource gaps. The generic directed square-root
endpoint helper remains an unqualified Hyperinterval migration candidate.

The missing witness reproduces an observable difference: symbolic multiplication
of [-1,1] by [-1,1] returns [-2,2], while the tight rational carrier returns [-1,1].
Squaring those generated results gives [0,4] versus [0,1]. Both product enclosures
are valid; a tighter bound does not establish identical-output refactoring.
Five new tests cover independent integer-corner extrema for 784 interval pairs
(81 wider symbolic products), generated-result composition, sqrt(2) endpoints,
quotient exclusions and independent quadratic Bernstein sign expectations.

The all-feature Hypersolve library/integration command reports 1,519 passes in
63 summaries and zero ignored reports; these are harness reports, not universal
coverage. Strict all-feature/all-target Clippy passes. Three unchanged Hypercurve
filters each run and pass one regression: strict-only uncertainty, polynomial
boundary-zero/Bernstein fallback and unused radical terms. The five focused tests
also pass on the combined main worktree, followed by formatting, unstaged/staged
diff and repository-budget checks. No production function, manifest, lockfile,
existing fixture or predicate order changes; no runtime/size improvement or new
migration cost/trace/serialization qualification is inferred.

Exact integration preserves all 41 staged extraction paths byte for byte, plus
Interface's eight pending paths and firmware's clean revision. All 49 previous
frozen stages verify unchanged. Evidence record `docs/evidence/2026-11-14-interval-real-interval-boundary`, local commit
`e3a53fbfba7cddd70a43757b057152a88d31e376`: 37 files / 244,016 bytes,
SHA-256 `e71d80fcbfaffaaaddfb27c20db53739d40262b10ca84e5b811702231e4c1db2`. The audit and tests make the next migration contract
explicit; they do not complete Phases 0/7 or the whole goal. Broader ownership,
implementation, cost, platform/HIL and release obligations remain open. No full
pre-push matrix, push or publication is claimed.

## Dyadic products: exact normalization at exponent storage limits

Owner: Hyperinterval, with Hypersolve expression and Hyperlattice interval
consumers. P1/P6/P18–P20/P36/P37, Phases 3/6. Source `efee9bbe3e4007fb81d24ebccffb53ac54d88426`
extends multiplication to representable temporary exponent overflows, using the
existing square/power normalization authority. Two acceptance witnesses failed
with `Overflow` on baseline `a7ea9382f`: `2^MAX * 2` now returns mantissa 2 at MAX,
and a lower-boundary hull `[-2,4] * 2^(MIN-1)` becomes `[-1,2] * 2^MIN`.
Genuinely unrepresentable lower endpoints still fail. The original upper hard
case is retained with its exact success expectation. `docs/MIGRATIONS.md` records
the intentional failure-contract extension and resource-admission precedence.

The ordinary successful path retains the four corner products, predicate order,
canonical endpoint construction and work accounting. No duplicated normalization
kernel, interval fitting, policy demotion, public name, feature or format changes.
Independent integer expectations exercise 12,150 boundary pairs in both operand
orders and 6,075 ordinary pairs/scales. Tests cover exact size/work thresholds,
real overflow, huge growth rejection, fresh-budget retry, empty/zero identities,
subsequent exact operations, canonical serialization and zero Real-dispatch counts.
Two generated results replay and feed later operations in a fresh process.

Owner all-feature tests/doctests report 226 passes in 42 summaries; Hyperlattice
reports 90 in 24; Hypersolve library/integration tests report 1,520 in 63. None are
ignored. New downstream tests consume normalized products through expression
programs and vector dot/scale. Strict all-feature/all-target Clippy for all three
crates, native and WASM no_std checks, strict owner rustdoc and formatting pass.
Integrated focused tests report 28 passes; diff and retention checks also pass.
These finite witnesses are not a universal closure or full-plan completion claim.

Matched external native probes materialize the same ordinary products and compare
canonical output, serialization, work and allocation requests: all agree across
five cases, each with 24 alternating samples per build. Observed median time ratios
are 1.0045–1.0295; executable size grows 2,568 bytes and the mul symbol 81 bytes.
This measured cost is retained for a concrete completeness gain, not described as
a performance-only refactor or speed improvement. The unpinned shared-host probe
has its own retained dependency lockfile; workspace tests use the workspace lock.
No peak-heap, whole-application, WASM runtime, MCU-cycle or firmware qualification
is inferred. DyadicBudget still bounds declared reservations/accounting units,
not total instructions or actual heap; no new cancellation contract is claimed.

Exact forward/inverse tree checks preserve all 41 staged extraction paths. The
migration file's pending additions remain identical; only patch blob identifiers
change. Interface's eight pending paths and firmware's clean revision are
unchanged. All 50 prior frozen stages verify unchanged. Evidence record
`docs/evidence/2026-11-14-interval-dyadic-product-exponents`, local commit `264c8052e49aafc00e534bf8b06a2665004ae335`: 57 files /
1,346,990 bytes, SHA-256 `0485a5d397e6ed60eb4a9f415d7fad7c93ba9d2444bff3942a3a9d2286a80419`. Broader migrations, dynamics
acceptance, platform/HIL and release obligations remain open; no full pre-push
matrix, push or publication is claimed.

## Directed dyadic quotient sharing: rejected candidates, retained regressions

Owner: Hyperinterval, P1/P4/P6/P10/P18–P20/P36/P37, Phases 3/5 and their
consumers. Source `d8d524297ca55fb802e0b2ca2e985e4f72e2276f` retains three independent private-kernel
acceptance tests and a performance disposition. Every production arithmetic body
is byte-identical to `4d4e22a31`; dyadic.rs adds only a cfg(test) module inclusion.

The existing scaled_quotient performs floor and ceiling divisions separately.
Two single-division candidates passed focused frozen-legacy comparisons of bounds,
errors and budget state, plus independent small integer expectations. Signed
quotient/remainder increased executable size by 432 bytes with workspace-derived
dependencies (num-bigint 0.4.6) and 1,008 with the external resolver's dependencies
(0.4.8). Retaining unsigned quotient/remainder until directed output construction
reduced the pinned executable by 1,224 bytes and function-address union by 446,
but increased the external executable by 104 bytes and function-address union by
254. Both candidates are rejected under the unchanged refactoring gates; the
favorable pinned result does not justify narrowing supported dependency versions.

Fifteen completed public-operation probes per lock compare exact serialized
outputs, domain qualification, work and allocation requests for signed/large/exact
division, rational conversion, square roots, elementary functions, rational powers
and split division. The magnitude candidate matches output/work in all cases and
does not increase allocation requests. These single probes are not a completed
statistical timing, policy-report, owner benchmark or successive_operations
campaign; later gates were not run after decisive size rejection. Both restored
native probe executables are byte-identical to their respective baselines.

The retained tests cover 11,390 small signed integer cases and 880 large-limb
multiplication-inequality cases, exact work/size thresholds, failure precedence,
huge-shift refusal, fresh-budget retry and zero Real-dispatch counts. The copied
legacy algorithm remains only in archived evidence. Full owner tests/doctests
report 229 passes in 42 summaries, none ignored. Strict all-feature/all-target
Clippy, no-default-feature tests, formatting and diffs pass. The main worktree's
three focused tests, formatting, staged/unstaged diffs and retention budget pass.
No new fuzz, coverage, platform, downstream runtime or performance gain is claimed.

All 41 pending extraction paths remain byte-identical, as do Interface's eight
pending paths and firmware's clean state. All 51 previous frozen stages verify
unchanged. Evidence `docs/evidence/2026-11-14-interval-dyadic-quotient-sharing`, local commit `1701779269dcacfaddb307e3e0d00cbb6112cd78`:
73 files / 1,668,127 bytes, SHA-256 `099a79dd7e9ef8e3036d03500153aa00b93cca0534d57858289366e74061a8f6`. The shared
quotient optimization remains unaccepted; broader migrations, performance,
platform/HIL and release requirements remain open. No full pre-push matrix,
push or publication occurred.

## Current Interface browser execution and canonical replay

Phases 8/14, Interface browser/client and firmware simulator consumers,
P6/P8/P9/P19/P27/P31–P37. Interface source `73cc71b1bc426cafca0ee829e2fbc40961a45054` repairs a
reproduced stale browser harness: it expected schema 13 while the release worker
emitted 15. All six harness command sites and strict receive validation now use
one explicit version 15 constant. A client-owned regression checks alignment with
WORKER_SCHEMA_VERSION. The production protocol is unchanged; the Rust addition
is cfg(test) only. Current schema documentation is corrected.

The release bundle was rebuilt from Interface b4a15bd plus its pending migration,
CSGRS e232d00 plus its 41 staged extraction paths, and firmware b9ecf8d. The
harness repair followed that build. Offline packaging initially missed the cached
matching wasm-bindgen executable; correcting PATH completed the unchanged build.
WASM validation and six compressed-asset round trips pass. Initial browser DNS
isolation incorrectly blocked loopback; that setup failure is preserved separately
from the reproduced application-level schema mismatch.

Chromium 155/SwiftShader and two loopback simulator actors pass four maintained
cases without weakening assertions or deadlines: clock qualification; attended
cached-job completion (531 snapshots); fresh-worker reattachment on the same
actors (seven snapshots); and completion after both applied job-confirm responses
were dropped (531 snapshots, exactly two confirming failure observations).
Each participant consumes 127,264 canonical bytes. Reattachment preserves both
exact local start cycles while leaving the unavailable original UI epoch absent.
This is combined-checkout qualification, not acceptance of the pending migrations,
all browser fault scenarios or physical-device behavior.

The actual CAD/CAM page renders with a ready worker and zero connected MCUs.
Browser exports of a 2,064-byte ALMCFG06 configuration and 476-byte ALMEVD03
evidence file replay in a fresh browser process/profile. The configuration is
transactionally reconstructed and evidence verified against that reconstruction.
A truncated evidence file is rejected; subsequent exports remain byte-identical
to both originals. Files, digests, screenshots and terminal results are archived.
Deterministic serialization is not treated as a universal semantic-equality proof.

The first UI observation reaches two animation frames after the Trunk event at
3,141.2 ms, with a 19,988,480-byte main-realm WASM buffer. After replay a separate
process uses 23,789,568 bytes. Seven browser processes sum to 375,753 KiB PSS at
blank-page observation and 621,815 KiB at rendered UI; these are shared-host
snapshots with software rendering, not peak allocation or portable requirements.
The raw report distinguishes main-realm memory, worker exclusions, instrumentation
limits and warm host caches. No startup/file-replay window errors were observed.

All 90 client tests pass, none ignored. Strict all-feature/all-target Clippy,
workspace formatting, integrated schema regression, diff and retention checks
pass. All owned service process groups are stopped after terminal capture. The
41 staged CSGRS paths and eight pending Interface paths remain byte-identical;
firmware remains clean. All 52 previous frozen stages remain unchanged.
Evidence `docs/evidence/2026-11-14-interval-interface-browser-execution`, local archive commit `9b6fbd62171d6ae53311abab552f45df9655862c`:
142 files / 3,373,703 bytes, SHA-256 `70e6efb5545e018acbee4abd2cce74ce0c0d5b69c2a144241cfe31a4c5cdaac9`.
Broader migration, performance, platform/HIL and release requirements remain
open. No full pre-push matrix, push or publication occurred.

## Current interval sanitizer campaigns and conversion admission

Phases 3/5/14, plan §14.3, Hyperinterval and independent test-side GMP oracles,
P1/P3–P6/P17–P20/P27/P36/P37. Source `14f78d34d392880d07ebf1b893d6dae9a2ab3678` adds a maintained
conversion fuzz target, its rational feature requirement, a bounded sanitizer CI
invocation and its input-domain documentation. The initial fifteen targets did
not directly exercise rational/dyadic conversion, exact-grid admission or IEEE
binary64-to-dyadic ingress. Existing deterministic regressions remain unchanged.
No production arithmetic, predicate order, policy, budget, dependency or lockfile
changes; the four changed paths are test/build configuration and documentation.

Generated signed 64-bit numerators/nonzero unsigned denominators, empty/singleton/
interval sources and grids -128 through 128 are checked against existing GMP
rational-transfer and directed-grid oracles. Output is serialized, reconstructed,
materialized as exact rational endpoints and consumed on another grid. Exact-grid
admission is checked against the original rational values. Exact observed work
thresholds, one-unit-short exhaustion, fresh-budget retry, reversed endpoints and
extreme-grid size refusal remain explicit. Arbitrary IEEE bits exercise finite
exact ingress and nonfinite rejection before budget use. This is not arbitrary
coefficient-size coverage or a total-work proof.

All fifteen existing ASan campaigns pass (867,644 libFuzzer runs); conversions
passes 79,026 more, totaling 946,670 engine runs. Trigonometry and conversions
end successfully at their campaign time limits before their requested run counts.
No crash artifacts or sanitizer diagnostics occur. Engine counts include rejected
arbitrary inputs; sancov counts include dependencies and are not a source-coverage
percentage. No production defect or universal correctness result is claimed.

Cargo-fuzz 0.13.2 and rustc 1.101.0-nightly (8006091eb, LLVM 23.1.4) build the
instrumented binaries. Existing targets use all fuzz features; the new target uses
rational. Direct native invocations retain the workflow's seed/run/length/time
settings plus explicit 30-second per-input, 2,048-MiB RSS, leak-detection and outer
deadline controls. None of those failure limits fires. This all-feature campaign
is not a claim that every hosted/feature-isolated workflow command ran. Original
and generated corpora, source/binary hashes, lockfile, commands and raw logs remain
in the evidence archive; GMP/MPFR C instrumentation is not claimed.

All 229 owner tests/doctests pass in 42 summaries, none ignored. Both fuzz feature
checks, strict all-feature fuzz-target Clippy, fuzz/workspace formatting, integrated
target checks, diffs and repository retention checks pass. Because CI changed,
repository-wide pedantic strict-collection Zizmor runs in both isolated and main
checkouts and reports no findings. Temporary-index tree equality proves source
integration while all 41 staged extraction paths remain byte-identical. Interface's
eight pending paths and firmware state remain unchanged; all 53 previous frozen
stages verify unchanged.

Evidence `docs/evidence/2026-11-14-interval-interval-fuzz-current`, local archive commit `4be983d4534316a1224790a21420aebdba6e4707`:
2465 files / 436,694 bytes, SHA-256 `6c2c9c8f22e633230472a8e1e76d32d39556c5f7ac0ead06ea67d122f96cd581`. Broader
migrations, source coverage, platform/HIL and coordinated release obligations
remain open. No performance gain, full pre-push matrix, push or publication is
claimed; the complete goal remains active.

## One-sided rational square-root migration: rejected

Phase 7, Hyperinterval arithmetic ownership and Hypersolve proof/fallback
ownership, P1/P4/P6/P10/P17–P20/P35–P37. At `a8eef70bbd5e07c7cecfee92f06852415b61657c`, a distinct
candidate relocates the existing one-sided rational grid square-root kernel,
without changing its nonnegative rational/nonpositive precision domain, eager
sign/div_rem/root/square check order or solver fallback. Typed owner errors map
to the original Option. The existing two-sided budgeted API stays unchanged;
the candidate explicitly preserves the legacy lack of total-work/heap accounting.
No new unbudgeted API is retained.

Twelve same-path Rust 1.95 release cases match in 4,506 bytes of native exact
endpoint JSON and downstream STRICT/APPROXIMATE_512 comparison reports. They
cover rational/symbolic radicands, clipped/empty domains, zero, exact squares,
positive/nonpositive or absent grid requests and large coefficients. The external
lock pins num-bigint 0.4.6; adding the production Hyperinterval dependency changes
no registry version. This is preliminary finite equivalence evidence, not complete
math/status/certainty/dispatch qualification.

The executable grows 3,029,720 → 3,029,792 bytes (+72), failing the unchanged
refactoring gate. Named function-size multisets and the 1,403,058-byte function
address union match. ELF exception tables grow 20 bytes; padding and other metadata
also change. Identical function lengths or lower aggregate data usage do not waive
binary growth or establish instruction/performance equivalence. No arbitrary
inlining/layout variants, timing/allocation campaign, owner benchmark or downstream
successive_operations campaign follows this decisive rejection.

All candidate bodies and dependencies are removed. Restoring source and lock
reproduces the baseline executable byte for byte; fresh-process output also
matches. Review commit `006e9060531d470951b374dbecd3f94638ded610` changes only one inventory row and
the existing adapter audit to record this outcome. Temporary-index tree equality
and patch hashes preserve all 41 staged extraction paths, Interface's eight pending
paths and firmware state. All 54 previous frozen stages verify unchanged.
Main diffs, retention and the restored probe pass; no full test/Clippy/platform
matrix is inferred from an early rejected candidate.

Evidence `docs/evidence/2026-11-14-interval-sqrt-endpoint-migration`, local archive commit `4b1102536d8b779f98901cca5582b093fafa7a9f`:
47 files / 1,641,347 bytes, SHA-256 `06bfb0059a3c964b310c88bb476259aba662739b94bba0de90b3fa1f222f91e3`. This migration,
broader performance/resource requirements, platform/HIL and coordinated release
remain open. No push, publication or full-goal completion is claimed.

## Native workspace qualification and maintained validation repairs (2026-11-15)

Phase 14 and current Hypersolve coverage; workspace CI routing and the
Hypersolve benchmark-report fixture own these changes. The input domain is the
existing maintained native corpus and its documented feature configurations,
not universal geometry closure. Base `fed4183e8ca77a043867d6c7177058fc696238e8` plus the exact
41-path pending dynamics extraction is tested in an isolated worktree. The
extraction's performance acceptance remains open.

The workspace check passes for all 24 members and targets. Thirteen of fourteen
original qualification commands pass, including the separately nested Hypervoxel
crate. The DRC release suite fails its unchanged full-board worker-CPU cost gate;
this stage does not qualify the complete native workflow.
This exercises default configurations, with DRC in release mode, on installed
stable Rust 1.99.0 / LLVM 23.1.1. It does not execute every dedicated workflow,
platform, feature or hosted job. Per-command raw logs and terminal records retain
all run limits and filtering; compilation is distinguished from test execution.

| Command group | Harness passes | Failures | Summaries | Outcome |
| --- | ---: | ---: | ---: | --- |
| hyperlimit-tests | 367 | 0 | 24 | passed |
| hyperpack-tests | 154 | 0 | 5 | passed |
| hyperevolution-tests | 25 | 0 | 5 | passed |
| hypersdf-tests | 122 | 0 | 12 | passed |
| hyperparts-tests | 23 | 0 | 5 | passed |
| hypercircuit-tests | 105 | 0 | 48 | passed |
| hyperpath-tests | 443 | 0 | 6 | passed |
| hyperphysics-tests | 72 | 0 | 16 | passed |
| hyperbrep-tests | 860 | 0 | 84 | passed |
| hyperdrc-tests | 1,560 | 1 | 1 | failed; later targets not reached |
| hypervoxel-tests | 116 | 0 | 4 | passed |
| hyperbrep-io-tests | 127 | 0 | 28 | passed |


Source `efd2d65329800667fb4e6afe1c9d3b034fc84213` adds the missing
`hyperbrep-io` matrix entry. Before the edit, the all-workspace check compiled its
tests but no workflow ran them. The new unchanged default-feature command passes
127 tests/doctests in 28 summaries, within its 30-minute limit. All 24 members now
have a test route through a dedicated workflow or the shared matrix; routing is
not proof that every such workflow ran here. Existing matrix entries and limits
are unchanged. Repository-wide pedantic strict-collection Zizmor passes in both
isolated and primary worktrees with no findings (default offline mode).

The original complete Hypersolve coverage attempt exits 101 before calculating
coverage: the benchmark-catalog parser correctly discovers ten registered targets,
while the example test still expects seven. The original log and raw profiles
remain archived. A focused invocation reproduces the same failure on the target
source. Source `70d6325cf606f00dccc0d0c18040c7fff0b7b759` preserves every old
expected name and assertion, adds the three already registered Taylor targets,
and checks each target's Criterion status and required feature. All twelve added
lines are inside the existing test module. No parser, kernel, benchmark workload,
policy or production body changes.

The focused catalog test passes with all features and with default features
disabled. Strict owner all-feature/all-target Clippy and final workspace formatting
pass. A fresh full coverage script run passes: 54,802 / 63,329 production lines
(86.54%), above the unchanged 85% threshold. Its three configurations report
4,427 harness passes in 194 summaries, zero failures/ignored tests and 18 filtered
instances; these include repeated configurations, not unique tests. The existing
Criterion test-mode and custom benchmark workloads execute as configured. Their
instrumented timings, overlapping other correctness work, are not performance
comparisons. The merged profile, annotated report, object identities and full logs
are retained; compiled objects remain outside the evidence archive.

The DRC library reports 1,560 passes and one failure: the KiCad smoke suite
uses 132.445845432 seconds of worker CPU against its existing 120-second limit.
Cargo initially stops before its remaining targets. Separate final release
integration and doctest commands pass 108 tests in eight summaries and three
doctests in two summaries, with no ignored tests. These do not turn the failed
library cost gate into a complete DRC qualification.
The saved binary reproduces the failure. Removing only the pending extraction
in the isolated checkout and rebuilding on the same path yields an accepted-tree
control that also fails at 127.253439273 seconds; the saved pending binary fails
near 127.3 seconds with no other owned tests/builds. This demonstrates that the
failure does not require the extraction, without establishing performance
equivalence or accepting that extraction. Both original board archives and every
check, assertion and deadline remain unchanged.

A 64-sample debugger probe observes repeated unary arrangement construction.
The investigated structural adjacency shortcut returns false before a full
metadata scan when a nonconsecutive pair contains no fragment zero. Its DRC
binary grows 392 bytes, including 48 bytes of text, and the unchanged smoke test
still fails at 125.969343863 worker seconds. The candidate is rejected and its
production source restored byte for byte. No arbitrary layout/inlining variants
or speedup claim follows; raw samples, symbols, binaries' digests and failures
remain in the archive.

Source `72a281c18aa5630ab7c60b6b44f25368688727ff` adds an independent cyclic-
adjacency regression: differently sized operand/loop partitions, all ordered
pairs and reversed storage under both policies, with zero numeric dispatch
asserted when tracing is enabled. The final combined checkout passes that test
with all features and no default features, strict all-target/all-feature owner
Clippy, formatting and diff checks. Clippy first reproduces a current-toolchain
lint in the unchanged interval-range replay test. The same source commit replaces
its constant chunks_exact iterator with fixed-size as_chunks iteration, preserving
the divisibility assertion, coordinate order, child-process reconstruction and
proof comparison. The final fresh-process replay test passes. A separate finite
standard-library probe compiles and passes on Rust 1.95; it is not a full owner
MSRV qualification. The original lint failure is retained. No production
geometry body is retained.
The original workspace runs remain valid for unchanged production paths; the
new test is checked separately, not credited as a rerun of every suite.

Temporary-index tree equality verifies all three integrations. All 41 staged extraction
paths, Interface's eight pending paths and firmware state remain unchanged. All
55 prior frozen stages verify unchanged. Diff and retention checks pass. No performance improvement,
full pre-push, complete platform/HIL or full-goal completion claim
follows from these checks. The rejected migration and extraction gates remain
open; no push or publication occurs.

Evidence `docs/evidence/2026-11-15-interval-workspace-qualification`, local archive commit `ee8790064afd94ee089a865e2061b9510a17ae46`:
137 files / 172,230,791 bytes, SHA-256 `24a5df0d14c1cc9dd1ec80dcad258de9aca345cbd6ad0c5e733adcd577a9b2d4`.

## Automatic affine-noise horizon candidate and retained dispatch regression (2026-11-15)

Phases 12/13, plan sections 10.7/10.8; Hyperdynamics owns finite-horizon
composition, while Hypersolve retains Taylor/noise proof authority. Base
`2bf1c6481f68ea7cdb0c696f768c22264deaa36a` is tested without the 41 pending
extraction paths. The missing witness is automatic consumption of a retained
`FlowNoiseState` through a bounded horizon without independent Cartesian demotion.
The proposed controller reuses accepted native Picard proofs, retains complete
Taylor/noise evidence and the full affine input domain, and exposes only explicit
certified prefixes after failure. Scalar refinement/payload and total-heap
accounting exclusions remain explicit. No controller implementation is retained.

The first shared endpoint-helper extraction passes 163 owner tests and unchanged
manual dispatch/work/allocation controls but fails pinned performance controls:
16 paired batches show mean shear ratios 1.005424 cold and 1.007430 warm, with
paired bootstrap intervals above one. It is rejected despite smaller binaries.
A revised private generic proof-source interface preserves the original manual
proof boundary and one endpoint/publication implementation. No forced inlining,
predicate reordering or arbitrary layout variants are used. Its pinned shear
ratios are 0.996934 cold and 0.996865 warm, with intervals containing parity;
this is no detected regression on those controls, not a proven speedup.

The revised candidate's four cold/warm constant/shear allocation, cleanup and
work-count controls match exactly. All eight final owner benchmark smoke runs
pass. Six existing benchmark executables are byte-identical to the baseline.
The QR evolution executable grows 7,542,040 -> 7,542,056 bytes. Code, read-only
and writable data, and unwind sections match byte for byte. The extra 16 bytes
are in nonallocated symbol strings; build-ID and symbol-table contents also
change. This still fails the unchanged raw binary-size gate. The candidate is
rejected without a waiver or further source-layout variants. Constant timing,
full owner timing, monomorphized-size, successive_operations, package-consumer,
WASM, fuzz and full pre-push qualification are not completed for it.

After the revised edit, stable and Rust 1.95 independent feature matrices pass
all 21 configurations. Stable and MSRV owner tests each pass 163 cases in 21
summaries; stable release lib/tests pass 163 in 20 summaries. Strict all-target
Clippy passes on both compilers with all features, and on stable without default
features. Stable workspace formatting, strict docs, minimal-feature tests and
doctests pass; the latter two execute zero assertions and are not behavioral
coverage. Hyperphysics all-feature tests pass 83 cases in 16 summaries. The
candidate witnesses include exact reversible shear, nonlinear Riccati and
time-dependent solutions, empty/unit inputs, prefix continuation, final
publication exhaustion, cancellation/retry and fresh-process replay. These
finite results do not establish universal closure, complete resource accounting
or delivered capability.

Source `0a3b609e008e28344db393dbb66564b2d3545b4f` retains only a dispatch-count
regression for the existing shipped manual affine-noise example and its
accepted-base fixture. The fixture was generated and confirmed on the original
accepted implementation and required unchanged for both candidates. The primary
pending-extraction tree passes the new test, strict owner all-target/all-feature
Clippy and formatting. Temporary-index tree equality and patch hashes preserve
all 41 staged extraction paths, Interface's eight pending paths and firmware.
The rejected production/API changes are removed after snapshotting. Both
restored manual probe binaries reproduce baseline hashes byte for byte. Final
restored-source owner tests pass 154 cases in 20 summaries; workspace formatting
passes. All 56 earlier frozen stages verify unchanged.

The evidence preserves two diagnostic corrections. An objcopy section dump
rewrote earlier baseline/candidate QR files; rebuilding matching source recovered
the original hashes before original-file comparison. Manual timing/allocation
binaries and the revised QR binary were unaffected. A control-driver selection
mistake initially reran the earlier benchmark list; all eight revised smokes
were subsequently run from the correct saved binaries. Original failures and
literal command records remain, rather than being overwritten.

Evidence `docs/evidence/2026-11-15-interval-flow-noise-evolution`, local archive
commit `77526600bc4fc390e9db37800c78f8d0ed8aa4b0`: 426 files / 7,717,756 bytes,
SHA-256 `f8782a79d00119ff4bc14a40372c6797d4dd0103a0cf2d296dd92c684964cd3e`.
This stage retains a regression test, not the automatic controller. Broader
wrapping/resource support, the extraction and other existing cost gates,
platform/HIL and coordinated release remain open. No push, publication or
full-goal completion is claimed.

## Strict IEEE enclosures on WebAssembly (2026-11-15)

Phases 4/14, plan section 3.12; Hyperinterval owns finite IEEE endpoint
qualification. Base `8d9b0d30c3515cd6c191f872f95b32d86059b1d0` reproduces
nonempty binary32/binary64 ingress rejection in a real no_std WASM module.
Source `0ba14a211b54ce01026a2ba40b8869554524820f` extends the two qualification
cfg guards to wasm32-unknown-unknown. Integer arithmetic kernels, rounding,
representations, predicate order and typed error semantics are unchanged.
Other unsupported targets retain rejection. This is a capability extension,
not an arithmetic refactor or a claim that all platform work is complete.

An independent integer/Fraction oracle checks full endpoint bits and typed
outcomes: finite ingress, every exponent class, subnormals, extrema, empty and
invalid inputs, arithmetic/domain failures, exact root inequalities, set and
width properties, and output-as-input composition. Each final module passes
55,802 arithmetic cases and 8,836 property queries. Rust 1.88, 1.95 and stable
1.99 artifacts each execute successfully in Node 22.23.5 and Chromium 155.
Hyperlattice downstream runs additionally pass 612 IEEE and 612 fixed chains
per artifact/engine, spanning nine dimensions, 17 seeds and native/SoA storage.
The maintained CI driver builds, lints, executes and inspects generated code;
local Chromium runs are separate from the hosted Node workflow.

Complete scalar modules contain no imports, memory.grow, native floating
arithmetic or floating conversions; bit reinterpretation remains permitted.
The no_std consumer links without a global allocator and has only Hyperinterval
as its normal dependency. The lattice fixture uses exact small native dyadics
for input construction and is not covered by that scalar instruction-scan claim.
Unsupported thumbv7em support flags are const-proved false; there is no MCU
runtime or HIL qualification. Finite corpus success is not exhaustive IEEE proof.

Native owner tests pass 229 cases, downstream lattice tests 90, and MSRV
primitive/serde tests 184. Release alternate-rounding/FTZ/DAZ controls pass.
Strict owner Clippy, docs, workspace/nested formatting and final repository-wide
pedantic strict Zizmor pass. Saved native baseline/candidate codegen controls
are byte-identical: 499,936 bytes, SHA-256
`876d177a67283fead193fe45c111426318ab5881eb5d49c8acbf041c5438f69d`.
No timing improvement or whole-application cost claim follows.

Clean-source packages build and verify on Rust 1.95; repeated archive creation
reproduces both hashes. An external no_std consumer uses the unpacked
Hyperinterval archive without workspace library dependencies. Rust 1.88 and
stable builds, strict Clippy, Node and Chromium pass the full scalar corpus.
Metadata confirms the two-node consumer/library dependency graph. This validates
local packaged delivery, not registry publication or every feature combination.

Original diagnostic failures remain in the evidence: a missing Zizmor PATH
entry, Cargo lock-wait stderr contaminating a dependency listing, an unnecessary
package patch warning, and an overstrong expectation of Thumb IR constant
folding. Explicit-path Zizmor, separated stderr, a fresh minimal consumer and
compile-time support assertions provide the stated final evidence. No oracle
expectation or acceptance gate was weakened.

The integrated primary checkout passes the complete stable WASM driver, all 22
native float tests, formatting, Zizmor, diff and repository-budget checks.
Temporary-index equality and final patch hashes preserve all 41 staged
extraction paths and Interface's eight pending paths; firmware is unchanged.
All 57 earlier frozen stages verify unchanged. Extraction, broader wrapping and
resource support, hosted platform/HIL and coordinated-release work remain open.
No full pre-push, push, publication or full-goal completion is claimed.

Evidence `docs/evidence/2026-11-15-interval-wasm-float-qualification`, local archive commit
`52a9ce3f00e56527dc11d1a25819211f66b47fbc`: 304 files / 40,877,079 bytes,
SHA-256 `542eeec3955e1a1feb68aef57bbe55c6ea217f883a0f6834686f66a7633702be`.

## Directed dyadic-to-IEEE conversion closure (2026-11-15)

Phases 3/4/14, plan sections 3.5/3.12; Hyperinterval owns scalar directed
conversion. Baseline `f9ca2901e92c7fd5471cb60ba262a71469c21ee5` reproduces the
missing public API. Source `d51c7bb4d9298c913e3149cb5a478e98dbd318d0` adds
`F32Enclosure::from_dyadic` and `F64Enclosure::from_dyadic`: the tightest finite
IEEE enclosure of a borrowed exact dyadic interval. This closes explicit
IEEE → dyadic arithmetic → IEEE enclosure chains. Off-grid endpoints widen;
containment does not claim equality with the retained source. Overflow,
unsupported environments and resource exhaustion remain typed. Empty is valid.

A private arbitrary-mantissa bridge reuses the existing IEEE encoder and sign
authority. Bit length selects the target grid; retained significand and a checked
discarded tail establish floor/ceiling without native floating arithmetic.
Extreme exponents never cause exponent-sized shifts. Both endpoints share the
caller’s DyadicBudget, reserving before limb inspection/allocation. Existing
arithmetic bodies, predicate order and production dependencies are unchanged.
The documented deterministic work/bit accounting is not a heap-byte or
cancellation guarantee.

Stable owner tests pass 238 cases, Hyperlattice interval 90, and the Rust 1.88
primitive/serde configuration 192. Eight new optimized tests and the release
MXCSR test pass; the latter executes 56 literal boundary cases in each of 16
isolated rounding/FTZ/DAZ modes. Independent GMP/MPFR checks compare tight bits
for 12,000 arbitrary-mantissa intervals. Separate tests exercise every finite
exponent field, subnormal/normal/extreme boundaries, huge mantissas, i32 exponent
limits, shared-budget failures/retry, immutability, replay and further arithmetic.
Strict owner Clippy, docs, formatting and ARM no_std compilation pass.

The maintained 14-command WASM driver passes on Rust 1.88, 1.95 and stable 1.99,
including the existing 55,802 scalar cases, 8,836 property queries and downstream
vector chains. A separate allocating dyadic consumer executes all 56 exact
boundary cases in Node and Chromium on each compiler; its complete generated
module has no native floating arithmetic/conversion or imports. The original
float-only module remains separately allocation-free. The expanded sanitizer-
backed strict-float fuzz target completes 100,000 runs without failure. Coverage
of the new private module is 38/38 lines and 85/87 regions, not whole-owner or
branch coverage.

The existing native float-only control remains byte-identical: 499,936 bytes,
SHA-256 `876d177a67283fead193fe45c111426318ab5881eb5d49c8acbf041c5438f69d`.
A separate allocation probe requests two allocations totaling 32 bytes for each
successful 65-, 4,097- and 1,000,001-bit mantissa fixture. Rejected size-budget
calls and i32 extreme-exponent fixtures allocate zero temporary bytes. Input
construction, allocator overhead and peak RSS are excluded; no timing speedup
or whole-application size claim is made.

Clean-source packages verify on Rust 1.95 and reproduce byte-identical archives.
External archive-only consumers pass native/WASM builds, strict Clippy, Node,
Chromium and dependency metadata checks on Rust 1.88 and stable. Original
validation diagnostics remain: cargo-fuzz was outside PATH, llvm-cov initially
received an incorrect positional object, and one successful browser execution
failed during raced profile cleanup. Explicit tool lookup, corrected export
arguments and bounded cleanup retries resolve them; no assertion was weakened.

Integration proves exact index-tree preservation of all 41 pending extraction
paths. The primary checkout passes the eight new tests, formatting, Zizmor,
diff and repository-budget checks. Interface’s eight paths and firmware remain
unchanged; all 58 earlier frozen stages verify unchanged. No full pre-push,
hosted OS/MCU/HIL, complete resource, whole-goal, push or publication claim is
made. Existing migration cost and coordinated-release gates remain open.

Evidence `docs/evidence/2026-11-15-interval-ieee-dyadic-egress`, local archive commit
`97a51a509d6fcd497580a9feee8d1c5671226fbb`: 589 files / 86,109,543 bytes,
SHA-256 `7f7c8843bfebb923ef3aacb841ceac2825cfde3d17f234d4cdab6e2df5d8930f`.


## 2026-11-15: matrix support candidate rejected; rectangular regressions retained

Source `0157d6b9bcb344ba60e1badfd0a2d2ff400aa655` extends only the companion's Cartesian tests.
A phase-6 / section-5.3 experiment added an opt-in borrowed exact-zero support
mask and reused the existing ordered dot kernel. Focused tests passed for
fixed, rational, dyadic and qualified IEEE inputs, including budget retry,
no-allocation and fresh-process source replay. It is not a retained API.

The same-path release control `centered_cost` grew from 1,696,312 to
1,696,320 bytes, failing the experiment's predeclared unchanged-size condition.
Code, allocated data and unwind sections are byte-identical; the growth is
seven symbol-string bytes plus padding. The candidate was removed without a
size waiver or layout/name/inlining variants. No timing campaign or complete
candidate qualification was run. Raw source, focused results, hashes and
section comparisons are archived so this rejection need not be rediscovered.

Four retained regression tests check 512 rectangular 2x3-by-3x2 products per
scalar configuration across six configurations, output columns, subsequent
signed-diagonal products, and empty/unit shapes. Exact integer endpoint-corner
expectations are independent of interval arithmetic. All prior test contents
remain a byte-identical prefix; no production or public API changes remain.
Stable owner tests pass 94 cases, no-default five, Rust 1.88 primitive/serde 80,
and downstream Hyperlattice interval surface seven. Strict all-target Clippy
passes on stable/all-features and Rust 1.88 primitives; its initial divisibility
style diagnostic was fixed and checks rerun. Formatting and primary-checkout
Cartesian tests, diff and repository budget pass. All eight old example
executables are byte-identical to baseline after candidate removal.

Integration preserves the 41-path staged extraction patch exactly. Interface's
eight edits and firmware remain unchanged; all 59 prior frozen stages verify.
Object-fact reuse, migration cost gates, resource completeness and coordinated
qualification remain open. This is not phase or goal completion. No full
pre-push, MCU/HIL, timing speedup, push or publication claim is made.

Evidence `docs/evidence/2026-11-15-interval-lattice-support-facts`, local archive commit
`cf37b5f152ebb0ffe652fb5609a350b6705abda7`: 70 files / 338,764 bytes,
SHA-256 `26196061d3bfcf4d24513b35b682773cd97a6169ff282f3f1aa77878568cf330`.


## 2026-11-15: preserve exact cell identity across SDF-to-voxel handoff

Source `87cd2481f23a3713ba945bef2370b64a2f7c240d` repairs the existing optional Hypersdf
adapter, a prerequisite for phase-13 conservative voxel adoption. Before editing,
the current revision accepted a Filled negative-x source cell with an unrelated
positive-x target origin. The regression's required rejection failed: address
validity alone did not preserve the physical cell certified by the source.

The adapter now requires matching declared units, positive source dimensions
within target capacity, valid source indices, and certified exact equality of
all origin/pitch coordinates. It reuses Hypersdf's existing strict construction
predicate. Mismatch is typed invalid input; undecided equality is UnknownOrdering
with its axis. Rectangular and partial batches still support sparse intake;
complete exact storage admission stays in Hypervoxel. This does not authenticate
caller-forged public occupancy labels or add an implicit transform/unit conversion.

Ten feature-enabled regressions cover the original witness, all five units,
2^-60 origin/pitch mismatches on each axis, independent algebraic equality,
undecided exact-zero identities, invalid/empty source batches, source indices,
retry, literal physical cell bounds, partial/unknown intake and fresh-process
materialization. Rational admission asserts exactly six strict comparisons and
zero approximation events. Original tests and fixtures remain unchanged. Stable
all-feature owner tests pass 172, no-default 122, Rust 1.95 adapter tests nine,
and the separate Hypervoxel workspace's affected intake/path/grid tests 35.
Strict all-target Clippy, rustdoc, formatting and Rust 1.95 WASM compilation pass.
Primary regression, format, diff and repository-budget checks pass. Initial test
fixture Result mistakes and the wrong downstream workspace command are retained
with their corrected successful runs.

Twenty isolated CPU0/ASLR-disabled same-path probe runs retain identical valid
payload digests and allocation counts/bytes for complete conversion, exact sparse
materialization and drop at 1/8/512/4096 cells. Median candidate/baseline time ratios
are 1.0265/1.0140/0.9789/0.9869; overlapping short samples do not establish a
speedup or no regression. The executable grows 2,320,560 to 2,619,616 bytes
(+299,056) when certified comparison becomes reachable. These costs are accepted
for the exactness repair; this is not a behavior-preserving refactor, and the old
adapter fails the repaired contract. No application-wide size or total-resource
claim follows. Raw binaries, samples, probe and validation logs are retained.

All 41 staged extraction paths and Interface's eight edits remain exact; firmware
is unchanged and all 60 prior frozen stages verify. Native-enclosure voxel intake,
remaining migration cost/resource gates, hardware qualification and coordinated
release remain open. No phase/goal completion, full pre-push, push or publication
is claimed.

Evidence `docs/evidence/2026-11-15-interval-sdf-voxel-frame`, local archive commit
`efe5493aa245b2a901202e649c99e25e63b8c228`: 61 files / 5,077,995 bytes,
SHA-256 `a0dc818a168e3cf93506605ab9775ebcd83be67fff9955a9c40dfe2c5c486291`.


## 2026-11-15: consume native source-bound enclosures in voxel storage

Source `cbc9dc9d2b84946994bc3abc65172668b1ff1daa` adds phase-13/Tier-3 native voxel intake behind
both existing enclosure-program and hypervoxel-adapter features. The current
base lacked SdfProgram::classify_voxel_grid; its missing-method witness is
retained. Hypervoxel remains the exact frame/address authority. Outward native
coordinate enclosures feed SdfProgram::image, and an immutable report keeps each
original source-bound proof, address, frame units and options. Export reuses
existing occupancy mapping and exact storage admission, with Unknown preserved.

Thirteen new regressions demonstrate independent sin/cos whole-cell certification
and exact materialization where the legacy path was Unknown; literal polynomial
images and subsequent consumption; algebraic frames and transformed sources;
source/metric identity; zero versus overlap; canonical empty storage; fresh-process
source reconstruction; typed domain, capacity, work, entry, arithmetic, storage and
cancellation failures with retry. Fresh reconstruction is not serialized proof
replay. Cell limits, graph work/entries and shared dyadic arithmetic are distinct;
frame scalar arithmetic, Hyperreal refinement, clone/drop and heap bytes retain
native exclusions. Cancellation during scalar refinement is not established.

Final stable owner tests pass: 181 main harness tests by --list, with fresh child
executions additional. The earlier raw stdout totals interleave child output and
are not canonical counts. No existing assertion was removed. Rust 1.95 new tests,
strict stable/MSRV all-target Clippy, rustdoc, single-feature/default checks, WASM
compilation and 35 downstream Hypervoxel intake/path/grid tests pass. A separate
Rust 1.95 path client executes the example; this is not packaged-release evidence.
Primary new tests, format, diff and repository-budget checks pass. Setup failures
and the explicitly invalid Fresh candidate build are retained with corrections.

The additive completeness gain is retained with measured costs, not presented as
a behavior-preserving refactor or optimization. Full proof/intake/materialization/
destruction probe medians: polynomial 1/8/512 cells 8.75 us / 63.22 us / 4.20 ms;
trig 8/64 cells 0.946 / 8.218 ms; algebraic-frame eight-cell empty storage 81.00 us.
Requested allocation bytes, work counters, source, raw samples and binaries are
archived. Source/frame construction and compilation are outside that measurement.
No speedup comparison to legacy Unknown or peak-memory claim follows.

All 46 existing classification benchmarks completed in a pinned CPU0, ASLR-disabled,
same-path B/C/C/B campaign. A longer repeat of the three largest adverse ratios
confirmed AABB interval 1.02962, arithmetic Lipschitz 1.11833, and batch classification
1.07006 candidate/baseline mean ratios, with separated adverse confidence intervals.
The full screen and repeats, including unfavorable data, remain in the archive.
Existing benchmark bytes grow 3344 and basic-example bytes 2920. Unchanged legacy
mathematical bodies do not establish unchanged compiler output or runtime; emitted
code/relocation differences were inspected, but no unique runtime cause is proved.
These costs are explicitly accepted for demonstrated native evidence consumption
under exactness > completeness > performance. No no-regression gate is claimed.

The 41 staged primary extraction paths and eight Interface edits remain byte-identical;
firmware is unchanged. All 61 prior frozen stages verified. Whole-resource closure,
remaining migration cost gates, hardware and coordinated release remain open.
This is not phase/goal completion; no full pre-push, push or publication occurred.

Evidence `docs/evidence/2026-11-15-interval-sdf-voxel-enclosures`, local archive commit
`32f176a919814448030d0a585fb552d248a5fa4f`: 3,197 files / 58,398,912 bytes,
SHA-256 `70662d68780559b476ebd4f0b7209ab72e1ba24432a04c21d3148d6ed392a401`.


## 2026-11-15: retain exact definitions behind transformed voxel boxes

Source `3366c48c36eb48c30613f9facffdf6e05b6f5b3d` adds the optional Hypervoxel interval-enclosure
feature for phase-13/Tier-3 transformed conservative bounds. Before editing, a
finite certified enclosure of a sine-addition identity still left exact affine
term ordering undecided. The new enclose_bounds method succeeds without asserting
that ordering: native scalar conversion and Hyperlattice matrix/vector arithmetic
produce a containing Cartesian box and retain the exact original transform and
source endpoints in an immutable AffineBoundsEnclosure. Existing exact map_bounds
arithmetic and comparison order remain unchanged.

Reversed endpoint pairs denote their closed hull; singleton sources and singular
maps are supported. The report is not the correlated image, occupancy or topology.
One dyadic budget covers conversion and arithmetic; scalar refinement, exact
clone/drop and heap bytes retain native exclusions. Typed native errors preserve
their source chain. Explicit Hyperlattice/interval-hyperreal and companion/hyperreal
feature forwarding reuses the existing dependency graph. The root lockfile and
primitive defaults remain unchanged; the separate voxel lockfile is updated.

Six self-contained regressions preserve the original blocker and new success,
17 generated integer maps checked against independent eight-corner extrema,
rational/algebraic coefficients, reversed/degenerate inputs, source mutation
isolation, subsequent native interval mapping, exact-budget/one-unit-short failures,
size failure/retry and fresh-process reconstruction. Reconstruction is not serialized
proof replay. The final owner suite passes 84 tests. Strict stable/MSRV all-target
Clippy, rustdoc, default checks, Rust 1.95 native tests and WASM compilation pass.
Native lattice bridge tests and 13 downstream SDF voxel tests pass. An independent
Rust 1.95 path client consumes the transformed enclosure in a source-bound SDF
whole-cell inside proof; both original source associations remain available.
The primitive fixed backend passes Rust 1.88 ARM compilation. A Rust 1.95 ARM
attempt lacked its installed target and is not claimed successful. Primary
regression, formatting, diff and repository-budget checks pass.

Pinned CPU0/ASLR-disabled warm release probes include full report production and
destruction, excluding authored input construction. Native integer/rational/
algebraic/undecided reports have median times 8.762/9.131/12.218/12.457 us and
requested allocations/bytes 90/720, 96/912, 129/1896, 138/1968. These are not peak
or retained-memory bounds. Eight same-path baseline/candidate exact-mapping runs
retain identical allocation counts/bytes and independent literal outputs. Their
median candidate/baseline ratios are 0.99935/1.00023/1.00624 for integer/rational/
algebraic cases. The legacy all-feature probe grows 15,832 bytes. These short
measurements are not a speedup or global no-regression claim. Costs are accepted
for the demonstrated new enclosure completion; the old exact output lacks the
retained-source report and cannot complete the undecided-order witness.

All raw failures and corrections remain recorded, including array inference,
the initially invalid tightest-rounding fixture assumption, unavailable ARM target,
and a probe literal inference error. No existing assertion or policy was weakened.
The 41 staged extraction paths and eight Interface edits remain byte-identical;
firmware is unchanged and all 62 prior frozen stages verify. The full plan remains
active: migration cost gates, whole-resource closure, broader required adoption,
hardware and coordinated release are not complete. No push/publication occurred.

Evidence `docs/evidence/2026-11-15-interval-voxel-affine-enclosures`, local archive commit
`0bb2d1329f98eabcbde4b31f9578ceac706e9c2b`: 141 files / 12,040,903 bytes,
SHA-256 `3cc6a550c09b8ef63734319ccecd06837f01a186bb0046630cadd6c17992a90c`.


## 2026-11-15: machine-profile and firmware-lowering fuzz targets

Interface source `f5341f36803ad77c856313315d67ac8b3bb63f9a` adds the missing plan-14.3 targets in
an unpublished standalone fuzz workspace. The baseline is Interface73cc71b,
CSGRSd78473b and firmwareb9ecf8d. Only 31 new fuzz/ paths are committed: harness,
canonical configuration/provenance, 20 seeds, deterministic regressions, replay
utility, manifests and documentation. Production arithmetic, validation, policies,
canonical encodings and existing assertions remain unchanged.

Structured inputs cover both axes, 1–32 microsteps and exact uncertainty fractions
0–7 over 1–4. The calibration arm scales uncertainty to ppm; nonzero microstep
uncertainty remains an invalid configuration under the existing integer contract.
A raw single-wire-byte mutation arm recomputes the content digest. Independent
exact command-density formulae and fresh profile reconstruction check accepted
profiles. Bounded exact straight paths pass through actual scheduling, lowering,
canonical packaging, stepper preflight/cached execution and evidence replay.
Successful output preserves configuration identity, tick/segment invariants and
deterministic evidence bytes; altered evidence is rejected with either digest.

Five final Rust1.95 tests pass, including every one of 4,096 structured profile
combinations and fresh-process lowering. Strict all-target/all-feature Clippy,
rustdoc and formatting pass. Nightly address-instrumented builds and bounded
campaigns pass: 22,647 profile and 205 lowering invocations, with requested
60-second campaigns, 10-second input timeout and 8GiB RSS limits. Peak reported
instrumented RSS is 410/586MiB; these are observations, not internal resource bounds.
Saved-corpus replay distinguishes 95 completed profiles, 272 invalid configurations
and 32 invalid profiles. Lowering replay completes 72 cached executions, with
7 budget, 10 configuration and 5 lowering rejections. Saved-corpus counts do not
represent every fuzz invocation. No universal coverage or performance claim.

The first campaigns found a new harness bug: unreduced zero/nonzero fractions
were passed to canonical wire constructors. Original source/inputs/logs remain;
Hyperreal normalization and named permanent regressions now preserve exact values.
An initial regression expected nonzero microstep uncertainty to reach profile
validation; the existing configuration integer guard correctly rejects it earlier.
The final test asserts that precise rejection and adds valid calibration coverage.
The earlier zero-nominal encoder panic and Clippy correction are recorded too.
No production defect was established or validator relaxed. Nightly emits an
existing alumina-runtime fetch_update deprecation warning; Rust1.95 strict checks
are warning-free. An accidental +1.95 alias install is recorded; subsequent commands
use +1.95.0. No CI workflow changed, and no hosted, WASM, embedded or hardware
qualification is inferred from these host-only fuzz targets.

Separate isolated compatibility testing applies the unchanged pending 41-path
CSGRS and eight-path Interface patches. The locked invocation rejects the changed
dependency graph; offline resolution adds 11 Hyperdynamics lock lines and all five
tests then pass. That provisional lock delta is retained separately, and both
isolated worktrees are restored clean. The committed fuzz lock targets accepted
revisions. This compatibility result does not accept the migrations or resolve
their runtime/size gates. Both primary pending patches remain byte-identical,
firmware unchanged, and all 63 previous frozen stages verify. The full plan remains
active; broader migration, resource, hardware and coordinated release gates remain.
No push/publication occurred.

Evidence `docs/evidence/2026-11-15-interval-interface-machine-fuzz`, local archive commit
`693cf4038c69be4550d73b7bf59cf010d308ff07`: 639 files / 880,428 bytes,
SHA-256 `1681f93512b7d12d128f9c565fe1cb3dec69ceb1b17f9593e1b076494305bc29`.


## 2026-11-15: reject static-arity machine interval products

Phase 8 is rechecked on accepted CSGRS `b51440ab08eef4a140822dc7e8650af4a661934a`, Interface
`f5341f36803ad77c856313315d67ac8b3bb63f9a` and unchanged firmware
`b9ecf8df54723e532a42867a1d3c549f827d5f6d`, excluding the pending extraction.
The distinct hypothesis is that the rejected positive-product helper's runtime
slice loses the known four-factor transmission shape at its shared API boundary.
A generic AsRef argument preserves array type through monomorphization while
retaining runtime slices/vectors. It uses the identical product body, arithmetic
order and positivity checks. No inlining/unrolling directive, degree/factor limit,
compiler option, dependency version or acceptance threshold changes.

The missing positive-chain API is reproduced before edits. The previous dynamic
candidate is rebuilt as a control, not a retention attempt. It passes 11 owner
tests. The static candidate passes 12 across three harnesses: retained independent
625-case corner enumeration, a large-rational GMP oracle, generated output as input,
invalid/empty refinements, fresh-process original/dynamic/static dispatch equality,
and new independently checked static/runtime products of lengths 0,1,2,4,8,16.
Strict all-feature/all-target owner Clippy passes. An initial unsupported usize
conversion in the new test is corrected to u128; the failure remains recorded.
No existing assertion is removed or weakened.

All seven complete native profile/identity/allocation transcripts match baseline.
All seven WASM transcripts match their native counterpart, reject invalid indices,
and make zero host calls. These are consumer comparisons, not an independent
full-profile or canonical schedule proof.

| Artifact | Baseline bytes | Dynamic control | Static candidate |
| --- | ---: | ---: | ---: |
| Native profile executable | 810416 | 811296 | 811296 |
| WASM profile module | 1069027 | 1069992 | 1069992 |

Both static artifacts are byte-identical to their dynamic controls. The +880-byte
native and +965-byte WASM size failures reproduce. This API-boundary change has
no effect on these configured artifacts; it neither explains compiler internals
nor identifies the remaining cost cause. The candidate is rejected and fully
removed. No timing, owner-wide code-size, full-profile both-policy trace, full
schedule/browser, sanitizer, coverage or pre-push campaign follows the decisive
size rejection. Build duration and WASM fuel are not performance evidence.

Restored native/WASM builds reproduce baseline hashes byte for byte and complete
native output. Restored owner tests and both workspace formatting checks pass.
The three isolated source worktrees are clean. Primary's 41 staged paths and
Interface's eight pending edits remain byte-identical; firmware is unchanged.
All 64 previous frozen stages verify. The full goal remains active, including
machine arithmetic/error composition, other rejected migrations, extraction cost
gates, whole-resource closure, hardware and coordinated release. No source from
this experiment is retained or pushed.

Evidence `docs/evidence/2026-11-15-interval-profile-static-products`, local archive commit
`45bbdbd247556273f926334a4d5954ebe9ba5373`: 73 files / 747,732 bytes,
SHA-256 `8a4be98432a778397ef84eeaea1b321b7c4d0ea472a5a283eda19ef55d140a9f`. Complete patches, source, locks, initial failures,
full transcripts and restoration receipts are retained. Do not repeat the
static-arity variant without evidence distinguishing it from this result.


## 2026-11-15: attribute interval migration code growth and reduce adapter cost

Phase 8 is investigated on accepted CSGRS `7168a68127e04233141f0baa4c5a9468e0a2c41c`, Interface
`f5341f36803ad77c856313315d67ac8b3bb63f9a` and firmware
`b9ecf8df54723e532a42867a1d3c549f827d5f6d`. No candidate is retained. Linker maps
for baseline and each candidate reproduce the corresponding qualified stripped
native binaries byte for byte, so the maps describe those actual emitted artifacts.
Original frozen harnesses are read-only with locked dependencies and output outside
the record. Exact paths, profiles and the admitted CSGRS build cwd remain fixed.

The rejected dynamic helper increases .text by416 bytes and the mapped symbol
union by415. Normalized symbol-family changes sum to that union delta; padding is
separate. The transmission derive shrinks87 bytes. New positive-proof/component
adapters and changed result boundaries dominate code growth. Disassembly retains
an empty/null check and unreachable panic after an algebraically nonempty nominal
uncertainty construction. Extra rodata, relocations and unwind sections are recorded.
NOBITS/relro-padding reductions do not waive executable growth. This attribution is
not a runtime measurement, a universal compiler explanation or full-owner code-size
qualification.

Two evidence-directed variants are tested. The first transports construction
nonemptiness through a native checked NonEmptyRationalInterval, sharing Bounds and
the original positive comparison; general empty/signed semantics remain unchanged.
It eliminates the impossible-empty consumer branch. The missing carrier is reproduced
before edits. The second combines canonical fact lookup and scalar-set construction
in one private helper, matching the baseline's single result boundary instead of
the candidate's two. It removes only the unused private forwarding constructor.
Lookup, conversions, validation, predicates and arithmetic retain their order;
Interface retains nominal values, units, identities and physical-fact errors.
No inlining, unrolling, layout directive or compiler-policy change is introduced.

Fourteen final owner tests across four harnesses pass, including independent
integer/rational endpoint oracles, large-rational GMP products, full-width radius
cases, nonempty/general transport, generated output as input, invalid/empty
refinements and fresh-process original/shared/nonempty dispatch comparisons.
Strict all-feature/all-target owner Clippy passes. The final Interface-only regrouping
does not modify that owner code. All seven complete native profile/identity/allocation
transcripts match baseline. WASM profiles match native, reject invalid indices and
make zero host calls. These consumer comparisons are not independent full-profile
or canonical schedule proofs; full Interface both-policy traces remain unclaimed.

| Variant | Native bytes | Native delta | WASM bytes | WASM delta |
| --- | ---: | ---: | ---: | ---: |
| Baseline | 810416 | 0 | 1069027 | 0 |
| Dynamic control | 811296 | +880 | 1069992 | +965 |
| Nonempty transport | 810864 | +448 | 1069649 | +622 |
| Combined fact ingress | 810608 | +192 | 1069527 | +500 |

Both interventions reduce overhead but still fail the required size gates. The
final .text remains80 bytes larger (mapped symbol union +73), with unwind
.eh_frame/.eh_frame_hdr +104/+16. No timing, full owner/downstream code-size,
complete schedule/browser, sanitizer, coverage or pre-push campaign follows
rejection. Build duration and WASM fuel are not performance claims.

An initial combined-ingress run used Interface's cwd, whose configuration adds
WASM rustflags. Its confirmed process group was stopped and the incomplete attempt
discarded; both builds were repeated under a distinct controlled label from the
original CSGRS cwd. The runner now rejects another cwd. Raw logs and the rejected
attempt remain. No altered flags are used for acceptance.

All experimental source is removed. Restored owner tests and both workspace
format checks pass; fresh native/WASM artifacts and full native output reproduce
baseline byte for byte. Three isolated worktrees are clean. The primary41 and
Interface8 pending patches remain unchanged, as does firmware. All65 previous
frozen stages verify. The complete goal remains active, with migration, extraction,
resource, hardware and coordinated release requirements unresolved. No push.

Evidence `docs/evidence/2026-11-15-interval-profile-codegen-attribution`, local archive commit
`726c24d55a9f704c21ee3bf1df85e82383f52a42`: 96 files / 6,514,571 bytes,
SHA-256 `363df0c84750b94853059462f44d659a605743e45acf2eb22f182410fb50c2f2`. Source variants, maps, disassembly, commands,
full transcripts, rejected attempt and restoration receipts are retained.


## 2026-11-15: reject generic final construction for machine interval migration

Phase 8 remains open. On accepted CSGRS `94459e71e26c06d42cb2ad79101b4771523e200c`, Interface
`f5341f36803ad77c856313315d67ac8b3bb63f9a` and firmware
`b9ecf8df54723e532a42867a1d3c549f827d5f6d`, a distinct private generic-finalizer
experiment tests whether construction of final domain objects within validated
scalar ingress removes intermediate Result overhead. The preceding combined-ingress
prototype remains the control. Hyperinterval owns exact interval arithmetic;
Interface owns nominal values, units, identities and typed physical-fact errors.
Original lookup, conversion, validation, predicate and arithmetic order is retained.
No compiler hints, configuration changes or assertion weakening are introduced.

The missing helper witness fails with E0425 before implementation. It passes after
implementation, checking one finalizer call on success and none after missing facts
or nonpositive lower bounds. All19 machine-profile unit tests and14 owner tests
across four harnesses pass. The latter retain independent endpoint/corner and GMP
oracles, full-width uncertainty, output-as-input and fresh-process dispatch tests.
Strict owner all-feature/all-target Clippy and Interface core default-feature
all-target Clippy pass. Complete consumer both-policy trace qualification remains
unclaimed.

Baseline and control native/WASM binaries reproduce their prior hashes. All seven
full native profile/identity/allocation transcripts match baseline, and WASM outputs
match native with invalid indices rejected and zero host calls. Locked harnesses,
absolute dependency paths, Rust1.95.0, release profile and CSGRS build cwd are fixed.
These comparisons supplement independent owner oracles; they do not establish
full-profile or full-canonical-corpus correctness by themselves.

| Source | Native bytes | Delta | WASM bytes | Delta |
| --- | ---: | ---: | ---: | ---: |
| Baseline | 810416 | 0 | 1069027 | 0 |
| Combined-ingress control | 810608 | +192 | 1069527 | +500 |
| Generic final construction | 811424 | +1008 | 1070423 | +1396 |

The candidate fails both size gates and is rejected. Its linker-map build reproduces
the normal stripped native binary exactly. Native .text grows736 bytes (mapped
symbol union and normalized family delta sum +737; padding differs by one byte),
and unwind sections grow248/32 bytes. Positive scalar wrappers are405 bytes each,
versus275/69 in the preceding control. Constructor/refinement functions also become
separate emitted functions. This is artifact attribution, not a runtime or universal
compiler claim. No broad timing, whole-owner code-size, full browser/schedule,
sanitizer, coverage or pre-push campaign follows rejection.

All experimental source is removed. Restored owner tests and both workspace format
checks pass. Fresh native/WASM artifacts and full native output reproduce baseline
byte for byte. Three isolated worktrees are clean. The primary41 and Interface8
pending patches, firmware and all66 previous frozen stages remain unchanged.
No production implementation is retained and no phase is marked complete. The goal
remains active with migration, extraction, resource, hardware and coordinated
release work unresolved. No push or publication.

Evidence `docs/evidence/2026-11-15-interval-profile-final-construction`, local archive commit
`d94336575bf3bf300c913b3a23620b1fc72b144e`: 66 files / 3,148,345 bytes,
SHA-256 `0bb39825d350fa3199182fd0aafdac63b6efc773404c5c49f670fbbb517808d2`. Candidate source, missing witness,
commands, raw maps, complete transcripts and restoration receipts are retained.


## 2026-11-15: retain interval boundary composition witnesses

Source `36a091faed15475b730af9b1601ce7ef185e72f0` retains nine tests on accepted parent
`2e6787fb61b4339cd864b34d18112cc668229f8d`. Hyperinterval production source, manifest,
workspace lock and existing assertions remain unchanged. Current instrumentation
identified missing witnesses before edits; no behavior repair is claimed.

The new tests cover owned rational bounds and exact reconstruction, positive empty
products/quotients with invalid counterparts, empty integer products at every
storage width, fixed empty rescaling/reflection with scale and zero-denominator
validation, affine proposal/strict-float empty results with nonfinite arguments,
exact neighboring versus touching floats, split/extended empty and singleton
function chains, empty rational trig at extreme grids and zero budget, and distinct
public diagnostics retaining the typed scalar source. Independent set identities,
integer/rational expectations and zero/one function values support the assertions.
All original assertions and hard cases remain.

Fresh Rust1.95.0 all-feature library/integration instrumentation has the same
production denominator before and after the additions:

| Production metric | Baseline | New witnesses |
| --- | ---: | ---: |
| Lines | 4658/4740 (98.270%) | 4688/4740 (98.903%) |
| Regions | 9270/9891 (93.722%) | 9320/9891 (94.227%) |
| Reported functions | 493/494 | 494/494 |

Raw reports retain every Hyperinterval src file. Only the two cfg(test) modules
are separated in the explicitly named production subtotal; no uncovered production
path is excluded. LLVM diagnostic dumps identify all16 mismatch warnings as Rug
OptRational::parts/hash0 outside that denominator. Fifty-two production lines remain
reported uncovered. No branch counters are available. Macro/generic instantiations
and individual LCOV line counts do not have identical denominators; both original
reports remain. Reported function coverage is not universal/API/branch completion.

Ten qualification commands pass: all-target/all-feature tests, no-default tests,
strict Clippy at both boundaries, release witnesses, warning-denying docs,
examples/benches, workspace formatting, Rust1.88 primitive witnesses and stable1.99
all-feature witnesses. Seven independent feature runs pass for integer, fixed,
proposal/strict float, dyadic, rational and rational+dyadic. Integrated main-tree
witnesses, formatting and repository budget pass. One test-only wrong type-name
import produced E0432 before correction; the original failed log is retained.

Production source hashes are identical, so no executable algorithm, predicate order,
dependency or consumer is changed. No new downstream performance, fuzz, HIL,
full-platform or pre-push claim is made. Timings are command durations, not benchmark
results. No CI change, push or publication occurs. Main's41 staged paths and
Interface's8 pending paths remain byte-identical; firmware remains clean. All67
preceding frozen stages verify. Full coverage and the entire15-phase goal remain
open, including unresolved implementation, resource, migration and release gates.

Evidence `docs/evidence/2026-11-15-interval-coverage-closure`, local archive commit
`cd8f9b7977102e187676bd401ac986cb566a504d`: 70 files / 7,576,352 bytes,
SHA-256 `489259f892188e071c9715aef8d385ea40686e9a100069729a8efa9355a27f32`. Exact commands, aggregate profiles,
compressed exports, raw diagnostic logs, source patch and integration receipts are
retained. Bulky unmerged profiles remain external with paths/hashes; the aggregate
profile and source/build commands support replay without claiming archived binaries.
