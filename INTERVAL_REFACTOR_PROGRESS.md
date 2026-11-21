# Interval refactor progress

Contract: [implementation plan](docs/INTERVAL_REFACTOR_PLAN.md). The full
15-phase objective remains active. No phase is claimed complete.

Current goal-specific size acceptance: the user permits bounded growth for useful
functionality and reconsideration of rejected refactors, with per-target caps,
final-binary measurements, firmware flash headroom and cumulative accounting.
[Plan section 9.7](docs/INTERVAL_REFACTOR_PLAN.md#97-goal-authorized-bounded-code-and-binary-growth)
records this exception. The subsequent user authorization also permits small,
bounded memory/stack growth for useful functionality, under
[plan section 9.8](docs/INTERVAL_REFACTOR_PLAN.md#98-goal-authorized-bounded-memory-growth)
and the separate memory admission protocol. Runtime and all other gates remain
intact; historical rejection records below remain evidence of their original
evaluations.

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
| 0 inventory/baseline | Native coverage measured for Hyperinterval, Hyperdynamics and Hyperlattice Interval. Independent syntax traversal matched 2,807 declarations in 36 planned machine source files. Phase 7 now covers all 1,101 syntax declarations and 38 compiled macro properties in its 24 named Hypersolve sources. All 34 shared-timing evidence declarations and all 40 simulator FOC declarations have body reviews. | remaining body and consumer audits, sibling qualification, costs, coverage, fuzz and hashes |
| 1 skeleton/laws | local acceptance passed | hosted OS jobs remain unexecuted |
| 2 integer/fixed | local correctness/cost probes passed, including exact fixed-point conversion to rational and dyadic intervals | firmware cycles/HIL and coordinated baseline gaps remain |
| 3 rational/dyadic/ball | carriers, exact rational scalar scaling, exact dyadic scalar maps and direct rational-to-IEEE projection implemented and locally qualified | solver kernel migration, full coverage and consumer cost gates |
| 4 proposals/float | proposal wrappers and strict integer IEEE core implemented; x86_64, bare WASM, opt-in ordinary WASI Preview 1 and opt-in AArch64 GNU/Linux (QEMU only) locally qualified, including rounded higher powers, full-width exponents, tight rational endpoint projection and direct binary32/binary64 conversion with maintained browser, WASI and native fuzz qualification | full surface/coverage, further targets and shipped consumers |
| 5 elementary functions | abs/min/max, integer powers, square roots, exp/log/atan/sin/cos, split division, rational powers and tan/cot implemented on documented backends | final function/backend coverage, resource and consumer qualification |
| 6 lattice | Cartesian/affine carriers, widths, center/radius forms, norm/contraction evidence, source-bound Jacobian projections and FOC/Krawczyk clients implemented | broader authored-form composition, facts/specialization and full consumer/performance gates |
| 7 solver | source-bound Jacobian matrix adapter qualified; native and WASI inventories matched across the preceding 72 solver test executables on both compilers and all 73 with the general conic oracle on Rust 1.95, including nested coefficient scopes, produced denominator growth and 4,212 Horner dispatch windows; targeted Horner fits the native stack allowance but exceeds four release size accounts. The weighted inequality Jacobian trial fits its bounded stack allowance and exceeds a Hyperreal benchmark section cap. Quadratic alpha and univariate quadratic Krawczyk rows require certified comparisons. Simultaneously saturated coefficient receipts display a proven lower bound. The affine oracle checks every field of each published coordinate, including the proved prefix of an outside-box report. The certified-stage affine Krawczyk repair is merged (20 November) | preserve coefficient-growth diagnostic counts above the 32-bit word limit and satisfy migration size/performance gates with unchanged statuses, certificates and dispatch |
| 8 interface | BoundedMeasurement construction delegates to Hyperinterval. The two-device shared timer fixture pins the complete evidence bytes and ordered transcript hash. The full 199-case Interface core suite now runs on WASI, including machine-profile replay and TinyBee firmware actors. | machine-profile arithmetic, named budgets, full browser/schedule and canonical-profile gates |
| 9 firmware | clock/Q30/servo delegation retained; clock and host dq simulator counter publication repaired; independent current-widening witness retained; current five-board final images and matched clock-fix comparison measured with unchanged executable bytes | 4 MiB flash fit, board partition/reserve declarations, whole-call stack (including +16-byte local changes), MCU cycles/HIL and remaining audit/release gates |
| 10 predicate filters | initial native-stage experiments and exact admission corpus qualified; two placements rejected | useful native-stage proof/cost/trace admission and representative geometry workloads |
| 11 path dynamics | direct joins require certified evidence through lookahead replay; all 449 Hyperpath checks pass on native and WASI; solver zero-tangent and varying affine-box verdicts remain open alongside the measured extraction stack excess | implement and qualify domain-neutral ownership while preserving exact reports, dispatch, canonical lowering and owner runtime |
| 12 ODE/reachability | owned fields/Jacobians, Picard tubes/end enclosures, bounded native integration in retained exact state charts, rational clocks, RK proposals and finite epsilon endpoint covers implemented and locally qualified | wider reachability/wrapping, whole-resource closure and remaining platform/consumer qualification |
| 13 Taylor/adoption | exact coefficients/remainder, source-bound affine forms and nonlinear program composition, polynomial arithmetic over existing Taylor proofs, certified composition of retained Taylor models, validated trajectory Taylor steps and fixed-order whole-horizon composition, bounded automatic order/duration selection with local remainder limits, direct exact chart transport, certified QR frame construction and bounded two-chart automatic evolution with exact endpoint-diameter selection, fixed state charts/Picard and whole-horizon transport, native SDF cell/Taylor enclosures for smooth expressions, polynomial primitives and affine frames, plus bounded adaptive covers, a measured three-strategy dynamics corpus and circuit residual/Jacobian enclosures with component tolerance bindings plus continuous point-mass flow with retained force/mass evidence and sensitivity-retained noise columns for static affine field-image queries and explicit sharing of prepared trajectory orders, retained after local qualification, plus a measured external CAPD comparison on sixteen validated point/box IVPs and manually composed affine-noise flow with continuous two-rate qualification | higher-utility automatic frame strategies, automatic ODE noise orchestration and global sensitivity, automatic-controller cross-order preparation reuse, general arithmetic across nonlinear remainder models, wider adoption and broader competitive completed-workload portfolio qualification remain open |
| 14 release | Interface native/browser lint and local packaging; 21 packages reproduce byte-for-byte across clean checkouts with each of Rust 1.95 and 1.99, and all 34 required checks pass on both, including seven worked interval examples from the release archives, 42 interval feature checks, dedicated Hyperdynamics workflow, 65 Cartesian arithmetic/replay tests, the original scalar Rust and MPFR oracle suites, Hypersolve and Hyperdynamics numerical and fresh-process replay cases, and Hyperpath and Interface schedules locally qualified on WASI Preview 1 | whole-stack CI/feature/platform qualification, hosted/HIL consumers, publication and remaining package gates |

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

Enclosure uses `B(Ax+b)+c=(B A)x+(Bb+c)`, pulling coefficient bounds to the
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


## 2026-11-15: require DRC smoke fixtures and reproduce the native CPU blocker

Source `71377cb5f0007f1ff2ed03ec2a3c36ce99f95fb4` retains a qualification repair on accepted parent
`29e0e82f37b29b3f825aed25f3b2e2e52e842c78`. The current original KiCad smoke suite fails
at 130.443995189 worker seconds against its unchanged 120-second
limit. Both original tracked archives are present with recorded SHA-256 hashes;
this baseline excludes the pending 41-path extraction. The prior performance
blocker is reproduced on the current revision, not inferred from older evidence.

Before the repair, two new tests fail because the app smoke fixture helper silently
skips missing required archives, unavailable unzip, and corrupt archives. Workspace
CI now sets HYPERDRC_REQUIRE_COMPLEX_FIXTURES=1. Presence of that variable, including
an empty value, requires archive existence. Extraction errors for present archives
always fail. Optional missing archives remain skippable for package-only use, whose
manifest excludes the assets. Five fresh child-process cases cover required and
optional absence, empty flag value, and unavailable extraction in both modes; a
corrupt archive and exact valid payload are separate controls. The tests avoid
process-global environment mutation. Existing smoke bodies, original fixtures,
120-second limits, production geometry and predicate ordering remain unchanged.

The two regression tests pass after repair, as do all 41 app tests with required
mode enabled, the original Gerber smoke suite, release all-target/all-feature
Clippy with -D warnings, workspace formatting and whole-repository
zizmor --pedantic --strict-collection. The unchanged real KiCad smoke suite still
fails at 127.134745903 worker seconds. The timing gate is not
waived; no performance improvement or full native CI completion is claimed.
Integrated-source fixture witnesses, formatting and repository budget also pass.
All executed commands, failures and successful controls are retained separately.

This is a test/CI qualification repair, not an interval arithmetic or geometry
optimization. The separate layer-level unzip fixture helper remains unchanged;
this stage does not claim to harden every individually filtered fixture test or
establish that all packaged-crate tests pass without assets. Full-board minimum
copper-neck qualification, the full pre-push/platform matrix, outstanding costs,
resource contracts, migrations and release/HIL gates remain open. No push or
publication occurs. Main's 41 staged paths and Interface's 8 pending paths remain
byte-identical; firmware remains clean. All 68 preceding frozen stages verify.
The entire 15-phase goal remains active.

Evidence `docs/evidence/2026-11-15-interval-drc-current-admission`, local archive commit
`84304984a69084d9ebf849e7d0ef0e064f766468`: 49 files / 1,471,494 bytes,
SHA-256 `d4d8df62bc8b4ec930a7b5408b181e1e4b6517e0a53e84af031b8d64fbef743c`. The source patch, negative witness logs,
source-bound builds, fixture hashes, command outputs and integration receipt are
archived. Test binaries remain external with paths and hashes; no archived-binary
or reproducible timing equivalence claim is made.


## 2026-11-15: current DRC arrangement and scalar cost attribution

On accepted `e221a08381032ef42e588cca75d0f7b9212974be`, the prior source-qualified release test binary
from `71377cb5f0007f1ff2ed03ec2a3c36ce99f95fb4` was checked by SHA-256 and
sampled through the original KiCad smoke workload. The intervening commit changes
only the ledger and manifest. Original tracked archive hashes verify unchanged.
The debugger collected 63 worker-stack samples at a two-second period, exited
cleanly and recorded the inferior's existing CPU-gate failure. Its 129-second
instrumented duration is diagnostic, not a new acceptance timing. The prior
uninstrumented 130.443995189/127.134745903-second failures against 120 remain open.

Inclusive samples contain native contour admission 29 times, PCB region aggregation
26, structural exact sign recognition 19, wide rational GCD 16, minimum copper-neck
12, and unary candidate scheduling, pair preparation, split topology and fragment
selection 10 each. Contour bounds occur in seven samples. These overlapping counts
are not an exclusive time breakdown or a statistically established speedup.

Targeted body/caller review confirms the workload already shares copper-layer
construction through run_checks' OnceCell. BoardModel has public mutable sources;
a persistent board cache cannot be justified without source binding and invalidation.
The samples do not justify repeating the rejected nonzero-adjacency shortcut.
Candidate preparation includes native-arc materialization and scalar recognition;
further work should isolate temporary rational-curve promotion and classification
cost while preserving the existing arithmetic and proof contracts. This is evidence
constraining the next experiment, not acceptance of any cache or algorithm change.

No implementation, test, fixture, numerical policy, dispatch order or dependency
changes. No new owning/downstream test or full CI completion is claimed for this
read-only investigation. Main's 41 staged paths and Interface's 8 pending paths
remain byte-identical; firmware is clean. The full 15-phase objective remains
active, including solver and Interface migrations, pending dynamics extraction,
resource, performance, coordinated release and HIL gates. No push or publication.

Evidence `docs/evidence/2026-11-15-interval-drc-arrangement-profile`, local archive commit
`0bf6988be70e2ab599a3349ec6cbcf5acd05be96`: 13 files / 1,821,800 bytes,
SHA-256 `aeb31e89a259b69d582a02a28d26c0c3f09c962a860eef4fa393357609971524`. Includes the bounded sampling script,
raw debugger stream, all original stacks, inclusive attribution, executable and
fixture hashes, reviewed-source hashes and preserved repository state. The binary
remains external at its hash-bound path. Source inventory is not whole-file review.


## 2026-11-15: borrowed quadratic promotion rejected on repeated owner cost

On accepted source parent `54e6337fe42ea4ac92f5585cde27349e1f7ad79c`, an isolated
candidate added borrowed rational-quadratic promotion and removed an intermediate
quadratic clone in native arc preparation. Owned promotion forwarded to the same
conversion body. Constructor decisions, exact controls, endpoints and retained
conic identities were preserved in the finite acceptance corpus.

The candidate is rejected. In the unchanged complete arc benchmark, sparse path
intersection over 256 arcs was slower in five of six initial controlled pairs.
A further 20 balanced pairs, with ten of each order shuffled using seed 20261115
and all runs pinned to CPU 2, found the candidate slower in 19 pairs. The median
paired ratio was 1.0170898908727681; mean paired ratio was 1.0174315890365915.
All non-timing output matches. Total arc CPU averaged about 1.3% lower, but that
aggregate and smaller binaries do not satisfy the owning-workload no-regression
gate. The initial overall arc and cached-decomposition concerns did not reproduce
in the larger series and are not the rejection rationale. No instruction-level
cause is established; no arbitrary layout or inline tuning is attempted.

Formatting, owner all-target/all-feature strict Clippy, both-policy promotion and
arc witnesses, and Rust 1.95 no-default controls pass. Full Hypercurve release
validation reports 3263 passes, zero failures and 37 ignored tests across 128
suites. Strict/approximate dispatch transcripts and full fixed-input DRC reports
match; an earlier path-dependent report mismatch and its corrected control remain
retained. All 20 downstream successive-operation tests, including long ignored
chains, pass. These finite checks are not a universal closure proof.

The initial owner portfolio completed 48 successful executions; both full editing
executions reached their actual 900-second external measurement limit. They are
incomplete measurements, not geometry failures or completed-workload evidence.
The native 100 MB pathological pair reports 268 decided Boolean outcomes and zero
blockers under STRICT, with matching counts and checksums. Its flattened comparison
is separate. Five comparative calibration iteration counts and rounded build RSS
observations of 86.6/86.7 MiB remain explicit rather than being masked.

The native CLI and all 25 owner benchmark artifacts show no file/text/named-text
growth. Paired WASM shrinks by 101 bytes; the attributed successive executable by
11360 bytes, including 20080 fewer text-section bytes. No size result is used to
waive the repeated runtime regression. All six KiCad timings still fail the
unchanged 120-worker-second gate (baseline/candidate means 127.6513528107 and
128.1120260313 seconds). No DRC speedup or release qualification is claimed.
Prepared larger-budget editing retries and full successive runtime comparisons
were not run after rejection; their acceptance obligations are not marked passed.

The isolated candidate is restored clean, with the rejected patch, final source,
all actual attempts and commands retained. Main's 41 staged paths and Interface's
eight pending paths remain byte-identical; firmware remains clean. All 70 prior
frozen stages verify unchanged. The full interval plan, pending migrations and
extraction, resource, performance, coordinated release and HIL gates remain open.
No push or publication occurs.

Evidence `docs/evidence/2026-11-15-interval-drc-borrowed-promotion`, local archive commit
`c7aa15fae3b5b5b7ec0f872afb167a40ed1f8d61`: 533 files / 19,117,162 bytes,
SHA-256 `022fcda07edb594db9d0a96175a62d1926061b495d62101f08db52f00e585773`. Includes raw timing series, failures, source-bound builds,
reports, witnesses, dispatch records, restoration receipts and losslessly compressed
symbol maps. Executables remain external at hash-bound paths. The earlier superseded
build helper and prepared unexecuted retries are identified separately from executed
work; source observations are not treated as implementation or performance proof.

## 2026-11-15: reject typed fraction ingress in the complete machine-profile consumer

Phase 8 is rechecked on accepted CSGRS `97713a6102d2b7bb090d6c7ce33e6e156f816588`,
Interface `f5341f36803ad77c856313315d67ac8b3bb63f9a`, and firmware
`b9ecf8df54723e532a42867a1d3c549f827d5f6d` in clean isolated checkouts. The scalar
owner lacked typed unsigned primitive fraction ingress despite the consumer's
validated nonzero denominator. The missing witness first reproduces E0277.
Properties P1, P4, P6, P10, P25, P27 and P36 govern this experiment.

The candidate adds `From<(u64, NonZeroU64)>` at Hyperreal's existing fraction
reducer without changing signed fallible fraction or reduction order. It reapplies
the previously rejected nonempty/positive interval migration and removes impossible
error transport from its unsigned radius factory. No earlier experimental API is
assumed accepted. The consumer retains its validation guards, nominal values,
physical identity, operation order and original fixtures.

Both scalar tests pass: 240 full-u64 pairs match independent canonical components
and quotient identities; 390 fresh-process construction traces match the historical
signed-fraction domain, including all rational statistics. Display/debug and serde
bytes match there. Sixteen interval tests pass with independent exact/GMP oracles,
empty/invalid distinctions, subsequent arithmetic and unchanged dispatch assertions.
Rust 1.95 all-feature/all-target Clippy with `-D warnings` passes for both owners;
both workspace format checks pass.

| Complete profile consumer | Baseline bytes | Candidate bytes | Delta |
| --- | ---: | ---: | ---: |
| Native | 810416 | 810752 | +336 |
| WASM | 1069027 | 1069529 | +502 |

The seven-case native profile, identity and allocation transcripts are byte-identical.
WASM replays match native, retain invalid-case behavior and make zero host calls.
These finite comparisons are supplemental; they are not an independent full-profile
proof or complete canonical schedule/policy qualification. Both artifacts fail the
no-growth gate, so the combined candidate and its scalar API are rejected. No
instruction-level cause or runtime speedup is established. Build duration and WASM
fuel are not performance evidence. Full Interface policy/schedule/browser, owner
runtime, successive-operation, monomorphized-size and pre-push campaigns do not follow
this early rejection; no arbitrary layout/inlining tuning or weakened gate is used.

All eleven changed source files and reapplicable patches are retained as evidence.
The isolated worktrees are restored clean. Restored scalar tests (23), rational
interval tests (5), and both format checks pass. Fresh restored native/WASM artifacts
and native output reproduce baseline byte for byte. The primary 41 staged and
Interface 8 pending paths remain byte-identical; firmware stays clean. All 71 prior
frozen stages verify unchanged. The full goal remains active; Phase 8's remaining
arithmetic, retained-proof ownership and named-budget work are not declared complete.
No push or publication.

Evidence `docs/evidence/2026-11-15-interval-profile-typed-fraction`, local archive commit
`224455e99bb2a6846e9ce9d1f35718cbc44bc969`: 91 files / 1,593,253 bytes,
SHA-256 `cae451f595416064198ca8f131c1441e2fc98ba944a9d0f5df005b8238f5f9f7`.

## 2026-11-15: retained machine interval ownership rejected on complete-consumer size

Phase 8, P1/P4/P6/P10/P25/P27/P36. At accepted CSGRS
`217bdda844c040be8c6909bce503686744cb1915`, Interface
`f5341f36803ad77c856313315d67ac8b3bb63f9a` and firmware
`b9ecf8df54723e532a42867a1d3c549f827d5f6d`, a distinct isolated candidate retains
shared nonempty interval storage inside the physical `ExactInterval` wrapper.
Positive products and quotients transport the same bounds into that owner instead
of unpacking them after temporary construction. A scalar-owned positive rational
witness lets nominal and shared bound divisions consume one converted nonzero
integer, preserving nominal/lower/upper order. Nominal values and physical identity
remain in Interface; the core interval has no nominal value. The rejected typed
fraction API and generic finalizers are not reintroduced.

The missing witness first fails with E0432. Final tests pass: 19 interval tests,
including original independent exact/GMP expectations, a new 1,200-digit repeated
division oracle, and 70 complete division-dispatch comparisons; one full-width
scalar witness; one Interface Debug/size/const-getter/scaling witness; all 18
existing machine-profile tests. Both owners pass Rust 1.95 all-feature/all-target
Clippy with `-D warnings`, and both workspace format checks pass. Existing
assertions, validation guards and operation order remain intact.

| Complete seven-profile consumer | Baseline bytes | Candidate bytes | Delta |
| --- | ---: | ---: | ---: |
| Native | 810416 | 810864 | +448 |
| WASM | 1069027 | 1069760 | +733 |

Native profile, configuration identity and allocation transcripts are byte-identical.
WASM replays native fields and invalid cases with zero host calls. These finite
comparisons do not establish full policy/canonical schedule/browser equivalence.
Both artifacts fail the no-growth gate; neither the ownership migration nor its
positive-scalar API is retained.

A candidate native link map reproduces the measured executable exactly. The prior
baseline map is reused only after its executable SHA matches the current baseline.
Native `.text` grows 352 bytes, the mapped function-address union grows 360 and
unwind sections grow 104. The largest growth lies in separated scalar-ingress
helpers. The main derivation symbol shrinks 95 bytes; manual Debug grows 31,
insufficient alone to explain the failed whole-consumer gate. Relro padding shrinks
448 bytes; this is not a heap-memory improvement. Native size equals the previous
rejected nonempty prototype but contents differ; WASM is 111 bytes larger. No
runtime benefit, instruction-level cost attribution, or zero cost for inlined
operations is inferred. No speculative Debug/layout/inlining variation follows.
Full owner runtime, successive-operation, whole-owner monomorphized-size and
policy/schedule/browser campaigns do not follow the decisive early rejection.

All 13 changed source files and reapplicable patches are preserved. Restored
worktrees are clean; all 18 original machine-profile tests, five rational interval
tests and both format checks pass. Fresh restored native/WASM artifacts and native
output reproduce baseline byte for byte. The primary 41 staged and Interface 8
pending paths remain byte-identical; firmware is clean. All 72 prior frozen stages
verify unchanged. Phase 8 and the complete fifteen-phase objective remain open.
A separate read-only review identifies repeated coefficient-prefix preparation in
the adaptive Taylor controller as a concrete remaining Phase 13 investigation;
no implementation or cost claim is made for it. No push or publication.

Evidence `docs/evidence/2026-11-15-interval-profile-owned-intervals`, local archive commit
`3114a7190c1dd122167ba1f62caf6bca38443bda`: 108 files / 3,765,395 bytes,
SHA-256 `850f1cb8f6d68ad01d20702c1a2d4d0b4b9102ba799afb4a643ddcf9cc9f517f`.

## 2026-11-15: adaptive coefficient ownership rejected on existing binary size

Phase 13, P1/P4/P5/P6/P8-P10/P17-P20/P27/P36. Accepted source
`d17bd974757330dd739a23556d409539e8e9d7cb` reproduces the missing complete
coefficient-association sharing witness: restricting a generated Taylor stage
copies its association payload. The prior adaptive-order-sharing experiment was
found during history review; its rejected extraction and timing results remain
binding. This candidate tests a distinct ownership change, wrapping each immutable
coefficient association in one Arc, together with that explicit shared preparation
strategy. Source-only zero-order extension uses the existing fresh preparation;
no generated prefix is lost. Existing predicate order and logical-work charges
stay intact. No eager maximum order or alternate differentiation kernel is added.

Twenty candidate tests pass across four binaries, including the original exact
coefficient, full history, resource/cancellation/retry and preserved hard-case
checks. Two observers match all 373,388 baseline graph/report, dispatch and work
artifact bytes for constant, exponential and Riccati fields. Owner Rust 1.95
Clippy with adaptive-taylor/dispatch-trace and all targets passes with -D warnings.
These are finite strict-enclosure comparisons, not full two-policy geometry
qualification. The stage layout shrinks from 104 to 48 bytes for one state
component, but the new association allocation prevents inferring a total-memory
benefit from that number.

| Existing release executable | Raw byte delta | .text delta |
| --- | ---: | ---: |
| adaptive_taylor | +4264 | +1664 |
| taylor_orders | +544 | +144 |
| taylor_flow | +1528 | +272 |
| taylor_evolution | +240 | +112 |
| qr_evolution | -8048 | -2032 |
| hyperdynamics, coordinate_transition, qr_coordinates | 0 | 0 |

Four existing executables fail the no-growth gate. Reductions in other binaries
or named owner symbols do not waive that result. The combined ownership/reuse
candidate is rejected; no allocation or runtime benefit is claimed. Full runtime,
successive-operation, downstream, coverage, fuzz and WASM campaigns do not follow
this decisive early rejection. No speculative layout or inlining variation is
used to evade it.

All five tracked edits and seven experiment-only files are saved in the archive.
The isolated checkout is restored clean. Original adaptive tests (9), order tests
(8), and workspace formatting pass; all eight rebuilt benchmark binaries reproduce
the baseline SHA-256 and sizes exactly. Primary 41 staged and Interface 8 pending
paths remain byte-identical; firmware stays clean. All 73 prior frozen stages
verify unchanged. Automatic cross-order reuse and the full fifteen-phase goal
remain open. No push or publication.

Evidence `docs/evidence/2026-11-15-interval-adaptive-stage-ownership`, local archive commit
`3b6b05b722c87740871f81dbe7e6d7a9b75c15c1`: 87 files / 17,444,948 bytes,
SHA-256 `c4d79b079bc26bc2575abe1c7932241326bb0b3b07471f24004f7325047e70b7`.

## 2026-11-15: external validated-dynamics comparison retained

Phase 13, P1/P6/P8/P19/P27/P35-P37. Production commit
`1ed29262bf80d2059f0d296dfa3fe46a5987d60d` adds a maintained standalone benchmark
and [comparison report](docs/research/validated-dynamics-comparison.md), without
changing production mathematics. Clean accepted source `e4d12c6f9` supplies
Hyperdynamics; the pending extraction is excluded. Official CAPD 6.1.0 source
`37057e5899bccaa6cd5701057b7e6ea5e232ef8b` is a separate-process comparator,
using native intervals. No third-party source or binary becomes a dependency
of the Hyper stack or is copied into the evidence archive.

Both solvers complete all eight constant/exponential/Riccati/oscillator point/box
IVPs on exact horizon [0,1/4]. Inputs, positive contiguous time and every retained
step <=1/16 are checked. Independent rational analytic expectations certify the
endpoint and continuous tube extrema, including the oscillator's interior maximum.
Both meet the unchanged per-coordinate endpoint width 1/64 and whole-tube width 1
limits. Full native evidence, materialized output and destruction remain in timed
work; fresh preparation and prepared reuse are measured separately. No sampled
path or solver-to-solver agreement supplies the oracle.

CPU0/ASLR-disabled ABBA sweeps record 320 samples of 100 integrations each. All 16
outcomes remain exactly unchanged within each adapter across fresh/reused and
measured runs. CAPD has a lower median latency in every configuration and narrower
nonconstant point enclosures. For example, oscillator-point prepared reuse is
1191.731 us Hyperdynamics / 55.738 us CAPD, while fresh work is 1333.611 us / 100.494 us.
The report preserves every row, actual widths, step counts, raw samples and
configuration limits. This identifies a concrete current cost gap; it does not
establish universal competitiveness, equal proof serialization, or memory/resource
API equivalence. Hyperdynamics order 3 and CAPD order 12 are explicit configurations,
not a claim of optimal tuning.

The acceptance audit caught and fixed a comparator-adapter error: CAPD TimeMap
overrides the solver's maximum-step setting. Six earlier two-piece outputs could
not satisfy the declared cap. Those runs remain archived as ineligible; only the
corrected exact-checkpoint/per-piece-audit campaign supplies reported timings.
Pre-fix source reconstruction matches measured SHA-256. A metadata key collision
was also fixed. No workload, width threshold or assertion was weakened.

All 42 upstream CAPD tests pass after its complete build; an earlier link/test race
is retained separately. Upstream's static-array test emits a GCC copy warning,
which is recorded and not claimed fixed. The final C++ adapter passes warnings as
errors with external headers designated system headers. Rust 1.95 standalone Clippy,
formatting, workspace formatting and repository budget pass. Nine corrupted-corpus
controls reject wrong inputs/endpoints/tubes, oversized steps, missing/duplicate
cases and width/horizon failures. Exact square-root controls cover 1500 rational
values and three series identities pass. No full fuzz/coverage/platform/HIL or
allocation comparison is claimed for these benchmark-only additions.

Main/isolated committed trees match. Measured runtime source hashes match the
committed files. Primary 41 staged and Interface 8 pending paths remain byte-identical;
firmware and isolated worktrees are clean. All 74 previous frozen stages verify
unchanged. Wider/stress portfolios, profiling this measured cost gap and all other
remaining plan gates stay open. No push or publication.

Evidence `docs/evidence/2026-11-15-interval-external-dynamics`, local archive commit
`27ae2575dffb0281865a3a45b65fc4c276476a62`: 92 files / 664,056 bytes,
SHA-256 `d550132d0ba9470f62905965c65665e3b48fcfaf0e89314a2ea772af7b26e54f`.

## 2026-11-15: profiled finer-grid regridding rejected on scalar latency

Phases 3/13, Hyperinterval owner, P1/P4-P6/P10/P19/P27/P36. The unchanged
accepted external consumer at `f21444763` completes its eight-IVP diagnostic
profile with 612,282,606 instructions. Its transcript matches the prior qualified
corpus and passes the independent analytic oracle. Regridding accounts for
63,028,733 inclusive instructions; inclusive attribution overlaps and this whole
process includes warmup, replay, serialization and destruction.

A missing allocation witness reproduces avoidable fine-grid mantissa expansion:
[-7,9]*2^-3 regridded to -67 allocates 48 bytes versus 16 for a source clone.
The candidate shares the existing alignment reservation calculation and copies
an already canonical interval for equal/finer grids. Exact outputs, conservative
charges, typed failures and call ordering remain unchanged. All 18 allocation
cases match clone costs. The exact identity includes zero/empty and the i32::MAX
normalization cap; it does not change coarse-grid rounding or budget semantics.

The candidate passes 2,020 core tests, 10 B-rep interval tests, one native curve
consumer, 186 SDF tests, 448 physics/circuit tests, 87 Rust 1.88 tests, strict
all-target core Clippy, no_std WASM/Thumb compilation and formatting. Complete
preparation/report/dispatch/work artifacts match all 373,388 baseline bytes;
new direct regrid/alignment count assertions remain zero. All 25 measured
binaries, .text sections and named code-symbol totals are nonincreasing, and
successive_operations is byte-identical. These scoped checks do not establish
full hosted/HIL, fuzz, coverage or pre-push qualification.

Rejected under the unchanged owning-benchmark performance gate. The external
ABBA run records 640 samples of 100 integrations with unchanged exact outputs;
all 16 fresh/prepared medians improve 3.6–13.1%. Whole-process instructions fall
7.23%. However, the owner campaign preserves all 380 exact/checksum/work rows
and exposes slower logarithms. At precision -256, dyadic wide log rises from
439.183 us to 454.986 us (+3.60%). Independent focused ABBA confirmation gives
439.030 us to 455.263 us (+3.70%); the partial-domain row rises 3.99%. Other
unfavorable observations remain in the full summaries. The native latency cause
is not established; smaller binaries and consumer wins do not waive the gate.
No arbitrary inlining/layout retry, threshold change or weakened fixture follows.

Both tracked edits and both experimental test files are archived and removed
from the isolated checkout. Original owner tests (247) and formatting pass;
all 25 rebuilt binaries match baseline bytes, and restored dynamics certificates
again match and pass the analytic oracle. Primary 41 staged and Interface 8
pending paths remain byte-identical; firmware and isolated worktrees are clean.
The allocation issue and all remaining plan gates stay open. No production
optimization, push or publication is retained.

Evidence `docs/evidence/2026-11-15-interval-dyadic-regrid`, local archive commit
`14ce442c6a28e65a7d8083b8b64e00c8532a044b`: 351 files / 5,881,523 bytes,
SHA-256 `14b9b7784a1ed4b2da11d8532f27f33fdef824364711732ebe455830619ba5f3`.

## 2026-11-15: sensitivity and matrix scaling prototypes rejected on size

Phases 6/13, Hyperlattice-interval and Hyperdynamics owners, P1/P5/P6/P8-P10/
P17-P20/P27/P36/P37. Inspection at `c64758fbc` confirms manual affine flow-noise
already exists. The new experiment instead reproduces missing public APIs for
whole-horizon initial-state sensitivity and matrix scaling. Missing convenience
methods do not imply that existing lower-level operations cannot express them.

The sensitivity prototype consumes certified Picard C1/Jacobian evidence,
encloses ordered variational-series terms plus a norm tail, and composes local
transitions on the left. One four-segment uncertain-shear witness passes.
This does not establish general sensitivity support: the planned noncommuting,
nonlinear, resource/cancellation, replay and physical-consumer corpus was not
completed before rejection. The combined prototype grows the existing
flow_noise executable by 800 raw bytes / 496 .text bytes and qr_coordinates
by 24 raw bytes / unchanged .text. Debug/drop symbol-size changes are retained.

The matrix-only candidate uses existing scalar multiplication and the owner's
fallible-array builder, with one row-major shared context. Its 92 owner tests
and 1,674 Hypersolve/Hyperdynamics downstream tests pass. Acceptance includes
independent endpoint oracles across families, generated product -> scale ->
vector consumption, fixed rounding/overflow, empty/unit products, exact dyadic
budget exhaustion/retry, zero primitive allocations and fresh-process replay.
It nevertheless grows reachability by 8 raw bytes and qr_coordinates by 24.
Only .strtab sizes grow (+14 and +26); named code-symbol sizes are unchanged,
and inspected allocated sections differ only in build ID. This is a raw-size
gate failure, not evidence of a runtime slowdown. No timing claim is made.

Both prototypes and all candidate tests are archived and removed. No layout or
inlining retry, stripped-size substitution or gate weakening follows. All 26
restored release metadata captures match baseline hashes and sizes. The driver
saved 25 distinct target names because taylor_orders is both an example and a
benchmark; comparison by original executable path covers both metadata rows,
while the overwritten physical artifact is explicitly recorded as a limitation.
Restored owner tests (89), two fixed dispatch controls and owner/dynamics
formatting pass. This is scoped validation, not full CI/MSRV/portable/fuzz,
coverage, successive_operations or latency qualification of the rejected APIs.

Primary 41 staged and Interface 8 pending paths remain byte-identical;
firmware and the isolated checkout remain clean. All 76 prior frozen records
verify unchanged. Neither extension nor any phase is marked complete, and no
production/test change, push or publication is retained. The full goal stays active.

Evidence `docs/evidence/2026-11-15-interval-flow-sensitivity`, local archive commit
`867f9efa52d49677c1d0e1b17289a6fa77eca859`: 179 files / 6,810,194 bytes,
SHA-256 `3765bdd254395fcad1627a2992ede2e67eee2972d3d56add521b9d7865265766`.

## 2026-11-15: sixteen-case external dynamics corpus qualified

Phase 13, standalone benchmark/validation owners, P1/P6/P8/P19/P27/P35-P37.
Implementation `17fbc05837920ce39891beb8bd903c250c9699b9` adds point and box
cases for `x'=t*x`, `x'=1-8t`, `(x',y',z')=(y,t*z,0)`, and
`(x',y')=(-x,-8y)`. Independent analytic bounds include the turnaround's
interior maximum and the noncommuting system's algebraic interior minimum.
The fixtures were specified before implementation: horizon `[0,1/4]`, radius
`2^-10`, endpoint width `1/64`, tube width `1`, actual step cap `1/16` except
`1/64` for the two-rate case. Existing Taylor orders remain 3 and 12.
All original eight IVPs and each solver's exact certificate bytes are preserved.

All 32 solver/case outcomes complete and pass independent exact rational
containment, width and continuous-time checks. The CPU-0, ASLR-disabled ABBA
campaign records 640 samples and 64,000 completed integrations, including native
evidence, endpoint/tube materialization and destruction. CAPD has lower median
latency in all 32 matched case/mode configurations. Hyperdynamics has the tighter
noncommuting point endpoint, about `5.29e-23` versus `8.88e-16`. Its uncertain
two-rate endpoint width is `0.014473501`, below but near the unchanged `0.015625`
limit, versus CAPD's `0.0015210953`. This supplies a concrete wrapping workload;
it does not establish universal stiffness, long-time or resource guarantees.

Two reproduced benchmark-validation gaps are fixed: optimized Python could
silently disable oracle assertions, and timing collection checked only total
record count. The oracle now refuses optimized Python; the runner requires
every case/mode/sample exactly once, matching solver/iteration counts and
positive integer timing values. Nine Python tests pass, including 1,500 exact
square-root controls. All 17 corrupted-corpus controls pass (the original nine
plus eight new controls), as do Rust 1.95 Clippy with warnings denied, formatting,
C++ warnings-as-errors, all 42 pinned CAPD native tests, diff and repository-budget
checks. The first extension pilot's CAPD signed-zero serialization difference
is retained and corrected; only the final `campaign` supplies published tables.

The [comparison report](docs/research/validated-dynamics-comparison.md) records
all new latency/quality rows and limits. Measured sources and binaries match the
committed implementation. No production mathematical library or feature changed.
Main 41 staged and Interface 8 pending paths remain byte-identical; firmware,
CAPD and the isolated checkout are clean. All 77 earlier frozen records verify
unchanged. Full-stack/hosted/HIL and remaining plan gates stay open; neither a
phase nor the full goal is marked complete. No push or publication.

Evidence `docs/evidence/2026-11-15-interval-external-dynamics-extended`, local archive commit
`701e672ceb746e72fe5914eab720f21fd2d5e8ab`: 93 files / 649,328 bytes,
SHA-256 `bee040027bb858af8ee74f5b5da8040dc6139204d16af4985402d6537c090d64`.

## 2026-11-15: continuous two-rate noise chains qualified; clone candidate restored

Phase 13, Hyperdynamics/Hypersolve owners, P1/P4/P5/P6/P8-P10/P18-P20/P36/P37.
Implementation `e139885a6e44b553a031fd114ff0615c0c1d4c52` qualifies existing
manual affine-noise propagation on both accepted two-rate point/box IVPs.
Each chain completes sixteen order-3 steps of `1/64`, retaining two noise
columns. Native Cartesian candidates are untrusted proposals: every noise step
re-certifies the actual incoming outer state. The maintained witness checks
every endpoint and continuous segment against independent rational exponential
bounds, materializes the whole tube, and preserves original source/proof identity,
nonzero residuals and fresh residual inputs through subsequent operations.
The existing adaptive-Taylor series helper moves into shared test support;
all original assertions remain. No mathematical production code is retained.

Final noise endpoint widths are `(4.98832206e-10, 3.51031049e-6)` for the point
input and `(0.00152109578, 0.000267950531)` for the box, versus Cartesian box
widths `(0.00250786278, 0.01447350104)`. Every continuous tube and the full hull
pass independent extrema checks. Maintained noise endpoint limits are stricter
than the unchanged external comparison contract: `(2^-29,2^-17)` for the point
and `(2^-9,2^-11)` for the box. These measurements establish this finite workload,
not general stiffness/wrapping closure or an automatic noise controller.
The [comparison report](docs/research/validated-dynamics-comparison.md) separates
noise-chain work counters from preparation, proposal and publication budgets.
No matched competitive noise latency or total CPU/heap bound is claimed.

Profiling identifies repeated Taylor construction as a substantial cost. One
candidate removes the node clone used only to schedule derivative children,
preserving visit order and every logical resource charge. Its 448 coefficient
control records (exact graph/output, both-policy receipts/dispatch, work and size
boundaries) and 36 wrapping records excluding timing are identical. The named
coefficient witness saves one 200-byte allocation per construction, with unchanged
peak/live bytes. Nevertheless `derive` grows from `0xc85` to `0xcb5` bytes, and
multiple old consumers gain 48 .text bytes and raw-file bytes. The candidate is
rejected without layout/inlining retries or stripped-size substitution. All 42
restored release artifacts match baseline hashes, with distinct source-path keys
for colliding example/benchmark names. Whole-process Callgrind instructions fall
465,881,247 -> 464,748,970; that does not establish a latency improvement or
supersede the failed gate. Full benchmarks/successive_operations were not run
for the rejected candidate.

Validation passes: 1,676 Hyperdynamics/Hypersolve tests and doctests in 84 suites,
zero failures/ignored cases; Rust 1.95 strict Clippy with all features/targets;
whole-workspace formatting; noise-only (13), adaptive-only (8), and composed-noise
(14) feature witnesses; diff and repository-budget checks. No full CI, new fuzz/
coverage campaign, hosted target, HIL, or broader release qualification is claimed.
All 78 prior evidence stages verify unchanged. Main's 41 staged and Interface's
eight pending paths remain protected by exact patch hashes. No push/publication,
phase completion or full-goal completion. The status table now distinguishes
implemented manual propagation from open automatic orchestration/global sensitivity.

Evidence `docs/evidence/2026-11-15-interval-flow-wrapping-diagnostic`, local archive commit
`af19ca2bbcbc7737547a94ff53fa69b17fd25e1d`: 202 files / 2,429,217 bytes,
SHA-256 `c1553e9b5ffc9a118a4c6462f503c4a645c74674f9240cee78d088d65bc9c385`.

## 2026-11-15: clock inventory review fixes partial publication on count exhaustion

Phases 0/9, alumina-clock owner, P6/P19/P27/P31-P37. Firmware implementation
`2d68e82b7341db7b747b20e4ad1455a4c1113a00`; inventory/audit implementation
`182b9d072c28d3121e99228d5b21e09f2446e1d5`. The previous turn was progress;
this turn returns to the required source inventory and produces a correctness fix.
All 52 original firmware inventory entries were still lexical/unreviewed. Body
review of the clock owner found a concrete failure: at accepted_samples=u32::MAX,
`observe` returns Arithmetic after replacing the retained model's newest sample
identity/timestamps. The pre-edit regression reproduces this on `b9ecf8df5`.
The Interface client updates its latest heartbeat only after successful observation,
so that partial estimator update breaks its source/report publication boundary.

The fix checks the prospective count after all original validation/intersection,
before assigning any model field. The last legal increment still succeeds;
earlier boot/policy errors retain precedence and diagnostic counter behavior.
The regression checks entire-state and published-estimate preservation, repeated
failure and explicit reset/reconstruction. No saturation, guessed result, new
error classification or dependency is introduced. This repairs erroneous failure
behavior; it is not presented as an identical-behavior arithmetic refactor.

The [clock ownership report](docs/research/firmware-clock-interval-audit.md) and
inventory now cover 13 source-located declarations: the two original clock rows
plus eleven omitted state/report/operation declarations. Generic integer range
intersection, affine arithmetic and directed quotient authority remain in
Hyperinterval; boot, causality, health/deadline policy and proof reports remain
in alumina-clock. `scaled_offset` is exact point arithmetic, not a duplicate
interval carrier. Selected Interface and simulator consumers were read. A
mechanical verifier checks source hashes/locators and proves all 1,512 non-clock
original rows unchanged. The other 50 original firmware entries remain open;
no full firmware inventory or phase completion is claimed.

Validation: Rust 1.88 native firmware tests/doctests pass (602, 55 suites), strict
all-target Clippy and workspace formatting pass; Interface client tests/doctests
pass on Rust 1.95 (90, 3 suites), as does its WASM check. Clock builds pass for
WASM/Thumb with Rust 1.88 and Xtensa ESP32 with ESP Rust 1.90. The normal dependency
tree remains no_std protocol plus integer-only Hyperinterval. Existing 24-case
clock-probe output/wire/query/rejection records (481) and zero-allocation records
are byte-identical. Its host binary stays 423,712 bytes / 267,100 .text bytes;
observe shrinks by 7 host bytes, 48 Thumb bytes and 34 Xtensa bytes. Local frames
stay 408 Thumb / 464 Xtensa bytes. Target assembly was assembled to measure real
code symbols; GNU nm could not supply symbols from the original LTO bitcode.

Eight CPU-0, ASLR-disabled ABBA blocks measure 960 samples / 5,898,240 completed
clock-probe operations. Medians 586.094 ns before / 584.945 ns after show no observed
median regression, not a speedup or MCU-cycle claim. Builds completed before the
timing campaign; subsequent WASM builds started later, as recorded timestamps
confirm. Full images/flash-fit, callee/interrupt stack and HIL remain unverified
for this fix and remain release work. No push, deployment or hardware safety claim.

All 79 earlier frozen records verify unchanged. Main's 41 staged and Interface's
eight pending paths remain protected by exact patch hashes. Only reviewed local
commits are brought forward; the full interval-plan goal remains active.

Evidence `docs/evidence/2026-11-15-interval-firmware-interval-inventory`, local archive commit
`c390ef8e50da1f274ce5b1217f83a3e0bf4bdcac`: 121 files / 4,544,975 bytes,
SHA-256 `0431237cb8e36eb53b0f86ed882f11a5a551f5a539ecd870cf25a5cdc3978cb6`.

## 2026-11-16: audit FOC ownership and authorize bounded size reconsideration

Phase 0/9 source review at firmware `2d68e82b7` replaces 27 lexical FOC
inventory dispositions: 15 production/support declarations, 11 tests/oracles/
layout gates, and the removed `ratio_bounds` helper. Seven omitted point and
component carriers are added. The [FOC audit](docs/research/firmware-foc-interval-audit.md)
distinguishes existing Q30/wide-Clarke/servo delegations from still-manual
min/max, expansion, ADC scalar-image and encoder quotient paths. Domain units,
angle/calibration/encoder evidence and controller saturation retain their owners.
The full-width position-error carrier remains widened; i64 substitution would
lose supported differences. The configuration wire ingress is explicitly recorded.

No production code or test changes. Post-edit mechanical validation checks
33 live source locators, one removed symbol, source hashes and byte preservation
of all 1,498 unrelated original rows. The 23 other original firmware lexical
rows remain open. The normal FOC feature tree is protocol plus fixed-only
Hyperinterval, without arbitrary precision. Existing 602 firmware tests/doctests,
including 86 FOC unit and two Q30 contract tests, strict Clippy and formatting
were executed in the prior clock stage at this same firmware revision. Source
and log checks tie those results to this audit; they are reused evidence, not
new runtime measurements. No new performance, allocation, final-image or HIL
claim follows from inventory review.

During this work the user authorized useful functionality to justify bounded
size growth with per-target caps, final-binary measurements, flash headroom and
cumulative accounting, then explicitly permitted reconsideration of previously
rejected refactors under that exception. Plan section 9.7 and AGENTS.md now
record the allowance. Exactness, behavior, provenance/certainty, dispatch/order,
performance, allocation, memory/stack and other validation gates remain intact.
No numerical cap is assigned and no growth is charged by this documentation
change. Historical candidate results are preserved; none is automatically retained.

Initial reconsideration priorities from the recorded evidence:

| Candidate | Prior result / remaining work under the new rule |
| --- | --- |
| Matrix scaling API (stage 77) | Size-only rejection: two old native files grew 8/24 bytes in string tables; owner/downstream correctness corpus passed. Reproduce on current revision, declare target caps, qualify final consumers and unchanged cost gates. |
| Taylor child-scheduling clone removal (stage 79) | Saves one 200-byte allocation per coefficient witness; 48-byte text growth in affected consumers. Current reproduction and full owner/successive latency gates remain necessary. |
| Global sensitivity prototype (stage 77) | Size rejection plus unfinished noncommuting/nonlinear/resource/replay/consumer corpus. Useful functionality is eligible, but the incomplete correctness corpus remains a blocker. |
| Fine-grid canonical regrid (stage 76) | Independent repeated owner logarithm latency regression. The size exception does not resolve it. |

Firmware flash allowance must use each actual packaged application's usable
partition and reserve. Existing firmware documentation reports a historical
4 MiB package overflow; it is not current headroom evidence and cannot establish
fit. Any affected firmware candidate still needs a current final-image comparison.
The audit itself changes no firmware image.

All 80 prior frozen stages verify unchanged. Primary 41 staged and Interface
eight pending paths remain protected by exact patch hashes. No push/publication
or phase completion; the full goal remains active. Next implementation review
will begin with the size-only matrix-scaling candidate under explicit caps.

Evidence `docs/evidence/2026-11-16-interval-foc-inventory`, local archive commit
`3c7205433a65b97d09e78478cdaddc338c6c9ff8`: 14 files / 836,893 bytes,
SHA-256 `4bc0348e1c7488f4ac3f6521f9354eebe2e27a436529ee0c3a11cbbe30e69912`.

## 2026-11-16: matrix scaling reconsidered under bounded size policy

The owner authorized managing all pending changes, including their own. The four
policy edits are committed coherently in `c9f1f9ac55216b762e9eb97672de1e66f3b8a3fb`;
the admission protocol owns the exception, with plan and agent-rule links. The
41 staged extraction and eight Interface paths remain unchanged while their own
implementation/qualification remains pending.

The [matrix reconsideration report](docs/research/matrix-scale-reconsideration.md)
records a fresh missing-API reproduction and current qualification of stage 77's
matrix-only candidate. Its existing-consumer growth fits the new bounded rule,
but a completed five-backend scalar-control comparison exposes an independent
latency gap (8.45–29.26%). A single-pass builder and one targeted public-wrapper
inline experiment preserve correctness but do not resolve that gap. All three
production candidates are removed; the proposed larger first-use client size
allowance is not adopted. No policy, fixture or performance gate is weakened.

The initial candidate passed 2,022 native tests/doctests in 133 suites, strict
Clippy/docs/fmt, MSRV and portable checks; a linked WASM client passed 243 cases
per binary against independent integer endpoint expectations. Native comparisons
include 128 ABBA runs / 72,640 samples / 10,731,520 completed operations, unchanged
output/allocation controls and Callgrind attribution. Missing Rust 1.95 Thumb
support was installed and the failed checks retried successfully. Raw failures,
proposals, controls and all unfavorable timing observations are retained.

Two independent first-error callback/drop-order tests remain on the original
fallible-array helper. Final owner tests/doctests (96), existing dispatch controls,
Rust 1.88 no-feature unit tests (2), strict Clippy and workspace formatting pass.
The retained test executable grows 24,848 file bytes / 12,208 text bytes / 11,915
named text bytes within its explicitly recorded 32,768/16,384/16,384-byte and 4%
caps. All 41 other measured native artifacts have identical code/data/unwind
sections, with raw-file increases no larger than 56 bytes; the 44,640-byte existing
WASM boundary remains byte-identical. No production function changes remain.

[The size account](docs/research/interval-size-accounts.json) records these test
coverage costs against a fixed goal-wide baseline, preserving both byte and
percentage remaining allowances per artifact. It does not accept the rejected
matrix client or grant firmware growth. Firmware and Interface normal graphs
exclude the owner crate. Full successive timing, fuzz/coverage, hosted CI and
release qualification of the rejected API remain unexecuted; no completion claim.

All 81 earlier frozen stages verify unchanged. No push/publication. The full
15-phase goal remains active; next reconsideration is the previously rejected
Taylor derivative child-scheduling clone removal, under fresh current checks
and the same cumulative size baseline.

Retained test/account commit `89d4fac48e5bba5ab538e4bf1b84c486383ff438`.
Evidence `docs/evidence/2026-11-16-interval-matrix-scale-reconsideration`, local archive commit
`3ecedc632cc6b4e92c14be86af9e539a13c6110e`: 607 files / 6,137,767 bytes,
SHA-256 `9ba4a6d94479521f114da38de1dc1c0015848b5640bee23807e08d650d5ca7cd`.


## 2026-11-16: Taylor scheduling reconsideration rejected on local stack

Phases 4/13, Hypersolve demanded-coefficient scheduling, P1/P4/P5/P6/P8-P10/
P18-P20/P36. Fresh review on `924bef144` reproduced the previously rejected
clone-removal candidate under the bounded size policy. The
[Taylor scheduling report](docs/research/taylor-scheduling-reconsideration.md)
records current measurements and the remaining qualification limits.

All 448 exact/resource/both-policy trace and receipt records and 36 wrapping
records match. One 200-byte allocation per coefficient construction is saved,
with peak/live heap unchanged. Four pinned ABBA blocks show roughly 1% faster
complete construction/evaluation/classification/serialization under both
policies. All 42 existing native artifacts and three standalone probes fit the
existing individual/cumulative caps against fixed baseline `c9f1f9ac`.

The candidate nevertheless increases the actual flow_noise derivative function's
local stack reservation from 312 to 328 bytes, with identical saved-register
pushes. Both coefficient probes independently reproduce the increase. This is
not a whole-program stack measurement, but fails the unchanged local stack gate.
The production patch is removed; no layout/inlining retry or new size budget is
accepted. The original cumulative account remains untouched.

A proposed in-repo trace assertion failed because its all-feature debug counts
were not the same as the standalone release fixture. That test and fixture are
also removed, with the full failure preserved. No existing assertion changed.
The candidate run stopped after 940 passing tests and this one new failure;
it is not reported as a complete passing run.

After the final restoration, 1,676 owner/downstream tests and doctests in
84 reported suites pass, as do strict owner/downstream all-target Clippy and
whole-workspace formatting. All 42 restored final native artifacts are
byte-identical to baseline. The candidate WASM and successive_operations builds
completed; paired execution and broader timings were not performed after the
independent rejection. No full CI, release, HIL or firmware flash-fit claim.

All 82 earlier frozen evidence stages verify unchanged. The 41 staged extraction
paths and 8 pending Interface paths remain byte-identical; user authority to
commit/discard them is recorded and remains in effect. No push or publication.
The full goal stays active. Next work should resolve the pending path-dynamics
extraction against its current owner timing gates, rather than retrying this
unchanged Taylor candidate.

Review commit `f7008cc03`. Evidence `docs/evidence/2026-11-16-interval-taylor-scheduling-reconsideration`, local archive commit
`4b31e7d22673bce20812bd2aa952f11074595a0f`: 261 files / 2,412,029 bytes,
SHA-256 `6f9f64f8e4ddc4ce74d3057b58ccacce2e4ec521d5bc2bb8f82e74bffafa4051`.


## 2026-11-16: retire unaccepted path-dynamics prototypes; retain canonical witness

Phase 11 review at e835617c freshly rebuilds the original baseline/extraction
owner binaries byte-identically to earlier reviewed artifacts. The
[current admission report](docs/research/path-dynamics-current-admission.md)
records the former size-only shared-error alternative under the owner's bounded
exception. Its 614 owner tests/doctests pass. New matched ABBA/BAAB timing confirms
pad-clearance, 256-record parser and exact-import regressions (about 1.4–2.8%).
Hodograph separation does not confirm in reverse order; all observations remain.
The proposed per-benchmark file-cap revision is not adopted. No inlining/layout
retry, compiler-policy change or unrelated algorithm adjustment follows.

Under explicit user authority to manage even user edits, the original 41 staged
CSGRS paths and eight pending Interface paths are archived as exact patches and
recoverable refs/interval-unaccepted/path-dynamics-20261116 snapshots. The
unaccepted production migration is removed from active worktrees. This corrects
the current phase table; it does not satisfy Phase 11 or weaken its contract.
Restored Hyperpath/Hyperdynamics pass 599 tests/doctests, strict all-feature/
all-target Clippy, and workspace formatting.

The independent Interface canonical digest assertion is retained at
`32a59de0d69436427e5b97297c4929fd908bf326`. Its original 41,795-byte transcript/digest supplements all existing
exact axis, bottleneck, replay and terminal-step assertions. All 201 core tests/
doctests and 199 configured-release tests pass, plus strict core Clippy and fmt.
The actual configured release test binary adds 432 bytes and complete .text adds
224 bytes. Both per-target byte/percentage caps pass against fixed c9f1f9ac /
Interface f5341f3 / firmware 2d68e82. A symbol-preserving diagnostic changes code
and is excluded; the stricter code allowance bounds ALL final .text instead.
The cumulative account gains this test artifact while preserving every one of
its 43 previous accounts and baseline. No production size exception is accepted.

All 83 prior frozen stages verify unchanged. No full owner timing portfolio,
whole release/CI, browser, HIL, firmware flash-fit, push or publication is claimed.
The full fifteen-phase goal remains active; the unresolved extraction must not
be silently restored or declared complete. Further work needs a distinct
supported implementation approach or another required plan capability.

Review/account commit `a3629dcac0ff81606d0e862154c95f0d082dbf20`. Evidence
`docs/evidence/2026-11-16-interval-path-dynamics-current-admission`, local archive commit `373fad71f6a875589c59db2fc3b79b24283be560`:
301 files / 22,253,496 bytes, SHA-256
`26a6964a2e716d408e16e73fec695e36afdcb85a464644f7799922a8cf1d9910`.


## 2026-11-16: affine-noise controller reconsideration rejected on owner runtime

Phases 12/13, P6/P8–P11/P17–P20/P31/P35–P39. The
[reconsideration report](docs/research/flow-noise-reconsideration.md) replays the
former size-only generic proof-provider candidate on da19d12ef. The automatic
controller passes its original witnesses and the preserved sixteen-step
continuous two-rate point/box contract. Existing manual dispatch/work/allocation
controls match, and all 42 existing native artifacts fit the fixed cumulative
caps. No size account changes or budget is consumed.

The full seven-executable owner portfolio nevertheless confirms ten existing
case regressions in ABBA and BAAB blocks (about 0.2–2.2%). Manual control gains
cannot offset those higher-priority losses. The complete eight-path candidate,
including the new continuous witness, is archived at recoverable unaccepted ref
refs/interval-unaccepted/flow-noise-20261116, commit 3dc973c69a0d146bb2b75ad13c6806cd90da279f,
and removed from the active worktree. No layout/inlining retry follows.

After restoration, 238 owner/Hyperphysics tests and doctests, strict all-feature/
all-target Clippy and workspace formatting pass. All 42 final native artifacts
and two manual probes are byte-identical to baseline. All 84 prior frozen stages
remain unchanged. Automatic retained-noise horizon evolution is still open;
this disposition does not complete phases 12/13 or the full goal. Further work
requires a distinct implementation approach or another required capability,
not another timing run of this unchanged candidate. No push or publication.


Review commit `b8aa7109b`. Evidence `docs/evidence/2026-11-16-interval-flow-noise-reconsideration`, local archive commit
`d14d110932ae8e548d4a3773020962e5b5a5ab6b`: 2,645 files / 9,032,213 bytes,
SHA-256 `abfa3ac4e8f36fa895cccb7f09d7c79b88a63dbdec02e4bdc7e08aeebbdb5680`.


## 2026-11-16: retained machine interval ownership fails current runtime gate

Phase 8, P1/P4/P6/P10/P25/P27/P31–P33/P36–P39. The
[reconsideration report](docs/research/profile-owned-reconsideration.md) replays
the earlier size-only retained-owner candidate on 3bea0ac90 / Interface 32a59de0.
Current native/WASM transcripts and allocation records match. Final native
file/text grow 448/352 bytes; WASM file/code grow 733/729 bytes. Those preliminary
metrics fit local caps, but no cumulative exception is accepted or consumed.

Warmed ABBA/BAAB confirmation finds all six successful complete profile cases
1.2–1.85% slower and the typed invalid case about 2.4% slower. Initial warmup
drift and every timing transcript remain recorded. No layout/inlining retry
follows. CSGRS and Interface snapshots are preserved outside active history at
refs/interval-unaccepted/profile-owned-20261116, commits
6b6b2ad259b83a10cc032bd33c26020edda36b81 and
c2c3b8b95f88a49bfaae992d1c2736652a403597 respectively. The production migration
and candidate-only tests are removed.

The candidate passes 200 Interface tests and owner/consumer Clippy, formatting,
minimal/isolated features, fixed-only thumb MSRV and documentation checks. Its
broad scalar/interval suite is intentionally stopped after the independent
runtime rejection: 70 completed result lines report 1,437 passes (including
nested subprocess fixtures), with four more tests completed before SIGTERM.
That command is incomplete, not reported as passing.

The accepted revision independently fails its GMP API coverage catalog on the
existing ratio_parts storage view. Retain its four-line classification repair;
keep API enumeration and missing/stale-classification assertions unchanged.
Remove candidate-only accessor classifications with their rejected types. No
production arithmetic or numeric benchmark coverage changes.

After restoration, interval checks report 247 passing test completions (with
nested subprocess fixtures), scalar library/catalog checks pass 1,053 tests,
and Interface passes 199 tests. Strict Clippy and formatting pass. All 41 owner
artifacts and both native/WASM profile executables are byte-identical to their
baselines; native transcripts and WASM replay match. All 85 prior frozen stages
verify unchanged. The independent catalog repair adds 64 file bytes and zero
code bytes to its optimized test executable; account 45 records that correction
under the existing caps, preserving all 44 previous accounts and fixed baseline.
Shared machine profile ownership and the full fifteen-phase goal remain open. No full CI,
release, complete stack, firmware flash-fit, push or publication claim.


Review commit `270eefdc6`. Evidence `docs/evidence/2026-11-16-interval-profile-owned-reconsideration`, local archive commit
`491d20ddfd2ffb256e2e6aa3ac037ed776406f20`: 337 files / 6,789,565 bytes,
SHA-256 `5cf531276f57325dd73ecf4e92f1008b750fb8858b6b64d17d734dcbbf38c67d`.


## 2026-11-16: exact radius composition preserves semantics but fails cost admission

Phase 8 section 8.3, P1/P4/P6/P8–P11/P19/P25–P27/P31–P33/P36–P39. The
[named-budget experiment](docs/research/budget-radius-composition.md) introduces
private-payload rational/Real radius proofs in an isolated candidate, preserving
native norms, validation order, named fields and the final strict decision.
The actual seven-case budget consumer preserves complete scalar dispatch,
serialized definitions, identities, allocation/live/peak records and native/WASM
replay. Owner checks report 253 passing completions including subprocess
fixtures; all 199 Interface tests pass, with owner strict Clippy and formatting.

Eight warmed ABBA/BAAB confirmation blocks show successful budgets modestly
faster, while immediate rejection adds about 4.6–4.9 ns per call. Nine existing
owner executables also exceed the predeclared +4,096-byte file cap, adding
4,144–4,488 bytes, dominated by +3,192-byte exception tables. No cap revision,
performance exception, strip-policy change or layout/inlining retry is adopted.
Full favorable/adverse data and the cache-sensitive legacy-versus-legacy trace
control remain in evidence. Complete trace assertions now use matched fresh
processes; no count assertion is loosened.

The candidate is preserved at refs/interval-unaccepted/budget-radius-20261116,
CSGRS 36316830cad23d4e56c52c9109fbb51a6950b755 and
Interface 2ce31a651e8ba18ff75302ed1714c9b51ba316e9. All production, feature,
proposed-test and documentation changes are removed. The new API is not retained
without its intended named-budget consumer. All 45 cumulative accounts and the
fixed program baseline remain unchanged. Restored interval checks report 247
passing completions (including subprocess fixtures), and all 199 Interface
tests, strict Clippy and formatting pass. All 41 owner executables, both budget
artifacts and the trace executable are byte-identical to baseline, with matching
transcripts and replay. All 86 earlier frozen stages verify unchanged.
This does not complete Phase 8 or the full
fifteen-phase goal. No push or publication.


Review commit `dc9e33712a3f24e1bdfe97f4205fc9992768641d`. Evidence archive commit
`c0e98b7bc4e9f810ed5226f8eca64aff55b14693`: 294 files / 3,439,487 bytes,
SHA-256 `e5c9452e59707f718315c92c25b4f4766b46d704a4bf2b23b57da1a192c1852f`.


## 2026-11-16: borrowed budget-radius proofs remain outside cost admission

Phase 8 section 8.3, P1/P4/P6/P8–P11/P19/P25–P27/P31–P33/P36–P39. The
[borrowed-radius experiment](docs/research/budget-borrowed-radius.md) keeps
rational input storage in the caller and transports a lifetime-bound proof.
It reproduces the missing API, preserves original validation and scalar order,
and passes original-storage/lifetime witnesses, full scalar traces, native/WASM
exact transcripts, serialization and allocation/live/peak records. Hyperinterval
reports 255 passing completions including nested subprocess fixtures; all 199
Interface tests and owner strict Clippy/formatting pass.

The candidate is rejected: nine owner binaries exceed the +4,096-byte file cap
with 4,128–4,488 bytes growth. Eight warmed ABBA/BAAB blocks retain faster
successful calculations and a repeatable 2.63–2.66% negative-source rejection
regression, 2.87–2.91 ns per call. No cap revision, performance exception or
layout/inlining retry is adopted. CSGRS snapshot
73e6669222e2d176045fd3126bd60f46bb3dd9d6 and Interface snapshot
40b731946a8f2743274dbe71c55e9a83e9659e9d remain recoverable through
refs/interval-unaccepted/budget-borrowed-radius-20261116.

All eight proposed paths are restored; no API is retained without its intended
consumer. Restored Hyperinterval reports 247 passing completions, Interface
passes 199 tests, and strict Clippy/formatting pass. All 44 measured artifacts
are byte-identical to baseline, with matching consumer transcripts and replay.
All 87 earlier frozen stages and all 45 cumulative size accounts remain
unchanged. This is not completion of Phase 8 or the full fifteen-phase goal.
No push or publication.


Review commit `2f463ef2830ab36a15585e66066105db6e759207`. Evidence archive commit
`c25f14ce87dfe51b40203f122e49e94e0770dbd8`: 269 files / 3,082,821 bytes,
SHA-256 `c26a51ee3419e79d1b7922a99b84c7de8c98e07b02d36e03c991f3cac7198a2d`.


## 2026-11-16: one-sided sqrt relocation fits initial size screen but regresses runtime

Phase 7, P1/P4/P6/P10/P17–P20/P35–P39. The
[current reconsideration](docs/research/sqrt-endpoint-reconsideration.md) revisits
the earlier size-only rejection under the bounded allowance. It relocates the
original one-sided rational/grid arithmetic unchanged, retaining solver
clipping, comparisons, fallback and evidence. A missing owner API is reproduced;
new tests verify domain order, exact squares and 3,315 independent tight-grid
cases. The owner suite reports 1,770 passing completions including subprocess
fixtures; 199 Interface tests, owner strict Clippy and formatting pass.

Exact endpoints, subsequent squaring, both-policy receipts, full scalar traces
and allocation/live/peak records match across twelve public cases. Native/WASM
Interface replay also matches. Initial screening finds no owner file-cap
exceedance; native sqrt probes shrink 24 bytes, Interface adds 128 file/32 text
bytes, and WASM file/code sizes remain unchanged. No complete cumulative
admission is claimed after the independent runtime rejection.

Two eight-block matched timing campaigns consistently regress fractional
rational and exact-square bounds, while large-coefficient cases improve. Full
favorable/adverse samples remain. An initial hash-check failure caused by an
objcopy inspection is preserved; no measured block had run. All four original
binaries are reproduced exactly before fresh timing. No threshold, cap, test,
policy or assertion is weakened, and no layout/inlining retry follows.

Unaccepted ref refs/interval-unaccepted/sqrt-endpoint-20261116 preserves CSGRS
4878c0fcddc1b005b667aa8c7d7dd4d46851942b and Interface
6a8f9eb07dd93f0bcc9c7defdde73e76538c4ecc. All seven candidate paths are
restored. The final owner suite reports 1,768 passing completions and Interface
passes 199 tests; strict Clippy/formatting pass. All 61 measured binaries are
byte-identical to baseline, with matching transcripts and replay. All 88 earlier
frozen stages and all 45 size accounts remain unchanged. Phase 7 kernel
migration and the full fifteen-phase goal remain open. No push or publication.


Review commit `e8a3d4e16595ac8b7e98c2bdc60be156ada2c309`. Evidence archive commit
`12d5c1d22714f9abca6dff94f848a80f66c35b1f`: 459 files / 10,966,221 bytes,
SHA-256 `c01b12f6f809ac391c3afea734ee16fd68ac98c2bf9dd4c6d4fe2886f11149dc`.


## 2026-11-16: FOC order-image delegation fails current runtime admission

Phase 9, P1/P3/P6/P19/P27/P31–P33/P35–P39. The
[order-image experiment](docs/research/foc-order-images.md) delegates two private
Q30 min/max helpers without changing modulation checks or caller order. Two new
independent witnesses pass before/after delegation; 4,108 complete result/error
rows match on native Rust 1.88/1.95 and WASM, with zero measured allocations in
twelve named cases. A separately strengthened test-only candidate passes 604
portable tests, 90 release FOC tests, strict Clippy and formatting.

The production candidate is rejected on two eight-block campaigns per compiler:
successful nonzero cases regress about 2% on Rust 1.88 and 0.6–0.9% on Rust 1.95;
several typed rejection paths regress more. All raw samples and noisy controls
are preserved. Positive host/WASM size growth does not excuse runtime cost;
final board-image/flash/stack/HIL qualification was not pursued after rejection.
The test-only proposal fits its declared file/code allowance but adds 824 bytes
(1.0871%) to `.data.rel.ro`, exceeding its unchanged 1% incremental cap. No
retroactive cap increase or layout/inlining retry is adopted.

Firmware refs preserve production snapshot `15dc920214c971554caccd7a64d1cbc05594f11a`
and tests-only snapshot `52d629964e3f5e1402bd0185bcd7df2b13a65df1`. Both paths are
restored at `2d68e82`. The restored portable suite passes 602 tests, release FOC
passes 88, and strict Clippy/formatting pass. All five measured artifacts are
byte-identical to baseline, with matching complete transcripts and replay.
All 89 earlier frozen stages and 45 size accounts remain unchanged. Two
inventory rows now record this current rejection; no migration is credited.
The full fifteen-phase goal remains active. No push or publication.


Review commit `59d1b3f6416b480b5a725c17b1403d111308808d`. Evidence archive commit
`55f99b524042bf8caaaf16e6b328dc5cc8406ea6`: 352 files / 21,273,567 bytes,
SHA-256 `40bcfaa2fe4394c74a78dc70295d5401cff7ebb4ab0cb7e3c9e6835dd59bb780`.


## 2026-11-16: current widening witness retained; delegation rejected

Phase 9, P1/P3/P6/P19/P27/P31–P33/P35–P39. The
[current-widening experiment](docs/research/current-widening.md) replaces manual
widening with the existing exact Minkowski-sum adapter while retaining negative
error and endpoint overflow behavior. All 4,108 complete sample/replay/Clarke
rows match on Rust 1.88 and 1.95, including full identities and explicit errors.
The MSRV consumer adds 4,184 file bytes, exceeding its unchanged 4,096-byte cap;
one eight-block host timing screen also shows adverse successful-case costs.
The production candidate is preserved at firmware snapshot
`518a1450d138d16761c642e2a42659abd95e71a5` and restored. No arbitrary layout,
inlining, cap or predicate-order change is attempted.

Firmware commit `dd2b8f987ae6440dbec696836f65f1cee55c5602` retains only an independent
252-case direct widening test. All 603 portable tests and 89 release FOC tests
pass, along with strict Clippy and formatting. Both native consumer executables
and the existing Q30 integration executable reproduce baseline bytes. The new
test executable adds 24,384 file bytes and 1,600 complete text bytes within its
predeclared test allowance; every growing data section meets unchanged caps.
Account 46 leaves 8,384 cumulative file bytes while preserving the prior 45
accounts and fixed program baseline. All 90 earlier frozen stages remain
unchanged. Phase 9 and the full fifteen-phase goal remain active; this witness
does not establish production migration or firmware/HIL qualification. No push.


Review commit `a64c0e58cdeaf3123d979ea243882a289c25fa2f`. Evidence archive commit
`2abaec72ada24ca1c61f4c382b63235e66b57739`: 147 files / 26,976,291 bytes,
SHA-256 `58c812f5e452dfa60e269f80e997c9685237038f6effee2bdb19d1859e5c8255`.


## 2026-11-16: Interface compiler/display repair retained

The [Interface admission](docs/research/interface-compiler-display-admission.md)
records commit `5b532c803af86dd9e3d7a66f3651163fd2f7c68b`: fixed-size byte
iteration, preserved compiler-compatible assertions and finite display
midpoints. The original overflow witness fails before the repair and passes
with exact-bit checks afterward. Stable/MSRV tests and strict native/WASM
Clippy, 109 release UI tests, final builds and exact canonical fixture replays
pass. All 2,496 completed-workload samples are retained; no adverse CPU effect
is statistically resolved, and matched peak RSS is unchanged. Source allocation
review and shipped native unwind-frame summaries support the memory review.

Final desktop/file growth is 64 bytes, delivered WASM 17 bytes and gzip 26
bytes. Every artifact/section meets the recorded caps, with explicit padding
budget revisions preserving the original failures. Hosted Brotli headroom is
688,115 bytes. Eight new accounts bring the total to 54; prior core-test growth
remains charged despite smaller new-layout measurements. No flash claim.

Evidence commit `efc1e26c633fa6e7004d583b4981b94876afde18` freezes the Interface
subrecord: 608 files / 857,548,059 bytes, SHA-256
`7f94dd7541f665d233bb8d41494e2a278d38fe3531c79dcebb5a5728c1583395`.
All 91 earlier frozen stages remain unchanged. Stage 92's remaining coordinated
checks, the certainty defect, and all fifteen phases remain active. No push.


## 2026-11-16: Horner requalification rejects persistent stack growth

The [current Horner admission](docs/research/horner-working-interval-admission.md)
records three unaccepted working-interval variants. Consuming translation passes
focused exact-output, dispatch, allocation and prospective size checks, but the
first cold tensor chain grows 80 stack bytes. A targeted inline experiment
removes its 64-byte adapter frame and still grows 16 bytes. Each persistent
increase appears in all eight processes of a 12,800-observation-per-variant
campaign. Outcomes match; no memory exception or cross-case offset is applied.
Both source worktrees are restored clean. No timing samples, production source
or size account is retained. Source snapshots, original failures and diagnostic
attribution are preserved.

Review commit `834d7d4785dd1e13d680878c74b22138dc12732f`; evidence commit
`4d0e306d02fbafdd0e9a52e4a8c8274b3ff7aa71`: 380 files / 195,329,447 bytes,
SHA-256 `54113bb2b11dc6d0488873a3f196390736d2a0cfcafd78a78a00d7755f7a559a`.
The Phase 7 migration and full goal remain open.

## 2026-11-16: strict Clippy coverage extended to workflow-derived configurations

The [matrix qualification](docs/research/strict-clippy-matrix-closure.md) records
46 additional successful strict-Clippy commands on clean `834d7d4785dd`:
ten workflow/script combinations, thirty release feature/target combinations,
and six generated consumer manifests. All 42 docs.rs compiler/package feature
configurations map to successful all-target lints. Fresh release, spatial README
and packaged FFI consumers preserve authored source and locked registry inputs;
no library-only or feature-superset result substitutes for the required scope.
No production source, policy, assertion or account changes are introduced.

Evidence commit `c15dd5c97d462dc65686ee9ff2014263dd50e427`: 2,904 files /
89,084,530 bytes, SHA-256
`8ae8dd43ec18ecd17cd625e0e3608046bf99ebb5c57ab73f470003e300102b14`.
The original default-root job on controlled CI revision `955593aa9e25` also
passes in 15,550.03 seconds under its unchanged 21,600-second watchdog; the
all-feature root job has started. Those root results remain in active Stage 92,
separate from this frozen lint subrecord. All 54 accounts and earlier frozen
records are preserved. Other CI, DRC CPU, implementation and embedded obligations
remain open; the fifteen-phase goal is active and no push is claimed.

## Current firmware final images — 2026-11-16

[Final-image qualification](docs/research/firmware-current-image-qualification.md)
measures current firmware `7e0d5a4` against a same-path diagnostic snapshot
restoring only the pre-fix clock source. All ten board release links pass;
three boards package on each revision. Final executable bytes/addresses,
section sizes and normalized local frames match. Diagnostic source locations
and ELF-derived package hashes differ and are exactly attributed, not discarded.

The unchanged 4 MiB default app partition rejects MKS FOC by 59,648 bytes and
TinyBee-4mb by 159,424 bytes on both revisions. Successful partitions are decoded
and their app payloads checked independently. All board flash-region declarations
remain empty; the explicit 64 KiB diagnostic reserve is not board qualification.
Current final-image measurements are now present; flash fit, declared partitions/
reserves, whole-call stack, MCU cycles, HIL and the full plan remain open.
No production source or any of the 54 cumulative size accounts changes. Frozen
evidence is registered in `docs/evidence-archive.tsv`; no push is claimed.

## Remaining firmware inventory entries — 2026-11-16

[The current source review](docs/research/firmware-remaining-interval-audit.md)
classifies the 23 firmware entries still provisional in the authoritative
inventory: 14 named tests and nine owner/host declarations. Earlier clock/FOC
reviews and all 1,509 other rows are preserved. All 70 currently inventoried
firmware entries have dispositions; exhaustive source coverage and migration
completion are not inferred from this finite list.

Independent recurrence stepping checks 21,952 sequences / 362,208 values against
the original machine-IR discrete-extrema helper on each of Rust 1.88 and 1.95.
The 23 existing owner tests also pass on each compiler; 30 xtask tests and strict
all-target owner/xtask Clippy pass on 1.88. The qualification patch is archived
and removed from the isolated checkout, not claimed as new CI regression coverage.
No production source, fixture, policy, size account or hardware claim changes.
Source locators, exact commands, scope and the preserved mechanical-check failure
are archived and registered in `docs/evidence-archive.tsv`. The full goal remains
active; no phase completion or push is claimed.

## 2026-11-16: coefficient ownership reconsidered and rejected on retained memory

The [current-revision reconsideration](docs/research/adaptive-stage-ownership-reconsideration.md)
reopens the historical size-only rejection under the authorized bounded exception.
The original missing sharing witness fails on `184faefa1`; combined candidate
`6af61b53b1b5ab53b591448599d17f0d3b7a546c` passes all 20 focused tests and both
observers match the complete 373,388-byte baseline output/work/dispatch artifact.

Fresh allocation measurement is decisive: ordinary order-three preparation
retains 16 extra bytes and makes three extra allocations in all 48 matched
observations. Restriction and peak-memory improvements are preserved in the
report, but do not waive that existing-workload gate. The full campaign contains
288 matched pairs across three fields and preparation/restriction compositions.
No runtime or new size acceptance is inferred. No assertion or cap is weakened.

The isolated checkout is restored clean; all 17 original order/controller tests
and formatting pass, and the rebuilt allocation probe matches the baseline hash.
The candidate and raw evidence remain available; no primary production source or
any of the 54 cumulative accounts changes. Frozen evidence commit
`cd6ef1cf2751ae30c6acc96fe1c22ca32d67b49d`: 125 files / 2,880,317 bytes,
SHA-256 `d6882e5ab9b8738c8eb7e1b8210e52878a8f03c9f61139a7fe1344d1d372bda3`.
Earlier archive records remain unchanged. Automatic noise-controller admission,
coordinated CI and the fifteen-phase goal remain open; no push is claimed.


## 2026-11-16: shared dyadic quotient reconsidered and rejected on stack

Owner Hyperinterval `scaled_quotient`, Phases 3/5 and affected consumers;
P1/P4/P6/P10/P18–P20/P35–P39. The prior size-only rejection was reproduced
on `17522b4eb` under the bounded exception. Candidate `3c3252832` shares the
unsigned division while preserving guards, scaling, work charges and public
predicate order. The three existing independent quotient tests pass before and
after the change. No production migration is retained.

Across two exact dependency locks, 1,656 fresh processes preserve 5,520 paired
serialized-output/work rows and 2,208 full traces. Allocation event/byte totals,
retained payload and peak do not increase; separate reallocation increases are
reported. External-lock maximum stack use nevertheless rises 16 bytes in twelve
completed cases. Its local helper reservation grows `0x118` to `0x128` with
unchanged register saves, whereas the pinned configuration shrinks by 16 bytes.
The [owner performance record](crates/hyperinterval/PERFORMANCE.md) explains why
this independent stack gate rejects retention despite favorable heap results.

Candidate qualification passes 247 owner completions on each full-feature
compiler, 73 dyadic-only completions on Rust 1.88 and 1.95, 96 lattice and 1,521
solver completions, strict Clippy, WASM library Clippy and formatting. All 21
fixed/current native artifact controls are byte-identical. Candidate file/code
initial caps pass; linker-padding section flags remain explicit. No complete
size exception, runtime improvement or firmware deployment claim is made.

The isolated checkout is restored clean to `17522b4eb`; all 21 restored final
artifacts reproduce baseline hashes. Restored owner tests, Clippy and formatting
pass. Snapshot `refs/interval-unaccepted/dyadic-quotient-current-20261116` retains
`3c32528322afa3278e944815786ebc8fbe10dc0b`. No timing was queued or run after
rejection; the existing 23-command quiet plan and all 54 size accounts remain
unchanged. No layout/inlining retry, push or publication occurred.

One missing helper row is added to the actual tracked inventory, preserving its
1,532 prior rows byte for byte. Its 1,533 current rows remain a partial inventory,
not exhaustive coverage; older ledger totals are not a substitute for inspecting
the current file. All fifteen phases and the full goal remain open.

Evidence `docs/evidence/2026-11-16-interval-dyadic-quotient-reconsideration`, commit `2de86748da8a00595cca3bbeffa3e399401ee2a9`:
1,865 files / 9,864,610 bytes, SHA-256
`1cd43363d05afc82015a5bd23b3bacac5e60767fa28fef41bb17397f493a245e`. Every previous archive path and manifest row is preserved.


## 2026-11-16: envelope result ownership rejected above its native code cap

Owner Hypervoxel `FieldEnvelopeFacts::from_envelopes`, Phase 13;
P6/P8/P9/P19/P35–P39. Public inverted bounds and malformed tensor dimensions
can currently produce certified aggregate facts. Validation candidate `644732fdf`
repairs those witnesses but adds stack even for empty input. Candidate
`200a3ecf9` constructs the result in place to remove that avoidable cost. Neither
candidate is admitted; original source `17522b4eb` and the fixed cumulative
baseline `c9f1f9ac5` remain the comparison anchors.

The initial 162-process screen reproduces the original stack increase. Another
594 fresh processes cover eleven cases, including failure after both vector and
tensor state hold symbolic clones. Compared with 644, all 528 allocation records,
528 full traces, outcomes and input-drop cleanup records match; all 264 paired
stack observations are nonincreasing. Empty and ordinary rational maxima fall
32 bytes below original. Added symbolic validation/cache costs remain explicit;
this does not prove a universal bound. A repository-pinned resource lock was
prepared but not run after the later size failure; only the external lock has
this resource evidence.

Final-source owner suites report 92 passes each on Rust 1.95 and stable;
Hypersdf reports 186 passing test reports, including nested output. Strict
owner/downstream Clippy, docs, WASM library Clippy and formatting pass. Expanded
native/WASM consumers check 225 independent rational hulls and subsequent use,
four malformed outcomes including dimension overflow, and zero-dimensional
composition. All fifteen final builds/functional runs pass, including restored
controls and 80 WASM calls. These are source consumers, not shipped packages.

The decisive native consumer grows 7,344 file bytes, 4,464 executable-code bytes
and 4,421 named-code bytes against both original and fixed baselines. Percentages
remain below 1%, but the shared 4,096-byte code cap fails by 368 and 325 bytes.
Benchmark growth is 1,360 file / 480 code bytes cumulatively; WASM grows
758 / 564. Savings elsewhere cannot offset a failed target. The earlier narrower
consumer measurement cannot replace this expanded completed-workload evidence.
No cap or account is changed, and no performance improvement is claimed.

The isolated checkout is restored clean; 12 original field/path tests, strict
Clippy and formatting pass. All three restored final binaries reproduce original
hashes. The unaccepted ref retains the patch; the old unstarted 644 owner timing
job is withdrawn with its previous plan preserved, leaving 22 queued commands.
The original root all-features CI command separately passed in 16,700 seconds
at `955593aa9`; adversarial validation continues. This is not full qualification
of the current revision. All fifteen phases remain open; no push occurred.

One missing reviewed method row is appended to the partial inventory, preserving
all 1,533 prior rows. Evidence `docs/evidence/2026-11-16-interval-voxel-envelope-state` at commit
`070189df831d97d142d3c369b20b625762c42d89` contains 904 files / 7,644,811 bytes,
SHA-256 `7f2065bd836e15fb9f624da28af619b78f55134caadfde9517d07761c38fb05e`. Earlier archive paths and rows are unchanged.


## 2026-11-16: interval rounding and resource boundary witnesses retained

Hyperinterval owns these missing conformance witnesses, under P1/P5/P6/P18–P20
and P35–P39, phases 3/5 and plan section 14. Qualified snapshot `6b54e0538`
adds three tests and extends one existing test; all previous assertions remain.
Production code, manifests, locks, APIs and predicate order are unchanged.

Twelve independently bracketed dyadic sources straddle sine/cosine output 1/2
at 64, 256 and 512 fractional bits. Directed MPFR/GMP expectations verify both
tight endpoints. One-unit-short work fails without publishing a range; fresh
exact-debit retry succeeds, and output enters subsequent squaring. Empty
trigonometric and rational-power inputs retain complete empty results through
composition, while a zero exponent denominator remains invalid. Compact powers
distinguish a declared bit-limit failure from signed-shift overflow without
materializing huge integers. Reciprocal completion consumes 154,105 declared
work units; the original 100,000-unit fixture remains an exhaustion witness.
Initial test-authoring failures are preserved, not credited as production fixes.

Full owner tests/doctests pass on Rust 1.95 and stable; the three affected suites
pass on Rust 1.88 with dyadic-only features. Strict Clippy passes all three
configurations; strict docs, formatting and final release test executables pass.
The same source selection yields 4,695/4,740 covered lines (99.05%), up from
4,688, and 9,333/9,891 covered regions, up from 9,320. All 494 grouped functions
execute. LLVM reports 27 code-bearing files from 28 selected paths; lib.rs has
no records. No denominator decreases. Branch coverage was not instrumented;
45 lines remain uncovered. The 26 external Rug zero-hash diagnostics persist
identically and diagnostic production tables reproduce. These are finite
configuration results, not universal completeness or full workflow validation.

Final test executable growth against both fixed and current controls is
9,696/1,696/12,976 file bytes, 6,656/1,408/9,600 executable-code bytes and
6,591/1,444/9,523 named-code bytes for trigonometry/tangent/fractional_power.
Each satisfies its predeclared 32 KiB file / 16 KiB code AND 2% caps. The
production trig_cost executable is byte-identical; no timing comparison ran
under unrelated routing load. Three host-test accounts bring the total to 57,
preserving all 54 prior accounts and fixed baseline `c9f1f9ac5`. This adds no
firmware or flash cost. All fifteen phases and broader validation remain open.

Evidence: `docs/evidence/2026-11-16-interval-coverage-boundaries`.
Archive commit `4844da1d38ddc5e38970d92416deb64871651aa4` contains 177 files /
9,554,477 bytes, SHA-256
`1698b22b10740be2121cbe597765b889665cb47250e181787d5ed710dc544764`. Prior archive records and manifest rows are unchanged.
Both original-source adversarial CI commands separately pass; the 22-command
quiet queue has not started under unrelated routing load. No push.


## 2026-11-17 UTC: Jacobian context bug reproduced; repairs rejected on stack

The required phase-7 body review reads all 136 original lines and six
Jacobian declarations, plus context/residual evaluation and the immediate
solver caller. These are solver-specific numerical proposal adapters, with
no generic interval kernel to extract. Six reviewed rows are appended to the
partial inventory, preserving all 1,534 previous rows; there are now 1,540.

At `557a349f9`, a rebound context gives forward difference `-5` where the
independent result is `7`; a weighted/fixed/unused/inactive case also fails.
Three repairs pass their focused correctness witnesses. Candidate `9e88db0ca`
also passes the full owner suite and strict local checks, but adds 32 stack
bytes in every measured default/trace group. Single-entry variant `89b58eb57`
reduces ordinary stack by 80 bytes but adds 128 for empty rows; owned-context
variant `4bd27bdb9` reduces ordinary stack by 128 but adds 176 for empty rows.
All three remain rejected. No size exception waives those stack results.

Four campaigns run 1,536 fresh processes. All 768 paired output matrices and
256 allocation/drop pairs match; the first candidate also preserves 64 full
trace snapshots. No timing ran. The isolated checkout is restored clean;
three original Jacobian tests, all-feature/all-target strict Clippy and
formatting pass. Both restored resource binaries are byte-identical to baseline.
No production code, new tests or size-account change is retained. The bug and
separate numerical-adapter review questions remain open.

See [the review](docs/research/solver-jacobian-context-review.md).
Evidence `docs/evidence/2026-11-17-interval-jacobian-context`, commit `a099a4fd21fd8d106acfb874bb943dedc628794d`,
contains 1,792 files / 1,280,121 bytes, SHA-256
`33911caecf0a7cd0d30a440b434f9bbff6331f88c514a7a711b71a4dbaeb42ce`. Earlier archive paths and rows are unchanged.
All 57 size accounts are unchanged. All fifteen goal phases remain active.
The separate alpha repair is unaccepted pending its queued comparisons;
24 quiet commands remain scheduled, with no Jacobian timing added. No push.

## Required dense linear adapter ownership review

At `15cf09783`, all445lines and25declarations in the required
`hypersolve/src/linalg.rs` file were reviewed, including its manifest, public
exports, report storage and immediate solver consumers. These are category4
lossy numerical proposal routines and category5 diagnostics, with no interval
carrier or enclosure kernel to extract. The original1540inventoryrows are
preserved and25reviewedrows appended, for1565partialinventoryrows.

Nine unchanged-source standalone controls pass under Rust1.95 with warnings
denied, and strict Clippy passes. Three missing acceptance witnesses explicitly
fail: normal equations and BFGS can return successful NaN steps from finite
overflow inputs; normal equations also accept a NaN input. Exact commands,
independent small-system expectations and failure diagnostics are preserved.
These results establish current gaps, not corrected support. The full owner
suite and cost/platform gates were not rerun for this source-unchanged audit.
See [the ownership review](docs/research/solver-linear-adapter-review.md).

No production or repository test changes are retained;57sizeaccounts remain
unchanged. A temporary build dependency held the quiet lane during the
standalone builds and was released afterward. All24queued commands are
unchanged. Other pending candidate admissions and all15phases remain active.

Evidence: `docs/evidence/2026-11-17-interval-linear-adapter-review`, local commit `8e85280f4f69874b21dc38bee78dbf879c7f6541`;
22files / 853,986bytes, SHA-256
`03745adae57d4258bd9ac21cc8fee79a63b8c9a72111b24b7f300f9c783c3210`. This stage is frozen. Every previous archive path and
manifest row is preserved. No push.

## Non-finite linear proposal repair rejected on stack

Baseline `a32805da2`; isolated candidate `0caf282ad` adds distinct invalid-input
and non-finite-arithmetic errors, checks overflowed norms before clipping, and
carries those reasons through the nonlinear solver. Three original failures
reproduce before editing; twelve standalone cases and four new owner tests pass
after repair. Strict all-feature/all-target owner Clippy and workspace fmt pass.
The initial fixture constructor error and its correction remain in the logs.

An external caller of the full Hypersolve crate preserves 384 paired outputs and
128 allocation/drop pairs over 768 fresh processes. The 32 stack groups include both
warm states, two fill patterns and four repetitions. Ordinary dogleg grows 16 bytes,
BFGS 48 bytes, and a zero-width normal system 24 bytes; other normal/singular groups fall 16 bytes.
Empty dogleg/BFGS paths cross the probe exclusion window, whose zero readings
are censored rather than actual zero stack. The uncensored increases alone
reject retention. No timing or size allowance was adopted. See
[the review](docs/research/solver-linear-adapter-review.md).

Original source is restored clean; the rebuilt external binary is identical to
baseline. Six original solver controls, strict Clippy and fmt pass. The build
barrier is released. Candidate `0caf282ad` remains on an unaccepted ref; no production,
test or migration change is retained. 1,565 inventory rows and 57 accounts remain
unchanged. Full owner tests, stable/WASM qualification, runtime and size gates
were not continued after resource rejection. All 15 phases remain active.

Frozen evidence: `docs/evidence/2026-11-17-interval-linear-finite-rejection`, local commit `d84e75a0e2c50a41daf74a5e982e9cf86e15c417`;
998files / 894,283bytes, SHA-256
`73555446eddb99a781d02abf3005dbd6c09c5a02a5c3657ba277041f22caef03`. Every prior archive path and manifest row is preserved.
No push.

## User-authorized bounded memory growth

The user explicitly permits small bounded memory growth around new functionality
and reconsideration of approaches previously rejected on that criterion. The
admission protocol now requires separate temporary/retained heap and stack caps,
complete-lifetime measurements, deployment headroom and cumulative accounting.
Exactness, certainty, provenance, predicate order, runtime, hard resource limits
and no-alloc firmware remain unchanged. This records the requested policy
migration; it does not weaken a fixture or admit a candidate.

The new `docs/research/interval-memory-accounts.json` uses the existing fixed
program baseline `c9f1f9ac5`. It initially has no accepted accounts or blanket
allowance. Earlier positive growth must be established and retained before a
new account is admitted; missing history cannot be charged as zero. All 57
binary-size accounts remain unchanged.

The first fresh reconsideration is the non-finite linear repair `0caf282ad`.
Its earlier stack rejection remains immutable. A new stage must resolve
censored empty-path measurements, declare justified target caps and complete
runtime, size and downstream qualification. The Jacobian context repair is also
eligible for review of its measured 32-byte increase; no prior runtime failure
or other gate is waived. Unchanged rejected designs with unrelated failures
remain rejected on those grounds.

## Domain-certification certainty review

Current revision `425c28eeb` reproduces a separate proof-qualification defect:
the domain preflight adapter discards predicate certainty and counts approximate
answers as exact domain certificates. All 532 lines and 15 declarations were
reviewed. Two lexical inventory rows are updated and thirteen missing
declarations added; all other rows remain unchanged. The inventory now has
1,578 rows and remains partial. Scalar unit-domain membership belongs in the
Hypersolve adapter, not in a new interval arithmetic kernel.

Twelve exact boundary/offset controls pass under both policies. Twelve public
acceptance witnesses fail across all ten domain-check kinds, including a
positive denominator reported `CertifiedInvalid` and a negative square-root
operand reported `CertifiedValid`. Hyperlimit explicitly marks the contributing
terminal equality as approximate; strict policy remains unknown. The independent
angle-addition identity and exact signed dyadic offsets establish the intended
domains. The initial square-root witness and harness constructor compile failure
are preserved separately. Both final harnesses pass strict Clippy and formatting.

No production repair, new repository test or resource allowance is adopted.
See [the review](docs/research/solver-domain-certainty-review.md). The frozen
review record is `docs/evidence/2026-11-17-interval-domain-certainty-review`;
its digest is registered in the evidence manifest. Runtime, full-owner,
resource and downstream qualification are not claimed by this audit.

The unaccepted alpha, Jacobian and non-finite linear candidates remain separate.
Their guarded runtime queue is unchanged; their proposed resource budgets do
not become independent cumulative allowances for shared final artifacts.
All 57 size accounts and zero adopted memory accounts remain unchanged.
All fifteen phases remain active. No push.

## Selected polynomial enclosure inventory review

The full 182-line bounds module and its immediate policy/work/consumer paths
were reviewed at `ee4932b4c`. Nine declarations are now mapped: two lexical rows
updated and seven missing rows added, preserving all other 1,576 rows. The
inventory has 1,585 rows and remains partial. The selected value and its shared
field authority make this a Hypersolve proof adapter/certificate, not a generic
two-endpoint interval to extract. See
[the ownership review](docs/research/selected-enclosure-ownership-review.md).

Existing selected-bound, original second-chamfer and selected-replay receipts
from composed candidate `f65589b44` are retained with exact source hashes;
they are not presented as fresh primary-revision tests. Static coverage and
preservation checks apply to this documentation/inventory change. No production
source, predicate order, runtime gate or resource account changes. The earlier
solver candidates remain unaccepted pending their quiet-host timing campaigns.
All fifteen phases remain active; no phase is declared complete.

Frozen ownership evidence: `docs/evidence/2026-11-17-interval-selected-enclosure-review`, local commit
`3382575d943f9d7801fe56d185827120576a2420`; 15 files / 1,672,600 bytes, SHA-256
`b05e1ff27456794dcbcb0432cbd77401e6cb7fbb45a8aab53ad30f413adb88c7`. Prior archive paths and manifest rows are preserved.
No push.

## Selected polynomial sign-filter inventory review

The complete 220-line filter module and its immediate sign/context/interval
paths were reviewed at `478795e48`. Eleven declarations are mapped: three
lexical rows updated and eight missing rows added, preserving all other 1,582
rows. The inventory has 1,593 rows and remains partial. The review retains the
selected-field authority, denominator guards, optional-filter fallback order
and distinct coefficient-evaluation paths; it does not propose extracting a
second generic interval kernel. See
[the ownership review](docs/research/selected-filter-ownership-review.md).

An existing frozen full-owner receipt contains 14 passing selected-field tests
at `f65589b44`, with reviewed source identities checked. This is reused evidence,
not a fresh test/benchmark run. Static inventory/source/documentation checks
apply to this change. Source-only simplification does not qualify for the bounded
memory exception. All source/resource proposals remain subject to their original
runtime and other gates; all fifteen phases remain active.

Frozen review evidence: `docs/evidence/2026-11-17-interval-selected-filter-review`, local commit
`b62e9eb5ae819cd788a7b379f16ad52147bd950c`; 12 files / 1,539,709 bytes, SHA-256
`071764eecb4a080fbdaafd15db2d365f14c8066fce643f711f39d04aab877b01`. Prior archive records and manifest rows are preserved.
No push.

## Successive-operation analysis and contended DRC replays

At primary `5d74e2c1d`, the pending combined solver qualification gained a
post-run analyzer for both original successive-operation histories. It checks
all eight attributed processes, 21 chains and 200 steps, complete certified
journals, unchanged geometric rows, artifact/driver/journal hashes and host
boundary records. It retains per-step and per-chain comparisons, treats absent
validation separately from zero, and never sums memory peaks across steps.
Twenty-six synthetic controls passed, including incomplete/corrupt evidence,
wrong attribution, external contention, zero baselines and an exact adverse
runtime example. These are analyzer controls, not measured candidate performance
or geometric qualification. The allocation columns cover step calls only and
cannot establish a complete-lifetime memory exception.

The external observer found 60 seconds without the watched workloads and the
38-command queue began. Routing workers restarted about nine seconds after
launch. Both predeclared DRC replays failed the unchanged 120-second worker-CPU
assertion: 189.525596316 and 188.118210819 seconds. The first also overlapped the
26.95-second synthetic-control campaign. Preserve both failures; neither run
establishes exclusive-host performance, and no causal attribution or runtime
waiver follows. The orchestrator is held before command three. A separate,
six-hour observer may resume the remaining queue after a fresh quiet window;
it never retries a command or resets its cumulative workload budget.

Only this progress record and the archive index change in the primary tree.
Measured baseline `ee4932b4c`, candidate `f65589b44`, all 57 accepted size
accounts, zero accepted memory accounts and the partial 1,593-row inventory
remain unchanged. All fifteen phases remain active. No production adoption or
push follows from this record.

Frozen finite evidence: `docs/evidence/2026-11-17-interval-successive-analysis-review`,
local commit `56b05cf2010fa75d6ff3ce00ffd52503a75a2512`; 24 files / 1,336,955 bytes,
SHA-256 `d084ca916a048e8bd10e79bd7930ee1c5b41c7d38620fb91074b08d082f79649`.
The live combined stage and queue remain mutable and are not sealed by this
snapshot. Both archive-preparation failures are retained; prior archive records
and manifest rows are unchanged. Final whitespace, spelling, source/account
preservation and repository-budget checks qualify this documentation change.

## Residual certification certainty review and isolated repair

The full 446-line certification module is now mapped at `4192d8c726`: twenty
declarations, two lexical rows updated and eighteen added, preserving the other
1,591 inventory rows. The resulting 1,611-row inventory remains partial. This
solver report/adapter stays in Hypersolve; it is not generic interval arithmetic.

Eight public witnesses reproduce approximate ball receipts being counted as
exact satisfaction, including nonzero equality residuals. The affine interval-box
facade promotes those counts to `Certified`. Four new regression tests fail on
the unchanged baseline and pass on isolated candidate `aad7409a59`, which keeps
qualified receipts, excludes them from exact summary claims and transports
their uncertainty into diagnostics. Exact controls and nonempty predicate
traces remain identical. See
[the review](docs/research/residual-certification-ownership-review.md).

The candidate passed 1,524 all-feature owner tests, 839 no-default owner tests,
strict Clippy, formatting, 444 Hyperpath tests and 155 Hyperdynamics tests. It
remains unaccepted pending runtime, resource, packaging and composition checks.
No primary production source, existing test, runtime gate or resource account
changes. All 57 accepted size accounts and zero memory accounts remain intact.
The earlier four solver repairs and all fifteen plan phases remain open.

The existing quiet queue was held during these bounded builds. Its same finite
host watcher was resumed afterwards without changing commands, cumulative
budgets or the recorded DRC failures. A full system disk interrupted Cargo cache
bookkeeping; a verified relocation of downloaded archives restored writable
space. Initial warnings and relocation failures are retained, with every archive
available through the original cache path. No push.

Frozen review evidence: `docs/evidence/2026-11-17-interval-residual-certainty-review`, local commit
`ae36f8d84f038d5789b690e41b0615c5929b7ca5`; 138 files / 2,686,921 bytes, SHA-256
`a5ce2987cfdc4ae978cbe44cc1cbbc3c85b61e8105819a93f80dfd8a9ab8ef20`. Prior archive records and manifest rows are preserved.
No push.

## Residual-certainty resource qualification

The isolated residual-certainty repair has completed 105,144 resource processes,
including 519,936 full-owned operation/cleanup cycles. The 113-case corpus covers
196 policy configurations across direct reports, interval-box consumers and
failed-constraint diagnostics. Original and repeated reports/traces agree on
unchanged completed contracts; corrected approximate outcomes remain separate.

Initial operation and lifetime campaigns exceed prospective caps in 113 and
68 accounts, respectively. Fresh repetitions reproduce within-binary allocation
and stack variation; all original positive charges and baseline drift remain
reserved. No persistent payload growth occurs after cycle two in the measured
lifetime campaigns. Instrumented final binaries pass their size caps. See the
[resource qualification](docs/research/residual-certification-ownership-review.md#finite-memory-qualification).

Only two stack configurations have a new unadopted proposal: 128 bytes AND 2%,
with cumulative charges of 80 and 104 bytes and unchanged total/headroom limits.
Temporary allocation failures remain open. Runtime, complete consumer capacity,
public/WASM artifacts, packaging, cancellation/retry and combination with the
four earlier repairs remain required. No primary production source, inventory
row, accepted account or existing gate changes. The 1,611-row inventory remains
partial, all 57 size accounts and zero memory accounts remain intact, and all
fifteen plan phases remain active. No push.

Frozen finite resource evidence: `docs/evidence/2026-11-17-interval-residual-certainty-resources`, local commit
`d62cded53e3df97f5c77ada6a040df93cda848c0`; 13 archive files / 134,998,864 bytes, SHA-256
`5e6547a66f1263debda2a5f5a794ccaef9aec58034c1de61b570926a80de2dc6`. The verified split archive contains 109,301 files /
2,875,171,645 uncompressed bytes, including all fourteen instrumented binaries.
The live admission stage remains mutable; later work is not sealed by this
snapshot. Prior archive records and manifest rows are unchanged. No push.

## Failed-constraint diagnostic ownership

The complete 826-line failed-constraint module now has 33 reviewed declaration
rows, preserving the previous 1,611 inventory rows. The 1,644-row inventory remains
partial. Diagnostic classification, rank redundancy and bounded removal searches
stay in Hypersolve; they are report/proof consumers, not generic interval
arithmetic. Source-row identity, uncertainty and current-candidate versus global
satisfiability claims remain distinct. See the
[ownership review](docs/research/failed-constraint-ownership-review.md).

Removal cardinality and precision do not establish total-work, memory or stack
bounds. Existing owner receipts and mapped tests remain evidence for their
original scope; no new removal-search runtime result is claimed. The separate
native/WASM residual consumer campaign completed fourteen final artifacts and
16,096 checks, including 13,744 full report/trace comparisons. Strict Clippy and
formatting pass. Original image caps pass: native accounts have no positive
charge and both WASM configurations retain 310-byte cumulative file/code charges.
Memory failures and the remaining owner/package/runtime/composition gates stay
open. No production or accepted resource account changes; the broader plan
remains active.

Frozen evidence: `docs/evidence/2026-11-17-interval-failed-constraint-ownership`, local commit
`8bc1f0256c0d8fe919cd7c0fd182ac94341145ac`; 139 files / 55,660,343 bytes, SHA-256
`ef3852c2547fec0b9019c3f552951d36f88d9819ff6b8244ee49eea74e7b8568`. All prior archive paths and manifest rows are
preserved. The ownership review is frozen; the separate live residual admission
stage remains mutable beyond this finite snapshot. No push.

## Residual-certainty packaged consumers

The isolated `aad7409a59` repair now passes the finite six-crate package boundary:
eighteen normally verified upload archives, twelve native/WASM final images and
12,960 full controls matching their workspace counterparts. Metadata rejects
workspace or stale-source fallback. Strict Clippy on both features/targets and
formatting pass. Original prospective image caps pass with every inherited and
new positive charge retained. See the
[packaged-consumer review](docs/research/residual-certainty-packaged-consumers.md).

The original optional-patch warning and interrupted attempt remain preserved;
the corrected harness keeps the same fixtures and total elapsed budget. Package
exclusion notices remain explicit. This is no quiet-runtime or native-memory
admission and does not qualify the complete stack release. Owner artifacts,
complete consumer capacity, cancellation/retry, runtime and combined-repair
qualification remain open. No production, inventory or accepted account changes.
The inventory remains partial at 1,644 rows and all fifteen plan phases remain
active.

Frozen evidence: `docs/evidence/2026-11-17-interval-residual-certainty-packaged`, local commit
`db4865f495af7e3c0ee640cea890e6edca344d5a`; 253 files / 56,808,003 bytes, SHA-256
`295c4976b18427b073f9990d010e4510f7b333d4f584c746b6106ae6bbfd2407`. All earlier archive paths and index rows remain unchanged.
Both package attempts and the review snapshot are frozen. No push.

## Residual-certainty owner images and trace allocation

The isolated repair now has 35 measured owning-crate release images and 5,264
passing functional checks across fixed/current/candidate revisions, no-default
and all-feature configurations. Strict Clippy and formatting pass; full dispatch
reports and existing test identities/outcomes are preserved. Six retained-fuzz
images are explicitly metadata-only, not executed fuzz workloads. See the
[owner and allocation review](docs/research/residual-certainty-owner-and-trace-review.md).

The no-default unit file has one original cap failure: 15,456 bytes of positive
growth exceed its 4 KiB incremental cap. Other metrics and declared reserves
pass. A separate, unadopted proposal would allow this correctness change to use
the existing 16 KiB cumulative file budget in one increment, preserving the
charge and 928 bytes of remaining allowance. No accepted account changes.

A 108-process diagnostic attributes the sampled active-trace 632-byte variation
to snapshot B-tree allocation in the original binaries. Full output/trace
strings agree; passive controls remain constant. This finite attribution does
not remove any historical charge or explain passive stack variation. Memory,
complete-consumer, runtime, cancellation/retry and combined-admission gates
remain open. No production or inventory change; all fifteen phases remain
active and the 1,644-row inventory remains partial.

Frozen evidence: `docs/evidence/2026-11-17-interval-residual-certainty-owner`, local commit
`1da0eebf99e52060d5f08dd1f7c80cf8e28def6d`; 548 files / 149,749,754 bytes, SHA-256
`984f4104fc268c76d6c94dcbc0e74b98741f43fbc8b6c436614807a21b3f6cef`. Earlier records and index rows are preserved.
The raw-log archive rejection and metadata collector error remain recorded
with their corrected verification; all source/output bytes are preserved.
Source stages are frozen. No push.

## Residual-certainty allocator usable-byte attribution

A finite 146-process review now reconciles one passive memory configuration's
usable-byte variation against the original fixed/current/candidate binaries.
All complete outputs and traces agree. The 73 instrumented processes have
identical request sequences and reconcile 219 operation and cleanup epochs;
73 corresponding event positions receive differing usable sizes despite
identical requests. Plain processes reproduce usable variation on each binary,
while requested traffic, peak and retained operation payload remain constant.
See the [usable-allocation review](docs/research/residual-certainty-usable-allocation-review.md).

This is attribution for one STRICT cold rebuilt-input case, not a general
allocator model, unchanged instrumented cost, or runtime evidence. Original
failures and positive charges remain; no cap or accepted account changes.
The isolated repair's remaining memory, capacity, runtime, cancellation/retry,
validation and combined-composition gates stay open. Production and the partial
1,644-row inventory are unchanged. The original quiet watcher resumes with its
unchanged finite horizon. All fifteen plan phases remain active.

Frozen evidence: `docs/evidence/2026-11-17-interval-residual-usable-allocation`,
local commit `d2842b8b326d916d541a670ada9a604ab9453e9a`; 171 files /
2,662,214 bytes, SHA-256
`fae97e7a8a16aebbd67440cf3f4a5ab665fcffbe42a845df79f431ec648bcb9e`.
Original bytes are preserved with source/stored digests; earlier archive paths
and index rows remain unchanged. The diagnostic stage is frozen. No push.

## Five-repair solver composition

Isolated candidate `4a04dd9744bfecb578963b42fa3bffef5b7062b5` composes the four pending solver repairs
with the unchanged residual-certainty repair. Four original residual regressions,
eight public witnesses and the additive composed-chain witness first reproduce
their failures on `f65589b44e1a010de75f8129b371ecbf503cc052`. The candidate
passes all affected owner/public/downstream checks, including both owner feature
configurations, strict Clippy, doctests and strict docs. Full owner dispatch
reports, all 32 residual policy traces and eight exact-control outputs match
the fresh baseline. See the [composition review](docs/research/solver-five-repair-composition.md).

The new witness consumes alpha/domain-qualified context through residual-ball
certification and diagnostics, including a return to STRICT with warmed caches.
All existing assertions remain. The original formatting rejection is preserved;
its correction changes only line wrapping and keeps the total deadline.

Source and accounts remain unaccepted. Final five-repair resource, capacity,
runtime, successive-operation, distribution and applicable validation evidence
remain necessary; earlier costs and failures are not reset. Original queued
artifacts/revisions and the finite watcher are preserved. The partial inventory
stays at 1,644 rows and the full fifteen-phase plan remains active.

Frozen evidence: `docs/evidence/2026-11-17-interval-five-repair-composition`, local commit
`db6870e1c3109c4e540d084e3f34039dc412158f`; 122 files / 7,822,613 bytes, SHA-256
`ca2012ee7e29ee64f569a2bf38e82cf71e917ca45c07dad2d5daef74329598e6`. Original observer executables, command outputs and patches
are preserved with lossless gzip encoding and source/stored digests. Earlier
archive paths and manifest rows remain unchanged. The qualification and initial
formatting-attempt stages are frozen. No push.


## 2026-11-17: five-repair complete-consumer resource controls

The unaccepted five-repair source passes 516 controls and reproduces eight
expected failures on the four-repair baseline. The additional consumer retains
all original inputs/assertions and consumes residual-ball reports in diagnostics.
Complete output/trace text agrees within 132 feature configurations; all four
strict Clippy configurations pass. See the [resource-control review](docs/research/solver-five-repair-resource-controls.md).

All 38 images are preserved; initial final-image budgets pass. Whole-owned
controls complete 8,256 cycles with no tracked persistent growth after cycle two.
All initial lifetime total caps pass, but ten growth metrics in four active-trace
configurations fail. Their charges and original fixed denominators remain;
no cap is revised and no account or source is admitted. The initial collector
failure and corrected feature separation preserve every output/assertion and
the original total deadline.

Repeated-process/stack measurements, reused-input whole-owned lifetime,
quiet runtime, successive-operation, distribution and resource/cancellation
qualification remain open. Payload observations do not establish RSS or
firmware fit. The 35 old residual-owner images now have verified RAM copies at
stable symlink paths, with immutable archived bytes and a recovery recipe.
No queued artifact changed. Inventory remains 1,644 rows; accepted accounts
remain 57 size and zero memory. Earlier failures and the fifteen-phase goal
remain active.

Frozen evidence: `docs/evidence/2026-11-17-interval-five-repair-resource-controls`, local commit
`b2dde626edd2ca1a4b4e5030192c3108d345b32a`; 843 files / 73,745,748 bytes, SHA-256
`c4339cb5da9ec04d23673a74dc20d2fdae210ebcdd59b3408af76f4c88b571d9`. Raw logs, collector attempts, recovery records and all
38 final executable images are preserved losslessly with source/stored digests.
Previous archive paths and index rows are unchanged.


## 2026-11-17: repeated five-repair resource and stack qualification

All 12,600 planned independent processes complete. Full output and trace text
agrees across 10,248 matched source comparisons, with 211 original cold controls
retained as supplementary cost observations. The 18 final images are unchanged.
All total payload, stack-growth and headroom gates pass. Of 168 accounts, 22
fail 52 heap/count growth metrics; every initial cap and positive charge remains.
The earlier ten whole-owned lifetime failures remain separate obligations.

The initial analysis timeout and single-pass parser correction are preserved;
no geometry is rerun or assertion weakened. Independent verification checks
all raw observations, source charges, full strings and capacity calculations.
See the [complete consumer review](docs/research/solver-five-repair-resource-controls.md).

Reused-input whole-owned lifetime, quiet runtime, successive-operation,
distribution, cancellation/resource contracts and complete validation remain
open. No implementation or account is admitted. Inventory remains 1,644 rows;
accepted accounts remain 57 size and zero memory. The frozen queue, finite
watcher horizon and full fifteen-phase goal remain intact.

Frozen repeated-measurement evidence: `docs/evidence/2026-11-17-interval-five-repair-resource-measurements`, local commit
`dbf779db93c6765fcdf1c349ab1f9330be864c3e`; 31 files / 33,237,852 bytes, SHA-256
`8a2c2ed024609657d66fc211aa8444739477f40de9b9830e27eb8b63fa103063`. The archive inventories all 12,600 log members and references
18 unchanged executable images in the preceding immutable record. Earlier
archive paths and index rows remain unchanged. No push.


## 2026-11-17: reused-input complete solver lifetimes

The additional observer owns its input from construction through 32 operations
and final destruction. All 726 successful histories complete 23,232 cycles;
the four-repair source reproduces four expected qualified-receipt failures.
Every original fixture assertion and allocator byte is preserved. Full histories
agree across 660 additional comparisons, and every first-cycle output matches
its archived cold control. Four strict Clippy configurations and formatting pass.

No successful history has negative reconstructed payload or net retention change
after cycle two. All total payload and image budgets pass. Final persistent
usable payload reaches 25,064 bytes under the unchanged 32 KiB limit. Thirteen
growth metrics fail across five accounts; original caps, charges and earlier
failures remain. See the [reused-input review](docs/research/solver-five-repair-reused-lifetime.md).

Twelve positive charges in earlier failed resource metrics occur between
byte-identical images. Their original rows and image bytes are independently
verified; this does not explain other boundaries or remove any charge.

The original six-hour quiet watcher exits at its fixed horizon, leaving 36
commands pending and the workflow parent held. Its terminal observation records
one external routing process; no watcher, horizon or command budget is reset.
Fresh evidence/review is required before queue resumption. All candidate
revisions and queued artifacts remain intact.

Source admission, remaining growth failures, quiet runtime, successive-operation,
distribution, resource/cancellation and full applicable validation remain open.
Inventory is still 1,644 rows; accepted accounts remain 57 size and zero memory.
The full fifteen-phase goal remains active. No push.

Frozen evidence: `docs/evidence/2026-11-17-interval-five-repair-reused-lifetime`, local commit
`01de43682923e9910cfe5607bba3d16ffaf4fc22`; 54 files / 125,109,846 bytes, SHA-256
`a41d45a26fdc8fbd0d155a0a8b715d80de7a0e3b0660531c03f7db05a199c458`. All 19 images and every compressed command/history log are
preserved losslessly with byte counts, hashes, modes and a container-member
inventory. Earlier archive paths and index rows remain unchanged.

## 2026-11-17: fresh profile of the queued DRC failure

The exact queued image at `955593aa` still fails the unchanged CPU gate. A
bounded diagnostic collects 66 worker stacks, preserves the failure and verifies
source, image, fixture and queue identity afterward. Debugger timing does not
qualify a replay or establish a speedup. The old borrowed-promotion candidate
remains rejected on runtime; the bounded-memory exception does not waive that
gate. See the [diagnostic review](docs/research/queued-drc-profile.md).

The workflow parent remains held, its original watcher terminal and 36 commands
pending. No queue, budget, source or accepted account changes. The full goal
remains active. Frozen evidence: `docs/evidence/2026-11-17-interval-queued-drc-profile`,
archive commit `a2cd1f4732df9805b8ba6bfd3e70b8eeabf4f179`; 12 files / 200,433
bytes, SHA-256 `e02a9ed79f7ff2c116f0156bd5b877d461a76831944f1d86d262f158fafba85a`.

## 2026-11-17: complete-chain trace allocation attribution

A separate 56-process diagnostic reproduces active-trace allocation variation
on the original fixed/five-repair complete-consumer images. Full outputs and
traces match in both recording domains. All captured size-632 allocations
resolve to snapshot B-tree insertion; the measured operation contains six to
nine such nodes. Each extra node corresponds to one allocation/deallocation
and 632 requested peak/traffic bytes, with operation-retained requested payload
unchanged at 10,381 bytes. Passive controls are constant. See the
[complete-chain attribution review](docs/research/solver-five-trace-attribution.md).

This finite attribution does not explain all historical charges, allocator-usable
or passive-trace failures, or establish ASLR causality. Interposition is not an
unchanged resource measurement or timing comparison. No source, cap or accepted
account changes; earlier charges remain. Source/package portable qualification
continues separately, and the full fifteen-phase goal remains active.

Frozen evidence: `docs/evidence/2026-11-17-interval-five-trace-attribution`, local
archive commit `8a2190d1a0f35ec794d8539fde0b209b240c77ab`; 129 files / 306,365
bytes, SHA-256 `21c21fcdc02ca3244204f681749cfffe4710524a7f3ff00ef14e0e2ab811ad1a`.

## 2026-11-17: complete native/WASM source and package consumer

The isolated five-repair source completes 2,064 successful configurations across
38 images, reproduces 32 expected baseline failures, and passes 1,800 full
output/trace comparisons. Eight strict Clippy configurations and formatting pass.
Six actual packages verify against 452 committed source files. Eight extracted
package images pass another 528 controls, each matching its source output and
trace, plus eight strict Clippy configurations and formatting.

All fourteen prospective image accounts pass, including positive packaging
charges added to every original source charge. No savings are netted. WASM
output ownership and bounds checks pass; its observed linear capacity is not a
retained-memory or RSS measurement. See the [distribution review](docs/research/solver-five-repair-portable-and-packaged.md).

No implementation or account is admitted. Earlier memory failures, quiet runtime,
resource/cancellation, broader release and full applicable validation remain.
Accepted accounts stay at 57 size and zero memory; inventory stays at 1,644 rows.
The original watcher remains terminal and its workflow parent held. The full
fifteen-phase goal stays active. No push.

Frozen source evidence: `docs/evidence/2026-11-17-interval-five-repair-portable`,
commit `f71d0b053690f057fdd173fe873726c4f0392809`; 309 files / 42,478,380 bytes,
SHA-256 `bb953e6601ba5c953b9b1d00172a45e431f4cf753f22a8761525b610ad276ab7`.
Frozen package evidence: `docs/evidence/2026-11-17-interval-five-repair-packaged`,
commit `2e30fed7f1350d6483eb1c329e95f6c4e4907db8`; 140 files / 11,295,447 bytes,
SHA-256 `dff37ca2318f78975a34c2bda4e38818bc486072c866b82b5dabb09036485ced`.

## 2026-11-17: cancellation publication boundary

Isolated candidate `9457d39b8` adds a final cancellation check before a cancellable
scope publishes its callback value. Original controls reproduce the documented
old behavior. All 1,321 candidate test invocations, the public native witness and
three WASM cases pass, with affected strict Clippy, docs and formatting. The
intentional contract extension preserves ordinary work-budget and Hypercurve
adapter code. See the [publication review](docs/research/solver-cancellation-publication.md).

Complete-consumer cost, distribution, quiet timing and remaining admission are
still open. Earlier failures and positive charges remain; no memory or size
account is admitted. Accepted totals stay at 57 size and zero memory, inventory
at 1,644 rows. The queue remains held and the original watcher terminal. The full
fifteen-phase goal remains active. No push.

Frozen evidence: `docs/evidence/2026-11-17-interval-cancellation-boundary`,
commit `6b9666fe36a5affa1bc289457d843b13f8a9f93f`; 79 files / 571,103 bytes,
SHA-256 `963fbacbf2f8aec16d500627514e2990071e4b1886639b17743d599cebdb33c2`.

## 2026-11-17: complete cancellation consumer cost

The wrapper candidate completes 3,906 successful consumer processes and retains
126 original publication failures. Full normal outputs/traces and added retry
outputs match. The 252 complete lifetime histories cover 8,064 cycles. Four
strict Clippy feature configurations, formatting and independent verification
pass. See the [complete cost review](docs/research/solver-cancellation-consumer-cost.md).

The 504 resource groups include 172 failures: normal operation stack grows
848–1,104 bytes, and some active-trace peak, traffic and count comparisons also
exceed their initial limits. All total capacities, cleanup/retention and initial
image checks pass. No cap or accepted account changes. An isolated alternative
will test a final check inside the scope function; this does not erase wrapper
charges or earlier failures. Fixed-baseline cumulative accounting, quiet runtime,
distribution and complete validation remain open. The full goal stays active.

Frozen evidence: `docs/evidence/2026-11-17-interval-cancellation-consumer`,
commit `0e2c1dad1ff7df5e9b8c9bea3a27227855ab7adc`; 14 files / 57,952,780 bytes,
SHA-256 `f49b4b07ece73452600445c65339605e0e9f24ef3734530be3bb5375da5cb065`. The lossless container retains 4,143 members,
all 24 images and recovery records for 26 relocated, previously archived images.
The original watcher is terminal and its parent held. No push.

## 2026-11-17: scope-local cancellation alternative

Isolated candidate `ab81708aa` moves the final cancellation check into the
existing scope function using a private const flag. All 1,321 affected test
invocations, native/WASM witnesses, 882 complete-consumer controls and 1,722 full
output/trace comparisons pass. All 336 initial stack groups pass unchanged caps:
operation delta versus five-repair is −160 to +160 bytes, versus wrapper −1,056
to −944 bytes. Strict owner/consumer/downstream Clippy, docs and formatting pass.
See the [qualified pilot review](docs/research/solver-cancellation-scope-pilot.md).

The first pilot stopped at the disk reserve during archival; its 442 completed
commands and stop are preserved. A reviewed continuation reused them without
reruns or a deadline reset. All checks finish within the original execution
budget. Older redundant sparse-checkout archive copies were removed only after
verifying committed data, with materialization instructions retained.

No source or account is admitted. Complete lifetime/resource, final-image,
cumulative, distribution, quiet runtime and release checks remain open; earlier
failures and positive charges remain. Accepted totals stay 57 size / zero memory,
inventory 1,644 rows. The full goal stays active; queue parent remains held.

Frozen evidence: `docs/evidence/2026-11-17-interval-cancellation-scope`,
commit `335b51a405c4fcb6be800aa41df4464a3c9d930e`; 968 files / 14,485,688 bytes,
SHA-256 `e5f317e42656b088fb53d1cf3d90b440496cac806caf1825d2c1c16ea339a92f`. No push.

## 2026-11-17: full cancellation qualification and matching launch names

Scope-local candidate `ab81708aa` now has full native lifetime/cumulative,
source native/WASM and packaged consumer evidence. Source distribution passes
3,600 controls with 336 expected old failures/traps; all 864 package controls
match their source outputs/traces. Six package archives match 452 source files.
All 90 native resource-image and 42 source plus 42 package image metric accounts
pass, including historical positive charges and required headroom. See the
[qualification review](docs/research/solver-cancellation-qualification.md).

The original native campaign has 264 failed resource accounts out of 648.
An 18-process factorial diagnostic proves a launch-name confound for one
23,168-byte usable-traffic discrepancy on unchanged executables. A full
15,396-process matching-launch campaign preserves all successful outputs and
traces, six expected failures, and 88,320 lifetime cycles. Its 267 failed accounts
and 287 conservative failed accounts show that this confound does not explain
all growth. Every legacy charge/failure remains; all total capacities, lifetime
retention and headrooms pass. The 26 remaining wrapper-to-scope failed groups
in the matching run all use active trace recording.

The original runner omitted helper processes from its guard and exceeded its
16,000-command ceiling by 41. This separate runner failure is preserved. Audited
source/package totals fit their caps, although their guards also omit helpers.
The matching runner counts all direct children and stays within its ceiling.
Preflight failures, corrections and original deadlines are retained; no failed
geometric attempt is silently replaced. Independent accounting/verification pass.

No implementation or exception is admitted. Accepted totals remain 57 size and
zero memory accounts; inventory remains 1,644. Quiet runtime, memory failures,
broader release checks and complete admission remain open. The queue remains
held, the original watcher terminal and the full fifteen-phase goal active.
No push.

Frozen evidence: `docs/evidence/2026-11-17-interval-cancellation-qualification`,
commit `e8bce8dcf528a98e2fad45f022fca70f93f84dfc`; 11 files / 199,940,791 bytes,
SHA-256 `6a97f9ba7c481c85fccf1accbac053a36998e0a5edef538395c96cfcbb690b5e`.
All 33,903 container members and committed parts are verified; earlier records
are unchanged. The archive retains all 172 images and the storage recovery
record. Documentation checks cover formatting, spelling, relative links,
repository budget, whitespace and the new archive manifest row.

## 2026-11-17: trace materialization tradeoffs and regression coverage

Two isolated collector alternatives complete 828 native controls with exact
independent outputs and 736 full comparisons. All nine owner tests and strict
owner/consumer Clippy checks pass for each source. Batch sorting fails 68 of 92
local accounts, including about 229 KiB peak growth with heavily aliased labels.
Incremental sorted aggregation fails 88 accounts: peak never grows in this
corpus, but traffic grows by up to 8,328 requested bytes and new reallocations
remain. Both are rejected; prior charges and allowances are unchanged. See the
[trace materialization review](docs/research/trace-materialization-pilot.md).

The independent ordered/content-equal snapshot and clearing-take regression is
retained on the primary worktree. Its nine affected owner tests, strict
all-target/all-feature Clippy and formatting pass after the final source edit.
The entire production prefix of the trace module is unchanged. Further work
will examine a content-keyed recording table, including event-recording cost;
collector-only evidence cannot qualify that change.

All 868 pilot direct children fit the original execution/count limits. A verifier
correction allows only two proven-identical Git revision reads sharing one log;
both command records, hashes and the original failed verifier are preserved.
No geometry rerun or deadline reset occurs. The full goal stays active, queue
held and watcher terminal. Accepted size/memory totals remain 57/0; inventory
remains 1,644. No push.

Frozen evidence: `docs/evidence/2026-11-17-interval-trace-materialization`,
commit `3cc14e7745bec36f5c6a0eebbf17587600aea49e`; 917 files / 4,824,326 bytes,
SHA-256 `e1c880a1930c61cdd0c3a8ddd4b7d62a6a68020ef9d1740a70f1c3caf66b7800`.
All source/stored/committed hashes and three images are verified; previous
archive records remain unchanged.

## 2026-11-17: trace key storage and event-time qualification

Three isolated trace candidates preserve all 276 complete outputs and pass their
nine owner tests, strict owner/consumer Clippy and formatting. Exact preallocation
repairs the canonical table's 32 retained-report failures; all 184 local memory
accounts then pass. A separate 384-control runtime screen rejects it: all 96
active pairs are slower, and 32 KiB labels add about 701–727 milliseconds per
16,384-event workload. See the [review](docs/research/trace-key-representations.md).

Borrowed report keys preserve event-recording code and reduce temporary bytes,
but three original allocation/free accounts fail by one. A diagnostic passes
identical label addresses to both collectors: all 552 actual iteration vectors,
full reports and allocation/free/reallocation counts match. All 1,104 paired
local accounts pass. Original failures and positive charges remain; this does
not establish full lifetime, stack, runtime, geometry or distribution admission.

Two zero-control diagnostic harness failures and their corrections are retained
within the original execution budget. All producers and independent verifiers
are terminal; the source worktree is restored to control. No production source
is merged. Accepted totals remain 57 size / zero memory, inventory 1,644 rows;
the full goal stays active, queue held and original watcher terminal. No push.

Frozen evidence: `docs/evidence/2026-11-17-interval-trace-key-representations`,
commit `5fd724d2341e4ad19fed277370f27e2bb66ccf0b`; 2,011 files / 19,028,727 bytes,
SHA-256 `f39d6584b892a9d999a06cafaf00e4b3922f14d35346987b6201af07cf279e9c`.
All source/stored/committed hashes and six validated native images are verified;
previous archive paths are unchanged.

## 2026-11-17: borrowed trace runtime and unwind cleanup

The borrowed-key candidate completes 1,344 source-runtime controls and 672 full
output comparisons. Large unique-label snapshots have median paired CPU increases
of about 6–7%; a 192-pair shared-input diagnostic preserves equal actual iteration
and full reports but still shows higher medians in those domains. Non-regression
remains unproven, so the implementation is not retained. Raw variability and
contrary samples remain; no universal slowdown or hardware cause is claimed.
See the [runtime review](docs/research/trace-borrowed-runtime.md).

An independent debug regression now verifies that aggregation unwind clears the
live counters, restores the recording scope and permits a later fresh report.
All ten affected tests pass on both implementations; strict owner Clippy and
formatting pass. Both production prefixes remain unchanged. Only this regression
is retained on primary; candidate-plus-test `024e1c1db` remains isolated.

Both timing producers and the test producer fit their original execution/count
limits. Offline verifiers pass. An archive whitespace failure is preserved; the
intentional test-fragment separator is stored losslessly compressed without
changing source or measurements. No charge, cap, fixture or policy is weakened.
All task producers are terminal, source control restored, queue held and original
watcher terminal. Accepted size/memory totals remain 57/0; inventory 1,644. The
full goal remains active. No push.

Frozen evidence: `docs/evidence/2026-11-17-interval-trace-borrowed-runtime`,
commit `0bb075b062f09e0df09727529becfd1f43c3d6ab`; 1,645 files / 10,550,284 bytes,
SHA-256 `0dd304266c9fdd74b8b04985f7034ef8f7e96eaffcb0dfaf331b68d882dc075a`.
All source/stored/committed hashes and three native timing images are verified;
previous archive paths are unchanged.

## 2026-11-17: standalone cancellation publication on primary

The scope-local publication check is now qualified independently of the unadmitted
solver migrations. Isolated candidate `8a454c860` reproduces the old failures on
current primary and passes 1,301 affected test invocations, native/WASM public
witnesses, strict affected lint/docs and formatting. See the
[standalone review](docs/research/cancellation-standalone-qualification.md).

Thirty final consumer images complete 3,294 observations: 3,288 successes and six
expected old publication failures. All 2,232 full output/trace and 2,760 output
comparisons pass; 592 complete histories cover 18,944 cycles. Twelve observed
resource accounts fail current growth allowances. Retaining historical floors
yields 17 failures, all active-trace cases. Absolute capacities, headrooms, stack
and lifetime retention checks pass. Image growth/headroom checks pass.

Historical review confirms only test additions on the accepted primary source
path and 1,644 corresponding historical/current full output/trace matches. It
retains 1,632 metric floors and 254 positive historical observations; the older
ten-boundary solver/wrapper ledger remains separate and unchanged. No numerical
allowance changes or source admission follows. The next work is bounded allowance
assessment for this functionality and its remaining runtime/distribution gates,
without requiring the optional trace refactor.

The Cargo cache warning and initial feature-coupling build failure remain recorded.
Reviewed corrections retain assertions and, for the consumer, its original deadline
and aggregate child ceiling. All validation producers/audits are terminal; source
control is restored, queue held and watcher terminal. Primary production code is
unchanged. Accepted totals remain 57 size / zero memory, inventory 1,644 rows.
No push; the full goal remains active.

Frozen evidence: `docs/evidence/2026-11-17-interval-cancellation-standalone`,
archive commit `a66bfd0ab72be6c91812d60ee6a0fc6993fe0c56`; 3 files / 28,577,694 bytes,
SHA-256 `5455967c5b5a948f2ca896e0a017a7ed26c1983be92ab4538b56b50814e2ba7e`.
The lossless payload contains 3,861 members / 259,497,686 raw bytes,
including 46 validated images. All original, decoded and committed hashes are
verified; earlier archive paths are unchanged.

## 2026-11-17: bounded cancellation bytes and focused runtime

The standalone cancellation candidate now has a reviewed per-target byte
proposal: tighter absolute small-case peak and relative stress-traffic limits
accompany bounded percentage/absolute growth respectively. All original values,
denominators, positive historical charges and verdicts remain. Across 288
accounts, 72 metric caps and 42 verdicts change; failures fall from 17 to one.
The historical allocation/free count failure remains under unchanged count caps.
See the [allowance/runtime review](docs/research/cancellation-allowance-runtime.md)
for exact caps, capacity/headroom rationale and the unchanged gates.

Eight final images pass 174 functional controls, six strict Clippy configurations
and formatting. The extraction build failure remains alongside its minimal
correction, sharing the original deadline and child ceiling. Timing completes
26 invocations / 552 case invocations / 16,560 samples, with 432 public full
control matches and 252 balanced comparisons over 63 common cases. Added retry
has 48 separately recorded absolute costs and no invented old-success baseline.

Default consumer comparisons are all faster; trace-build case medians are all
higher, ranging from +0.25% to +3.96%. Two passive-recording cases are slower in
all four pairs. All contrary samples remain. Non-regression is unproven, so the
candidate remains isolated. No runtime allowance, source admission or unchanged
benchmark rerun follows. Full owner/successive/distribution gates remain open.

All producers are terminal and audits pass within their declared bounds. Source
control is restored clean; the workflow queue remains held and its original
watcher terminal. Primary production code and accepted totals of 57 size / zero
memory exceptions remain unchanged; inventory remains 1,644 rows. No push; the
full goal remains active.

Frozen evidence: `docs/evidence/2026-11-17-interval-cancellation-allowance-runtime`,
archive commit `1286123944e231ef7c2e4706cd9df2d502420788`; 3 files / 25,668,639 bytes,
SHA-256 `c6b396475714d5702a72a53f5ec73deba66ebaf831b61f31c36872a8dc6f7b53`.
The payload retains 4,677 members / 100,888,275 raw bytes and nine
validated images. Original/decoded/committed hashes pass; old archive paths
are unchanged.

## 2026-11-17: cancellation finish-borrow candidate

Isolated candidate `1abfbc3fc` performs the final cancellation decision inside
scope finish's existing frame borrow. A shared owner helper preserves the prior
first-failure lookup and unchanged cancellation scan; ordinary scopes still skip
the final poll. The new destructor-reentry witness checks parent restoration,
admitted work during cleanup and fresh retry. See the
[finish-borrow review](docs/research/cancellation-finish-borrow.md).

Current primary reproduces six owner publication failures and the adapter failure,
with three passing owner controls. The original publication implementation passes
all nine publication tests. The revised candidate passes 1,303 affected test
invocations, strict owner/adapter lint/docs, examples/benches and formatting.
Independent source/output audit passes. No existing test or adapter assertion
changes; only the owner implementation differs from the reference with this test.

Four runtime images pass 128 functional controls and two strict lint configurations.
The targeted eight-case screen completes 152 case invocations / 4,560 samples,
with 608 public full control matches and 96 balanced comparisons. All eight case
medians are lower than the original publication candidate. Against primary, seven
are lower and one is +0.94%, with mixed individual pairs. All contrary evidence
remains. This supports further investigation, not full non-regression or admission.

All producers and audits finish within their declared limits. Full runtime,
lifetime memory, final-artifact and platform qualification remain open; old byte
verdicts do not automatically transfer to the changed source. Historical count
charges and earlier failed obligations remain. Source control is restored clean,
queue held and original watcher terminal. Primary production and accepted totals
of 57 size / zero memory exceptions are unchanged; inventory remains 1,644 rows.
No push; the full goal remains active.

Frozen evidence: `docs/evidence/2026-11-17-interval-cancellation-finish`,
archive commit `6cd1ef20d98dfa409d14ed74272850f562f8a6d8`; 3 files / 9,784,408 bytes,
SHA-256 `8f3669c71dad32650f9a5fdc4be768c35af7c2c1ea551686009d33b57e65ca01`.
The lossless payload contains 1,434 members / 33,176,529 raw bytes and
four validated images. Original, decoded and committed hashes are verified;
earlier archive paths are unchanged.

## 2026-11-17: finish-borrow complete consumer resources

The revised cancellation candidate now has fresh complete-consumer resource
measurements across fixed baseline, primary, original publication and revised
implementation. Thirty hash-verified images are reused and remeasured; twelve
new images are built against unchanged consumer sources, observers, assertions
and dependency lock. Four strict consumer lint configurations and formatting
pass. See the [resource review](docs/research/cancellation-finish-resources.md).

Collection completes 4,938 observations, with 4,932 successes and six expected
primary failures. All 3,876 full output/trace and 4,404 output comparisons pass.
888 complete histories cover 28,416 cycles. Fresh accounting has 14 failed
accounts; historical charges raise that to 25. The unchanged byte proposal leaves
15 failures, including counts, cumulative bytes and cleanup stack. All absolute
capacities, headrooms, lifetime retention and final-image limits pass.

Historical reconciliation verifies 3,288 matching full output/trace pairs and
preserves original values, denominators, positive charges and verdicts across
3,264 metric records. Its 744 positive entries include aggregate floors, not
744 independent measurements. An independent fraction verifier confirms the
accounting and unchanged gates. No additional cap increase or source admission
follows; both publication implementations remain isolated.

The producer uses 5,219 direct children / about 1,186 seconds within 6,500 /
5,400. Three audits pass in one shared 300-second window, taking about 73 seconds.
All producers/audits are terminal and source control is restored clean; queue
held and original watcher terminal. Full runtime, distribution and platform
qualification remain open. Primary production and accepted totals of 57 size /
zero memory exceptions are unchanged; inventory remains 1,644 rows. No push;
the full fifteen-phase goal remains active.

Frozen evidence: `docs/evidence/2026-11-17-interval-cancellation-finish-consumer`,
archive commit `11a20d85962cf35c577cad25b61e0b9d367941c1`; 3 files / 32,476,138 bytes,
SHA-256 `e37499a3986a178c9c6c310691cc7e8a9d53f575c8ceaff0146241ec4338ee91`.
The lossless payload contains 5,313 members / 323,661,870 raw bytes and
42 validated images. Original, decoded and committed hashes are verified;
earlier archive paths are unchanged.

## Isolated affine axis owner extraction (17 November 2026)

[The new review](docs/research/axis-projection-owner.md) records a smaller Phase 11
candidate: Hyperdynamics owns exact affine axis projection behind an opt-in
feature; Hyperpath retains deprecated wrappers and unchanged error variants.
The algorithm/report bodies and all original test assertions are preserved.
Final owner suites pass 602 tests, Interface core passes 201, and the independent
canonical motion transcript remains unchanged. Four external full-output/trace
comparisons match under both policies; the direct consumer needs no geometry
crate. Strict local lint, documentation, formatting and repository checks pass.

CSGRS `8f351042e1880a8d77b50606df8b387066522c45` and Interface
`e5af3b4383d33c440d087e0d2b933fccde766b9c` remain isolated, unaccepted source
candidates. Runtime, complete lifetime/stack, final artifact and distribution
admission remain open. No new memory or size allowance is accepted; prior
charges, rejections and the held release queue are unchanged. Feed/lookahead/jerk
extraction and the full 15-phase objective remain required.

## Axis projection costs and locked consumer repair

The six previously selected Hyperpath benchmarks improved in both matched
timing blocks. The owner binary grew 10,976 bytes, exceeding its ordinary
incremental cap. A consumer selecting Hyperpath tracing alone exposed an extra
lattice event. Isolated candidate `75dd0273e` restores the original counts and
the standalone dependency locks. Full runtime, memory and downstream size
admission remain open. Primary commit `4e19d4d33` repairs the two stale Alumina
measurement locks. Native tests pass and the release WASM module replays 4,369
exact-oracle calls. [Measurements and evidence](docs/research/axis-projection-owner.md#matched-owner-timing-and-legacy-trace-repair).

## Native projection memory and wrapper qualification

The Hyperpath legacy adapter used 448 more stack bytes than baseline in 18
workloads. Two targeted forwarding hints on isolated candidate `62bcec291`
reduced the increase to 48 bytes throughout. The direct owner matched baseline.
The revised owner benchmark is eight bytes smaller than the fixed baseline.
Four-cycle histories covered rejection, planning, materialized independent
replay and cleanup. The complete Alumina diagonal G1 fixture also passed on all
three measured revisions. Resource and timing results, including adverse
observations and remaining gates, are recorded in the
[native projection review](docs/research/axis-projection-memory.md).

## Dyadic atanh at the open domain limits

Phase 5 owner qualification at `1051053ae` covers P6 and P11 for a finite
closed dyadic source and the open domain (−1,1). On `13a8309ad`, the four
existing inverse-function tests passed. Their one-sided atanh examples checked
a finite endpoint of zero. The new regression checks the direction of each
finite endpoint against 512-bit directed MPFR, grid membership, local coverage,
the side of each infinite limit and the exact empty-domain outcome.

The deterministic matrix passed all 448 combinations across four source grids
and four output grids, including a source endpoint within 2^-60 of either pole.
The new CI fuzz target completed 10,000 local ASan runs from an empty corpus
with seed 20261126. The final run took 16 seconds. The inverse suite passed on
Rust 1.88 with dyadic alone and on 1.95 with all features. All Hyperinterval
all-feature tests and doctests and nine Hypersolve expression-interval consumer
tests passed on 1.95. Strict owner and fuzz Clippy, both formatting checks,
the locked fuzz build check and the pedantic workflow audit passed. These are
native Linux results. Hosted workflow execution remains open.

The change adds tests and CI coverage. The memory and binary growth accounts
receive no charge. Archive commit `cfe9563cf6b5eb92f2a5b4af49720c839871811a`
contains the final corpus, logs and commands at
`docs/evidence/2026-11-17-interval-atanh-domain`.

## Fixed-point conversion to host intervals

Section 3.7 requires fixed-point intervals to convert to both exact host
families. At `024bff5ff`, the new public integration witness failed to compile
for both missing methods. Commit `26a6c6b59` implements the conversions on
`FixedInterval` under their respective features. An ordered numerator pair
`[a,b]` with scale `F` maps directly to `[a,b] × 2^-F`. The rational
constructor uses the positive denominator `2^F`. The dyadic constructor
reserves two linear endpoint stages before materialization and normalizes the
common exponent. Empty requires zero work. P1, P5, P6 and P18 apply.

The owner corpus recovers both original numerators for 4,320 endpoint
pairs across all 32 i32 scales and 64 i64 scales. It also consumes converted
widths, checks empty and singleton results, and exercises both kinds of budget
refusal and a fresh retry. Every instrumented line of each new method ran.
The conversion fuzz target completed 100,000 ASan runs, including independent
GMP checks at 30, 32 and 63 fractional bits. Node executed 720 exact endpoint
checks on the release WASM module for four representative source formats.

All-feature owner tests and doctests passed on Rust 1.95, including 251 tests
counted once across suites. The fixed/rational suite passed independently on
1.95 and stable 1.99. Rust 1.88 passed the fixed/dyadic regression and both
fixed-only and fixed/dyadic checks for Thumb and WASM. The fixed-only normal
dependency tree contains only Hyperinterval. All 87 downstream Alumina FOC
library tests passed in release on 1.88. Strict owner/fuzz Clippy, strict
rustdoc, formatting, the pedantic workflow audit and repository budget passed.

Four existing native release executables matched the baseline byte for byte.
The fixed example measured 443,208 bytes with fixed alone and 444,296 with
all features. The all-feature rational cost example measured 1,217,824 bytes,
and the Alumina FOC host test executable measured 6,151,184. The new standalone
caller measured 838,368 bytes, or 843,816 with the measuring allocator.
The WASM verification module measured 214,716 bytes.

Nine warmed blocks of 65,536 conversions on one pinned CPU included owned
result destruction. Across four nonempty cases, dyadic median times ranged
from 142 to 149 ns and rational median times from 117 to 261 ns. A separate
warmed allocator probe observed 64 bytes peak requested heap for dyadics,
168 to 336 bytes for rationals, zero for empty, and zero live bytes after
destruction in each case. These finite observations cover the new workload.
No memory or binary exception was needed. Hosted CI, MCU cycles, board flash
fit and hardware-in-the-loop qualification remain open.

Verified archive commit `f21108ae036892f7832facaaab05079fdc4efb38` contains
the logs, corpus, probe sources, output measurements and before/after images
at `docs/evidence/2026-11-17-interval-fixed-host-conversions`.

## Fused affine hulls for primitive integer ranges

Plan §3.5 calls for an affine image. At `e42236cad` the new public witness
failed to compile for `IntegerRange`. A checked scale followed by a checked
translation can fail even when both final endpoints fit. Commit `06ebc9e55`
implements the operation in Hyperinterval for all twelve signed and unsigned
primitive families. It returns the smallest discrete interval containing the
image, including skipped integers when the factor has magnitude above one.
Empty and zero-factor images compose with subsequent affine operations.

For P1, P5 and P6 the implementation bounds the unsigned displacement from
the offset by the exact distance to the appropriate type limit. Any larger
displacement is unrepresentable, even with cancellation. An admitted
displacement gives an exact representable endpoint. Monotonicity in the
factor's sign determines which source endpoint supplies each extremum.
The implementation uses fixed-width arithmetic throughout, including i128.

An independent widened-integer release oracle passed all 33,554,432 signed
and unsigned byte singleton triples. Owner tests cover every endpoint family,
enumerated nonsingleton images and full-width GMP comparisons. The GMP-backed
sanitizer target completed 100,000 runs. The 32-bit WASM module passed 40,620
calls on 17,484 JS BigInt cases across i128, u128, isize and usize, with
5,784 representable hulls and 11,700 expected overflows. Its final 7,272-byte
module has zero imports. The Thumb callable ELF linked with 5,502 text bytes
and no unresolved symbols. MCU execution, timing and board flash fit remain
open.

Rust 1.95 passed 254 all-feature owner tests and doctests. Rust 1.88 passed
the integer-only suite plus Thumb and WASM release builds. The eight Alumina
clock release tests passed before and after. Both its native executable
(4,610,432 bytes) and the all-feature interval order benchmark (969,152 bytes)
matched their baselines byte for byte. Owner and fuzz Clippy, rustdoc,
formatting, the workflow audit and the repository budget passed.

Seven native workloads covering signed and unsigned success, cancellation,
overflow and empty made zero allocation calls in 4,096 iterations apiece.
Twelve warmed blocks of 262,144 calls per case on one pinned CPU measured
median complete-call costs from 3.93 ns (empty) to 20.75 ns (full-width
signed success). These are absolute observations for the new API.
The source requires no new memory or binary exception for the measured
existing callers.

Verified archive commit `2ef71922b1f8bb6ad7b542ebea396d843eb7b4d0`
contains the logs, exact checks, final corpus, caller images and raw samples
at `docs/evidence/2026-11-17-interval-integer-affine`.

## Exact dyadic midpoint partition and consumer qualification

Hyperinterval now splits a nonsingleton into two exact closed intervals
on the coarsest common grid needed for the partition. For source mantissas
`a,b`, an even sum gives midpoint `(a+b)/2` on the source grid. An odd
sum requires the next finer grid with endpoints `2a,a+b,2b`. In the latter
case the midpoint is odd. In the former case both outer endpoints are odd
unless the existing exponent is maximal. These facts establish canonical
children without revalidating or normalizing temporary intervals. Empty and
singleton return no split. An unrepresentable half at the minimum exponent
returns typed overflow. P1, P5, P6 and the Phase 6/13 shared work limits apply.

The retained scalar API at `f402f67bd` splits `[-3,4] × 2^-20` into
`[-6,1] × 2^-21` and `[1,8] × 2^-21` within four bits and 16 total
work units. The existing box translates and clips the source, requiring 106
units at 16 bits and reporting size exhaustion at four bits. Independent
integer expectations cover exponent extrema, 8,192-bit input, repeated
splits, empty and singleton sets, shared facets, exhaustion and retry.

Rust 1.95 completed 554 owner, lattice, SDF, reachability and documentation
tests. The dyadic-only Rust 1.88 tests, Thumb and WebAssembly release builds,
100,000 sanitizer runs, strict owner and fuzz Clippy, and formatting passed.
The 43,834-byte direct WebAssembly caller exercised every ordered pair from
the bounded input grid at four exponents and the 16-unit split.
The native and WASM box clients and two full native SDF covers preserve
their original work, results and heap measurements on primary. Whole-call
stack high-water measurements also agree across the fixed, current and
retained revisions for both SDF sources and the reachability example.
The three measured callbacks used at most 29,432 bytes on guarded 2 MiB
stacks with cold and warm entry. These observations cover one x86_64 worker.

The separate box migration at `921378c02` reproduced both complete cover
definitions and the four-box Hyperdynamics reachability result. It reduces
the dependency cover from 227,460 to 227,189 scalar work units and the affine
sphere from 3,624,110 to 3,623,512. Both peaks and post-drop live bytes
agree. Its release reachability executable is 1,856 bytes smaller, but its
allocated RELRO padding grows from 560 to 1,880 bytes. Two assembly variants
also crossed existing size gates. The migration remains isolated. The
scalar-only change charges eight file bytes to the SDF client, sixteen to
the reachability-only example and four unwind bytes per native SDF or
reachability configuration. Measured native function bytes and existing
box WASM file/code sizes hold. All prior charges are preserved in the
[per-target accounts](docs/research/interval-size-accounts.json).

The qualification archive includes controls, candidate and retained binaries,
WASM execution, raw timing blocks, allocation and stack observers and the
reproduced failures. Evidence commit `4db3f8e17`, record
`2026-11-17-interval-bisection`. [Evidence index](docs/evidence-archive.tsv)

## Taylor coefficient sharing under the memory allowance

Order-three preparation's 16-byte increase fits the proposed retained-memory
allowance with cumulative accounting. Two independently measured linked
size metrics keep the candidate isolated. All 21 targeted tests and both
complete output/trace comparisons passed. The historical rejection and
original fixtures are preserved. [Reassessment and numerical gates](docs/research/adaptive-stage-ownership-reconsideration.md#reassessment-under-the-bounded-memory-exception)

## Phase 7 solver radius certification and bounded stack

Hypersolve now requires a certified nonnegative radius in all three interval
packages and all three Krawczyk packages. A negative input hidden by terminal
rounding previously produced satisfaction and unique-root certificates.
The owner suites passed 1,523 all-feature and 1,388 minimal-feature completions,
and all 11 downstream reachability tests passed.

The first native stack exception charges up to 32 bytes per completed workload
and 48 bytes cumulatively. Requested heap remained within its previous peak,
and individually measured native and WASM size growth fits the existing target
caps. The separate center-replay and approximate-row aggregation defects remain
open. Their broader repair exceeded the per-section binary-size gate.
[Admission, measurements and remaining defects](docs/research/interval-box-proof-scope.md)

## Phase 7 quadratic proof corpus and whole-box replay

The primary branch now checks 5,400 solver reports against independent
polynomial extrema under both predicate policies. Both deliberate remainder
omissions produced false sign certificates and failed the new tests. The
restored source passed 1,525 owner tests and doctests, strict checks and all
11 downstream reachability tests.

The composed repair on `interval-box-proof-and-diagnostics-review` corrects
the outer verdict, certification counts and failed-row diagnostics together.
It passed 1,535 owner tests and doctests. On the 84-row client it changes
24 verdicts while preserving all evidence, dispatch and allocation records.
Two native allocated-section measurements prevent retention of that version.
A second composition passed both native size gates but exceeded the existing
WASM file/code caps and eight per-workload stack caps. The review covers the exact fixtures, attempted implementations,
portable execution and linked measurements.
[Phase 7 qualification](docs/research/interval-box-proof-scope.md#exact-extrema-qualification-and-composed-repair-review)

## Phase 3 dyadic set-query review, 18 November 2026

The primary branch now checks complete normalized components for 729 inputs,
exact inclusion for 12,544 pairs and budgeted separation at opposite exponent
limits. Four existing release cost executables stayed byte-identical.
The full and finite-only named overlap APIs passed the measured linked-size
caps. Both remain on local review branches after native mixed-client timing
regressions. The finite version allocated zero heap bytes on the measured
2,049-bit comparisons. The existing memory allowance and cumulative charges
remain in force. [Dyadic overlap qualification](docs/research/interval-dyadic-overlap.md)

## Phase 3 exact dyadic overlap and disjointness, 18 November 2026

Commit `207e88c55` delivers all six named queries for finite, extended and
split dyadic sets. Empty sets, closed contact, extreme exponent gaps and
division-produced rays retain their exact set meaning. The three-CPU
campaign covered both arithmetic dependency locks and eliminated the
previous mixed-client timing regression. Nine comparable final binaries
passed the size caps. The bounded memory exception charges 16 bytes of stack
on each affected composition workload, with at least 2,085,520 bytes of
measured thread headroom. Owner, downstream, native release, WASM and fuzz
checks are recorded with the raw measurements.
[Retained query qualification](docs/research/interval-dyadic-overlap.md#retained-query-implementation)

## Phase 7 multivariate Krawczyk proof policy, 18 November 2026

Retained `4d4b472e6`. The multivariate quadratic certifier now requires
certified decisions throughout its inverse construction, containment and
strict contraction checks. Three independent regressions reproduced false
verdicts on the previous primary. A separate affine oracle in `aaebcb395`
checks 600 reports and the exact coordinates of every accepted root.

The regular and adversarial memory workloads passed their individual caps.
Peak requested heap held, and the largest new stack charge was 48 bytes.
The existing linked WASM client and its code section each grew by one byte.
Affine and univariate quadratic Krawczyk callers still require `STRICT` for
root certificates. The full repair remains on `interval-krawczyk-proof-review`
after a reproducible one-variable runtime regression.
[Proof witnesses, costs and remaining scope](docs/research/interval-box-proof-scope.md#krawczyk-containment-contraction-and-elimination)

## Phase 0 inventory refresh and review preservation, 18 November 2026

At `703f86de6`, running the inventory command on a copy replaced all 230
revision-specific reviews with lexical candidates. It missed the three
public Krawczyk certifiers and the six dyadic overlap/disjoint methods.
In 14 rows the inferred directory owner differed from the Cargo package.

Commit `7d4687ae8` refreshes the live rows from the plan’s source modules,
public declarations in the interval and dynamics crates, workspace-wide
name matches, and names previously reviewed in the same file. Revision-specific
reviews keep their original fields, including historical line numbers.
A moved or repeated name acquires its own fresh candidate row. The command
writes a temporary file and checks for intervening review edits before
replacement. It reproduced the same TSV bytes both in place and in a fresh
destination. From a named worktree its default CSGRS source is that checkout.

The inventory contains **4,125 current declaration sites in 473 Rust files**
(3,463 CSGRS, 337 Interface, 325 Firmware) and **239 review records**. Every
current line was checked against its source and every package name against
Cargo metadata, including four manifests outside the initial workspace
metadata. Nine new records cover the six dyadic set queries and the three
Krawczyk proof entry points. Ten dyadic and seven solver integration tests
passed. Four negative witnesses from `0dcfb17ba` still fail on
`703f86de6`, including an incorrect affine root coordinate and terminal
equality promoted to a root certificate. The multivariate point-box result
remains undecided on the same witness.

Five generator regressions passed. Eight pinned process runs compared
buffering and streaming the **same** completed inventory. Median elapsed
time was 2.62 seconds for both. Observed peak RSS was 24,240–24,516 KiB
with buffering and 21,912–22,112 KiB with streaming. This is host tooling;
the geometry and firmware resource accounts have no new charge.

Interface was at `5b532c803` and Firmware at `7e0d5a43d`. Live candidates
still require body and consumer review. Macro expansions, reexports, and
unnamed endpoint tuples need separate discovery. The raw checks and matched
measurements are indexed under `2026-11-18-interval-inventory-refresh`.
[Source inventory](interval-inventory.tsv) · [Evidence index](docs/evidence-archive.tsv)

## Phase 3 closed real-ball boundaries, 18 November 2026

Commit `7584d3163` checks the exact center, radius and both exported
endpoints after reflection, scaling, translation, addition and subtraction.
An independent grid-residue calculation covers coarse, odd, even and
128-bit refinement and a return to coarse precision. The second test checks
cold and warm cancellation, both subtraction orders, typed failure and
successful retry when the displacement carries the signal.

At `d36d2b5c8`, reflecting and scaling `[-2,0]` by `-1/3` and
translating by `1/3` constructs the closed interval `[1/3,1]` exactly.
The public membership filter projects the displacement onto a dyadic grid.
It returns `Unresolved` for either exact endpoint even at precision -128.
We evaluated three repairs on ten newly decidable and ten control fixtures
(including a repeated symbolic control). Each produced the expected results
in a native client and 80 WASM calls across two module instances. The
fallback additionally passed 1,485 integer-oracle comparisons across closed
rational balls. The unchanged primary still has the off-grid membership gap.

| Candidate | Measurements against the primary | Disposition |
| --- | --- | --- |
| Retain the displacement and consult its exact rational value on overlap | Control dispatch receipts matched. Native file +208 B, defined functions +150 B, WASM file +84 B and code +58 B. No measured stack growth. Eight newly decidable fixtures used an additional 120–168 B of peak requested heap. | Retained for review at `4f05060aa`. The fresh dyadic upper-endpoint control slowed by a paired median of 0.72% on each of two CPUs, in 17/18 and 16/18 pairs. The fresh negative exclusion control slowed by 0.80% and 0.75%. |
| Consume certified rational displacement bounds directly | The native file shrank 5,480 B. All ten control trace receipts changed. Three native images exceeded individual section allowances. Warm stack on the reflected lower endpoint rose from 1,760 to 1,848 B. | Exceeded the per-workload 1% stack cap and changed existing dispatch. |
| Consult both input rationals after ambiguous grid comparisons | Control traces and image caps passed. Four previously completed warm dyadic controls used 16 additional stack bytes each, exceeding their 1% caps. One new endpoint used 88 additional stack bytes. | Exceeded the per-workload memory limits. |

The fixed program reference was `c9f1f9ac5`. The 20 single-worker workloads
peaked at 1,680 requested heap bytes and 1,848 allocator-usable heap bytes
on every revision. Every post-drop sample returned to zero. The smallest
measured thread headroom was 2,083,784 bytes. Individual workload charges
and verdicts are recorded separately. None of the three candidates was
admitted, so no production memory or image charge was added.

A separate witness isolates signal loss below the ball operation.
With an active abort on `sqrt(2)`, its own scalar enclosure returns
`None`. Both orders of subtraction with zero preserve that outcome.
With the nonzero shift `7/5`, the derived scalar supplies bounds and the
ball returns a verdict while the signal is active, cold or warm. The
original negative witness remains executable on the review branch.
These runs demonstrate a cancellation defect. They did not demonstrate
a wrong geometric answer on that fixture.

On the primary, 273 Hyperinterval tests and doctests passed with all
features. The release real-ball targets passed five tests, the disabled
feature target compiled, and seven downstream tests and doctests passed
in Hyperlattice Interval and Hyperdynamics with their default features.
Workspace formatting, owner all-feature/all-target Clippy and owner rustdoc
passed. Two additional owner reviews bring the inventory to 241 reviewed
records. Raw observer sources, paired timings, section measurements, stack
and allocation traces, all candidate patches, exact witnesses and WASM
images are indexed under `2026-11-18-interval-real-ball-boundary`.
[Evidence index](docs/evidence-archive.tsv)

## Phase 14 translated enclosures, cancellation and retry, 18 November 2026

Commit `3ad284efe` exercises the public Hyperreal-to-dyadic bridge on
`sqrt(109) − 7/5`. The oracle translates both exported endpoints back, checks
the positive branch and squares them independently against 109. A cancelled
clone leaves the shared result cache cold. Another reader can refine it, the
cancelled reader still reports unavailability, and a larger budget after
cancellation permits a finer successful retry. Three native workers request
different precisions from the shared expression while a fourth participant
uses its own cancelled handle. The two release tests passed in 32 fresh
processes.

The bare-WASM client runs the same serial oracle twice in each of two fresh
instances and requires zero imports. The existing interval runner detects
the client's declared Rust version from Cargo metadata. On Rust 1.88 the
existing float, lattice and dyadic clients still run. Rust 1.95 also runs
the Real bridge. Stable Rust 1.99.0 passed the native tests and the same
four WASM calls (module SHA-256
`478f86c47d32204cdf8cd1c9cdbea703621358b45e8d6192bb23dc8eaa13485d`).
All three toolchains passed the existing 55,802-case float corpus. An attempted cross-build of the native Cargo test targets still
stops in the GMP development dependency; the standalone WASM client has
no GMP dependency. Removing the scalar cancellation checks or collapsing
the returned dyadic interval to its lower endpoint independently caused
the maintained native tests to fail and the WASM client to trap.
[Lifecycle tests](crates/hyperinterval/tests/real_dyadic_lifecycle.rs) · [WASM invocation](crates/hyperinterval/conformance/README.md#real-to-dyadic-cancellation-and-retry)

**Open scalar defect.** On `8fe441513` a nonrational operand can lose its
registered abort signal when translated by a nonzero rational. An exact
arithmetic fold preserves the flag on an operand, but a new computable sum
can acquire a root with no signal. Descendant evaluation takes the signal
from that root. The cached `π−3` shortcut also returns a value without the
caller's control. With an active signal, 24 ordered native records and 96
WASM records returned scalar bounds, dyadic bounds and ball verdicts. Each
returned bound and membership verdict agreed with the independent numerical
expectation on these cases. The failure was publication during cancellation.
Attaching the signal to the completed expression is the demonstrated
working case on the primary.

**Candidate disposition.** We compared four implementations for carrying the
sole refining operand’s flag through a rational translation. The final candidate
at `7e881ac12` on `interval-translation-signal-review` covers both orders of
addition and subtraction, repeated shifts, cached constants, independent
readers, retry and an independently signalled exact rational. All 24 native
and 96 WASM controlled records rejected the active scalar, dyadic and ball
requests and recovered after clearing the flag. The original failing ball
witness passed. Two independently controlled nonrational operands are outside
this qualification.

All four candidates preserved the 20 ordinary native trace records and
80 WASM control verdicts. The first exceeded native function-byte and WASM
size limits. The next two passed the image limits but exceeded individual
stack caps. The final candidate moved the folds into the shared sum
constructor. It passed ordinary heap and stack caps and reduced the native
client by 2,608 bytes and WASM by 1,494 bytes. The native client's
`.relro_padding` grew from 1,336 to 3,264 bytes and the owner test image's
from 2,688 to 3,200 bytes. Each exceeded its section limit. On the unchanged
warm upper-endpoint query the median paired slowdown was 10.79% on both
pinned CPUs, with all 18 pairs slower on each. The slower result was still
`Unresolved` on both revisions.

The controlled workloads with the repair returned zero retained bytes and
peaked at 2,936 requested and 3,152 allocator-usable heap bytes. Peak callee
stack reached 4,592 bytes with 2,082,960 bytes of measured headroom on a
guarded 2 MiB worker. These runs include publication versus refusal followed
by retry, so their raw resource differences are recorded separately from
the unchanged controls. No candidate was admitted and the production
resource accounts receive no charge.

Primary validation passed 260 all-target/all-feature tests, 15 all-feature
doctests and seven downstream tests and doctests, together with the
no-default-feature suite, strict owner Clippy, rustdoc, formatting and the
WASM library build. Four body and consumer reviews bring the inventory to
245 review records. The candidate patches, expected failures, runner reports,
allocator observations, section accounts and paired timings are stored under
`2026-11-18-interval-translation-signal`.
[Evidence index](docs/evidence-archive.tsv)

## Phase 3 and 6 finite dyadic order, 18 November 2026

The retained change at `0f50ed48f` checks closed containment, containment in
the real interior, and strict left separation against independently scaled
signed integers. It covers 19,600 ordered interval pairs (58,800 verdicts),
shared endpoints, empty operands, exhausted work with retry, and 4,097-bit
magnitudes with matching leading powers and distinct low bits. The arithmetic
fuzz target now checks the same three relations. The budget documentation
also records the existing one-unit minimum for a linear stage.
[Maintained oracle](crates/hyperinterval/tests/dyadic_finite_inclusion.rs)

| Reproduced gap on `68fd84f3a` | Candidate result | Admission decision |
| --- | --- | --- |
| Including a one-bit interval at the minimum exponent in one at the maximum exponent exhausts the common-grid reservation. | Six implementations were linked. The shared scalar comparator at `63e798155` also handles subdivision outputs as subsequent Cartesian operands. All four previously exhausted native and WASM fixture pairs completed. Both SDF covers and the reachability result replayed exactly. | The direct native file shrank 2,168 bytes and the WASM file shrank 2,124 bytes. The native `.relro_padding` section grew from 1,192 to 2,928 bytes, exceeding its individual cap. All six implementations exceeded at least one linked-image limit. |
| Comparing two zero endpoints reports size exhaustion under a zero-bit magnitude cap, including when the work allowance would cover the answer. | `ff3632485` charges the actual width. `7cde9d615` handles zero on the failed size reservation. Both distinguish size from work exhaustion and preserve retry. The direct-width variant passed 28 ordered WASM calls across two fresh instances. | Direct-width comparison slowed the unchanged standalone ray workload by paired medians of 2.08% and 1.62% on two pinned CPUs. The recovery path slowed the same workload by 2.08% on both CPUs in the isolated rerun (18/18 and 17/18 pairs). A third variant using stored signs exceeded native section caps. |

The memory evaluation applied the predeclared limits to individual workloads
on a guarded 2 MiB native worker. Direct-width comparison added zero bytes to
the peaks of all 25 workloads (seven zero-width cases, 15 finite/Cartesian
cases, and three complete downstream reports). The newly successful all-zero
membership and scalar order calls allocated no heap and used at most 960 bytes
of measured callee stack. Across this campaign, the complete consumers
retained zero heap after drop and left at least 2,056,824 bytes of worker
headroom. The historical overlap query, full lifetime and allocator
observations also matched the preceding revision under both dependency locks.
The shared-comparator candidate passed all 18 of its measured workload caps.
The previously charged production memory accounts carry forward without a new
charge.

On the retained revision, the direct native and WASM images and the native
downstream measurement client were byte-identical to the preceding revision.
The standalone oracle occupies 1,100,464 bytes as a release test executable.
The four affected crates passed 715 all-feature tests and doctests. Rust 1.88
passed 123 dyadic-only owner and lattice tests and doctests, plus bare WASM
and Cortex-M library builds. Strict Clippy, rustdoc, formatting, the release
oracle, the WASM consumer, inventory tests and 100,000 ASan fuzz executions
passed. Hardware execution was outside this host qualification. The full
extreme-scale and zero-bit success witnesses remain executable on the named
review branches. The primary retains both implementation gaps.
[Exact checks, candidate patches, per-workload costs and paired timings](docs/evidence-archive.tsv)

## Phase 12 and 14 reachability step and subdivision policies

Revision `b5f2f6798` adds the ODE policy fuzz target required by §14.3.
The preceding coordinate-change target integrated trajectories but did not
call the finite cover search. The new target and two ordinary integration
tests run the same independent integer oracle against `ExpressionField::reach`.
[Oracle and fixtures](crates/hyperdynamics/tests/reachability_policy.rs) · [Fuzzing and reproduction](crates/hyperdynamics/fuzz/README.md#reachability-policies)

| Generated family | Evidence checked through the public API |
| --- | --- |
| Two-dimensional constant flow from a nonpoint rectangle | Signed integer velocities translate each initial cell exactly. Check every native prefix and its transported box, the full widest-axis partition including ties, and every member of the final Cartesian grid with its multiplicity. Exact cell, depth, evolution and segment allowances complete. One-short limits produce the expected typed stop and withhold the cover. |
| One-dimensional affine-in-time flow from a point | For `x' = a*t + b` the integral determines the endpoint, and the left and right rectangle sums determine the tight Picard interval on these fixtures. Check both endpoints after every step of every time-refinement attempt, including attempts too wide to supply the requested cover. Check refusal one refinement short and completion at the exact epsilon boundary. |

The 126 deterministic configurations run in the normal Cargo suite. Two
native ASan campaigns each executed 10,000 inputs (42 and 46 seconds). The
second started from the two seeds configured in CI. Short inputs can be
discarded by the fixed-width decoder; both final corpora contain all seven
policy modes. The maintained tests independently detected a duplicated final leaf,
premature acceptance of an over-wide root, and publication of a cover
after a stopped search. The duplicate-leaf seed also failed
in the compressed ASan client, with a file and line in the shared oracle.

The first sanitizer image occupied 64,534,360 bytes and missed the 16 MiB
free-space requirement in the 64 MiB qualification envelope. The Linux CI
link now compresses ELF debug sections. The resulting 26,760,744-byte image
retains line information and leaves 40,348,120 bytes free. The release test
executable occupies 4,352,760 bytes. The release reachability example
(3,323,936 bytes) and its complete output matched the preceding revision
byte for byte. These qualification-only executables have individual size
records. Production memory and binary charges are unchanged.
[Size accounts](docs/research/interval-size-accounts.json)

The local Hyperdynamics workflow matrix ran on Rust 1.95.0 and stable
1.99.0. Each toolchain checked 20 feature combinations and passed 157 tests
in both all-target debug and optimized suites. Formatting, strict Clippy at
both feature boundaries, fuzz build and lint, documentation, inventory tests
and the repository-wide strict zizmor audit passed. Hosted runners have yet
to execute this workflow revision. The analytic oracles qualify the two
specified field families; general nonlinear ODE coverage remains part of
the wider plan. [Raw campaign, mutation and matrix evidence](docs/evidence-archive.tsv)

## Phase 7 and 14 solver certificate fuzzing

Revision `c9a94af00` adds a maintained oracle for the three public Hypersolve
Krawczyk certifiers. Given two original roots, the generator constructs an
affine system, a quadratic in one variable, and a quadratic system in two
variables with an invertible change of coordinates and mixed equation rows.
Integer arithmetic independently gives the original solutions, their closed-box
membership, the Newton step, and the inclusion and contraction bounds. Both
`STRICT` and `APPROXIMATE_512` must return the expected ordered outcome on
these rational inputs. [Native witnesses and fuzz instructions](crates/hypersolve/fuzz/README.md#interval-krawczyk-certificates)

| Qualification | Result |
| --- | --- |
| Ordinary native replay | 2,025 generated cases and seven checked-in inputs exercised 12,192 reports. The grid contained 1,264 boxes with zero roots, 573 with one distinct root and 188 with two. The seeds also exercise the second singular pivot and successful coupled certification with positive unequal radii. |
| Fuzz and cost replay | The final bounded ASan campaign executed 10,000 inputs in 57 seconds from the seven seeds. Its 394 distinct eight-byte survivors included all seven coupled outcomes. Two checked-in seeds were the two most expensive by median on each of two pinned CPUs in the initial 399-input replay. |
| Oracle sensitivity | Three isolated mutations independently failed, covering an incorrect affine root, acceptance of equality at the strict contraction bound, and use of the first width twice in the mixed quadratic remainder. The unmodified control passed. |
| Final images | The retained solver certification and retained-fuzz benchmark executables matched their preceding release builds byte for byte. The new all-feature and minimal release test images occupied 4,413,576 and 4,104,784 bytes. The sanitizer image occupied 29,787,760 bytes, leaving 37,321,104 bytes in its 64 MiB qualification envelope. [Size accounts](docs/research/interval-size-accounts.json) |

The original aggregate retained-fuzz benchmark stopped on the registered
bivariate target. Its dispatch table also omitted the optional Jacobian
target. The repair on `interval-krawczyk-collector-review` (`5b149113d`)
replayed the whole registry and preserved historical promotions when run
without the optional target. Linking all three workloads grew the final
collector from 9,377,104 to 10,474,648 bytes against a 4,096-byte cap.
The primary CI therefore runs the solver-owned oracle through the existing
interval conformance workspace. The aggregate collector gap remains open.

### Reassessment of the affine and univariate proof repair

On the retained revision, four symbolic regressions failed under
`APPROXIMATE_512`. They reproduced false uniqueness, an affine root that
fails its original equations, and rejection of a contractive bound. The full repair on
`interval-krawczyk-proof-review` passed all six symbolic regressions and the
two maintained rational tests. The comparison uses the current primary, which
already requires certified predicates for the multivariate proof.

Across 24 fresh native processes per revision, the repair passed all 120
cold/warm stack rows and all 60 heap rows. The cumulative account carries
forward the previously accepted growth for each workload. Twenty rows used
additional stack, at most 48 bytes. The repaired client's largest measured
callee stack was 12,824 bytes and its smallest measured free worker space
was 2,074,152 bytes. Requested heap peaked at 15,036 bytes and returned to
zero after drop. Each revision also passed 240 ordered bare-WASM calls.
Both revisions' rebuilt native certification, lifetime and WASM files were
byte-identical to their respective archived images. All 66 measured section,
function and final-file rows across these three images passed their
incremental and cumulative caps.

A separate stress run used the original complete symbolic inputs, two stack
watermark patterns and 48 fresh processes per revision. For each previously
incorrect result, the comparison used the same input's completed strict proof
and recorded the prior approximate result separately. All 24 stack and 12 heap
rows passed, including earlier positive charges. The repair used at most
15,192 bytes of callee stack and 15,436 bytes of requested heap. The minimum
worker headroom was 2,071,784 bytes. The stress WASM client checked each
revision's expected statuses in fresh instances.

The runtime gate still blocks the full repair. The one-variable multivariate
quadratic benchmark completes the same exact proof on both revisions. In the
first balanced crossover the repair's median was 0.42% slower on CPU 5 and
0.91% slower on CPU 15. A run on CPUs 3 and 5 overlapped other qualification
work and mixed approximately 4.5 and 9.1 microsecond process medians. An
independent crossover after the owner tests finished reversed the assignment
order on CPUs 5 and 15. The repair was 0.96% and 0.95% slower, respectively.
Each crossover used four processes per revision per CPU and 50 Criterion
samples per process. The production affine and univariate paths still require
`STRICT` for authoritative root certification. Commit `d2a67f1d1` documents the policy
requirement and its observed failure modes at both public entry points.
Rebuilding the two benchmark images after this documentation edit changed the
ELF build ID and symbol string table. Their machine code, section sizes, union
of defined function bytes and final file sizes matched the preceding revision.
The fuzz corpus adds no production memory or binary charge.

On Rust 1.95.0, 1,533 all-feature and 1,396 minimal solver tests passed, as
did the optimized certificate fixtures and 11 affected Hyperdynamics tests.
The retained fixtures also passed on stable 1.99.0. After the public
documentation change, the two native certificate tests and 14 downstream
reachability tests passed. The Rust 1.88 default interval fuzz build, strict
Clippy, rustdoc, formatting, inventory tests and the
repository-wide strict zizmor audit passed. The inventory contains 4,129 scan
sites and 252 review records. Hosted CI and hardware execution were outside
this local run. [Raw campaigns, mutation patches, costs and timing samples](docs/evidence-archive.tsv)

## Phase 6 and 14 rectangular interval replay

Commit `8ad5c4c02` adds the vector and matrix shape fuzzer required by §14.3.
The existing point-matrix fuzzer uses square 3D and 4D inputs. The new
target consumes signed quarter-grid interval matrices through the public
Serde decoder. Its dimensions are 0×0, 0×3, 3×0, 2×3, 3×2, 1×33 and 33×1.
It checks row and column indices on empty and nonempty operands, transpose,
the product with a vector, and multiplication by the transpose after the
first output has been serialized and reloaded.
[Owner test and fuzz commands](crates/hyperlattice-interval/README.md)

The input edits independently specify which wires must fail. They include
wrong lengths, a reversed endpoint pair after an empty entry, and an endpoint
outside the scalar type. Valid zero-dimensional unit products and canonical
empty sets have distinct wire images and multiplication results. For each
nonempty row, four widened corner products per lane give the expected
extrema. The oracle rounds each term outward to the quarter grid and checks
every prefix sum for representability. The five retained inputs include a
sum that overflows before later cancellation, and cases where only the
second multiplication overflows, in a term or in an accumulated sum.

| Qualification | Result |
| --- | --- |
| Native cases | All 6,615 combinations of shape, wire edits, index and endpoint encoding passed, along with the five named fixtures. Endpoint encodings cover small signed values, signed extrema and little-endian `i32` numerators. |
| Final seeded ASan run | 10,000 executions in four seconds. The 338 retained inputs covered all seven shapes and all seven action outcomes, including rejection, empty, two completed products and all four overflow sites. A separate 100,000-execution run passed in 37 seconds. |
| Mutation checks | The unmodified control passed. The native suite failed independently when a unit product was collapsed to empty, transpose reversed its rows, or an overflowing dot-product term was silently replaced by zero. |
| Owner and consumers | Rust 1.88 passed 84 owner tests and doctests with fixed, Serde, dyadic and float-enclosure features. Rust 1.95 passed 98 with all features. The native fixtures passed on stable and in release. All seven Hyperlattice facade tests passed, along with the Cortex-M fixed/Serde check, strict Clippy, formatting, inventory tests and the strict repository-wide zizmor audit. |
| Final images | The native qualification test occupied 1,582,528 bytes. The ASan target occupied 26,556,040 bytes, leaving 40,552,824 bytes in its 64 MiB envelope. The fixed-affine native and WASM example binaries matched their baseline builds byte for byte at 430,920 and 36,996 bytes. [Size accounts](docs/research/interval-size-accounts.json) |

The change adds qualification executables and leaves the production memory
accounts intact. The host runs cover fixed-point Cartesian products. Hardware timing and
the resource behavior of other scalar backends require separate qualification. [Raw campaigns, corpus, mutation patches and image hashes](docs/evidence-archive.tsv)

## Section 17 worked interval examples

CSGRS `df9cb4dde` and Alumina Firmware `96ac22cad` add six worked examples
and execute them in their owner workflows. Each new source is present in the
owner's Cargo package file list. The commands below exercised all nine example
topics on Linux. Run each command from its repository root.

| Plan topic | Command and observed result |
| --- | --- |
| Rational measurement and checked floating-point proposal | `cargo run -p hyperinterval --features rational,float-proposal --example proposal_replay`. Translating 1/2 ± 1/4 by one and back preserves both proposed endpoints. At 2⁵³ the proposal collapses to a singleton. Exact replay from the authored values gives [1/4, 3/4] in both cases. |
| Fixed Q30 enclosure | `cargo run -p hyperinterval --features fixed --example fixed_bounds` checks inclusion, sign, square, midpoint and intersection with empty on [1/2, 1]. |
| Real-to-dyadic conversion | `cargo run -p hyperinterval --features hyperreal --example real_to_dyadic` retries after typed work exhaustion. The 2⁻⁴ grid contains √2 and 181/128. At 2⁻²⁴ the lower endpoint proves strict separation. Both enclosures bracket the positive root by exact squaring. |
| Interval matrix affine image | `cargo run -p hyperlattice-interval --features fixed --example affine_image` retains the shared parameter of (t, 2t+1) for t ∈ [1, 3]. Applying 2x−y yields exactly −1. Applying the same linear form to the Cartesian hull gives [−5, 3]. |
| Hypersolve Krawczyk certificate | `cargo run -p hypersolve --example krawczyk_certificate` uses two retained quadratic rows and strict predicates. The image radii are 5/48 and 7/48, and the contraction bounds are 1/6 and 1/2. Tightening the second radius to 3/32 gives `ImageOutsideBox` on y even though the root remains in the smaller box. |
| Specialized path schedule | `cargo run -p hyperpath --example certified_schedule` plans consecutive straight spans of lengths 8 and 2 from rest to rest. Acceleration 1 permits a forward join feed of 4. Stopping requires 2. Exact replay identifies the second span when the join feed is raised to 4. |
| Validated ODE flow tube | `cargo run -p hyperdynamics --features field --example expression_field` certifies Picard steps and integrates to an exact rational final time. |
| Firmware current calibration | In Alumina Firmware, `cargo run -p alumina-foc --example current_calibration` maps ADC counts 2051 and 2045 to [2048, 7168] and [−7168, −2048] in Q30 numerator units. Count 2057 returns the configured window error. The calibration also passed against the CSGRS revision pinned in the firmware CI workflow. |

The proposal example ran in four x86-64 rounding modes. At the large offset,
rounding up produced [2, 2]. The other three modes produced [0, 0]. Exact
replay rejected all four, and each process preserved its starting mode.
An independent rational calculation checked the positive root, the inverse
Jacobian, both radius choices, the affine images, path limits and current
bounds. The mixed system with an affine equality row reproduced
`NonQuadraticRow`; this multivariate certificate still consumes retained
quadratic packages.

The release examples occupied 460,112 to 3,881,840 bytes on x86-64. Each
passed its individual 64 MiB host qualification envelope with at least 16 MiB
free. Four existing release example binaries from Hyperinterval, the lattice
companion, Hypersolve and Hyperpath matched the preceding revision byte for
byte in a common checkout. The examples add no production memory charge.
[Per-example file sizes and hashes](docs/research/interval-size-accounts.json)

Local checks passed 278 all-feature Hyperinterval tests and doctests, 1,533
all-target/all-feature and 1,396 minimal Hypersolve tests, 443 Hyperpath tests
and doctests, and 49 fixed/Serde lattice companion tests and doctests. The
firmware default-member suite passed 603 tests and doctests. Formatting,
strict Clippy, rustdoc, both repository-wide strict zizmor audits and the
affected Cortex-M library checks passed. Hosted workflows and board timing
remain outside these local results.

### Locked native and WebAssembly FOC conformance

Both documented locked builds initially failed before compilation. The
native and bare-WASM lockfiles omitted the dependency of `alumina-foc` on
`hyperinterval`. Revision `b56d2e770` restores that edge in each lock.
The set of packages and versions stayed identical. All seven checked-in
locks containing `alumina-foc` across CSGRS, Alumina Firmware and Alumina
Interface record the edge.

The standalone native release tests, allocation check, Clippy, cost client
and Cortex-M library build passed. A maintained Node runner executed the
linked import-free WASM client. Each of two fresh instances checked 825
transform cases and 147 range cases, including overflow in a checked
intermediate. The cardinal Park digest oracle detected selection of the
wrong sample even when both transform paths used it. The range oracle
detected conversion of a genuine overflow to success. These two deliberate
mutations failed independently.

The two native probes and the WASM client matched controls built from the
previous source with the two missing lock edges restored locally. The
unmodified previous locks had no successful linked result. Timing from the
cost probe documents execution only. [FOC commands](crates/hyperlattice-interval/conformance/alumina-foc/README.md) · [Raw observations and reproduction material](docs/evidence-archive.tsv)

## Phase 14 locked sibling sources and linked consumers

Alumina Interface `fdb7b153` pins CSGRS `c1a5fdd17` and Alumina Firmware
`96ac22cad` in its deployment workflow. The earlier audit accepted a second
checkout with the same firmware package names and versions. It also returned
success after rewriting an Interface lockfile that omitted a dependency.
The revised audit checks the resolved Cargo manifest, source and version of
each owned package, verifies that required packages are reachable from the
consumer, and uses locked resolution throughout. The Interface native and
browser license checks remain active.

A clean local replay used only the three named checkouts. All nine target
graphs across seven lockfiles passed. The alternate checkout and missing
edges in the Interface, native FOC and WASM FOC lockfiles each failed.
Every stale lockfile remained byte-identical across the failed audit. Both
GPL-family and absent license declarations still failed their controls.

The two standalone native conformance suites passed 12 tests and doctests.
The FOC library passed its Cortex-M check. Two fresh WASM instances each ran
825 transform cases and 147 range cases. The measurement runner independently
checked 4,369 inputs per instance, including signed mixed fractions,
64-bit extremes, negative uncertainty and a zero denominator. The archived
Python Fraction oracle agreed on all inputs and recorded zero host calls.
Independently replacing the upper endpoint with the lower one or substituting
an unexpected-error sentinel for the negative-uncertainty result failed the
linked runner. Full-value native tests accompany the WASM digest comparisons.

All 400 Interface workspace tests and doctests, both strict Clippy runs,
formatting, the strict local zizmor audit and the release browser build passed.
The optimized browser WASM validated at 10,660,425 bytes, and all workflow
compression checks passed. This change affects verification and adds no
production memory charge. The revisions are committed locally. A hosted run
requires the pinned sibling commits on GitHub. Board timing and physical
hardware execution remain open. [Reproduction material and artifact hashes](docs/evidence-archive.tsv)

## Phase 7 finite-difference Jacobian memory reassessment

The public Jacobian still returns −5 for a forward difference of `x²` with
model value 1, context binding 3 and step 1. The expected value is 7.
Primary `3cd22cc86` documents the defect and how to supply the intended
center. Its new public corpus checks 392 matrices on supported inputs,
including independent fixed-variable bindings, and checks the typed
unbound-symbol error. The 2,352 entries are compared bitwise, including
positive and negative zero. Three deliberate changes each failed the corpus.

The bounded-memory review ran 8,064 fresh resource processes on the fixed
baseline, preceding revision and twelve candidate implementations. The
borrowed-center repair at `9268e4371` corrects the bad derivative with zero
incremental or cumulative measured stack growth across 48 workload groups.
Peak requested heap, allocation events and cleanup also matched. The linked
native and WASM public clients passed their size caps, and the WASM repair
passed 1,568 matrix entries in each of two fresh instances. Two independent
gates keep the change on `interval-jacobian-borrowed-context-review`. The
default measurement executable added 840 bytes of allocated RELRO padding;
the trace-enabled weighted case was 4.22–5.15% slower across three cores in
the reversed-order confirmation. No production memory allowance was charged.
[Per-candidate results and scope](docs/research/solver-jacobian-context-review.md)

Rust 1.95 passed 1,535 all-target/all-feature owner tests, the all-feature
doctest, and 1,398 minimal tests and doctests. All 13 downstream reachability
and policy tests passed, as did strict Clippy and rustdoc, formatting,
inventory tests, strict zizmor, and the new fixture on stable and in release.
The new release test occupies 3,857,784 bytes. The primary and preceding
standalone native clients matched in size and machine code. Their WASM clients
matched in size and code-section bytes; data bytes differed. The refreshed
inventory has 4,129 scan rows and 257 review records.
[Qualification artifacts and commands](docs/evidence-archive.tsv)

## Phase 11 lookahead schedule equivalence and memory reassessment

Primary `d8eb6fa0c` adds a public Hyperpath regression and fuzz target.
On connected line routes it computes each node's squared speed independently
from every local limit and its distance to that node. The corpus covers
both traversal directions, caller caps, reversals, radius-limited corners
and edited schedules. It checks exact residuals and proof-status replay.
All three deliberate faults failed the deterministic tests. Both sanitizer
builds completed 10,000 fuzz executions. The fixed matrix has 256 cases
and six additional retained seeds. Curved and jerk-limited paths have
separate qualification requirements.

The rebased full extraction reproduced all 26,406 ordered certificate rows.
In 144 fresh native processes its requested heap matched at each measured
stage, while whole-call stack grew 320 bytes (2.62%) in every group. This
exceeds the 128-byte and 1% limits. The retained change adds test coverage
with zero production memory charge.
[Reassessment and measured headroom](docs/research/path-dynamics-current-admission.md#lookahead-replay-and-whole-call-stack-reassessment)

Rust 1.95 passed 446 all-target/all-feature owner tests and 445 minimal
tests. The new suite passed on stable and in release. Strict Clippy,
rustdoc, formatting, standalone fuzz checks and all five inventory tests
passed. The linked native and WASM example clients matched their preceding
images byte-for-byte. The native release test occupies 4,785,624 bytes.
The default ASan image exceeded the separate 64 MiB slot at 67,733,848 bytes.
Lossless linker compression produced a 28,894,000-byte image, retaining
identical machine code, sanitizer coverage sections and all decompressed debug
sections. The release test and compressed ASan image meet the required
16 MiB headroom in their respective qualification slots.
[Artifact accounts](docs/research/interval-size-accounts.json)

## Phase 3 finite dyadic scalar maps

Primary `f10679ece` adds exact scalar multiplication and affine images with
caller-budgeted translations and signed exponent repair. The ordinary signed
scale fixture completes with two products in 50 work units. Adding its offset
takes ten more. Both operations preserve the original interval on failure
and support a fresh-budget retry.

All 270 all-target/all-feature Hyperinterval tests, 15 doctests, 98 downstream
Hyperlattice interval tests/doctests and nine Hypersolve expression tests
passed. The two existing native/WASM clients and the linked native solver
example matched their preceding binaries byte-for-byte. Standalone Rust 1.88
and 1.95 clients passed the 32,768-case oracle on native and WASM. The
direct native example reduced the maximum measured whole-call stack by 672
bytes and peak requested heap by 192 bytes. The size ledger preserves the
two positive section charges. No production memory allowance was consumed.
[Resource limits, rejected implementations and completed workloads](docs/research/dyadic-scalar-affine.md)


## Hyperinterval native coverage and arctangent refinement

Primary `d8760fc6b` adds the §14.5 coverage command. It runs the all-feature
library and integration suites, checks the LLVM file list against the crate
source tree and emits annotated source for incomplete metrics. Each measured
run merged 151 profiles from 48 native test executables. The production
denominator contains 29 code-bearing files. Two test-only modules are
excluded and the declarations in `lib.rs` have no coverage records.

| Toolchain and metric | Preceding revision | With new witnesses |
| --- | ---: | ---: |
| Rust 1.95 lines | 4,902 / 4,948 | 4,905 / 4,948 |
| Rust 1.95 regions | 9,821 / 10,370 | 9,828 / 10,370 |
| Nightly lines | 4,886 / 4,926 | 4,889 / 4,926 |
| Nightly regions | 6,204 / 6,747 | 6,209 / 6,747 |
| Nightly branches | 980 / 1,004 | 981 / 1,004 |

Every grouped production function executed on both compilers (512 on 1.95,
518 on nightly). Rust 1.95 supplied no branch counters. The nightly measurement
used `8006091eb` and `-Zcoverage-options=branch`. Each compiler retained
its own denominators. The 100% gate still fails on both. On nightly, seven
of the 23 uncovered branches are in the IEEE backend.

At spacings `2^-189` and `2^-645`, the unbounded arctangent limit needs
another precision pass. An entire source and both one-sided rays retain
their tight endpoints against a directed MPFR oracle. A budget one unit
short returns work exhaustion and a fresh exact-debit retry completes.
Stopping at the first enclosure passed the original tests and the
`2^-189` case alone. At `2^-645`, the resulting outer endpoint is one
output-grid unit too wide. The empty-difference test detected a second
mutation on the integer storage paths. Production source is unchanged.

Rust 1.95 passed 270 all-target/all-feature tests, 15 doctests, five minimal
tests and 98 downstream Hyperlattice interval tests/doctests. Strict Clippy,
strict rustdoc, formatting and example/bench checks passed. Rust 1.88 passed
the seven affected tests enabled by the integer and dyadic features.
Against the preceding revision, the release arctangent test executable grew
928 bytes and the boundary-composition executable grew 2,696 bytes. Both
passed the fixed-baseline and incremental file and code-size limits.
[Size accounts](docs/research/interval-size-accounts.json) · [Coverage reports, controls and commands](docs/evidence-archive.tsv)

## Strict IEEE guest-stack measurements and WASI qualification attempt

Revision `11d365f66` adds a guest-stack observation to the existing bare-WASM
conformance run. For each of Rust 1.88.0, 1.95.0 and 1.99.0, two fresh instances
each completed 55,802 arithmetic cases and 8,836 property cases twice. Both
sentinels showed writes reaching 704 bytes below the initial stack pointer in
cold and warm runs. The instance had 1,114,112 bytes of fixed linear memory
and a 1,048,576-byte linker stack. This measures bytes touched during the
completed corpus. Unwritten stack reservations and engine or host memory are
outside the observation. Three malformed disassembly controls stopped the probe.

The existing WebAssembly checks also passed 612 floating and 612 fixed lattice
chains per compiler and 56 dyadic conversion cases. Each compiler meeting the
Real bridge's Rust 1.95 minimum completed four cancellation/retry calls across
two instances. Chromium executed the bare primitive corpus. Rust 1.95 passed
270 all-target/all-feature Hyperinterval tests, 15 doctests and 98 downstream
Hyperlattice interval tests/doctests. Strict Clippy, strict rustdoc, formatting
and the strict workflow audit passed. All four retained Rust 1.95 bare-WASM
modules match the preceding checkout byte for byte.

### Preview 1 candidate

On `101a905b4` a valid finite singleton returned `UnqualifiedEnvironment` for
both widths on WASI Preview 1. A first guard using architecture, OS and ABI
also admitted the threaded target on Rust 1.95. Its real module imports
shared memory. A negative execution test caught that guard. An exact Cargo
target-name guard passed on ordinary Preview 1 and retained typed refusal on
the threaded ABI and `wasm32v1-none`. On ordinary Preview 1 all three compilers
passed the arithmetic, property, lattice and dyadic suites. The primitive
module required no imports, allocation, memory growth or floating arithmetic.
The two newer compilers also passed the Real bridge. The 1.95 ordinary
Preview 1 client touched 704 guest-stack bytes. Its previous refusal supplies
no comparable successful-workload resource baseline.

The experiment declared zero growth for the existing linked qualification
artifacts. Each column below compares a Rust 1.95 release pair built in one
physical checkout path, with the same compiler and lockfiles.

| Bare module | Primary checkout file delta | Detached checkout file delta | Code-section size delta in both |
| --- | ---: | ---: | ---: |
| Primitive float | −4 B | +4 B | 0 B |
| Lattice | −3 B | +2 B | 0 B |
| Dyadic | +1 B | 0 B | 0 B |
| Real bridge | +1 B | −1 B | 0 B |

Only the function-name section changed length. Ordering and other section
contents could differ at equal length. The native float example measured
502,600 bytes on both revisions. The positive file deltas exceeded this
experiment's raw-file limit. The exact-target candidate and both sets of
measurements are archived. The retained CI check executes the current typed
refusal, canonical empty and endpoint traps on both Preview 1 ABIs.
No production binary or memory exception was charged by this experiment.
The partial native coverage results above and the remaining interval-plan
phases are still open.
[Commands, proposal patch, linked section hashes and individual memory observations](docs/evidence-archive.tsv)

## Direct binary32 scalar ingress

Revision `551ea7a70` adds `DyadicInterval::from_f32` for the Phase 3/4 exact
conversion boundary. Both float features use the same integer validation,
decoding and budget reservation as existing dyadic ingress. Finite inputs
produce exact singletons, including subnormals and canonical zero. NaNs and
infinities fail before consuming work. A normal binary32 value needs 24
significand bits and 48 work units. Widening `1.0_f32` and importing it through
the existing binary64 constructor required 53 bits and 106 units.

The maintained tests check independent rational values across all finite
exponent classes, both signs, and every subnormal leading-bit position. They
also check exact reservations, separate size/work errors, retry and exact
addition of the returned point. Widening before import, bypassing finite
validation and dropping the sign each failed a regression test. An early test
underestimated the separate budget for exporting the result to rational form.
The final test derives that allowance from the rational egress cost and keeps
the binary32 import limit at 24 bits and 48 units.

A separate Python oracle supplied 35,370 IEEE bit patterns. The native
release client checked each materialized mantissa, exponent or typed failure
in all 16 combinations of rounding mode, FTZ and DAZ on this x86_64 host.
Bare WebAssembly and WASI Preview 1 each executed the same corpus in two
fresh instances. Direct scalar ingestion works on Preview 1 independently of
the strict arithmetic target gate. Both direct WASM modules on both locks
contained no floating arithmetic or conversions. The widening controls used
`f64.promote_f32`.

| Rust 1.95 linked client | Workspace lock, widening → direct | Standalone lock, widening → direct |
| --- | ---: | ---: |
| Native shared library | 468,040 → 468,024 B | 493,584 → 493,560 B |
| Bare WebAssembly | 80,691 → 80,639 B | 99,805 → 99,785 B |
| WASI Preview 1 | 104,438 → 104,386 B | 123,568 → 123,548 B |

On the native workspace lock, both paths peaked at 40 requested heap bytes,
64 allocator-usable bytes and 848 bytes of touched stack. On the standalone
lock both peaked at zero requested heap bytes and 896 bytes of touched stack.
All calls released their allocations. Each lock kept the same stack and heap
peaks for the direct size-denial, work-denial and successful-retry sequence.
Bare-WASM guest-stack measurements were 284 bytes on the workspace lock and
320 bytes on the standalone lock for direct import, compared with 284 and
336 bytes for widening. The standalone bare direct client also avoided one
64 KiB linear-memory growth during the corpus. Guest-stack observations
exclude host memory and untouched frame reservations.

In 48 alternating native release pairs, median direct/widening time ratios
were 0.848 on the workspace lock and 0.805 on the standalone lock, for batches
of 150,000 source values with materialized outputs. The inline prototype grew
the standalone native file by 1,272 bytes, exceeding its 512-byte allowance.
Keeping the same ordinary Rust call boundary as binary64 produced the retained
sizes. The four existing bare-WASM modules and the native strict-float example
were byte-identical to their preceding builds. The size ledger preserves the
earlier eight-byte standalone native file charge. This change consumed no
production memory exception. [Per-target accounts](docs/research/interval-size-accounts.json)

Rust 1.95 passed 272 Hyperinterval all-target/all-feature tests, 15 doctests,
98 downstream Hyperlattice tests/doctests and eight Hyperdynamics RK proposal
tests. Both minimal feature combinations passed on Rust 1.88 and built for
Thumb. Bare WASM and Preview 1 also executed direct import on Rust 1.88.
Strict Clippy, rustdoc and formatting passed. A single-package offline build
could not resolve Hyperreal 0.14 from the cached public index. Cargo then
packaged and verified Hyperreal and Hyperinterval together. A clean external
consumer of the unpacked Hyperinterval package passed the native and bare-WASM
oracle on Rust 1.88. The full interval plan remains active.
[Reproduction scripts, oracle and resource measurements](docs/evidence-archive.tsv)

## Three-variable affine certificate replay and bounded stack review

On `07825c92f`, the Phase 7 affine certificate oracle covered coupled
two-variable systems. Commit `b815965eb` adds 48 reports for three coupled
equations under both predicate policies. Each report must return the
prescribed symbols and coordinates. The test also substitutes those values
into all three original expressions and requires exact zero. The corpus uses
both signs of √2 and 2⁻²⁰⁴⁸ and all six row permutations. A mutation that
replaced only the third reported root with its candidate coordinate passed
the two-variable and fuzz-seed suites and failed the new test.
[Independent residual replay](crates/hypersolve/tests/krawczyk_affine_residuals.rs)

The public affine and univariate quadratic Krawczyk APIs still require
`PredicatePolicy::STRICT` for proof. Four archived assertions failed on this
revision under `APPROXIMATE_512`, including the original-equation check on
a reported affine root. Three complete repairs passed all four witnesses.
The policy-only and affine-only candidates also passed the ordinary and
stress native memory gates. The two balanced, opposing-order timing
campaigns reproduced a slowdown on the already certified one-dimensional
multivariate workload. Across 48 fresh stress processes per revision, each
policy-only candidate peaked at 15,436 requested heap bytes and 15,192 callee
stack bytes, within separate 16 KiB limits. At least 2,071,784 bytes remained
on its guarded 2 MiB native worker against 1 MiB of required headroom.

| Candidate | Observed result on the current revision |
| --- | --- |
| Certified-only at both entries | Passed 66 linked-image checks. The four pinned-core timing comparisons rose 0.46%, 0.70%, 1.51% and 0.57%. |
| Certified-only at the affine entry | Corrected the original-equation and affine elimination cases. All 30 ordinary Krawczyk reports and traces and all 84 box replay records matched. Timings rose 0.42%, 1.13%, 0.47% and 0.97%. The univariate failures remained reproducible. |
| Move the normalized pivot row during elimination | Passed all ordinary and stress stack and requested-heap comparisons. Native RELRO padding grew 2,824 bytes in the benchmark and 288 in the lifetime observer, exceeding their section caps. |
| Reuse the solved right-hand side and inverse row storage | Six ordinary and eight stress workloads grew in peak requested heap, by up to 144 bytes. The linked WASM file grew 6,536 bytes and its code section grew 5,746. |

The separate owned-radius, final-row box replay had previously failed the
stack gate. Its original patch applied to the current source. It passed nine
previously failing assertions for row certainty and proof scope and changed
24 of the 84 box verdicts. All 84 underlying evidence/dispatch records and
all 84 allocation records matched. Across 24 processes per revision, all
60 peak requested heap measurements passed. Of 120 cold/warm stack
comparisons, 24 exceeded the incremental cap and 16 exceeded the cumulative
relative cap. One affine call rose from 3,302 to 3,350 bytes, or 1.45%
against its 1% incremental limit. Constructing the final report directly
also exceeded 24 stack caps. Each variant passed 240 linked WASM calls.
The raw observations retain each workload's baseline and previous charges.

Only the residual replay test was admitted. Its Rust 1.95 release executable
uses 4,365,824 bytes of its 64 MiB host qualification slot, leaving
62,743,040 bytes free. Default, all-feature and release executions passed,
as did targeted all-feature Clippy with warnings denied, workspace formatting
and the repository budget check. No production memory or flash charge was
added. A final shared-target run reused the mutant executable. Its digest
matched a fresh isolated mutant build. Independent builds passed all four
retained tests, reproduced the intended mutation failure and matched the
release image byte for byte. Rebuilding Hypersolve in the shared target
also restored all four passes. Evidence commits `d56504756` and
`73fedff86` preserve the measurements and isolated build logs.
[Per-target accounts](docs/research/interval-size-accounts.json) ·
[Raw observations and candidate patches](docs/evidence-archive.tsv)

## Binary32 proposal half-width and exact subnormal RK4 replay

Commit `9ce30c35c` adds `F32ProposalInterval::radius_estimate` to the Phase 4
primitive proposal layer. It takes the native midpoint of the negated lower
endpoint and the upper endpoint. With gradual underflow and round-to-nearest,
the half-width of an interval spanning one to three minimum subnormal units is
one unit. Separately halving the endpoints gave two. The half-width of the full
finite binary32 range is representable, and an empty interval returns `None`.
The result carries proposal certainty. [Public implementation](crates/hyperinterval/src/proposal.rs)

An independent integer oracle checked 89,008 ordered inputs spanning all
finite binary32 exponent classes. The previous formula disagreed on 4,760.
The retained operation matched every expected half-width on x86-64 and in two
fresh bare-WASM instances. A mutation restoring the earlier formula failed
the maintained test on the interval from −31 to −30 minimum subnormal units.
[Regression corpus](crates/hyperinterval/tests/proposal.rs)

| Linked half-width client | Preceding | Retained | Cumulative charge | Free capacity |
| --- | ---: | ---: | ---: | ---: |
| Native file | 402,008 B | 402,024 B | 16 B | 66,706,840 B |
| Bare-WASM file | 1,034 B | 1,035 B | 1 B | 64,501 B |

Native code grew by 16 bytes and WASM code by one. The fixed and preceding
half-width controls had the same sizes. The cold and warm native calls took
120 bytes of stack for ordinary, subnormal and near-maximum intervals across
both watermark patterns. No heap allocation occurred. Free worker space was
2,088,520 bytes. All 24 measured ordinary/stress RK4 heap and stack rows also
passed, including exhaustion, retry and cleanup. No production memory or
firmware flash allowance was charged. In two quiet timing campaigns on four
pinned cores, constructing the interval and returning its binary32 half-width
took about 24% less time on the ordinary inputs. An earlier run overlapping
compilation varied between −23.96% and +0.17%. The measured existing native
text, read-only data and initialized data sections matched the preceding build.
[Per-target charges, caps and remaining allowance](docs/research/interval-size-accounts.json)

Commit `c6d582c49` exercises the Phase 12 proof boundary. Three subnormal
initial sets pass through a stationary flow and then a translating flow. The
test checks each exact endpoint against its dyadic closed interval. Replacing
the original initial box with the numerical candidate gave [−4, −3] units
where the first stationary endpoint should have been the singleton −3.
The two release test executables have separate cumulative file charges of
2,504 and 7,856 bytes, each within its 32 KiB and 2% limits.
[Exact replay regression](crates/hyperdynamics/tests/rk_proposal.rs)

The wider midpoint and RK4 repairs remain under review. Commit `90cc4e789`
matched all 872,128 midpoint and half-width expectations on each runtime and
all 35 ordinary RK4 records. It met the complete-lifetime memory caps. Its
time-affine budget-one rejection slowed 6.6–6.7% on four cores. The separate
joint binary32 operation in `ac68fef5a` improved the single-field timing but
slowed the caller materializing both results. Commit `8e18908dd` withdrew it.
A width-specific constructor recovered the packed-result time and passed the
linked file and code caps, but repeated exact replay timings increased. The
existing midpoint methods and RK4 numerical stages can still misround
subnormal inputs. The retained RK4 regression qualifies original-source
proof replay for the tested flows.

The retained code passed both owning crates' all-feature tests and Clippy,
Hyperinterval documentation and optimized floating-environment tests,
Rust 1.88 proposal tests and the no-default-feature Thumb build. The strict
WASM suite passed 55,802 cases, and native code inspection passed for 70
strict symbols. Workspace formatting and the repository budget passed.
[Commands, probes, raw observations and review patches](docs/evidence-archive.tsv)

## ABI-qualified functions in the Phase 0 inventory

A fresh three-repository scan on `25a4597e7` omitted all seven public entry
points in Hyperlattice's fixed-point WASM consumer and the firmware comparison
client's `range_probe`. The scanner now recognizes Rust function declarations
with an explicit or implicit foreign ABI. The regression exercises visibility,
unsafe functions, a declaration split across lines, an extern block and an
extern crate. It failed on the preceding scanner. All six inventory tests pass.
Two full refreshes produced byte-identical output containing 4,141 lexical
candidates and all 259 prior review records in their original order. Eight
candidates are attributable to the scanner change. The refresh also includes
two previously unrecorded declarations from the current source tree.
[Scanner and regression tests](scripts/test-interval-inventory.py)

## Fixed-point Cartesian WASM replay in coordinated validation

Commit `700888155` wires the existing fixed-point Cartesian guest into
the coordinated WASM runner. The seven exports exercise affine images and
source correlation, chains through depth 32, matrix norms and measures,
full-width intervals, midpoint/radius reconstruction and contraction margins.
The earlier Wasmtime qualification and its per-call fuel observations remain
in the evidence archive. The maintained Node runner now checks the returned
values in forward and reverse input order, using two fresh module instances.
It follows each of the four source corners step by step to obtain the
affine-chain expectation. A mutation that preserved every guest bound check
and changed only the returned depth-32 value produced −3091 where the host
expected −3092. [Executable oracle](crates/hyperlattice-interval/conformance/fixed-wasm/check.mjs)

The Rust 1.88.0 and 1.95.0 runs each passed 172,284 fixed-lattice calls and
the 55,802 existing strict scalar cases. The fixed guest occupied 62,720 and
62,676 linked bytes respectively. Both guests had zero imports, no floating
arithmetic or memory-growth instruction, and a constant 1,114,112-byte
linear memory in each instance. These are bare-WASM validation measurements.
The four previously measured WASM images on Rust 1.95.0 matched their preceding
SHA-256 hashes. The separate original-source Real bridge ran on 1.95.0, its
declared minimum. The owner crate's all-feature, all-target tests and Clippy,
workspace and standalone formatting, and the repository budget passed.
[Coordinated runner](scripts/check_interval_wasm.py)

The sibling firmware comparison also passed 825 transform cases and 147
range cases in each of two Rust 1.88.0 WASM instances. The inventory now
contains body reviews for the seven Cartesian probes and both firmware
comparison exports. Naming the additional firmware entry in a review also
allows the scanner to include it on subsequent refreshes. A clean refresh
reproduced all 4,142 current lexical records and all 268 historical review
records byte for byte. The interval plan remains active.
[Inventory](interval-inventory.tsv) · [Evidence archive](docs/evidence-archive.tsv)

## Independent replay of Alumina firmware transform endpoints

Commit `7084c9f0d` extends the Phase 9/14 linked comparison to all 825
Q30 transform outputs, including the noncardinal rotation and the complete
chain. Each of two fresh Rust 1.88.0 WASM instances returned the expected
type tag, signed lane endpoints and digest, and matched 251 independent range
expectations (79 range errors). Three separately compiled mutations passed
the preceding checker. The new checker identifies the wrong chain digest,
the one-unit error in the third inverse-Clarke lane and the suppressed Park
overflow. The sibling validation also passed the native no-allocation and
transform tests, the Thumb library check and 4,369 Interface measurement
cases in each of two fresh WASM instances. All nine coordinated package
graph audits passed. [FOC conformance client](crates/hyperlattice-interval/conformance/alumina-foc/README.md)

The first implementation added 32 bytes of guest stack to the existing
digest calls. The retained observer reads the firmware endpoints separately.
Across the cold and warm corpus, the existing transform calls used 400–624
bytes of guest stack and the range calls used 480 bytes, matching both
controls. Reading all four output words took 304–368 bytes, with at least
1,047,952 bytes of guest stack free across the measured workloads. All 75
previous WASM function bodies matched after normalizing two disassembler
alias labels. All 1,076 previous output and Wasmtime fuel pairs also matched
individually. Paired host timings varied by core, including a 4.59% Park
increase on one core and 0.13% on the other. Hardware cycle counts remain
unmeasured. The three native release comparison clients retained their file
lengths and identical text, read-only data and data sections.

The linked WASM fixture grew by 311 bytes to 35,858 bytes, including 276
additional code-section bytes. For each metric both incremental limits
(1,024 bytes and 5%) and both cumulative limits (4,096 bytes and 10%) passed.
The 128 KiB host fixture slot has 95,214 bytes free against its 32 KiB
reserve. The fixed historical standalone lock omitted the local firmware
dependency on hyperinterval and refused a locked build. Rebuilding the
historical and preceding sources with the same corrected lock produced
byte-identical modules. This entry charges only the test fixture's binary
size. Firmware deployment, physical hardware validation and hosted CI
remain open. [Per-target account](docs/research/interval-size-accounts.json)

Three current ABI reviews bring the inventory to 4,143 scan rows and 271
preserved reviews. A fresh scan reproduced it byte for byte and all six
scanner tests passed. Formatting, standalone Clippy and the repository
budget passed locally. Evidence commit `8852dec62` contains the two controls,
candidate measurements, original lock failure, mutations and comparison
scripts under `docs/evidence/2026-11-18-interval-foc-endpoints`.
[Inventory](interval-inventory.tsv) · [Evidence index](docs/evidence-archive.tsv)

## Hyperdynamics native coverage and domain graph-budget replay

Commit `fd4da5bb3` adds Hyperdynamics to the native coverage command for
§14.5. On the preceding revision, passing `--package hyperdynamics` to
the scalar collector still ran 273 Hyperinterval tests. The shared command
now selects the Cargo package and checks its own source inventory. Running
the original Hyperinterval entry point reproduced every previous per-file
coverage count on Rust 1.95. [Coverage command](scripts/interval_coverage.py)

The owner suite previously executed zero of the five functions on the
borrowed graph-budget adapter. The new test checks a precharged counter
through tree compilation, cancelled validation and exact-budget replay. It
also checks that a sum of structural entries exceeding `usize::MAX` is
rejected. The physical consumer uses a seven-binding zero-force graph and
1,024 distinct source references. At a limit of 1,049 entries construction
refuses before spending graph or scalar work. At exactly 1,050 it retains
every ordered reference and reproduces the zero vector field at rest.
Checking each count independently let the damaged implementation construct
the over-limit model. Both original test targets passed this mutation and
both new tests detected it. A separate wrapping-add mutation failed at the
owner's machine-word limit.
[Owner witness](crates/hyperdynamics/tests/expression_field.rs) ·
[Physical consumer](crates/hyperphysics/tests/certified_flow.rs)

| Hyperdynamics production metric | Before | After |
| --- | ---: | ---: |
| Rust 1.95 lines | 4,222 / 4,581 | 4,246 / 4,581 |
| Rust 1.95 regions | 6,086 / 6,893 | 6,110 / 6,893 |
| Rust 1.95 grouped functions | 466 / 530 | 471 / 530 |
| Nightly lines | 4,190 / 4,526 | 4,214 / 4,526 |
| Nightly regions | 3,804 / 4,569 | 3,822 / 4,569 |
| Nightly grouped functions | 474 / 538 | 479 / 538 |
| Nightly branches | 224 / 264 | 224 / 264 |

Each compiler measured 19 production files and merged 53 profiles from 20
test executables. The declaration-only crate root had no mapped code.
Rust 1.95 supplied no branch counters. All 159 Hyperdynamics tests passed
in each final run, and the 100% coverage gate still fails on both compilers.
The two-crate all-feature, all-target run passed 243 tests, and the minimal
run passed 72. All-target Clippy with warnings denied, documentation,
workspace formatting and the repository budget passed.

The release test executables grew by 7,768 bytes for Hyperdynamics and
15,424 bytes for Hyperphysics. The corresponding code-section increases
were 3,856 and 8,656 bytes. Both fit the predeclared 16 KiB and 1% per-target
incremental caps and 32 KiB and 2% cumulative caps. Including preceding
growth, the file charges are 7,800 and 15,504 bytes. Each 64 MiB host slot
retains over 59 MiB free against a 16 MiB reserve. This qualification
changed test and tooling code only. Evidence commit `f61f0aad9` contains the
full per-source reports, annotated gaps, linked-image measurements, mutation
controls and validation logs. [Size accounts](docs/research/interval-size-accounts.json) ·
[Evidence index](docs/evidence-archive.tsv)

## Phase 9 Cortex-M4 execution of the firmware transform corpus

Commit `7dc70b88c` links the existing Rust 1.88 Thumb probes as an ARM
hard-float ELF and executes their C ABI in Unicorn's Cortex-M4 model. On
`7a65d7a07` the maintained Thumb step compiled the conformance library.
The new command first checks two fresh WASM instances against the integer
oracle, then replays the checked calls on Thumb. Every pass includes the
825 full transform digests, 3,315 output-word reads and 251 range outcomes.
Forward and reverse orders, two stack fill patterns, and cold and warm calls
produced 35,128 successful Thumb calls. The runner checks the return PC,
callee-saved registers, restored SP and returned bits on every call.
[Simulator command](crates/hyperlattice-interval/conformance/alumina-foc/README.md)

Three ARM-only negative controls passed the WASM run. The simulator detected
a one-unit error in the inverse-Clarke third lane, a call that exhausted its
one-million-instruction allowance and a frame that wrote into the unmapped
stack reserve. An initial loop using a processor hint encountered an unsupported
simulator instruction. An ordinary branch loop independently exercised the
instruction limit.

The primary checkout's ELF occupied 23,860 bytes in the 64 KiB simulator
image slot and left 41,676 bytes free against a 16 KiB reserve. The entire
corpus touched at most 704 bytes of its 8 KiB stack. A separate hook sampled
SP at every simulated instruction and confirmed the same peak, leaving
7,488 bytes free. All four runs of the original 1,076 digest/range calls
matched the fixed and preceding controls individually in return value,
stack depth and instruction count. The longest call used 6,429 guest
instructions. The historical lock needed its missing local dependency
edge restored. With the same source path and corrected lock, all three
two-export ELFs had equal file, load and code sizes. The preceding and
retained three-export ELFs were byte-identical. Their longer embedded paths
added 40 bytes of read-only data relative to the primary checkout.

An isolated build read the same endpoints from the shared interval
implementation. For each transform, the following totals cover 165 samples
and four ABI reads per sample. Each read recomputes the transform. Both
observers returned the same complete corpus.

| Transform | Firmware instructions | Shared instructions | Firmware peak stack | Shared peak stack |
| --- | ---: | ---: | ---: | ---: |
| Clarke | 481,270 | 819,350 | 488 B | 640 B |
| Park | 532,394 | 602,354 | 448 B | 604 B |
| Inverse Park | 532,522 | 603,142 | 448 B | 604 B |
| Inverse Clarke | 287,830 | 284,530 | 368 B | 352 B |
| Complete chain | 1,599,758 | 2,413,198 | 488 B | 640 B |

The complete shared chain used 50.8% more guest instructions and 152 more
stack bytes in this fixture. Inverse Clarke used fewer of both. The firmware
implementation stays in place. The comparison supplies simulated instruction
counts. Physical board cycles, interrupt-inclusive stack, deployed flash
headroom and HIL qualification remain open.

The coordinated firmware and Interface suites, nine sibling-source graph
audits, standalone Clippy with warnings denied, formatting and repository
budget passed locally. No production size or memory allowance was charged.
Evidence commit `1bf538726` contains the linked images, checked call lists,
per-instruction measurement code, comparison records and mutations.
[Evidence index](docs/evidence-archive.tsv)

## Phase 9 inverse Clarke through space-vector modulation

Firmware commit `35d1b0da6` adds a public Q2.30 regression test for inverse
Clarke, common-mode injection, the three centered phases, all duty endpoints
and the order of range failures. Rounding a duty upper bound down by one bit
passed the preceding 87 unit tests and both integration tests. The retained
test caught 751,619,276 where the exact ceiling is 751,619,277. Its signed
extreme also distinguishes an unrepresentable sum before halving from an
invalid modulation limit. The owner is `alumina-foc`. Hyperinterval supplies
the outward-rounded scalar operations.

An independent integer oracle checked the public calls on all 30,492 products
of two signed interval inputs and seven voltage limits. The fixed-point grid
has 11 selected endpoint values, including both signed extremes. All six
arithmetic implementations and preceding firmware produced identical endpoints
and typed errors. There were 1,341 successful modulations, 16,900 arithmetic
range errors and 12,251 modulation-range errors. At the inverse step, outward
multiplication by positive one-half followed by negation equals outward
division by positive two followed by negation on the entire signed grid.
Halving makes both endpoints safely negatable. The combined candidate keeps
the intermediate checked sum before applying this identity to common mode.

| Firmware expression | Native probe .text | Transform WASM file | Transform Thumb .text | Thumb inverse instructions, 660 word reads |
| --- | ---: | ---: | ---: | ---: |
| Preceding | 260,092 B | 35,862 B | 20,080 B | 287,830 |
| Divide by −2 | 260,076 B | 35,867 B | 20,028 B | 282,230 |
| Multiply by −½ | 260,108 B | 35,908 B | 20,108 B | 299,710 |
| Affine map with slope −½ | 260,108 B | 35,908 B | 20,108 B | 299,710 |
| Chain fixed multiply and negate | 260,092 B | 35,874 B | 20,052 B | 287,830 |
| Divide by two and negate in inverse Clarke | 260,076 B | 35,849 B | 20,028 B | 281,230 |
| Divide by two and negate in both stages | 260,076 B | 35,849 B | 20,028 B | 281,230 |

All 17,564 cold/warm transform calls matched the oracle-checked return values
on each linked Cortex-M4 variant. Neither division-by-positive-two version
increased an individual transform's simulated instruction count, stack or
WASM fuel. The combined version also passed a separate 39,879-call complete
modulation replay on both architectures. It checked every result tag and all
seven interval words of each success. Each successful Thumb call consumed 25
fewer instructions. Maximum stack use fell from 456 to 448 bytes on successes
and from 392 to 384 bytes on both error classes. No individual call increased
Thumb instructions, stack or WASM fuel. The complete modulation caller's Thumb
code fell from 5,792 to 5,672 bytes and the linked WASM from 38,458 to 38,432
bytes.

Standalone native processes initially suggested that the inverse-only version
sped up successful modulation by 1.6%. Chain timing varied from −2.70% to
+5.17% on the two cores. Repeating the comparison with both libraries in each
process, alternating load and call order, gave a successful-modulation
slowdown of 0.56–0.63% on all four cores. The combined version's medians for
that case ranged from 0.00% to +0.04%. Inverse Clarke improved by 3.68–3.83%,
overmodulation rejection improved by 0.95–1.23%, and the complete transform
chain varied from −0.20% to +0.16%. These measurements cover 21,120 completed
operations per timed block. They do not establish physical control-loop
timing.

The board-selected MKS ESP32 FOC V1 release ELF built for all three stages,
with the same two existing unused network-member warnings. The preceding and
inverse-only ELFs were byte-identical. The combined version differed only
in `.debug_info` and `.debug_line`. Each build had 1,005,041 executable
bytes. On all three, espflash 4.7.0 rejected the 4,190,800-byte application
for the declared 4 MiB board's 4,128,768-byte partition. The isolated change
cannot provide the missing 62,032 bytes. The production candidate is preserved
at firmware `d77bafd74` on `interval-inverse-clarke-halves-review`. Physical
cycles, interrupt-inclusive stack and HIL replay remain unqualified.

Only the regression test entered the firmware work branch. Its final release
executable grew from 940,368 to 949,648 bytes, with 4,510 bytes of additional
code. The initial assertion formatting exceeded the 1% file cap. Keeping the
same endpoint and error assertions within that cap left 66,159,216 bytes of the
64 MiB host test slot free. The fixed and preceding measurements were equal.
The separate host test account records both charges and their remaining
allowances. Production memory and flash charges are zero.
[Per-target size account](docs/research/interval-size-accounts.json)

On the retained and combined revisions, Rust 1.88 passed 87 FOC unit tests,
three FOC integration tests, 68 simulator unit tests and eight simulator
integration tests. FOC all-target/all-feature Clippy with warnings denied
and firmware formatting passed. The coordinated host conformance crate also
passed all seven tests against retained firmware. Evidence commit `0d510b87f`
contains all candidate patches, the mutation, oracles, linked guest images,
raw comparisons, and per-stage board packaging logs.
[Evidence index](docs/evidence-archive.tsv)

## Phase 4 and 14 ordinary WASI Preview 1 qualification

Commit `d1a81beae` adds the `float-enclosure-wasip1` Cargo feature to
Hyperinterval and forwards it through Hyperlattice Interval. It enables
certified binary32 and binary64 arithmetic on the ordinary `wasm32-wasip1`
target. Cargo's stable target cfgs coincide for the ordinary and threaded ABIs
on the three tested compilers. The optional `no_std` dependency
`target-tuple` supplies the exact target name at compile time. The linked
scalar reactor imports nothing and contains no floating arithmetic, allocator
or memory growth. The earlier raw-file rejection remains recorded above.

On preceding `3ebe192ae`, both widths returned `UnqualifiedEnvironment` for
a valid ordinary Preview 1 singleton. The same public client now accepts that
input when the feature is selected. The default configuration, the threaded
ABI and `wasm32v1-none` preserve typed refusal, valid empty and endpoint traps
at their unqualified boundaries. One Cargo invocation built both Preview 1
ABIs with the feature. A deliberately overbroad selector passed ordinary
ingress and failed the threaded negative control.

The automatic selector added 109,717 bytes to an existing mixed lattice
consumer on ordinary Preview 1, including deployments using only its fixed
operations. Making selection explicit preserved the default footprint.
Putting target detection in Hyperinterval's own build script also changed
executable bytes in an unrelated fixed-only ESP32 build. An isolated optional
helper removed that coupling. The retained implementation uses the published
target-name crate and avoids adding a Hyper package for one build fact.

| Rust 1.95 ordinary Preview 1 client | Default file | Opt-in file | Opt-in code | File cap | Slot headroom |
| --- | ---: | ---: | ---: | ---: | ---: |
| Primitive IEEE | 29,882 B | 58,507 B | 48,044 B | 65,536 B | 72,565 B |
| Mixed lattice | 443,306 B | 553,023 B | 492,554 B | 655,360 B | 3,641,281 B |
| Dyadic conversion | 60,912 B | 80,488 B | 58,750 B | 98,304 B | 4,113,816 B |

These matched builds used the same source and Cargo registry paths. All
12 existing default artifacts matched the preceding files byte for byte. The
three new opt-in artifacts used 8, 3 and 1 more file bytes respectively than
the automatic same-ABI implementation. Code-section and function-body sums
matched that implementation on all three. In the separate bare-target
feature-selected mixed lattice client, one additional file byte was charged.
The fixed and preceding measurements were equal in all original target
configurations, using the introduction revision for clients absent at the
fixed program baseline. The primary checkout's opt-in primitive image measured
58,479 bytes. The per-target accounts retain each path's role and the full
initial footprint of each new configuration.
[Size accounts](docs/research/interval-size-accounts.json)

Rust 1.88, 1.95 and stable 1.99 completed 55,802 exact arithmetic cases and
8,836 property cases per qualified target. Each also passed 612 floating and
612 fixed lattice chains, 56 dyadic conversions and the separate fixed-lattice
replay's 172,284 calls. Rust 1.95 and stable passed all four Real bridge retry
calls per target. Wasmtime 50 independently matched all 165,138 public scalar
status, endpoint and property calls. Each call used exactly the same fuel in
the preceding bare, automatic Preview 1 and retained Preview 1 builds. All
1,224 previously successful ordinary Preview 1 fixed-lattice endpoint reads
matched individually in value and fuel after selection. Median complete Node
oracle replay times for the feature-selected versus automatic builds varied
from −0.70% to +2.11% across four pinned, alternated process runs.

The retained stack probe measures both the deepest original guest write and
the lowest guest stack pointer in an independently replayed instrumented
copy. It checks agreement with the linked disassembly and verifies that every
exported call restores the pointer. A 4,096-byte unwritten frame touched only
704 bytes and passed the old probe. The new one recorded both values. It also
detected a leaked 16-byte frame and a mismatched module/disassembly pair.
Across all three compilers and both qualified WASM targets the primitive
workload used 704 bytes by both measures, cold and warm.

On ordinary Preview 1 the mixed lattice corpus reserved 393,440 bytes of its
1 MiB guest stack and made zero Rust allocator requests. The existing
fixed-only workload reserved 393,376 bytes both before and after selection.
Dyadic conversion reserved 816 stack bytes and made 62 allocations and 62
frees, with 2,604 peak requested heap bytes and zero retained. These stack
and requested-heap observations matched the corresponding completed bare
and automatic controls. Preview 1 stayed at its initial 1,114,112 bytes of
linear memory. Bare dyadic conversion committed one further 64 KiB page.
No positive memory exception was charged.
[Memory accounts](docs/research/interval-memory-accounts.json)

The complete MKS ESP32 FOC V1 ELF was byte-identical to its matched preceding
build, including debug information. Both packaging runs reproduced the
existing 4 MiB board partition failure. With the same temporary Cargo home on
each side, espflash measured 4,191,408 application bytes for a 4,128,768-byte
partition, a 62,640-byte shortfall. No firmware flash allowance was spent.

Rust 1.95 passed 366 all-target/all-feature owner tests, 20 doctests and 12
minimal tests/doctests. Rust 1.88 passed 280 compatible owner tests/doctests.
Packaged crates reproduced both positive and negative WASI behavior. Chromium
ran the bare oracle, the coordinated firmware and Interface checks passed,
and the native code-generation scan, 16 MXCSR modes, Clippy, strict rustdoc,
formatting, strict zizmor and 10,000 address-sanitized native fuzz iterations
passed. Evidence commit `db1b189a8` holds the linked measurements, full
per-call fuel observations, allocator and stack probes, intermediate variants
and regression controls. [Evidence index](docs/evidence-archive.tsv)

## Phase 0 and 6 Hyperlattice Interval coverage and translated sources, 18 November 2026

Commit `00df0ab10` registers Hyperlattice Interval in the shared native
coverage command. On the preceding revision the package selector rejected the
owner before running a test. The collector now covers 14 production source
files using 24 native test executables. The two array lifecycle tests moved
verbatim to the private `tests` module so LLVM can attribute their counters
separately. The collector's existing package behavior and thresholds are
unchanged. Separate target directories are necessary when comparing
checkouts, as LLVM records the physical source paths.

The missing witness concerned the affine source's signed parameters, absolute
enclosure and the image of an empty domain. The scalar oracle checks the
translated source through the public trait on fixed, rational, dyadic and
IEEE backends. A constant point map with value 7 must send both the canonical
empty domain and an empty restriction with a retained center to the empty
set. Three independent mutations passed all 93 previous owner tests. The new
assertions detected returning the center as the parameter domain, returning
offsets as the absolute enclosure, and publishing the singleton 7 for an
empty input. The last mutation also failed the optimized release test.

| Native all-feature production coverage | Before the witness | With the witness |
| --- | --- | --- |
| Rust 1.95 lines, regions, functions | 1367/1404, 1861/1970, 266/270 | 1377/1404, 1869/1970, 268/270 |
| Nightly lines, regions, functions | 1352/1389, 1332/1434, 267/271 | 1362/1389, 1338/1434, 269/271 |
| Nightly branches | 81/88 | 82/88 |

Both comparisons exclude the unit-test module on each side. The nightly
translated-source file reached 92/92 lines and 8/8 branches. The package's
100% production gate still fails on both compilers. Rust 1.95 does not
instrument branches in this configuration.

Matched Rust 1.95 release builds used a single isolated checkout and cleaned
the owner before each of the fixed, preceding and final builds. The unit-test
executable shrank by 240 bytes, with identical code size. Its read-only data
grew by 16 bytes, bringing measured cumulative growth in that section to
864 bytes (1.58%). The centered-affine executable retained its preceding
file and code sizes. The complete fixed-affine consumer executable matched the
preceding build byte for byte. Prior lifetime-test charges remain in their
original account. No production memory or firmware exception was used.
[Per-target size accounts](docs/research/interval-size-accounts.json)

Final local checks passed for the 93 all-feature owner tests, five doctests,
seven minimal tests and doctests, 159 downstream Hyperdynamics tests, strict
Clippy and rustdoc, and workspace formatting. Rust 1.88 passed the seven
compatible focused tests. Both bare WASM and WASI Preview 1 repeated 55,802
scalar oracle cases and the 172,284-call fixed-lattice replay on Rust 1.95.
Evidence commit `80d0f8ac1` contains the per-file reports, annotated lines,
mutation patches, linked measurements and validation logs.
[Evidence index](docs/evidence-archive.tsv)

## Phase 6 and 14 Cartesian tests on WASI Preview 1, 18 November 2026

Commit `9a6546a65` extends the existing coordinated WASM command. Cargo uses
Node's Preview 1 command runtime to run the owner's Rust tests in the guest.
The complete fixed, dyadic and qualified IEEE library/integration selection
passed 60 tests. The fixed JSON and rectangular shape replay suites passed
three and two tests respectively. Rust 1.88, 1.95 and stable 1.99 each ran
all 65. A real failing tight-budget test propagated its guest panic through
Cargo. Uncaptured output retained the original `Scalar(WorkExhausted)`.
A guest exiting with status 23 returned 23 to the host. A filter that removed
every arithmetic test made the checker fail even though Cargo exited zero.

The 65 passing names matched their native cases on every compiler. The native
configuration has one further case, `float_decomposition_and_reconstruction`,
whose existing x86-only guard excludes it on WASI. Enabling it experimentally
passed all 66 cases on all three guest compilers, including both IEEE widths,
subnormals, maximal endpoints and reconstruction. The complete guest test
file grew by 58,247, 58,989 and 67,102 bytes respectively. The code sections
grew by 50,254, 50,083 and 49,482 bytes. Each crossed the predeclared
32 KiB and 6% incremental limits for that hosted test module. On Rust 1.95
a matched size-optimized fat-LTO build still added 48,018 file bytes and
36,430 code bytes. The guard change was declined. The new routine check
reports only executed cases.

All 14 existing Preview 1 client images were byte-identical to the preceding
checker on their respective compilers. The 55,802 scalar oracle cases and
172,284-call fixed-lattice replay passed on each compiler. Rust 1.95 bare
WASM reproduced the previous complete report. The standard-library guest
harnesses use WASI command imports and are separate from the existing kernel
import, disassembly and allocation checks. Running the scalar owner's native
test package in the guest remains open because its GMP development dependency
rejects that cross-build. Formatting, JavaScript syntax, Python compilation
and repository retention checks passed.

### Dyadic box bisection requalification

Against `5f1d06746`, the existing translated-and-clipped implementation
exhausted work on the four-bit, sixteen-unit single-box witness and the
227,200-unit SDF cover. The prior direct-split migration and a variant that
reused the proof of nonempty lanes completed both. Both reproduced the full
11-cell dependency cover, the full 23-cell affine sphere cover and the exact
four-box reachability output. The dependency workload fell from 227,460 to
227,189 units and the sphere from 3,624,110 to 3,623,512. Peak requested heap
and post-drop bytes matched in every observed cover, capacity and exhaustion
case. The largest observed guarded-stack writes for the two SDF callbacks and
the reachability callback were 26,552, 26,552 and 29,432 bytes respectively
for all three builds.

The matched Rust 1.95 release reachability client exceeded two existing gates
for both box candidates. The union of linked function ranges grew by 1,846
and 1,870 bytes against the 1,024-byte incremental cap. RELRO padding grew
from 440 bytes to 2,584 and 2,568 bytes respectively. Both implementations
remain outside the retained library. Two independent ordered-axis witness
variants also detected a reversed-child mutation missed by the 93 previous
owner tests. Both crossed the host test executable's RELRO padding limit.
No memory or code allowance was charged. The original harder cases, alternative
implementations, native and guest transcripts, complete cover definitions
and per-target measurements are in evidence commit `362ddb6ae`.
[Evidence index](docs/evidence-archive.tsv)

## Phase 3, 5 and 14 scalar oracles on WASI Preview 1, 18 November 2026

Commit `933c9ae56` makes Hyperinterval's existing test executables build and
run with GMP/MPFR on ordinary WASI Preview 1. On `7d4fc5d7e`, Cargo rejected
even the direct dyadic bisection test because its development dependency
refused the cross build. After enabling cross compilation, GMP itself failed
to compile its error handling against WASI libc. The retained target-specific
development dependency and C toolchain configuration resolve both failures.
The C build uses the WASI sysroot and its emulated signal library. The linked
arctangent test contains GMP and MPFR code and executes its directed oracle
in the guest.

| Toolchain | Passed in the guest | Probed in the guest and passed on the host | Additional target-gated native cases |
| --- | ---: | ---: | ---: |
| Rust 1.88, compatible features | 178 | 6 | 32 |
| Rust 1.95, all features | 228 | 9 | 33 |
| Rust 1.99 stable, all features | 228 | 9 | 33 |

Every guest case matched an existing native case in the same test executable.
The guest coverage includes exact bisection, typed budget failures and retries,
directed transcendental bounds, and rational cases on the two newer compilers.
The two rightmost columns have distinct evidence. The process and thread tests
were executed individually in the guest and hit an unsupported OS call. Their
original tests then completed natively. The remaining cases compile only under
their existing target guards. The report names each guest pass and each case
that required the host. Native-only inventories are archived separately.
A guest-only incorrect midpoint passed native bisection tests and failed the
guest's independent midpoint assertion. An unrelated unsupported error in a
process probe also failed the new check.

The dedicated CI job targets Ubuntu 24.04. Local runs used its Clang 18,
LLD, compiler runtime and WASI libc packages, then independently reproduced
the outcomes with Debian Clang 19. Node 22 executed the guest programs.
The GitHub-hosted job itself has not been observed. The force-cross feature
appeared only in the exact Preview 1 development dependency graph. The
ordinary dependency graph excludes Rug and GMP. The representative native
release client and native extended-interval test executable were byte-identical
to the preceding build. All 14 existing Preview 1 conformance images and
their complete reports also matched. This change used no production flash or
memory allowance.

An experiment enabled the existing x86 extended-IEEE ingress witness on
WASI. Both IEEE widths passed. With the original assertion, the guest file
grew by 5,024, 4,027 and 5,052 bytes respectively, exceeding the 4,096-byte
incremental cap on Rust 1.88 and stable. Comparing the returned error directly
preserved the typed expectation and reduced the three increments to 2,470,
1,553 and 2,442 bytes. All three guest file and code caps then passed, but
native release test RELRO padding increased from 48 to 1,560 bytes. Both
variants remain in the evidence archive. The retained driver preserves the
original witness and its target guard.

Strict native and cross-target Clippy, strict native rustdoc, owner doctests,
the downstream Hyperlattice Interval tests, workspace formatting, package
verification for both Hyperreal and Hyperinterval, strict zizmor and repository
budget checks passed. Evidence commit `76f0c1977` contains the inventory by
test name, transcripts, linked measurements, normalized dependency checks,
source patches and failed controls.
[Evidence index](docs/evidence-archive.tsv)

## Phases 3, 7 and 14, solver intervals and dispatch on WASI Preview 1, 18 November 2026

The solver acceptance job in `7b5fb0fdb` runs twelve complete public
integration suites and both exact Horner migration tests in WASI. The 63
tests cover interval expression evaluation, radius validity, quadratic
extrema, Jacobian traces, Krawczyk certificates, symbolic enclosures and
trivariate tensor capacity. Each guest integration inventory must match the
corresponding native executable. The Horner tests exercise the production,
frozen and candidate polynomial paths on the existing 702-case corpus.

The two Horner trace comparisons cover 1,404 polynomial evaluation windows
and 2,808 root-exclusion and derivative windows. Hyperinterval adds 450
rational product windows. In each comparison the native and guest frozen
and shared implementations run in four fresh processes. Every window
compares its ordered dispatch counts, reducer statistics and full result.
A guest-only extra rational temporary left the exact four-corner product
oracle green and failed this comparison on its first row. The original
native parent-process assertions still pass.

The preceding solver test build failed in `wait-timeout`; after isolating
Proptest's process-fork features, Criterion rejected its threaded backend
on WASI. The target-specific development dependencies retain in-process
property generation, bit sets and Criterion plotting. The 64-bit tensor
capacity test also contained shifts that Rust rejected when compiling a
32-bit `usize`. Commit `f18c2f79b` preserves the original 64-bit cases and
exercises shape-product overflow and each unreservable axis on 32-bit.
The wide determinant fixture retains its explicitly typed heap-backed matrix.

| Local qualification | Rust 1.95.0 | Rust 1.99.0 |
| --- | ---: | ---: |
| Solver library tests passing directly in WASI | 622 of 627 | 622 of 627 |
| Solver integration tests passing directly in WASI | 887 of 904 | 887 of 904 |
| All-feature native solver tests passing | 1,533 | 1,533 |
| Complete dispatch windows matching across both targets | 4,662 | 4,662 |

All 69 solver guest executables were linked and every listed case ran in an
isolated guest. Two library cases invoked a parent process that WASI cannot
spawn. Their full child computations were separately qualified by the Horner
comparison. The other three library cases require threads or panic unwinding.
Seventeen integration cases reached a missing process, thread or temporary
directory facility. Every one passed in the native inventory. Two further
replay cases compile only for the native target. None of these counts includes
a timed-out test. The scalar driver also passed 178 guest tests on Rust 1.88
and 228 on each newer compiler, with the original 6/9/9 native OS probes.

Eight matched native test executables retained their preceding allocated
section sizes and defined-function byte counts. Four file sizes grew by 16
to 104 bytes, within their per-artifact and cumulative limits. Earlier matrix
layouts and a common pointer-width shift expression had measurable costs.
The three-row matrix table exceeded its Rust 1.95 all-feature RELRO padding
cap. The common shift version added 40 bytes to a stable all-feature test
file whose historical growth already exceeded its cumulative allowance. The
retained variant reduced that file by 24 bytes. All previous charges remain
reserved. A separate 64 MiB hosted test slot with 16 MiB reserved covers each
new guest executable, with at least 58,839,873 bytes free. The ordinary
Krawczyk example as a downstream release client remained byte-identical on
native (3,368,824 bytes) and WASI (1,704,627 bytes), with matching complete
output. No deployment, memory or stack exception was consumed.
[Size accounts](docs/research/interval-size-accounts.json)

Strict native and cross-target Clippy, formatting, all six packages through a
local registry and strict zizmor passed. The hosted CI job has not been run.
These results qualify solver tests on another target. Production Horner
migration still requires the existing runtime and size gates. Evidence commit
`b6c8d68f8` contains the transcripts, observed inventories, comparison
controls, rejected fixture layouts and final-artifact measurements.
[Evidence index](docs/evidence-archive.tsv)

## Phase 3 and 7 rational Horner memory reassessment, 18 November 2026

Rebuilt both consuming Horner prototypes on `79b183c51` and measured the fixed
program control `c9f1f9ac5`. Through report construction, tensor-result reuse,
validation and destruction, the cold stack peak was 12,424 B in both controls.
The ordinary-inline form reached 12,504 B and exceeded the 32 B incremental
and 64 B cumulative caps. Targeted inlining brought it to 12,440 B, within
both caps. All reports and requested allocation measurements matched on every
paired row. The other 99 case and cache-state combinations stayed within their
stack allowances.

On the targeted candidate all 4,212 actual production dispatch windows matched
the frozen routines across native and WASI. Two nine-workload Criterion
campaigns did not reproduce the earlier exact-point slowdown. The current
default-feature binary fit its prospective size screen. Four existing Rust
1.95.0 release accounts blocked retention. The certification executable alone
grew 38,880 B against its 4,096 B incremental limit. Both prototypes remain
outside production, with zero new charges to the size and memory accounts.
[Horner measurements and disposition](docs/research/horner-working-interval-admission.md)

## Phase 3 exact rational scalar scaling, 18 November 2026

Revision `e221c81ff` adds `RationalInterval::scale(&Rational)`. Section 3.5
requires scalar scaling for finite exact families. The existing rational
`affine` method already handled shifts. The new entry point passes the
borrowed factor as both scalar endpoints to the existing sign-partitioned
product kernel. This preserves predicate order and avoids constructing a
singleton interval.

The public tests check 495 interval/factor combinations against signed
integer numerator products over a common denominator, invert every nonzero
factor, preserve empty sets under zero and negative scaling, and replay a
129-bit cancellation before a second map. A retained counter assertion checks
two endpoint products on a crossing interval. An isolated extra-product
mutation preserved every endpoint and failed that assertion. Fresh native
and WASI processes also matched all 160 complete result rows and all 160
dispatch and reducer windows against explicit singleton multiplication.

| Final release client, Rust 1.99 | Singleton multiplication | Borrowed scalar |
| --- | ---: | ---: |
| Native replay and affine composition | 1,277,048 B | 1,275,912 B |
| WASI replay and affine composition | 435,261 B | 434,775 B |
| Native trace client | 1,229,624 B | 1,228,864 B |
| WASI trace client | 367,766 B | 367,818 B |

The [size accounts](docs/research/interval-size-accounts.json) charge the
52 B WASI trace file increase. Its code section shrank by 28 B. The existing
singleton trace client has its own 4 B file charge. No exception was needed
for heap or stack. The fixed and preceding controls matched across all 16
case and cache-state maxima. Checking and destroying the complete direct
result used 64–88 B less stack. The highest measured stack was 8,504 B and the
highest requested live heap was 1,280 B. Allocation counts, requested bytes
and retained bytes matched. The stack harness includes a calibrated 1,024 B
caller frame to expose shallow calls beneath its protected area.

In two paired timing campaigns, direct scaling took 55–71% of singleton
multiplication time across the seven nonempty cases and about 13% for empty.
Each individual paired 95% interval stayed below one. These are warm,
single-threaded operations on the measured host. The first method placement
grew the existing rational test executable's text section by 64 B. With the
final inline placement the rational power example, Hypersolve's
`enclosed_program` example and the existing rational tests retained identical
executable code.

The full Hyperinterval release suite passed 291 tests on each of Rust 1.95.0
and 1.99.0. The scalar WASI runner passed 231 guest tests and all nine
host-required cases. All three new feature-enabled tests also executed in
WASI on both compilers. Hyperlattice Interval passed 98 downstream tests and
Hypersolve passed 627 library tests. Strict owner Clippy and rustdoc passed on
both compilers. Firmware dependency inspection confirmed the clock and FOC
select the integer and fixed features respectively. Phase 3 and the broader
plan remain open.

The evidence archive commit is `79eceffd89912fe260bf9f6e7566f5c900455dc2`,
record `docs/evidence/2026-11-18-interval-rational-scale`. Its 569 members
include both placements, negative controls, raw resource and timing rows,
feature graphs and final-artifact measurements. [Record digest](docs/evidence-archive.tsv)

## Phase 12–14 Hyperdynamics numerical suites and fresh WASI replay, 18 November 2026

On the preceding revision, Criterion's default Rayon feature prevented the
WASI test build. Commit `973fb0cf8` selects Criterion's single-threaded
configuration for WASI and adds a cross-runtime test driver. Production
dependencies resolve to the same graphs on Linux and WASI. All nine release
examples retained their exact executable bytes on both targets under Rust
1.95.0 and 1.99.0.

On each compiler, the driver accounted for all 159 original library and
integration tests. It ran 141 tests directly in WASI. For 17 fresh-process
cases the native parents supplied their original inputs to 33 separate WASI
children and applied the existing assertions to the results. These include
source identities, Jacobians, rational clocks, Taylor and QR evidence and
file-backed noise histories. The final case exercises destruction on a real
64 KiB native thread. The guest reports unsupported thread creation.
A changed guest that interchanged the image and Jacobian completed its child
test, and the original native parent's equality assertion caught the change.

| Measurement | Rust 1.95.0 | Rust 1.99.0 |
| --- | ---: | ---: |
| Complete warmed process-group memory peak | 114,257,920 B | 114,069,504 B |
| Largest regular native test file increase | 152 B | 72 B |
| Largest regular WASI test file increase | 186 B | 2,549 B |
| Largest opt-in coordinator file increase over its ordinary test | 128 B | 1,952 B |

The memory measurement includes Cargo's warm checks, native libtest, Python,
Node, guest execution and cleanup in separate 1 GiB Linux cgroups with swap
disabled. Each run finished with over 915 MiB of headroom and zero limit or
OOM events. Cold compiler memory, firmware and other input corpora are
outside that measurement. The run charges no production memory or stack
allowance.

The [per-target size accounts](docs/research/interval-size-accounts.json)
retain the preceding charges, including 7,856 B previously charged to the
Rust 1.95 native RK4 test file. Its 152 B increase brings that account to
8,008 B of the 8,192 B campaign cap. Native regular-test executable code
showed no increase. Every new charge fit its predeclared caps. The
fixed and preceding WASI size controls use the Criterion manifest adjustment
to make their unmodified test and example sources buildable.

Local verification covered both compilers' independent feature matrix,
minimal and all-feature tests, optimized suites, bench entry points,
doctests, strict native and guest Clippy, strict rustdoc and fuzz-target
checks. The reachability policy fuzzer completed 10,000 address-sanitized
executions. Seven packages verified through Cargo's local registry and
strict zizmor passed. The new CI job uses Node 24.22.3. Local Node 22.23.5
terminated during the nonlinear reachability case. All eleven direct
reachability cases completed on Node 24.22.3 and 26.13.1. Hosted execution and the remaining
platform and release requirements stay open.

The [evidence index](docs/evidence-archive.tsv) registers 509 observations
and supporting files in archive commit `63d6c4e7ef3b7549d80726ed6c9059c12122084c`,
record `docs/evidence/2026-11-18-interval-dynamics-wasi`.

## Phase 7 univariate proof replay and bounded memory review, 18 November 2026

Commit `442615f77` adds an independent regression test for quadratic
certificate reports with translated algebraic roots. On Rust 1.95.0 and
1.99.0, native and WASI each passed all 72 reports and 648 exact or
certified-filtered equalities. A signed-derivative mutation failed the new
test while preserving its first root verdict. The all-feature Hypersolve
release suite and doctest recorded 1,536 passes on each compiler. Both
compilers passed strict native and guest Clippy, strict rustdoc and workspace
formatting. Each WASI solver run passed 64 tests and matched all 4,212
native/guest Horner dispatch windows. The Rust 1.95 downstream native
Hyperdynamics run passed all 159 tests.
[Algebraic-root oracle](crates/hypersolve/tests/krawczyk_univariate_algebraic.rs)

On the preceding production revision, `APPROXIMATE_512` certified a nonroot
in the middle of a three-row point-box problem and rejected an unresolved
strict contraction bound. Use `PredicatePolicy::STRICT` for proofs from the
public univariate certifier. Seven changes passed both failing cases.
The two that reached complete-lifetime qualification fit every ordinary and
stress memory allowance, including all previous positive stack charges.
Both independently regressed the owning benchmark in repeated two-core
campaigns. Borrowing both divisors improved the same benchmark by 2.05–2.24%
and exceeded the lifetime client's allocated ELF section limit by growing
`.relro_padding` 208 bytes. The production repair remains open.
[Inputs, measurements and individual decisions](docs/research/interval-box-proof-scope.md#univariate-quadratic-proofs-at-algebraic-roots-and-unresolved-thresholds)

For the call-site variant, the three-stage build produced 42 registered
release executables per revision. All 33 with matching registered features
passed the incremental and cumulative file, text and named-text checks.
The downstream six-step `((A-B)-C)+D feedback` model completed all steps
with certified, closed manifolds on the control and candidate. With matched
executable and output paths every semantic and allocation column agreed.
Its linked file grew 16 bytes, with identical allocated section sizes and
defined-function coverage in bytes. These downstream observations cover
that feedback chain. The sole new size account is for the retained test
on four target/compiler combinations. Production memory charges carry forward.
[Size accounts](docs/research/interval-size-accounts.json)

The evidence archive contains 1,396 observations and supporting files at
`92bd98328f7f809eab8315f9e0419d274f8f3561`, record
`docs/evidence/2026-11-18-interval-univariate-proof`, digest
`23c956a12c18f474ab8c919605d40746fb1a0ae9600398fb72fe24790ccd9b5d`.
[Evidence index](docs/evidence-archive.tsv)

## Phase 7 and 14 whole-solver WASI qualification, 18 November 2026

Commit `0e3566c05` extends the Hypersolve WASI job from 64 direct tests to
the full library and integration inventory. Rust 1.95.0 and 1.99.0 each
accounted for the same 1,535 cases across 70 test executables. Each ran
1,511 tests directly on WASI, 17 native parent cases with 26 fresh guest
executions, two Horner comparisons covering 4,212 ordered dispatch windows,
and five native tests for threading or unwinding. The runner checks the
expected WASI diagnostic for each native-only case. Splitting the cost
overflow assertion from its threading test lets the overflow check run on
the 32-bit guest. The restored vertical-component and field-resultant cases
reconstruct their original algebraic inputs in fresh guests.
[Qualification driver and execution modes](crates/hyperinterval/conformance/README.md)

Eight native parent cases also compared 15 complete output records from
14 guest calls. The records cover exact Jacobians, charts, Taylor coefficients
and remainders, work counts, invalid source references and dispatch.
Each emitted record was byte-identical across the two compilers.
Changing one guest expression value failed the original parent assertion.
Increasing one Taylor graph-work count by one passed the guest's repeatability
check and failed the comparison against the native model.

| Warm qualification job | Previous peak | Full-inventory peak | Free under 1 GiB cap | Previous / full wall time |
| --- | ---: | ---: | ---: | ---: |
| Rust 1.95.0 | 66,109,440 B | 84,496,384 B | 989,245,440 B | 5.267 / 76.165 s |
| Rust 1.99.0 | 68,030,464 B | 82,939,904 B | 990,801,920 B | 5.250 / 76.119 s |

Each measurement includes Cargo lookup, the native, Node and Python process
trees, recorded output and cleanup. Swap was disabled. All four cgroups kept
the required 256 MiB reserve and recorded zero limit or OOM events. The
account charges the full observed peak to each new test workload. Cold builds
and production heap and stack are outside this measurement.
[Memory accounts](docs/research/interval-memory-accounts.json)

All 24 release example images matched their controls byte for byte. Maximum
ordinary file growth on Rust 1.95.0 was 992 B native and 246 B WASI. On
Rust 1.99.0 it was 640 B native and 165 B WASI. Enabling the two previously
excluded algebraic replay cases added 247,039–357,703 B to their respective
guest executables, measured independently of the driver change. Each fit
the 16 MiB test executable cap. Two stable native Taylor test executables
already carried excess cumulative file growth. Borrowing the UTF-8 stdout
from each process result recovered 1,024 B and 656 B on those executables.
The fixed, preceding, ported and retained measurements preserve all prior
positive charges. [Per-target size accounts](docs/research/interval-size-accounts.json)

A public budget probe found an open 32-bit diagnostic defect. With a limit
of 4,294,967,296 bits, observing 4,294,967,297 bits correctly returns
`WorkKind::CoefficientGrowth` on both targets. On WASI the public error
fields and display report 4,294,967,295 for both counts. The native 64-bit
receipt preserves both inputs. The witness calls the budget API directly.
[Coefficient observer](crates/hypersolve/src/work_budget.rs)

Both compilers passed the native Hypersolve release suite and doctest,
strict native and guest Clippy, strict rustdoc, the owner bench targets in
test mode, and fuzz-target checks. The shared child runner also passed each
Hyperdynamics WASI suite with 141 direct tests, 17 fresh replay cases and one
native thread case. Workspace formatting and strict zizmor passed locally.
Hosted jobs and the remaining release and solver migration gates stay open.

Evidence commit `657e4535e65fe4bb9b2cf7ef7b7caa2d50fc7606` contains
885 files for this qualification, including both mutations, the 32-bit
reproducer, compiler-specific lifetime measurements and linked binary
accounts. Record `docs/evidence/2026-11-18-interval-solver-library-wasi`
has digest `532e947f4b97cd0d0c9fa3dd615b01fec2af7e9701a5d07592ae0e3c4f0b7291`.
[Evidence index](docs/evidence-archive.tsv)

## Phase 7 coefficient-width diagnostics and scope ownership, 18 November 2026

Commit `063cb703d` qualifies 512 outer-limit, inner-limit and observed-size
triples through the public solver API. The corpus crosses 2³² and 2⁶³ and
includes equality at `u64::MAX`. Six exact receipts identify the outermost
scope that denied work, including requests at each target's maximum word
value. The corpus checks publication, work usage, sticky failure inside a
new child, parent recovery and independent retry. All cases passed in
release on native Linux and WASI with Rust 1.95.0 and 1.99.0. Narrowing
the comparison to 32 bits or searching from the innermost scope failed the
independent oracle on both targets. The WASI inventory requires the new
suite. [Public acceptance tests](crates/hypersolve/tests/coefficient_bit_budget.rs)

The configured limit and observation both accept `u64`. The public error
still carries its limit and request in `usize`. With limit 4,294,967,296 and
request 4,294,967,297 the current WASI receipt saturates both fields to
4,294,967,295. The exhaustion kind and decision are correct. Five trial
implementations preserved all exact receipt values in a separate 81-pair
corpus on both targets and compilers. Changing the public field types also
requires migrating callers that store their counts in `usize`.
[Public scope and diagnostic limitation](crates/hypersolve/README.md)

Simply widening the two fields grew the WASI receipt from 16 to 24 bytes.
Retained requested heap on a fresh successful scope rose from 224 to 256
bytes, and from 1,792 to 2,048 bytes with 32 nested scopes. Both exceeded
the 5% incremental limit. Sharing the first failure across its affected
scopes brought those observations down to 160 and 1,280 bytes respectively.
Keeping a frame's limits or its first failure in the same storage reached
192 and 1,536 bytes. Four designs passed the measured comparable heap
screens. Each of the five trials independently exceeded at least one
existing linked executable's 4,096-byte incremental cap. For example, the
frame-state design grew the stable WASI ordered-field resultant executable
by 11,922 bytes. All production memory and size charges carry forward.
The new acceptance executable is charged separately per target.
[Four compiler and target size accounts](docs/research/interval-size-accounts.json)

Evidence commit `f1b4a6ca53b4a7e8be2c05e930833cb06128f251` contains 237
files, including the failing 32-bit client output, linked measurements for
all five designs, heap lifetimes and mutation transcripts. Record
`docs/evidence/2026-11-18-interval-coefficient-budget-width` has digest
`b22e93b75ee33e08cd6299f17db2c203c5bde348ab7f563ddf3e66674fab42d5`.
[Evidence index](docs/evidence-archive.tsv)

## Phase 7 and 14 tensor coefficient budgets and coverage, 18 November 2026

The retained test in `c73863d18` checks an exact product whose denominator
determines its cost. In `(x + 1/16)(x - 1/16) = x² - 1/256`, the largest
stored rational needs ten bits. Nine bits produces an exact
`CoefficientGrowth` receipt, and the enclosing ten-bit scope recovers,
computes the expected coefficients and consumes them in another product.
A trial that counted only numerator bits failed on Linux and WASI with
both Rust 1.95.0 and 1.99.0. The full-width and nested-scope tests still
run in the same mandatory executable. [Public regression suite](crates/hypersolve/tests/coefficient_bit_budget.rs)

The fused `sum_products` operation can publish `256 - x²` under a
nine-bit limit. Its automatic repair must check the completed sum because
opposite ten-bit products cancel to exact zero under a one-bit limit.
Removing the check or moving it inside the term loop failed the witness
on both targets. The public scope documentation names the affected operation
and explains how to observe a fused result before use. A caller using the
existing public APIs completed all 64 signed polynomial cases, including
denial, exact retry, 128 complete dispatch traces and 128 allocation records
per compiler and target. [Published solver scope](crates/hypersolve/README.md)

The unannotated automatic check exceeded the stable WASI stack allowance by
increasing the measured stack by 32 bytes. The inlined version
`3f32da7ae` eliminated positive comparable stack and heap growth on all
four compiler and target combinations. It passed 33 feature-matched legacy
artifact accounts. On the same resultant benchmark its paired medians were
2.796 and 2.789 microseconds on the two cores, against 2.693 and 2.731.
The old work-only fused workload also took 2.99% longer on 1.95 and 2.22%
longer on 1.99. A lazy bit observer exceeded the native linked solver
file cap. A scope-aware observer fit the measured stack but grew the stable
native selected-field executable by 4,232 bytes against its 4,096-byte cap.

Two opt-in API designs also preserved the materialized outputs. Exposing
the private observer directly passed the size and memory screens. Six paired
owner benchmark observations per core gave median regression from
2.719 to 2.792 microseconds and 2.757 to 2.815 microseconds. The separate
public wrapper added eight bytes to the already exhausted stable native
trivariate file account. Their source revisions are `8065c9447` and
`bd4b04a0d`. These alternatives remain under review. Accepted production
size, heap and stack charges carry forward. The additional retained test
workload has its own cumulative file and CI memory accounts.
[Size accounts](docs/research/interval-size-accounts.json) · [Memory accounts](docs/research/interval-memory-accounts.json)

The automatic review passed native release tests, strict Clippy and Rustdoc
on both toolchains, packed-crate external consumption, 10,000 libFuzzer runs,
six certified `successive_operations` steps with identical semantic and heap
rows, the bounded filleted-hyperbola stroke, and fresh-process generated
NURBS trim replay. Its full WASI run completed 1,514 direct tests, 17 native
parents spawning guests, five platform-specific native tests and 4,212
matched dispatch windows. These runs qualify the measured review revision.
The production fused-path and 32-bit count-reporting gaps remain open.

Commits `4f57942ad` and `beb37e7cf` repair the owner coverage report.
A relocated Cargo cache brought dependency sources into the old denominator
and produced 39.73%. The corrected report keeps all 124 measured solver
source rows identical on the same binaries and profile, including all 59,089
owner lines and all 53,085 covered lines. It reports 89.84% on that review.
Empty input and a source with zero executable lines now fail explicitly.
The threshold and mathematical test corpus are unchanged.
[Coverage gate](crates/hypersolve/scripts/coverage.sh)

On the retained revision the full production coverage run reached
89.84% (53,085 of 59,089 lines).
The native/WASI qualification completed the 71-executable inventory on both
compilers. The two compiled CI jobs peaked at 83,251,200 and 84,017,152 bytes in
their separate 1 GiB, swap-disabled cgroups. Both recorded zero OOM events.
The denominator suite, both strict Clippy configurations, Rustdoc, doctests,
all four scalar-cache representation selections and the repository retention
budget passed locally. The test binary observations and cumulative charges
are in the linked accounts. Hosted CI and hardware runs remain open in the
larger program.

Evidence commit `f13164d20eb6b0bdecf1dc864967bd08324b8fd8` contains 1,795
files. Record `docs/evidence/2026-11-18-interval-fused-tensor-budget` has digest
`ec76096f429dfc49f8b9ec49ad8de63e702ecb19d0b4f23d06b21548c84f8e93`. [Evidence index](docs/evidence-archive.tsv)

## Phases 3, 5, 7 and 14, scalar replay and bounded Horner review, 18 November 2026

Commit `88e11ea7b` exercises five native-produced scalar results in fresh
WASI guests. The native parent supplies the actual serialized fixed division,
dyadic grid, rational power, split division or tangent result. Each guest
performs the corresponding operation on the decoded value. The runner
requires a successful parent and one receipt from the named child.
The original native subprocess cases and the direct WASI probes still run.
[Scalar conformance runner](crates/hyperinterval/conformance/wasi-tests.py)

Before the stronger assertions, both the fractional-power and tangent cases
accepted the canonical empty set. The split-division, power and tangent cases
also accepted another operation’s pair of rays. The retained tests check
component counts, exact endpoints and the subsequent operation. Directed
256-bit MPFR independently determines the two finite tangent endpoints.
Across all three compilers the five genuine payloads agreed byte for byte,
and all thirteen substitutions per compiler were rejected on both targets.
The substitutions include malformed JSON, unrelated canonical intervals
and an extra dyadic grid entry.

| Rust release | Direct guest tests | OS-dependent native cases | Extra fresh guests | Warm whole-job peak |
| --- | ---: | ---: | ---: | ---: |
| 1.88 | 178 | 6 | 5 | 225,976,320 B |
| 1.95 | 231 | 9 | 5 | 197,312,512 B |
| 1.99 | 231 | 9 | 5 | 187,351,040 B |

The two Hyperreal-enabled configurations also matched all 450 ordered
rational dispatch windows. Each job ran with 1 GiB of memory and zero
swap, retained its 256 MiB reserve and recorded zero OOM events. These
peaks include native harness relinking and exclude cold dependency
compilation. The individual compiled replay workloads are recorded
separately. Native release tests, strict native and WASI Clippy, workspace
formatting and the downstream Cartesian replay and dyadic suites passed.

The initial assertions exceeded two existing image budgets. Keeping the
exact cardinality check on the decoded vector and comparing both power
rays without emitting an extra array formatter put all five native and
guest harnesses within their existing per-image caps. The largest file
charge was 4,057 B on the Rust 1.99 WASI power test, leaving 39 B
incrementally and 4,135 B cumulatively. The ordinary and opt-in native
images have separate accounts. [Size accounts](docs/research/interval-size-accounts.json) · [Memory accounts](docs/research/interval-memory-accounts.json)

The targeted-inline Horner trial and its ordinary-inline alternative were
rebuilt against this revision. Across the 100 case and cache-state
combinations, both preserved the six heap and allocation maxima. The cold
tensor chain used 12,424 B in the control,
12,440 B with targeted inlining and 12,504 B
with ordinary inlining. Targeted inlining fits the 32 B incremental and
64 B cumulative stack limits. Ordinary inlining exceeds both. The targeted
trial grew the registered Rust 1.95 certification executable by
38,896 B against its 4,096 B cap. The trials were
discarded. The scalar replay commit affects test binaries and has its
own hosted workload account.

Evidence commit `ce9133a4b9fed19d833b1a36eb9600e3a4ba6943` contains 1,200
files under `docs/evidence/2026-11-18-interval-scalar-fresh-replay`, with digest
`24a1641bc7ed36017f9aafe0449fb867df4cfe82cefd84d39a52d9eb4279955e`. [Evidence index](docs/evidence-archive.tsv)

## Phases 2 and 14, tight fixed-point arithmetic hulls, 18 November 2026

In `9fa17b080`, Hyperinterval's Q2.30 and Q31.32 checks require an exact
attainable extremum to fall beyond each inward lattice neighbor. Signed
integer cross-products prove containment, tightness and representability.
The square and absolute-value checks include zero when the interval crosses
it. The original 16-byte fuzz inputs retain their Q2.30 interpretation, and
a 32-byte input additionally supplies all four Q31.32 endpoints.
[Independent oracle](crates/hyperinterval/tests/support/fixed_hull_oracle.rs) · [Fuzz target](crates/hyperinterval/fuzz/fuzz_targets/fixed_arithmetic.rs)

On `5261b146c`, a product widened by one lattice unit passed the Q31.32
extreme matrix and 100,000 sanitizer fuzz runs. Both revised checks rejected
the mutation. A separate one-unit error confined to the 64-bit upper bound
was detected by a 32-byte seed. The native and WASI arithmetic suites
passed on Rust 1.88, 1.95 and 1.99. Three Alumina firmware Q30 differential
tests passed against firmware `35d1b0da6`. The CI fuzz command and three
bounded candidate campaigns completed 100,000 runs each, including an empty
starting corpus and all 4,050 width-specific boundary seeds. The latter
campaign peaked at 128,925,696 bytes within its 1 GiB swap-disabled cgroup.

The largest ordinary executable growth was 688 bytes. Native and WASI emitted
code shrank on all three compilers. The sanitizer image added
14,112 emitted code bytes and fit its
per-image allowance. All production accounts carry forward.
[Size accounts](docs/research/interval-size-accounts.json) · [Memory accounts](docs/research/interval-memory-accounts.json)

Native release tests, strict native Clippy, the fuzz target's strict Clippy
check, strict Rustdoc, formatting and the workflow security audit passed.
The whole-owner coverage gate is still open. The preceding and retained Rust
1.99 runs measured identical production coverage on every file, with
4,920/4,963 lines,
9,863/10,405 regions and
517/517 functions. This
compiler supplied no branch counts. Hosted CI and physical hardware
execution remain outside this qualification.

Evidence commit `7ae7db07739918a6adae8e322084dfbfe7cdea2a` contains 139 files under
`docs/evidence/2026-11-18-interval-fixed-hull-oracle` with digest
`2d9d4a3a3610ba9c45e79aa97445d661447858bf70c33669262e2dd9f1f47edc`. [Evidence index](docs/evidence-archive.tsv)

## Phases 5 and 14, periodic refinement and work exhaustion, 18 November 2026

Commit `8d667f098` checks 18 non-singleton dyadic intervals of width
2⁻⁴⁸. Three neighboring cells around π/6 and π/3 distinguish the grid
images on either side of sin(x) or cos(x) equal to 1/2. At π/4, tan(x)
and cot(x) cross 1. At the respective poles π/2 and π their center cells
produce two rays and partial domain coverage. The neighboring cells have
finite images and complete coverage. Directed 512-bit MPFR bounds agree
on each expected endpoint after rounding to the 2⁻⁸ grid. The reflected
sources independently verify parity, including ray order and signs.
[Periodic conformance tests](crates/hyperinterval/tests/periodic_refinement.rs)

A 64-bit size limit produces typed exhaustion on all 18 inputs. Each
operation also fails one work unit below its completed cost and succeeds
on retry with exactly that cost. The highest observed debit was 341,095,460
units within the 2²⁹-unit fixture allowance. Native and WASI receipts agree.

For the wider intervals [1,2] and [1/2,2], the tests visit every work
reservation made by the public sine and tangent calls. The spent prefix of a
failed call identifies the preceding reservation. This gives 284 and 426
cutoffs on the dyadic source and 272 and 392 on the rational source.
Publishing a whole sine range or the entire tangent line after exhaustion
passed the respective existing numerical suites. The new sweeps rejected
both alterations independently on native and WASI, then verified the exact
retry in the unmodified implementation.

| Rust release | New native / WASI test cases | Complete scalar job direct WASI cases | New native image | New WASI image |
| --- | ---: | ---: | ---: | ---: |
| 1.88 | 3 / 3 | 181 | 1,287,648 B | 684,733 B |
| 1.95 | 4 / 4 | 235 | 1,657,184 B | 759,756 B |
| 1.99 | 4 / 4 | 235 | 1,686,944 B | 779,148 B |

All six new executables fit their 4 MiB file caps. The largest individual
compiled job peaked at 27,377,664 bytes in a swap-disabled 256 MiB cgroup. The complete
scalar jobs kept their 256 MiB reserves, original OS-dependent host cases
and five native-parent replays. The two Hyperreal-enabled compilers also
matched all 450 rational dispatch comparisons. [Size accounts](docs/research/interval-size-accounts.json) · [Memory accounts](docs/research/interval-memory-accounts.json)

The complete stable coverage run gained 10 production lines and 81 regions
with identical denominators. It now covers 4,930/4,963 lines and
9,944/10,405 regions. Nightly gained six lines, 81 regions and a
trigonometry branch, reaching 983/1,005 branches. The additional branch
requires another pass after evaluating both ends of a non-singleton interval. Per-file covered counts
held or improved on both compilers. The 100% owner gate remains open.

Native release suites, minimal-feature tests, native and WASI strict Clippy,
strict rustdoc and formatting passed. The downstream Hypersolve interval
expression and Hyperdynamics expression-field suites passed 9 and 15 tests.
The bounded ASan campaigns completed 8,160 trigonometry
runs under the workflow time limit and 10,000 tangent runs.
A second trigonometry campaign completed all 10,000 runs. Hosted CI and hardware execution
remain outside these measurements.

Evidence commit `1ab2a968d967ff10d05d3c04f17f157c50fc9e57` contains 856
files under `docs/evidence/2026-11-18-interval-periodic-refinement` with digest
`83bca27b5f0984e75f1c4f3faae5dbcd1e321b029861dc05960cb3950760bef0`. [Evidence index](docs/evidence-archive.tsv)

## Phases 3, 5 and 14, rational source budgets for four periodic functions, 18 November 2026

On `b8bfae793`, sine, cosine, tangent and cotangent could reduce a generated
coordinate before checking the caller's budget. Refusal on a cold point with
value 3 and a 98-bit stored numerator made eight allocations and one
reallocation, retaining 128 bytes on x86-64. Commit `5d15ce394` checks both
endpoints before normalizing or copying either. The same refusal now allocates
and retains zero bytes and consumes zero work. A later successful retry still
returns the certified dyadic bounds.

The kernels reserve four linear copies at the larger stored width and three
quadratic stages at each endpoint that may require reduction. The Hyperreal
width query reads the storage representation independently of the cache.
The kernels copy canonical fractions directly. Already-reduced inputs kept
their measured work receipts. A successful sine evaluation of 108/36 on the
2⁻⁸ grid costs an additional 314 work units. Independent directed 512-bit
MPFR bounds cover the point 3 and the interval [2,3] for all four functions.
The tests check different encodings of the same point, denial on the second
endpoint, an exact success budget, a one-unit-short failure, and a retry.
Two cold generated endpoints produced two reductions and the warm repeat
produced none. [Interval witnesses](crates/hyperinterval/tests/periodic_ingress.rs) ·
[Scalar query witnesses](crates/hyperreal/tests/reduction_input_bits.rs)

| Rust / target | Largest stack increase | Temporary / retained heap increase | Minimum stack headroom | Largest existing release file increase |
| --- | ---: | ---: | ---: | ---: |
| 1.95 / x86-64 Linux | 0 B | 0 / 0 B | 2,083,120 B | 320 B |
| 1.95 / WASI Preview 1 | 16 B | 0 / 0 B | 1,046,004 B | 514 B |
| 1.99 / x86-64 Linux | 0 B | 0 / 0 B | 2,083,256 B | 296 B |
| 1.99 / WASI Preview 1 | 16 B | 0 / 0 B | 1,046,048 B | 512 B |

The 16-byte charge appeared with the workspace dependency lock on both WASI
compilers and was at most 0.77% per affected workload. Stack allowances
were 128 B and 2% per step, 256 B and 4% cumulative, with a 256 KiB reserve.
The register includes both dependency locks, operation and whole-ownership
heap epochs, cold and warm inputs, disposal and retry, for one caller per
target. The smallest remaining cumulative allowance among ordinary file
accounts is 400 B. The legacy Rust 1.95 tangent benchmark also passed its
separate relocation, GOT, file and named-function size gates.
[Memory accounts](docs/research/interval-memory-accounts.json) ·
[Size accounts](docs/research/interval-size-accounts.json)

Keeping unreduced coefficients throughout added 351 allocations on a
single-point sine case. A callback added 48 bytes to a 2,320-byte guest
stack measurement, exceeding 2%. Returning all borrowed parts together
increased a guest refusal from 172 to 272 bytes. The separate-reservation
implementation in `5de33dbc9` fit the memory caps but slowed wide rational
sine on repeated linked measurements. Its trial charges are excluded from
the program accounts.

Each of 156 linked release cost cases completed with identical work and
completion counts for 20 paired samples per core. On the two final-build
cores the wide rational sine ratios at precision −256 were 0.996 and
0.973, compared with 1.030–1.086 for the earlier implementation across
four cores. Across all cases, final per-core suite medians ranged from
1.009 to 1.017. Byte-identical controls on separate cores ranged from
0.994 to 1.012. No individual case measured more than 3% slower on both
final-build cores. A separate direct comparison of 152 warm, cold and
refusal workloads had a median ratio of 0.983 on each of two cores.
These measurements resolve the earlier wide-sine slowdown. Smaller
runtime differences remain within the resolution of this comparison.

Native release owner suites, strict native/WASI Clippy, strict rustdoc,
formatting and six inventory tests passed. Hyperreal passed 1,053 unit
tests and both scalar query tests. Hypersolve and Hyperdynamics passed nine
expression tests and fifteen field tests respectively. Both packaged crates
were installed together and the three interval regressions passed on Rust
1.95 and 1.99. The identical scalar query fixture passed on native and WASI
with tracing enabled and disabled. Its portable WASI client avoids an
unsupported OS dependency of the Hyperreal development test harness.
Both ASan campaigns completed 10,000 inputs with their follow-up runs.
Stable covered 4,940/4,973 production lines and 519/519 functions. Nightly
covered 987/1,009 branches. Whole-owner coverage remains open.

The full scalar jobs ran 181 direct guest cases on Rust 1.88 and 238 each on
Rust 1.95 and 1.99, plus the required native cases and five fresh guest
replays in every lane. An earlier Rust 1.95 compile hit the 1 GiB limit.
After separate build preparation, the final Rust 1.95 job peaked at
201,060,352 bytes under the same swap-disabled limit with zero limit or OOM
events. These runs qualify warmed jobs on native Linux and WASI. Cold
compilation at that limit and physical embedded execution remain unqualified.

Evidence commit `d4975085f429bf729670e3302f49dce856a01dd2` contains 1,254 files under
`docs/evidence/2026-11-18-interval-periodic-ingress` with digest
`461e5a31caa0aa20d907da11aafbaccc67befaaa837e9e5f3631a2cef0a9ae39`. [Evidence index](docs/evidence-archive.tsv)

## Phases 3, 5 and 14, generated inputs to exponential, logarithm and arctangent, 18 November 2026

Commits `b34e822ad` and `41e99d06e` exercise three public interval operations
with rational coordinates generated by exact line intersection. The tests
check directed 512-bit MPFR endpoints on the 2⁻⁸ grid, exact work cutoffs
and retry, and the logarithm's excluded and partial domains. On [2,3], the
materialized images of log(exp), exp(log) and atan(atan) have grid indices
[511,769], [511,771] and [213,230] respectively. On [−8,−7], exp encloses
its result in [0,1] grid units. Applying log to that published result gives
partial coverage and a lower-unbounded image with upper grid index −1419.
[Composition tests](crates/hyperinterval/tests/monotone_ingress.rs)

The 24 existing release artifacts rebuilt from these revisions and their
predecessor matched byte for byte across Rust 1.95/1.99 and Linux/WASI.
They cover three cost examples and three owning test executables. Moving
the shared line-intersection fixture into a nested Rust module added
5,992 bytes to the Rust 1.95 native periodic test. Keeping the fixture at
its original module scope preserved emitted code and function sizes on all
six compiler/target combinations. File growth was 0–64 bytes.

Two work-budget defects remain reproducible on this production revision.
Each operation reduced a cold 98-bit stored fraction before refusing a
zero-bit budget, leaving all 100 work units untouched. Each also performed
a large cross product to test whether two endpoints were equal before
rejecting an oversized second operand. The source review at `aada22a95`
admits copies, reduction and comparisons before they execute. For near-zero
arctangent it uses both signed endpoint bounds to prove the dyadic cell
directly. With 1,075-bit denominators the signed-cell cases completed at
4,302 work units and failed at 4,301. Attempting the proof with a 1,074-bit
limit also failed. Directed expectations, boundary tests and three
10,000-input sanitizer campaigns are preserved with the trial.

For that review the largest measured stack increases were 96 and 80 bytes
on Linux and 0 and 16 bytes on WASI, respectively, for Rust 1.95 and 1.99.
The corrected operation and complete-ownership measurements found zero
growth in peak or retained requested heap. Earlier ownership rows that
accidentally exercised periodic functions were excluded and rerun with
the three intended functions. The largest native arctangent test grew
5,336 bytes. Four further sharing and comparison variants also missed the
4,096-byte per-target file cap. The closest added 4,424 bytes on Rust 1.95
and 4,432 bytes on 1.99. The source repair remains on
`interval-monotone-source-review`. Only the admitted test workloads enter
the cumulative registers. [Memory accounts](docs/research/interval-memory-accounts.json) ·
[Size accounts](docs/research/interval-size-accounts.json)

The three new cases passed natively, in WASI and against local registry
packages on Rust 1.95 and 1.99. Rust 1.88 exercised the dyadic-only feature
set, which excludes these rational cases. The largest compiled-test job
peaked at 23,658,496 bytes under a 256 MiB swap-disabled cap with a 64 MiB
reserve. Native owner suites and doctests, strict native/WASI Clippy,
rustdoc, workspace formatting and six inventory checks passed. The affected
Hypersolve and Hyperdynamics suites passed nine and fifteen cases. All 213
packaged production source files matched the admitted revision. Stable
production coverage was 4,940/4,973 lines and 519/519 functions. Nightly
covered 987/1,009 branches. Whole-owner coverage remains open.

Measured complete scalar jobs executed 181 guest cases on Rust 1.88 and
241 each on 1.95 and 1.99. The respective native case counts were 6, 9
and 9, with five additional fresh guest replays in each lane. The warmed
jobs peaked at 220,835,840, 199,544,832 and 186,904,576 bytes
under the 1 GiB swap-disabled limit. All limit and OOM counters were zero.
After carrying forward prior positive charges, the smallest remaining
headroom above the 256 MiB reserve was 578,535,424 bytes.
This qualifies the warmed Linux/WASI jobs. Cold builds and physical
embedded targets require separate qualification.

Evidence commit `9ef2682fed5bc11010ff527e835a913b8d49c6d4` contains 1,628 files
under `docs/evidence/2026-11-18-interval-monotone-ingress` with digest
`dd3abae4b695631e74792daa17a60ef3db61c99da54df592bb2efbd6adf1dba2`. [Evidence index](docs/evidence-archive.tsv)

## Phases 4 and 14, strict IEEE WebAssembly release checks, 18 November 2026

On `1fb7b8a1f`, the release scanner reported zero floating operations in an
assembled module containing `f32x4.add` and `i32x4.trunc_sat_f64x2_s_zero`.
Commit `ec92da934` checks folded scalar and SIMD instructions in Binaryen
disassembly. Each qualification run now assembles a control, disassembles the
binary, and checks all four expected arithmetic and conversion instructions.
The control also exercises integer SIMD, same-width bit transfers, and an
export name containing a floating instruction spelling. A separate module
reproduced the former omission for relaxed multiply-add and truncation.
[Scanner](scripts/check_interval_wasm.py) · [Control module](crates/hyperinterval/conformance/wasm-float/codegen-control.wat)

Both deliberately changed clients passed the old 55,802-case arithmetic
corpus. One returned zero for the lower binary32 endpoint of an empty
result. The other returned zero for the upper binary64 endpoint after
division by a zero-containing interval. Commit `eef113658` verifies both
selectors for empty results and each of the five qualified-target errors at
both widths. Each of the 24 probes checks the status on a fresh instance,
then requires a WebAssembly trap. Both changed clients now fail at their
respective selectors. The original endpoints of complete and partially
defined images still pass the independent bit-exact oracle.
[ABI verifier](crates/hyperinterval/conformance/wasm-float/check.mjs)

All six release runs passed on Rust 1.88, 1.95 and 1.99, each on
`wasm32-unknown-unknown` and `wasm32-wasip1`. Each passed 55,802 arithmetic
cases, 8,836 set-property cases and 24 endpoint traps. Chromium 155 passed
the same checks on each bare module. The Preview 1 lattice suites passed
65 additional tests per compiler. Each downstream run also passed 612
floating and 612 fixed layout chains, 172,284 fixed-client calls and 56
dyadic conversions. The exact-real bridge ran on the two compilers meeting
its MSRV. Three Rust 1.95 unqualified-target configurations preserved all
eight refusal/empty endpoint traps apiece.

All 28 applicable linked modules matched their predecessor byte for byte
with the same checkout and target paths. Builds from different checkout
paths produced different hashes and were excluded from that comparison.
The primitive guest reserved at most 704 stack bytes on the normal
completion corpus. In six complete Node verifier jobs, peak host group
memory ranged from 61,337,600 to 64,503,808 bytes under the 256 MiB limit
with 64 MiB reserved and zero OOM or limit events. Host measurements include
startup, corpus loading, fresh instances, result publication and exit. The
existing guest stack and allocation accounts retain their previous charges.
[Per-target host measurements](docs/research/interval-memory-accounts.json)

Evidence commit `6c09f756fd47645778a2e0ee246f7d9cc57519f3` contains 403
files under `docs/evidence/2026-11-18-interval-wasm-float-qualification`,
with digest `0acd1b23c76d615e640fddc016f34c824c8df068101a89efe7fd544f49d88c7d`.
[Evidence index](docs/evidence-archive.tsv)


## Alumina source inventory and shared retiming compatibility, 18 November 2026

The Phase 0 scanner now follows the Interface and firmware modules named in
sections 8 and 9, including nested graph modules. At CSGRS `6a8ca3de3`, a
separate Rust parser found 2,807 function, method, type, struct, trait and
enum declarations across the 36 files. All matched by file, line, kind and
name. The preceding scanner found 617 there. The full inventory grew from
4,148 to 6,338 scan rows, including inline tests. Each of the 278 historical
reviews was preserved and 34 new body reviews document the shared timing
evidence module. Two independent refreshes produced identical bytes and all
seven inventory tests passed. Macro expansion and broader body audits
remain open. All 68 simulator library tests passed on the firmware MSRV, Rust
1.88, including the seven FOC tests. [Inventory](interval-inventory.tsv)

Phase 8 properties P31–P33 and P36–P37 require deterministic machine evidence.
At Interface `fdb7b15`, changing the transcript format version or swapping
the two serialized factor fields passed the existing two-device test.
The earlier implementation generated both sides of its main comparison.
The captured 104-byte outer record commits to a 64,597-byte transcript,
a common dilation of 4,151/4,096, 20 search rounds and 40 participant
preflights. Reading the fixed record independently reproduced these fields
and its SHA-256. The new assertion caught both changes at the expected byte
positions. The original replay, permutation, framing, preflight and global
job checks all ran.

Interface commit `96dd1b4fb4` adds the reference to the
existing test. All 199 native release unit tests passed on Rust 1.95 and
1.99. The exact control graph integration test, formatting, and native and
bare-WASM Clippy checks also passed. The WebAssembly check compiled the test
code. Execution of this specific two-device witness was native.

| Linked x86_64 test harness | Change in final file | Change in executable text | Remaining cumulative file allowance | Remaining cumulative text allowance |
| --- | ---: | ---: | ---: | ---: |
| Rust 1.95 | +320 B | +160 B | 15,632 B | 3,712 B |
| Rust 1.99 | +336 B | +128 B | 15,664 B | 3,744 B |

Every allocated section passed its incremental and cumulative limits.
The fixed-revision comparisons used the same paths and linked profiles
as the predecessor and candidate. The previous Rust 1.95 file and text charges remain on the account.
In a separate temporary layout the linker wrapped RELRO padding from
8 bytes on the fixed revision to 3,712 on the predecessor and 3,416 on
the candidate. That layout failed the cumulative cap. For the recorded
Rust 1.95 release layout the rebuilt fixed executable matched the archived
binary byte for byte. The table reports that release layout and the
separately measured Rust 1.99 layout. The figures measure test-only code.
[Size accounts](docs/research/interval-size-accounts.json)

The fixed, previous and candidate native tests completed 18 fresh memory
scopes under 256 MiB with swap disabled. The largest observed group peak
was 14,991,360 bytes, including the host observer. Every run kept the
64 MiB reserve. The check covers the two-device evidence and its downstream
job through process exit. [Memory accounts](docs/research/interval-memory-accounts.json)

Evidence commit `a50777920934644c60c97d1dba34e7f054d9812f` contains 97 checked members under
`docs/evidence/2026-11-18-interval-machine-source-map`. Its record digest is
`2bef5edd21c91e56e44539d7502dac797867d95980c241e41eda30441607c226`. [Evidence index](docs/evidence-archive.tsv)

## Phase 9 host dq simulator counter boundary, 19 November 2026

At firmware `35d1b0d`, a valid public plant update after transition
`u64::MAX` returned `TraceLength` but changed the d-axis current from one-half
to one-quarter. Firmware commit `56c2c28` stages the two rounded Q30 coordinates
and the incremented count before writing either field. The existing unit-disk
check, per-axis arithmetic and error precedence retain their order. The owner
test exercises the final successful transition from zero current and then two
rejected drives that would move the state in opposite directions. It checks
the typed outcomes and every plant field. The same test failed on the
predecessor. This repairs the individual call used by host clients. The batch
current loop already prechecks the combined transition count.

On Rust 1.88 and 1.99, all 68 simulator unit tests and eight integration
tests passed. Both also passed workspace formatting and simulator rustdoc.
An independent integer recurrence matched all 2,097 ordered records from
81 constructor cases on each compiler, including both coordinates, transition
counts and typed errors. The linked public client also matched an independent
terminal state and checksum after one million accepted and one million rejected
calls. Strict simulator Clippy passed on the firmware CI toolchain, Rust 1.88.
On 1.99, the clean predecessor and the repair both reported the same three
`chunks_exact_to_as_chunks` errors in `http_job.rs` and `motion.rs`.

| Final linked native artifact | File change | Executable text change | Named code change | Remaining cumulative file allowance | Remaining cumulative text allowance |
| --- | ---: | ---: | ---: | ---: | ---: |
| Rust 1.88 simulator tests | +2,464 B | +144 B | +149 B | 13,920 B | 3,952 B |
| Rust 1.99 simulator tests | +3,608 B | +240 B | +254 B | 12,776 B | 3,856 B |
| Rust 1.88 public client | −792 B | 0 B | −2 B | 16,376 B | 2,325 B |
| Rust 1.99 public client | −992 B | 0 B | −2 B | 16,376 B | 2,221 B |

Every allocated section passed the established caps. Both client file accounts
retain their previous eight-byte charge. The first regression-test form added
10,296 and 13,944 bytes to the complete test files. Even the final fixture with
the alternative counter assignment cost 4,160 bytes on Rust 1.99, exceeding
the incremental limit by 64 bytes. The accepted implementation and test fit the
same limits with the release profile's debug information included.
[Size accounts](docs/research/interval-size-accounts.json)

The largest peak across 36 fresh process groups was 12,431,360 bytes under the
256 MiB limit with 64 MiB reserved and swap disabled. All limit-event counters
were zero. In a separate 36-run allocation measurement, each artifact matched
its fixed revision and immediate predecessor in requested peak bytes, total bytes
and allocation counts. Public-client heap peaked at 1,736 bytes on both compilers.
Emitted client frame offsets also matched across revisions. Whole-thread stack,
device RAM, flash and physical cycle timing remain outside this host result.
[Memory accounts](docs/research/interval-memory-accounts.json)

Across the initial 16 warmed pairs per compiler, median elapsed time was
75.97 to 76.29 ms on Rust 1.88 and 78.59 to 78.33 ms on 1.99. A separate
48-pair run balanced execution order on one CPU. Its median elapsed times
were 71.39 to 70.97 ms and 80.89 to 80.45 ms, respectively. These observations
cover process startup, two million public calls, output and exit.

Eleven body reviews now identify the nominal fixed-point owner, its trace
consumer and the distinct enclosing-interval operations. Two refreshes
produced identical inventories with 6,338 scan rows and 323 reviews, including
all 312 previous reviews. All seven scanner tests passed.
[Inventory](interval-inventory.tsv)

Evidence commit `94e38e6a52ad849fe428ef4c625ee2c0539e8a9b` retains 249
members. The indexed record occupies 79,060 bytes and has digest
`a1eebf796402ce94ae95620c73c989143440f2968eb11aa60a1c264a9947aa15`.
[Evidence index](docs/evidence-archive.tsv)

## Phase 0 simulator FOC source review, 19 November 2026

At firmware `56c2c28`, all 40 named declarations in the simulator FOC
module now have body reviews, including four fixture helpers and seven tests.
The independent Rust syntax visitor from evidence commit `a50777920` matched
all 40 inventory entries by line, kind and name. The 29 additional reviews
preserve the previous 323. Two inventory refreshes agreed on all 6,338 scan
rows and 352 reviews. All seven inventory tests passed. [Inventory](interval-inventory.tsv)

The servo plant integrates the rounded Q30 displacement into Q31.32 by
multiplying the displacement bits by four. Its constructor validates the
two response and scale coefficients and copies the supplied initial velocity.
The cascaded controller receives singleton position and velocity intervals
on velocity ticks. The encoder replay derives a raw reading from signed truth
and compares the portable estimator's recovered count against it at every step.

The existing host tests exercised a 12,000-step cascade with 240 position
and 1,200 velocity updates, plus 1,600 encoder steps in each direction. The
counter prechecks in the current and cascaded loop builders establish that
the transition count fits. Their output vectors reserve memory separately,
and admission for arbitrarily large byte capacities remains open. These
reviews concern the simulator. Device timing and whole-board memory still
require separate qualification. [Test evidence](docs/evidence-archive.tsv)

## Phases 4 and 14, rounded IEEE interval powers, 19 November 2026

At `56f976d80`, two independently damaged WebAssembly reactors passed the
entire strict-float corpus. One discarded all but the low four exponent bits.
The other rounded the upper magnitudes down during multiplication. The corpus
tested higher powers on exactly representable small-integer examples. Commit
`c60e2eae1` adds 186 arithmetic cases and 26 property queries and preserves
every preceding record. Both damaged reactors fail the new cases in binary32
and binary64. [Oracle and verifier](crates/hyperinterval/conformance/wasm-float/README.md)

The oracle checks 132 normal higher powers against two independent requirements.
The result must contain the exact mathematical image and both endpoints must
fit the accumulated directed-rounding error. A rational bound on the omitted
binomial terms resolves finite powers of one plus one spacing through
`u32::MAX` in binary64. The binary32 cases include alternating exponent bits
and both neighbors of 2²⁰. An additional 54 cases check unit values, signs,
underflow to adjacent subnormal bounds, and typed overflow at the unsigned
exponent boundary. On each width, all 66 tight projections passed, all 66
whole-format substitutes failed, and 55 actual guest results were wider than
the tight projection.

Rust 1.88, 1.95 and 1.99 each passed all 55,988 arithmetic cases, 8,862
property queries and 24 endpoint-trap checks on both bare WebAssembly and
opt-in WASI Preview 1. The three bare modules passed the same corpus in
Chromium 155. All six primitive modules matched their archived release bytes
and a separate same-path before/after rebuild. Peak reserved guest stack
remained 704 bytes. The lattice and dyadic consumers passed on all six builds,
including 612 floating and 612 fixed chains and 56 directed conversions per
build. Each WASI run passed all 65 applicable lattice tests. The default
WASI feature set still rejected nonempty certified float ingress.

Across 60 complete Node process groups, the highest peak on the expanded
corpus was 65,933,312 bytes. Across six corpus-generation groups, the highest
expanded peak was 100,048,896 bytes. Each workload stayed below its matched
predecessor peak and preserved the 64 MiB reserve within 256 MiB with swap
disabled. Earlier positive charges carry forward. The Node timing record
includes the old and new checker on the original corpus and the new checker
on the expanded corpus. CPU and wall medians varied by ABI. These measurements
concern the verifier, generator and linked guest. The production power kernel
has no edits in this change. [Memory accounts](docs/research/interval-memory-accounts.json)

The final corpus has SHA-256
`c00e935bc7b562a941021c63a3b03e6e37d841d10c986497e952a1652fd2cf3a`.
Evidence commit `bf6a6da794aeed129b0c75b40c6d98c72d2ad3aa` retains 444
verified members in 313,644 bytes with record digest
`69eacbfa08dd1377287e13df5ad1ec64c9d5d85c08891fca43aae176bb789ff9`.
[Evidence index](docs/evidence-archive.tsv)

## Phase 7 source coverage, inequality gradients and scalar underflow

The inventory previously found 241 declarations in the 24 Hypersolve files
named by section 7.1 and their split modules. Matching module families brings
that count to 1,139. A separate Rust syntax visitor found all 1,101 declarations
outside macro token trees. The remaining 38 functions appear by name in the
compiled native test list. The complete scan gained 898 rows. Its 352 prior
hand reviews survived the refresh, and nine new body reviews cover every
declaration in `alpha.rs`. Two independent refreshes matched, and all eight
scanner regressions passed. [Inventory](interval-inventory.tsv)

At `x = −3`, weight 2, Hypersolve's normalized residual for `x >= 0` is
`−2x`. Both symbolic Jacobian APIs returned +2 while a unit forward difference
returned −2. With zero damping, the numerical solver stopped after 32 updates
at weighted residual 12,884,901,888. Encoding the same row as `−x <= 0`
returned −2 and converged in one update. A second witness found that binary64
projection changes the zero sign between exact negative magnitudes 2⁻¹⁰⁷⁵
and 2⁻¹⁰⁷⁶. The public guides describe both limitations.
[Hypersolve](crates/hypersolve/README.md) · [Hyperreal](crates/hyperreal/README.md)

The review branch `interval-symbolic-inequality-orientation` at `769938b87`
retains the combined repair and its three regressions. All three failed on
the predecessor. They pass together with signed weights, sparse and ordinary
columns, exact zero, subnormal and underflow cases. The release client grew
by 112/96 bytes on native and 43/60 bytes on WASI for Rust 1.95/stable.
The largest added native stack use on the common workload was 16 bytes,
within the separately measured allowance of 32 bytes and 0.5% per toolchain
and workload. The outputs and complete allocator counts matched on
the common inputs after holding the process argument name fixed; both guest
stacks shrank by 16 bytes. Four of 72 dispatch windows gained a structural
zero query for an active greater-than-or-equal derivative. All other windows
preserved their counts.

The two actual solver artifacts met their size caps. The Hyperreal
`scalar_micro` benchmark shrank by 96 bytes, but its allocated ELF RELRO
padding grew from 216 to 280 bytes. The incremental allowance was two bytes.
Applying the sign on the early return produced 232 bytes of padding, and
sharing sign application across conversions produced 280. Exact normalization
variants also exceeded WASI code/file caps. The production change remains on
the review branch with no charge to retained production budgets.

The retained GMP coverage fix classifies `reduction_input_bits`, the stored
width of a deferred reduction independent of cache warmth. The existing test
failed on the predecessor. The release executable adds 48 bytes, bringing
the cumulative file charge to 112 bytes and leaving 16,272 bytes. Its text
and named function union match the predecessor.
[Size account](docs/research/interval-size-accounts.json)

## Certified univariate quadratic alpha statuses

For `f(x) = x² + z*x`, with `z = sin(2e) − 2 sin(e) cos(e) = 0`, zero
is a double root. On the preceding source, the strict derivative comparison
returned unknown, yet the report returned `ExactSimpleRoot`. For
`f(x) = x² + x + z + 2⁻⁶⁰⁰`, zero is not a root: the approximate policy
reported `ExactSimpleRoot`. The same double-root failure reproduced with
the angle-addition identity and a power-based Pythagorean identity. The
multiplication-based Pythagorean control produced an exact equality and a
correct multiple-root status.

`bf9d28a5e` uses certified-only comparisons in the original residual,
derivative and bound order. Exact zero residual with unresolved derivative
returns `Undecided` under both policies. Strict inequality in the sufficient
bound proves a positive denominator because its left side is nonnegative.
Equality at the threshold requires separate nonzero derivative evidence.
The near nonroot receives a certified basin bound. The maintained regression
also covers simple roots, multiple roots, failed bounds and a proved zero
derivative away from a root. [Witness](crates/hypersolve/tests/alpha_proof_classification.rs)

The dispatch counts, exact statuses and reducer statistics for 63
ordinary integer-coefficient rows under both policies matched all 126
preceding recording windows. Native and WASI checked the full output
materialization. All 1,541 tests in the final native all-feature Hypersolve
release suite passed. The wider three-crate check on the preceding production
kernel passed 3,241 tests. The eight inventory tests passed, and a fresh
three-repository scan reproduced all 7,236 scan rows and 365 historical
and current body reviews.

On the ordinary 63-row workload each complete native process peaked at
132,082 requested heap bytes. The native stack high-water marks were 9,032
bytes on Rust 1.95 and 9,016 bytes on Rust 1.99. Each WASI module reserved
5,808 guest stack bytes. The measurements agreed across fixed, preceding
and retained builds, for both one and seven repeats. The corrected opaque
case has separate completion measurements. All 28 process groups stayed
within their 256 MiB limits and retained the 64 MiB reserve.
[Memory accounts](docs/research/interval-memory-accounts.json)

The registered release smoke test and certification benchmark grew by 16
and 24 bytes respectively. The public client changed by +32/−136 bytes on
native and +42/+84 bytes on WASI for Rust 1.95/1.99. Revision `9a9c245a8`
authorizes one 16-byte linker alignment unit on the 1.99 native proof client
when the file shrinks and mapped pages, permissions, protected pages and
initialized RELRO payload all match. The previous 1% incremental cap
allowed six bytes. Padding measured 1,424/656/672 bytes at fixed, preceding
and retained revisions. The 16-byte cumulative charge leaves 12 bytes
under the original cumulative cap. A fresh linked build reproduced the
exact SHA-256. Other targets retain their existing budgets.
[Size accounts](docs/research/interval-size-accounts.json)

The locked Alumina Interface release pipeline produced a 10,678,893-byte optimized browser module. Its final file changed by +0 bytes and its code section by +0 bytes.
The optimized module matched the predecessor byte for byte.
Formatting, all-target Clippy with warnings denied, and warnings-denied rustdoc passed for the retained solver change.

Evidence commit `f7b4458706319c4e2008a597e756823492628ade` contains 319 verified members in 180,072 bytes. [Evidence index](docs/evidence-archive.tsv)

### Hypersolve retained-fuzz replay, 19 November 2026

The Phase 0 and Phase 14 adversarial workload at `07f1326a0` stopped on
`bivariate_vertical_components` before the first ranking in both the default
and interval-lattice configurations. The manifest also registered an interval
Jacobian with no replay adapter. Commit `8fa4de4a9` runs the same assertions
in the benchmark and both fuzz binaries. The full benchmark now requires
`interval-lattice`. Its Cargo test invocation exercises one seed for each
registered name. The 67 earlier adapters retain their input mapping, including
the shared point-distance fixtures behind many sketch labels.

The native release campaign sampled 20,000 cases across 69 targets and
measured 100 promoted cases plus the aggregate replay. Each repaired target
had 290 seeds. All 290 vertical-component cases entered the worst 1,000.
Seed 106 was promoted at 4.879 ms best of five. Its unit circle meets repeated
vertical factors at x = −1/2 and −1/4. The highest Jacobian time was
73.931 µs and none of its seeds ranked in the worst 1,000. Rotation preserved
99 previous promoted cases. The complete refresh took 22.790 seconds with
78,704 KiB peak RSS. Its score includes a newly admitted workload and a
different host run. These observations establish a new baseline for replay.
[Current history](crates/hypersolve/slow_performers.txt) and [promotions](crates/hypersolve/promoted_slow_offenders.txt)

Both ASan targets completed 1,000 fuzz iterations and replayed all 290
full-width benchmark inputs each. Release benchmark smoke, the seven
vertical-component and six Jacobian integration tests, two report-generator
tests, formatting and affected Clippy checks passed. A subsequent read-only
rotation consumed the refreshed history and preserved all three report
digests. At identical interval-lattice settings the native benchmark
executable grew from 9,058,800 to 10,026,488 bytes to link the added test
workloads. The changes affect development tooling and its measured reports.
Coverage of the broader interval lexicase matrix remains open.

Evidence commit `989ac011dea07367f384408775d831c1ec73e4eb` records 1,018 verified members in 143,000 bytes.
[Evidence index](docs/evidence-archive.tsv)

### Phase 4 strict IEEE range fuzzing, 19 November 2026

At `a76696da9`, the strict-float fuzz target passed four inputs against a
disposable kernel missing the corner formed by the first lower endpoint and
the second upper endpoint. Its multiplication and division calls used point
operands. The owner already checked 4,000 generated endpoint quartets per
format in a deterministic test. Commit `db62aae75` shares that test's GMP
and directed MPFR oracle with the fuzzer and broadens CI input length to
32 bytes. The original 24-byte point and affine inputs retain their decoding.

Both binary32 and binary64 now detect the missing lower bound of −15 for
[−3, 2] × [−4, 5] and the tight lower rounding of 2/9 for [2, 6] / [3, 9].
The unmodified kernel passed all four replays. Each format also checks 3,025
pairs of ranges spanning signed zeros, subnormals, normal values and finite
extremes. Product and quotient checks compare both endpoint bits and the
zero-divisor or overflow outcome where applicable.
[Shared oracle](crates/hyperinterval/tests/support/float_range_oracle.rs)

The native ASan campaign completed 43,584 iterations in 31 seconds. It also
passed 2,048 independently generated full-width inputs and 64 legacy inputs.
For binary64, the full-width sample included 332 overflowing quotients,
1,040 zero-containing divisors and 673 finite quotients. Three additional
inputs had a nonfinite endpoint. The two formats' fixed and random tests ran
on Rust 1.88 and 1.95. All 45 affected all-feature release tests, both
Clippy checks, formatting and the pedantic workflow scan passed. This change
qualifies the host test and fuzz paths. Packaged library source, manifests
and lockfiles matched the baseline.

Evidence commit `7a4bbb78c51a6f0eafe7082e36950e419f67c621` records 281 verified members in 40,932 bytes.
[Evidence index](docs/evidence-archive.tsv)

### Phase 3 and 5 — charge deferred rational reduction before fractional powers

At `bc8fe3061` the cube root of a generated coordinate representing 3
completed with a 97-bit coefficient limit. Its unreduced representation
requires 98 bits. The result [1, 2] enclosed the root correctly, but reading
the canonical numerator could already have performed the unadmitted reduction.
This violated the input and work contract for nontrivial powers on exact
rational intervals (P5, P6, P36, P38).

Commit `5bd199578` charges three quadratic stages at each reducible
endpoint's stored width before either endpoint can warm its reduction cache.
A shared point pays both charges and can reuse its materialized coefficients.
The two original sign decisions retain their order. The 97-bit call now
returns `SizeExhausted`. Insufficient work for the second endpoint leaves
both caches cold. Larger-budget retry succeeds, and running at the exact
work cutoff produces the same enclosure and domain coverage on warm and cold
inputs. Positive, negative, partial-domain and split powers exercise the
same admission rule. The fuzzer checks generated endpoints against the
independent GMP and MPFR oracle.
[Acceptance witnesses](crates/hyperinterval/tests/fractional_power_ingress.rs)

| Final public client | Linked file change | Code change | Function-byte change | Largest stack growth |
| --- | ---: | ---: | ---: | ---: |
| Rust 1.95 Linux | +152 B | −192 B | −211 B | +80 B |
| Rust 1.95 bare Wasm | +939 B | +729 B | +725 B | 0 B |
| Rust 1.99 Linux | +792 B | +304 B | +281 B | +72 B |
| Rust 1.99 bare Wasm | +654 B | +810 B | +810 B | 0 B |

Each row compares the same consumer to the preceding revision. The size
register carries prior positive charges from the fixed program baseline.
The narrowest remaining cumulative file allowance is 136 B for the Rust
1.99 Linux trigonometry test executable. The firmware selects integer and
fixed backends. The completed client workloads peak at 816 requested heap
bytes on Linux and 424 on Wasm, with zero retained after drop. Peak requested
heap held or fell across all matched jobs. Every measured stack retained
more than 1,046,000 B of usable headroom. Three generated inputs per target
that previously bypassed the 97-bit limit are recorded separately.
[Size accounts](docs/research/interval-size-accounts.json) ·
[Memory accounts](docs/research/interval-memory-accounts.json)

Across the six native workloads, paired median elapsed times ranged from
5.0% lower to 2.1% higher for construction through completion, and from
6.1% lower to 1.6% higher for an already constructed input. On the longer
generated-point run, the Rust 1.95 medians were 1.5% higher through
construction and 2.3% lower on a retained point. Earlier implementations
exceeded the bare-Wasm linked-file or stack caps. These measurements describe
this corpus and do not establish a general speedup.

All 307 native Hyperinterval release tests passed. The affected Hypersolve
and Hyperdynamics suites passed 9 and 10 tests respectively. Rust 1.95 and
1.99 each passed 36 WASI cases, including two fresh guests consuming
native-produced results. The ASan campaign completed 10,000 runs.
Formatting, owner and fuzz Clippy, warnings-denied rustdoc and the
pedantic workflow audit passed. Rust 1.88's reduced CI feature set excludes
the rational provider. The evidence archive contains 415 verified members in
98,268 bytes at `f109536ea6d42fa214b616f4ed07c6cd99cfd4cc`.
[Evidence index](docs/evidence-archive.tsv)

### Phase 14 — verify sibling crates in the target dependency graph

At Alumina Interface `96dd1b4` the coordinated source check accepted a
required Wasm dependency reachable only through a test, a build script, or
an optional feature enabled by tests. Cargo's compiled artifacts confirmed
the difference on all three fixtures. In the deployment-pinned FOC guest,
moving its interval conformance library into development dependencies also
passed the previous check with the same lockfile.

Alumina Interface `abfb182eb` now asks Cargo for the selected target's
normal dependency graph with host procedural macros excluded. The path,
version and source checks still cover all resolved copies of owned packages.
The native and browser applications must both reach the exact measurement
stack. The fuzz and native measurement checks use their respective
`fuzzing` and `trace` features. The FOC mutation now identifies both
missing libraries, and restoring the manifest passes the coordinated check.
These are release-tooling changes for §16 and P36–P37.

All nine target graphs and both license inventories passed on the current
three-repository stack and the deployment pins. All seven lockfiles per
stack matched their committed bytes. The regression suite ran on Cargo
1.88, 1.95 and 1.99. Five paired local runs increased the median duration of
the coordinated dependency audit from 2.31 to
3.73 seconds.

The consumer script also ran the Cortex-M FOC check and 12
native tests on the current stack (12 on the
deployment pins). Each of two fresh Wasm instances checked 825 FOC
transforms and 251 range cases on current source, and 825 transforms and
147 range cases on the pinned source. Each measurement guest checked 4,369
cases. These local results qualify the consumer checks; hosted deployment
and hardware execution remain open.

Evidence commit `fd8b6909ca1aed2c653439fcb6cc27b95eaa546f` records 36 verified
members in 14,680 bytes.
[Evidence index](docs/evidence-archive.tsv)

### Phase 8 — qualify reciprocal travel uncertainty through cached lowering

Alumina Interface `f88370742` adds the missing travel-denominator case
to the machine-profile and lowering corpus in §14.3 (P31–P33, P36–P38).
At `abfb182eb` the old tests accepted division by nominal travel at both
endpoints. They also accepted conversion of the nominal density as the
upper millimetre bound. The new regressions caught all three individual
mutations. For 3,200 microsteps per turn and travel from 7/4000 to
9/4000 metre per turn, the exact density endpoints are 12,800,000/9 and
12,800,000/7 microsteps per metre. The tests check the selected axis,
both units, the separate calibration uncertainty, and the typed cause
when travel reaches or crosses zero.

Across 512 profile cases on two axes, 384 derived exact profiles and
128 failed the positive-travel precondition. Of 36 lowering cases, 10
replayed through the firmware cache, 18 exceeded the requested error
budget, and 8 had invalid travel. All 20 earlier seeds retained their
individual outcomes. In 51 alternating CPU-pinned pairs, the median
end-to-end replay time for the earlier profile corpus changed from
7.08 to 7.19 ms and for lowering from 45.98 to 47.43 ms. These timings
include process startup and measure developer-side replay.

The new checks share the existing profile path and integration executable.
Two earlier layouts exceeded predeclared test-tool size caps. With the
retained layout the release integration ELF grew by 27,040 bytes, the
release replay tool shrank by 1,072 bytes, and the two ASan tools grew
by 10,032 and 10,480 bytes. The per-target size account records the final
file, text, function bytes, remaining allowance and headroom. The ASan
profile campaign reached 4,096 runs and lowering reached 333 at its time
limit. Lifetime peak RSS was 180,948 and 575,520 KiB respectively with
a 1,536 MiB limit per worker. This change affects development tools.
[Size accounts](docs/research/interval-size-accounts.json)

All ten integration tests passed on current source and the coordinated
deployment pins. Both stacks passed the Cortex-M FOC build check. In
each of two fresh Wasm instances, the current and pinned stacks checked
825 transforms and respectively 251 and 147 range cases. Both checked
4,369 measurements per instance. An intermediate run reused one Cargo
target directory across the two source graphs and mixed incompatible
types. Separate target directories qualified the final sources.
Rust 1.95 release tests, strict Clippy, rustdoc and formatting passed.
The machine-profile arithmetic migration and hardware qualification remain
open. Evidence commit `08853958e4c1e8047e6897c830284ae2555f3c60` contains 292 verified members in
56,164 compressed bytes. [Evidence index](docs/evidence-archive.tsv)

### Phase 5 — budget generated rational square-root inputs

Hyperinterval now checks the stored width of each positive rational endpoint
before either endpoint can trigger reduction. On `c5e8b5168`, an exact line
intersection produced the value 3 in a 98-bit stored fraction. With a 97-bit
cap, square root at grid 2⁻⁴ returned [27, 28]/16. On `16733722a`, the same
call returns `SizeExhausted` before reducing either endpoint. A sufficient
budget returns the tight interval. Cold and warm sources pay the same three
quadratic stages per potentially unreduced positive endpoint. Negative-only
inputs and negative lower bounds clipped to zero retain their domain semantics.

The root kernel uses the quotient and remainder from one unsigned division.
The remainder distinguishes a true square from an input just above one.
The regressions check the work boundary, agreement with rational power 1/2,
and both endpoints after squaring the result. Four injected faults were
detected, including early cache warming and ignoring the division remainder.
Both Rust 1.95 and 1.99 passed 309 native owner tests and doctests, strict
Clippy and rustdoc. Both ran the nine generated-ingress tests under WASI and
10,773 independent dyadic grid cases. Five Hypersolve consumer checks and the
Rust 1.88 Cortex-M integer/fixed/dyadic compile passed. The native sanitizer
campaign completed 10,000 square-root runs from 210 seeds, reaching 152 MiB RSS.

A trial using unreduced ratios throughout slowed reused generated inputs.
Another stack layout exceeded the per-call Wasm cap. The retained kernel
added no peak requested heap, allocations or touched stack on matched completed
calls. The largest observed stack was 7,128 bytes native and 1,904 bytes in
bare Wasm. Each compiler reduced the ordinary native client by over 7 KiB
and the bare Wasm client by over 6,100 bytes. The two expanded test programs
grew 9,469–20,792 bytes across native and WASI and fit their individual caps.
Seven alternating campaigns on 32 completed square-root benchmark rows gave
median paired changes of −18.5% and −18.6% across rows. The stable rational
point at grid 2⁻²⁵⁶ was 1.6% slower in that series and 6.8% faster in an
independent nine-pair repeat. [Size accounts](docs/research/interval-size-accounts.json)
and [memory accounts](docs/research/interval-memory-accounts.json).

Evidence commit `54cb5b7ca8beeeed996533e43e2ef5cac3b84e61` contains
797 verified members in 304,660 compressed bytes, including the original
failure, rejected trials, measured final images' hashes and workload-level
accounts. The remaining interval migrations and platform qualification stay
open. [Evidence index](docs/evidence-archive.tsv)

## Phase 14 — execute the current release's required clients

On `c2f658bef`, the release driver returned success when its C smoke
executable was absent and ran a quick start left by an earlier release.
The compiler had produced a known set of programs, but the driver searched
a reusable build directory and silently skipped missing paths.

Commit `66083909e` requests each Rust consumer by name and runs the executable
Cargo reports for that consumer manifest. Both linked C and C++ programs
must also execute. The compiled regression keeps a retired binary on disk,
builds the remaining programs a second time, and checks missing and
foreign-manifest artifacts. The execution checks detected a missing Rust
quick start, a missing C program and a nonexecutable C++ program.
[Release check](scripts/release-check.sh)

The complete local release check passed on Rust 1.95 and 1.99. Each packaged
and verified 21 crates, rendered 21 documentation configurations and passed
all 27 client checks. These included 20 packaged README quick starts, the
user path, the curve-range example, both linked programs and the Python, Go
and TypeScript bindings. An executable left in the reused target directory
would have exited 93 if called. The archive contains all 22 selected Rust
executable hashes and all 21 rendered crate index hashes for each compiler.

All 21 final package archives reproduced byte-for-byte in separate clean
checkouts with each Cargo version. Across the two versions, 14 archive hashes
matched. Three gzip streams differed with identical decompressed tar bytes.
Four packages had only dependency checksum differences in the generated
`Cargo.lock`. Every changed checksum matched the corresponding rebuilt archive.
The selected dependency names and versions and packaged sources matched.

The first offline run lacked the `target-tuple` index entry. Both complete
runs used registry access. Cargo also warned about the existing yanked
`async-executor` 1.14.0 lockfile entry. The documentation builds used workspace
sources and packaging used the documented local Hypervoxel overlay. Hosted
documentation, registry publication and hardware qualification remain open.
[Release conditions](docs/RELEASE.md)

Evidence commit `3bca756ddf4c0b1a4972e9d6e5f1cdc275b33eb9` records 41 verified members
in 39,816 compressed bytes. [Evidence index](docs/evidence-archive.tsv)

## Phase 14 — repair the released Bevy executor dependency

Cargo reproduced the yanked `async-executor` 1.14.0 warning on `eed465b87`
when packaging csgrs 0.23.0. Commit `311f3cca2` updates the workspace,
the three standalone GUI lockfiles and the OpenSCAD corpus worker to
1.14.1. The version and checksum are the only two changed fields in
each lock. Of the 21 coordinated packages, only csgrs includes this
executor, through the optional Bevy feature. Hypervoxel's independent
lock already used an unyanked version.
[Upstream repair](https://github.com/smol-rs/async-executor/releases/tag/v1.14.1)

The full release driver passed on Rust 1.95.0 and 1.99.0. Each toolchain
verified 21 packages, rendered 21 local documentation configurations and
passed all 27 packaged consumer checks. Across the archived dependency
graphs the only version change was the executor. Every changed sibling
checksum matched its rebuilt archive. A separate clean checkout reproduced
all 21 final archives byte-for-byte on each compiler, offline. On 1.95,
the standalone UIs passed 42 Hypercurve, 8 Hypermesh and 6 Hypertri tests,
strict all-target Clippy, and native and Wasm release builds. Each Wasm
binary matched its predecessor byte-for-byte.

The packaged Bevy client checked cube corners, face counts, orientation
and 18 completed scoped tasks. Its executable grew by 50,376 bytes on
Rust 1.95 and 1,536 bytes on Rust 1.99.
The original caps rejected both the patched client and the earlier
unaffected 1.13.3 alternative. Commit `0d37f7b88` records the revised
desktop budget. The native Hypercurve, Hypermesh and Hypertri demos grew
by 2,488, 3,736 and 2,576 bytes respectively. Each executable, code
section and measured function total passed its individual and cumulative
caps against `c9f1f9ac5`.
[Size accounts](docs/research/interval-size-accounts.json)

Across the fixed, preceding and patched builds, cold peak requested
heap was 22,736 bytes and warm peak was 13,552 bytes. A cold run retained
9,184 bytes of one-time state and the warm job returned to its starting
level. Peak calling-thread stack was 15,096 bytes on Rust 1.95 and 15,528
on Rust 1.99. The account includes separate process RSS samples.
[Memory accounts](docs/research/interval-memory-accounts.json)

The randomized paired run found median increases of 0.22–1.19 percent in
the complete short pool, mesh and task workload across two CPUs and two
compilers. Three of four per-lane bootstrap intervals were above zero.
The initial three-version run measured 0.51–0.74 percent. The release
accepts this small complete-job cost for the upstream executor lifetime
repair under the plan's higher-priority correctness rule. The repository's
separate sphere-to-Bevy conversion benchmark measured 165.45 ms before and
166.74 ms after (+0.78%) with identical work, output and checksum
receipts. Per-run medians overlapped. The archive includes all samples
and repeat controls. [Performance protocol](docs/INTERVAL_REFACTOR_PLAN.md#15-benchmark-and-resource-matrix)

The 26 OpenSCAD collector and protocol tests passed. The historical
SynapsCAD revision was unavailable from its public repository. Both
executor locks produced the same 29 compiler diagnostics against public
SynapsCAD `a63c4c95f` and parser `c0d92e6ea`, after matching the local
manifest layout. The worker's end-to-end execution remains open. Hosted
publication and hardware qualification also remain open.

Evidence commit `9037725d9d3ed084ad239a42bac9780c904b0619` records 253 verified
members in 204,436 compressed bytes. [Evidence index](docs/evidence-archive.tsv)

## Phases 4 and 12 — subnormal midpoints in numerical proposals

On `e17866b40`, taking half of each endpoint before adding moved some
midpoints outside their own singleton. Across 89,008 binary32 and 347,056
binary64 input pairs, an independent integer oracle found 4,830 and 4,882
incorrect midpoint results respectively. In RK4, a stationary initial
interval spanning one to three minimum-subnormal units produced a numerical
candidate spanning zero to four. Both failures reproduced in release builds.

Commit `6ee367956` rounds the primitive midpoint once under round-to-nearest
with gradual underflow. RK4 uses the native midpoint for the initial value,
half-width and intermediate time. The stationary example now proposes
exactly the original range. A separate fixture checks all four stage times
and state values. The exact initial set continues into Picard certification
and the next evolution step. Substituting the numerical candidate for the
original request made the replay regression fail.

All 436,064 midpoint pairs passed on both x86-64 Linux and bare Wasm with
Rust 1.88, 1.95 and 1.99. A linked Cortex-M4 image passed a 6,886-case
subset in simulation. The radius-only and combined binary32 clients also
passed. Across five ordinary fields, all 30 RK preparation, proof, replay
and exhaustion workloads kept their dispatch counts and allocation receipts.
The five serialized proof records matched byte-for-byte. The csgrs
successive-operations benchmark's resolved dependency tree includes neither
changed crate.

On two pinned cores, taking the binary32 midpoint and radius together cost
13.5% and 19.8% less time than before. The time-affine rejection with one
unit of graph work, which had blocked a previous repair, took 4.3% less
time on both cores. Two other RK workloads had positive medians on both
cores, ranging from 0.6% to 2.2%. Each paired interval included zero.
The archive includes all samples and repeat controls.

On Rust 1.95 the direct midpoint client shrank by 256 bytes on Linux and
269 bytes on Wasm. On Rust 1.88 it grew by 40 and 4 bytes respectively.
The example RK executable grew by 96 bytes. The Cortex-M4 image used 24
additional flash bytes, leaving 63,040 of 65,536 free. Its measured stack
use fell from 144 to 120 bytes. Native RK4 peaked at 7,832 stack bytes
and 2,580 requested heap bytes, with zero retained after drop. No measured
caller consumed a memory growth allowance. The accounts include the
test executables and carry existing size charges forward.
[Size accounts](docs/research/interval-size-accounts.json) ·
[Memory accounts](docs/research/interval-memory-accounts.json)

The two all-feature, all-target suites passed 296 Hyperinterval and 161
Hyperdynamics tests. Formatting, Clippy with warnings denied, docs,
examples, benches and no-default-feature checks passed for the affected
crates. Both fuzz targets completed 100,000 runs. The strict arithmetic
checks verified 70 emitted symbols and passed 55,988 Wasm arithmetic cases
and 172,284 fixed-lattice calls. The full coverage gate remains open at
5,000/5,033 lines and 10,093/10,546 regions, with 528/528 functions and
unmeasured branches. Physical-board qualification remains open.

Evidence commit `b062d2acb06ac08e4f45bee6434df30d1d615696` contains 1,537
verified members in 881,240 compressed bytes. [Evidence index](docs/evidence-archive.tsv)

## Phase 5 — refinement at one endpoint of a trigonometric interval

On `ace4bf62f`, both trigonometric suites accepted a control that returned
the whole range when the left endpoint's quadrant was known but the right
endpoint's quadrant was still unresolved. Commit `ce8820eb1` adds rational
intervals starting at 1/4 and ending just below or above π/2 and π. Directed
MPFR and independent pole counts check the exact outward grid bounds, the
number of components and domain coverage. Tangent and cotangent also cross
the output grid at one near π/4. The altered tangent implementation reported
partial coverage for a complete interval. Both suites now reject it.

For sine near π/6 and cosine near π/3, the tests now include nondegenerate
intervals from zero to either side of the point whose image is 1/2. They
check both endpoints at input spacings of 2^-64, 2^-256 and 2^-512. Each
interval exhausts one work unit below its measured debit and completes on
exact-budget retry. A separate control that published a full range after
endpoint evaluation failed passed the original sine/cosine suite. The new
fixture detected a lower bound of −1 where the exact grid bound is zero.

Nightly `8006091eb` now covers both outcomes of right-endpoint reduction
in each kernel. Across Hyperinterval, covered branch outcomes rose from
1,004 to 1,006 of 1,027. Rust 1.99 covered 5,002 of 5,033 lines and
10,095 of 10,546 regions, up two in each metric. The 100% gate remains open.
[Trigonometric tests](crates/hyperinterval/tests/trigonometry.rs) ·
[Pole and domain tests](crates/hyperinterval/tests/tangent.rs)

Rust 1.95 passed 296 Hyperinterval all-target tests, 15 doctests, five
minimal tests and 98 downstream Hyperlattice interval tests and doctests.
Formatting, strict Clippy, rustdoc and example/bench checks passed. Rust
1.88 passed the dyadic suites. The optional Hyperreal-backed rational
configuration requires Rust 1.95. Both ASan fuzz targets completed 10,000
inputs with their directed oracles. The dyadic library built for bare
Wasm and Thumb. On WASI Preview 1, all six sine/cosine cases and seven
tangent/cotangent cases passed. The native suite exercised the additional
process-spawning replay fixture.

The native sine/cosine and tangent/cotangent test executables grew by
2,136 and 1,152 bytes. Their WASI modules grew by 2,177 and 854 bytes.
All four met the file, code and function budgets including prior charges.
For the fixed historical WASI comparison, the archived build adds the
GMP cross-compilation feature required by the toolchain. The changes are
confined to test sources. [Size accounts](docs/research/interval-size-accounts.json)

Evidence commit `ce53bad0f62abdd2cf792ccf85742a0fe0008122` records 120 verified members in 882,536 compressed bytes. [Evidence index](docs/evidence-archive.tsv)

## Phase 7 univariate Krawczyk proof eligibility

Hypersolve now requires certified comparisons for each univariate quadratic
Krawczyk row under both public policies. On `6ec893190`, the middle of three
rows on the point box {0} falsely certified a root. Its exact residual was
2⁻²⁰⁴⁸ by the sine addition identity. The retained row returns `Undecided`,
preserves the two neighboring certificates and reports two certified rows.
At radius ½ − 2⁻²⁰⁴⁸ the solver also returns `Undecided`. The exact ½ control
still reports a noncontractive derivative. The rational point takes four
exact comparisons under either policy. Reusing the derivative magnitude for
division avoids cloning the value retained as evidence in the report.
[Univariate tests](crates/hypersolve/tests/krawczyk_univariate_algebraic.rs) ·
[Proof scope and affine exposure](docs/research/interval-box-proof-scope.md)

Thirty ordinary reports and their dispatch counts and all 84 box replay
records matched the preceding implementation. Both bare-Wasm clients passed.
Native Rust 1.95 and 1.99 each passed 1,543 solver tests, the doctest,
all-target Clippy with warnings denied, rustdoc and formatting. Rust 1.95
passed 161 downstream Hyperdynamics tests and six no-default-feature
Krawczyk tests. WASI Preview 1 passed 1,518
direct tests, 17 parent tests with fresh guest children and five native
platform tests. All 4,212 Horner dispatch windows matched. The added
regressions also passed individually on native and WASI under both
compilers.

The 16-row owner benchmark improved by 1.33% and 1.72% on two cores.
Independent interleaved measurements gave improvements of 0.78% and 1.77%.
The six-step downstream feedback model retained every geometry, validation
and allocation field. Using symlinks produced repeatable differences of
opposite sign on two cores. Overwriting the same regular executable inode
before each run gave median wall-time changes of +0.08% and +0.00% in 64
counterbalanced blocks per core. Both 95% intervals included zero. These
measurements cover the completed ordinary workload. The former incorrect
approximate verdicts are recorded separately.

Across 60 ordinary and 12 stress inputs, peak requested heap stayed at 15,036
and 15,436 bytes. Maximum callee stack was 12,824 and 15,208 bytes. Maximum
positive stack increments of 48 and 64 bytes brought the largest cumulative
charges to 120 and 112 bytes, respectively. Minimum thread headroom was
2,071,768 bytes. Glibc usable capacity peaked at 16,584 bytes, up 48 bytes
overall, with a largest per-input increase of 64 bytes. Requested and usable
bytes returned to zero after drop. [Memory accounts](docs/research/interval-memory-accounts.json)

The native full-report and allocator-usable clients shrank by 8 and 176
file bytes. Their alignment padding increased by 16 and 176 bytes without
adding a page to any load or protected segment. The ordinary lifetime client
shrank 184 file bytes. Its padding went from 3,936 to 16 bytes. A third clean
build reproduced identical fixed, previous and retained images. The 3,208
bytes of earlier same-path padding growth remain charged, with zero new
padding allowance. The trace observer traded one executable page for one
RELRO-protected page at unchanged total load pages. The ordinary and stress
bare-Wasm modules grew by 177 and 160 bytes. The univariate test images grew
by 30,981–40,864 bytes across four compiler/target combinations. Sharing the
symbolic input kept the multivariate suite's code and function sizes
unchanged. [Per-target caps and full-image accounts](docs/research/interval-size-accounts.json)

The 42 legacy native builds included 33 exact feature matches. The other
nine belong to upstream Hyperlattice interval targets. The fixed program's
retained-fuzz benchmark predates the two replay workloads added at
`8fa4de4a9`. On the same preceding solver the expanded harness occupied 995,264 more file bytes.
Relinking each benchmark workload separately against the preceding and
repaired solver kept its code and function sizes unchanged. The old and
expanded executables shrank by 64 and 120 file bytes. The archive preserves
both sets of costs and the original cross-workload comparison.

Implementation commit `0783f2ddc`. Affine Krawczyk proof eligibility and
the 32-bit coefficient diagnostic are still open.

Evidence commit `2f8688e31f6e79b0eff61e962539f0699f5e50e7` records 1,935 verified members in 392,796 compressed bytes. [Evidence index](docs/evidence-archive.tsv)

## Phase 7 coefficient denials at the 32-bit reporting ceiling

On `282fec460`, an observed coefficient of 4,294,967,297 bits against a
4,294,967,296-bit ceiling correctly exhausted its scope on WASI. Its error
claimed that 4,294,967,295 bits exceeded 4,294,967,295. In `e5f757ad1` the
message preserves the known inequality and gives the configured limit's
lower bound when both fields reach the reporting ceiling. The regression
covers a limit exactly at the 32-bit maximum, a larger limit and the
largest possible denied `u64` observation. The 512 nested triples also
check the exact value of each independently representable field. The public
fields still saturate on 32-bit targets. A single saturated request can
still appear as the word maximum in the message.
[Public tests](crates/hypersolve/tests/coefficient_bit_budget.rs)

Both Rust 1.95.0 and 1.99.0 passed 1,544 native solver tests and the
doctest, the minimal-feature coefficient suite, all-target all-feature
Clippy, and Rustdoc with warnings denied. Each WASI qualification passed
1,519 direct tests, 17 native parents with fresh guest children, five
native platform cases and 4,212 matched Horner dispatch windows. Seven
Hyperbrep budget tests and the filleted-hyperbola Hypercurve stroke passed.
The six-step feedback benchmark preserved every semantic and allocation
column. Outside the build ID, all loaded native certification-benchmark
sections matched. Its timings varied while the machine was compiling.

Materializing the corrected WASI message used 77 fewer requested bytes
and two fewer reallocations. The warm measurement after the message was
dropped returned to zero. A cold run retained the same 224-byte frame
capacity. The bare-Wasm client grew by 104 file bytes and 43 code bytes on
both compilers. Its warm corrected-message stack fell by 32 bytes.
Two opposing measurement orders on two cores gave 43–49% lower times
for the corrected message, including checksum, destruction and retry.
The independent equal-result controls varied with instance order.
Across 44 registered release images the largest file increase was 144
bytes. The expanded coefficient suite grew by 3,216–5,705 file bytes.
[Size accounts](docs/research/interval-size-accounts.json) ·
[Memory accounts](docs/research/interval-memory-accounts.json)

Four independent source trials preserved the full `u64` counts in the
81-pair public oracle on both compilers and targets. Storing the
unrestricted internal ceiling as `u64::MAX` held the WASI frame heap
constant. The wider receipt still added 520 bytes of stack for 32 nested
unlimited scopes in that trial. Sharing one receipt by failing-frame index
reduced native cost, but slowed WASM. Eliding redundant unlimited child
frames sped up deep nesting, while a single successful WASM scope remained
18–23% slower. The shared denial-loop trial exceeded both WASI solver
unit-test image budgets. The full-width public-field migration remains open.
The trials consume no production allowance. Evidence commit `b2608b12bc368b80246991e645e587ba1b4d4379`.
[Evidence index](docs/evidence-archive.tsv)

## Phase 0 inventory and Phase 11 path proofs

Commits `ba98a61b5` and `6aafc0260` restore the three named Hyperpath
source modules to the scanner and record 53 body reviews. A Rust syntax-tree
walk agreed on all 137 declarations across routing, tangent, solve and affine
projection. The nine scanner tests passed. Two refreshes preserved all 418
manual records. The direct and solver G1 oracle reports 12 incorrect public
outcomes for zero tangents, a hidden endpoint gap, a nonparallel pair and a
small nonzero tangent. [Source review](docs/research/axis-projection-owner.md)

The primary affine planner and independent replay both certified an invalid
negative absolute derivative under `APPROXIMATE_512`. Local trial
`cce4eb9f9` returns `PredicateUnresolved` and keeps the control reports,
dispatch, heap and stack across all 36 native workloads. Seven repairs were
tested. The public-input guard reproduced a slowdown on the Rust 1.95 32×8
approximate workload with compiled, passive tracing. Six alternative source
patches and their independent gate findings are archived. The guard remains
on `interval-axis-proof-qualification`; the production path and dynamics
ownership transfer are open. No trial spent a production allowance.

Commit `b1abbdb4c` restores strict Rust 1.99 Clippy for the retained schedule
equivalence oracle. It consumes complete byte pairs and ignores the same odd
tail and excess edges. Both decoder revisions matched the independent byte
grammar on 2,097,174 inputs, completed 2,500 ASan fuzz runs apiece and
replayed the same 826 distinct generated inputs. Both Rust 1.95 and 1.99
passed 446 all-feature Hyperpath tests and strict all-target Clippy. The two
release suites shrank by 8 and 16 file bytes. The ASan driver shrank by 432.
[Target measurements](docs/research/interval-size-accounts.json)

Raw reports, traces, independent oracles, complete memory histories, linked
sizes and paired timing samples are in evidence commit `1818ef32b8debc1724ae48647bd6562279928ba9`.
[Evidence index](docs/evidence-archive.tsv)

## Phase 8, 11 and 14 schedule replay on WASI Preview 1

Hyperpath `f34587258` selects in-process proptest and single-threaded Criterion
for WASI. The preceding revision failed to compile its test suite in
`wait-timeout`. All 446 test names matched across native and WASI and passed
on Rust 1.95 and 1.99. The independent schedule oracle retained its reversal,
edited-feed and certificate assertions. The packaged example certified feed
2 and rejected feed 4 on both targets. All seven native test, example and
benchmark binaries matched the preceding revision byte for byte on each
compiler. Both guest benchmark images completed all 194 smoke cases.

Alumina Interface `f7086884f` supplies the real wasm-bindgen imports and
executes the three previously native-only actor tests on WASI. All 199 core
tests passed on both targets and compilers, including 18 machine-profile
cases and the frozen digest of the 41,795-byte planner transcript. The stable WASI run also
passed with the firmware and csgrs revisions pinned for deployment. Each
final Interface WASI command grew by less than 32 KiB. With a 1 MiB guest stack, the largest observed
stack use was 62,992 bytes. Peak linear memory remained 15,400,960 bytes for
the complete suite on each compiler. The canonical diagonal workload retained
its stack use and linear memory extent. Source ownership of the path solver,
the previously measured extraction stack excess, and firmware board timing
remain open. [Size accounts](docs/research/interval-size-accounts.json) ·
[Memory accounts](docs/research/interval-memory-accounts.json)

The complete per-case inventories, execution logs, image and archive hashes,
and two-pattern guest stack measurements are in evidence commit `6f209ac271789c4e24ffd312f1ba75df266ef039`.
[Evidence index](docs/evidence-archive.tsv)

## Phase 11 certified tangent decisions and schedule replay

At `6ba224717` the direct path classifier could turn a hidden endpoint gap
into a joined endpoint, a nonparallel pair into a smooth join, and a small
nonzero tangent into a degenerate tangent under the approximate policy.
At `e8b1bef51` each definite label requires certified evidence. A shared
endpoint remains known when its tangent alignment is unresolved. Corner
certification, schedule planning and independent replay preserve the distinction
between an unresolved predicate and proved unsuitable geometry. The ten
rational controls retained their complete dispatch traces under both policies.
[Owner review](docs/research/axis-projection-owner.md)

All 449 Hyperpath checks and 194 benchmark smoke cases passed on Linux and
WASI with Rust 1.95 and 1.99. Both compilers also passed strict Clippy on
both targets, native rustdoc, and a packaged client that certified feed 2
and rejected feed 4. Alumina Interface passed all 199 native and guest tests
on Rust 1.99 before and after, including the canonical planner digest.

Among the seven existing owner commands per platform and compiler, the
largest file increases were 192 B on Linux and 783 B on WASI. Two Rust 1.95
ELF commands used an additional 24 B and 40 B of linker alignment padding.
Their cumulative charges are 40 B and 88 B of a 256 B cap per command,
with identical load and protection page counts and permissions since first
registration. All paired native and guest stack depths matched. Six paired
owner timing runs and eight additional three-run controls per compiler showed
no repeatable slowdown on the measured workloads.
[Binary accounts](docs/research/interval-size-accounts.json) ·
[Memory accounts](docs/research/interval-memory-accounts.json)

The solver still accepts nine oriented zero-tangent cases on the primary
branch. The paired `interval-g1-open-constraint` commits pass the original
twelve-case oracle, exact strict half-line and residual-ball regressions, and
the existing Interface planner digest. Its new public constraint variant
requires coordinated consumer releases and wider solver qualification.
Hyperdrc's 120 s KiCad worker-time gate also remains open. The unchanged
control took 139.4 s, and the isolated revised run took 142.2 s. The sparse
spacing test passed in isolation on both compilers, as did all downstream
integration and documentation tests.

Nine added owner reviews bring the interval inventory to 427 reviews and
7,354 scan rows. Two regenerations produced identical bytes. The 797 archived
sources, manifests, samples and logs were verified from evidence commit
`1d5a51ef247d701490b0faa4eb50b5e011531ac7`. [Evidence index](docs/evidence-archive.tsv)

## Phases 7 and 11, generated tangent models over variable boxes

On `4c4385b48`, a generated axial G1 model produced 36 incorrect whole-box
verdicts in 120 independent cases. For example, target tangent `(2, 0)`
and candidate tangent `(x, 0)` with `x` in `[-1, 3]` have dot products
from −2 to 6. Both quadratic packages could certify orientation using the
center alone. The affine package enclosed this varying row. The strict
residual trial `7433a54bd` corrected the twelve singleton outcomes, leaving
24 incorrect crossing-box results. A new public example on the primary branch
exercises the supported affine enclosure and explains the scope of fallback
rows. Both Rust 1.95 and 1.99 passed the doctests and strict rustdoc build.
[Public box API](crates/hypersolve/src/interval.rs) ·
[Solver proof scope](docs/research/interval-box-proof-scope.md)

The mixed-degree trial independently encloses an affine row using the sum
of absolute source coefficients times their variable radii. It retains the
source constraint index, normalized residual, policy-qualified sign and
aggregate counts. The combined branch `interval-g1-box-composition` at
`c112d08f6` passed all 180 generated G1 box checks and the mixed-degree,
open half-line and hidden-sign regressions on both compilers and targets.
The 12 unchanged owner report/trace comparisons per implementation matched
byte for byte.

A full linked comparison covered 88 existing Hypersolve commands on each of
four compiler/target combinations. The unchanged revision ran 1,543 native
tests. The strict-row, in-place affine and source-dependency trials each ran
1,545. Each trial ran 1,521 direct WASI tests, 17 native parents making 26
fresh guest calls and five native platform cases, and matched all 4,212
Horner dispatch windows. This qualifies the measured trials, including their
new test executable. Release coordination and the binary limits still block
the strict-row migration. Its Rust 1.99 Linux representation test grew by
4,304 file bytes against a 4,096-byte incremental cap.

Source-dependency checks removed the first affine implementation's
256–272-byte stack increase on constant-only controls. With 24 fresh
processes on each side per compiler, maximum positive stack changes across
the 60 registered workloads were 48 bytes on Rust 1.95 and 40 on Rust 1.99.
Every paired requested-heap peak matched. The 12 additional mixed-workload
cells had unchanged measured stack maxima. For the registered Rust 1.95
account, the largest cumulative stack debit in the trial reconstruction was
168 bytes. All 120 cold/warm rows were within their cumulative limits.
The raw account preserves every positive charge and the fixed references.

The source-dependency trial `1bcfa0a2e` still added 8,704 Linux file bytes
and 13,305 WASI file bytes to the Rust 1.95 owner certification benchmark.
The grouped-ball alternative `580b8617e` added 14,136 and 13,493 file bytes
respectively. Neither change was admitted. The public scope documentation
and four reviewed inventory rows are retained. The inventory now contains
431 reviews and 7,354 scan rows. Evidence commit `f3e0c3520bcbbce73fde0b36eb038b68dd093d0d` includes the
independent source oracles, release measurements, full test inventories,
stack accounts, paired timings and trial patches.
[Evidence index](docs/evidence-archive.tsv)

## Phase 7, quadratic certificates over closed rectangles

At `88f2e5d5b`, the maintained conic oracle checks the original polynomial
at corners, on edges and at interior stationary points using exact integer
arithmetic on rational coordinates. It also requires a decision whenever the
documented Taylor ball has zero radius or excludes zero. On the 360-case
deterministic grid, 272 inputs required a decision and 28 had a sign range
that the corners alone missed. Additional fixtures reach an interior zero at
(1/6, 1/2), an edge minimum at (2, −3/5), a stationary line and a sign
change caused by the mixed term. Removing either stationary-point check
failed its analytic control. Omitting the mixed term from the solver's
remainder produced an incorrect sign and failed the retained regression.
[Solver tests](crates/hypersolve/tests/conic_box_certificates.rs) · [Fuzz target](crates/hypersolve/fuzz/README.md#quadratic-residuals-over-rectangles)

Both compilers passed all 764 deterministic report checks on Linux and WASI.
Rust 1.95 passed 1,546 native tests in 73 executables. The complete
WASI qualification executed 1,522 direct tests, 17 native parents with
fresh guest children, 5 host platform checks and 4,212 matching dispatch
windows. Linux ASan completed 10,000 inputs from six checked-in seeds in 13
seconds. The four existing release controls matched their preceding SHA-256
digests in all sixteen comparisons. The five new qualification images fit the
existing 64 MiB capacity and 16 MiB free-space requirement. Clippy, workspace and standalone
formatting, the repository retention check and the pedantic workflow security
audit passed.
[Size accounts](docs/research/interval-size-accounts.json)

Three proposed repairs were measured at the same source and build paths.
The table shows final certification benchmark file changes in bytes against
`dba6a5604`. The corresponding native and WASI screens were 4,096 and
1,024 bytes.

| Trial | Rust 1.95 Linux | Rust 1.95 WASI | Rust 1.99 Linux | Rust 1.99 WASI |
| --- | ---: | ---: | ---: | ---: |
| Shared quadratic row traversal, `f2eae56c3` | +11,376 | −1,021 | −6,264 | −36 |
| Shared ordered quadratic remainder, `8f5150b98` | −1,832 | +6,682 | −8,688 | −511 |
| Fully affine system dispatch, `d28dbeaf5` | +6,112 | +15,488 | −5,696 | +4,629 |

Both mixed-degree implementations passed the two whole-box witnesses on all
four compiler/target combinations. The fully affine dispatch corrected 24 of
the 36 incorrect results across the 120 nonzero-target G1 checks. Its twelve
singleton orientation errors and the sixty checks with a zero target still
failed. Each trial exceeded an existing linked-image cap and remains on its
review branch. No production memory or size allowance was charged.
[Proof scope and zero-coefficient reproduction](docs/research/interval-box-proof-scope.md)

The inventory regenerated twice with 7,354 scan rows and 433 body reviews.
All 750 archived files, including per-image sections, case logs, seeds,
mutations and build commands, were verified from evidence commit
`b30ca5f10c6869c4fbf6d274145cc7c331b48ace`. [Evidence index](docs/evidence-archive.tsv)

## Phase 3 and 4, exact rational intervals to strict IEEE enclosures

At `6cd10d93e`, a caller could project through a common dyadic grid. An
interval from `1/(2^70 + 1)` to `2^70/3` exhausted a 160-bit budget on
the binary64 subnormal grid. At `97c59c69f` both strict float types accept a
`RationalInterval` directly. On each endpoint, the numerator and denominator
bit lengths bound the leading exponent to two adjacent possibilities. The
chosen dyadic grid divides the IEEE spacing on both binades. Rounding on that
grid and then in the same direction to IEEE preserves the tight endpoint.
The mixed-scale interval now fits the 160-bit budget and reproduces the
previous high-budget bounds. Empty, subnormal, overflow, shared-budget
exhaustion and retry have independent witnesses.
[Public conversion](crates/hyperinterval/README.md) · [Regression suite](crates/hyperinterval/tests/rational_float_projection.rs)

The directed GMP/MPFR oracle checked 2,392 interval/width pairs on Linux and
ordinary WASI Preview 1 with Rust 1.95 and 1.99. The Rust 1.95 Linux scalar
and lattice suites passed 392 tests and 20 doctests. The scalar WASI driver
passed 255 guest tests, 9 native platform cases and 5 fresh guest replays.
The opt-out WASI run exercised the typed environment rejection. The two
packaged scalar crates supplied a standalone executable on Linux, bare
WebAssembly and WASI with both compilers. Clippy, rustdoc with warnings denied,
workspace formatting and the allocation-free ARM feature check passed.
The inventory regenerated twice with 7,356 scan rows and 435 body reviews.

For twelve completed native inputs per compiler, including zero, mixed scales
and ratios with 2,201-bit denominators, direct projection reduced peak
requested heap in every case. Its largest peak was 1,432 requested bytes and
1,472 glibc-usable bytes. All twelve released all measured bytes. Cold and
warm stack watermarks never increased over the two-step conversion in the
matched measurements. The largest observed callee watermark was 2,776 bytes
on a guarded 2 MiB worker. These measurements include construction of the
source, projection, materialization of both endpoint bits and cleanup.
On Rust 1.95, median conversion times on prepared inputs ranged from 5.9% to
41.6% of the two-step path. The existing float-chain executable had paired
median time ratios of 0.991 and 0.983. The order and host-cost controls had
identical loaded section addresses and bytes apart from the GNU build ID.
Their observed group medians ranged from 0.987 to 1.063 across paired runs.

The largest new charge across existing size metrics was 34 bytes. Four native
ELF padding accounts had already exceeded their cumulative caps before this
change. Each retained its prior charge and spent zero additional bytes.
Three bare WebAssembly control images matched their previous SHA-256 digests.
The standalone direct client was 7,090 to 12,664 bytes smaller than the
equivalent two-step client across six compiler/target combinations. The
intermediate interval, signed quotient and out-of-line unsigned variants
remain recorded with their separate padding, stack and heap measurements.
The final projection required no memory or firmware allowance.
[Per-target size accounts](docs/research/interval-size-accounts.json) · [Evidence index](docs/evidence-archive.tsv)

All 375 observations verified from local evidence commit
`6373c2840b7ef98935865b5fcff813cbcde20058`. The compressed record occupies
120,164 bytes. [Evidence index](docs/evidence-archive.tsv)

## Phase 14, worked interval examples from the release archives

At `ce6cf85f5`, the Hyperinterval README's Q30 command worked in a
checkout. Cargo reported a missing `fixed_bounds` target when the same
command used the release archive. The published manifest contained only
`proposal_replay` and `real_to_dyadic`. Commit `bbf96fd7f` includes all
three example sources and makes the release check run seven worked targets
from their unpacked owning crates, using the feature flags in their documented
commands. A control that changed the squared lower bound from 2²⁸ to 2²⁷
compiled successfully and failed when the release check executed it.
[Package manifest](crates/hyperinterval/Cargo.toml) · [Release procedure](docs/RELEASE.md)

The seven executables cover eight worked topics in section 17. The proposal at
an offset of 2⁵³ collapsed to zero while exact replay retained [1/4, 3/4].
The fine dyadic grid separated √2 from 181/128. The affine image retained
2x−y = −1, the Krawczyk example checked both contraction bounds, the schedule
replay identified the failed stopping constraint, and the ODE example exercised
certified steps and a complete horizon. In the ninth topic, unchanged Alumina
Firmware `56c2c28` consumed the packaged fixed-only library. Its calibration
example checked the two ADC intervals and the invalid count. The Cortex-M4F
library check passed on Rust 1.88. Board execution and timing remain open.
[Example expectations](#section-17-worked-interval-examples)

Each Linux release run verified 21 packages and their 21 workspace
documentation configurations. All 34 named checks passed on Rust 1.95 and
1.99. The seven new executables produced identical outputs across compilers.
A separate build rendered the documentation of all five example owners from
the unpacked archives on both compilers. All resolved first-party manifests
in those ten builds came from the extracted packages. The hosted docs.rs
build remains a separate release gate.

All 21 archives were byte-identical between two clean checkouts using the
same compiler. Fourteen also matched across compiler versions. Of the seven
remaining cross-compiler differences, four had differences confined to `Cargo.lock` member
contents and three had identical member contents. Against the preceding
revision on each compiler, the Hyperinterval archive grew by 157 bytes to
92,726 bytes in the coordinated 21-package build. The added Q30 source is
705 bytes. The normalized manifest declares the `fixed` feature for its
target. The dependency versions and edges in the packaged locks stayed
the same after accounting for sibling archive checksums.

The packaged Q30 Cargo target also ran on its fixed backend's Rust 1.88
minimum. An external program using the same source and only the `fixed`
feature produced byte-identical native and bare WebAssembly images against
the preceding and retained libraries on Rust 1.88, 1.95 and 1.99. All six
image pairs executed their assertions. This distribution change consumed no
production binary or memory allowance. Both workspace formatting checks
passed and the inventory regenerated with 7,356 scan rows and 435 body
reviews. Publication and the full cross-platform and HIL matrix remain open.

The archived record contains 84 observations in 52,964 bytes. Verification
against local evidence commit `f50729e1ca9d62834ba5d02f1e70910f8b23e6a8`
matched all 258 November interval records to the index, covering 93,284
files and 6,298,330,796 bytes. [Evidence index](docs/evidence-archive.tsv)

## Phase 7, partial affine reports and certified elimination

Commit `f2a30f542` extends the rational two-variable oracle to check the
entire reported prefix against known roots and closed intervals. Of 600
reports across both policies, 240 reject the first coordinate, 180 retain
the first and reject the second, and 180 certify both. Each published step
must carry its original symbol, center, displacement, root and radius.
Removing the first step only on an outside verdict passed the preceding
test and failed the revised one. All four Linux and WASI Preview 1 builds on
Rust 1.95 and 1.99 reproduced both sides of that control.
[Affine oracle](crates/hypersolve/tests/krawczyk_affine_oracle.rs)

The native test files shrank by 2,904 and 2,368 bytes. The WASI files grew
by 1,466 and 1,318 bytes. The largest cumulative file charge was 1,628
bytes. Two builds from different source directories agreed on the final
file, executable code and function byte counts on each target. Each image
retains at least 62.6 million free bytes of its 64 MiB qualification slot.
Minimal-feature and all-feature tests, strict targeted Clippy on both
compilers, workspace formatting, nine inventory tests and the inventory
comparison passed. The retained change modifies only the oracle.
[Per-target size accounts](docs/research/interval-size-accounts.json)

At `6aad6b95f`, terminal equality under `APPROXIMATE_512` still certified
two rootless affine boxes and returned a root that failed an original
equation. A row-swap witness reproduced the wrong outside-box verdict after
a proved first coordinate. Seven repairs passed the 48 adversarial guest
calls, the 240 ordinary guest calls and the 30 ordinary native report and
dispatch comparisons. All 84 whole-box report and trace records matched.
The policy-only change passed the eight measured image budgets and 216
ordinary and stress heap/stack gates. On opposite load orders, the two-core
paired CPU-time runs reproduced an affine slowdown. Returning the owned
right-hand side directly exceeded eight heap caps by 8–144 bytes.
[Proof scope and caller guidance](docs/research/interval-box-proof-scope.md#krawczyk-containment-contraction-and-elimination)

Review commit `6fc8f3deb` on `interval-affine-proof-normalization` uses
certified comparisons and borrows the operands of pivot normalization. It
passed 1,550 native tests and two doctests. The Rust 1.95 WASI suite passed
1,525 direct cases, 17 native parents with fresh WASI children, five native
platform checks and 4,212 matched Horner dispatch windows. All 216 measured
heap and stack comparisons passed. Both guest release clients shrank.
The ordinary native lifetime client also shrank by 1,704 file bytes, but
its uninitialized RELRO padding grew by 1,912 bytes against its zero-byte
incremental allowance. It gained a protected page and lost an executable
page. Other link layouts and the original positive charges are recorded
individually. This repair remains on the review branch. Use
`PredicatePolicy::STRICT` for affine Krawczyk proof on the interval branch.

The variant borrowing both pivot and elimination operands reduced the
measured affine CPU time on both cores in the same-path process comparison.
It exceeded the ordinary guest file cap by 922 bytes, the guest code cap
by 10 bytes, and the native trace benchmark's named-code cap by 714 bytes.
The qualification records include fixed and preceding builds of all 42
registered native artifacts, with 33 feature-matched comparisons, plus the
six-step downstream model on the policy-only and normalization trials.
Production size and memory charges remain at their preceding values.

All 1,774 archived members verified at evidence commit
`91660fccff4acd321079fca31a13d260b9ec579c`. The record occupies
431,268 bytes. [Evidence index](docs/evidence-archive.tsv)

## Phase 4, direct conversion between certified IEEE intervals

At `db9fa026c`, Hyperinterval widens a finite binary32 interval to binary64
exactly and narrows binary64 to the tightest finite binary32 enclosure.
The conversion decodes the source significand and exponent, applies the
target's existing directed rounding, and selects the lower bound for the
first endpoint and the upper for the second. This covers every finite
encoding, including values below the smallest positive binary32 subnormal.
Both APIs preserve empty. Narrowing an interval extending beyond finite
binary32 returns `IntervalError::Overflow`. Only `float-enclosure` is required.
[Public contract](crates/hyperinterval/README.md)

The new calls failed to compile at `dd275ffda`. The retained tests cover every
finite binary32 exponent field and all 16 native rounding and FTZ/DAZ
combinations on x86_64. A mutation rounding the upper endpoint downward
failed both tests. Each of three bare WebAssembly builds and the opt-in
Rust 1.95 WASI client passed 260,948 endpoint or overflow comparisons.
The default WASI build preserved empty and rejected nonempty ingress.
All 417 owner and lattice tests and doctests passed on Rust 1.99. Strict
Clippy, workspace formatting, the Thumb feature check and nine inventory
tests passed. A version-only external client consumed the staged package
offline on Rust 1.88, 1.95 and 1.99. [Acceptance tests](crates/hyperinterval/tests/float_format.rs)

On 65,536 mixed-range round trips, the direct and intermediate-dyadic paths
produced the same endpoint checksum. The direct path made zero allocation
requests. The dyadic path made 1,413,237 requests totaling 31,370,464 bytes.
Median times over five interleaved samples per path on one pinned core were
9.60 ms and 108.27 ms respectively. Each new bare guest reserved and touched
224 bytes of its 1 MiB stack. The existing Rust 1.95 conformance workload
retained its 704-byte stack measurement. Five existing native controls had
no positive size delta. The Rust 1.95 bare client grew by three file bytes
with identical code and function-body sizes, bringing its cumulative charge
to 17 of 4,096 bytes. [Per-target accounts](docs/research/interval-size-accounts.json)

The inventory reproduces 7,359 scan rows and 438 body reviews.
All 73 archived members verified at evidence commit
`cc5e4c0310629943dbfc278ac7cce87ff10fb582`.
[Evidence index](docs/evidence-archive.tsv)

## Phase 7, affine rank and uniqueness

At `e5524c9ca`, the public Hypersolve rank oracle enumerates all 129
singular two-by-two integer matrices with entries in `[-2,2]`. Each has one
consistent and one inconsistent right-hand side. Across the two policies,
100 reports must identify column zero and 416 must identify column one as
the first missing pivot. Each must report two rows, two variables and no
certified coordinates. Changing the index to zero or publishing a unique
root on solver failure passed both older affine suites. The new oracle
caught each mutation. [Integer rank oracle](crates/hypersolve/tests/krawczyk_affine_rank.rs)

A separate symbolic system has two proportional rows and two explicit
solutions in the box. The returned origin solves both original equations.
On the production implementation, 16 of 64 bare WebAssembly calls returned
a false uniqueness certificate. Independent release-mode native execution
reproduced it. Restricting the affine certifier to the certified stages of
its supplied policy corrected all 64 guest verdicts. The cold guest call
sequence reserved 6,704 stack bytes and touched 6,696. The warm sequence
reserved and touched 6,192. Both versions reached 1,310,720 bytes of linear
memory from 1,245,184 at instantiation. [Equations and proof scope](docs/research/interval-box-proof-scope.md#affine-uniqueness-with-dependent-rows)

The new rank test passed all four Rust 1.95 and 1.99 native Linux and
WASI Preview 1 builds. The largest executable was 4,419,552 bytes with
62,689,312 bytes free in its qualification slot. The primary Rust 1.95
release suite passed 1,550 tests including doctests. Its WASI runner passed
1,523 direct tests, 17 native parents with fresh WASI children and five
native platform tests. All 4,212 Horner dispatch windows matched. The
minimal-feature rank test, targeted strict Clippy on both compilers,
formatting, nine inventory tests and the 7,359-row inventory comparison
passed. [Per-target accounts](docs/research/interval-size-accounts.json)

Eleven complete solver trials and two partial repairs were reconsidered.
Returning a boxed right-hand side eliminated the previous eight heap
failures. The ordinary lifetime client grew its padding by 3,600 bytes against
a zero-byte incremental cap. The full-report client added 224 bytes to
480 already charged, exceeding its 512-byte cumulative cap. Sharing an augmented elimination matrix changed ten of 30
ordinary dispatch traces. Borrowing its pivot slices fixed the single
remaining stack failure while preserving that trace difference. Both
partial repairs grew full-report padding by 2,496 bytes. Each complete
trial has separate final-image and complete-lifetime results in the archive.

The policy-only repair at `5ca642d0a` preserved all 30 ordinary reports
and dispatch traces and all 84 whole-box records. All 216 native heap and
stack gates passed. The largest incremental stack charge was 48 bytes.
Both existing bare guests grew by three file bytes and one code byte.
The eight linked image accounts passed and all six native instruments
preserved their load and protected page counts. The review revision
passed 1,554 native tests including doctests and 1,527 direct WASI tests
with the same parent, platform and cross-target dispatch counts. The
six-step downstream chain retained every certified manifold result and
checksum in 32 paired blocks on each of two cores.

All 42 registered native artifacts were rebuilt at the fixed, preceding
and trial revisions. Thirty-three had the registered features. The nine
upstream lattice artifacts with changed feature sets were byte-identical
between preceding and trial. The registered fuzz benchmark gained two
replay workloads after the fixed revision. Rebuilding each suite against
its own reference gave file changes of −16 bytes for the original and
−24 bytes for the expanded suite. Code and function sizes matched for
each pair. The archive preserves the raw cross-workload difference and
both comparable accounts.

The four additional symbolic and dispatch regression cases are carried
in the review branch's affine residual executable. Across the two native
compilers the file grew by 84,392 and 97,040 bytes. Across the two WASI
compilers it grew by 54,588 and 56,550 bytes. These are qualification
harness costs, accounted separately from the production library consumers.

After compilation finished, the six owning-crate Criterion workloads had
lower medians on both pinned cores. The affine workload improved by 1.65%
and 1.64%. Same-address process timing found no resolved change for the
16-row client on either core. In two further opposing-load-order campaigns,
the 16-row client on CPU 17 with the repair loaded first took 1.68% and
0.89% longer than the baseline. The bootstrap 95% intervals were
[0.41%, 1.90%] and [0.13%, 1.66%]. The other load order on that core and
the same-binary controls included zero. Direction varied on CPU 7. The
raw measurements include the earlier owner run that overlapped compilation.
The measured client regression keeps the repair on
`interval-affine-certified-rank` at `5ca642d0a`. The interval branch
retains the integer oracle and its standalone qualification accounts.
Use `PredicatePolicy::STRICT` when consuming an affine uniqueness proof.

All 2,670 members of the 438,244-byte record verified from committed blobs
at evidence revision `7547422348806efb15cf9e05ce245e2d41d56129`.
[Evidence index](docs/evidence-archive.tsv)

## Phases 4 and 14, cross-format IEEE qualification

Commit `c842e8ec0` adds direct binary32/binary64 conversions to the
maintained WebAssembly conformance client. The Fraction oracle checks both
destination endpoints, including one binary64 spacing on each side of every
binary32 normal exponent boundary, subnormal leading-bit positions, sign
changes and overflow. Its 5,587 new queries include 1,047 narrowing overflows.
All 55,988 previous arithmetic cases and 8,862 previous property records
reproduced. The expanded corpus contains 61,575 arithmetic cases and 13,866
property records. The old guest fails when asked for a conversion. The
current verifier also detects a missing conversion corpus.
[Guest and oracle](crates/hyperinterval/conformance/wasm-float/README.md)

Node and Chromium passed the expanded corpus on Rust 1.88, 1.95 and 1.99
for both bare WebAssembly and opt-in WASI Preview 1. Each checked 38
unavailable endpoint traps. Default WASI and both Rust 1.95 threaded
configurations rejected unqualified nonempty input and preserved empty.
The ordinary arithmetic, lattice, dyadic egress and code-generation
checks passed. The qualified scalar guests retained their linear memory
size and their 704-byte complete-corpus peak on a 1 MiB stack. The new
queries alone used 240–256 bytes. Two earlier placements added 32 bytes
to at least one existing workload. Separate conversion exports kept the
original peak on all six targets. The largest qualification file increment
was 2,538 bytes across eleven host-tested WebAssembly configurations.
[Size accounts](docs/research/interval-size-accounts.json) ·
[Memory accounts](docs/research/interval-memory-accounts.json)

Commit `8b0644c6b` checks conversion of full interval hulls in the
native strict-float fuzzer using exact GMP values and directed MPFR bounds.
Both the previous fuzzer and the previous WebAssembly corpus accepted
a narrowed upper endpoint rounded downward and a widened upper endpoint
copied from the lower endpoint. The revised verifiers rejected each mutant.
The native ASan build passed 5,339 generated conversion seeds and 151
retained inputs. An empty-corpus campaign completed 60,310 executions in
31 seconds. The fuzzer grew by 16,760 file bytes and 6,096 text bytes. Five
interleaved process runs per revision and corpus put the largest incremental
RSS charge at 4,517,888 bytes. The 417 native owner/lattice tests and
doctests, strict Clippy, formatting and nine inventory tests passed. The
inventory reproduced 7,359 scan rows and 438 reviews. These are
qualification-client costs. The production library receives no charge.
[Native fuzz target](crates/hyperinterval/fuzz/fuzz_targets/strict_float.rs)

All 721 archived members verified from evidence commit
`4ce56c26832a03b98d38cb786deb429b71e554e8` (233,196 bytes).
[Evidence index](docs/evidence-archive.tsv)

## Phase 3 certified rational point enclosures

At `4ad0c199c`, callers can convert a Hyperreal point directly into a checked
`RationalInterval` with the `hyperreal` feature. Rational points such as
−7/3 retain equal endpoints even at the two extreme i32 precision requests.
For a symbolic point, the requested precision applies to its computable
factor. The exact rational scale determines the final endpoint spacing.
An actively cancelled symbolic source produces `EnclosureUnavailable`.
Callers can clear the signal and retry using the same source.
[Public constructor](crates/hyperinterval/src/rational.rs)

Four tests check exact singleton arithmetic, both signs of √2/3 and its
squared endpoint relations, 15 comparisons against directed 512-bit MPFR
bounds for scaled π, and full dispatch receipts against one scalar enclosure
plus checked endpoint construction. They passed on Rust 1.95 and 1.99 on
x86-64 Linux and WASI Preview 1. Across Hyperinterval, Hyperlattice Interval
and Hyperdynamics, 582 native release tests and doctests passed. The full
Rust 1.99 scalar runner passed 260 cases directly in WASI and nine with
native platform support, including five fresh guest replays. All 450
rational dispatch windows matched. Strict owner Clippy, rustdoc, formatting,
the rational-only build and nine inventory tests passed. The refreshed
inventory has 7,360 scan rows and 439 reviewed entries.
[Registered tests](crates/hyperinterval/tests/real_rational_enclosure.rs)

Across 20 consumer fixtures, replacing manual checked construction with
the public call produced byte-identical Rust 1.95 release executables for
the ordinary, traced and two memory-instrumented native clients, and for
the bare WebAssembly module. Peak requested heap was 1,704 bytes and peak
usable heap was 1,864 bytes. Both returned to zero after drop. The largest
native call used 3,560 stack bytes. The bare guest reserved 1,648 bytes and
touched 1,624 on its 1 MiB stack across cold and warm calls. The largest
new standalone test occupied 3,471,912 bytes of its 16 MiB qualification
slot. Each target has its own charge. The earlier fixed-to-preceding padding
charges on three ball clients persist. The bridge added zero bytes to those
sections. [Size accounts](docs/research/interval-size-accounts.json) ·
[Memory accounts](docs/research/interval-memory-accounts.json)

The closed-ball classifier still uses dyadic displacement bounds. At
`0426a60b6` it returned unresolved on both closed ends of the reflected
ball [1/3, 1]. It also missed a coarse containment proof for ±√2/32 in
[−1/8, 1/8]. Consuming rational bounds directly fixed both cases and
passed the original 1,485 integer comparisons and 207 translated comparisons.
It changed the receipts on all ten existing control cases, added 88 bytes
to the warm reflected lower-endpoint stack and exceeded five linked-section
allowances. A fallback that reused rational operands or a source opposite
zero passed the same geometric witnesses and preserved all ten control
receipts. Four existing warm calls gained 16 stack bytes apiece, exceeding
their individual 1% caps. Both trials still returned a verdict for a
nonzero translated displacement with an active source abort.

Rebuilding the earlier rational-only fallback from `4f05060aa` preserved
the ten control traces and all measured stack peaks. Eight newly decidable
fixtures needed 120–168 additional peak heap bytes. On the current revision,
fresh classification of a symbolic center against itself took 3.35% and
3.36% longer on CPUs 3 and 5, with all 18 pairs slower on each. The original
MPFR test executable also gained 3,936 bytes in its allocated read-only
relocation padding. Both findings are independent of the memory allowance.
All three classifier trials remain outside the production branch.

All 748 members of the 108,288-byte archive verified from committed blobs
at evidence revision `ce92fb4f546d0b8de4770da69479ef6486562700`.
[Evidence index](docs/evidence-archive.tsv)

## Phase 4, strict IEEE enclosures on AArch64 GNU/Linux, 20 November 2026

The opt-in `float-enclosure-aarch64-linux-gnu` feature qualifies strict
binary32/binary64 enclosures on little-endian LP64 `aarch64-unknown-linux-gnu`.
Plain `float-enclosure` on that target still returns `UnqualifiedEnvironment`
for nonempty ingress. The hyperlattice-interval feature forwards it. A new
CI job targets the hosted `ubuntu-24.04-arm` runner. That job has not run.
[Backend gate](crates/hyperinterval/src/float.rs) ·
[FPCR sweep](crates/hyperinterval/tests/support/fpcr.rs)

Without two feature-gated `inline(never)` placements, LLVM vectorized
endpoint sign-mask comparisons and negations into SIMD `FNEG`. This occurred
in hull, intersection, min/max images and binary64 subtraction, producing 13
scan findings. `FNEG` is a floating-point instruction whose NaN handling
depends on `FPCR.AH`. The kernel would therefore no longer be integer-only.
With the placements on `Ord::cmp` and `Format::negate`, the release scanner
found no native floating arithmetic, comparison or conversion in 74 emitted
symbols on Rust 1.88 and 76 on Rust 1.95 and 1.99. The codegen script now
accepts `--target` and `--objdump`.

All checks used user-mode QEMU 10.0.13 with Debian cross binutils 2.44,
GCC 14.2 and glibc 2.41. They did not run on AArch64 hardware. The job's
commands passed on Rust 1.88, 1.95 and 1.99. These covered the target gate,
the directed-MPFR scalar suites, `float_format`, `dyadic_float_egress`,
`rational_float_projection` (1.95 and 1.99), the lattice
cartesian/affine/no-alloc tests and Clippy with `-D warnings`. The scalar
suite sweeps the rounding, flush-to-zero and default-NaN FPCR controls. It
also sweeps the FEAT_AFP FIZ and AH bits wherever they read back. The
75,441-row WebAssembly conformance corpus reproduced byte-for-byte from the
exact Fraction oracle in `conformance/wasm-float/oracle.py`. Every row
matched in all 64 verified FPCR modes under `-cpu max`. It also matched in
the 16 modes available under `-cpu cortex-a53`, which reads both AFP bits
back as zero.

On x86-64, the release stack client had identical section sizes and
identical `.text` bytes before and after the change on all three toolchains.
The disassembly differed only in `.llvm.<hash>` symbol suffixes. AArch64
clients without the feature were byte-identical. The qualified AArch64
client used a peak of 1,264 stack bytes (1,232 on x86-64) and made no
allocations. Its file was 59,184–78,928 bytes larger than the unqualified
client, which had returned an error before computing. No existing image is
charged. The x86-64 `float` test list matched the base revision: 24 tests.
Native fmt, float, lattice and rational suites passed, with and without the
inert feature. So did all-feature Clippy and rustdoc with `-D warnings`,
strict zizmor 1.32.0 and the nine inventory tests. The refreshed inventory
has 7,360 scan rows and 439 reviews.
[Size accounts](docs/research/interval-size-accounts.json) ·
[Memory accounts](docs/research/interval-memory-accounts.json)

All 117 archived members verified from evidence commit
`798cbede6fc47e04f1ed87db6bcfd3c6e99651b0` (433,368 bytes).
[Evidence index](docs/evidence-archive.tsv)

## Phase 7, certified-stage affine Krawczyk proofs merged, 20 November 2026

The policy-only repair from `interval-affine-certified-rank` (`5ca642d0a`)
is now on the interval branch. `certify_affine_krawczyk_box` evaluates pivot
selection, elimination and containment with `policy.certified_only()`.
Before the merge, three of its four retained regressions failed on this
branch under `APPROXIMATE_512`. In those cases a point box was certified
without a proved equality. Two proportional rows were reported unique, and
an unresolved elimination factor reached a verdict after row swaps. All four
pass after the merge. The full all-feature Hypersolve suite passed 1,554
tests across 102 test executables, the count recorded for the review
revision. Strict Clippy, rustdoc, workspace formatting and the nine
inventory tests passed. The refreshed inventory still has 7,360 scan rows
and 439 reviews.

This reverses the earlier hold. The repair was held on one 16-row client
that took 0.89–1.68% longer in one load order on one core. The other order
on that core and the controls included zero, and on the second core the
direction varied. The governing order puts exactness above performance.
The admission rule rejects a lower-rung improvement that brings a
significant higher-rung regression. This repair has the opposite shape. It
fixes false certificates at the cost of a small, inconsistently reproduced
runtime change. Its size and memory results are recorded above: all 216 heap
and stack gates, all eight linked-image accounts, and a three-byte file and
one-byte code growth on each guest. Those results fall under the
correctness provision of the bounded size exception. The timing finding
stays in the record as an accepted cost of the repair. It was not
re-measured, and the WASI suite was not rerun for this merge.

The borrowed-operand normalization trial (`6fc8f3deb`) remains on its review
branch. Its additional changes are cost optimizations, not needed for
correctness, and its RELRO padding finding is unchanged. Callers no
longer need `PredicatePolicy::STRICT` as a workaround for this certifier.
[Proof scope](docs/research/interval-box-proof-scope.md#krawczyk-containment-contraction-and-elimination)

## Outstanding candidates resolved, 21 November 2026

At the user's request every outstanding candidate was evaluated and either
accepted or discarded. Four read-only reviews covered all 67 unmerged review
branches. Each checked the ledger and the tip for landed equivalents. The
rule applied is the governing order. A repair of an exactness,
certainty, completeness or declared-resource defect is accepted when it was
held only on lower-ranked stack, size, padding or timing costs. Those costs
remain as previously recorded. They were not re-measured for this
integration and are accepted as the price of the repair. Optimizations,
refactors and extractions held on measured regressions are discarded.

Accepted on the interval branch, with the defect each repairs:

| Commit | Source | Defect repaired |
| --- | --- | --- |
| `454550ae9` | `9268e4371` | A rebound Jacobian column for `x²` at 3 gave −5 instead of 7 |
| `ac6007f0e` | `21b794649` | Approximate domain answers counted as exact; a negative sqrt operand was `CertifiedValid` |
| `d53bc8720` | `aad7409a5` | Approximate residual receipts became exact satisfaction and `Certified` boxes |
| `4d39b1209` | `e4d27cc62` | Normal equations and BFGS returned successful NaN steps |
| `9905d08d1` | `4e7c83c61` | AABB-sphere decisions dropped approximate axis receipts |
| `b47e0f818` | `200a3ecf9` | Reversed bounds or malformed tensors produced certified voxel envelope facts |
| `128db8e44` | reapplies `3f32da7ae` | Fused `sum_products` published `256 − x²` under a nine-bit limit |
| `7b01fc7ab` | `7f24fbb65` | Voxel occupancy counts above 2^63 wrapped; a full depth-21 subtree reported −1 |
| `8826689de` | `769938b87` | `x ≥ 0` with weight 2 at −3 gave Jacobian +2 instead of −2; negative underflow lost its sign |
| `12a5bcc09` | `cce4eb9f9` | Axis projection certified a negative absolute derivative under `APPROXIMATE_512` |
| `d51d7643b` | `ff3632485` | Zero dyadic comparisons were refused under a zero-bit cap despite sufficient work |
| `358c0036f` | `63e798155` | Finite inclusion across the exponent range was refused as exhausted |
| `8346b343e` | `0988ec1d9` | Refused monotone images still performed unadmitted reduction work |
| `85b155ace` | `4f05060aa` | Closed rational balls were unresolved at exact endpoints |
| `004151550` | `d4816b5f2`, `c799e8acb`, `1abfbc3fc` | A cancelled scope published `Ok(value)` |
| `1fa941889` | `b44486e25`, `88beac671` | Box verdicts came from center-only rows; `x³ = 0` was `Certified` on [−1, 1] |
| `b74ad2851` … `b22df7250` | `interval-g1-box-composition` | A zero tangent passed as an oriented G1 join; 36 of 120 G1 box verdicts were wrong |

Alumina Interface `f3916f3` encodes the new `ConstraintKind::GreaterThan`
as schedule-evidence tag 4. Existing tags and canonical bytes are unchanged.
Every newly reachable witness fails before its repair and passes after it,
where the witness can compile without the new API. API-gated witnesses
rely on the reproductions recorded in their original entries.
`4939d05f2` adds the composed solver-chain contract test. `8b3a38904`
replaces the stale center-only caveat on `certify_interval_box_candidate`.

Declined: `7e881ac12`, abort-signal propagation through rational
translations. The values it affects are correct, and it adds 10.79% to
Hyperreal's generic add/sub path. That is a material hot-path cost without
an exactness gain. Its witness in `real_ball_boundary.rs` is `#[ignore]`
with the reason recorded, so the gap stays visible.

Discarded, all preserved as refs, not deleted:

- Unmerged review branches were moved to `refs/interval-unaccepted/branches/<name>`
  (67 in CSGRS, 2 in Interface, 1 in firmware). Eighteen branches already
  contained in the interval branch were deleted.
- The dynamics extraction stacks, trace-materialization experiments,
  sharing and inlining variants, alternative proofs of accepted repairs and
  the firmware inverse-Clarke halving change remain archived. The halving
  change gives identical values and changes only timing.
- The 21 remaining worktrees with uncommitted state were snapshotted
  byte-for-byte as `refs/interval-unaccepted/worktree-<name>-20261121`.
  Tracked and untracked contents were verified, then the worktrees were
  removed. They were earlier states of the archived path-dynamics extraction
  or Alumina migration scaffolding whose delegations are already on the tips.

Composing the whole-box scope check with the G1 composition exposed one
interaction, which `a3c220ccf` repairs. An axial cross product depends on
the box variable syntactically, but its affine residual has a structurally
zero box radius. Replay skips refining such a row, so its evidence stayed
center-only and valid G1 boxes returned `Unknown`. A row with zero affine
box radius is constant over the box and now covers it. The test is the
same one replay uses to skip refinement.

Local validation on the final revision used Rust 1.99 release builds with
all features. Every listed suite passed:

| Crate | Passed | Ignored |
| --- | --- | --- |
| Hyperreal | 1,400 | — |
| Hyperinterval | 329 | 1 (recorded gap) |
| Hyperlattice Interval | 99 | — |
| Hyperlimit | 384 | — |
| Hypersolve | 1,592 | — |
| Hypersdf | 187 | — |
| Hyperdynamics | 161 | — |
| Hyperpath | 455 | — |
| Hypercircuit | 365 | — |
| Hyperphysics | 84 | — |
| Hypervoxel, owner `--all-features` | 99 | — |
| Hypervoxel, workspace default | 106 | — |
| Alumina Interface workspace | 400 | — |
| Alumina firmware, `cargo +1.88.0 test` | 604 | — |

Strict Clippy passed for every changed CSGRS crate, Hypervoxel, the Interface
native and WASM targets, and firmware on Rust 1.88. Formatting and rustdoc
with `-D warnings` also passed.

Hypercurve passed all but one test. The heap-retention test
`cancelled_stalled_stroke_releases_its_temporary_heap` failed in two of three
runs on the final revision. With the previous `work_budget.rs` restored it
failed in one of three runs, retaining 32,648 of 1,085,328 bytes against a
1% limit. It is a pre-existing, timing-dependent flake. Today's cancellation
check did not introduce it, and it remains open. Under `--all-features`, the
vendored `voxelis` library tests were killed by the host after more than
60 s in set-pattern cases. CI does not run that combination, and no change
touched `voxelis`.

Interface Clippy on stable prints one deprecation warning in firmware's
`alumina-runtime` (`fetch_update` renamed `try_update`). It predates this
work. Hosted CI, coverage, fuzz and size/memory re-measurement of the
accepted repairs were not run in this step.
