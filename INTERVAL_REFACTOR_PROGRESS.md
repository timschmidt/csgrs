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
  remains unavailable; arithmetic implementation has not started.
- Raw local evidence staging: `/tmp/interval-refactor/baseline`. Archive and
  digest registration remain pending; staging alone is not durable evidence.

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
| 1 skeleton/laws | implemented; qualification in progress | fuzz, evidence archive, full API review |
| 2 integer/fixed | not started | exact exhaustive and Alumina differential oracles |
| 3 rational/dyadic/ball | not started | product differential, conversions, independent Real bounds |
| 4 proposals/float | not started | environment qualification and rational oracle |
| 5 elementary functions | not started | MPFR domain/extrema/pole corpus |
| 6 lattice | not started | component oracles and allocation/cost evidence |
| 7 solver | not started | unchanged statuses, certificates, dispatch and costs |
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
  rows retained. Full production coverage baseline is running under an external
  1800-second containment deadline, not an internal work-bound claim.
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
- Full source-map body/consumer audit, baseline coverage/fuzz, matched binary
  and WASM/firmware size measurements, HIL and performance gates remain open.
  No phase-0 completion or downstream migration is claimed.

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

