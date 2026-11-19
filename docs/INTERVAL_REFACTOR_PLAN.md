# Hyperinterval, Hyperlattice Interval Algebra, and Hyperdynamics Refactor Plan

## Governing brief for the implementation agent

This document is the implementation contract for extracting, generalizing, and deploying interval machinery across the Hyper/CSGRS/Alumina stack.

On the development host, keep this program's checkouts, build products,
temporary files and validation runs under `/mnt/development`. Do not use
`/dev/shm` for this work. Retain the required evidence and remove disposable
worktrees, build caches and scratch files when their runs finish.
Check free space before starting a build batch. If overlapping target
directories would use the remaining headroom, run the builds in sequence
and remove each target directory after recording its measurements.

The work is not a greenfield interval experiment. The workspace already contains several strong, independently useful interval and reachability implementations:

- Hyperpath has exact forward/reverse path-speed reachability, exact-stop barriers, bounded component refinement, and independent replay.
- Alumina Interface has exact rational measurement/configuration intervals, conservative physical-parameter propagation, named error budgets, and exact-to-machine lowering.
- Alumina Firmware has an allocation-free fixed-point interval kernel, interval vector transforms, current reconstruction, servo error intervals, and an affine clock set-membership estimator.
- Hypersolve has rational interval kernels, exact `Real`-endpoint enclosures, interval-box certification, Krawczyk/Newton reports, Jacobians, and proof-bearing solver outcomes.
- Hyperreal already exposes certified dyadic point enclosures and must remain the exact point-valued scalar authority.
- Hyperlimit already owns certified decision policy and must remain the authority for irreversible sign/order/topology decisions.

The objective is to recognize these as one validated-computation family without flattening their semantic distinctions or weakening their current guarantees.

---

# 1. Priority order and non-negotiable rules

Retain changes only when they improve, in strict order:

1. exactness;
2. completeness;
3. performance;
4. memory use;
5. binary size;
6. source size.

Every lower-priority improvement is gated by all higher-priority contracts.

Useful functionality and correctness repairs can justify bounded growth under
the [code and binary size protocol](admission.md#bounded-code-and-binary-size-exceptions)
and the [memory and stack protocol](admission.md#bounded-memory-growth-exceptions).
Each applicable target and workload must pass its own incremental and
cumulative limits and retain its required headroom. Cycle-time, exact-output
and evidence gates apply independently. Source reduction alone is insufficient
justification.

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

---

# 2. Target crate structure

## 2.1 Production dependency graph

The intended dependency direction is:

```text
hyperreal
   ▲
   │ optional host bridge
hyperinterval
   ▲
   │
hyperlattice  ───────────────► hyperreal
   ▲
   │
hyperlimit    ───────────────► hyperreal
   ▲                            hyperlattice
   │                            hyperinterval
hypersolve    ───────────────► hyperreal
   ▲                            hyperinterval
   │                            hyperlattice
   │                            hyperlimit
hyperdynamics ──────────────► the four lower layers
   ▲
   │ adapters / static path semantics
hyperpath
```

Domain consumers then depend on the appropriate layers:

```text
hypercurve / hypermesh / hyperbrep / hypersdf / hyperphysics / hypercircuit
csgrs
alumina-interface
alumina-firmware (hyperinterval fixed/integer subset only)
```

The arrow means “depends on.” There must be no cycle from `hyperinterval` back through `hyperlimit` or `hypersolve`.

## 2.2 `hyperinterval`

Create a new crate for scalar set arithmetic and representation conversion.

It owns:

- exact integer ranges;
- exact fixed-point intervals;
- exact rational intervals;
- exact dyadic intervals;
- exact-center balls;
- primitive float proposal types;
- strictly qualified primitive float enclosures;
- explicit empty, unbounded, split, and domain states;
- elementary interval functions;
- directed conversion among interval representations;
- common conformance laws and test corpora.

It does **not** own:

- geometric predicate policy;
- topology decisions;
- solver-specific symbols or report types;
- ODE stepping;
- units or material meaning;
- firmware protocols;
- arbitrary user callbacks claimed to be rigorous without a proof contract.

## 2.3 `hyperlattice`

Add the interval vector/matrix/box layer directly to `hyperlattice`, as requested.

It owns:

- interval vectors;
- interval matrices;
- axis-aligned boxes;
- midpoint/radius forms;
- exact affine images;
- interval matrix/vector products;
- interval norms and matrix measures;
- Jacobian carriers;
- object-level interval facts used for scheduling.

It must depend on `hyperinterval` and continue depending on `hyperreal`.

Do not create a separate production `hyperinterval-lattice` crate unless compile-time or dependency evidence later proves the split necessary. A dev-only conformance helper crate is acceptable.

## 2.4 `hyperdynamics`

Create a new crate for time evolution, reachable sets, and validated scheduling.

Recommended modules:

```text
hyperdynamics::path
hyperdynamics::function
hyperdynamics::ode
hyperdynamics::reach
hyperdynamics::taylor
hyperdynamics::hybrid       // later, behind a feature until complete
hyperdynamics::proposal
hyperdynamics::evidence
```

It owns:

- domain-neutral path-dynamics traits;
- existing exact feed/lookahead/jerk scheduling after extraction;
- enclosed functions and Jacobians;
- validated ODE steps;
- flow tubes;
- end enclosures and covers;
- bounded adaptive refinement;
- proposal-versus-certification orchestration;
- dynamics evidence and replay.

## 2.5 Keep `hyperpath` as the static path-domain crate

Do **not** wholesale rename the current `hyperpath` package at the beginning of this work.

The current crate contains substantial non-dynamic functionality: exact lines/arcs/Béziers, arrangements, offsets, PCB traces/pads/vias, Specctra exchange, CAM pockets, infill, rest material, and static route constraints. Those concepts should not be relabeled as ODE/dynamics infrastructure merely to obtain a broader name.

Adopt this split:

```text
hyperpath
    path geometry
    PCB/CAM/routing semantics
    static arrangements and constraints
    adapters implementing hyperdynamics path traits
    temporary compatibility wrappers/reexports

hyperdynamics
    path speed/acceleration/jerk evolution
    general validated ODE/reachability
```

A repository/package rename may be reconsidered only after the split is complete and a dependency/API review shows that the remaining package names are misleading. Do not make the rename the mechanism by which architecture is discovered.

---

# 3. `hyperinterval` semantic model

## 3.1 Avoid one unconstrained public `Interval<T>`

Do not expose an open trait that allows arbitrary downstream types to claim certified endpoint semantics.

Preferred design:

- share small internal generic helpers;
- expose concrete public interval families;
- seal the traits that confer certified ordering/directed-rounding authority;
- keep production monomorphization and binary growth measurable.

Suggested public families:

```rust
IntegerRange<I>
FixedInterval<I, const FRAC: u32>
RationalInterval
DyadicInterval
RealBall<R = DyadicRadius>
F32ProposalInterval
F64ProposalInterval
F32Enclosure
F64Enclosure
ExtendedInterval<I>
IntervalSet2<I>
```

Names may be adjusted after auditing existing naming conventions, but proof strength must remain visible in the type.

## 3.2 Empty and nonempty states

Never encode an invalid/empty interval by reversing endpoints.

Use an explicit representation, conceptually:

```rust
pub enum Interval<I> {
    Empty,
    NonEmpty(NonEmptyInterval<I>),
}

pub struct NonEmptyInterval<I> {
    lower: I,
    upper: I,
}
```

For hot primitive types, a specialized layout is allowed if it preserves the same public semantics and measurable performance.

Constructors must validate order. Deserialization must rebuild through those constructors.

## 3.3 Domain coverage

Elementary functions need to distinguish the image from whether the source interval lies completely inside the function domain.

Use an explicit result, conceptually:

```rust
pub enum DomainCoverage {
    Complete,
    Partial,
}

pub struct RangeEnclosure<I> {
    pub range: I,
    pub domain: DomainCoverage,
}
```

Return typed errors for an empty real domain, unsupported branch, pole, overflow, precision exhaustion, or unqualified float environment.

Examples:

- `sqrt([-1, 4])` may return `[0, 2]` with `Partial` coverage.
- strict callers may reject `Partial`.
- `log([-1, 0])` is undefined over the real domain.
- `tan` over a pole must not be computed from endpoints alone.

Do not silently clip domains without reporting that clipping occurred.

## 3.4 Sign-set API

Intervals should report possible signs rather than pretend to have scalar order.

Provide a compact sign set:

```text
Negative
Zero
Positive
NegativeOrZero
ZeroOrPositive
Nonzero
Mixed
Empty
```

or an equivalent bit representation.

Hyperlimit may use an interval to prove a scalar sign only when the sign set is a singleton or otherwise excludes the disputed case.

Do not implement an interval `PartialOrd` whose meaning could be confused with scalar ordering. Provide named relations:

- `strictly_left_of`;
- `disjoint`;
- `subset_of`;
- `interior_subset_of`;
- `contains_point`;
- `overlaps`.

## 3.5 Core operations

Every finite exact interval family should support, where mathematically closed:

- checked construction;
- point construction;
- `is_empty`, `is_singleton`;
- lower/upper access;
- hull;
- intersection;
- subset and interior-subset tests;
- point containment;
- width;
- midpoint with explicit rounding policy;
- radius;
- sign set;
- add, subtract, negate;
- multiply using sign partitioning;
- square using zero-aware specialization;
- integer powers;
- reciprocal/division when zero is excluded;
- affine map;
- scale by known-positive/known-negative values;
- directed conversion to stronger/weaker endpoint forms.

For division by a zero-containing denominator, provide two APIs:

1. a strict API returning `ContainsZero` / `RequiresSubdivision`;
2. an extended API returning an explicit split/unbounded image when implemented.

Do not force every consumer to allocate a general interval union. `IntervalSet2` is enough for the first extended reciprocal/division implementation; add a dynamic normalized union only when a concrete consumer needs it.

## 3.6 `IntegerRange`

`IntegerRange` denotes an exact set of discrete integers and may be authoritative for:

- ticks;
- step counts;
- voxel/grid indices;
- queue/storage bounds;
- discrete reachability;
- loop and allocation limits;
- overflow proofs.

It is not the same semantic object as a real interval with integer endpoints. Keep conversion explicit.

Support signed and unsigned primitive widths with checked widening.

## 3.7 `FixedInterval`

Implement exact fixed-point intervals over a known dyadic lattice.

Conceptual value:

```text
stored_integer / 2^FRAC
```

Requirements:

- `no_std`;
- allocation-free;
- `Copy` where the endpoint is `Copy`;
- widened intermediates;
- checked overflow;
- outward floor/ceiling for multiplication and division;
- no saturation unless an explicitly named saturating API is requested;
- exact addition/subtraction when representable;
- strict zero-containing denominator rejection;
- exact width in lattice units;
- deterministic midpoint with named rounding;
- conversions to rational/dyadic host intervals.

Start with the widths actually used by Alumina (`i32` Q2.30 and `i64` Q31.32). Generalize only as far as tests and consumers justify.

The result carrier may need a wider endpoint type than the point carrier. Preserve the existing Alumina pattern where differences of valid endpoints use `i64`/`i128` widened storage rather than overflowing the original lattice.

## 3.8 `RationalInterval`

Use `hyperreal::Rational` under an optional host feature.

Requirements:

- exact endpoint order;
- exact arithmetic;
- sign-specialized multiplication;
- exact monotone positive product/division helpers;
- construction from nominal ± uncertainty;
- exact intersection/hull;
- conversion to dyadic enclosure at caller-selected precision;
- no stored nominal value in the core type.

Domain wrappers such as `ExactInterval` may continue to retain a nominal value next to the shared `RationalInterval`.

Move the optimized rational product-hull logic from `hypersolve::interval` into this layer after exact differential testing.

## 3.9 `DyadicInterval`

Use an exact dyadic representation with a common scale where practical:

```text
[lower_integer, upper_integer] * 2^exponent
```

This should be the main adaptive-computation enclosure form because it aligns with Hyperreal’s requested-precision approximation model.

Requirements:

- exact common-scale normalization;
- cheap coarsening by outward shift;
- exact add/subtract/multiply;
- caller-precision outward division;
- exact conversion to rational endpoints;
- construction from `Real::certified_dyadic_interval` without reinterpreting the result;
- stable precision semantics documented in bits;
- no independent hidden cache that duplicates Hyperreal’s node cache.

If the current Hyperreal public method returns rational dyadic endpoints, validate and convert them. Add only the narrowest neutral Hyperreal bridge required; do not make Hyperreal return a Hyperinterval type.

## 3.10 `RealBall`

Represent a symbolic exact center plus a totally ordered exact radius:

```rust
pub struct RealBall<R> {
    center: Real,
    radius: R,
}
```

The radius must be validated nonnegative and should normally be dyadic or rational.

Initial operations:

- construction;
- point ball;
- center/radius access;
- conversion to dyadic/rational interval at requested precision;
- addition/subtraction;
- exact affine transforms where radius propagation is straightforward;
- containment/exclusion helpers that return unresolved rather than guessing.

Do not rush generic multiplication/division. Those require certified magnitude bounds for exact centers. Implement them only with an explicit bound/certificate input or by producing a dyadic/rational enclosure first.

Use this type to replace duplicated “exact center plus exact radius” payloads where it improves clarity, but retain solver symbols and report metadata in Hypersolve.

## 3.11 Primitive proposal intervals

Add explicit proposal types for fast inexact work:

```rust
F32ProposalInterval
F64ProposalInterval
```

They may support ordinary primitive arithmetic and may be used to propose:

- roots;
- Newton steps;
- ODE trajectories;
- step sizes/orders;
- Jacobians;
- Krawczyk preconditioners;
- subdivision axes;
- likely intersection parameters;
- broad-phase ordering;
- candidate schedules.

They must not expose methods whose names imply certified containment or exclusion.

A proposal may be checked by recomputing a certified enclosure from the exact source. Do not “upgrade” a proposal by merely expanding around the approximate output unless a rigorous error derivation is attached.

## 3.12 Certified primitive float enclosures

Implement `F32Enclosure`/`F64Enclosure` only under a strictly documented backend contract.

Initial certified surface:

- finite construction;
- exact import of finite IEEE values;
- hull/intersection;
- add/subtract;
- multiply;
- divide when zero is excluded;
- reciprocal;
- square;
- integer powers;
- affine forms;
- `next_down` / `next_up` expansion;
- explicit overflow/infinity behavior;
- signed-zero and subnormal tests.

Constraints:

- no ambient rounding-mode changes;
- no `fast-math`;
- no assumption that GPU shader operations are rigorous;
- no automatic certification on targets with unqualified FTZ/DAZ behavior;
- no standard-library transcendental treated as correctly rounded without a documented bound;
- no NaN interval endpoints.

Use compile-time target gates plus runtime/environment self-checks where useful. Unsupported environments return a typed error and fall back to proposal or stronger exact/dyadic paths.

Treat GPU float intervals as proposals until a separate GPU arithmetic contract is proved.

## 3.13 Elementary functions

Implement in this order:

1. `abs`, `min/max image`, integer powers;
2. `sqrt`;
3. `exp`;
4. `log`;
5. `atan`;
6. `sin`/`cos`;
7. rational powers;
8. `tan` and functions with poles;
9. additional scientific functions only when a consumer exists.

Requirements:

- outward enclosure;
- domain coverage reporting;
- monotonicity specialization;
- critical-point/extremum detection;
- periodic range reduction;
- whole-range fallback where appropriate;
- pole detection and split/entire results;
- MPFR/Rug independent oracle tests over boundaries, huge arguments, subnormals, and periodic endpoints;
- no endpoint-only trig implementation.

Use Hyperreal-certified point bounds as a substrate where useful, but remember that applying a point function to interval endpoints is valid only when monotonicity/domain/extrema have been established.

---

# 4. Feature and platform design

Use `MIT OR Apache-2.0` for new crates unless the user directs otherwise.

`hyperinterval` should be `no_std` by default and must not pull arbitrary precision into firmware builds.

Proposed feature shape:

```toml
[features]
default = []
std = ["alloc"]
alloc = []
integer = []
fixed = []
float-proposal = []
float-enclosure = []
bigint = ["alloc", "dep:num-bigint", "dep:num-traits"]
rational = ["std", "dep:hyperreal"]
dyadic = ["bigint"]
hyperreal = ["rational", "dyadic"]
real-ball = ["hyperreal"]
serde = ["dep:serde", "hyperreal?/serde"]
dispatch-trace = ["hyperreal?/dispatch-trace"]
```

Exact names may be refined, but preserve these properties:

- firmware can enable only `integer` and `fixed`;
- host exact arithmetic can enable `rational`, `dyadic`, and `real-ball`;
- float proposal code can compile without Hyperreal;
- certified float code can be disabled independently;
- no default feature silently adds `std`, allocation, serialization, or float certification;
- production dependencies remain minimal;
- oracle libraries remain dev-dependencies only.

Set the MSRV no higher than the lowest actively supported stack requirement unless a measured capability requires it. Current Alumina Firmware compatibility is a hard gate.

---

# 5. Hyperlattice interval algebra

## 5.1 New types

Add concrete interval carriers with fixed-size storage:

```rust
IntervalVector<I, const N: usize>
IntervalMatrix<I, const R: usize, const C: usize>
BoxN<I, const N: usize>
MidpointRadiusVector<C, R, const N: usize>
MidpointRadiusMatrix<C, R, const RWS: usize, const CLS: usize>
```

Use structure-of-arrays layouts for primitive endpoints when measurements show a benefit. Do not force the exact-rational/dyadic path into a SIMD-oriented layout that increases allocation or cloning.

## 5.2 Required operations

Implement:

- per-lane construction and validation;
- point-vector/point-matrix lifting;
- hull/intersection;
- subset/interior subset;
- box width and widest-axis selection;
- midpoint and radius decomposition;
- exact affine image of a box;
- point matrix × interval vector;
- interval matrix × point vector;
- interval matrix × interval vector;
- interval matrix multiplication;
- transpose;
- row/column extraction;
- infinity norm / one norm;
- interval norm bounds;
- logarithmic norms / matrix measures needed by reachability;
- diagonal-dominance and contraction helpers;
- Krawczyk support primitives;
- Jacobian enclosure carriers.

Keep solver certification reports out of Hyperlattice.

## 5.3 Object facts

Retain immutable derived facts when they demonstrably save work:

- zero-containing lane mask;
- sign-stable lane mask;
- singleton lane mask;
- widest lane;
- common dyadic scale;
- zero rows/columns;
- diagonal/triangular/sparse structure;
- maximum component width;
- known-positive radius lanes;
- norm bounds.

Facts may schedule algorithms but never substitute for a proof they do not contain.

Serialized values must validate/rebuild facts rather than trusting stale cached metadata.

## 5.4 Specialized kernels

Port and generalize only after equivalence tests:

- Hypersolve’s sign-partitioned rational product hull;
- Alumina FOC’s widened fixed-point products/divisions;
- FOC Clarke/Park transforms as conformance/benchmark clients;
- clock affine-image/intersection patterns;
- exact positive product/division fast paths from machine profiles.

Avoid a generic trait matrix so broad that compile time, binary size, and API complexity exceed the saved source.

---

# 6. Integration into Hyperlimit

Hyperlimit remains the decision authority.

Extend the predicate cascade only after profiling and with traceable stages:

```text
exact identity/rational/common-scale fact
→ exact fixed/integer enclosure
→ qualified f32/f64 certified enclosure
→ rational/dyadic enclosure
→ certified Real refinement
→ explicit Unknown or policy-authorized terminal result
```

Rules:

- Ordinary float proposals may prioritize/refine but never decide.
- A certified interval may decide only by exclusion/containment, for example `upper < 0`, `lower > 0`, or disjoint boxes.
- Interval overlap remains inconclusive.
- No interval containing zero produces `Zero`.
- Existing `PredicateOutcome`, certainty, and policy semantics must remain unchanged unless a versioned public change is explicitly approved.
- Add dispatch-trace events for every new filter stage.
- New filters are disabled or bypassed when they add cost without improving the workload.

Benchmark on representative Hypertri, Hypercurve, Hypersolve, and Hypermesh predicates, not scalar microbenchmarks alone.

---

# 7. Integration into Hypersolve

## 7.1 Inventory and disposition

Audit at least:

```text
hypersolve/src/interval.rs
hypersolve/src/real_interval.rs
hypersolve/src/jacobian.rs
hypersolve/src/linalg.rs
hypersolve/src/alpha.rs
hypersolve/src/root_isolation.rs
hypersolve/src/algebraic*.rs
hypersolve/src/certification.rs
hypersolve/src/residual_replay.rs
```

Classify every interval-like type/function as:

1. general scalar interval arithmetic — move to `hyperinterval`;
2. interval vector/matrix arithmetic — move to `hyperlattice`;
3. exact-`Real` proof adapter — retain or narrow in Hypersolve;
4. solver symbol/domain wrapper — retain;
5. report/certificate/provenance — retain;
6. duplicated logic removable after adapter equivalence.

## 7.2 Expected moves

Likely move/generalize:

- rational product hull;
- rational interval arithmetic;
- dyadic square-root endpoint helpers;
- common interval hull/intersection/sign utilities;
- matrix/vector interval arithmetic;
- generic box containment and width operations.

Likely retain in Hypersolve:

- `VariableBall` because it owns `SymbolId`;
- `IntervalBoxCertificationPackage`;
- all certification statuses/reports;
- active-row semantics;
- candidate context binding;
- affine/quadratic package selection;
- Krawczyk uniqueness and residual replay;
- algebraic-root identity and isolator provenance;
- policy-aware exact `Real` endpoint adapters.

## 7.3 `RealInterval` transition

Do not immediately move `RealInterval` into Hyperinterval.

It currently relies on strict exact comparisons through Hyperlimit. Moving it would create the wrong dependency direction.

Instead:

- replace cases that only need exact rational/dyadic bounds with `RationalInterval` or `DyadicInterval`;
- replace center-plus-radius cases with `RealBall`;
- retain a narrowly scoped `RealInterval` adapter for symbolic endpoint cases;
- measure whether exact symbolic endpoints are actually faster/tighter than dyadic enclosures;
- delete it only when all consumers have stronger replacements.

## 7.4 Migration method

For each migrated path:

- retain the old implementation in a test-only differential module;
- run old and new against identical generated and retained-fuzz corpora;
- require identical status, bounds, certificates, and failure reasons where public behavior is intended to remain unchanged;
- preserve serialization and report field semantics;
- benchmark CPU, allocation calls/bytes, peak memory, binary size, and source size;
- remove the legacy implementation only after the new path passes all gates.

---

# 8. Integration into Alumina Interface

Audit at least:

```text
crates/alumina-interface-core/src/boundary.rs
crates/alumina-interface-core/src/machine_profile.rs
crates/alumina-interface-core/src/motion_schedule.rs
crates/alumina-interface-core/src/compiler.rs
crates/alumina-interface-core/src/direct_motion.rs
crates/alumina-interface-core/src/servo_motion.rs
crates/alumina-interface-core/src/shared_timing_evidence.rs
crates/alumina-interface-core/src/graph/*
```

## 8.1 Preserve domain types

Keep:

- `ExactValue<U>`;
- `BoundedMeasurement<U>`;
- `ExactInterval` or a more domain-specific renamed wrapper;
- canonical step/cycle types;
- display-only projection types;
- named evidence reports.

Change their internals to use `RationalInterval` where this removes duplicate arithmetic.

Do not turn measurement intervals into exact design values or make display floats round-trip into authority.

## 8.2 Machine profiles

Replace manual endpoint formulas only after field-for-field differential validation.

Preserve:

- positive-lower preconditions before monotone multiplication/division;
- usable travel contraction;
- conservative lower velocity/acceleration/jerk limits;
- conservative upper command density/following error;
- exact unit typing;
- complete configuration/capability identity.

Add helper APIs such as `positive_product`, `positive_quotient`, and conservative endpoint selection rather than repeating endpoint expressions.

## 8.3 Error budgets

Keep named error sources separate in reports. Use shared interval/radius arithmetic to compose them, but do not replace the evidence with one anonymous total.

Existing canonical fixture bytes, content-addressed digests, and imported/replayed evidence must remain byte-identical unless a deliberate versioned encoding change is approved.

## 8.4 WASM and UI

Measure:

- WASM binary size;
- compile time;
- browser memory;
- interactive schedule latency;
- serialization size;
- exact/display boundary behavior.

Do not pull arbitrary-precision interval machinery into browser paths that only need fixed/rational facts unless profiles justify it.

---

# 9. Integration into Alumina Firmware

Audit at least:

```text
crates/alumina-foc/src/lib.rs
crates/alumina-foc/src/angle.rs
crates/alumina-foc/src/current.rs
crates/alumina-foc/src/servo.rs
crates/alumina-foc/src/pwm.rs
crates/alumina-clock/src/lib.rs
crates/alumina-sim/src/foc.rs
```

## 9.1 Firmware dependency contract

Firmware may depend on:

```toml
hyperinterval = { default-features = false, features = ["integer", "fixed"] }
```

It must not acquire:

- `std`;
- heap allocation;
- `hyperreal`;
- arbitrary precision;
- serialization not already required;
- new runtime global state.

## 9.2 `Q30Interval`

Do not delete the public/domain `Q30Interval` immediately.

First make it delegate to or wrap `FixedInterval<i32, 30>` while preserving:

- layout where relied upon;
- exact constants;
- checked errors;
- current method names;
- outward rounding;
- cycle counts;
- no-alloc behavior;
- HIL expectations.

If a type alias is eventually smaller and equally clear, adopt it only after API/layout review.

## 9.3 Servo intervals

Use the generic fixed interval where it fits, but retain widened position/velocity error carriers when their endpoint range exceeds the point type.

Do not narrow an `i128` error proof merely to reuse a generic type.

## 9.4 FOC transforms

Keep `Phase3`, `AlphaBeta`, `DqInterval`, `Rotation`, `Duty3`, current calibration, and modulation as domain types.

Use them as high-value conformance tests for:

- interval matrix/vector transforms;
- outward multiplication;
- norm enclosure;
- range admission;
- overmodulation rejection.

## 9.5 Clock estimator

The clock estimator is an affine set-membership observer. Generalize only reusable arithmetic:

- integer/rational range intersection;
- affine image/inverse image;
- outward integer division;
- uncertainty-width admission.

Keep boot identity, causal ordering, freshness, health, and scheduling policy in `alumina-clock`.

Require identical estimates for the complete existing heartbeat corpus.

## 9.6 Embedded performance gates

Measure:

- code size per crate and final firmware;
- stack usage;
- cycle counts for interval operations and control updates;
- register pressure where observable;
- no allocation;
- deterministic output bytes;
- HIL traces.

No hot firmware path may regress materially without an explicit higher-priority correctness/completeness gain and user-approved acceptance note.

## 9.7 Goal-authorized bounded code and binary growth

The user authorizes bounded size growth for useful functionality and explicitly
permits reconsideration of previously rejected refactors. Apply the normative
[bounded exception protocol](admission.md#bounded-code-and-binary-size-exceptions):
per-target absolute-byte and percentage caps, final-artifact measurements,
firmware flash headroom and cumulative accounting are required. All other gates
remain intact. Record each candidate's benefit and fresh qualification in the
execution ledger, preserving historical dispositions. This rule alone assigns
no numerical caps and qualifies no implementation.

## 9.8 Goal-authorized bounded memory growth

The user also permits small, bounded memory growth around new functionality
and reconsideration of approaches previously rejected on memory or stack.
Apply the separate [memory exception protocol](admission.md#bounded-memory-growth-exceptions):
declare per-target temporary/retained heap and stack caps, measure complete
consumer lifetimes, preserve deployment headroom and account cumulatively
against a fixed baseline. Correctness repairs qualify as useful functionality.
Runtime, exactness, certainty, provenance, dispatch order, no-alloc firmware,
hard resource limits and required validation remain unchanged. Preserve earlier
rejection evidence and reassess candidates on the current target revision.
This authorization assigns no numerical allowance and admits no candidate.

---

# 10. Hyperdynamics extraction and implementation

## 10.1 First extract the existing specialized path solver

Move the dynamic subset of Hyperpath, not its static geometry.

Initial source candidates:

```text
hyperpath/src/routing/axis_projection.rs
hyperpath/src/routing/feed.rs
hyperpath/src/routing/lookahead.rs
hyperpath/src/routing/jerk_schedule.rs
```

Also inspect adjacent report/error/types in `routing.rs`, `tangent.rs`, `solve.rs`, and public reexports.

## 10.2 Refactor before moving

Avoid a dependency cycle by first making the algorithms domain-neutral.

Define traits/types in Hyperdynamics such as:

```rust
trait MetricSpan {
    fn exact_length(&self) -> Result<Real, DynamicsError>;
    fn dynamics_class(&self) -> SpanDynamicsClass;
}

trait JoinEvidence {
    fn join_class(&self) -> CertifiedJoinClass;
    fn retained_radius(&self) -> Option<&Real>;
}
```

Exact names may differ. The requirements are:

- Hyperdynamics depends only on lower mathematical crates.
- Hyperpath implements the traits for its concrete line/arc/path types.
- Hyperpath may retain legacy wrapper/report types that delegate to Hyperdynamics during migration.
- No generic dynamics layer approximates an exact path merely to fit the trait.

## 10.3 Preserve the specialized algorithm

The existing path solver remains a specialized fast path:

- squared-speed forward reachability;
- reverse deceleration reachability;
- exact caller/global/corner ceilings;
- exact G1/reversal/stop barriers;
- exact positive-component partitioning;
- bounded halving for jerk feasibility;
- independent Hypersolve replay;
- retained bottlenecks and transition evidence.

Do not replace it with a generic interval ODE solver. The generic solver is a fallback for systems without a stronger closed-form recurrence.

## 10.4 Compatibility

For at least one coordinated minor release:

- retain old Hyperpath entry points as deprecated wrappers;
- preserve existing result semantics;
- update examples and Alumina consumers to import Hyperdynamics directly;
- add compile tests for old paths;
- document the migration;
- do not silently change serialized/evidence forms.

## 10.5 General enclosed functions

Add an owned/replayable function interface, not an unchecked closure claimed rigorous.

Conceptual contract:

```rust
trait EnclosedVectorField<const N: usize> {
    type Interval;
    type Identity;

    fn identity(&self) -> Self::Identity;
    fn image(
        &self,
        time: &Self::Interval,
        state: &BoxN<Self::Interval, N>,
        policy: &EnclosurePolicy,
    ) -> Result<IntervalVector<Self::Interval, N>, DynamicsError>;

    fn jacobian(
        &self,
        time: &Self::Interval,
        state: &BoxN<Self::Interval, N>,
        policy: &EnclosurePolicy,
    ) -> Result<IntervalMatrix<Self::Interval, N, N>, DynamicsError>;
}
```

Initial implementation should adapt Hypersolve expressions and symbolic derivatives so function identity, variables, and replay are explicit.

## 10.6 Validated ODE baseline

Implement the smallest complete validated path first:

1. exact constant/affine vector fields;
2. rational/dyadic time intervals;
3. interval Euler enclosure;
4. Picard self-map existence enclosure;
5. Jacobian/Lipschitz or logarithmic-norm uniqueness evidence;
6. bounded dyadic step halving;
7. exact end enclosure;
8. complete evidence report.

A step report should retain:

- vector-field identity;
- input time/state box;
- candidate/proposal source;
- full trajectory enclosure;
- end enclosure;
- Jacobian enclosure;
- existence/uniqueness evidence;
- rounding and truncation bounds;
- step/refinement policy;
- every rejected attempt;
- final status: certified, unknown, exhausted, domain error.

## 10.7 Taylor and wrapping control

After the baseline is sound and measured, add:

- demanded coefficient generation;
- sparse monomial support;
- interval/Taylor series with certified remainder;
- Taylor models with exact `Real` or rational coefficients and dyadic/rational remainder;
- centered/affine forms;
- coordinate transforms;
- QR-style wrapping reduction where justified;
- independent noise symbols and sensitivity-based retention;
- algorithm portfolios that select among already-sound enclosures by approximate volume/cost.

Never discard a term without charging its contribution to the remainder.

## 10.8 Reachability

Add:

```text
FlowTubeSegment
FlowTube
EndEnclosure
EndCover
ReachabilityPolicy
ReachabilityStatus
```

Support:

- uncertain initial boxes;
- finite time horizon;
- certified adaptive subdivision;
- finite end covers;
- epsilon-thickening contracts;
- final contribution checking;
- positive progress;
- exact horizon capping;
- deterministic evidence ordering.

## 10.9 Hybrid dynamics later

Hybrid guards/resets/events require replayable semidecision and crossing evidence. Keep this behind a feature until the contract is complete.

Do not treat Hyperlimit’s terminal `Unknown` as a persistent refinable proposition. Introduce a dedicated semidecision object only when a concrete continuous/hybrid consumer and convergence proof exist.

---

# 11. Effective use throughout the stack

Adopt in this order so the new layer is exercised by real consumers early.

## Tier 1: direct source owners

1. Hypersolve interval/Krawczyk paths.
2. Alumina Interface rational uncertainty and error budgets.
3. Alumina Firmware fixed intervals and clock observer.
4. Hyperlattice vectors/matrices/boxes.

## Tier 2: predicate and curve performance

### Hyperlimit

- exact/fixed/dyadic/qualified-float filters;
- interval sign/exclusion stages;
- traceable escalation.

### Hypercurve

Use intervals for scheduling and exclusion:

- parameter boxes;
- native subcurve bounds;
- derivative ranges;
- monotonicity checks;
- interval Newton/Krawczyk refinement;
- fast disjointness rejection;
- certified subdivision priorities;
- curve/surface range bounds.

Exact replay and CurveRegion topology remain authoritative.

## Tier 3: geometry and fields

### Hypertri / Hypermesh

- interval AABB/broad-phase filters;
- exact fixed-grid filters;
- box transforms;
- primitive proposal ordering;
- no topological decision from an uncertified box.

### Hyperbrep

- surface parameter boxes;
- patch range/Jacobian enclosures;
- surface-surface intersection scheduling;
- trimming and singularity isolation.

### Hypersdf / F-rep

- whole-cell positive/negative classification;
- Lipschitz bounds;
- conservative occupancy;
- adaptive subdivision with explicit unresolved cells.

### Hypervoxel

- conservative cell occupancy and transformed bounds.

## Tier 4: physical domains

### Hyperphysics

- interval-valued material properties;
- uncertain initial states;
- certified small-step reports;
- collision/time-of-impact proposals followed by exact/interval replay.

### Hypercircuit

- component tolerance intervals;
- interval residual/Jacobian systems;
- operating-region proofs;
- transient reachable envelopes.

### Hypergraphics

- display-only enclosure projections and diagnostic uncertainty; never reverse-authoritative.

## Tier 5: CSGRS and applications

- consume improved lower-layer bounds through existing geometry APIs;
- avoid a direct interval dependency where domain crates already expose certified reports;
- keep compile-time backend selection intact.

---

# 12. Source extraction map

Create a machine-readable inventory before implementation. At minimum include these seeds:

| Source | Existing role | Disposition |
|---|---|---|
| `hypersolve/src/interval.rs` | rational product hull, candidate balls, interval-box/Krawczyk certification | move only generic arithmetic; retain solver reports/symbols |
| `hypersolve/src/real_interval.rs` | strict exact-`Real` endpoint enclosures | retain as proof adapter; replace rational/dyadic cases incrementally |
| `alumina-interface-core/src/boundary.rs` | exact values, rational bounded measurements, canonical/display boundary | retain wrappers; delegate interval core |
| `alumina-interface-core/src/machine_profile.rs` | nominal/lower/upper physical facts and conservative propagation | retain semantic type; delegate rational arithmetic |
| `alumina-interface-core/src/motion_schedule.rs` | spatial/time error budgets, exact one-sided lowering | reuse interval/radius kernels; preserve evidence |
| `alumina-foc/src/lib.rs` | `Q30Interval` arithmetic | wrap/delegate fixed interval after equivalence |
| `alumina-foc/src/current.rs` | calibration-to-current enclosure | retain domain semantics; reuse fixed intervals |
| `alumina-foc/src/servo.rs` | position/velocity intervals and widened errors | reuse where exact; retain widened domain types |
| `alumina-clock/src/lib.rs` | affine rate/offset set estimator | generalize arithmetic only; retain causal policy |
| `hyperpath/src/routing/*` | specialized path reachability and jerk refinement | extract dynamic algorithms into Hyperdynamics |
| `hyperreal::Real::certified_dyadic_interval` | certified point enclosure bridge | consume from Hyperinterval; no reverse dependency |
| Hyperlimit predicate stages | certified decision cascade | add interval filters after qualification |

For every discovered item record:

```text
repository
path
symbol/type
endpoint representation
point/set/proposal/certificate semantics
allocation model
std/no_std
public/serialized status
current consumers
migration classification
baseline tests
baseline benchmarks
```

---

# 13. Phased execution plan

Every phase gets its own focused commit(s), ledger entry, tests, benchmark evidence, and rollback point.

## Phase 0 — Inventory and immutable baseline

### Tasks

- Record all repository states and user-owned edits.
- Create `INTERVAL_REFACTOR_PROGRESS.md` at the workspace root.
- Create `interval-inventory.tsv` or JSON from the source map above plus workspace-wide search.
- Record dependency graphs and feature matrices.
- Record public APIs and serialized/canonical fixtures.
- Run and archive baseline tests, coverage, fuzz smoke, benchmarks, allocation profiles, binary sizes, and firmware/WASM sizes.
- Identify current slow-performer/lexicase entries touching intervals.

### Acceptance

- No production source changed.
- Baseline artifacts are reproducible and hash-bound.
- Every interval-like implementation has a disposition owner.

## Phase 1 — Create `hyperinterval` skeleton and laws

### Tasks

- Add manifest, license, README, CI, docs, examples, performance ledger.
- Establish `no_std`/feature gates.
- Implement explicit interval state, errors, domain coverage, sign sets, named relations.
- Create dev-only conformance harness/corpus.
- Add API accounting and layout tests.

### Acceptance

- `--no-default-features` builds.
- `fixed`/`integer` build on firmware MSRV/targets.
- No Hyperreal dependency without its feature.
- Empty/reversed/domain states cannot be forged through safe API/deserialization.

## Phase 2 — Integer and fixed-point intervals

### Tasks

- Implement integer ranges.
- Implement Q2.30/Q31.32-capable fixed intervals with widened arithmetic.
- Port current Q30 exact oracle cases into the conformance suite.
- Exhaustively test reduced-width analogues.

### Acceptance

- Every sampled/exhaustive exact point result is enclosed.
- Overflow never silently saturates/wraps.
- Existing Q30 behavior has a differential adapter with zero mismatch.
- No allocation and no std.
- Code/cycle size is recorded.

## Phase 3 — Rational and dyadic intervals plus Hyperreal bridge

### Tasks

- Implement `RationalInterval`.
- Move and qualify rational product hull.
- Implement common-scale `DyadicInterval`.
- Implement directed conversions.
- Consume `Real::certified_dyadic_interval`.
- Implement initial `RealBall`.

### Acceptance

- Exact rational oracle/properties pass.
- Dyadic-to-rational cross-checks pass.
- Every Real bridge interval encloses independent MPFR/Rug approximations.
- No dependency cycle.
- Hyperreal source/layout/cache behavior unchanged unless a narrow bridge is proven necessary.

## Phase 4 — Primitive proposal and strict float layers

### Tasks

- Implement proposal wrappers first.
- Implement strict finite f32/f64 basic enclosures.
- Add target/environment qualification.
- Add exact IEEE input conversion and cross-backend tests.

### Acceptance

- Proposal APIs cannot create certified outcomes.
- Certified basic operations pass exhaustive edge classes and random exact-rational oracles.
- NaN/inf/subnormal/signed-zero/overflow behavior is explicit.
- Unsupported targets fail closed or expose proposal only.

## Phase 5 — Elementary interval functions

### Tasks

- Implement the ordered function list from Section 3.13.
- Add domain coverage, extrema, pole, periodic, and huge-argument tests.
- Benchmark by precision band and interval width.

### Acceptance

- Independent directed MPFR/Rug corpus passes.
- No endpoint-only trig.
- Partial domains are reported.
- No primitive transcendental is certified without a rigorous bound.

## Phase 6 — Hyperlattice interval vectors/matrices/boxes

### Tasks

- Add dependency and feature forwarding.
- Implement carriers, operations, facts, and specialized kernels.
- Add structure-of-arrays primitive experiments.
- Add FOC transform and Krawczyk arithmetic clients.

### Acceptance

- Exact componentwise oracles pass.
- Matrix/affine images enclose exhaustive small-dimensional samples.
- Fixed-size primitive paths allocate nothing.
- Hyperlattice’s existing exact `Real` API and performance remain qualified.

## Phase 7 — Hypersolve migration

### Tasks

- Add Hyperinterval/Hyperlattice adapters.
- Migrate generic rational/dyadic/vector/matrix arithmetic.
- Retain solver proof/report ownership.
- Differential old/new implementations.
- Narrow `RealInterval` usage.

### Acceptance

- All existing Hypersolve statuses and certificates match.
- Krawczyk two-root and unequal-radius adversarial cases remain correct.
- Root isolation/algebraic identities unchanged.
- Full tests/fuzz/coverage pass.
- Time/allocation/size regressions are documented and justified or reverted.

## Phase 8 — Alumina Interface migration

### Tasks

- Delegate `BoundedMeasurement` and machine `ExactInterval` arithmetic.
- Replace repeated endpoint formulas.
- Use interval/radius operations in named error composition.
- Preserve canonical output/evidence.

### Acceptance

- Existing canonical fixtures and digests are byte-identical.
- Machine profiles and schedules are field-identical.
- WASM/native tests pass.
- No display value gains authority.

## Phase 9 — Alumina Firmware migration

### Tasks

- Add fixed-only dependency.
- Wrap/delegate Q30 and position intervals.
- Differential FOC/current/servo/clock behavior.
- Run HIL/simulator/stack/code-size gates.

### Acceptance

- No std/alloc/arbitrary precision.
- Firmware bytes/protocols unchanged where expected.
- Interval outputs exactly match existing implementations.
- No cycle-time, stack, or code-size regression accepted silently.

## Phase 10 — Hyperlimit interval filter integration

### Tasks

- Add traceable certified interval stages.
- Keep proposals scheduling-only.
- Profile stage ordering by predicate family.
- Retain only filters that pay their way.

### Acceptance

- Strict policy results are unchanged or more complete, never less exact.
- Terminal approximate policy remains explicit.
- No interval-containing-zero path returns exact zero.
- Representative geometry workloads improve or remain within documented cost.

## Phase 11 — Hyperdynamics path extraction

### Tasks

- Refactor to domain-neutral traits.
- Move path-dynamics algorithms/reports.
- Add Hyperpath adapters and compatibility wrappers.
- Update Alumina imports.

### Acceptance

- Existing schedules, bottlenecks, halving counts, phase reports, and certificates are field-identical.
- Canonical machine lowering/evidence remains byte-identical.
- No generic ODE path replaces the specialized solver.

## Phase 12 — Validated ODE/reachability baseline

### Tasks

- Implement enclosed vector fields/Jacobians.
- Implement constant/affine, Euler/Picard, bounded step halving, existence/uniqueness, flow tube, and end enclosure.
- Add f64 RK proposal as non-authoritative candidate.
- Add constant flow, affine flow, harmonic oscillator, Riccati/domain-stress fixtures with independent or analytic controls.

### Acceptance

- Every certified tube encloses the independent exact/high-precision solution corpus.
- Unknown/exhausted/domain cases remain explicit.
- Halving strictly progresses and is bounded.
- Final time is exactly capped.
- Final new contribution is checked before success.

## Phase 13 — Taylor models and wider stack adoption

### Tasks

- Add demanded sparse coefficients and Taylor remainder.
- Add wrapping-control forms.
- Integrate curve/surface/F-rep/physics/circuit consumers one at a time.
- Add competitive validated-dynamics benchmarks.

### Acceptance

- Each consumer has a demonstrated exactness/completeness/performance use case.
- No speculative dependency is added merely for architectural symmetry.
- All retained changes have proportional evidence.

## Phase 14 — Coordinated release

### Tasks

- Regularize READMEs/API guides.
- Add migration guide.
- Finish feature matrix and docs.rs builds.
- Run coordinated whole-stack CI and representative application/HIL tests.
- Produce release notes with accepted tradeoffs and known limitations.

### Acceptance

- No stale duplicate interval implementation remains without a documented reason.
- Public authority boundaries are documented.
- Versions and dependency requirements are coordinated.
- Release artifacts reproduce from clean checkouts.

---

# 14. Test and proof matrix

## 14.1 Universal interval laws

Test for each applicable backend:

- checked endpoint ordering;
- empty and singleton semantics;
- hull/intersection idempotence and commutativity;
- subset transitivity;
- midpoint/radius enclosure;
- width nonnegativity;
- sign-set correctness;
- arithmetic enclosure;
- square specialization;
- reciprocal/division zero handling;
- conversion enclosure;
- serialization round trip through checked constructors;
- no history dependence;
- no mutation through aliases;
- deterministic results across query order.

## 14.2 Independent oracles

Use:

- exhaustive small integer/fixed grids;
- exact rational enumeration;
- Hyperreal exact identities;
- Rug/MPFR directed checks as dev-only oracle;
- analytic ODE solutions;
- existing competitor corpora;
- retained fuzz/lexicase worst performers;
- independently rebuilt graphs/values to avoid pointer-sharing-only success.

## 14.3 Fuzz targets

At minimum:

- interval constructors/serialization;
- integer/fixed operations;
- rational/dyadic conversions;
- strict float special values;
- elementary domain boundaries;
- vector/matrix shapes;
- Krawczyk certificates;
- ODE step/subdivision policies;
- Hyperpath schedule equivalence;
- machine profile and firmware lowering.

Every discovered failure becomes a named permanent regression.

## 14.4 Concurrency and lifecycle

Test:

- concurrent Hyperreal-to-dyadic enclosure requests;
- warmed/coarse/fine query order;
- clone/drop behavior;
- no global rounding interference;
- aborted work not published;
- serde/cache reconstruction;
- no leaked retained graphs or process-global state.

## 14.5 Coverage and static gates

Per affected crate:

```text
cargo fmt --all -- --check
cargo test --all-targets --all-features
cargo test --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
cargo check --examples --benches --all-features
coverage script to the project’s required threshold
cargo fuzz build and bounded smoke runs
ASan/UBSan/Valgrind/Massif where supported
Miri for small no-unsafe core paths where useful
```

Run target-specific no_std/WASM/embedded builds.

---

# 15. Benchmark and resource matrix

Record baseline and candidate results with raw samples, not only summaries.

## Hyperinterval

- fixed add/sub/mul/div/square;
- rational sign partitions;
- dyadic alignment/coarsening;
- Real-to-dyadic enclosure;
- float proposal vs certified enclosure;
- elementary functions by width/precision/domain.

## Hyperlattice

- interval affine image;
- matrix-vector and matrix-matrix;
- midpoint/radius conversion;
- Krawczyk primitives;
- primitive SoA vs AoS;
- common-scale dyadic vectors.

## Hypersolve

- affine/quadratic interval certification;
- multivariate Krawczyk;
- root isolation endpoint work;
- algebraic image bounds;
- repeated warm and independently rebuilt operands.

## Hyperlimit / geometry

- orientation/incircle/ordering filters;
- CurveRegion intersection/boolean/offset sequences;
- mesh broad phase and difficult degeneracies;
- exact/primitive/dyadic stage hit rates.

## Alumina Interface

- profile derivation;
- schedule/error composition;
- timer search;
- WASM size and latency;
- canonical evidence build/replay.

## Firmware

- Q30 operations;
- Clarke/Park/modulation;
- current reconstruction;
- servo update;
- clock observation/prediction;
- code size, stack, cycles.

## Hyperdynamics

- existing path schedules before/after extraction;
- validated affine/ODE steps;
- flow-tube widths;
- rejected-step counts;
- proposal cost vs certification cost;
- allocation and evidence size.

A statistically significant hot-path regression must be either fixed, reverted, or explicitly accepted because it buys a higher-priority property. Do not hide small-case regressions behind aggregate wins.

---

# 16. CI matrix

`hyperinterval` CI should include at least:

```text
no default features
integer
fixed
float-proposal
float-enclosure on qualified host
bigint + dyadic
rational + hyperreal
real-ball
serde combinations
all features
minimum supported Rust
stable Rust
Linux/macOS/Windows where available
WASM build
representative embedded no_std targets
```

Whole-stack coordinated CI must test the exact sibling workspace, not silently resolve older crates.io versions.

Add dependency/version checks preventing accidental fallback to stale published Hyper crates.

---

# 17. Documentation requirements

Each public type must state:

- the mathematical set represented;
- whether endpoints are exact;
- whether arithmetic is exact or outward-rounded;
- what environment assumptions apply;
- how empty/partial/unbounded domains are represented;
- what operations may return unknown/exhausted;
- whether it is authoritative, certified filter, or proposal-only;
- allocation and std/no_std behavior;
- conversion loss/rounding direction;
- which layer owns final decisions.

Provide examples for:

- rational measurement interval;
- fixed Q30 enclosure;
- Real-to-dyadic enclosure;
- float proposal checked by exact interval replay;
- interval matrix affine image;
- Hypersolve Krawczyk certificate;
- Hyperpath/Hyperdynamics schedule;
- validated ODE flow tube;
- firmware current calibration.

[Runnable commands and qualification for these nine example topics](../INTERVAL_REFACTOR_PROGRESS.md#section-17-worked-interval-examples)

---

# 18. Stop/reject conditions

Stop and record the experiment rather than merging when any candidate:

- changes exact answers;
- changes an explicit unknown into a guessed decision;
- treats zero containment as equality;
- loses domain/provenance/unit information;
- introduces a dependency cycle;
- adds global precision/rounding state;
- makes output depend on query history;
- breaks no_std/allocation-free firmware;
- discards useful Hyperreal caches;
- requires unchecked user promises;
- adds an unbounded loop/refinement;
- changes canonical evidence bytes unintentionally;
- causes a material hot-path regression without higher-priority payoff;
- expands binary/source/API surface without a demonstrated consumer.

Preserve rejected candidates, raw evidence, and reasons in the ledger. Do not repeatedly rediscover them.

---

# 19. Definition of done

The project is complete only when all of the following are true:

1. `hyperinterval` exists with explicit exact/fixed/rational/dyadic/proposal/qualified-float semantics and a full conformance suite.
2. Hyperreal remains point-valued and dependency-cycle free.
3. Hyperlattice exposes interval vectors, matrices, boxes, affine images, Jacobian carriers, and measured specialized kernels.
4. Hypersolve uses the common interval/lattice arithmetic while retaining solver-specific reports, symbols, and exact replay authority.
5. Alumina Interface uses shared rational interval arithmetic while preserving units, domain types, canonical evidence, and exact/display separation.
6. Alumina Firmware uses or conforms to the shared fixed interval kernel without std, allocation, arbitrary precision, or timing regressions.
7. Hyperlimit can exploit certified primitive/dyadic/rational interval filters without changing proof authority.
8. `hyperdynamics` exists and contains the current exact path dynamics plus a validated general ODE/reachability baseline.
9. Hyperpath retains static path/PCB/CAM semantics and compatibility adapters; existing functionality is not lost.
10. At least one real higher-layer geometry consumer and one Alumina consumer demonstrate measurable benefit from the new layer.
11. Full tests, fuzz, coverage, memory, binary-size, WASM, embedded, and coordinated CI gates pass.
12. Every retained optimization and every accepted regression has recorded raw evidence and rationale.
13. Documentation clearly distinguishes proposals, enclosures, certificates, and decisions.
14. Clean checkouts reproduce the coordinated release.

---

# 20. Immediate first actions for the agent

Execute these before implementing arithmetic:

1. Read this brief and `interval-assessment.md` in full.
2. Inventory all repository states and create an isolated branch/worktree plan.
3. Create `INTERVAL_REFACTOR_PROGRESS.md` with the priority order and non-negotiable rules copied verbatim.
4. Generate the workspace-wide interval inventory using the source seeds in Section 12.
5. Record baseline tests, performance, memory, binary/WASM/firmware size, and canonical fixture hashes.
6. Produce a proposed `hyperinterval/Cargo.toml`, module tree, public API inventory, and dependency graph.
7. Verify that the proposed graph contains no cycle and that fixed-only firmware builds do not pull Hyperreal.
8. Implement only Phase 1 after the baseline and API plan are recorded.
9. Keep each phase reviewable and independently revertible.
10. Never claim a phase complete from code inspection alone; require the acceptance evidence listed above.
