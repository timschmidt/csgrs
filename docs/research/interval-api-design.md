# Interval API and dependency admission

This is the pre-implementation API proposal for
[`INTERVAL_REFACTOR_PLAN.md`](../INTERVAL_REFACTOR_PLAN.md), phases 1–3.
It is not a completion claim. The root progress record tracks executed gates.

The phase-4 `float-proposal` extension adds F32ProposalInterval and
F64ProposalInterval. Their public arithmetic is native, inexact and
non-authoritative; endpoint access is named `estimates`. They do not expose
containment, exclusion, sign sets, or conversion into certified carriers.
Input must be finite and ordered, using integer bit comparisons even in an
environment that flushes subnormals. Signed zero canonicalizes as a set endpoint.
Arithmetic overflow is explicit. Serde uses optional integer IEEE bit pairs,
revalidated on import. No std, allocator or host dependency is required.
This is not the strict float enclosure backend.

## Ownership and missing witnesses

Hyperinterval owns scalar sets; Hyperlattice owns their linear algebra;
Hyperlimit owns decisions; Hypersolve owns certification; Hyperdynamics owns
evolution. Domain wrappers retain units, symbols, selection and provenance.
P1/P3/P5/P6 govern the new core; P8–P11/P19 govern consumers;
P31–P33 govern machine boundaries. No existing predicate sequence changes in
the skeleton phase.

Phase 1 witnesses are currently missing: constructors rejecting reversed
endpoints, separate empty state, complete sign-set truth table, exact named
set relations, validated serde ingress, no_std feature closure, layouts and
Rust 1.88 firmware compatibility. Phase 2 adds independent point-image
oracles and differential Q30 tests before domain delegation.

## Proposed manifest

```toml
[package]
name = "hyperinterval"
version = "0.1.0"
edition = "2024"
rust-version = "1.88"
license = "MIT OR Apache-2.0"

[features]
default = []
alloc = []
std = ["alloc"]
integer = []
fixed = []
serde = ["dep:serde"]

[dependencies]
serde = { version = "1", default-features = false, features = ["derive"], optional = true }
```

Only implemented features ship. Later phases add the optional host features
specified by the plan. They must not become defaults. Hyperreal's current
MSRV is 1.95; host features therefore require 1.95 while the default,
integer/fixed and proposal subsets must remain usable on 1.88. Test both
configurations explicitly; a package-level rust-version alone is insufficient.

## Module and API inventory

| Module | Public surface | Invariant / authority |
| --- | --- | --- |
| `state` | `IntervalError`, `DomainCoverage`, `RangeEnclosure<I>`, `SignSet` | Domain qualification is separate from image; no decision status |
| private `bounds` | none | Checked finite closed endpoints and named relations; no public generic interval |
| `integer` | sealed `IntegerEndpoint`, `IntegerRange<I>` | Exact discrete set, explicit empty, primitive signed/unsigned endpoints |
| `fixed` | sealed `FixedEndpoint`, `FixedInterval<I, FRAC>` | Closed real interval with exact dyadic endpoints, i32/i64, validated scale |
| later `rational` | `RationalInterval` | Exact rational set arithmetic, private ordered endpoints |
| later `dyadic` | `DyadicInterval` | Exact common-scale representation; explicit outward coarsening |
| later `real_ball` | `RealBall` | Exact point center plus nonnegative exact radius |
| later `proposal` | f32/f64 proposal types | No proof-producing interface |
| later `float` | f32/f64 qualified enclosures | Explicit environment qualification before arithmetic |
| later `elementary` | domain-qualified range functions | Bounded refinement, critical points, poles |

Both initial carriers offer `new`, `point`, `empty`, `is_empty`,
`is_singleton`, `bounds`, `lower`, `upper`, `contains_point`, `subset_of`,
`interior_subset_of`, `strictly_left_of`, `disjoint`, `overlaps`, `hull`,
`intersection`, and `signs`. Empty endpoints return `None`; empty is a subset
of every set. `strictly_left_of` is false if either operand is empty.
`interior_subset_of` uses the real endpoint interior even for integer ranges;
it is named and documented rather than conflated with discrete topology.

Phase 2 adds checked arithmetic and explicitly rounded midpoint/radius
operations. No `PartialOrd` or midpoint-as-point conversion exists.
The internal interval state is `Option<Bounds<I>>`, where `Bounds` fields
are private. Serde reads an optional endpoint pair and reconstructs through
`new`; null is empty. Scale validation applies even to empty fixed intervals.
Serde never imports cached facts or unchecked constructor flags.

## Dependency graph

Production edges point from consumer to dependency:

```text
hyperinterval (default/integer/fixed) -> no dependencies
hyperinterval (serde)               -> serde (no default features)
hyperinterval (future host)         -> hyperreal, num-bigint/num-traits
hyperlattice                        -> hyperinterval, hyperreal
hyperlimit                          -> hyperinterval, hyperlattice, hyperreal
hypersolve                          -> hyperinterval, hyperlattice, hyperlimit, hyperreal
hyperdynamics                       -> the preceding mathematical layers
hyperpath                           -> hyperdynamics and mathematical layers
alumina-firmware                    -> hyperinterval integer/fixed only
alumina-interface                   -> appropriate host and domain layers
```

This proposed order is acyclic. Cargo metadata and feature-tree checks must
verify the implemented graph; this diagram is not their substitute.
`RealInterval` stays in Hypersolve: its comparisons call Hyperlimit.
`Real::certified_dyadic_interval` already supplies a neutral bridge with grid
spacing `2^precision`; that spacing is not a bound on enclosure width.

## Baseline scope and rollout

The CSGRS baseline is commit `1671894b4a896b091416503910e3720ecf233c82`.
Interface is `ef94d9e60e0004f3ec894d174fa49c00d547c63f`; firmware is
`bdee397d0b80841f16bf17e7bcd029b4feb2c212`. Both were clean after cloning.
Their older sibling-path layout initially failed Cargo resolution even with
links: Cargo distinguished the sibling path from the monorepo path. The
Interface manifest now names the exact `../csgrs/crates/...` locations and its
lockfile has been regenerated. Two independently reproduced API compatibility
fixes are recorded in the progress log. This is a normalized local baseline,
not release packaging evidence. No stale crates.io Hyper package may fill a missing path.

The generated root `interval-inventory.tsv` is a lexical candidate inventory,
not a completed body audit. It records owners and provisional dispositions;
each migrated row needs body, consumer and baseline review. Raw source and
fixture hashes, feature metadata, commands and output are staged outside the
source tree until registered in the evidence archive.

Initial inspected seeds: rational product hull moves only after all sign
partitions agree with four-corner exact products; solver `VariableBall`,
root isolators, Krawczyk reports and residual evidence remain domain-owned.
Alumina `ExactInterval` keeps its nominal value, and `Q30Interval` keeps its
public wrapper and layout pending measured equivalence. Servo differences
retain widened integer storage. Clock observation policy never moves into
the scalar kernel.
