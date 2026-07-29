# 0001: Exact Scalar and Decision Contract

Status: Accepted
Date: 2026-07-29

## Context

`hyperreal::Real` deliberately separates fast structural equality from
mathematical equality. Two values may denote the same real number while
`PartialEq` returns false because their representations differ. Conversely,
bounded refinement may be unable to decide a valid mathematical question.

A BREP kernel cannot turn representation identity or failure to decide into a
topological decision. Vertex incidence, edge closure, loop nesting, surface
membership, orientation, and Boolean classification all depend on mathematical
predicates.

The existing HyperBREP implementation contains direct point equality and
`Real::partial_cmp` in authoritative paths. The clean-break rebuild needs one
contract before it introduces final geometry and topology types.

## Decision

### Authoritative scalar

`hyperreal::Real` is the only scalar accepted or stored by core geometry.
There is no generic scalar parameter.

Primitive floats may be accepted only by explicitly named boundary adapters.
An adapter classifies conversion as exact lifting, certified approximation,
lossy import/export, or rejection. A float never becomes an implicit tolerance
or topology policy.

### Three distinct relations

The kernel keeps these concepts separate:

1. Identity: typed IDs refer to the same model record. Use ordinary `Eq`.
2. Representation equality: two values have the same authored/canonical
   representation. Use a specifically named method if an operation needs it.
3. Mathematical equality: two values or geometric images denote the same
   object. Use a certified predicate.

Core geometric carriers do not expose a misleading `PartialEq` as a substitute
for mathematical equality. Tests of geometric meaning use certified helpers,
not `assert_eq!` on carriers.

### Predicate ownership

- Scalar sign/order/equality is implemented by `hyperreal` and surfaced through
  Hyperlimit's predicate API.
- Point equality and lexicographic order use `hyperlimit::point2_equal`,
  `point3_equal`, and the point comparison predicates.
- Orientation, incidence, containment, intersection, and distance comparison
  use the corresponding Hyperlimit predicates.
- Algebraic candidate generation and root isolation belong to `hypersolve`.
- HyperBREP composes those decisions into topology but does not implement a
  second private scalar predicate layer.

The only allowed structural-fact use in HyperBREP is conservative scheduling:
selecting a certified fast path, eliminating work already proved irrelevant, or
choosing an arithmetic kernel. Missing structural facts cannot decide geometry.

### Outcomes

At the predicate boundary, HyperBREP consumes `PredicateOutcome<T>` without
discarding `Unknown`:

```rust
pub enum PredicateOutcome<T> {
    Decided {
        value: T,
        certainty: Certainty,
        stage: Escalation,
    },
    Unknown {
        needed: RefinementNeed,
        stage: Escalation,
    },
}
```

At an operation boundary, outcomes are translated without conflation:

- invalid authored input;
- unsupported geometry family or combination;
- unresolved mathematical predicate;
- exhausted caller resource/refinement policy;
- successful certified result.

Resource exhaustion is not mathematical ambiguity. Unsupported is not
inequality. Unknown is not false.

### Resource policy

Ordinary APIs have no tolerance argument. Operations that can require unbounded
work accept a named resource/refinement policy. The policy controls time,
refinement, expression growth, or fallback availability; it does not redefine
geometric equality.

The default policy must preserve exact semantics by returning an unresolved
error when its resource bound is reached.

### Proposal and certification

Approximate arithmetic, spatial indexes, and numerical solvers may propose:

- candidate pairs;
- parameter intervals;
- roots;
- split locations;
- classifications;
- correspondence between intersection branches.

No proposal changes topology until exact replay or certification succeeds.
Approximate results may be returned only by explicitly approximate downstream
operations such as tessellation.

## API Rules

- Geometry inputs and retained values use `Real`, `Point2`, `Point3`, exact
  vectors, and exact transforms.
- Functions borrow `&Real` for evaluation parameters when ownership is not
  required.
- IDs and direction/role enums implement `Eq`, `Ord`, and `Hash`.
- Geometry records keep fields private.
- Constructors return `Result`; geometric input never causes a public panic.
- A successful `Model` guarantees its documented invariants.
- `RawModel` and failed builder/edit commits retain all known diagnostic
  blockers.
- Successful conventional operations return conventional values rather than a
  readiness report.

## Required Regression Fixtures

The test suite must include:

- two structurally unequal `Real` expressions that are mathematically equal;
- two points with structurally unequal but mathematically equal coordinates;
- a nearby-but-unequal value requiring refinement;
- an equality that remains unresolved under a deliberately small policy;
- exact zero extents represented by non-identical expressions;
- exact loop closure across distinct coordinate representations;
- vertex classification using semantic rather than structural equality.

The canonical representation-distinct equality fixture is:

```rust
let left = Real::pi() + Real::e();
let right = Real::e() + Real::pi();

assert_ne!(left, right); // representation relation
assert_eq!(compare_reals(&left, &right).value(), None);
```

The expressions are mathematically equal by commutativity, but the current
bounded predicate implementation does not certify that identity. The required
kernel behavior is therefore `Unknown`, not `false`. Separate fixtures cover
certified equality when the scalar layer has a proof route.

## Consequences

- Some formerly boolean APIs become fallible or explicitly outcome-bearing.
- Algorithms cannot use derived `PartialEq` for convenience on geometric
  carriers.
- Exact common cases remain fast because structural facts and certified filters
  are still valid dispatch tools.
- Tests become clearer about whether they assert identity, representation, or
  geometry.
- Cross-crate equality defects must be fixed before HyperBREP relies on the
  affected operation.

## Rejected Alternatives

### Treat `Real::PartialEq` as mathematical equality

Rejected because it is intentionally structural and incomplete.

### Use an epsilon in HyperBREP

Rejected because epsilon equality is scale-dependent, non-transitive, and
cannot authoritatively determine topology.

### Convert to `f64` for broad or common cases

Rejected for decisions. Conservative floating filters are acceptable only when
their error bounds certify the result.

### Treat unknown as non-equal or outside

Rejected because it silently changes topology according to compute budget.

### Make the scalar generic

Rejected because the goal is one exact kernel with one semantic contract, not a
family of weaker numeric implementations.
