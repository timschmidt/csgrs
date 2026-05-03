# Evaluator Refactor Plan

## Goal

Replace the current recursive approximation and magnitude-discovery flow with an evaluator that:

- does not depend on deep Rust call stacks
- separates bound discovery from digit approximation
- preserves the current fast paths for exact and shallow expressions
- gives `Multiply` and `Square` a non-recursive way to choose working precision

This is primarily motivated by:

- remaining performance cost in `msd()` / `Multiply` / `Square`
- remaining stack-depth risk in low-stack environments such as WASM

## Current Problems

The current architecture mixes three concerns:

1. expression construction
2. approximation scheduling
3. magnitude discovery

That creates a circular dependency for non-exact product trees:

- `Multiply` / `Square` need child magnitude information
- magnitude information is discovered by recursive approximation
- approximation requires precision planning
- precision planning asks magnitude questions again

This is why local iterative fixes around `approx_signal()` helped structural nodes but did not solve general `Multiply` / `Square`.

## Target Architecture

The target architecture is a demand-driven evaluator with explicit work stacks and cached bounds.

Each node should eventually have:

- opcode
- children
- exact-value shortcut when available
- cached approximation(s)
- cached bound information

Bound information should include:

- definitely zero / definitely nonzero
- sign when known
- exact MSD when known
- or a bounded MSD range when only partial knowledge is available

These categories should not remain one shared semantic bucket.

The evaluator should distinguish at least two APIs:

- exact facts:
  - safe for public queries such as `sign()` and `msd()`
  - safe for constructor-time rewrites that must preserve semantics exactly
- planning facts:
  - safe only for precision planning and internal scheduling
  - may include lower bounds or other conservative facts that are not exact answers

The cached data should eventually reflect that split as well:

- approximation cache
- exact-facts cache
- planning-facts cache

The current branch has already shown why this matters:

- planning MSD lower bounds were useful for `Multiply` / `Square` / `Sqrt` planning
- using planning facts through public APIs caused semantic leakage
- the first corrective step was separating exact MSD from planning MSD
- the next step is to keep public APIs and constructor rewrites on exact facts only

The evaluator should run in two phases:

1. bound propagation
2. approximation

Approximation should only request tighter child bounds when the current bounds are insufficient to choose a safe working precision.

## Design Constraints

The refactor should preserve the current good properties:

- exact `Int` / `Ratio` nodes stay cheap
- cached approximations remain effective
- specialized kernels (`exp`, `ln`, `sin`, `cos`, `tan`, `sqrt`) remain as leaf computations
- shallow common-case expressions should not pay full scheduler overhead

This means the end state should be hybrid:

- cheap direct path for simple/exact/shallow cases
- iterative evaluator path for deeper or structurally hard cases

## Staged Plan

### Stage 1: Bound Model

Introduce a small bound representation for `Computable`.

Initial shape:

- exact zero / exact nonzero
- exact sign when known
- exact MSD when known

This stage should not replace the evaluator. It should only centralize today’s ad hoc structural rules.

Deliverables:

- `BoundInfo` or equivalent internal type
- helper for cheap exact/structural bound inference
- tests for exact and structural bound propagation

### Stage 1.5: Semantic Split

Before pushing bound usage further, separate the APIs and cached-data roles:

- exact sign vs planning sign
- exact MSD vs planning MSD lower bound
- public queries vs planner queries

Deliverables:

- explicit planner-facing helpers such as `planning_msd()` / `planning_sign_and_msd()`
- exact-fact helpers used by `sign()`, `msd()`, and constructor rewrites
- removal of planner-only facts from public semantic paths
- regression coverage proving the split

### Stage 2: Iterative Bound Propagation

Add an explicit-stack bound walker for structural nodes and selected arithmetic nodes.

Initial scope:

- `Negate`
- `Offset`
- `Add`
- `Multiply`
- `Square`
- `Inverse`

This stage should compute bounds without asking for full approximations unless required.

Deliverables:

- iterative bound request API
- cache integration for discovered bounds
- regression tests for deep product/square trees

### Stage 3: `Multiply` / `Square` Precision Planning on Bounds

Rewrite `Approximation::multiply` and `Approximation::square` to use bound requests instead of recursive `msd()` discovery.

This is the key functional milestone. At this point:

- child magnitude planning should no longer recurse structurally through `msd()`
- stack depth for product trees should drop substantially

Deliverables:

- updated multiply/square precision planning
- targeted microbenchmarks for mixed exact/non-exact product trees
- stack-depth regression coverage

### Stage 4: Iterative Approximation Scheduler

Generalize the existing limited explicit-stack path in `approx_signal()` into a work-stack evaluator for structural approximation dependencies.

Initial objective:

- unify bound requests and approximation requests under one scheduling model

Do not convert transcendental kernels themselves. They should remain leaf computations that pull child approximations through the scheduler.

### Stage 5: Heuristics and Mode Selection

Add low-overhead selection rules for when to use:

- direct recursive fast path
- iterative evaluator path

Candidate triggers:

- depth threshold
- node-shape threshold (`Multiply` / `Square` heavy)
- explicit low-stack mode for WASM consumers

## First Increment to Land

The first implementation step in this branch should be:

1. add a bound model
2. route current cheap MSD logic through it
3. extend that logic for `Inverse`, `Square`, and exact-known `Multiply`
4. add focused tests and benchmarks

This is intentionally small. It improves the architecture without forcing a whole-engine rewrite in one change.

## Current Branch Guidance

Based on the retained experiments so far:

- push bound usage into:
  - planner-facing `Add` / `Multiply` / `Square` / `Sqrt`
  - constructor-time exact/no-op normalization
- do not push planning facts into:
  - public `sign()`
  - public `msd()`
  - broad eager writes from every approximation path

In practice, this means future changes should prefer:

1. exact-fact rewrites at construction time
2. planning-bound use inside approximation kernels
3. explicit benchmarks whenever a public semantic API starts consulting more cached data

## Benchmarking Strategy

The refactor should be judged with targeted microbenchmarks, not only broad end-to-end benchmarks.

Add or keep focused cases for:

- deep structural add chains
- deep exact multiply chains
- deep identity multiply chains
- deep scaled product chains
- square-heavy chains
- mixed exact/non-exact expression evaluation in `Simple`

Success criteria:

- no meaningful regression on current hot-path microbenchmarks
- measurable improvement on product/square-heavy trees
- reduced stack sensitivity for deep expression trees

## Risks

Main risks:

- turning every evaluation into an interpreter and regressing common paths
- duplicating caches or making cache invalidation confusing
- accidentally widening precision requests and losing performance

Mitigation:

- keep stages small
- preserve shallow exact fast paths
- benchmark every retained change

## Non-Goals

This refactor is not intended to:

- turn `Computable` into a full symbolic algebra system
- fully intern all nodes into a global DAG immediately
- rewrite transcendental kernels from scratch

The focus is evaluator structure and scheduling, not symbolic expressiveness.
