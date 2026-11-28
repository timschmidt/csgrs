# Straight skeleton: development plan

This is the plan for `Contour2::straight_skeleton` and the work that builds on
it: real-world coverage, holes, weights, roofs and mitred offsets. It responds
to a user report that the skeleton blocked on almost every surveyed building
footprint and cadastral parcel. Every item is ranked by the project's priority
ladder, and each states the acceptance evidence that closes it.

Status: 28 November 2026. Related records:
- [`FUTURE_WORK.md`](FUTURE_WORK.md): radical tower signs, §1;
- [`BLOCKER_REGRESSIONS.md`](BLOCKER_REGRESSIONS.md);
- [`PERFORMANCE.md`](PERFORMANCE.md) (straight-skeleton benchmark and fuzzer);
- the admission protocol, [`docs/admission.md`](../../docs/admission.md).

## 1. The priority ladder decides

The governing order is the one in [`AGENTS.md`](../../AGENTS.md):

**exactness > completeness > performance > memory > binary size > source size**

The owner's guidance adds that CAD/CAM/Sim workflows come first. Broader
scientific use should follow at little extra cost.

For the straight skeleton the ladder means:

1. **Exactness is not traded for coverage.** A completed skeleton must be the
   exact skeleton of the source polygon: a tree with the right incidences,
   exact node coordinates and times, and arcs that do not cross. A blocker is
   an honest result. A wrong graph is not.
   - Approximate decisions are allowed only under an explicitly approximate
     policy (`provisional`, `APPROXIMATE_512`), and they must keep their
     certainty receipt.
   - They must never be needed for relations that are structurally true.
     Those are proved structurally (section 3).
2. **Completeness on the declared domain comes next.** General simple polygons
   with integer or decimal coordinates, and so irrational edge directions, are
   in scope. Surveyed footprints and parcels are the reference workload. Holes,
   weights and curved sources extend the domain in that order (section 5).
3. **Performance and memory follow.** They are measured in the resource atlas
   and must not regress the shapes that already complete.

### Contributions

The reporter offered work on roofs, mitred offsets, per-edge weights and holes.
Whether and how such contributions are accepted depends on the ladder above:

- **Accepted** when the work extends completeness on the declared domain
  without weakening exactness. Examples:
  - holes;
  - weights with an exact speed model;
  - faces, roofs and offsets derived from the exact skeleton.
- **Required with it:** acceptance tests on the reported corpus shapes,
  dispatch traces on the touched paths, and no regression in the skeleton
  benchmarks or atlas operations. The [admission protocol](../../docs/admission.md)
  governs; development artifacts never block on size.
- **Not accepted:**
  - shortcuts that complete more polygons by guessing structural relations
    numerically under `STRICT`;
  - a second offset kernel that bypasses the skeleton or the existing region
    offsets (see section 5.4).
- **Where:**
  - skeleton construction belongs in `src/straight_skeleton/`;
  - derived products (faces, roofs, offsets) go in a new
    `src/straight_skeleton/derived.rs`, consumed through `StraightSkeleton2`;
  - tests go under `tests/straight_skeleton_*.rs`.

## 2. Current state

| Input | State |
| --- | --- |
| Simple polygon, rational edge normals (axis-aligned, 3-4-5 and similar) | complete |
| Simple polygon, several distinct edge-length radicals | complete since 28 November (this plan's first fix) |
| Uniform-width shapes that collapse everywhere at once (T, plus, H) | complete as trees since 28 November |
| Symmetric polygons with exactly tied event times across radicals | `UncertainEventOrdering` under `STRICT` (5.1) |
| Near-regular polygons, n ≥ 16 (resource atlas) | blocked; recheck after the fix (4.1) |
| Orthogonal comb (many reflex vertices, resource atlas) | `InvalidSplitTopology`; unexplained (4.3) |
| Contours with circular arcs | staged bubble and splice events; separate blockers |
| Polygons with holes | no API (5.2) |
| Weighted edges | no API (5.3) |
| Faces, roofs, offsets at a time | no API (5.4, 5.5) |

### Fixed on 28 November

The reporter's first two blockers asked exact arithmetic to prove relations
that hold by construction. When several unrelated square roots are involved,
it cannot always prove such a zero.

- **Edge events.** Both moving vertices of an edge lie on that edge's moving
  line, so their difference vanishes exactly when its component along the
  line does. The event is now solved along the edge direction, and the second
  Cartesian residual is no longer tested.
- **New wavefront vertices.** A new vertex inherits the node of the edge that
  collapsed between its two supports. Adjacent edges that collapse at the same
  time share one node, because two non-parallel moving lines meet at one
  point. Coordinates are no longer compared for equality. This also removes
  the duplicate nodes after split events, which explained the 12–14-node
  graphs in the report.
- **Degenerate terminal collapse.** The final ridges of a fully simultaneous
  collapse are now split at the final nodes strictly inside them. Before, a
  ridge ran straight through a multi-way vertex event and closed a cycle.

Regression tests: `tests/straight_skeleton_radicals.rs`. They check each
polygon under `STRICT` and `provisional`:
- the graph is one tree;
- every event node lies inside the polygon;
- generic shapes have exactly 2n − 2 nodes.

## 3. Structural relations: audit every exact zero

**Rung: exactness, completeness.** Each remaining exact-zero or equality test
on the line paths is either *structural* (true by construction) or *genuine*
(depends on the input). A structural test must be replaced by its proof. A
genuine test stays exact under `STRICT`, and only an explicit policy may
approximate it.

| Site | Relation | Class | Action |
| --- | --- | --- | --- |
| `line_builders::edge_event_candidate` | two adjacent vertices meet | structural | done |
| `line_builders` pair-start matching | new vertex at its collapse node | structural | done |
| `line_builders::apply_general_edge_events` | simultaneous adjacent collapses share a node | structural | done |
| `line_builders::finish_terminal_parallel_cycle` | final nodes on the collapsed boundary | genuine (collinearity) | exact sign; blocks if undecided |
| `line_builders::finish_terminal_parallel_cycle`, incident supports | final node lies on each incident support | structural for supports of the cycle | **todo**: derive incidence from cycle membership, not from a residual sign |
| `line_builders` vertex candidate (`real_sign(residual)`) | two non-adjacent vertices meet | genuine | keep; tie handling in 5.1 |
| `merge_codirected_coincident_edges` | codirected supports coincide after a split | partly structural | **todo**: two supports from one split share their offset line by construction |
| `supports_are_opposed_and_coincident` | the two remaining supports of a split output are one line | genuine | keep, with tie handling |
| `event_node` point and time equality | merging non-adjacent nodes | genuine | keep; structural merging covers the adjacent case |
| `shape_preserving_edge_event_candidate` | arc endpoint distance to centre | genuine | keep |
| Event-time ordering (`compare_reals`) | which event is next; exact ties | genuine | 5.1 |

**Acceptance.** Each site is either removed or has a test that reaches it with
several distinct radicals. In that test the structural sites complete under
`STRICT` and the genuine sites give their typed blocker.

## 4. Qualification and measurement

### 4.1 Real-world corpus survey

**Rung: completeness. Owner: maintainers. Needs: the reporter's 20,000 rings
or an equivalent open corpus.**

- Ask the reporter for the 10,000 BAG building footprints and 10,000 cadastral
  parcels, and the probe that produced their table. Both datasets are Dutch
  open data; confirm the licence before archiving.
- Run each ring through `from_finite_ring` and `straight_skeleton` under
  `STRICT` and `provisional`. Use national-grid metres, and whole millimetres
  moved to a local origin, as the report did.
- Record:
  - the completion rate;
  - a histogram of blocker kinds and stages;
  - events per completed skeleton;
  - time and peak heap per ring;
  - the largest coordinate expression (`Shape` depth and bits through the
    resource atlas).
- Check the soundness of every completed result:
  - one tree;
  - nodes inside the polygon;
  - arcs that do not cross;
  - node times equal to the distance to the node's defining edges;
  - for weighted runs, the weighted distance.
- Archive the run in the evidence repository. Commit only a few hundred rings
  chosen to cover every blocker class, as a test corpus with their expected
  outcomes.

**Acceptance.** Completion is reported against the report's baseline (8.84% of
buildings, 0.39% of parcels). Every remaining blocker class is either fixed or
recorded here with its frequency. No completed result fails a soundness check.

### 4.2 Resource atlas coverage

**Rung: performance, memory.** Extend the Hypercurve atlas `straight skeleton`
operation with these sweeps:
- vertex count, for random convex footprints;
- reflex-vertex count;
- coordinate bits;
- regular n-gons, which carry exact ties.

Record node time and coordinate expression depth as `Shape` depth and bits.

The atlas description still says near-regular polygons with n ≥ 16 block with
`UncertainWavefrontRelation`. Recheck it after the 28 November fix, and update
it.

### 4.3 The orthogonal comb

**Rung: completeness.** The atlas comb (many reflex vertices, rational
normals) ends with `InvalidSplitTopology`.
- Reproduce it and find the split that fails.
- Fix it, or record why the split is undecidable as stated.
- Add the comb to the regression tests.

## 5. Extensions

### 5.1 Exact ties between event times

**Rung: exactness, completeness.** Symmetric footprints are common: rectangles
with chamfers, regular bays, mirrored wings. They give event times that are
exactly equal but are written with different radicals, so `STRICT` cannot
always order them and returns `UncertainEventOrdering`.

- **Exact route.** Decide equality of event times that are linear combinations
  of square roots by the radical tower sign (FUTURE_WORK §1) or an algebraic
  number equality certificate. Equal times then merge; unequal times order by
  refinement.
- **Policy route.** Under an approximate policy, a time difference within the
  policy's tolerance may be treated as a tie. The resulting skeleton is
  provisional and its certainty receipt must say so. Ties must merge into one
  multi-way event, never into two events in arbitrary order. The report's
  12–14-node graphs came from unmerged ties.

**Acceptance.** Regular hexagons and octagons, and a chamfered rectangle, are
built from decimal vertices:
- the exact route completes them under `STRICT`;
- the policy route completes them provisionally, with merged ties and correct
  trees.

### 5.2 Polygons with holes

**Rung: completeness.** This is needed for parcels with enclaves and buildings
with courtyards.

- Add `CurveRegion2::straight_skeleton` for one outer boundary and any number
  of holes, oriented by the region's winding.
- The wavefront starts as several cycles. A reflex vertex of one cycle can hit
  an edge of another, which merges the cycles instead of splitting one. Add
  that merge event beside split events, with the same structural node
  identification.
- Edge events, split events and the tests in section 3 are unchanged inside
  each cycle.

**Acceptance.**
- A square with a square hole gives a known skeleton.
- A rectangle with an off-centre hole and a slanted hole completes under
  `STRICT`.
- Corpus parcels with enclaves complete.
- Every result is a tree per connected region, with nodes between boundaries.

### 5.3 Weighted skeletons

**Rung: completeness.** This covers per-edge roof slopes and offsets with
different setbacks per edge.

- **Model.** Each edge moves at an exact positive weight w (time per unit
  distance, or speed). Its velocity is w times the unit normal.
  - The structural edge-event relation still holds: both moving vertices lie
    on the edge's moving line.
  - The vertex-trajectory formula takes the weights as factors.
- **Weights to admit first.** Positive rational, and positive real with exact
  representation.
  - Zero weights (vertical walls, gables) and negative weights change the
    topology rules, as described in the literature on weighted skeletons. They
    are a separate item, with their own blockers.
- **Correctness rules.** Weighted skeletons can lose the tree property. The
  published counterexamples are polygons with reflex vertices and very unequal
  weights. The contract must say which weight ranges guarantee a tree, and give
  a typed blocker outside them. It must never return a non-tree as complete.

**Acceptance.**
- Equal weights reproduce the unweighted skeleton exactly.
- A rectangle with two slopes gives its analytic ridge.
- Random convex footprints with weights in [1/2, 2] give trees whose node
  times equal weighted distances.

### 5.4 Faces and mitred offsets

**Rung: completeness, then performance.**

- **Faces.** Return the face of each source edge: the exact polygon swept by
  that edge, bounded by skeleton arcs. A hipped roof, inward offsets and
  per-edge attribution are all built from these faces.
- **Offsets at a time.** The active wavefront at time t, between events, is
  the exact inward mitred offset at distance t (weighted: at weighted time t).
  Return it as a `CurveRegion2` with one contour per wavefront cycle. At an
  event time, return the region after the event.
- **One authority.** Hypercurve already has region offsets with
  `OffsetCornerStyle2::Miter { limit }` (`src/offset.rs`). The skeleton offset
  is the unlimited mitre for inward line offsets. Before adding the API:
  - decide whether region offsets call the skeleton for that case, or whether
    the skeleton offset is a documented specialisation with a cross-check test;
  - make sure the two paths agree exactly where both apply.

  The rule in [`AGENTS.md`](../../AGENTS.md) is that adapters must not create a
  second geometry kernel.

**Acceptance.**
- Face polygons tile the source polygon exactly: their exact areas sum to the
  source area.
- The offset at t equals the region offset with an infinite mitre limit for
  every corpus shape where both complete.
- Offsets across an event time match their limits from both sides.

### 5.5 Roofs

**Rung: completeness, for the CAD/CAM workflow.**

- **Hipped roof with one slope s.** Each skeleton node gets height s · time.
  Each face is lifted to a planar 3D polygon. The output is an exact roof
  surface whose border is the footprint at height 0.
- **Per-edge slopes.** Use the weighted skeleton (5.3) with weights 1/s_i.
- **Gables and vertical walls.** These need zero weights; they are a separate
  item (5.3).
- **Consumer.** Close the roof with the footprint walls into a Hypermesh solid,
  so roofs compose with booleans, exports and the csgrs user path.

**Acceptance.**
- Every roof face is planar, by exact coplanarity of its lifted nodes.
- The closed solid is manifold with exact volume.
- A rectangle with slope 1 gives the analytic hip roof.

## 6. Performance and memory

**Rung: performance, memory.**

- **Event scheduling.** Line paths recompute every candidate after each event,
  which is quadratic in the event count. Use a priority queue keyed by exact
  time and invalidate only the candidates next to changed vertices. Ties
  follow 5.1.
- **Coordinate growth.** Each event time and point is built from earlier ones.
  Through the resource atlas, measure whether depth and bit size grow with
  event count, and how. Then decide whether node coordinates should be
  recomputed from their defining supports, a fixed-size closed form. They are
  three source lines meeting at a time, so this keeps coordinates from
  compounding through earlier nodes.
- **Survey timings.** The report measured 0.1–0.2 ms to the blocker. Completed
  runs are new; take their timings from 4.1 and set the benchmark baselines
  from them.

**Acceptance.** No atlas or benchmark regression on shapes that completed
before 28 November. A completed corpus footprint stays under 10 ms at the
median, to be checked once 4.1 exists.

## 7. Order of work

1. 4.1 corpus survey, to measure the effect of the 28 November fixes.
2. Section 3 audit of the remaining structural sites.
3. 4.3, the comb split.
4. 5.1, exact ties, first the exact route.
5. 5.4 faces, and offsets once the single-authority decision is made.
6. 5.5 roofs with one slope.
7. 5.2 holes.
8. 5.3 weights.
9. Section 6 scheduling and growth.

Items 1–4 raise completeness on the existing domain and come first by the
ladder. Items 5–6 are the reporter's CAD/CAM workflow. Items 7–8 extend the
domain.
