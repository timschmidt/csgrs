# Successive operations

Release requirement: a result can be fed into further operations
indefinitely. Each intermediate must stay exactly representable with bounded
representation growth, stay usable (certified closed topology, and later
predicates still decide under `STRICT`), and keep per-step cost and memory
from growing faster than its output. This record measures that through the
public `csgrs` API. It is an admission record under
[`admission.md`](admission.md), not a proof of the requirement.

## Corpus and commands

[`tests/successive_operations.rs`](../tests/successive_operations.rs) runs the
chains in [`tests/support/successive_operations.rs`](../tests/support/successive_operations.rs).
Each step feeds the previous result to the next operation under `STRICT` and
records the P39 outcome, certified closed-manifold validation, sizes, and the
stored size of every coordinate. The coordinate size uses
`hyperreal::Real::representation_size`. Box chains are compared with an
independent exact volume oracle on the grid of all box coordinates. The
default test also pins exact triangle rows, so a change to the growth below
must update them.

```sh
cargo test --test successive_operations              # default chains, 13 s in debug
cargo test --release --test successive_operations -- --ignored   # long chains, about 75 s
cargo bench --bench successive_operations            # table below, with allocation
```

## Measurements

One release run of the timing harness: default features, one thread, on a
shared host. Times are focused timer-scale probes, not benchmarks, and two
runs of identical work differed by up to 2x in time. Their allocated bytes
matched, so the comparison uses allocation. "Without
fixes" reran the same build with both hypermesh changes below turned off by
a temporary switch, since removed. Both runs publish identical rows except
where noted. Raw output: SHA-256 `379d507f…0ca9` (with) and `323bc4c3…d5d`
(without), not archived.

**Update after `hyperreal`'s cyclotomic zero proof (2026-10-21).** The table
below was measured before that proof. With it, the rows marked undecided now
complete certified. The 16-gon (four drills), 32-gon (two drills), 10-degree
(three cuts) chains and the sampled sphere's cuts are asserted as certified
by `tests/successive_operations.rs` and `tests/sampled_sphere_strict.rs`. Their
size and cost columns were not re-measured; the 10-degree-tilted 32-gon drill
chain took 5.0, 14.8, 66.4 and 158.7 s for cuts 1 to 4 in one release run.

| Chain | Steps | Last outcome | Triangles, first → last | Max denominator bits | Max graph depth | Last step ms | Last step MB allocated | Total MB allocated | Total MB without fixes |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| square drills | 32 | certified | 48 → 2204 | 14 | 0 | 21 | 10.1 | 172 | 1,148 |
| 4-gon drills (rational) | 16 | certified | 48 → 1600 | 10 | 0 | 9 | 4.9 | 51 | 204 |
| 6-gon drills (sqrt 3) | 8 | certified | 64 → 1212 | 4 | 58 | 8,937 | 9,813.6 | 26,353 | 52,229 |
| 8-gon drills (sqrt 2) | 6 | certified | 80 → 1048 | 1 | 57 | 11,169 | 11,007.7 | 23,133 | 39,618 |
| 16-gon drills (sin pi/8) | 4 | undecided: `surface arrangement point on segment` | 144 → 632 | 1 | 30 | 8,036 | 6,552.8 | 15,392 | 16,077 (undecided `Real sign classification` at step 4) |
| 32-gon drills (sin pi/16) | 2 | undecided: `surface arrangement point on segment` | 272 → 272 | 1 | 9 | 566 | 545.3 | 10,581 | 11,039 |
| cross bars | 20 | certified | 48 → 3324 | 16 | 0 | 22 | 9.9 | 110 | 229 |
| 3-4-5 rotated cube cuts | 16 | certified | 34 → 2592 | 20 | 0 | 41 | 7.6 | 50 | 139 |
| 30-degree cube cuts (sqrt 3) | 8 | certified | 44 → 558 | 5 | 53 | 1,730 | 1,960.7 | 10,578 | 28,712 |
| 45-degree cube cuts (sqrt 2) | 8 | certified | 36 → 796 | 7 | 41 | 8,585 | 9,858.9 | 15,206 | 22,374 |
| 10-degree cube cuts (sin pi/18) | 2 | undecided: `segment_intersection` | 44 → 44 | 1 | 19 | 59 | 62.6 | 160 | 262 |
| cube unions | 16 | certified | 48 → 660 | 8 | 0 | 4 | 1.8 | 17 | 29 |
| octahedron unions | 16 | certified | 24 → 264 | 3 | 0 | 1 | 1.0 | 9 | 10 |
| icosahedron unions (sqrt 5) | 6 | certified | 52 → 276 | 2 | 45 | 2,835 | 1,934.0 | 6,514 | 11,857 |
| ((A-B)-C)+D feedback | 6 | certified | 104 → 648 | 12 | 0 | 10 | 5.7 | 20 | 53 |
| square drills, binary STL round trip | 8 | certified | 48 → 356 | 23 | 0 | 14 | 7.5 | 30 | 30 |
| square drills, OBJ round trip | 8 | certified | 48 → 356 | 52 | 0 | 12 | 3.6 | 16 | 16 |
| extruded L profile cuts | 6 | certified | 72 → 512 | 10 | 0 | 4 | 1.6 | 7 | 10 |
| revolved ring, adjacent axis cuts | 6 | certified | 156 → 54 | 2 | 19 | 611 | 530.4 | 2,359 | 1,056 (undecided `Real sign classification` at step 2) |
| sampled 32x16 sphere cut (known gap) | 1 | undecided: `Real sign classification` | 0 → 0 | 0 | 0 | 1,362 | 1,354.3 | 1,354 | 1,354 |

## Conclusions

- **Exactness holds.** Every completed step is certified, closed and manifold,
  and every box chain matches the oracle exactly. Rational coordinates stay
  rational. Their reduced denominators grow slowly but do grow. Square drills
  need 3 bits; their output reaches 12 bits after 16 holes and 14 after 32.
  Crossing bars from integer inputs, whose minimal output has only integer
  vertices, reach 16 bits after 20 steps. These lengths do not establish a
  bound.
- **Output grows faster than the represented set.** Each Boolean keeps every
  corefinement vertex. New cuts then cross fan edges left by earlier ones.
  Sixteen 4-gon drills leave 1,600 triangles where 332 suffice, and 3-4-5
  rotated cuts grow quadratically. Repeating a cut that does not change the
  set adds nothing, so new geometry drives the growth. See
  [hypermesh future work](../crates/hypermesh/FUTURE_WORK.md#successive-operations).
- **Symbolic chains slow down and then stop deciding.** The extra vertices
  are interpolated from earlier ones, so their expression graphs deepen at
  every step (6-gon drills: depth 5 to 58 over eight drills). Per-step cost
  grows much faster than output: from 44 MB allocated and 43 ms for 64
  triangles to 9.8 GB and 8.9 s for 1,212. Most of that time goes to
  structural sign expansion
  ([hyperreal future work](../crates/hyperreal/FUTURE_WORK.md)). Exact zeros
  among the deep values became undecided: 16-gon drills at drill 4, 32-gon
  drills at drill 2, and 10-degree rotated cuts at the second cut. The
  cyclotomic zero proof now decides them, but their cost still rises steeply
  with each step.
- **Interchange stays certified but drops retained facts.** STL and OBJ
  round trips between steps stay certified. Reimport marks missing source
  history, and STL also marks binary32 projection. The reimported mesh
  carries none of the arrangement's retained facts, so these chains allocate
  the same with and without the fixes below.
- **Tracked elsewhere.** The sampled 32 x 16 sphere was undecided on its
  first cut; it now completes
  ([hypermesh future work](../crates/hypermesh/FUTURE_WORK.md#convex-hulls-of-sampled-spheres)).
  `try_sweep` is not exercised here; `tests/sweep_frames.rs` composes rational
  sweeps with strict Booleans.

## Changes made for this record

- `hypermesh`: a crossing on an axis-aligned rational plane keeps the plane's
  exact coordinate. Before, a revolved ring cut twice in one plane was
  undecided at the second cut. The ring test and a hypermesh unit test fail
  without it.
- `hypermesh`: Boolean outputs retain that their triangles meet only in shared
  vertices or edges, so a later Boolean does not test them against each
  other. The full self-test and the retained path published identical rows on
  ten chains; a hypermesh test pins one. Together with the change above,
  allocation fell 1.7 to 6.7 times on rational chains and 1.5 to 2.7 times on
  symbolic ones. Round trips, whose reimported meshes carry no retained
  facts, are unchanged.
- `hyperreal`: decimals longer than a machine word parsed to unreduced
  rationals that were not marked unreduced. Equal values then hashed
  differently, and OBJ coordinates stored 170-bit denominators instead of 52.
- `hyperreal`: `Real::representation_size` reports stored expression nodes,
  depth and rational bits, the growth measure used here.

Open: the gaps above, the earcut cost noted in
[hypertri future work](../crates/hypertri/FUTURE_WORK.md), and a recursive
drop of deep expression graphs that aborts at depth 20,000. Raw harness output
was not archived in the evidence repository; regenerate it with the command
above.
