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
independent exact volume oracle on the grid of all box coordinates. Drill
chains whose set fixes the fewest triangles and vertices a triangulation needs
assert exactly that size; the other chains pin their measured sizes, so a
change to the growth below must update them.

```sh
cargo test --test successive_operations              # default chains, 9 s in debug
cargo test --release --test successive_operations -- --ignored   # long chains, about 50 s
cargo bench --bench successive_operations            # table below, with allocation
```

## Measurements

One release run of the timing harness per revision: default features, one
thread, on a shared host. Times are focused timer-scale probes, not
benchmarks; other work ran on the host during both runs, and two runs of
identical work have differed by up to 2x in time. Allocated bytes are
deterministic, so the comparison uses allocation. "Before" is revision
`432a2d7ebd`, which kept every corefinement vertex; the columns without
parentheses are this change. The 10-degree-tilted 32-gon drill chain was
added to the harness for this record; its "before" row is the same chain
built against `432a2d7ebd`. Every step of both runs is certified, closed and
manifold. Raw output: SHA-256 `4e6b4b3c…1f76` (before) and `98e32be3…23be`
(after), not archived.

| Chain | Steps | Last outcome | Triangles, first → last | Triangles before | Max denominator bits (before) | Max graph depth (before) | Last step ms (before) | Last step MB allocated (before) | Total MB allocated | Total MB before |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| square drills | 32 | certified | 32 → 652 | 2,204 | 3 (14) | 0 (0) | 6.6 (21) | 4.7 (10.1) | 76 | 172 |
| 4-gon drills (rational) | 16 | certified | 32 → 332 | 1,600 | 1 (10) | 0 (0) | 2.9 (9.0) | 2.1 (4.9) | 22 | 51 |
| 6-gon drills (sqrt 3) | 8 | certified | 40 → 236 | 1,212 | 2 (4) | 4 (58) | 153 (8,327) | 182.0 (9,813.6) | 1,305 | 26,353 |
| 8-gon drills (sqrt 2) | 6 | certified | 48 → 228 | 1,048 | 1 (1) | 4 (57) | 353 (9,964) | 438.8 (11,007.7) | 2,206 | 23,133 |
| 16-gon drills (sin pi/8) | 4 | certified | 80 → 284 | 980 | 1 (1) | 5 (40) | 853 (5,590) | 888.0 (6,139.2) | 4,078 | 11,782 |
| 32-gon drills (sin pi/16) | 2 | certified | 144 → 276 | 692 | 1 (1) | 5 (19) | 1,807 (3,056) | 1,852.0 (3,070.3) | 11,892 | 13,106 |
| 32-gon drills tilted 10 degrees | 4 | certified | 144 → 540 | 1,988 | 1 (1) | 15 (55) | 7,571 (311,539) | 9,371.2 (677,349.1) | 35,498 | 845,348 |
| cross bars | 20 | certified | 32 → 612 | 3,324 | 1 (16) | 0 (0) | 6.9 (21) | 4.7 (9.9) | 50 | 109 |
| 3-4-5 rotated cube cuts | 16 | certified | 24 → 204 | 2,592 | 5 (20) | 0 (0) | 1.6 (39) | 1.0 (7.6) | 10 | 50 |
| 30-degree cube cuts (sqrt 3) | 8 | certified | 28 → 144 | 558 | 2 (5) | 29 (53) | 108 (1,587) | 131.5 (1,960.9) | 840 | 10,579 |
| 45-degree cube cuts (sqrt 2) | 8 | certified | 24 → 144 | 796 | 1 (7) | 31 (41) | 85 (7,635) | 115.6 (9,858.9) | 903 | 15,206 |
| 10-degree cube cuts (sin pi/18) | 3 | certified | 28 → 60 | 172 | 2 (3) | 38 (39) | 304 (1,614) | 347.8 (1,851.7) | 697 | 2,516 |
| cube unions | 16 | certified | 36 → 316 | 660 | 3 (8) | 0 (0) | 2.2 (3.5) | 1.4 (1.8) | 14 | 17 |
| octahedron unions | 16 | certified | 24 → 264 | 264 | 3 (3) | 0 (0) | 1.5 (1.6) | 1.2 (1.0) | 10 | 9 |
| icosahedron unions (sqrt 5) | 6 | certified | 44 → 164 | 276 | 2 (2) | 29 (45) | 3,320 (2,481) | 2,492.6 (1,935.4) | 7,913 | 6,518 |
| ((A-B)-C)+D feedback | 6 | certified | 64 → 324 | 648 | 5 (12) | 0 (0) | 6.0 (9.5) | 4.4 (5.7) | 16 | 20 |
| square drills, binary STL round trip | 8 | certified | 32 → 172 | 356 | 23 (23) | 0 (0) | 5.3 (13) | 3.1 (7.4) | 14 | 30 |
| square drills, OBJ round trip | 8 | certified | 32 → 172 | 356 | 52 (52) | 0 (0) | 4.9 (12) | 1.9 (3.7) | 9 | 17 |
| extruded L profile cuts | 6 | certified | 40 → 120 | 512 | 3 (10) | 0 (0) | 1.6 (3.9) | 0.7 (1.6) | 4 | 7 |
| revolved ring, adjacent axis cuts | 6 | certified | 124 → 16 | 54 | 2 (2) | 12 (19) | 221 (399) | 226.9 (400.5) | 1,440 | 1,963 |
| sampled 32x16 sphere cut | 1 | certified | 992 → 992 | 1,048 | 1 (1) | 22 (23) | 1,928 (1,931) | 2,212.5 (2,152.8) | 2,212 | 2,153 |

Over the twenty chains measured at both revisions, allocation fell from
113.8 GB to 33.7 GB; with the tilted chain, from 959 GB to 69 GB. Two chains
allocated more: icosahedron unions (6.5 to 7.9 GB, explained below) and
octahedron unions (9 to 10 MB, the reduction's bookkeeping on a chain that
has nothing to remove). The sampled sphere cut allocated 3% more for the
reduction itself.

## Conclusions

- **Exactness holds.** Every completed step is certified, closed and manifold,
  and every box chain matches the oracle exactly. Rational coordinates stay
  rational, and their denominators no longer grow: square drills keep the
  inputs' 3-bit denominators after 32 holes (14 bits before), and crossing
  bars from integer inputs keep only integer vertices (16 bits before).
- **Output is proportional to the represented set where it is minimal.**
  Each result keeps only vertices where three planes meet, crease ends and
  contacts. Square and 4-gon drill chains have exactly 12 + 20k triangles
  (1,600 for k = 16 before, 332 now), hexagonal drills 12 + 28k, and every
  3-4-5 rotated cut adds 12 triangles (increments grew by 20 per cut before).
  A cut that leaves the set unchanged reproduces its input rows.
- **Drill chains through sampled polygons stay bounded.** Hole corners are
  drill edges crossing a face plane that results now retain, so coordinate
  graphs keep one size: depth 4 or 5 for the 6- to 32-gon chains (58 after
  eight 6-gon drills before) and 15 for the tilted 32-gon chain (55 before).
  Per-step cost no longer rises with the step: the tilted chain takes about
  7.5 s and 9 GB per cut, where its fourth cut took 312 s and 677 GB before,
  and without retained planes it became undecided at that cut. The first cut
  of either 32-gon chain still allocates 9 to 10 GB, as it did before.
- **Overlapping symbolic cuts still deepen, more slowly.** Rotated cube cuts
  overlap the previous cut, so their corners are true intersections with
  earlier walls, built from earlier derived vertices. Depth reaches 29, 31
  and 38 for the 30-, 45- and 10-degree chains (53, 41 and 39 before), and
  allocation fell 3.6 to 17 times.
- **Coplanar unions along a long crease cost more.** Unions of translated
  icosahedra merge four coplanar face strips whose minimal triangulation fans
  out from the ends of one long straight edge. Each new icosahedron's
  coplanar face then overlaps every fan triangle at the growing end. The
  output is 164 triangles instead of 276, but allocation grew from 6.5 to
  7.9 GB, and per-step cost still grows with the number of unions. See
  [hypermesh future work](../crates/hypermesh/FUTURE_WORK.md#coplanar-unions-along-a-long-crease-concentrate-fans).
- **Interchange stays certified and minimal.** STL and OBJ round trips between
  steps stay certified. Reimport marks missing source history, and STL also
  marks binary32 projection. The reimported mesh carries none of the
  arrangement's retained facts, but the next Boolean decides the coplanarity
  of its adjacent triangles itself, so these chains also reach the minimal
  12 + 20k triangles.
- **Tracked elsewhere.** `try_sweep` is not exercised here;
  `tests/sweep_frames.rs` composes rational sweeps with strict Booleans.

## Changes made for this record

- `hypermesh`: Boolean results keep only the vertices their set needs. Facets
  join plane classes from coplanarity the arrangement already decided or that
  an earlier result retained; vertices inside one class, or on a straight
  crease between two distinct planes, are removed and the space around them
  is retriangulated by Hypertri under the operation policy. The argument and
  the two removal passes are documented in `surface_reduction.rs`. This is a
  P27 migration of published rows; see "Boolean output rows" in
  [`MIGRATIONS.md`](MIGRATIONS.md).
- `hypermesh`: certified results retain each triangle's plane class, the
  distinct-plane facts between classes, and the exact support plane of the
  source face it was cut from. The next Boolean merges coplanar triangles
  without retesting them and builds new points on those planes rather than on
  planes rebuilt from derived vertices.

Earlier changes for this record: a crossing on an axis-aligned rational plane
keeps the plane's exact coordinate; Boolean outputs retain that their
triangles meet only in shared vertices or edges, so a later Boolean does not
test them against each other; `hyperreal` reduces long decimals on parse; and
`Real::representation_size` reports the growth measure used here.

Open: the icosahedron-union cost above, the deepening of overlapping symbolic
cuts, the earcut cost noted in
[hypertri future work](../crates/hypertri/FUTURE_WORK.md), and a recursive
drop of deep expression graphs that aborts at depth 20,000. Raw harness output
was not archived in the evidence repository; regenerate it with the command
above.
