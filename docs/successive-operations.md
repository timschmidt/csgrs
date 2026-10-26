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
cargo test --test successive_operations              # default chains, 10 s in debug
cargo test --release --test successive_operations -- --ignored   # long chains, about 50 s
cargo bench --bench successive_operations            # table below, with allocation
cargo bench --bench successive_operations -- --untouched [<chain>]   # no validation between steps
SUCCESSIVE_OPERATIONS_RAW=rows.tsv cargo bench --bench successive_operations   # also exact raw rows
# Attributed, crash-surviving research records (Python 3.11+, native Unix):
python3 scripts/successive-operations.py build --output /tmp/successive-build
python3 scripts/successive-operations.py run --binary /tmp/successive-build/successive_operations --output /tmp/successive-run --timeout 300 -- '4-gon drills'
python3 scripts/test-successive-evidence.py --binary /tmp/successive-build/successive_operations --output /tmp/successive-check
```

The harness times `chain.step`, the closed-manifold validation after it, and
the drop of the previous result once the new one replaces it, separately.
Values the two results share stay alive, so the drop is the teardown of what
the step did not reuse; the last result's drop is not timed. The bench's
counting allocator reports, per step, the bytes allocated (every `alloc` and
`alloc_zeroed` request plus the growth of every `realloc` that grows; a
shrinking `realloc` adds nothing), the peak live-heap growth, the live heap
when the step starts (the previous result plus everything the process
retains) and the allocator calls (`alloc`, `alloc_zeroed` and `realloc`).
Allocated bytes are traffic, not memory in use, and say nothing about peak
or resident memory. `--untouched` feeds each result to the next step without
validating it or measuring its sizes, so no later step can reuse what
validation cached; the corpus test checks that this changes no result
(`validation_between_steps_does_not_change_results`). The raw rows give
every number in bytes and nanoseconds. Each complete row is written and synced
after its operation, validation, inspection and predecessor drop, before the
next step starts. Journal IO is outside those measured intervals. Raw output
paths must be new; existing evidence is never overwritten.

The research driver embeds a build record in the executable: source revision,
content hashes of tracked and new build inputs, the dirty patch and lock digest,
Cargo command, and the actual compiler invocation and `rustc -Vv`. The build
directory retains the patch, new input files, file manifest and Cargo artifact
records, and verifies that source inputs stayed unchanged during compilation.
`--features`, `--default-features`, `--profile` and `--target` select the build;
defaults are release with only `curve,stl-io,obj-io`. Each attributed build
recompiles the benchmark to embed its own record. Reuse its copied executable
for repeated measurements. Compiler wrappers are replaced for this benchmark;
nested workspace wrappers are unsupported. Direct Cargo builds still work,
but their raw header explicitly says `unattributed`.

The run directory retains the actual executable's SHA-256, embedded build
record, arguments, host/CPU affinity, timeout, stdout/stderr and raw rows. It
does not query the current checkout or the runtime compiler to identify the
build. `run.json` starts as `incomplete`; normal completion, a signal and a
timeout are distinct. A missing completion marker is never a completed run.
Killing the driver itself can leave that initial incomplete state. The
interruption tests cover process termination, not storage-device failure.
Completed harness execution is distinct from certified geometry completion:
the manifest counts every outcome category, including typed failures.

The 25 October R25 check reproduced both old failures (a moved executable
reported an unrelated revision; SIGKILL after completed steps lost the raw
file). Copied-binary, replaced runtime-tool, validated/untouched and forced
termination checks now pass. The 16-step rational drill control retains its
geometry/receipt/representation columns and operation allocation totals. This
does not retroactively establish build identity for historical raw records.
The [R25 admission](research-program.md#r02-experiments-and-r25-harness-milestone)
links the retained build/run evidence and its digest.

## Measurements

One release run of the timing harness per revision: default features, one
thread, on a shared host. Times are focused timer-scale probes, not
benchmarks; other work ran on the host during both runs, and two runs of
identical work have differed by up to 2x in time. Allocated bytes are
deterministic up to the 0.1 MB rounding of each step, so the comparison uses
allocation. "Before" is revision `7c726d4c9e`, the first revision that
reduced Boolean outputs; the values without parentheses are this change,
measured in a later run. Every chain keeps its triangle and vertex counts,
and every step of both runs is certified, closed and manifold. Raw output:
SHA-256 `260cfba7…6d56` (before) and `f7bc70dd…5411` (after), not archived.

| Chain | Steps | Last outcome | Triangles, first → last | Max denominator bits | Max graph depth (before) | Last step ms (before) | Last step MB allocated (before) | Total MB allocated (before) |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| square drills | 32 | certified | 32 → 652 | 3 | 0 (0) | 5.2 (6.8) | 4.8 (4.7) | 76.2 (76.0) |
| 4-gon drills (rational) | 16 | certified | 32 → 332 | 1 | 0 (0) | 2.7 (2.9) | 2.1 (2.1) | 21.8 (21.8) |
| 6-gon drills (sqrt 3) | 8 | certified | 40 → 236 | 2 | 4 (4) | 146 (150) | 171.2 (182.0) | 1,266 (1,305) |
| 8-gon drills (sqrt 2) | 6 | certified | 48 → 228 | 1 | 4 (4) | 347 (355) | 421.6 (438.8) | 2,154 (2,206) |
| 16-gon drills (sin pi/8) | 4 | certified | 80 → 284 | 1 | 5 (5) | 821 (853) | 846.5 (888.0) | 3,998 (4,078) |
| 32-gon drills (sin pi/16) | 2 | certified | 144 → 276 | 1 | 5 (5) | 1,797 (1,807) | 1,820.7 (1,852.0) | 11,860 (11,892) |
| 32-gon drills tilted 10 degrees | 4 | certified | 144 → 540 | 1 | 15 (15) | 7,066 (7,523) | 8,694.0 (9,371.2) | 34,337 (35,498) |
| cross bars | 20 | certified | 32 → 612 | 1 | 0 (0) | 6.1 (6.9) | 4.7 (4.7) | 49.8 (50.0) |
| 3-4-5 rotated cube cuts | 16 | certified | 24 → 204 | 5 | 0 (0) | 1.5 (1.6) | 1.0 (1.0) | 10.5 (10.5) |
| 30-degree cube cuts (sqrt 3) | 8 | certified | 28 → 144 | 2 | 15 (29) | 93.7 (106) | 112.0 (131.5) | 733 (840) |
| 45-degree cube cuts (sqrt 2) | 8 | certified | 24 → 144 | 1 | 17 (31) | 49.3 (84.2) | 62.2 (115.6) | 768 (903) |
| 10-degree cube cuts (sin pi/18) | 3 | certified | 28 → 60 | 2 | 18 (38) | 169 (303) | 178.7 (347.8) | 506 (697) |
| cube unions | 16 | certified | 36 → 316 | 3 | 0 (0) | 2.1 (2.2) | 1.4 (1.4) | 13.5 (13.6) |
| octahedron unions | 16 | certified | 24 → 264 | 3 | 0 (0) | 1.4 (1.5) | 1.2 (1.2) | 10.5 (10.5) |
| icosahedron unions (sqrt 5) | 6 | certified | 44 → 164 | 2 | 27 (29) | 928 (3,346) | 686.3 (2,492.6) | 3,972 (7,913) |
| ((A-B)-C)+D feedback | 6 | certified | 64 → 324 | 5 | 0 (0) | 5.4 (6.2) | 4.4 (4.4) | 16.0 (15.9) |
| square drills, binary STL round trip | 8 | certified | 32 → 172 | 23 | 0 (0) | 5.1 (5.3) | 3.1 (3.1) | 13.6 (13.6) |
| square drills, OBJ round trip | 8 | certified | 32 → 172 | 52 | 0 (0) | 4.7 (4.9) | 1.9 (1.9) | 9.3 (9.4) |
| extruded L profile cuts | 6 | certified | 40 → 120 | 3 | 0 (0) | 1.6 (1.7) | 0.7 (0.7) | 3.6 (3.6) |
| revolved ring, adjacent axis cuts | 6 | certified | 124 → 16 | 2 | 12 (12) | 208 (219) | 216.6 (226.9) | 1,391 (1,440) |
| sampled 32x16 sphere cut | 1 | certified | 992 → 992 | 1 | 22 (22) | 1,936 (1,924) | 2,200.6 (2,212.5) | 2,201 (2,212) |

Allocation over the corpus fell from 69.2 GB to 63.4 GB. The rotated cube
cuts allocate 13 to 27% less and icosahedron unions 50% less, also below the
6,518 MB they allocated before outputs were reduced (`432a2d7ebd`). Drill,
ring and sphere chains allocate 0.3 to 3.4% less. Square drills allocate
0.2 MB more over 32 steps and the feedback chain 0.1 MB more (0.3% and
0.6%); the change's own bookkeeping, such as the broad phase's exclusion
groups, is not separated from the effect of different diagonals there.
Sharing equal output coordinates, the last of these changes, changes no
rational chain's allocation, lowers the icosahedron chain from 4,225 MB to
3,972 MB and the 10-degree cuts by 0.4%, and raises the sampled sphere cut by
5.7 MB (0.3%), measured against `76a7d8aa76` in one run. Bounding the
sharing search per coordinate (2026-10-22) changes no row and no step's
allocation at 0.1 MB resolution, measured against `cc0d350d04` in one full
run each: no chain had more than one candidate group in an approximation
window.

Separate probes of the same chains found constant depth beyond the table:
the 30-degree chain stays at depth 16 and 41 graph nodes from its fourth cut
through its sixteenth, and the 10-degree chain at depth 18 and 60 nodes for
eight cuts, between 101 and 226 MB per cut (both before coordinates were
shared). Twelve icosahedron unions, counted with a probe allocator that also
counts each reallocation's full size, allocate 712 MB at the fourth union
and 812 MB at the twelfth (849 and 928 ms) while the result grows from 116
to 308 triangles, at depth 27 throughout; at `76a7d8aa76` the same probe
grows from 780 to 1,354 MB (915 to 1,554 ms).

### Validation, teardown and retained heap (2026-10-22)

One release run of the whole corpus in each mode, at `61da737db3` with this
change's harness (one thread, shared host): `-- --untouched` and the default,
which validates between steps. Raw rows: SHA-256 `4b87ef12…f024` (untouched)
and `64728486…2e3e` (validated), not archived. The steps' wall times differ
between the runs because other jobs ran during the validated one, so they are
not compared.

- **Validation does not warm later steps.** Every row has the same outcome,
  triangles and vertices in both modes. 197 of the 200 steps allocate the
  same bytes and the same number of allocator calls; three steps (8-gon
  drills 5 and 6, 30-degree cuts 5) allocate 336 to 344 bytes and 4 to 12
  calls more when untouched. Validation's cached facts therefore save no
  measurable later work here, and the published per-step costs, which
  exclude validation, are also those of a pipeline that never inspects its
  intermediate results.
- **Validation and teardown are small next to the steps.** Validation took
  258 ms over the corpus against 91.5 s of steps. Dropping each previous
  result took 2.0 ms in all (1.3 ms untouched), at most 43 microseconds for
  one result. Deep graphs that could make a drop expensive (see the open
  recursive-drop item below) do not occur in this corpus.
- **Retained heap stays small.** The live heap when a step starts is the
  previous result plus everything the process keeps, including the
  thread-local modeling caches. It grows from 0.04 MB at the first step to
  4.3 MB at the last step of the corpus in both modes, and by at most 1.7 MB
  within one chain (the tilted 32-gon drills, whose results grow to 540
  triangles).

### Construction incidences (2026-10-22)

One release run of the whole corpus at `2aea178386` and one with the change
that takes incidences from construction (shared host, one thread). Every step
keeps its outcome, validation, triangle, vertex and coordinate counts,
denominator bits and graph sizes, and a digest of every validated step's
exact rows is identical. Allocation over the corpus falls from 63,408 MB to
45,125 MB (28.8%): the symbolic chains allocate 5 to 50% less, the icosahedron
unions 1,998 MB instead of 3,972 MB and the tilted 32-gon drills 20,465 MB
instead of 34,337 MB, and the rational chains the same bytes when run alone.
Details are in the
[hypermesh performance record](../crates/hypermesh/PERFORMANCE.md#2026-10-22-take-incidences-from-construction-instead-of-zero-proofs).

### Exact normal forms of crossing coordinates (2026-10-23)

One validated release run of the whole corpus at `6d1f6e467c` and one with
the change that stores segment-plane crossing coordinates as their exact
pi-Laurent normal forms (shared host, one thread; the second build also
contains Hyperlimit's extended relation stage, which no corpus predicate
reaches, so the two builds execute the same instructions without the
compaction). Raw rows: SHA-256 `3dfd84b3…2420` (before) and `63c8174a…9526`
(after), not archived. Every step keeps its outcome, validation, triangle
and vertex counts and denominator bits. Allocation over the corpus falls
from 45,125 MB to 42,951 MB: the 6- and 8-gon drills allocate 46-47% less
and the 30- and 45-degree cuts 52% and 57% less, the 10-degree cuts and the
revolved ring 2.7% less, and the icosahedron unions 0.9% more, where every
compaction attempt declines. The 30-degree cuts keep coordinate depth 4 over
eight steps instead of 15, and the first two 45-degree cuts publish only
rational coordinates. Details, with instruction counts, are in the
[hypermesh performance record](../crates/hypermesh/PERFORMANCE.md#2026-10-23-store-crossing-coordinates-as-their-exact-normal-form).

Not adopted in the harness: resident-set sampling, which needs a
platform-specific reader and would add the allocator's own retention to the
live-heap numbers above; timing the last result's drop, which no later step
depends on; and a driver that alternates processes for A/B timing, which
would be a benchmarking framework rather than a record of this corpus. A/B
comparisons should keep using allocated bytes, which are deterministic, and
treat time as noisy.

## Conclusions

- **Exactness holds.** Every completed step is certified, closed and manifold,
  and every box chain matches the oracle exactly. Rational coordinates stay
  rational, and their denominators no longer grow: square drills keep the
  inputs' 3-bit denominators after 32 holes (14 bits before outputs were
  reduced), and crossing bars from integer inputs keep only integer vertices
  (16 bits before).
- **Output is proportional to the represented set where it is minimal.**
  Each result keeps only vertices where three planes meet, crease ends and
  contacts. Square and 4-gon drill chains have exactly 12 + 20k triangles
  (1,600 for k = 16 before reduction, 332 now), hexagonal drills 12 + 28k,
  and every 3-4-5 rotated cut adds 12 triangles. A cut that leaves the set
  unchanged reproduces its input rows.
- **Coordinate size stays bounded.** Hole corners are drill edges crossing a
  face plane that results retain, so drill chains keep one graph size: depth
  4 or 5 for the 6- to 32-gon chains and 15 for the tilted 32-gon chain.
  Rotated cube cuts overlap the previous cut, so their corners lie on creases
  between a block face and an earlier wall, whose endpoints earlier cuts
  built. Those corners are now built from the crease's two retained face
  planes and the cutting wall, so the 30-, 45- and 10-degree chains kept depth
  15, 17 and 18, where interpolation along the crease reached 29, 31 and 38
  and kept growing. Crossing coordinates in Q(sqrt 3) and Q(sqrt 2) are now
  also stored as their exact normal forms, so the 30- and 45-degree chains
  keep depth 4; the 10-degree chain's sines of pi/18 keep depth 18. Per-step
  cost no longer rises with the step on these chains; the first cut of
  either 32-gon chain still allocates 9 to 10 GB.
- **Coplanar unions keep a constant per-step cost.** Unions of translated
  icosahedra merge four coplanar strips whose one side is a single straight
  edge, so their fewest triangles fan from that edge's ends. Reduction now
  fans from the end the previous result brought (its vertices are used
  first, not because they are older), so each new icosahedron's coplanar faces
  overlap a bounded number of strip triangles, and the broad phase no longer enumerates
  pairs of the previous result's own triangles. The copies also share
  coordinate values that successive unions built in different ways, and every
  exact bound or order over them proved the equality again; each output now
  stores equal coordinates as one expression, so later comparisons are
  structural. The six unions allocate 3,972 MB instead of 7,913 MB, and the
  fourth to sixth allocate 654, 681 and 686 MB for 116, 140 and 164
  triangles, where they allocated 714, 755 and 893 MB. Over twelve unions the
  per-step cost rises 14% while the result grows 2.65-fold, the work of
  reading a larger input
  ([hypermesh future work](../crates/hypermesh/FUTURE_WORK.md#kept-equal-output-coordinates-share-one-expression)).
- **Interchange stays certified and minimal.** STL and OBJ round trips between
  steps stay certified. Reimport marks missing source history, and STL also
  marks binary32 projection. The reimported mesh carries none of the
  arrangement's retained facts, but the next Boolean decides the coplanarity
  of its adjacent triangles itself, so these chains also reach the minimal
  12 + 20k triangles.
- **Tracked elsewhere.** `try_sweep` is not exercised here;
  `tests/sweep_frames.rs` composes rational sweeps with strict Booleans.

## Changes made for this record

- `hypermesh`: where a Boolean cuts an edge of an earlier certified result
  that joins two triangles whose plane classes were retained as distinct
  planes, the new vertex is the intersection of those two support planes and
  the cutting plane, computed by Cramer's rule. Both planes contain the edge
  and are distinct, so they meet exactly in its line; the crossing is proper,
  so the three normals are independent and the point is the one that
  interpolating between the edge's endpoints gives. It is used only when the
  endpoints' expression graphs are deeper than the two planes', so rational
  endpoints and unchanged source vertices interpolate as before. A coordinate
  that both endpoints store as one expression keeps it, one that an exact
  axis plane fixes keeps its rational value, and a division that does not
  decide falls back to interpolation.
- `hypertri`: `triangulate_simple_ring_around` fans a simple ring around a
  chosen vertex wherever it sees the polygon. `hypermesh` output reduction
  uses it to fan every retriangulated polygon around its least arena vertex,
  the first used; Hypertri's ear clipping had fanned around whichever vertex
  its first rejected ear left, which put two of the icosahedron chain's four
  strips' fans at their growing end.
- `hypermesh`: the broad phase skips pairs of triangles from one certified
  embedded input and records their radial separation from edge adjacency,
  which gives the arrangement the same facts.
- `hypermesh`: each Boolean output stores the coordinates of one axis that
  strict policy proves equal as one expression, the smallest of them, so a
  later Boolean compares them structurally instead of proving their
  difference zero again. Values never change, and an undecided comparison
  keeps both expressions. A segment crossing keeps a coordinate both
  endpoints store as one expression, which keeps shared coordinates shared.
- `hypermesh`, `hypertri`, `hyperlimit`: a Boolean arrangement takes the
  incidences its points have by construction (on a face's plane or edges, on
  an authored line, on the plane of a coplanar face) instead of proving the
  zeros, and passes each line's points to Hypertri's triangulation as
  collinear facts. Rows are unchanged.
- `hypermesh`: a segment-plane crossing stores each derived coordinate as its
  exact pi-Laurent normal form when that is smaller, so coordinates in a
  fixed quadratic field stay a few nodes deep. Values are unchanged; rows
  and coordinate sizes change.

Every change except the incidence change published rows but not sets, and
only the last also changes coordinate sizes; see "Boolean
output rows" in [`MIGRATIONS.md`](MIGRATIONS.md). Earlier changes for this
record: Boolean results keep only the vertices their set needs and retain
their plane classes, distinct-plane facts and source support planes; a
crossing on an axis-aligned rational plane keeps the plane's exact
coordinate; Boolean
outputs retain that their triangles meet only in shared vertices or edges, so
a later Boolean does not test them against each other; `hyperreal` reduces
long decimals on parse; and `Real::representation_size` reports the growth
measure used here. The reduction itself cut corpus allocation from 113.8 GB
to 33.7 GB over the twenty chains measured before it.

Open: the earcut cost noted in
[hypertri future work](../crates/hypertri/FUTURE_WORK.md); and a recursive
drop of deep expression graphs that aborts at depth 20,000. Raw harness
output was not archived in the evidence repository; regenerate it with the
command above.
