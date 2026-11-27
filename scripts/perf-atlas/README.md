# Performance atlas

The performance atlas gives reviewers a visual sense of how each operation's
cost changes with its inputs. It shows where cost grows smoothly, where it
jumps (cliffs), where some inputs are unusually cheap (valleys), and where
calls fail or stop. Each participating crate has an `examples/perf_atlas.rs`
built on the [`perf-atlas`](../../crates/perf-atlas) harness. The renderer
combines their results into one self-contained HTML report.

## Running it

```sh
scripts/perf-atlas/run.sh            # full sweep of every crate with an atlas
scripts/perf-atlas/run.sh --quick    # short batches and fewer samples
scripts/perf-atlas/run.sh --crates "hyperreal hypersolve" --filter sin
```

Results go to `target/perf-atlas/`: one JSON document per crate and
`index.html`. Open the HTML file in a browser; it needs no network access.
One crate can also be run directly:

```sh
cargo run --release -p hyperreal --all-features --example perf_atlas -- --out hyperreal.json [--quick] [--filter TEXT] [--seed N]
python3 scripts/perf-atlas/render.py report hyperreal.json
```

The `perf-atlas` workflow runs the same script on demand (Actions → perf-atlas
→ Run workflow) and keeps the report as an artifact. Hosted runners are shared
machines, so their absolute times are indicative only.

## What is measured

Each operation is swept along one magnitude axis, such as operand bits,
argument exponent, polynomial degree, dimension, horizon, point count or
distance to degeneracy. Each sweep runs for several input kinds: different
representations, backends or input shapes. For each point:

- **Construction:** the input is built outside the timer.
- **Batches:** the call repeats until a batch takes at least the target
  duration, then the median, minimum and maximum time per call are recorded
  over several batches.
- **Failures:** a call that returns an error, `None` or `false` is still
  timed, but marked. A fast refusal is not a fast success.
- **Ceiling:** a single call longer than the ceiling ends that series, and
  larger magnitudes are recorded as skipped.

Fuzzed inputs are timed one by one, to show the spread hidden by a median.
Two-argument operations can also be swept over a grid of both operand sizes.

| Setting | Full | `--quick` |
| --- | --- | --- |
| target batch | 2 ms | 200 µs |
| samples per point | 7 | 3 |
| time per point | 250 ms | 25 ms |
| single-call ceiling | 2 s | 250 ms |
| fuzzed inputs per magnitude | 24 | 6 |

Inputs are seeded, so reruns time the same values; `--seed` changes them.

## Reading the report

- **Line charts** plot median time per call against the magnitude. When the
  data spans a wide range, both axes are logarithmic. Then a straight line is a
  power law, and its slope is the exponent: slope 1 is linear and 2 is
  quadratic. Each input kind has its own line, colour and marker shape, with at
  most four kinds per panel.
- **Fuzz scatters** show one dot per random input; the dashed line is their
  median. A tall spread at one magnitude means cost depends on the value, not
  only its size.
- **Heatmaps** show two-argument operations over both operand sizes. Darker
  cells are slower; × marks failed calls, and empty cells were skipped.
- **Highlight tables** at the top list the largest cliffs, valleys, failures
  and ceilings across all crates. Each row links to its chart.
  - A **cliff** is a rise of at least 4× between adjacent magnitudes.
  - A **valley** is a point at least 2× faster than both neighbours.

Every chart has a data table, and hovering shows exact values. Times come
from one host and toolchain. Compare reports only when they were made on the
same machine with the same settings, and with nothing else heavy running.

## Resource atlases: memory and depth

The same declarations also produce resource atlases, which audit how much
memory and how much evaluation depth each operation needs across its inputs
and intermediate results:

```sh
scripts/perf-atlas/run.sh --resources            # memory and depth atlases
scripts/perf-atlas/run.sh --resources --counters # also per-call diagnostic counters
```

Results go to `target/resource-atlas/`. `index.html` links one page per
measure, each drawn with the same line charts, fuzz scatters, heatmaps,
cliff and valley flags and data tables as the timing report. Each point is one
warm call (after a discarded warm-up call) on a freshly built input:

| Atlas | Measure | Source |
| --- | --- | --- |
| Memory | peak live heap above the call's start | counting allocator |
| Memory | allocations, reallocations, bytes requested | counting allocator |
| Memory | live bytes still held by the result | counting allocator |
| Memory | residue: live bytes left after the result is dropped (caches on the input or global caches, or leaks) | counting allocator |
| Depth | stack high-water mark | private measurement stack |
| Depth | result tree depth, nodes, largest stored scalar in bits | `Shaped` results |
| Counters | change in each registered counter, such as dispatch-trace events | `Atlas::counter`, with `--counters` |

How it works, and why it costs production code nothing:

- **Heap.** Each atlas binary installs `perf_atlas::CountingAllocator` as its
  `#[global_allocator]`. It delegates to the system allocator and keeps atomic
  counters. Only atlas examples install it; library builds are unchanged.
- **Stack.** Each measured call runs on a private stack: an anonymous mapping
  with a guard page, entered through `psm::on_stack`, reset with
  `MADV_DONTNEED` before the call and inspected with `mincore` afterwards.
  The deepest touched page, refined to its first nonzero word, gives the
  high-water mark within one page (4 KiB). Transparent huge pages are disabled
  on the mapping. The private stack reserves 1 GiB of address space, so calls
  far deeper than an ordinary 8 MiB thread stack can still be measured; a call
  beyond that hits the guard page and ends the run. Stack depth is measured
  on Linux x86_64 and aarch64 only.
- **Shape.** An operation reports the structure of its result by returning
  `Shaped::new(value, reader)`. The reader runs only in resource atlases and
  only after the measured call; in timing runs wrapping costs one function
  pointer. Hyperreal reports `Real::representation_size` (expression nodes,
  depth and largest stored rational) for arithmetic results.
- **Counters.** `Atlas::counter(name, read)` records the change of a process
  counter across each call. Recording can allocate inside the call, so
  binaries register counters only when `--counters` is passed. Hyperreal
  registers its dispatch-trace event total when built with `dispatch-trace`
  (as `run.sh` does with `--all-features`).
- **Ceilings.** A call over the time ceiling, or whose peak heap exceeds 4 GiB
  (1 GiB in `--quick`), ends its series like a timing ceiling.

Heap and stack figures depend on the host's allocator, toolchain and
optimization settings. Compare resource reports only when they come from the
same host and settings.

## Adding a crate

1. Add `perf-atlas = { path = "../perf-atlas" }` to the crate's
   `[dev-dependencies]`.
2. Write `examples/perf_atlas.rs`, following
   [the Hyperreal atlas](../../crates/hyperreal/examples/perf_atlas.rs):
   - Call `Atlas::from_args("crate")`.
   - For each operation, call `atlas.operation(group, name, axis, description)`,
     then any of `sweep`, `fuzz` and `grid`.
   - Finish with `atlas.finish()`.
3. Make the timed call do real work. Force lazy values to a fixed precision,
   and return `Option`, `Result` or `bool` so that failures are marked.
4. Install the counting allocator for resource atlases:
   `#[global_allocator] static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;`
5. Optionally wrap results in `Shaped` to report their structure, and
   register diagnostic counters behind `atlas.config().counters`.
6. Add the crate to the default list in `run.sh`.

## Archived runs

The first full run (24 November 2026, 442 operations, about 110 minutes on a
40-thread Xeon E5-2680 v2) is archived with its rendered report at
`docs/evidence/2026-11-24-perf-atlas` in the
[evidence repository](../../docs/evidence-archive.md).
