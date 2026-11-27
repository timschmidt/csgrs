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

- **Construction:** the input factory runs outside the timer. Work performed
  inside the callback, including rebuilding lazy expressions, is timed. By
  default the factory is invoked for each call; shared caches and
  external factory state are not reset.
- **Batches:** one initial call estimates a fixed number of calls per batch.
  Batches need not reach the target duration. Their average times per call supply
  the minimum, maximum, and upper median. The reported iteration count is calls
  per batch, not the total number of calls.
- **Failures:** only the initial result is inspected. `None`, `Err`, and `false`
  mark failures, while `Some(Err(...))`, `Ok(false)`, and numeric results still
  count as success. Callbacks must check their intended result and required
  evidence. Ordinary failed probes remain timed and do not stop a sweep.
- **Ceiling:** an initial call over the ceiling ends the sweep after it returns;
  its record retains the duration but discards the result's failure status and
  reason. Remaining magnitudes are skipped. A timing-growth heuristic can also
  stop a series before attempting a magnitude. Caught unwinding panics stop it
  with an error record.

Fuzz series sample multiple seeded fixtures per magnitude, requesting one timing
batch per fixture and quarter-sized target/budget settings. A ceiling or panic ends the
current magnitude; two consecutive such magnitudes end the series. Grid rows use
at most three samples and half the point budget. Their stopping boundaries block
the affected column and all columns to its right in subsequent rows.

Point budgets are checked between batches, and later calls are not individually
checked against the ceiling. These settings do not interrupt a stuck callback or
bound input generation, total work, or memory.

| Setting | Full | `--quick` |
| --- | --- | --- |
| target batch | 2 ms | 200 µs |
| maximum samples per point | 7 | 3 |
| point-budget threshold | 250 ms | 25 ms |
| initial-call ceiling | 2 s | 250 ms |
| maximum fuzzed inputs per magnitude | 24 | 6 |

Operation and kind labels select seeded streams; `--seed` changes the base seed.
Matching inputs also require deterministic factories and the same sampling path.
Timing-based stops can change which samples are reached, particularly in fuzz
series and grids. Different kind labels use different streams, so comparable
series do not automatically measure identical inputs.

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

## Adding a crate

1. Add `perf-atlas = { path = "../perf-atlas" }` to the crate's
   `[dev-dependencies]`.
2. Write `examples/perf_atlas.rs`, following
   [the Hyperreal atlas](../../crates/hyperreal/examples/perf_atlas.rs):
   - Call `Atlas::from_args("crate")`.
   - For each operation, call `atlas.operation(group, name, axis, description)`,
     then any of `sweep`, `fuzz` and `grid`.
   - Finish with `atlas.finish()`.
3. Make the timed call do the work being claimed. Force lazy values to the stated
   precision or materialization boundary, and return a status that checks the
   intended result and required evidence. `Option` and `Result` check only their
   outer variant; the harness does not validate their payloads or certainty.
4. Add the crate to the default list in `run.sh`.

## Archived runs

The first full run (24 November 2026, 442 operations, about 110 minutes on a
40-thread Xeon E5-2680 v2) is archived with its rendered report at
`docs/evidence/2026-11-24-perf-atlas` in the
[evidence repository](../../docs/evidence-archive.md).
