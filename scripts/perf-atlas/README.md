# Performance atlas

The performance atlas gives reviewers a visual sense of how each operation's
cost changes with its inputs. It shows where cost grows smoothly, where it
jumps (cliffs), where some inputs are unusually cheap (valleys), and where
calls fail or stop. Each participating crate has an `examples/perf_atlas.rs`
built on the [`perf-atlas`](../../crates/perf-atlas) harness. The renderer
combines their results into one self-contained HTML report.

## Running it

```sh
scripts/perf-atlas/run.sh            # full settings for the runner's default crate list
scripts/perf-atlas/run.sh --quick    # short batches and fewer samples
scripts/perf-atlas/run.sh --crates "hyperreal hypersolve" --filter sin
```

Results go to `target/perf-atlas/`: one JSON document per crate and
`index.html`. Open the HTML file in a browser; it needs no network access.
The runner builds selected examples sequentially in release mode with all
features. It skips missing examples and stops at the first failing command.
Existing output files are not cleared, so a failed run can leave an older report
alongside newly written JSON. Use a new `--out DIR` when keeping runs separate.
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

- **Line charts** always use logarithmic time. Magnitude is logarithmic when
  its finite values are positive and span at least 16×. A straight line on
  log-log axes can suggest a power law over the measured range; it does not prove
  algorithmic complexity. Lines connect retained non-error measurements across
  gaps. Each panel has at most four kinds; colours cycle after eight kinds.
- **Fuzz scatters** show measured samples with horizontal jitter. Their dashed
  path uses the upper median at each magnitude, including measured errors and
  ceilings. Missing-duration failures have no dot. Spread can reflect input cost,
  scheduling, or other measurement variation.
- **Heatmaps** use logarithmic colour bins and equal-sized cells. Consult the
  colour key: the palette reverses in dark mode. A measured error gets ×, but an
  empty cell can represent a skipped call or a panic without a duration. A grid
  with no durations has no heatmap.
- **Highlight tables** inspect ordinary series only, across all input documents.
  They list the largest 40 cliffs, largest 20 valleys, and first 40 errors or
  ceilings. Fuzz/grid failures are not included in these totals.
  - A **cliff** is a rise of at least 4× between successive retained `ok`/`single`
    rows, which can have failed or missing rows between them.
  - A **valley** is at least 2× faster than both neighbours in that filtered list.

Hover timings are rounded to three significant figures before browser formatting.
Data tables are summaries: ordinary series omit min/max and iteration counts,
fuzz tables aggregate measured durations, and grid tables omit error flags. Keep
the source JSON for recorded details. Duplicate magnitudes use the last row for
line-chart hover but the first row in the ordinary table.

The renderer expects trusted harness documents with finite plotted magnitudes,
positive durations where present, and consistent matrix shapes. Its format-tag
assertion is not full input validation. Compare equivalent completed workloads
under matched host, toolchain, and settings; labels and report metadata alone do
not establish those conditions.

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

## Evidence

Retain the JSON, source revision, toolchain, host, and workload settings with a
report. Historical run claims and their verification status are recorded in the
[renderer audit](../../docs/research/rustdoc-audit.md#performance-atlas-renderer-and-runner-2026-11-26).
The separate [evidence archive](../../docs/evidence-archive.md) holds published
records; a cited path without a verified artifact is not performance evidence.
