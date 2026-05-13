# hyperdrc Source Tree

This folder contains the Rust implementation of `hyperdrc`. The code is kept
shallow on purpose: top-level modules own the CLI, IO, parsers, reports, and
application orchestration, while subfolders hold the larger semantic areas.

## Entry Points

- [`main.rs`](main.rs) is the binary entry point. It parses the CLI and hands
  control to the application layer.
- [`lib.rs`](lib.rs) exposes the crate modules and defines shared PCB sketch
  aliases used by parsers and checks.
- [`app.rs`](app.rs) is the runtime pipeline. It loads configuration, converts
  requested input packages, discovers and parses inputs, applies IPC-D-356 net
  annotations, validates explicit layer roles, runs selected checks, applies
  waivers, collects parser diagnostics, emits summaries, writes SVG overlays,
  and exits nonzero when active violations remain.

## User-Facing Configuration

- [`cli.rs`](cli.rs) defines all `hyperdrc` command line flags and check names.
  New checks should be added to the `Check` enum and `DEFAULT_CHECKS` here when
  they are intended to run by default.
- [`config.rs`](config.rs) defines JSON rule-deck loading and merge behavior
  between config-file values, CLI overrides, and built-in defaults.
- [`waiver.rs`](waiver.rs) defines JSON waiver parsing, matching by stable
  violation ID, check name, layer list, and message text, plus governance
  validation for waiver metadata completeness (reason, owner, review date, source,
  and geometry hash).

## Input And Conversion

- [`io.rs`](io.rs) centralizes input discovery and provenance. It records which
  adapter loaded each input, the input role, the path, and optional conversion
  origin. Report manifests use these records. `--gerber-dir` now discovers
  common package sidecars in the same directory, including Excellon, IPC-D-356,
  BOM, centroid, netlist, README, fabrication drawing, assembly drawing, and
  rout/panel drawing files.
- Production manifest-sidecar flags are now tracked through the same provenance model:
  `--bom`, `--centroid`, `--netlist`, `--fab-drawing`, `--assembly-drawing`,
  `--readme`, and `--rout-drawing` are surfaced in the report input list and
  `file-manifest-readiness`; discovered directory sidecars use the same path.
  BOM, centroid, netlist, README, fabrication drawing, assembly drawing, and
  rout drawing files are also consumed by `production-artifact-readiness` for
  assembly-sidecar structure, reference parity, grouped BOM references, DNP/DNI
  handling, drawing role, and release-note/order-preflight checks. Binary
  spreadsheet-like text sidecars are loaded lossily for now so the check can
  report missing structure instead of aborting the run.
- [`conversion.rs`](conversion.rs) owns external converter integration. The
  current backend shells out to TransJLC and then feeds the converted Gerber
  directory back through the normal Gerber loading path. `--conversion-arg`
  provides backend-specific pass-through flags so additional converter options can
  be surfaced without redesigning the adapter surface.
- [`excellon.rs`](excellon.rs) parses common Excellon drill files into drill
  features and parser diagnostics used by spacing, clearance, aspect-ratio,
  panel, and table checks.
- [`ipc356.rs`](ipc356.rs) parses common IPC-D-356 electrical-test records into
  point reports. Parsed points can annotate nearby KiCad copper and drills,
  support coverage checks, and cross-check drill diameters; malformed recognized
  test records are surfaced as parser diagnostics instead of being silently
  dropped.
- [`sexp.rs`](sexp.rs) is a small S-expression parser used by the KiCad loader.

## Reports And Artifacts

- [`github_annotations.rs`](github_annotations.rs) renders GitHub Actions
  workflow-command annotations so CI logs can surface active findings without a
  separate upload step.
- [`html_report.rs`](html_report.rs) renders a self-contained browser report
  with the SVG overlay, summary table, input manifest, and finding cards.
- [`jsonl.rs`](jsonl.rs) renders JSON Lines output with one run, input,
  diagnostic, or violation record per line for streaming analytics and long-term
  trend stores.
- [`junit.rs`](junit.rs) renders a conservative JUnit XML subset for CI systems
  that expose test report publishers but not SARIF ingestion.
- [`report.rs`](report.rs) defines violation data, parser diagnostics, stable
  violation IDs, summaries, and GeoJSON conversion.
- [`sarif.rs`](sarif.rs) renders SARIF 2.1.0 output for CI and code review
  systems, keeping PCB coordinates and polygons in result properties because
  DRC findings are geometric rather than line-based.
- [`svg_overlay.rs`](svg_overlay.rs) renders active findings into SVG review
  overlays for local review or CI artifacts.

## Subfolders

- [checks](checks/README.md) contains the design-readiness checks, now split
  across generic layer, drill fabrication, board-context, stencil, assembly,
  manifest, artifact, and surface-finish helper modules.
- [geometry](geometry/README.md) contains geometry construction and conversion
  helpers around `csgrs` and `geo`.
- [kicad](kicad/README.md) contains the KiCad board model and parser helpers.

## Development Notes

Most `hyperdrc` behavior is covered by focused unit tests in the module that
owns the behavior. Parser fuzz/property regressions are persisted under
[`../proptest-regressions`](../proptest-regressions/README.md). The
[test guide](../docs/testing.md) explains what each suite looks for and how the
tests exercise the crate. The root [README](../README.md) covers user-facing
commands and repository navigation.
