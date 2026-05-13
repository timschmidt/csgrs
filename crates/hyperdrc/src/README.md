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
  waivers, emits summaries, writes SVG overlays, and exits nonzero when active
  violations remain.

## User-Facing Configuration

- [`cli.rs`](cli.rs) defines all `hyperdrc` command line flags and check names.
  New checks should be added to the `Check` enum and `DEFAULT_CHECKS` here when
  they are intended to run by default.
- [`config.rs`](config.rs) defines JSON rule-deck loading and merge behavior
  between config-file values, CLI overrides, and built-in defaults.
- [`waiver.rs`](waiver.rs) defines JSON waiver parsing and matching by stable
  violation ID, check name, layer list, and message text.

## Input And Conversion

- [`io.rs`](io.rs) centralizes input discovery and provenance. It records which
  adapter loaded each input, the input role, the path, and optional conversion
  origin. Report manifests use these records.
- [`conversion.rs`](conversion.rs) owns external converter integration. The
  current backend shells out to TransJLC and then feeds the converted Gerber
  directory back through the normal Gerber loading path.
- [`excellon.rs`](excellon.rs) parses common Excellon drill files into drill
  features used by spacing, clearance, aspect-ratio, panel, and table checks.
- [`ipc356.rs`](ipc356.rs) parses common IPC-D-356 electrical-test records.
  Parsed points can annotate nearby KiCad copper and drills, support coverage
  checks, and cross-check drill diameters.
- [`sexp.rs`](sexp.rs) is a small S-expression parser used by the KiCad loader.

## Reports And Artifacts

- [`report.rs`](report.rs) defines violation data, stable violation IDs,
  summaries, and GeoJSON conversion.
- [`svg_overlay.rs`](svg_overlay.rs) renders active findings into SVG review
  overlays for local review or CI artifacts.

## Subfolders

- [checks](checks/README.md) contains the design-readiness checks.
- [geometry](geometry/README.md) contains geometry construction and conversion
  helpers around `csgrs` and `geo`.
- [kicad](kicad/README.md) contains the KiCad board model and parser helpers.

## Development Notes

Most `hyperdrc` behavior is covered by focused unit tests in the module that
owns the behavior. Parser fuzz/property regressions are persisted under
[`../proptest-regressions`](../proptest-regressions/README.md). The root
[README](../README.md) covers user-facing commands and repository navigation.
