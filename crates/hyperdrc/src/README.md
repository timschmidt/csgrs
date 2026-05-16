# hyperdrc Source Tree

This folder contains the Rust implementation of `hyperdrc`. The code is kept
shallow on purpose: top-level modules own the CLI, IO, parsers, reports, and
application orchestration, while subfolders hold the larger semantic areas.

## Architectural Choices

The source tree follows a few explicit boundaries:

- Parsers recover useful manufacturing evidence and diagnostics; they do not
  decide whether a design is acceptable.
- Checks consume normalized geometry, board models, sidecar summaries, and
  resolved policy. They emit violations but do not own output formatting.
- Reports are stable data contracts. Output renderers translate the same report
  into text, JSON, JSON Lines, GeoJSON, SARIF, HTML, JUnit, GitHub annotations,
  and SVG overlays.
- The CLI layer maps user intent into library inputs and process exit status.
  It does not contain separate checking behavior.
- Module boundaries follow PCB concepts where possible: layer geometry, drill
  fabrication, board context, assembly, artifacts, stackup constraints, and
  source-format parsing.

## Entry Points

- [`main.rs`](main.rs) is the thin binary entry point. It parses the CLI and
  calls `hyperdrc::run_cli`; all substantive loading, checking, reporting, and
  exit-status decisions live in the library.
- [`lib.rs`](lib.rs) exposes the docs.rs-facing library API, crate-level
  documentation, shared PCB sketch aliases, parser/check/report modules, and
  crate-root re-exports such as `run`, `run_cli`, `Cli`, and `Report`.
- [`app.rs`](app.rs) is the runtime pipeline. It loads configuration, converts
  requested input packages, discovers and parses inputs, applies IPC-D-356 net
  annotations, validates explicit layer roles, runs selected checks, applies
  waivers, collects parser diagnostics, emits summaries, writes SVG overlays,
  and returns a `RunOutcome`. Its `run_cli` wrapper is the only layer that turns
  active findings into a non-zero process exit.

## User-Facing Configuration

- [`cli.rs`](cli.rs) defines all `hyperdrc` command line flags and check names.
  New checks should be added to the `Check` enum and `DEFAULT_CHECKS` here when
  they are intended to run by default.
- [`config.rs`](config.rs) defines JSON rule-deck loading and merge behavior
  between config-file values, CLI overrides, and built-in defaults. It also
  defines the optional stackup and net-class sections consumed by config-driven
  readiness checks, plus manifest freshness settings such as
  `generated_date_stale_days`.
- [`assembly_policy.rs`](assembly_policy.rs) defines assembly profiles and
  resolved thresholds for component clearance, connector rework, testpoint
  access, tooling holes, mouse bites, fiducials, and dense-pad escape checks.
  Profiles cover prototype, production SMT, double-sided SMT, fixture,
  hand-assembly, selective/wave solder, press-fit, and conformal-coating review
  assumptions, including process-specific keepout thresholds.
- [`constraint_policy.rs`](constraint_policy.rs) defines the stackup and
  net-class rule-deck structures consumed by config-driven checks. Stackup
  config carries process metadata such as material, finish, soldermask,
  laminate Dk/Df/Tg, IPC/fabricator class, fabrication capability thresholds,
  and impedance handoff. Net classes can carry width, clearance, current-width,
  voltage-clearance, reference-plane, via-count, layer-count, differential-pair,
  approximate length/skew, and impedance-control target/tolerance intent.
- [`package_policy.rs`](package_policy.rs) defines named package profiles
  (`full-production`, `fabrication-only`, `assembly-only`, and
  `electrical-test`) and resolves those profiles with `required_artifacts` and
  `required_layers` field overrides for manifest-readiness checks.
- [`date.rs`](date.rs) contains day-level Gregorian parsing/comparison helpers
  shared by waiver governance and manifest freshness checks.
- [`waiver.rs`](waiver.rs) defines JSON waiver parsing, matching by stable
  violation ID, check name, layer list, and message text, plus governance
  validation for waiver metadata completeness and freshness (reason, owner,
  ISO `YYYY-MM-DD` review date, source, and geometry hash).

## Input And Conversion

- [`io.rs`](io.rs) centralizes input discovery and provenance. It records which
  adapter loaded each input, the input role, the path, and optional conversion
  origin. Report manifests use these records. `--gerber-dir` now discovers
  common package sidecars in the same directory, including Excellon, IPC-D-356,
  BOM, centroid, netlist, README, fabrication drawing, assembly drawing, and
  rout/panel drawing files.
- [`gerber_metadata.rs`](gerber_metadata.rs) extracts Gerber image setup
  commands such as `%MO...*%` units and `%FS...*%` coordinate format, plus the
  `%LP...*%` dark/clear image polarity stream, `%LM/%LR/%LS` image
  transform stream, `G01`/`G02`/`G03`
  interpolation modes, `G74`/`G75` quadrant modes, `G36*`/`G37*` region-mode
  transitions, `%SR...*%` step-and-repeat transitions, `%AM...*%` aperture
  macros, `%ADD...*%` aperture definitions, `D01`/`D02`/`D03` coordinate
  operations, `%TD...*%` attribute deletes, and `Dnn` aperture-use commands needed for parser diagnostics, and the
  file-level Gerber X2 attributes consumed by manifest checks, currently `.FileFunction`,
  `.Part`, `.FilePolarity`, `.SameCoordinates`, `.CreationDate`, and
  `.GenerationSoftware`, `.ProjectId`, and `.MD5`, plus aperture-level
  `.AperFunction` and object-level `.N`, `.C`, and `.P` intent for structured
  parser diagnostics, without duplicating the geometry parser. Missing,
  duplicate, or conflicting metadata attributes are reported as structured
  parser diagnostics, along with structurally invalid consumed `.FileFunction`
  role forms and non-standard `.Part`, `.FilePolarity`, `.CreationDate`,
  `.GenerationSoftware`, `.ProjectId`, and `.MD5` values, plus malformed common
  image-setup, image-polarity, region-mode, step-and-repeat, aperture-macro,
  aperture-definition/use,
  `.AperFunction`, net, component-refdes, and component-pin forms.
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
- Manifest checks also flag side-role filename conflicts, single-copper packages
  with opposite-side outputs, and paste exports without a matching same-side
  solder mask layer.
- [`conversion.rs`](conversion.rs) owns external converter integration. The
  current backend shells out to TransJLC and then feeds the converted Gerber
  directory back through the normal Gerber loading path. `--conversion-arg`
  provides backend-specific pass-through flags so additional converter options can
  be surfaced without redesigning the adapter surface.
- [`excellon.rs`](excellon.rs) parses common Excellon drill files into drill
  features and parser diagnostics, including `METRIC`/`INCH` plus `M71`/`M72`
  unit commands, `M48`/`%`/`M30` program-structure evidence, unsupported
  unit-like declarations, zero-suppression/tool, unit-declaration summary
  evidence, tool-table summary evidence, routed-command summary evidence,
  drill-hit and drill-geometry summary evidence, routed-slot
  command warnings, and filename-inferred PTH/NPTH plating intent
  used by spacing, clearance, aspect-ratio, panel, and table checks.
- [`ipc356.rs`](ipc356.rs) parses common IPC-D-356 electrical-test records into
  point reports. Parsed points can annotate nearby KiCad copper and drills,
  support coverage checks, cross-check drill diameters, and carry optional
  access-side, feature-type, and soldermask hints for fixture-access readiness.
  Report-level record-code and sidecar-metadata counts preserve recognized
  `317`/`327`/`367` dialect evidence, access-side coverage, feature-class
  coverage, soldermask-access coverage, net-name coverage,
  reference-designator/pin coverage, diameter-field coverage, and coordinate
  envelope coverage; malformed recognized test records are surfaced as parser
  diagnostics with report-level diagnostic summary counters instead of being
  silently dropped.
- [`sexp.rs`](sexp.rs) is a small S-expression parser used by the KiCad loader.

## Reports And Artifacts

- [`baseline.rs`](baseline.rs) renders proposed waiver-stub JSON,
  active-finding baseline JSON, and baseline comparison JSON. Baseline
  comparison buckets current findings into new, resolved, and unchanged groups so
  release review can track drift without using baselines as suppressions.
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
  across generic layer, drill fabrication, board-context, mechanical, stencil,
  assembly, manifest, artifact, stackup/net-constraint, and surface-finish
  helper modules.
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
