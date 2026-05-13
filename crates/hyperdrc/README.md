<h1>
  hyperdrc
  <img src="./docs/hyperdrc.png" alt="hyperdrc logo" width="144" align="right">
</h1>

`hyperdrc` is a Rust command line tool for PCB design-readiness checks over
Gerber, KiCad, Excellon, and IPC-D-356 inputs. It uses the latest git version
of [`csgrs`](https://github.com/timschmidt/csgrs) for Gerber parsing, polygon
offsets, and boolean geometry.

## Current Status

`hyperdrc` is an active prototype with a broad regression suite for
fabrication-readiness rules. It supports layer-level Gerber checks, net-aware
KiCad checks, Excellon drill sidecars, IPC-D-356 electrical-test sidecars,
JSON/JSON Lines/GeoJSON/SARIF/HTML/JUnit/text reports, GitHub Actions
annotations, SVG review overlays, JSON waivers, JSON rule configuration,
TransJLC conversion, Gerber-directory sidecar discovery, and structured input
provenance with parser diagnostics.

The implemented checks are useful for CI and local design review, but they are
not a replacement for a fabricator's final DFM/DRC pass. Some geometry is still
conservative: KiCad oval and rectangular drill declarations are treated as
circular keepouts using their larger dimension until exact routed-slot geometry
is modeled, and IPC-D-356 parsing focuses on common test records rather than the
full fixed-column dialect.

## Quick Start

Run all default checks against one or more Gerber layers:

```sh
cargo run -- path/to/top.gbr path/to/bottom.gbr
```

Load every Gerber-like file from a directory. The same directory is also scanned
for common Excellon, IPC-D-356, BOM, centroid, netlist, README, fabrication
drawing, assembly drawing, and rout/panel drawing sidecars:

```sh
cargo run -- --gerber-dir path/to/gerber-package
```

Include pre-production sidecars for manifest-driven readiness checks:

```sh
cargo run -- \
  --check file-manifest-readiness \
  --bom parts.csv \
  --centroid placement.txt \
  --netlist netlist.csv \
  --fab-drawing fab.dxf \
  --assembly-drawing assembly.dxf \
  --readme release-notes.md \
  --rout-drawing panel.dxf \
  --declared-copper-layer-count 4 \
  --kicad-pcb board.kicad_pcb \
  --excellon board.drl \
  top.gbr \
  bottom.gbr
```

Convert a Gerber package with
[`TransJLC`](https://github.com/HalfSweet/TransJLC) before loading the converted
outputs:

```sh
cargo run -- \
  --convert-input path/to/source-gerbers \
  --conversion-output-dir build/hyperdrc-converted \
  --source-eda kicad \
  --transjlc-bin TransJLC \
  --conversion-arg --colorize \
  --conversion-arg --zip-name=upload
```

Run KiCad-aware checks against a `.kicad_pcb` file:

```sh
cargo run -- \
  --config examples/hyperdrc-config.json \
  --kicad-pcb board.kicad_pcb \
  --kicad-copper-layer F.Cu \
  --kicad-copper-layer B.Cu \
  --excellon panel-holes.drl \
  --ipc356 board.ipc \
  --format geojson
```

Run a specific check sequence:

```sh
cargo run -- \
  --check mask-island-keepout \
  --check copper-overlap \
  --keepout 0.2 \
  --pair 0:1 \
  --format json \
  path/to/mask.gbr path/to/copper.gbr
```

Layer roles are explicit zero-based indexes into the Gerber input list:

```sh
cargo run -- \
  --board-outline 2 \
  --copper-layer 0 \
  --copper-layer 1 \
  --paste-pair 3:0 \
  --mask-pair 0:4 \
  --silk-layer 5 \
  --silk-pair 5:4 \
  top.gbr bottom.gbr edge.gbr paste.gbr mask.gbr silk.gbr
```

Output formats are `text`, `json`, `jsonl`, `geojson`, `sarif`,
`github-annotations`, `html`, and `junit`. JSON reports include stable violation
IDs, severity, layers, polygon coordinates, point locations where applicable,
short messages, structured parser diagnostics, and a structured input manifest.
JSON Lines emits one run/input/diagnostic/violation object per line for
streaming analytics. SARIF output preserves stable hyperdrc finding IDs and PCB
geometry in result properties for CI/code-review systems. GitHub annotation
output emits workflow commands that surface findings in Actions logs. HTML
output embeds the SVG overlay with summary, parser diagnostic, input, and
finding tables for review packets. JUnit XML output maps active findings into
testcase failures for CI systems with JUnit publishers. SVG review overlays can
be written with `--svg-overlay violations.svg`.
Active-finding waiver stubs and baselines can be written with `--waiver-stubs
waiver-stubs.json` and `--baseline-file baseline.json`. A current run can also
be compared to a saved baseline with `--baseline-reference previous.json` and
`--baseline-diff-file baseline-diff.json`, producing new, resolved, and unchanged
finding buckets for release review.

Rule thresholds can be placed in a JSON config file and loaded with `--config`.
CLI flags override config values. See
[examples/hyperdrc-config.json](examples/hyperdrc-config.json).

## Readiness Coverage

The default suite covers the main `hyperdrc` readiness surfaces:

- Layer geometry: copper overlap, edge clearance, mask and paste alignment,
  silkscreen clearance, minimum feature width, acid traps, copper balance, and
  board-outline sanity.
- Drill and fabrication context: annular ring, drill spacing, drill-to-copper
  clearance, routed-slot readiness, castellation intent, aspect ratio, and
  cross-source drill-table consistency.
- KiCad board context: net intent, high-speed and high-current heuristics,
  reference-plane and return-path coverage, gold fingers, ESD proximity,
  panelization clearance, component edge/hole clearance, dense-pad escape, and
  config-driven stackup/net-class constraints.
- Assembly and test readiness: fiducials, tooling holes, mouse bites,
  testpoint coverage/accessibility, pad-pair asymmetry, and IPC-D-356 coverage.
- Production package readiness: Gerber package completeness, sidecar discovery,
  BOM/centroid/netlist structure, README release notes, fabrication and assembly
  drawings, rout drawings, order-parameter consistency, generated-date freshness,
  and surface-finish handoff notes.

The check implementations and exact ownership are documented in
[src/checks](src/checks/README.md). The roadmap and remaining gaps are tracked
in [docs/design-readiness-plan.md](docs/design-readiness-plan.md).

Important tunables include `--keepout`, `--clearance`, `--min-width`,
`--min-mask-width`, `--acid-trap-angle`, `--annular-ring`,
`--drill-clearance`, `--board-thickness`, `--max-drill-aspect-ratio`,
`--min-paste-area-ratio`, `--max-paste-area-ratio`, `--stencil-thickness`,
`--min-stencil-area-ratio`, `--max-copper-imbalance-ratio`, `--net-clearance`,
`--registration-tolerance`, `--panel-clearance`, `--ipc356-tolerance`,
`--min-area`, `--max-layer-area`, and `--generated-date-stale-days`.

## Waivers And CI

Waiver files are JSON and can suppress findings by `id`, `check`, `layers`, and
message text. The system also emits readiness warnings for incomplete waiver
metadata so production waivers remain auditable: `reason`, `owner`,
`review_date`, `source`, and `geometry_hash` are expected. `review_date` must be
an ISO `YYYY-MM-DD` date and is warned when it has expired, so standing
exceptions stay visible in pre-production review. A compact CI summary can be
written with `--summary-file`. Proposed waiver stubs and active-finding
baselines can be generated without suppressing anything. Baseline comparison is
an audit artifact: it classifies drift in the active finding set, but waivers
remain the mechanism for intentionally suppressing accepted findings.

```json
{
  "waivers": [
    {
      "check": "acid-trap-candidate",
      "layers": ["F.Cu"],
      "message_contains": "below 30",
      "reason": "accepted connector footprint geometry",
      "owner": "DRC reviewer",
      "review_date": "2027-05-01",
      "source": "https://jira.example/issues/123",
      "geometry_hash": "sha256:0000"
    }
  ]
}
```

```sh
cargo run -- \
  --kicad-pcb board.kicad_pcb \
  --waiver waivers.json \
  --summary-file summary.json \
  --svg-overlay violations.svg \
  --waiver-stubs waiver-stubs.json \
  --baseline-file baseline.json \
  --baseline-reference previous-baseline.json \
  --baseline-diff-file baseline-diff.json
```

## Repository Map

Each folder has its own local README with the hyperdrc-specific ownership
details for that part of the tree:

- [src](src/README.md): Rust crate structure, runtime pipeline, parsers,
  reports, configuration, and submodule map.
- [src/checks](src/checks/README.md): all design-readiness checks grouped by
  layer, drill, board, stencil, assembly, manifest, artifact, surface-finish,
  and helper responsibilities.
- [src/geometry](src/geometry/README.md): polygon construction, sketch
  conversion, shape extraction, and geometry-test expectations.
- [src/kicad](src/kicad/README.md): KiCad board model, S-expression parsing,
  graphics parsing, and current parser scope.
- [docs](docs/README.md): roadmap, design-readiness backlog, and visual assets.
- [docs/testing.md](docs/testing.md): test-suite guide explaining what the
  current tests look for and how they exercise `hyperdrc`.
- [examples](examples/README.md): runnable configuration examples.
- [benches](benches/README.md): benchmark and smoke-performance entry points.
- [proptest-regressions](proptest-regressions/README.md): persisted fuzz and
  property-test regression seeds.

## Known Gaps

Not yet modeled: exact routed slot shapes, plated-slot/edge-plating electrical
semantics, KiCad silkscreen text side/mirroring, per-pad paste or mask
attributes, fabricator-specific rule-deck libraries, deeper stackup/net-class
semantics such as impedance and length matching, semantic XLS/XLSX spreadsheet
parsing, richer parser diagnostics for all input formats, and ODB++/IPC-2581
input.

See [docs/design-readiness-plan.md](docs/design-readiness-plan.md) for the
long-form design-readiness roadmap.

## References

hyperdrc comments and readiness heuristics cite these design and manufacturing
references where the code implements related checks. Entries are kept in MLA
style so they can be copied into engineering review notes.

- Areny, F. A., et al. "A Study of SnAgCu Solder Paste Transfer Efficiency and Effects of Optimal Reflow Profile on Solder Deposits." *Microelectronic Engineering*, 2011, https://doi.org/10.1016/j.mee.2011.02.104.
- Eurocircuits. "Tombstoning." *Eurocircuits Technical Guidelines*, https://www.eurocircuits.com/technical-guidelines/pcb-assembly-guidelines/tombstoning/. Accessed 13 May 2026.
- FixturFab. "Design for Test: How to Design Test Points for PCB Testing." *FixturFab Resources*, https://fixturfab.com/resources/how-to-test/design-for-test. Accessed 13 May 2026.
- GitHub. "Workflow Commands for GitHub Actions." *GitHub Docs*, https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-commands. Accessed 13 May 2026.
- Harter, Stefan, et al. "The Effect of Area Shape and Area Ratio on Solder Paste Printing Performance." *SMTA International*, 2016, https://www.circuitnet.com/programs/55115.html.
- Hinnant, Howard. "chrono-Compatible Low-Level Date Algorithms." *Howard Hinnant's Date Algorithms*, https://howardhinnant.github.io/date_algorithms.html. Accessed 13 May 2026.
- IPC. *Generic Standard on Printed Board Design: IPC-2221B*. IPC, https://www.ipc.org/TOC/IPC-2221B.pdf. Accessed 13 May 2026.
- IPC. *Bare Substrate Electrical Test Data Format: IPC-D-356B*. IPC, 1 Oct. 2002, https://shop.electronics.org/ipc-d-356/ipc-d-356-standard-only.
- IPC. *Performance Specification for Electroless Nickel/Immersion Gold (ENIG) Plating for Printed Boards: IPC-4552B*. IPC, Apr. 2021, https://www.ipc.org/TOC/IPC-4552B-toc.pdf.
- IPC. *Qualification and Performance Specification for Rigid Printed Boards: IPC-6012D*. IPC, https://www.ipc.org/TOC/IPC-6012D.pdf. Accessed 13 May 2026.
- IPC. *Specification for Electroless Nickel/Electroless Palladium/Immersion Gold (ENEPIG) Plating for Printed Circuit Boards: IPC-4556*. IPC, 5 Feb. 2013, https://shop.electronics.org/ipc-4556/ipc-4556-standard-only/Revision-0/english.
- IPC. *Specification for Immersion Silver Plating for Printed Boards: IPC-4553A*. IPC, 16 June 2009, https://webstore.ansi.org/standards/ipc/ipc4553a2009.
- IPC. *Stencil Design Guidelines: IPC-7525B*. IPC, https://www.ipc.org/TOC/IPC-7525B.pdf. Accessed 13 May 2026.
- OASIS. *Static Analysis Results Interchange Format (SARIF) Version 2.1.0*. Edited by Michael C. Fanning and Laurence J. Golding, OASIS Committee Specification 01, 23 July 2019, https://docs.oasis-open.org/sarif/sarif/v2.1.0/cs01/sarif-v2.1.0-cs01.html.
- Wilcoxon, Ross, Tim Pearson, and David Hillman. "Modeling the Effects of Thermal Pad Voiding on Quad Flatpack No-Lead (QFN) Components." *Journal of Surface Mount Technology*, vol. 36, no. 2, 2023, https://doi.org/10.37665/smt.v36i2.37.
