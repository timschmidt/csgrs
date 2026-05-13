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

Rule thresholds can be placed in a JSON config file and loaded with `--config`.
CLI flags override config values. See
[examples/hyperdrc-config.json](examples/hyperdrc-config.json).

## Default Readiness Checks

The default suite includes geometry, drill, sidecar, package, and provenance
checks:

- `mask-island-keepout`, `copper-overlap`, `board-edge-clearance`,
  `board-outline-sanity`, `board-outline-fragments`,
  `board-outline-self-intersection-readiness`, `board-outline-notch-readiness`,
  `board-outline-duplicate-readiness`, `board-outline-nesting-readiness`,
  `board-outline-cutout-clearance`,
  `paste-overhang`,
  `paste-aperture-coverage`, `paste-aperture-ratio`,
  `thermal-pad-paste-windowpane-readiness`,
  `stencil-area-ratio-readiness`,
  `paste-aperture-aspect-ratio-readiness`, `tombstone-paste-imbalance-readiness`,
  `paste-via-exposure-readiness`,
  `minimum-paste-aperture`, `paste-aperture-spacing`,
  `paste-mask-alignment`, `exposed-copper`, `solder-mask-opening-coverage`,
  `solder-mask-expansion`, `solder-mask-overlap-clearance`,
  `solder-mask-board-edge-clearance`, `silkscreen-overlap`, `silkscreen-clearance`,
  `silkscreen-board-edge-clearance`, `silkscreen-min-width`,
  `min-copper-neck`, `acid-trap`, `layer-sanity`, `copper-balance`,
  `mechanical-layer-geometry`, `solder-mask-sliver`, and
  `minimum-mask-opening`, and `solder-mask-opening-spacing`.
- `annular-ring`, `annular-ring-tolerance`, `plating-intent`, `routed-slot-readiness`,
  `castellation-intent`, `castellation-hole-readiness`,
  `drill-copper-clearance`, `via-in-pad-readiness`,
  `board-outline-drill-clearance`, `drill-spacing`, `drill-aspect-ratio`,
  `drill-table-consistency`, `copper-width-readiness`, `copper-net-intent`,
  `teardrop-readiness`, `thermal-relief-readiness`, `plane-clearance-readiness`,
  `board-edge-exposure`, `high-speed-edge-readiness`,
  `edge-copper-pullback-readiness`,
  `high-voltage-edge-readiness`,
  `controlled-impedance-readiness`,
  `edge-stitching-readiness`, `differential-pair-readiness`,
  `differential-pair-spacing-readiness`,
  `differential-pair-via-symmetry-readiness`,
  `reference-plane-readiness`,
  `reference-plane-void-readiness`, `orphaned-zone-readiness`,
  `same-net-island-readiness`, `return-path-readiness`,
  `high-current-readiness`, `power-via-array-readiness`, `thermal-via-readiness`,
  `power-plane-readiness`, `high-current-neck-readiness`, `voltage-clearance-readiness`,
  `sensitive-net-spacing-readiness`, `sensitive-return-readiness`,
  `rf-keepout-readiness`, `chassis-stitching-readiness`, `gold-finger-readiness`,
  `gold-finger-edge-readiness`, `gold-finger-spacing-readiness`,
  `gold-finger-drill-keepout-readiness`,
  `component-edge-clearance-readiness`, `component-hole-clearance-readiness`,
  `connector-rework-clearance-readiness`, `pad-pair-asymmetry-readiness`,
  `connector-return-path-readiness`,
  `decoupling-proximity-readiness`, `esd-protection-readiness`,
  `switch-node-keepout-readiness`,
  `testpoint-coverage-readiness`, `testpoint-accessibility-readiness`,
  `tooling-hole-readiness`, `mouse-bite-readiness`, `fiducial-readiness`,
  `local-fiducial-readiness`, `dense-pad-escape-readiness`,
  `thermal-pad-via-readiness`, `thermal-copper-area-readiness`,
  `hot-component-spacing-readiness`, `thermal-mechanical-keepout-readiness`,
  `net-spacing`, `registration-tolerance`, and
  `panelization-clearance`.
- `excellon-readiness`, `file-manifest-readiness`, `ipc356-coverage`,
  `ipc356-drill-diameter`, `production-artifact-readiness`.
  `file-manifest-readiness` (now validating BOM/centroid/netlist/fab drawing/
  assembly/readme/rout-drawing availability, optional declared copper-layer count,
  KiCad-to-Gerber copper stack parity, odd copper stack counts, orphaned
  companion layers, mixed project/revision/date tags, stale-looking package
  filenames, and sidecars discovered from `--gerber-dir`).
  `production-artifact-readiness` validates common BOM, centroid, netlist,
  README, fabrication drawing, assembly drawing, and rout drawing sidecars for
  required structure, BOM procurement metadata, BOM value/footprint coverage, BOM
  lifecycle/status review, broader lifecycle-risk vocabulary, distinct approved
  alternate coverage, optional unit-cost/price sanity, procurement consistency
  across manufacturer/supplier/lifecycle fields, placeholder release metadata,
  semicolon- and whitespace-delimited BOM/placement tables, quantity/refdes
  agreement, zero-quantity population intent, assembly/build variant handoff,
  grouped reference expansion,
  DNP/DNI parity handling, BOM/centroid
  assembly-side, value, footprint, and rotation parity, unusual reference
  designators, duplicate reference designators, conflicting part metadata,
  polarity/MSL/component-height handoff metadata, malformed placement
  coordinates, unusually large placement coordinates, centroid/netlist
  placeholder metadata, out-of-range rotations, side values, duplicate placement
  coordinates, conflicting centroid metadata, pin/net conflicts, repeated
  netlist rows, one-pin net review, reference parity, DNP/DNI placement
  conflicts, release/manufacturing notes, order-parameter intent, contradictory
  fabrication, layer-count, assembly, coating, programming, and test-fixture
  notes, panel/rout drawing parity, BOM/centroid double-sided assembly
  handoff evidence, BOM-driven through-hole solder, BGA/CSP/LGA inspection, and
  programmable-device handoff evidence, firmware traceability, programming
  method, functional-test acceptance criteria, serialization/barcode handoff,
  fabrication marking zones, packaging/ESD/moisture notes, surface-finish
  compatibility notes for edge contacts, fine-pitch packages, press-fit, and
  wire bonding, selective/wave solder and conformal-coating process notes, fab/assembly
  drawing parity for special fabrication and assembly handoffs, preflight
  evidence, release revision/date consistency, text/drawing role names, empty
  sidecar tables, empty or
  placeholder-sized drawings, and common sidecar extensions.

Important tunables include `--keepout`, `--clearance`, `--min-width`,
`--min-mask-width`, `--acid-trap-angle`, `--annular-ring`,
`--drill-clearance`, `--board-thickness`, `--max-drill-aspect-ratio`,
`--min-paste-area-ratio`, `--max-paste-area-ratio`,
`--stencil-thickness`, `--min-stencil-area-ratio`,
`--max-copper-imbalance-ratio`, `--net-clearance`,
`--registration-tolerance`, `--panel-clearance`, `--ipc356-tolerance`,
`--min-area`, and `--max-layer-area`.

## Waivers And CI

Waiver files are JSON and can suppress findings by `id`, `check`, `layers`, and
message text. The system also emits readiness warnings for incomplete waiver
metadata so production waivers remain auditable: `reason`, `owner`,
`review_date`, `source`, and `geometry_hash` are expected. A compact CI summary
can be written with `--summary-file`:

```json
{
  "waivers": [
    {
      "check": "acid-trap-candidate",
      "layers": ["F.Cu"],
      "message_contains": "below 30",
      "reason": "accepted connector footprint geometry",
      "owner": "DRC reviewer",
      "review_date": "2026-05-01",
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
  --svg-overlay violations.svg
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
- [examples](examples/README.md): runnable configuration examples.
- [benches](benches/README.md): benchmark and smoke-performance entry points.
- [proptest-regressions](proptest-regressions/README.md): persisted fuzz and
  property-test regression seeds.

## Known Gaps

Not yet modeled: exact routed slot shapes, plated-slot/edge-plating electrical
semantics, KiCad silkscreen text side/mirroring, per-pad paste or mask
attributes, fabricator-specific rule decks, stackup/net-class constraints,
semantic XLS/XLSX spreadsheet parsing, richer parser diagnostics for all input
formats, and ODB++/IPC-2581 input.

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
- IPC. *Generic Standard on Printed Board Design: IPC-2221B*. IPC, https://www.ipc.org/TOC/IPC-2221B.pdf. Accessed 13 May 2026.
- IPC. *Bare Substrate Electrical Test Data Format: IPC-D-356B*. IPC, 1 Oct. 2002, https://shop.electronics.org/ipc-d-356/ipc-d-356-standard-only.
- IPC. *Performance Specification for Electroless Nickel/Immersion Gold (ENIG) Plating for Printed Boards: IPC-4552B*. IPC, Apr. 2021, https://www.ipc.org/TOC/IPC-4552B-toc.pdf.
- IPC. *Qualification and Performance Specification for Rigid Printed Boards: IPC-6012D*. IPC, https://www.ipc.org/TOC/IPC-6012D.pdf. Accessed 13 May 2026.
- IPC. *Specification for Electroless Nickel/Electroless Palladium/Immersion Gold (ENEPIG) Plating for Printed Circuit Boards: IPC-4556*. IPC, 5 Feb. 2013, https://shop.electronics.org/ipc-4556/ipc-4556-standard-only/Revision-0/english.
- IPC. *Specification for Immersion Silver Plating for Printed Boards: IPC-4553A*. IPC, 16 June 2009, https://webstore.ansi.org/standards/ipc/ipc4553a2009.
- IPC. *Stencil Design Guidelines: IPC-7525B*. IPC, https://www.ipc.org/TOC/IPC-7525B.pdf. Accessed 13 May 2026.
- OASIS. *Static Analysis Results Interchange Format (SARIF) Version 2.1.0*. Edited by Michael C. Fanning and Laurence J. Golding, OASIS Committee Specification 01, 23 July 2019, https://docs.oasis-open.org/sarif/sarif/v2.1.0/cs01/sarif-v2.1.0-cs01.html.
- Wilcoxon, Ross, Tim Pearson, and David Hillman. "Modeling the Effects of Thermal Pad Voiding on Quad Flatpack No-Lead (QFN) Components." *Journal of Surface Mount Technology*, vol. 36, no. 2, 2023, https://doi.org/10.37665/smt.v36i2.37.
