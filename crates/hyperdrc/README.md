<h1>
  hyperdrc
  <img src="./docs/hyperdrc.png" alt="Hyper, a clever math thief" width="144" align="right">
</h1>

`hyperdrc` is a Rust command line tool for design readiness checks on PCB
Gerber and KiCad board files. It uses the latest git version of
[`csgrs`](https://github.com/timschmidt/csgrs) for Gerber parsing, polygon
offsets, and boolean geometry.

## Current Status

The project is an active prototype with a broad regression suite for
fabrication-readiness rules. It currently supports layer-level Gerber checks,
net-aware KiCad checks, Excellon drill sidecars, IPC-D-356 electrical-test
sidecars, JSON/GeoJSON/text reports, SVG review overlays, JSON waivers, and JSON
rule configuration with CLI overrides.

The implemented checks are useful for CI and local design review, but they are
not a replacement for a fabricator's final DFM/DRC pass. Some geometry is still
conservative: KiCad oval and rectangular drill declarations are treated as
circular keepouts using their larger dimension until exact routed-slot geometry
is modeled, and IPC-D-356 parsing focuses on common test records rather than the
full fixed-column dialect.

## Usage

Run all checks against one or more Gerber layers:

```sh
cargo run -- path/to/top.gbr path/to/bottom.gbr
```

Load every Gerber-like file from a directory:

```sh
cargo run -- --gerber-dir path/to/gerber-package
```

Convert a Gerber package with
[`TransJLC`](https://github.com/HalfSweet/TransJLC) before loading the converted
outputs:

```sh
cargo run -- \
  --convert-input path/to/source-gerbers \
  --conversion-output-dir build/hyperdrc-converted \
  --source-eda kicad \
  --transjlc-bin TransJLC
```

The conversion path is backend-oriented. `transjlc` is currently the only
backend, and `hyperdrc` invokes it as an external executable using TransJLC's
directory-based `--path`, `--output_path`, `--eda`, `--zip`, and optional
color-silkscreen arguments. Converted Gerber files are then loaded through the
same layer pipeline as direct Gerber inputs.

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

Available checks:

- `mask-island-keepout`: offsets each mask island and the remaining mask
  geometry by the keepout distance, intersects them, and reports resulting
  violation polygons.
- `copper-overlap`: intersects copper layers and reports overlapping regions.
- `board-edge-clearance`: subtracts the board outline eroded by `--clearance`
  from selected copper layers.
- `board-outline-sanity`: warns when an explicit board outline layer or KiCad
  `Edge.Cuts` data has no closed polygon area.
- `paste-overhang`: reports paste outside paired copper, allowing
  `--paste-tolerance`.
- `paste-aperture-coverage`: reports paired copper that is not covered by paste
  apertures.
- `exposed-copper`: reports copper intersecting paired solder mask openings.
- `solder-mask-opening-coverage`: reports copper that is not covered by paired
  solder mask openings.
- `silkscreen-overlap`: reports silkscreen intersecting paired blocker geometry
  such as copper, mask openings, V-score, or slot/panel geometry.
- `silkscreen-min-width`: reports silkscreen geometry removed by the
  `--min-width` morphology check.
- `min-copper-neck`: uses an erode-and-grow morphology check to flag copper
  features removed by `--min-width`, with a fallback to avoid false positives on
  simple compliant line features.
- `acid-trap`: reports copper polygon vertices below `--acid-trap-angle`.
- `layer-sanity`: warns on empty layers, missing bounds, and optional
  `--max-layer-area` excursions.
- `mechanical-layer-geometry`: warns when polygon geometry is present on layers
  whose names look mechanical, fabrication, ECO, margin, or user-defined.
- `solder-mask-sliver`: uses an erode-and-grow morphology check to flag mask
  webs removed by `--min-mask-width`.
- `annular-ring`: checks KiCad plated drills against nearby same-net copper.
- `drill-copper-clearance`: offsets KiCad and Excellon drills by
  `--drill-clearance` and intersects copper. Same-net plated drills are
  suppressed; NPTH drills still require copper clearance.
- `drill-spacing`: checks KiCad and Excellon drill edge-to-edge spacing using
  `--drill-clearance`, including conservative slot keepouts parsed from KiCad
  oval/rectangular drill declarations.
- `net-spacing`: checks different-net KiCad copper features on the same layer
  using `--net-clearance`.
- `registration-tolerance`: reports cross-layer KiCad copper features within
  `--registration-tolerance`.
- `panelization-clearance`: checks copper against KiCad panel graphics, KiCad
  NPTH drills, and Excellon drill features using `--panel-clearance`.
- `ipc356-coverage`: warns when IPC-D-356 test records do not have nearby parsed
  KiCad copper within `--ipc356-tolerance`.
- `ipc356-drill-diameter`: warns when nearby IPC-D-356 drill diameter records
  conflict with parsed KiCad drill diameters beyond `--ipc356-tolerance`.

Layer roles are explicit zero-based indexes into the input file list:

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

Layer role validation rejects out-of-range indexes, duplicate explicit indexes,
duplicate explicit layer pairs, marking every Gerber input as silkscreen, and
marking the same Gerber input as both `--board-outline` and an explicit
copper/mask/silkscreen layer.

Output formats are `text`, `json`, and `geojson`. The JSON reports include
stable violation IDs, severity, layers, polygon coordinates, point locations
where applicable, and a short message. SVG review overlays can be written with
`--svg-overlay violations.svg`.

Rule thresholds can be placed in a JSON config file and loaded with `--config`.
CLI flags override config values. See
[examples/hyperdrc-config.json](examples/hyperdrc-config.json).

Waiver files are JSON and can suppress findings by `id`, `check`, `layers`, and
message text. A compact CI summary can be written with `--summary-file`:

```json
{
  "waivers": [
    {
      "check": "acid-trap-candidate",
      "layers": ["F.Cu"],
      "message_contains": "below 30",
      "reason": "accepted connector footprint geometry"
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

KiCad support currently reads the common `.kicad_pcb` S-expression objects used
for readiness checks: nets, pads, segments, vias, zones, `Edge.Cuts` lines,
rectangles, circles, arcs, oval pads, oval/rectangular drill declarations, and
common custom pad primitives. Simple panel feature graphics are detected on
common panel, V-score, tab-route, castellated, and edge-plating layer names.
Excellon support reads common `METRIC`/`INCH` tool definitions and drill hits.
IPC-D-356 support reads common test records and uses them to annotate nearby
KiCad copper or drills with net names when source board objects are missing net
data.

Generic IO support includes direct Gerber files, Gerber directories, converted
Gerber directories, KiCad boards, Excellon drills, IPC-D-356 netlists, JSON
config, JSON waivers, text/JSON/GeoJSON reports, SVG overlays, and compact JSON
summaries.

Not yet modeled: exact routed slot shapes, plated-slot/edge-plating electrical
semantics, KiCad silkscreen text side/mirroring, per-pad paste or mask
attributes, fabricator-specific rule decks, stackup/net-class constraints, and
ODB++/IPC-2581 input.

See [docs/design-readiness-plan.md](docs/design-readiness-plan.md) for suggested
future checks and reporting improvements.
