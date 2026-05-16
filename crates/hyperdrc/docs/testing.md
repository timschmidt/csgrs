# hyperdrc Test Guide

`hyperdrc` keeps tests close to the module that owns the behavior. Test names
are intentionally descriptive: the part before `::tests::` names the subsystem,
and the function name states the condition that should pass or fail.

Run the suite:

```sh
cargo test
```

List every current test name:

```sh
cargo test -- --list
```

The full suite currently covers parser behavior, geometry construction,
design-readiness checks, report serialization, CLI parsing, waiver governance,
conversion command construction, and property-generated edge cases.

## How To Read A Test

Most tests follow one of four patterns:

- `*_reports_*` builds intentionally bad input and asserts that `hyperdrc`
  emits the expected violation or parser diagnostic.
- `*_allows_*`, `*_accepts_*`, `*_is_clean_*`, and `*_has_no_*` build compliant
  input and assert no finding is emitted.
- `*_respects_selected_layers`, `*_defaults_*`, and `*_rejects_*` verify control
  flow, filtering, or validation around CLI/config choices.
- `generated_*` and `arbitrary_*_never_panics` are property tests. They generate
  many inputs and assert invariants such as finite geometry, stable area, or
  panic-free parsing.

## Application Pipeline Tests

Location: [`../src/app.rs`](../src/app.rs)

These tests exercise the runtime pipeline around the checks:

- Input validation tests confirm `hyperdrc` rejects empty input sets,
  out-of-range layer indexes, duplicate layer pairs, conflicting board-outline
  roles, and impossible silkscreen roles before running checks.
- Discovery tests build temporary Gerber/package directories and assert stable
  ordering, JLC-style filename recognition, sidecar discovery, and provenance
  records.
- Loader tests create missing or malformed files and assert the error message
  preserves useful path context instead of failing opaquely.
- Manifest tests build synthetic layers and KiCad board models to verify
  declared copper count, KiCad copper layer count, board outline presence, and
  drill availability are passed into `file-manifest-readiness`, including X2
  Part/FileFunction/FilePolarity/SameCoordinates/CreationDate/
  GenerationSoftware/ProjectId/MD5 metadata parsed from Gerber headers. Manifest
  unit tests also verify
  revision/date/project token consistency, invalid generated dates, stale
  generated dates, future generated dates, and backup/archive name detection,
  filename layer-count convention parity, negative-copper-polarity review,
  invalid X2 Part/FileFunction/FilePolarity/CreationDate/GenerationSoftware/ProjectId/MD5
  diagnostics, partial X2 FileFunction role-evidence coverage, partial and mixed
  X2 FilePolarity evidence, X2 Part single/panel/coupon intent consistency, SameCoordinates
  alignment-evidence consistency, X2 CreationDate freshness, GenerationSoftware and ProjectId provenance gaps, X2 MD5 checksum-evidence coverage, including the configurable generated-date freshness window,
  named package profiles, and field-level required-artifact and required-layer
  policies.
- Parser geometry tests feed KiCad boards with rotated drill offsets,
  chamfered/rounded/trapezoid pads, custom pad primitives, footprint copper
  graphics, explicit unfilled graphic strokes, board-declared wildcard copper
  layer expansion, footprint Bezier aliases, and board/panel graphics to verify
  parser objects land in the same geometry path used by checks.
- Parser diagnostic tests feed malformed Excellon and IPC-D-356 sidecars and
  malformed Gerber X2 metadata, then verify their parser issues are collected
  separately from active DRC violations.
- Binary sidecar tests feed non-UTF-8 spreadsheet-like bytes and verify the run
  remains non-fatal so production-artifact checks can report missing structure.

## Check Tests

Location: [`../src/checks`](../src/checks/README.md)

Check tests use small synthetic board fragments. They usually build circles,
rectangles, traces, zones, drills, or text sidecars directly in memory, call one
check function, and assert either an empty result or a specific violation
message.

### Layer Geometry

Location: [`../src/checks/layer.rs`](../src/checks/layer.rs)

Layer tests verify flattened 2D geometry checks:

- Mask, paste, solder-mask, and silkscreen tests compare paired layers with
  boolean intersection, difference, erosion, or expansion.
- Solder-mask opening-ratio and annular-ring tests check undersized, oversized,
  tight, and compliant paired mask openings plus sparse-opening culling;
  silkscreen text-height tests check tiny legend islands while allowing
  compliant tall glyphs and long rule lines.
- Board-outline tests build rectangles, cutouts, bow ties, duplicate contours,
  reversed contours, nested regions, and sharp notches to verify outline
  sanity, fragment detection, self-intersection detection, notch detection,
  duplicate detection, nesting detection, and cutout clearance.
- Minimum-width tests use erode/grow morphology to confirm thin traces or mask
  webs are reported and compliant features pass.
- Minimum copper-neck regression tests extract the complex project fixture zips
  in `docs/` and verify representative KiCad copper layers complete without the
  whole-layer morphology stall seen on complex pours. App-level smoke tests also
  extract a small Gerber outline subset when present so package
  extraction and Gerber loading stay covered without turning the fixture into a
  long-running full-board DRC job.
- Copper-overlap and exposed-copper tests assert violation coordinates are
  preserved so reports can point to the actual intersection.
- Layer-sanity tests inject empty geometry, huge area, malformed contours, holes,
  self-intersections, and non-finite coordinates to verify defensive reporting.
- Copper-balance tests compare selected copper areas and verify only meaningful
  imbalance is reported.

### Drill And Fabrication

Locations:
[`../src/checks/drill.rs`](../src/checks/drill.rs),
[`../src/checks/excellon.rs`](../src/checks/excellon.rs)

Drill tests verify hole and drill-table readiness:

- Annular-ring tests build pads and drills at passing and failing ring margins,
  including tolerance-driven worst-case failures.
- Drill spacing tests compare circular holes and conservative slot keepouts,
  including tangent holes, multiple violating pairs, and Excellon sidecar hits.
- Drill-to-copper clearance tests distinguish same-net plated holes from
  same-net NPTH holes and verify slot/trace clearance cases.
- Board-outline drill-clearance tests check holes inside, near, just outside,
  and intruding into outline clearance bands, including orientation invariance.
- Plating, routed-slot, castellation, and edge-hole tests check plated versus
  non-plated intent around copper and board edges.
- Drill aspect-ratio tests verify board-thickness-to-hole-diameter limits and
  zero-diameter handling.
- Drill-table consistency tests compare nearby KiCad, Excellon, and IPC-D-356
  drill diameters and allow exact matches within tolerance.
- Excellon readiness tests construct reports with missing units, unit conflicts,
  zero-suppression and unsupported-unit declarations, duplicate/redefined tools,
  unknown tool selections, routed-slot commands, empty drill sets, duplicate
  drill geometry, drill-diameter outliers, plated/non-plated split conflicts,
  and mixed batch units.
- Gerber metadata parser tests construct image setup commands, image polarity
  commands, image transform commands, interpolation and quadrant mode commands,
  coordinate operations, region-mode transitions, step-and-repeat transitions, aperture macro
  definitions, aperture definitions and uses, attribute-delete commands, X2 headers with valid attributes, standardized
  `G04 #@!` comments, missing required values, duplicate attributes, conflicting
  redefinitions, and structural validation for the FileFunction role forms HyperDRC consumes. They also cover X2/X3
  `.AperFunction` extraction and common malformed aperture-function forms, plus
  `.N`, `.C`, and `.P` object attributes for net, component-refdes, and
  component-pin parser evidence.

### Board Context

Location: [`../src/checks/board.rs`](../src/checks/board.rs)

Board tests use synthetic `BoardModel` values with KiCad-like copper features,
drills, outlines, zones, nets, and sidecars:

- Net and spacing tests verify unnetted copper, different-net spacing, selected
  layer filtering, and trace-distance fallback behavior.
- High-speed, differential-pair, reference-plane, return-path, and stitching
  tests use net-name heuristics plus nearby ground copper/vias to check signal
  integrity readiness, including pair-side return/guard proximity.
- Mixed-signal tests verify sensitive analog/RF/sensor spacing, local
  ground-return proximity, and digital/control partition heuristics.
- High-current, power-plane, power-via-array, thermal-via, thermal-pad, and
  thermal-copper tests verify power and heat-spreading heuristics around zones,
  via count/distribution, and large pads.
- Gold-finger tests verify finger identification, edge proximity, spacing,
  via-on-finger risk, and drill keepout.
- Edge, high-voltage, edge-copper pullback, chassis, RF keepout, antenna
  copper keepout, RF via-fence, sensitive-net, ESD protection and return-path,
  switch-node, connector, inductor copper keepout, decoupling, and
  hot-component tests verify proximity heuristics using board outlines,
  same-layer ground, ground vias, chassis, or protection copper.
- Component, mechanical-hole, panelization, mouse-bite, tooling-hole, fiducial,
  fiducial-keepout, local-fiducial, and dense-pad tests verify
  DFA/manufacturing geometry around
  pads, holes, panel graphics, and dense clusters.
- IPC-D-356 tests verify test records annotate nearby copper, recover missing
  drill net/diameter metadata, report missing coverage, and detect drill
  diameter conflicts.

### Mechanical Context

Location: [`../src/checks/mechanical.rs`](../src/checks/mechanical.rs)

Mechanical tests build synthetic KiCad-like boards with large non-plated holes
and nearby copper:

- Mounting-hole grounding tests verify large NPTH holes are reported when no
  ground/chassis copper is nearby and accepted when chassis copper provides
  bonding evidence.
- Mounting-hole copper keepout tests place non-ground copper inside the
  screw/standoff keepout, verify ground copper is ignored, and check selected
  layer filtering.
- Mounting-hole edge-clearance tests compare the screw/washer keepout against a
  parsed board outline and verify missing outlines are non-fatal.
- Mounting-hole plating-intent tests flag large plated mounting-style holes with
  no ground/chassis evidence and accept explicit ground nets or nearby chassis
  copper.
- Mounting-hole distribution tests report single-hole and clustered hardware
  patterns while accepting absent or well-spaced mounting hardware.
- Mounting-hole spacing tests report tight edge-to-edge hardware-hole spacing
  while ignoring tiny non-hardware drills and accepting compliant gaps.
- Panel-feature outline tests report parsed panel graphics when no board
  outline is available or when the graphic floats away from the outline, while
  accepting absent panel features and edge-registered panel geometry.
- Edge-plating intent tests report edge-named copper near the outline and
  ordinary copper crossing outside the outline, while accepting interior copper
  and selected-out layers.
- Castellation pitch tests report tight plated edge-hole candidates and accept
  distant or interior plated holes.

### Stackup And Net Constraints

Location: [`../src/checks/constraints.rs`](../src/checks/constraints.rs)

Constraint tests use typed config structures and synthetic KiCad copper:

- Stackup tests compare configured copper-layer counts and layer lists against
  parsed KiCad copper layers, then check missing copper weights and missing
  dielectric/core/prepreg thickness entries when a finished thickness is
  declared. They also check material family, surface finish, soldermask
  process/color, target IPC class, fabricator profile, built-in/custom
  fabrication capability thresholds, laminate Dk/Df/Tg policy ranges, and
  HASL-style controlled-impedance finish warnings.
- Net-class tests match exact nets and simple `*` wildcard patterns, then verify
  configured minimum trace width, current-carrying width, different-net
  clearance, voltage-class clearance, maximum layer count, minimum via count for
  layer-changing nets, maximum via count, explicit differential-pair side and
  spacing rules, approximate parsed copper length/skew limits,
  inherited class defaults, rectangular region scopes,
  missing/cyclic/conflicting inheritance diagnostics,
  reference-plane intent, impedance-control target/tolerance handoff metadata,
  and first-pass single-ended outer-layer microstrip plus centered-stripline
  impedance estimates for complete stackups.
- Passing tests build compliant or unmatched nets to ensure explicit config
  constraints do not affect unrelated copper.

### Assembly

Location: [`../src/checks/assembly.rs`](../src/checks/assembly.rs)

Assembly tests focus on manufacturing intent that is not just copper clearance:

- Pad-pair asymmetry tests compare nearby two-terminal pads and report likely
  tombstoning/reflow risk when neighbors are unbalanced.
- Component edge, hole, and spacing clearance tests verify component-like copper
  stays away from board edges, mechanical holes, and neighboring large pad
  proxies on the same side.
- Connector rework and return-path tests check dense connector neighborhoods and
  edge-rate nets for nearby ground return.
- Testpoint tests compare likely critical nets against IPC-D-356 records and
  report missing probe diameter, tight spacing, board-edge, missing access-side,
  missing or covered soldermask-access metadata, and IPC-D-356 access-side hints
  that disagree with nearby same-net KiCad pad/via side. Copper-clearance tests
  place unrelated KiCad copper inside and outside the probe keepout and verify
  same-net or selected-out copper is ignored.
- Tooling, mouse-bite, fiducial, fiducial-keepout, local-fiducial, and
  dense-pad tests verify assembly fixture and panelization readiness, including
  same-layer copper intrusions into fiducial optical keepouts.
- Process keepout tests place likely through-hole solder, press-fit, and
  no-coat contact/test features near neighboring pads to verify selective/wave,
  insertion-tool, and coating-mask warnings.
- Assembly policy tests verify profile defaults preserve existing production-SMT
  thresholds, process-specific profiles resolve distinct hand/solder/press-fit/
  coating defaults, and field-level overrides replace individual resolved
  values.

### Stencil And Paste

Location: [`../src/checks/stencil.rs`](../src/checks/stencil.rs)

Stencil tests keep paste-printing heuristics separate from generic layer tests:

- Area-ratio tests calculate aperture area-to-wall-area behavior from paste
  geometry and stencil thickness, then assert small apertures are reported.
- Aperture aspect-ratio tests detect long sliver apertures and allow compact
  apertures.
- Thermal-pad windowpane tests report a single large paste aperture and allow
  split windows or small pads.
- Tombstone imbalance tests compare paired neighboring paste apertures and allow
  balanced or distant pairs.
- Paste-via exposure tests compare paste apertures to via drill openings and
  allow distant or unselected vias.

### Manifest And Production Artifacts

Locations:
[`../src/checks/manifest.rs`](../src/checks/manifest.rs),
[`../src/checks/artifacts.rs`](../src/checks/artifacts.rs),
[`../src/checks/surface_finish.rs`](../src/checks/surface_finish.rs)

Manifest and artifact tests use synthetic filenames and short text sidecars:

- Manifest tests classify Gerber names into copper, mask, paste, silkscreen,
  outline, drill, and companion roles, with X2 FileFunction coverage for opaque
  filenames. They verify missing required roles, duplicate core roles, odd
  copper counts, inner layers without outers, orphan side outputs,
  single-copper packages with opposite-side outputs, paste exports without
  same-side mask companions, side-token filename conflicts, X2 negative copper
  polarity, invalid X2 Part/FileFunction/FilePolarity/CreationDate/GenerationSoftware/ProjectId/MD5 values, partial X2 FileFunction role evidence, partial or mixed X2 Part intent, partial or mixed X2
  FilePolarity evidence, partial or mixed X2 SameCoordinates evidence, mixed X2 CreationDate values, partial or mixed X2
  GenerationSoftware provenance,
  partial or mixed X2 ProjectId revision provenance, partial X2 MD5 checksum evidence, mixed
  project/revision/date tags, filename layer-count conflicts, stale
  backup/archive names, and complete packages.
- BOM tests parse CSV/TSV/semicolon/whitespace tables and check required
  columns, blank parts, grouped reference expansion, quantity/refdes agreement,
  DNP/DNI handling, duplicate refs, lifecycle risk vocabulary, approved
  alternates, optional cost sanity, procurement consistency, placeholder cells,
  polarity/MSL/height metadata, and programming/fixture handoff triggers.
- Centroid tests check required placement columns, reference parity, side
  values, rotations, duplicate coordinates, unusual coordinates, placeholder
  cells, and value/package/rotation parity with BOM rows.
- Netlist tests check missing columns, empty rows, placeholder cells, duplicate
  pin assignments, repeated rows, one-pin nets, and parity against BOM/centroid
  references.
- README tests check revision and manufacturing notes, order parameters,
  contradictory fabrication/assembly/test/coating/programming intent, panel and
  double-sided handoff parity, preflight evidence, variant handoff, surface
  finish notes, marking/serialization/packaging notes, conditional process
  notes, preflight overlay/waiver-diff evidence, and claimed
  engineering-review-packet completeness.
- Drawing tests check filename role clarity, common extensions, non-empty
  content, placeholder-sized files, and parity with special fabrication or
  assembly requests.

### Distance Helpers

Location: [`../src/checks/distance.rs`](../src/checks/distance.rs)

Distance tests isolate geometric fallback math used by multiple checks:

- Segment tests cover endpoint touch, collinear overlap, parallel gaps,
  projections onto segment interiors, degenerate segments, and non-finite
  endpoints.
- Polygon boundary-distance tests cover separated polygons, touching polygons,
  holes, symmetry between hole and outer inputs, invalid coordinates, and empty
  geometry.

## Parser And Model Tests

### Geometry Primitives

Location: [`../src/geometry`](../src/geometry/README.md)

Geometry tests verify low-level polygon construction:

- Circle, rectangle, line, arc, transform, and polygon-from-points tests cover
  valid shapes, degenerate inputs, signed dimensions, non-finite coordinates,
  closed/open rings, clockwise/counterclockwise arcs, full-circle arcs, and
  zero-radius or zero-angle cases.
- Sketch and multipolygon tests verify metadata preservation, hole preservation,
  empty sketch handling, and strict area filtering.
- Property tests generate many circles, rectangles, lines, arcs, and transforms
  to assert positive finite area, expected area, radius preservation, one polygon
  per nonzero arc chord, edge-length preservation, and area preservation.

### KiCad

Locations:
[`../src/kicad.rs`](../src/kicad.rs),
[`../src/kicad`](../src/kicad/README.md)

KiCad tests parse small `.kicad_pcb` snippets:

- Minimal and malformed board tests verify empty boards are accepted and bad
  S-expressions return errors.
- Basic board tests check nets, pads, tracks, vias, zones, drills, and outlines
  are extracted into the simplified model.
- Pad geometry tests cover rounded rectangles, trapezoid `rect_delta` pads,
  rotations, and expanded copper layers.
- Drill parser tests verify offset pad drill centers are rotated into board
  coordinates before checks see them.
- Custom pad tests verify primitive lines, Bezier strokes, conservative text
  bounding boxes, and shapes are rotated, transformed, and skipped when
  degenerate.
- Footprint copper-graphics tests verify `fp_*` lines, rectangles, circles,
  arcs, polygons, Bezier curves, and text become unnetted copper while
  non-copper graphics are ignored.
- Oval and rectangular drill tests verify current conservative keepout behavior.
- Panel graphic tests parse common panelization layer names, arcs, and Bezier
  strokes.
- Edge-line stitching tests verify unordered outline segments are stitched into
  a single outline polygon.
- Zone tests skip underdefined polygons while preserving valid zones.

### Excellon And IPC-D-356

Locations:
[`../src/excellon.rs`](../src/excellon.rs),
[`../src/ipc356.rs`](../src/ipc356.rs)

Parser tests verify manufacturing sidecars:

- Excellon tests parse metric and inch drill hits, unit declarations, tool
  conflicts, unknown tools, non-numeric coordinates, routed-slot commands,
  `M71`/`M72` unit commands, `M48`/`%`/`M30` program-structure markers,
  unit-declaration summary counters, tool-table summary counters, routed-command summary counters,
  drill-hit and drill-geometry summary counters, unsupported unit-like declarations, missing units with parsed geometry,
  filename-inferred PTH/NPTH plating intent, and hits before active tool
  selection.
- Excellon property tests generate metric hits and arbitrary text to verify
  finite drill geometry and panic-free parsing.
- IPC-D-356 tests parse loose and fixed test records, ignore comments and
  unknown record types, report malformed recognized records, preserve optional
  access-side/feature/soldermask metadata, count recognized `317`/`327`/`367`
  record classes, summarize access-side/feature/soldermask coverage at the
  report boundary, summarize net-name, reference/pin, and diameter-field
  coverage, summarize coordinate/dia range envelopes, summarize parser issue
  categories, and preserve finite coordinates from generated records.
- IPC-D-356 property tests feed arbitrary text to ensure malformed netlists do
  not panic the parser.

### S-Expressions

Location: [`../src/sexp.rs`](../src/sexp.rs)

S-expression tests verify quoted atoms, escaped quotes, unbalanced-list
rejection, atom roundtrips, and panic-free parsing of arbitrary text.

## CLI, Config, Conversion, IO, Reports, And Waivers

These tests protect user-facing behavior around the check engine:

- CLI tests verify multiple checks and input paths parse, unknown check names
  are rejected, Gerber directories and conversion flags parse, manufacturing
  sidecar flags parse, and every report format enum variant is accepted.
- Config tests verify malformed JSON is rejected, unknown config fields are
  ignored, CLI overrides take precedence over config values and defaults, and
  package-profile defaults can be overridden by individual manifest policy
  fields. They also parse assembly profiles, assembly threshold overrides, and
  extended process-profile defaults.
- Conversion tests verify TransJLC command construction, pass-through arguments,
  color-image arguments, zip/output-directory options, missing converter errors,
  command failure context, and successful output directory handoff.
- IO tests verify Gerber extension and keyword detection, false-positive
  rejection, stable directory discovery, missing directory errors, sidecar role
  classification, and source-record serialization.
- Report tests verify stable violation IDs, summary counts, GeoJSON point and
  polygon features, SARIF rule/result/geometry properties, JSON Lines
  run/input/diagnostic/violation records, GitHub annotation escaping, HTML
  escaping/overlay embedding, JUnit XML escaping, SVG overlay behavior, waiver
  stub generation, active-finding baseline records, and baseline diff bucketing
  for new/resolved/unchanged findings.
- Waiver tests verify malformed waiver rejection, matching by ID/check/layer/
  message scope, non-matching waivers leaving findings active, and governance
  warnings for missing reason, owner, review date, source, or geometry hash.
  They also cover malformed ISO review dates and expired review dates.

## Bench Smoke Target

Location: [`../benches`](../benches/README.md)

The benchmark target is not a correctness suite. It is a smoke-performance
entry point for parser, geometry, local copper-density, and trace-junction
acid-trap hot paths, plus dense-pad via-spacing, mask-bridge review, and
single-ended net-class microstrip/centered-stripline impedance estimation.
It also exercises inherited and region-scoped net-class clearance defaults so
selector, resolver, and diagnostics overhead remain visible in the smoke path.
Behavioral expectations belong in the module-level unit and property tests
above.

Return to the [docs README](README.md).
