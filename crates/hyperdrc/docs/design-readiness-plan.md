# HyperDRC Design Readiness Plan

This project starts as a command line Gerber and KiCad checker built on `csgrs`
`Sketch` geometry. Gerber input covers layer-level polygon checks. KiCad input
adds net, drill, pad, via, track, zone, and board-outline context for richer
readiness checks. Excellon drill files are supported as an industry-standard
sidecar input where possible.

## Implemented Checks

- `mask-island-keepout`: treat each closed polygon or multipolygon island in a
  layer as the target island, offset that island and the remaining mask geometry
  by the requested keepout distance, intersect the two offset regions, and report
  the resulting coordinates as a violation when a non-empty shape remains.
- `copper-overlap`: intersect one copper layer with another copper layer and
  report overlapping geometry that may create unintended capacitance.
- `board-edge-clearance`: erode the board outline by the requested clearance,
  subtract that allowed region from copper, and report any remaining copper.
- `board-outline-cutout-clearance`: detect nested board outlines that represent
  cutouts and report subject geometry that enters those regions within the
  configured clearance.
- `board-outline-sanity`: warn when an explicit board outline layer or KiCad
  `Edge.Cuts` data has no closed polygon area.
- `board-outline-fragments`: warn when an explicit board outline layer or KiCad
  `Edge.Cuts` data parses to multiple disconnected outline regions.
- `board-outline-self-intersection-readiness`: report explicit board-outline
  and parsed KiCad `Edge.Cuts` contours that self-intersect.
- `board-outline-notch-readiness`: report sharp inside-corners in outline
  contours that are likely below router capability.
- `board-outline-duplicate-readiness`: flag duplicate board-outline contours that
  typically indicate accidental overlap or double-exported profile geometry.
- `board-outline-nesting-readiness`: flag nested board-outline contours where
  one contour is fully contained by another, which can indicate malformed profile
  structure.
- `paste-overhang`: subtract paired copper, optionally expanded by a tolerance,
  from paste apertures.
- `paste-aperture-coverage`: subtract paste apertures from paired copper and
  report copper that does not have paste coverage.
- `paste-aperture-ratio`: compare paste area over each paired copper island
  against configured minimum and maximum paste-to-copper ratios.
- `thermal-pad-paste-windowpane-readiness`: warn when a large copper island has
  one high-coverage paste aperture instead of a split or reduced windowpane
  pattern.
- `stencil-area-ratio-readiness`: warn when explicit paste apertures have low
  estimated IPC-7525-style area ratio for the configured stencil thickness.
- `paste-aperture-aspect-ratio-readiness`: warn when paste apertures have high
  aspect ratio, calling out stencil release and paste slumping risk.
- `tombstone-paste-imbalance-readiness`: warn when neighboring similarly sized
  copper pads have a large paste-ratio mismatch that can increase tombstoning
  risk.
- `paste-via-exposure-readiness`: warn when explicit paste apertures overlap
  parsed KiCad via drill openings so via fill, cap, tent, or stencil keepout
  intent is reviewed before assembly.
- `minimum-paste-aperture`: warn when explicit paste layers contain apertures
  whose minimum bounding dimension is below the configured minimum width.
- `paste-aperture-spacing`: warn when apertures on an explicit paste layer are
  closer than the configured minimum width, catching stencil bridge risk.
- `paste-mask-alignment`: join explicit paste/copper and copper/mask pairs by
  their shared copper layer and warn when paste extends outside the matching
  solder mask opening.
- `exposed-copper`: intersect paired copper and solder mask opening geometry.
- `solder-mask-opening-coverage`: subtract solder mask openings from paired
  copper and report copper that would remain covered by mask.
- `solder-mask-expansion`: subtract copper expanded by the configured clearance
  from paired solder mask openings and warn on excessive opening growth.
- `solder-mask-overlap-clearance`: expand paired solder mask openings by the
  configured clearance, subtract the intentional opening, and warn on covered
  copper in that vulnerable band.
- `solder-mask-board-edge-clearance`: warn when explicit mask-opening layers
  fall outside the board outline eroded by the configured clearance.
- `solder-mask-opening-spacing`: warn when openings on an explicit mask layer
  are closer than the configured minimum mask bridge width.
- `silkscreen-overlap`: intersect silkscreen with explicitly paired blocker
  geometry such as copper, paste, or mask openings.
- `silkscreen-clearance`: expand explicitly paired blocker geometry by the
  configured clearance and warn when silkscreen falls inside that clipping
  region.
- `silkscreen-board-edge-clearance`: warn when explicit silkscreen layers fall
  outside the board outline eroded by the configured clearance.
- `waiver-governance`: warn when waiver entries are underspecified. Scope
  checks require at least one of `id`, `check`, `layers`, or `message_contains`;
  metadata checks require non-empty `reason`, `owner`, `review_date`, `source`,
  and `geometry_hash`; review dates must be valid ISO `YYYY-MM-DD` dates that
  have not expired.
- `silkscreen-min-width`: approximate thin silkscreen strokes and text geometry
  by eroding and re-growing silkscreen with half the requested minimum width.
- `min-copper-neck`: approximate thin copper features by eroding and re-growing
  copper with half the requested minimum width, then reporting removed geometry.
- `solder-mask-sliver`: approximate thin solder mask webs by eroding and
  re-growing mask geometry with half the requested minimum mask web width.
- `minimum-mask-opening`: warn when explicit mask-opening layers contain
  openings whose minimum bounding dimension is below the configured mask width.
- `acid-trap`: report copper polygon vertices whose angle is below a configured
  threshold.
- `layer-sanity`: report empty polygon geometry, missing bounds, and optional
  maximum-area excursions that can indicate polarity or layer-role problems.
- `copper-balance`: compare selected Gerber copper layers or parsed KiCad
  copper layers and warn when largest-to-smallest copper area exceeds a
  configured ratio.
- `mechanical-layer-geometry`: warn when polygon geometry appears on layers
  whose names look mechanical, fabrication, ECO, margin, or user-defined.
- `annular-ring`: compare KiCad plated drill diameter against nearby same-net
  copper and report rings below the configured threshold.
- `annular-ring-tolerance`: warn when a plated KiCad drill nominally passes the
  configured annular ring but fails after subtracting the configured
  registration tolerance.
- `plating-intent`: warn when a KiCad NPTH drill has nearby copper or when a
  plated drill has no nearby same-net pad or via copper.
- `routed-slot-readiness`: warn when KiCad non-plated mechanical drill
  diameters are below the configured minimum width, as an initial routed cutter
  manufacturability signal.
- `castellation-intent`: warn when a plated KiCad drill hole crosses the board
  outline, so castellated-hole or plated-edge intent can be reviewed.
- `castellation-hole-readiness`: warn when a plated KiCad drill hole crossing
  the board outline is below the configured minimum castellation diameter.
- `via-in-pad-readiness`: warn when KiCad via copper overlaps a same-net pad on
  the same layer, so fill, tenting, and paste treatment can be reviewed.
- `drill-to-copper-clearance`: offset KiCad and Excellon drill holes by a
  configured clearance and intersect against other-net KiCad copper.
- `board-outline-drill-clearance`: offset KiCad and Excellon drill holes by
  their radius plus the configured drill clearance and report keepout area that
  is not contained by the KiCad board outline.
- `drill-spacing`: compare KiCad and Excellon drill edge-to-edge clearance and
  report holes closer than the configured drill clearance.
- `drill-aspect-ratio`: compare finished board thickness against KiCad and
  Excellon drill diameters and warn when the configured aspect-ratio limit is
  exceeded.
- `drill-table-consistency`: compare nearby KiCad, Excellon, and IPC-D-356
  drill records and warn when sidecar drill diameters conflict.
- `excellon-readiness`: validate Excellon sidecars for unit declarations, tool
  table integrity, unknown-tool/undefined-tool drill hits, and malformed hit
  lines before downstream drill spacing and consistency checks consume geometry.
- `copper-width-readiness`: warn when parsed KiCad copper features on selected
  layers have a minimum bounding width below the configured threshold.
- `copper-net-intent`: warn when parsed KiCad copper still has no net after
  native board parsing and IPC-D-356 annotation.
- `teardrop-readiness`: warn when a narrow KiCad same-net segment enters a pad
  or via on the same layer below the configured minimum width.
- `thermal-relief-readiness`: warn when KiCad same-net pad or via copper
  intersects a copper zone on the same layer, so direct plane connections and
  thermal-relief intent can be reviewed before fabrication.
- `plane-clearance-readiness`: warn when KiCad non-plated mechanical holes
  intersect copper zones on selected layers, so antipad and pour-clearance
  intent can be reviewed.
- `board-edge-exposure`: warn when parsed KiCad copper extends outside the
  parsed board outline, so edge plating, castellations, and copper pullback can
  be reviewed.
- `high-speed-edge-readiness`: warn when likely high-speed KiCad copper is
  inside the configured board-edge clearance band.
- `edge-copper-pullback-readiness`: warn when non-edge-intent copper intrudes into
  the board-edge clearance band, catching accidental near-edge traces and planes.
- `high-voltage-edge-readiness`: warn when likely high-voltage KiCad copper is
  inside the configured board-edge clearance band.
- `edge-stitching-readiness`: warn when likely high-speed or RF/antenna copper is
  near the board edge without nearby parsed ground stitch vias.
- `controlled-impedance-readiness`: warn when likely high-speed KiCad nets span
  multiple selected copper layers without a parsed same-net via transition.
- `differential-pair-readiness`: warn when likely differential-pair KiCad nets
  are missing their mate or put pair sides on different selected copper layers.
- `differential-pair-spacing-readiness`: warn when likely differential-pair
  KiCad copper sides are farther apart than the configured review threshold.
- `differential-pair-via-symmetry-readiness`: warn when one side of a likely
  differential pair has an asymmetric via count or via layer set.
- `differential-pair-return-readiness`: warn when likely differential-pair
  copper has no parsed same-layer ground copper inside the guard/return review
  distance.
- `reference-plane-readiness`: warn when likely high-speed KiCad nets are
  present without a parsed ground zone on selected copper layers.
- `reference-plane-void-readiness`: warn when likely high-speed KiCad copper
  does not overlap parsed ground-zone coverage on selected copper layers.
- `orphaned-zone-readiness`: warn when a KiCad copper zone has no parsed
  same-net pad, via, or segment anchor within the configured net clearance.
- `same-net-island-readiness`: warn when one net appears as disconnected
  copper islands on the same selected layer.
- `return-path-readiness`: warn when likely high-speed KiCad via transitions do
  not have a parsed ground stitching via within the configured net clearance.
- `high-current-readiness`: warn when likely power KiCad nets span multiple
  selected copper layers with fewer than two parsed same-net vias.
- `power-via-array-readiness`: warn when likely power KiCad vias are isolated
  from the rest of their same-net via array.
- `thermal-via-readiness`: warn when likely power or thermal KiCad zones have
  too few parsed same-net vias.
- `power-plane-readiness`: warn when likely power KiCad nets have no parsed
  same-net copper zone on selected layers.
- `high-current-neck-readiness`: warn when likely power KiCad nets have copper
  neck widths below the preferred power width.
- `voltage-clearance-readiness`: warn when likely high-voltage KiCad nets are
  close to different-net copper using an expanded net-clearance threshold.
- `sensitive-net-spacing-readiness`: warn when likely analog, RF, or sensor
  KiCad nets are close to likely noisy power, switching, motor, or high-speed
  nets.
- `sensitive-return-readiness`: warn when likely analog, RF, or sensor KiCad
  nets have no parsed same-layer ground copper nearby.
- `rf-keepout-readiness`: warn when likely RF or antenna KiCad nets are close
  to non-ground copper on the same selected layer.
- `rf-via-fence-readiness`: warn when likely RF or antenna KiCad copper has no
  parsed same-layer ground via inside the configured via-fence review distance.
- `chassis-stitching-readiness`: warn when likely chassis or shield KiCad nets
  have no parsed ground stitching via nearby.
- `gold-finger-readiness`: warn when likely gold-finger or edge-connector KiCad
  nets contain via copper on selected layers.
- `gold-finger-edge-readiness`: warn when likely gold-finger copper is not near
  the board edge where card-edge contacts and beveling are expected.
- `gold-finger-spacing-readiness`: warn when neighboring likely gold-finger
  contacts are closer than the configured finger spacing.
- `gold-finger-drill-keepout-readiness`: warn when likely gold-finger copper
  intersects KiCad or Excellon drill/mechanical keepouts.
- `component-edge-clearance-readiness`: warn when parsed KiCad component pads
  sit inside the configured board-edge assembly clearance band, skipping likely
  edge connectors and fiducials.
- `component-hole-clearance-readiness`: warn when parsed KiCad component pads
  intersect a clearance band around non-plated KiCad or Excellon mechanical
  holes, slots, screw holes, standoffs, or chassis keepouts.
- `component-spacing-readiness`: warn when large same-side KiCad pad proxies sit
  closer than the configured assembly spacing, giving an initial component
  courtyard/body clearance review signal before full courtyard parsing exists.
- `connector-rework-clearance-readiness`: warn when likely connector pads have
  neighboring non-same-net pads inside the configured hand-solder/rework
  clearance band.
- `pad-pair-asymmetry-readiness`: warn when neighboring small pads on the same
  layer have a large copper-area mismatch that can increase tombstoning or
  passive placement risk.
- `connector-return-path-readiness`: warn when likely connector edge-rate nets
  sit near the board edge without nearby same-layer ground return copper.
- `decoupling-proximity-readiness`: warn when likely power pads or vias have no
  parsed same-layer ground copper nearby, giving decoupling loop-area and return
  proximity an early review signal.
- `esd-protection-readiness`: warn when likely edge connector nets sit near the
  board edge without parsed ESD, chassis, or ground protection copper nearby.
- `switch-node-keepout-readiness`: warn when likely switching, boot, gate,
  motor, or PWM nodes are close to non-ground neighboring copper on the same
  selected layer.
- `testpoint-coverage-readiness`: warn when likely critical KiCad nets have no
  matching IPC-D-356 test record, giving production test coverage an early
  sidecar-driven review signal.
- `testpoint-accessibility-readiness`: warn when IPC-D-356 testpoints have no
  parsed probe diameter, undersized probe diameter, tight probe-to-probe
  spacing, insufficient board-edge fixture clearance, missing access-side
  metadata, missing soldermask-access metadata, soldermask-covered access, or
  contradictory SMD/both-side access hints, and when top/bottom IPC-D-356 access
  conflicts with nearby same-net KiCad pad/via side.
- `testpoint-copper-clearance-readiness`: warn when an IPC-D-356 testpoint
  probe keepout intersects unrelated selected KiCad copper, catching fixture
  short and unreliable-contact review risks.
- `tooling-hole-readiness`: warn when parsed KiCad or Excellon drill data has
  fewer than two likely non-plated tooling holes, or when tooling candidates sit
  inside the fixture edge-clearance review band.
- `mouse-bite-readiness`: warn when likely small non-plated mouse-bite drills
  have undersized diameters or nearest-neighbor spacing outside the expected
  perforation range.
- `fiducial-readiness`: infer likely unnetted circular fiducial pads and warn
  when populated copper sides have fewer than two or when candidates sit inside
  the configured edge-clearance review band.
- `local-fiducial-readiness`: warn when dense fine-pitch pad clusters do not
  have at least two likely local fiducials nearby on the same side.
- `fiducial-keepout-readiness`: warn when same-layer KiCad copper intrudes into
  the configured optical keepout around likely fiducial targets.
- `dense-pad-escape-readiness`: warn when dense fine-pitch pad clusters have no
  parsed nearby via escape, so BGA/QFN/fine-pitch breakout strategy is reviewed.
- `selective-wave-solder-keepout-readiness`: warn when likely through-hole
  solder process features sit too close to neighboring pads for wave/selective
  solder pallet, solder-thief, or masking review.
- `press-fit-keepout-readiness`: warn when likely press-fit connector holes sit
  too close to neighboring pads for insertion-tool and deformation-clearance
  review.
- `conformal-coating-keepout-readiness`: warn when likely contacts,
  testpoints, or fiducials have neighboring pads inside a no-coat keepout band.
- `thermal-pad-via-readiness`: warn when large ground or power pads that look
  like exposed thermal pads have no parsed same-net via in pad.
- `thermal-copper-area-readiness`: warn when likely heat or power pads, vias, or
  traces have no parsed same-net copper zone nearby for heat spreading and
  current return.
- `hot-component-spacing-readiness`: warn when likely hot pads or zones sit close
  to neighboring non-ground copper, calling out derating and placement review.
- `thermal-mechanical-keepout-readiness`: warn when likely hot copper sits inside
  the keepout around non-plated mechanical holes, standoffs, screws, chassis, or
  heatsink clearance features.
- `mounting-hole-grounding-readiness`: warn when likely large non-plated
  mounting holes have no parsed nearby ground or chassis copper, so chassis
  bonding versus isolation intent is explicit before release.
- `mounting-hole-copper-keepout-readiness`: warn when non-ground copper enters a
  circular keepout around likely large non-plated mounting holes, catching screw,
  washer, standoff, and chassis-clearance review gaps.
- `mounting-hole-edge-clearance-readiness`: warn when a likely mounting-hole
  screw/washer keepout extends beyond the parsed board outline.
- `mounting-hole-plating-intent-readiness`: warn when a large plated
  mounting-style hole lacks a parsed ground/chassis net or nearby bonding copper.
- `mounting-hole-distribution-readiness`: warn when parsed hardware-style holes
  are single-point or clustered below the mounting distribution review spacing.
- `mounting-hole-spacing-readiness`: warn when likely hardware-hole edge-to-edge
  spacing is below the configured mechanical review spacing.
- `panel-feature-outline-readiness`: warn when parsed KiCad panel, route,
  V-score, tab, or rail graphics have no board outline for registration or sit
  farther from the outline than the panel-feature review distance.
- `edge-plating-intent-readiness`: warn when selected KiCad copper reaches or
  crosses the board outline in a way that should be paired with explicit
  edge-plating, castellation, bevel, or copper-pullback fabrication intent.
- `castellation-pitch-readiness`: warn when plated edge-hole candidates have
  edge-to-edge spacing below the configured castellation pitch review distance.
- `different-net-spacing`: offset same-layer KiCad copper features by a
  configured clearance and report different-net proximity.
- `layer-registration-tolerance`: compare KiCad copper layers with an offset
  tolerance to identify features vulnerable to registration shifts.
- `panelization-clearance`: check copper against KiCad panel graphics, KiCad
  NPTH drills, and Excellon drill hits.
- `ipc356-coverage`: warn when IPC-D-356 electrical test records do not have
  nearby parsed KiCad copper.
- `ipc356-drill-diameter`: warn when IPC-D-356 drill diameter records conflict
  with nearby parsed KiCad drill diameters.
- `stackup-readiness`: compare configured copper-layer count, copper layer
  names, copper weights, finished thickness, and dielectric/core/prepreg
  thickness metadata against parsed KiCad copper layers.
- `net-constraint-readiness`: apply exact-name and wildcard net classes for
  minimum width, current-carrying width, clearance, voltage-clearance,
  layer-count, via-count, reference-plane, and impedance-control handoff review.
- `file-manifest-readiness`: warn when a Gerber package is missing recognizable
  copper, board outline, drill data, or matching solder mask layers, and warn on
  duplicate core manufacturing roles. It now also:
  - flags missing or duplicate BOM, centroid, fab-drawing, assembly-drawing, and
    readme artifacts, plus netlist and rout-drawing artifacts, using named
    `package_profile` defaults (`full-production`, `fabrication-only`,
    `assembly-only`, or `electrical-test`) plus configurable
    `required_artifacts` overrides so workflows can differ without losing
    duplicate detection;
  - flags missing same-side solder mask, solder paste, and silkscreen companion
    layers whenever matching copper layers are detected, using the same
    `package_profile` defaults plus configurable `required_layers` overrides
    for partial handoffs;
  - validates duplicated companion roles;
  - flags orphaned side outputs, inner copper without both outer copper layers,
    odd recognized copper layer counts, one-copper packages that also contain
    opposite-side outputs, paste outputs without a same-side solder mask
    companion, and Gerber filenames whose side tokens conflict with their
    inferred role;
  - checks Gerber-recognized copper layer counts against KiCad-declared copper
    layer counts and optional order-declared layer-count metadata;
  - warns when filenames appear to mix project/job names, revision tags, or
    generated-date tags across Gerbers and package artifacts;
  - warns when generated-date tags are older than the current freshness window
    or later than the run date, with the freshness window configurable from the
    rule deck or CLI; and
  - warns when package filenames include stale/archive tokens such as backup,
    old, previous, or obsolete.
- `production-artifact-readiness`: validate common BOM, centroid, netlist,
  README, fabrication drawing, assembly drawing, and rout drawing sidecars for
  required headers, BOM quantity/refdes agreement, duplicate reference
  designators, malformed placement coordinates/rotations, invalid side values,
  duplicate pin/net assignments, reference parity between
  purchase/placement/netlist artifacts, missing README revision/manufacturing
  notes, BOM compliance/traceability/source-control evidence for sensitive
  rows, centroid placement unit/origin/rotation-convention handoff, non-empty
  drawing files, common drawing extensions, and role-specific drawing filename
  tokens.

## Supported Inputs

- Gerber RS-274X through `csgrs` `gerber-io`.
- Gerber directories and converter-produced Gerber directories. The first
  converter backend shells out to TransJLC to normalize common EDA Gerber
  packages into JLCEDA/JLCPCB-style Gerber names before loading them through the
  same layer pipeline.
- KiCad `.kicad_pcb` S-expression files for common board objects: nets, pads,
  oval pads, oval/rectangular drill declarations, common custom pad primitives,
  tracks, vias, zones, `Edge.Cuts` lines/arcs/rectangles/circles, and panel
  graphics.
- Excellon drill files with common `METRIC`/`INCH` tool definitions and drill
  hit coordinates.
- IPC-D-356 electrical test netlists with common test records. Parsed records can
  annotate nearby KiCad copper and drills by net name, and common loose metadata
  tokens can carry access-side, feature-type, and soldermask hints into
  testpoint-accessibility checks.

## Remaining Checks Requiring Deeper Inputs

- Full KiCad custom pad primitive semantics, including subtractive primitives,
  exact roundrect radii, text primitives, and all graphic variants.
- Broader IPC-D-356 fixed-column dialect coverage, plus deeper cross-checks that
  combine parsed access-side, feature-type, and soldermask flags with explicit
  fixture declarations and mask-opening geometry.
- ODB++ or IPC-2581 ingestion for richer manufacturing stackup and fabrication
  rules.
- Fabricator-specific rule decks, class-based constraints, and board stackup
  tolerances.

## Research-Derived Readiness Backlog

The backlog below combines PCB fabricator DRC/DFM guidance, KiCad preflight
concepts, and assembly-oriented DFA review items. Each item should eventually
have one failing fixture, one passing fixture, and a rule-deck configuration
example before it is considered production-ready.

### Data Model and Rule Deck Foundations

- Unit model: preserve source units, normalize internal distances, and report
  both source and normalized units. Avoid comparing mil/mm thresholds against
  untagged parser output.
- Board stackup model: layer count, copper weight, dielectric/core/prepreg
  thickness, material family, surface finish, soldermask process/color, target
  IPC class, fabricator profile, and initial fabrication capability threshold
  libraries are implemented for readiness review, including laminate dielectric
  constant, loss tangent, and Tg metadata/range checks. Remaining stackup work:
  actual impedance target solving and richer vendor-specific capability decks.
- Net-class model: exact-name and simple wildcard net-class config is
  implemented for minimum width, current-carrying width, minimum clearance,
  voltage-class clearance, maximum layer count, minimum via-count,
  maximum via-count, explicit differential-pair side/spacing rules,
  approximate parsed copper length/skew limits, reference-plane intent, and
  impedance-control target/tolerance metadata review. Remaining net-class work:
  true routed pair length/skew extraction, actual impedance solving, and richer
  class inheritance constraints.
- Manufacturing capability profiles: initial `prototype-fab`, `standard-fab`,
  `advanced-fab`, and custom JSON threshold decks are implemented for stackup
  review, including optional material-property windows. Remaining work:
  vendor-specific JLCPCB, PCBWay, Eurocircuits-style service classes that
  separate hard minimum, preferred minimum, and cost-escalation thresholds.
- Assembly profile: prototype SMT, production SMT, double-sided SMT,
  fixture-focused, hand-assembly, selective-solder, wave-solder, press-fit, and
  conformal-coating threshold profiles are implemented for component clearance,
  connector rework, testpoint access, tooling, mouse-bite, fiducial, and
  dense-pad escape checks plus process-specific solder, press-fit, and coating
  keepout checks. Remaining profile work: package-class-specific reflow
  assumptions and richer process-specific keepout geometry.
- Mechanical model: routed outline, V-score lines, mouse bites, tabs, slots,
  cutouts, countersinks, castellations, plated edges, bevels, tooling holes,
  fiducials, keepout zones, stiffeners, and enclosure constraints. Initial KiCad
  checks now review likely large NPTH mounting holes for nearby ground/chassis
  bonding evidence and non-ground copper keepout intrusion.
- BOM/position model: component outlines, package classes, rotations, heights,
  polarity marks, fiducials, centroid files, assembly-side data, and alternate
  parts.
- File manifest model: expected copper/mask/paste/silk/drill/fab/drawing files,
  BOM/centroid/netlist/readme/assembly/rout artifacts, generated timestamp, source
  EDA, board revision, order parameters, and declared layer count. Initial role
  inference
  and missing/duplicate core-file checks are implemented by
  `file-manifest-readiness`, including named package profiles and configurable
  production-artifact and layer-role requirements.

### Fabrication Geometry Checks

- Minimum copper feature width by layer, copper weight, and service class.
  Initial KiCad checks warn when selected copper features have a minimum
  bounding width below the configured threshold.
- Copper spacing matrix: trace-to-trace, trace-to-pad, trace-to-via,
  pad-to-pad, pad-to-via, via-to-via, copper-to-hole, copper-to-slot,
  copper-to-cutout, and copper-to-board-edge.
- Voltage-aware clearance: compare net voltage classes against internal,
  external-coated, external-uncoated, slot, and creepage/clearance rules.
  Initial KiCad checks warn when likely high-voltage nets are close to
  different-net copper using an expanded net-clearance threshold. Config-driven
  net classes can also set explicit `min_voltage_clearance` values that are
  applied to same-layer different-net copper.
- Annular ring with tolerance: compute nominal, worst-case drill-wander, and
  registration-adjusted ring for vias, component holes, press-fit holes, and
  plated slots. Initial KiCad checks warn when nominal annular ring passes but
  the registration-adjusted ring fails the configured minimum.
- Drill aspect ratio: board thickness divided by finished drill diameter, with
  separate thresholds for standard through holes, microvias, blind vias, buried
  vias, and backdrills. Initial through-hole style checks are implemented for
  KiCad and Excellon drill features with configurable board thickness and
  maximum aspect ratio.
- Drill table consistency: compare Excellon tools, KiCad drill declarations,
  IPC-D-356 drill data, fabrication drawing notes, and order metadata. Initial
  sidecar consistency checks compare nearby KiCad/Excellon and Excellon/IPC-D-
  356 drill records.
- Plated versus non-plated intent: detect copper pads around NPTH holes,
  missing pad stacks around PTH holes, ambiguous slot plating, and NPTH copper
  clearance violations. Initial KiCad checks warn on NPTH drills with nearby
  copper and plated drills without nearby same-net pad or via copper.
- Routed slot geometry: exact obround/rectangular slot outlines, slot width,
  slot end radius, slot-to-copper spacing, slot-to-slot spacing, and minimum
  routed cutter diameter. Initial KiCad checks warn when non-plated mechanical
  drill diameters are below the configured minimum width.
- Board outline validity: duplicate outlines, nested cutouts, outline-to-hole
  tolerance, and router-access validation.
  Initial outline-fragment checks warn when the parsed outline has multiple
  disconnected regions. Initial outline self-intersection, duplicate, nesting,
  and notch checks report self-crossing contours, duplicate/nested contours, and
  sharp router-bottleneck corners.
- Copper balance: layer copper area, local copper density, plane void islands,
  sparse inner layers, high copper imbalance across the stack, and bow/twist
  risk flags. Initial layer-area imbalance checks are implemented for KiCad
  copper layers and explicitly selected Gerber copper layers.
- Isolated copper: floating copper islands, unconnected pours, orphaned zone
  remnants, and copper slivers below etchable area. Initial KiCad net-intent
  checks warn on parsed copper that remains unnetted after optional IPC-D-356
  annotation, and warn when a parsed zone has no same-net pad, via, or segment
  anchor within the configured net clearance. They also warn when a single net
  appears as disconnected copper islands on the same selected layer.
- Acid traps: acute polygon vertices, acute trace junctions, narrow wedge voids,
  and trapped etchant pockets inside plane pours.
- Teardrop recommendations: narrow trace-to-pad and trace-to-via junctions below
  a configured width or annular-ring margin. Initial KiCad checks warn when
  narrow same-net segment geometry enters pads or vias.
- Thermal relief: starved thermals, missing thermal spokes where required,
  excessive spoke width for solderability, asymmetric reliefs, and direct plane
  connections on hand-soldered through-hole pads. Initial KiCad checks warn
  when same-net pads or vias intersect copper zones so the plane connection
  style can be reviewed.
- Plane clearances: antipad size on inner layers, missing clearance pads for
  through-hole pins, copper pour clearance around mechanical holes, and shorts
  from insufficient antipads. Initial KiCad checks warn when non-plated
  mechanical holes intersect copper zones on selected layers.
- Mounting-hole and chassis readiness: screw/washer/standoff copper keepouts,
  explicit chassis bonding, intentional isolation, enclosure clearance, and
  large NPTH classification. Initial KiCad checks infer likely large non-plated
  mounting holes and warn when they lack nearby ground/chassis copper or when
  non-ground copper enters the configured keepout region. They also warn when
  mounting-hole keepouts breach the board outline, when large plated
  mounting-style holes lack ground/chassis bonding evidence, and when parsed
  hardware-style holes are single-point, clustered, or too tightly spaced
  edge-to-edge.
- Board edge exposure: exposed copper at routed edges, unintentional edge
  plating, copper too close to V-score, copper too close to tab routes, and
  missing edge-plating declarations when copper intentionally reaches an edge.
  Initial KiCad checks warn when copper geometry extends outside the parsed
  board outline, and warn when likely high-speed or high-voltage copper enters
  the configured board-edge clearance band. Initial mechanical checks also warn
  when selected copper reaches or crosses the board outline in a way that should
  be paired with edge-plating, castellation, bevel, or copper-pullback release
  intent.
- Gold fingers: finger width/spacing, bevel keepout, mask opening, plating
  finish requirement, no vias/text/silk/paste on fingers, and consistent finger
  length. Initial KiCad checks warn when likely gold-finger or edge-connector
  nets contain via copper, when likely finger copper is away from the board
  edge, when finger spacing is tight, and when finger copper intersects drill or
  mechanical keepouts.
- Castellations and half holes: minimum hole diameter, annular ring, edge
  registration, plating intent, pad pitch, and copper pullback around routed
  board edge. Initial KiCad checks warn when plated drill holes cross the board
  outline, warn when those crossing plated holes are below the configured
  minimum diameter, and now warn when neighboring plated edge-hole candidates
  have tight edge-to-edge spacing below the castellation pitch review distance.
- High-current copper: trace width, neck-down length, via array current sharing,
  copper pour bottlenecks, thermal vias, and connector/pad current-density
  warnings. Initial KiCad checks warn when likely power nets change layers with
  fewer than two parsed same-net vias, and warn when likely power-net copper
  necks fall below the preferred power width, and warn when those likely power
  nets have no parsed same-net copper zone on selected layers. They also warn
  when likely power vias are isolated from the rest of their same-net via array,
  and when likely power or thermal zones have too few same-net vias.
- Controlled impedance readiness: impedance-rule presence, stackup completeness,
  reference-plane continuity, coplanar ground spacing, return-path voids, and
  layer-change via stitching. Initial KiCad checks warn when likely high-speed
  nets span multiple selected copper layers without a parsed same-net via
  transition, and warn when likely high-speed via transitions lack a nearby
  parsed ground stitching via. They also warn when likely high-speed nets are
  present without a parsed ground zone on selected copper layers, and when
  likely high-speed copper does not overlap parsed ground-zone coverage.
  Config-driven net classes can require reference-plane and impedance-control
  handoff metadata, including target impedance and tolerance values, for exact
  nets or wildcard groups.
- Differential pair constraints: exact configured net classes can declare
  `differential_pair`, `differential_role`, pair spacing bounds, and pair-skew
  limits. Initial
  checks verify both sides are present, side layer sets agree, and same-layer
  copper spacing is inside the configured range. KiCad geometry checks also warn
  when pair-side copper lacks nearby same-layer ground copper for guard/return
  intent. Configured length/skew checks use parsed segment bounding-box
  estimates; true routed length/skew extraction is still future work.

### Solder Mask, Paste, and Finish Checks

- Solder mask expansion: mask opening larger than copper pad by the selected
  process margin; flag mask-on-pad and excessive exposure. Initial paired
  copper/mask checks warn when openings exceed the configured clearance around
  copper.
- Solder mask web/sliver: residual mask dam width between pads, vias, and
  openings; classify by mask process and color where known. Initial explicit
  mask-layer checks warn when neighboring openings leave less than the
  configured mask bridge width.
- Mask overlap clearance: copper track or plane too close to mask opening to
  remain reliably covered. Initial paired copper/mask checks warn when covered
  copper falls within the configured mask-opening clearance band.
- Minimum mask opening: openings too small to resolve in the selected mask
  process. Initial explicit mask-layer checks flag openings whose minimum
  bounding dimension is below the configured mask width.
- Mask-to-board-edge behavior: required pullback near routed edges, V-scores,
  panel tabs, and specified exposed-edge areas. Initial explicit mask-layer
  checks warn when openings violate the configured board-edge clearance.
- Via tenting/filling: vias under components, exposed vias in pads, tented-via
  intent versus mask openings, plugged/capped/fill requirements, and mismatch
  between design and order notes. Initial KiCad checks warn when same-net via
  copper overlaps pad copper on the same layer.
- BGA mask rules: NSMD/SMD consistency, mask bridge width, opening ratio, escape
  via proximity, dogbone geometry, and via-in-pad treatment.
- Paste reduction/expansion: paste aperture area ratio versus copper, per-pad
  paste overrides, excessive paste on thermal pads, and missing paste on SMD
  pads. Initial paired-layer island ratio checks are implemented with
  configurable minimum and maximum paste-to-copper area ratios, thermal-pad
  windowpane review, and neighboring-pad paste imbalance review.
- Stencil manufacturability: aperture minimum width, aperture area ratio,
  aspect ratio, home-plate apertures, windowpane thermal pads, fine-pitch bridge
  risk, and tombstoning imbalance on two-terminal parts. Initial explicit paste
  layer checks flag apertures whose minimum bounding dimension is below the
  configured minimum width, apertures whose spacing is below that width, and
  apertures with high aspect ratio or low estimated area ratio for the
  configured stencil thickness.
- Paste-to-mask/copper alignment: paste outside mask opening, paste outside pad,
  paste over vias, paste bridging between adjacent pads, and paste aperture
  slivers below stencil process capability. Initial explicit triple checks warn
  when paste extends outside the solder mask opening associated with the same
  copper layer, and KiCad-plus-paste checks warn when paste overlaps parsed via
  openings.
- Surface finish compatibility: ENIG/ENEPIG/hard gold/HASL constraints for
  fine-pitch, press-fit, gold fingers, wire bonding, and high-voltage creepage.
  Initial README/order-note checks warn when edge contacts lack hard/electrolytic
  gold or ENEPIG intent, when HASL is paired with fine-pitch or array-package
  assembly language, and when press-fit or wire-bond language lacks compatible
  surface-finish context.

### Silkscreen, Legend, and Marking Checks

- Legend line width and text height by fabrication capability.
- Silkscreen overlap with exposed copper, mask openings, paste, vias, holes,
  slots, board edges, V-score, tab routes, and gold fingers. Initial explicit
  silkscreen plus board-outline clearance checks are implemented.
- Silkscreen clipping risk: legend too close to mask cutbacks or pads where the
  fabricator will clip text fragments. Initial explicit silkscreen/blocker
  pairs warn when legend geometry falls within the configured clearance.
- Bottom-side mirroring and side intent for silkscreen text.
- Polarity and pin-1 indicators present and visible for polarized parts,
  connectors, ICs, diodes, LEDs, electrolytics, and batteries.
- Reference designator completeness, duplicate refdes detection, refdes outside
  board outline, unreadable refdes, and assembly drawing consistency. Initial
  production artifact checks flag duplicate BOM/centroid references and
  BOM/centroid/netlist reference mismatches.
- Fabrication marking checks: date code, UL mark, impedance coupon label,
  serialization, revision text, and customer-required markings in allowed zones.
- Fiducial label and keepout clarity: global/local fiducials not covered by
  silk, mask, copper clutter, or components.

### Assembly and DFA Checks

- Component-to-component clearance using courtyard, body, and height data.
  Initial KiCad checks now warn when large same-side pad proxies sit closer than
  the configured assembly spacing, which is a conservative review signal until
  true courtyard/body geometry is modeled.
- Component-to-board-edge clearance for pick-and-place, depanelization, clamps,
  rework, and hand soldering. Initial KiCad checks warn when parsed component
  pads sit inside the board-edge assembly clearance band. The threshold is now
  resolved through `assembly_profile` and `assembly` rule-deck overrides.
- Component-to-hole/slot/mechanical clearance for screws, standoffs, chassis,
  keepout volumes, and connector mating envelopes. Initial KiCad/Excellon checks
  warn when component pads intersect the configured non-plated mechanical-hole
  clearance band. The threshold is now resolved through `assembly_profile` and
  `assembly` rule-deck overrides.
- Orientation consistency for polarized packages and same-package arrays.
- Tombstoning risk: asymmetric pad sizes, thermal imbalance, paste imbalance,
  copper imbalance, and unequal trace connections on small passives. Initial
  KiCad checks warn when neighboring small pads have asymmetric copper area, and
  explicit paste checks warn when neighboring pad paste ratios are imbalanced.
- Fine-pitch bridging risk: pad pitch, paste aperture spacing, mask dam absence,
  and soldermask-defined/NSMD mismatch.
- QFN/DFN thermal pad rules: paste windowpane, via-in-pad fill/tent intent,
  solder voiding risk, and exposed pad copper balance. Initial KiCad checks warn
  when large likely thermal pads have no parsed same-net via in pad.
- BGA assembly risk: pitch class, escape route feasibility, dogbone via size,
  microvia requirements, soldermask web, inspection accessibility, and X-ray
  requirement flag. Initial KiCad checks warn when dense fine-pitch pad clusters
  have no parsed nearby escape via, and BOM/README checks require X-ray, AOI, or
  inspection handoff notes for likely BGA/CSP/LGA rows.
- Connector/rework access: soldering iron access, cable insertion direction,
  latch clearance, mating height, and keepout zones. Initial KiCad checks warn
  when likely connector pads have tight neighboring pads that may block rework
  access.
- Test point readiness: minimum probe diameter, soldermask opening, net
  coverage, side accessibility, spacing, height clearance, and no soldermask or
  silkscreen obstruction. Initial KiCad/IPC-D-356 checks warn when likely
  critical nets have no matching IPC-D-356 test record, and warn on IPC-D-356
  testpoint diameter, spacing, edge-clearance, missing/covered soldermask access,
  missing or contradictory access-side fixture-access risks, and access-side
  hints that disagree with nearby same-net KiCad pad/via side. They now also
  warn when an IPC-D-356 probe keepout intersects unrelated selected KiCad
  copper. Probe diameter, spacing, copper-clearance, and edge-clearance
  thresholds are assembly-profile driven.
- BOM, centroid, netlist, README, and drawing readiness: required columns,
  manufacturer/supplier metadata, lifecycle/status review, broader lifecycle-risk
  vocabulary, approved alternate coverage, same-as-primary alternate detection,
  value/footprint coverage, optional unit-cost/price sanity, procurement
  consistency across
  manufacturer/supplier/lifecycle fields, placeholder release metadata,
  semicolon- and whitespace-delimited sidecar tables, quantity/refdes agreement,
  zero-quantity population intent, assembly/build variant handoff, grouped
  reference expansion, DNP/DNI
  placement parity handling, empty component/placement/netlist sidecars, unusual
  reference designators, duplicate reference designators, conflicting MPN
  value/footprint and procurement metadata,
  polarity/MSL/component-height handoff metadata, malformed placement
  coordinates, unusually large placement coordinates, centroid/netlist
  placeholder metadata, out-of-range rotations, invalid side values, duplicate
  centroid coordinates,
  duplicate pin-to-net assignments, repeated netlist rows, one-pin net review,
  BOM/centroid assembly-side, value, footprint, and rotation parity, conflicting
  centroid value/footprint/rotation metadata, release revision notes, drawing
  role naming, text sidecar naming/extensions, drawing file size, placeholder
  drawing detection, order-parameter intent, contradictory order notes including
  conflicting finish, mask, thickness, copper-weight, and via-treatment values,
  conflicting layer-count, coating, programming, and test-fixture values,
  panel/rout drawing parity, BOM/centroid double-sided assembly handoff evidence,
  BOM-driven through-hole solder, BGA/CSP/LGA inspection, and programmable-device
  handoff evidence, firmware traceability, programming method, functional-test
  acceptance criteria, serialization/barcode handoff, fabrication marking-zone
  drawing parity, packaging/ESD/moisture notes, release preflight evidence,
  surface-finish compatibility notes for edge contacts, fine-pitch packages,
  press-fit hardware, and wire bonding,
  selective/wave solder and conformal-coating process notes,
  fab/assembly drawing parity for special fabrication and assembly handoffs,
  release revision/date consistency, DNP/DNI placement conflicts, and package
  parity between purchase, placement, and netlist files. Initial CSV/TSV,
  semicolon/whitespace table, text, and
  path/metadata checks are implemented by
  `production-artifact-readiness`.
- Fiducials: global/local count, symmetry, copper diameter, mask opening,
  keepout, edge clearance, and side coverage. Initial KiCad checks infer likely
  unnetted circular fiducial pads and warn on low count or edge-clearance risk,
  warn when same-layer copper intrudes into the fiducial optical keepout, and
  warn when dense fine-pitch pad clusters have too few nearby local fiducials.
- Panel fiducials/tooling: panel-level fiducials, tooling holes, rails, bad-board
  marks, breakaway tabs, mouse bite hole size/spacing, and V-score residual web.
  Initial KiCad/Excellon checks warn when likely non-plated tooling holes are
  missing or too close to the board edge for fixture access, and when likely
  mouse-bite drills have suspect diameter or spacing.
- Double-sided assembly: heavy parts on second side, reflow shadowing, adhesive
  requirements, and through-hole/wave conflicts. Initial artifact checks require
  README handoff evidence and an assembly drawing when centroid data or release
  notes indicate bottom-side placements.
- Selective or wave solder readiness: component side, solder thieves, keepouts,
  orientation, shadowing, and thermal relief around through-hole pins. Initial
  README checks require selective/wave process notes when through-hole, wave, or
  selective assembly is mentioned, and BOM-driven checks require process handoff
  when populated rows look through-hole or hand-soldered. Initial KiCad geometry
  checks now flag neighboring pads inside keepout bands around likely
  through-hole solder features.
- Moisture/cleanliness/coating: conformal coating keepouts, no-clean flux risk
  under low-standoff packages, and unmasked test pads in coated areas. Initial
  BOM/README checks require MSL metadata for likely moisture-sensitive packages
  and coating keepout/cleanliness notes when conformal coating is mentioned;
  KiCad geometry checks now flag neighboring pads inside no-coat feature
  keepout bands.

### Electrical and Functional Validation Checks

- Netlist parity: schematic-to-PCB net consistency, missing nets, extra copper
  islands, unconnected pads, and intentional no-connect handling.
- Same-net continuity: broken traces, missing zone refill, unstitched plane
  islands, disconnected thermal spokes, and trace severed by holes or slots.
  Initial KiCad checks warn when a net appears as disconnected copper islands
  on the same selected layer.
- Different-net shorts: copper overlap with net awareness, inner-layer antipad
  shorts, drill breakout shorts, and paste/mask-induced assembly short risk.
- Differential pairs: pair membership, skew, intra-pair spacing, width,
  neck-downs, pair-to-pair spacing, layer-change symmetry, and reference plane
  continuity. Initial KiCad checks infer common pair suffixes and warn when one
  side is missing, the pair sides occupy different selected copper layers, or
  the nearest parsed pair-side spacing exceeds the configured review threshold,
  and when pair-side copper lacks nearby same-layer ground guard/return copper.
- High-speed return paths: reference-plane void crossings, split-plane
  crossings, missing stitching vias at layer changes, and loop-area excursions.
  Initial KiCad checks warn on likely high-speed copper outside parsed
  ground-zone coverage and likely high-speed vias without nearby ground
  stitching.
- Power integrity: decoupling capacitor proximity/orientation, plane neck-downs,
  via count per rail, high-current bottlenecks, and starved regulator thermal
  pads. Initial KiCad checks warn on likely power nets without same-net copper
  zones, narrow power copper, sparse layer-change vias, isolated via-array
  members, and power pads or vias without nearby same-layer ground return copper.
- Analog/digital/RF segregation: region keepouts, noisy-net proximity, guard
  traces, via fences, antenna keepouts, and copper-free regions under inductors
  or antennas. Initial KiCad checks warn when likely analog, RF, or sensor nets
  run close to likely noisy power, switching, motor, or high-speed nets; when
  those sensitive nets lack nearby same-layer ground copper; when likely RF or
  antenna nets are close to non-ground copper; and when likely RF or antenna
  copper lacks a nearby same-layer ground via fence.
- ESD/safety: creepage and clearance by voltage class, slot barriers, spark-gap
  geometry, protective earth spacing, fuse/MOV keepouts, and high-voltage
  silkscreen warnings. Initial KiCad checks warn when likely high-voltage nets
  are close to other nets or enter the board-edge clearance band, and when
  likely edge connector nets lack nearby ESD/chassis/ground protection copper.
  Configured net classes can now carry explicit voltage-clearance requirements.
- Thermal validation: thermal via arrays, copper area under heat-generating
  parts, thermal relief versus heat spreading, hot component spacing, and
  heatsink/mechanical keepouts. Initial KiCad checks warn when likely thermal
  pads lack via-in-pad support, when likely heat or power features lack nearby
  same-net copper zone area, when likely hot features crowd neighboring
  non-ground copper, and when hot copper enters non-plated mechanical-hole
  keepouts.
- EMC readiness: edge-rate nets near board edge, connector return pins, chassis
  stitching, ground moat mistakes, cable shield connection intent, switching-node
  copper keepouts, and loop antenna risk. Initial KiCad checks warn when likely
  chassis or shield nets lack nearby parsed ground stitching vias, when likely
  connector edge-rate nets sit near the board edge without nearby same-layer
  ground return copper, and when likely switching nodes are close to non-ground
  neighboring copper.

### Manufacturing File and Pre-Production Workflow Checks

- File completeness: expected copper, soldermask, paste, silkscreen, drill,
  rout, fab drawing, assembly drawing, centroid, BOM, netlist, and readme files.
  `--gerber-dir` now discovers common drill, IPC-D-356, BOM, centroid, netlist,
  README, fabrication drawing, assembly drawing, and rout/panel sidecars in the
  same package directory and preserves them in report provenance. Required
  sidecars are configurable with `required_artifacts`, and required layer roles
  are configurable with `required_layers`.
- Layer count parity: declared order layer count, stackup, Gerber set, KiCad
  stackup, and filename conventions agree.
- Layer role inference: Gerber X2 attributes, file extensions, JLC-style names,
  KiCad names, and explicit CLI role overrides resolve to a coherent stack.
  Initial filename side-token conflict checks are implemented in
  `file-manifest-readiness`.
- Polarity and mirroring: negative planes, bottom-layer mirroring, text
  orientation, drill origin, and coordinate origin consistency.
- Drill file checks: plated/non-plated split, duplicate drill files, missing
  tools, unsupported units/zeros, route slots versus drill hits, and tool
  diameter outliers. Initial implementation now covers missing/duplicate tools,
  tool-unit sequencing, unresolved tool hits, and malformed coordinate records via
  `excellon-readiness`.
- Gerber sanity: empty layers, tiny aperture flashes, unbounded fills, huge
  areas, malformed regions, duplicate layers, stale plot files, and mixed
  revisions.
- Order-parameter parity: board thickness, copper weight, soldermask color,
  surface finish, impedance, castellations, edge plating, via fill, controlled
  depth, and panelization options match file content.
- Revision consistency: board revision appears consistently in fab drawing,
  silkscreen, README, filename, BOM, placement file, and source metadata.
  Initial manifest checks compare recognizable project/job prefixes, revision
  tags, and generated-date tags across Gerber and sidecar filenames and warn on
  stale/archive filename tokens. They also flag generated-date tags that are
  stale or later than the current run date, using configurable freshness days.
- Preflight sequence: refill zones, run EDA DRC/ERC, generate fresh fabrication
  outputs, reload outputs into independent viewer/parser, run HyperDRC, generate
  overlay artifacts, review waiver diff, and archive exact submitted package.
- Waiver governance: require reason, owner, expiry/review date, source link,
  unchanged geometry hash, and non-expired ISO review dates for production
  waivers.
- CI gating: fail on errors, warn on fabricator-cost escalations, attach SVG and
  GeoJSON artifacts, compare active findings against a saved baseline, and block
  stale generated outputs. Initial generated-date freshness warnings are
  implemented in `file-manifest-readiness` and can be tuned by
  `generated_date_stale_days` or `--generated-date-stale-days`.
- Engineering review packet: checklist summary, stackup, rule deck, plots,
  DRC/ERC reports, BOM/centroid checks, and open manufacturing questions.

### Data Sources to Consider Next

- Gerber X2/X3 attributes for layer role, net, component, and aperture intent.
- Additional IPC-D-356 dialect records and cross-source validation of parsed
  access-side, feature-type, soldermask, net-name, and drill-diameter fields.
- KiCad board setup: net classes, constraints, custom DRC rules, stackup,
  differential pair settings, pad properties, via properties, zones, component
  footprints, 3D/body boxes, and fabrication output settings.
- Pick-and-place and BOM files for assembly-side, package, refdes, value,
  population, rotation, and centroid consistency.
- Fabrication and assembly drawings, parsed initially as metadata sidecars or
  structured JSON notes.
- ODB++ or IPC-2581 for richer stackup, components, nets, drill spans, materials,
  and manufacturing intents.

## Test Ideas From Fabrication DRC Guidance

Fabrication-facing guidance from PCBWay, JLCPCB, Eurocircuits, KiCad, Sierra
Circuits, and other DFM references should be turned into small synthetic
Gerber/KiCad fixtures with one clear violation and one matching non-violation.

### Existing-check regression fixtures

- Implemented in unit tests: 3 mil minimum trace-width violation coverage, 6 mil
  preferred trace pass coverage,
  KiCad via annular-ring fail/pass coverage, pad-to-via, via-to-via,
  trace-to-trace, trace-to-pad, trace-to-via, hole-to-trace, hole-to-hole,
  conservative slot-to-slot, and slot-like drill-to-trace spacing coverage,
  0.20 mm board-edge trace
  clearance, pad-crossing-outline coverage, oversized solder-mask opening
  coverage, undersized/missing paste and solder-mask opening coverage,
  silkscreen-over-pad/blocker and silkscreen-over-V-score coverage, thin
  silkscreen stroke coverage,
  Excellon/NPTH-style panel drill clearance coverage, tab-route, V-score,
  castellated, and edge-plating panel-layer recognition coverage, same-net
  plated drill clearance suppression, same-net NPTH drill-to-trace clearance
  coverage, layer-sanity empty/bounds/area coverage,
  layer-registration tolerance coverage, IPC-D-356 missing-copper coverage, and
  IPC-D-356 drill net/diameter recovery and conflict coverage, KiCad
  oval/rectangular drill parsing coverage, no-input rejection coverage, missing
  Gerber/KiCad/Excellon/IPC-D-356 file coverage, and mechanical/user layer
  geometry, board-outline sanity, duplicate explicit layer-pair rejection, and
  duplicate explicit layer-role validation coverage, including
  all-layers-as-silkscreen rejection and board-outline versus explicit
  copper/mask/silkscreen role conflict rejection.
- Remaining spacing refinements: add exact routed slot-to-slot and plated-slot
  fixtures once slots preserve exact routed geometry and plated-slot semantics.
- Remaining drill/open-circuit fixtures: add through-hole containment fixtures
  where a drill cuts through a same-net trace so future open-circuit logic has
  expected behavior captured.
- Remaining paste and mask opening geometry: add per-pad paste exemptions once
  KiCad pad attributes are preserved through rule checks.
- Remaining silkscreen manufacturability: add fixtures for mirrored bottom
  silkscreen and slot geometry drawn on silkscreen once silkscreen side/text
  semantics are modeled.
- Remaining panelization features: add richer plated-half-hole and edge-plating
  copper treatment once side-plating semantics are modeled separately from
  generic panel graphics.
- Remaining layer sanity and file completeness: add CLI/app-level fixtures for
  missing layer-order metadata and ambiguous one-layer versus two-layer inputs.
- Remaining IPC-D-356 coverage: add more fixed-column dialect variants and
  cross-check parsed access-side, feature-type, and soldermask flags against
  KiCad pad side, explicit soldermask openings, and fixture-side declarations.

### Future-rule fixture backlog

- Power and ground trace sizing: fixtures with named power nets whose trace
  widths differ from a configured per-net or net-class minimum.
- Differential pair and critical signal constraints: fixtures for pair length
  mismatch, intra-pair spacing violations, missing/insufficient guard traces,
  and impedance target metadata beyond the current stackup/net-class and
  differential-pair spacing checks.
- Region separation rules: fixtures that intentionally mix analog/digital,
  input/output, high-frequency/low-frequency, or high-power/low-power placement
  zones once board regions are modeled.
- Gold fingers and beveling: fixtures for no gold-finger design despite order
  metadata, missing mask opening on fingers, copper/text on fingers, and bevel
  keepout violations.
- BGA-specific checks: fixtures for BGA pad opening, solder-mask bridge, via
  escape, and pad/via spacing at dense pitch.
- NPTH and slot semantics: fixtures that distinguish cutouts, round holes,
  oval slots, rectangle slots, NPTH holes, plated slots, and route slots drawn on
  the wrong layer.
- Soldermask process fixtures: LDI versus conventional/PI mask examples with
  different minimum web, mask annular ring, and mask-overlap clearances.
- Voltage clearance fixtures: same geometry passing at low voltage and failing
  at high voltage, with internal/external layer variants and optional coating.
- Aspect-ratio fixtures: board thickness and drill diameter combinations that
  pass/fail standard via, microvia, blind via, and through-hole limits.
- Copper-balance fixtures: large pour imbalance, isolated local copper density
  islands, and dense copper regions near sparse opposite-side geometry.
- Board-outline fixtures: unordered outlines, self-crossing outlines, duplicate
  outlines, tiny route notches, nested cutouts, and route slots touching copper.
- File-package fixtures: stale generated Gerber files, missing paste/mask/drill
  files, mismatched declared layer count, duplicate layers, mixed units, and
  mismatched revision strings.
- DFA fixtures: component courtyard collisions, component too close to edge,
  missing global fiducials, testpoint spacing failures, QFN thermal pad paste
  overfill, and via-in-pad without fill/cap metadata.
- Electrical-readiness fixtures: missing zone refill, unconnected copper island,
  split-plane crossing, differential-pair skew, missing return stitching via,
  and high-current bottleneck.

## Implemented Reporting

- Stable violation IDs derived from check name, layer names, island index, and
  rounded coordinates.
- Text, JSON, JSON Lines, GeoJSON, SARIF, GitHub Actions annotation, HTML, and
  JUnit XML output.
- SVG violation overlays for quick local or CI artifact review.
- Severity on each violation.
- Structured parser diagnostics separated from active DRC violations. Excellon
  reports contribute unit/tool/coordinate diagnostics and IPC-D-356 reports
  contribute malformed recognized-record diagnostics.
- Structured JSON input manifest entries with adapter, role, path, and
  conversion-origin provenance.
- JSON waiver files that suppress findings by ID, check name, layers, or message
  text, with governance warnings for incomplete, malformed, or expired
  production-review metadata.
- Compact JSON CI summaries with error, warning, waiver, and per-check counts.
- JSON rule configuration with CLI overrides for clearance thresholds, area
  thresholds, and KiCad copper layer selection.

## Reporting Roadmap

- Export violation overlays as Gerber for direct review in board viewers.
- Expand parser diagnostics beyond Excellon and IPC-D-356 to Gerber, KiCad,
  BOM/centroid/netlist table parsing, and converter adapters.

## Input Roadmap

- Infer layer roles from file extensions and X2 attributes.
- Accept explicit layer-role flags for ambiguous files. Initial flags are present
  for board outline, copper, paste/copper pairs, copper/mask pairs, and
  silkscreen/blocker pairs.
- Expand the implemented assembly, stackup, and net-class project config
  sections with package-specific assembly classes, length, routed via-span,
  impedance-target, material property ranges, and fabrication-class threshold
  constraints.
- Add IPC-356 netlist ingestion so copper overlap can distinguish intended from
  unintended coupling.
- Add IPC-2581 and ODB++ importers if licensing and available crate support make
  that practical.
- Preserve Gerber units and report normalized units explicitly.

## IO Sources and Sinks Roadmap

The project should treat IO as a set of adapters around a shared internal board
and layer model. Direct Rust crates are preferable for deterministic CI behavior,
but command-line adapters are still valuable when an EDA tool already provides a
well-maintained exporter.

### Highest-Value Sources

- KiCad native files via `kiutils-rs` or `kiutils_kicad`: replace or augment the
  hand-rolled KiCad parser with a typed, lossless KiCad API. Useful inputs:
  `.kicad_pcb`, `.kicad_pro`, `.kicad_dru`, `.kicad_sch`, `.kicad_mod`,
  symbol/footprint library tables, and worksheet/title-block files. This would
  unlock native net classes, custom DRC rules, stackup, title/revision data,
  footprint metadata, pad properties, and schematic/PCB parity checks.
- KiCad CLI as a conversion/check backend: shell out to `kicad-cli` for DRC,
  Gerber, drill, DXF, SVG, PDF, IPC-D-356, IPC-2581, ODB++, STEP, GLB, PLY,
  position-file, and statistics exports. This is the fastest path to high
  fidelity for formats that KiCad already owns.
- Gerber X2/X3 and Gerber job files through `gerber_parser` / `gerber-types`:
  mine attributes for layer role, net/component metadata, file function,
  polarity, board profile, drill map, and job-level manifest data. Keep `csgrs`
  for geometry if it remains best for boolean operations, but use parser-level
  metadata to reduce manual layer-role flags.
- Gerber package libraries through Gerbonara as an optional Python adapter:
  useful for robust folder-level file-role inference across real-world CAD
  packages from KiCad, Altium, Eagle, Allegro, gEDA, Fritzing, PADS, and
  Target3001. Rust can drive this through a command-line or Python subprocess
  bridge if direct Rust coverage is insufficient.
- GenCAD via the `gencad` crate: import a richer manufacturing/test model with
  components, nets, routes, and part data. This is attractive for DFT/testpoint
  readiness and assembly checks because GenCAD was designed around PCB
  fabrication and testing interchange.
- DXF/DWG mechanical data via `dxf` or `acadrust`: import enclosures,
  panel drawings, mechanical keepouts, board outlines, slots, fab notes, and
  assembly fixture constraints. `dxf` is a mature read/write path; `acadrust`
  broadens scope to DWG and ACIS/SAT but should be evaluated for stability and
  license fit.
- SVG drawings via `usvg`: import laser/mechanical outlines, vendor templates,
  board-edge graphics, and review annotations. `usvg` resolves CSS, inherited
  attributes, basic shapes, and path commands into simpler absolute paths.
- BOM and placement files via `csv`, `serde`, and `calamine`: parse CSV/TSV,
  JSON, XLS/XLSX/XLSM/XLSB/ODS BOMs, centroid files, AVL/AML sheets, and
  assembly notes. This supports refdes completeness, DNP variants, centroid
  sanity, side/rotation checks, and manufacturer order-parameter parity.
- ZIP/TGZ/package ingestion via Rust archive crates: accept manufacturer upload
  packages directly, unpack to a temp workspace, detect Gerber/drill/BOM/PNP
  roles, and preserve a manifest of exactly what was checked.

### Medium-Value or Specialized Sources

- IPC-2581 / IPC-DPMX through XML parsing: IPC-2581 is an open, bidirectional
  PCB design/manufacturing exchange format and includes fabrication and assembly
  information. There does not appear to be a mature Rust-native parser today,
  but it is XML, so an incremental parser using `quick-xml`, `roxmltree`, or
  `xml-rs` is realistic. Existing web tooling such as BoardUI can serve as a
  reference for object modeling.
- ODB++ through external services or adapters: ODB++ is data-rich and widely
  used for CAM/assembly, but the format is proprietary. Options include parsing
  a practical subset, driving KiCad's ODB++ exporter, using an external
  extractor, or integrating a service such as OdbDesign. Note licensing risk:
  OdbDesign is AGPL for the open-source edition, so use it as a subprocess or
  optional service only if license constraints are acceptable.
- STEP/STEPZ for enclosure and mechanical validation: use KiCad CLI export,
  OCCT-based tooling, `ruststep`, or `vcad` depending on whether the goal is
  metadata, B-rep geometry, or simple collision envelopes. Rust-native STEP
  support exists but may be limited for full AP242 assembly/GD&T workflows; OCCT
  remains the mature path through C++ or external tools.
- 3D mesh formats: STL, OBJ, PLY, GLB/glTF, and U3D can be useful sinks for
  review artifacts and collision envelopes. Rust has `stl_io`, glTF ecosystem
  crates, and `vcad` export paths; these are best treated as visualization or
  approximate mechanical-clearance sinks rather than authoritative CAD inputs.
- PDF/PostScript/HPGL: useful mostly as human review sinks or fab-drawing
  sources. Direct semantic extraction is weak; prefer producing PDFs from
  structured report/overlay data and using PDF parsing only for rough metadata
  or text-note extraction.
- Image inputs: PNG/TIFF renderings from Gerber, KiCad, or vendor viewers can be
  used for visual regression tests, ML-assisted review, or review overlays.
  They should not be the source of truth for dimensional checks.
- Pick-and-place variants and assembly drawings: many CMs require only refdes,
  X, Y, rotation, and side for centroid data. Add explicit parsers for common
  KiCad `.pos`, Altium CSV, JLCPCB CPL, and generic centroid schemas.
- Test and inspection formats: IPC-D-356, GenCAD, boundary-scan netlists,
  flying-probe reports, AOI placement exports, and bed-of-nails fixture formats
  can improve DFT coverage and production feedback loops.
- Vendor/order APIs: JLCPCB, PCBWay, Eurocircuits, DigiKey, Mouser, Octopart,
  Nexar, and internal PLM/ERP APIs could provide capabilities, stackup options,
  BOM availability, lifecycle status, pricing, and order-parameter parity. Keep
  these behind optional network-enabled adapters and cache results for CI.

### Highest-Value Sinks

- Gerber/Excellon output: export violation overlays or generated keepout layers
  as Gerber so designers can load findings in existing PCB viewers.
- KiCad markers or rule areas: write warnings back into `.kicad_pcb` as markers,
  graphics, zones, or generated user layers once lossless KiCad writing is
  available.
- KiCad `.kicad_dru`: generate or update custom rule files from HyperDRC rule
  decks so designers can catch issues earlier in KiCad.
- IPC-D-356 output: emit an annotated electrical-test netlist or diff showing
  expected test access, uncovered nets, and drill/net mismatches.
- IPC-2581 or ODB++ export: longer-term single-package manufacturing handoff
  with embedded DRC/DFM annotations, if library/tool support matures.
- GenCAD output: useful for DFT/test fixture workflows and integration with
  test engineering tooling.
- DXF/SVG/PDF overlays: produce human-review artifacts for mechanical, fab, and
  assembly teams. SVG already exists; DXF/PDF would make review easier in CAD
  and document workflows.
- HTML report bundle: initial self-contained HTML output is implemented with an
  embedded SVG overlay, source manifest, summary table, and finding cards. Layer
  toggles and waiver-state interaction remain future enhancements.
- SARIF/JUnit/GitHub annotations: SARIF output is implemented with stable
  finding IDs and PCB geometry properties; GitHub Actions annotation output is
  implemented for CI log annotations; JUnit XML output is implemented for CI
  systems with test-report publishers.
- JSON Lines / SQLite / Parquet: JSON Lines output is implemented for streaming
  analysis across many boards, vendors, revisions, and rule decks. SQLite and
  Parquet remain future structured stores.
- Waiver and baseline update sinks: proposed waiver stubs, active-finding
  baselines, and baseline diff JSON are implemented for controlled production
  exceptions and release drift review. Waiver governance reports expired review
  dates. Richer geometry fingerprints remain future work.

### Adapter Architecture Notes

- Keep `IoAdapter` boundaries explicit: `discover`, `read`, `convert`, `write`,
  and `capabilities`. A format may support only one operation.
- Separate semantic richness from geometry richness. A Gerber may have excellent
  polygon geometry but poor design intent; KiCad/IPC-2581/ODB++ may carry nets,
  components, stackup, and manufacturing intent.
- Preserve provenance for every loaded object: source file, adapter, layer,
  units, original identifier, and transformation history. Initial report-level
  provenance is implemented for direct Gerbers, Gerber directories, converted
  Gerbers, KiCad boards, Excellon drills, IPC-D-356 netlists, explicit
  pre-production sidecars, Gerber-directory-discovered sidecars, and waivers.
- Treat external tools as hermetic conversion steps with captured command line,
  version output, stdout/stderr, input hash, and output manifest.
- Prefer optional Cargo features for heavy or license-sensitive integrations:
  `io-kicad-typed`, `io-dxf`, `io-svg`, `io-xlsx`, `io-gencad`,
  `io-ipc2581`, `io-step`, and `io-archives`.
- Provide fixture corpora by adapter: tiny synthetic files for unit tests, plus
  opt-in real-world packages for integration tests.

## Research Sources

- KiCad PCB Editor documentation: DRC should be run before generating
  manufacturing files, with zone refill and board/schematic parity checks
  considered part of preflight; KiCad also documents DFM-oriented DRC classes
  such as board-edge and hole clearances.
- PCBWay DRC and layout guidance: common checks include minimum line width,
  spacing between traces/pads/vias, via size, power/ground trace sizing,
  critical-signal routing, differential-pair/guard-trace constraints, and
  analog/digital/high-power/high-frequency separation.
- JLCPCB/TransJLC ecosystem: TransJLC normalizes Gerber packages from common
  EDAs to JLCEDA/JLCPCB-style naming; JLCPCB capability discussions emphasize
  via diameter, via hole size, and annular-ring interpretation.
- Eurocircuits PCB Checker and tolerances: DRC compares measured values against
  configured track, isolation, and annular-ring limits; DFM adds plating index,
  copper area, solderpaste surface, exposed copper, and fiducial-like pad
  information. Their tolerances also highlight bow/twist, drill tolerances,
  hole-to-hole, NPTH-to-copper, soldermask web/opening/overlap, legend, edge
  clearance, slot width, and profile tolerance.
- Eurocircuits soldermask guidance: soldermask openings are usually negative
  data and larger than copper pads; mask annular ring, mask segment/web, mask
  overlap clearance, minimum opening, via tenting/filling, NPTH mask openings,
  and board-outline mask behavior all affect manufacturability.
- Sierra Circuits DFM/DFA guidance: annular-ring breakout, drill spacing,
  via-in-pad filling/capping, solder wicking, component footprint correctness,
  and soldermask bridge sizes are common production-delay causes.
- General DFM references: checklist themes repeatedly include trace/space,
  annular ring, aspect ratio, soldermask/paste alignment, silkscreen clipping,
  component spacing, thermal relief, copper balance, test points, fiducials,
  documentation completeness, and pre-production review packets.
- KiCad CLI documentation: exposes useful source/sink conversions for Gerber,
  Excellon, DRC, DXF, GenCAD, IPC-D-356, IPC-2581, ODB++, PDF, position files,
  statistics, STEP, SVG, and 3D formats.
- Rust IO ecosystem notes: `kiutils-rs`/`kiutils_kicad` cover typed/lossless
  KiCad files; `gerber_parser` covers Gerber parsing with `gerber-types`;
  `gencad` parses GenCAD; `dxf` reads/writes DXF/DXB; `acadrust` targets
  DXF/DWG/ACIS; `usvg` parses SVG; `calamine` reads Excel/OpenDocument
  spreadsheets; `ruststep` and `vcad` are possible STEP/CAD paths.
