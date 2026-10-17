<h1>
  HyperDRC
  <img src="./docs/hyperdrc.png" alt="HyperDRC logo" width="144" align="right">
</h1>

HyperDRC is a Rust library and command-line design-readiness reviewer for
printed-circuit-board release packages. It loads Gerber, KiCad, Excellon,
IPC-D-356, archives, and manufacturing sidecars; runs an explicit set of
fabrication, assembly, test, package, and policy checks; and emits
evidence-rich findings for local review or CI.

It answers a broader question than polygon clearance alone: “Does this release
package contain coherent, reviewable evidence for the board we intend to
manufacture?”

HyperDRC is not a fabricator-certified CAM engine and does not claim that a
passing report guarantees manufacturability. It is a preflight layer that keeps
parser assumptions, source grids, conversion provenance, rule policy,
uncertainty, waivers, and baselines visible.

## What HyperDRC is for

A PCB handoff can fail even when its copper looks plausible. Layer roles may be
ambiguous, drill and netlist files may disagree, stackup evidence may be
missing, generated files may be stale, or assembly intent may have disappeared
between the authoring tool and Gerber.

HyperDRC treats the release as a set of related evidence:

```text
Gerber / KiCad / drills / IPC-D-356 / sidecars / archives
                              │
                              ▼
                 parsers + source provenance
                              │
                              ▼
          rules + capabilities + readiness check plan
                              │
                              ▼
          Report + coverage + diagnostics + findings
              │            │             │
              ▼            ▼             ▼
        text / JSON      SARIF / CI   review artifacts
```

Every selected check receives an explicit execution status. A check may pass,
fail, be inapplicable, remain uncertain, or be skipped by policy; it does not
silently disappear because an adapter lacked data.

## Primary types

- `Cli` is the reusable command-line configuration model.
- `run(Cli) -> Result<RunOutcome>` executes the same loading, checking,
  waiver, reporting, and side-artifact pipeline as the binary without exiting
  the process.
- `RunOutcome` contains the completed `Report` and elapsed time.
- `Report` contains source records, parser diagnostics, check coverage,
  active findings, waived findings, and aggregate counts.
- `Violation` is one stable finding with severity, evidence identity, source
  layers, geometry, locations, semantic subjects, and an optional message.
- `Check` identifies a registered readiness check.
- `ReadinessRunner` executes an ordered native check plan and produces
  `CheckCoverage`.
- `CheckExecutionRecord` and `CheckExecutionStatus` preserve whether each
  selected check passed, failed, was inapplicable, uncertain, or skipped.
- `RuleConfig`, `EffectiveRules`, `CapabilityProfile`, `Waiver`, and
  `BaselineFile` describe policy and review state.
- `PcbRegion` is the current exact-aware profile compatibility carrier.
  `PcbGeometryUncertainty` is returned when a geometry decision cannot be
  certified.
- `Scalar` is
  [`hyperreal::Real`](https://github.com/timschmidt/hyperreal), used for exact
  finite rule values.

## Numerical and geometry crates

HyperDRC uses repository-root links for related crates so readers land on each
crate's own project rather than a source-file blob:

- [CSGRS](https://github.com/timschmidt/csgrs) provides Gerber import and
  curve-region adapters.
- [Hyperreal](https://github.com/timschmidt/hyperreal) owns exact-aware scalar
  arithmetic.
- [Hyperlattice](https://github.com/timschmidt/hyperlattice) owns exact-aware
  lattice and bounding-volume primitives.
- [Hyperlimit](https://github.com/timschmidt/hyperlimit) owns certified
  predicate policy.
- [Hypercurve](https://github.com/timschmidt/hypercurve) owns exact curve and
  filled-region topology.
- [Hyperpath](https://github.com/timschmidt/hyperpath) owns exact PCB path
  models.
- [Hypermesh](https://github.com/timschmidt/hypermesh),
  [Hypertri](https://github.com/timschmidt/hypertri),
  [Hyperphysics](https://github.com/timschmidt/hyperphysics), and
  [Hypersolve](https://github.com/timschmidt/hypersolve) support integration
  and development checks.

## Installation

Install the command-line tool:

```sh
cargo install hyperdrc --version 0.3.0
hyperdrc --help
```

Or add the library to a Rust project:

```toml
[dependencies]
hyperdrc = "0.3.0"
```

## Quick start

This native example plans two checks and records why one does not apply. It
shows the coverage contract without requiring a board file.

<!-- quickstart:start -->
```rust
use hyperdrc::{Check, CheckExecutionStatus, CheckRunDisposition, ReadinessRunner};

fn main() {
    let runner = ReadinessRunner::new([Check::StackupReadiness, Check::TestpointCoverageReadiness]);
    let mut findings = Vec::new();
    let coverage = runner
        .run(&mut findings, |check, _| {
            Ok::<_, std::convert::Infallible>(match check {
                Check::StackupReadiness => CheckRunDisposition::Executed,
                _ => {
                    CheckRunDisposition::NotApplicable("design declares no testpoint intent".into())
                }
            })
        })
        .expect("infallible readiness adapter");

    assert!(coverage.is_complete());
    assert_eq!(coverage.checks[0].status, CheckExecutionStatus::Passed);
    assert_eq!(
        coverage.checks[1].status,
        CheckExecutionStatus::NotApplicable
    );
}
```
<!-- quickstart:end -->

Run the repository copy with:

```sh
cargo run --example basic
```

Native authoring systems can use `ReadinessRunner` to share the same coverage
accounting while supplying richer geometry and semantic identities than a
file-only pipeline can recover.

## Command-line use

Review one or more Gerber layers:

```sh
hyperdrc board-F_Cu.gbr board-B_Cu.gbr
hyperdrc --gerber-dir release/gerbers
```

Review a manufacturing archive and emit JSON:

```sh
hyperdrc --package-archive release-board.zip --format json
```

Combine KiCad and manufacturing sidecars with a rule configuration:

```sh
hyperdrc \
  --config examples/hyperdrc-config.json \
  --kicad-pcb board.kicad_pcb \
  --excellon board.drl \
  --ipc356 board.ipc \
  --format sarif
```

Generate review artifacts without turning active findings into a failing
process status:

```sh
hyperdrc \
  --allow-findings \
  --kicad-pcb board.kicad_pcb \
  --format html \
  --svg-overlay violations.svg \
  --summary-file summary.json
```

The binary maps malformed input or impossible evidence construction to exit
status 1 and active findings to status 2. `--allow-findings` retains the report
but suppresses the latter status for exploratory automation.

Use `--check` to select check families, `--config` and rule flags to resolve
policy, `--waiver-file` and baseline options to govern known findings, and
`--help` for the complete generated option list. Converter entry points include
`--convert-input`, `--converter`, `--conversion-output-dir`, and the
tool-specific handoff/review export flags.

## Inputs and outputs

The file-oriented pipeline recognizes:

- Gerber Layer Format and X2/X3 metadata;
- KiCad board S-expressions and native rule context;
- Excellon drill programs and IPC-D-356 electrical test data;
- ZIP, TAR, TAR.GZ, and TGZ release packages;
- BOM, centroid, netlist, stackup, material, drawing, and manifest sidecars;
- converter manifests and generated handoff artifacts.

All report sinks consume the same `Report`. Supported outputs include text,
JSON, JSON Lines, GeoJSON, SARIF, GitHub annotations, HTML, JUnit, SQLite,
Arrow IPC, and Parquet. Review companions include SVG, Gerber, Excellon, DXF,
PDF, KiCad markers/rules, IPC-D-356, GenCAD, and IPC-2581 projections.

### Exact straight tracks and load failures

The KiCad loader now rejects malformed straight-track `start`, `end`, and
`width` fields instead of dropping, defaulting, or truncating them. Each field
must occur once with two endpoint coordinates or one width. Negative widths
fail using the exact source value, including values below binary64 range.
Validated zero width retains the existing empty-set behavior.

`load_kicad_pcb` keeps its `anyhow::Result<BoardModel>` signature. A caller can
downcast the error to `kicad::TrackLoadError` and inspect its zero-based segment
index and `TrackLoadFailure`: invalid fields, an undecided width sign, shared
budget exhaustion, unsupported geometry, a structured native `ExactCurveError`,
or a native geometry validation failure. `TrackLoadFailure::Construction`
retains the kernel diagnosis through `std::error::Error::source`.
No partial board is returned. The CLI reports the source path and diagnosis
with exit status 1, including with `--allow-findings`.

Straight tracks now retain exact decimal/scientific endpoints and widths through
native round-cap construction. Coincident positive-width endpoints denote the
existing exact disk; distinct endpoints denote the segment's round stroke.
Endpoint caps, subrange widths and coordinates beyond binary64 range no longer
depend on the finite rectangular adapter. The selected geometry is certified
under STRICT even inside an ambient approximate or preview scope, and supports
checked area, bounds, point queries and native handoff without a report projection.
`BoardModel::all_copper` and `copper_layers` aggregate the native owners through
the shared profile implementation. Selected invalid inputs still fail before
empty or singleton shortcuts. A finite report projection remains explicitly
fallible and is not required to compose exact geometry.

This is an explicit source-fidelity migration. Previously materialized rectangles
omitted the round caps and could round coordinates; their represented sets and
resulting findings can change. Readiness implementation version 22 invalidates
version-21 check evidence. The original finite-range failure fixtures now require
successful exact geometry, while malformed-source and budget failures retain
their typed contracts. Native boundary subdivisions are not source geometry.
The public finite `geometry::line_polygon` adapter and other KiCad object families
retain their existing contracts and open limitations. Shared refinement budgets
do not bound source parsing, total work, heap bytes, or cancellation.

## Useful API

The generated Rust documentation contains full signatures. These groups cover
the useful public entry points and modules.

### Complete runs and check planning

- `run`, `run_cli`, `Cli`, `OutputFormat`, `RunOutcome`;
- `default_checks`, `ReadinessRunner`, `ReadinessContext`;
- `CheckCoverage`, `CheckExecutionRecord`, `CheckExecutionStatus`, and
  `CheckRunDisposition`.

Call `run` when an embedder wants CLI-compatible discovery, parsing, policy,
waivers, baselines, reports, and side artifacts. Call `ReadinessRunner`
directly when a native authoring system already owns the input model.

### Reports and evidence

- `report`: `Report`, `Violation`, `Severity`, `Diagnostic`,
  `EvidenceContext`, `FindingSubject`, `FindingSourceSpan`,
  `FindingSourcePosition`, `report_summary`, `report_to_geojson`;
- `baseline`: `report_to_baseline`, `load_baseline`, `compare_baselines`,
  `report_to_waiver_stubs`;
- `waiver`: `load_waivers`, `apply_waivers`, `governance_violations`;
- report modules: `sarif`, `jsonl`, `html_report`, `junit`,
  `github_annotations`, `sqlite_report`, `arrow_report`,
  `parquet_report`;
- review modules: `svg_overlay`, `gerber_overlay`, `excellon_overlay`,
  `dxf_overlay`, `pdf_overlay`, `kicad_markers`, `ipc356_review`,
  `gencad_review`, `ipc2581_review`.

`Violation::new`, `with_subjects`, and `with_evidence_context` build durable
findings for native adapters.

### Rules, profiles, and capabilities

- `config`: `RuleConfig::load`, `RuleOverrides`, `effective_rules`,
  `EffectiveRules`;
- `assembly_policy`, `constraint_policy`, and `package_policy` resolve
  assembly, electrical, and artifact requirements;
- `CapabilityProfile`, `CapabilityProfileClass`, `DrillCapability`,
  `ImagingCapability`, `PanelAssemblyCapability`;
- `opinionated_prototype_profile`, `mainstream_profile`, `hdi_profile`.

Capability profiles validate and digest the process envelope so release
evidence can name the exact policy it used.

### Parsers and source records

- `gerber_metadata` parses image setup, units, coordinate grids, aperture
  macros/uses, interpolation, polarity, transforms, regions,
  step-and-repeat, and X2/X3 attributes;
- `kicad::load_kicad_pcb` returns `BoardModel` with copper and drill features;
- `excellon` and `ipc356` parse drill and test data;
- `io`: `SourceRecord`, `DiscoveredFile`, `discover_gerber_dir`,
  `discover_gerber_tree_from_archive`, sidecar discovery helpers, and
  `is_gerber_path`;
- `package_archive::ExtractedPackages` safely stages supported archives;
- `conversion::convert` executes an explicit `ConversionRequest` and returns
  provenance-bearing `ConversionOutput`.

### Geometry and exact path handoffs

- `PcbRegion::new`, `from_gerber`, `offset`, `try_union`,
  `try_difference`, `try_intersection`, `try_xor`, `metadata`;
- `PcbRegion::classify_xy` returns inside, outside or boundary with typed
  uncertainty; `try_bounding_box` returns checked bounds, `None` for an empty
  region, or a typed failure. Both respect STRICT and the shared work budget;
- `PcbRegion::try_area` and `try_is_empty` retain source, measurement and budget
  failures. An unavailable area integral cannot become zero;
- `Polygon::try_area` also returns a checked native area and caches only
  successful measurements. `geometry::multipolygon_to_shapes_scalar` returns
  `Result`: unavailable areas, unknown thresholds, exhausted budgets and
  non-finite report output fail the whole projection;
- `Polygon::try_bounds` returns the checked native envelope, with `None` only
  for certified empty geometry. Required bounds, centers and dimensions retain
  failures. `Polygon::bounding_rect` is a diagnostic report view and may use
  report-ring bounds; it cannot replace a required native measurement;
- `PcbRegion::try_into_native` and `Polygon::try_exact_region` explicitly hand
  off the native carrier after checking source and budget failures. Subsequent
  native operations use the caller's policy. Implicit `Deref`/`DerefMut` access
  is removed; `replace_native` clears old source facts and caches while retaining
  layer metadata;
- `PcbRegionExt::try_to_multipolygon` returns an exact-backed report view or a
  `PcbGeometryUncertainty`; `BoardModel::copper_layers` and `all_copper` also
  return `Result` so callers retain geometry failures;
- `exact_path_rules::certify_annular_ring` and
  `classify_via_drill_policy` delegate via geometry to Hyperpath while keeping
  DRC policy labels in HyperDRC;
- `geometry` exposes source-unit/grid facts, primitive construction, and
  report-shape projections.

Boolean and offset operations check source failures and the active shared work
budget under STRICT, including fast paths. Offset bounds come from the resulting
native region, so vanished components cannot leave cached input bounds. Required
operation failures become `geometry-uncertainty` findings and `Uncertain`
readiness receipts.

`geometry::transform_polygon` applies its native affine transform under the same
STRICT and shared-budget guard, including identity and empty-region fast paths.
It retains failed construction on the returned polygon, so a transform made in
an exhausted scope cannot become valid after leaving that scope. Retry from the
original polygon in a fresh budget. Enclosing preview policies do not change
the transform's decisions; finite rings remain report views.

`geometry::rect_polygon` imports finite center, size and degree inputs before
corner arithmetic, then constructs and transforms the native rectangle under
STRICT. Tiny sizes, corners beyond the finite report range and exact right-angle
rotations retain their geometry. Signed dimensions describe the same regularized
set; a zero dimension is certified empty. The historical report corner view can
collapse without changing the native source. Non-finite input and denied
construction retain typed failures. This repairs the finite rectangle boundary;
KiCad primitive pads also retain unavailable or underflowed size as a source
failure instead of manufacturing zero dimensions. Admitted custom primitives
need no unused pad size. Other source adapters still need audit. See the
[rectangle evidence](PERFORMANCE.md#native-rectangle-construction-2026-10-09).

Rounded and chamfered rectangles and trapezoid pads also construct native
boundaries before finite corner arithmetic. Rounded corners are circular arcs;
their sample count changes only the report view. Radius and chamfer limits are
computed after exact import, including for subnormal dimensions. Trapezoids use
KiCad's half-delta convention, retain valid limiting triangles and report
out-of-range deltas as source failures. These constructors enforce STRICT and
shared-budget guards. Parser ratio arithmetic, authored placement and other
source adapters still need native conversion. See the
[pad primitive evidence](PERFORMANCE.md#native-pad-primitive-construction-2026-10-09).
Duplicate detection can also reject a pair when its native bounding-box overlap
is provably too small for the existing area tolerance. Required intersection
integrals remain checked when that bound cannot decide the pair; some retained
circular algebraic fragments still lack area integration.

Layer-polygon candidate searches use certified native enclosures. Unknown,
unrepresentable or oversized enclosures retain candidates for the required
predicate. Duplicate-island detection uses native area and intersection, so
changing circle report samples cannot suppress a duplicate. Optional display
markers do not decide whether a proved finding exists.

Stencil pad-pair candidates share that enclosure grid using native centers and
a certified upper radius. Unrepresentable projections retain candidates. The
pair-distance and via/drill matching predicates compare native squared distances
under STRICT and preserve uncertainty; an unresolved drill match cannot select
the copper fallback. Unrepresentable via report anchors return a typed failure.

Drill-spacing candidates use certified native circular envelopes; missing or
oversized envelopes retain candidates. Drill-table and IPC diameter checks keep
every source index and match native centers, so an unprojectable point cannot
shift another record's identity. Required spacing, diameter and matching orders
retain uncertainty, and report markers use checked finite projection. Mounting
and castellation spacing share these predicates. Castellation selection measures
native center-to-outline clearance with the drill radius; report chords cannot
decide membership. Drill and slot inputs still use their documented circular
proxies where the source lacks complete geometry.

IPC net annotation and coverage also match native centers through certified
enclosures. `apply_ipc356_nets` returns a typed result and stages metadata before
publishing: an unresolved required match, invalid copied diameter or denied
budget leaves board nets and diameters unchanged. Existing metadata and the first
matching record keep priority. The CLI propagates annotation failure before
running checks or emitting a readiness report. Coverage retains unresolved
candidates as uncertainty unless another candidate certifies a match; required
missing-point markers use checked finite projection.

KiCad copper layers are projected in lexical layer-name order, preserving
authored feature order within each layer. Duplicate-layer checks prepare and
pair inputs in lexical source-name order as well, so reversing distinctly named
layers cannot change the selected marker source, finding order or failure
source. Equal names retain caller order. Source labels still participate in
finding identity; moving inputs to different paths can change their IDs.

Board stitching, power-via arrays, differential-via proximity/return checks and
mouse-bite rows use native center enclosures and checked radius predicates.
Unknown candidates remain uncertainty unless a certified alternative proves
the required match. Power arrays require distinct native centers, so coincident
copies on different copper layers do not count as neighbors. Report markers
are projected only when a finding needs them, and unavailable markers fail typed.
Mouse-bite rows retain unresolved diameter and pitch decisions. They can prove
a valid pitch without ranking equally distant neighbors; a certified too-close
pair proves a finding even when another candidate is unresolved. Optional
selection between certified bad pairs keeps the nearest provable pair, retaining
the first when that ordering is unavailable. All seven checks run under the
shared STRICT and work-budget guard, including empty fast paths.
Mouse-bite diagnostic numbers use `Scalar::fixed_decimal()` at six fractional
places, so an unresolved near-zero distance cannot start an unbounded search
for a scientific-notation exponent. Those display values never decide pitch.

Connector return paths, decoupling proximity, RF via fences, plating intent,
and both mounting-hole bonding checks also match authored native centers.
Layer, net and copper-kind filters apply before required matching. NPTH matching
uses the validated drill radius plus tolerance. Mounting-size eligibility keeps
unknown comparisons visible, including in the shared copper-keepout and
edge-clearance selectors; those two checks still have other legacy geometry
paths. An explicit ground/chassis net proves plated mounting intent without
requiring proximity. All six center checks retain STRICT and shared-budget
failures and project report markers only when needed. See the
[validation and remaining scope](PERFORMANCE.md#native-authored-center-consumers-2026-10-09).

The four dense-pad checks qualify clusters with native pitch and averaged
authored centers. Fiducial dimensions, nearby fiducial counts and escape-via
matches use checked predicates. Pad/via and mask-web clearance use native copper
boundaries, with conservative shape enclosures independent of authored centers.
A certified qualifying pair can prove a finding when optional minimum ordering
is unavailable; diagnostics then describe a certified witness instead of a
minimum. Required failures, exhausted budgets and unavailable report markers
retain uncertainty. See the
[dense-cluster evidence and limits](PERFORMANCE.md#native-dense-cluster-consumers-2026-10-09).

IPC fixture coverage projects only the reported markers for missing nets.
Accessibility checks retain native diameter, side, outline and circular-spacing
decisions; missing diameters stay explicit metadata findings. Probe keepouts use
native copper-shape candidates, independent of authored markers. Effective probe
radii are tested as alternatives, avoiding an unnecessary ordering between the
declared diameter and its minimum. A side mismatch requires both a certified
opposite-side match and certified absence on the requested side. Thermal
copper-area readiness also uses checked native centers; its zone-center proxy
does not measure heat-spreading area. These four checks retain required failures
and exhausted budgets as uncertainty. See the
[fixture-access evidence](PERFORMANCE.md#native-fixture-access-consumers-2026-10-09).

Profile aggregation shares a single source's native carrier. Multiple line/arc
sources retain the bulk constructor; other supported curve families use native
regularized union. Failed construction stays attached to the returned owner and
blocks checked queries. A caller can retry construction from the original source
polygons in a fresh budget. Authored report rings and sample counts remain views.

Whole-layer area and paste/mask coverage measure regularized native unions, so
overlapping or repeated profiles do not multiply material. Windowpane checks
count connected aperture regions before clipping to copper; a copper hole does
not split one physical opening into several. Required area and ratio failures
produce uncertainty findings. KiCad pad parsing excludes certified empty pads
and retains failed geometry for those diagnostics.

Net-length readiness uses the checked native endpoint-span/envelope proxy. A
failed contribution produces uncertainty and prevents a partial net or pair sum
from being reported as a completed measurement. This proxy does not reconstruct
routed-path length.

### Check families

The `checks` module groups the executable policy surface:

- board outline, layer, copper, mask, paste, silkscreen, stencil, and drill;
- continuity, annular ring, routed-slot, castellation, via-in-pad, and
  package manifests;
- controlled impedance, differential pairs, return paths, RF, power,
  high-current, safety, and grounding;
- assembly spacing, dense-pad escape, thermal, mechanical, panelization,
  fiducials, tooling, mouse bites, and coatings;
- testpoint and IPC-D-356 coverage.

The check ownership map is maintained in
[src/checks/README.md](src/checks/README.md). `Check` and `hyperdrc --help`
remain the authoritative enumerations.

## Guarantees and boundaries

- A `Report` preserves every selected check's coverage status, including
  uncertainty and policy skips.
- Exact decimal rule values are promoted to `Scalar`; source units, grids,
  hashes, transformations, parser diagnostics, waivers, and conversion
  provenance remain attached to evidence.
- Geometry decisions use Hyperreal, Hyperlimit, Hypercurve, and Hyperpath.
  Finite coordinate rings and CSGRS CAM projections remain named report and
  interchange boundaries; they do not own filled-region topology.
- Findings are conservative release-review evidence, not a substitute for a
  board fabricator's DFM/DRC pass or process warranty.
- Full ODB++ and IPC-2581 import, glyph-accurate text, general custom-pad
  Booleans, and broad electromagnetic/thermal field solving are outside the
  current certified surface.
- Report formats do not rerun analysis. They project one completed `Report`.
- External converters are explicit subprocess adapters. Their executable
  identity, arguments, outputs, and diagnostics must remain reviewable.

Measured implementation work and rejected optimization experiments are kept in
[PERFORMANCE.md](PERFORMANCE.md), not presented here as timeless throughput
claims.

## Repository guides

- [src](src/README.md) maps the library and runtime pipeline.
- [checks](src/checks/README.md) assigns check ownership.
- [geometry](src/geometry/README.md) explains geometry-test expectations.
- [KiCad](src/kicad/README.md) documents parser/model scope.
- [examples](examples/README.md) explains the sample configuration.
- [benchmarks](benches/README.md) defines the smoke and spatial-index audits.
- [testing](docs/testing.md) describes the test suite.

## References

Normative format and reporting sources:

- Ucamco,
  [*Gerber Layer Format Specification*, revision 2026.05](https://www.ucamco.com/en/gerber/downloads).
- IPC, [IPC-D-356B: Bare Substrate Electrical Test Data Format](https://shop.electronics.org/ipc-d-356/ipc-d-356-standard-only).
- IPC, [IPC-2221B: Generic Standard on Printed Board Design](https://www.ipc.org/TOC/IPC-2221B.pdf).
- IPC, [IPC-2152: Current Carrying Capacity in Printed Board Design](https://shop.ipc.org/ipc-2152/ipc-2152-standard-only).
- IPC, [IPC-7351B: Land Pattern Standard](https://shop.ipc.org/ipc-7351/ipc-7351-standard-only).
- IPC, [IPC-7525B: Stencil Design Guidelines](https://www.ipc.org/TOC/IPC-7525B.pdf).
- IPC, [IPC-2581C manufacturing description and transfer standard](https://shop.ipc.org/ipc-2581/ipc-2581-standard-only).
- KiCad,
  [S-expression file-format documentation](https://dev-docs.kicad.org/en/file-formats/sexpr-intro/).
- OASIS,
  [SARIF 2.1.0](https://docs.oasis-open.org/sarif/sarif/v2.1.0/sarif-v2.1.0.html).
- Apache Software Foundation,
  [Arrow Columnar Format](https://arrow.apache.org/docs/format/Columnar.html)
  and [Parquet](https://parquet.apache.org/docs/).

Geometry and electrical-model sources:

- Chee K. Yap, [“Towards Exact Geometric Computation”](https://doi.org/10.1016/0925-7721(95)00040-2),
  *Computational Geometry* 7, 1997, pp. 3–23.
- Jon Louis Bentley,
  [“Multidimensional Binary Search Trees Used for Associative Searching”](https://doi.org/10.1145/361002.361007),
  *Communications of the ACM* 18(9), 1975.
- Matthias Teschner et al.,
  [“Optimized Spatial Hashing for Collision Detection of Deformable Objects”](http://hdl.handle.net/20.500.11850/52292),
  VMV 2003.
- E. Hammerstad and O. Jensen,
  [“Accurate Models for Microstrip Computer-Aided Design”](https://doi.org/10.1109/MWSYM.1980.1124303),
  IEEE MTT-S, 1980.
- M. Kirschning and R. H. Jansen,
  [“Accurate Wide-Range Design Equations for Parallel Coupled Microstrip Lines”](https://doi.org/10.1109/TMTT.1984.1132616),
  *IEEE Transactions on Microwave Theory and Techniques* 32(1), 1984.
- STMicroelectronics,
  [AN576: Influence of PCB Layout on ESD Protection](https://www.st.com/resource/en/application_note/an576-pcb-layout-optimisation-stmicroelectronics.pdf).

These references define inputs, evidence formats, or the limited analytical
models used by checks. They do not turn heuristic readiness findings into
standards-conformance certification.

## Acknowledgements

HyperDRC builds on the Gerber, KiCad, IPC, OASIS, Arrow, Parquet, SQLite, Rust,
and Hyper geometry ecosystems. CSGRS currently supplies CAM profile adapters;
Hyperreal, Hyperlattice, Hyperlimit, Hypercurve, and Hyperpath supply
exact-aware scalar, predicate, curve, and routed-path handoffs. Hypercircuit
and Hyperphysics supply native semantic and material context where available.

Thanks are due to the authors and maintainers of those specifications,
libraries, and public technical sources, and to contributors who supplied
reproducible release-package fixtures and review cases.

## Development

```sh
cargo fmt --all -- --check
cargo test --all-targets
cargo check --benches
cargo clippy --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps
```

The detailed design history remains in
[docs/design-readiness-plan.md](docs/design-readiness-plan.md); it is not the
public support contract.

## License

HyperDRC is available under the [MIT License](LICENSE).

## Extension protocol

Extensions follow the stack-wide [admission protocol](../../docs/admission.md).
It defines the exactness, certainty, boundary, cache, validation, and evidence
requirements shared by the Hyper crates.
