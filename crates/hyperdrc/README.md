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
- `PcbRegionExt` supplies conversions used by geometry checks;
- `exact_path_rules::certify_annular_ring` and
  `classify_via_drill_policy` delegate via geometry to Hyperpath while keeping
  DRC policy labels in HyperDRC;
- `geometry` exposes source-unit/grid facts, primitive construction, and
  report-shape projections.

An uncertain certified offset becomes `PcbGeometryUncertainty` and then an
error-severity `geometry-uncertainty` finding. The pipeline does not silently
replace it with an approximate result.

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
