# Changelog

## [0.3.0] - Unreleased

Initial release.

HyperDRC is a Rust library and command-line design-readiness reviewer for
printed-circuit-board release packages. It loads Gerber, KiCad, Excellon,
IPC-D-356, archives, and manufacturing sidecars, runs an explicit set of
fabrication, assembly, test, package, and policy checks, and reports findings
for local review or CI. It is a preflight layer, not a fabricator-certified
CAM engine: a passing report does not guarantee manufacturability.

- `run` executes the same discovery, parsing, policy, waiver, baseline, and
  reporting pipeline as the `hyperdrc` binary without exiting the process.
  `ReadinessRunner` runs an ordered check plan for native authoring systems
  that already own their input model.
- Every selected check receives a `CheckExecutionStatus` (`Passed`, `Failed`,
  `NotApplicable`, `Uncertain`, or `Skipped`), so a check does not silently
  disappear when an input lacks data.
- Inputs include Gerber with X2/X3 metadata, KiCad board files with rule
  context, Excellon drill programs, IPC-D-356 test data, ZIP, TAR, TAR.GZ,
  and TGZ release packages, and BOM, centroid, netlist, stackup, material,
  drawing, and manifest sidecars.
- A completed `Report` can be written as text, JSON, JSON Lines, GeoJSON,
  SARIF, GitHub annotations, HTML, JUnit, SQLite, Arrow IPC, or Parquet, with
  SVG, Gerber, Excellon, DXF, PDF, KiCad, IPC-D-356, GenCAD, and IPC-2581
  review companions.
- Rule configurations, capability profiles (`opinionated_prototype_profile`,
  `mainstream_profile`, and `hdi_profile`), waivers, and baselines are
  explicit inputs; profiles are validated and digested so a report names the
  policy it used.
- Geometry decisions use Hyperreal, Hyperlimit, Hypercurve, and Hyperpath
  under a strict predicate policy and a shared work budget. A required
  decision that cannot be certified produces a `geometry-uncertainty` finding
  and an `Uncertain` check status rather than a pass.
- The binary exits with status 1 for malformed input and status 2 for active
  findings; `--allow-findings` suppresses the latter. The KiCad loader
  rejects malformed straight-track fields with `kicad::TrackLoadError` and
  returns no partial board.
- Full ODB++ and IPC-2581 import, glyph-accurate text, general custom-pad
  Booleans, and electromagnetic or thermal field solving are not supported.
  Some checks use documented proxies: circular proxies for drill and slot
  inputs that lack complete geometry, and an endpoint-span proxy instead of
  routed net length.
- XML and spreadsheet sidecars are read with quick-xml 0.42 and calamine
  0.37, which include the fixes for the quick-xml advisories
  RUSTSEC-2026-0194 and RUSTSEC-2026-0195.
- Requires Rust 1.95.0 or later.
