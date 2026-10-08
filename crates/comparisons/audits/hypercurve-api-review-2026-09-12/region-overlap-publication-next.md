# Public region overlap evidence — completed migration

Completed by Hypercurve `dfc8ff21c6f86f4feb8e75f90da010cd5c0f3bf2`. The following is the preserved pre-implementation audit; final behavior and qualification are in `region-overlap-correspondence-evidence-audit.md`.

The current common dispatcher retains nonlinear and selected overlap correspondence. Region publication still discards it:

- `CurveRegionIntersectionOverlap2` stores an optional native `CurveIntersectionOverlap2` separately from its clipped ranges and orientation.
- `CurveRegionBooleanContext::build_evidence` keeps `RegionPairOverlapSource::Bezier` but maps `Correspondence` and `AlgebraicChordRational` to `None`.
- `CurveRegionIntersectionOverlap2::source()` therefore exposes construction history instead of a general replay contract.

The intended correction is one retained overlap authority shared by common and region reports, with original correspondence charts, active paired ranges, and endpoint inclusion. Remove the native-only optional source interface and migrate controlled callers. Preserve the distinction between clipping a positive-length component and retaining an isolated open-path contact.

The earlier assumption that the region chord/rational adapter already implements general inverse clipping was incorrect. `clip_algebraic_chord_rational_overlap` clips to the source range and relies on the region carrier's own finite chord domain. The common `ChordRational` adapter does the same forward clipping but checks a separately supplied chord restriction and reports unsupported if an inverse restriction is needed. Consolidation must retain that check or implement the inverse branch proof; merely deleting it would lose domain semantics.

Migrating all region pair dispatch wholesale to the current common dispatcher would be premature: the common analytic adapters explicitly guard the unit source domain. Exact exterior results produced by TrimOrExtend must keep their existing region routes until complete finite-domain correspondence is supported. Public evidence retention can be implemented without introducing that regression.
