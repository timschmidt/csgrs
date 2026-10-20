# Changelog

## [0.3.0] - Unreleased

### Breaking

- `PhysicsError` gains `TriangleIndexOutOfBounds` and `IntegrationPolicy`
  gains `SymplecticEulerReplay`. Both enums are exhaustive, so `match`
  expressions over them must handle the new variants.
- `MaterialPropertyGraph::resolve` compares exact assertions by certified value
  instead of structural equality. Numerically equal assertions written in
  different forms now resolve to one value. Assertions whose agreement cannot
  be certified report `PropertyResolutionStatus::Unknown` instead of
  `Conflict`.

### Added

- Exact GJK collision and distance queries: `gjk_query_3d` and
  `gjk_query_3d_with_config` over any pair of `ExactSupportMap3` shapes. A
  `GjkQueryReport3` carries the intersection classification, termination
  reason, iteration count, final simplex, closest-point witnesses, and the
  exact squared distance and distance.
- `triangle_mesh_uniform_density_mass_properties` computes uniform-density mass
  properties of a `hypermesh::TriangleMesh`. Out-of-range triangle indices
  return `PhysicsError::TriangleIndexOutOfBounds`. Repeated queries on the
  same mesh buffers and density reuse the result from a per-thread cache of
  eight entries.
- `ClosedTriangleMesh3::to_hypermesh_exact` converts a physics mesh to a
  `hypermesh::TriangleMesh` and returns Hypermesh's error if polygon-soup
  preparation fails.
- `StepReplayReport3::symplectic_euler_replay` names the velocity-first
  (symplectic) Euler update that `explicit_euler_replay` has always performed.
  `explicit_euler_replay` keeps its policy label and results.
- `BeerLambertSlabReport::transmittance` evaluates `exp(-optical_depth)` as an
  exact real on demand.
- `dispatch-trace` feature, which forwards the exact-dispatch instrumentation
  of `hyperreal`, `hyperlattice`, `hyperlimit`, and `hypermesh`.

### Changed

- Sign and ordering decisions use `hyperlimit`'s strict predicate policy
  (`PredicatePolicy::STRICT`) instead of `Real::refine_sign_until(-64)` or
  `PartialOrd`. Box, plane, segment, and AABB contact classification delegate
  to the matching `hyperlimit` predicates. Decisions the policy cannot certify
  still return the existing unknown or error outcome.
- New dependencies on `hyperlimit` and `hypermesh`.
- Mass properties accumulate exact numerators and divide once per mesh rather
  than once per triangle. Triangle point classification derives its result from
  the edge signs it already computed. `PERFORMANCE.md` records local
  benchmark medians before and after these changes, for example 3.669 µs to
  1.287 µs for the exact triangle point report.
- License is now `MIT OR Apache-2.0` (was `Apache-2.0`).
- Declares a minimum supported Rust version (MSRV) of 1.95.0; 0.2.1 declared
  none.
- Development moved to the
  [csgrs repository](https://github.com/timschmidt/csgrs), which the
  `repository` field now names.

### Removed

- The empty placeholder `hyperphysics` binary target.

For 0.2.1 and earlier, see [crates.io](https://crates.io/crates/hyperphysics/versions).
