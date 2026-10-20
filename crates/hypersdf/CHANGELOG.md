# Changelog

## [0.2.0] - Unreleased

Initial release.

Hypersdf provides signed-distance and implicit-field expressions for the
Hyper geometry stack. It retains expression structure and exact parameters so
that inside, boundary, and outside questions are answered from the field
itself, not from primitive-float samples or a preview mesh. Not every
supported expression is certified as a Euclidean distance;
`SdfMetricStatus::SignEquivalent` marks fields whose sign and zero set are
usable but whose value is not a distance guarantee.

- `SdfExpr` builds retained fields from coordinates, constants, and the plane,
  sphere, box, rounded box, cylinder, capsule, torus, and slab primitives;
  sign-based `union`, `intersection`, and `complement`; scalar arithmetic,
  `abs`, `sqrt`, `sin`, `cos`, and `tan`; and `offset`, `translate`, and
  `affine_transform`.
- `Sdf` caches structural facts once (`SdfFacts`) and answers exact or
  certified queries: `evaluate_point`, `classify_point`, conservative
  `classify_cell`, `interval_cell`, `gradient_point`, `normal_point`, and
  `lipschitz_cell`, with batch forms.
- Reports keep domain, evidence, metric, gradient, normal, and Lipschitz
  status separate. Invalid domains, undecided branches, unsupported
  trigonometric cell ranges, and scalar failures are explicit statuses or
  typed `SdfEvaluationError` values; unknown evidence is not reported as
  outside.
- `Metaball` and `MetaballField` evaluate the normalized inverse-square field
  `(iso - I) / (iso + I)` exactly, without truncating distant contributions.
  The field is sign-equivalent, not a distance.
- Preview and proposal APIs lower exact grid points to `f32` or `f64` for
  sampling, Surface Nets and dual-contouring diagnostics, and GLSL export.
  Their output is explicitly lossy display or proposal data and does not
  establish exact topology.
- `replay_projection_proposal` replays exact boundary classification for a
  surface point proposed by an external solver before certifying it.
- `Sdf::classify_voxel_grid` conservatively classifies an exact voxel grid;
  the optional `hypervoxel-adapter` feature lowers the result into Hypervoxel
  continuous-field intake records. There are no default features.
- Requires Rust 1.95.0 or later.
