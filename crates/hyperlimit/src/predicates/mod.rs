//! Geometric predicate implementations and their shared decision machinery.
//!
//! Public crate exports select the modeling-facing operation names from these
//! modules. Input validity, degeneracy, certainty and replay contracts belong
//! to each operation; a common module tree does not imply universal completion
//! or uniform report contents. Scalar arithmetic remains owned by Hyperreal.

/// Axis-aligned box membership, intersection and broad-phase query helpers.
pub mod aabb;
/// Point membership in convex polygon and oriented-plane halfspace carriers.
pub mod convex;
/// Projected coplanar triangle relations and exact planar construction helpers.
pub mod coplanar;
/// Shared local certainty/stage aggregation and unresolved-child transport.
pub(crate) mod decision;
/// Squared-distance comparisons and circle, sphere and box proximity relations.
pub mod distance;
/// Retained directional support slabs, k-DOP relations and their witness reports.
pub mod dop;
/// Private exact-rational determinant schedules used by geometric predicates.
mod exact;
/// Enclosure-based sign filters and their policy-qualified outcomes.
pub mod filters;
/// Closed three-dimensional halfspace feasibility and witness replay.
pub mod halfspace;
/// Ordered closed-interval membership and intersection predicates.
pub mod interval;
/// Dimension-parameterized orientation, affine independence and in-sphere predicates.
pub mod nd;
/// Scalar and lexicographic point comparisons, equality and ordered scalar helpers.
pub mod order;
/// Orientation and lifted-circle/sphere predicates with retained geometric evidence.
pub mod orient;
/// Polygon-ring turn/area summaries and even-odd point membership reports.
pub mod ring;
/// Planar/spatial point-segment and segment-segment relations and line intersections.
pub mod segment;
/// Segment-plane classification, exact parameter constructions and retained event reports.
pub mod segment_plane;
/// Simplex membership and segment/ray-triangle relations with construction reports.
pub mod triangle;
/// Spatial triangle-triangle classification and validation of retained relations.
pub mod triangle_triangle;
