//! Exact coplanar triangle overlap classification.
//!
//! `hyperlimit` owns the reusable projected-predicate machinery: coordinate
//! projection selection, projected segment/triangle predicates, retained
//! predicate certificates, and self-validation. `hypermesh` re-exports those
//! helpers so mesh topology code consumes one shared implementation of the
//! Yap-style predicate/construction boundary.

pub use hyperlimit::{
    CoplanarProjection, CoplanarTriangleClassification, CoplanarTriangleRelation,
    CoplanarTriangleValidationError, choose_coplanar_projection, classify_coplanar_triangle_points,
    classify_coplanar_triangles, derive_coplanar_triangle_relation, orient2d_value, project_point3,
    project_triangle3, projected_line_parameter3, projected_polygon_area2_sign,
    projected_polygon_area2_value, projected_segment_parameter3,
};
