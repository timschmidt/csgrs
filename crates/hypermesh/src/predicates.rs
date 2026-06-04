//! Predicate-backed geometric checks used by exact mesh validation.
//!
//! Triangle degeneracy classification is shared through `hyperlimit`, where it
//! is implemented as exact projected orientation predicates with retained
//! certificates. Re-exporting the shared implementation keeps `hypermesh`
//! of maintaining a second determinant helper.

pub use hyperlimit::{
    TriangleDegeneracy, TrianglePredicateReport,
    classify_triangle3_degeneracy as classify_triangle_degeneracy,
};
