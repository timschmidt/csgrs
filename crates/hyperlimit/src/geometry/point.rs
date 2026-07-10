//! Predicate-facing point aliases.
//!
//! Point storage and structural point facts live in `hyperlattice`. This module
//! keeps the existing `hyperlimit::geometry` API as the predicate-facing import
//! point while avoiding a second owner for shared-scale and sparse point facts.

pub use hyperlattice::{Point2, Point2Facts, Point3, Point3Facts, PointSharedScaleView};
