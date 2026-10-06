//! Axis-aligned bounding boxes over exact point carriers.

use crate::{Point3, Real};

/// Three-dimensional axis-aligned bounding box.
#[derive(Clone, Debug, PartialEq)]
pub struct Aabb {
    /// Minimum corner.
    pub mins: Point3,
    /// Maximum corner.
    pub maxs: Point3,
}

impl Aabb {
    /// Constructs an axis-aligned bounding box from minimum and maximum corners.
    #[inline]
    pub const fn new(mins: Point3, maxs: Point3) -> Self {
        Self { mins, maxs }
    }

    /// Constructs the degenerate box at the origin.
    #[inline]
    pub fn origin() -> Self {
        let zero = Real::zero();
        Self::new(
            Point3::new(zero.clone(), zero.clone(), zero.clone()),
            Point3::new(zero.clone(), zero.clone(), zero),
        )
    }
}
