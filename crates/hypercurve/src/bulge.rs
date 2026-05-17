//! Exact bulge vertex helpers for line and circular-arc chains.

use hyperreal::Real;

use crate::{CurveResult, Point2, Segment2};

/// A point plus the bulge for the outgoing segment.
#[derive(Clone, Debug, PartialEq)]
pub struct BulgeVertex2 {
    point: Point2,
    bulge: Real,
}

impl BulgeVertex2 {
    /// Constructs a bulge vertex from a point and outgoing bulge.
    pub const fn new(point: Point2, bulge: Real) -> Self {
        Self { point, bulge }
    }

    /// Returns the vertex point.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }

    /// Returns the outgoing bulge.
    pub const fn bulge(&self) -> &Real {
        &self.bulge
    }

    /// Builds the outgoing segment from this vertex to `next`.
    pub fn segment_to(&self, next: &Self) -> CurveResult<Segment2> {
        Segment2::from_bulge(self.point.clone(), next.point.clone(), self.bulge.clone())
    }
}
