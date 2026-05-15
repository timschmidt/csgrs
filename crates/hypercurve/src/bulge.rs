//! Cavalier-compatible bulge vertex helpers.

use hyperlattice::{Backend, DefaultBackend, Scalar};

use crate::{CurveResult, Point2, Segment2};

/// A point plus the bulge for the outgoing segment.
#[derive(Clone, Debug, PartialEq)]
pub struct BulgeVertex2<B: Backend = DefaultBackend> {
    point: Point2<B>,
    bulge: Scalar<B>,
}

impl<B: Backend> BulgeVertex2<B> {
    /// Constructs a bulge vertex from a point and outgoing bulge.
    pub const fn new(point: Point2<B>, bulge: Scalar<B>) -> Self {
        Self { point, bulge }
    }

    /// Returns the vertex point.
    pub const fn point(&self) -> &Point2<B> {
        &self.point
    }

    /// Returns the outgoing bulge.
    pub const fn bulge(&self) -> &Scalar<B> {
        &self.bulge
    }

    /// Builds the outgoing segment from this vertex to `next`.
    pub fn segment_to(&self, next: &Self) -> CurveResult<Segment2<B>> {
        Segment2::from_cavalier_bulge(self.point.clone(), next.point.clone(), self.bulge.clone())
    }
}
