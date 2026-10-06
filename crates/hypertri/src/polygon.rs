//! Shared polygon input normalization.
//!
//! Polygon triangulation algorithms should consume this module rather than
//! reading hole indices independently. The separation keeps region semantics
//! explicit and prevents algorithms from interpreting the same flat input
//! differently.

use crate::error::{Error, Result};
#[cfg(any(feature = "earcut", all(feature = "cdt", feature = "runtime-select")))]
use crate::predicate_evaluator::PredicateEvaluator;
use crate::types::ExactPoint;

/// A borrowed ring over a flat polygon vertex buffer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RingRange {
    /// First vertex index in the flat input.
    pub start: usize,
    /// One-past-the-end vertex index in the flat input.
    pub end: usize,
}

impl RingRange {
    /// Number of vertices in the ring range.
    pub const fn len(self) -> usize {
        self.end - self.start
    }

    /// Returns true when the ring has no vertices.
    pub const fn is_empty(self) -> bool {
        self.start == self.end
    }
}

/// Validated borrowed polygon rings.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PolygonRings {
    exterior: RingRange,
    holes: Vec<RingRange>,
}

impl PolygonRings {
    /// Exterior ring range.
    pub const fn exterior(&self) -> RingRange {
        self.exterior
    }

    /// Hole ring ranges.
    pub fn holes(&self) -> &[RingRange] {
        &self.holes
    }

    /// Returns true when the polygon has holes.
    pub fn has_holes(&self) -> bool {
        !self.holes.is_empty()
    }
}

/// Validate earcut-compatible hole indices and return ring ranges.
pub fn rings_from_hole_indices(
    vertices: &[ExactPoint],
    hole_indices: &[usize],
) -> Result<PolygonRings> {
    if vertices.is_empty() {
        if hole_indices.is_empty() {
            return Ok(PolygonRings {
                exterior: RingRange { start: 0, end: 0 },
                holes: Vec::new(),
            });
        }
        return Err(Error::InvalidInput {
            reason: "holes require exterior vertices",
        });
    }

    let mut previous = 0;
    for &hole_start in hole_indices {
        if hole_start <= previous || hole_start >= vertices.len() {
            return Err(Error::InvalidInput {
                reason: "hole indices must be strictly increasing interior starts",
            });
        }
        previous = hole_start;
    }

    let exterior_end = hole_indices.first().copied().unwrap_or(vertices.len());
    if exterior_end < 3 {
        return Err(Error::InvalidInput {
            reason: "exterior ring requires at least three vertices",
        });
    }

    let mut holes = Vec::with_capacity(hole_indices.len());
    for (i, &start) in hole_indices.iter().enumerate() {
        let end = hole_indices.get(i + 1).copied().unwrap_or(vertices.len());
        if end - start < 3 {
            return Err(Error::InvalidInput {
                reason: "hole ring requires at least three vertices",
            });
        }
        holes.push(RingRange { start, end });
    }

    Ok(PolygonRings {
        exterior: RingRange {
            start: 0,
            end: exterior_end,
        },
        holes,
    })
}

/// Build a ring index list, dropping a duplicated closing point when present.
#[cfg(any(feature = "earcut", all(feature = "cdt", feature = "runtime-select")))]
pub(crate) fn open_ring_indices(
    evaluator: &PredicateEvaluator,
    vertices: &[ExactPoint],
    range: RingRange,
) -> Result<Vec<usize>> {
    if range.len() < 2 {
        return Ok((range.start..range.end).collect());
    }

    let mut end = range.end;
    if crate::predicates::points_equal(evaluator, &vertices[range.start], &vertices[range.end - 1])?
    {
        end -= 1;
    }

    Ok((range.start..end).collect())
}

#[cfg(all(
    test,
    any(feature = "earcut", all(feature = "cdt", feature = "runtime-select"))
))]
mod tests {
    use super::*;
    use crate::context::TriangulationContext;
    use crate::types::Real;

    #[test]
    fn open_ring_handles_short_and_explicitly_closed_ranges() {
        let context = TriangulationContext::new(hyperlimit::PredicatePolicy::STRICT);
        let evaluator = PredicateEvaluator::new(&context);
        let points = vec![
            ExactPoint::new(Real::from(0), Real::from(0)),
            ExactPoint::new(Real::from(2), Real::from(0)),
            ExactPoint::new(Real::from(0), Real::from(2)),
            ExactPoint::new(Real::from(0), Real::from(0)),
        ];

        assert_eq!(
            open_ring_indices(&evaluator, &points, RingRange { start: 1, end: 2 }).unwrap(),
            vec![1]
        );
        assert_eq!(
            open_ring_indices(&evaluator, &points, RingRange { start: 0, end: 4 }).unwrap(),
            vec![0, 1, 2]
        );
    }
}
