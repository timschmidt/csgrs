//! Exact measurements over retained Bezier/conic carriers.
//!
//! Retained regions may contain algebraic endpoint-image fragments that are not
//! native Bezier subcurves yet.  This module therefore exposes measurements
//! whose scope is explicit.  An endpoint envelope bounds retained boundary
//! endpoints only: native endpoints contribute exact point coordinates, and
//! algebraic endpoint images contribute the certified isolating intervals of
//! their represented coordinates.  It never samples an algebraic root and it
//! does not claim curve-interior extrema.  This is the construction/decision
//! split advocated by Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 3-23 (1997); endpoint intervals are
//! retained exact objects, while callers decide how to consume the conservative
//! envelope.  The broad-phase role mirrors Bentley and Ottmann's sweep-line
//! candidate filters, "Algorithms for Reporting and Counting Geometric
//! Intersections," *IEEE Transactions on Computers* C-28(9), 643-647 (1979).

use hyperreal::Real;
use hypersolve::AlgebraicRootRepresentation;

use crate::classify::compare_reals;
use crate::{
    Aabb2, BezierEndpointPointImage2, BezierRetainedBoundaryLoop2, BezierRetainedRegion2,
    BezierSplitFragment2, Classification, CurvePolicy, Point2, UncertaintyReason,
};

/// Exact endpoint envelope for a retained Bezier region or loop.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierRetainedEndpointEnvelope2 {
    envelope: Aabb2,
    native_endpoint_count: usize,
    algebraic_endpoint_count: usize,
}

impl BezierRetainedEndpointEnvelope2 {
    /// Constructs an endpoint envelope for a retained region.
    ///
    /// Empty regions are unsupported because there is no finite neutral
    /// envelope. Retained algebraic fragments must provide endpoint point
    /// images for every endpoint they contribute; otherwise the envelope is
    /// explicit boundary uncertainty rather than a partial box.
    pub fn from_region(
        region: &BezierRetainedRegion2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let mut accumulator = EndpointEnvelopeAccumulator::default();
        for boundary_loop in region.boundary_loops() {
            match accumulator.include_loop(boundary_loop, policy) {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
        accumulator.finish(policy)
    }

    /// Constructs an endpoint envelope for one retained boundary loop.
    pub fn from_loop(
        boundary_loop: &BezierRetainedBoundaryLoop2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let mut accumulator = EndpointEnvelopeAccumulator::default();
        match accumulator.include_loop(boundary_loop, policy) {
            Classification::Decided(()) => accumulator.finish(policy),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    /// Returns the conservative endpoint envelope.
    pub const fn envelope(&self) -> &Aabb2 {
        &self.envelope
    }

    /// Returns how many native endpoint points contributed to this envelope.
    pub const fn native_endpoint_count(&self) -> usize {
        self.native_endpoint_count
    }

    /// Returns how many algebraic endpoint images contributed to this envelope.
    pub const fn algebraic_endpoint_count(&self) -> usize {
        self.algebraic_endpoint_count
    }

    /// Returns true when at least one represented algebraic endpoint image
    /// contributed interval evidence.
    pub const fn has_algebraic_endpoints(&self) -> bool {
        self.algebraic_endpoint_count > 0
    }
}

#[derive(Clone, Debug)]
struct CoordinateInterval {
    lower: Real,
    upper: Real,
}

#[derive(Clone, Debug)]
struct EndpointInterval {
    x: CoordinateInterval,
    y: CoordinateInterval,
    kind: EndpointIntervalKind,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum EndpointIntervalKind {
    Native,
    Algebraic,
}

#[derive(Default)]
struct EndpointEnvelopeAccumulator {
    min_x: Option<Real>,
    min_y: Option<Real>,
    max_x: Option<Real>,
    max_y: Option<Real>,
    native_endpoint_count: usize,
    algebraic_endpoint_count: usize,
}

impl EndpointEnvelopeAccumulator {
    fn include_loop(
        &mut self,
        boundary_loop: &BezierRetainedBoundaryLoop2,
        policy: &CurvePolicy,
    ) -> Classification<()> {
        for fragment in boundary_loop.fragments() {
            match self.include_fragment(fragment, policy) {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
        Classification::Decided(())
    }

    fn include_fragment(
        &mut self,
        fragment: &BezierSplitFragment2,
        policy: &CurvePolicy,
    ) -> Classification<()> {
        match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => {
                let (start, end) = curve.endpoints();
                match self.include_endpoint(native_endpoint_interval(&start), policy) {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
                self.include_endpoint(native_endpoint_interval(&end), policy)
            }
            BezierSplitFragment2::AlgebraicEndpointImages {
                start_image,
                end_image,
                ..
            } => {
                let Some(start_image) = start_image else {
                    return Classification::Uncertain(UncertaintyReason::Boundary);
                };
                let Some(end_image) = end_image else {
                    return Classification::Uncertain(UncertaintyReason::Boundary);
                };
                let Some(start) = algebraic_endpoint_interval(start_image.point()) else {
                    return Classification::Uncertain(UncertaintyReason::Boundary);
                };
                let Some(end) = algebraic_endpoint_interval(end_image.point()) else {
                    return Classification::Uncertain(UncertaintyReason::Boundary);
                };
                match self.include_endpoint(start, policy) {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
                self.include_endpoint(end, policy)
            }
            BezierSplitFragment2::Unresolved { .. } => {
                Classification::Uncertain(UncertaintyReason::Boundary)
            }
        }
    }

    fn include_endpoint(
        &mut self,
        endpoint: EndpointInterval,
        policy: &CurvePolicy,
    ) -> Classification<()> {
        if self
            .include_coordinate(&endpoint.x.lower, &endpoint.x.upper, Axis::X, policy)
            .is_none()
            || self
                .include_coordinate(&endpoint.y.lower, &endpoint.y.upper, Axis::Y, policy)
                .is_none()
        {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        }
        match endpoint.kind {
            EndpointIntervalKind::Native => self.native_endpoint_count += 1,
            EndpointIntervalKind::Algebraic => self.algebraic_endpoint_count += 1,
        }
        Classification::Decided(())
    }

    fn include_coordinate(
        &mut self,
        lower: &Real,
        upper: &Real,
        axis: Axis,
        policy: &CurvePolicy,
    ) -> Option<()> {
        if compare_reals(lower, upper, policy)? == std::cmp::Ordering::Greater {
            return None;
        }
        let (min, max) = match axis {
            Axis::X => (&mut self.min_x, &mut self.max_x),
            Axis::Y => (&mut self.min_y, &mut self.max_y),
        };
        match (min.as_mut(), max.as_mut()) {
            (Some(min), Some(max)) => {
                if compare_reals(lower, min, policy)? == std::cmp::Ordering::Less {
                    *min = lower.clone();
                }
                if compare_reals(upper, max, policy)? == std::cmp::Ordering::Greater {
                    *max = upper.clone();
                }
            }
            (None, None) => {
                *min = Some(lower.clone());
                *max = Some(upper.clone());
            }
            _ => return None,
        }
        Some(())
    }

    fn finish(self, _policy: &CurvePolicy) -> Classification<BezierRetainedEndpointEnvelope2> {
        let (Some(min_x), Some(min_y), Some(max_x), Some(max_y)) =
            (self.min_x, self.min_y, self.max_x, self.max_y)
        else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        Classification::Decided(BezierRetainedEndpointEnvelope2 {
            envelope: Aabb2::new_unchecked(Point2::new(min_x, min_y), Point2::new(max_x, max_y)),
            native_endpoint_count: self.native_endpoint_count,
            algebraic_endpoint_count: self.algebraic_endpoint_count,
        })
    }
}

#[derive(Clone, Copy)]
enum Axis {
    X,
    Y,
}

fn native_endpoint_interval(point: &Point2) -> EndpointInterval {
    EndpointInterval {
        x: CoordinateInterval {
            lower: point.x().clone(),
            upper: point.x().clone(),
        },
        y: CoordinateInterval {
            lower: point.y().clone(),
            upper: point.y().clone(),
        },
        kind: EndpointIntervalKind::Native,
    }
}

fn algebraic_endpoint_interval(point: &BezierEndpointPointImage2) -> Option<EndpointInterval> {
    let (x, y) = match point {
        BezierEndpointPointImage2::Polynomial(point) => {
            (point.x()?.representation()?, point.y()?.representation()?)
        }
        BezierEndpointPointImage2::RationalQuadratic(point) => {
            (point.x()?.representation()?, point.y()?.representation()?)
        }
    };
    Some(EndpointInterval {
        x: represented_coordinate_interval(x),
        y: represented_coordinate_interval(y),
        kind: EndpointIntervalKind::Algebraic,
    })
}

fn represented_coordinate_interval(root: &AlgebraicRootRepresentation) -> CoordinateInterval {
    if let Some(witness) = root.exact_rational_witness() {
        return CoordinateInterval {
            lower: witness.clone(),
            upper: witness.clone(),
        };
    }
    CoordinateInterval {
        lower: root.interval.lower.clone(),
        upper: root.interval.upper.clone(),
    }
}
