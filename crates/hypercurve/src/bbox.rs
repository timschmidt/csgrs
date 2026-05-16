//! Exactness-aware axis-aligned bounding boxes for planar curve primitives.
//!
//! These boxes are deliberately a broad-phase filter, not a replacement for
//! exact segment topology. A decided non-overlap lets callers skip an exact
//! segment relation; uncertain ordering falls back to the exact relation. This
//! mirrors the candidate-pruning role of bounding intervals in sweep-line
//! intersection algorithms such as Bentley and Ottmann, "Algorithms for
//! Reporting and Counting Geometric Intersections" (1979).

use std::cmp::Ordering;

use hyperlattice::{Backend, DefaultBackend, Scalar};

use crate::classify::compare_scalars;
use crate::{
    CircularArc2, Classification, Contour2, CurvePolicy, CurveResult, CurveString2, LineSeg2,
    Point2, Region2, RegionView2, Segment2, UncertaintyReason,
};

/// An axis-aligned bounding box for two-dimensional curve geometry.
///
/// The box is closed: points on `min`/`max` edges are considered contained and
/// two boxes whose edges touch are considered overlapping. All constructors
/// return uncertainty when the active policy cannot order a needed coordinate.
#[derive(Clone, Debug, PartialEq)]
pub struct Aabb2<B: Backend = DefaultBackend> {
    min: Point2<B>,
    max: Point2<B>,
}

impl<B: Backend> Aabb2<B> {
    /// Constructs a box without checking `min <= max`.
    ///
    /// Prefer the geometry constructors unless the caller already proved the
    /// coordinate ordering.
    pub const fn new_unchecked(min: Point2<B>, max: Point2<B>) -> Self {
        Self { min, max }
    }

    /// Constructs a zero-area box containing exactly one point.
    pub fn from_point(point: Point2<B>) -> Self {
        Self {
            min: point.clone(),
            max: point,
        }
    }

    /// Constructs the smallest box containing all supplied points.
    ///
    /// Empty input is reported as unsupported because there is no neutral finite
    /// bounding box for an empty point set in this crate's scalar model.
    pub fn from_points<'a, I>(points: I, policy: &CurvePolicy) -> Classification<Self>
    where
        I: IntoIterator<Item = &'a Point2<B>>,
        B: 'a,
    {
        let mut points = points.into_iter();
        let Some(first) = points.next() else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };

        let mut min_x = first.x().clone();
        let mut min_y = first.y().clone();
        let mut max_x = first.x().clone();
        let mut max_y = first.y().clone();

        for point in points {
            if include_coordinate(&mut min_x, &mut max_x, point.x(), policy).is_none()
                || include_coordinate(&mut min_y, &mut max_y, point.y(), policy).is_none()
            {
                return Classification::Uncertain(UncertaintyReason::Ordering);
            }
        }

        Classification::Decided(Self {
            min: Point2::new(min_x, min_y),
            max: Point2::new(max_x, max_y),
        })
    }

    /// Constructs the bounding box of a finite line segment.
    pub fn from_line(line: &LineSeg2<B>, policy: &CurvePolicy) -> Classification<Self> {
        Self::from_points([line.start(), line.end()], policy)
    }

    /// Constructs the bounding box of a finite circular arc.
    ///
    /// The endpoints always contribute. Cardinal circle extrema contribute only
    /// when the arc sweep contains the corresponding point, preserving native
    /// circular-arc geometry without tessellation. If sweep membership at a
    /// cardinal point is uncertain, the box is uncertain because a too-small box
    /// would make broad-phase pruning unsound.
    pub fn from_arc(
        arc: &CircularArc2<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let mut bbox = match Self::from_points([arc.start(), arc.end()], policy) {
            Classification::Decided(bbox) => bbox,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        let radius = arc.radius_squared().sqrt()?;
        let candidates = [
            Point2::new(arc.center().x() + &radius, arc.center().y().clone()),
            Point2::new(arc.center().x() - &radius, arc.center().y().clone()),
            Point2::new(arc.center().x().clone(), arc.center().y() + &radius),
            Point2::new(arc.center().x().clone(), arc.center().y() - &radius),
        ];

        for candidate in &candidates {
            match arc.contains_sweep_point(candidate, policy) {
                Classification::Decided(true) => match bbox.include_point(candidate, policy) {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }

        Ok(Classification::Decided(bbox))
    }

    /// Constructs the bounding box of a native line or circular-arc segment.
    pub fn from_segment(
        segment: &Segment2<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        match segment {
            Segment2::Line(line) => Ok(Self::from_line(line, policy)),
            Segment2::Arc(arc) => Self::from_arc(arc, policy),
        }
    }

    /// Constructs the bounding box of an open curve string.
    pub fn from_curve_string(
        curve: &CurveString2<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let mut segments = curve.segments().iter();
        let Some(first) = segments.next() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };

        let mut bbox = match Self::from_segment(first, policy)? {
            Classification::Decided(bbox) => bbox,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        for segment in segments {
            let segment_bbox = match Self::from_segment(segment, policy)? {
                Classification::Decided(bbox) => bbox,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            bbox = match bbox.union(&segment_bbox, policy) {
                Classification::Decided(bbox) => bbox,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        }

        Ok(Classification::Decided(bbox))
    }

    /// Constructs the bounding box of a closed contour.
    pub fn from_contour(
        contour: &Contour2<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        Self::from_curve_string(contour.curve_string(), policy)
    }

    /// Constructs the bounding box of an owned region.
    ///
    /// Material and hole contours both contribute because a region-level box is
    /// a broad-phase envelope for boundary topology, not a filled-area
    /// containment proof.
    pub fn from_region(
        region: &Region2<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        Self::from_region_view(&region.as_view(), policy)
    }

    /// Constructs the bounding box of a borrowed region view.
    ///
    /// Empty regions report unsupported because there is no finite closed box
    /// that represents the absence of geometry. Callers that need empty-region
    /// fast paths should handle emptiness before asking for a box.
    pub fn from_region_view(
        region: &RegionView2<'_, B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let mut contours = region
            .material_contours()
            .iter()
            .chain(region.hole_contours().iter())
            .copied();
        let Some(first) = contours.next() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };

        let mut bbox = match Self::from_contour(first, policy)? {
            Classification::Decided(bbox) => bbox,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        for contour in contours {
            let contour_bbox = match Self::from_contour(contour, policy)? {
                Classification::Decided(bbox) => bbox,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            bbox = match bbox.union(&contour_bbox, policy) {
                Classification::Decided(bbox) => bbox,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        }

        Ok(Classification::Decided(bbox))
    }

    /// Returns the minimum corner.
    pub const fn min(&self) -> &Point2<B> {
        &self.min
    }

    /// Returns the maximum corner.
    pub const fn max(&self) -> &Point2<B> {
        &self.max
    }

    /// Returns the minimum x coordinate.
    pub const fn min_x(&self) -> &Scalar<B> {
        self.min.x()
    }

    /// Returns the minimum y coordinate.
    pub const fn min_y(&self) -> &Scalar<B> {
        self.min.y()
    }

    /// Returns the maximum x coordinate.
    pub const fn max_x(&self) -> &Scalar<B> {
        self.max.x()
    }

    /// Returns the maximum y coordinate.
    pub const fn max_y(&self) -> &Scalar<B> {
        self.max.y()
    }

    /// Expands this box so it contains `point`.
    pub fn include_point(&mut self, point: &Point2<B>, policy: &CurvePolicy) -> Classification<()> {
        let mut min_x = self.min.x().clone();
        let mut min_y = self.min.y().clone();
        let mut max_x = self.max.x().clone();
        let mut max_y = self.max.y().clone();

        if include_coordinate(&mut min_x, &mut max_x, point.x(), policy).is_none()
            || include_coordinate(&mut min_y, &mut max_y, point.y(), policy).is_none()
        {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        }

        self.min = Point2::new(min_x, min_y);
        self.max = Point2::new(max_x, max_y);
        Classification::Decided(())
    }

    /// Returns the smallest box containing both inputs.
    pub fn union(&self, other: &Self, policy: &CurvePolicy) -> Classification<Self> {
        let mut merged = self.clone();
        match merged.include_point(other.min(), policy) {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
        match merged.include_point(other.max(), policy) {
            Classification::Decided(()) => Classification::Decided(merged),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    /// Classifies whether this closed box contains `point`.
    pub fn contains_point(&self, point: &Point2<B>, policy: &CurvePolicy) -> Classification<bool> {
        let Some(x_inside) = scalar_between(point.x(), self.min_x(), self.max_x(), policy) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        if !x_inside {
            return Classification::Decided(false);
        }

        let Some(y_inside) = scalar_between(point.y(), self.min_y(), self.max_y(), policy) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        Classification::Decided(y_inside)
    }

    /// Classifies whether two closed boxes overlap.
    ///
    /// Edge and corner contacts count as overlap. This inclusive convention is
    /// necessary for tangent, endpoint, and shared-boundary curve topology.
    pub fn overlaps(&self, other: &Self, policy: &CurvePolicy) -> Classification<bool> {
        let Some(self_left_of_other) = scalar_less(self.max_x(), other.min_x(), policy) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(other_left_of_self) = scalar_less(other.max_x(), self.min_x(), policy) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(self_below_other) = scalar_less(self.max_y(), other.min_y(), policy) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(other_below_self) = scalar_less(other.max_y(), self.min_y(), policy) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };

        Classification::Decided(
            !(self_left_of_other || other_left_of_self || self_below_other || other_below_self),
        )
    }
}

pub(crate) fn decided_segment_aabb<B: Backend>(
    segment: &Segment2<B>,
    policy: &CurvePolicy,
) -> Option<Aabb2<B>> {
    match Aabb2::from_segment(segment, policy) {
        Ok(Classification::Decided(bbox)) => Some(bbox),
        Ok(Classification::Uncertain(_)) | Err(_) => None,
    }
}

pub(crate) fn decided_contour_aabb<B: Backend>(
    contour: &Contour2<B>,
    policy: &CurvePolicy,
) -> Option<Aabb2<B>> {
    match Aabb2::from_contour(contour, policy) {
        Ok(Classification::Decided(bbox)) => Some(bbox),
        Ok(Classification::Uncertain(_)) | Err(_) => None,
    }
}

pub(crate) fn aabbs_decided_disjoint<B: Backend>(
    first: &Aabb2<B>,
    second: &Aabb2<B>,
    policy: &CurvePolicy,
) -> bool {
    matches!(
        first.overlaps(second, policy),
        Classification::Decided(false)
    )
}

pub(crate) fn aabb_decided_misses_point<B: Backend>(
    bbox: &Aabb2<B>,
    point: &Point2<B>,
    policy: &CurvePolicy,
) -> bool {
    matches!(
        bbox.contains_point(point, policy),
        Classification::Decided(false)
    )
}

fn include_coordinate<B: Backend>(
    min: &mut Scalar<B>,
    max: &mut Scalar<B>,
    value: &Scalar<B>,
    policy: &CurvePolicy,
) -> Option<()> {
    if matches!(compare_scalars(value, min, policy)?, Ordering::Less) {
        *min = value.clone();
    }
    if matches!(compare_scalars(value, max, policy)?, Ordering::Greater) {
        *max = value.clone();
    }
    Some(())
}

fn scalar_less<B: Backend>(
    left: &Scalar<B>,
    right: &Scalar<B>,
    policy: &CurvePolicy,
) -> Option<bool> {
    Some(matches!(
        compare_scalars(left, right, policy)?,
        Ordering::Less
    ))
}

fn scalar_between<B: Backend>(
    value: &Scalar<B>,
    min: &Scalar<B>,
    max: &Scalar<B>,
    policy: &CurvePolicy,
) -> Option<bool> {
    let lower = compare_scalars(value, min, policy)?;
    let upper = compare_scalars(value, max, policy)?;
    Some(!matches!(lower, Ordering::Less) && !matches!(upper, Ordering::Greater))
}
