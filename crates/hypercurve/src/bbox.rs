//! Exactness-aware axis-aligned bounding boxes for planar curve primitives.
//!
//! These boxes are deliberately a broad-phase filter, not a replacement for
//! exact segment topology. A decided non-overlap lets callers skip an exact
//! segment relation; uncertain ordering falls back to the exact relation. This
//! mirrors the candidate-pruning role of bounding intervals in sweep-line
//! intersection algorithms such as Bentley and Ottmann, "Algorithms for
//! Reporting and Counting Geometric Intersections" (*IEEE Transactions on
//! Computers* C-28(9), 643-647, 1979).

use std::cmp::Ordering;

use hyperreal::Real;

use crate::classify::compare_reals;
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
pub struct Aabb2 {
    min: Point2,
    max: Point2,
}

impl Aabb2 {
    /// Constructs a box without checking `min <= max`.
    ///
    /// Prefer the geometry constructors unless the caller already proved the
    /// coordinate ordering.
    pub const fn new_unchecked(min: Point2, max: Point2) -> Self {
        Self { min, max }
    }

    /// Constructs a zero-area box containing exactly one point.
    pub fn from_point(point: Point2) -> Self {
        Self {
            min: point.clone(),
            max: point,
        }
    }

    /// Constructs the smallest box containing all supplied points.
    ///
    /// Empty input is reported as unsupported because there is no neutral finite
    /// bounding box for an empty point set in this crate's Real model.
    pub fn from_points<'a, I>(points: I, policy: &CurvePolicy) -> Classification<Self>
    where
        I: IntoIterator<Item = &'a Point2>,
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
    pub fn from_line(line: &LineSeg2, policy: &CurvePolicy) -> Classification<Self> {
        Self::from_points([line.start(), line.end()], policy)
    }

    /// Constructs the bounding box of a finite circular arc.
    ///
    /// The endpoints always contribute. Cardinal circle extrema contribute only
    /// when the arc sweep contains the corresponding point, preserving native
    /// circular-arc geometry without tessellation. If sweep membership at a
    /// cardinal point is uncertain, the box is uncertain because a too-small box
    /// would make broad-phase pruning unsound. This is the standard conservative
    /// broad-phase rule from computational geometry: filters may remove only
    /// certified misses; uncertain candidates must reach the exact predicate
    /// stage described by Shewchuk's robust-geometry work.
    pub fn from_arc(arc: &CircularArc2, policy: &CurvePolicy) -> CurveResult<Classification<Self>> {
        let mut bbox = match Self::from_points([arc.start(), arc.end()], policy) {
            Classification::Decided(bbox) => bbox,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        if matches!(policy.numeric_mode, crate::NumericMode::EdgePreview) {
            // Preview topology prefers conservative candidate retention over a
            // tight arc box. Cardinal sweep tests contain radicals after
            // rotation; keeping the full circle envelope prevents broad-phase
            // pruning from hiding true line/arc slice events.
            if let (Some(center_x), Some(center_y), Some(radius_squared)) = (
                arc.center().x().to_f64_lossy(),
                arc.center().y().to_f64_lossy(),
                arc.radius_squared().to_f64_lossy(),
            ) && center_x.is_finite()
                && center_y.is_finite()
                && radius_squared.is_finite()
                && radius_squared >= 0.0
            {
                let radius = radius_squared.sqrt();
                let candidates = [
                    Point2::new(Real::try_from(center_x + radius)?, arc.center().y().clone()),
                    Point2::new(Real::try_from(center_x - radius)?, arc.center().y().clone()),
                    Point2::new(arc.center().x().clone(), Real::try_from(center_y + radius)?),
                    Point2::new(arc.center().x().clone(), Real::try_from(center_y - radius)?),
                ];

                return Ok(Self::from_points(
                    [arc.start(), arc.end()]
                        .into_iter()
                        .chain(candidates.iter()),
                    policy,
                ));
            }
        }

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
        segment: &Segment2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        match segment {
            Segment2::Line(line) => Ok(Self::from_line(line, policy)),
            Segment2::Arc(arc) => Self::from_arc(arc, policy),
        }
    }

    /// Constructs the bounding box of an open curve string.
    pub fn from_curve_string(
        curve: &CurveString2,
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
        contour: &Contour2,
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
        region: &Region2,
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
        region: &RegionView2<'_>,
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
    pub const fn min(&self) -> &Point2 {
        &self.min
    }

    /// Returns the maximum corner.
    pub const fn max(&self) -> &Point2 {
        &self.max
    }

    /// Returns the minimum x coordinate.
    pub const fn min_x(&self) -> &Real {
        self.min.x()
    }

    /// Returns the minimum y coordinate.
    pub const fn min_y(&self) -> &Real {
        self.min.y()
    }

    /// Returns the maximum x coordinate.
    pub const fn max_x(&self) -> &Real {
        self.max.x()
    }

    /// Returns the maximum y coordinate.
    pub const fn max_y(&self) -> &Real {
        self.max.y()
    }

    /// Classifies whether the stored corners satisfy `min <= max` on both axes.
    ///
    /// This certifies boxes that entered through [`Self::new_unchecked`] before
    /// they are retained as provenance-bearing evidence.
    pub fn has_valid_ordering(&self, policy: &CurvePolicy) -> Classification<bool> {
        let Some(x_order) = compare_reals(self.min_x(), self.max_x(), policy) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(y_order) = compare_reals(self.min_y(), self.max_y(), policy) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        Classification::Decided(
            !matches!(x_order, Ordering::Greater) && !matches!(y_order, Ordering::Greater),
        )
    }

    /// Expands this box so it contains `point`.
    pub fn include_point(&mut self, point: &Point2, policy: &CurvePolicy) -> Classification<()> {
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
    pub fn contains_point(&self, point: &Point2, policy: &CurvePolicy) -> Classification<bool> {
        let Some(x_inside) = real_between(point.x(), self.min_x(), self.max_x(), policy) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        if !x_inside {
            return Classification::Decided(false);
        }

        let Some(y_inside) = real_between(point.y(), self.min_y(), self.max_y(), policy) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        Classification::Decided(y_inside)
    }

    /// Classifies whether two closed boxes overlap.
    ///
    /// Edge and corner contacts count as overlap. This inclusive convention is
    /// necessary for tangent, endpoint, and shared-boundary curve topology.
    pub fn overlaps(&self, other: &Self, policy: &CurvePolicy) -> Classification<bool> {
        let Some(self_left_of_other) = real_less(self.max_x(), other.min_x(), policy) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(other_left_of_self) = real_less(other.max_x(), self.min_x(), policy) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(self_below_other) = real_less(self.max_y(), other.min_y(), policy) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(other_below_self) = real_less(other.max_y(), self.min_y(), policy) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };

        Classification::Decided(
            !(self_left_of_other || other_left_of_self || self_below_other || other_below_self),
        )
    }

    /// Returns the sole point in the intersection of two closed boxes.
    ///
    /// `None` means the boxes are disjoint or their intersection has positive
    /// extent in at least one axis. Uncertain coordinate order remains explicit.
    pub fn singleton_intersection(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Classification<Option<Point2>> {
        let x = match singleton_interval_intersection(
            self.min_x(),
            self.max_x(),
            other.min_x(),
            other.max_x(),
            policy,
        ) {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let y = match singleton_interval_intersection(
            self.min_y(),
            self.max_y(),
            other.min_y(),
            other.max_y(),
            policy,
        ) {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        Classification::Decided(x.zip(y).map(|(x, y)| Point2::new(x, y)))
    }
}

fn singleton_interval_intersection(
    first_min: &Real,
    first_max: &Real,
    second_min: &Real,
    second_max: &Real,
    policy: &CurvePolicy,
) -> Classification<Option<Real>> {
    let lower = match compare_reals(first_min, second_min, policy) {
        Some(Ordering::Less) => second_min,
        Some(Ordering::Equal | Ordering::Greater) => first_min,
        None => return Classification::Uncertain(UncertaintyReason::Ordering),
    };
    let upper = match compare_reals(first_max, second_max, policy) {
        Some(Ordering::Less | Ordering::Equal) => first_max,
        Some(Ordering::Greater) => second_max,
        None => return Classification::Uncertain(UncertaintyReason::Ordering),
    };
    match compare_reals(lower, upper, policy) {
        Some(Ordering::Equal) => Classification::Decided(Some(lower.clone())),
        Some(Ordering::Less | Ordering::Greater) => Classification::Decided(None),
        None => Classification::Uncertain(UncertaintyReason::Ordering),
    }
}

pub(crate) fn decided_segment_aabb(segment: &Segment2, policy: &CurvePolicy) -> Option<Aabb2> {
    match Aabb2::from_segment(segment, policy) {
        Ok(Classification::Decided(bbox)) => Some(bbox),
        Ok(Classification::Uncertain(_)) | Err(_) => None,
    }
}

pub(crate) fn decided_contour_aabb(contour: &Contour2, policy: &CurvePolicy) -> Option<Aabb2> {
    match Aabb2::from_contour(contour, policy) {
        Ok(Classification::Decided(bbox)) => Some(bbox),
        Ok(Classification::Uncertain(_)) | Err(_) => None,
    }
}

pub(crate) fn aabbs_decided_disjoint(first: &Aabb2, second: &Aabb2, policy: &CurvePolicy) -> bool {
    matches!(
        first.overlaps(second, policy),
        Classification::Decided(false)
    )
}

pub(crate) fn aabb_decided_misses_point(
    bbox: &Aabb2,
    point: &Point2,
    policy: &CurvePolicy,
) -> bool {
    matches!(
        bbox.contains_point(point, policy),
        Classification::Decided(false)
    )
}

fn include_coordinate(
    min: &mut Real,
    max: &mut Real,
    value: &Real,
    policy: &CurvePolicy,
) -> Option<()> {
    if matches!(policy.numeric_mode, crate::NumericMode::EdgePreview)
        && let (Some(value_approx), Some(min_approx), Some(max_approx)) =
            (value.to_f64_lossy(), min.to_f64_lossy(), max.to_f64_lossy())
        && value_approx.is_finite()
        && min_approx.is_finite()
        && max_approx.is_finite()
    {
        if value_approx < min_approx {
            *min = value.clone();
        }
        if value_approx > max_approx {
            *max = value.clone();
        }
        return Some(());
    }

    if matches!(compare_reals(value, min, policy)?, Ordering::Less) {
        *min = value.clone();
    }
    if matches!(compare_reals(value, max, policy)?, Ordering::Greater) {
        *max = value.clone();
    }
    Some(())
}

fn real_less(left: &Real, right: &Real, policy: &CurvePolicy) -> Option<bool> {
    if matches!(policy.numeric_mode, crate::NumericMode::EdgePreview)
        && let (Some(left), Some(right)) = (left.to_f64_lossy(), right.to_f64_lossy())
        && left.is_finite()
        && right.is_finite()
    {
        return Some(left < right - edge_preview_tolerance(policy));
    }

    Some(matches!(
        compare_reals(left, right, policy)?,
        Ordering::Less
    ))
}

fn real_between(value: &Real, min: &Real, max: &Real, policy: &CurvePolicy) -> Option<bool> {
    if matches!(policy.numeric_mode, crate::NumericMode::EdgePreview)
        && let (Some(value), Some(min), Some(max)) =
            (value.to_f64_lossy(), min.to_f64_lossy(), max.to_f64_lossy())
        && value.is_finite()
        && min.is_finite()
        && max.is_finite()
    {
        let tolerance = edge_preview_tolerance(policy);
        return Some(value >= min - tolerance && value <= max + tolerance);
    }

    let lower = compare_reals(value, min, policy)?;
    let upper = compare_reals(value, max, policy)?;
    Some(!matches!(lower, Ordering::Less) && !matches!(upper, Ordering::Greater))
}

fn edge_preview_tolerance(policy: &CurvePolicy) -> f64 {
    policy
        .tolerance
        .map(|tolerance| tolerance.absolute.max(tolerance.relative))
        .unwrap_or(1e-12)
}
