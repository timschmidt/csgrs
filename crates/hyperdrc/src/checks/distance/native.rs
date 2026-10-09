//! Certified boundary queries over the regularized native PCB region.

use std::cmp::Ordering;

use hypercurve::{Aabb2, Classification, Point2, Segment2};

use crate::{PREDICATE_POLICY, PcbGeometryUncertainty, PcbRegion, Scalar};

type MetricResult<T> = Result<T, PcbGeometryUncertainty>;

/// An immutable prepared boundary, including material and hole contours.
///
/// Segments come from the authoritative regularized region. Optional outward
/// finite bounds only schedule candidates; absent bounds keep a segment in
/// every query. Neither a report ring nor a failed metric becomes a proof.
pub(in crate::checks) struct PreparedRegionBoundary {
    segments: Vec<NativeSegment>,
}

struct NativeSegment {
    segment: Segment2,
    bounds: Option<Aabb2>,
    finite: Option<[f64; 4]>,
}

impl PreparedRegionBoundary {
    pub(in crate::checks) fn new(region: &PcbRegion) -> MetricResult<Self> {
        region.evaluate_native("boundary-preparation", |native| {
            if native.is_empty() {
                return Ok(Self { segments: vec![] });
            }
            let native = native
                .native_contours_fast_path()
                .map_err(|error| failure("boundary-preparation", error))?
                .ok_or_else(|| {
                    failure(
                        "boundary-preparation",
                        "region has no exact line/arc boundary",
                    )
                })?;
            let mut segments = native
                .material_contours()
                .iter()
                .chain(native.hole_contours())
                .flat_map(|contour| contour.segments())
                .map(|segment| {
                    let bounds = match Aabb2::from_segment(segment) {
                        Ok(Classification::Decided(bounds)) => Some(bounds),
                        _ => None,
                    };
                    let finite = bounds.as_ref().and_then(finite_bounds);
                    NativeSegment {
                        segment: segment.clone(),
                        bounds,
                        finite,
                    }
                })
                .collect::<Vec<_>>();
            segments.sort_by(|left, right| match (&left.finite, &right.finite) {
                (None, None) => Ordering::Equal,
                (None, Some(_)) => Ordering::Less,
                (Some(_), None) => Ordering::Greater,
                (Some(left), Some(right)) => left[0].total_cmp(&right[0]),
            });
            Ok(Self { segments })
        })
    }

    pub(in crate::checks) fn distance_to(&self, other: &Self) -> MetricResult<Option<Scalar>> {
        evaluate("boundary-distance", || {
            let (outer, indexed) = if self.segments.len() <= other.segments.len() {
                (&self.segments, other)
            } else {
                (&other.segments, self)
            };
            let (Some(left), Some(right)) = (outer.first(), indexed.segments.first()) else {
                return Ok(None);
            };
            let mut minimum = Some(
                left.segment
                    .distance_squared(&right.segment)
                    .map_err(|error| failure("boundary-distance", error))?,
            );
            for left in outer {
                let radius = minimum.as_ref().and_then(|squared| {
                    squared
                        .clone()
                        .sqrt()
                        .ok()
                        .and_then(|value| finite_enclosure(&value).map(|b| b[1]))
                });
                for right in indexed.candidates(left.finite.as_ref(), radius) {
                    if let Some(current) = &minimum
                        && lower_bound_squared(left.bounds.as_ref(), right.bounds.as_ref())
                            .is_some_and(|lower| {
                                crate::scalar::compare(&lower, current)
                                    .is_some_and(|order| !order.is_lt())
                            })
                    {
                        continue;
                    }
                    let squared = left
                        .segment
                        .distance_squared(&right.segment)
                        .map_err(|error| failure("boundary-distance", error))?;
                    if minimum.as_ref().map_or(Ok(true), |current| {
                        compare(&squared, current, "boundary-distance").map(|order| order.is_lt())
                    })? {
                        if compare(&squared, &Scalar::zero(), "boundary-distance")?
                            == Ordering::Equal
                        {
                            return Ok(Some(Scalar::zero()));
                        }
                        minimum = Some(squared);
                    }
                }
            }
            minimum
                .map(|squared| {
                    squared
                        .sqrt()
                        .map_err(|error| failure("boundary-distance", error))
                })
                .transpose()
        })
    }

    pub(in crate::checks) fn within(&self, other: &Self, threshold: &Scalar) -> MetricResult<bool> {
        evaluate("boundary-clearance", || {
            if compare(threshold, &Scalar::zero(), "boundary-clearance")?.is_lt() {
                return Ok(false);
            }
            let threshold_squared = threshold * threshold;
            let radius = finite_enclosure(threshold).map(|bounds| bounds[1]);
            for left in &self.segments {
                for right in other.candidates(left.finite.as_ref(), radius) {
                    if lower_bound_squared(left.bounds.as_ref(), right.bounds.as_ref()).is_some_and(
                        |lower| {
                            crate::scalar::compare(&lower, &threshold_squared)
                                == Some(Ordering::Greater)
                        },
                    ) {
                        continue;
                    }
                    let squared = left
                        .segment
                        .distance_squared(&right.segment)
                        .map_err(|error| failure("boundary-clearance", error))?;
                    if !compare(&squared, &threshold_squared, "boundary-clearance")?.is_gt() {
                        return Ok(true);
                    }
                }
            }
            Ok(false)
        })
    }

    pub(in crate::checks) fn point_within(
        &self,
        point: &[Scalar; 2],
        threshold: &Scalar,
    ) -> MetricResult<bool> {
        evaluate("point-boundary-clearance", || {
            if compare(threshold, &Scalar::zero(), "point-boundary-clearance")?.is_lt() {
                return Ok(false);
            }
            let point = Point2::new(point[0].clone(), point[1].clone());
            let bounds = Aabb2::from_point(point.clone());
            let finite = finite_bounds(&bounds);
            let threshold_squared = threshold * threshold;
            let radius = finite_enclosure(threshold).map(|bounds| bounds[1]);
            for segment in self.candidates(finite.as_ref(), radius) {
                if lower_bound_squared(Some(&bounds), segment.bounds.as_ref()).is_some_and(
                    |lower| {
                        crate::scalar::compare(&lower, &threshold_squared)
                            == Some(Ordering::Greater)
                    },
                ) {
                    continue;
                }
                let squared = segment
                    .segment
                    .point_distance_squared(&point)
                    .map_err(|error| failure("point-boundary-clearance", error))?;
                if !compare(&squared, &threshold_squared, "point-boundary-clearance")?.is_gt() {
                    return Ok(true);
                }
            }
            Ok(false)
        })
    }

    fn candidates<'a>(
        &'a self,
        query: Option<&'a [f64; 4]>,
        radius: Option<f64>,
    ) -> impl Iterator<Item = &'a NativeSegment> {
        let query = query
            .zip(radius)
            .filter(|(_, radius)| radius.is_finite() && *radius >= 0.0);
        let upper = query.map_or(self.segments.len(), |(bounds, radius)| {
            let maximum_x = (bounds[2] + radius).next_up();
            self.segments.partition_point(|segment| {
                segment
                    .finite
                    .as_ref()
                    .is_none_or(|finite| finite[0] <= maximum_x)
            })
        });
        self.segments[..upper].iter().filter(move |segment| {
            let Some(((bounds, radius), other)) = query.zip(segment.finite.as_ref()) else {
                return true;
            };
            !axis_gap_exceeds(bounds[0], bounds[2], other[0], other[2], radius)
                && !axis_gap_exceeds(bounds[1], bounds[3], other[1], other[3], radius)
        })
    }
}

pub(in crate::checks) fn region_boundary_distance(
    left: &PcbRegion,
    right: &PcbRegion,
) -> MetricResult<Option<Scalar>> {
    PreparedRegionBoundary::new(left)?.distance_to(&PreparedRegionBoundary::new(right)?)
}

pub(in crate::checks) fn region_boundaries_within(
    left: &PcbRegion,
    right: &PcbRegion,
    threshold: &Scalar,
) -> MetricResult<bool> {
    PreparedRegionBoundary::new(left)?.within(&PreparedRegionBoundary::new(right)?, threshold)
}

pub(in crate::checks) fn point_region_boundary_within(
    point: &[Scalar; 2],
    region: &PcbRegion,
    threshold: &Scalar,
) -> MetricResult<bool> {
    PreparedRegionBoundary::new(region)?.point_within(point, threshold)
}

/// Compare a measured distance or rule threshold without dropping an unresolved
/// decision or borrowing the caller's preview policy.
pub(in crate::checks) fn compare_metrics(left: &Scalar, right: &Scalar) -> MetricResult<Ordering> {
    evaluate("boundary-metric-comparison", || {
        compare(left, right, "boundary-metric-comparison")
    })
}

/// Select the nearest measured candidate, retaining failures from every query
/// and comparison. Only a successfully empty boundary has no candidate.
pub(in crate::checks) fn minimum_metric<T>(
    candidates: impl IntoIterator<Item = MetricResult<Option<(T, Scalar)>>>,
) -> MetricResult<Option<(T, Scalar)>> {
    evaluate("boundary-metric-minimum", || {
        let mut minimum: Option<(T, Scalar)> = None;
        for candidate in candidates {
            let Some(candidate) = candidate? else {
                continue;
            };
            if minimum.as_ref().map_or(Ok(true), |current| {
                compare(&candidate.1, &current.1, "boundary-metric-minimum")
                    .map(|order| order.is_lt())
            })? {
                minimum = Some(candidate);
            }
        }
        Ok(minimum)
    })
}

fn failure(operation: &str, detail: impl std::fmt::Display) -> PcbGeometryUncertainty {
    PcbGeometryUncertainty {
        operation: operation.to_owned(),
        source: None,
        detail: detail.to_string(),
    }
}

fn evaluate<T>(operation: &str, body: impl FnOnce() -> MetricResult<T>) -> MetricResult<T> {
    check_budget(operation)?;
    let result = hypercurve::evaluate_under(PREDICATE_POLICY, body);
    check_budget(operation)?;
    result
        .certified()
        .ok_or_else(|| failure(operation, "native metric lacks a STRICT receipt"))?
}

fn check_budget(operation: &str) -> MetricResult<()> {
    match hypersolve::work_budget::exhaustion() {
        Some(exhaustion) => Err(failure(
            operation,
            format!("work budget exhausted: {exhaustion:?}"),
        )),
        None => Ok(()),
    }
}

fn compare(left: &Scalar, right: &Scalar, operation: &str) -> MetricResult<Ordering> {
    check_budget(operation)?;
    let value = crate::scalar::compare(left, right);
    check_budget(operation)?;
    value.ok_or_else(|| failure(operation, "native metric comparison is unresolved"))
}

pub(in crate::checks) use crate::projection::finite_enclosure;

fn finite_bounds(bounds: &Aabb2) -> Option<[f64; 4]> {
    Some([
        finite_enclosure(bounds.min_x())?[0],
        finite_enclosure(bounds.min_y())?[0],
        finite_enclosure(bounds.max_x())?[1],
        finite_enclosure(bounds.max_y())?[1],
    ])
}

fn lower_bound_squared(left: Option<&Aabb2>, right: Option<&Aabb2>) -> Option<Scalar> {
    let (left, right) = (left?, right?);
    let gap = |left_min: &Scalar, left_max: &Scalar, right_min: &Scalar, right_max: &Scalar| {
        if crate::scalar::compare(left_max, right_min) == Some(Ordering::Less) {
            right_min - left_max
        } else if crate::scalar::compare(right_max, left_min) == Some(Ordering::Less) {
            left_min - right_max
        } else {
            Scalar::zero()
        }
    };
    let dx = gap(left.min_x(), left.max_x(), right.min_x(), right.max_x());
    let dy = gap(left.min_y(), left.max_y(), right.min_y(), right.max_y());
    Some(&dx * &dx + &dy * &dy)
}

fn axis_gap_exceeds(
    left_min: f64,
    left_max: f64,
    right_min: f64,
    right_max: f64,
    radius: f64,
) -> bool {
    let gap = if left_max < right_min {
        right_min - left_max
    } else if right_max < left_min {
        left_min - right_max
    } else {
        0.0
    };
    gap.next_down() > radius
}
