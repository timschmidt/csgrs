//! Intersection result types and early line-line topology.

use std::cmp::Ordering;

use hyperlattice::{Backend, DefaultBackend, Scalar};

use crate::classify::{
    at_unit_interval_endpoint, compare_scalars, in_closed_unit_interval, is_zero, max_scalar,
    min_scalar, sort_pair,
};
use crate::{CurvePolicy, CurveResult, LineSeg2, Point2, UncertaintyReason};

/// Parameter range on a segment.
#[derive(Clone, Debug, PartialEq)]
pub struct ParamRange<B: Backend = DefaultBackend> {
    start: Scalar<B>,
    end: Scalar<B>,
}

impl<B: Backend> ParamRange<B> {
    /// Constructs a parameter range.
    pub const fn new(start: Scalar<B>, end: Scalar<B>) -> Self {
        Self { start, end }
    }

    /// Range start.
    pub const fn start(&self) -> &Scalar<B> {
        &self.start
    }

    /// Range end.
    pub const fn end(&self) -> &Scalar<B> {
        &self.end
    }
}

/// Local intersection kind.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IntersectionKind {
    /// Proper crossing at both segment interiors.
    Crossing,
    /// Contact at one or more segment endpoints.
    Endpoint,
    /// Collinear overlap.
    Overlap,
}

/// Intersection between two line segments.
#[derive(Clone, Debug, PartialEq)]
pub enum LineLineIntersection<B: Backend = DefaultBackend> {
    /// No intersection.
    None,
    /// A single intersection point.
    Point {
        /// Intersection point.
        point: Point2<B>,
        /// Parameter on the first segment.
        a_param: Scalar<B>,
        /// Parameter on the second segment.
        b_param: Scalar<B>,
        /// Local kind of point contact.
        kind: IntersectionKind,
    },
    /// A collinear overlapping interval.
    Overlap {
        /// Overlapping segment geometry.
        segment: LineSeg2<B>,
        /// Parameter range on the first segment.
        a_range: ParamRange<B>,
        /// Parameter range on the second segment.
        b_range: ParamRange<B>,
    },
    /// The active policy could not classify this relation.
    Uncertain {
        /// Why classification stopped.
        reason: UncertaintyReason,
    },
}

impl<B: Backend> LineSeg2<B> {
    /// Intersects this line segment with another line segment.
    pub fn intersect_line(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<LineLineIntersection<B>> {
        let (rx, ry) = self.delta();
        let (sx, sy) = other.delta();
        let qmp = other.start().delta_from(self.start());

        let denominator = cross(&rx, &ry, &sx, &sy);
        match is_zero(&denominator, policy) {
            Some(false) => intersect_non_parallel(self, policy, &rx, &ry, &sx, &sy, qmp),
            Some(true) => intersect_parallel(self, other, policy, &rx, &ry, qmp),
            None => Ok(LineLineIntersection::Uncertain {
                reason: UncertaintyReason::ScalarSign,
            }),
        }
    }
}

fn intersect_non_parallel<B: Backend>(
    a: &LineSeg2<B>,
    policy: &CurvePolicy,
    rx: &Scalar<B>,
    ry: &Scalar<B>,
    sx: &Scalar<B>,
    sy: &Scalar<B>,
    qmp: (Scalar<B>, Scalar<B>),
) -> CurveResult<LineLineIntersection<B>> {
    let denominator = cross(rx, ry, sx, sy);
    let t_numerator = cross(&qmp.0, &qmp.1, sx, sy);
    let u_numerator = cross(&qmp.0, &qmp.1, rx, ry);
    let t = (t_numerator / &denominator)?;
    let u = (u_numerator / &denominator)?;

    let Some(t_in_range) = in_closed_unit_interval(&t, policy) else {
        return Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::Ordering,
        });
    };
    let Some(u_in_range) = in_closed_unit_interval(&u, policy) else {
        return Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::Ordering,
        });
    };

    if !(t_in_range && u_in_range) {
        return Ok(LineLineIntersection::None);
    }

    let t_endpoint = at_unit_interval_endpoint(&t, policy).ok_or_else(|| {
        crate::CurveError::Scalar("could not classify first line parameter endpoint".into())
    })?;
    let u_endpoint = at_unit_interval_endpoint(&u, policy).ok_or_else(|| {
        crate::CurveError::Scalar("could not classify second line parameter endpoint".into())
    })?;
    let kind = if t_endpoint || u_endpoint {
        IntersectionKind::Endpoint
    } else {
        IntersectionKind::Crossing
    };

    Ok(LineLineIntersection::Point {
        point: a.point_at(t.clone()),
        a_param: t,
        b_param: u,
        kind,
    })
}

fn intersect_parallel<B: Backend>(
    a: &LineSeg2<B>,
    b: &LineSeg2<B>,
    policy: &CurvePolicy,
    rx: &Scalar<B>,
    ry: &Scalar<B>,
    qmp: (Scalar<B>, Scalar<B>),
) -> CurveResult<LineLineIntersection<B>> {
    let collinear_test = cross(&qmp.0, &qmp.1, rx, ry);
    match is_zero(&collinear_test, policy) {
        Some(false) => Ok(LineLineIntersection::None),
        Some(true) => intersect_collinear(a, b, policy),
        None => Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::ScalarSign,
        }),
    }
}

fn intersect_collinear<B: Backend>(
    a: &LineSeg2<B>,
    b: &LineSeg2<B>,
    policy: &CurvePolicy,
) -> CurveResult<LineLineIntersection<B>> {
    let t0 = parameter_on_line(a, b.start(), policy)?;
    let t1 = parameter_on_line(a, b.end(), policy)?;
    let Some((t_min, t_max)) = sort_pair(t0, t1, policy) else {
        return Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::Ordering,
        });
    };

    let overlap_start = max_scalar(t_min, Scalar::<B>::zero(), policy);
    let overlap_end = min_scalar(t_max, Scalar::<B>::one(), policy);
    let (Some(overlap_start), Some(overlap_end)) = (overlap_start, overlap_end) else {
        return Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::Ordering,
        });
    };

    match compare_scalars(&overlap_start, &overlap_end, policy) {
        Some(Ordering::Greater) => Ok(LineLineIntersection::None),
        Some(Ordering::Equal) => {
            let point = a.point_at(overlap_start.clone());
            let b_param = parameter_on_line(b, &point, policy)?;
            Ok(LineLineIntersection::Point {
                point,
                a_param: overlap_start,
                b_param,
                kind: IntersectionKind::Endpoint,
            })
        }
        Some(Ordering::Less) => {
            let start_point = a.point_at(overlap_start.clone());
            let end_point = a.point_at(overlap_end.clone());
            let b_start = parameter_on_line(b, &start_point, policy)?;
            let b_end = parameter_on_line(b, &end_point, policy)?;
            let segment = LineSeg2::try_new(start_point, end_point)?;
            Ok(LineLineIntersection::Overlap {
                segment,
                a_range: ParamRange::new(overlap_start, overlap_end),
                b_range: ParamRange::new(b_start, b_end),
            })
        }
        None => Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::Ordering,
        }),
    }
}

fn parameter_on_line<B: Backend>(
    line: &LineSeg2<B>,
    point: &Point2<B>,
    policy: &CurvePolicy,
) -> CurveResult<Scalar<B>> {
    let (dx, dy) = line.delta();
    let delta = point.delta_from(line.start());

    match is_zero(&dx, policy) {
        Some(false) => (delta.0 / dx).map_err(Into::into),
        Some(true) => (delta.1 / dy).map_err(Into::into),
        None => match is_zero(&dy, policy) {
            Some(false) => (delta.1 / dy).map_err(Into::into),
            _ => Err(crate::CurveError::Scalar(
                "could not choose nonzero line component".into(),
            )),
        },
    }
}

fn cross<B: Backend>(ax: &Scalar<B>, ay: &Scalar<B>, bx: &Scalar<B>, by: &Scalar<B>) -> Scalar<B> {
    (ax * by) - (ay * bx)
}
