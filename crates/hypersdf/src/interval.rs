//! Certified interval reports for retained SDF expressions.
//!
//! Intervals here are exact scalar ranges over a closed AABB query. They are
//! not primitive-float samples. For sphere primitives the range is over the
//! sign-equivalent squared-distance-minus-radius expression, preserving the
//! square-root-free decision route used by `hyperlimit`. Arithmetic nodes use
//! classical outward-rounded interval arithmetic formulas, but over exact
//! `Real` endpoints; see Moore, *Interval Analysis* (1966). Monotone `sqrt`
//! intervals are certified only when the whole child interval is nonnegative.
//! The resulting interval is only accepted as predicate evidence when the sign
//! decision is certified, following Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997).

use core::cmp::Ordering;

use hyperlimit::{Escalation, Point3, PredicateOutcome, RefinementNeed, compare_reals};
use hyperreal::Real;

use crate::expr::{SdfCoordinate, SdfExpr};
use crate::primitive::{
    SdfPrimitive, capsule_domain, cylinder_domain, half_width_domain, radial_squared,
    radius_squared_domain, squared_distance3, torus_domain,
};
use crate::status::{SdfEvidenceStatus, SdfFreshness, SdfMetricStatus};

/// Certified scalar interval over a query domain.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfInterval {
    /// Inclusive lower bound.
    pub lower: Real,
    /// Inclusive upper bound.
    pub upper: Real,
    /// Metric meaning of the bounded scalar.
    pub metric_status: SdfMetricStatus,
}

/// Report returned by interval evaluation over a closed AABB.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfIntervalReport {
    /// Minimum corner of the closed query cell.
    pub min: Point3,
    /// Maximum corner of the closed query cell.
    pub max: Point3,
    /// Certified interval, or `None` when unsupported or undecided.
    pub interval: Option<SdfInterval>,
    /// Evidence for the interval decision.
    pub evidence: SdfEvidenceStatus,
    /// Prepared-source freshness.
    pub freshness: SdfFreshness,
}

impl SdfIntervalReport {
    /// Returns whether the report contains a certified interval.
    pub const fn is_certified(&self) -> bool {
        self.interval.is_some() && self.evidence.is_certified()
    }
}

pub(crate) fn interval_expr_cell(
    expr: &SdfExpr,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<SdfInterval> {
    match expr {
        SdfExpr::Constant(value) => constant_interval(value),
        SdfExpr::Coordinate(axis) => coordinate_interval(*axis, min, max),
        SdfExpr::Linear {
            coefficients,
            offset,
        } => linear_interval(coefficients, offset, min, max),
        SdfExpr::Primitive(primitive) => interval_primitive_cell(primitive, min, max),
        SdfExpr::Union(left, right) => combine_interval_pair(
            interval_expr_cell(left, min, max),
            interval_expr_cell(right, min, max),
            interval_min,
        ),
        SdfExpr::Intersection(left, right) => combine_interval_pair(
            interval_expr_cell(left, min, max),
            interval_expr_cell(right, min, max),
            interval_max,
        ),
        SdfExpr::Add(left, right) => combine_interval_pair(
            interval_expr_cell(left, min, max),
            interval_expr_cell(right, min, max),
            interval_add,
        ),
        SdfExpr::Sub(left, right) => combine_interval_pair(
            interval_expr_cell(left, min, max),
            interval_expr_cell(right, min, max),
            interval_sub,
        ),
        SdfExpr::Mul(left, right) => combine_interval_pair(
            interval_expr_cell(left, min, max),
            interval_expr_cell(right, min, max),
            interval_mul,
        ),
        SdfExpr::Abs(inner) => map_interval_abs(interval_expr_cell(inner, min, max)),
        SdfExpr::Sqrt(inner) => map_interval_sqrt(interval_expr_cell(inner, min, max)),
        SdfExpr::Complement(inner) => map_interval_complement(interval_expr_cell(inner, min, max)),
        SdfExpr::Offset { child, amount } => {
            map_interval_offset(interval_expr_cell(child, min, max), amount)
        }
        SdfExpr::Transform { child, transform } => {
            let Some((child_min, child_max)) = transform.inverse_aabb(min, max) else {
                return crate::transform::unsupported_transform_outcome();
            };
            interval_expr_cell(child, &child_min, &child_max)
        }
    }
}

fn constant_interval(value: &Real) -> PredicateOutcome<SdfInterval> {
    PredicateOutcome::decided(
        SdfInterval {
            lower: value.clone(),
            upper: value.clone(),
            metric_status: SdfMetricStatus::SignEquivalent,
        },
        hyperlimit::Certainty::Exact,
        Escalation::Exact,
    )
}

fn coordinate_interval(
    axis: SdfCoordinate,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<SdfInterval> {
    let (a, b) = match axis {
        SdfCoordinate::X => (&min.x, &max.x),
        SdfCoordinate::Y => (&min.y, &max.y),
        SdfCoordinate::Z => (&min.z, &max.z),
    };
    let (lower, upper) = match ordered_pair(a, b) {
        Ok(pair) => pair,
        Err(outcome) => return outcome,
    };
    PredicateOutcome::decided(
        SdfInterval {
            lower,
            upper,
            metric_status: SdfMetricStatus::SignEquivalent,
        },
        hyperlimit::Certainty::Exact,
        Escalation::Exact,
    )
}

fn linear_interval(
    coefficients: &hyperlattice::Vector3,
    offset: &Real,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<SdfInterval> {
    let corners = corners(min, max);
    let mut lower = linear_value(coefficients, offset, &corners[0]);
    let mut upper = lower.clone();
    for corner in &corners[1..] {
        let value = linear_value(coefficients, offset, corner);
        lower = match min_real(lower, value.clone()) {
            Ok(value) => value,
            Err(outcome) => return outcome,
        };
        upper = match max_real(upper, value) {
            Ok(value) => value,
            Err(outcome) => return outcome,
        };
    }
    PredicateOutcome::decided(
        SdfInterval {
            lower,
            upper,
            metric_status: SdfMetricStatus::SignEquivalent,
        },
        hyperlimit::Certainty::Exact,
        Escalation::Exact,
    )
}

fn interval_primitive_cell(
    primitive: &SdfPrimitive,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<SdfInterval> {
    match primitive {
        SdfPrimitive::Plane { plane } => plane_interval(plane, min, max),
        SdfPrimitive::Sphere {
            center,
            radius_squared,
        } => match radius_squared_domain(radius_squared) {
            PredicateOutcome::Decided { value: true, .. } => {
                sphere_interval(center, radius_squared, min, max)
            }
            PredicateOutcome::Decided { value: false, .. } => {
                PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided)
            }
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        },
        SdfPrimitive::Aabb {
            min: shape_min,
            max: shape_max,
        } => aabb_interval(shape_min, shape_max, min, max),
        SdfPrimitive::Cylinder {
            axis,
            center,
            radius_squared,
            half_height,
        } => match cylinder_domain(radius_squared, half_height) {
            PredicateOutcome::Decided { value: true, .. } => {
                cylinder_interval(*axis, center, radius_squared, half_height, min, max)
            }
            PredicateOutcome::Decided { value: false, .. } => {
                PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided)
            }
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        },
        SdfPrimitive::Capsule {
            axis,
            center,
            radius_squared,
            half_length,
        } => match capsule_domain(radius_squared, half_length) {
            PredicateOutcome::Decided { value: true, .. } => {
                capsule_interval(*axis, center, radius_squared, half_length, min, max)
            }
            PredicateOutcome::Decided { value: false, .. } => {
                PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided)
            }
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        },
        SdfPrimitive::Torus {
            axis,
            center,
            major_radius_squared,
            minor_radius_squared,
        } => match torus_domain(major_radius_squared, minor_radius_squared) {
            PredicateOutcome::Decided { value: true, .. } => torus_interval(
                *axis,
                center,
                major_radius_squared,
                minor_radius_squared,
                min,
                max,
            ),
            PredicateOutcome::Decided { value: false, .. } => {
                PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided)
            }
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        },
        SdfPrimitive::Slab { plane, half_width } => match half_width_domain(half_width) {
            PredicateOutcome::Decided { value: true, .. } => {
                slab_interval(plane, half_width, min, max)
            }
            PredicateOutcome::Decided { value: false, .. } => {
                PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided)
            }
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        },
    }
}

fn plane_interval(
    plane: &hyperlimit::Plane3,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<SdfInterval> {
    let corners = corners(min, max);
    let mut lower = plane_value(plane, &corners[0]);
    let mut upper = lower.clone();
    for corner in &corners[1..] {
        let value = plane_value(plane, corner);
        lower = match min_real(lower, value.clone()) {
            Ok(value) => value,
            Err(outcome) => return outcome,
        };
        upper = match max_real(upper, value) {
            Ok(value) => value,
            Err(outcome) => return outcome,
        };
    }
    PredicateOutcome::decided(
        SdfInterval {
            lower,
            upper,
            metric_status: SdfMetricStatus::SignEquivalent,
        },
        hyperlimit::Certainty::Exact,
        Escalation::Exact,
    )
}

fn sphere_interval(
    center: &Point3,
    radius_squared: &Real,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<SdfInterval> {
    let closest = match closest_point_in_aabb(center, min, max) {
        Ok(point) => point,
        Err(outcome) => return outcome,
    };
    let lower = &squared_distance3(center, &closest) - radius_squared;

    let corners = corners(min, max);
    let mut farthest = squared_distance3(center, &corners[0]);
    for corner in &corners[1..] {
        let distance = squared_distance3(center, corner);
        farthest = match max_real(farthest, distance) {
            Ok(value) => value,
            Err(outcome) => return outcome,
        };
    }
    let upper = &farthest - radius_squared;
    PredicateOutcome::decided(
        SdfInterval {
            lower,
            upper,
            metric_status: SdfMetricStatus::SignEquivalent,
        },
        hyperlimit::Certainty::Exact,
        Escalation::Exact,
    )
}

fn aabb_interval(
    shape_min: &Point3,
    shape_max: &Point3,
    cell_min: &Point3,
    cell_max: &Point3,
) -> PredicateOutcome<SdfInterval> {
    let intervals = [
        (&shape_min.x - &cell_max.x, &shape_min.x - &cell_min.x),
        (&cell_min.x - &shape_max.x, &cell_max.x - &shape_max.x),
        (&shape_min.y - &cell_max.y, &shape_min.y - &cell_min.y),
        (&cell_min.y - &shape_max.y, &cell_max.y - &shape_max.y),
        (&shape_min.z - &cell_max.z, &shape_min.z - &cell_min.z),
        (&cell_min.z - &shape_max.z, &cell_max.z - &shape_max.z),
    ];
    let mut lower = intervals[0].0.clone();
    let mut upper = intervals[0].1.clone();
    for (candidate_lower, candidate_upper) in intervals.iter().skip(1) {
        lower = match max_real(lower, candidate_lower.clone()) {
            Ok(value) => value,
            Err(outcome) => return outcome,
        };
        upper = match max_real(upper, candidate_upper.clone()) {
            Ok(value) => value,
            Err(outcome) => return outcome,
        };
    }
    PredicateOutcome::decided(
        SdfInterval {
            lower,
            upper,
            metric_status: SdfMetricStatus::SignEquivalent,
        },
        hyperlimit::Certainty::Exact,
        Escalation::Exact,
    )
}

fn slab_interval(
    plane: &hyperlimit::Plane3,
    half_width: &Real,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<SdfInterval> {
    match map_interval_abs(plane_interval(plane, min, max)) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(
            SdfInterval {
                lower: &value.lower - half_width,
                upper: &value.upper - half_width,
                metric_status: SdfMetricStatus::SignEquivalent,
            },
            certainty,
            stage,
        ),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn cylinder_interval(
    axis: SdfCoordinate,
    center: &Point3,
    radius_squared_value: &Real,
    half_height: &Real,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<SdfInterval> {
    let radial = match cylinder_radial_interval(axis, center, radius_squared_value, min, max) {
        Ok(interval) => interval,
        Err(outcome) => return outcome,
    };
    let axial = match cylinder_axial_interval(axis, center, half_height, min, max) {
        Ok(interval) => interval,
        Err(outcome) => return outcome,
    };
    match interval_max(radial, axial) {
        Ok(interval) => {
            PredicateOutcome::decided(interval, hyperlimit::Certainty::Exact, Escalation::Exact)
        }
        Err(outcome) => outcome,
    }
}

fn capsule_interval(
    axis: SdfCoordinate,
    center: &Point3,
    radius_squared_value: &Real,
    half_length: &Real,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<SdfInterval> {
    let zero = Real::zero();
    let radial = match cylinder_radial_interval(axis, center, &zero, min, max) {
        Ok(interval) => interval,
        Err(outcome) => return outcome,
    };
    let axial = match capsule_axial_squared_interval(axis, center, half_length, min, max) {
        Ok(interval) => interval,
        Err(outcome) => return outcome,
    };
    let combined = match interval_add(radial, axial) {
        Ok(interval) => interval,
        Err(outcome) => return outcome,
    };
    PredicateOutcome::decided(
        SdfInterval {
            lower: &combined.lower - radius_squared_value,
            upper: &combined.upper - radius_squared_value,
            metric_status: SdfMetricStatus::SignEquivalent,
        },
        hyperlimit::Certainty::Exact,
        Escalation::Exact,
    )
}

fn torus_interval(
    axis: SdfCoordinate,
    center: &Point3,
    major_radius_squared: &Real,
    minor_radius_squared: &Real,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<SdfInterval> {
    let zero = Real::zero();
    let radial = match cylinder_radial_interval(axis, center, &zero, min, max) {
        Ok(interval) => interval,
        Err(outcome) => return outcome,
    };
    let axial = match axis_squared_interval(axis, center, min, max) {
        Ok(interval) => interval,
        Err(outcome) => return outcome,
    };
    let radial_plus_axial = match interval_add(radial.clone(), axial) {
        Ok(interval) => interval,
        Err(outcome) => return outcome,
    };
    let q = SdfInterval {
        lower: &(&radial_plus_axial.lower + major_radius_squared) - minor_radius_squared,
        upper: &(&radial_plus_axial.upper + major_radius_squared) - minor_radius_squared,
        metric_status: SdfMetricStatus::SignEquivalent,
    };
    let q_squared = match square_interval(q) {
        Ok(interval) => interval,
        Err(outcome) => return outcome,
    };
    let four_major = Real::from(4_i32) * major_radius_squared;
    let radial_term = SdfInterval {
        lower: &four_major * &radial.lower,
        upper: &four_major * &radial.upper,
        metric_status: SdfMetricStatus::SignEquivalent,
    };
    match interval_sub(q_squared, radial_term) {
        Ok(interval) => {
            PredicateOutcome::decided(interval, hyperlimit::Certainty::Exact, Escalation::Exact)
        }
        Err(outcome) => outcome,
    }
}

fn cylinder_radial_interval(
    axis: SdfCoordinate,
    center: &Point3,
    radius_squared_value: &Real,
    min: &Point3,
    max: &Point3,
) -> Result<SdfInterval, PredicateOutcome<SdfInterval>> {
    let closest = closest_point_in_radial_aabb(axis, center, min, max)?;
    let lower = &radial_squared(axis, center, &closest) - radius_squared_value;
    let corners = corners(min, max);
    let mut farthest = radial_squared(axis, center, &corners[0]);
    for corner in &corners[1..] {
        farthest = max_real(farthest, radial_squared(axis, center, corner))?;
    }
    Ok(SdfInterval {
        lower,
        upper: &farthest - radius_squared_value,
        metric_status: SdfMetricStatus::SignEquivalent,
    })
}

fn cylinder_axial_interval(
    axis: SdfCoordinate,
    center: &Point3,
    half_height: &Real,
    min: &Point3,
    max: &Point3,
) -> Result<SdfInterval, PredicateOutcome<SdfInterval>> {
    let (axis_min, axis_max, center_axis) = match axis {
        SdfCoordinate::X => (&min.x, &max.x, &center.x),
        SdfCoordinate::Y => (&min.y, &max.y, &center.y),
        SdfCoordinate::Z => (&min.z, &max.z, &center.z),
    };
    let (lower, upper) = ordered_pair(axis_min, axis_max)?;
    let shifted = SdfInterval {
        lower: &lower - center_axis,
        upper: &upper - center_axis,
        metric_status: SdfMetricStatus::SignEquivalent,
    };
    let absolute = abs_interval(shifted)?;
    Ok(SdfInterval {
        lower: &absolute.lower - half_height,
        upper: &absolute.upper - half_height,
        metric_status: SdfMetricStatus::SignEquivalent,
    })
}

fn capsule_axial_squared_interval(
    axis: SdfCoordinate,
    center: &Point3,
    half_length: &Real,
    min: &Point3,
    max: &Point3,
) -> Result<SdfInterval, PredicateOutcome<SdfInterval>> {
    let axial = cylinder_axial_interval(axis, center, half_length, min, max)?;
    let zero = Real::zero();
    let lower = match compare_reals(&axial.lower, &zero) {
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            ..
        } => axial.lower,
        PredicateOutcome::Decided { .. } => zero.clone(),
        PredicateOutcome::Unknown { needed, stage } => {
            return Err(PredicateOutcome::unknown(needed, stage));
        }
    };
    let upper = match compare_reals(&axial.upper, &zero) {
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            ..
        } => axial.upper,
        PredicateOutcome::Decided { .. } => zero,
        PredicateOutcome::Unknown { needed, stage } => {
            return Err(PredicateOutcome::unknown(needed, stage));
        }
    };
    Ok(SdfInterval {
        lower: &lower * &lower,
        upper: &upper * &upper,
        metric_status: SdfMetricStatus::SignEquivalent,
    })
}

fn axis_squared_interval(
    axis: SdfCoordinate,
    center: &Point3,
    min: &Point3,
    max: &Point3,
) -> Result<SdfInterval, PredicateOutcome<SdfInterval>> {
    let (axis_min, axis_max, center_axis) = match axis {
        SdfCoordinate::X => (&min.x, &max.x, &center.x),
        SdfCoordinate::Y => (&min.y, &max.y, &center.y),
        SdfCoordinate::Z => (&min.z, &max.z, &center.z),
    };
    let (lower, upper) = ordered_pair(axis_min, axis_max)?;
    square_interval(SdfInterval {
        lower: &lower - center_axis,
        upper: &upper - center_axis,
        metric_status: SdfMetricStatus::SignEquivalent,
    })
}

fn square_interval(interval: SdfInterval) -> Result<SdfInterval, PredicateOutcome<SdfInterval>> {
    let zero = Real::zero();
    match compare_reals(&interval.lower, &zero) {
        PredicateOutcome::Decided {
            value: Ordering::Greater | Ordering::Equal,
            ..
        } => Ok(SdfInterval {
            lower: &interval.lower * &interval.lower,
            upper: &interval.upper * &interval.upper,
            metric_status: interval.metric_status,
        }),
        PredicateOutcome::Decided { .. } => match compare_reals(&interval.upper, &zero) {
            PredicateOutcome::Decided {
                value: Ordering::Less | Ordering::Equal,
                ..
            } => Ok(SdfInterval {
                lower: &interval.upper * &interval.upper,
                upper: &interval.lower * &interval.lower,
                metric_status: interval.metric_status,
            }),
            PredicateOutcome::Decided { .. } => {
                let neg_lower = -&interval.lower;
                let upper_abs = max_real(neg_lower, interval.upper)?;
                Ok(SdfInterval {
                    lower: zero,
                    upper: &upper_abs * &upper_abs,
                    metric_status: interval.metric_status,
                })
            }
            PredicateOutcome::Unknown { needed, stage } => {
                Err(PredicateOutcome::unknown(needed, stage))
            }
        },
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

fn combine_interval_pair<F>(
    left: PredicateOutcome<SdfInterval>,
    right: PredicateOutcome<SdfInterval>,
    combine: F,
) -> PredicateOutcome<SdfInterval>
where
    F: FnOnce(SdfInterval, SdfInterval) -> Result<SdfInterval, PredicateOutcome<SdfInterval>>,
{
    match (left, right) {
        (
            PredicateOutcome::Decided { value: a, .. },
            PredicateOutcome::Decided { value: b, .. },
        ) => match combine(a, b) {
            Ok(interval) => {
                PredicateOutcome::decided(interval, hyperlimit::Certainty::Exact, Escalation::Exact)
            }
            Err(outcome) => outcome,
        },
        (PredicateOutcome::Unknown { needed, stage }, _)
        | (_, PredicateOutcome::Unknown { needed, stage }) => {
            PredicateOutcome::unknown(needed, stage)
        }
    }
}

fn interval_min(
    left: SdfInterval,
    right: SdfInterval,
) -> Result<SdfInterval, PredicateOutcome<SdfInterval>> {
    Ok(SdfInterval {
        lower: min_real(left.lower, right.lower)?,
        upper: min_real(left.upper, right.upper)?,
        metric_status: left.metric_status.csg_pair(right.metric_status),
    })
}

fn interval_max(
    left: SdfInterval,
    right: SdfInterval,
) -> Result<SdfInterval, PredicateOutcome<SdfInterval>> {
    Ok(SdfInterval {
        lower: max_real(left.lower, right.lower)?,
        upper: max_real(left.upper, right.upper)?,
        metric_status: left.metric_status.csg_pair(right.metric_status),
    })
}

fn interval_add(
    left: SdfInterval,
    right: SdfInterval,
) -> Result<SdfInterval, PredicateOutcome<SdfInterval>> {
    Ok(SdfInterval {
        lower: &left.lower + &right.lower,
        upper: &left.upper + &right.upper,
        metric_status: left.metric_status.csg_pair(right.metric_status),
    })
}

fn interval_sub(
    left: SdfInterval,
    right: SdfInterval,
) -> Result<SdfInterval, PredicateOutcome<SdfInterval>> {
    Ok(SdfInterval {
        lower: &left.lower - &right.upper,
        upper: &left.upper - &right.lower,
        metric_status: left.metric_status.csg_pair(right.metric_status),
    })
}

fn interval_mul(
    left: SdfInterval,
    right: SdfInterval,
) -> Result<SdfInterval, PredicateOutcome<SdfInterval>> {
    let products = [
        &left.lower * &right.lower,
        &left.lower * &right.upper,
        &left.upper * &right.lower,
        &left.upper * &right.upper,
    ];
    let mut lower = products[0].clone();
    let mut upper = products[0].clone();
    for product in products.iter().skip(1) {
        lower = min_real(lower, product.clone())?;
        upper = max_real(upper, product.clone())?;
    }
    Ok(SdfInterval {
        lower,
        upper,
        metric_status: left.metric_status.csg_pair(right.metric_status),
    })
}

fn map_interval_complement(
    outcome: PredicateOutcome<SdfInterval>,
) -> PredicateOutcome<SdfInterval> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(
            SdfInterval {
                lower: -&value.upper,
                upper: -&value.lower,
                metric_status: value.metric_status.complemented(),
            },
            certainty,
            stage,
        ),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn map_interval_abs(outcome: PredicateOutcome<SdfInterval>) -> PredicateOutcome<SdfInterval> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => match abs_interval(value) {
            Ok(interval) => PredicateOutcome::decided(interval, certainty, stage),
            Err(outcome) => outcome,
        },
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn map_interval_sqrt(outcome: PredicateOutcome<SdfInterval>) -> PredicateOutcome<SdfInterval> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => match sqrt_interval(value) {
            Ok(interval) => PredicateOutcome::decided(interval, certainty, stage),
            Err(outcome) => outcome,
        },
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn abs_interval(interval: SdfInterval) -> Result<SdfInterval, PredicateOutcome<SdfInterval>> {
    let zero = Real::zero();
    match compare_reals(&interval.lower, &zero) {
        PredicateOutcome::Decided {
            value: Ordering::Greater | Ordering::Equal,
            ..
        } => Ok(interval),
        PredicateOutcome::Decided { .. } => match compare_reals(&interval.upper, &zero) {
            PredicateOutcome::Decided {
                value: Ordering::Less | Ordering::Equal,
                ..
            } => Ok(SdfInterval {
                lower: -&interval.upper,
                upper: -&interval.lower,
                metric_status: interval.metric_status,
            }),
            PredicateOutcome::Decided { .. } => Ok(SdfInterval {
                lower: zero,
                upper: max_real(-&interval.lower, interval.upper)?,
                metric_status: interval.metric_status,
            }),
            PredicateOutcome::Unknown { needed, stage } => {
                Err(PredicateOutcome::unknown(needed, stage))
            }
        },
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

fn sqrt_interval(interval: SdfInterval) -> Result<SdfInterval, PredicateOutcome<SdfInterval>> {
    let zero = Real::zero();
    match compare_reals(&interval.lower, &zero) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => Err(PredicateOutcome::unknown(
            RefinementNeed::Unsupported,
            Escalation::Undecided,
        )),
        PredicateOutcome::Decided { .. } => {
            let lower = interval.lower.sqrt().map_err(|_| {
                PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided)
            })?;
            let upper = interval.upper.sqrt().map_err(|_| {
                PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided)
            })?;
            Ok(SdfInterval {
                lower,
                upper,
                metric_status: interval.metric_status,
            })
        }
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

fn map_interval_offset(
    outcome: PredicateOutcome<SdfInterval>,
    amount: &Real,
) -> PredicateOutcome<SdfInterval> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(
            SdfInterval {
                lower: &value.lower - amount,
                upper: &value.upper - amount,
                metric_status: value.metric_status,
            },
            certainty,
            stage,
        ),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn closest_point_in_aabb(
    point: &Point3,
    min: &Point3,
    max: &Point3,
) -> Result<Point3, PredicateOutcome<SdfInterval>> {
    Ok(Point3::new(
        clamp_real(&point.x, &min.x, &max.x)?,
        clamp_real(&point.y, &min.y, &max.y)?,
        clamp_real(&point.z, &min.z, &max.z)?,
    ))
}

fn closest_point_in_radial_aabb(
    axis: SdfCoordinate,
    point: &Point3,
    min: &Point3,
    max: &Point3,
) -> Result<Point3, PredicateOutcome<SdfInterval>> {
    let mut closest = point.clone();
    match axis {
        SdfCoordinate::X => {
            closest.y = clamp_real(&point.y, &min.y, &max.y)?;
            closest.z = clamp_real(&point.z, &min.z, &max.z)?;
        }
        SdfCoordinate::Y => {
            closest.x = clamp_real(&point.x, &min.x, &max.x)?;
            closest.z = clamp_real(&point.z, &min.z, &max.z)?;
        }
        SdfCoordinate::Z => {
            closest.x = clamp_real(&point.x, &min.x, &max.x)?;
            closest.y = clamp_real(&point.y, &min.y, &max.y)?;
        }
    }
    Ok(closest)
}

fn clamp_real(value: &Real, a: &Real, b: &Real) -> Result<Real, PredicateOutcome<SdfInterval>> {
    let (lo, hi) = ordered_pair(a, b)?;
    match compare_reals(value, &lo) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => Ok(lo),
        PredicateOutcome::Decided { .. } => match compare_reals(value, &hi) {
            PredicateOutcome::Decided {
                value: Ordering::Greater,
                ..
            } => Ok(hi),
            PredicateOutcome::Decided { .. } => Ok(value.clone()),
            PredicateOutcome::Unknown { needed, stage } => {
                Err(PredicateOutcome::unknown(needed, stage))
            }
        },
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

fn ordered_pair(a: &Real, b: &Real) -> Result<(Real, Real), PredicateOutcome<SdfInterval>> {
    match compare_reals(a, b) {
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            ..
        } => Ok((b.clone(), a.clone())),
        PredicateOutcome::Decided { .. } => Ok((a.clone(), b.clone())),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

fn min_real(left: Real, right: Real) -> Result<Real, PredicateOutcome<SdfInterval>> {
    match compare_reals(&left, &right) {
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            ..
        } => Ok(right),
        PredicateOutcome::Decided { .. } => Ok(left),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

fn max_real(left: Real, right: Real) -> Result<Real, PredicateOutcome<SdfInterval>> {
    match compare_reals(&left, &right) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => Ok(right),
        PredicateOutcome::Decided { .. } => Ok(left),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

fn plane_value(plane: &hyperlimit::Plane3, point: &Point3) -> Real {
    let nx = &plane.normal.x * &point.x;
    let ny = &plane.normal.y * &point.y;
    let nz = &plane.normal.z * &point.z;
    &(&(&nx + &ny) + &nz) + &plane.offset
}

fn linear_value(coefficients: &hyperlattice::Vector3, offset: &Real, point: &Point3) -> Real {
    let x = &coefficients.0[0] * &point.x;
    let y = &coefficients.0[1] * &point.y;
    let z = &coefficients.0[2] * &point.z;
    &(&(&x + &y) + &z) + offset
}

fn corners(min: &Point3, max: &Point3) -> [Point3; 8] {
    [
        Point3::new(min.x.clone(), min.y.clone(), min.z.clone()),
        Point3::new(max.x.clone(), min.y.clone(), min.z.clone()),
        Point3::new(min.x.clone(), max.y.clone(), min.z.clone()),
        Point3::new(max.x.clone(), max.y.clone(), min.z.clone()),
        Point3::new(min.x.clone(), min.y.clone(), max.z.clone()),
        Point3::new(max.x.clone(), min.y.clone(), max.z.clone()),
        Point3::new(min.x.clone(), max.y.clone(), max.z.clone()),
        Point3::new(max.x.clone(), max.y.clone(), max.z.clone()),
    ]
}
