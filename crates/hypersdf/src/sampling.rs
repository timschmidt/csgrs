//! Report-bearing preview sampling adapters.
//!
//! Primitive-float samples are display and adapter data, not exact topology.
//! This module makes that boundary explicit: values are lowered through
//! `Real::to_f32_lossy`/`Real::to_f64_lossy`, reports count failed lowerings,
//! and the topology status remains preview-only. This is the same EGC boundary
//! described by Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7.1-2 (1997): approximate values can be useful views, but they do
//! not certify combinatorial decisions.

use core::cmp::Ordering;

use hyperlimit::{Point3, PredicateOutcome, compare_reals};
use hyperreal::Real;

use crate::expr::{SdfCoordinate, SdfExpr};
use crate::status::{SdfFreshness, SdfMetricStatus};

/// Primitive scalar precision requested from a preview sampler.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfSamplingPrecision {
    /// Lower values to `f32`.
    F32,
    /// Lower values to `f64`.
    F64,
}

/// Whether sampled values may be used as topology evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfSampleTopologyStatus {
    /// Preview values are explicitly not topology evidence.
    PreviewOnly,
    /// Exact replay accepted the samples for a specific downstream purpose.
    CertifiedReplay,
    /// The adapter could not establish topology status.
    Unknown,
}

/// One lossy preview sample.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfPreviewSample {
    /// Query point.
    pub point: Point3,
    /// Lowered scalar value as `f64` storage.
    ///
    /// `F32` samples are widened after `Real::to_f32_lossy` so reports can
    /// store one scalar field while preserving the requested precision.
    pub value: Option<f64>,
}

/// Report returned by preview sampling.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfSamplingReport {
    /// Requested primitive precision.
    pub precision: SdfSamplingPrecision,
    /// Metric claim of the source expression before scalar lowering.
    pub metric_status: SdfMetricStatus,
    /// Whether the samples may be consumed as topology evidence.
    pub topology_status: SdfSampleTopologyStatus,
    /// Prepared-source freshness.
    pub freshness: SdfFreshness,
    /// Number of query points.
    pub sample_count: usize,
    /// Number of samples that could not be lowered to a finite primitive float.
    pub non_finite_count: usize,
    /// Lowered sample records.
    pub samples: Vec<SdfPreviewSample>,
}

/// Axis-aligned exact preview grid.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfPreviewGrid {
    /// Exact origin of grid index `(0, 0, 0)`.
    pub origin: Point3,
    /// Exact per-axis step between adjacent samples.
    pub step: Point3,
    /// Number of samples on each axis.
    pub dimensions: [u32; 3],
}

impl SdfPreviewGrid {
    /// Construct an exact preview grid.
    pub const fn new(origin: Point3, step: Point3, dimensions: [u32; 3]) -> Self {
        Self {
            origin,
            step,
            dimensions,
        }
    }

    /// Return the total number of grid points, validating dimensions.
    pub fn point_count(&self) -> Result<usize, SdfGridSamplingError> {
        if self.dimensions.contains(&0) {
            return Err(SdfGridSamplingError::EmptyDimension);
        }
        let count = self
            .dimensions
            .iter()
            .try_fold(1_u64, |acc, dimension| {
                acc.checked_mul(u64::from(*dimension))
            })
            .ok_or(SdfGridSamplingError::TooManySamples)?;
        usize::try_from(count).map_err(|_| SdfGridSamplingError::TooManySamples)
    }

    /// Return all exact grid points in z-major, then y, then x-fast order.
    pub fn points(&self) -> Result<Vec<Point3>, SdfGridSamplingError> {
        let mut points = Vec::with_capacity(self.point_count()?);
        for z in 0..self.dimensions[2] {
            for y in 0..self.dimensions[1] {
                for x in 0..self.dimensions[0] {
                    points.push(self.point(x, y, z));
                }
            }
        }
        Ok(points)
    }

    fn point(&self, x: u32, y: u32, z: u32) -> Point3 {
        let x = Real::from(x);
        let y = Real::from(y);
        let z = Real::from(z);
        Point3::new(
            &self.origin.x + &(&self.step.x * &x),
            &self.origin.y + &(&self.step.y * &y),
            &self.origin.z + &(&self.step.z * &z),
        )
    }
}

/// Input validation error for preview-grid sampling.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfGridSamplingError {
    /// At least one grid dimension is zero.
    EmptyDimension,
    /// The requested grid point count overflows host memory indexing.
    TooManySamples,
}

/// Report returned by regular-grid preview sampling.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfGridSamplingReport {
    /// Exact grid frame that generated the query points.
    pub grid: SdfPreviewGrid,
    /// Point-sampling report over generated grid points.
    pub samples: SdfSamplingReport,
}

pub(crate) fn sample_expr_points_preview<'a, I>(
    expr: &SdfExpr,
    points: I,
    precision: SdfSamplingPrecision,
    metric_status: SdfMetricStatus,
    freshness: SdfFreshness,
) -> SdfSamplingReport
where
    I: IntoIterator<Item = &'a Point3>,
{
    let mut samples = Vec::new();
    let mut non_finite_count = 0_usize;
    for point in points {
        let exact_value = scalar_expr_point(expr, point);
        let value = exact_value.as_ref().and_then(|value| match precision {
            SdfSamplingPrecision::F32 => value.to_f32_lossy().map(f64::from),
            SdfSamplingPrecision::F64 => value.to_f64_lossy(),
        });
        if value.is_none() {
            non_finite_count += 1;
        }
        samples.push(SdfPreviewSample {
            point: point.clone(),
            value,
        });
    }
    SdfSamplingReport {
        precision,
        metric_status,
        topology_status: SdfSampleTopologyStatus::PreviewOnly,
        freshness,
        sample_count: samples.len(),
        non_finite_count,
        samples,
    }
}

pub(crate) fn sample_expr_grid_preview(
    expr: &SdfExpr,
    grid: SdfPreviewGrid,
    precision: SdfSamplingPrecision,
    metric_status: SdfMetricStatus,
    freshness: SdfFreshness,
) -> Result<SdfGridSamplingReport, SdfGridSamplingError> {
    let points = grid.points()?;
    let samples =
        sample_expr_points_preview(expr, points.iter(), precision, metric_status, freshness);
    Ok(SdfGridSamplingReport { grid, samples })
}

pub(crate) fn scalar_expr_point(expr: &SdfExpr, point: &Point3) -> Option<Real> {
    match expr {
        SdfExpr::Constant(value) => Some(value.clone()),
        SdfExpr::Coordinate(axis) => Some(match axis {
            SdfCoordinate::X => point.x.clone(),
            SdfCoordinate::Y => point.y.clone(),
            SdfCoordinate::Z => point.z.clone(),
        }),
        SdfExpr::Linear {
            coefficients,
            offset,
        } => {
            let x = &coefficients.0[0] * &point.x;
            let y = &coefficients.0[1] * &point.y;
            let z = &coefficients.0[2] * &point.z;
            Some(&(&(&x + &y) + &z) + offset)
        }
        SdfExpr::Primitive(primitive) => primitive.scalar_value(point),
        SdfExpr::Union(left, right) => {
            let left = scalar_expr_point(left, point)?;
            let right = scalar_expr_point(right, point)?;
            choose_min(left, right)
        }
        SdfExpr::Intersection(left, right) => {
            let left = scalar_expr_point(left, point)?;
            let right = scalar_expr_point(right, point)?;
            choose_max(left, right)
        }
        SdfExpr::Add(left, right) => {
            Some(&scalar_expr_point(left, point)? + &scalar_expr_point(right, point)?)
        }
        SdfExpr::Sub(left, right) => {
            Some(&scalar_expr_point(left, point)? - &scalar_expr_point(right, point)?)
        }
        SdfExpr::Mul(left, right) => {
            Some(&scalar_expr_point(left, point)? * &scalar_expr_point(right, point)?)
        }
        SdfExpr::Abs(inner) => {
            let value = scalar_expr_point(inner, point)?;
            match compare_reals(&value, &Real::zero()) {
                PredicateOutcome::Decided {
                    value: Ordering::Less,
                    ..
                } => Some(-&value),
                PredicateOutcome::Decided { .. } => Some(value),
                PredicateOutcome::Unknown { .. } => None,
            }
        }
        SdfExpr::Sqrt(inner) => scalar_expr_point(inner, point)?.sqrt().ok(),
        SdfExpr::Complement(inner) => scalar_expr_point(inner, point).map(|value| -&value),
        SdfExpr::Offset { child, amount } => {
            scalar_expr_point(child, point).map(|value| &value - amount)
        }
        SdfExpr::Transform { child, transform } => {
            scalar_expr_point(child, &transform.inverse_point(point))
        }
    }
}

fn choose_min(left: Real, right: Real) -> Option<Real> {
    match compare_reals(&left, &right) {
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            ..
        } => Some(right),
        PredicateOutcome::Decided { .. } => Some(left),
        PredicateOutcome::Unknown { .. } => None,
    }
}

fn choose_max(left: Real, right: Real) -> Option<Real> {
    match compare_reals(&left, &right) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => Some(right),
        PredicateOutcome::Decided { .. } => Some(left),
        PredicateOutcome::Unknown { .. } => None,
    }
}
