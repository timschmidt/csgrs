//! Exact field transforms.
//!
//! `hypersdf` keeps transform nodes factored instead of baking coordinates
//! into expanded scalar expressions. A translation or affine frame can be
//! replayed by transforming the
//! exact query point or cell before invoking the child predicate, preserving
//! the child's certified topology route. General affine AABB replay uses
//! Arvo's per-axis interval accumulation, retaining exact `Real` endpoints
//! without materializing and transforming all eight corners.

use core::cmp::Ordering;

use hyperlattice::{Matrix4, Vector4};
use hyperlimit::{Escalation, Point3, PredicateOutcome, RefinementNeed, compare_reals};
use hyperreal::{Problem, Real, ZeroOneMinusOneStatus};

use crate::status::{SdfGradientStatus, SdfLipschitzStatus, SdfMetricStatus};

/// Exact transform node supported by the first SDF expression core.
#[derive(Clone, Debug, PartialEq)]
pub enum SdfTransform {
    /// Translate the field in object space.
    ///
    /// The transformed field evaluates as `child(point - offset)`. A positive
    /// offset therefore moves the child shape in world space by `offset`.
    Translation {
        /// Exact world-space translation offset.
        offset: Point3,
    },
    /// Invertible affine transform represented by exact homogeneous matrices.
    ///
    /// The transformed field evaluates as `child(inverse * point)`. Both the
    /// source matrix and its exact inverse are retained so repeated
    /// classification does not rediscover the inverse at every query.
    Affine {
        /// Exact child-to-world affine matrix.
        matrix: Matrix4,
        /// Exact world-to-child affine matrix.
        inverse: Matrix4,
    },
}

impl SdfTransform {
    /// Construct a translation transform.
    pub const fn translation(offset: Point3) -> Self {
        Self::Translation { offset }
    }

    /// Construct an invertible affine transform.
    ///
    /// Only homogeneous matrices with last row `[0, 0, 0, 1]` are accepted.
    /// Projective transforms are deliberately rejected until the SDF model has
    /// explicit denominator-domain reports. The inverse is computed once using
    /// `hyperlattice` exact matrix inversion and retained with the transform.
    pub fn affine(matrix: Matrix4) -> Result<Self, SdfTransformError> {
        if !is_affine_matrix(&matrix) {
            return Err(SdfTransformError::NonAffineMatrix);
        }
        let inverse = matrix
            .clone()
            .inverse()
            .map_err(SdfTransformError::SingularMatrix)?;
        if !is_affine_matrix(&inverse) {
            return Err(SdfTransformError::NonAffineInverse);
        }
        Ok(Self::Affine { matrix, inverse })
    }

    /// Map a world-space point to child-object coordinates.
    pub fn inverse_point(&self, point: &Point3) -> Point3 {
        match self {
            Self::Translation { offset } => subtract_point(point, offset),
            Self::Affine { inverse, .. } => transform_point(inverse, point),
        }
    }

    /// Map a world-space AABB to child-object coordinates.
    pub fn inverse_aabb(&self, min: &Point3, max: &Point3) -> Option<(Point3, Point3)> {
        match self {
            Self::Translation { offset } => {
                Some((subtract_point(min, offset), subtract_point(max, offset)))
            }
            Self::Affine { inverse, .. } => transformed_aabb(inverse, min, max),
        }
    }

    /// Return all exact parameters carried by the transform.
    pub fn parameters(&self) -> Vec<&Real> {
        match self {
            Self::Translation { offset } => vec![&offset.x, &offset.y, &offset.z],
            Self::Affine { matrix, inverse } => matrix
                .0
                .iter()
                .flatten()
                .chain(inverse.0.iter().flatten())
                .collect(),
        }
    }

    /// Return the metric status after applying this transform to a child field.
    pub const fn metric_status(&self, child: SdfMetricStatus) -> SdfMetricStatus {
        match self {
            Self::Translation { .. } => child,
            Self::Affine { .. } => child.csg_pair(SdfMetricStatus::SignEquivalent),
        }
    }

    /// Return the gradient status after applying this transform to a child field.
    pub const fn gradient_status(&self, child: SdfGradientStatus) -> SdfGradientStatus {
        match self {
            Self::Translation { .. } => child,
            Self::Affine { .. } => SdfGradientStatus::PiecewiseExact,
        }
    }

    /// Return the Lipschitz status after applying this transform to a child field.
    pub const fn lipschitz_status(&self, child: SdfLipschitzStatus) -> SdfLipschitzStatus {
        match self {
            Self::Translation { .. } => child,
            Self::Affine { .. } => SdfLipschitzStatus::LocalOnly,
        }
    }
}

/// Construction error for exact SDF transforms.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SdfTransformError {
    /// The supplied matrix is not a homogeneous affine transform.
    NonAffineMatrix,
    /// The exact matrix inverse could not be constructed.
    SingularMatrix(Problem),
    /// The inverse did not retain affine homogeneous form.
    NonAffineInverse,
}

fn subtract_point(point: &Point3, offset: &Point3) -> Point3 {
    Point3::new(
        &point.x - &offset.x,
        &point.y - &offset.y,
        &point.z - &offset.z,
    )
}

fn is_affine_matrix(matrix: &Matrix4) -> bool {
    matches!(
        (
            matrix.0[3][0].zero_one_or_minus_one(),
            matrix.0[3][1].zero_one_or_minus_one(),
            matrix.0[3][2].zero_one_or_minus_one(),
            matrix.0[3][3].zero_one_or_minus_one(),
        ),
        (
            ZeroOneMinusOneStatus::Zero,
            ZeroOneMinusOneStatus::Zero,
            ZeroOneMinusOneStatus::Zero,
            ZeroOneMinusOneStatus::One
        )
    )
}

fn transform_point(matrix: &Matrix4, point: &Point3) -> Point3 {
    let transformed = matrix.transform_vec4_point(&Vector4([
        point.x.clone(),
        point.y.clone(),
        point.z.clone(),
        Real::one(),
    ]));
    Point3::new(
        transformed.0[0].clone(),
        transformed.0[1].clone(),
        transformed.0[2].clone(),
    )
}

fn transformed_aabb(matrix: &Matrix4, min: &Point3, max: &Point3) -> Option<(Point3, Point3)> {
    let (lower_x, upper_x) = transformed_axis_interval(matrix, 0, min, max)?;
    let (lower_y, upper_y) = transformed_axis_interval(matrix, 1, min, max)?;
    let (lower_z, upper_z) = transformed_axis_interval(matrix, 2, min, max)?;
    Some((
        Point3::new(lower_x, lower_y, lower_z),
        Point3::new(upper_x, upper_y, upper_z),
    ))
}

fn transformed_axis_interval(
    matrix: &Matrix4,
    row: usize,
    min: &Point3,
    max: &Point3,
) -> Option<(Real, Real)> {
    let lower_endpoints = [&min.x, &min.y, &min.z];
    let upper_endpoints = [&max.x, &max.y, &max.z];
    let mut lower = matrix.0[row][3].clone();
    let mut upper = lower.clone();

    for axis in 0..3 {
        let from_lower = &matrix.0[row][axis] * lower_endpoints[axis];
        let from_upper = &matrix.0[row][axis] * upper_endpoints[axis];
        lower = &lower + &min_real(from_lower.clone(), from_upper.clone())?;
        upper = &upper + &max_real(from_lower, from_upper)?;
    }
    Some((lower, upper))
}

fn min_real(left: Real, right: Real) -> Option<Real> {
    match compare_reals(&left, &right) {
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            ..
        } => Some(right),
        PredicateOutcome::Decided { .. } => Some(left),
        PredicateOutcome::Unknown { .. } => None,
    }
}

fn max_real(left: Real, right: Real) -> Option<Real> {
    match compare_reals(&left, &right) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => Some(right),
        PredicateOutcome::Decided { .. } => Some(left),
        PredicateOutcome::Unknown { .. } => None,
    }
}

pub(crate) fn unsupported_transform_outcome<T>() -> PredicateOutcome<T> {
    PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided)
}
