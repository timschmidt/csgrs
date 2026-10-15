//! Fallible construction of retained scalar values. Branch decisions are strict.

use core::{cmp::Ordering, fmt};
use hyperlimit::{Escalation, Point3, PredicateOutcome, RefinementNeed};
use hyperreal::{Problem, Real};

use crate::{SdfCoordinate, SdfExpr, policy::compare_reals_for_construction};

/// Why a retained scalar value could not be constructed.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum SdfEvaluationError {
    /// A primitive parameter was proved outside its supported domain.
    InvalidDomain { operation: &'static str },
    /// A domain check or branch choice lacks a strict proof.
    Undecided {
        operation: &'static str,
        needed: RefinementNeed,
        stage: Escalation,
    },
    /// Scalar arithmetic failed, retaining its native cause.
    Scalar {
        operation: &'static str,
        source: Problem,
    },
}

impl fmt::Display for SdfEvaluationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidDomain { operation } => write!(f, "invalid domain for {operation}"),
            Self::Undecided {
                operation,
                needed,
                stage,
            } => {
                write!(f, "{operation} is undecided: {needed:?} at {stage:?}")
            }
            Self::Scalar { operation, source } => write!(f, "{operation}: {source}"),
        }
    }
}

impl std::error::Error for SdfEvaluationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Scalar { source, .. } => Some(source),
            _ => None,
        }
    }
}

pub(crate) fn decide<T>(
    outcome: PredicateOutcome<T>,
    operation: &'static str,
) -> Result<T, SdfEvaluationError> {
    match outcome {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => Err(SdfEvaluationError::Undecided {
            operation,
            needed,
            stage,
        }),
    }
}

pub(crate) fn require_domain(
    outcome: PredicateOutcome<bool>,
    operation: &'static str,
) -> Result<(), SdfEvaluationError> {
    if decide(outcome, operation)? {
        Ok(())
    } else {
        Err(SdfEvaluationError::InvalidDomain { operation })
    }
}

pub(crate) fn choose_min(left: Real, right: Real) -> Result<Real, SdfEvaluationError> {
    match decide(compare_reals_for_construction(&left, &right), "SDF minimum")? {
        Ordering::Greater => Ok(right),
        _ => Ok(left),
    }
}

pub(crate) fn choose_max(left: Real, right: Real) -> Result<Real, SdfEvaluationError> {
    match decide(compare_reals_for_construction(&left, &right), "SDF maximum")? {
        Ordering::Less => Ok(right),
        _ => Ok(left),
    }
}

pub(crate) fn evaluate_expr(expr: &SdfExpr, point: &Point3) -> Result<Real, SdfEvaluationError> {
    match expr {
        SdfExpr::Constant(value) => Ok(value.clone()),
        SdfExpr::Coordinate(axis) => Ok(match axis {
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
            Ok(&(&(&x + &y) + &z) + offset)
        }
        SdfExpr::Primitive(primitive) => primitive.try_scalar_value(point),
        SdfExpr::Union(left, right) => {
            choose_min(evaluate_expr(left, point)?, evaluate_expr(right, point)?)
        }
        SdfExpr::Intersection(left, right) => {
            choose_max(evaluate_expr(left, point)?, evaluate_expr(right, point)?)
        }
        SdfExpr::Add(left, right) => {
            Ok(&evaluate_expr(left, point)? + &evaluate_expr(right, point)?)
        }
        SdfExpr::Sub(left, right) => {
            Ok(&evaluate_expr(left, point)? - &evaluate_expr(right, point)?)
        }
        SdfExpr::Mul(left, right) => {
            Ok(&evaluate_expr(left, point)? * &evaluate_expr(right, point)?)
        }
        SdfExpr::Abs(inner) => {
            let value = evaluate_expr(inner, point)?;
            match decide(
                compare_reals_for_construction(&value, &Real::zero()),
                "SDF absolute value",
            )? {
                Ordering::Less => Ok(-&value),
                _ => Ok(value),
            }
        }
        SdfExpr::Sqrt(inner) => {
            evaluate_expr(inner, point)?
                .sqrt()
                .map_err(|source| SdfEvaluationError::Scalar {
                    operation: "SDF square root",
                    source,
                })
        }
        SdfExpr::Sin(inner) => Ok(evaluate_expr(inner, point)?.sin()),
        SdfExpr::Cos(inner) => Ok(evaluate_expr(inner, point)?.cos()),
        SdfExpr::Tan(inner) => {
            evaluate_expr(inner, point)?
                .tan()
                .map_err(|source| SdfEvaluationError::Scalar {
                    operation: "SDF tangent",
                    source,
                })
        }
        SdfExpr::Complement(inner) => evaluate_expr(inner, point).map(|value| -&value),
        SdfExpr::Offset { child, amount } => {
            evaluate_expr(child, point).map(|value| &value - amount)
        }
        SdfExpr::Transform { child, transform } => {
            evaluate_expr(child, &transform.inverse_point(point))
        }
    }
}
