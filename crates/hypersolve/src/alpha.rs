//! Alpha-theory style certificates for prepared polynomial solver rows.
//!
//! This module is intentionally narrow. It does not try to make `hypersolve`
//! a computer algebra system; it consumes the retained
//! [`PreparedUnivariateQuadraticResidual`](crate::PreparedUnivariateQuadraticResidual)
//! package and proves a conservative Newton basin condition with exact
//! `Real` arithmetic. The trust boundary follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997): numerical iteration may
//! propose a point, while exact replay and proof-producing filters decide what
//! is accepted. The Newton-basin shape follows Smale's alpha theory; see
//! S. Smale, "Newton's Method Estimates from Data at One Point," in *The
//! Merging of Disciplines: New Directions in Pure, Applied, and Computational
//! Mathematics*, Springer, 1986.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;

use crate::eval::EvaluationContext;
use crate::model::ConstraintKind;
use crate::prepared::PreparedProblem;
use crate::symbolic::SymbolId;

/// Result of an alpha-theory proof attempt for one prepared quadratic row.
#[derive(Clone, Debug, PartialEq)]
pub struct UnivariateQuadraticAlphaRow {
    /// Source constraint index in the problem.
    pub constraint_index: usize,
    /// Constraint name copied for diagnostics.
    pub name: String,
    /// Solver symbol used by the prepared quadratic.
    pub symbol: SymbolId,
    /// Exact residual value at the candidate center.
    pub residual: Real,
    /// Exact derivative `2*a*x + b` at the candidate center.
    pub derivative: Real,
    /// Exact numerator `|f(x)|*|a|` of the conservative alpha bound.
    pub alpha_numerator: Option<Real>,
    /// Exact denominator `|f'(x)|^2` of the conservative alpha bound.
    pub alpha_denominator: Option<Real>,
    /// Proof status for this row.
    pub status: UnivariateQuadraticAlphaStatus,
}

/// Status for a univariate quadratic alpha proof.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum UnivariateQuadraticAlphaStatus {
    /// The candidate is already an exact simple root of this row.
    ExactSimpleRoot,
    /// The exact sufficient test `8*|f(x)|*|a| <= |f'(x)|^2` succeeded.
    ///
    /// This uses the conservative alpha threshold `1/8`, which is below
    /// Smale's common alpha constant `(13 - 3*sqrt(17))/4`. Keeping the bound
    /// rational avoids introducing a square-root comparison solely for the
    /// certificate threshold.
    CertifiedSmaleAlphaBound,
    /// The candidate is an exact multiple root, so Newton uniqueness from a
    /// nonzero derivative is not available.
    ExactMultipleRoot,
    /// The derivative is certified zero while the residual is not.
    ZeroDerivative,
    /// Exact replay could not bind the prepared row's symbol.
    UnboundCandidateSymbol {
        /// Missing symbol.
        symbol: SymbolId,
    },
    /// A sign or ordering needed by the proof was not decidable under policy.
    Undecided,
    /// The alpha sufficient condition was exactly checked and failed.
    BoundFailed,
}

impl UnivariateQuadraticAlphaStatus {
    /// Returns whether this status is a proof-producing acceptance.
    pub const fn is_certified(&self) -> bool {
        matches!(self, Self::ExactSimpleRoot | Self::CertifiedSmaleAlphaBound)
    }
}

/// Report for alpha-theory checks over prepared univariate quadratic rows.
#[derive(Clone, Debug, PartialEq)]
pub struct UnivariateQuadraticAlphaReport {
    /// Per-row proof attempts.
    pub rows: Vec<UnivariateQuadraticAlphaRow>,
    /// Number of rows that produced proof-bearing statuses.
    pub certified_rows: usize,
    /// Number of active prepared univariate quadratic equality rows examined.
    pub examined_rows: usize,
}

impl UnivariateQuadraticAlphaReport {
    /// Returns true when at least one row was examined and every examined row
    /// produced a proof-bearing status.
    pub fn all_examined_rows_certified(&self) -> bool {
        self.examined_rows > 0 && self.certified_rows == self.examined_rows
    }
}

/// Certify prepared univariate quadratic equality rows using an exact alpha
/// sufficient condition.
///
/// For `f(x) = a*x^2 + b*x + c`, Smale-style alpha theory reduces to the exact
/// scalar quantity `alpha = |f(x)|*|a| / |f'(x)|^2` when `f'(x) != 0`. This
/// helper proves the conservative condition `alpha <= 1/8` by checking
/// `8*|f(x)|*|a| <= |f'(x)|^2` with `hyperlimit`'s exact `Real` comparison
/// policy. It is a local Newton-basin certificate, not a replacement for
/// ordinary residual replay or domain-specific geometry predicates.
pub fn certify_univariate_quadratic_alpha(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    policy: PredicatePolicy,
) -> UnivariateQuadraticAlphaReport {
    let mut rows = Vec::new();

    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        let Some(quadratic) = &prepared.univariate_quadratic_residuals()[constraint_index] else {
            continue;
        };
        let Some(candidate) = context.bindings().get(&quadratic.symbol()) else {
            rows.push(UnivariateQuadraticAlphaRow {
                constraint_index,
                name: constraint.name.clone(),
                symbol: quadratic.symbol(),
                residual: Real::zero(),
                derivative: Real::zero(),
                alpha_numerator: None,
                alpha_denominator: None,
                status: UnivariateQuadraticAlphaStatus::UnboundCandidateSymbol {
                    symbol: quadratic.symbol(),
                },
            });
            continue;
        };

        let residual = quadratic
            .eval_real(prepared.problem().variables.as_slice(), context.bindings())
            .unwrap_or_else(|_| Real::zero());
        let derivative = quadratic.quadratic().clone() * Real::from(2) * candidate.clone()
            + quadratic.linear().clone();
        let status_and_bound =
            classify_alpha_bound(&residual, &derivative, quadratic.quadratic(), policy);
        rows.push(UnivariateQuadraticAlphaRow {
            constraint_index,
            name: constraint.name.clone(),
            symbol: quadratic.symbol(),
            residual,
            derivative,
            alpha_numerator: status_and_bound.alpha_numerator,
            alpha_denominator: status_and_bound.alpha_denominator,
            status: status_and_bound.status,
        });
    }

    let certified_rows = rows.iter().filter(|row| row.status.is_certified()).count();
    let examined_rows = rows.len();
    UnivariateQuadraticAlphaReport {
        rows,
        certified_rows,
        examined_rows,
    }
}

struct AlphaBoundClassification {
    status: UnivariateQuadraticAlphaStatus,
    alpha_numerator: Option<Real>,
    alpha_denominator: Option<Real>,
}

fn classify_alpha_bound(
    residual: &Real,
    derivative: &Real,
    quadratic: &Real,
    policy: PredicatePolicy,
) -> AlphaBoundClassification {
    let Some(residual_abs) = abs_real_with_policy(residual, policy) else {
        return alpha_status(UnivariateQuadraticAlphaStatus::Undecided);
    };
    let Some(derivative_abs) = abs_real_with_policy(derivative, policy) else {
        return alpha_status(UnivariateQuadraticAlphaStatus::Undecided);
    };
    let Some(quadratic_abs) = abs_real_with_policy(quadratic, policy) else {
        return alpha_status(UnivariateQuadraticAlphaStatus::Undecided);
    };

    let residual_is_zero = compare_reals_with_policy(&residual_abs, &Real::zero(), policy).value()
        == Some(Ordering::Equal);
    let derivative_is_zero = compare_reals_with_policy(&derivative_abs, &Real::zero(), policy)
        .value()
        == Some(Ordering::Equal);

    if residual_is_zero && derivative_is_zero {
        return alpha_status(UnivariateQuadraticAlphaStatus::ExactMultipleRoot);
    }
    if residual_is_zero {
        return AlphaBoundClassification {
            status: UnivariateQuadraticAlphaStatus::ExactSimpleRoot,
            alpha_numerator: Some(Real::zero()),
            alpha_denominator: Some(derivative_abs.clone() * derivative_abs),
        };
    }
    if derivative_is_zero {
        return alpha_status(UnivariateQuadraticAlphaStatus::ZeroDerivative);
    }

    let numerator = residual_abs * quadratic_abs;
    let denominator = derivative_abs.clone() * derivative_abs;
    let sufficient_left = numerator.clone() * Real::from(8);
    let status = match compare_reals_with_policy(&sufficient_left, &denominator, policy).value() {
        Some(Ordering::Less | Ordering::Equal) => {
            UnivariateQuadraticAlphaStatus::CertifiedSmaleAlphaBound
        }
        Some(Ordering::Greater) => UnivariateQuadraticAlphaStatus::BoundFailed,
        None => UnivariateQuadraticAlphaStatus::Undecided,
    };
    AlphaBoundClassification {
        status,
        alpha_numerator: Some(numerator),
        alpha_denominator: Some(denominator),
    }
}

fn alpha_status(status: UnivariateQuadraticAlphaStatus) -> AlphaBoundClassification {
    AlphaBoundClassification {
        status,
        alpha_numerator: None,
        alpha_denominator: None,
    }
}

fn abs_real_with_policy(value: &Real, policy: PredicatePolicy) -> Option<Real> {
    match compare_reals_with_policy(value, &Real::zero(), policy).value()? {
        Ordering::Less => Some(-value.clone()),
        Ordering::Equal | Ordering::Greater => Some(value.clone()),
    }
}
