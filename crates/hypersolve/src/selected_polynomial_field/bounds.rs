//! Certified enclosures keep the exact field value as their authority.

use super::*;
use crate::RealInterval;

/// A checked closed interval around one retained exact field value.
///
/// This is an enclosure certificate, not an independently selected algebraic
/// scalar. Its private bounds can only be constructed or narrowed by strict
/// comparisons against the original value. Clones share that immutable value.
#[derive(Clone)]
pub struct SelectedPolynomialEnclosure {
    value: SelectedPolynomialValue,
    interval: RealInterval,
}

impl std::fmt::Debug for SelectedPolynomialEnclosure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Diagnostics must not expand the shared field/value graph as a tree.
        f.debug_struct("SelectedPolynomialEnclosure")
            .field("interval", &self.interval)
            .finish_non_exhaustive()
    }
}

impl SelectedPolynomialValue {
    /// Finds exact dyadic bounds by strict sign-guided expansion from zero.
    ///
    /// Each new bound comparison consumes one shared root-refinement unit.
    /// This counts comparisons, not their internal algebraic work or memory.
    /// An unresolved sign or reached limit returns uncertainty; no approximate
    /// terminal or projected scalar replaces this value.
    pub fn enclosed<C: SelectedAlgebraContext>(
        &self,
        policy: &C,
    ) -> ExactResult<SelectedPolynomialEnclosure, C::Error> {
        finish(policy.strict_predicate_pass(|| {
            if self.stored_zero() {
                return Ok(SelectedPolynomialEnclosure::exact(
                    self.clone(),
                    Real::zero(),
                ));
            }
            let sign = compare_at(self, &Real::zero(), policy)?;
            if sign == RealSign::Zero {
                return Ok(SelectedPolynomialEnclosure::exact(
                    self.clone(),
                    Real::zero(),
                ));
            }
            let negative = sign == RealSign::Negative;
            let magnitude = if negative {
                self.scale(&Real::from(-1))?
            } else {
                self.clone()
            };
            let mut lower = Real::zero();
            let mut upper = Real::one();
            loop {
                match compare_at(&magnitude, &upper, policy)? {
                    RealSign::Zero => {
                        return Ok(SelectedPolynomialEnclosure::exact(
                            self.clone(),
                            if negative { -upper } else { upper },
                        ));
                    }
                    RealSign::Negative => break,
                    RealSign::Positive => {
                        lower = upper.clone();
                        upper = &upper + &upper;
                    }
                }
            }
            let interval = if negative {
                RealInterval {
                    lower: -upper,
                    upper: -lower,
                }
            } else {
                RealInterval { lower, upper }
            };
            Ok(SelectedPolynomialEnclosure {
                value: self.clone(),
                interval,
            })
        }))
    }
}

impl SelectedPolynomialEnclosure {
    fn exact(value: SelectedPolynomialValue, point: Real) -> Self {
        Self {
            value,
            interval: RealInterval {
                lower: point.clone(),
                upper: point,
            },
        }
    }

    /// Validates closed bounds against the retained value. Caller-supplied
    /// bounds carry no trusted containment or root-count claim.
    pub fn try_new<C: SelectedAlgebraContext>(
        value: SelectedPolynomialValue,
        interval: RealInterval,
        policy: &C,
    ) -> ExactResult<Self, C::Error> {
        finish(policy.strict_predicate_pass(|| {
            let width = policy
                .real_sign(&(&interval.upper - &interval.lower))
                .ok_or(SelectedPolynomialError::Uncertain(
                    UncertaintyReason::RealSign,
                ))?;
            if width == RealSign::Negative {
                return Err(SelectedPolynomialError::InvalidEnclosure);
            }
            let lower = compare_at(&value, &interval.lower, policy)?;
            if lower == RealSign::Negative || (width == RealSign::Zero && lower != RealSign::Zero) {
                return Err(SelectedPolynomialError::InvalidEnclosure);
            }
            if width == RealSign::Zero {
                return Ok(Self::exact(value, interval.lower));
            }
            if compare_at(&value, &interval.upper, policy)? == RealSign::Positive {
                return Err(SelectedPolynomialError::InvalidEnclosure);
            }
            Ok(Self { value, interval })
        }))
    }

    /// The unchanged exact scalar, available for subsequent field arithmetic.
    pub fn value(&self) -> &SelectedPolynomialValue {
        &self.value
    }

    /// Certified closed bounds; neither endpoint is a replacement coordinate.
    pub fn interval(&self) -> &RealInterval {
        &self.interval
    }

    /// Performs at most `steps` exact midpoint comparisons on a clone.
    ///
    /// Each comparison halves the width or proves an exact singleton. Zero
    /// steps and an existing singleton reuse their certificate without new
    /// refinement work. A failed attempt does not mutate this certificate.
    pub fn refined<C: SelectedAlgebraContext>(
        &self,
        steps: usize,
        policy: &C,
    ) -> ExactResult<Self, C::Error> {
        finish(policy.strict_predicate_pass(|| {
            let mut result = self.clone();
            for _ in 0..steps {
                if result.interval.lower == result.interval.upper {
                    break;
                }
                let midpoint = Real::average_pair(&result.interval.lower, &result.interval.upper);
                match compare_at(&result.value, &midpoint, policy)? {
                    RealSign::Negative => result.interval.upper = midpoint,
                    RealSign::Positive => result.interval.lower = midpoint,
                    RealSign::Zero => {
                        result = Self::exact(result.value, midpoint);
                        break;
                    }
                }
            }
            Ok(result)
        }))
    }
}

fn compare_at<C: SelectedAlgebraContext>(
    value: &SelectedPolynomialValue,
    point: &Real,
    policy: &C,
) -> Result<RealSign, SelectedPolynomialError<C::Error>> {
    crate::work_budget::consume(crate::work_budget::WorkKind::RootRefinement, 1)
        .map_err(|_| SelectedPolynomialError::Uncertain(UncertaintyReason::BudgetExhausted))?;
    value
        .subtract(&value.field().constant(point.clone())?)?
        .sign_strict(policy)
}
