//! Adapter for `inari::Interval` interval arithmetic.

use crate::backend::BackendCapabilities;
use crate::predicate::{Sign, SignKnowledge};
use crate::scalar::{MagnitudeBounds, PredicateScalar, ScalarFacts, StructuralScalar};

/// Capabilities provided by the `inari` interval backend.
pub const CAPABILITIES: BackendCapabilities = BackendCapabilities {
    structural_signs: true,
    exact_zero: true,
    magnitude_bounds: true,
    exact_arithmetic: false,
    adaptive_refinement: false,
    robust_fallback: false,
};

impl StructuralScalar for inari::Interval {
    fn scalar_facts(&self) -> ScalarFacts {
        let interval = *self;
        let sign = interval_sign(interval);
        let exact_zero = exact_zero(interval);
        let provably_nonzero = provably_nonzero(interval);

        ScalarFacts {
            sign,
            exact_zero,
            provably_nonzero,
            exact: singleton_finite(interval).map(|_| true),
            rational_only: singleton_finite(interval).map(|_| false),
            magnitude: magnitude_bounds(interval),
        }
    }

    fn known_sign(&self) -> SignKnowledge {
        let interval = *self;
        if exact_zero(interval) == Some(true) {
            SignKnowledge::exact(Sign::Zero)
        } else if let Some(sign) = interval_sign(interval) {
            SignKnowledge::filtered(sign)
        } else if provably_nonzero(interval) == Some(true) {
            SignKnowledge::NonZero
        } else {
            SignKnowledge::Unknown
        }
    }
}

impl PredicateScalar for inari::Interval {
    fn to_f64(&self) -> Option<f64> {
        singleton_finite(*self)
    }
}

fn interval_sign(interval: inari::Interval) -> Option<Sign> {
    let inf = interval.inf();
    let sup = interval.sup();

    if !valid_bounds(inf, sup) {
        None
    } else if inf == 0.0 && sup == 0.0 {
        Some(Sign::Zero)
    } else if inf > 0.0 {
        Some(Sign::Positive)
    } else if sup < 0.0 {
        Some(Sign::Negative)
    } else {
        None
    }
}

fn exact_zero(interval: inari::Interval) -> Option<bool> {
    let inf = interval.inf();
    let sup = interval.sup();

    if !valid_bounds(inf, sup) {
        None
    } else if inf == 0.0 && sup == 0.0 {
        Some(true)
    } else if inf > 0.0 || sup < 0.0 || inf == sup {
        Some(false)
    } else {
        None
    }
}

fn provably_nonzero(interval: inari::Interval) -> Option<bool> {
    let inf = interval.inf();
    let sup = interval.sup();

    if !valid_bounds(inf, sup) {
        None
    } else if inf > 0.0 || sup < 0.0 {
        Some(true)
    } else if inf == 0.0 && sup == 0.0 {
        Some(false)
    } else {
        None
    }
}

fn magnitude_bounds(interval: inari::Interval) -> Option<MagnitudeBounds> {
    if !valid_bounds(interval.inf(), interval.sup()) {
        return None;
    }

    let abs_lower = interval.mig();
    let abs_upper = interval.mag();
    (abs_lower.is_finite() && abs_upper.is_finite()).then_some(MagnitudeBounds {
        abs_lower,
        abs_upper,
    })
}

fn singleton_finite(interval: inari::Interval) -> Option<f64> {
    let inf = interval.inf();
    let sup = interval.sup();
    (inf == sup && inf.is_finite()).then_some(inf)
}

fn valid_bounds(inf: f64, sup: f64) -> bool {
    inf <= sup && !inf.is_nan() && !sup.is_nan()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::predicate::Certainty;

    #[test]
    fn interval_signs_are_filtered_when_zero_is_excluded() {
        let positive = inari::const_interval!(1.0, 2.0);
        let negative = inari::const_interval!(-2.0, -1.0);
        let spanning = inari::const_interval!(-1.0, 1.0);

        assert_eq!(
            positive.known_sign(),
            SignKnowledge::Known {
                sign: Sign::Positive,
                certainty: Certainty::Filtered
            }
        );
        assert_eq!(
            negative.known_sign(),
            SignKnowledge::Known {
                sign: Sign::Negative,
                certainty: Certainty::Filtered
            }
        );
        assert_eq!(spanning.known_sign(), SignKnowledge::Unknown);
    }

    #[test]
    fn interval_to_f64_only_exposes_singletons() {
        assert_eq!(inari::const_interval!(3.0, 3.0).to_f64(), Some(3.0));
        assert_eq!(inari::const_interval!(2.0, 3.0).to_f64(), None);
    }
}
