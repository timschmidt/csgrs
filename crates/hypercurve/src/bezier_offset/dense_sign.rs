//! Hypercurve's evaluation context for exact selected-algebra constructions.
//!
//! Dense tuple signs live in Hypersolve; Hypercurve supplies its policy
//! protocol and the retained algebraic-parameter sign authority, whose
//! interval, replay and witness caches serve every one-root query.

use super::*;

impl hypersolve::SelectedAlgebraContext for CurveContext {
    type Error = CurveError;

    fn real_sign(&self, value: &Real) -> Option<RealSign> {
        real_sign(value, self)
    }

    fn strict_counterpart(&self) -> Self {
        Self::strict_counterpart(self)
    }

    fn strict_predicate_pass<T>(&self, evaluate: impl FnOnce() -> T) -> T {
        Self::strict_predicate_pass(self, evaluate)
    }

    fn bounded_exact_predicate_pass<T>(&self, evaluate: impl FnOnce() -> T) -> T {
        Self::bounded_exact_predicate_pass(self, evaluate)
    }

    fn has_bounded_exact_predicate_budget(&self) -> bool {
        Self::has_bounded_exact_predicate_budget(self)
    }

    fn univariate_sign_at_root(
        &self,
        coefficients: &[Real],
        root: &AlgebraicRootRepresentation,
    ) -> CurveResult<Classification<RealSign>> {
        let result =
            match BezierParameter2::from_algebraic_root_representation_unbounded(root, self)? {
                Classification::Decided(parameter) => {
                    signed_coefficients_at_parameter(coefficients, &parameter, self)?
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            };
        #[cfg(feature = "dispatch-trace")]
        if result.is_decided() {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "dense-polynomial-tuple-sign",
                "retained-univariate-parameter",
            );
        }
        Ok(result)
    }
}
