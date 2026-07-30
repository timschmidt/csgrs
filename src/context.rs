//! Immutable operation policy and aggregate predicate certainty.

use std::cell::Cell;

use hypercurve::{Classification, CurveCertainty, CurveOutcome, CurvePolicy, CurveResult};
use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy};
use hypermesh::{MeshCertainty, MeshContext, MeshOutcome};

use crate::errors::ValidationError;

/// Immutable predicate policy selected for one CSG operation.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct GeometryContext {
    predicates: PredicatePolicy,
}

impl GeometryContext {
    /// Strict certified topology.
    pub const STRICT: Self = Self::new(PredicatePolicy::STRICT);

    /// Certified topology with Hyperlimit's terminal 512-bit interpretation.
    pub const APPROXIMATE_512: Self = Self::new(PredicatePolicy::APPROXIMATE_512);

    /// Construct a context with the selected Hyperlimit predicate policy.
    pub const fn new(predicates: PredicatePolicy) -> Self {
        Self { predicates }
    }

    /// Return the selected predicate policy.
    pub const fn predicate_policy(self) -> PredicatePolicy {
        self.predicates
    }

    /// Derive the matching Hypercurve policy.
    pub fn curve_policy(self) -> CurvePolicy {
        if self.predicates == PredicatePolicy::APPROXIMATE_512 {
            CurvePolicy::APPROXIMATE_512
        } else {
            CurvePolicy::STRICT
        }
    }

    /// Derive the matching Hypermesh context.
    pub const fn mesh_context(self) -> MeshContext {
        MeshContext::new(self.predicates)
    }
}

/// Weakest predicate certainty consumed by a completed CSG operation.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum GeometryCertainty {
    /// Every consumed topology decision was certified.
    Certified,
    /// At least one decision used Hyperlimit's terminal 512-bit interpretation.
    Approximate512Consumed,
}

/// Completed CSG value paired with aggregate predicate certainty.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GeometryOutcome<T> {
    /// Completed value.
    pub value: T,
    /// Weakest certainty consumed while producing `value`.
    pub certainty: GeometryCertainty,
}

impl<T> GeometryOutcome<T> {
    const fn new(value: T, certainty: GeometryCertainty) -> Self {
        Self { value, certainty }
    }

    /// Transform the value without changing aggregate certainty.
    pub fn map<U>(self, map: impl FnOnce(T) -> U) -> GeometryOutcome<U> {
        GeometryOutcome::new(map(self.value), self.certainty)
    }

    /// Fallibly transform the value without changing aggregate certainty.
    pub fn try_map<U, E>(
        self,
        map: impl FnOnce(T) -> Result<U, E>,
    ) -> Result<GeometryOutcome<U>, E> {
        map(self.value).map(|value| GeometryOutcome::new(value, self.certainty))
    }

    /// Consume the outcome and return its value.
    pub fn into_value(self) -> T {
        self.value
    }
}

// Feature-minimal builds retain the public context and mesh accumulator while
// curve/predicate consumers are compiled out.
#[allow(dead_code)]
pub(crate) struct GeometryDecisions {
    context: GeometryContext,
    curve_policy: CurvePolicy,
    mesh_context: MeshContext,
    certainty: Cell<GeometryCertainty>,
}

#[allow(dead_code)]
impl GeometryDecisions {
    pub(crate) fn new(context: &GeometryContext) -> Self {
        Self {
            context: *context,
            curve_policy: context.curve_policy(),
            mesh_context: context.mesh_context(),
            certainty: Cell::new(GeometryCertainty::Certified),
        }
    }

    pub(crate) const fn curve_policy(&self) -> &CurvePolicy {
        &self.curve_policy
    }

    pub(crate) const fn predicate_policy(&self) -> PredicatePolicy {
        self.context.predicate_policy()
    }

    pub(crate) const fn mesh_context(&self) -> &MeshContext {
        &self.mesh_context
    }

    pub(crate) const fn finish<T>(&self, value: T) -> GeometryOutcome<T> {
        GeometryOutcome::new(value, self.certainty.get())
    }

    pub(crate) fn observe(&self, certainty: GeometryCertainty) {
        if certainty == GeometryCertainty::Approximate512Consumed {
            self.certainty.set(certainty);
        }
    }

    pub(crate) fn consume_curve<T>(&self, outcome: CurveOutcome<T>) -> T {
        if outcome.certainty == CurveCertainty::Approximate512Consumed {
            self.observe(GeometryCertainty::Approximate512Consumed);
        }
        outcome.value
    }

    pub(crate) fn consume_mesh<T>(&self, outcome: MeshOutcome<T>) -> T {
        if outcome.certainty == MeshCertainty::Approximate512Consumed {
            self.observe(GeometryCertainty::Approximate512Consumed);
        }
        outcome.value
    }

    pub(crate) fn decide<T>(
        &self,
        outcome: PredicateOutcome<T>,
        predicate: &'static str,
    ) -> Result<T, ValidationError> {
        match outcome {
            PredicateOutcome::Decided {
                value, certainty, ..
            } => {
                if certainty == Certainty::Approximate {
                    self.observe(GeometryCertainty::Approximate512Consumed);
                }
                Ok(value)
            },
            PredicateOutcome::Unknown { .. } => Err(ValidationError::Geometry(format!(
                "{predicate} is undecided under {:?}",
                self.context.predicate_policy()
            ))),
        }
    }

    pub(crate) fn classify_curve<T>(
        &self,
        operation: &'static str,
        mut evaluate: impl FnMut(&CurvePolicy) -> CurveResult<Classification<T>>,
    ) -> Result<T, ValidationError> {
        let resolve = |classification| match classification {
            Classification::Decided(value) => Ok(value),
            Classification::Uncertain(reason) => Err(ValidationError::Geometry(format!(
                "{operation} is uncertain: {reason:?}"
            ))),
        };

        if self.context.predicate_policy() != PredicatePolicy::APPROXIMATE_512 {
            return evaluate(self.curve_policy())
                .map_err(|error| ValidationError::Geometry(error.to_string()))
                .and_then(resolve);
        }

        match evaluate(&CurvePolicy::STRICT)
            .map_err(|error| ValidationError::Geometry(error.to_string()))?
        {
            Classification::Decided(value) => Ok(value),
            Classification::Uncertain(_) => {
                let value = evaluate(&self.curve_policy)
                    .map_err(|error| ValidationError::Geometry(error.to_string()))
                    .and_then(resolve)?;
                self.observe(GeometryCertainty::Approximate512Consumed);
                Ok(value)
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn contexts_and_certainty_remain_one_byte() {
        assert_eq!(core::mem::size_of::<GeometryContext>(), 1);
        assert_eq!(core::mem::size_of::<GeometryCertainty>(), 1);
    }

    #[test]
    fn one_context_derives_matching_curve_and_mesh_policies() {
        assert_eq!(GeometryContext::STRICT.curve_policy(), CurvePolicy::STRICT);
        assert_eq!(
            GeometryContext::APPROXIMATE_512.curve_policy(),
            CurvePolicy::APPROXIMATE_512
        );
        assert_eq!(
            GeometryContext::STRICT.mesh_context().predicate_policy(),
            PredicatePolicy::STRICT
        );
        assert_eq!(
            GeometryContext::APPROXIMATE_512
                .mesh_context()
                .predicate_policy(),
            PredicatePolicy::APPROXIMATE_512
        );
    }
}
