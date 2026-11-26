//! Immutable operation policy and aggregate predicate certainty.

use std::cell::Cell;

use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy};
use hypermesh::{MeshCertainty, MeshContext, MeshOutcome};
use hypertri::{TriangulationCertainty, TriangulationContext, TriangulationOutcome};

use crate::errors::ValidationError;

/// Immutable predicate policy selected for one CSG operation.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct GeometryContext {
    /// Policy forwarded unchanged to each participating geometry kernel.
    predicates: PredicatePolicy,
}

impl GeometryContext {
    /// Require certified predicate decisions; unresolved predicates remain failures.
    pub const STRICT: Self = Self::new(PredicatePolicy::STRICT);

    /// Permit Hyperlimit's terminal 512-bit interpretation when proof is unavailable.
    ///
    /// Selecting this policy alone does not make a receipt approximate; consuming
    /// the terminal does. It does not independently certify topology or fidelity.
    pub const APPROXIMATE_512: Self = Self::new(PredicatePolicy::APPROXIMATE_512);

    /// Construct a context with the selected Hyperlimit predicate policy.
    pub const fn new(predicates: PredicatePolicy) -> Self {
        Self { predicates }
    }

    /// Return the selected predicate policy.
    pub const fn predicate_policy(self) -> PredicatePolicy {
        self.predicates
    }

    /// Derive the matching Hypermesh context.
    pub const fn mesh_context(self) -> MeshContext {
        MeshContext::new(self.predicates)
    }

    /// Derive the matching Hypertri context.
    pub const fn triangulation_context(self) -> TriangulationContext {
        TriangulationContext::new(self.predicates)
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

impl GeometryCertainty {
    /// The certainty of a claim that depends on both receipts: approximate if
    /// either consumed the terminal.
    pub const fn combine(self, other: Self) -> Self {
        match (self, other) {
            (Self::Certified, Self::Certified) => Self::Certified,
            _ => Self::Approximate512Consumed,
        }
    }
}

/// Completed CSG value paired with operation-local predicate certainty.
///
/// The receipt covers the decisions of the operation that produced `value`.
/// For meshes, inherited construction qualifications are separately queryable
/// through [`hypermesh::TriangleMesh::construction_evidence`]. A certified local
/// receipt does not clear an earlier approximate construction or missing history.
/// Curve regions and query values carry no inherited qualification, so a
/// caller that needs the certainty of a whole chain keeps it with
/// [`Self::and_then`] or [`GeometryCertainty::combine`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GeometryOutcome<T> {
    /// Completed value.
    pub value: T,
    /// Weakest certainty consumed while producing `value`.
    pub certainty: GeometryCertainty,
}

impl<T> GeometryOutcome<T> {
    /// Pair a completed value with its supplied receipt without validating either.
    const fn new(value: T, certainty: GeometryCertainty) -> Self {
        Self { value, certainty }
    }

    /// Transform the value without changing aggregate certainty.
    ///
    /// The closure's own predicate decisions are not recorded. Use
    /// [`Self::and_then`] for an operation that returns its own outcome.
    pub fn map<U>(self, map: impl FnOnce(T) -> U) -> GeometryOutcome<U> {
        GeometryOutcome::new(map(self.value), self.certainty)
    }

    /// Fallibly transform the value without changing aggregate certainty.
    ///
    /// As with [`Self::map`], the closure's own decisions are not recorded.
    pub fn try_map<U, E>(
        self,
        map: impl FnOnce(T) -> Result<U, E>,
    ) -> Result<GeometryOutcome<U>, E> {
        map(self.value).map(|value| GeometryOutcome::new(value, self.certainty))
    }

    /// Run a dependent operation on the value and combine both receipts.
    ///
    /// The result is approximate if either this outcome or the dependent
    /// operation consumed the terminal; a certified later step cannot certify
    /// an earlier approximate one. A failed dependent operation returns its
    /// error.
    pub fn and_then<U, E>(
        self,
        operation: impl FnOnce(T) -> Result<GeometryOutcome<U>, E>,
    ) -> Result<GeometryOutcome<U>, E> {
        let certainty = self.certainty;
        operation(self.value).map(|outcome| {
            GeometryOutcome::new(outcome.value, certainty.combine(outcome.certainty))
        })
    }

    /// Consume the outcome and discard its operation-local certainty receipt.
    ///
    /// Retain or combine `certainty` first when subsequent work needs that claim.
    pub fn into_value(self) -> T {
        self.value
    }
}

// Feature-minimal builds retain the public context and mesh accumulator while
// curve/predicate consumers are compiled out.
/// Operation-local bridge between kernel receipts and CSG outcomes.
///
/// Interior mutation accumulates the weakest consumed predicate certainty and
/// mesh construction qualifications separately. Callers must record dependencies;
/// this accumulator neither discovers missing history nor validates geometry.
#[allow(dead_code)]
pub(crate) struct GeometryDecisions {
    /// Policy chosen by the enclosing modeling operation.
    context: GeometryContext,
    /// Matching mesh policy retained for borrowed kernel calls.
    mesh_context: MeshContext,
    /// Monotone summary of receipts explicitly consumed by this accumulator.
    certainty: Cell<GeometryCertainty>,
    /// Inherited mesh qualifications, independent of local predicate certainty.
    construction: Cell<hypermesh::ConstructionEvidence>,
}

#[allow(dead_code)]
impl GeometryDecisions {
    /// Start with no consumed approximate decisions and declared-coordinate history.
    ///
    /// Input mesh qualifications must subsequently be added with
    /// [`Self::record_construction`].
    pub(crate) const fn new(context: GeometryContext) -> Self {
        Self {
            context,
            mesh_context: context.mesh_context(),
            certainty: Cell::new(GeometryCertainty::Certified),
            construction: Cell::new(hypermesh::ConstructionEvidence::DECLARED_COORDINATES),
        }
    }

    /// Return the policy used for scalar and curve decisions.
    pub(crate) const fn predicate_policy(&self) -> PredicatePolicy {
        self.context.predicate_policy()
    }

    /// Borrow the matching mesh context without changing accumulated receipts.
    pub(crate) const fn mesh_context(&self) -> &MeshContext {
        &self.mesh_context
    }

    /// Create a triangulation context using the same predicate policy.
    pub(crate) const fn triangulation_context(&self) -> TriangulationContext {
        self.context.triangulation_context()
    }

    /// Snapshot the local receipt alongside a value, without attaching mesh history.
    ///
    /// This does not reset the accumulator. Use [`Self::finish_mesh`] when
    /// construction qualifications must be attached to a mesh.
    pub(crate) const fn finish<T>(&self, value: T) -> GeometryOutcome<T> {
        GeometryOutcome::new(value, self.certainty.get())
    }

    /// Merge dependency qualifications without changing local predicate certainty.
    pub(crate) fn record_construction(&self, evidence: hypermesh::ConstructionEvidence) {
        self.construction
            .set(self.construction.get().combine(evidence));
    }

    /// Attach accumulated history and local predicate certainty to a mesh.
    ///
    /// Existing mesh qualifications remain; this neither validates topology nor
    /// upgrades missing history to a certificate.
    pub(crate) fn qualify_mesh(
        &self,
        mesh: hypermesh::TriangleMesh,
    ) -> hypermesh::TriangleMesh {
        let certainty = match self.certainty.get() {
            GeometryCertainty::Certified => MeshCertainty::Certified,
            GeometryCertainty::Approximate512Consumed => MeshCertainty::Approximate512Consumed,
        };
        mesh.with_inherited_construction(
            self.construction.get().with_predicate_certainty(certainty),
        )
    }

    /// Attach construction qualifications and return the mesh with its local receipt.
    pub(crate) fn finish_mesh(
        &self,
        mesh: hypermesh::TriangleMesh,
    ) -> GeometryOutcome<hypermesh::TriangleMesh> {
        self.finish(self.qualify_mesh(mesh))
    }

    /// Folds a nested operation's receipt into this operation and returns
    /// its value.
    pub(crate) fn record<T>(&self, outcome: GeometryOutcome<T>) -> T {
        self.observe(outcome.certainty);
        outcome.value
    }

    /// Record approximate consumption irreversibly for this accumulator.
    pub(crate) fn observe(&self, certainty: GeometryCertainty) {
        if certainty == GeometryCertainty::Approximate512Consumed {
            self.certainty.set(certainty);
        }
    }

    /// Runs principal exact Hypercurve operations under this context's
    /// predicate policy through [`hypercurve::evaluate_under`], recording any
    /// approximate terminal they consumed.
    pub(crate) fn exact_curve<T>(&self, operation: impl FnOnce() -> T) -> T {
        let provisional = hypercurve::evaluate_under(self.predicate_policy(), operation);
        if !provisional.is_certified() {
            self.observe(GeometryCertainty::Approximate512Consumed);
        }
        provisional.into_unverified()
    }

    /// Extract a mesh-kernel value while retaining its predicate receipt.
    ///
    /// This generic helper does not inspect or merge construction history.
    pub(crate) fn consume_mesh<T>(&self, outcome: MeshOutcome<T>) -> T {
        if outcome.certainty == MeshCertainty::Approximate512Consumed {
            self.observe(GeometryCertainty::Approximate512Consumed);
        }
        outcome.value
    }

    /// Extract a nested CSG value and merge its local predicate certainty.
    pub(crate) fn consume_geometry<T>(&self, outcome: GeometryOutcome<T>) -> T {
        self.observe(outcome.certainty);
        outcome.value
    }

    /// Extract a triangulation value and merge its predicate certainty.
    pub(crate) fn consume_triangulation<T>(&self, outcome: TriangulationOutcome<T>) -> T {
        if outcome.certainty == TriangulationCertainty::Approximate512Consumed {
            self.observe(GeometryCertainty::Approximate512Consumed);
        }
        outcome.value
    }

    /// Classify a real sign under the selected policy and retain consumed certainty.
    ///
    /// An unresolved sign becomes `PredicateUndecided`, labeled by `predicate`.
    pub(crate) fn sign(
        &self,
        value: &hyperreal::Real,
        predicate: &'static str,
    ) -> Result<hyperreal::RealSign, ValidationError> {
        self.decide(
            hyperlimit::classify_real_sign(value, self.predicate_policy()),
            predicate,
        )
        .map(|sign| match sign {
            hyperlimit::Sign::Negative => hyperreal::RealSign::Negative,
            hyperlimit::Sign::Zero => hyperreal::RealSign::Zero,
            hyperlimit::Sign::Positive => hyperreal::RealSign::Positive,
        })
    }

    /// Decide coordinate equality of two native 3D points under this policy.
    ///
    /// Retains consumed certainty; unresolved equality becomes
    /// `PredicateUndecided` rather than `false`.
    pub(crate) fn points_equal(
        &self,
        left: &hyperlattice::Point3,
        right: &hyperlattice::Point3,
    ) -> Result<bool, ValidationError> {
        let left = hyperlimit::Point3::new(left.x.clone(), left.y.clone(), left.z.clone());
        let right = hyperlimit::Point3::new(right.x.clone(), right.y.clone(), right.z.clone());
        self.decide(
            hyperlimit::point3_equal(&left, &right, self.predicate_policy()),
            "point equality",
        )
    }

    /// Consume a decided predicate and record its certainty, or report uncertainty.
    ///
    /// Unknown outcomes become `PredicateUndecided` with the caller label and
    /// current policy; the underlying unknown reason is not retained here.
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
            PredicateOutcome::Unknown { .. } => Err(ValidationError::PredicateUndecided {
                predicate,
                policy: self.context.predicate_policy(),
            }),
        }
    }

    /// Try an optional predicate, recording decided certainty and mapping unknown to
    /// `None`. Absence is not a negative geometric classification.
    pub(crate) fn probe<T>(&self, outcome: PredicateOutcome<T>) -> Option<T> {
        self.decide(outcome, "optional geometry predicate").ok()
    }
}

/// Regression checks for context layout, policy forwarding, and receipt composition.
#[cfg(test)]
mod tests {
    use super::*;

    /// Guards the compact policy and receipt representation.
    #[test]
    fn contexts_and_certainty_remain_one_byte() {
        assert_eq!(core::mem::size_of::<GeometryContext>(), 1);
        assert_eq!(core::mem::size_of::<GeometryCertainty>(), 1);
    }

    /// Checks every receipt pair and propagation of a dependent failure.
    #[test]
    fn dependent_receipts_combine_to_the_weakest_certainty() {
        use GeometryCertainty::{Approximate512Consumed as Approximate, Certified};
        for (first, second, combined) in [
            (Certified, Certified, Certified),
            (Certified, Approximate, Approximate),
            (Approximate, Certified, Approximate),
            (Approximate, Approximate, Approximate),
        ] {
            assert_eq!(first.combine(second), combined);
            let chained = GeometryOutcome::new(1, first)
                .and_then(|value| Ok::<_, ()>(GeometryOutcome::new(value + 1, second)))
                .unwrap();
            assert_eq!(chained, GeometryOutcome::new(2, combined));
            assert_eq!(
                GeometryOutcome::new(1, first)
                    .map(|value| value + 1)
                    .certainty,
                first
            );
        }
        assert_eq!(
            GeometryOutcome::new(1, Approximate)
                .and_then(|_| Err::<GeometryOutcome<i32>, _>("failed")),
            Err("failed")
        );
    }

    /// Ensures mesh and triangulation contexts receive the selected policy.
    #[test]
    fn one_context_derives_matching_kernel_policies() {
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
        for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
            assert_eq!(
                context.triangulation_context().predicate_policy(),
                context.predicate_policy()
            );
        }
    }
}
