//! Predicate-controlled topology and explicit edge-preview policy.

use std::cell::Cell;
use std::sync::OnceLock;

use crate::{Classification, UncertaintyReason};

/// Numeric policy for a curve operation.
///
/// [`CurvePolicy::STRICT`] accepts only exact or certified-refinement
/// predicate decisions. [`CurvePolicy::APPROXIMATE_512`] additionally permits
/// Hyperlimit's terminal 512-bit interpretation. The named edge-preview
/// constructors permit lossy finite views only at rendering and diagnostics
/// boundaries.
///
/// The representation is intentionally closed: callers cannot combine a
/// certified operation with preview tolerances or request preview behavior
/// without tolerances.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePolicy {
    pub(crate) mode: NumericMode,
    #[cfg(feature = "predicates")]
    pub(crate) predicate_policy: hyperlimit::PredicatePolicy,
    pub(crate) preview_tolerance: Option<PreviewTolerance>,
}

std::thread_local! {
    /// Certainty observation for the innermost composite operation on this thread.
    ///
    /// Frames save and restore the bit, so nested operations report their own
    /// certainty and propagate every consumed terminal to their caller.
    /// Predicate work is currently synchronous; parallel kernels must carry
    /// the observation frame explicitly rather than relying on this scope.
    static APPROXIMATE_512_CONSUMED: Cell<bool> = const { Cell::new(false) };
}

struct OperationObservation {
    prior: bool,
    active: bool,
}

impl OperationObservation {
    fn begin() -> Self {
        Self {
            prior: APPROXIMATE_512_CONSUMED.with(|consumed| consumed.replace(false)),
            active: true,
        }
    }

    fn finish(mut self) -> CurveCertainty {
        let consumed = APPROXIMATE_512_CONSUMED.with(Cell::get);
        self.restore(consumed);
        if consumed {
            CurveCertainty::Approximate512Consumed
        } else {
            CurveCertainty::Certified
        }
    }

    fn restore(&mut self, consumed: bool) {
        if self.active {
            APPROXIMATE_512_CONSUMED.with(|current| current.set(self.prior || consumed));
            self.active = false;
        }
    }
}

impl Drop for OperationObservation {
    fn drop(&mut self) {
        let consumed = APPROXIMATE_512_CONSUMED.with(Cell::get);
        self.restore(consumed);
    }
}

/// Aggregate certainty consumed by a completed curve operation.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum CurveCertainty {
    /// Every topology decision was exact or certified.
    Certified,
    /// At least one decision consumed Hyperlimit's policy-authorized 512-bit terminal.
    Approximate512Consumed,
}

/// A completed curve operation paired with its aggregate predicate certainty.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CurveOutcome<T> {
    /// Completed operation value.
    pub value: T,
    /// Weakest certainty consumed while producing `value`.
    pub certainty: CurveCertainty,
}

impl<T> CurveOutcome<T> {
    pub(crate) const fn new(value: T, certainty: CurveCertainty) -> Self {
        Self { value, certainty }
    }

    /// Transform the completed value without changing its certainty.
    pub fn map<U>(self, map: impl FnOnce(T) -> U) -> CurveOutcome<U> {
        CurveOutcome::new(map(self.value), self.certainty)
    }

    /// Consume the outcome and return its value.
    pub fn into_value(self) -> T {
        self.value
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum NumericMode {
    EdgePreview,
    Certified,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PreviewTolerance {
    pub(crate) absolute: f64,
    pub(crate) relative: f64,
}

impl CurvePolicy {
    /// Topology accepts only certified predicate decisions.
    pub const STRICT: Self = Self {
        mode: NumericMode::Certified,
        #[cfg(feature = "predicates")]
        predicate_policy: hyperlimit::PredicatePolicy::STRICT,
        preview_tolerance: None,
    };

    /// Topology may consume Hyperlimit's terminal 512-bit interpretation.
    pub const APPROXIMATE_512: Self = Self {
        mode: NumericMode::Certified,
        #[cfg(feature = "predicates")]
        predicate_policy: hyperlimit::PredicatePolicy::APPROXIMATE_512,
        preview_tolerance: None,
    };

    /// Strict-predicate edge preview for diagnostics and exploratory rendering.
    ///
    /// The tolerances are available only to named curve-local preview
    /// operations.
    pub const fn edge_preview_strict(absolute_tolerance: f64, relative_tolerance: f64) -> Self {
        Self {
            mode: NumericMode::EdgePreview,
            #[cfg(feature = "predicates")]
            predicate_policy: hyperlimit::PredicatePolicy::STRICT,
            preview_tolerance: Some(PreviewTolerance {
                absolute: absolute_tolerance,
                relative: relative_tolerance,
            }),
        }
    }

    /// Approximate-512-predicate edge preview for diagnostics and rendering.
    pub const fn edge_preview_approximate_512(
        absolute_tolerance: f64,
        relative_tolerance: f64,
    ) -> Self {
        Self {
            mode: NumericMode::EdgePreview,
            #[cfg(feature = "predicates")]
            predicate_policy: hyperlimit::PredicatePolicy::APPROXIMATE_512,
            preview_tolerance: Some(PreviewTolerance {
                absolute: absolute_tolerance,
                relative: relative_tolerance,
            }),
        }
    }

    /// Return the selected Hyperlimit predicate policy.
    #[cfg(feature = "predicates")]
    pub const fn predicate_policy(&self) -> hyperlimit::PredicatePolicy {
        self.predicate_policy
    }

    pub(crate) fn permits_approximate_512(&self) -> bool {
        #[cfg(feature = "predicates")]
        {
            self.predicate_policy == hyperlimit::PredicatePolicy::APPROXIMATE_512
        }
        #[cfg(not(feature = "predicates"))]
        {
            false
        }
    }

    fn observe_approximate_512(&self) {
        APPROXIMATE_512_CONSUMED.with(|consumed| consumed.set(true));
    }

    #[cfg(feature = "predicates")]
    pub(crate) fn consume_predicate<T>(
        &self,
        outcome: hyperlimit::PredicateOutcome<T>,
    ) -> Option<T> {
        match outcome {
            hyperlimit::PredicateOutcome::Decided {
                value, certainty, ..
            } => {
                if certainty == hyperlimit::Certainty::Approximate {
                    self.observe_approximate_512();
                }
                Some(value)
            }
            hyperlimit::PredicateOutcome::Unknown { .. } => None,
        }
    }

    pub(crate) const fn strict_counterpart(&self) -> Self {
        match (self.mode, self.preview_tolerance) {
            (NumericMode::Certified, _) => Self::STRICT,
            (NumericMode::EdgePreview, Some(tolerance)) => {
                Self::edge_preview_strict(tolerance.absolute, tolerance.relative)
            }
            (NumericMode::EdgePreview, None) => Self::STRICT,
        }
    }
}

/// Clone-shared carrier cache with separate certified and approximate slots.
///
/// A successful strict computation may answer either policy. An
/// approximate-512 computation retains the strict blocker beside its value so
/// a later strict operation cannot consume the weaker fact. Separate
/// `OnceLock`s permit independently obtained certified construction evidence
/// to upgrade the cache without replacing or synchronizing mutable carrier
/// state.
pub(crate) struct PolicyClassificationCache<T> {
    certified: OnceLock<T>,
    approximate_512: OnceLock<(T, UncertaintyReason)>,
}

impl<T> PolicyClassificationCache<T> {
    pub(crate) const fn new() -> Self {
        Self {
            certified: OnceLock::new(),
            approximate_512: OnceLock::new(),
        }
    }

    pub(crate) fn certify(&self, value: T) {
        let _ = self.certified.set(value);
    }

    pub(crate) fn certified(&self) -> Option<&T> {
        self.certified.get()
    }

    #[cfg(test)]
    pub(crate) fn is_empty(&self) -> bool {
        self.certified.get().is_none() && self.approximate_512.get().is_none()
    }
}

pub(crate) fn resolve_cached_classification<'a, T, E>(
    cache: &'a PolicyClassificationCache<T>,
    policy: &CurvePolicy,
    mut evaluate: impl FnMut(&CurvePolicy) -> Result<Classification<T>, E>,
) -> Result<Classification<&'a T>, E> {
    if let Some(value) = cache.certified.get() {
        return Ok(Classification::Decided(value));
    }

    if policy.permits_approximate_512() {
        if let Some((value, _)) = cache.approximate_512.get() {
            policy.observe_approximate_512();
            return Ok(Classification::Decided(value));
        }

        match evaluate(&policy.strict_counterpart())? {
            Classification::Decided(value) => {
                let _ = cache.certified.set(value);
            }
            Classification::Uncertain(strict_reason) => match evaluate(policy)? {
                Classification::Decided(value) => {
                    let _ = cache.approximate_512.set((value, strict_reason));
                    policy.observe_approximate_512();
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        }
    } else {
        if let Some((_, strict_reason)) = cache.approximate_512.get() {
            return Ok(Classification::Uncertain(*strict_reason));
        }
        match evaluate(policy)? {
            Classification::Decided(value) => {
                let _ = cache.certified.set(value);
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }

    if let Some(value) = cache.certified.get() {
        Ok(Classification::Decided(value))
    } else if policy.permits_approximate_512() {
        Ok(Classification::Decided(
            &cache
                .approximate_512
                .get()
                .expect("a decided approximate classification was retained")
                .0,
        ))
    } else {
        unreachable!("a decided strict classification was retained")
    }
}

/// Run one composite operation while aggregating every consumed predicate.
///
/// `APPROXIMATE_512` follows the same operation once. Hyperlimit performs its
/// certified pipeline before any terminal interpretation, and only a terminal
/// decision actually consumed by the operation weakens the returned certainty.
pub(crate) fn resolve_certified_operation<T>(
    policy: &CurvePolicy,
    mut evaluate: impl FnMut(&CurvePolicy) -> crate::ExactCurveResult<T>,
) -> crate::ExactCurveResult<CurveOutcome<T>> {
    let observation = OperationObservation::begin();
    evaluate(policy).map(|value| CurveOutcome::new(value, observation.finish()))
}

#[cfg(all(test, feature = "predicates"))]
mod tests {
    use std::cell::Cell;

    use hyperreal::{Real, RealSign};

    use super::{
        CurveCertainty, CurvePolicy, PolicyClassificationCache, resolve_cached_classification,
        resolve_certified_operation,
    };
    use crate::{Classification, UncertaintyReason};

    #[test]
    fn curve_modes_name_both_hyperlimit_policies() {
        fn assert_send_sync<T: Send + Sync>() {}

        assert_send_sync::<CurvePolicy>();
        assert_eq!(core::mem::size_of::<CurvePolicy>(), 32);
        assert_eq!(core::mem::size_of::<super::CurveCertainty>(), 1);
        assert_eq!(
            CurvePolicy::STRICT.predicate_policy,
            hyperlimit::PredicatePolicy::STRICT
        );
        assert_eq!(
            CurvePolicy::APPROXIMATE_512.predicate_policy,
            hyperlimit::PredicatePolicy::APPROXIMATE_512
        );
        assert_eq!(
            CurvePolicy::edge_preview_strict(1.0e-6, 1.0e-6).predicate_policy,
            hyperlimit::PredicatePolicy::STRICT
        );
        assert_eq!(
            CurvePolicy::edge_preview_approximate_512(1.0e-6, 1.0e-6).predicate_policy,
            hyperlimit::PredicatePolicy::APPROXIMATE_512
        );
    }

    #[test]
    fn approximate_operation_runs_once_and_records_only_a_consumed_terminal() {
        let calls = Cell::new(0_u8);
        let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
        let outcome = resolve_certified_operation(&CurvePolicy::APPROXIMATE_512, |operation| {
            calls.set(calls.get() + 1);
            assert_eq!(
                crate::classify::real_sign(&undecidable_zero, operation),
                Some(RealSign::Zero)
            );
            Ok(())
        })
        .unwrap();
        assert_eq!(calls.get(), 1);
        assert_eq!(outcome.certainty, CurveCertainty::Approximate512Consumed);

        let certified = resolve_certified_operation(&CurvePolicy::APPROXIMATE_512, |operation| {
            assert_eq!(
                crate::classify::real_sign(&Real::one(), operation),
                Some(RealSign::Positive)
            );
            Ok(())
        })
        .unwrap();
        assert_eq!(certified.certainty, CurveCertainty::Certified);

        let nested = resolve_certified_operation(&CurvePolicy::APPROXIMATE_512, |outer_policy| {
            assert_eq!(
                crate::classify::real_sign(&undecidable_zero, outer_policy),
                Some(RealSign::Zero)
            );
            let inner =
                resolve_certified_operation(&CurvePolicy::APPROXIMATE_512, |inner_policy| {
                    assert_eq!(
                        crate::classify::real_sign(&Real::one(), inner_policy),
                        Some(RealSign::Positive)
                    );
                    Ok(())
                })?;
            assert_eq!(inner.certainty, CurveCertainty::Certified);
            Ok(())
        })
        .unwrap();
        assert_eq!(nested.certainty, CurveCertainty::Approximate512Consumed);

        let propagated =
            resolve_certified_operation(&CurvePolicy::APPROXIMATE_512, |_outer_policy| {
                let inner =
                    resolve_certified_operation(&CurvePolicy::APPROXIMATE_512, |inner_policy| {
                        assert_eq!(
                            crate::classify::real_sign(&undecidable_zero, inner_policy),
                            Some(RealSign::Zero)
                        );
                        Ok(())
                    })?;
                assert_eq!(inner.certainty, CurveCertainty::Approximate512Consumed);
                Ok(())
            })
            .unwrap();
        assert_eq!(propagated.certainty, CurveCertainty::Approximate512Consumed);
    }

    #[test]
    fn approximate_cached_classification_never_answers_strict() {
        let cache = PolicyClassificationCache::new();
        let approximate =
            resolve_cached_classification(&cache, &CurvePolicy::APPROXIMATE_512, |policy| {
                Ok::<_, ()>(if policy == &CurvePolicy::STRICT {
                    Classification::Uncertain(UncertaintyReason::Predicate)
                } else {
                    Classification::Decided(7_u8)
                })
            })
            .unwrap();
        assert_eq!(approximate, Classification::Decided(&7));

        let strict = resolve_cached_classification(
            &cache,
            &CurvePolicy::STRICT,
            |_| -> Result<Classification<u8>, ()> {
                panic!("the retained strict blocker should be reused")
            },
        )
        .unwrap();
        assert_eq!(
            strict,
            Classification::Uncertain(UncertaintyReason::Predicate)
        );

        cache.certify(8);
        let upgraded = resolve_cached_classification(
            &cache,
            &CurvePolicy::STRICT,
            |_| -> Result<Classification<u8>, ()> {
                panic!("explicit certified evidence should supersede the approximate fact")
            },
        )
        .unwrap();
        assert_eq!(upgraded, Classification::Decided(&8));
    }
}
