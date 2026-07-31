//! Predicate-controlled topology and explicit edge-preview policy.

use std::cell::Cell;
use std::sync::OnceLock;

use crate::{Classification, UncertaintyReason};

/// Immutable predicate policy for a curve operation.
///
/// [`CurveContext::STRICT`] accepts only exact or certified-refinement
/// predicate decisions. [`CurveContext::APPROXIMATE_512`] additionally permits
/// Hyperlimit's terminal 512-bit interpretation.
///
/// Rendering tolerances are intentionally absent. Lossy inspection uses
/// [`CurvePreviewOptions`] and cannot enlarge the topology context.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct CurveContext(u8);

const APPROXIMATE_512_CONTEXT: u8 = 1 << 0;
const EDGE_PREVIEW_CONTEXT: u8 = 1 << 1;

std::thread_local! {
    /// Certainty observation for the innermost composite operation on this thread.
    ///
    /// Frames save and restore the bit, so nested operations report their own
    /// certainty and propagate every consumed terminal to their caller.
    /// Predicate work is currently synchronous; parallel kernels must carry
    /// the observation frame explicitly rather than relying on this scope.
    static APPROXIMATE_512_CONSUMED: Cell<bool> = const { Cell::new(false) };

    /// Lossy tolerances scoped to an explicit preview adapter.
    static ACTIVE_PREVIEW_TOLERANCE: Cell<Option<PreviewTolerance>> = const { Cell::new(None) };
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

struct PreviewFrame {
    prior: Option<PreviewTolerance>,
    active: bool,
}

impl PreviewFrame {
    fn begin(tolerance: PreviewTolerance) -> Self {
        Self {
            prior: ACTIVE_PREVIEW_TOLERANCE.with(|active| active.replace(Some(tolerance))),
            active: true,
        }
    }

    fn restore(&mut self) {
        if self.active {
            ACTIVE_PREVIEW_TOLERANCE.with(|active| active.set(self.prior));
            self.active = false;
        }
    }
}

impl Drop for PreviewFrame {
    fn drop(&mut self) {
        self.restore();
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
pub(crate) struct PreviewTolerance {
    pub(crate) absolute: f64,
    pub(crate) relative: f64,
}

/// Explicit lossy edge-preview adapter.
///
/// Preview tolerances may recover finite display evidence from rounded input,
/// but they never authorize exact topology or replacement geometry. The
/// adapter scopes its tolerances only for the synchronous operation passed to
/// [`Self::evaluate`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurvePreviewOptions {
    context: CurveContext,
    tolerance: PreviewTolerance,
}

impl CurvePreviewOptions {
    /// Construct validated preview options around an immutable topology context.
    pub fn try_new(
        context: CurveContext,
        absolute_tolerance: f64,
        relative_tolerance: f64,
    ) -> crate::CurveResult<Self> {
        if !absolute_tolerance.is_finite()
            || !relative_tolerance.is_finite()
            || absolute_tolerance < 0.0
            || relative_tolerance < 0.0
        {
            return Err(crate::CurveError::InvalidPreviewOptions);
        }
        Ok(Self {
            context,
            tolerance: PreviewTolerance {
                absolute: absolute_tolerance,
                relative: relative_tolerance,
            },
        })
    }

    /// Construct strict-predicate preview options.
    pub fn try_strict(
        absolute_tolerance: f64,
        relative_tolerance: f64,
    ) -> crate::CurveResult<Self> {
        Self::try_new(CurveContext::STRICT, absolute_tolerance, relative_tolerance)
    }

    /// Construct Approximate-512-predicate preview options.
    pub fn try_approximate_512(
        absolute_tolerance: f64,
        relative_tolerance: f64,
    ) -> crate::CurveResult<Self> {
        Self::try_new(
            CurveContext::APPROXIMATE_512,
            absolute_tolerance,
            relative_tolerance,
        )
    }

    /// Return the immutable predicate context used by this adapter.
    pub const fn context(self) -> CurveContext {
        self.context
    }

    /// Return the absolute finite preview tolerance.
    pub const fn absolute_tolerance(self) -> f64 {
        self.tolerance.absolute
    }

    /// Return the relative finite preview tolerance.
    pub const fn relative_tolerance(self) -> f64 {
        self.tolerance.relative
    }

    /// Evaluate one synchronous lossy preview operation.
    ///
    /// The returned value is preview evidence. It must not be retained as
    /// certified topology or exact construction provenance.
    pub fn evaluate<T>(&self, evaluate: impl FnOnce(&CurveContext) -> T) -> T {
        let _frame = PreviewFrame::begin(self.tolerance);
        evaluate(&self.context.with_edge_preview())
    }
}

#[cold]
#[inline(never)]
pub(crate) fn preview_tolerance() -> Option<PreviewTolerance> {
    ACTIVE_PREVIEW_TOLERANCE.with(Cell::get)
}

impl CurveContext {
    /// Topology accepts only certified predicate decisions.
    pub const STRICT: Self = Self(0);

    /// Topology may consume Hyperlimit's terminal 512-bit interpretation.
    pub const APPROXIMATE_512: Self = Self(APPROXIMATE_512_CONTEXT);

    /// Return the selected Hyperlimit predicate policy.
    #[cfg(feature = "predicates")]
    pub const fn predicate_policy(self) -> hyperlimit::PredicatePolicy {
        if self.0 & APPROXIMATE_512_CONTEXT == 0 {
            hyperlimit::PredicatePolicy::STRICT
        } else {
            hyperlimit::PredicatePolicy::APPROXIMATE_512
        }
    }

    pub(crate) fn permits_approximate_512(&self) -> bool {
        #[cfg(feature = "predicates")]
        {
            self.0 & APPROXIMATE_512_CONTEXT != 0
        }
        #[cfg(not(feature = "predicates"))]
        {
            false
        }
    }

    const fn with_edge_preview(self) -> Self {
        Self(self.0 | EDGE_PREVIEW_CONTEXT)
    }

    #[inline]
    pub(crate) fn is_edge_preview(&self) -> bool {
        self.0 & EDGE_PREVIEW_CONTEXT != 0 && preview_tolerance().is_some()
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
        Self(self.0 & EDGE_PREVIEW_CONTEXT)
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
    policy: &CurveContext,
    mut evaluate: impl FnMut(&CurveContext) -> Result<Classification<T>, E>,
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
    policy: &CurveContext,
    evaluate: impl FnOnce(&CurveContext) -> crate::ExactCurveResult<T>,
) -> crate::ExactCurveResult<CurveOutcome<T>> {
    let observation = OperationObservation::begin();
    evaluate(policy).map(|value| CurveOutcome::new(value, observation.finish()))
}

#[cfg(test)]
mod layout_tests {
    use super::{CurveCertainty, CurveContext};

    #[test]
    fn curve_context_and_certainty_are_one_byte() {
        fn assert_send_sync<T: Send + Sync>() {}

        assert_send_sync::<CurveContext>();
        assert_eq!(core::mem::size_of::<CurveContext>(), 1);
        assert_eq!(core::mem::size_of::<CurveCertainty>(), 1);
    }
}

#[cfg(all(test, feature = "predicates"))]
mod tests {
    use std::cell::Cell;

    use hyperreal::{Real, RealSign};

    use super::{
        CurveCertainty, CurveContext, CurvePreviewOptions, PolicyClassificationCache,
        preview_tolerance, resolve_cached_classification, resolve_certified_operation,
    };
    use crate::{Classification, UncertaintyReason};

    #[test]
    fn curve_modes_name_both_hyperlimit_policies() {
        assert_eq!(
            CurveContext::STRICT.predicate_policy(),
            hyperlimit::PredicatePolicy::STRICT
        );
        assert_eq!(
            CurveContext::APPROXIMATE_512.predicate_policy(),
            hyperlimit::PredicatePolicy::APPROXIMATE_512
        );

        let strict_preview = CurvePreviewOptions::try_strict(1.0e-6, 2.0e-6).unwrap();
        assert_eq!(
            strict_preview.context().predicate_policy(),
            hyperlimit::PredicatePolicy::STRICT
        );
        assert_eq!(preview_tolerance(), None);
        let escaped = strict_preview.evaluate(|context| {
            assert!(context.is_edge_preview());
            assert_eq!(
                context.predicate_policy(),
                hyperlimit::PredicatePolicy::STRICT
            );
            assert_eq!(preview_tolerance().unwrap().absolute, 1.0e-6);
            assert_eq!(preview_tolerance().unwrap().relative, 2.0e-6);
            *context
        });
        assert_eq!(preview_tolerance(), None);
        assert!(!escaped.is_edge_preview());

        assert_eq!(
            CurvePreviewOptions::try_approximate_512(1.0e-6, 1.0e-6)
                .unwrap()
                .context()
                .predicate_policy(),
            hyperlimit::PredicatePolicy::APPROXIMATE_512
        );
        assert_eq!(
            CurvePreviewOptions::try_strict(f64::NAN, 0.0),
            Err(crate::CurveError::InvalidPreviewOptions)
        );
    }

    #[test]
    fn preview_tolerances_are_nested_and_unwind_safe() {
        let outer = CurvePreviewOptions::try_strict(1.0e-6, 2.0e-6).unwrap();
        let inner = CurvePreviewOptions::try_strict(3.0e-6, 4.0e-6).unwrap();

        outer.evaluate(|outer_context| {
            assert!(outer_context.is_edge_preview());
            assert_eq!(preview_tolerance().unwrap().absolute, 1.0e-6);
            inner.evaluate(|inner_context| {
                assert!(inner_context.is_edge_preview());
                assert_eq!(preview_tolerance().unwrap().absolute, 3.0e-6);
            });
            assert_eq!(preview_tolerance().unwrap().absolute, 1.0e-6);
        });
        assert_eq!(preview_tolerance(), None);

        let panic = std::panic::catch_unwind(|| {
            outer.evaluate(|context| {
                assert!(context.is_edge_preview());
                panic!("preview unwind sentinel");
            });
        });
        assert!(panic.is_err());
        assert_eq!(preview_tolerance(), None);
    }

    #[test]
    fn approximate_operation_runs_once_and_records_only_a_consumed_terminal() {
        let calls = Cell::new(0_u8);
        let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
        let outcome = resolve_certified_operation(&CurveContext::APPROXIMATE_512, |operation| {
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

        let certified = resolve_certified_operation(&CurveContext::APPROXIMATE_512, |operation| {
            assert_eq!(
                crate::classify::real_sign(&Real::one(), operation),
                Some(RealSign::Positive)
            );
            Ok(())
        })
        .unwrap();
        assert_eq!(certified.certainty, CurveCertainty::Certified);

        let nested = resolve_certified_operation(&CurveContext::APPROXIMATE_512, |outer_policy| {
            assert_eq!(
                crate::classify::real_sign(&undecidable_zero, outer_policy),
                Some(RealSign::Zero)
            );
            let inner =
                resolve_certified_operation(&CurveContext::APPROXIMATE_512, |inner_policy| {
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
            resolve_certified_operation(&CurveContext::APPROXIMATE_512, |_outer_policy| {
                let inner =
                    resolve_certified_operation(&CurveContext::APPROXIMATE_512, |inner_policy| {
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
            resolve_cached_classification(&cache, &CurveContext::APPROXIMATE_512, |policy| {
                Ok::<_, ()>(if policy == &CurveContext::STRICT {
                    Classification::Uncertain(UncertaintyReason::Predicate)
                } else {
                    Classification::Decided(7_u8)
                })
            })
            .unwrap();
        assert_eq!(approximate, Classification::Decided(&7));

        let strict = resolve_cached_classification(
            &cache,
            &CurveContext::STRICT,
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
            &CurveContext::STRICT,
            |_| -> Result<Classification<u8>, ()> {
                panic!("explicit certified evidence should supersede the approximate fact")
            },
        )
        .unwrap();
        assert_eq!(upgraded, Classification::Decided(&8));
    }
}
