#![allow(dead_code)]

#[path = "scalars.rs"]
mod scalars;
#[allow(unused_imports)]
pub(crate) use scalars::*;

/// A principal operation's value with the certainty of the policy that ran it.
#[derive(Debug)]
pub(crate) struct Outcome<T> {
    pub(crate) value: T,
    pub(crate) certainty: hypercurve::CurveCertainty,
}

impl<T> Outcome<T> {
    pub(crate) fn into_value(self) -> T {
        self.value
    }

    pub(crate) fn map<U>(self, map: impl FnOnce(T) -> U) -> Outcome<U> {
        Outcome {
            value: map(self.value),
            certainty: self.certainty,
        }
    }
}

/// Runs an exact principal operation directly under STRICT, or inside
/// `hypercurve::provisional` for any other policy, keeping its certainty.
pub(crate) fn under<T, E>(
    policy: &hypercurve::CurveContext,
    evaluate: impl FnOnce() -> Result<T, E>,
) -> Result<Outcome<T>, E> {
    if *policy == hypercurve::CurveContext::STRICT {
        return evaluate().map(|value| Outcome {
            value,
            certainty: hypercurve::CurveCertainty::Certified,
        });
    }
    let provisional = hypercurve::provisional(evaluate);
    let certainty = provisional.certainty();
    provisional
        .into_unverified()
        .map(|value| Outcome { value, certainty })
}

/// A completed operation result whose topology decisions must all be certified.
pub(crate) trait IntoCertified<T> {
    fn into_certified(self) -> T;
}

impl<T> IntoCertified<T> for hypercurve::CurveOutcome<T> {
    fn into_certified(self) -> T {
        assert_eq!(self.certainty, hypercurve::CurveCertainty::Certified);
        self.value
    }
}

impl<T> IntoCertified<T> for Outcome<T> {
    fn into_certified(self) -> T {
        assert_eq!(self.certainty, hypercurve::CurveCertainty::Certified);
        self.value
    }
}

/// Runs an exact principal operation under `policy`, discarding certainty,
/// for operations whose result never carried it.
pub(crate) fn under_value<T, E>(
    policy: &hypercurve::CurveContext,
    evaluate: impl FnOnce() -> Result<T, E>,
) -> Result<T, E> {
    under(policy, evaluate).map(Outcome::into_value)
}

/// Runs an exact principal query under `policy`, keeping an undecided
/// predicate as `Classification::Uncertain` and any other error as `Err`.
pub(crate) fn under_classified_result<T>(
    policy: &hypercurve::CurveContext,
    evaluate: impl FnOnce() -> hypercurve::ExactCurveResult<T>,
) -> hypercurve::ExactCurveResult<hypercurve::Classification<T>> {
    match under_value(policy, evaluate) {
        Ok(value) => Ok(hypercurve::Classification::Decided(value)),
        Err(hypercurve::ExactCurveError::Blocked(blocker)) => {
            Ok(hypercurve::Classification::Uncertain(blocker.reason()))
        }
        Err(error) => Err(error),
    }
}

/// Runs an exact principal predicate under `policy` as a classification.
///
/// The predicate's inputs are valid by construction, so an invalid-state
/// error is a test failure.
pub(crate) fn under_classified<T>(
    policy: &hypercurve::CurveContext,
    evaluate: impl FnOnce() -> hypercurve::ExactCurveResult<T>,
) -> hypercurve::Classification<T> {
    under_classified_result(policy, evaluate)
        .unwrap_or_else(|error| panic!("exact predicate rejected its input: {error:?}"))
}

/// Runs an exact principal query under `policy`, keeping its certainty and
/// an undecided predicate as `Classification::Uncertain`.
pub(crate) fn under_outcome_classified<T>(
    policy: &hypercurve::CurveContext,
    evaluate: impl FnOnce() -> hypercurve::ExactCurveResult<T>,
) -> hypercurve::ExactCurveResult<Outcome<hypercurve::Classification<T>>> {
    let (result, certainty) = if *policy == hypercurve::CurveContext::STRICT {
        (evaluate(), hypercurve::CurveCertainty::Certified)
    } else {
        let provisional = hypercurve::provisional(evaluate);
        let certainty = provisional.certainty();
        (provisional.into_unverified(), certainty)
    };
    match result {
        Ok(value) => Ok(Outcome {
            value: hypercurve::Classification::Decided(value),
            certainty,
        }),
        Err(hypercurve::ExactCurveError::Blocked(blocker)) => Ok(Outcome {
            value: hypercurve::Classification::Uncertain(blocker.reason()),
            certainty,
        }),
        Err(error) => Err(error),
    }
}
