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
