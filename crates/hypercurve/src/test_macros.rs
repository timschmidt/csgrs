//! Exact literal macros shared by unit and integration tests.

/// An exact rational or integer test literal: `r!(3 / 7)` is the exact value
/// 3/7 and `r!(5)` is the integer 5. It expands to the same `Real`
/// construction a test would write by hand, so it changes no value.
#[allow(unused_macros)]
macro_rules! r {
    ($numerator:literal / $denominator:literal) => {
        (::hyperreal::Real::from($numerator) / ::hyperreal::Real::from($denominator)).unwrap()
    };
    ($value:literal) => {
        ::hyperreal::Real::from($value)
    };
}

/// An exact test point: `pt!(x, y)` is `Point2::from_values(x, y)`, with
/// `Point2` resolved where the macro is used. It changes no value.
#[allow(unused_macros)]
macro_rules! pt {
    ($x:expr, $y:expr $(,)?) => {
        Point2::from_values($x, $y)
    };
}
