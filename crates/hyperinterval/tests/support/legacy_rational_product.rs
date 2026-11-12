// Frozen differential baseline from Hypersolve at 01eabec82.
// Keep the original dispatch sequence; this is a test oracle, not production authority.
use hyperreal::Rational;

pub fn rational_interval_product(
    left_lower: &Rational,
    left_upper: &Rational,
    right_lower: &Rational,
    right_upper: &Rational,
) -> (Rational, Rational) {
    if !left_lower.is_negative() {
        if !right_lower.is_negative() {
            return (left_lower * right_lower, left_upper * right_upper);
        }
        if !right_upper.is_positive() {
            return (left_upper * right_lower, left_lower * right_upper);
        }
        return (left_upper * right_lower, left_upper * right_upper);
    }
    if !left_upper.is_positive() {
        if !right_lower.is_negative() {
            return (left_lower * right_upper, left_upper * right_lower);
        }
        if !right_upper.is_positive() {
            return (left_upper * right_upper, left_lower * right_lower);
        }
        return (left_lower * right_upper, left_lower * right_lower);
    }
    if !right_lower.is_negative() {
        return (left_lower * right_upper, left_upper * right_upper);
    }
    if !right_upper.is_positive() {
        return (left_upper * right_lower, left_lower * right_lower);
    }
    let first_lower = left_lower * right_upper;
    let second_lower = left_upper * right_lower;
    let first_upper = left_lower * right_lower;
    let second_upper = left_upper * right_upper;
    let lower = if first_lower <= second_lower {
        first_lower
    } else {
        second_lower
    };
    let upper = if first_upper >= second_upper {
        first_upper
    } else {
        second_upper
    };
    (lower, upper)
}
