/// Upper bound on the width of a finite, ordered binary64 interval.
fn width_upper_bound(low: f64, high: f64) -> f64 {
    if low == high {
        return 0.0;
    }
    let width = high - low;
    if !width.is_finite() {
        return width;
    }
    // Two-Diff recovers the subtraction error. Preserve an exact result or
    // an existing upper bound; step upward only for downward rounding.
    // See https://people.eecs.berkeley.edu/~jrs/papers/robustr.pdf, Theorem 7.
    let virtual_low = high - width;
    let residual = (high - (width + virtual_low)) + (virtual_low - low);
    if residual.is_finite() && residual <= 0.0 {
        width
    } else {
        // Nonfinite intermediates cannot establish a tighter upper bound.
        width.next_up()
    }
}
