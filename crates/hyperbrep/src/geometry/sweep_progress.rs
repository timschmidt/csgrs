//! Exact progress and straight-image certificates shared by construction and replay.

use super::*;

/// How a certified sweep path advances through its parallel section planes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum SweepProgress {
    /// Plane progress is an exact affine function of the path parameter, so
    /// one exact division gives the section parameter of a point.
    Affine,
    /// Plane progress has a certified positive derivative on the closed
    /// parameter domain without being affine. A point's section parameter is
    /// the path's unique exact contact with the point's section plane. This is
    /// admitted only for a fixed profile frame.
    Monotone,
}

/// Certifies that a rational Bézier path with positive weights crosses the
/// planes `normal · x = c` with strictly positive speed at every parameter of
/// the closed unit domain.
///
/// Write the plane progress as `h = N / W`. The exact affine identity
/// `N = (start + (end - start) t) W` is tried first. Otherwise `h' > 0` on
/// `[0, 1]` exactly when `D = N'W - NW' > 0` there, since `W > 0`. `D` is
/// written in the degree `2d - 1` Bernstein basis; nonnegative coefficients
/// with positive endpoint coefficients prove positivity. If an interior
/// coefficient is negative, an exact Sturm count of the roots of `D` in
/// `(0, 1]` decides. A zero of `h'` anywhere, including a stationary endpoint
/// such as `h = t²`, is outside this certificate and returns `None`, as does
/// progress that decreases somewhere although its endpoints increase.
pub(crate) fn certified_sweep_progress(
    control_points: &[Point3],
    weights: &[Real],
    normal: &Vector3,
) -> GeometryResult<Option<SweepProgress>> {
    let Some(degree) = control_points
        .len()
        .checked_sub(1)
        .filter(|degree| *degree > 0)
    else {
        return Ok(None);
    };
    let Some(elevated_degree) = degree.checked_add(1) else {
        return Ok(None);
    };
    if weights.len() != control_points.len() {
        return Ok(None);
    }
    let scalar = |point: &Point3| normal.dot(&Vector3::from(point.clone()));
    let start = scalar(&control_points[0]);
    let end = scalar(&control_points[degree]);
    if decided_order(compare_reals(
        &(&end - &start),
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? != Ordering::Greater
    {
        return Ok(None);
    }
    let numerators = control_points
        .iter()
        .zip(weights)
        .map(|(control, weight)| weight * scalar(control))
        .collect::<Vec<_>>();
    if affine_progress_identity(&numerators, weights, &start, &end, elevated_degree)? {
        return Ok(Some(SweepProgress::Affine));
    }
    if monotone_progress_certificate(control_points, weights, &start, scalar, false)? {
        Ok(Some(SweepProgress::Monotone))
    } else {
        Ok(None)
    }
}

/// Proves an injective straight image with possibly isolated stationary
/// parameters. Returned Bernstein values retain the exact authored progress
/// in a regular planar side chart; they do not assert a regular source chart.
/// Positive weights and distinct progress endpoints are prerequisites.
/// Nonnegative polynomial `N'W - NW'` then has only finitely many zeros, so
/// every nonempty parameter interval has positive integrated progress.
pub(crate) fn certified_straight_sweep_image(
    control_points: &[Point3],
    weights: &[Real],
    normal: &Vector3,
) -> GeometryResult<Option<Vec<Real>>> {
    let start = &control_points[0];
    let end = &control_points[control_points.len() - 1];
    let direction = end - start;
    let progress = normal.dot(&direction);
    if decided_order(compare_reals(
        &progress,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? != Ordering::Greater
    {
        return Ok(None);
    }
    for point in control_points {
        if !progress_values_equal(
            &(point - start).cross(&direction).norm_squared(),
            &Real::zero(),
        )? {
            return Ok(None);
        }
    }
    let scalar = |point: &Point3| normal.dot(&Vector3::from(point.clone()));
    if !monotone_progress_certificate(control_points, weights, &scalar(start), scalar, true)? {
        return Ok(None);
    }
    Ok(Some(
        control_points
            .iter()
            .map(|point| (normal.dot(&(point - start)) / &progress).map_err(GeometryError::Numeric))
            .collect::<Result<Vec<_>, _>>()?,
    ))
}

fn affine_progress_identity(
    numerators: &[Real],
    weights: &[Real],
    start: &Real,
    end: &Real,
    elevated_degree: usize,
) -> GeometryResult<bool> {
    let degree = elevated_degree - 1;
    for index in 0..=elevated_degree {
        let lower_count = Real::from(
            u128::try_from(elevated_degree - index).expect("usize is representable as u128"),
        );
        let upper_count =
            Real::from(u128::try_from(index).expect("usize is representable as u128"));
        let elevated_numerator = (if index <= degree {
            &lower_count * &numerators[index]
        } else {
            Real::zero()
        }) + if index > 0 {
            &upper_count * &numerators[index - 1]
        } else {
            Real::zero()
        };
        let affine_times_weight = (if index <= degree {
            &lower_count * start * &weights[index]
        } else {
            Real::zero()
        }) + if index > 0 {
            &upper_count * end * &weights[index - 1]
        } else {
            Real::zero()
        };
        if !progress_values_equal(&elevated_numerator, &affine_times_weight)? {
            return Ok(false);
        }
    }
    Ok(true)
}

/// Sign certificate for `D = N'W - NW'`, where `N = Σ wᵢ hᵢ Bᵢᵈ` and
/// `W = Σ wᵢ Bᵢᵈ`. With `aᵢ = wᵢ hᵢ`, the coefficient of `Bₖ²ᵈ⁻¹` is a
/// positive multiple of
/// `Eₖ = Σ_{i+j=k} C(d-1,i) C(d,j) ((aᵢ₊₁ - aᵢ) wⱼ - (wᵢ₊₁ - wᵢ) aⱼ)`.
/// Translating `h` leaves `D` unchanged, so heights are taken from `start`.
fn monotone_progress_certificate(
    control_points: &[Point3],
    weights: &[Real],
    start: &Real,
    scalar: impl Fn(&Point3) -> Real,
    allow_stationary: bool,
) -> GeometryResult<bool> {
    let degree = control_points.len() - 1;
    let numerators = control_points
        .iter()
        .zip(weights)
        .map(|(control, weight)| weight * (scalar(control) - start))
        .collect::<Vec<_>>();
    let derivative_binomials = (0..degree)
        .map(|index| binomial_real(degree - 1, index))
        .collect::<Result<Vec<_>, _>>()?;
    let binomials = (0..=degree)
        .map(|index| binomial_real(degree, index))
        .collect::<Result<Vec<_>, _>>()?;
    let product_degree = 2 * degree - 1;
    let mut coefficients = Vec::with_capacity(product_degree + 1);
    let mut interior_negative = false;
    for product_index in 0..=product_degree {
        let mut coefficient = Real::zero();
        for first in product_index.saturating_sub(degree)..=product_index.min(degree - 1) {
            let second = product_index - first;
            let numerator_step = &numerators[first + 1] - &numerators[first];
            let weight_step = &weights[first + 1] - &weights[first];
            coefficient += &derivative_binomials[first]
                * &binomials[second]
                * (numerator_step * &weights[second] - weight_step * &numerators[second]);
        }
        let endpoint = product_index == 0 || product_index == product_degree;
        match decided_order(compare_reals(
            &coefficient,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? {
            Ordering::Greater => {}
            // A nonpositive endpoint rate is a stationary or retreating start
            // or end, not merely a weak sufficient certificate.
            Ordering::Equal | Ordering::Less if endpoint && !allow_stationary => return Ok(false),
            Ordering::Equal => {}
            Ordering::Less => interior_negative = true,
        }
        coefficients.push(coefficient);
    }
    if !interior_negative {
        return Ok(true);
    }
    // `D / d = Σ Eₖ tᵏ (1 - t)²ᵈ⁻¹⁻ᵏ` in the power basis.
    let mut power = vec![Real::zero(); product_degree + 1];
    for (index, coefficient) in coefficients.iter().enumerate() {
        for offset in 0..=product_degree - index {
            let term = coefficient * binomial_real(product_degree - index, offset)?;
            if offset % 2 == 0 {
                power[index + offset] += term;
            } else {
                power[index + offset] -= term;
            }
        }
    }
    let undecided = || GeometryError::PredicateUnresolved {
        needed: hyperlimit::RefinementNeed::RealRefinement,
        stage: hyperlimit::Escalation::Undecided,
    };
    if allow_stationary {
        return hypersolve::certify_univariate_polynomial_nonnegative(
            &power,
            &Real::zero(),
            &Real::one(),
            crate::STRICT_PREDICATES,
        )
        .ok_or_else(undecided);
    }
    let sturm = hypersolve::UnivariateSturmSequence::new(&power, crate::STRICT_PREDICATES)
        .ok_or_else(undecided)?;
    // D(0) > 0 and D(1) > 0 are proved above, so a root in (0, 1] is interior.
    Ok(sturm
        .count_distinct_roots(&Real::zero(), &Real::one(), crate::STRICT_PREDICATES)
        .ok_or_else(undecided)?
        == 0)
}

fn progress_values_equal(a: &Real, b: &Real) -> GeometryResult<bool> {
    Ok(decided_order(compare_reals(a, b, crate::STRICT_PREDICATES))? == Ordering::Equal)
}
