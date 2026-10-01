//! Exact rational roots, bivariate square roots, bilinear factorizations and
//! constraint resultants for parameter polynomials.

use hyperreal::{Real, RealSign, ZeroKnowledge};
use num::{BigInt, BigUint, Integer, One, Signed, ToPrimitive};

use crate::bivariate_arithmetic::*;
use crate::curve_resultant::{
    BivariatePolynomial, divide_bivariate_polynomial_exact, divide_univariate_polynomial_exact,
    greatest_common_divisor_univariate_polynomials_exact,
};

/// Exact STRICT sign: a structural zero, else a certified predicate sign.
fn strict_real_sign(value: &Real) -> Option<RealSign> {
    if value.zero_status() == ZeroKnowledge::Zero {
        return Some(RealSign::Zero);
    }
    match crate::classify_real_sign_predicate(value, hyperlimit::PredicatePolicy::STRICT).value()? {
        crate::PredicateSign::Negative => Some(RealSign::Negative),
        crate::PredicateSign::Zero => Some(RealSign::Zero),
        crate::PredicateSign::Positive => Some(RealSign::Positive),
    }
}

/// Returns a rational root of an exact rational polynomial, if a bounded rational-root search finds one.
pub fn exact_rational_polynomial_root(polynomial: &[Real]) -> Option<Real> {
    const MAX_RATIONAL_ROOT_FACTOR: u64 = 1_000_000_000;

    if polynomial.len() < 2 {
        return None;
    }
    let coefficients = polynomial
        .iter()
        .map(Real::exact_rational)
        .collect::<Option<Vec<_>>>()?;
    let common_denominator = coefficients
        .iter()
        .fold(BigUint::one(), |common, coefficient| {
            common.lcm(coefficient.denominator())
        });
    let mut integer_coefficients = coefficients
        .iter()
        .map(|coefficient| {
            let scale = &common_denominator / coefficient.denominator();
            let magnitude = BigInt::from(coefficient.numerator().clone()) * BigInt::from(scale);
            if coefficient.is_negative() {
                -magnitude
            } else {
                magnitude
            }
        })
        .collect::<Vec<_>>();
    let content = integer_coefficients
        .iter()
        .fold(BigInt::from(0_i8), |content, coefficient| {
            content.gcd(coefficient)
        })
        .abs();
    if content != BigInt::from(0_i8) && content != BigInt::from(1_i8) {
        for coefficient in &mut integer_coefficients {
            *coefficient /= &content;
        }
    }
    let constant = integer_coefficients[0].abs().to_u64()?;
    let leading = integer_coefficients.last()?.abs().to_u64()?;
    if constant == 0
        || leading == 0
        || constant > MAX_RATIONAL_ROOT_FACTOR
        || leading > MAX_RATIONAL_ROOT_FACTOR
    {
        return None;
    }
    let numerators = positive_divisors(constant);
    let denominators = positive_divisors(leading);
    for numerator in numerators {
        for factor_denominator in &denominators {
            if numerator.gcd(factor_denominator) != 1 {
                continue;
            }
            let numerator = i64::try_from(numerator).ok()?;
            for signed_numerator in [numerator, -numerator] {
                let candidate = Real::new(
                    hyperreal::Rational::fraction(signed_numerator, *factor_denominator).ok()?,
                );
                if Real::eval_poly(polynomial, &candidate).definitely_zero() {
                    return Some(candidate);
                }
            }
        }
    }
    None
}

/// Exact support of a nonzero bivariate polynomial: its degree in each
/// parameter, then the powers and coefficient of its lexicographically
/// leading term `(first_degree, second_degree, first, second, coefficient)`.
pub type BivariateNonzeroMetadata = (usize, usize, usize, usize, Real);

/// Returns the exact support of a bivariate polynomial: `Some(None)` when it
/// is exactly zero, and `None` when a coefficient sign is undecided.
#[cold]
#[inline(never)]
pub fn bivariate_exact_nonzero_metadata(
    polynomial: &BivariatePolynomial,
) -> Option<Option<BivariateNonzeroMetadata>> {
    let mut first_degree = 0;
    let mut second_degree = 0;
    let mut leading = None;
    for (first, row) in polynomial.coefficients.iter().enumerate() {
        for (second, coefficient) in row.iter().enumerate() {
            match strict_real_sign(coefficient)? {
                RealSign::Zero => {}
                RealSign::Negative | RealSign::Positive => {
                    first_degree = first_degree.max(first);
                    second_degree = second_degree.max(second);
                    if leading.as_ref().is_none_or(|(old_first, old_second, _)| {
                        (first, second) > (*old_first, *old_second)
                    }) {
                        leading = Some((first, second, coefficient.clone()));
                    }
                }
            }
        }
    }
    Some(leading.map(|(first, second, coefficient)| {
        (first_degree, second_degree, first, second, coefficient)
    }))
}

/// Adds one term to a partial exact bivariate square root.
#[cold]
#[inline(never)]
fn bivariate_square_root_add_term(
    residual: &mut BivariatePolynomial,
    root: &mut BivariatePolynomial,
    first: usize,
    second: usize,
    coefficient: Real,
) -> Option<()> {
    if strict_real_sign(root.coefficients.get(first)?.get(second)?)? != RealSign::Zero {
        return None;
    }
    let twice = Real::from(2_i8);
    for (old_first, row) in root.coefficients.iter().enumerate() {
        for (old_second, old_coefficient) in row.iter().enumerate() {
            if strict_real_sign(old_coefficient)? == RealSign::Zero {
                continue;
            }
            residual.coefficients[old_first + first][old_second + second] -=
                &twice * old_coefficient * &coefficient;
        }
    }
    residual.coefficients[first.checked_mul(2)?][second.checked_mul(2)?] -=
        &coefficient * &coefficient;
    root.coefficients[first][second] = coefficient;
    Some(())
}

/// Recovers an exact bivariate square by descending lexicographic terms.
#[cold]
#[inline(never)]
pub fn bivariate_exact_square_root(
    polynomial: &BivariatePolynomial,
) -> Option<BivariatePolynomial> {
    let Some((first_degree, second_degree, leading_first, leading_second, leading_square)) =
        bivariate_exact_nonzero_metadata(polynomial)?
    else {
        return Some(BivariatePolynomial::new(vec![vec![Real::zero()]]));
    };
    if !first_degree.is_multiple_of(2)
        || !second_degree.is_multiple_of(2)
        || !leading_first.is_multiple_of(2)
        || !leading_second.is_multiple_of(2)
    {
        return None;
    }
    let root_first_count = first_degree / 2 + 1;
    let root_second_count = second_degree / 2 + 1;
    let mut residual = BivariatePolynomial::new(try_zero_bivariate_coefficients(
        first_degree + 1,
        second_degree + 1,
    )?);
    for (target, source) in residual
        .coefficients
        .iter_mut()
        .zip(&polynomial.coefficients)
    {
        for (target, source) in target.iter_mut().zip(source) {
            *target = source.clone();
        }
    }
    let mut root = BivariatePolynomial::new(try_zero_bivariate_coefficients(
        root_first_count,
        root_second_count,
    )?);
    let leading = leading_square.clone().sqrt().ok()?;
    if strict_real_sign(&(&leading * &leading - leading_square))? != RealSign::Zero {
        return None;
    }
    let root_leading_first = leading_first / 2;
    let root_leading_second = leading_second / 2;
    bivariate_square_root_add_term(
        &mut residual,
        &mut root,
        root_leading_first,
        root_leading_second,
        leading.clone(),
    )?;
    let twice_leading = Real::from(2_i8) * &leading;
    for _ in 1..root_first_count.checked_mul(root_second_count)? {
        let Some((_, _, residual_first, residual_second, residual_coefficient)) =
            bivariate_exact_nonzero_metadata(&residual)?
        else {
            return Some(root);
        };
        let next_first = residual_first.checked_sub(root_leading_first)?;
        let next_second = residual_second.checked_sub(root_leading_second)?;
        if next_first >= root_first_count || next_second >= root_second_count {
            return None;
        }
        let coefficient = (residual_coefficient / &twice_leading).ok()?;
        bivariate_square_root_add_term(
            &mut residual,
            &mut root,
            next_first,
            next_second,
            coefficient,
        )?;
    }
    bivariate_exact_nonzero_metadata(&residual)?
        .is_none()
        .then_some(root)
}

/// Evaluates an exact bivariate polynomial at two exact values.
#[cold]
#[inline(never)]
pub fn bivariate_evaluate_exact(
    polynomial: &BivariatePolynomial,
    first: &Real,
    second: &Real,
) -> Real {
    polynomial
        .coefficients
        .iter()
        .rev()
        .fold(Real::zero(), |value, row| {
            value * first + Real::eval_poly(row, second)
        })
}

/// Exact specializations can reject, but never assert, a global repeated
/// cubic factor. A genuine repeated factor makes the discriminant relation
/// vanish identically at every probe. Multiple cheap probes avoid constructing
/// the full bivariate invariants when one accidental specialization vanishes.
#[cold]
#[inline(never)]
pub fn cubic_specialization_rejects_repeated_factor(
    coefficients: [&BivariatePolynomial; 4],
) -> bool {
    [(1_i8, 2_i8), (0, 0), (0, 1), (1, 0), (-1, 2)]
        .into_iter()
        .any(|(first, second)| {
            let first = Real::from(first);
            let second = Real::from(second);
            let [constant, linear, quadratic, cubic] = coefficients
                .map(|coefficient| bivariate_evaluate_exact(coefficient, &first, &second));
            let cubic_square = &cubic * &cubic;
            let delta_zero = &quadratic * &quadratic - Real::from(3_i8) * &cubic * &linear;
            let delta_one = Real::from(2_i8) * &quadratic * &quadratic * &quadratic
                - Real::from(9_i8) * &cubic * &quadratic * &linear
                + Real::from(27_i8) * &cubic_square * &constant;
            let discriminant_relation = &delta_one * &delta_one
                - Real::from(4_i8) * &delta_zero * &delta_zero * &delta_zero;
            matches!(
                strict_real_sign(&discriminant_relation),
                Some(RealSign::Negative | RealSign::Positive)
            )
        })
}

/// Extracts every exact-rational linear factor that the bounded rational-root
/// theorem finds. An irreducible remainder is retained implicitly; factors
/// already recovered before that boundary remain valid candidates.
#[cold]
#[inline(never)]
fn exact_rational_univariate_roots(mut polynomial: Vec<Real>) -> Option<Vec<Real>> {
    let mut roots = Vec::with_capacity(polynomial.len().saturating_sub(1));
    loop {
        while polynomial.len() > 1 && strict_real_sign(polynomial.last()?)? == RealSign::Zero {
            polynomial.pop();
        }
        if polynomial.len() <= 1 {
            break;
        }
        let root = if strict_real_sign(&polynomial[0])? == RealSign::Zero {
            Real::zero()
        } else {
            let Some(root) = exact_rational_polynomial_root(&polynomial) else {
                break;
            };
            root
        };
        let factor = [-root.clone(), Real::one()];
        let quotient = divide_univariate_polynomial_exact(&polynomial, &factor)?;
        if quotient.len() >= polynomial.len() {
            return None;
        }
        roots.push(root);
        polynomial = quotient;
    }
    (!roots.is_empty()).then_some(roots)
}

/// Returns one 3x3 minor of a 3x4 exact matrix.
fn three_by_three_minor(rows: &[[Real; 4]; 3], omitted: usize) -> Real {
    let [first, second, third] = match omitted {
        0 => [1, 2, 3],
        1 => [0, 2, 3],
        2 => [0, 1, 3],
        3 => [0, 1, 2],
        _ => unreachable!("one column is omitted from a three-by-four matrix"),
    };
    &rows[0][first] * (&rows[1][second] * &rows[2][third] - &rows[1][third] * &rows[2][second])
        - &rows[0][second] * (&rows[1][first] * &rows[2][third] - &rows[1][third] * &rows[2][first])
        + &rows[0][third]
            * (&rows[1][first] * &rows[2][second] - &rows[1][second] * &rows[2][first])
}

/// Removes structurally zero trailing rows and columns.
pub fn bivariate_trim_exact(mut polynomial: BivariatePolynomial) -> Option<BivariatePolynomial> {
    for row in &mut polynomial.coefficients {
        while row.len() > 1 && strict_real_sign(row.last()?)? == RealSign::Zero {
            row.pop();
        }
    }
    while polynomial.coefficients.len() > 1
        && polynomial
            .coefficients
            .last()?
            .iter()
            .all(|coefficient| strict_real_sign(coefficient) == Some(RealSign::Zero))
    {
        polynomial.coefficients.pop();
    }
    Some(polynomial)
}

/// Reconstructs the bilinear factor `u0+u1*y+x*(v0+v1*y)` from three exact
/// specialized roots. Signed maximal minors give the one-dimensional nullspace;
/// exact bivariate division remains the authority for the proposed factor.
pub fn bivariate_bilinear_factor_from_roots(
    samples: [&Real; 3],
    roots: [&Real; 3],
) -> Option<BivariatePolynomial> {
    let rows: [[Real; 4]; 3] = std::array::from_fn(|index| {
        [
            Real::one(),
            samples[index].clone(),
            roots[index].clone(),
            roots[index] * samples[index],
        ]
    });
    let coefficients: [Real; 4] = std::array::from_fn(|column| {
        let minor = three_by_three_minor(&rows, column);
        if column.is_multiple_of(2) {
            minor
        } else {
            -minor
        }
    });
    let pivot = coefficients.iter().find(|coefficient| {
        matches!(
            strict_real_sign(coefficient),
            Some(RealSign::Negative | RealSign::Positive)
        )
    });
    if pivot.is_none() {
        let repeated = roots[1..]
            .iter()
            .all(|root| strict_real_sign(&(*root - roots[0])) == Some(RealSign::Zero));
        return repeated
            .then(|| {
                BivariatePolynomial::new(vec![
                    vec![-roots[0].clone(), Real::zero()],
                    vec![Real::one(), Real::zero()],
                ])
            })
            .and_then(bivariate_trim_exact);
    }
    let pivot = pivot?;
    let normalized = coefficients
        .iter()
        .map(|coefficient| (coefficient / pivot).ok())
        .collect::<Option<Vec<_>>>()?;
    let factor = BivariatePolynomial::new(vec![
        vec![normalized[0].clone(), normalized[1].clone()],
        vec![normalized[2].clone(), normalized[3].clone()],
    ]);
    bivariate_exact_nonzero_metadata(&BivariatePolynomial::new(vec![
        factor.coefficients[1].clone(),
    ]))??;
    bivariate_trim_exact(factor)
}

/// Uses an unused exact specialization only to reject a proposed factor.
/// A nonzero specialized axis coefficient fixes one rational root; if that
/// root is absent from a nonempty exact root set, full bivariate division
/// cannot succeed. Empty or undecidable root sets make no negative claim.
fn bivariate_bilinear_factor_matches_roots_at_sample(
    factor: &BivariatePolynomial,
    sample: &Real,
    roots: &[Real],
) -> bool {
    if roots.is_empty() {
        return true;
    }
    let constant = Real::eval_poly(&factor.coefficients[0], sample);
    let linear = Real::eval_poly(&factor.coefficients[1], sample);
    match strict_real_sign(&linear) {
        Some(RealSign::Zero) | None => true,
        Some(RealSign::Negative | RealSign::Positive) => {
            let Some(root) = (-constant / linear).ok() else {
                return true;
            };
            let mut undecidable = false;
            for candidate in roots {
                match strict_real_sign(&(&root - candidate)) {
                    Some(RealSign::Zero) => return true,
                    Some(RealSign::Negative | RealSign::Positive) => {}
                    None => undecidable = true,
                }
            }
            undecidable
        }
    }
}

/// Returns the exact content in the second parameter.
fn bivariate_second_parameter_content(polynomial: &BivariatePolynomial) -> Option<Vec<Real>> {
    let mut content: Option<Vec<Real>> = None;
    for row in &polynomial.coefficients {
        let mut fiber = row.clone();
        while fiber
            .last()
            .is_some_and(|coefficient| strict_real_sign(coefficient) == Some(RealSign::Zero))
        {
            fiber.pop();
        }
        if fiber.is_empty() {
            continue;
        }
        content = Some(match content {
            None => fiber,
            Some(previous) => {
                greatest_common_divisor_univariate_polynomials_exact(&previous, &fiber)?
            }
        });
        if content.as_ref().is_none_or(|content| content.len() <= 1) {
            return None;
        }
    }
    content.filter(|content| content.len() > 1)
}

/// Multiplies factors by the retained second-parameter content.
fn bivariate_attach_second_parameter_content(
    polynomial: &BivariatePolynomial,
    factor: &BivariatePolynomial,
    quotient: &BivariatePolynomial,
) -> Vec<(BivariatePolynomial, BivariatePolynomial)> {
    if factor.coefficients.iter().any(|row| {
        row.get(1)
            .is_some_and(|coefficient| strict_real_sign(coefficient) != Some(RealSign::Zero))
    }) {
        return Vec::new();
    }
    let Some(content) = bivariate_second_parameter_content(quotient) else {
        return Vec::new();
    };
    exact_rational_univariate_roots(content)
        .unwrap_or_default()
        .into_iter()
        .filter_map(|root| {
            let content_factor = BivariatePolynomial::new(vec![vec![-root, Real::one()]]);
            let expanded = bivariate_trim_exact(try_bivariate_multiply(factor, &content_factor)?)?;
            let quotient = divide_bivariate_polynomial_exact(polynomial, &expanded)?;
            Some((expanded, quotient))
        })
        .collect()
}

/// Recovers up to `maximum_factorizations` distinct bilinear divisors visible
/// through four bounded exact rational specializations, inspecting at most
/// `maximum_proposals` root triples. Four choose three sample triples tolerate
/// one degree drop without making a sampled value part of the proof. An unused
/// fourth specialization rejects mismatched proposals before exact division.
#[cold]
#[inline(never)]
pub fn bivariate_bilinear_factorizations_bounded(
    polynomial: &BivariatePolynomial,
    maximum_factorizations: usize,
    maximum_proposals: usize,
) -> Vec<(BivariatePolynomial, BivariatePolynomial)> {
    if maximum_factorizations == 0 || maximum_proposals == 0 {
        return Vec::new();
    }
    const SAMPLE_TRIPLES: [[usize; 3]; 4] = [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]];
    let samples = [
        Real::zero(),
        Real::one(),
        Real::from(-1_i8),
        Real::from(2_i8),
    ];
    let roots = samples
        .iter()
        .map(|sample| {
            exact_rational_univariate_roots(bivariate_specialize_second(polynomial, sample))
                .unwrap_or_default()
        })
        .collect::<Vec<_>>();
    let mut factorizations = Vec::with_capacity(3);
    let mut proposals = 0_usize;
    for [first, second, third] in SAMPLE_TRIPLES {
        for first_root in &roots[first] {
            for second_root in &roots[second] {
                for third_root in &roots[third] {
                    if proposals >= maximum_proposals {
                        return factorizations;
                    }
                    proposals += 1;
                    let Some(factor) = bivariate_bilinear_factor_from_roots(
                        [&samples[first], &samples[second], &samples[third]],
                        [first_root, second_root, third_root],
                    ) else {
                        continue;
                    };
                    if factorizations
                        .iter()
                        .any(|(existing, _)| existing == &factor)
                    {
                        continue;
                    }
                    if (0..samples.len()).any(|index| {
                        ![first, second, third].contains(&index)
                            && !bivariate_bilinear_factor_matches_roots_at_sample(
                                &factor,
                                &samples[index],
                                &roots[index],
                            )
                    }) {
                        continue;
                    }
                    let Some(quotient) = divide_bivariate_polynomial_exact(polynomial, &factor)
                    else {
                        continue;
                    };
                    for expanded in
                        bivariate_attach_second_parameter_content(polynomial, &factor, &quotient)
                    {
                        if !factorizations
                            .iter()
                            .any(|(existing, _)| existing == &expanded.0)
                        {
                            factorizations.push(expanded);
                            if factorizations.len() == maximum_factorizations {
                                return factorizations;
                            }
                        }
                    }
                    factorizations.push((factor, quotient));
                    if factorizations.len() == maximum_factorizations {
                        return factorizations;
                    }
                }
            }
        }
    }
    factorizations
}

/// Returns the four coefficients of a bilinear polynomial.
pub fn bivariate_bilinear_coefficients(polynomial: &BivariatePolynomial) -> Option<[Real; 4]> {
    if polynomial.coefficients.len() > 2
        || polynomial
            .coefficients
            .iter()
            .any(|coefficients| coefficients.len() > 2)
    {
        return None;
    }
    Some(std::array::from_fn(|index| {
        polynomial
            .coefficients
            .get(index / 2)
            .and_then(|coefficients| coefficients.get(index % 2))
            .cloned()
            .unwrap_or_else(Real::zero)
    }))
}

/// Adds a scaled bivariate polynomial in place.
pub fn bivariate_add_scaled_assign(
    target: &mut BivariatePolynomial,
    source: &BivariatePolynomial,
    scale: &Real,
) -> Option<()> {
    let first_count = target.coefficients.len().max(source.coefficients.len());
    let second_count = target
        .coefficients
        .iter()
        .chain(&source.coefficients)
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    target
        .coefficients
        .try_reserve(first_count.saturating_sub(target.coefficients.len()))
        .ok()?;
    while target.coefficients.len() < first_count {
        let mut row = Vec::new();
        row.try_reserve_exact(second_count).ok()?;
        row.resize_with(second_count, Real::zero);
        target.coefficients.push(row);
    }
    for row in &mut target.coefficients {
        row.try_reserve(second_count.saturating_sub(row.len()))
            .ok()?;
        row.resize(second_count, Real::zero());
    }
    for (target, source) in target.coefficients.iter_mut().zip(&source.coefficients) {
        for (target, source) in target.iter_mut().zip(source) {
            *target += source * scale;
        }
    }
    Some(())
}

/// Returns `linear^degree * defining(-constant / linear)`, whose zero set is
/// the resultant of the defining polynomial and `constant + axis * linear`.
pub fn bivariate_linear_root_resultant(
    constant: &BivariatePolynomial,
    linear: &BivariatePolynomial,
    defining: &[Real],
) -> Option<BivariatePolynomial> {
    let degree = defining.len().checked_sub(1)?;
    if degree == 0 {
        return None;
    }
    // Homogeneous Horner evaluation retains only the current result and power
    // of `linear`; no table of every bivariate power is materialized.
    let mut result = BivariatePolynomial::new(vec![vec![defining[degree].clone()]]);
    let mut linear_power = BivariatePolynomial::new(vec![vec![Real::one()]]);
    let negative_constant = bivariate_scale(constant.clone(), &(-Real::one()));
    for coefficient in defining[..degree].iter().rev() {
        linear_power = try_bivariate_multiply(&linear_power, linear)?;
        result = try_bivariate_multiply(&negative_constant, &result)?;
        bivariate_add_scaled_assign(&mut result, &linear_power, coefficient)?;
    }
    Some(result)
}

/// Returns the target-axis coefficients of the exact resultant between one
/// bivariate polynomial of degree at most two in its first axis and one
/// constant-coefficient quadratic constraint on that axis.
///
/// For `F=a*x^2+b*x+c` and `G=d*x^2+e*x+f`, the Sylvester determinant is
///
/// `(a*f-c*d)^2 - (a*e-b*d)*(b*f-c*e)`.
///
/// Evaluating this closed form avoids the generic interpolation/resultant
/// path's exact divisions. Selected circle-pair projections reach this case
/// after eliminating their other quadratic source root, and their target-axis
/// coefficients may already be large exact rationals.
pub fn bivariate_quadratic_constraint_resultant(
    polynomial: &BivariatePolynomial,
    defining: &[Real],
) -> Option<Vec<Real>> {
    if polynomial.coefficients.len() > 3 || defining.len() != 3 {
        return None;
    }
    let coefficient = |power: usize| {
        polynomial
            .coefficients
            .get(power)
            .cloned()
            .unwrap_or_else(|| vec![Real::zero()])
    };
    let c = coefficient(0);
    let b = coefficient(1);
    let a = coefficient(2);
    let af_minus_cd = polynomial_subtract(
        &polynomial_scale(&a, &defining[0]),
        &polynomial_scale(&c, &defining[2]),
    );
    let ae_minus_bd = polynomial_subtract(
        &polynomial_scale(&a, &defining[1]),
        &polynomial_scale(&b, &defining[2]),
    );
    let bf_minus_ce = polynomial_subtract(
        &polynomial_scale(&b, &defining[0]),
        &polynomial_scale(&c, &defining[1]),
    );
    Some(polynomial_subtract(
        &polynomial_multiply(&af_minus_cd, &af_minus_cd),
        &polynomial_multiply(&ae_minus_bd, &bf_minus_ce),
    ))
}

/// Reparameterizes a univariate polynomial from an interval onto the unit interval.
pub fn polynomial_restrict_to_interval(
    coefficients: &[Real],
    start: &Real,
    end: &Real,
) -> Vec<Real> {
    let degree = coefficients.len().saturating_sub(1);
    let powers = polynomial_powers(&[start.clone(), end - start], degree);
    let mut restricted = vec![Real::zero(); degree + 1];
    for (power, coefficient) in coefficients.iter().enumerate() {
        for (index, factor) in powers[power].iter().enumerate() {
            restricted[index] += coefficient * factor;
        }
    }
    restricted
}

fn positive_divisors(value: u64) -> Vec<u64> {
    let mut low = Vec::new();
    let mut high = Vec::new();
    let mut divisor = 1_u64;
    while divisor <= value / divisor {
        if value.is_multiple_of(divisor) {
            low.push(divisor);
            let paired = value / divisor;
            if paired != divisor {
                high.push(paired);
            }
        }
        divisor += 1;
    }
    high.reverse();
    low.extend(high);
    low
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(value: i64) -> Real {
        Real::from(value)
    }

    #[test]
    fn rational_roots_and_metadata_are_exact() {
        // (2x - 1)(x + 3) = 2x^2 + 5x - 3.
        let root = exact_rational_polynomial_root(&[r(-3), r(5), r(2)]).unwrap();
        assert_eq!(Real::eval_poly(&[r(-3), r(5), r(2)], &root), Real::zero());
        // 3 + 4st + 5s^2 has degrees (2, 1) and leading term 5s^2.
        let polynomial = BivariatePolynomial::new(vec![vec![r(3)], vec![r(0), r(4)], vec![r(5)]]);
        let (first_degree, second_degree, first, second, coefficient) =
            bivariate_exact_nonzero_metadata(&polynomial)
                .unwrap()
                .unwrap();
        assert_eq!((first_degree, second_degree, first, second), (2, 1, 2, 0));
        assert_eq!(coefficient, r(5));
        assert_eq!(
            bivariate_exact_nonzero_metadata(&BivariatePolynomial::new(vec![vec![r(0)]])),
            Some(None)
        );
    }

    #[test]
    fn square_roots_and_bilinear_factors_reproduce_the_input() {
        let base = BivariatePolynomial::new(vec![vec![r(1), r(2)], vec![r(3)]]);
        let square = bivariate_multiply(&base, &base);
        let root = bivariate_exact_square_root(&square).unwrap();
        let (s, t) = (r(2), r(-5));
        let value = bivariate_evaluate_exact(&base, &s, &t);
        assert_eq!(
            bivariate_evaluate_exact(&root, &s, &t) * bivariate_evaluate_exact(&root, &s, &t),
            &value * &value
        );
        assert!(
            bivariate_exact_square_root(&bivariate_add(
                &square,
                &BivariatePolynomial::new(vec![vec![r(1)]])
            ))
            .is_none()
        );

        // (s - 2 + st)(1 + 3t - s) factors into two bilinear polynomials.
        let first = BivariatePolynomial::new(vec![vec![r(-2)], vec![r(1), r(1)]]);
        let second = BivariatePolynomial::new(vec![vec![r(1), r(3)], vec![r(-1)]]);
        let product = bivariate_multiply(&first, &second);
        let factorizations = bivariate_bilinear_factorizations_bounded(&product, 4, 64);
        assert!(!factorizations.is_empty());
        for (left, right) in factorizations {
            for (s, t) in [(r(0), r(0)), (r(3), r(-2)), (r(-1), r(4))] {
                assert_eq!(
                    bivariate_evaluate_exact(&left, &s, &t)
                        * bivariate_evaluate_exact(&right, &s, &t),
                    bivariate_evaluate_exact(&product, &s, &t)
                );
            }
        }
    }
}
