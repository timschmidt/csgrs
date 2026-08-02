//! Exact resultant helpers for low-degree parametric curve intersections.
//!
//! Curve crates own topology, span subdivision, and curve-specific validity
//! rules. This module supplies the bounded exact algebraic handoff they need:
//! two polynomial parametric curves in power basis are substituted into
//! `x1(t) - x2(u)` and `y1(t) - y2(u)`, the second parameter is eliminated by
//! Sylvester resultants, and the retained-parameter polynomial is reconstructed
//! by exact interpolation. This follows the elimination view of Bezier clipping, while keeping
//! the exact-computation boundary explicit: the resultant is candidate
//! algebraic evidence, not an accepted topology event, until a downstream
//! curve package replays it against retained geometry.

use hyperreal::{CertifiedRealSign, Real, RealSign};

use crate::bareiss::{BareissError, determinant_bareiss};
use crate::curve_substitution::RationalParametricCurve2;
use crate::integer_interpolation::primitive_integer_polynomial_gcd;
use crate::resultant::{
    UnivariateResultantError, resultant_univariate_polynomials, sylvester_matrix,
};
use crate::root_isolation::{polynomial_div_rem, polynomial_gcd};

/// Selects which curve parameter remains after exact elimination.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveResultantParameter {
    /// Retain the first curve's parameter.
    First,
    /// Retain the second curve's parameter.
    Second,
}

/// Power-basis representation of a 2D polynomial parametric curve.
///
/// Coefficients are stored in ascending power order. For example,
/// `x_coefficients = [1, 2, 3]` means `x(t) = 1 + 2*t + 3*t^2`.
#[derive(Clone, Debug, PartialEq)]
pub struct PolynomialParametricCurve2 {
    /// Power-basis coefficients for the x-coordinate.
    pub x_coefficients: Vec<Real>,
    /// Power-basis coefficients for the y-coordinate.
    pub y_coefficients: Vec<Real>,
}

impl PolynomialParametricCurve2 {
    /// Construct a 2D polynomial parametric curve in power basis.
    pub const fn new(x_coefficients: Vec<Real>, y_coefficients: Vec<Real>) -> Self {
        Self {
            x_coefficients,
            y_coefficients,
        }
    }
}

/// Exact polynomial in two parameters.
///
/// `coefficients[first_power][second_power]` multiplies
/// `first_parameter^first_power * second_parameter^second_power`. Rows may be
/// ragged; omitted coefficients are zero.
#[derive(Clone, Debug, PartialEq)]
pub struct BivariatePolynomial {
    /// Coefficient grid in ascending powers of both parameters.
    pub coefficients: Vec<Vec<Real>>,
}

impl BivariatePolynomial {
    /// Constructs a bivariate polynomial from its ascending-power grid.
    pub const fn new(coefficients: Vec<Vec<Real>>) -> Self {
        Self { coefficients }
    }
}

/// Final status for exact univariate axis-factor extraction from two bivariate equations.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BivariatePolynomialAxisFactorStatus {
    /// Neither equation pair has a nonconstant common factor depending on only
    /// one parameter.
    Primitive,
    /// At least one nonconstant common axis factor was extracted exactly.
    Reduced,
    /// Both equations are identically zero, so no greatest finite factor exists.
    ZeroSystem,
    /// An authored coefficient tower could not be reduced by exact arithmetic.
    UnsupportedCoefficient,
    /// Division by a computed exact GCD did not leave zero remainder.
    DivisionFailed,
}

/// Exact common axis factors and, when needed, their primitive equation pair.
///
/// Factors are stored in ascending power order and normalized to primitive
/// integer coefficients with positive leading coefficient. A successful absent
/// factor is represented by `[1]`; factors are empty when extraction did not
/// complete. `reduced_equations` is allocated only for
/// [`BivariatePolynomialAxisFactorStatus::Reduced`].
///
/// This report does **not** authorize saturation. Removing one of these factors
/// deletes the complete fiber at each of its roots. A geometry caller must first
/// prove that every extracted factor is nonzero throughout that parameter's
/// authored domain before substituting `reduced_equations` for the originals.
#[derive(Clone, Debug, PartialEq)]
pub struct BivariatePolynomialAxisFactorReport {
    /// Final extraction status.
    pub status: BivariatePolynomialAxisFactorStatus,
    /// Greatest common factor depending only on the first parameter.
    pub first_parameter_factor: Vec<Real>,
    /// Greatest common factor depending only on the second parameter after the
    /// first-parameter factor has been divided out.
    pub second_parameter_factor: Vec<Real>,
    /// Primitive equation pair when a nonconstant factor was extracted.
    pub reduced_equations: Option<[BivariatePolynomial; 2]>,
}

/// Extract exact common factors supported on either parameter axis.
///
/// For equations `F(t, u)` and `G(t, u)`, the first factor is the univariate GCD
/// of every `u`-coefficient polynomial in `t` from both equations. After exact
/// division, the second factor is the GCD of every remaining `t`-coefficient
/// polynomial in `u`. Thus genuinely bivariate common components remain in the
/// returned equations; only separable axis content is reported.
///
/// The integer-polynomial route remains the fast path. Exact `Real` coefficient
/// towers fall through to strict field polynomial GCD and division. In
/// particular, APPROXIMATE_512 is never used to infer an algebraic factor.
/// Callers remain responsible for domain-certified saturation as described by
/// [`BivariatePolynomialAxisFactorReport`].
pub fn extract_bivariate_polynomial_system_axis_factors(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
) -> BivariatePolynomialAxisFactorReport {
    let initial =
        extract_bivariate_polynomial_system_axis_factors_once(first_equation, second_equation);
    if !matches!(
        initial.status,
        BivariatePolynomialAxisFactorStatus::UnsupportedCoefficient
            | BivariatePolynomialAxisFactorStatus::DivisionFailed
    ) {
        return initial;
    }
    retry_bivariate_polynomial_system_axis_factors(first_equation, second_equation)
        .unwrap_or(initial)
}

#[cold]
#[inline(never)]
fn retry_bivariate_polynomial_system_axis_factors(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
) -> Option<BivariatePolynomialAxisFactorReport> {
    let (Some((first_normalized, first_scale)), Some((second_normalized, second_scale))) = (
        normalize_bivariate_projective_scale(first_equation),
        normalize_bivariate_projective_scale(second_equation),
    ) else {
        return None;
    };
    let mut retried = extract_bivariate_polynomial_system_axis_factors_once(
        &first_normalized,
        &second_normalized,
    );
    if !matches!(
        retried.status,
        BivariatePolynomialAxisFactorStatus::Primitive
            | BivariatePolynomialAxisFactorStatus::Reduced
    ) {
        return None;
    }
    if let Some(reduced) = &mut retried.reduced_equations {
        scale_bivariate_in_place(&mut reduced[0], &first_scale);
        scale_bivariate_in_place(&mut reduced[1], &second_scale);
    }
    Some(retried)
}

fn extract_bivariate_polynomial_system_axis_factors_once(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
) -> BivariatePolynomialAxisFactorReport {
    let equations = [first_equation, second_equation];
    let has_nonrational_coefficient = equations.iter().any(|equation| {
        equation
            .coefficients
            .iter()
            .flatten()
            .any(|coefficient| coefficient.exact_rational_ref().is_none())
    });
    if equations
        .iter()
        .all(|equation| exact_bivariate_is_zero(equation))
    {
        return axis_factor_failure(BivariatePolynomialAxisFactorStatus::ZeroSystem);
    }

    let Some(first_parameter_factor) =
        common_axis_factor(&equations, CurveResultantParameter::First)
    else {
        return axis_factor_failure(if has_nonrational_coefficient {
            BivariatePolynomialAxisFactorStatus::UnsupportedCoefficient
        } else {
            BivariatePolynomialAxisFactorStatus::DivisionFailed
        });
    };
    if exact_polynomial_is_zero(&first_parameter_factor) {
        return axis_factor_failure(BivariatePolynomialAxisFactorStatus::DivisionFailed);
    }

    let mut reduced_equations = if first_parameter_factor.len() > 1 {
        let (Some(first), Some(second)) = (
            divide_bivariate_by_axis(
                first_equation,
                &first_parameter_factor,
                CurveResultantParameter::First,
            ),
            divide_bivariate_by_axis(
                second_equation,
                &first_parameter_factor,
                CurveResultantParameter::First,
            ),
        ) else {
            return axis_factor_failure(BivariatePolynomialAxisFactorStatus::DivisionFailed);
        };
        Some([first, second])
    } else {
        None
    };

    let reduced_refs = match &reduced_equations {
        Some(reduced) => [&reduced[0], &reduced[1]],
        None => equations,
    };
    let Some(second_parameter_factor) =
        common_axis_factor(&reduced_refs, CurveResultantParameter::Second)
    else {
        return axis_factor_failure(if has_nonrational_coefficient {
            BivariatePolynomialAxisFactorStatus::UnsupportedCoefficient
        } else {
            BivariatePolynomialAxisFactorStatus::DivisionFailed
        });
    };
    if exact_polynomial_is_zero(&second_parameter_factor) {
        return axis_factor_failure(BivariatePolynomialAxisFactorStatus::DivisionFailed);
    }

    if second_parameter_factor.len() > 1 {
        let sources = match &reduced_equations {
            Some(reduced) => [&reduced[0], &reduced[1]],
            None => equations,
        };
        let (Some(first), Some(second)) = (
            divide_bivariate_by_axis(
                sources[0],
                &second_parameter_factor,
                CurveResultantParameter::Second,
            ),
            divide_bivariate_by_axis(
                sources[1],
                &second_parameter_factor,
                CurveResultantParameter::Second,
            ),
        ) else {
            return axis_factor_failure(BivariatePolynomialAxisFactorStatus::DivisionFailed);
        };
        reduced_equations = Some([first, second]);
    }

    let status = if reduced_equations.is_some() {
        BivariatePolynomialAxisFactorStatus::Reduced
    } else {
        BivariatePolynomialAxisFactorStatus::Primitive
    };
    BivariatePolynomialAxisFactorReport {
        status,
        first_parameter_factor,
        second_parameter_factor,
        reduced_equations,
    }
}

fn normalize_bivariate_projective_scale(
    polynomial: &BivariatePolynomial,
) -> Option<(BivariatePolynomial, Real)> {
    let (_, _, scale) = leading_bivariate_term(&polynomial.coefficients)?;
    let coefficients = polynomial
        .coefficients
        .iter()
        .map(|row| {
            row.iter()
                .map(|coefficient| (coefficient / &scale).ok())
                .collect::<Option<Vec<_>>>()
        })
        .collect::<Option<Vec<_>>>()?;
    Some((canonical_exact_bivariate(coefficients), scale))
}

fn scale_bivariate_in_place(polynomial: &mut BivariatePolynomial, scale: &Real) {
    for coefficient in polynomial.coefficients.iter_mut().flatten() {
        *coefficient *= scale;
    }
}

/// Configuration for bounded exact curve resultant construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CurveIntersectionResultantConfig {
    /// Precision bound used when coefficient signs are certified.
    pub min_precision: i32,
    /// Maximum retained-parameter degree this bounded helper may interpolate.
    pub max_resultant_degree: usize,
}

impl Default for CurveIntersectionResultantConfig {
    fn default() -> Self {
        Self {
            min_precision: hyperlimit::PredicatePolicy::MAX_REFINEMENT_PRECISION,
            max_resultant_degree: 32,
        }
    }
}

/// Final status for a curve intersection resultant report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CurveIntersectionResultantStatus {
    /// The retained-parameter resultant polynomial was constructed exactly.
    Constructed,
    /// At least one coordinate polynomial was empty.
    EmptyCoordinatePolynomial,
    /// A coefficient needed for degree trimming could not be certified.
    UndecidedCoefficient,
    /// The conservative resultant degree bound exceeded the configured budget.
    DegreeBoundExceeded,
    /// A sampled Sylvester resultant failed.
    ResultantError,
    /// Exact interpolation required a division that failed.
    InterpolationDivisionFailed,
    /// A homogeneous rational curve had certified zero weight everywhere.
    InvalidHomogeneousWeight,
}

/// One exact sample used to reconstruct the retained-parameter resultant.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionResultantSample {
    /// Retained parameter value used for this sample.
    pub parameter_value: Real,
    /// Exact sampled resultant after eliminating the other parameter.
    pub resultant: Real,
}

/// Exact report for eliminating one parameter from two parametric curves.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionResultantReport {
    /// Final construction status.
    pub status: CurveIntersectionResultantStatus,
    /// Parameter that remains in `resultant_coefficients`.
    pub retained_parameter: CurveResultantParameter,
    /// Parameter eliminated by the Sylvester resultant.
    pub eliminated_parameter: CurveResultantParameter,
    /// Conservative degree bound used to choose interpolation samples.
    pub degree_bound: usize,
    /// Sampled exact resultants in retained-parameter order.
    pub samples: Vec<CurveIntersectionResultantSample>,
    /// Resultant polynomial coefficients in ascending retained-parameter power
    /// order. Empty unless `status == Constructed`.
    pub resultant_coefficients: Vec<Real>,
    /// Sampled resultant error, if construction failed at that boundary.
    pub resultant_error: Option<UnivariateResultantError>,
}

/// Final status for a rational lift of the eliminated system parameter.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CurveIntersectionParameterLiftStatus {
    /// Every requested parameter-recovery polynomial was constructed exactly.
    Constructed,
    /// At least one bivariate equation was empty.
    EmptyEquation,
    /// A coefficient needed for degree certification remained undecided.
    UndecidedCoefficient,
    /// One equation is constant in the eliminated parameter, so a linear
    /// common-root lift cannot certify the pairing.
    UnsupportedEliminatedDegree,
    /// The conservative cofactor degree exceeded the configured budget.
    DegreeBoundExceeded,
    /// An exact cofactor determinant could not be constructed.
    DeterminantError,
    /// Exact cofactor interpolation or trimming did not complete.
    InterpolationFailed,
}

/// One exact adjugate ratio for lifting the eliminated parameter.
///
/// At a retained root where the specialized Sylvester matrix has nullity one,
/// at least one returned denominator is nonzero and
/// `eliminated = numerator(retained) / denominator(retained)`.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionParameterLiftMap {
    /// Sylvester row whose two signed cofactors define this ratio.
    pub cofactor_row: usize,
    /// Numerator coefficients in ascending retained-parameter power order.
    pub numerator_coefficients: Vec<Real>,
    /// Denominator coefficients in ascending retained-parameter power order.
    pub denominator_coefficients: Vec<Real>,
}

/// Exact report for linear common-root lifts of one bivariate system parameter.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionParameterLiftReport {
    /// Final construction status.
    pub status: CurveIntersectionParameterLiftStatus,
    /// Parameter supplied to every returned rational map.
    pub retained_parameter: CurveResultantParameter,
    /// Parameter produced by every returned rational map.
    pub lifted_parameter: CurveResultantParameter,
    /// Conservative interpolation degree used for every signed cofactor.
    pub degree_bound: usize,
    /// Candidate adjugate ratios. Identically zero denominators are omitted.
    pub maps: Vec<CurveIntersectionParameterLiftMap>,
    /// Exact determinant failure, when construction stopped at that boundary.
    pub determinant_error: Option<BareissError>,
}

/// Final status for one generic positive-dimensional component of a bivariate system.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BivariatePolynomialComponentStatus {
    /// A rational map for one generic linear fiber factor was constructed.
    Rational,
    /// A non-linear generic common-fiber polynomial was constructed exactly.
    Implicit,
    /// At least one bivariate equation was empty.
    EmptyEquation,
    /// A coefficient needed for degree certification remained undecided.
    UndecidedCoefficient,
    /// One equation is constant in the prospective lifted parameter.
    UnsupportedLiftedDegree,
    /// The conservative coefficient degree exceeded the configured budget.
    DegreeBoundExceeded,
    /// No supported component could be extracted from the generic common fiber.
    NoSupportedComponent,
    /// An exact subresultant determinant could not be constructed.
    DeterminantError,
    /// Exact coefficient interpolation did not complete.
    InterpolationFailed,
    /// The constructed component did not divide both authored equations exactly.
    DivisionFailed,
}

/// One exact positive-dimensional bivariate component and its residual system.
///
/// When `status` is [`BivariatePolynomialComponentStatus::Rational`],
/// the prospective lifted parameter is `numerator(retained) / denominator(retained)`.
/// When it is [`BivariatePolynomialComponentStatus::Implicit`],
/// `implicit_component` contains the primitive bivariate common-fiber
/// polynomial. Geometry callers must still certify the finite real branches
/// admitted by the authored parameter domains and geometric predicates.
#[derive(Clone, Debug, PartialEq)]
pub struct BivariatePolynomialComponentReport {
    /// Final construction status.
    pub status: BivariatePolynomialComponentStatus,
    /// Parameter retained while constructing the generic common fiber.
    pub retained_parameter: CurveResultantParameter,
    /// Parameter lifted by the generic common fiber.
    pub lifted_parameter: CurveResultantParameter,
    /// Conservative interpolation degree used by the selected exact
    /// reconstruction lane.
    pub degree_bound: usize,
    /// Rational-map numerator in ascending retained-parameter power order.
    pub numerator_coefficients: Vec<Real>,
    /// Rational-map denominator in ascending retained-parameter power order.
    pub denominator_coefficients: Vec<Real>,
    /// Primitive non-linear common-fiber polynomial, when the component is not
    /// representable by a supported rational-linear map.
    pub implicit_component: Option<BivariatePolynomial>,
    /// Both authored equations after exact division by the published component.
    /// Present for rational and implicit component reports.
    ///
    /// Geometry callers must solve this residual system as well as transport
    /// the positive-dimensional component. A component and isolated contacts
    /// may coexist in one authored system.
    pub reduced_equations: Option<[BivariatePolynomial; 2]>,
    /// Exact determinant failure, when construction stopped at that boundary.
    pub determinant_error: Option<BareissError>,
}

/// Eliminates one parameter from two exact bivariate polynomial equations.
///
/// Both equations use [`BivariatePolynomial`]'s `(first, second)` coefficient
/// order. The implementation certifies their bidegrees, evaluates exact
/// univariate specializations, skips the finitely many specializations whose
/// eliminated-variable degree drops, and reconstructs the resultant by exact
/// interpolation. The returned polynomial is candidate evidence only; callers
/// must replay isolated roots against their original equations and geometric
/// branch conditions.
pub fn resultant_bivariate_polynomial_system(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    config: CurveIntersectionResultantConfig,
) -> CurveIntersectionResultantReport {
    let eliminated_parameter = match retained_parameter {
        CurveResultantParameter::First => CurveResultantParameter::Second,
        CurveResultantParameter::Second => CurveResultantParameter::First,
    };
    if bivariate_polynomial_is_empty(first_equation)
        || bivariate_polynomial_is_empty(second_equation)
    {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::EmptyCoordinatePolynomial,
            retained_parameter,
            eliminated_parameter,
            0,
            Vec::new(),
            Vec::new(),
            None,
        );
    }

    let Ok(first_degree) = certified_bivariate_degree(first_equation, config.min_precision) else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };
    let Ok(second_degree) = certified_bivariate_degree(second_equation, config.min_precision)
    else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };
    if first_degree.is_none() || second_degree.is_none() {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::Constructed,
            retained_parameter,
            eliminated_parameter,
            0,
            vec![CurveIntersectionResultantSample {
                parameter_value: Real::zero(),
                resultant: Real::zero(),
            }],
            vec![Real::zero()],
            None,
        );
    }
    let first_degree = first_degree.expect("nonzero bivariate equation has a bidegree");
    let second_degree = second_degree.expect("nonzero bivariate equation has a bidegree");
    let (first_retained_degree, first_eliminated_degree) =
        oriented_bidegree(first_degree, retained_parameter);
    let (second_retained_degree, second_eliminated_degree) =
        oriented_bidegree(second_degree, retained_parameter);
    let degree_bound = second_eliminated_degree * first_retained_degree
        + first_eliminated_degree * second_retained_degree;
    if degree_bound > config.max_resultant_degree {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::DegreeBoundExceeded,
            retained_parameter,
            eliminated_parameter,
            degree_bound,
            Vec::new(),
            Vec::new(),
            None,
        );
    }

    // Rational coefficient systems with a modest determinant dimension are
    // substantially cheaper to eliminate as one polynomial determinant than
    // as `degree_bound + 1` independent exact determinants followed by dense
    // interpolation. Equal eliminated degrees use the half-size Bezout matrix;
    // other small systems retain sparse Sylvester rows. Both subset programs
    // are division-free and have bounded `O(n 2^n)` state. Keep the sampled
    // report contract by evaluating the constructed determinant at the same
    // degree-preserving parameter schedule used by the generic path.
    let sylvester_dimension = first_eliminated_degree + second_eliminated_degree;
    let rational_coefficients = bivariate_has_only_rational_coefficients(first_equation)
        && bivariate_has_only_rational_coefficients(second_equation);
    let symbolic_resultant = rational_coefficients
        .then(|| {
            if first_eliminated_degree == second_eliminated_degree && first_eliminated_degree <= 12
            {
                symbolic_bivariate_bezout_resultant(
                    first_equation,
                    second_equation,
                    retained_parameter,
                    first_eliminated_degree,
                )
            } else if sylvester_dimension <= 12 {
                symbolic_bivariate_sylvester_resultant(
                    first_equation,
                    second_equation,
                    retained_parameter,
                    first_eliminated_degree,
                    second_eliminated_degree,
                )
            } else {
                None
            }
        })
        .flatten();
    if let Some(resultant_coefficients) = symbolic_resultant {
        let Ok(resultant_coefficients) =
            trim_trailing_zeroes(resultant_coefficients, config.min_precision)
        else {
            return undecided_report(retained_parameter, eliminated_parameter);
        };
        let Some(samples) = resultant_polynomial_samples(
            first_equation,
            second_equation,
            retained_parameter,
            first_eliminated_degree,
            second_eliminated_degree,
            degree_bound,
            &resultant_coefficients,
            config.min_precision,
        ) else {
            return undecided_report(retained_parameter, eliminated_parameter);
        };
        return curve_resultant_report(
            CurveIntersectionResultantStatus::Constructed,
            retained_parameter,
            eliminated_parameter,
            degree_bound,
            samples,
            resultant_coefficients,
            None,
        );
    }

    let mut samples = Vec::with_capacity(degree_bound + 1);
    let mut index = 0_usize;
    while samples.len() <= degree_bound {
        let parameter_value = Real::from(index as u64);
        index += 1;
        let mut first = evaluate_bivariate_at_retained_parameter(
            first_equation,
            &parameter_value,
            retained_parameter,
        );
        let mut second = evaluate_bivariate_at_retained_parameter(
            second_equation,
            &parameter_value,
            retained_parameter,
        );
        // Existing curve-resultant callers define both coordinate differences
        // as retained minus eliminated. Preserve that orientation when the
        // second authored parameter is retained.
        if retained_parameter == CurveResultantParameter::Second {
            first
                .iter_mut()
                .for_each(|coefficient| *coefficient = -coefficient.clone());
            second
                .iter_mut()
                .for_each(|coefficient| *coefficient = -coefficient.clone());
        }
        let (Ok(first_sample_degree), Ok(second_sample_degree)) = (
            certified_nonzero_degree(&first, config.min_precision),
            certified_nonzero_degree(&second, config.min_precision),
        ) else {
            return undecided_report(retained_parameter, eliminated_parameter);
        };
        // Specializing the retained parameter may cancel an eliminated
        // leading coefficient. A lower-degree Sylvester determinant is not the
        // specialization of the generic resultant, so replace that sample.
        if first_sample_degree != Some(first_eliminated_degree)
            || second_sample_degree != Some(second_eliminated_degree)
        {
            continue;
        }
        let resultant =
            match resultant_univariate_polynomials(&first, &second, config.min_precision) {
                Ok(report) => report.resultant,
                Err(error) => {
                    return curve_resultant_report(
                        CurveIntersectionResultantStatus::ResultantError,
                        retained_parameter,
                        eliminated_parameter,
                        degree_bound,
                        samples,
                        Vec::new(),
                        Some(error),
                    );
                }
            };
        samples.push(CurveIntersectionResultantSample {
            parameter_value,
            resultant,
        });
    }

    let Some(resultant_coefficients) = interpolate_samples(&samples, config.min_precision) else {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::InterpolationDivisionFailed,
            retained_parameter,
            eliminated_parameter,
            degree_bound,
            samples,
            Vec::new(),
            None,
        );
    };
    let Ok(resultant_coefficients) =
        trim_trailing_zeroes(resultant_coefficients, config.min_precision)
    else {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::UndecidedCoefficient,
            retained_parameter,
            eliminated_parameter,
            degree_bound,
            samples,
            Vec::new(),
            None,
        );
    };
    curve_resultant_report(
        CurveIntersectionResultantStatus::Constructed,
        retained_parameter,
        eliminated_parameter,
        degree_bound,
        samples,
        resultant_coefficients,
        None,
    )
}

fn bivariate_has_only_rational_coefficients(polynomial: &BivariatePolynomial) -> bool {
    polynomial
        .coefficients
        .iter()
        .flatten()
        .all(|coefficient| coefficient.exact_rational_ref().is_some())
}

fn symbolic_bivariate_bezout_resultant(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    eliminated_degree: usize,
) -> Option<Vec<Real>> {
    let mut first = bivariate_fiber_coefficient_polynomials(first, retained_parameter);
    let mut second = bivariate_fiber_coefficient_polynomials(second, retained_parameter);
    first.truncate(eliminated_degree + 1);
    second.truncate(eliminated_degree + 1);
    if first.len() != eliminated_degree + 1 || second.len() != eliminated_degree + 1 {
        return None;
    }
    let mut matrix = vec![Vec::<Real>::new(); eliminated_degree.checked_mul(eliminated_degree)?];
    for high in 1..=eliminated_degree {
        for low in 0..high {
            let cross = subtract_exact_polynomials(
                &multiply_exact_polynomials(&first[high], &second[low]),
                &multiply_exact_polynomials(&first[low], &second[high]),
            );
            if exact_polynomial_is_zero(&cross) {
                continue;
            }
            for offset in 0..(high - low) {
                let row = high - 1 - offset;
                let column = low + offset;
                let index = row * eliminated_degree + column;
                matrix[index] = add_exact_polynomials(&matrix[index], &cross);
            }
        }
    }
    let mut determinant = determinant_polynomial_matrix(&matrix, eliminated_degree)?;
    if !(eliminated_degree * eliminated_degree.saturating_sub(1) / 2).is_multiple_of(2) {
        determinant
            .iter_mut()
            .for_each(|coefficient| *coefficient = -coefficient.clone());
    }
    Some(determinant)
}

fn symbolic_bivariate_sylvester_resultant(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    first_eliminated_degree: usize,
    second_eliminated_degree: usize,
) -> Option<Vec<Real>> {
    let mut first = bivariate_fiber_coefficient_polynomials(first, retained_parameter);
    let mut second = bivariate_fiber_coefficient_polynomials(second, retained_parameter);
    first.truncate(first_eliminated_degree + 1);
    second.truncate(second_eliminated_degree + 1);
    let dimension = first_eliminated_degree.checked_add(second_eliminated_degree)?;
    let mut matrix = vec![Vec::<Real>::new(); dimension.checked_mul(dimension)?];
    let negate = retained_parameter == CurveResultantParameter::Second;
    let oriented = |polynomial: &[Real]| {
        if negate {
            polynomial
                .iter()
                .map(|coefficient| -coefficient.clone())
                .collect()
        } else {
            polynomial.to_vec()
        }
    };
    for row in 0..second_eliminated_degree {
        for (degree, coefficient) in first.iter().enumerate() {
            matrix[row * dimension + row + first_eliminated_degree - degree] =
                oriented(coefficient);
        }
    }
    for row in 0..first_eliminated_degree {
        for (degree, coefficient) in second.iter().enumerate() {
            matrix[(second_eliminated_degree + row) * dimension + row + second_eliminated_degree
                - degree] = oriented(coefficient);
        }
    }

    determinant_polynomial_matrix(&matrix, dimension)
}

fn determinant_polynomial_matrix(matrix: &[Vec<Real>], dimension: usize) -> Option<Vec<Real>> {
    if matrix.len() != dimension.checked_mul(dimension)? {
        return None;
    }
    let state_count = 1_usize.checked_shl(u32::try_from(dimension).ok()?)?;
    let mut partials = vec![None; state_count];
    partials[0] = Some(vec![Real::one()]);
    for mask in 0..state_count {
        let row = usize::try_from(mask.count_ones()).ok()?;
        if row == dimension {
            continue;
        }
        let Some(mut partial) = partials[mask].take() else {
            continue;
        };
        trim_exact_polynomial_in_place(&mut partial);
        if exact_polynomial_is_zero(&partial) {
            continue;
        }
        for column in 0..dimension {
            let column_bit = 1_usize.checked_shl(u32::try_from(column).ok()?)?;
            if mask & column_bit != 0 {
                continue;
            }
            let entry = &matrix[row * dimension + column];
            if entry.is_empty() || exact_polynomial_is_zero(entry) {
                continue;
            }
            let sign_is_negative = (mask >> (column + 1)).count_ones() % 2 != 0;
            add_signed_polynomial_product(
                partials[mask | column_bit].get_or_insert_with(Vec::new),
                &partial,
                entry,
                sign_is_negative,
            );
        }
    }
    let mut determinant = partials.pop().flatten()?;
    trim_exact_polynomial_in_place(&mut determinant);
    Some(determinant)
}

fn add_signed_polynomial_product(
    target: &mut Vec<Real>,
    first: &[Real],
    second: &[Real],
    subtract: bool,
) {
    let required = first.len() + second.len() - 1;
    if target.len() < required {
        target.resize_with(required, Real::zero);
    }
    for (first_power, first_coefficient) in first.iter().enumerate() {
        if exact_real_is_zero(first_coefficient) {
            continue;
        }
        for (second_power, second_coefficient) in second.iter().enumerate() {
            if exact_real_is_zero(second_coefficient) {
                continue;
            }
            let term = first_coefficient * second_coefficient;
            if subtract {
                target[first_power + second_power] -= term;
            } else {
                target[first_power + second_power] += term;
            }
        }
    }
}

fn trim_exact_polynomial_in_place(polynomial: &mut Vec<Real>) {
    while polynomial.len() > 1 && polynomial.last().is_some_and(exact_real_is_zero) {
        polynomial.pop();
    }
    if polynomial.is_empty() {
        polynomial.push(Real::zero());
    }
}

#[allow(clippy::too_many_arguments)]
fn resultant_polynomial_samples(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    first_eliminated_degree: usize,
    second_eliminated_degree: usize,
    degree_bound: usize,
    resultant: &[Real],
    min_precision: i32,
) -> Option<Vec<CurveIntersectionResultantSample>> {
    let mut samples = Vec::with_capacity(degree_bound + 1);
    let mut index = 0_usize;
    while samples.len() <= degree_bound {
        let parameter_value = Real::from(index as u64);
        index += 1;
        let first_sample =
            evaluate_bivariate_at_retained_parameter(first, &parameter_value, retained_parameter);
        let second_sample =
            evaluate_bivariate_at_retained_parameter(second, &parameter_value, retained_parameter);
        if certified_nonzero_degree(&first_sample, min_precision).ok()?
            != Some(first_eliminated_degree)
            || certified_nonzero_degree(&second_sample, min_precision).ok()?
                != Some(second_eliminated_degree)
        {
            continue;
        }
        let value = resultant
            .iter()
            .rev()
            .fold(Real::zero(), |value, coefficient| {
                value * &parameter_value + coefficient
            });
        samples.push(CurveIntersectionResultantSample {
            parameter_value,
            resultant: value,
        });
    }
    Some(samples)
}

/// Constructs rational lifts for the eliminated parameter of a bivariate system.
///
/// After specializing the retained parameter, the Sylvester matrix annihilates
/// `[u^(n-1), ..., u, 1]` at every common root `u`. When that matrix has
/// nullity one, any nonzero adjugate column is a scalar multiple of this power
/// vector, so the ratio of its last two entries is exactly `u`. This function
/// interpolates those signed cofactors as exact polynomials of the retained
/// parameter. Callers must still prove that a denominator is nonzero at their
/// retained root and replay the resulting rational image against the intended
/// lifted-root interval.
pub fn linear_parameter_lifts_bivariate_polynomial_system(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    config: CurveIntersectionResultantConfig,
) -> CurveIntersectionParameterLiftReport {
    let lifted_parameter = opposite_parameter(retained_parameter);
    if bivariate_polynomial_is_empty(first_equation)
        || bivariate_polynomial_is_empty(second_equation)
    {
        return parameter_lift_report(
            CurveIntersectionParameterLiftStatus::EmptyEquation,
            retained_parameter,
            lifted_parameter,
            0,
            Vec::new(),
            None,
        );
    }
    let (Ok(first_degree), Ok(second_degree)) = (
        certified_bivariate_degree(first_equation, config.min_precision),
        certified_bivariate_degree(second_equation, config.min_precision),
    ) else {
        return parameter_lift_report(
            CurveIntersectionParameterLiftStatus::UndecidedCoefficient,
            retained_parameter,
            lifted_parameter,
            0,
            Vec::new(),
            None,
        );
    };
    let (Some(first_degree), Some(second_degree)) = (first_degree, second_degree) else {
        return parameter_lift_report(
            CurveIntersectionParameterLiftStatus::UnsupportedEliminatedDegree,
            retained_parameter,
            lifted_parameter,
            0,
            Vec::new(),
            None,
        );
    };
    let (first_retained_degree, first_lifted_degree) =
        oriented_bidegree(first_degree, retained_parameter);
    let (second_retained_degree, second_lifted_degree) =
        oriented_bidegree(second_degree, retained_parameter);
    if first_lifted_degree == 0 || second_lifted_degree == 0 {
        return parameter_lift_report(
            CurveIntersectionParameterLiftStatus::UnsupportedEliminatedDegree,
            retained_parameter,
            lifted_parameter,
            0,
            Vec::new(),
            None,
        );
    }
    let sylvester_dimension = first_lifted_degree + second_lifted_degree;
    let retained_entry_degree = first_retained_degree.max(second_retained_degree);
    let degree_bound = (sylvester_dimension - 1) * retained_entry_degree;
    if degree_bound > config.max_resultant_degree {
        return parameter_lift_report(
            CurveIntersectionParameterLiftStatus::DegreeBoundExceeded,
            retained_parameter,
            lifted_parameter,
            degree_bound,
            Vec::new(),
            None,
        );
    }

    let mut parameters = Vec::with_capacity(degree_bound + 1);
    let mut numerators = vec![Vec::with_capacity(degree_bound + 1); sylvester_dimension];
    let mut denominators = vec![Vec::with_capacity(degree_bound + 1); sylvester_dimension];
    let mut sample_index = 0_usize;
    while parameters.len() <= degree_bound {
        let parameter = Real::from(sample_index as u64);
        sample_index += 1;
        let first = evaluate_bivariate_at_retained_parameter(
            first_equation,
            &parameter,
            retained_parameter,
        );
        let second = evaluate_bivariate_at_retained_parameter(
            second_equation,
            &parameter,
            retained_parameter,
        );
        let (Ok(first_sample_degree), Ok(second_sample_degree)) = (
            certified_nonzero_degree(&first, config.min_precision),
            certified_nonzero_degree(&second, config.min_precision),
        ) else {
            return parameter_lift_report(
                CurveIntersectionParameterLiftStatus::UndecidedCoefficient,
                retained_parameter,
                lifted_parameter,
                degree_bound,
                Vec::new(),
                None,
            );
        };
        if first_sample_degree != Some(first_lifted_degree)
            || second_sample_degree != Some(second_lifted_degree)
        {
            continue;
        }
        let sylvester = sylvester_matrix(&first, &second);
        for cofactor_row in 0..sylvester_dimension {
            let numerator = match signed_cofactor(
                &sylvester,
                cofactor_row,
                sylvester_dimension - 2,
                config.min_precision,
            ) {
                Ok(value) => value,
                Err(error) => {
                    return parameter_lift_report(
                        CurveIntersectionParameterLiftStatus::DeterminantError,
                        retained_parameter,
                        lifted_parameter,
                        degree_bound,
                        Vec::new(),
                        Some(error),
                    );
                }
            };
            let denominator = match signed_cofactor(
                &sylvester,
                cofactor_row,
                sylvester_dimension - 1,
                config.min_precision,
            ) {
                Ok(value) => value,
                Err(error) => {
                    return parameter_lift_report(
                        CurveIntersectionParameterLiftStatus::DeterminantError,
                        retained_parameter,
                        lifted_parameter,
                        degree_bound,
                        Vec::new(),
                        Some(error),
                    );
                }
            };
            numerators[cofactor_row].push(numerator);
            denominators[cofactor_row].push(denominator);
        }
        parameters.push(parameter);
    }

    let mut maps = Vec::with_capacity(sylvester_dimension);
    for cofactor_row in 0..sylvester_dimension {
        let Some(numerator) = interpolate_parameter_values(
            &parameters,
            &numerators[cofactor_row],
            config.min_precision,
        ) else {
            return parameter_lift_report(
                CurveIntersectionParameterLiftStatus::InterpolationFailed,
                retained_parameter,
                lifted_parameter,
                degree_bound,
                Vec::new(),
                None,
            );
        };
        let Some(denominator) = interpolate_parameter_values(
            &parameters,
            &denominators[cofactor_row],
            config.min_precision,
        ) else {
            return parameter_lift_report(
                CurveIntersectionParameterLiftStatus::InterpolationFailed,
                retained_parameter,
                lifted_parameter,
                degree_bound,
                Vec::new(),
                None,
            );
        };
        let Ok(denominator_is_zero) =
            is_certified_zero_polynomial(&denominator, config.min_precision)
        else {
            return parameter_lift_report(
                CurveIntersectionParameterLiftStatus::UndecidedCoefficient,
                retained_parameter,
                lifted_parameter,
                degree_bound,
                Vec::new(),
                None,
            );
        };
        if !denominator_is_zero {
            maps.push(CurveIntersectionParameterLiftMap {
                cofactor_row,
                numerator_coefficients: numerator,
                denominator_coefficients: denominator,
            });
        }
    }
    parameter_lift_report(
        CurveIntersectionParameterLiftStatus::Constructed,
        retained_parameter,
        lifted_parameter,
        degree_bound,
        maps,
        None,
    )
}

/// Extracts one exact generic common component and retains both residual equations.
///
/// The first nonzero subresultant of two polynomials in the lifted parameter
/// is their generic GCD up to retained-parameter content. The degree-one path
/// publishes a primitive rational map. Split quadratics and repeated cubics
/// retain their smaller rational-linear fast paths. Otherwise the first
/// nonzero higher subresultant is made primitive and monic, divided exactly
/// from both authored equations, and returned as an implicit component. A
/// sampled-fiber GCD fast path first interpolates the low-degree GCD after
/// scaling it by one authored leading coefficient; polynomial content removes
/// that scale, and exact division of both authored equations is the authority.
/// Any failed reconstruction falls through to the complete subresultant scan.
/// This continues through the complete lifted-degree bound rather than
/// discarding irreducible or multivalued parameter correspondences. Callers
/// can apply the function again to the exact residual pair. A geometry caller
/// must still replay and decompose the finite real branches as documented by
/// [`BivariatePolynomialComponentReport`].
pub fn parameter_component_bivariate_polynomial_system(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    config: CurveIntersectionResultantConfig,
) -> BivariatePolynomialComponentReport {
    let lifted_parameter = opposite_parameter(retained_parameter);
    if bivariate_polynomial_is_empty(first_equation)
        || bivariate_polynomial_is_empty(second_equation)
    {
        return component_report(
            BivariatePolynomialComponentStatus::EmptyEquation,
            retained_parameter,
            lifted_parameter,
            0,
            Vec::new(),
            Vec::new(),
            None,
            None,
        );
    }
    let (Ok(first_degree), Ok(second_degree)) = (
        certified_bivariate_degree(first_equation, config.min_precision),
        certified_bivariate_degree(second_equation, config.min_precision),
    ) else {
        return component_report(
            BivariatePolynomialComponentStatus::UndecidedCoefficient,
            retained_parameter,
            lifted_parameter,
            0,
            Vec::new(),
            Vec::new(),
            None,
            None,
        );
    };
    let (Some(first_degree), Some(second_degree)) = (first_degree, second_degree) else {
        return component_report(
            BivariatePolynomialComponentStatus::NoSupportedComponent,
            retained_parameter,
            lifted_parameter,
            0,
            Vec::new(),
            Vec::new(),
            None,
            None,
        );
    };
    let (first_retained_degree, first_lifted_degree) =
        oriented_bidegree(first_degree, retained_parameter);
    let (second_retained_degree, second_lifted_degree) =
        oriented_bidegree(second_degree, retained_parameter);
    let terminal_degree = first_lifted_degree.min(second_lifted_degree);
    if terminal_degree == 0 {
        return component_report(
            BivariatePolynomialComponentStatus::UnsupportedLiftedDegree,
            retained_parameter,
            lifted_parameter,
            0,
            Vec::new(),
            Vec::new(),
            None,
            None,
        );
    }

    if terminal_degree >= 2 {
        let degree_bound = (second_lifted_degree - 1) * first_retained_degree
            + (first_lifted_degree - 1) * second_retained_degree;
        if degree_bound > config.max_resultant_degree {
            return component_report(
                BivariatePolynomialComponentStatus::DegreeBoundExceeded,
                retained_parameter,
                lifted_parameter,
                degree_bound,
                Vec::new(),
                Vec::new(),
                None,
                None,
            );
        }
        if let Some(report) = sampled_common_fiber_component_report(
            first_equation,
            second_equation,
            retained_parameter,
            lifted_parameter,
            first_retained_degree,
            first_lifted_degree,
            second_retained_degree,
            second_lifted_degree,
            terminal_degree,
            config,
        ) {
            return report;
        }
    }

    let (mut constant, mut linear, degree_bound) = if terminal_degree == 1 {
        let (equation, degree_bound) = if first_lifted_degree <= second_lifted_degree {
            (first_equation, first_retained_degree)
        } else {
            (second_equation, second_retained_degree)
        };
        let coefficients = bivariate_fiber_coefficient_polynomials(equation, retained_parameter);
        (
            coefficients.first().cloned().unwrap_or_default(),
            coefficients.get(1).cloned().unwrap_or_default(),
            degree_bound,
        )
    } else {
        let degree_bound = (second_lifted_degree - 1) * first_retained_degree
            + (first_lifted_degree - 1) * second_retained_degree;
        if degree_bound > config.max_resultant_degree {
            return component_report(
                BivariatePolynomialComponentStatus::DegreeBoundExceeded,
                retained_parameter,
                lifted_parameter,
                degree_bound,
                Vec::new(),
                Vec::new(),
                None,
                None,
            );
        }
        let mut parameters = Vec::with_capacity(degree_bound + 1);
        let mut constant_samples = Vec::with_capacity(degree_bound + 1);
        let mut linear_samples = Vec::with_capacity(degree_bound + 1);
        let mut sample_index = 0_usize;
        while parameters.len() <= degree_bound {
            let parameter = Real::from(sample_index as u64);
            sample_index += 1;
            let first = evaluate_bivariate_at_retained_parameter(
                first_equation,
                &parameter,
                retained_parameter,
            );
            let second = evaluate_bivariate_at_retained_parameter(
                second_equation,
                &parameter,
                retained_parameter,
            );
            let (Ok(first_sample_degree), Ok(second_sample_degree)) = (
                certified_nonzero_degree(&first, config.min_precision),
                certified_nonzero_degree(&second, config.min_precision),
            ) else {
                return component_report(
                    BivariatePolynomialComponentStatus::UndecidedCoefficient,
                    retained_parameter,
                    lifted_parameter,
                    degree_bound,
                    Vec::new(),
                    Vec::new(),
                    None,
                    None,
                );
            };
            if first_sample_degree != Some(first_lifted_degree)
                || second_sample_degree != Some(second_lifted_degree)
            {
                continue;
            }
            let coefficients =
                match first_subresultant_coefficients(&first, &second, config.min_precision) {
                    Ok(coefficients) => coefficients,
                    Err(error) => {
                        return component_report(
                            BivariatePolynomialComponentStatus::DeterminantError,
                            retained_parameter,
                            lifted_parameter,
                            degree_bound,
                            Vec::new(),
                            Vec::new(),
                            None,
                            Some(error),
                        );
                    }
                };
            constant_samples.push(coefficients[0].clone());
            linear_samples.push(coefficients[1].clone());
            parameters.push(parameter);
        }
        let Some(constant) =
            interpolate_parameter_values(&parameters, &constant_samples, config.min_precision)
        else {
            return component_report(
                BivariatePolynomialComponentStatus::InterpolationFailed,
                retained_parameter,
                lifted_parameter,
                degree_bound,
                Vec::new(),
                Vec::new(),
                None,
                None,
            );
        };
        let Some(linear) =
            interpolate_parameter_values(&parameters, &linear_samples, config.min_precision)
        else {
            return component_report(
                BivariatePolynomialComponentStatus::InterpolationFailed,
                retained_parameter,
                lifted_parameter,
                degree_bound,
                Vec::new(),
                Vec::new(),
                None,
                None,
            );
        };
        (constant, linear, degree_bound)
    };

    if exact_polynomial_is_zero(&linear) {
        if exact_polynomial_is_zero(&constant) && terminal_degree >= 2 {
            return higher_nullity_component_report(
                first_equation,
                second_equation,
                retained_parameter,
                lifted_parameter,
                first_retained_degree,
                first_lifted_degree,
                second_retained_degree,
                second_lifted_degree,
                config,
            );
        }
        return component_report(
            BivariatePolynomialComponentStatus::NoSupportedComponent,
            retained_parameter,
            lifted_parameter,
            degree_bound,
            Vec::new(),
            Vec::new(),
            None,
            None,
        );
    }
    if let Some(content) = exact_polynomial_gcd(&constant, &linear)
        && !exact_unit_polynomial(&content)
        && !exact_polynomial_is_zero(&content)
    {
        let (Some(reduced_constant), Some(reduced_linear)) = (
            divide_polynomial_exact(constant.clone(), &content),
            divide_polynomial_exact(linear.clone(), &content),
        ) else {
            return component_report(
                BivariatePolynomialComponentStatus::InterpolationFailed,
                retained_parameter,
                lifted_parameter,
                degree_bound,
                Vec::new(),
                Vec::new(),
                None,
                None,
            );
        };
        constant = reduced_constant;
        linear = reduced_linear;
    }
    let numerator_coefficients: Vec<Real> = constant.into_iter().map(std::ops::Neg::neg).collect();
    let denominator_coefficients = linear;
    let (Some(first_reduced), Some(second_reduced)) = (
        divide_bivariate_by_rational_component(
            first_equation,
            &numerator_coefficients,
            &denominator_coefficients,
            retained_parameter,
        ),
        divide_bivariate_by_rational_component(
            second_equation,
            &numerator_coefficients,
            &denominator_coefficients,
            retained_parameter,
        ),
    ) else {
        return component_report(
            BivariatePolynomialComponentStatus::DivisionFailed,
            retained_parameter,
            lifted_parameter,
            degree_bound,
            Vec::new(),
            Vec::new(),
            None,
            None,
        );
    };
    component_report(
        BivariatePolynomialComponentStatus::Rational,
        retained_parameter,
        lifted_parameter,
        degree_bound,
        numerator_coefficients,
        denominator_coefficients,
        Some([first_reduced, second_reduced]),
        None,
    )
}

#[allow(clippy::too_many_arguments)]
fn sampled_common_fiber_component_report(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    lifted_parameter: CurveResultantParameter,
    first_retained_degree: usize,
    first_lifted_degree: usize,
    second_retained_degree: usize,
    second_lifted_degree: usize,
    terminal_degree: usize,
    config: CurveIntersectionResultantConfig,
) -> Option<BivariatePolynomialComponentReport> {
    let (scale_first, interpolation_degree) = if first_retained_degree <= second_retained_degree {
        (true, first_retained_degree)
    } else {
        (false, second_retained_degree)
    };
    let sample_count = interpolation_degree + 1;
    let mut samples_by_degree = (0..=terminal_degree)
        .map(|_| Vec::new())
        .collect::<Vec<Vec<(Real, Vec<Real>)>>>();
    let maximum_samples = sample_count
        .saturating_mul(4)
        .saturating_add(terminal_degree)
        .saturating_add(8);

    for sample_index in 0..maximum_samples {
        let parameter = Real::from(sample_index as u64);
        let first = evaluate_bivariate_at_retained_parameter(
            first_equation,
            &parameter,
            retained_parameter,
        );
        let second = evaluate_bivariate_at_retained_parameter(
            second_equation,
            &parameter,
            retained_parameter,
        );
        let (Ok(Some(first_degree)), Ok(Some(second_degree))) = (
            certified_nonzero_degree(&first, config.min_precision),
            certified_nonzero_degree(&second, config.min_precision),
        ) else {
            continue;
        };
        if first_degree != first_lifted_degree || second_degree != second_lifted_degree {
            continue;
        }
        let scale = if scale_first {
            first.get(first_lifted_degree)?.clone()
        } else {
            second.get(second_lifted_degree)?.clone()
        };
        let gcd = polynomial_gcd(first, second, hyperlimit::PredicatePolicy::STRICT)?;
        let Ok(Some(gcd_degree)) = certified_nonzero_degree(&gcd, config.min_precision) else {
            continue;
        };
        if gcd_degree == 0 || gcd_degree > terminal_degree {
            continue;
        }
        let leading = gcd.get(gcd_degree)?.clone();
        let scaled = if leading == Real::one() {
            gcd.into_iter()
                .take(gcd_degree + 1)
                .map(|coefficient| coefficient * &scale)
                .collect()
        } else {
            gcd.into_iter()
                .take(gcd_degree + 1)
                .map(|coefficient| (coefficient * &scale / &leading).ok())
                .collect::<Option<Vec<_>>>()?
        };
        let samples = &mut samples_by_degree[gcd_degree];
        if samples.is_empty() {
            samples.reserve(sample_count);
        }
        samples.push((parameter, scaled));
        if samples.len() < sample_count {
            continue;
        }

        let parameters = samples
            .iter()
            .map(|(parameter, _)| parameter.clone())
            .collect::<Vec<_>>();
        let coefficients = (0..=gcd_degree)
            .map(|power| {
                let values = samples
                    .iter()
                    .map(|(_, coefficients)| coefficients[power].clone())
                    .collect::<Vec<_>>();
                interpolate_parameter_values(&parameters, &values, config.min_precision)
            })
            .collect::<Option<Vec<_>>>()?;
        if let Ok(Some(report)) = common_fiber_component_report_from_coefficients(
            first_equation,
            second_equation,
            retained_parameter,
            lifted_parameter,
            interpolation_degree,
            coefficients,
        ) {
            return Some(report);
        }
        return None;
    }
    None
}

#[allow(clippy::too_many_arguments)]
fn higher_nullity_component_report(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    lifted_parameter: CurveResultantParameter,
    first_retained_degree: usize,
    first_lifted_degree: usize,
    second_retained_degree: usize,
    second_lifted_degree: usize,
    config: CurveIntersectionResultantConfig,
) -> BivariatePolynomialComponentReport {
    let terminal_degree = first_lifted_degree.min(second_lifted_degree);
    for order in 2..=terminal_degree {
        let (coefficients, degree_bound) = match common_fiber_subresultant_coefficients(
            first_equation,
            second_equation,
            retained_parameter,
            first_retained_degree,
            first_lifted_degree,
            second_retained_degree,
            second_lifted_degree,
            order,
            config,
        ) {
            Ok(result) => result,
            Err((status, degree_bound, determinant_error)) => {
                return component_report(
                    status,
                    retained_parameter,
                    lifted_parameter,
                    degree_bound,
                    Vec::new(),
                    Vec::new(),
                    None,
                    determinant_error,
                );
            }
        };
        match common_fiber_component_report_from_coefficients(
            first_equation,
            second_equation,
            retained_parameter,
            lifted_parameter,
            degree_bound,
            coefficients,
        ) {
            Ok(Some(report)) => return report,
            Ok(None) => continue,
            Err(()) => {
                return component_report(
                    BivariatePolynomialComponentStatus::DivisionFailed,
                    retained_parameter,
                    lifted_parameter,
                    degree_bound,
                    Vec::new(),
                    Vec::new(),
                    None,
                    None,
                );
            }
        }
    }

    component_report(
        BivariatePolynomialComponentStatus::NoSupportedComponent,
        retained_parameter,
        lifted_parameter,
        0,
        Vec::new(),
        Vec::new(),
        None,
        None,
    )
}

fn common_fiber_component_report_from_coefficients(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    lifted_parameter: CurveResultantParameter,
    degree_bound: usize,
    mut coefficients: Vec<Vec<Real>>,
) -> Result<Option<BivariatePolynomialComponentReport>, ()> {
    while coefficients
        .last()
        .is_some_and(|coefficient| exact_polynomial_is_zero(coefficient))
    {
        coefficients.pop();
    }
    if coefficients.is_empty() {
        return Ok(None);
    }

    let maps = match coefficients.len() - 1 {
        1 => vec![(
            scale_exact_polynomial(&coefficients[0], &Real::from(-1_i8)),
            coefficients[1].clone(),
        )],
        2 => quadratic_rational_maps(
            coefficients[0].clone(),
            coefficients[1].clone(),
            coefficients[2].clone(),
        ),
        3 => repeated_cubic_rational_maps(coefficients.clone()),
        _ => Vec::new(),
    };
    if let Some((numerator, denominator, reduced_equations)) =
        first_dividing_rational_map(first_equation, second_equation, retained_parameter, maps)
    {
        return Ok(Some(component_report(
            BivariatePolynomialComponentStatus::Rational,
            retained_parameter,
            lifted_parameter,
            degree_bound,
            numerator,
            denominator,
            Some(reduced_equations),
            None,
        )));
    }

    let component = primitive_common_fiber_component(coefficients, retained_parameter).ok_or(())?;
    let (first_reduced, second_reduced) = (
        divide_bivariate_polynomial_exact(first_equation, &component).ok_or(())?,
        divide_bivariate_polynomial_exact(second_equation, &component).ok_or(())?,
    );
    Ok(Some(implicit_component_report(
        retained_parameter,
        lifted_parameter,
        degree_bound,
        component,
        [first_reduced, second_reduced],
    )))
}

#[allow(clippy::too_many_arguments)]
fn common_fiber_subresultant_coefficients(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    first_retained_degree: usize,
    first_lifted_degree: usize,
    second_retained_degree: usize,
    second_lifted_degree: usize,
    order: usize,
    config: CurveIntersectionResultantConfig,
) -> Result<(Vec<Vec<Real>>, usize), ComponentConstructionError> {
    let terminal_degree = first_lifted_degree.min(second_lifted_degree);
    debug_assert!(order >= 2 && terminal_degree >= order);
    if terminal_degree == order {
        let (equation, degree_bound) = if first_lifted_degree <= second_lifted_degree {
            (first_equation, first_retained_degree)
        } else {
            (second_equation, second_retained_degree)
        };
        let mut coefficients =
            bivariate_fiber_coefficient_polynomials(equation, retained_parameter);
        coefficients.resize_with(order + 1, || vec![Real::zero()]);
        coefficients.truncate(order + 1);
        return Ok((coefficients, degree_bound));
    }

    let degree_bound = (second_lifted_degree - order) * first_retained_degree
        + (first_lifted_degree - order) * second_retained_degree;
    if degree_bound > config.max_resultant_degree {
        return Err((
            BivariatePolynomialComponentStatus::DegreeBoundExceeded,
            degree_bound,
            None,
        ));
    }
    let mut parameters = Vec::with_capacity(degree_bound + 1);
    let mut coefficient_samples = (0..=order)
        .map(|_| Vec::with_capacity(degree_bound + 1))
        .collect::<Vec<_>>();
    let mut sample_index = 0_usize;
    while parameters.len() <= degree_bound {
        let parameter = Real::from(sample_index as u64);
        sample_index += 1;
        let first = evaluate_bivariate_at_retained_parameter(
            first_equation,
            &parameter,
            retained_parameter,
        );
        let second = evaluate_bivariate_at_retained_parameter(
            second_equation,
            &parameter,
            retained_parameter,
        );
        let (Ok(first_sample_degree), Ok(second_sample_degree)) = (
            certified_nonzero_degree(&first, config.min_precision),
            certified_nonzero_degree(&second, config.min_precision),
        ) else {
            return Err((
                BivariatePolynomialComponentStatus::UndecidedCoefficient,
                degree_bound,
                None,
            ));
        };
        if first_sample_degree != Some(first_lifted_degree)
            || second_sample_degree != Some(second_lifted_degree)
        {
            continue;
        }
        let coefficients =
            match subresultant_coefficients(&first, &second, order, config.min_precision) {
                Ok(coefficients) => coefficients,
                Err(error) => {
                    return Err((
                        BivariatePolynomialComponentStatus::DeterminantError,
                        degree_bound,
                        Some(error),
                    ));
                }
            };
        for (samples, coefficient) in coefficient_samples.iter_mut().zip(coefficients) {
            samples.push(coefficient);
        }
        parameters.push(parameter);
    }
    let coefficients = coefficient_samples
        .into_iter()
        .map(|samples| {
            interpolate_parameter_values(&parameters, &samples, config.min_precision).ok_or((
                BivariatePolynomialComponentStatus::InterpolationFailed,
                degree_bound,
                None,
            ))
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok((coefficients, degree_bound))
}

fn quadratic_rational_maps(
    constant: Vec<Real>,
    linear: Vec<Real>,
    quadratic: Vec<Real>,
) -> Vec<(Vec<Real>, Vec<Real>)> {
    let discriminant = subtract_exact_polynomials(
        &multiply_exact_polynomials(&linear, &linear),
        &scale_exact_polynomial(
            &multiply_exact_polynomials(&constant, &quadratic),
            &Real::from(4_i8),
        ),
    );
    let Some(root) = exact_polynomial_square_root(discriminant) else {
        return Vec::new();
    };
    let negative_linear = scale_exact_polynomial(&linear, &Real::from(-1_i8));
    let denominator = scale_exact_polynomial(&quadratic, &Real::from(2_i8));
    vec![
        (
            add_exact_polynomials(&negative_linear, &root),
            denominator.clone(),
        ),
        (
            subtract_exact_polynomials(&negative_linear, &root),
            denominator,
        ),
    ]
}

fn repeated_cubic_rational_maps(coefficients: Vec<Vec<Real>>) -> Vec<(Vec<Real>, Vec<Real>)> {
    let [constant, linear, quadratic, cubic]: [Vec<Real>; 4] = match coefficients.try_into() {
        Ok(coefficients) => coefficients,
        Err(_) => return Vec::new(),
    };
    if exact_polynomial_is_zero(&cubic) {
        return Vec::new();
    }
    let quadratic_squared = multiply_exact_polynomials(&quadratic, &quadratic);
    let linear_squared = multiply_exact_polynomials(&linear, &linear);
    let discriminant = subtract_exact_polynomials(
        &add_exact_polynomials(
            &subtract_exact_polynomials(
                &scale_exact_polynomial(
                    &multiply_exact_polynomials(
                        &multiply_exact_polynomials(&cubic, &quadratic),
                        &multiply_exact_polynomials(&linear, &constant),
                    ),
                    &Real::from(18_i8),
                ),
                &scale_exact_polynomial(
                    &multiply_exact_polynomials(
                        &multiply_exact_polynomials(&quadratic_squared, &quadratic),
                        &constant,
                    ),
                    &Real::from(4_i8),
                ),
            ),
            &multiply_exact_polynomials(&quadratic_squared, &linear_squared),
        ),
        &add_exact_polynomials(
            &scale_exact_polynomial(
                &multiply_exact_polynomials(
                    &cubic,
                    &multiply_exact_polynomials(&linear_squared, &linear),
                ),
                &Real::from(4_i8),
            ),
            &scale_exact_polynomial(
                &multiply_exact_polynomials(
                    &multiply_exact_polynomials(&cubic, &cubic),
                    &multiply_exact_polynomials(&constant, &constant),
                ),
                &Real::from(27_i8),
            ),
        ),
    );
    if !exact_polynomial_is_zero(&discriminant) {
        return Vec::new();
    }
    let delta = subtract_exact_polynomials(
        &quadratic_squared,
        &scale_exact_polynomial(
            &multiply_exact_polynomials(&cubic, &linear),
            &Real::from(3_i8),
        ),
    );
    let repeated_numerator = subtract_exact_polynomials(
        &scale_exact_polynomial(
            &multiply_exact_polynomials(&cubic, &constant),
            &Real::from(9_i8),
        ),
        &multiply_exact_polynomials(&quadratic, &linear),
    );
    if exact_polynomial_is_zero(&delta) {
        vec![(
            scale_exact_polynomial(&quadratic, &Real::from(-1_i8)),
            scale_exact_polynomial(&cubic, &Real::from(3_i8)),
        )]
    } else {
        vec![(
            repeated_numerator,
            scale_exact_polynomial(&delta, &Real::from(2_i8)),
        )]
    }
}

fn first_dividing_rational_map(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    maps: Vec<(Vec<Real>, Vec<Real>)>,
) -> Option<(Vec<Real>, Vec<Real>, [BivariatePolynomial; 2])> {
    for (numerator, denominator) in maps {
        let (numerator, denominator) = primitive_rational_map(numerator, denominator);
        let (Some(first_reduced), Some(second_reduced)) = (
            divide_bivariate_by_rational_component(
                first_equation,
                &numerator,
                &denominator,
                retained_parameter,
            ),
            divide_bivariate_by_rational_component(
                second_equation,
                &numerator,
                &denominator,
                retained_parameter,
            ),
        ) else {
            continue;
        };
        return Some((numerator, denominator, [first_reduced, second_reduced]));
    }
    None
}

type ComponentConstructionError = (
    BivariatePolynomialComponentStatus,
    usize,
    Option<BareissError>,
);

fn primitive_rational_map(numerator: Vec<Real>, denominator: Vec<Real>) -> (Vec<Real>, Vec<Real>) {
    let Some(content) = polynomial_gcd(
        numerator.clone(),
        denominator.clone(),
        hyperlimit::PredicatePolicy::STRICT,
    ) else {
        return (numerator, denominator);
    };
    if exact_unit_polynomial(&content) || exact_polynomial_is_zero(&content) {
        return (numerator, denominator);
    }
    match (
        divide_polynomial_exact(numerator.clone(), &content),
        divide_polynomial_exact(denominator.clone(), &content),
    ) {
        (Some(numerator), Some(denominator)) => (numerator, denominator),
        _ => (numerator, denominator),
    }
}

fn exact_polynomial_square_root(mut polynomial: Vec<Real>) -> Option<Vec<Real>> {
    while polynomial.len() > 1 && polynomial.last().is_some_and(exact_real_is_zero) {
        polynomial.pop();
    }
    if exact_polynomial_is_zero(&polynomial) {
        return Some(vec![Real::zero()]);
    }
    let degree = polynomial.len() - 1;
    if !degree.is_multiple_of(2) {
        return None;
    }
    let root_degree = degree / 2;
    let leading = polynomial.last()?.clone().sqrt().ok()?;
    if exact_real_is_zero(&leading) {
        return None;
    }
    let twice_leading = Real::from(2_i8) * &leading;
    let mut root = vec![Real::zero(); root_degree + 1];
    root[root_degree] = leading;
    for power in (0..root_degree).rev() {
        let product = multiply_exact_polynomials(&root, &root);
        let known = product
            .get(root_degree + power)
            .cloned()
            .unwrap_or_else(Real::zero);
        root[power] = ((polynomial[root_degree + power].clone() - known) / &twice_leading).ok()?;
    }
    exact_polynomial_is_zero(&subtract_exact_polynomials(
        &multiply_exact_polynomials(&root, &root),
        &polynomial,
    ))
    .then_some(root)
}

fn scale_exact_polynomial(polynomial: &[Real], scale: &Real) -> Vec<Real> {
    polynomial
        .iter()
        .map(|coefficient| coefficient * scale)
        .collect()
}

fn subtract_exact_polynomials(first: &[Real], second: &[Real]) -> Vec<Real> {
    add_exact_polynomials(first, &scale_exact_polynomial(second, &Real::from(-1_i8)))
}

/// Eliminate one parameter from a pair of 2D polynomial parametric curves.
///
/// For `retained_parameter == First`, this constructs
/// `Res_u(x1(t) - x2(u), y1(t) - y2(u))`. For `Second`, the roles are swapped
/// and the returned polynomial is in the second curve's parameter. The degree
/// bound is the classical bidegree resultant bound
/// `deg_u(g)*deg_t(f) + deg_u(f)*deg_t(g)`, evaluated after certified trimming.
/// The implementation samples that many plus one exact integer parameter
/// values and interpolates with exact Lagrange basis polynomials. Sylvester's
/// determinant resultant supplies elimination evidence; the exact-decision discipline requires
/// downstream exact replay before topology accepts a candidate root.
pub fn resultant_parametric_curve_intersection(
    first: &PolynomialParametricCurve2,
    second: &PolynomialParametricCurve2,
    retained_parameter: CurveResultantParameter,
    config: CurveIntersectionResultantConfig,
) -> CurveIntersectionResultantReport {
    if first.x_coefficients.is_empty()
        || first.y_coefficients.is_empty()
        || second.x_coefficients.is_empty()
        || second.y_coefficients.is_empty()
    {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::EmptyCoordinatePolynomial,
            retained_parameter,
            opposite_parameter(retained_parameter),
            0,
            Vec::new(),
            Vec::new(),
            None,
        );
    }
    resultant_bivariate_polynomial_system(
        &parametric_coordinate_difference(&first.x_coefficients, &second.x_coefficients),
        &parametric_coordinate_difference(&first.y_coefficients, &second.y_coefficients),
        retained_parameter,
        config,
    )
}

/// Eliminate one parameter from a pair of homogeneous rational parametric curves.
///
/// This is the rational/conic counterpart to
/// [`resultant_parametric_curve_intersection`]. It constructs homogeneous
/// coordinate equations
/// `X1(t) * W2(u) - X2(u) * W1(t)` and
/// `Y1(t) * W2(u) - Y2(u) * W1(t)`, then eliminates the other parameter by
/// exact Sylvester resultants. The cross-multiplied form follows the standard
/// rational Bezier model in the Bernstein/de Casteljau construction, and
/// keeps Bezier clipping inside the exact EGC
/// rule: denominator validity and topology are not inferred from this helper;
/// downstream curve code must replay candidates against retained geometry.
pub fn resultant_rational_parametric_curve_intersection(
    first: &RationalParametricCurve2,
    second: &RationalParametricCurve2,
    retained_parameter: CurveResultantParameter,
    config: CurveIntersectionResultantConfig,
) -> CurveIntersectionResultantReport {
    if rational_curve_has_empty_polynomial(first) || rational_curve_has_empty_polynomial(second) {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::EmptyCoordinatePolynomial,
            retained_parameter,
            opposite_parameter(retained_parameter),
            0,
            Vec::new(),
            Vec::new(),
            None,
        );
    }
    let Ok(first_weight_zero) = is_certified_zero_polynomial(&first.weight, config.min_precision)
    else {
        return undecided_report(retained_parameter, opposite_parameter(retained_parameter));
    };
    let Ok(second_weight_zero) = is_certified_zero_polynomial(&second.weight, config.min_precision)
    else {
        return undecided_report(retained_parameter, opposite_parameter(retained_parameter));
    };
    if first_weight_zero || second_weight_zero {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::InvalidHomogeneousWeight,
            retained_parameter,
            opposite_parameter(retained_parameter),
            0,
            Vec::new(),
            Vec::new(),
            None,
        );
    }

    resultant_bivariate_polynomial_system(
        &rational_cross_equation(
            &first.x_numerator,
            &first.weight,
            &second.x_numerator,
            &second.weight,
        ),
        &rational_cross_equation(
            &first.y_numerator,
            &first.weight,
            &second.y_numerator,
            &second.weight,
        ),
        retained_parameter,
        config,
    )
}

fn curve_resultant_report(
    status: CurveIntersectionResultantStatus,
    retained_parameter: CurveResultantParameter,
    eliminated_parameter: CurveResultantParameter,
    degree_bound: usize,
    samples: Vec<CurveIntersectionResultantSample>,
    resultant_coefficients: Vec<Real>,
    resultant_error: Option<UnivariateResultantError>,
) -> CurveIntersectionResultantReport {
    CurveIntersectionResultantReport {
        status,
        retained_parameter,
        eliminated_parameter,
        degree_bound,
        samples,
        resultant_coefficients,
        resultant_error,
    }
}

fn parameter_lift_report(
    status: CurveIntersectionParameterLiftStatus,
    retained_parameter: CurveResultantParameter,
    lifted_parameter: CurveResultantParameter,
    degree_bound: usize,
    maps: Vec<CurveIntersectionParameterLiftMap>,
    determinant_error: Option<BareissError>,
) -> CurveIntersectionParameterLiftReport {
    CurveIntersectionParameterLiftReport {
        status,
        retained_parameter,
        lifted_parameter,
        degree_bound,
        maps,
        determinant_error,
    }
}

#[allow(clippy::too_many_arguments)]
fn component_report(
    status: BivariatePolynomialComponentStatus,
    retained_parameter: CurveResultantParameter,
    lifted_parameter: CurveResultantParameter,
    degree_bound: usize,
    numerator_coefficients: Vec<Real>,
    denominator_coefficients: Vec<Real>,
    reduced_equations: Option<[BivariatePolynomial; 2]>,
    determinant_error: Option<BareissError>,
) -> BivariatePolynomialComponentReport {
    BivariatePolynomialComponentReport {
        status,
        retained_parameter,
        lifted_parameter,
        degree_bound,
        numerator_coefficients,
        denominator_coefficients,
        implicit_component: None,
        reduced_equations,
        determinant_error,
    }
}

fn implicit_component_report(
    retained_parameter: CurveResultantParameter,
    lifted_parameter: CurveResultantParameter,
    degree_bound: usize,
    implicit_component: BivariatePolynomial,
    reduced_equations: [BivariatePolynomial; 2],
) -> BivariatePolynomialComponentReport {
    BivariatePolynomialComponentReport {
        status: BivariatePolynomialComponentStatus::Implicit,
        retained_parameter,
        lifted_parameter,
        degree_bound,
        numerator_coefficients: Vec::new(),
        denominator_coefficients: Vec::new(),
        implicit_component: Some(implicit_component),
        reduced_equations: Some(reduced_equations),
        determinant_error: None,
    }
}

fn bivariate_fiber_coefficient_polynomials(
    polynomial: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
) -> Vec<Vec<Real>> {
    match retained_parameter {
        CurveResultantParameter::First => {
            let fiber_count = polynomial
                .coefficients
                .iter()
                .map(Vec::len)
                .max()
                .unwrap_or(0);
            (0..fiber_count)
                .map(|fiber_power| {
                    polynomial
                        .coefficients
                        .iter()
                        .map(|row| row.get(fiber_power).cloned().unwrap_or_else(Real::zero))
                        .collect()
                })
                .collect()
        }
        CurveResultantParameter::Second => polynomial.coefficients.clone(),
    }
}

fn divide_bivariate_by_rational_component(
    polynomial: &BivariatePolynomial,
    numerator: &[Real],
    denominator: &[Real],
    retained_parameter: CurveResultantParameter,
) -> Option<BivariatePolynomial> {
    let mut fibers = bivariate_fiber_coefficient_polynomials(polynomial, retained_parameter);
    while fibers
        .last()
        .is_some_and(|fiber| exact_polynomial_is_zero(fiber))
    {
        fibers.pop();
    }
    if fibers.is_empty() {
        return Some(BivariatePolynomial::new(vec![vec![Real::zero()]]));
    }
    if fibers.len() < 2 || exact_polynomial_is_zero(denominator) {
        return None;
    }

    // For C(r,l)=D(r)l-N(r) and E=CQ, coefficient comparison gives
    // E_m=D Q_(m-1), E_k=D Q_(k-1)-N Q_k, and E_0=-N Q_0.
    // Descending recurrence avoids fraction-field arithmetic: every division
    // is exact in the retained-parameter polynomial ring or the report fails.
    let quotient_degree = fibers.len() - 2;
    let mut quotient = vec![vec![Real::zero()]; quotient_degree + 1];
    quotient[quotient_degree] =
        divide_polynomial_exact(fibers[quotient_degree + 1].clone(), denominator)?;
    for lifted_power in (1..=quotient_degree).rev() {
        let dividend = add_exact_polynomials(
            &fibers[lifted_power],
            &multiply_exact_polynomials(numerator, &quotient[lifted_power]),
        );
        quotient[lifted_power - 1] = divide_polynomial_exact(dividend, denominator)?;
    }
    let remainder = add_exact_polynomials(
        &fibers[0],
        &multiply_exact_polynomials(numerator, &quotient[0]),
    );
    if !exact_polynomial_is_zero(&remainder) {
        return None;
    }
    Some(bivariate_from_fiber_coefficients(
        quotient,
        retained_parameter,
    ))
}

fn bivariate_from_fiber_coefficients(
    fibers: Vec<Vec<Real>>,
    retained_parameter: CurveResultantParameter,
) -> BivariatePolynomial {
    match retained_parameter {
        CurveResultantParameter::Second => canonical_exact_bivariate(fibers),
        CurveResultantParameter::First => {
            let retained_count = fibers.iter().map(Vec::len).max().unwrap_or(0);
            let mut coefficients = vec![vec![Real::zero(); fibers.len()]; retained_count];
            for (lifted_power, fiber) in fibers.into_iter().enumerate() {
                for (retained_power, coefficient) in fiber.into_iter().enumerate() {
                    coefficients[retained_power][lifted_power] = coefficient;
                }
            }
            canonical_exact_bivariate(coefficients)
        }
    }
}

fn primitive_common_fiber_component(
    mut fibers: Vec<Vec<Real>>,
    retained_parameter: CurveResultantParameter,
) -> Option<BivariatePolynomial> {
    for fiber in &mut fibers {
        while fiber.len() > 1 && fiber.last().is_some_and(exact_real_is_zero) {
            fiber.pop();
        }
    }
    let mut nonzero = fibers
        .iter()
        .filter(|fiber| !exact_polynomial_is_zero(fiber));
    let mut content = nonzero.next()?.clone();
    for fiber in nonzero {
        content = polynomial_gcd(content, fiber.clone(), hyperlimit::PredicatePolicy::STRICT)?;
        if exact_unit_polynomial(&content) {
            break;
        }
    }
    if !exact_unit_polynomial(&content) && !exact_polynomial_is_zero(&content) {
        for fiber in &mut fibers {
            if !exact_polynomial_is_zero(fiber) {
                *fiber = divide_polynomial_exact(fiber.clone(), &content)?;
            }
        }
    }
    let mut component = bivariate_from_fiber_coefficients(fibers, retained_parameter);
    let (_, _, leading) = leading_bivariate_term(&component.coefficients)?;
    if leading != Real::one() {
        for coefficient in component.coefficients.iter_mut().flatten() {
            *coefficient = (coefficient.clone() / &leading).ok()?;
        }
    }
    (!exact_bivariate_is_zero(&component)).then_some(component)
}

/// Returns the exact quotient when one bivariate polynomial divides another.
///
/// The divisor and dividend are canonicalized first. Multivariate long
/// division proceeds over the exact `Real` field and succeeds only when every
/// remainder coefficient is certified zero. No predicate policy or
/// approximate equality participates. A zero divisor is rejected; a zero
/// dividend has the canonical zero quotient.
pub fn divide_bivariate_polynomial_exact(
    dividend: &BivariatePolynomial,
    divisor: &BivariatePolynomial,
) -> Option<BivariatePolynomial> {
    let divisor = canonical_exact_bivariate(divisor.coefficients.clone());
    let (divisor_first_power, divisor_second_power, divisor_leading) =
        leading_bivariate_term(&divisor.coefficients)?;
    let mut remainder = canonical_exact_bivariate(dividend.coefficients.clone()).coefficients;
    let mut quotient = Vec::<Vec<Real>>::new();
    while let Some((first_power, second_power, leading)) = leading_bivariate_term(&remainder) {
        if first_power < divisor_first_power || second_power < divisor_second_power {
            return None;
        }
        let quotient_first_power = first_power - divisor_first_power;
        let quotient_second_power = second_power - divisor_second_power;
        let scale = (leading / &divisor_leading).ok()?;
        if quotient.len() <= quotient_first_power {
            quotient.resize_with(quotient_first_power + 1, Vec::new);
        }
        if quotient[quotient_first_power].len() <= quotient_second_power {
            quotient[quotient_first_power].resize_with(quotient_second_power + 1, Real::zero);
        }
        quotient[quotient_first_power][quotient_second_power] += &scale;
        for (divisor_first, row) in divisor.coefficients.iter().enumerate() {
            let target_first = quotient_first_power + divisor_first;
            if remainder.len() <= target_first {
                remainder.resize_with(target_first + 1, Vec::new);
            }
            for (divisor_second, coefficient) in row.iter().enumerate() {
                if exact_real_is_zero(coefficient) {
                    continue;
                }
                let target_second = quotient_second_power + divisor_second;
                if remainder[target_first].len() <= target_second {
                    remainder[target_first].resize_with(target_second + 1, Real::zero);
                }
                remainder[target_first][target_second] -= &scale * coefficient;
            }
        }
    }
    Some(canonical_exact_bivariate(quotient))
}

fn leading_bivariate_term(coefficients: &[Vec<Real>]) -> Option<(usize, usize, Real)> {
    coefficients
        .iter()
        .enumerate()
        .rev()
        .find_map(|(first_power, row)| {
            row.iter()
                .enumerate()
                .rev()
                .find(|(_, coefficient)| !exact_real_is_zero(coefficient))
                .map(|(second_power, coefficient)| (first_power, second_power, coefficient.clone()))
        })
}

fn add_exact_polynomials(first: &[Real], second: &[Real]) -> Vec<Real> {
    let mut sum = vec![Real::zero(); first.len().max(second.len())];
    for (target, coefficient) in sum.iter_mut().zip(first) {
        *target += coefficient;
    }
    for (target, coefficient) in sum.iter_mut().zip(second) {
        *target += coefficient;
    }
    while sum.len() > 1 && sum.last().is_some_and(exact_real_is_zero) {
        sum.pop();
    }
    sum
}

fn multiply_exact_polynomials(first: &[Real], second: &[Real]) -> Vec<Real> {
    if first.is_empty() || second.is_empty() {
        return vec![Real::zero()];
    }
    let mut product = vec![Real::zero(); first.len() + second.len() - 1];
    for (first_power, first_coefficient) in first.iter().enumerate() {
        for (second_power, second_coefficient) in second.iter().enumerate() {
            product[first_power + second_power] += first_coefficient * second_coefficient;
        }
    }
    while product.len() > 1 && product.last().is_some_and(exact_real_is_zero) {
        product.pop();
    }
    product
}

fn first_subresultant_coefficients(
    first: &[Real],
    second: &[Real],
    min_precision: i32,
) -> Result<[Real; 2], BareissError> {
    let coefficients = subresultant_coefficients(first, second, 1, min_precision)?;
    Ok([coefficients[0].clone(), coefficients[1].clone()])
}

fn subresultant_coefficients(
    first: &[Real],
    second: &[Real],
    order: usize,
    min_precision: i32,
) -> Result<Vec<Real>, BareissError> {
    let first_degree = first.len() - 1;
    let second_degree = second.len() - 1;
    debug_assert!(order > 0);
    debug_assert!(first_degree > order && second_degree > order);
    let row_count = first_degree + second_degree - 2 * order;
    let column_count = first_degree + second_degree - order;
    let first_row_count = second_degree - order;
    let second_row_count = first_degree - order;
    let mut matrix = vec![vec![Real::zero(); column_count]; row_count];
    for (row, matrix_row) in matrix.iter_mut().take(first_row_count).enumerate() {
        for (power, coefficient) in first.iter().enumerate() {
            matrix_row[row + first_degree - power] = coefficient.clone();
        }
    }
    for (row, matrix_row) in matrix
        .iter_mut()
        .skip(first_row_count)
        .take(second_row_count)
        .enumerate()
    {
        for (power, coefficient) in second.iter().enumerate() {
            matrix_row[row + second_degree - power] = coefficient.clone();
        }
    }

    let mut coefficients = vec![Real::zero(); order + 1];
    for (power, coefficient) in coefficients.iter_mut().enumerate() {
        let last_column = column_count - 1 - power;
        let minor = matrix
            .iter()
            .map(|row| {
                row.iter()
                    .take(row_count - 1)
                    .chain(std::iter::once(&row[last_column]))
                    .cloned()
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        *coefficient = determinant_bareiss(&minor, min_precision)?.determinant;
    }
    Ok(coefficients)
}

fn signed_cofactor(
    matrix: &[Vec<Real>],
    removed_row: usize,
    removed_column: usize,
    min_precision: i32,
) -> Result<Real, BareissError> {
    let minor = matrix
        .iter()
        .enumerate()
        .filter(|(row, _)| *row != removed_row)
        .map(|(_, row)| {
            row.iter()
                .enumerate()
                .filter(|(column, _)| *column != removed_column)
                .map(|(_, coefficient)| coefficient.clone())
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let determinant = determinant_bareiss(&minor, min_precision)?.determinant;
    Ok(if (removed_row + removed_column).is_multiple_of(2) {
        determinant
    } else {
        -determinant
    })
}

fn interpolate_parameter_values(
    parameters: &[Real],
    values: &[Real],
    min_precision: i32,
) -> Option<Vec<Real>> {
    if parameters.len() != values.len() {
        return None;
    }
    let samples = parameters
        .iter()
        .zip(values)
        .map(
            |(parameter_value, resultant)| CurveIntersectionResultantSample {
                parameter_value: parameter_value.clone(),
                resultant: resultant.clone(),
            },
        )
        .collect::<Vec<_>>();
    let coefficients = interpolate_samples(&samples, min_precision)?;
    trim_trailing_zeroes(coefficients, min_precision).ok()
}

fn undecided_report(
    retained_parameter: CurveResultantParameter,
    eliminated_parameter: CurveResultantParameter,
) -> CurveIntersectionResultantReport {
    curve_resultant_report(
        CurveIntersectionResultantStatus::UndecidedCoefficient,
        retained_parameter,
        eliminated_parameter,
        0,
        Vec::new(),
        Vec::new(),
        None,
    )
}

#[derive(Clone, Copy)]
struct BivariateDegree {
    first: usize,
    second: usize,
}

const fn opposite_parameter(parameter: CurveResultantParameter) -> CurveResultantParameter {
    match parameter {
        CurveResultantParameter::First => CurveResultantParameter::Second,
        CurveResultantParameter::Second => CurveResultantParameter::First,
    }
}

const fn oriented_bidegree(
    degree: BivariateDegree,
    retained_parameter: CurveResultantParameter,
) -> (usize, usize) {
    match retained_parameter {
        CurveResultantParameter::First => (degree.first, degree.second),
        CurveResultantParameter::Second => (degree.second, degree.first),
    }
}

fn axis_factor_failure(
    status: BivariatePolynomialAxisFactorStatus,
) -> BivariatePolynomialAxisFactorReport {
    BivariatePolynomialAxisFactorReport {
        status,
        first_parameter_factor: Vec::new(),
        second_parameter_factor: Vec::new(),
        reduced_equations: None,
    }
}

fn common_axis_factor(
    equations: &[&BivariatePolynomial; 2],
    parameter: CurveResultantParameter,
) -> Option<Vec<Real>> {
    let mut factor = vec![Real::zero()];
    for equation in equations {
        match parameter {
            CurveResultantParameter::First => {
                let column_count = equation
                    .coefficients
                    .iter()
                    .map(Vec::len)
                    .max()
                    .unwrap_or(0);
                for second_power in 0..column_count {
                    let column = equation
                        .coefficients
                        .iter()
                        .map(|row| row.get(second_power).cloned().unwrap_or_else(Real::zero))
                        .collect::<Vec<_>>();
                    factor = exact_polynomial_gcd(&factor, &column)?;
                    if exact_unit_polynomial(&factor) {
                        return Some(factor);
                    }
                }
            }
            CurveResultantParameter::Second => {
                for row in &equation.coefficients {
                    factor = exact_polynomial_gcd(&factor, row)?;
                    if exact_unit_polynomial(&factor) {
                        return Some(factor);
                    }
                }
            }
        }
    }
    Some(factor)
}

fn exact_polynomial_gcd(left: &[Real], right: &[Real]) -> Option<Vec<Real>> {
    primitive_integer_polynomial_gcd(left, right).or_else(|| {
        polynomial_gcd(
            left.to_vec(),
            right.to_vec(),
            hyperlimit::PredicatePolicy::STRICT,
        )
    })
}

fn divide_bivariate_by_axis(
    polynomial: &BivariatePolynomial,
    factor: &[Real],
    parameter: CurveResultantParameter,
) -> Option<BivariatePolynomial> {
    let coefficients = match parameter {
        CurveResultantParameter::First => {
            let column_count = polynomial
                .coefficients
                .iter()
                .map(Vec::len)
                .max()
                .unwrap_or(0);
            let mut quotient_columns = Vec::with_capacity(column_count);
            let mut row_count = 0;
            for second_power in 0..column_count {
                let column = polynomial
                    .coefficients
                    .iter()
                    .map(|row| row.get(second_power).cloned().unwrap_or_else(Real::zero))
                    .collect::<Vec<_>>();
                let quotient = divide_polynomial_exact(column, factor)?;
                row_count = row_count.max(quotient.len());
                quotient_columns.push(quotient);
            }
            let mut coefficients = vec![vec![Real::zero(); column_count]; row_count];
            for (second_power, column) in quotient_columns.into_iter().enumerate() {
                for (first_power, coefficient) in column.into_iter().enumerate() {
                    coefficients[first_power][second_power] = coefficient;
                }
            }
            coefficients
        }
        CurveResultantParameter::Second => polynomial
            .coefficients
            .iter()
            .map(|row| divide_polynomial_exact(row.clone(), factor))
            .collect::<Option<Vec<_>>>()?,
    };
    Some(canonical_exact_bivariate(coefficients))
}

fn divide_polynomial_exact(dividend: Vec<Real>, divisor: &[Real]) -> Option<Vec<Real>> {
    let (quotient, remainder) =
        polynomial_div_rem(dividend, divisor, hyperlimit::PredicatePolicy::STRICT)?;
    exact_polynomial_is_zero(&remainder).then_some(quotient)
}

fn canonical_exact_bivariate(mut coefficients: Vec<Vec<Real>>) -> BivariatePolynomial {
    for row in &mut coefficients {
        while row.last().is_some_and(exact_real_is_zero) {
            row.pop();
        }
    }
    while coefficients.last().is_some_and(Vec::is_empty) {
        coefficients.pop();
    }
    if coefficients.is_empty() {
        coefficients.push(vec![Real::zero()]);
    }
    BivariatePolynomial::new(coefficients)
}

fn exact_bivariate_is_zero(polynomial: &BivariatePolynomial) -> bool {
    polynomial
        .coefficients
        .iter()
        .flatten()
        .all(exact_real_is_zero)
}

fn exact_polynomial_is_zero(polynomial: &[Real]) -> bool {
    polynomial.iter().all(exact_real_is_zero)
}

fn exact_unit_polynomial(polynomial: &[Real]) -> bool {
    polynomial.len() == 1
        && polynomial[0]
            .exact_rational_ref()
            .is_some_and(|coefficient| coefficient.is_one())
}

fn exact_real_is_zero(value: &Real) -> bool {
    if let Some(coefficient) = value.exact_rational_ref() {
        return coefficient.is_zero();
    }
    matches!(
        value.certified_sign_until(hyperlimit::PredicatePolicy::MAX_REFINEMENT_PRECISION),
        CertifiedRealSign::Known {
            sign: RealSign::Zero,
            ..
        }
    )
}

fn bivariate_polynomial_is_empty(polynomial: &BivariatePolynomial) -> bool {
    polynomial.coefficients.is_empty() || polynomial.coefficients.iter().all(Vec::is_empty)
}

fn certified_bivariate_degree(
    polynomial: &BivariatePolynomial,
    min_precision: i32,
) -> Result<Option<BivariateDegree>, ()> {
    let first = certified_first_parameter_degree(polynomial, min_precision)?;
    let second = certified_second_parameter_degree(polynomial, min_precision)?;
    match (first, second) {
        (Some(first), Some(second)) => Ok(Some(BivariateDegree { first, second })),
        (None, None) => Ok(None),
        _ => unreachable!("a nonzero bivariate polynomial has both axis degrees"),
    }
}

fn certified_first_parameter_degree(
    polynomial: &BivariatePolynomial,
    min_precision: i32,
) -> Result<Option<usize>, ()> {
    for (index, row) in polynomial.coefficients.iter().enumerate().rev() {
        let mut unknown = false;
        for coefficient in row {
            match coefficient.certified_sign_until(min_precision) {
                CertifiedRealSign::Known {
                    sign: RealSign::Zero,
                    ..
                } => {}
                CertifiedRealSign::Known { .. } => return Ok(Some(index)),
                CertifiedRealSign::Unknown { .. } => unknown = true,
            }
        }
        if unknown {
            return Err(());
        }
    }
    Ok(None)
}

fn certified_second_parameter_degree(
    polynomial: &BivariatePolynomial,
    min_precision: i32,
) -> Result<Option<usize>, ()> {
    let coefficient_count = polynomial
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    for index in (0..coefficient_count).rev() {
        let mut unknown = false;
        for row in &polynomial.coefficients {
            let Some(coefficient) = row.get(index) else {
                continue;
            };
            match coefficient.certified_sign_until(min_precision) {
                CertifiedRealSign::Known {
                    sign: RealSign::Zero,
                    ..
                } => {}
                CertifiedRealSign::Known { .. } => return Ok(Some(index)),
                CertifiedRealSign::Unknown { .. } => unknown = true,
            }
        }
        if unknown {
            return Err(());
        }
    }
    Ok(None)
}

fn certified_nonzero_degree(
    coefficients: &[Real],
    min_precision: i32,
) -> Result<Option<usize>, ()> {
    for (index, coefficient) in coefficients.iter().enumerate().rev() {
        match coefficient.certified_sign_until(min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => {}
            CertifiedRealSign::Known { .. } => return Ok(Some(index)),
            CertifiedRealSign::Unknown { .. } => return Err(()),
        }
    }
    Ok(None)
}

fn trim_trailing_zeroes(coefficients: Vec<Real>, min_precision: i32) -> Result<Vec<Real>, ()> {
    for (index, coefficient) in coefficients.iter().enumerate().rev() {
        match coefficient.certified_sign_until(min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => {}
            CertifiedRealSign::Known { .. } => return Ok(coefficients[..=index].to_vec()),
            CertifiedRealSign::Unknown { .. } => return Err(()),
        }
    }
    Ok(vec![Real::zero()])
}

fn rational_curve_has_empty_polynomial(curve: &RationalParametricCurve2) -> bool {
    curve.x_numerator.is_empty() || curve.y_numerator.is_empty() || curve.weight.is_empty()
}

fn is_certified_zero_polynomial(coefficients: &[Real], min_precision: i32) -> Result<bool, ()> {
    for coefficient in coefficients {
        match coefficient.certified_sign_until(min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => {}
            CertifiedRealSign::Known { .. } => return Ok(false),
            CertifiedRealSign::Unknown { .. } => return Err(()),
        }
    }
    Ok(true)
}

fn parametric_coordinate_difference(first: &[Real], second: &[Real]) -> BivariatePolynomial {
    let mut coefficients = vec![vec![Real::zero(); second.len()]; first.len()];
    for (index, coefficient) in first.iter().enumerate() {
        coefficients[index][0] += coefficient;
    }
    for (index, coefficient) in second.iter().enumerate() {
        coefficients[0][index] -= coefficient;
    }
    BivariatePolynomial::new(coefficients)
}

fn rational_cross_equation(
    first_coordinate: &[Real],
    first_weight: &[Real],
    second_coordinate: &[Real],
    second_weight: &[Real],
) -> BivariatePolynomial {
    let first_count = first_coordinate.len().max(first_weight.len());
    let second_count = second_coordinate.len().max(second_weight.len());
    let mut coefficients = vec![vec![Real::zero(); second_count]; first_count];
    for (first_index, first_coordinate) in first_coordinate.iter().enumerate() {
        for (second_index, second_weight) in second_weight.iter().enumerate() {
            coefficients[first_index][second_index] += first_coordinate * second_weight;
        }
    }
    for (first_index, first_weight) in first_weight.iter().enumerate() {
        for (second_index, second_coordinate) in second_coordinate.iter().enumerate() {
            coefficients[first_index][second_index] -= first_weight * second_coordinate;
        }
    }
    BivariatePolynomial::new(coefficients)
}

fn evaluate_bivariate_at_retained_parameter(
    polynomial: &BivariatePolynomial,
    parameter: &Real,
    retained_parameter: CurveResultantParameter,
) -> Vec<Real> {
    match retained_parameter {
        CurveResultantParameter::First => {
            let coefficient_count = polynomial
                .coefficients
                .iter()
                .map(Vec::len)
                .max()
                .unwrap_or(0);
            (0..coefficient_count)
                .map(|second_power| {
                    polynomial
                        .coefficients
                        .iter()
                        .rev()
                        .fold(Real::zero(), |value, row| {
                            value * parameter
                                + row.get(second_power).cloned().unwrap_or_else(Real::zero)
                        })
                })
                .collect()
        }
        CurveResultantParameter::Second => polynomial
            .coefficients
            .iter()
            .map(|row| eval_univariate(row, parameter))
            .collect(),
    }
}

fn eval_univariate(coefficients: &[Real], value: &Real) -> Real {
    let mut result = Real::zero();
    for coefficient in coefficients.iter().rev() {
        result = result * value.clone() + coefficient.clone();
    }
    result
}

fn interpolate_samples(
    samples: &[CurveIntersectionResultantSample],
    min_precision: i32,
) -> Option<Vec<Real>> {
    let mut result = vec![Real::zero(); samples.len()];
    for (sample_index, sample) in samples.iter().enumerate() {
        let mut basis = vec![Real::one()];
        let mut denominator = Real::one();
        for (other_index, other) in samples.iter().enumerate() {
            if sample_index == other_index {
                continue;
            }
            basis = multiply_by_linear_factor(basis, -other.parameter_value.clone());
            denominator *= sample.parameter_value.clone() - other.parameter_value.clone();
        }
        let scale = (sample.resultant.clone() / denominator).ok()?;
        for (index, coefficient) in basis.into_iter().enumerate() {
            result[index] += coefficient * scale.clone();
        }
    }
    trim_trailing_zeroes(result, min_precision).ok()
}

fn multiply_by_linear_factor(polynomial: Vec<Real>, constant: Real) -> Vec<Real> {
    let mut result = vec![Real::zero(); polynomial.len() + 1];
    for (index, coefficient) in polynomial.into_iter().enumerate() {
        result[index] += coefficient.clone() * constant.clone();
        result[index + 1] += coefficient;
    }
    result
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::*;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    fn reported_rational_component(
        report: &BivariatePolynomialComponentReport,
    ) -> BivariatePolynomial {
        match report.retained_parameter {
            CurveResultantParameter::First => {
                let row_count = report
                    .numerator_coefficients
                    .len()
                    .max(report.denominator_coefficients.len());
                BivariatePolynomial::new(
                    (0..row_count)
                        .map(|power| {
                            vec![
                                -report
                                    .numerator_coefficients
                                    .get(power)
                                    .cloned()
                                    .unwrap_or_else(Real::zero),
                                report
                                    .denominator_coefficients
                                    .get(power)
                                    .cloned()
                                    .unwrap_or_else(Real::zero),
                            ]
                        })
                        .collect(),
                )
            }
            CurveResultantParameter::Second => BivariatePolynomial::new(vec![
                report
                    .numerator_coefficients
                    .iter()
                    .cloned()
                    .map(std::ops::Neg::neg)
                    .collect(),
                report.denominator_coefficients.clone(),
            ]),
        }
    }

    fn line_x_axis() -> PolynomialParametricCurve2 {
        PolynomialParametricCurve2::new(vec![real(0), real(1)], vec![real(0)])
    }

    fn multiply_axis_factors(
        polynomial: &BivariatePolynomial,
        first_factor: &[Real],
        second_factor: &[Real],
    ) -> BivariatePolynomial {
        let first_degree_count = polynomial.coefficients.len() + first_factor.len() - 1;
        let second_degree_count = polynomial
            .coefficients
            .iter()
            .map(Vec::len)
            .max()
            .unwrap_or(0)
            + second_factor.len()
            - 1;
        let mut coefficients = vec![vec![Real::zero(); second_degree_count]; first_degree_count];
        for (first_power, row) in polynomial.coefficients.iter().enumerate() {
            for (second_power, coefficient) in row.iter().enumerate() {
                for (first_delta, first_scale) in first_factor.iter().enumerate() {
                    for (second_delta, second_scale) in second_factor.iter().enumerate() {
                        coefficients[first_power + first_delta][second_power + second_delta] +=
                            coefficient * first_scale * second_scale;
                    }
                }
            }
        }
        canonical_exact_bivariate(coefficients)
    }

    fn multiply_bivariate(
        first: &BivariatePolynomial,
        second: &BivariatePolynomial,
    ) -> BivariatePolynomial {
        let first_columns = first.coefficients.iter().map(Vec::len).max().unwrap_or(0);
        let second_columns = second.coefficients.iter().map(Vec::len).max().unwrap_or(0);
        let mut coefficients = vec![
            vec![Real::zero(); first_columns + second_columns - 1];
            first.coefficients.len() + second.coefficients.len() - 1
        ];
        for (first_power, first_row) in first.coefficients.iter().enumerate() {
            for (second_power, second_row) in second.coefficients.iter().enumerate() {
                for (first_column, first_coefficient) in first_row.iter().enumerate() {
                    for (second_column, second_coefficient) in second_row.iter().enumerate() {
                        coefficients[first_power + second_power][first_column + second_column] +=
                            first_coefficient * second_coefficient;
                    }
                }
            }
        }
        canonical_exact_bivariate(coefficients)
    }

    fn assert_bivariate_exactly_equal(left: &BivariatePolynomial, right: &BivariatePolynomial) {
        let first_count = left.coefficients.len().max(right.coefficients.len());
        let second_count = left
            .coefficients
            .iter()
            .chain(&right.coefficients)
            .map(Vec::len)
            .max()
            .unwrap_or(0);
        for first_power in 0..first_count {
            for second_power in 0..second_count {
                let left = left
                    .coefficients
                    .get(first_power)
                    .and_then(|row| row.get(second_power))
                    .cloned()
                    .unwrap_or_else(Real::zero);
                let right = right
                    .coefficients
                    .get(first_power)
                    .and_then(|row| row.get(second_power))
                    .cloned()
                    .unwrap_or_else(Real::zero);
                assert!(matches!(
                    (left - right).certified_sign_until(-512),
                    CertifiedRealSign::Known {
                        sign: RealSign::Zero,
                        ..
                    }
                ));
            }
        }
    }

    fn swap_bivariate(polynomial: &BivariatePolynomial) -> BivariatePolynomial {
        let second_count = polynomial
            .coefficients
            .iter()
            .map(Vec::len)
            .max()
            .unwrap_or(0);
        let mut coefficients =
            vec![vec![Real::zero(); polynomial.coefficients.len()]; second_count];
        for (first_power, row) in polynomial.coefficients.iter().enumerate() {
            for (second_power, coefficient) in row.iter().enumerate() {
                coefficients[second_power][first_power] = coefficient.clone();
            }
        }
        canonical_exact_bivariate(coefficients)
    }

    #[test]
    fn curve_resultant_finds_parabola_horizontal_intersections() {
        let parabola = PolynomialParametricCurve2::new(
            vec![real(0), real(1)],
            vec![real(0), real(0), real(1)],
        );
        let horizontal = PolynomialParametricCurve2::new(vec![real(0), real(1)], vec![real(1)]);

        let report = resultant_parametric_curve_intersection(
            &parabola,
            &horizontal,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
        assert_eq!(report.degree_bound, 2);
        assert_eq!(
            report.resultant_coefficients,
            vec![real(-1), real(0), real(1)]
        );
        assert_eq!(report.samples.len(), 3);
    }

    #[test]
    fn curve_resultant_reports_parallel_disjoint_line_constant() {
        let first = line_x_axis();
        let second = PolynomialParametricCurve2::new(vec![real(0), real(1)], vec![real(1)]);

        let report = resultant_parametric_curve_intersection(
            &first,
            &second,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
        assert_eq!(report.resultant_coefficients, vec![real(-1)]);
    }

    #[test]
    fn curve_resultant_retaining_second_swaps_parameters() {
        let vertical = PolynomialParametricCurve2::new(vec![real(2)], vec![real(0), real(1)]);
        let horizontal = line_x_axis();

        let report = resultant_parametric_curve_intersection(
            &vertical,
            &horizontal,
            CurveResultantParameter::Second,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
        assert_eq!(report.retained_parameter, CurveResultantParameter::Second);
        assert_eq!(report.eliminated_parameter, CurveResultantParameter::First);
        assert_eq!(report.resultant_coefficients, vec![real(-2), real(1)]);
    }

    #[test]
    fn bivariate_system_eliminates_either_parameter_with_the_same_root() {
        // `u-t=0` and `u+t-1=0` meet only at `t=u=1/2`.
        let first = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
        let second = BivariatePolynomial::new(vec![vec![real(-1), real(1)], vec![real(1)]]);

        for retained_parameter in [
            CurveResultantParameter::First,
            CurveResultantParameter::Second,
        ] {
            let report = resultant_bivariate_polynomial_system(
                &first,
                &second,
                retained_parameter,
                CurveIntersectionResultantConfig::default(),
            );
            assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
            assert_eq!(report.degree_bound, 2);
            assert_eq!(report.resultant_coefficients.len(), 2);
            assert_eq!(
                &report.resultant_coefficients[0] * real(2),
                -report.resultant_coefficients[1].clone()
            );
        }
    }

    #[test]
    fn equal_degree_bezout_resultant_preserves_exact_orientation_on_either_axis() {
        // Eliminating `u` from `u^2-t` and `u^2-1` gives `(t-1)^2`.
        // Degree two exercises the sign correction between the Bezout
        // determinant convention and `resultant(first, second)`.
        let first = BivariatePolynomial::new(vec![vec![real(0), real(0), real(1)], vec![real(-1)]]);
        let second = BivariatePolynomial::new(vec![vec![real(-1), real(0), real(1)]]);
        let expected = vec![real(1), real(-2), real(1)];

        for (first, second, retained_parameter) in [
            (
                first.clone(),
                second.clone(),
                CurveResultantParameter::First,
            ),
            (
                swap_bivariate(&first),
                swap_bivariate(&second),
                CurveResultantParameter::Second,
            ),
        ] {
            assert_eq!(
                symbolic_bivariate_bezout_resultant(&first, &second, retained_parameter, 2),
                Some(expected.clone())
            );
            let report = resultant_bivariate_polynomial_system(
                &first,
                &second,
                retained_parameter,
                CurveIntersectionResultantConfig::default(),
            );
            assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
            assert_eq!(report.resultant_coefficients, expected);
        }
    }

    #[test]
    fn equal_degree_bezout_matches_sylvester_through_degree_five() {
        for degree in 1..=5 {
            let coefficients = |salt: usize| {
                (0..=2)
                    .map(|retained_power| {
                        (0..=degree)
                            .map(|eliminated_power| {
                                let value = ((retained_power + 2) * (eliminated_power + 3)
                                    + salt * (retained_power + eliminated_power + 1))
                                    % 11;
                                real(value as i64 - 5)
                            })
                            .collect::<Vec<_>>()
                    })
                    .collect::<Vec<_>>()
            };
            let mut first = coefficients(1);
            let mut second = coefficients(3);
            first[0][degree] = real(1);
            second[0][degree] = real(-2);
            let first = BivariatePolynomial::new(first);
            let second = BivariatePolynomial::new(second);

            for (first, second, retained_parameter) in [
                (
                    first.clone(),
                    second.clone(),
                    CurveResultantParameter::First,
                ),
                (
                    swap_bivariate(&first),
                    swap_bivariate(&second),
                    CurveResultantParameter::Second,
                ),
            ] {
                assert_eq!(
                    symbolic_bivariate_bezout_resultant(
                        &first,
                        &second,
                        retained_parameter,
                        degree,
                    ),
                    symbolic_bivariate_sylvester_resultant(
                        &first,
                        &second,
                        retained_parameter,
                        degree,
                        degree,
                    ),
                    "degree {degree}, retained {retained_parameter:?}"
                );
            }
        }
    }

    #[test]
    fn bivariate_axis_factors_extract_first_parameter_content_exactly() {
        let factor = vec![real(-2), real(1)];
        let first_primitive = BivariatePolynomial::new(vec![vec![real(1), real(2)], vec![real(3)]]);
        let second_primitive =
            BivariatePolynomial::new(vec![vec![real(4)], vec![real(5), real(6)]]);
        let first = multiply_axis_factors(&first_primitive, &factor, &[real(1)]);
        let second = multiply_axis_factors(&second_primitive, &factor, &[real(1)]);

        let report = extract_bivariate_polynomial_system_axis_factors(&first, &second);

        assert_eq!(report.status, BivariatePolynomialAxisFactorStatus::Reduced);
        assert_eq!(report.first_parameter_factor, factor);
        assert_eq!(report.second_parameter_factor, vec![real(1)]);
        assert_eq!(
            report.reduced_equations,
            Some([first_primitive, second_primitive])
        );
    }

    #[test]
    fn bivariate_axis_factors_extract_both_axes_sequentially() {
        let first_factor = vec![real(-2), real(1)];
        let second_factor = vec![real(3), real(1)];
        let first_primitive = BivariatePolynomial::new(vec![vec![real(1), real(2)], vec![real(3)]]);
        let second_primitive =
            BivariatePolynomial::new(vec![vec![real(4)], vec![real(5), real(6)]]);
        let first = multiply_axis_factors(&first_primitive, &first_factor, &second_factor);
        let second = multiply_axis_factors(&second_primitive, &first_factor, &second_factor);

        let report = extract_bivariate_polynomial_system_axis_factors(&first, &second);

        assert_eq!(report.status, BivariatePolynomialAxisFactorStatus::Reduced);
        assert_eq!(report.first_parameter_factor, first_factor);
        assert_eq!(report.second_parameter_factor, second_factor);
        assert_eq!(
            report.reduced_equations,
            Some([first_primitive, second_primitive])
        );
    }

    #[test]
    fn bivariate_axis_factor_preserves_nonrational_coefficient_tower() {
        let first_factor = vec![(real(1) / real(2)).unwrap().sqrt().unwrap(), real(1)];
        let second_factor = vec![real(1)];
        let first_primitive = BivariatePolynomial::new(vec![vec![real(1), real(2)], vec![real(3)]]);
        let second_primitive =
            BivariatePolynomial::new(vec![vec![real(4)], vec![real(5), real(6)]]);
        let first = multiply_axis_factors(&first_primitive, &first_factor, &second_factor);
        let second = multiply_axis_factors(&second_primitive, &first_factor, &second_factor);

        let report = extract_bivariate_polynomial_system_axis_factors(&first, &second);

        assert_eq!(report.status, BivariatePolynomialAxisFactorStatus::Reduced);
        assert_eq!(report.first_parameter_factor, first_factor);
        assert_eq!(report.second_parameter_factor, second_factor);
        let reduced = report
            .reduced_equations
            .expect("the non-rational axis factor retains authored residuals");
        assert_bivariate_exactly_equal(
            &multiply_axis_factors(&reduced[0], &first_factor, &second_factor),
            &first,
        );
        assert_bivariate_exactly_equal(
            &multiply_axis_factors(&reduced[1], &first_factor, &second_factor),
            &second,
        );
    }

    #[test]
    fn bivariate_axis_factors_remove_independent_nonrational_equation_scales() {
        let first_factor = vec![real(-2), real(1)];
        let second_factor = vec![real(3), real(1)];
        let mut first = multiply_axis_factors(
            &BivariatePolynomial::new(vec![vec![real(1), real(2)], vec![real(3)]]),
            &first_factor,
            &second_factor,
        );
        let mut second = multiply_axis_factors(
            &BivariatePolynomial::new(vec![vec![real(4)], vec![real(5), real(6)]]),
            &first_factor,
            &second_factor,
        );
        scale_bivariate_in_place(&mut first, &real(2).sqrt().unwrap());
        scale_bivariate_in_place(&mut second, &real(3).sqrt().unwrap());

        let report = extract_bivariate_polynomial_system_axis_factors(&first, &second);

        assert_eq!(report.status, BivariatePolynomialAxisFactorStatus::Reduced);
        assert_eq!(report.first_parameter_factor, first_factor);
        assert_eq!(report.second_parameter_factor, second_factor);
        let reduced = report
            .reduced_equations
            .expect("projective scales retain both authored residual equations");
        assert_bivariate_exactly_equal(
            &multiply_axis_factors(&reduced[0], &first_factor, &second_factor),
            &first,
        );
        assert_bivariate_exactly_equal(
            &multiply_axis_factors(&reduced[1], &first_factor, &second_factor),
            &second,
        );
    }

    #[test]
    fn bivariate_axis_factors_leave_primitive_system_unallocated() {
        let first = BivariatePolynomial::new(vec![vec![real(1), real(2)], vec![real(3)]]);
        let second = BivariatePolynomial::new(vec![vec![real(4)], vec![real(5), real(6)]]);

        let report = extract_bivariate_polynomial_system_axis_factors(&first, &second);

        assert_eq!(
            report.status,
            BivariatePolynomialAxisFactorStatus::Primitive
        );
        assert_eq!(report.first_parameter_factor, vec![real(1)]);
        assert_eq!(report.second_parameter_factor, vec![real(1)]);
        assert!(report.reduced_equations.is_none());
    }

    #[test]
    fn bivariate_axis_factors_reduce_one_zero_equation_without_losing_evidence() {
        let zero = BivariatePolynomial::new(vec![vec![real(0)]]);
        let first_factor = vec![real(-2), real(1)];
        let second_factor = vec![real(3), real(1)];
        let nonzero = multiply_axis_factors(
            &BivariatePolynomial::new(vec![vec![real(5)]]),
            &first_factor,
            &second_factor,
        );

        let report = extract_bivariate_polynomial_system_axis_factors(&zero, &nonzero);

        assert_eq!(report.status, BivariatePolynomialAxisFactorStatus::Reduced);
        assert_eq!(report.first_parameter_factor, first_factor);
        assert_eq!(report.second_parameter_factor, second_factor);
        assert_eq!(
            report.reduced_equations,
            Some([
                BivariatePolynomial::new(vec![vec![real(0)]]),
                BivariatePolynomial::new(vec![vec![real(5)]])
            ])
        );
    }

    #[test]
    fn bivariate_axis_factors_report_zero_and_reduce_transcendental_content() {
        let zero = BivariatePolynomial::new(vec![vec![real(0)]]);
        let zero_report = extract_bivariate_polynomial_system_axis_factors(&zero, &zero);
        assert_eq!(
            zero_report.status,
            BivariatePolynomialAxisFactorStatus::ZeroSystem
        );
        assert!(zero_report.first_parameter_factor.is_empty());
        assert!(zero_report.second_parameter_factor.is_empty());
        assert!(zero_report.reduced_equations.is_none());

        let transcendental = BivariatePolynomial::new(vec![vec![Real::pi(), real(1)]]);
        let transcendental_report =
            extract_bivariate_polynomial_system_axis_factors(&transcendental, &zero);
        assert_eq!(
            transcendental_report.status,
            BivariatePolynomialAxisFactorStatus::Reduced
        );
        assert_eq!(transcendental_report.first_parameter_factor, vec![real(1)]);
        assert_eq!(
            transcendental_report.second_parameter_factor,
            vec![Real::pi(), real(1)]
        );
        assert_eq!(
            transcendental_report.reduced_equations,
            Some([
                BivariatePolynomial::new(vec![vec![real(1)]]),
                BivariatePolynomial::new(vec![vec![real(0)]])
            ])
        );
    }

    #[test]
    fn bivariate_rational_component_recovers_primitive_nonlinear_map() {
        // F(x,y)=(2+x)y-x(1+x), so y=x(1+x)/(2+x). Multiplying F by
        // independent linear cofactors makes the generic fiber degrees two
        // while preserving exactly one common root.
        let factor = BivariatePolynomial::new(vec![
            vec![real(0), real(2)],
            vec![real(-1), real(1)],
            vec![real(-1)],
        ]);
        let first = multiply_bivariate(
            &factor,
            &BivariatePolynomial::new(vec![vec![real(1), real(1)]]),
        );
        let second = multiply_bivariate(
            &factor,
            &BivariatePolynomial::new(vec![vec![real(3), real(1)], vec![real(1)]]),
        );
        let report = parameter_component_bivariate_polynomial_system(
            &first,
            &second,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig {
                min_precision: -64,
                max_resultant_degree: 16,
            },
        );

        assert_eq!(report.status, BivariatePolynomialComponentStatus::Rational);
        assert_eq!(report.retained_parameter, CurveResultantParameter::First);
        assert_eq!(report.lifted_parameter, CurveResultantParameter::Second);
        assert_eq!(
            eval_univariate(&report.numerator_coefficients, &real(0)),
            real(0)
        );
        let numerator_at_one = eval_univariate(&report.numerator_coefficients, &real(1));
        let denominator_at_one = eval_univariate(&report.denominator_coefficients, &real(1));
        assert_eq!(numerator_at_one * real(3), denominator_at_one * real(2));
        let component = reported_rational_component(&report);
        let reduced = report
            .reduced_equations
            .as_ref()
            .expect("a constructed component retains both residual equations");
        assert_eq!(multiply_bivariate(&component, &reduced[0]), first);
        assert_eq!(multiply_bivariate(&component, &reduced[1]), second);
        assert_eq!(
            eval_univariate(
                &evaluate_bivariate_at_retained_parameter(
                    &reduced[0],
                    &real(-2),
                    CurveResultantParameter::First,
                ),
                &real(-1),
            ),
            real(0)
        );
        assert_eq!(
            eval_univariate(
                &evaluate_bivariate_at_retained_parameter(
                    &reduced[1],
                    &real(-2),
                    CurveResultantParameter::First,
                ),
                &real(-1),
            ),
            real(0)
        );
    }

    #[test]
    fn bivariate_rational_component_preserves_nonrational_map_coefficients() {
        let alpha = (real(1) / real(2)).unwrap().sqrt().unwrap();
        let component =
            BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![-alpha.clone()]]);
        let first = multiply_bivariate(
            &component,
            &BivariatePolynomial::new(vec![vec![real(1), real(1)]]),
        );
        let second = multiply_bivariate(
            &component,
            &BivariatePolynomial::new(vec![vec![real(2)], vec![real(1)]]),
        );

        let report = parameter_component_bivariate_polynomial_system(
            &first,
            &second,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig {
                min_precision: -64,
                max_resultant_degree: 16,
            },
        );

        assert_eq!(report.status, BivariatePolynomialComponentStatus::Rational);
        let image_at_one = (eval_univariate(&report.numerator_coefficients, &real(1))
            / eval_univariate(&report.denominator_coefficients, &real(1)))
        .unwrap();
        assert!(matches!(
            (image_at_one - alpha).certified_sign_until(-512),
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            }
        ));
        let extracted = reported_rational_component(&report);
        let reduced = report
            .reduced_equations
            .expect("the non-rational component retains both residual equations");
        assert_eq!(multiply_bivariate(&extracted, &reduced[0]), first);
        assert_eq!(multiply_bivariate(&extracted, &reduced[1]), second);
    }

    #[test]
    fn bivariate_component_removes_independent_nonrational_equation_scales() {
        let component = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
        let first = multiply_bivariate(
            &component,
            &BivariatePolynomial::new(vec![vec![real(1), real(1)]]),
        );
        let second = multiply_bivariate(
            &component,
            &BivariatePolynomial::new(vec![vec![real(2)], vec![real(1)]]),
        );
        let first_scale = real(2).sqrt().unwrap();
        let second_scale = real(3).sqrt().unwrap();
        let scaled = |polynomial: &BivariatePolynomial, scale: &Real| {
            BivariatePolynomial::new(
                polynomial
                    .coefficients
                    .iter()
                    .map(|row| row.iter().map(|coefficient| coefficient * scale).collect())
                    .collect(),
            )
        };
        let first = scaled(&first, &first_scale);
        let second = scaled(&second, &second_scale);

        let report = parameter_component_bivariate_polynomial_system(
            &first,
            &second,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig {
                min_precision: -64,
                max_resultant_degree: 16,
            },
        );

        assert_eq!(report.status, BivariatePolynomialComponentStatus::Rational);
        let extracted = reported_rational_component(&report);
        let reduced = report
            .reduced_equations
            .expect("projective equation scales retain both authored residuals");
        assert_eq!(multiply_bivariate(&extracted, &reduced[0]), first);
        assert_eq!(multiply_bivariate(&extracted, &reduced[1]), second);
    }

    #[test]
    fn bivariate_component_retains_an_irreducible_quadratic_fiber() {
        let factor = BivariatePolynomial::new(vec![
            vec![real(-1), real(0), real(1)],
            vec![real(0), real(-1)],
        ]);
        let first = multiply_bivariate(
            &factor,
            &BivariatePolynomial::new(vec![vec![real(1), real(1)]]),
        );
        let second = multiply_bivariate(
            &factor,
            &BivariatePolynomial::new(vec![vec![real(2)], vec![real(1)]]),
        );
        for (first, second, retained_parameter) in [
            (
                first.clone(),
                second.clone(),
                CurveResultantParameter::First,
            ),
            (
                swap_bivariate(&first),
                swap_bivariate(&second),
                CurveResultantParameter::Second,
            ),
        ] {
            let report = parameter_component_bivariate_polynomial_system(
                &first,
                &second,
                retained_parameter,
                CurveIntersectionResultantConfig {
                    min_precision: -64,
                    max_resultant_degree: 16,
                },
            );
            assert_eq!(report.status, BivariatePolynomialComponentStatus::Implicit);
            assert!(report.numerator_coefficients.is_empty());
            assert!(report.denominator_coefficients.is_empty());
            let component = report
                .implicit_component
                .expect("the irreducible quadratic remains an implicit component");
            let reduced = report
                .reduced_equations
                .expect("the implicit component retains both exact residuals");
            assert_eq!(
                leading_bivariate_term(&component.coefficients)
                    .expect("the implicit component is nonzero")
                    .2,
                real(1)
            );
            assert_eq!(multiply_bivariate(&component, &reduced[0]), first);
            assert_eq!(multiply_bivariate(&component, &reduced[1]), second);
        }
    }

    #[test]
    fn sampled_fiber_gcd_removes_high_degree_cofactor_scale_exactly() {
        // H(t,u)=(6-3u)(t^2+t)+2u^2-8u is irreducible in either
        // parameter. The unrelated cofactors reproduce the bidegrees of the
        // analytic-parallel fixture that motivated the sampled fast path:
        // (7,6) and (8,12). Scaling each monic specialized GCD by an authored
        // leading coefficient leaves polynomial content, which must be
        // removed before exact two-equation division authorizes H.
        let component = BivariatePolynomial::new(vec![
            vec![real(0), real(-8), real(2)],
            vec![real(6), real(-3)],
            vec![real(6), real(-3)],
        ]);
        let first_cofactor = BivariatePolynomial::new(vec![
            vec![real(1), real(0), real(0), real(0), real(1)],
            vec![],
            vec![],
            vec![],
            vec![],
            vec![real(1)],
        ]);
        let second_cofactor = BivariatePolynomial::new(vec![
            vec![
                real(2),
                real(0),
                real(0),
                real(0),
                real(0),
                real(0),
                real(0),
                real(0),
                real(0),
                real(0),
                real(1),
            ],
            vec![],
            vec![],
            vec![],
            vec![],
            vec![],
            vec![real(1)],
        ]);
        let first = multiply_bivariate(&component, &first_cofactor);
        let second = multiply_bivariate(&component, &second_cofactor);
        let config = CurveIntersectionResultantConfig {
            min_precision: -64,
            max_resultant_degree: 256,
        };

        for (retained_parameter, expected_interpolation_degree) in [
            (CurveResultantParameter::First, 7),
            (CurveResultantParameter::Second, 6),
        ] {
            let report = parameter_component_bivariate_polynomial_system(
                &first,
                &second,
                retained_parameter,
                config,
            );
            assert_eq!(report.status, BivariatePolynomialComponentStatus::Implicit);
            assert_eq!(report.degree_bound, expected_interpolation_degree);
            let extracted = report
                .implicit_component
                .expect("the sampled fiber GCD must publish its exact component");
            let residual = report
                .reduced_equations
                .expect("the sampled component must retain exact residuals");
            assert_eq!(multiply_bivariate(&extracted, &residual[0]), first);
            assert_eq!(multiply_bivariate(&extracted, &residual[1]), second);
        }
    }

    #[test]
    fn bivariate_rational_component_extracts_two_linear_quadratic_factors() {
        // C1(x,y)=y-x and C2(x,y)=y+x-1. Their product is the generic
        // quadratic fiber GCD, while the independent cofactors keep both
        // authored lifted degrees above two so the quadratic subresultant is
        // reconstructed rather than copied from an equation.
        let first_component =
            BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
        let second_component =
            BivariatePolynomial::new(vec![vec![real(-1), real(1)], vec![real(1)]]);
        let common = multiply_bivariate(&first_component, &second_component);
        let first = multiply_bivariate(
            &common,
            &BivariatePolynomial::new(vec![vec![real(1), real(1)], vec![real(1)]]),
        );
        let second = multiply_bivariate(
            &common,
            &BivariatePolynomial::new(vec![vec![real(2), real(1)], vec![real(-1)]]),
        );
        let config = CurveIntersectionResultantConfig {
            min_precision: -64,
            max_resultant_degree: 32,
        };

        for (first, second, retained_parameter) in [
            (
                first.clone(),
                second.clone(),
                CurveResultantParameter::First,
            ),
            (
                swap_bivariate(&first),
                swap_bivariate(&second),
                CurveResultantParameter::Second,
            ),
        ] {
            let first_report = parameter_component_bivariate_polynomial_system(
                &first,
                &second,
                retained_parameter,
                config,
            );
            assert_eq!(
                first_report.status,
                BivariatePolynomialComponentStatus::Rational
            );
            let first_factor = reported_rational_component(&first_report);
            let first_reduced = first_report
                .reduced_equations
                .as_ref()
                .expect("the first quadratic factor retains its residual system");
            assert_eq!(multiply_bivariate(&first_factor, &first_reduced[0]), first);
            assert_eq!(multiply_bivariate(&first_factor, &first_reduced[1]), second);

            let second_report = parameter_component_bivariate_polynomial_system(
                &first_reduced[0],
                &first_reduced[1],
                retained_parameter,
                config,
            );
            assert_eq!(
                second_report.status,
                BivariatePolynomialComponentStatus::Rational
            );
            let retained = (real(1) / real(4)).unwrap();
            let image = |report: &BivariatePolynomialComponentReport| {
                (eval_univariate(&report.numerator_coefficients, &retained)
                    / eval_univariate(&report.denominator_coefficients, &retained))
                .unwrap()
            };
            let mut images = [image(&first_report), image(&second_report)];
            images.sort_by(|left, right| {
                left.partial_cmp(right)
                    .expect("rational component images are ordered")
            });
            assert_eq!(images, [retained.clone(), real(1) - retained]);
        }
    }

    #[test]
    fn bivariate_rational_component_extracts_repeated_quadratic_factor() {
        let component = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
        let repeated = multiply_bivariate(&component, &component);
        let first = multiply_bivariate(
            &repeated,
            &BivariatePolynomial::new(vec![vec![real(1), real(1)]]),
        );
        let second = multiply_bivariate(
            &repeated,
            &BivariatePolynomial::new(vec![vec![real(2)], vec![real(1)]]),
        );
        let config = CurveIntersectionResultantConfig {
            min_precision: -64,
            max_resultant_degree: 32,
        };

        let first_report = parameter_component_bivariate_polynomial_system(
            &first,
            &second,
            CurveResultantParameter::First,
            config,
        );
        assert_eq!(
            first_report.status,
            BivariatePolynomialComponentStatus::Rational
        );
        let first_reduced = first_report.reduced_equations.as_ref().unwrap();
        let second_report = parameter_component_bivariate_polynomial_system(
            &first_reduced[0],
            &first_reduced[1],
            CurveResultantParameter::First,
            config,
        );
        assert_eq!(
            second_report.status,
            BivariatePolynomialComponentStatus::Rational
        );
        assert_eq!(
            multiply_exact_polynomials(
                &first_report.numerator_coefficients,
                &second_report.denominator_coefficients,
            ),
            multiply_exact_polynomials(
                &second_report.numerator_coefficients,
                &first_report.denominator_coefficients,
            )
        );
    }

    #[test]
    fn bivariate_rational_component_extracts_repeated_cubic_factor() {
        let repeated_component =
            BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
        let distinct_component =
            BivariatePolynomial::new(vec![vec![real(-1), real(1)], vec![real(1)]]);
        let common = multiply_bivariate(
            &multiply_bivariate(&repeated_component, &repeated_component),
            &distinct_component,
        );
        let first = multiply_bivariate(
            &common,
            &BivariatePolynomial::new(vec![vec![real(1), real(1)], vec![real(1)]]),
        );
        let second = multiply_bivariate(
            &common,
            &BivariatePolynomial::new(vec![vec![real(2), real(1)], vec![real(-1)]]),
        );
        let config = CurveIntersectionResultantConfig {
            min_precision: -64,
            max_resultant_degree: 32,
        };

        for (first, second, retained_parameter) in [
            (
                first.clone(),
                second.clone(),
                CurveResultantParameter::First,
            ),
            (
                swap_bivariate(&first),
                swap_bivariate(&second),
                CurveResultantParameter::Second,
            ),
        ] {
            let mut residual = [first, second];
            let retained = (real(1) / real(4)).unwrap();
            let mut images = Vec::new();
            for _ in 0..3 {
                let report = parameter_component_bivariate_polynomial_system(
                    &residual[0],
                    &residual[1],
                    retained_parameter,
                    config,
                );
                assert_eq!(report.status, BivariatePolynomialComponentStatus::Rational);
                images.push(
                    (eval_univariate(&report.numerator_coefficients, &retained)
                        / eval_univariate(&report.denominator_coefficients, &retained))
                    .unwrap(),
                );
                residual = report
                    .reduced_equations
                    .expect("each cubic factor retains the exact residual");
            }
            images.sort_by(|left, right| {
                left.partial_cmp(right)
                    .expect("rational component images are ordered")
            });
            assert_eq!(
                images,
                vec![retained.clone(), retained.clone(), real(1) - retained]
            );
        }
    }

    #[test]
    fn bivariate_rational_component_extracts_triple_cubic_factor() {
        let component = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
        let common = multiply_bivariate(&multiply_bivariate(&component, &component), &component);
        let first = multiply_bivariate(
            &common,
            &BivariatePolynomial::new(vec![vec![real(1), real(1)], vec![real(1)]]),
        );
        let second = multiply_bivariate(
            &common,
            &BivariatePolynomial::new(vec![vec![real(2), real(1)], vec![real(-1)]]),
        );
        let config = CurveIntersectionResultantConfig {
            min_precision: -64,
            max_resultant_degree: 32,
        };

        for (first, second, retained_parameter) in [
            (
                first.clone(),
                second.clone(),
                CurveResultantParameter::First,
            ),
            (
                swap_bivariate(&first),
                swap_bivariate(&second),
                CurveResultantParameter::Second,
            ),
        ] {
            let mut residual = [first, second];
            let retained = (real(1) / real(4)).unwrap();
            for _ in 0..3 {
                let report = parameter_component_bivariate_polynomial_system(
                    &residual[0],
                    &residual[1],
                    retained_parameter,
                    config,
                );
                assert_eq!(report.status, BivariatePolynomialComponentStatus::Rational);
                assert_eq!(
                    (eval_univariate(&report.numerator_coefficients, &retained)
                        / eval_univariate(&report.denominator_coefficients, &retained))
                    .unwrap(),
                    retained
                );
                residual = report
                    .reduced_equations
                    .expect("each cubic factor retains the exact residual");
            }
        }
    }

    #[test]
    fn bivariate_component_retains_square_free_distinct_cubic_implicitly() {
        let first_component =
            BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
        let second_component =
            BivariatePolynomial::new(vec![vec![real(-1), real(1)], vec![real(1)]]);
        let third_component =
            BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-2)]]);
        let common = multiply_bivariate(
            &multiply_bivariate(&first_component, &second_component),
            &third_component,
        );
        let first = multiply_bivariate(
            &common,
            &BivariatePolynomial::new(vec![vec![real(1), real(1)], vec![real(1)]]),
        );
        let second = multiply_bivariate(
            &common,
            &BivariatePolynomial::new(vec![vec![real(2), real(1)], vec![real(-1)]]),
        );
        let report = parameter_component_bivariate_polynomial_system(
            &first,
            &second,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig {
                min_precision: -64,
                max_resultant_degree: 32,
            },
        );

        assert_eq!(report.status, BivariatePolynomialComponentStatus::Implicit);
        let component = report
            .implicit_component
            .expect("the square-free cubic remains an implicit component");
        let reduced = report
            .reduced_equations
            .expect("the implicit component retains both exact residuals");
        assert_eq!(
            leading_bivariate_term(&component.coefficients)
                .expect("the implicit component is nonzero")
                .2,
            real(1)
        );
        assert_eq!(multiply_bivariate(&component, &reduced[0]), first);
        assert_eq!(multiply_bivariate(&component, &reduced[1]), second);
    }

    #[test]
    fn bivariate_component_retains_degree_four_common_fiber_implicitly() {
        let components = [
            BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]),
            BivariatePolynomial::new(vec![vec![real(-1), real(1)], vec![real(1)]]),
            BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-2)]]),
            BivariatePolynomial::new(vec![vec![real(-1), real(1)], vec![real(2)]]),
        ];
        let common = components.iter().fold(
            BivariatePolynomial::new(vec![vec![real(1)]]),
            |product, component| multiply_bivariate(&product, component),
        );
        let first = common.clone();
        let second = multiply_bivariate(
            &common,
            &BivariatePolynomial::new(vec![vec![real(2), real(1)], vec![real(1)]]),
        );
        let config = CurveIntersectionResultantConfig {
            min_precision: -64,
            max_resultant_degree: 64,
        };

        for (first, second, retained_parameter) in [
            (
                first.clone(),
                second.clone(),
                CurveResultantParameter::First,
            ),
            (
                swap_bivariate(&first),
                swap_bivariate(&second),
                CurveResultantParameter::Second,
            ),
        ] {
            let report = parameter_component_bivariate_polynomial_system(
                &first,
                &second,
                retained_parameter,
                config,
            );
            assert_eq!(report.status, BivariatePolynomialComponentStatus::Implicit);
            let component = report
                .implicit_component
                .expect("the degree-four common fiber remains implicit");
            let reduced = report
                .reduced_equations
                .expect("the implicit component retains both exact residuals");
            assert_eq!(multiply_bivariate(&component, &reduced[0]), first);
            assert_eq!(multiply_bivariate(&component, &reduced[1]), second);
        }
    }

    #[test]
    fn exact_bivariate_division_replays_non_monic_ragged_products() {
        let divisor = BivariatePolynomial::new(vec![
            vec![real(2), real(-3), real(1)],
            vec![real(0), real(4)],
        ]);
        let quotient =
            BivariatePolynomial::new(vec![vec![real(-1), real(2)], vec![], vec![real(3)]]);
        let dividend = multiply_bivariate(&divisor, &quotient);

        assert_eq!(
            divide_bivariate_polynomial_exact(&dividend, &divisor),
            Some(quotient.clone())
        );
        assert!(
            divide_bivariate_polynomial_exact(
                &dividend,
                &BivariatePolynomial::new(vec![vec![real(1), real(1)], vec![real(1)],])
            )
            .is_none()
        );

        let zero = BivariatePolynomial::new(vec![vec![real(0)]]);
        assert_eq!(
            divide_bivariate_polynomial_exact(&zero, &divisor),
            Some(zero.clone())
        );
        assert!(divide_bivariate_polynomial_exact(&dividend, &zero).is_none());

        let alpha = real(2).sqrt().unwrap();
        let nonrational_divisor =
            BivariatePolynomial::new(vec![vec![alpha.clone(), real(1)], vec![real(1)]]);
        let nonrational_dividend = multiply_bivariate(&nonrational_divisor, &quotient);
        assert_bivariate_exactly_equal(
            &divide_bivariate_polynomial_exact(&nonrational_dividend, &nonrational_divisor)
                .expect("the exact Real divisor must replay"),
            &quotient,
        );
    }

    #[test]
    fn bivariate_rational_component_handles_unequal_degrees_in_both_orientations() {
        // C(x,y)=(2+x)y-(x^2+3x+1). Independent degree-one and degree-two
        // cofactors leave C as the unique generic common fiber factor while
        // exercising rectangular first-subresultant minors.
        let component = BivariatePolynomial::new(vec![
            vec![real(-1), real(2)],
            vec![real(-3), real(1)],
            vec![real(-1)],
        ]);
        let first = multiply_bivariate(
            &component,
            &BivariatePolynomial::new(vec![vec![real(1), real(1)], vec![real(1)]]),
        );
        let second = multiply_bivariate(
            &component,
            &BivariatePolynomial::new(vec![vec![real(1), real(1), real(1)]]),
        );

        for (first, second, retained_parameter) in [
            (
                first.clone(),
                second.clone(),
                CurveResultantParameter::First,
            ),
            (
                swap_bivariate(&first),
                swap_bivariate(&second),
                CurveResultantParameter::Second,
            ),
        ] {
            let report = parameter_component_bivariate_polynomial_system(
                &first,
                &second,
                retained_parameter,
                CurveIntersectionResultantConfig {
                    min_precision: -64,
                    max_resultant_degree: 32,
                },
            );
            assert_eq!(report.status, BivariatePolynomialComponentStatus::Rational);
            let component = reported_rational_component(&report);
            let reduced = report
                .reduced_equations
                .as_ref()
                .expect("both orientations retain exact residual equations");
            assert_eq!(multiply_bivariate(&component, &reduced[0]), first);
            assert_eq!(multiply_bivariate(&component, &reduced[1]), second);
            for value in [real(0), real(1), real(2), real(3)] {
                let numerator = eval_univariate(&report.numerator_coefficients, &value);
                let denominator = eval_univariate(&report.denominator_coefficients, &value);
                let component_denominator = real(2) + &value;
                let component_numerator = &value * &value + real(3) * &value + real(1);
                assert_eq!(
                    numerator * component_denominator,
                    denominator * component_numerator
                );
            }
        }
    }

    #[test]
    fn bivariate_linear_lifts_recover_either_parameter_from_adjugate_ratios() {
        // `u-t=0` and `u+t-1=0` meet only at `t=u=1/2`.
        let first = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
        let second = BivariatePolynomial::new(vec![vec![real(-1), real(1)], vec![real(1)]]);
        let half = (real(1) / real(2)).unwrap();

        for retained_parameter in [
            CurveResultantParameter::First,
            CurveResultantParameter::Second,
        ] {
            let report = linear_parameter_lifts_bivariate_polynomial_system(
                &first,
                &second,
                retained_parameter,
                CurveIntersectionResultantConfig::default(),
            );
            assert_eq!(
                report.status,
                CurveIntersectionParameterLiftStatus::Constructed
            );
            assert_eq!(report.retained_parameter, retained_parameter);
            assert_eq!(
                report.lifted_parameter,
                opposite_parameter(retained_parameter)
            );
            assert_eq!(report.degree_bound, 1);
            assert!(!report.maps.is_empty());
            assert!(report.maps.iter().any(|map| {
                let denominator = eval_univariate(&map.denominator_coefficients, &half);
                if denominator == real(0) {
                    return false;
                }
                let numerator = eval_univariate(&map.numerator_coefficients, &half);
                numerator == &half * denominator
            }));
        }
    }

    #[test]
    fn bivariate_linear_lifts_recover_a_nonlinear_common_root() {
        // `u^2-t=0` and `4u+4t-3=0` share `(t,u)=(1/4,1/2)`.
        let first = BivariatePolynomial::new(vec![vec![real(0), real(0), real(1)], vec![real(-1)]]);
        let second = BivariatePolynomial::new(vec![vec![real(-3), real(4)], vec![real(4)]]);
        let quarter = (real(1) / real(4)).unwrap();
        let half = (real(1) / real(2)).unwrap();

        for (retained_parameter, retained, lifted) in [
            (CurveResultantParameter::First, &quarter, &half),
            (CurveResultantParameter::Second, &half, &quarter),
        ] {
            let report = linear_parameter_lifts_bivariate_polynomial_system(
                &first,
                &second,
                retained_parameter,
                CurveIntersectionResultantConfig::default(),
            );
            assert_eq!(
                report.status,
                CurveIntersectionParameterLiftStatus::Constructed
            );
            assert!(report.maps.iter().any(|map| {
                let denominator = eval_univariate(&map.denominator_coefficients, retained);
                if denominator == real(0) {
                    return false;
                }
                let numerator = eval_univariate(&map.numerator_coefficients, retained);
                numerator == lifted * denominator
            }));
        }
    }

    #[test]
    fn bivariate_linear_lifts_report_nonlinear_fibers_explicitly() {
        // The first equation vanishes as a polynomial in `u` at `t=1/2`, so
        // the specialized common-root fiber is not certified by a nullity-one
        // Sylvester lift.
        let first = BivariatePolynomial::new(vec![vec![real(-1)], vec![real(2)]]);
        let second = BivariatePolynomial::new(vec![vec![real(-1), real(0), real(2)]]);
        let report = linear_parameter_lifts_bivariate_polynomial_system(
            &first,
            &second,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(
            report.status,
            CurveIntersectionParameterLiftStatus::UnsupportedEliminatedDegree
        );
        assert!(report.maps.is_empty());
    }

    #[test]
    fn bivariate_system_reports_an_identically_degenerate_equation() {
        let zero = BivariatePolynomial::new(vec![vec![real(0)]]);
        let nonzero = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
        let report = resultant_bivariate_polynomial_system(
            &zero,
            &nonzero,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
        assert_eq!(report.resultant_coefficients, vec![real(0)]);
        assert_eq!(report.samples.len(), 1);
    }

    #[test]
    fn curve_resultant_rejects_empty_inputs_and_degree_over_budget() {
        let invalid = PolynomialParametricCurve2::new(Vec::new(), vec![real(0)]);
        let line = line_x_axis();
        let invalid_report = resultant_parametric_curve_intersection(
            &invalid,
            &line,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        );
        assert_eq!(
            invalid_report.status,
            CurveIntersectionResultantStatus::EmptyCoordinatePolynomial
        );

        let quadratic = PolynomialParametricCurve2::new(
            vec![real(0), real(0), real(1)],
            vec![real(0), real(1)],
        );
        let limited = resultant_parametric_curve_intersection(
            &quadratic,
            &quadratic,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig {
                max_resultant_degree: 0,
                ..CurveIntersectionResultantConfig::default()
            },
        );
        assert_eq!(
            limited.status,
            CurveIntersectionResultantStatus::DegreeBoundExceeded
        );
    }

    #[test]
    fn rational_curve_resultant_cross_multiplies_weighted_parabola() {
        let weighted_parabola = RationalParametricCurve2::new(
            vec![real(0), real(1)],
            vec![real(0), real(0), real(1)],
            vec![real(1), real(1)],
        );
        let horizontal = RationalParametricCurve2::from_polynomial(
            &PolynomialParametricCurve2::new(vec![real(0), real(1)], vec![real(1)]),
        );

        let report = resultant_rational_parametric_curve_intersection(
            &weighted_parabola,
            &horizontal,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
        assert_eq!(
            report.resultant_coefficients,
            vec![real(-1), real(-1), real(1)]
        );
    }

    #[test]
    fn rational_curve_resultant_rejects_certified_zero_weight() {
        let invalid =
            RationalParametricCurve2::new(vec![real(0), real(1)], vec![real(1)], vec![real(0)]);
        let line = RationalParametricCurve2::from_polynomial(&line_x_axis());

        let report = resultant_rational_parametric_curve_intersection(
            &invalid,
            &line,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(
            report.status,
            CurveIntersectionResultantStatus::InvalidHomogeneousWeight
        );
    }

    #[test]
    fn rational_resultant_skips_specialized_degree_drop_samples() {
        let first = RationalParametricCurve2::new(
            vec![real(5), real(51), real(-15), real(-18)],
            vec![real(6), real(12), real(-12)],
            vec![real(1), real(3), real(-3)],
        );
        let second = RationalParametricCurve2::new(
            vec![real(28), real(160), real(-252), real(69)],
            vec![real(4), real(84), real(-84)],
            vec![real(1), real(9), real(-9)],
        );
        let report = resultant_rational_parametric_curve_intersection(
            &first,
            &second,
            CurveResultantParameter::Second,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
        let expected = [
            -1, -491, -9495, 66678, 1737203, 1859523, -33269637, 76011300, -71374833, 24098715,
            5240457, -5130702, 771282,
        ]
        .map(real);
        assert_eq!(report.resultant_coefficients.len(), expected.len());
        for (coefficient, expected_coefficient) in
            report.resultant_coefficients.iter().zip(&expected)
        {
            assert_eq!(
                coefficient * &expected[0],
                expected_coefficient * &report.resultant_coefficients[0]
            );
        }
        assert_eq!(report.samples.len(), report.degree_bound + 1);
        assert!(
            report
                .samples
                .last()
                .is_some_and(|sample| sample.parameter_value > real(report.degree_bound as i64)),
            "a degree-drop sample must be replaced, not interpolated at lower Sylvester degree"
        );
    }

    proptest! {
        #[test]
        fn generated_exact_bivariate_division_replays_ragged_products(
            divisor_coefficients in prop::collection::vec(-3_i8..=3, 1..=9),
            quotient_coefficients in prop::collection::vec(-3_i8..=3, 1..=9),
        ) {
            let polynomial = |coefficients: Vec<i8>| {
                canonical_exact_bivariate(
                    coefficients
                        .chunks(3)
                        .map(|row| row.iter().map(|coefficient| real(i64::from(*coefficient))).collect())
                        .collect(),
                )
            };
            let divisor = polynomial(divisor_coefficients);
            prop_assume!(!exact_bivariate_is_zero(&divisor));
            let quotient = polynomial(quotient_coefficients);
            let dividend = multiply_bivariate(&divisor, &quotient);

            prop_assert_eq!(
                divide_bivariate_polynomial_exact(&dividend, &divisor),
                Some(quotient)
            );
        }

        #[test]
        fn generated_exact_bivariate_division_rejects_a_perturbed_product(
            divisor_coefficients in prop::collection::vec(-3_i8..=3, 2..=9),
            quotient_coefficients in prop::collection::vec(-3_i8..=3, 1..=9),
        ) {
            let polynomial = |coefficients: Vec<i8>| {
                canonical_exact_bivariate(
                    coefficients
                        .chunks(3)
                        .map(|row| row.iter().map(|coefficient| real(i64::from(*coefficient))).collect())
                        .collect(),
                )
            };
            let divisor = polynomial(divisor_coefficients);
            let Some((first_power, second_power, _)) =
                leading_bivariate_term(&divisor.coefficients)
            else {
                prop_assume!(false);
                unreachable!();
            };
            prop_assume!(first_power != 0 || second_power != 0);
            let quotient = polynomial(quotient_coefficients);
            let mut perturbed = multiply_bivariate(&divisor, &quotient);
            if perturbed.coefficients[0].is_empty() {
                perturbed.coefficients[0].push(Real::zero());
            }
            perturbed.coefficients[0][0] += real(1);

            prop_assert!(divide_bivariate_polynomial_exact(&perturbed, &divisor).is_none());
        }

        #[test]
        fn generated_horizontal_line_intersects_parabola_at_exact_height(
            height in 1_i16..=32,
        ) {
            let height = i64::from(height);
            let parabola = PolynomialParametricCurve2::new(
                vec![real(0), real(1)],
                vec![real(0), real(0), real(1)],
            );
            let horizontal = PolynomialParametricCurve2::new(
                vec![real(0), real(1)],
                vec![real(height)],
            );

            let report = resultant_parametric_curve_intersection(
                &parabola,
                &horizontal,
                CurveResultantParameter::First,
                CurveIntersectionResultantConfig::default(),
            );

            prop_assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
            prop_assert_eq!(
                report.resultant_coefficients,
                vec![real(-height), real(0), real(1)]
            );
        }

        #[test]
        fn generated_rational_weighted_parabola_resultant_cross_multiplies_height(
            height in 1_i16..=32,
        ) {
            let height = i64::from(height);
            let weighted_parabola = RationalParametricCurve2::new(
                vec![real(0), real(1)],
                vec![real(0), real(0), real(1)],
                vec![real(1), real(1)],
            );
            let horizontal = RationalParametricCurve2::from_polynomial(
                &PolynomialParametricCurve2::new(vec![real(0), real(1)], vec![real(height)]),
            );

            let report = resultant_rational_parametric_curve_intersection(
                &weighted_parabola,
                &horizontal,
                CurveResultantParameter::First,
                CurveIntersectionResultantConfig::default(),
            );

            prop_assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
            prop_assert_eq!(
                report.resultant_coefficients,
                vec![real(-height), real(-height), real(1)]
            );
        }
    }
}
