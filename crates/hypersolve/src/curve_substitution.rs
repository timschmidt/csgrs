//! Exact curve-substitution carriers for resultant-based handoffs.
//!
//! This module keeps curve-specific algebraic rewrites explicit before a
//! resultant is built. Bernstein control nets are converted to power basis by
//! the exact Bernstein expansion used in CAGD texts such as Farin, *Curves and
//! Surfaces for CAGD* (2002). Rational parametric curves retain homogeneous
//! numerator/weight polynomials so intersection code can cross-multiply
//! equations instead of dividing by a possibly vanishing weight. This is the
//! Yap EGC boundary: preserve structure and report the algebraic transform,
//! then let downstream geometry replay decide topology.

use hyperreal::{CertifiedRealSign, Real, RealSign};

use crate::curve_resultant::PolynomialParametricCurve2;

const MAX_INTERNAL_BEZIER_POWER_BASIS_DEGREE: usize = 32;

/// Exact 2D control point used by curve substitution helpers.
#[derive(Clone, Debug, PartialEq)]
pub struct PolynomialCurvePoint2 {
    /// x-coordinate.
    pub x: Real,
    /// y-coordinate.
    pub y: Real,
}

impl PolynomialCurvePoint2 {
    /// Construct an exact 2D curve point.
    pub const fn new(x: Real, y: Real) -> Self {
        Self { x, y }
    }
}

/// Exact weighted 2D control point for homogeneous rational Bezier curves.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalCurveControlPoint2 {
    /// Affine x-coordinate before homogeneous weighting.
    pub x: Real,
    /// Affine y-coordinate before homogeneous weighting.
    pub y: Real,
    /// Homogeneous Bernstein weight.
    pub weight: Real,
}

/// Homogeneous 2D control point with numerator coordinates and weight.
#[derive(Clone, Debug, PartialEq)]
pub struct HomogeneousCurveControlPoint2 {
    /// Weighted x-coordinate numerator.
    pub x_numerator: Real,
    /// Weighted y-coordinate numerator.
    pub y_numerator: Real,
    /// Homogeneous weight.
    pub weight: Real,
}

impl HomogeneousCurveControlPoint2 {
    /// Construct an exact homogeneous 2D control point.
    pub const fn new(x_numerator: Real, y_numerator: Real, weight: Real) -> Self {
        Self {
            x_numerator,
            y_numerator,
            weight,
        }
    }
}

impl RationalCurveControlPoint2 {
    /// Construct an exact weighted control point.
    pub const fn new(x: Real, y: Real, weight: Real) -> Self {
        Self { x, y, weight }
    }
}

/// Homogeneous rational 2D parametric curve in power basis.
///
/// The represented affine curve is
/// `(x_numerator(t) / weight(t), y_numerator(t) / weight(t))` when the weight
/// is nonzero. Resultant construction consumes the homogeneous equations
/// directly, so this carrier deliberately does not divide by `weight`.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalParametricCurve2 {
    /// Power-basis numerator for the x-coordinate.
    pub x_numerator: Vec<Real>,
    /// Power-basis numerator for the y-coordinate.
    pub y_numerator: Vec<Real>,
    /// Power-basis homogeneous weight.
    pub weight: Vec<Real>,
}

impl RationalParametricCurve2 {
    /// Construct a homogeneous rational parametric curve in power basis.
    pub const fn new(x_numerator: Vec<Real>, y_numerator: Vec<Real>, weight: Vec<Real>) -> Self {
        Self {
            x_numerator,
            y_numerator,
            weight,
        }
    }

    /// Lift a polynomial parametric curve into homogeneous form with unit
    /// weight.
    pub fn from_polynomial(curve: &PolynomialParametricCurve2) -> Self {
        Self {
            x_numerator: curve.x_coefficients.clone(),
            y_numerator: curve.y_coefficients.clone(),
            weight: vec![Real::one()],
        }
    }
}

/// Configuration for Bernstein-to-power-basis substitution.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BezierPowerBasisSubstitutionConfig {
    /// Maximum Bernstein degree this bounded exact package may expand.
    pub max_degree: usize,
}

/// Configuration for bounded exact B-spline knot-span substitution.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BsplineKnotSpanSubstitutionConfig {
    /// Precision bound used when knot ordering and zero denominators are certified.
    pub min_precision: i32,
    /// Maximum B-spline degree this bounded exact package may expand.
    pub max_degree: usize,
}

impl Default for BsplineKnotSpanSubstitutionConfig {
    fn default() -> Self {
        Self {
            min_precision: -64,
            max_degree: 8,
        }
    }
}

impl Default for BezierPowerBasisSubstitutionConfig {
    fn default() -> Self {
        Self { max_degree: 8 }
    }
}

/// Final status for Bernstein-to-power-basis substitution.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BezierPowerBasisSubstitutionStatus {
    /// The control net was expanded exactly.
    Constructed,
    /// The control net was empty.
    EmptyControlNet,
    /// The requested degree exceeds the configured bounded package.
    DegreeExceeded,
}

/// Report for exact Bernstein-to-power-basis substitution.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierPowerBasisSubstitutionReport {
    /// Final substitution status.
    pub status: BezierPowerBasisSubstitutionStatus,
    /// Bernstein degree when known.
    pub degree: Option<usize>,
    /// Source control points retained for replay.
    pub control_points: Vec<PolynomialCurvePoint2>,
    /// Exact power-basis curve when construction succeeds.
    pub power_basis: Option<PolynomialParametricCurve2>,
}

/// Final status for rational Bezier homogeneous substitution.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RationalBezierPowerBasisSubstitutionStatus {
    /// The weighted control net was expanded exactly.
    Constructed,
    /// The weighted control net was empty.
    EmptyControlNet,
    /// The requested degree exceeds the configured bounded package.
    DegreeExceeded,
}

/// Report for exact rational Bezier homogeneous power-basis substitution.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalBezierPowerBasisSubstitutionReport {
    /// Final substitution status.
    pub status: RationalBezierPowerBasisSubstitutionStatus,
    /// Bernstein degree when known.
    pub degree: Option<usize>,
    /// Source weighted control points retained for replay.
    pub control_points: Vec<RationalCurveControlPoint2>,
    /// Exact homogeneous power-basis curve when construction succeeds.
    pub homogeneous_power_basis: Option<RationalParametricCurve2>,
}

/// Final status for exact B-spline knot-span extraction.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BsplineKnotSpanSubstitutionStatus {
    /// The knot span was extracted and expanded exactly.
    Constructed,
    /// The control net or knot vector was empty.
    EmptyInput,
    /// The requested degree is zero or exceeds the configured bounded package.
    InvalidDegree,
    /// Knot vector length does not match `control_count + degree + 1`.
    InvalidKnotVectorLength,
    /// Knot vector order could not be certified as nondecreasing.
    UndecidedKnotOrder,
    /// Knot vector order is certified decreasing somewhere.
    DecreasingKnotVector,
    /// The requested span is outside the valid B-spline span range.
    SpanOutOfRange,
    /// The requested knot span has certified zero length.
    ZeroLengthSpan,
    /// A de Boor/blossom denominator was certified zero.
    DivisionByZero,
    /// Exact division failed while applying de Boor/blossom weights.
    DivisionFailed,
}

/// Exact report for converting one B-spline knot span to Bezier/power basis.
#[derive(Clone, Debug, PartialEq)]
pub struct BsplineKnotSpanSubstitutionReport {
    /// Final substitution status.
    pub status: BsplineKnotSpanSubstitutionStatus,
    /// B-spline degree requested by the caller.
    pub degree: usize,
    /// Knot span index `k`, representing `[knots[k], knots[k + 1]]`.
    pub span_index: usize,
    /// Certified nonzero knot span domain when available.
    pub span_domain: Option<(Real, Real)>,
    /// Bezier control points over the normalized span parameter.
    pub bezier_control_points: Vec<PolynomialCurvePoint2>,
    /// Exact normalized-span power-basis curve when construction succeeds.
    pub power_basis: Option<PolynomialParametricCurve2>,
}

/// Exact report for converting one NURBS knot span to homogeneous Bezier/power basis.
#[derive(Clone, Debug, PartialEq)]
pub struct NurbsKnotSpanSubstitutionReport {
    /// Final substitution status.
    pub status: BsplineKnotSpanSubstitutionStatus,
    /// NURBS degree requested by the caller.
    pub degree: usize,
    /// Knot span index `k`, representing `[knots[k], knots[k + 1]]`.
    pub span_index: usize,
    /// Certified nonzero knot span domain when available.
    pub span_domain: Option<(Real, Real)>,
    /// Homogeneous Bezier controls over the normalized span parameter.
    pub homogeneous_bezier_control_points: Vec<HomogeneousCurveControlPoint2>,
    /// Exact normalized-span homogeneous power-basis curve when construction succeeds.
    pub homogeneous_power_basis: Option<RationalParametricCurve2>,
}

/// Convert a polynomial Bezier control net to exact power basis.
///
/// For degree `n`, the coefficient of `t^k` is
/// `sum_{i=0..k} P_i * binom(n, i) * binom(n-i, k-i) * (-1)^(k-i)`.
/// This is just the exact expansion of `P_i * B_i^n(t)`, not a sampled fit.
pub fn substitute_bezier_power_basis(
    control_points: &[PolynomialCurvePoint2],
    config: BezierPowerBasisSubstitutionConfig,
) -> BezierPowerBasisSubstitutionReport {
    if control_points.is_empty() {
        return BezierPowerBasisSubstitutionReport {
            status: BezierPowerBasisSubstitutionStatus::EmptyControlNet,
            degree: None,
            control_points: Vec::new(),
            power_basis: None,
        };
    }

    let degree = control_points.len() - 1;
    if degree > config.max_degree || degree > MAX_INTERNAL_BEZIER_POWER_BASIS_DEGREE {
        return BezierPowerBasisSubstitutionReport {
            status: BezierPowerBasisSubstitutionStatus::DegreeExceeded,
            degree: Some(degree),
            control_points: control_points.to_vec(),
            power_basis: None,
        };
    }

    let mut x_coefficients = vec![Real::zero(); degree + 1];
    let mut y_coefficients = vec![Real::zero(); degree + 1];
    for power in 0..=degree {
        for (basis_index, point) in control_points.iter().enumerate().take(power + 1) {
            let mut scale = Real::from(
                (binomial(degree, basis_index)
                    * binomial(degree - basis_index, power - basis_index)) as i64,
            );
            if (power - basis_index) % 2 == 1 {
                scale = -scale;
            }
            x_coefficients[power] += point.x.clone() * scale.clone();
            y_coefficients[power] += point.y.clone() * scale;
        }
    }

    BezierPowerBasisSubstitutionReport {
        status: BezierPowerBasisSubstitutionStatus::Constructed,
        degree: Some(degree),
        control_points: control_points.to_vec(),
        power_basis: Some(PolynomialParametricCurve2::new(
            x_coefficients,
            y_coefficients,
        )),
    }
}

/// Convert a rational Bezier control net to homogeneous exact power basis.
///
/// A rational Bezier curve is represented as
/// `sum_i w_i P_i B_i^n(t) / sum_i w_i B_i^n(t)`. This function expands the
/// three Bernstein polynomials `w_i*x_i`, `w_i*y_i`, and `w_i` into power
/// basis and returns a [`RationalParametricCurve2`] without ever dividing by
/// the weight. This is the homogeneous rational Bezier construction described
/// by Farin, *Curves and Surfaces for CAGD* (2002), used here as a Yap-style
/// exact proposal package: denominator/domain validity remains a replay
/// obligation for downstream curve code.
pub fn substitute_rational_bezier_power_basis(
    control_points: &[RationalCurveControlPoint2],
    config: BezierPowerBasisSubstitutionConfig,
) -> RationalBezierPowerBasisSubstitutionReport {
    if control_points.is_empty() {
        return RationalBezierPowerBasisSubstitutionReport {
            status: RationalBezierPowerBasisSubstitutionStatus::EmptyControlNet,
            degree: None,
            control_points: Vec::new(),
            homogeneous_power_basis: None,
        };
    }

    let degree = control_points.len() - 1;
    if degree > config.max_degree || degree > MAX_INTERNAL_BEZIER_POWER_BASIS_DEGREE {
        return RationalBezierPowerBasisSubstitutionReport {
            status: RationalBezierPowerBasisSubstitutionStatus::DegreeExceeded,
            degree: Some(degree),
            control_points: control_points.to_vec(),
            homogeneous_power_basis: None,
        };
    }

    let weighted_x = control_points
        .iter()
        .map(|point| point.x.clone() * point.weight.clone())
        .collect::<Vec<_>>();
    let weighted_y = control_points
        .iter()
        .map(|point| point.y.clone() * point.weight.clone())
        .collect::<Vec<_>>();
    let weights = control_points
        .iter()
        .map(|point| point.weight.clone())
        .collect::<Vec<_>>();
    let x_numerator = bernstein_scalars_to_power_basis(&weighted_x);
    let y_numerator = bernstein_scalars_to_power_basis(&weighted_y);
    let weight = bernstein_scalars_to_power_basis(&weights);

    RationalBezierPowerBasisSubstitutionReport {
        status: RationalBezierPowerBasisSubstitutionStatus::Constructed,
        degree: Some(degree),
        control_points: control_points.to_vec(),
        homogeneous_power_basis: Some(RationalParametricCurve2::new(
            x_numerator,
            y_numerator,
            weight,
        )),
    }
}

/// Extract one polynomial B-spline knot span as exact Bezier/power basis.
///
/// The returned Bezier controls use the normalized local parameter
/// `s = (t - knots[span_index]) / (knots[span_index + 1] - knots[span_index])`.
/// Control extraction evaluates the B-spline blossom at
/// `a^(p-j), b^j`, where `a` and `b` are the span endpoints and `p` is the
/// degree. This is the standard de Boor/blossom relationship for B-splines;
/// see de Boor, *A Practical Guide to Splines* (1978), and Piegl and Tiller,
/// *The NURBS Book* (1997). The function is report-bearing because Yap's EGC
/// discipline requires knot ordering, span length, and exact divisions to be
/// explicit evidence, not hidden preconditions.
pub fn substitute_bspline_knot_span_power_basis(
    control_points: &[PolynomialCurvePoint2],
    knots: &[Real],
    degree: usize,
    span_index: usize,
    config: BsplineKnotSpanSubstitutionConfig,
) -> BsplineKnotSpanSubstitutionReport {
    if control_points.is_empty() || knots.is_empty() {
        return bspline_span_report(
            BsplineKnotSpanSubstitutionStatus::EmptyInput,
            degree,
            span_index,
            None,
            Vec::new(),
            None,
        );
    }
    if degree == 0 || degree > config.max_degree || degree > MAX_INTERNAL_BEZIER_POWER_BASIS_DEGREE
    {
        return bspline_span_report(
            BsplineKnotSpanSubstitutionStatus::InvalidDegree,
            degree,
            span_index,
            None,
            Vec::new(),
            None,
        );
    }
    if knots.len() != control_points.len() + degree + 1 {
        return bspline_span_report(
            BsplineKnotSpanSubstitutionStatus::InvalidKnotVectorLength,
            degree,
            span_index,
            None,
            Vec::new(),
            None,
        );
    }
    for pair in knots.windows(2) {
        let difference = pair[1].clone() - pair[0].clone();
        match difference.certified_sign_until(config.min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Negative,
                ..
            } => {
                return bspline_span_report(
                    BsplineKnotSpanSubstitutionStatus::DecreasingKnotVector,
                    degree,
                    span_index,
                    None,
                    Vec::new(),
                    None,
                );
            }
            CertifiedRealSign::Known { .. } => {}
            CertifiedRealSign::Unknown { .. } => {
                return bspline_span_report(
                    BsplineKnotSpanSubstitutionStatus::UndecidedKnotOrder,
                    degree,
                    span_index,
                    None,
                    Vec::new(),
                    None,
                );
            }
        }
    }

    let last_control = control_points.len() - 1;
    if span_index < degree || span_index > last_control || span_index + 1 >= knots.len() {
        return bspline_span_report(
            BsplineKnotSpanSubstitutionStatus::SpanOutOfRange,
            degree,
            span_index,
            None,
            Vec::new(),
            None,
        );
    }
    let span_start = knots[span_index].clone();
    let span_end = knots[span_index + 1].clone();
    let span_length = span_end.clone() - span_start.clone();
    match span_length.certified_sign_until(config.min_precision) {
        CertifiedRealSign::Known {
            sign: RealSign::Zero,
            ..
        } => {
            return bspline_span_report(
                BsplineKnotSpanSubstitutionStatus::ZeroLengthSpan,
                degree,
                span_index,
                None,
                Vec::new(),
                None,
            );
        }
        CertifiedRealSign::Known {
            sign: RealSign::Positive,
            ..
        } => {}
        CertifiedRealSign::Known { .. } => {
            return bspline_span_report(
                BsplineKnotSpanSubstitutionStatus::DecreasingKnotVector,
                degree,
                span_index,
                None,
                Vec::new(),
                None,
            );
        }
        CertifiedRealSign::Unknown { .. } => {
            return bspline_span_report(
                BsplineKnotSpanSubstitutionStatus::UndecidedKnotOrder,
                degree,
                span_index,
                None,
                Vec::new(),
                None,
            );
        }
    }

    let mut bezier_control_points = Vec::with_capacity(degree + 1);
    for bezier_index in 0..=degree {
        let mut blossom_parameters = Vec::with_capacity(degree);
        blossom_parameters.extend(std::iter::repeat_n(
            span_start.clone(),
            degree - bezier_index,
        ));
        blossom_parameters.extend(std::iter::repeat_n(span_end.clone(), bezier_index));
        let point = match bspline_blossom_point(
            control_points,
            knots,
            degree,
            span_index,
            &blossom_parameters,
            config.min_precision,
        ) {
            Ok(point) => point,
            Err(status) => {
                return bspline_span_report(
                    status,
                    degree,
                    span_index,
                    Some((span_start, span_end)),
                    bezier_control_points,
                    None,
                );
            }
        };
        bezier_control_points.push(point);
    }
    let power_report = substitute_bezier_power_basis(
        &bezier_control_points,
        BezierPowerBasisSubstitutionConfig {
            max_degree: config.max_degree,
        },
    );
    bspline_span_report(
        BsplineKnotSpanSubstitutionStatus::Constructed,
        degree,
        span_index,
        Some((span_start, span_end)),
        bezier_control_points,
        power_report.power_basis,
    )
}

/// Extract one NURBS knot span as homogeneous exact Bezier/power basis.
///
/// The input controls are affine weighted controls `(P_i, w_i)`. This function
/// lifts them to homogeneous coordinates `(w_i*x_i, w_i*y_i, w_i)` and applies
/// the same blossom/de Boor span extraction used by
/// [`substitute_bspline_knot_span_power_basis`]. The returned curve is a
/// [`RationalParametricCurve2`] over the normalized local span parameter; it
/// intentionally does not divide by the extracted weight polynomial. This is
/// the standard NURBS homogeneous construction from Piegl and Tiller,
/// *The NURBS Book* (1997), kept at Yap's exact-computation boundary so
/// denominator/domain validity is replay evidence owned by downstream curve
/// code, not an implicit side effect of substitution.
pub fn substitute_nurbs_knot_span_power_basis(
    control_points: &[RationalCurveControlPoint2],
    knots: &[Real],
    degree: usize,
    span_index: usize,
    config: BsplineKnotSpanSubstitutionConfig,
) -> NurbsKnotSpanSubstitutionReport {
    if control_points.is_empty() || knots.is_empty() {
        return nurbs_span_report(
            BsplineKnotSpanSubstitutionStatus::EmptyInput,
            degree,
            span_index,
            None,
            Vec::new(),
            None,
        );
    }
    if degree == 0 || degree > config.max_degree || degree > MAX_INTERNAL_BEZIER_POWER_BASIS_DEGREE
    {
        return nurbs_span_report(
            BsplineKnotSpanSubstitutionStatus::InvalidDegree,
            degree,
            span_index,
            None,
            Vec::new(),
            None,
        );
    }
    if knots.len() != control_points.len() + degree + 1 {
        return nurbs_span_report(
            BsplineKnotSpanSubstitutionStatus::InvalidKnotVectorLength,
            degree,
            span_index,
            None,
            Vec::new(),
            None,
        );
    }
    let polynomial_controls = control_points
        .iter()
        .map(|point| {
            PolynomialCurvePoint2::new(
                point.x.clone() * point.weight.clone(),
                point.y.clone() * point.weight.clone(),
            )
        })
        .collect::<Vec<_>>();
    let polynomial_validation = substitute_bspline_knot_span_power_basis(
        &polynomial_controls,
        knots,
        degree,
        span_index,
        config,
    );
    if polynomial_validation.status != BsplineKnotSpanSubstitutionStatus::Constructed {
        return nurbs_span_report(
            polynomial_validation.status,
            degree,
            span_index,
            polynomial_validation.span_domain,
            Vec::new(),
            None,
        );
    }

    let span_start = polynomial_validation
        .span_domain
        .as_ref()
        .map(|(start, _)| start.clone())
        .expect("constructed B-spline span carries a domain start");
    let span_end = polynomial_validation
        .span_domain
        .as_ref()
        .map(|(_, end)| end.clone())
        .expect("constructed B-spline span carries a domain end");
    let mut homogeneous_bezier_control_points = Vec::with_capacity(degree + 1);
    for bezier_index in 0..=degree {
        let mut blossom_parameters = Vec::with_capacity(degree);
        blossom_parameters.extend(std::iter::repeat_n(
            span_start.clone(),
            degree - bezier_index,
        ));
        blossom_parameters.extend(std::iter::repeat_n(span_end.clone(), bezier_index));
        let point = match bspline_blossom_homogeneous_point(
            control_points,
            knots,
            degree,
            span_index,
            &blossom_parameters,
            config.min_precision,
        ) {
            Ok(point) => point,
            Err(status) => {
                return nurbs_span_report(
                    status,
                    degree,
                    span_index,
                    polynomial_validation.span_domain,
                    homogeneous_bezier_control_points,
                    None,
                );
            }
        };
        homogeneous_bezier_control_points.push(point);
    }

    let x_numerator = bernstein_scalars_to_power_basis(
        &homogeneous_bezier_control_points
            .iter()
            .map(|point| point.x_numerator.clone())
            .collect::<Vec<_>>(),
    );
    let y_numerator = bernstein_scalars_to_power_basis(
        &homogeneous_bezier_control_points
            .iter()
            .map(|point| point.y_numerator.clone())
            .collect::<Vec<_>>(),
    );
    let weight = bernstein_scalars_to_power_basis(
        &homogeneous_bezier_control_points
            .iter()
            .map(|point| point.weight.clone())
            .collect::<Vec<_>>(),
    );

    nurbs_span_report(
        BsplineKnotSpanSubstitutionStatus::Constructed,
        degree,
        span_index,
        polynomial_validation.span_domain,
        homogeneous_bezier_control_points,
        Some(RationalParametricCurve2::new(
            x_numerator,
            y_numerator,
            weight,
        )),
    )
}

fn bspline_span_report(
    status: BsplineKnotSpanSubstitutionStatus,
    degree: usize,
    span_index: usize,
    span_domain: Option<(Real, Real)>,
    bezier_control_points: Vec<PolynomialCurvePoint2>,
    power_basis: Option<PolynomialParametricCurve2>,
) -> BsplineKnotSpanSubstitutionReport {
    BsplineKnotSpanSubstitutionReport {
        status,
        degree,
        span_index,
        span_domain,
        bezier_control_points,
        power_basis,
    }
}

fn nurbs_span_report(
    status: BsplineKnotSpanSubstitutionStatus,
    degree: usize,
    span_index: usize,
    span_domain: Option<(Real, Real)>,
    homogeneous_bezier_control_points: Vec<HomogeneousCurveControlPoint2>,
    homogeneous_power_basis: Option<RationalParametricCurve2>,
) -> NurbsKnotSpanSubstitutionReport {
    NurbsKnotSpanSubstitutionReport {
        status,
        degree,
        span_index,
        span_domain,
        homogeneous_bezier_control_points,
        homogeneous_power_basis,
    }
}

fn bspline_blossom_point(
    control_points: &[PolynomialCurvePoint2],
    knots: &[Real],
    degree: usize,
    span_index: usize,
    parameters: &[Real],
    min_precision: i32,
) -> Result<PolynomialCurvePoint2, BsplineKnotSpanSubstitutionStatus> {
    let mut work = (0..=degree)
        .map(|index| control_points[span_index - degree + index].clone())
        .collect::<Vec<_>>();
    for step in 1..=degree {
        let parameter = &parameters[step - 1];
        for index in (step..=degree).rev() {
            let left_knot = knots[span_index - degree + index].clone();
            let right_knot = knots[span_index + index + 1 - step].clone();
            let denominator = right_knot - left_knot.clone();
            match denominator.certified_sign_until(min_precision) {
                CertifiedRealSign::Known {
                    sign: RealSign::Zero,
                    ..
                } => return Err(BsplineKnotSpanSubstitutionStatus::DivisionByZero),
                CertifiedRealSign::Unknown { .. } => {
                    return Err(BsplineKnotSpanSubstitutionStatus::DivisionFailed);
                }
                CertifiedRealSign::Known { .. } => {}
            }
            let alpha = ((parameter.clone() - left_knot) / denominator)
                .map_err(|_| BsplineKnotSpanSubstitutionStatus::DivisionFailed)?;
            let one_minus_alpha = Real::one() - alpha.clone();
            work[index] = PolynomialCurvePoint2 {
                x: work[index - 1].x.clone() * one_minus_alpha.clone()
                    + work[index].x.clone() * alpha.clone(),
                y: work[index - 1].y.clone() * one_minus_alpha + work[index].y.clone() * alpha,
            };
        }
    }
    Ok(work[degree].clone())
}

fn bspline_blossom_homogeneous_point(
    control_points: &[RationalCurveControlPoint2],
    knots: &[Real],
    degree: usize,
    span_index: usize,
    parameters: &[Real],
    min_precision: i32,
) -> Result<HomogeneousCurveControlPoint2, BsplineKnotSpanSubstitutionStatus> {
    let mut work = (0..=degree)
        .map(|index| {
            let point = &control_points[span_index - degree + index];
            HomogeneousCurveControlPoint2::new(
                point.x.clone() * point.weight.clone(),
                point.y.clone() * point.weight.clone(),
                point.weight.clone(),
            )
        })
        .collect::<Vec<_>>();
    for step in 1..=degree {
        let parameter = &parameters[step - 1];
        for index in (step..=degree).rev() {
            let left_knot = knots[span_index - degree + index].clone();
            let right_knot = knots[span_index + index + 1 - step].clone();
            let denominator = right_knot - left_knot.clone();
            match denominator.certified_sign_until(min_precision) {
                CertifiedRealSign::Known {
                    sign: RealSign::Zero,
                    ..
                } => return Err(BsplineKnotSpanSubstitutionStatus::DivisionByZero),
                CertifiedRealSign::Unknown { .. } => {
                    return Err(BsplineKnotSpanSubstitutionStatus::DivisionFailed);
                }
                CertifiedRealSign::Known { .. } => {}
            }
            let alpha = ((parameter.clone() - left_knot) / denominator)
                .map_err(|_| BsplineKnotSpanSubstitutionStatus::DivisionFailed)?;
            let one_minus_alpha = Real::one() - alpha.clone();
            work[index] = HomogeneousCurveControlPoint2 {
                x_numerator: work[index - 1].x_numerator.clone() * one_minus_alpha.clone()
                    + work[index].x_numerator.clone() * alpha.clone(),
                y_numerator: work[index - 1].y_numerator.clone() * one_minus_alpha.clone()
                    + work[index].y_numerator.clone() * alpha.clone(),
                weight: work[index - 1].weight.clone() * one_minus_alpha
                    + work[index].weight.clone() * alpha,
            };
        }
    }
    Ok(work[degree].clone())
}

fn bernstein_scalars_to_power_basis(values: &[Real]) -> Vec<Real> {
    let degree = values.len() - 1;
    let mut coefficients = vec![Real::zero(); degree + 1];
    for (power, coefficient) in coefficients.iter_mut().enumerate().take(degree + 1) {
        for (basis_index, value) in values.iter().enumerate().take(power + 1) {
            let mut scale = Real::from(
                (binomial(degree, basis_index)
                    * binomial(degree - basis_index, power - basis_index)) as i64,
            );
            if (power - basis_index) % 2 == 1 {
                scale = -scale;
            }
            *coefficient += value.clone() * scale;
        }
    }
    coefficients
}

fn binomial(n: usize, k: usize) -> u64 {
    if k > n {
        return 0;
    }
    let k = k.min(n - k);
    let mut result = 1u64;
    for index in 1..=k {
        result = result * (n - k + index) as u64 / index as u64;
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

    fn point(x: i64, y: i64) -> PolynomialCurvePoint2 {
        PolynomialCurvePoint2::new(real(x), real(y))
    }

    fn weighted(x: i64, y: i64, weight: i64) -> RationalCurveControlPoint2 {
        RationalCurveControlPoint2::new(real(x), real(y), real(weight))
    }

    #[test]
    fn quadratic_bezier_substitution_expands_exact_power_basis() {
        let report = substitute_bezier_power_basis(
            &[point(0, 0), point(1, 2), point(2, 0)],
            BezierPowerBasisSubstitutionConfig::default(),
        );

        assert_eq!(
            report.status,
            BezierPowerBasisSubstitutionStatus::Constructed
        );
        let curve = report.power_basis.unwrap();
        assert_eq!(curve.x_coefficients, vec![real(0), real(2), real(0)]);
        assert_eq!(curve.y_coefficients, vec![real(0), real(4), real(-4)]);
    }

    #[test]
    fn bezier_substitution_reports_empty_and_degree_limit() {
        let empty =
            substitute_bezier_power_basis(&[], BezierPowerBasisSubstitutionConfig::default());
        assert_eq!(
            empty.status,
            BezierPowerBasisSubstitutionStatus::EmptyControlNet
        );

        let too_large = substitute_bezier_power_basis(
            &[point(0, 0), point(1, 1), point(2, 2)],
            BezierPowerBasisSubstitutionConfig { max_degree: 1 },
        );
        assert_eq!(
            too_large.status,
            BezierPowerBasisSubstitutionStatus::DegreeExceeded
        );
        assert!(too_large.power_basis.is_none());
    }

    #[test]
    fn rational_quadratic_bezier_substitution_expands_homogeneous_power_basis() {
        let report = substitute_rational_bezier_power_basis(
            &[weighted(0, 0, 1), weighted(1, 2, 2), weighted(2, 0, 1)],
            BezierPowerBasisSubstitutionConfig::default(),
        );

        assert_eq!(
            report.status,
            RationalBezierPowerBasisSubstitutionStatus::Constructed
        );
        let curve = report.homogeneous_power_basis.unwrap();
        assert_eq!(curve.x_numerator, vec![real(0), real(4), real(-2)]);
        assert_eq!(curve.y_numerator, vec![real(0), real(8), real(-8)]);
        assert_eq!(curve.weight, vec![real(1), real(2), real(-2)]);
    }

    #[test]
    fn rational_bezier_substitution_reports_empty_and_degree_limit() {
        let empty = substitute_rational_bezier_power_basis(
            &[],
            BezierPowerBasisSubstitutionConfig::default(),
        );
        assert_eq!(
            empty.status,
            RationalBezierPowerBasisSubstitutionStatus::EmptyControlNet
        );

        let too_large = substitute_rational_bezier_power_basis(
            &[weighted(0, 0, 1), weighted(1, 1, 1), weighted(2, 2, 1)],
            BezierPowerBasisSubstitutionConfig { max_degree: 1 },
        );
        assert_eq!(
            too_large.status,
            RationalBezierPowerBasisSubstitutionStatus::DegreeExceeded
        );
        assert!(too_large.homogeneous_power_basis.is_none());
    }

    #[test]
    fn quadratic_bspline_span_extracts_middle_bezier_segment_exactly() {
        let controls = [point(0, 0), point(1, 2), point(3, 2), point(4, 0)];
        let knots = [
            real(0),
            real(0),
            real(0),
            real(1),
            real(2),
            real(2),
            real(2),
        ];

        let report = substitute_bspline_knot_span_power_basis(
            &controls,
            &knots,
            2,
            2,
            BsplineKnotSpanSubstitutionConfig::default(),
        );

        assert_eq!(
            report.status,
            BsplineKnotSpanSubstitutionStatus::Constructed
        );
        assert_eq!(report.span_domain, Some((real(0), real(1))));
        assert_eq!(
            report.bezier_control_points,
            vec![point(0, 0), point(1, 2), point(2, 2)]
        );
        let curve = report.power_basis.unwrap();
        assert_eq!(curve.x_coefficients, vec![real(0), real(2), real(0)]);
        assert_eq!(curve.y_coefficients, vec![real(0), real(4), real(-2)]);
    }

    #[test]
    fn bspline_span_reports_bad_knots_ranges_and_zero_spans() {
        let controls = [point(0, 0), point(1, 0), point(2, 0)];
        let good_knots = [real(0), real(0), real(1), real(2), real(2)];
        let bad_len = substitute_bspline_knot_span_power_basis(
            &controls,
            &good_knots[..4],
            1,
            1,
            BsplineKnotSpanSubstitutionConfig::default(),
        );
        assert_eq!(
            bad_len.status,
            BsplineKnotSpanSubstitutionStatus::InvalidKnotVectorLength
        );

        let decreasing = [real(0), real(0), real(2), real(1), real(2)];
        let bad_order = substitute_bspline_knot_span_power_basis(
            &controls,
            &decreasing,
            1,
            1,
            BsplineKnotSpanSubstitutionConfig::default(),
        );
        assert_eq!(
            bad_order.status,
            BsplineKnotSpanSubstitutionStatus::DecreasingKnotVector
        );

        let out_of_range = substitute_bspline_knot_span_power_basis(
            &controls,
            &good_knots,
            1,
            0,
            BsplineKnotSpanSubstitutionConfig::default(),
        );
        assert_eq!(
            out_of_range.status,
            BsplineKnotSpanSubstitutionStatus::SpanOutOfRange
        );

        let repeated_controls = [point(0, 0), point(1, 0), point(2, 0), point(3, 0)];
        let repeated_span = substitute_bspline_knot_span_power_basis(
            &repeated_controls,
            &[
                real(0),
                real(0),
                real(0),
                real(1),
                real(1),
                real(2),
                real(2),
            ],
            2,
            3,
            BsplineKnotSpanSubstitutionConfig::default(),
        );
        assert_eq!(
            repeated_span.status,
            BsplineKnotSpanSubstitutionStatus::ZeroLengthSpan
        );
    }

    #[test]
    fn quadratic_nurbs_span_extracts_homogeneous_bezier_segment_exactly() {
        let controls = [
            weighted(0, 0, 1),
            weighted(1, 2, 2),
            weighted(3, 2, 2),
            weighted(4, 0, 1),
        ];
        let knots = [
            real(0),
            real(0),
            real(0),
            real(1),
            real(2),
            real(2),
            real(2),
        ];

        let report = substitute_nurbs_knot_span_power_basis(
            &controls,
            &knots,
            2,
            2,
            BsplineKnotSpanSubstitutionConfig::default(),
        );

        assert_eq!(
            report.status,
            BsplineKnotSpanSubstitutionStatus::Constructed
        );
        assert_eq!(report.span_domain, Some((real(0), real(1))));
        assert_eq!(
            report.homogeneous_bezier_control_points,
            vec![
                HomogeneousCurveControlPoint2::new(real(0), real(0), real(1)),
                HomogeneousCurveControlPoint2::new(real(2), real(4), real(2)),
                HomogeneousCurveControlPoint2::new(real(4), real(4), real(2)),
            ]
        );
        let curve = report.homogeneous_power_basis.unwrap();
        assert_eq!(curve.x_numerator, vec![real(0), real(4), real(0)]);
        assert_eq!(curve.y_numerator, vec![real(0), real(8), real(-4)]);
        assert_eq!(curve.weight, vec![real(1), real(2), real(-1)]);
    }

    #[test]
    fn nurbs_span_reuses_bspline_validation_statuses() {
        let controls = [weighted(0, 0, 1), weighted(1, 0, 1)];
        let bad_len = substitute_nurbs_knot_span_power_basis(
            &controls,
            &[real(0), real(0), real(1)],
            1,
            1,
            BsplineKnotSpanSubstitutionConfig::default(),
        );
        assert_eq!(
            bad_len.status,
            BsplineKnotSpanSubstitutionStatus::InvalidKnotVectorLength
        );

        let out_of_range = substitute_nurbs_knot_span_power_basis(
            &controls,
            &[real(0), real(0), real(1), real(1)],
            1,
            0,
            BsplineKnotSpanSubstitutionConfig::default(),
        );
        assert_eq!(
            out_of_range.status,
            BsplineKnotSpanSubstitutionStatus::SpanOutOfRange
        );
    }

    proptest! {
        #[test]
        fn generated_linear_bezier_substitution_matches_endpoint_delta(
            x0 in -32_i16..=32,
            y0 in -32_i16..=32,
            x1 in -32_i16..=32,
            y1 in -32_i16..=32,
        ) {
            let (x0, y0, x1, y1) = (
                i64::from(x0),
                i64::from(y0),
                i64::from(x1),
                i64::from(y1),
            );
            let report = substitute_bezier_power_basis(
                &[point(x0, y0), point(x1, y1)],
                BezierPowerBasisSubstitutionConfig::default(),
            );

            prop_assert_eq!(report.status, BezierPowerBasisSubstitutionStatus::Constructed);
            let curve = report.power_basis.unwrap();
            prop_assert_eq!(curve.x_coefficients, vec![real(x0), real(x1 - x0)]);
            prop_assert_eq!(curve.y_coefficients, vec![real(y0), real(y1 - y0)]);
        }

        #[test]
        fn generated_linear_rational_bezier_substitution_keeps_homogeneous_weights(
            x0 in -16_i16..=16,
            y0 in -16_i16..=16,
            x1 in -16_i16..=16,
            y1 in -16_i16..=16,
            w0 in 1_i16..=8,
            w1 in 1_i16..=8,
        ) {
            let (x0, y0, x1, y1, w0, w1) = (
                i64::from(x0),
                i64::from(y0),
                i64::from(x1),
                i64::from(y1),
                i64::from(w0),
                i64::from(w1),
            );
            let report = substitute_rational_bezier_power_basis(
                &[weighted(x0, y0, w0), weighted(x1, y1, w1)],
                BezierPowerBasisSubstitutionConfig::default(),
            );

            prop_assert_eq!(
                report.status,
                RationalBezierPowerBasisSubstitutionStatus::Constructed
            );
            let curve = report.homogeneous_power_basis.unwrap();
            prop_assert_eq!(curve.x_numerator, vec![real(x0 * w0), real(x1 * w1 - x0 * w0)]);
            prop_assert_eq!(curve.y_numerator, vec![real(y0 * w0), real(y1 * w1 - y0 * w0)]);
            prop_assert_eq!(curve.weight, vec![real(w0), real(w1 - w0)]);
        }

        #[test]
        fn generated_linear_bspline_span_matches_segment_endpoints(
            x0 in -32_i16..=32,
            y0 in -32_i16..=32,
            x1 in -32_i16..=32,
            y1 in -32_i16..=32,
        ) {
            let (x0, y0, x1, y1) = (
                i64::from(x0),
                i64::from(y0),
                i64::from(x1),
                i64::from(y1),
            );
            let controls = [point(x0, y0), point(x1, y1)];
            let knots = [real(0), real(0), real(1), real(1)];
            let report = substitute_bspline_knot_span_power_basis(
                &controls,
                &knots,
                1,
                1,
                BsplineKnotSpanSubstitutionConfig::default(),
            );

            prop_assert_eq!(report.status, BsplineKnotSpanSubstitutionStatus::Constructed);
            prop_assert_eq!(report.bezier_control_points, vec![point(x0, y0), point(x1, y1)]);
            let curve = report.power_basis.unwrap();
            prop_assert_eq!(curve.x_coefficients, vec![real(x0), real(x1 - x0)]);
            prop_assert_eq!(curve.y_coefficients, vec![real(y0), real(y1 - y0)]);
        }

        #[test]
        fn generated_linear_nurbs_span_matches_homogeneous_segment(
            x0 in -16_i16..=16,
            y0 in -16_i16..=16,
            x1 in -16_i16..=16,
            y1 in -16_i16..=16,
            w0 in 1_i16..=8,
            w1 in 1_i16..=8,
        ) {
            let (x0, y0, x1, y1, w0, w1) = (
                i64::from(x0),
                i64::from(y0),
                i64::from(x1),
                i64::from(y1),
                i64::from(w0),
                i64::from(w1),
            );
            let controls = [weighted(x0, y0, w0), weighted(x1, y1, w1)];
            let knots = [real(0), real(0), real(1), real(1)];
            let report = substitute_nurbs_knot_span_power_basis(
                &controls,
                &knots,
                1,
                1,
                BsplineKnotSpanSubstitutionConfig::default(),
            );

            prop_assert_eq!(report.status, BsplineKnotSpanSubstitutionStatus::Constructed);
            prop_assert_eq!(
                report.homogeneous_bezier_control_points,
                vec![
                    HomogeneousCurveControlPoint2::new(real(x0 * w0), real(y0 * w0), real(w0)),
                    HomogeneousCurveControlPoint2::new(real(x1 * w1), real(y1 * w1), real(w1)),
                ]
            );
            let curve = report.homogeneous_power_basis.unwrap();
            prop_assert_eq!(curve.x_numerator, vec![real(x0 * w0), real(x1 * w1 - x0 * w0)]);
            prop_assert_eq!(curve.y_numerator, vec![real(y0 * w0), real(y1 * w1 - y0 * w0)]);
            prop_assert_eq!(curve.weight, vec![real(w0), real(w1 - w0)]);
        }
    }
}
