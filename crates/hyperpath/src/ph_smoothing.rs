//! Exact certification for retained PH smoothing spans.
//!
//! PH smoothing is useful only if the proposed curve is accepted as topology
//! and process evidence, not merely as a numeric curve fit. Following Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997), this module treats a smoothing span as a constructed candidate and
//! replays exact endpoint and G1-branch constraints before callers may use its
//! length in CAM or route-feed reports. The PH carrier itself follows Farouki
//! and Sakkalis, "Pythagorean hodographs," *IBM Journal of Research and
//! Development* 34.5 (1990): the curve derivative is the square of a complex
//! polynomial hodograph, giving exact endpoint tangents and polynomial length.

use hyperlimit::Point2;
use hyperreal::{Real, RealSign};
use hypersolve::{
    CandidateCertificationReport, Constraint, ConstraintKind, Expr, PreparedProblem, Problem,
    certify_candidate, context_from_problem,
};

use crate::ph::{PhCurveError, QuinticPythagoreanHodograph};
use crate::tangent::TangentSpan;

/// Exact replay report for accepting a quintic PH span as a G1 smoothing join.
#[derive(Clone, Debug)]
pub struct QuinticPhG1SmoothingReport {
    /// Exact PH candidate whose endpoints and branch tangents were replayed.
    pub curve: QuinticPythagoreanHodograph,
    /// Retained point the PH span must start at.
    pub start: Point2,
    /// Retained nonzero tangent direction the PH start derivative must follow.
    pub start_tangent: Point2,
    /// Retained point the PH span must end at.
    pub end: Point2,
    /// Retained nonzero tangent direction the PH end derivative must follow.
    pub end_tangent: Point2,
    /// Exact PH derivative at `t = 0`.
    pub curve_start_derivative: Point2,
    /// Exact PH derivative at `t = 1`.
    pub curve_end_derivative: Point2,
    /// Solver replay report for endpoint equality, tangent cross rows, and
    /// same-direction tangent branch inequalities.
    pub certification: CandidateCertificationReport,
}

impl QuinticPhG1SmoothingReport {
    /// Return whether every endpoint and tangent-branch row was certified.
    pub fn all_satisfied(&self) -> bool {
        self.certification.all_satisfied()
    }
}

/// Certify a retained quintic PH span against explicit endpoint/tangent data.
///
/// The replay rows are:
///
/// - endpoint equality for start and end coordinates,
/// - `PH'(0) x start_tangent = 0`,
/// - `PH'(0) . start_tangent >= 0`,
/// - `PH'(1) x end_tangent = 0`,
/// - `PH'(1) . end_tangent >= 0`.
///
/// The dot-product rows are deliberate branch evidence: a same supporting line
/// with the opposite tangent is a reversed join, not a valid G1 smoothing
/// acceptance. Zero source or PH endpoint tangents reject before replay because
/// they cannot prove an oriented G1 branch.
pub fn certify_quintic_ph_g1_smoothing(
    curve: &QuinticPythagoreanHodograph,
    start: Point2,
    start_tangent: Point2,
    end: Point2,
    end_tangent: Point2,
) -> Result<QuinticPhG1SmoothingReport, PhCurveError> {
    let curve_start_derivative = curve.start_derivative();
    let curve_end_derivative = curve.end_derivative();
    require_nonzero_tangent(&start_tangent)?;
    require_nonzero_tangent(&end_tangent)?;
    require_nonzero_tangent(&curve_start_derivative)?;
    require_nonzero_tangent(&curve_end_derivative)?;

    let mut problem = Problem::default();
    add_point_equality_rows(&mut problem, "PH smoothing start", curve.start(), &start);
    add_point_equality_rows(&mut problem, "PH smoothing end", curve.end(), &end);
    add_same_direction_rows(
        &mut problem,
        "PH smoothing start tangent",
        &curve_start_derivative,
        &start_tangent,
    );
    add_same_direction_rows(
        &mut problem,
        "PH smoothing end tangent",
        &curve_end_derivative,
        &end_tangent,
    );

    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);
    Ok(QuinticPhG1SmoothingReport {
        curve: curve.clone(),
        start,
        start_tangent,
        end,
        end_tangent,
        curve_start_derivative,
        curve_end_derivative,
        certification: certify_candidate(&prepared, &context),
    })
}

/// Certify a PH smoothing candidate inserted between two retained path spans.
///
/// The PH span is checked from `incoming.end` along `incoming.end_tangent` to
/// `outgoing.start` along `outgoing.start_tangent`. This keeps the smoothing
/// candidate separate from route planning: a solver may propose the PH
/// hodograph, but exact replay decides whether the candidate is admissible.
pub fn certify_quintic_ph_g1_smoothing_between(
    curve: &QuinticPythagoreanHodograph,
    incoming: &TangentSpan,
    outgoing: &TangentSpan,
) -> Result<QuinticPhG1SmoothingReport, PhCurveError> {
    certify_quintic_ph_g1_smoothing(
        curve,
        incoming.end.clone(),
        incoming.end_tangent.clone(),
        outgoing.start.clone(),
        outgoing.start_tangent.clone(),
    )
}

fn add_point_equality_rows(
    problem: &mut Problem,
    prefix: &str,
    actual: &Point2,
    expected: &Point2,
) {
    let actual_x = problem.add_variable(format!("{prefix} x"), actual.x.clone());
    let actual_y = problem.add_variable(format!("{prefix} y"), actual.y.clone());
    problem.add_constraint(Constraint::equality(
        format!("{prefix} x equality"),
        Expr::symbol(actual_x.into(), "actual_x") - Expr::real(expected.x.clone()),
    ));
    problem.add_constraint(Constraint::equality(
        format!("{prefix} y equality"),
        Expr::symbol(actual_y.into(), "actual_y") - Expr::real(expected.y.clone()),
    ));
}

fn add_same_direction_rows(
    problem: &mut Problem,
    prefix: &str,
    actual: &Point2,
    expected: &Point2,
) {
    let actual_x = problem.add_variable(format!("{prefix} x"), actual.x.clone());
    let actual_y = problem.add_variable(format!("{prefix} y"), actual.y.clone());
    let expected_x = problem.add_variable(format!("{prefix} expected x"), expected.x.clone());
    let expected_y = problem.add_variable(format!("{prefix} expected y"), expected.y.clone());
    let ax = Expr::symbol(actual_x.into(), "actual_x");
    let ay = Expr::symbol(actual_y.into(), "actual_y");
    let ex = Expr::symbol(expected_x.into(), "expected_x");
    let ey = Expr::symbol(expected_y.into(), "expected_y");
    problem.add_constraint(Constraint::equality(
        format!("{prefix} cross equality"),
        ax.clone() * ey.clone() - ay.clone() * ex.clone(),
    ));
    problem.add_constraint(Constraint {
        name: format!("{prefix} dot same-direction"),
        kind: ConstraintKind::GreaterOrEqual,
        residual: ax * ex + ay * ey,
        weight: Real::one(),
        active: true,
    });
}

fn require_nonzero_tangent(tangent: &Point2) -> Result<(), PhCurveError> {
    let norm = tangent.x.clone() * tangent.x.clone() + tangent.y.clone() * tangent.y.clone();
    match norm.structural_facts().sign {
        Some(RealSign::Zero) => Err(PhCurveError::DegenerateTangent),
        _ => Ok(()),
    }
}
