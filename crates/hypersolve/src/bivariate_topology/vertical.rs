//! Complete axis components, separated from the residual finite-fiber graph.
use super::*;
use crate::bivariate_arithmetic::{bivariate_multiply, polynomial_derivative};
use crate::curve_resultant::{
    BivariatePolynomialAxisFactorStatus, divide_bivariate_polynomial_exact,
    extract_bivariate_polynomial_system_axis_factors,
};

/// One whole closed second-coordinate interval at a selected event parameter.
///
/// Its exact first coordinate is `topology.events()[event_index].parameter()`;
/// its second-coordinate domain is `topology.domain().second`. The event roots
/// retain every residual contact; neighboring slabs retain its half-branches.
#[derive(Clone, Debug, PartialEq)]
pub struct BivariateVerticalComponent {
    event_index: usize,
    exponent: usize,
}

impl BivariateVerticalComponent {
    /// Event selecting this component's first coordinate and residual contacts.
    pub const fn event_index(&self) -> usize {
        self.event_index
    }

    /// Root multiplicity in the original first-coordinate content.
    /// This is neither winding nor the local order at a residual intersection.
    pub const fn defining_exponent(&self) -> usize {
        self.exponent
    }
}

/// Checked identity `original(x,y) = factor(x) * residual_equation(x,y)`.
///
/// Content retains all vertical powers, and the exact quotient retains the
/// remaining scalar and factors. Geometric support uses one copy of each
/// vertical root. No source factor is silently discarded.
/// This live evidence is reconstructed from the original equation on replay.
#[derive(Clone, Debug, PartialEq)]
pub struct BivariateVerticalContent {
    factor: Vec<Real>,
    residual: BivariatePolynomial,
    geometric: BivariatePolynomial,
    components: Vec<BivariateVerticalComponent>,
}

impl BivariateVerticalContent {
    /// Exact first-coordinate content, in ascending powers.
    pub fn factor(&self) -> &[Real] {
        &self.factor
    }
    /// Original nonvertical quotient, before any repeated-factor reduction.
    pub fn residual_equation(&self) -> &BivariatePolynomial {
        &self.residual
    }
    /// Reduced full supporting equation, including vertical components.
    pub fn geometric_equation(&self) -> &BivariatePolynomial {
        &self.geometric
    }
    /// All distinct vertical components in the closed rectangle.
    pub fn components(&self) -> &[BivariateVerticalComponent] {
        &self.components
    }
}

fn axis_polynomial(coefficients: &[Real]) -> BivariatePolynomial {
    BivariatePolynomial::new(coefficients.iter().map(|c| vec![c.clone()]).collect())
}

pub(super) fn decompose(
    equation: &BivariatePolynomial,
    domain: &BivariateCurveDomain,
    config: BivariateTopologyConfig,
) -> Result<BivariateCurveTopology, BivariateTopologyError> {
    let equation = rational_equation(equation)?;
    let report = extract_bivariate_polynomial_system_axis_factors(&equation, &equation);
    match report.status {
        BivariatePolynomialAxisFactorStatus::Reduced => {}
        BivariatePolynomialAxisFactorStatus::BudgetExhausted => {
            return Err(BivariateTopologyError::BudgetExhausted);
        }
        _ => return Err(BivariateTopologyError::Undecided),
    }
    let factor = report.first_parameter_factor;
    if factor.len() <= 1 {
        return Err(BivariateTopologyError::Undecided);
    }
    let residual = divide_bivariate_polynomial_exact(&equation, &axis_polynomial(&factor))
        .ok_or(BivariateTopologyError::Undecided)?;
    // The exact division authority checks the full reconstruction, including
    // original content powers. Its second-axis extraction is not used here.
    let support =
        square_free_part(factor.clone(), STRICT).ok_or(BivariateTopologyError::Undecided)?;
    // Every vertical root is a projection event even if it is a regular point
    // of the residual curve. Residual arcs are split there before adjacency.
    let mut topology = decompose_finite(&residual, domain, config, Some(&support))?;
    let mut components = Vec::new();
    for (event_index, event) in topology.events.iter().enumerate() {
        let mut derivative = factor.clone();
        let mut exponent = 0;
        while fiber_sign(
            &axis_polynomial(&derivative),
            &event.parameter,
            &Real::zero(),
        )? == Ordering::Equal
        {
            if derivative.len() <= 1 {
                return Err(BivariateTopologyError::Undecided);
            }
            derivative = polynomial_derivative(&derivative);
            exponent += 1;
        }
        if exponent > 0 {
            components.push(BivariateVerticalComponent {
                event_index,
                exponent,
            });
        }
    }
    if components.is_empty() {
        return Err(BivariateTopologyError::Undecided);
    }
    let geometric = bivariate_multiply(&axis_polynomial(&support), topology.graph_equation());
    topology.equation = equation;
    topology.vertical = Some(Box::new(BivariateVerticalContent {
        factor,
        residual,
        geometric,
        components,
    }));
    Ok(topology)
}
