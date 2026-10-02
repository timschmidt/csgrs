//! Exact common components of two-equation bivariate systems.
//!
//! Positive-dimensional common components are extracted by exact division
//! before a zero-dimensional residual is solved. Discovery reports may
//! propose a factor; exact division is the authority that removes it.

use hyperreal::Real;

use crate::bivariate_arithmetic::{
    bivariate_multiply, bivariate_outer_product, bivariate_storage_bidegree_sum,
    divide_bivariate_system_component,
};
use crate::classification::{Classification, UncertaintyReason};
use crate::curve_resultant::*;

/// Common components of a two-equation bivariate system and its residual.
pub struct BivariateSystemComponents {
    /// Product of the distinct extracted component supports, if any.
    pub support: Option<BivariatePolynomial>,
    /// The system after exact division by every extracted component.
    pub residual_equations: [BivariatePolynomial; 2],
}

/// The bivariate support `denominator(t)*u - numerator(t)` of a rational
/// component `u = numerator(t)/denominator(t)` on the retained parameter `t`.
pub fn rational_parameter_component_support(
    retained_parameter: CurveResultantParameter,
    numerator: &[Real],
    denominator: &[Real],
) -> Option<BivariatePolynomial> {
    if numerator.is_empty() || denominator.is_empty() {
        return None;
    }
    let support = match retained_parameter {
        // lifted = numerator(retained) / denominator(retained)
        CurveResultantParameter::First => {
            let count = numerator.len().max(denominator.len());
            (0..count)
                .map(|power| {
                    vec![
                        -numerator.get(power).cloned().unwrap_or_else(Real::zero),
                        denominator.get(power).cloned().unwrap_or_else(Real::zero),
                    ]
                })
                .collect()
        }
        CurveResultantParameter::Second => vec![
            numerator.iter().map(|coefficient| -coefficient).collect(),
            denominator.to_vec(),
        ],
    };
    Some(BivariatePolynomial::new(support))
}

/// Common components fixed on one parameter axis.
pub struct BivariateAxisComponents {
    /// Square-free supports of the extracted axis fibers.
    pub supports: Vec<BivariatePolynomial>,
    /// The system after exact division by those fibers.
    pub residual_equations: [BivariatePolynomial; 2],
}

/// Extracts exact common fibers before the generic component solver.
///
/// A component fixed on one parameter axis is not finite over that axis, so a
/// lifted subresultant need not publish it. Hypersolve's axis-content report
/// discovers those factors; exact division here remains the authorization for
/// saturation. Square-free reduction keeps one copy of each geometric fiber
/// whenever strict exact arithmetic can prove it.
pub fn extract_bivariate_axis_components(
    equations: &[BivariatePolynomial; 2],
) -> Classification<Option<BivariateAxisComponents>> {
    let report = extract_bivariate_polynomial_system_axis_factors(&equations[0], &equations[1]);
    if report.status != BivariatePolynomialAxisFactorStatus::Reduced {
        return match report.status {
            BivariatePolynomialAxisFactorStatus::ZeroSystem => {
                Classification::Uncertain(UncertaintyReason::Boundary)
            }
            BivariatePolynomialAxisFactorStatus::Primitive
            | BivariatePolynomialAxisFactorStatus::UnsupportedCoefficient
            | BivariatePolynomialAxisFactorStatus::DivisionFailed => Classification::Decided(None),
            BivariatePolynomialAxisFactorStatus::Reduced => unreachable!(),
        };
    }

    let mut residual = equations.clone();
    let mut supports = Vec::with_capacity(2);
    for (factor, axis) in [
        (
            report.first_parameter_factor,
            CurveResultantParameter::First,
        ),
        (
            report.second_parameter_factor,
            CurveResultantParameter::Second,
        ),
    ] {
        if factor.len() <= 1 {
            continue;
        }
        let square_free = crate::square_free_part(factor.clone(), crate::PredicatePolicy::STRICT)
            .unwrap_or_else(|| factor.clone());
        let support = match axis {
            CurveResultantParameter::First => bivariate_outer_product(&square_free, &[Real::one()]),
            CurveResultantParameter::Second => {
                bivariate_outer_product(&[Real::one()], &square_free)
            }
        };
        let division_support = match axis {
            CurveResultantParameter::First => bivariate_outer_product(&factor, &[Real::one()]),
            CurveResultantParameter::Second => bivariate_outer_product(&[Real::one()], &factor),
        };
        let Some(reduced) = divide_bivariate_system_component(&residual, &division_support) else {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        };
        supports.push(support);
        residual = reduced;
    }
    if supports.is_empty() {
        return Classification::Uncertain(UncertaintyReason::Boundary);
    }
    Classification::Decided(Some(BivariateAxisComponents {
        supports,
        residual_equations: residual,
    }))
}

/// Extracts every exactly published common factor and retains its geometric
/// support separately from the zero-dimensional residual system.
///
/// Hypersolve's reduced equations are useful discovery evidence, but exact
/// division here is the authority that permits saturation. Multiplicity is
/// removed from the residual while each distinct support is multiplied only
/// once into the union that must subsequently be intersected with the exact
/// norm eliminant. This is the essential distinction from the
/// historical blanket saturation path: no radical component is discarded
/// before all of its possible selected-branch points have been replayed.
pub fn extract_bivariate_system_components(
    mut residual_equations: [BivariatePolynomial; 2],
    config: CurveIntersectionResultantConfig,
) -> Classification<BivariateSystemComponents> {
    let mut support: Option<BivariatePolynomial> = None;
    loop {
        match extract_bivariate_axis_components(&residual_equations) {
            Classification::Decided(Some(axis)) => {
                for component in axis.supports {
                    support = Some(match support {
                        Some(support) => bivariate_multiply(&support, &component),
                        None => component,
                    });
                }
                residual_equations = axis.residual_equations;
                continue;
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Classification::Uncertain(reason);
            }
        }
        let previous_degree = residual_equations
            .iter()
            .map(bivariate_storage_bidegree_sum)
            .sum::<usize>();
        let mut next = None;
        let mut blocker = None;
        for retained_parameter in [
            CurveResultantParameter::First,
            CurveResultantParameter::Second,
        ] {
            let report = parameter_component_bivariate_polynomial_system_complete(
                &residual_equations[0],
                &residual_equations[1],
                retained_parameter,
                config,
            );
            let component = match report.status {
                BivariatePolynomialComponentStatus::Rational => {
                    rational_parameter_component_support(
                        retained_parameter,
                        &report.numerator_coefficients,
                        &report.denominator_coefficients,
                    )
                }
                BivariatePolynomialComponentStatus::Implicit => report.implicit_component,
                BivariatePolynomialComponentStatus::UndecidedCoefficient => {
                    if support.is_none() {
                        blocker = Some(UncertaintyReason::RealSign);
                    }
                    continue;
                }
                BivariatePolynomialComponentStatus::EmptyEquation
                | BivariatePolynomialComponentStatus::UnsupportedLiftedDegree
                | BivariatePolynomialComponentStatus::DegreeBoundExceeded
                | BivariatePolynomialComponentStatus::DeterminantError
                | BivariatePolynomialComponentStatus::InterpolationFailed => {
                    if support.is_none() {
                        blocker = Some(UncertaintyReason::Boundary);
                    }
                    continue;
                }
                BivariatePolynomialComponentStatus::NoSupportedComponent
                | BivariatePolynomialComponentStatus::DivisionFailed => continue,
            };
            let Some(component) = component else {
                if support.is_none() {
                    blocker = Some(UncertaintyReason::Boundary);
                }
                continue;
            };
            let Some(reduced) = divide_bivariate_system_component(&residual_equations, &component)
            else {
                if support.is_none() {
                    blocker = Some(UncertaintyReason::Boundary);
                }
                continue;
            };
            next = Some((component, reduced));
            break;
        }
        let Some((component, reduced)) = next else {
            return if let Some(reason) = blocker {
                Classification::Uncertain(reason)
            } else {
                Classification::Decided(BivariateSystemComponents {
                    support,
                    residual_equations,
                })
            };
        };
        let next_degree = reduced
            .iter()
            .map(bivariate_storage_bidegree_sum)
            .sum::<usize>();
        if next_degree >= previous_degree {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
        support = Some(match support {
            Some(support) => bivariate_multiply(&support, &component),
            None => component,
        });
        residual_equations = reduced;
    }
}

/// The union of two optional component supports.
pub fn merge_parameter_component_support(
    first: Option<BivariatePolynomial>,
    second: Option<BivariatePolynomial>,
) -> Option<BivariatePolynomial> {
    match (first, second) {
        (Some(first), Some(second)) => Some(bivariate_multiply(&first, &second)),
        (Some(support), None) | (None, Some(support)) => Some(support),
        (None, None) => None,
    }
}

/// Keeps the configured component extractor as the hot schedule without
/// allowing its degree budget to discard an exact positive-dimensional path.
#[inline]
pub fn parameter_component_bivariate_polynomial_system_complete(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    config: CurveIntersectionResultantConfig,
) -> BivariatePolynomialComponentReport {
    let report = parameter_component_bivariate_polynomial_system(
        first_equation,
        second_equation,
        retained_parameter,
        config,
    );
    if report.status != BivariatePolynomialComponentStatus::DegreeBoundExceeded
        || config.max_resultant_degree == usize::MAX
    {
        return report;
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypersolve",
        "bivariate-component",
        "unbounded-cold-continuation",
    );
    parameter_component_bivariate_polynomial_system(
        first_equation,
        second_equation,
        retained_parameter,
        CurveIntersectionResultantConfig {
            max_resultant_degree: usize::MAX,
            ..config
        },
    )
}
