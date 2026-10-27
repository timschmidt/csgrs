//! Incidence of selected polynomial branches in a shared exact source chart.
//!
//! Resultants discover candidate abscissas, never selected points. Each point
//! is replayed on both original fibers. Common factors retain selected branch
//! overlaps and the finite residual and mixed-component contacts.

use std::cmp::Ordering;

use hyperreal::RealSign;
use hypersolve::bivariate_arithmetic::{
    bivariate_multiply, bivariate_parameter_derivative as derivative, bivariate_subtract,
};
use hypersolve::{
    BivariatePolynomial, CurveIntersectionResultantConfig, CurveIntersectionResultantStatus,
    CurveResultantParameter,
};

use super::*;
use crate::CurveFamily2;
use crate::curve::polynomial_is_zero;
use crate::{PolynomialCurveBranch2, Similarity2};

fn invalid(cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(
        CurveOperation2::Intersection,
        CurveFamily2::ImplicitPolynomial,
        cause,
    )
}

fn blocked(reason: UncertaintyReason) -> ExactCurveError {
    ExactCurveError::blocked(
        CurveOperation2::Intersection,
        CurveFamily2::ImplicitPolynomial,
        reason,
    )
}

fn decided<T>(value: Classification<T>) -> ExactCurveResult<T> {
    match value {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(blocked(reason)),
    }
}

fn order(
    a: &CurveParameter2,
    b: &CurveParameter2,
    policy: &CurveContext,
) -> ExactCurveResult<Ordering> {
    decided(a.cmp_by_refinement(b, policy).map_err(invalid)?)
}

fn publish(
    contacts: Vec<CurveIntersectionContact2>,
    overlaps: Vec<CurveIntersectionOverlap2>,
) -> CurveIntersectionResult2 {
    CurveIntersectionResult2 {
        data: Arc::new(CurveIntersectionResultData {
            span_pair_count: 1,
            contacts: contacts.into(),
            overlaps: overlaps.into(),
            blockers: Arc::from([]),
            parameter_components: None,
        }),
    }
}

fn resultant(equations: &[BivariatePolynomial; 2]) -> ExactCurveResult<Vec<Real>> {
    let report = hypersolve::curve_resultant::resultant_bivariate_polynomial_system_complete(
        &equations[0],
        &equations[1],
        CurveResultantParameter::First,
        CurveIntersectionResultantConfig::default(),
    );
    match report.status {
        CurveIntersectionResultantStatus::Constructed => Ok(report.resultant_coefficients),
        CurveIntersectionResultantStatus::BudgetExhausted => {
            Err(blocked(UncertaintyReason::BudgetExhausted))
        }
        CurveIntersectionResultantStatus::UndecidedCoefficient => {
            Err(blocked(UncertaintyReason::Predicate))
        }
        _ => Err(blocked(UncertaintyReason::Unsupported)),
    }
}

/// Keep a retained boundary parameter when the candidate is proved equal.
fn local_parameter(
    curve: &Curve2,
    value: &CurveParameter2,
    policy: &CurveContext,
) -> ExactCurveResult<CurveParameter2> {
    for boundary in [
        curve.parameter_domain().start(),
        curve.parameter_domain().end(),
    ] {
        if order(value, boundary, policy)? == Ordering::Equal {
            return Ok(boundary.clone());
        }
    }
    Ok(value.clone())
}

struct TangentForms {
    first_x: BivariatePolynomial,
    first_y: BivariatePolynomial,
    second_y: BivariatePolynomial,
    determinant: BivariatePolynomial,
    same_support: bool,
}

impl TangentForms {
    fn new(first: &BivariatePolynomial, second: &BivariatePolynomial, same_support: bool) -> Self {
        let first_x = derivative(first, CurveResultantParameter::First);
        let first_y = derivative(first, CurveResultantParameter::Second);
        let second_x = derivative(second, CurveResultantParameter::First);
        let second_y = derivative(second, CurveResultantParameter::Second);
        let determinant = bivariate_subtract(
            &bivariate_multiply(&first_x, &second_y),
            &bivariate_multiply(&second_x, &first_y),
        );
        Self {
            first_x,
            first_y,
            second_y,
            determinant,
            same_support,
        }
    }

    fn cross(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<RealSign>> {
        let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let sign = |polynomial| match crate::bezier_offset::point_rational_form_sign(
            point, polynomial, &one, policy,
        )
        .map_err(invalid)?
        {
            Classification::Decided(sign) => Ok(Some(sign)),
            Classification::Uncertain(UncertaintyReason::BudgetExhausted) => {
                Err(blocked(UncertaintyReason::BudgetExhausted))
            }
            Classification::Uncertain(_) => Ok(None),
        };
        let (Some(fy), Some(gy)) = (sign(&self.first_y)?, sign(&self.second_y)?) else {
            return Ok(None);
        };
        if self.same_support {
            // A smooth common support has one tangent line, including at a
            // vertical event. A singular gradient does not prove tangency.
            return Ok(
                if fy != RealSign::Zero || sign(&self.first_x)?.is_some_and(|s| s != RealSign::Zero)
                {
                    Some(RealSign::Zero)
                } else {
                    None
                },
            );
        }
        if fy == RealSign::Zero || gy == RealSign::Zero {
            return Ok(None);
        }
        Ok(sign(&self.determinant)?.map(|sign| {
            if (fy == RealSign::Negative) ^ (gy == RealSign::Negative) {
                negate(sign)
            } else {
                sign
            }
        }))
    }
}

fn negate(sign: RealSign) -> RealSign {
    match sign {
        RealSign::Positive => RealSign::Negative,
        RealSign::Negative => RealSign::Positive,
        RealSign::Zero => RealSign::Zero,
    }
}

/// A common invertible image preserves source incidence and parameter order.
/// Compare affine coefficients by certified value, including an explicit
/// identity versus an unmapped source. This is a sufficient chart certificate;
/// other images retain their existing dispatch and capability blockers.
fn shared_similarity(
    first: &PolynomialCurveBranch2,
    second: &PolynomialCurveBranch2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<Option<Similarity2>>> {
    if !first.rational_maps().is_empty() || !second.rational_maps().is_empty() {
        return Ok(None);
    }
    let coefficients = |branch: &PolynomialCurveBranch2| {
        branch.image_transform().map_or_else(
            || {
                [
                    Real::one(),
                    Real::zero(),
                    Real::zero(),
                    Real::one(),
                    Real::zero(),
                    Real::zero(),
                ]
            },
            |image| {
                let (a, b, d, e, x, y) = image.affine_components();
                [
                    a.clone(),
                    b.clone(),
                    d.clone(),
                    e.clone(),
                    x.clone(),
                    y.clone(),
                ]
            },
        )
    };
    let a = coefficients(first);
    let b = coefficients(second);
    let same = policy.strict_predicate_pass(|| -> ExactCurveResult<bool> {
        for (a, b) in a.into_iter().zip(b) {
            if order(&a.into(), &b.into(), policy)? != Ordering::Equal {
                return Ok(false);
            }
        }
        Ok(true)
    })?;
    if !same {
        return Ok(None);
    }
    Ok(Some(
        first
            .image_transform()
            .or_else(|| second.image_transform())
            .cloned(),
    ))
}

pub(super) fn intersect(
    first: &Curve2,
    second: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurveIntersectionResult2>> {
    let (Some(a), Some(b)) = (first.polynomial_branch(), second.polynomial_branch()) else {
        return Ok(None);
    };
    let Some(image) = shared_similarity(a, b, policy)? else {
        return Ok(None);
    };
    let inverse = image
        .as_ref()
        .map(Similarity2::inverse)
        .transpose()
        .map_err(invalid)?;
    let reflected = image
        .as_ref()
        .is_some_and(Similarity2::reverses_orientation);
    let policy = &policy.strict_counterpart();
    let [a_low, a_high] = decided(
        first
            .parameter_domain()
            .ordered_endpoints(policy)
            .map_err(invalid)?,
    )?;
    let [b_low, b_high] = decided(
        second
            .parameter_domain()
            .ordered_endpoints(policy)
            .map_err(invalid)?,
    )?;
    let low = if order(a_low, b_low, policy)? == Ordering::Less {
        b_low
    } else {
        a_low
    };
    let high = if order(a_high, b_high, policy)? == Ordering::Greater {
        b_high
    } else {
        a_high
    };
    let interval_order = order(low, high, policy)?;
    if interval_order == Ordering::Greater {
        return Ok(Some(publish(Vec::new(), Vec::new())));
    }

    let original = [
        a.topology().equation().clone(),
        b.topology().equation().clone(),
    ];
    let mut candidates = resultant(&original)?;
    let mut tangent_supports = original.clone();
    let mut shared_selected_support = false;
    let mut endpoints = false;
    if polynomial_is_zero(&candidates, policy)? {
        let components = decided(
            hypersolve::bivariate_components::extract_bivariate_system_components(
                original,
                CurveIntersectionResultantConfig::default(),
            ),
        )?;
        let Some(support) = components.support else {
            return Err(blocked(UncertaintyReason::Unsupported));
        };
        let a_shared = a.selects_component(&support, policy)?;
        let b_shared = b.selects_component(&support, policy)?;
        if a_shared && b_shared && interval_order == Ordering::Less {
            let sample = CurveParameter2::from(decided(
                low.strict_scalar_between_ordered(high, policy)
                    .map_err(invalid)?,
            )?);
            let p = first.point_at_with_policy(&sample, policy)?.into_value();
            let q = second.point_at_with_policy(&sample, policy)?.into_value();
            if decided(p.same_point(&q, policy))? {
                // Each original slab has only simple open-fiber roots. The
                // same selected common root cannot separate before an event.
                let (start, end) = if first.source_traversal_is_reversed() {
                    (high, low)
                } else {
                    (low, high)
                };
                let range = CurveParameterRange2::new_validated(start.clone(), end.clone());
                // The identity chart must cover the original source domain,
                // including negative and nonunit abscissas. The retained
                // overlap supplies its selected branch and active trim.
                let [source_low, source_high] = &a.topology().domain().first;
                let source_range = ParamRange::new(source_low.clone(), source_high.clone());
                return Ok(Some(publish(
                    Vec::new(),
                    vec![CurveIntersectionOverlap2 {
                        first_span_index: 0,
                        second_span_index: 0,
                        first_range: range.clone(),
                        second_range: range,
                        orientation: if first.source_traversal_is_reversed()
                            ^ second.source_traversal_is_reversed()
                        {
                            CurveOverlapOrientation2::Reversed
                        } else {
                            CurveOverlapOrientation2::Same
                        },
                        endpoint_inclusion: [true, true],
                        parameter_correspondence: CurveOverlapCorrespondence2::affine(
                            &source_range,
                            &source_range,
                        ),
                    }],
                )));
            }
        }
        tangent_supports = [
            if a_shared {
                support.clone()
            } else {
                components.residual_equations[0].clone()
            },
            if b_shared {
                support
            } else {
                components.residual_equations[1].clone()
            },
        ];
        shared_selected_support = a_shared && b_shared;
        // A mixed contact with the common factor is a projection event of
        // the source containing both factors. Its selected slab ends there.
        // Keep those boundaries as well as isolated residual-system roots.
        endpoints = true;
        candidates = resultant(&components.residual_equations)?;
    }
    let mut abscissas = a.section_abscissas(candidates)?;
    if endpoints {
        abscissas.extend([low.clone(), high.clone()]);
    }
    let mut retained: Vec<CurveParameter2> = Vec::new();
    for value in abscissas {
        if order(&value, low, policy)? == Ordering::Less
            || order(&value, high, policy)? == Ordering::Greater
        {
            continue;
        }
        let mut index = 0;
        while index < retained.len() && order(&retained[index], &value, policy)? == Ordering::Less {
            index += 1;
        }
        if index == retained.len() || order(&retained[index], &value, policy)? != Ordering::Equal {
            retained.insert(index, value);
        }
    }
    let forms = std::cell::OnceCell::new();
    let mut contacts = Vec::new();
    for value in retained {
        let first_parameter = local_parameter(first, &value, policy)?;
        let second_parameter = local_parameter(second, &value, policy)?;
        let point = first
            .point_at_with_policy(&first_parameter, policy)?
            .into_value();
        let other = second
            .point_at_with_policy(&second_parameter, policy)?
            .into_value();
        if !decided(point.same_point(&other, policy))? {
            continue;
        }
        let forms = forms.get_or_init(|| {
            TangentForms::new(
                &tangent_supports[0],
                &tangent_supports[1],
                shared_selected_support,
            )
        });
        let source_point = inverse.as_ref().map_or_else(
            || point.clone(),
            |inverse| {
                CurvePoint2::from(crate::bezier_offset::BezierSimilarityPoint2::new(
                    point.clone(),
                    inverse.clone(),
                    policy,
                ))
            },
        );
        let tangent_cross_sign = forms.cross(&source_point, policy)?.map(|sign| {
            if reflected
                ^ first.source_traversal_is_reversed()
                ^ second.source_traversal_is_reversed()
            {
                negate(sign)
            } else {
                sign
            }
        });
        let location = |parameter| CurveLocation2 {
            span_index: 0,
            span_range: CurveSpanRange2::from_affine_chart(&Real::one(), &Real::zero()),
            local_parameter: parameter,
        };
        contacts.push(CurveIntersectionContact2 {
            first: location(first_parameter),
            second: location(second_parameter),
            point,
            certified_transverse: tangent_cross_sign.is_some_and(|sign| sign != RealSign::Zero),
            tangent_cross_sign,
        });
    }
    Ok(Some(publish(contacts, Vec::new())))
}
