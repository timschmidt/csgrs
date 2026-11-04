//! Relative similarities retain both selected source parameters.
use super::*;
use crate::bezier_split::CurveParameterDomain2;
use hypersolve::bivariate_arithmetic::{bivariate_add, bivariate_scale};

#[derive(Clone, Debug)]
pub(crate) struct PolynomialPairCorrespondence2 {
    curves: [Curve2; 2],
    ranges: [CurveParameterRange2; 2],
}

impl PolynomialPairCorrespondence2 {
    pub(crate) fn map_parameter(
        &self,
        parameter: &CurveParameter2,
        forward: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        let from = usize::from(!forward);
        let to = 1 - from;
        ExactCurveError::classify_result((|| {
            if !decided(
                CurveParameterDomain2::new(&self.ranges[from], None)
                    .contains_finite_parameter(parameter, policy)
                    .map_err(invalid)?,
            )? {
                return Ok(None);
            }
            let point = self.curves[from]
                .point_at_with_policy(parameter, policy)?
                .into_value();
            let value = branch(&self.curves[to]).point_parameter(&point, policy)?;
            match value {
                Some(value)
                    if decided(
                        CurveParameterDomain2::new(&self.ranges[to], None)
                            .contains_finite_parameter(&value, policy)
                            .map_err(invalid)?,
                    )? =>
                {
                    Ok(Some(value))
                }
                _ => Ok(None),
            }
        })())
    }

    pub(crate) fn clipped_ranges(
        &self,
        first: &CurveParameterRange2,
        second: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(CurveParameterRange2, CurveParameterRange2)>>> {
        crate::bezier_split::clip_corresponding_parameter_ranges(
            &self.ranges[0],
            &self.ranges[1],
            first,
            second,
            policy,
            |p| self.map_parameter(p, true, policy),
            |p| self.map_parameter(p, false, policy),
        )
    }
}

fn branch(curve: &Curve2) -> &PolynomialCurveBranch2 {
    curve.polynomial_branch().expect("selected polynomial pair")
}
fn one() -> BivariatePolynomial {
    BivariatePolynomial::new(vec![vec![Real::one()]])
}
fn identity() -> Similarity2 {
    Similarity2::try_from_real_affine(1.into(), 0.into(), 0.into(), 1.into(), 0.into(), 0.into())
        .expect("the exact identity is a nonsingular similarity")
}
fn pullback(
    polynomial: &BivariatePolynomial,
    map: &Similarity2,
) -> ExactCurveResult<BivariatePolynomial> {
    crate::RationalMap2::from_similarity(map)
        .pullback(polynomial, &one())
        .map(|(numerator, _)| numerator)
        .ok_or_else(|| blocked(UncertaintyReason::Unsupported))
}
fn insert(
    values: &mut Vec<CurveParameter2>,
    value: CurveParameter2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    let mut i = 0;
    while i < values.len() && order(&values[i], &value, policy)?.is_lt() {
        i += 1;
    }
    if i == values.len() || !order(&values[i], &value, policy)?.is_eq() {
        values.insert(i, value);
    }
    Ok(())
}
fn location(parameter: CurveParameter2) -> CurveLocation2 {
    CurveLocation2 {
        span_index: 0,
        span_range: CurveSpanRange2::from_affine_chart(&Real::one(), &Real::zero()),
        local_parameter: parameter,
    }
}

pub(super) fn intersect(
    first: &Curve2,
    second: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurveIntersectionResult2>> {
    let (a, b) = (branch(first), branch(second));
    if !a.rational_maps().is_empty() || !b.rational_maps().is_empty() {
        return Ok(None);
    }
    let a_image = a.image_transform().cloned().unwrap_or_else(identity);
    let b_image = b.image_transform().cloned().unwrap_or_else(identity);
    let relative = a_image.then(&b_image.inverse().map_err(invalid)?);
    let (aa, ab, ac, ad, ax, ay) = relative.affine_components();
    let normalized =
        [aa, ab, ac, ad, ax, ay].map(|value| value.exact_rational_normal_form().map(Real::new));
    let [Some(aa), Some(ab), Some(ac), Some(ad), Some(ax), Some(ay)] = normalized else {
        return Ok(None);
    };
    // This is a proved rational query view. The original image definitions
    // and their shared dependencies remain on both retained operands.
    let relative = Similarity2::try_from_real_affine(aa, ab, ac, ad, ax, ay).map_err(invalid)?;
    let inverse_relative = relative.inverse().map_err(invalid)?;
    let inverse_world = a_image.inverse().map_err(invalid)?;
    let original = [
        a.topology().graph_equation().clone(),
        pullback(b.topology().graph_equation(), &relative)?,
    ];
    let mut supports = original.clone();
    let coefficients = resultant(&original)?;
    let shared = polynomial_is_zero(&coefficients, policy)?;
    let mut both_shared = false;
    let mut candidates;
    if shared {
        let components = decided(
            hypersolve::bivariate_components::extract_bivariate_system_components(
                original,
                CurveIntersectionResultantConfig::default(),
            ),
        )?;
        let support = components
            .support
            .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
        let a_shared = a.selects_component(&support, policy)?;
        let b_shared = b.selects_component(&pullback(&support, &inverse_relative)?, policy)?;
        both_shared = a_shared && b_shared;
        supports = [
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
        candidates = if both_shared {
            Vec::new()
        } else {
            a.section_abscissas(resultant(&components.residual_equations)?)?
        };
    } else {
        candidates = a.section_abscissas(coefficients)?;
    }
    let [low, high] = decided(
        first
            .parameter_domain()
            .ordered_endpoints(policy)
            .map_err(invalid)?,
    )?;
    // A graph chart is injective and regular in its open slab. Common selected
    // arcs can enter/leave one another only through an endpoint. Transport all
    // endpoints before testing the intervening connected intervals.
    let mut cuts = Vec::new();
    if shared {
        cuts.extend([low.clone(), high.clone()]);
        for parameter in [
            second.parameter_domain().start(),
            second.parameter_domain().end(),
        ] {
            let point = second.point_at_with_policy(parameter, policy)?.into_value();
            if let Some(value) = a.point_parameter(&point, policy)? {
                insert(&mut cuts, value, policy)?;
            }
        }
    }
    let mut overlaps = Vec::new();
    let mut covered = Vec::new();
    if both_shared {
        for ends in cuts.windows(2) {
            let sample = CurveParameter2::from(decided(
                ends[0]
                    .strict_scalar_between_ordered(&ends[1], policy)
                    .map_err(invalid)?,
            )?);
            let point = first.point_at_with_policy(&sample, policy)?.into_value();
            if b.point_parameter(&point, policy)?.is_none() {
                continue;
            }
            let mut second_ends = Vec::new();
            for end in ends {
                let point = first.point_at_with_policy(end, policy)?.into_value();
                second_ends.push(b.point_parameter(&point, policy)?.ok_or_else(|| {
                    invalid(CurveError::Topology(
                        "a closed common arc lost its endpoint".into(),
                    ))
                })?);
            }
            let increasing = order(&second_ends[0], &second_ends[1], policy)?;
            if increasing.is_eq() {
                return Err(invalid(CurveError::Topology(
                    "injective common charts have collapsed parameter transport".into(),
                )));
            }
            let (start, end) = if first.source_traversal_is_reversed() {
                (1, 0)
            } else {
                (0, 1)
            };
            let first_range =
                CurveParameterRange2::new_validated(ends[start].clone(), ends[end].clone());
            let second_range = CurveParameterRange2::new_validated(
                second_ends[start].clone(),
                second_ends[end].clone(),
            );
            let same = increasing.is_lt()
                ^ first.source_traversal_is_reversed()
                ^ second.source_traversal_is_reversed();
            overlaps.push(CurveIntersectionOverlap2 {
                first_span_index: 0,
                second_span_index: 0,
                first_range: first_range.clone(),
                second_range: second_range.clone(),
                orientation: if same {
                    CurveOverlapOrientation2::Same
                } else {
                    CurveOverlapOrientation2::Reversed
                },
                endpoint_inclusion: [true, true],
                parameter_correspondence: CurveOverlapCorrespondence2::PolynomialPair(Arc::new(
                    PolynomialPairCorrespondence2 {
                        curves: [first.clone(), second.clone()],
                        ranges: [first_range, second_range],
                    },
                )),
            });
            covered.push([ends[0].clone(), ends[1].clone()]);
        }
    }
    candidates.extend(cuts);
    let mut retained = Vec::new();
    for value in candidates {
        if order(&value, low, policy)?.is_lt() || order(&value, high, policy)?.is_gt() {
            continue;
        }
        insert(&mut retained, value, policy)?;
    }
    let forms = std::cell::OnceCell::new();
    let mut contacts = Vec::new();
    for value in retained {
        let mut on_overlap = false;
        for ends in &covered {
            if !order(&value, &ends[0], policy)?.is_lt()
                && !order(&value, &ends[1], policy)?.is_gt()
            {
                on_overlap = true;
                break;
            }
        }
        if on_overlap {
            continue;
        }
        let first_parameter = local_parameter(first, &value, policy)?;
        let point = first
            .point_at_with_policy(&first_parameter, policy)?
            .into_value();
        let Some(second_parameter) = b.point_parameter(&point, policy)? else {
            continue;
        };
        let source = CurvePoint2::from(crate::bezier_offset::BezierSimilarityPoint2::new(
            point.clone(),
            inverse_world.clone(),
            policy,
        ));
        let forms = forms.get_or_init(|| {
            let mut forms = TangentForms::new(&supports[0], &supports[1], both_shared);
            let (_, bb, _, bd, _, _) = inverse_relative.affine_components();
            forms.second_y = bivariate_add(
                &bivariate_scale(derivative(&supports[1], CurveResultantParameter::First), bb),
                &bivariate_scale(
                    derivative(&supports[1], CurveResultantParameter::Second),
                    bd,
                ),
            );
            forms
        });
        let tangent_cross_sign = forms.cross(&source, policy)?.map(|sign| {
            if a_image.reverses_orientation()
                ^ relative.reverses_orientation()
                ^ first.source_traversal_is_reversed()
                ^ second.source_traversal_is_reversed()
            {
                negate(sign)
            } else {
                sign
            }
        });
        contacts.push(CurveIntersectionContact2 {
            first: location(first_parameter),
            second: location(second_parameter),
            point,
            certified_transverse: tangent_cross_sign.is_some_and(|s| s != RealSign::Zero),
            tangent_cross_sign,
        });
    }
    Ok(Some(publish(contacts, overlaps)))
}
