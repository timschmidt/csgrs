//! Native rational preimages retain the query's selected coefficient field.
use super::*;
use crate::bezier_split::CurveParameterDomain2;
use crate::polynomial_point::PolynomialPoint2;
use crate::{CurveFamily2, CurveParameterRange2, CurveParameterSet2};

pub(super) type SpanPreimages = Vec<(CurveParameterSet2, [bool; 2])>;
use hyperreal::RealSign;
use hypersolve::BivariatePolynomial;

fn decide<T>(value: Classification<T>) -> ExactCurveResult<T> {
    ExactCurveError::decided(CurveOperation2::Intersection, value)
}

/// Complete local preimages, with open ends at original denominator zeros.
/// Roots of one original coordinate equation are complete candidates; the
/// second equation, original denominator and active range validate each one.
pub(super) fn parameters(
    source: &RationalBezier2,
    range: &CurveParameterRange2,
    point: &CurvePoint2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<SpanPreimages> {
    parameters_with_map(source, range, point, family, policy, None)
}

pub(super) fn mapped_parameters(
    source: &RationalBezier2,
    range: &CurveParameterRange2,
    point: &CurvePoint2,
    family: CurveFamily2,
    policy: &CurveContext,
    map: &crate::RationalMap2,
) -> ExactCurveResult<SpanPreimages> {
    parameters_with_map(source, range, point, family, policy, Some(map))
}

fn parameters_with_map(
    source: &RationalBezier2,
    range: &CurveParameterRange2,
    point: &CurvePoint2,
    family: CurveFamily2,
    policy: &CurveContext,
    map: Option<&crate::RationalMap2>,
) -> ExactCurveResult<SpanPreimages> {
    let invalid = |cause| ExactCurveError::invalid(CurveOperation2::Intersection, family, cause);
    let basis = source.homogeneous_power_basis().map_err(invalid)?;
    let mapped = map
        .map(|map| {
            use hypersolve::bivariate_arithmetic::{
                bivariate_active_total_degree, bivariate_homogeneous_compose,
            };
            // The common degree below is sufficient for every coordinate. A
            // failed checked composition therefore reaches a platform storage or
            // allocation limit, rather than a missing geometric algorithm.
            let polynomials = [
                &map.numerators()[0],
                &map.numerators()[1],
                map.denominator(),
            ];
            let degree = polynomials
                .iter()
                .map(|p| {
                    bivariate_active_total_degree(p).ok_or_else(|| {
                        ExactCurveError::blocked(
                            CurveOperation2::Intersection,
                            family,
                            UncertaintyReason::BudgetExhausted,
                        )
                    })
                })
                .collect::<ExactCurveResult<Vec<_>>>()?
                .into_iter()
                .max()
                .unwrap_or(0);
            let univariate = |values: &[Real]| {
                BivariatePolynomial::new(values.iter().cloned().map(|value| vec![value]).collect())
            };
            let [x, y, w] =
                [&basis.x_numerator, &basis.y_numerator, &basis.weight].map(|v| univariate(v));
            polynomials
                .map(|p| {
                    bivariate_homogeneous_compose(p, &x, &y, &w, degree)
                        .ok_or_else(|| {
                            ExactCurveError::blocked(
                                CurveOperation2::Intersection,
                                family,
                                UncertaintyReason::BudgetExhausted,
                            )
                        })
                        .map(|p| {
                            p.coefficients
                                .into_iter()
                                .map(|row| row.into_iter().next().unwrap_or_else(Real::zero))
                                .collect::<Vec<_>>()
                        })
                })
                .into_iter()
                .collect::<ExactCurveResult<Vec<_>>>()
        })
        .transpose()?;
    let (coordinates, weight) = match &mapped {
        Some(values) => (
            [values[0].as_slice(), values[1].as_slice()],
            values[2].as_slice(),
        ),
        None => (
            [basis.x_numerator.as_slice(), basis.y_numerator.as_slice()],
            basis.weight.as_slice(),
        ),
    };

    let (_, bounds) = decide(
        CurveParameterDomain2::new(range, None)
            .finite_envelope(policy)
            .map_err(invalid)?,
    )?;
    let coefficient_forms = |coordinate: &[Real], axis| {
        (0..coordinate.len().max(weight.len()))
            .map(|i| {
                let c = coordinate.get(i).cloned().unwrap_or_else(Real::zero);
                let w = -weight.get(i).cloned().unwrap_or_else(Real::zero);
                let polynomial = if axis == 0 {
                    BivariatePolynomial::new(vec![vec![c], vec![w]])
                } else {
                    BivariatePolynomial::new(vec![vec![c, w]])
                };
                point.rational_form(
                    polynomial,
                    BivariatePolynomial::new(vec![vec![Real::one()]]),
                )
            })
            .collect::<ExactCurveResult<Vec<_>>>()
    };
    let mut candidates = None;
    for (axis, coordinate) in coordinates.into_iter().enumerate() {
        if let Some(roots) = PolynomialPoint2::selected_root_candidates(
            coefficient_forms(coordinate, axis)?,
            [bounds[0].clone(), bounds[1].clone()],
            hypersolve::AlgebraicFiberRootIsolationConfig::default(),
            policy,
        )? {
            candidates = Some(roots);
            break;
        }
    }
    let Some(candidates) = candidates else {
        // Identically vanishing equations describe the entire defined domain,
        // including all punctures of the ORIGINAL source and map guards.
        let mut holes: Vec<CurveParameter2> = Vec::new();
        for guard in [Some(basis.weight.as_slice()), map.map(|_| weight)]
            .into_iter()
            .flatten()
        {
            if crate::curve::polynomial_is_zero(guard, policy)? {
                return if guard == basis.weight.as_slice() {
                    Err(invalid(CurveError::ZeroRationalPointDenominator))
                } else {
                    Ok(Vec::new())
                };
            }
            let coefficients = guard
                .iter()
                .map(|c| {
                    point.rational_form(
                        BivariatePolynomial::new(vec![vec![c.clone()]]),
                        BivariatePolynomial::new(vec![vec![Real::one()]]),
                    )
                })
                .collect::<ExactCurveResult<Vec<_>>>()?;
            let roots = PolynomialPoint2::selected_root_candidates(
                coefficients,
                [bounds[0].clone(), bounds[1].clone()],
                hypersolve::AlgebraicFiberRootIsolationConfig::default(),
                policy,
            )?
            .expect("a nonzero guard has isolated roots");
            for root in roots {
                let parameter = decide(
                    crate::bezier_offset::point_affine_form_parameter(
                        &root,
                        &[Real::one(), Real::zero(), Real::zero()],
                        policy,
                    )
                    .map_err(invalid)?,
                )?;
                if !decide(
                    CurveParameterDomain2::new(range, None)
                        .contains_finite_parameter(&parameter, policy)
                        .map_err(invalid)?,
                )? {
                    continue;
                }
                let mut index = 0;
                while index < holes.len()
                    && decide(
                        holes[index]
                            .cmp_by_refinement(&parameter, policy)
                            .map_err(invalid)?,
                    )?
                    .is_lt()
                {
                    index += 1;
                }
                if index == holes.len()
                    || !decide(
                        holes[index]
                            .cmp_by_refinement(&parameter, policy)
                            .map_err(invalid)?,
                    )?
                    .is_eq()
                {
                    holes.insert(index, parameter);
                }
            }
        }
        let [start, end] = decide(range.ordered_endpoints(policy).map_err(invalid)?)?;
        let mut previous = start.clone();
        let mut included = true;
        let mut components = Vec::new();
        for hole in holes {
            if !decide(previous.cmp_by_refinement(&hole, policy).map_err(invalid)?)?.is_eq() {
                components.push((
                    CurveParameterSet2::Range(CurveParameterRange2::new_validated(
                        previous,
                        hole.clone(),
                    )),
                    [included, false],
                ));
            }
            previous = hole;
            included = false;
        }
        if !decide(previous.cmp_by_refinement(end, policy).map_err(invalid)?)?.is_eq() {
            components.push((
                CurveParameterSet2::Range(CurveParameterRange2::new_validated(
                    previous,
                    end.clone(),
                )),
                [included, true],
            ));
        }
        return Ok(components);
    };
    let mut parameters = Vec::new();
    for root in candidates {
        let mut parameter = decide(
            crate::bezier_offset::point_affine_form_parameter(
                &root,
                &[Real::one(), Real::zero(), Real::zero()],
                policy,
            )
            .map_err(invalid)?,
        )?;
        if !decide(
            CurveParameterDomain2::new(range, None)
                .contains_finite_parameter(&parameter, policy)
                .map_err(invalid)?,
        )? {
            continue;
        }
        let mut defined = true;
        for guard in [Some(basis.weight.as_slice()), map.map(|_| weight)]
            .into_iter()
            .flatten()
        {
            defined &= decide(parameter.polynomial_sign(guard, policy).map_err(invalid)?)?
                != RealSign::Zero;
        }
        if !defined {
            continue;
        } // Original source and map poles stay excluded.
        for endpoint in [range.start(), range.end()] {
            if decide(
                parameter
                    .cmp_by_refinement(endpoint, policy)
                    .map_err(invalid)?,
            )?
            .is_eq()
            {
                parameter = endpoint.clone();
                break;
            }
        }
        let mut evaluated =
            crate::curve::curve_evaluation::rational_point(source, &parameter, family, policy)?;
        if let Some(map) = map {
            evaluated = evaluated.rational_image_with_policy(map, policy)?;
        }
        if decide(evaluated.same_point(point, policy))? {
            parameters.push(parameter);
        }
    }
    Ok(parameters
        .into_iter()
        .map(|p| (CurveParameterSet2::Single(p), [true, true]))
        .collect())
}
