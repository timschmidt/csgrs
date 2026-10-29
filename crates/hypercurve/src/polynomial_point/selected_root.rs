//! Selected roots retain every coefficient's original point query.

use super::*;
use crate::bezier_offset::PointFormSource2;
use crate::selected_field::{self as field, FieldError, Value};
use crate::{CurvePointDefinition2, CurvePointRationalForm2};

fn coefficient_values(
    coefficients: &[CurvePointRationalForm2],
    policy: &CurveContext,
) -> field::Result<Vec<Value>> {
    let mut values: Vec<Value> = Vec::new();
    for coefficient in coefficients {
        // Even a zero or trailing coefficient retains its original pole guard.
        let value = crate::bezier_offset::selected_field_query(
            &PointFormSource2::new(
                coefficient.source().clone(),
                coefficient.numerator().clone(),
                coefficient.denominator().clone(),
            ),
            policy,
        )?;
        if let Some(first) = values.first() {
            let join = field::decided(first.field().joined_with(value.field(), policy)?)?;
            values = values
                .iter()
                .map(|value| join.embed(value))
                .collect::<Result<_, _>>()?;
            values.push(join.embed(&value)?);
        } else {
            values.push(value);
        }
    }
    if values.is_empty() {
        return Err(FieldError::ZeroPolynomial);
    }
    Ok(values)
}

fn selected_root(
    coefficients: Vec<Value>,
    root: &IsolatedRootInterval,
    policy: &CurveContext,
) -> field::Result<Value> {
    let parent = coefficients[0].field().clone();
    let Some(exact) = &root.exact_root else {
        let selected = field::decided(parent.select(
            coefficients,
            root.lower.clone(),
            root.upper.clone(),
            policy,
        )?)?;
        return Ok(selected.generator()?);
    };
    for boundary in [&root.lower, &root.upper] {
        match crate::classify::compare_reals(boundary, exact, policy) {
            Some(Ordering::Equal) => {}
            Some(_) => return Err(FieldError::InvalidSelector),
            None => return Err(FieldError::Uncertain(UncertaintyReason::Ordering)),
        }
    }
    let mut nonzero = false;
    for coefficient in &coefficients {
        nonzero |= field::decided(coefficient.sign(policy)?)? != hyperreal::RealSign::Zero;
    }
    if !nonzero {
        return Err(FieldError::ZeroPolynomial);
    }
    if field::decided(
        parent
            .evaluate_polynomial(&coefficients, exact)?
            .sign(policy)?,
    )? != hyperreal::RealSign::Zero
    {
        return Err(FieldError::NoRoot);
    }
    Ok(parent.constant(exact.clone())?)
}

impl PolynomialPoint2 {
    pub(crate) fn from_selected_root(
        coefficients: Vec<CurvePointRationalForm2>,
        root: IsolatedRootInterval,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePoint2> {
        let strict = policy.strict_counterpart();
        let result = field::finish(|| {
            let value = selected_root(coefficient_values(&coefficients, &strict)?, &root, &strict)?;
            let zero = value.field().constant(Real::zero())?;
            Ok(Self::from_joint_values(
                CurvePointDefinition2::SelectedPolynomialRoot {
                    coefficients,
                    root: IsolatedRootInterval {
                        distinct_root_count: 1,
                        ..root
                    },
                },
                [value, zero],
            ))
        });
        ExactCurveError::decided(
            CurveOperation2::Construction,
            result.map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Construction,
                    crate::CurveFamily2::ImplicitPolynomial,
                    cause,
                )
            })?,
        )
    }
}

impl PolynomialPoint2 {
    /// Finite roots on a closed chart. `None` denotes an identically zero
    /// substitution, which the incidence owner must handle as a component.
    pub(crate) fn selected_root_candidates(
        coefficients: Vec<CurvePointRationalForm2>,
        domain: [Real; 2],
        config: hypersolve::AlgebraicFiberRootIsolationConfig,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<Vec<CurvePoint2>>> {
        let strict = policy.strict_counterpart();
        let result = field::finish(|| {
            if coefficients.is_empty() {
                return Ok(None);
            }
            let values = coefficient_values(&coefficients, &strict)?;
            let mut context = hypersolve::selected_polynomial_field::SelectedPolynomialContext {
                field: values[0].field().clone(),
                policy: &strict,
            };
            let report = hypersolve::isolate_ordered_field_polynomial_roots_complete_with_config(
                values.clone(),
                &domain[0],
                &domain[1],
                hypersolve::OrderedFieldRootIsolationConfig {
                    max_subdivision_depth: config.max_subdivision_depth,
                    refinement_steps: config.refinement_steps,
                },
                &mut context,
            )?;
            use hypersolve::OrderedFieldRootIsolationStatus as Status;
            match report.status {
                Status::Isolated => {}
                Status::IdenticallyZero => return Ok(None),
                Status::BudgetExhausted => {
                    return Err(FieldError::Uncertain(UncertaintyReason::BudgetExhausted));
                }
                Status::InvalidInterval => return Err(FieldError::InvalidSelector),
                Status::CompleteFallbackRequired => {
                    return Err(FieldError::Uncertain(UncertaintyReason::Unsupported));
                }
            }
            let points = report
                .intervals
                .into_iter()
                .map(|root| {
                    let value = selected_root(values.clone(), &root, &strict)?;
                    let zero = value.field().constant(Real::zero())?;
                    Ok(Self::from_joint_values(
                        CurvePointDefinition2::SelectedPolynomialRoot {
                            coefficients: coefficients.clone(),
                            root,
                        },
                        [value, zero],
                    ))
                })
                .collect::<field::Result<Vec<_>>>()?;
            Ok(Some(points))
        });
        ExactCurveError::decided(
            CurveOperation2::Intersection,
            result.map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Intersection,
                    crate::CurveFamily2::ImplicitPolynomial,
                    cause,
                )
            })?,
        )
    }
}
