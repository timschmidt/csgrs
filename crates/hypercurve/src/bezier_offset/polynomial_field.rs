//! Lossless imports of admitted native authorities into selected polynomial fields.

use super::*;
use crate::selected_field::{self as field, Field, Value};
use crate::selected_scalar::SelectedScalar2;

pub(crate) fn selected_field_bezier(
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> field::Result<Value> {
    let sources = match parameter {
        BezierParameter2::Exact(_) => Vec::new(),
        BezierParameter2::Algebraic(root) => vec![certified_parameter_representation(root, policy)],
    };
    let rank = sources.len();
    let one = field::available(DenseTensorPolynomial::try_new(
        vec![1; rank],
        vec![Real::one()],
    ))?;
    let native = field::available(RecursiveQuadraticField::base(sources, one.clone(), one))?;
    let field = Field::from_recursive(native.clone());
    match parameter {
        BezierParameter2::Exact(value) => Ok(field.constant(value.clone())?),
        BezierParameter2::Algebraic(_) => {
            let RecursiveQuadraticField::Base(base) = native else {
                unreachable!("native base")
            };
            let axis = field::available(DenseTensorPolynomial::from_axis_polynomial(
                1,
                0,
                &[Real::zero(), Real::one()],
            ))?;
            let value = field::available(recursive_quadratic_rational_value(&base, axis))?;
            Ok(field.import_recursive(&value)?)
        }
    }
}

pub(crate) fn selected_field_fiber(
    parameter: &BezierAlgebraicSelectedFiberParameter2,
    policy: &CurveContext,
) -> field::Result<[Value; 2]> {
    parameter
        .validate_policy(policy)
        .map_err(field::FieldError::Context)?;
    let (equation, base) = parameter.defining_fiber();
    let x = selected_field_bezier(&BezierParameter2::Algebraic(base.clone()), policy)?;
    let parent = x.field();
    let root = parameter.root();
    if let Some(value) = &root.exact_root {
        return Ok([x.clone(), parent.constant(value.clone())?]);
    }
    let degree = equation
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    let coefficients = (0..degree)
        .map(|power| {
            let coefficients = equation
                .coefficients
                .iter()
                .map(|row| row.get(power).cloned().unwrap_or_else(Real::zero))
                .collect::<Vec<_>>();
            field::polynomial(&coefficients, &x)
        })
        .collect::<field::Result<Vec<_>>>()?;
    // Non-point fiber selectors are admitted endpoint-free open intervals.
    // Native base roots retain their own endpoint ownership in the native field.
    let selected = field::decided(parent.select(
        coefficients,
        root.lower.clone(),
        root.upper.clone(),
        policy,
    )?)?;
    Ok([selected.lift(&x)?, selected.generator()?])
}

pub(crate) fn selected_field_scalar(
    parameter: &SelectedScalar2,
    policy: &CurveContext,
) -> field::Result<Value> {
    let strict = policy.strict_counterpart();
    let policy = &strict;
    match parameter {
        SelectedScalar2::Bezier(value) => selected_field_bezier(value, policy),
        SelectedScalar2::SelectedFiber(value) => {
            Ok(selected_field_fiber(value, policy)?[1].clone())
        }
        SelectedScalar2::Polynomial(value) => value.value(),
        SelectedScalar2::RecursiveProjective(value) => {
            value
                .validate_policy(policy)
                .map_err(field::FieldError::Context)?;
            if let Some(value) = value.projective_scalar() {
                let field = Field::from_recursive(value.numerator.field());
                let numerator = field.import_recursive(&value.numerator)?;
                let denominator = field.import_recursive(&value.denominator)?;
                field::decided(numerator.divide(&denominator, policy)?)
            } else {
                let native = field::curve(value.promoted_bezier_parameter_complete(policy))?;
                selected_field_bezier(&native, policy)
            }
        }
    }
}

pub(crate) fn selected_field_query(
    query: &PointFormSource2,
    policy: &CurveContext,
) -> field::Result<Value> {
    let pulled = field::curve(point_form_source(
        &query.source,
        &query.numerator,
        &query.denominator,
        policy,
    ))?;
    if let CurvePointData2::Polynomial(point) = &pulled.source.0 {
        return point.field_query(&pulled.numerator, &pulled.denominator, policy);
    }
    let value = field::available(field::curve(
        super::rational_form::recursive_rational_value(
            &pulled.source,
            &pulled.numerator,
            &pulled.denominator,
            policy,
        ),
    )?)?;
    let field = Field::from_recursive(value.numerator.field());
    field::decided(
        field
            .import_recursive(&value.numerator)?
            .divide(&field.import_recursive(&value.denominator)?, policy)?,
    )
}
