//! Reuse a known ordinate after checking its exact fiber rank and full census.

use super::*;
use crate::bezier_offset::PointFormSource2;
use crate::selected_field::{self as field, FieldError, Value};
use hypersolve::selected_polynomial_field::SelectedPolynomialContext;
use hypersolve::{
    ordered_field_count_roots_between_values as count,
    ordered_field_polynomial_linear_quotient as quotient,
    ordered_field_polynomial_sign_at_value as sign_at,
};

fn remove_known_root(
    mut polynomial: Vec<Value>,
    root: &Value,
    context: &mut SelectedPolynomialContext<'_, CurveContext>,
) -> field::Result<Vec<Value>> {
    // Only the temporary counting polynomial loses these factors. Every
    // original equation, multiplicity and selected point remains unchanged.
    loop {
        polynomial = quotient(&polynomial, root, context)?;
        if polynomial.len() <= 1 || sign_at(&polynomial, root, context)? != Ordering::Equal {
            return Ok(polynomial);
        }
    }
}

impl PolynomialPoint2 {
    pub(crate) fn query_fiber_rank(
        equation: &BivariatePolynomial,
        parameter: &CurveParameter2,
        ordinate: &PointFormSource2,
        domain: &[Real; 2],
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<(usize, usize)>> {
        let strict = policy.strict_counterpart();
        selected_fiber::resolve(CurveOperation2::Intersection, || {
            let (x, polynomial) = selected_fiber::coefficients(equation, parameter, &strict)?;
            let y = crate::bezier_offset::selected_field_query(ordinate, &strict)?;
            let join = field::decided(x.field().joined_with(y.field(), &strict)?)?;
            let y = join.embed(&y)?;
            let mut polynomial = polynomial
                .iter()
                .map(|value| join.embed(value))
                .collect::<std::result::Result<Vec<_>, _>>()?;
            let base = join.field();
            let lower = base.constant(domain[0].clone())?;
            let upper = base.constant(domain[1].clone())?;
            let mut context = SelectedPolynomialContext {
                field: base.clone(),
                policy: &strict,
            };
            if field::decided(upper.subtract(&lower)?.sign(&strict)?)?
                != hyperreal::RealSign::Positive
            {
                return Err(FieldError::InvalidSelector);
            }
            let first = field::decided(y.subtract(&lower)?.sign(&strict)?)?;
            let last = field::decided(y.subtract(&upper)?.sign(&strict)?)?;
            if first == hyperreal::RealSign::Negative || last == hyperreal::RealSign::Positive {
                return Ok(None);
            }
            while let Some(leading) = polynomial.last() {
                if field::decided(leading.sign(&strict)?)? != hyperreal::RealSign::Zero {
                    break;
                }
                polynomial.pop();
            }
            if polynomial.is_empty() {
                return Err(FieldError::ZeroPolynomial);
            }
            if sign_at(&polynomial, &y, &mut context)? != Ordering::Equal {
                return Ok(None);
            }
            polynomial = remove_known_root(polynomial, &y, &mut context)?;
            let mut at_lower = 0_usize;
            let mut at_upper = 0_usize;
            if first != hyperreal::RealSign::Zero
                && sign_at(&polynomial, &lower, &mut context)? == Ordering::Equal
            {
                polynomial = remove_known_root(polynomial, &lower, &mut context)?;
                at_lower = 1;
            }
            if last != hyperreal::RealSign::Zero
                && sign_at(&polynomial, &upper, &mut context)? == Ordering::Equal
            {
                polynomial = remove_known_root(polynomial, &upper, &mut context)?;
                at_upper = 1;
            }
            let mut roots = |a, b| {
                count(&polynomial, a, b, &mut context)?
                    .ok_or(FieldError::Uncertain(UncertaintyReason::Predicate))
            };
            let interior = roots(&lower, &upper)?;
            let rank = if first == hyperreal::RealSign::Zero {
                0
            } else {
                roots(&lower, &y)? + at_lower
            };
            Ok(Some((rank, interior + at_lower + at_upper + 1)))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CurvePointDefinition2, CurvePointRationalForm2};

    fn p(rows: &[&[i32]]) -> BivariatePolynomial {
        BivariatePolynomial::new(
            rows.iter()
                .map(|row| row.iter().copied().map(Real::from).collect())
                .collect(),
        )
    }
    fn source() -> CurvePoint2 {
        CurvePointDefinition2::PolynomialFiber {
            equation: p(&[&[0, 0, 0, 1], &[-1]]),
            parameter: (Real::one() / Real::from(2)).unwrap().into(),
            root: IsolatedRootInterval {
                lower: 0.into(),
                upper: 1.into(),
                exact_root: None,
                distinct_root_count: 1,
            },
        }
        .into_point()
        .unwrap()
    }
    fn query(point: &CurvePoint2, numerator: BivariatePolynomial) -> CurvePointRationalForm2 {
        point.rational_form(numerator, p(&[&[1]])).unwrap()
    }

    #[test]
    fn known_fiber_rank_counts_repeated_boundary_roots_without_merging_nearby_points() {
        let source = source();
        let equation = p(&[&[0, 1, 0, -2, 0, 1]]); // y(y^2-1)^2
        let domain = [Real::from(-1), Real::one()];
        let epsilon = (Real::one() / Real::from(1_u128 << 80)).unwrap();
        for (y, expected) in [
            (Real::from(-1), Some((0, 3))),
            (Real::zero(), Some((1, 3))),
            (Real::one(), Some((2, 3))),
            (&Real::one() - &epsilon, None),
            (Real::from(2), None),
        ] {
            let point = CurvePointDefinition2::RationalQueryPair {
                sources: [source.clone(), source.clone()],
                numerators: [p(&[&[0, 1]]), BivariatePolynomial::new(vec![vec![y]])],
                denominators: [p(&[&[1]]), p(&[&[1]])],
            }
            .into_point()
            .unwrap();
            let parameter = query(&point, p(&[&[], &[1]])).parameter().unwrap();
            let ordinate = PointFormSource2::new(point.clone(), p(&[&[0, 1]]), p(&[&[1]]));
            assert_eq!(
                PolynomialPoint2::query_fiber_rank(
                    &equation,
                    &parameter,
                    &ordinate,
                    &domain,
                    &CurveContext::STRICT
                )
                .unwrap(),
                expected
            );
            let pole = PointFormSource2::new(point, p(&[&[0]]), p(&[&[0]]));
            assert!(
                PolynomialPoint2::query_fiber_rank(
                    &equation,
                    &parameter,
                    &pole,
                    &domain,
                    &CurveContext::STRICT
                )
                .is_err()
            );
        }
    }
}
