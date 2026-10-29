//! Joint query points preserve sources as well as their common scalar field.

use super::*;
use crate::bezier_offset::PointFormSource2;
use crate::selected_field::{self as field, Value};
use std::sync::Arc;

pub(super) struct JointQueries2 {
    sources: [PointFormSource2; 2],
    values: [Value; 2],
}

impl std::fmt::Debug for JointQueries2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Shared operand definitions must not expand into diagnostic trees.
        f.debug_struct("JointQueries2").finish_non_exhaustive()
    }
}

impl PartialEq for JointQueries2 {
    fn eq(&self, other: &Self) -> bool {
        std::ptr::eq(self, other)
    }
}

impl JointQueries2 {
    pub(super) fn definition(&self) -> crate::CurvePointDefinition2 {
        crate::CurvePointDefinition2::RationalQueryPair {
            sources: self.sources.each_ref().map(|query| query.source.clone()),
            numerators: self.sources.each_ref().map(|query| query.numerator.clone()),
            denominators: self
                .sources
                .each_ref()
                .map(|query| query.denominator.clone()),
        }
    }
}

impl PolynomialPoint2 {
    pub(crate) fn from_queries(
        sources: [PointFormSource2; 2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurvePoint2>> {
        let strict = policy.strict_counterpart();
        let policy = &strict;
        field::finish(|| {
            let first = crate::bezier_offset::selected_field_query(&sources[0], policy)?;
            let second = crate::bezier_offset::selected_field_query(&sources[1], policy)?;
            let values = field::joined(&first, &second, policy)?;
            Ok(CurvePoint2(CurvePointData2::Polynomial(Arc::new(Self {
                authority: PolynomialPointAuthority2::Queries(Arc::new(JointQueries2 {
                    sources,
                    values,
                })),
                maps: Vec::new(),
            }))))
        })
    }

    pub(crate) fn field_query(
        &self,
        numerator: &BivariatePolynomial,
        denominator: &BivariatePolynomial,
        policy: &CurveContext,
    ) -> field::Result<Value> {
        let (numerator, denominator) = field::available(self.pullback(numerator, denominator))?;
        let native;
        let values = match &self.authority {
            PolynomialPointAuthority2::Fiber { fiber, .. } => {
                native = crate::bezier_offset::selected_field_fiber(fiber, policy)?;
                &native
            }
            PolynomialPointAuthority2::Queries(queries) => &queries.values,
        };
        field::query([&numerator, &denominator], values, policy)
    }

    pub(super) fn joint_parameter(
        &self,
        numerator: &BivariatePolynomial,
        denominator: &BivariatePolynomial,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveParameter2>> {
        field::finish(|| {
            let value = self.field_query(numerator, denominator, policy)?;
            let query = PointFormSource2::new(
                CurvePoint2(CurvePointData2::Polynomial(Arc::new(self.clone()))),
                numerator.clone(),
                denominator.clone(),
            );
            Ok(CurveParameter2::from_polynomial_query(
                crate::selected_query_scalar::SelectedQueryScalar2::new(query, value, policy)?,
            ))
        })
    }

    pub(super) fn joint_bounds(
        &self,
        steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        let result = field::finish(|| {
            let coordinate = |axis| match axis {
                Axis2::X => BivariatePolynomial::new(vec![vec![Real::zero()], vec![Real::one()]]),
                Axis2::Y => BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()]]),
            };
            let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
            let x = field::bounds(
                &self.field_query(&coordinate(Axis2::X), &one, policy)?,
                steps,
                policy,
            )?;
            let y = field::bounds(
                &self.field_query(&coordinate(Axis2::Y), &one, policy)?,
                steps,
                policy,
            )?;
            Ok(Aabb2::new_unchecked(
                Point2::new(x.lower, y.lower),
                Point2::new(x.upper, y.upper),
            ))
        });
        result.unwrap_or(Classification::Uncertain(UncertaintyReason::Predicate))
    }
}
