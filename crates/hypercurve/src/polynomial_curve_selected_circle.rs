//! Circle charts retain their exact center as coefficient queries.

use super::*;
use crate::{CurvePointDefinition2, CurvePointRationalForm2};
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_arithmetic::bivariate_active_total_degree;
use hypersolve::tensor_resultant::DenseTensorPolynomial as Tensor;

fn available<T>(value: Option<T>) -> ExactCurveResult<T> {
    value.ok_or_else(|| blocked(UncertaintyReason::Unsupported))
}
fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn form(
    point: &CurvePoint2,
    numerator: BivariatePolynomial,
) -> ExactCurveResult<CurvePointRationalForm2> {
    point.rational_form(numerator, polynomial(&[&[1]]))
}
fn powers(value: &Tensor, degree: usize) -> Option<Vec<Tensor>> {
    let mut powers = vec![Tensor::try_new(vec![1; 3], vec![Real::one()])?];
    for _ in 0..degree {
        powers.push(powers.last()?.multiply(value)?);
    }
    Some(powers)
}
fn coefficients(
    equation: &BivariatePolynomial,
    center: &CurvePoint2,
    radius: &Real,
    left: bool,
) -> ExactCurveResult<Vec<CurvePointRationalForm2>> {
    let degree = available(bivariate_active_total_degree(equation))?;
    let cx = available(Tensor::from_axis_polynomial(3, 0, &[0.into(), 1.into()]))?;
    let cy = available(Tensor::from_axis_polynomial(3, 1, &[0.into(), 1.into()]))?;
    let w = available(Tensor::from_axis_polynomial(
        3,
        2,
        &[1.into(), 0.into(), 1.into()],
    ))?;
    let dx = available(Tensor::from_axis_polynomial(
        3,
        2,
        &[1.into(), 0.into(), (-1).into()],
    ))?;
    let dy = available(Tensor::from_axis_polynomial(3, 2, &[0.into(), 2.into()]))?;
    let radial_x = if left {
        -radius.clone()
    } else {
        radius.clone()
    };
    let x = available(available(cx.multiply(&w))?.add(&available(dx.scale(&radial_x))?))?;
    let y = available(available(cy.multiply(&w))?.add(&available(dy.scale(radius))?))?;
    let xp = available(powers(&x, degree))?;
    let yp = available(powers(&y, degree))?;
    let wp = available(powers(&w, degree))?;
    let mut result = available(Tensor::zero(vec![1; 3]))?;
    for (i, row) in equation.coefficients.iter().enumerate() {
        for (j, coefficient) in row.iter().enumerate() {
            if coefficient.zero_status() == hyperreal::ZeroKnowledge::Zero {
                continue;
            }
            let remaining = available(degree.checked_sub(i).and_then(|d| d.checked_sub(j)))?;
            let term = available(available(xp[i].multiply(&yp[j]))?.multiply(&wp[remaining]))?;
            result = available(result.add(&available(term.scale(coefficient))?))?;
        }
    }
    let shape = result.dimensions();
    (0..shape[2])
        .map(|power| {
            let rows = (0..shape[0])
                .map(|i| {
                    (0..shape[1])
                        .map(|j| {
                            result
                                .coefficient(&[i, j, power])
                                .expect("validated tensor shape")
                                .clone()
                        })
                        .collect()
                })
                .collect();
            form(center, BivariatePolynomial::new(rows))
        })
        .collect()
}
fn point_on_circle(
    center: &CurvePoint2,
    radius: &Real,
    parameter: &CurvePoint2,
    left: bool,
    policy: &CurveContext,
) -> ExactCurveResult<CurvePoint2> {
    let denominator = polynomial(&[&[1], &[], &[1]]);
    let dx = parameter
        .rational_form(polynomial(&[&[1], &[], &[-1]]), denominator.clone())?
        .scaled(&if left {
            -radius.clone()
        } else {
            radius.clone()
        })?;
    let dy = parameter
        .rational_form(polynomial(&[&[], &[2]]), denominator)?
        .scaled(radius)?;
    let x = form(center, polynomial(&[&[], &[1]]))?.add(&dx)?;
    let y = form(center, polynomial(&[&[0, 1]]))?.add(&dy)?;
    // These live queries already retain their admitted source selections.
    // The joint constructor checks both original denominators and preserves
    // their common field. Replaying each source here would duplicate that
    // construction graph before the very next incidence predicate.
    ExactCurveError::decided(
        CurveOperation2::Construction,
        PolynomialPoint2::from_queries(
            [x, y].map(|form| {
                crate::bezier_offset::PointFormSource2::new(
                    form.source().clone(),
                    form.numerator().clone(),
                    form.denominator().clone(),
                )
            }),
            policy,
        )
        .map_err(invalid)?,
    )
}
impl PolynomialCurveBranch2 {
    pub(crate) fn selected_circle_sections(
        &self,
        center: &CurvePoint2,
        radius: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<(CurveParameter2, CurvePoint2)>> {
        if !self.maps.is_empty() {
            return Err(blocked(UncertaintyReason::Unsupported));
        }
        match crate::classify::compare_reals(radius, &Real::zero(), policy) {
            Some(Ordering::Greater) => {}
            Some(_) => return Err(invalid(CurveError::InvalidBezierAlgebraicParameter)),
            None => return Err(blocked(UncertaintyReason::Ordering)),
        }
        let (local_center, local_radius) = match &self.transform {
            Some(transform) => {
                let inverse = transform.inverse().map_err(invalid)?;
                let center = CurvePointDefinition2::Similarity {
                    source: center.clone(),
                    transform: inverse,
                }
                .into_point()?;
                (
                    center,
                    (radius / transform.scale()).map_err(|cause| invalid(cause.into()))?,
                )
            }
            None => (center.clone(), radius.clone()),
        };
        let mut contacts = Vec::new();
        for left in [false, true] {
            let coefficients = coefficients(
                self.topology().geometric_equation(),
                &local_center,
                &local_radius,
                left,
            )?;
            let roots = PolynomialPoint2::selected_root_candidates(
                coefficients,
                [(-1).into(), 1.into()],
                self.source.fiber_config,
                policy,
            )?
            .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
            for root in roots {
                if left {
                    let t = form(&root, polynomial(&[&[], &[1]]))?;
                    if t.compare_to_real(&Real::from(-1))? == Ordering::Equal
                        || t.compare_to_real(&Real::one())? == Ordering::Equal
                    {
                        continue;
                    }
                }
                let local = point_on_circle(&local_center, &local_radius, &root, left, policy)?;
                let point = self.map_point(local, policy)?;
                if let Some(parameter) = self.point_parameter(&point, policy)? {
                    contacts.push((parameter, point));
                }
            }
        }
        Ok(contacts)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hypersolve::IsolatedRootInterval;
    use hypersolve::bivariate_topology::{
        BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
    };
    fn q(n: i32, d: i32) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }
    fn set(equation: BivariatePolynomial, first: [Real; 2]) -> PolynomialCurveSet2 {
        let topology = decompose_bivariate_curve(
            &equation,
            &BivariateCurveDomain {
                first,
                second: [(-1).into(), 1.into()],
            },
            BivariateTopologyConfig::default(),
        )
        .unwrap();
        PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
            .unwrap()
    }
    fn check(curve: Curve2, equation: BivariatePolynomial) {
        let center = curve.end();
        assert!(center.coordinates().is_none());
        let radius = q(1, 64);
        let cuts = curve
            .polynomial_branch()
            .unwrap()
            .selected_circle_sections(&center, &radius, &CurveContext::STRICT)
            .unwrap();
        assert_eq!(
            cuts.len(),
            1,
            "the retained branch has exactly one chord-setback cut"
        );
        for (parameter, point) in cuts {
            assert_eq!(
                form(&point, equation.clone())
                    .unwrap()
                    .compare_to_real(&Real::zero())
                    .unwrap(),
                Ordering::Equal
            );
            let dx = form(&point, polynomial(&[&[], &[1]]))
                .unwrap()
                .subtract(&form(&center, polynomial(&[&[], &[1]])).unwrap())
                .unwrap();
            let dy = form(&point, polynomial(&[&[0, 1]]))
                .unwrap()
                .subtract(&form(&center, polynomial(&[&[0, 1]])).unwrap())
                .unwrap();
            let distance = dx
                .multiply(&dx)
                .unwrap()
                .add(&dy.multiply(&dy).unwrap())
                .unwrap();
            assert_eq!(
                distance.compare_to_real(&(&radius * &radius)).unwrap(),
                Ordering::Equal
            );
            assert_eq!(
                parameter.compare(curve.parameter_domain().start()).unwrap(),
                Ordering::Greater
            );
            assert_eq!(
                parameter.compare(curve.parameter_domain().end()).unwrap(),
                Ordering::Less
            );
            assert!(
                curve
                    .point_at(&parameter)
                    .unwrap()
                    .coincides_with(&point)
                    .unwrap()
            );
        }
    }
    #[test]
    fn selected_circle_sections_cut_a_polynomial_endpoint_without_coordinates() {
        let equation = polynomial(&[&[0, 0, 1], &[-1]]);
        let source = set(equation.clone(), [0.into(), q(1, 4)]);
        check(source.curves()[0].clone(), equation);
    }
    #[test]
    fn selected_circle_sections_cut_an_algebraically_trimmed_elliptic_endpoint() {
        let equation = polynomial(&[&[-1, 0, 1], &[], &[], &[-1]]);
        let source = set(equation.clone(), [(-1).into(), 0.into()]);
        let alpha: CurveParameter2 = BezierParameter2::Algebraic(
            crate::BezierAlgebraicParameter2::try_isolate(
                crate::BezierParameterPolynomial::try_new_power_basis(vec![
                    q(3, 4),
                    0.into(),
                    0.into(),
                    1.into(),
                ])
                .unwrap(),
                crate::BezierParameterInterval::try_new((-1).into(), q(-1, 2)).unwrap(),
            )
            .unwrap(),
        )
        .into();
        let restricted = source.curves()[1]
            .restrict_source(
                crate::CurveParameterRange2::try_new(Real::from(-1).into(), alpha).unwrap(),
            )
            .unwrap();
        assert_eq!(
            form(&restricted.end(), polynomial(&[&[0, 1]]))
                .unwrap()
                .compare_to_real(&q(1, 2))
                .unwrap(),
            Ordering::Equal
        );
        check(restricted, equation);
    }

    #[test]
    fn known_fiber_branch_incidence_preserves_order_reversal_similarity_and_replay() {
        let source = CurvePointDefinition2::PolynomialFiber {
            equation: polynomial(&[&[0, 0, 0, 1], &[-1]]),
            parameter: q(1, 2).into(),
            root: IsolatedRootInterval {
                lower: 0.into(),
                upper: 1.into(),
                exact_root: None,
                distinct_root_count: 1,
            },
        }
        .into_point()
        .unwrap();
        let point = CurvePointDefinition2::RationalQueryPair {
            sources: [source.clone(), source],
            numerators: [polynomial(&[&[0, 0, 1]]), polynomial(&[&[0, 1]])],
            denominators: [polynomial(&[&[1]]), polynomial(&[&[1]])],
        }
        .into_point()
        .unwrap();
        let curves = set(polynomial(&[&[0, 0, 1], &[-1]]), [0.into(), 1.into()]);
        let transform = crate::Similarity2::try_from_real_affine(
            0.into(),
            (-2).into(),
            2.into(),
            0.into(),
            1.into(),
            3.into(),
        )
        .unwrap();
        for moved in [false, true] {
            let point = if moved {
                CurvePointDefinition2::Similarity {
                    source: point.clone(),
                    transform: transform.clone(),
                }
                .into_point()
                .unwrap()
            } else {
                point.clone()
            };
            let upper = if moved {
                curves.curves()[1].transform_similarity(&transform).unwrap()
            } else {
                curves.curves()[1].clone()
            };
            let lower = if moved {
                curves.curves()[0].transform_similarity(&transform).unwrap()
            } else {
                curves.curves()[0].clone()
            };
            assert!(
                lower
                    .polynomial_branch()
                    .unwrap()
                    .point_parameter(&point, &CurveContext::STRICT)
                    .unwrap()
                    .is_none()
            );
            for curve in [upper.clone(), upper.reversed().unwrap()] {
                let parameter = curve
                    .polynomial_branch()
                    .unwrap()
                    .point_parameter(&point, &CurveContext::STRICT)
                    .unwrap()
                    .unwrap();
                assert!(
                    curve
                        .point_at(&parameter)
                        .unwrap()
                        .coincides_with(&point)
                        .unwrap()
                );
                let restricted = curve
                    .restrict_source(
                        crate::CurveParameterRange2::try_new(parameter, Real::one().into())
                            .unwrap(),
                    )
                    .unwrap();
                assert!(
                    restricted.start().coincides_with(&point).unwrap()
                        || restricted.end().coincides_with(&point).unwrap()
                );
                let replay = point.definition().unwrap().into_point().unwrap();
                let replayed_parameter = curve
                    .polynomial_branch()
                    .unwrap()
                    .point_parameter(&replay, &CurveContext::STRICT)
                    .unwrap()
                    .unwrap();
                assert!(
                    curve
                        .point_at(&replayed_parameter)
                        .unwrap()
                        .coincides_with(&point)
                        .unwrap()
                );
            }
        }
    }
    #[test]
    fn selected_circle_source_dependencies_remain_shared() {
        use crate::bezier_offset::{PointFormSource2, selected_field_query};
        fn x_value(point: &CurvePoint2) -> crate::selected_field::Value {
            selected_field_query(
                &PointFormSource2::new(
                    point.clone(),
                    polynomial(&[&[], &[1]]),
                    polynomial(&[&[1]]),
                ),
                &CurveContext::STRICT,
            )
            .unwrap()
        }
        fn degrees(field: &crate::selected_field::Field) -> Vec<usize> {
            let mut result = Vec::new();
            let mut field = field;
            while let Some(parent) = field.parent() {
                result.push(field.defining_polynomial().unwrap().len() - 1);
                field = parent;
            }
            result
        }
        let equation = polynomial(&[&[-1, 0, 1], &[], &[], &[-1]]);
        let source = set(equation, [(-1).into(), 0.into()]);
        let alpha: CurveParameter2 = BezierParameter2::Algebraic(
            crate::BezierAlgebraicParameter2::try_isolate(
                crate::BezierParameterPolynomial::try_new_power_basis(vec![
                    q(3, 4),
                    0.into(),
                    0.into(),
                    1.into(),
                ])
                .unwrap(),
                crate::BezierParameterInterval::try_new((-1).into(), q(-1, 2)).unwrap(),
            )
            .unwrap(),
        )
        .into();
        let curve = source.curves()[1]
            .restrict_source(
                crate::CurveParameterRange2::try_new(Real::from(-1).into(), alpha).unwrap(),
            )
            .unwrap();
        let branch = curve.polynomial_branch().unwrap();
        let center = curve.end();
        let center_value = x_value(&center);
        let radius = q(1, 64);
        let coefficients =
            coefficients(branch.topology().equation(), &center, &radius, true).unwrap();
        let roots = PolynomialPoint2::selected_root_candidates(
            coefficients,
            [(-1).into(), 1.into()],
            branch.source.fiber_config,
            &CurveContext::STRICT,
        )
        .unwrap()
        .unwrap();
        assert!(!roots.is_empty());
        for root in roots {
            let root_value = x_value(&root);
            let point =
                point_on_circle(&center, &radius, &root, true, &CurveContext::STRICT).unwrap();
            let point_value = x_value(&point);
            eprintln!(
                "shared circle fields: center={:?}, root={:?}, point={:?}",
                degrees(center_value.field()),
                degrees(root_value.field()),
                degrees(point_value.field())
            );
            assert!(
                root_value
                    .field()
                    .parent()
                    .unwrap()
                    .shares_authority(center_value.field())
            );
            assert!(
                point_value.field().shares_authority(root_value.field()),
                "Combining a root with its original coefficient source must reuse its selected dependencies"
            );
        }
    }
    #[test]
    fn selected_circle_chart_seams_own_tangent_contacts_once_through_similarity() {
        use crate::Point2;
        // The horizontal line y=1/2 touches the radius-1/2 circle at its
        // upper chart seam. Both rational charts discover the same repeated
        // endpoint root; exactly one chart owns the contact.
        let source = set(polynomial(&[&[-1, 2]]), [(-1).into(), 1.into()]);
        let transform = crate::Similarity2::try_from_real_affine(
            0.into(),
            (-2).into(),
            2.into(),
            0.into(),
            1.into(),
            3.into(),
        )
        .unwrap();
        for moved in [false, true] {
            let (curve, center, radius, expected) = if moved {
                (
                    source.curves()[0].transform_similarity(&transform).unwrap(),
                    CurvePoint2::from(Point2::new(1.into(), 3.into())),
                    Real::one(),
                    CurvePoint2::from(Point2::new(0.into(), 3.into())),
                )
            } else {
                (
                    source.curves()[0].clone(),
                    CurvePoint2::from(Point2::new(0.into(), 0.into())),
                    q(1, 2),
                    CurvePoint2::from(Point2::new(0.into(), q(1, 2))),
                )
            };
            for curve in [curve.clone(), curve.reversed().unwrap()] {
                let cuts = curve
                    .polynomial_branch()
                    .unwrap()
                    .selected_circle_sections(&center, &radius, &CurveContext::STRICT)
                    .unwrap();
                assert_eq!(cuts.len(), 1);
                let (parameter, point) = &cuts[0];
                assert_eq!(
                    parameter.compare(&Real::zero().into()).unwrap(),
                    Ordering::Equal
                );
                assert!(point.coincides_with(&expected).unwrap());
                assert!(
                    curve
                        .point_at(parameter)
                        .unwrap()
                        .coincides_with(point)
                        .unwrap()
                );
            }
        }
    }
}
