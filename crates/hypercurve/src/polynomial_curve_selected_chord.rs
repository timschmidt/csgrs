//! Sections by a finite line whose endpoints retain selected point fields.

use super::*;
use crate::bezier_offset::PointFormSource2;
use crate::{BezierAlgebraicChord2, CurvePointRationalForm2};
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_arithmetic::bivariate_parameter_derivative;

type Form = CurvePointRationalForm2;

fn polynomial(rows: Vec<Vec<Real>>) -> BivariatePolynomial {
    BivariatePolynomial::new(rows)
}

fn form(point: &CurvePoint2, numerator: BivariatePolynomial) -> ExactCurveResult<Form> {
    point.rational_form(numerator, polynomial(vec![vec![Real::one()]]))
}

fn coordinates(point: &CurvePoint2) -> ExactCurveResult<[Form; 2]> {
    Ok([
        form(point, polynomial(vec![vec![], vec![Real::one()]]))?,
        form(point, polynomial(vec![vec![Real::zero(), Real::one()]]))?,
    ])
}

fn times_linear(values: Vec<Form>, linear: &[Form; 2]) -> ExactCurveResult<Vec<Form>> {
    let mut result: Vec<Form> = Vec::new();
    for (index, value) in values.into_iter().enumerate() {
        let constant = value.multiply(&linear[0])?;
        if let Some(existing) = result.get_mut(index) {
            *existing = existing.add(&constant)?;
        } else {
            result.push(constant);
        }
        result.push(value.multiply(&linear[1])?);
    }
    Ok(result)
}

/// Horner substitution uses the existing common-field query arithmetic. The
/// resulting coefficients retain the original selected endpoint dependencies.
fn substitute(
    equation: &BivariatePolynomial,
    origin: &CurvePoint2,
    x: &[Form; 2],
    y: &[Form; 2],
) -> ExactCurveResult<Vec<Form>> {
    let mut result = Vec::<Form>::new();
    for row in equation.coefficients.iter().rev() {
        result = times_linear(result, x)?;
        let mut substituted = Vec::<Form>::new();
        for coefficient in row.iter().rev() {
            substituted = times_linear(substituted, y)?;
            let constant = form(origin, polynomial(vec![vec![coefficient.clone()]]))?;
            if let Some(existing) = substituted.first_mut() {
                *existing = existing.add(&constant)?;
            } else {
                substituted.push(constant);
            }
        }
        for (index, coefficient) in substituted.into_iter().enumerate() {
            if let Some(existing) = result.get_mut(index) {
                *existing = existing.add(&coefficient)?;
            } else {
                result.push(coefficient);
            }
        }
    }
    Ok(result)
}

fn from_queries(forms: [Form; 2], policy: &CurveContext) -> ExactCurveResult<CurvePoint2> {
    ExactCurveError::decided(
        CurveOperation2::Intersection,
        PolynomialPoint2::from_queries(
            forms.map(|form| {
                PointFormSource2::new(
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
    /// `None` denotes a proved common selected line component; a finite list
    /// includes all contacts on the closed chord and this selected branch.
    pub(crate) fn selected_chord_sections(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<Vec<PolynomialLineSection2>>> {
        if !self.maps.is_empty() {
            return Err(blocked(UncertaintyReason::Unsupported));
        }
        chord.validate_policy(policy).map_err(invalid)?;
        let local = |point: &CurvePoint2| -> ExactCurveResult<CurvePoint2> {
            Ok(match &self.transform {
                Some(transform) => CurvePoint2::from(BezierSimilarityPoint2::new(
                    point.clone(),
                    transform.inverse().map_err(invalid)?,
                    &policy.retained_object_policy(),
                )),
                None => point.clone(),
            })
        };
        let start = local(chord.start())?;
        let end = local(chord.end())?;
        let [ax, ay] = coordinates(&start)?;
        let [bx, by] = coordinates(&end)?;
        let dx = bx.subtract(&ax)?;
        let dy = by.subtract(&ay)?;
        let coefficients = substitute(
            self.topology().equation(),
            &start,
            &[ax.clone(), dx.clone()],
            &[ay.clone(), dy.clone()],
        )?;
        let roots = PolynomialPoint2::selected_root_candidates(
            coefficients,
            [Real::zero(), Real::one()],
            self.source.fiber_config,
            policy,
        )?;
        let collinear = |point: &CurvePoint2| -> ExactCurveResult<bool> {
            let [x, y] = coordinates(point)?;
            Ok(dx
                .multiply(&y.subtract(&ay)?)?
                .subtract(&dy.multiply(&x.subtract(&ax)?)?)?
                .compare_to_real(&Real::zero())?
                == Ordering::Equal)
        };
        let Some(roots) = roots else {
            // A global line component and the complete open-slab certificate
            // prove support identity when the selected sample lies on it.
            // Otherwise a simple graph root cannot meet that component in the
            // open slab: only the two selected boundaries can be contacts.
            if collinear(&self.checked_slab_sample()?)? {
                return Ok(None);
            }
            let mut sections = Vec::new();
            for (parameter, point) in [self.range.start(), self.range.end()]
                .into_iter()
                .zip(&self.endpoints)
            {
                if collinear(&local(point)?)? {
                    sections.push(PolynomialLineSection2 {
                        parameter: parameter.clone(),
                        point: point.clone(),
                        cross: None,
                    });
                }
            }
            return Ok(Some(sections));
        };
        let fx = bivariate_parameter_derivative(
            self.topology().equation(),
            CurveResultantParameter::First,
        );
        let fy = bivariate_parameter_derivative(
            self.topology().equation(),
            CurveResultantParameter::Second,
        );
        let mut sections = Vec::new();
        for root in roots {
            let t = coordinates(&root)?[0].clone();
            let point = if t.compare_to_real(&Real::zero())? == Ordering::Equal {
                chord.start().clone()
            } else if t.compare_to_real(&Real::one())? == Ordering::Equal {
                chord.end().clone()
            } else {
                self.map_point(
                    from_queries(
                        [ax.add(&dx.multiply(&t)?)?, ay.add(&dy.multiply(&t)?)?],
                        policy,
                    )?,
                    policy,
                )?
            };
            let Some(parameter) = self.point_parameter(&point, policy)? else {
                continue;
            };
            let tangent = || -> ExactCurveResult<Option<RealSign>> {
                let local = local(&point)?;
                let fy = form(&local, fy.clone())?;
                if fy.compare_to_real(&Real::zero())? == Ordering::Equal {
                    return Ok(None);
                }
                let sign = form(&local, fx.clone())?
                    .multiply(&dx)?
                    .add(&fy.multiply(&dy)?)?
                    .multiply(&fy)?
                    .compare_to_real(&Real::zero())?;
                let sign = match sign {
                    Ordering::Less => RealSign::Negative,
                    Ordering::Equal => RealSign::Zero,
                    Ordering::Greater => RealSign::Positive,
                };
                Ok(Some(
                    if self
                        .transform
                        .as_ref()
                        .is_some_and(|t| t.reverses_orientation())
                    {
                        negate(sign)
                    } else {
                        sign
                    },
                ))
            };
            let cross = match tangent() {
                Ok(sign) => sign,
                Err(ExactCurveError::Blocked(blocker))
                    if blocker.reason() != UncertaintyReason::BudgetExhausted =>
                {
                    None
                }
                Err(error) => return Err(error),
            };
            sections.push(PolynomialLineSection2 {
                parameter,
                point,
                cross,
            });
        }
        Ok(Some(sections))
    }
}
