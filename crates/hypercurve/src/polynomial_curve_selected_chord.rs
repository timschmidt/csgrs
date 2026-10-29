//! Sections by a finite line whose endpoints retain selected point fields.

use super::*;
use crate::bezier_offset::PointFormSource2;
use crate::{BezierAlgebraicChord2, CurvePointRationalForm2};
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_arithmetic::bivariate_parameter_derivative;

type Form = CurvePointRationalForm2;

enum AbscissaSection {
    Unresolved,
    Empty,
    Endpoint(usize),
}

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
    /// A finite affine chord can only meet a graph where their exact source
    /// abscissa intervals overlap. A singleton overlap identifies its actual
    /// endpoint; the selected branch and tangent still need validation.
    fn abscissa_section(
        &self,
        abscissas: [&Form; 2],
        policy: &CurveContext,
    ) -> ExactCurveResult<AbscissaSection> {
        let points = [abscissas[0].parameter()?, abscissas[1].parameter()?];
        for (boundary, outside) in [
            (self.range.start(), Ordering::Less),
            (self.range.end(), Ordering::Greater),
        ] {
            let first = decided(
                points[0]
                    .cmp_by_refinement(boundary, policy)
                    .map_err(invalid)?,
            )?;
            let second = decided(
                points[1]
                    .cmp_by_refinement(boundary, policy)
                    .map_err(invalid)?,
            )?;
            if first == outside && second == outside {
                return Ok(AbscissaSection::Empty);
            }
            if first == Ordering::Equal && second == outside {
                return Ok(AbscissaSection::Endpoint(0));
            }
            if first == outside && second == Ordering::Equal {
                return Ok(AbscissaSection::Endpoint(1));
            }
        }
        Ok(AbscissaSection::Unresolved)
    }

    /// Excludes a chord strictly across the source x-axis from this selected
    /// branch. The complete native section census, not endpoint positions,
    /// proves that the branch cannot cross the separator inside its original
    /// slab. Continuity includes the closed ends in the same half-plane.
    fn separated_by_source_axis(
        &self,
        ordinates: [&Form; 2],
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        let zero = Real::zero();
        let side = ordinates[0].compare_to_real(&zero)?;
        if side == Ordering::Equal || ordinates[1].compare_to_real(&zero)? != side {
            return Ok(false);
        }
        let sample = coordinates(&self.checked_slab_sample()?)?[1].compare_to_real(&zero)?;
        if sample == side {
            return Ok(false);
        }
        let section = self
            .topology()
            .equation()
            .coefficients
            .iter()
            .map(|row| row.first().cloned().unwrap_or_else(Real::zero))
            .collect::<Vec<_>>();
        if !polynomial_is_zero(&section, policy)? {
            let events = self.topology().events();
            let start = CurveParameter2::from(parameter(events[self.slab].parameter())?);
            let end = CurveParameter2::from(parameter(events[self.slab + 1].parameter())?);
            for root in self.section_abscissas(section)? {
                if decided(root.cmp_by_refinement(&start, policy).map_err(invalid)?)?
                    == Ordering::Greater
                    && decided(root.cmp_by_refinement(&end, policy).map_err(invalid)?)?
                        == Ordering::Less
                {
                    return Ok(false);
                }
            }
        }
        // If F(x,0) vanishes identically, y=0 is a global component. Ordered
        // simple roots cannot meet that component within this checked slab;
        // a sample on it selects that entire component. This case is not a
        // claim that the zero polynomial has no roots.
        Ok(true)
    }

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
        let abscissa_section = match self.abscissa_section([&ax, &bx], policy) {
            Ok(AbscissaSection::Empty) => return Ok(Some(Vec::new())),
            Ok(section) => section,
            Err(ExactCurveError::Blocked(blocker))
                if blocker.reason() != UncertaintyReason::BudgetExhausted =>
            {
                AbscissaSection::Unresolved
            }
            Err(error) => return Err(error),
        };
        match self.separated_by_source_axis([&ay, &by], policy) {
            Ok(true) => return Ok(Some(Vec::new())),
            Ok(false) => {}
            // Separation is an optional sufficient proof. Unavailable proof
            // leaves the existing complete section algorithm authoritative.
            Err(ExactCurveError::Blocked(blocker))
                if blocker.reason() != UncertaintyReason::BudgetExhausted => {}
            Err(error) => return Err(error),
        }
        let dx = bx.subtract(&ax)?;
        let dy = by.subtract(&ay)?;
        let fx = bivariate_parameter_derivative(
            self.topology().equation(),
            CurveResultantParameter::First,
        );
        let fy = bivariate_parameter_derivative(
            self.topology().equation(),
            CurveResultantParameter::Second,
        );
        let tangent = |point: &CurvePoint2| -> ExactCurveResult<Option<RealSign>> {
            let local = local(point)?;
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
        if let AbscissaSection::Endpoint(index) = abscissa_section {
            let point = [chord.start(), chord.end()][index];
            let boundary_section = || -> ExactCurveResult<Option<Vec<PolynomialLineSection2>>> {
                let Some(parameter) = self.point_parameter(point, policy)? else {
                    return Ok(Some(Vec::new()));
                };
                let Some(cross) = tangent(point)? else {
                    // At a singular boundary the complete component route
                    // may establish stronger evidence; retain that route.
                    return Ok(None);
                };
                Ok(Some(vec![PolynomialLineSection2 {
                    parameter,
                    point: point.clone(),
                    cross: Some(cross),
                }]))
            };
            match boundary_section() {
                Ok(Some(sections)) => return Ok(Some(sections)),
                Ok(None) => {}
                Err(ExactCurveError::Blocked(blocker))
                    if blocker.reason() != UncertaintyReason::BudgetExhausted => {}
                Err(error) => return Err(error),
            }
        }
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
            let cross = match tangent(&point) {
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
