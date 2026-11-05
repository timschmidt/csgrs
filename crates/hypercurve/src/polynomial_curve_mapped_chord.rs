//! Selected line coefficients pull back through retained rational image maps.

use super::selected_chord::{coordinates, form};
use super::*;
use crate::{BezierAlgebraicChord2, CurvePointRationalForm2};
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_arithmetic::{
    bivariate_multiply, bivariate_parameter_derivative as derivative,
};
use hypersolve::ordered_field_resultant::{
    OrderedFieldResultantConfig, OrderedFieldResultantError, resultant_bivariate_ordered_field,
};

type Form = CurvePointRationalForm2;

struct Coefficients<'a> {
    origin: &'a CurvePoint2,
}
impl hypersolve::OrderedFieldPolynomialContext<Form> for Coefficients<'_> {
    type Error = ExactCurveError;
    fn constant(&mut self, value: &Real) -> ExactCurveResult<Form> {
        form(
            self.origin,
            BivariatePolynomial::new(vec![vec![value.clone()]]),
        )
    }
    fn add(&mut self, a: &Form, b: &Form) -> ExactCurveResult<Form> {
        a.add(b)
    }
    fn multiply(&mut self, a: &Form, b: &Form) -> ExactCurveResult<Form> {
        a.multiply(b)
    }
    fn scale(&mut self, value: &Form, scale: &Real) -> ExactCurveResult<Form> {
        value.scaled(scale)
    }
    fn sign(&mut self, value: &Form) -> ExactCurveResult<Ordering> {
        value.compare_to_real(&Real::zero())
    }
    fn sign_if_separated(&mut self, value: &Form) -> ExactCurveResult<Option<Ordering>> {
        self.sign(value).map(Some)
    }
    fn normalize_positive_scale(&mut self, _: &mut [Form]) {}
}

/// Linear combination of three retained real-coefficient source polynomials.
/// The coefficients retain the original endpoint fields and pole guards.
fn combination(
    polynomials: [&BivariatePolynomial; 3],
    coefficients: &[Form; 3],
) -> ExactCurveResult<Vec<Vec<Form>>> {
    let rows = polynomials
        .iter()
        .map(|p| p.coefficients.len())
        .max()
        .unwrap_or(0);
    (0..rows)
        .map(|i| {
            let columns = polynomials
                .iter()
                .filter_map(|p| p.coefficients.get(i))
                .map(Vec::len)
                .max()
                .unwrap_or(0);
            (0..columns)
                .map(|j| {
                    let term = |index: usize| {
                        coefficients[index].scaled(
                            polynomials[index]
                                .coefficients
                                .get(i)
                                .and_then(|row| row.get(j))
                                .unwrap_or(&Real::zero()),
                        )
                    };
                    term(0)?.add(&term(1)?)?.add(&term(2)?)
                })
                .collect()
        })
        .collect()
}

fn evaluate(
    point: &CurvePoint2,
    polynomials: [&BivariatePolynomial; 3],
    coefficients: &[Form; 3],
) -> ExactCurveResult<Form> {
    let term = |i: usize| form(point, polynomials[i].clone())?.multiply(&coefficients[i]);
    term(0)?.add(&term(1)?)?.add(&term(2)?)
}

impl PolynomialCurveBranch2 {
    /// Complete isolated contacts with a selected chord, through the original
    /// source branch. A zero resultant needs component correspondences and
    /// remains unsupported. The local elimination budget counts coefficient
    /// calls and matrix entries; it is not a total arithmetic or memory bound.
    pub(crate) fn mapped_selected_chord_sections(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<PolynomialLineSection2>> {
        chord.validate_policy(policy).map_err(invalid)?;
        let [ax, ay] = coordinates(chord.start())?;
        let [bx, by] = coordinates(chord.end())?;
        let dx = bx.subtract(&ax)?;
        let dy = by.subtract(&ay)?;
        // L(X,Y) = dy*X - dx*Y + dx*ay - dy*ax.
        let coefficients = [
            dy.clone(),
            dx.scaled(&Real::from(-1))?,
            dx.multiply(&ay)?.subtract(&dy.multiply(&ax)?)?,
        ];
        self.mapped_selected_line_sections(chord.start(), coefficients, true, policy)
    }

    /// A selected coordinate level through a foreign point uses the same
    /// line-pullback authority as generated chord incidence. It does not infer
    /// an inverse map from the point's construction graph.
    pub(super) fn mapped_selected_axis_sections(
        &self,
        point: &CurvePoint2,
        axis: usize,
        map: Option<&crate::RationalMap2>,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<PolynomialLineSection2>> {
        let constant = |value| form(point, BivariatePolynomial::new(vec![vec![value]]));
        let mut coefficients = [
            constant(Real::zero())?,
            constant(Real::zero())?,
            coordinates(point)?[axis].scaled(&Real::from(-1))?,
        ];
        coefficients[axis] = constant(Real::one())?;
        self.mapped_selected_line_sections_with_map(point, coefficients, false, policy, map)
    }

    fn mapped_selected_line_sections(
        &self,
        origin: &CurvePoint2,
        coefficients: [Form; 3],
        classify_tangents: bool,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<PolynomialLineSection2>> {
        self.mapped_selected_line_sections_with_map(
            origin,
            coefficients,
            classify_tangents,
            policy,
            None,
        )
    }

    fn mapped_selected_line_sections_with_map(
        &self,
        origin: &CurvePoint2,
        coefficients: [Form; 3],
        classify_tangents: bool,
        policy: &CurveContext,
        map: Option<&crate::RationalMap2>,
    ) -> ExactCurveResult<Vec<PolynomialLineSection2>> {
        let (nx, wx) = self
            .mapped_coordinate_form(0, map)
            .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
        let (ny, wy) = self
            .mapped_coordinate_form(1, map)
            .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
        let [x, y, w] = if wx == wy {
            [nx, ny, wx]
        } else {
            [
                bivariate_multiply(&nx, &wy),
                bivariate_multiply(&ny, &wx),
                bivariate_multiply(&wx, &wy),
            ]
        };
        let pulled = combination([&x, &y, &w], &coefficients)?;
        let original = self
            .topology()
            .geometric_equation()
            .coefficients
            .iter()
            .map(|row| {
                row.iter()
                    .map(|c| form(origin, BivariatePolynomial::new(vec![vec![c.clone()]])))
                    .collect()
            })
            .collect::<ExactCurveResult<Vec<Vec<Form>>>>()?;
        let report = resultant_bivariate_ordered_field(
            &original,
            &pulled,
            OrderedFieldResultantConfig::default(),
            &mut Coefficients { origin },
        )
        .map_err(|error| match error {
            OrderedFieldResultantError::Coefficient(error) => error,
            OrderedFieldResultantError::OperationLimit { .. }
            | OrderedFieldResultantError::MatrixLimit { .. }
            | OrderedFieldResultantError::AllocationFailed
            | OrderedFieldResultantError::SizeOverflow => {
                blocked(UncertaintyReason::BudgetExhausted)
            }
            OrderedFieldResultantError::Numeric(error) => invalid(error.into()),
        })?;
        let roots = PolynomialPoint2::selected_root_candidates(
            report.coefficients,
            self.topology().domain().first.clone(),
            self.source.fiber_config,
            policy,
        )?
        .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
        let source = Curve2::from_polynomial_branch(self.source_branch(policy)?);
        let fx = derivative(
            self.topology().geometric_equation(),
            CurveResultantParameter::First,
        );
        let fy = derivative(
            self.topology().geometric_equation(),
            CurveResultantParameter::Second,
        );
        let gx = [&x, &y, &w].map(|p| derivative(p, CurveResultantParameter::First));
        let gy = [&x, &y, &w].map(|p| derivative(p, CurveResultantParameter::Second));
        let mut sections = Vec::new();
        for root in roots {
            let mut parameter = coordinates(&root)?[0].parameter()?;
            let low = decided(
                parameter
                    .cmp_by_refinement(self.range.start(), policy)
                    .map_err(invalid)?,
            )?;
            let high = decided(
                parameter
                    .cmp_by_refinement(self.range.end(), policy)
                    .map_err(invalid)?,
            )?;
            if low.is_lt() || high.is_gt() {
                continue;
            }
            if low.is_eq() {
                parameter = self.range.start().clone();
            }
            if high.is_eq() {
                parameter = self.range.end().clone();
            }
            let point = source
                .point_at_with_policy(&parameter, policy)?
                .into_value();
            let mut image = if low.is_eq() {
                self.endpoints[0].clone()
            } else if high.is_eq() {
                self.endpoints[1].clone()
            } else {
                self.map_point(point.clone(), policy)?
            };
            if let Some(map) = map {
                // Elimination can propose an original map pole even when a
                // removable factor cancels. It is not a preimage.
                if form(&image, map.denominator().clone())?.compare_to_real(&Real::zero())?
                    == Ordering::Equal
                {
                    continue;
                }
                image = image.rational_image_with_policy(map, policy)?;
            }
            let [px, py] = coordinates(&image)?;
            if coefficients[0]
                .multiply(&px)?
                .add(&coefficients[1].multiply(&py)?)?
                .add(&coefficients[2])?
                .compare_to_real(&Real::zero())?
                != Ordering::Equal
            {
                continue;
            }
            // At a validated line contact, G = w*L(image). Its source
            // derivative has sign sign(w)*sign(T_image x chord_direction).
            let cross = (|| -> ExactCurveResult<Option<RealSign>> {
                if !classify_tangents {
                    return Ok(None);
                }
                let denominator = form(&point, fy.clone())?.multiply(&form(&point, w.clone())?)?;
                let weight = denominator.compare_to_real(&Real::zero())?;
                if weight == Ordering::Equal {
                    return Ok(None);
                }
                let a = evaluate(&point, [&gx[0], &gx[1], &gx[2]], &coefficients)?
                    .multiply(&form(&point, fy.clone())?)?;
                let b = evaluate(&point, [&gy[0], &gy[1], &gy[2]], &coefficients)?
                    .multiply(&form(&point, fx.clone())?)?;
                let order = a.subtract(&b)?.compare_to_real(&Real::zero())?;
                let sign = match order {
                    Ordering::Less => RealSign::Negative,
                    Ordering::Equal => RealSign::Zero,
                    Ordering::Greater => RealSign::Positive,
                };
                Ok(Some(if weight == Ordering::Less {
                    negate(sign)
                } else {
                    sign
                }))
            })();
            let cross = match cross {
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
                point: image,
                cross,
            });
        }
        Ok(sections)
    }
}
