//! Implicit jets and smooth-event directions in the original graph chart.

use super::*;
use hypersolve::BivariatePolynomial as Polynomial;
use hypersolve::bivariate_arithmetic::{
    bivariate_add as add, bivariate_multiply as multiply,
    bivariate_parameter_derivative as derivative, bivariate_scale as scale,
    bivariate_subtract as subtract,
};

fn one() -> Polynomial {
    Polynomial::new(vec![vec![Real::one()]])
}

struct GraphDifferential {
    fx: Polynomial,
    fy: Polynomial,
}

impl GraphDifferential {
    fn new(equation: &Polynomial) -> Self {
        Self {
            fx: derivative(equation, CurveResultantParameter::First),
            fy: derivative(equation, CurveResultantParameter::Second),
        }
    }

    // delta = Fy * partial_x - Fx * partial_y; d/dx = delta / Fy.
    fn delta(&self, p: &Polynomial) -> Polynomial {
        subtract(
            &multiply(&self.fy, &derivative(p, CurveResultantParameter::First)),
            &multiply(&self.fx, &derivative(p, CurveResultantParameter::Second)),
        )
    }

    fn direction(&self, n: &Polynomial, w: &Polynomial) -> (Polynomial, Polynomial) {
        (
            subtract(&multiply(&self.delta(n), w), &multiply(n, &self.delta(w))),
            multiply(w, w),
        )
    }
}

/// At order k >= 1 the denominator is W^(k+1) Fy^(2k-1).
/// Keeping those exponents explicit avoids repeatedly squaring the whole
/// previous denominator. No factor is cancelled from the source domain.
struct CoordinateJet {
    numerator: Polynomial,
    denominator: Polynomial,
    weight: Polynomial,
    delta_weight: Polynomial,
}

impl CoordinateJet {
    fn first(n: Polynomial, w: Polynomial, d: &GraphDifferential) -> Self {
        let (numerator, denominator) = d.direction(&n, &w);
        Self {
            numerator,
            denominator: multiply(&denominator, &d.fy),
            delta_weight: d.delta(&w),
            weight: w,
        }
    }

    fn advance(
        &mut self,
        order: usize,
        d: &GraphDifferential,
        delta_fy: &Polynomial,
    ) -> ExactCurveResult<()> {
        let k = u64::try_from(order).map_err(|_| blocked(UncertaintyReason::BudgetExhausted))?;
        let weight_power = k
            .checked_add(1)
            .ok_or_else(|| blocked(UncertaintyReason::BudgetExhausted))?;
        let fy_power = k
            .checked_mul(2)
            .and_then(|n| n.checked_sub(1))
            .ok_or_else(|| blocked(UncertaintyReason::BudgetExhausted))?;
        let correction = add(
            &scale(
                multiply(&self.delta_weight, &d.fy),
                &Real::from(weight_power),
            ),
            &scale(multiply(&self.weight, delta_fy), &Real::from(fy_power)),
        );
        self.numerator = subtract(
            &multiply(&multiply(&d.delta(&self.numerator), &self.weight), &d.fy),
            &multiply(&self.numerator, &correction),
        );
        self.denominator = multiply(
            &multiply(&self.denominator, &self.weight),
            &multiply(&d.fy, &d.fy),
        );
        Ok(())
    }
}

fn vector(
    point: &CurvePoint2,
    first: (&Polynomial, &Polynomial),
    second: (&Polynomial, &Polynomial),
) -> ExactCurveResult<CurveVector2> {
    let (numerators, denominator) = if first.1 == second.1 {
        ([first.0.clone(), second.0.clone()], first.1.clone())
    } else {
        (
            [multiply(first.0, second.1), multiply(second.0, first.1)],
            multiply(first.1, second.1),
        )
    };
    Ok(CurveVector2::joint(point.rational_image(
        &crate::RationalMap2::new(numerators, denominator),
    )?))
}

impl PolynomialCurveBranch2 {
    fn differential_source_point(
        &self,
        value: &CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePoint2> {
        let compare = |boundary: &CurveParameter2| {
            decided(value.cmp_by_refinement(boundary, policy).map_err(invalid)?)
        };
        if compare(self.range.start())?.is_lt() || compare(self.range.end())?.is_gt() {
            return Err(invalid(CurveError::InvalidCurveParameter));
        }
        let arc = &self.topology().slabs()[self.slab].arcs()[self.root];
        for (event_index, root_index) in [
            (self.slab, arc.left_root()),
            (self.slab + 1, arc.right_root()),
        ] {
            let event = &self.topology().events()[event_index];
            let base = parameter(event.parameter())?;
            if compare(&base.clone().into())?.is_eq() {
                return PolynomialPoint2::from_fiber(
                    self.topology().equation().clone(),
                    base,
                    &event.roots()[root_index..=root_index],
                )?
                .pop()
                .ok_or_else(|| invalid(CurveError::InvalidBezierAlgebraicParameter));
            }
        }
        self.interior_point(value, policy)
    }

    pub(in crate::curve) fn derivatives_at(
        &self,
        value: &CurveParameter2,
        max_order: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveVector2>> {
        let mut result = Vec::new();
        result
            .try_reserve_exact(max_order)
            .map_err(|_| blocked(UncertaintyReason::BudgetExhausted))?;
        let point = self.differential_source_point(value, policy)?;
        if max_order == 0 {
            return Ok(result);
        }
        let d = GraphDifferential::new(self.topology().geometric_equation());
        if point
            .rational_form(d.fy.clone(), one())?
            .compare_to_real(&Real::zero())?
            .is_eq()
        {
            // The x chart is singular here. A finite limit of a particular
            // image may still exist, but requires a separate branch-limit proof.
            return Err(blocked(UncertaintyReason::Unsupported));
        }
        let (nx, wx) = self
            .coordinate_form(0)
            .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
        let (ny, wy) = self
            .coordinate_form(1)
            .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
        let mut x = CoordinateJet::first(nx, wx, &d);
        let mut y = CoordinateJet::first(ny, wy, &d);
        let delta_fy = d.delta(&d.fy);
        for order in 1..=max_order {
            result.push(vector(
                &point,
                (&x.numerator, &x.denominator),
                (&y.numerator, &y.denominator),
            )?);
            if order < max_order {
                x.advance(order, &d, &delta_fy)?;
                y.advance(order, &d, &delta_fy)?;
            }
        }
        Ok(result)
    }

    /// A nonzero tangent direction oriented toward increasing source parameter.
    ///
    /// At a smooth vertical event this remains finite even though the x-chart
    /// derivative is unavailable. The sign of Fy on the certified open slab
    /// fixes the one-sided orientation. Reversal retains this parameter chart,
    /// so it does not negate the returned direction. No unit length is promised.
    /// Gradient-zero events and images with zero first differential need a
    /// separate branch-limit proof and return a typed unsupported blocker.
    pub fn tangent_direction_at(&self, value: &CurveParameter2) -> ExactCurveResult<CurveVector2> {
        let policy = crate::policy::principal_context();
        let point = self.differential_source_point(value, &policy)?;
        let d = GraphDifferential::new(self.topology().geometric_equation());
        let zero = Real::zero();
        let fy_order = point
            .rational_form(d.fy.clone(), one())?
            .compare_to_real(&zero)?;
        if fy_order.is_eq()
            && point
                .rational_form(d.fx.clone(), one())?
                .compare_to_real(&zero)?
                .is_eq()
        {
            return Err(blocked(UncertaintyReason::Unsupported));
        }
        let orientation = if fy_order.is_eq() {
            let slab = &self.topology().slabs()[self.slab];
            let sample = PolynomialPoint2::from_fiber(
                self.topology().equation().clone(),
                BezierParameter2::Exact(slab.sample().clone()),
                &slab.roots()[self.root..=self.root],
            )?
            .pop()
            .ok_or_else(|| invalid(CurveError::InvalidBezierAlgebraicParameter))?;
            let order = sample
                .rational_form(d.fy.clone(), one())?
                .compare_to_real(&zero)?;
            if order.is_eq() {
                return Err(blocked(UncertaintyReason::Predicate));
            }
            order
        } else {
            fy_order
        };
        let sign = Real::from(if orientation.is_lt() { -1 } else { 1 });
        let (nx, wx) = self
            .coordinate_form(0)
            .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
        let (ny, wy) = self
            .coordinate_form(1)
            .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
        let (nx, wx) = d.direction(&nx, &wx);
        let (ny, wy) = d.direction(&ny, &wy);
        let vector = vector(&point, (&scale(nx, &sign), &wx), (&scale(ny, &sign), &wy))?;
        if vector.coordinate_sign(crate::Axis2::X)? == RealSign::Zero
            && vector.coordinate_sign(crate::Axis2::Y)? == RealSign::Zero
        {
            return Err(blocked(UncertaintyReason::Unsupported));
        }
        Ok(vector)
    }
}
