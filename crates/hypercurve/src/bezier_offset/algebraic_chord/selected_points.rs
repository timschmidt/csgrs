//! Normalized chord queries retain one positive speed over their source field.

use super::*;
use crate::selected_field::{self as field, Value};

fn coordinates(point: &CurvePoint2, policy: &CurveContext) -> field::Result<[Value; 2]> {
    let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
    let coordinate = |polynomial| {
        selected_field_query(
            &PointFormSource2::new(point.clone(), polynomial, one.clone()),
            policy,
        )
    };
    let values = [
        coordinate(BivariatePolynomial::new(vec![
            vec![Real::zero()],
            vec![Real::one()],
        ]))?,
        coordinate(BivariatePolynomial::new(vec![vec![
            Real::zero(),
            Real::one(),
        ]]))?,
    ];
    field::joined(&values[0], &values[1], policy)
}

fn common_values(values: Vec<Value>, policy: &CurveContext) -> field::Result<Vec<Value>> {
    let mut iter = values.into_iter();
    let mut values = vec![iter.next().expect("source coordinates are nonempty")];
    for value in iter {
        let join = field::decided(values[0].field().joined_with(value.field(), policy)?)?;
        values = values
            .iter()
            .map(|value| join.embed(value))
            .collect::<Result<_, _>>()?;
        values.push(join.embed(&value)?);
    }
    Ok(values)
}

impl BezierAlgebraicChordParallelPoint2 {
    pub(crate) fn uses_selected_field(&self) -> bool {
        [
            self.source_endpoint(),
            self.data.source.start(),
            self.data.source.end(),
        ]
        .into_iter()
        .any(point_retains_joint_field)
    }

    pub(crate) fn selected_field_values(&self, policy: &CurveContext) -> field::Result<[Value; 2]> {
        if !self.accepts_policy(policy) {
            return Err(field::FieldError::Uncertain(UncertaintyReason::Unsupported));
        }
        self.data
            .source
            .validate_policy(policy)
            .map_err(field::FieldError::Context)?;
        if hypersolve::work_budget::exhaustion().is_some() {
            return Err(field::FieldError::Uncertain(
                UncertaintyReason::BudgetExhausted,
            ));
        }
        if let Some(values) = self.data.selected_points.get() {
            return Ok(values[usize::from(self.at_end)].clone());
        }
        let values = policy.strict_predicate_pass(|| self.build_selected_points(policy))?;
        if hypersolve::work_budget::exhaustion().is_some() {
            return Err(field::FieldError::Uncertain(
                UncertaintyReason::BudgetExhausted,
            ));
        }
        // Immutable source, direction, origin, distance and policy own this
        // derived pair. Every racing reader uses the retained winner. Failed
        // attempts never become capability evidence or poison larger retries.
        let _ = self.data.selected_points.set(values);
        Ok(self
            .data
            .selected_points
            .get()
            .expect("successful selected point import is retained")[usize::from(self.at_end)]
        .clone())
    }

    fn build_selected_points(&self, policy: &CurveContext) -> field::Result<[[Value; 2]; 2]> {
        let origins = match self.data.source_point.as_deref() {
            Some(origin) => [origin, origin],
            None => [self.data.source.start(), self.data.source.end()],
        };
        let [first, second] = origins.map(|point| coordinates(point, policy));
        let origins = [first?, second?];
        let displacement = [&self.data.translation_x, &self.data.translation_y];
        if self.data.distance.definitely_zero() {
            let translate = |origin: &[Value; 2]| -> field::Result<[Value; 2]> {
                Ok([
                    origin[0].add(&origin[0].field().constant(displacement[0].clone())?)?,
                    origin[1].add(&origin[1].field().constant(displacement[1].clone())?)?,
                ])
            };
            return Ok([translate(&origins[0])?, translate(&origins[1])?]);
        }
        let [start, end] = self
            .source_direction_endpoints(policy)
            .map(|point| coordinates(point, policy));
        let values = common_values(
            [start?, end?, origins[0].clone(), origins[1].clone()]
                .into_iter()
                .flatten()
                .collect(),
            policy,
        )?;
        let parent = values[0].field();
        let dx = values[2].subtract(&values[0])?;
        let dy = values[3].subtract(&values[1])?;
        let squared = dx.multiply(&dx)?.add(&dy.multiply(&dy)?)?;
        if field::decided(squared.sign(policy)?)? != RealSign::Positive {
            return Err(field::FieldError::Context(CurveError::Topology(
                "a selected chord displacement lost its nonzero direction".into(),
            )));
        }
        // The exact positive radicand has an upper bound u>0; u+1 is a
        // strict upper bound for sqrt(u). This original quadratic and its
        // positive open selector remain the shared speed's authority.
        let upper = field::decided(squared.enclosed(policy)?)?
            .interval()
            .upper
            .clone()
            + Real::one();
        let selected = field::decided(parent.select(
            vec![
                squared.scale(&Real::from(-1))?,
                parent.constant(Real::zero())?,
                parent.constant(Real::one())?,
            ],
            Real::zero(),
            upper,
            policy,
        )?)?;
        let speed = selected.generator()?;
        let factor = field::decided(
            selected
                .constant(self.data.distance.clone())?
                .divide(&speed, policy)?,
        )?;
        let (ux, uy) = match self.data.direction {
            BezierAlgebraicChordUnitDisplacement2::Tangent => (dx, dy),
            BezierAlgebraicChordUnitDisplacement2::LeftNormal => (dy.scale(&Real::from(-1))?, dx),
        };
        let ux = selected
            .lift(&ux)?
            .multiply(&factor)?
            .add(&selected.constant(displacement[0].clone())?)?;
        let uy = selected
            .lift(&uy)?
            .multiply(&factor)?
            .add(&selected.constant(displacement[1].clone())?)?;
        let point = |offset| -> field::Result<[Value; 2]> {
            Ok([
                selected.lift(&values[offset])?.add(&ux)?,
                selected.lift(&values[offset + 1])?.add(&uy)?,
            ])
        };
        Ok([point(4)?, point(6)?])
    }
}
