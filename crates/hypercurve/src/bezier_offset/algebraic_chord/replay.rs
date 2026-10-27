//! Strict reconstruction of a selected chord's support and parameter chart.

use super::*;

impl BezierAlgebraicChord2 {
    /// Cardinal directions are reconstructed by the ordinary chord owner.
    /// An explicitly transported oblique direction must retain its witness.
    pub(crate) fn native_replay_unit_tangent(&self) -> Option<[Real; 2]> {
        if self.start().coordinates().is_some() && self.end().coordinates().is_some() {
            // The ordinary constructor already re-establishes this direction
            // from its represented endpoints; keep the existing wire form.
            return None;
        }
        self.data.certified_unit_tangent.as_deref().cloned()
    }

    /// Checks an optional represented unit tangent against the actual chord.
    /// The archive supplies components, never a trusted certification flag.
    pub(crate) fn replay_unit_tangent(
        &self,
        tangent: [Real; 2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        self.validate_policy(policy)?;
        let [x, y] = &tangent;
        match real_sign(&(x * x + y * y - Real::one()), policy) {
            Some(RealSign::Zero) => {}
            Some(_) => return Err(CurveError::InvalidCurveParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        if let Some((known_x, known_y)) = self.certified_unit_tangent()
            && compare_reals(x, &known_x, policy) == Some(std::cmp::Ordering::Equal)
            && compare_reals(y, &known_y, policy) == Some(std::cmp::Ordering::Equal)
        {
            return Ok(Classification::Decided(self.clone()));
        }
        for (a, b, expected) in [
            (-y.clone(), x.clone(), std::cmp::Ordering::Equal),
            (x.clone(), y.clone(), std::cmp::Ordering::Less),
        ] {
            match algebraic_chord_points_linear_order(self.start(), self.end(), &a, &b, policy)? {
                Classification::Decided(order) if order == expected => {}
                Classification::Decided(_) => return Err(CurveError::InvalidCurveParameter),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        let mut chord = self.clone();
        Arc::make_mut(&mut chord.data).certified_unit_tangent = Some(Arc::new(tangent));
        Ok(Classification::Decided(chord))
    }

    /// Auxiliary contact definitions need their own replay records. A chart
    /// and two points cannot replace those relations merely by fitting a line.
    pub(crate) fn native_replay_chart(&self) -> Option<(Axis2, bool)> {
        (self.data.certified_circle_transverse_endpoints == 0
            && self.parallel_tangent_contacts().is_empty())
        .then_some((
            self.data.parameter_axis.axis,
            self.data.parameter_axis.coordinate_increases,
        ))
    }

    /// Re-establishes the serialized monotone coordinate before retaining it.
    /// The ordinary constructor owns all other endpoint and direction proofs.
    pub(crate) fn try_from_replay_definition(
        start: CurvePoint2,
        end: CurvePoint2,
        axis: Axis2,
        coordinate_increases: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let expected = if coordinate_increases {
            std::cmp::Ordering::Less
        } else {
            std::cmp::Ordering::Greater
        };
        match algebraic_chord_point_coordinate_order(&start, &end, axis, policy)? {
            Classification::Decided(order) if order == expected => {}
            Classification::Decided(_) => return Err(CurveError::InvalidCurveParameter),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        Ok(Self::try_new(start, end, policy)?.map(|mut chord| {
            Arc::make_mut(&mut chord.data).parameter_axis = BezierAlgebraicChordParameterAxis2 {
                axis,
                coordinate_increases,
            };
            chord
        }))
    }

    /// The support witness supplies an infinite line chart. Endpoint points
    /// may extend beyond that witness, but must prove support incidence and
    /// strict order in its original chart before forming the retained chord.
    pub(crate) fn replay_selected_range(
        &self,
        start: CurvePoint2,
        end: CurvePoint2,
        axis: Axis2,
        coordinate_increases: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        self.validate_policy(policy)?;
        for point in [&start, &end] {
            match self.oriented_side_by_refinement(point, policy)? {
                Classification::Decided(crate::classify::LineSide::On) => {}
                Classification::Decided(_) => return Err(CurveError::InvalidCurveParameter),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let start = self.parameter_at_certified_support_point(start, policy)?;
        let end = self.parameter_at_certified_support_point(end, policy)?;
        let reversed = match start.cmp_by_refinement(&end, policy)? {
            Classification::Decided(std::cmp::Ordering::Less) => false,
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return Err(CurveError::ZeroLengthLine);
            }
            Classification::Decided(std::cmp::Ordering::Greater) => true,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let (start, end) = if reversed { (end, start) } else { (start, end) };
        let chord = Self::from_certified_ordered_parameter_range(self, &start, &end, policy)?;
        let mut chord = if reversed { chord.reversed() } else { chord };
        let expected = if coordinate_increases {
            std::cmp::Ordering::Less
        } else {
            std::cmp::Ordering::Greater
        };
        match algebraic_chord_point_coordinate_order(chord.start(), chord.end(), axis, policy)? {
            Classification::Decided(order) if order == expected => {}
            Classification::Decided(_) => return Err(CurveError::InvalidCurveParameter),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        Arc::make_mut(&mut chord.data).parameter_axis = BezierAlgebraicChordParameterAxis2 {
            axis,
            coordinate_increases,
        };
        Ok(Classification::Decided(chord))
    }
}
