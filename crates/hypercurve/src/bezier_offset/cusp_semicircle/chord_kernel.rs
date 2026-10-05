//! Selected circle/chord systems and their exact intersections.

use super::*;

impl BezierAlgebraicCuspSemicircle2 {
    pub(in crate::bezier_offset) fn normalized_circle_frame(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspNormalizedCircleFrame2>> {
        let frame = &self.data.frame.rational_required()?.data;
        let cusp_parameter = BezierParameter2::Algebraic(frame.parameter.clone());
        let (mut center_x, mut center_y) = self
            .data
            .frame
            .point_numerators_at_parallel_distance(&self.center_parallel_distance())?;
        let mut normal_x = frame.normal_x_numerator.clone();
        let mut normal_y = frame.normal_y_numerator.clone();
        let mut denominator = frame.denominator.clone();
        let denominator_sign =
            match signed_coefficients_at_parameter(&denominator, &cusp_parameter, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        match denominator_sign {
            RealSign::Positive => {}
            RealSign::Negative => {
                let negative_one = Real::from(-1_i8);
                center_x = polynomial_scale(&center_x, &negative_one);
                center_y = polynomial_scale(&center_y, &negative_one);
                normal_x = polynomial_scale(&normal_x, &negative_one);
                normal_y = polynomial_scale(&normal_y, &negative_one);
                denominator = polynomial_scale(&denominator, &negative_one);
            }
            RealSign::Zero => {
                return Err(CurveError::Topology(
                    "selected algebraic circle center had a zero affine denominator".into(),
                ));
            }
        }
        Ok(Classification::Decided(
            BezierAlgebraicCuspNormalizedCircleFrame2 {
                center_x,
                center_y,
                normal_x,
                normal_y,
                denominator,
                cusp_parameter,
            },
        ))
    }

    pub(in crate::bezier_offset) fn axis_chord_system_for_support_point(
        &self,
        support_point: &CurvePoint2,
        direction: BezierAlgebraicChordAxisDirection2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleChordSystem2>> {
        let support = match algebraic_axis_point_coordinates(support_point, policy)? {
            Classification::Decided(support) => support,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let BezierAlgebraicCuspNormalizedCircleFrame2 {
            center_x,
            center_y,
            normal_x,
            normal_y,
            denominator: center_denominator,
            cusp_parameter,
        } = match self.normalized_circle_frame(policy)? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        let (tangent_x, tangent_y) = direction.unit_tangent();
        let normal_axis_x = -tangent_y.clone();
        let normal_axis_y = tangent_x.clone();
        let delta_x = bivariate_parameter_difference(
            &center_denominator,
            &support.x,
            &center_x,
            &support.denominator,
        );
        let delta_y = bivariate_parameter_difference(
            &center_denominator,
            &support.y,
            &center_y,
            &support.denominator,
        );
        let support_delta = bivariate_add(
            &bivariate_scale(delta_x, &normal_axis_x),
            &bivariate_scale(delta_y, &normal_axis_y),
        );
        let common_denominator = bivariate_outer_product(&center_denominator, &support.denominator);
        let radius_squared = self.radial_distance() * self.radial_distance();
        let discriminant = bivariate_subtract(
            &bivariate_scale(
                bivariate_multiply(&common_denominator, &common_denominator),
                &radius_squared,
            ),
            &bivariate_multiply(&support_delta, &support_delta),
        );

        let linear = |x: &[Real], y: &[Real], x_scale: &Real, y_scale: &Real| {
            polynomial_add(&polynomial_scale(x, x_scale), &polynomial_scale(y, y_scale))
        };
        let normal_cross_support = linear(
            &normal_x,
            &normal_y,
            &normal_axis_y,
            &(-normal_axis_x.clone()),
        );
        let normal_cross_tangent = linear(&normal_x, &normal_y, &tangent_y, &(-tangent_x.clone()));
        let normal_dot_support = linear(&normal_x, &normal_y, &normal_axis_x, &normal_axis_y);
        let normal_dot_tangent = linear(&normal_x, &normal_y, &tangent_x, &tangent_y);
        let turn_radius = self.turn_sign() * self.radial_distance();
        let selected_half_plane = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_scale(
                bivariate_multiply_first_parameter(&support_delta, &normal_cross_support),
                &turn_radius,
            ),
            radical: bivariate_scale(
                bivariate_outer_product(&normal_cross_tangent, &[Real::one()]),
                &turn_radius,
            ),
        };
        let diameter_side = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_scale(
                bivariate_multiply_first_parameter(&support_delta, &normal_dot_support),
                self.radial_distance(),
            ),
            radical: bivariate_scale(
                bivariate_outer_product(&normal_dot_tangent, &[Real::one()]),
                self.radial_distance(),
            ),
        };
        let radius_squared_denominator = bivariate_scale(
            bivariate_multiply_first_parameter(&common_denominator, &center_denominator),
            &radius_squared,
        );

        let center_x_common = bivariate_outer_product(&center_x, &support.denominator);
        let center_y_common = bivariate_outer_product(&center_y, &support.denominator);
        let point_x = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_add(
                &center_x_common,
                &bivariate_scale(support_delta.clone(), &normal_axis_x),
            ),
            radical: bivariate_scale(
                bivariate_outer_product(&[Real::one()], &[Real::one()]),
                &tangent_x,
            ),
        };
        let point_y = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_add(
                &center_y_common,
                &bivariate_scale(support_delta, &normal_axis_y),
            ),
            radical: bivariate_scale(
                bivariate_outer_product(&[Real::one()], &[Real::one()]),
                &tangent_y,
            ),
        };
        let center_tangent = linear(&center_x, &center_y, &tangent_x, &tangent_y);
        let support_tangent = linear(&support.x, &support.y, &tangent_x, &tangent_y);
        let point_minus_support_axis = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_parameter_difference(
                &center_tangent,
                &support.denominator,
                &center_denominator,
                &support_tangent,
            ),
            radical: bivariate_outer_product(&[Real::one()], &[Real::one()]),
        };
        let incidence = match &support.parameter {
            BezierParameter2::Algebraic(parameter) => {
                BivariatePolynomial::new(vec![parameter.polynomial().coefficients().to_vec()])
            }
            BezierParameter2::Exact(_) => BivariatePolynomial::new(vec![vec![Real::one()]]),
        };
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleChordSystem2 {
                incidence,
                discriminant,
                selected_half_plane,
                diameter_side,
                radius_squared_denominator,
                common_denominator,
                center_x: center_x_common,
                center_y: center_y_common,
                point_x,
                point_y,
                point_minus_support_axis,
                cusp_parameter,
                support_parameter: support.parameter,
                direction,
            },
        ))
    }

    pub(in crate::bezier_offset) fn oblique_chord_system(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleObliqueChordSystem2>> {
        chord.validate_policy(policy)?;
        let support = chord.retained_support();
        let [support_start, support_end] = if chord.retained_support_orientation_is_reversed() {
            [support.end(), support.start()]
        } else {
            [support.start(), support.end()]
        };
        let [start, end] =
            match algebraic_chord_endpoint_images(support_start, support_end, policy)? {
                Classification::Decided(endpoints) => endpoints,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let start = match positive_algebraic_point_field(&start, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match positive_algebraic_point_field(&end, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let frame = match self.normalized_circle_frame(policy)? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        let system = (|| {
            let axis = |coefficients: &[Real], axis| {
                TrivariatePolynomial::from_axis_polynomial(coefficients, axis)
            };
            let ax = axis(&start.x, 0)?;
            let ay = axis(&start.y, 0)?;
            let aw = axis(&start.denominator, 0)?;
            let bx = axis(&end.x, 1)?;
            let by = axis(&end.y, 1)?;
            let bw = axis(&end.denominator, 1)?;
            let cx = axis(&frame.center_x, 2)?;
            let cy = axis(&frame.center_y, 2)?;
            let cw = axis(&frame.denominator, 2)?;
            let nx = axis(&frame.normal_x, 2)?;
            let ny = axis(&frame.normal_y, 2)?;
            let one = TrivariatePolynomial::from_axis_polynomial(&[Real::one()], 0)?;

            let dx = TrivariatePolynomial::sum_products(&[(&bx, &aw, false), (&ax, &bw, true)])?;
            let dy = TrivariatePolynomial::sum_products(&[(&by, &aw, false), (&ay, &bw, true)])?;
            let vx = TrivariatePolynomial::sum_products(&[(&ax, &cw, false), (&cx, &aw, true)])?;
            let vy = TrivariatePolynomial::sum_products(&[(&ay, &cw, false), (&cy, &aw, true)])?;
            let d_squared =
                TrivariatePolynomial::sum_products(&[(&dx, &dx, false), (&dy, &dy, false)])?;
            let v_dot_d =
                TrivariatePolynomial::sum_products(&[(&vx, &dx, false), (&vy, &dy, false)])?;
            let aw_cw = aw.multiply(&cw)?;
            let radius_squared = self.radial_distance() * self.radial_distance();
            let v_squared =
                TrivariatePolynomial::sum_products(&[(&vx, &vx, false), (&vy, &vy, false)])?;
            let radius_term = aw_cw.multiply(&aw_cw)?.scale(&radius_squared)?;
            let v_squared_minus_radius = v_squared.subtract(&radius_term)?;
            let discriminant = TrivariatePolynomial::sum_products(&[
                (&v_dot_d, &v_dot_d, false),
                (&d_squared, &v_squared_minus_radius, true),
            ])?;

            let radial_x_rational = TrivariatePolynomial::sum_products(&[
                (&vx, &d_squared, false),
                (&dx, &v_dot_d, true),
            ])?;
            let radial_y_rational = TrivariatePolynomial::sum_products(&[
                (&vy, &d_squared, false),
                (&dy, &v_dot_d, true),
            ])?;
            let center_x = cx.multiply(&aw)?.multiply(&d_squared)?;
            let center_y = cy.multiply(&aw)?.multiply(&d_squared)?;
            let point_x = SquareRootExpression {
                rational: center_x.add(&radial_x_rational)?,
                radical: dx.clone(),
            };
            let point_y = SquareRootExpression {
                rational: center_y.add(&radial_y_rational)?,
                radical: dy.clone(),
            };
            let common_denominator = aw_cw.multiply(&d_squared)?;

            let turn_radius = self.turn_sign() * self.radial_distance();
            let selected_half_plane = SquareRootExpression {
                rational: TrivariatePolynomial::sum_products(&[
                    (&nx, &radial_y_rational, false),
                    (&ny, &radial_x_rational, true),
                ])?
                .scale(&turn_radius)?,
                radical: TrivariatePolynomial::sum_products(&[
                    (&nx, &dy, false),
                    (&ny, &dx, true),
                ])?
                .scale(&turn_radius)?,
            };
            let diameter_side = SquareRootExpression {
                rational: TrivariatePolynomial::sum_products(&[
                    (&nx, &radial_x_rational, false),
                    (&ny, &radial_y_rational, false),
                ])?
                .scale(self.radial_distance())?,
                radical: TrivariatePolynomial::sum_products(&[
                    (&nx, &dx, false),
                    (&ny, &dy, false),
                ])?
                .scale(self.radial_distance())?,
            };
            let radius_squared_denominator =
                cw.multiply(&common_denominator)?.scale(&radius_squared)?;
            let point_minus_start = SquareRootExpression {
                rational: v_dot_d.scale(&Real::from(-1_i8))?,
                radical: one,
            };
            let point_minus_end = SquareRootExpression {
                rational: TrivariatePolynomial::sum_products(&[
                    (&bw, &v_dot_d, true),
                    (&cw, &d_squared, true),
                ])?,
                radical: bw,
            };
            Some(BezierAlgebraicCuspSemicircleObliqueChordSystem2 {
                retained: BezierAlgebraicCuspSemicircleObliqueChordParameterMapSystem2 {
                    discriminant,
                    diameter_side,
                    radius_squared_denominator,
                    common_denominator,
                    center_x,
                    center_y,
                    point_x,
                    point_y,
                    first_parameter: start.parameter,
                    second_parameter: end.parameter,
                    cusp_parameter: frame.cusp_parameter,
                },
                selected_half_plane,
                point_minus_start,
                point_minus_end,
            })
        })();
        Ok(system.map_or(
            Classification::Uncertain(UncertaintyReason::Unsupported),
            Classification::Decided,
        ))
    }

    /// Builds the exact line-circle system for a retained chord whose support
    /// is stored as a procedural Euclidean normal offset of two independent
    /// endpoint fields.
    ///
    /// The inner positive radical is the source chord speed.  The outer
    /// radical is the ordinary line-circle contact discriminant, whose value
    /// is affine in that speed.  Keeping those two radicals nested preserves
    /// the authored offset sheet and avoids materializing either endpoint in
    /// a common primitive field.
    pub(in crate::bezier_offset) fn retained_offset_chord_system(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleRetainedOffsetChordSystem2>> {
        chord.validate_policy(policy)?;
        if self.data.frame.rational().is_none() {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        let support = chord.retained_support();
        let [support_start, support_end] = if chord.retained_support_orientation_is_reversed() {
            [support.end(), support.start()]
        } else {
            [support.start(), support.end()]
        };
        let (
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(offset_start)),
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(offset_end)),
        ) = (support_start, support_end)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if !offset_start.shares_carrier(offset_end)
            || offset_start.at_end == offset_end.at_end
            || offset_start.data.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        let distance = if offset_start.at_end {
            -offset_start.data.distance.clone()
        } else {
            offset_start.data.distance.clone()
        };
        let translation_x = offset_start.data.translation_x.clone();
        let translation_y = offset_start.data.translation_y.clone();
        let source = &offset_start.data.source;
        let source_support = source.retained_support();
        let [source_start, source_end] = if source.retained_support_orientation_is_reversed() {
            [source_support.end(), source_support.start()]
        } else {
            [source_support.start(), source_support.end()]
        };
        let [source_start, source_end] = if offset_start.at_end {
            [source_end, source_start]
        } else {
            [source_start, source_end]
        };
        let [start, end] = match algebraic_chord_endpoint_images(source_start, source_end, policy)?
        {
            Classification::Decided(endpoints) => endpoints,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let start = match positive_algebraic_point_field(&start, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match positive_algebraic_point_field(&end, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let frame = match self.normalized_circle_frame(policy)? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        let system = (|| {
            let axis = |coefficients: &[Real], axis| {
                TrivariatePolynomial::from_axis_polynomial(coefficients, axis)
            };
            let ax = axis(&start.x, 0)?;
            let ay = axis(&start.y, 0)?;
            let aw = axis(&start.denominator, 0)?;
            let bx = axis(&end.x, 1)?;
            let by = axis(&end.y, 1)?;
            let bw = axis(&end.denominator, 1)?;
            let cx = axis(&frame.center_x, 2)?;
            let cy = axis(&frame.center_y, 2)?;
            let cw = axis(&frame.denominator, 2)?;
            let nx = axis(&frame.normal_x, 2)?;
            let ny = axis(&frame.normal_y, 2)?;
            let one = axis(&[Real::one()], 0)?;

            let dx = TrivariatePolynomial::sum_products(&[(&bx, &aw, false), (&ax, &bw, true)])?;
            let dy = TrivariatePolynomial::sum_products(&[(&by, &aw, false), (&ay, &bw, true)])?;
            let speed_squared =
                TrivariatePolynomial::sum_products(&[(&dx, &dx, false), (&dy, &dy, false)])?;
            let translated_ax = ax.add(&aw.scale(&translation_x)?)?;
            let translated_ay = ay.add(&aw.scale(&translation_y)?)?;
            let vx = TrivariatePolynomial::sum_products(&[
                (&translated_ax, &cw, false),
                (&cx, &aw, true),
            ])?;
            let vy = TrivariatePolynomial::sum_products(&[
                (&translated_ay, &cw, false),
                (&cy, &aw, true),
            ])?;
            let line_cross =
                TrivariatePolynomial::sum_products(&[(&dx, &vy, false), (&dy, &vx, true)])?;
            let line_dot =
                TrivariatePolynomial::sum_products(&[(&dx, &vx, false), (&dy, &vy, false)])?;
            let support_denominator = aw.multiply(&bw)?;
            let center_denominator = aw.multiply(&cw)?;
            let denominator_squared_speed = center_denominator
                .multiply(&center_denominator)?
                .multiply(&speed_squared)?;
            let radius_squared = self.radial_distance() * self.radial_distance();
            let distance_squared = &distance * &distance;
            let contact_discriminant = SquareRootExpression {
                rational: denominator_squared_speed
                    .scale(&(radius_squared - distance_squared))?
                    .subtract(&line_cross.multiply(&line_cross)?)?,
                radical: center_denominator
                    .multiply(&line_cross)?
                    .scale(&(Real::from(-2_i8) * &distance))?,
            };
            let offset_denominator = center_denominator.scale(&distance)?;
            let zero = axis(&[Real::zero()], 0)?;
            let radial_x = BezierAlgebraicCuspRetainedOffsetChordNestedExpression2 {
                retained: SquareRootExpression {
                    rational: dy.multiply(&line_cross)?.scale(&Real::from(-1_i8))?,
                    radical: dy
                        .multiply(&offset_denominator)?
                        .scale(&Real::from(-1_i8))?,
                },
                candidate: SquareRootExpression {
                    rational: dx.clone(),
                    radical: zero.clone(),
                },
            };
            let radial_y = BezierAlgebraicCuspRetainedOffsetChordNestedExpression2 {
                retained: SquareRootExpression {
                    rational: dx.multiply(&line_cross)?,
                    radical: dx.multiply(&offset_denominator)?,
                },
                candidate: SquareRootExpression {
                    rational: dy.clone(),
                    radical: zero.clone(),
                },
            };
            let center_x = cx.multiply(&aw)?.multiply(&speed_squared)?;
            let center_y = cy.multiply(&aw)?.multiply(&speed_squared)?;
            let point_x = BezierAlgebraicCuspRetainedOffsetChordNestedExpression2::from_rational(
                center_x.clone(),
            )?
            .add(&radial_x)?;
            let point_y = BezierAlgebraicCuspRetainedOffsetChordNestedExpression2::from_rational(
                center_y.clone(),
            )?
            .add(&radial_y)?;
            let common_denominator = center_denominator.multiply(&speed_squared)?;

            let selected_half_plane = radial_y
                .multiply_rational(&nx)?
                .subtract(&radial_x.multiply_rational(&ny)?)?
                .scale(&(self.turn_sign() * self.radial_distance()))?;
            let diameter_side = radial_x
                .multiply_rational(&nx)?
                .add(&radial_y.multiply_rational(&ny)?)?
                .scale(self.radial_distance())?;
            let radius_squared_denominator = cw
                .multiply(&common_denominator)?
                .scale(&(self.radial_distance() * self.radial_distance()))?;

            let point_minus_start = BezierAlgebraicCuspRetainedOffsetChordNestedExpression2 {
                retained: SquareRootExpression {
                    rational: line_dot.scale(&Real::from(-1_i8))?,
                    radical: zero.clone(),
                },
                candidate: SquareRootExpression {
                    rational: one,
                    radical: zero.clone(),
                },
            };
            let point_minus_end = BezierAlgebraicCuspRetainedOffsetChordNestedExpression2 {
                retained: SquareRootExpression {
                    rational: line_dot
                        .multiply(&support_denominator)?
                        .scale(&Real::from(-1_i8))?
                        .subtract(&center_denominator.multiply(&speed_squared)?)?,
                    radical: zero.clone(),
                },
                candidate: SquareRootExpression {
                    rational: support_denominator,
                    radical: zero,
                },
            };
            let turn = self.turn_sign();
            let tangent_dot = SquareRootExpression {
                rational: line_cross.scale(&(-turn.clone()))?,
                radical: center_denominator.scale(&(-turn * distance))?,
            };
            Some(BezierAlgebraicCuspSemicircleRetainedOffsetChordSystem2 {
                retained: BezierAlgebraicCuspSemicircleRetainedOffsetChordParameterMapSystem2 {
                    speed_squared,
                    contact_discriminant,
                    diameter_side,
                    tangent_dot,
                    radius_squared_denominator,
                    common_denominator,
                    center_x,
                    center_y,
                    point_x,
                    point_y,
                    first_parameter: start.parameter,
                    second_parameter: end.parameter,
                    cusp_parameter: frame.cusp_parameter,
                },
                selected_half_plane,
                point_minus_start,
                point_minus_end,
            })
        })();
        Ok(system.map_or(
            Classification::Uncertain(UncertaintyReason::Unsupported),
            Classification::Decided,
        ))
    }

    /// Builds the exact four-selected-root line system for a pair-radial
    /// circle and a chord whose affine support is itself determined by two
    /// independent algebraic endpoint fields.
    pub(in crate::bezier_offset) fn selected_radial_chord_system(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSelectedRadialCircleChordSystem2>> {
        chord.validate_policy(policy)?;
        let support = chord.retained_support();
        let [support_start, support_end] = if chord.retained_support_orientation_is_reversed() {
            [support.end(), support.start()]
        } else {
            [support.start(), support.end()]
        };
        let [start, end] =
            match algebraic_chord_endpoint_images(support_start, support_end, policy)? {
                Classification::Decided(endpoints) => endpoints,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let start = match positive_algebraic_point_field(&start, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match positive_algebraic_point_field(&end, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let frame = match self.selected_radial_frame_system(policy)? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radial_distance = self.radial_distance().clone();
        let turn = self.turn_sign();
        let BezierSelectedRadialCircleFrameSystem2 {
            pair_map,
            canonical_pair_field: _,
            branch: pair_branch,
            discriminant,
            denominator,
            center_x,
            center_y,
            radial_x,
            radial_y,
            normal_denominator,
        } = frame;
        let system = (|| {
            let axis = |coefficients: &[Real], axis| {
                DenseTensorPolynomial::from_axis_polynomial(4, axis, coefficients)
            };
            let lift = |polynomial: &TrivariatePolynomial| {
                DenseTensorPolynomial::from_trivariate(polynomial, 4, [0, 1, 2])
            };
            let pair = |expression: &SquareRootExpression<TrivariatePolynomial>| {
                Some(SquareRootExpression {
                    rational: lift(&expression.rational)?,
                    radical: lift(&expression.radical)?,
                })
            };
            let rational_pair =
                |polynomial: DenseTensorPolynomial| SquareRootExpression::from_rational(polynomial);
            let ax = axis(&start.x, 2)?;
            let ay = axis(&start.y, 2)?;
            let aw = axis(&start.denominator, 2)?;
            let bx = axis(&end.x, 3)?;
            let by = axis(&end.y, 3)?;
            let bw = axis(&end.denominator, 3)?;
            let pair_discriminant = lift(&discriminant)?;
            let contact_denominator = lift(&denominator)?;
            let center_x = pair(&center_x)?;
            let center_y = pair(&center_y)?;
            let radial_x = pair(&radial_x)?;
            let radial_y = pair(&radial_y)?;
            let one = DenseTensorPolynomial::from_axis_polynomial(4, 0, &[Real::one()])?;

            let dx = DenseTensorPolynomial::sum_products(&[(&bx, &aw, false), (&ax, &bw, true)])?;
            let dy = DenseTensorPolynomial::sum_products(&[(&by, &aw, false), (&ay, &bw, true)])?;
            let d_squared =
                DenseTensorPolynomial::sum_products(&[(&dx, &dx, false), (&dy, &dy, false)])?;
            let start_x = ax.multiply(&contact_denominator)?;
            let start_y = ay.multiply(&contact_denominator)?;
            let vx = rational_pair(start_x)?.subtract(&center_x.multiply_rational(&aw)?)?;
            let vy = rational_pair(start_y)?.subtract(&center_y.multiply_rational(&aw)?)?;
            let v_dot_d = vx
                .multiply_rational(&dx)?
                .add(&vy.multiply_rational(&dy)?)?;
            let v_squared = vx
                .square(&pair_discriminant)?
                .add(&vy.square(&pair_discriminant)?)?;
            let aw_contact = aw.multiply(&contact_denominator)?;
            let radius_term = aw_contact
                .multiply(&aw_contact)?
                .scale(&(&radial_distance * &radial_distance))?;
            let v_squared_minus_radius = v_squared.subtract(&rational_pair(radius_term)?)?;
            let chord_discriminant = v_dot_d
                .square(&pair_discriminant)?
                .subtract(&v_squared_minus_radius.multiply_rational(&d_squared)?)?;

            let radial_base_x = vx
                .multiply_rational(&d_squared)?
                .subtract(&v_dot_d.multiply_rational(&dx)?)?;
            let radial_base_y = vy
                .multiply_rational(&d_squared)?
                .subtract(&v_dot_d.multiply_rational(&dy)?)?;
            let center_x_numerator = center_x
                .multiply_rational(&aw)?
                .multiply_rational(&d_squared)?;
            let center_y_numerator = center_y
                .multiply_rational(&aw)?
                .multiply_rational(&d_squared)?;
            let point_x = BezierSelectedRadialCircleChordNestedExpression2 {
                retained: center_x_numerator.add(&radial_base_x)?,
                candidate: rational_pair(dx.clone())?,
            };
            let point_y = BezierSelectedRadialCircleChordNestedExpression2 {
                retained: center_y_numerator.add(&radial_base_y)?,
                candidate: rational_pair(dy.clone())?,
            };
            let common_denominator = aw_contact.multiply(&d_squared)?;

            let pair_cross =
                |first_x: &SquareRootExpression<DenseTensorPolynomial>,
                 first_y: &SquareRootExpression<DenseTensorPolynomial>,
                 second_x: &SquareRootExpression<DenseTensorPolynomial>,
                 second_y: &SquareRootExpression<DenseTensorPolynomial>| {
                    first_x
                        .multiply(second_y, &pair_discriminant)?
                        .subtract(&first_y.multiply(second_x, &pair_discriminant)?)
                };
            let pair_dot =
                |first_x: &SquareRootExpression<DenseTensorPolynomial>,
                 first_y: &SquareRootExpression<DenseTensorPolynomial>,
                 second_x: &SquareRootExpression<DenseTensorPolynomial>,
                 second_y: &SquareRootExpression<DenseTensorPolynomial>| {
                    first_x
                        .multiply(second_x, &pair_discriminant)?
                        .add(&first_y.multiply(second_y, &pair_discriminant)?)
                };
            let direction_x = rational_pair(dx.clone())?;
            let direction_y = rational_pair(dy.clone())?;
            let radial_scale = &radial_distance * &normal_denominator;
            let selected_scale = turn * &radial_scale;
            let selected_half_plane = BezierSelectedRadialCircleChordNestedExpression2 {
                retained: pair_cross(&radial_x, &radial_y, &radial_base_x, &radial_base_y)?
                    .scale(&selected_scale)?,
                candidate: pair_cross(&radial_x, &radial_y, &direction_x, &direction_y)?
                    .scale(&selected_scale)?,
            };
            let diameter = BezierSelectedRadialCircleChordNestedExpression2 {
                retained: pair_dot(&radial_x, &radial_y, &radial_base_x, &radial_base_y)?
                    .scale(&radial_scale)?,
                candidate: pair_dot(&radial_x, &radial_y, &direction_x, &direction_y)?
                    .scale(&radial_scale)?,
            };
            let radius_squared_denominator =
                common_denominator.multiply(&contact_denominator)?.scale(
                    &(&radial_distance
                        * &radial_distance
                        * &normal_denominator
                        * &normal_denominator),
                )?;
            let negative_v_dot = v_dot_d.scale(&Real::from(-1_i8))?;
            let point_minus_start = BezierSelectedRadialCircleChordNestedExpression2 {
                retained: negative_v_dot.clone(),
                candidate: rational_pair(one)?,
            };
            let point_minus_end = BezierSelectedRadialCircleChordNestedExpression2 {
                retained: negative_v_dot
                    .multiply_rational(&bw)?
                    .subtract(&rational_pair(contact_denominator.multiply(&d_squared)?)?)?,
                candidate: rational_pair(bw)?,
            };
            Some(BezierSelectedRadialCircleChordSystem2 {
                retained: BezierSelectedRadialCircleChordParameterMapSystem2 {
                    pair_map,
                    pair_branch,
                    pair_discriminant,
                    chord_discriminant,
                    diameter,
                    radius_squared_denominator,
                    common_denominator,
                    center_x: center_x_numerator,
                    center_y: center_y_numerator,
                    point_x,
                    point_y,
                    first_parameter: start.parameter,
                    second_parameter: end.parameter,
                },
                selected_half_plane,
                point_minus_start,
                point_minus_end,
            })
        })();
        Ok(system.map_or(
            Classification::Uncertain(UncertaintyReason::Unsupported),
            Classification::Decided,
        ))
    }

    /// Recovers the preimage of a selected-radial carrier whose center chart
    /// retains certified similarity provenance. Parameter values are
    /// unchanged; signed radial data is divided by the positive scale and
    /// reflection parity is undone.
    pub(in crate::bezier_offset) fn selected_radial_similarity_source(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<(Self, Similarity2)>> {
        let Some(frame) = self.data.frame.selected_radial() else {
            return Ok(None);
        };
        let BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
            source,
            point: CurvePoint2(CurvePointData2::Similarity(point)),
            policy: transport_policy,
            ..
        } = frame.center_parameter.as_ref()
        else {
            return Ok(None);
        };
        if !policy.accepts_retained_policy(frame.policy)
            || !policy.accepts_retained_policy(*transport_policy)
            || !policy.accepts_retained_policy(point.data.policy)
        {
            return Err(CurveError::Topology(
                "a selected-radial similarity crossed predicate policies".into(),
            ));
        }
        let BezierAlgebraicCuspSemicircleParameter2::Mapped(source_parameter) = source else {
            return Ok(None);
        };
        let transform = point.data.transform.clone();
        let (source_frame, radial_distance, clockwise) =
            if let Some(source) = frame.similarity_source.as_ref() {
                (
                    source.frame.clone(),
                    source.radial_distance.clone(),
                    source.clockwise,
                )
            } else {
                let inverse_scale = (Real::one() / transform.scale().clone())?;
                let mut normal_denominator = &frame.normal_denominator * &inverse_scale;
                let mut radial_distance = self.radial_distance() * &inverse_scale;
                if transform.reverses_orientation() {
                    normal_denominator = -normal_denominator;
                    radial_distance = -radial_distance;
                }
                (
                    Arc::new(BezierSelectedRadialFrameData2 {
                        center_parameter: source_parameter.clone(),
                        normal_denominator,
                        similarity_source: None,
                        policy: frame.policy,
                    }),
                    radial_distance,
                    self.is_clockwise() ^ transform.reverses_orientation(),
                )
            };
        Ok(Some((
            Self {
                data: Arc::new(BezierAlgebraicCuspSemicircleData2 {
                    parallel_system_cache: Mutex::default(),
                    frame: BezierSelectedCircleFrame2::SelectedRadial(source_frame),
                    radial_distance,
                    clockwise,
                }),
            },
            transform,
        )))
    }

    pub(in crate::bezier_offset) fn recursive_line_parameter_identity(
        &self,
        line: &LineSeg2,
    ) -> Option<Arc<BezierRecursiveLineParameterIdentity2>> {
        let BezierSelectedCircleFrame2::SelectedRadial(frame) = &self.data.frame else {
            return None;
        };
        let (source_frame, source_radial_distance, source_clockwise, transform) =
            if let Some(source) = frame.similarity_source.as_ref() {
                let transform = match frame.center_parameter.as_ref() {
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                        point: CurvePoint2(CurvePointData2::Similarity(point)),
                        ..
                    } => Some(point.data.transform.clone()),
                    _ => None,
                };
                (
                    source.frame.clone(),
                    source.radial_distance.clone(),
                    source.clockwise,
                    transform,
                )
            } else {
                (
                    frame.clone(),
                    self.radial_distance().clone(),
                    self.is_clockwise(),
                    None,
                )
            };
        Some(Arc::new(BezierRecursiveLineParameterIdentity2 {
            source_frame,
            source_radial_distance,
            source_clockwise,
            line: line.clone(),
            transform,
        }))
    }

    /// Imports an authored circle-pair center directly into the recursive
    /// quadratic authority.  This is the smallest exact representation of a
    /// first-generation selected-radial frame: two selected source roots and
    /// the single positive circle-pair discriminant.
    pub(in crate::bezier_offset) fn recursive_selected_pair_frame_authority(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveCircleFrame2>>> {
        let Some(frame) = self.data.frame.selected_radial() else {
            return Ok(Classification::Decided(None));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a recursive selected-pair frame crossed predicate policies".into(),
            ));
        }
        // A similarity leaves every authored curve parameter unchanged. Peel
        // the retained transport before rebuilding the pair system, then move
        // only the two projective points through the affine map. This keeps
        // the original selected roots and positive quadratic generator by
        // identity instead of creating a mathematically equivalent tower.
        if let Some((source_circle, transform)) = self.selected_radial_similarity_source(policy)? {
            match source_circle.recursive_selected_pair_frame_authority(policy)? {
                Classification::Decided(Some(source)) => {
                    let transform_point = |projective: BezierRecursiveQuadraticProjectivePoint2| {
                        let (a, b, d, e, xoff, yoff) = transform.affine_components();
                        Some(BezierRecursiveQuadraticProjectivePoint2 {
                            x: projective
                                .x
                                .scale(a)?
                                .add(&projective.y.scale(b)?)?
                                .add(&projective.denominator.scale(xoff)?)?,
                            y: projective
                                .x
                                .scale(d)?
                                .add(&projective.y.scale(e)?)?
                                .add(&projective.denominator.scale(yoff)?)?,
                            denominator: projective.denominator,
                        })
                    };
                    let Some((center, support_center)) =
                        transform_point(source.center).zip(transform_point(source.support_center))
                    else {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    };
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "recursive-selected-pair-frame",
                        "similarity-source-authority",
                    );
                    return Ok(Classification::Decided(Some(BezierRecursiveCircleFrame2 {
                        field: source.field,
                        center,
                        support_center,
                        normal_denominator: frame.normal_denominator.clone(),
                    })));
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let system = match self.selected_radial_frame_system(policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(UncertaintyReason::Unsupported) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if !(-1..=1).contains(&system.branch) {
            return Err(CurveError::Topology(
                "a selected pair frame retained an invalid radical branch".into(),
            ));
        }
        let Some(parameters) = system.pair_map.compact_source_parameters() else {
            return Ok(Classification::Decided(None));
        };
        let sources = parameters
            .iter()
            .map(bezier_parameter_root_representation)
            .collect::<Vec<_>>();
        let dense = |polynomial: &TrivariatePolynomial| {
            let polynomial = polynomial.to_dense_polynomial()?;
            let polynomial = polynomial.remove_certified_independent_axis(
                2,
                hypersolve::PredicatePolicy::MAX_REFINEMENT_PRECISION,
            )?;
            dense_reduce_selected_tuple_relations(polynomial, &sources)
        };
        let Some(discriminant) = dense(&system.discriminant) else {
            return Ok(Classification::Decided(None));
        };
        let field = if system.canonical_pair_field {
            if let Some(field) = system.pair_map.data.recursive_field.get() {
                field.clone()
            } else {
                let Some(field) =
                    recursive_quadratic_pair_base(sources.clone(), discriminant.clone())
                else {
                    return Ok(Classification::Decided(None));
                };
                let _ = system.pair_map.data.recursive_field.set(field.clone());
                system
                    .pair_map
                    .data
                    .recursive_field
                    .get()
                    .cloned()
                    .unwrap_or(field)
            }
        } else {
            let Some(field) = recursive_quadratic_pair_base(sources.clone(), discriminant) else {
                return Ok(Classification::Decided(None));
            };
            field
        };
        let RecursiveQuadraticField::Base(base) = &field else {
            unreachable!("a selected pair frame begins in its dense base field")
        };
        let pair_value = |expression: &SquareRootExpression<TrivariatePolynomial>| {
            recursive_quadratic_pair_value(
                base,
                dense(&expression.rational)?,
                dense(&expression.radical)?,
                system.branch,
            )
        };
        let Some((mut denominator, mut center_x, mut center_y, radial_x, radial_y)) = (|| {
            Some((
                recursive_quadratic_rational_value(base, dense(&system.denominator)?)?,
                pair_value(&system.center_x)?,
                pair_value(&system.center_y)?,
                pair_value(&system.radial_x)?,
                pair_value(&system.radial_y)?,
            ))
        })() else {
            return Ok(Classification::Decided(None));
        };
        let Some((mut support_x, mut support_y)) = center_x
            .subtract(&radial_x)
            .zip(center_y.subtract(&radial_y))
        else {
            return Ok(Classification::Decided(None));
        };
        let denominator_sign = denominator.sign(&CurveContext::STRICT)?;
        match denominator_sign {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Negative) => {
                let negative = Real::from(-1_i8);
                let Some((
                    next_denominator,
                    next_center_x,
                    next_center_y,
                    next_support_x,
                    next_support_y,
                )) = (|| {
                    Some((
                        denominator.scale(&negative)?,
                        center_x.scale(&negative)?,
                        center_y.scale(&negative)?,
                        support_x.scale(&negative)?,
                        support_y.scale(&negative)?,
                    ))
                })()
                else {
                    return Ok(Classification::Decided(None));
                };
                denominator = next_denominator;
                center_x = next_center_x;
                center_y = next_center_y;
                support_x = next_support_x;
                support_y = next_support_y;
            }
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a selected pair frame retained a zero projective denominator".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        Ok(Classification::Decided(Some(BezierRecursiveCircleFrame2 {
            field: field.clone(),
            center: BezierRecursiveQuadraticProjectivePoint2 {
                x: center_x,
                y: center_y,
                denominator: denominator.clone(),
            },
            support_center: BezierRecursiveQuadraticProjectivePoint2 {
                x: support_x,
                y: support_y,
                denominator,
            },
            normal_denominator: system.normal_denominator,
        })))
    }

    /// Re-enters the recursive coefficient authority retained by a
    /// selected-radial center.  A chord-normal parent supplies the dense base;
    /// every later line contact simply returns its already-shared extension.
    pub(in crate::bezier_offset) fn recursive_selected_radial_frame_authority(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveCircleFrame2>>> {
        let Some(frame) = self.data.frame.selected_radial() else {
            return Ok(Classification::Decided(None));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a recursive selected-radial solve crossed predicate policies".into(),
            ));
        }
        let support = frame.center_parameter.semicircle_carrier();
        let center_parameter =
            BezierAlgebraicCuspSemicircleParameter2::Mapped(frame.center_parameter.clone());
        let center = match center_parameter.coincident_point_evidence(support, policy)? {
            Classification::Decided(Some(CurvePoint2(CurvePointData2::AlgebraicCuspChord(
                center,
            )))) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-selected-radial-frame",
                    "correlated-chord-center",
                );
                center
            }
            Classification::Decided(Some(_point)) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-selected-radial-frame",
                    match _point {
                        CurvePoint2(CurvePointData2::Exact(_)) => "exact-center",
                        CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic-center",
                        CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "chord-pair-center",
                        CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => {
                            "derived-chord-center"
                        }
                        CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => {
                            "chord-parallel-center"
                        }
                        CurvePoint2(CurvePointData2::AnalyticParallel(_)) => {
                            "analytic-parallel-center"
                        }
                        CurvePoint2(
                            CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_),
                        ) => "similarity-center",
                        CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => {
                            unreachable!("the correlated chord center was handled above")
                        }
                    },
                );
                return Ok(Classification::Decided(None));
            }
            Classification::Decided(None) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-selected-radial-frame",
                    "missing-center-evidence",
                );
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (center_map, center_contact) = center.map_contact();
        center_map.validate_policy(policy)?;
        if center_map.data.semicircle != *support {
            return Err(CurveError::Topology(
                "a recursive selected-radial center lost its support-circle map".into(),
            ));
        }

        let imported = match center_map.recursive_contact_frame(center_contact, policy)? {
            Classification::Decided(Some(imported)) => imported,
            Classification::Decided(None) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-selected-radial-frame",
                    "unsupported-parent",
                );
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-selected-radial-frame",
            "imported-contact-parent",
        );
        Ok(Classification::Decided(Some(BezierRecursiveCircleFrame2 {
            field: imported.field,
            center: imported.point,
            support_center: imported.center,
            normal_denominator: frame.normal_denominator.clone(),
        })))
    }

    /// Continues a retained dense or recursive selected-radial center through
    /// an exact affine-line contact without projecting the new parameter to a
    /// global univariate norm.  The analytic quadratic solve appends one
    /// positive square root to the already-selected field and publishes the
    /// ordinary chord-map predicates over that shared tower.
    pub(in crate::bezier_offset) fn recursive_selected_radial_quadratic_line_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        line: &LineSeg2,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleChordIntersections2>>> {
        macro_rules! unsupported {
            ($path:literal) => {{
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-selected-radial-line-blocker",
                    $path,
                );
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }};
        }
        macro_rules! uncertain {
            ($path:literal, $reason:expr) => {{
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-selected-radial-line-uncertain",
                    $path,
                );
                return Ok(Classification::Uncertain($reason));
            }};
        }
        // A directly pair-authored center already owns the compact source
        // roots and pair discriminant needed by this line solve. Import that
        // authority before asking the descendant replay path to recover a
        // chord-map contact; a `Pair` parameter has no chord map, and the
        // generic evidence bridge would only wrap the same pair point as a
        // derived coordinate before declining the recursive solve.
        let authority = match self.recursive_selected_pair_frame_authority(policy)? {
            Classification::Decided(Some(authority)) => authority,
            Classification::Decided(None) => {
                match self.recursive_selected_radial_frame_authority(policy)? {
                    Classification::Decided(Some(authority)) => authority,
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Uncertain(reason) => {
                        uncertain!("frame-authority", reason);
                    }
                }
            }
            Classification::Uncertain(reason) => {
                uncertain!("frame-authority", reason);
            }
        };
        let BezierRecursiveCircleFrame2 {
            field: parent_field,
            center,
            support_center,
            normal_denominator,
        } = authority;

        let Some((anchor_x, anchor_y, anchor_denominator)) =
            center.difference_numerators(&support_center)
        else {
            unsupported!("anchor-difference");
        };
        let (delta_x, delta_y) = line.delta();
        let Some((
            a,
            b,
            c,
            discriminant,
            line_start_radial_x,
            line_start_radial_y,
            line_center_cross,
        )) = (|| {
            let start_x = parent_field.constant(line.start().x().clone())?;
            let start_y = parent_field.constant(line.start().y().clone())?;
            let vx = center.denominator.multiply(&start_x)?.subtract(&center.x)?;
            let vy = center.denominator.multiply(&start_y)?.subtract(&center.y)?;
            let direction_squared = &delta_x * &delta_x + &delta_y * &delta_y;
            let inverse_direction_squared = (Real::one() / direction_squared.clone()).ok()?;
            let denominator_squared = center.denominator.square()?;
            // Divide the line/circle quadratic by |delta|^2 before adjoining
            // its radical. A simultaneous similarity scales the unnormalized
            // coefficients by s^2 and the discriminant by s^4 even though the
            // line parameter is unchanged. This positive normalization makes
            // the authored parameter field similarity-invariant and prevents
            // exact transformed contacts from becoming unrelated radicals.
            let a = denominator_squared.clone();
            let b = vx
                .scale(&delta_x)?
                .add(&vy.scale(&delta_y)?)?
                .multiply(&center.denominator)?
                .scale(&(Real::from(2_i8) * &inverse_direction_squared))?;
            let radius_squared = self.radial_distance() * self.radial_distance();
            let c = vx
                .square()?
                .add(&vy.square()?)?
                .subtract(&denominator_squared.scale(&radius_squared)?)?
                .scale(&inverse_direction_squared)?;
            // The expanded `b^2 - 4ac` form asks the scalar layer to
            // rediscover the exact Lagrange cancellation between
            // `(v·d)^2` and `|v|^2|d|^2`. Preserve that identity directly:
            //
            //   discriminant / |d|^4
            //     = 4D^2(D^2 r^2 / |d|^2 - (cross(v,d) / |d|^2)^2).
            //
            // This is both smaller and keeps recursively correlated radical
            // coefficients from becoming opaque scalar zero expressions.
            let cross = vx.scale(&delta_y)?.subtract(&vy.scale(&delta_x)?)?;
            let normalized_cross = cross.scale(&inverse_direction_squared)?;
            let radial_term =
                denominator_squared.scale(&(radius_squared * &inverse_direction_squared))?;
            let discriminant = denominator_squared
                .multiply(&radial_term.subtract(&normalized_cross.square()?)?)?
                .scale(&Real::from(4_i8))?;
            Some((a, b, c, discriminant, vx, vy, cross))
        })()
        else {
            unsupported!("quadratic-coefficients");
        };
        match a.sign(&CurveContext::STRICT)? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "an exact recursive line retained a nonpositive quadratic coefficient".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                uncertain!("quadratic-leading-sign", reason);
            }
        }
        let discriminant_sign = match discriminant.sign(&CurveContext::STRICT)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                uncertain!("quadratic-discriminant-sign", reason);
            }
        };
        if discriminant_sign == RealSign::Negative {
            return Ok(Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
            )));
        }

        let branches: &[i8] = if discriminant_sign == RealSign::Zero {
            &[0]
        } else {
            &[-1, 1]
        };
        let line_identity = self.recursive_line_parameter_identity(line);
        let extension = if discriminant_sign == RealSign::Positive {
            Some(
                parent_field
                    .extension(discriminant.clone())
                    .ok_or_else(|| {
                        CurveError::Topology(
                            "a recursive line discriminant could not extend its parent field"
                                .into(),
                        )
                    })?,
            )
        } else {
            None
        };
        let radial_scale = self.radial_distance() * &normal_denominator;
        let selected_scale = self.turn_sign() * &radial_scale;
        let selected_linear = (|| {
            let constant = anchor_x
                .multiply(&line_start_radial_y)?
                .subtract(&anchor_y.multiply(&line_start_radial_x)?)?
                .scale(&selected_scale)?;
            let slope = anchor_x
                .scale(&delta_y)?
                .subtract(&anchor_y.scale(&delta_x)?)?
                .multiply(&center.denominator)?
                .scale(&selected_scale)?;
            Some((constant, slope))
        })();
        let diameter_endpoint_line_sides = (|| {
            // The two full-circle contacts lie on opposite authored halves
            // exactly when the target line separates the diameter endpoints.
            // Evaluate those two endpoint sides in the parent field; this is
            // the unsquared geometric equivalent of the product of the two
            // selected-half values and does not adjoin the contact radical.
            let center_side = line_start_radial_x
                .scale(&delta_y)?
                .subtract(&line_start_radial_y.scale(&delta_x)?)?;
            let anchor_side = anchor_y
                .scale(&delta_x)?
                .subtract(&anchor_x.scale(&delta_y)?)?;
            let normal_squared = &normal_denominator * &normal_denominator;
            let center_term = center_side
                .multiply(&anchor_denominator)?
                .scale(&normal_squared)?;
            let radial_term = anchor_side
                .multiply(&center.denominator)?
                .scale(&radial_scale)?;
            Some([
                center_term.add(&radial_term)?,
                center_term.subtract(&radial_term)?,
            ])
        })();
        let turn_sign = if self.is_clockwise() {
            RealSign::Negative
        } else {
            RealSign::Positive
        };
        let tangent_dot_sign = line_center_cross
            .bounded_interval_sign(0..=512)
            .map(|sign| product_sign(sign, turn_sign));
        let selected_root_signs = if let Some((constant, slope)) = selected_linear.as_ref() {
            recursive_quadratic_affine_predicate_root_signs(
                &a,
                &b,
                &c,
                discriminant_sign,
                constant,
                slope,
                diameter_endpoint_line_sides.as_ref(),
            )?
        } else {
            None
        };
        let mut retained_contacts = Vec::with_capacity(branches.len());
        let mut contacts = Vec::with_capacity(branches.len());
        for &quadratic_branch in branches {
            let field = extension.as_ref().unwrap_or(&parent_field);
            let Some((parameter_numerator, parameter_denominator)) = (|| {
                let retained = b.scale(&Real::from(-1_i8))?;
                let denominator = a.scale(&Real::from(2_i8))?;
                if let Some(extension) = extension.as_ref() {
                    let radical = parent_field.constant(Real::from(quadratic_branch))?;
                    Some((
                        extension.element(retained, radical)?,
                        extension.lift(&denominator)?,
                    ))
                } else {
                    Some((retained, denominator))
                }
            })() else {
                unsupported!("contact-parameter");
            };
            let Some((point, center, anchor_x, anchor_y)) = (|| {
                let start_x = field.constant(line.start().x().clone())?;
                let start_y = field.constant(line.start().y().clone())?;
                let point = BezierRecursiveQuadraticProjectivePoint2 {
                    x: parameter_denominator
                        .multiply(&start_x)?
                        .add(&parameter_numerator.scale(&delta_x)?)?,
                    y: parameter_denominator
                        .multiply(&start_y)?
                        .add(&parameter_numerator.scale(&delta_y)?)?,
                    denominator: parameter_denominator.clone(),
                };
                Some((
                    point,
                    BezierRecursiveQuadraticProjectivePoint2 {
                        x: field.lift(&center.x)?,
                        y: field.lift(&center.y)?,
                        denominator: field.lift(&center.denominator)?,
                    },
                    field.lift(&anchor_x)?,
                    field.lift(&anchor_y)?,
                ))
            })() else {
                unsupported!("contact-point");
            };
            let Some((radial_x, radial_y, radial_denominator)) =
                point.difference_numerators(&center)
            else {
                unsupported!("contact-radial");
            };
            let Some((
                selected_half_plane,
                diameter,
                radius_squared_denominator,
                tangent_cross,
                angular_tangent,
            )) = (|| {
                // Retain the projective oriented area itself as the topology
                // predicate. Its affine-in-parameter expansion remains useful
                // for the paired-root product below, but evaluating that
                // expansion at an endpoint can obscure the exact determinant
                // cancellation in unrelated scalar coefficients.
                let selected_half_plane = anchor_x
                    .multiply(&radial_y)?
                    .subtract(&anchor_y.multiply(&radial_x)?)?
                    .scale(&selected_scale)?;
                let dot = anchor_x
                    .multiply(&radial_x)?
                    .add(&anchor_y.multiply(&radial_y)?)?;
                let diameter = dot.scale(&radial_scale)?;
                // `diameter` is the true start-radial dot product multiplied
                // by n^2 times both projective vector denominators. Retain
                // r^2 under exactly that same positive scale so angular
                // parameter comparisons need no second quadratic solve.
                let radius_squared_scale = self.radial_distance()
                    * self.radial_distance()
                    * &normal_denominator
                    * &normal_denominator;
                let radius_squared_denominator = field
                    .lift(&anchor_denominator)?
                    .multiply(&radial_denominator)?
                    .scale(&radius_squared_scale)?;
                let tangent_cross = radial_x
                    .scale(&delta_x)?
                    .add(&radial_y.scale(&delta_y)?)?
                    .scale(&(-self.turn_sign()))?;
                let angular_tangent = radial_x
                    .scale(&delta_y)?
                    .subtract(&radial_y.scale(&delta_x)?)?;
                Some((
                    selected_half_plane,
                    diameter,
                    radius_squared_denominator,
                    tangent_cross,
                    angular_tangent,
                ))
            })()
            else {
                unsupported!("contact-predicates");
            };
            let selected_sign = if let Some(signs) = selected_root_signs {
                Classification::Decided(signs[usize::from(quadratic_branch > 0)])
            } else {
                selected_half_plane.sign(&CurveContext::STRICT)?
            };
            let cusp_location = match selected_sign {
                Classification::Decided(RealSign::Negative) => continue,
                Classification::Decided(RealSign::Positive) => {
                    BezierAlgebraicCuspSemicircleContactLocation2::Interior
                }
                Classification::Decided(RealSign::Zero) => {
                    match diameter.sign(&CurveContext::STRICT)? {
                        Classification::Decided(RealSign::Positive) => {
                            BezierAlgebraicCuspSemicircleContactLocation2::Start
                        }
                        Classification::Decided(RealSign::Negative) => {
                            BezierAlgebraicCuspSemicircleContactLocation2::End
                        }
                        Classification::Decided(RealSign::Zero) => {
                            return Err(CurveError::Topology(
                                "a nonzero recursive circle contact had zero local diameter".into(),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            uncertain!("diameter-sign", reason);
                        }
                    }
                }
                Classification::Uncertain(reason) => {
                    uncertain!("selected-half-sign", reason);
                }
            };
            let chord_location = if clip_to_finite_chord {
                let lower = match parameter_numerator.sign(&CurveContext::STRICT)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        uncertain!("finite-lower-sign", reason);
                    }
                };
                if lower == RealSign::Negative {
                    continue;
                }
                if lower == RealSign::Zero {
                    BezierAlgebraicCuspSemicircleContactLocation2::Start
                } else {
                    let upper = parameter_numerator
                        .subtract(&parameter_denominator)
                        .ok_or_else(|| {
                            CurveError::Topology(
                                "a recursive line upper-bound predicate exceeded its field budget"
                                    .into(),
                            )
                        })?;
                    match upper.sign(&CurveContext::STRICT)? {
                        Classification::Decided(RealSign::Positive) => continue,
                        Classification::Decided(RealSign::Zero) => {
                            BezierAlgebraicCuspSemicircleContactLocation2::End
                        }
                        Classification::Decided(RealSign::Negative) => {
                            BezierAlgebraicCuspSemicircleContactLocation2::Interior
                        }
                        Classification::Uncertain(reason) => {
                            uncertain!("finite-upper-sign", reason);
                        }
                    }
                }
            } else {
                BezierAlgebraicCuspSemicircleContactLocation2::Interior
            };
            // At `(-b +/- sqrt(discriminant)) / 2a`, the radial/direction
            // dot product has exactly the quadratic branch sign.  Every
            // omitted factor is certified positive, so circle orientation is
            // the only sign adjustment; do not re-evaluate the expanded
            // radical expression in the descendant field.
            let tangent_cross_sign =
                recursive_circle_contact_tangent_cross_sign(quadratic_branch, turn_sign);
            let branch = quadratic_branch;
            let certified_bounds =
                clip_to_finite_chord.then(|| chord_location.certified_unit_bounds());
            let parameter = match BezierRecursiveProjectiveParameter2::new_with_certified_bounds(
                RecursiveQuadraticProjectiveScalar {
                    numerator: parameter_numerator.clone(),
                    denominator: parameter_denominator.clone(),
                },
                certified_bounds,
                policy,
            )? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    uncertain!("contact-parameter", reason);
                }
            };
            let parameter = match line_identity.as_ref() {
                Some(identity) => parameter.with_line_identity(identity.clone(), branch),
                None => parameter,
            };
            retained_contacts.push(BezierRecursiveQuadraticLineContactSystem2 {
                branch,
                parameter,
                tangent_dot_sign,
                point,
                diameter,
                radius_squared_denominator,
                tangent_cross,
                angular_tangent,
            });
            contacts.push(BezierAlgebraicCuspSemicircleChordContact2 {
                branch,
                projective_parameter: None,
                cusp_location,
                chord_location,
                tangent_cross_sign,
            });
        }
        if contacts.is_empty() {
            return Ok(Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
            )));
        }
        if contacts.len() > 2
            || retained_contacts
                .iter()
                .map(|contact| contact.branch)
                .collect::<std::collections::BTreeSet<_>>()
                .len()
                != retained_contacts.len()
        {
            return Err(CurveError::Topology(
                "a recursive line/circle solve retained duplicate contact branches".into(),
            ));
        }
        let parameter_map = BezierAlgebraicCuspSemicircleChordParameterMap2::new(
            self.clone(),
            chord.clone(),
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RecursiveQuadraticLine(
                BezierRecursiveQuadraticLineParameterMapSystem2 {
                    center,
                    contacts: retained_contacts,
                },
            ),
            clip_to_finite_chord,
            policy,
        );
        Ok(Classification::Decided(Some(
            BezierAlgebraicCuspSemicircleChordIntersections2::Contacts {
                contacts,
                parameter_map,
            },
        )))
    }

    /// Imports a retained chord and this circle's authored center/radial point
    /// into an existing recursive endpoint field. This is the cold shared-
    /// tower bridge for a later bevel or offset endpoint meeting an earlier
    /// selected circle; no standalone Cartesian primitive element is needed.
    pub(in crate::bezier_offset) fn recursive_exact_parallel_normal_frame_chord_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        certified_endpoint_incidence: Option<BezierCertifiedFiniteChordEndpointIncidence2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleChordIntersections2>>> {
        let Some(frame) = self.data.frame.parallel_normal() else {
            return Ok(Classification::Decided(None));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "an exact parallel-normal circle frame crossed predicate policies".into(),
            ));
        }
        let Some(parameter) = frame.center_parameter.scalar() else {
            return Ok(Classification::Decided(None));
        };
        let center = match frame
            .center_support
            .point_at_with_policy(parameter, policy)?
        {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let tangent = match frame.center_support.source_tangent_at(parameter, policy)? {
            Classification::Decided(tangent) => tangent,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let speed_squared = Real::dot2_refs([&tangent.0, &tangent.1], [&tangent.0, &tangent.1]);
        match real_sign(&speed_squared, &CurveContext::STRICT) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "an exact parallel-normal circle frame had negative squared speed".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let speed = speed_squared.sqrt()?;
        let points =
            match recursive_projective_evidence_points(&[chord.start(), chord.end()], policy)? {
                Classification::Decided(Some(points)) => points,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let [start, end]: [BezierRecursiveQuadraticProjectivePoint2; 2] = points
            .try_into()
            .expect("an exact-frame chord bridge retains two authored endpoints");
        let start = match positive_recursive_projective_point(start)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match positive_recursive_projective_point(end)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let field = start.denominator.field();
        let Some(authority) = (|| {
            let denominator = field.constant(Real::one())?;
            Some(BezierRecursiveCircleFrame2 {
                field: field.clone(),
                center: BezierRecursiveQuadraticProjectivePoint2 {
                    x: field.constant(center.x().clone())?,
                    y: field.constant(center.y().clone())?,
                    denominator: denominator.clone(),
                },
                // The unnormalized left normal is `(-ty, tx)`. Therefore
                // `C - support_center` is that vector and `speed` is its
                // positive normalization denominator. Keeping this
                // projective frame avoids introducing two exact quotients.
                support_center: BezierRecursiveQuadraticProjectivePoint2 {
                    x: field.constant(center.x() + &tangent.1)?,
                    y: field.constant(center.y() - &tangent.0)?,
                    denominator,
                },
                normal_denominator: speed,
            })
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        self.recursive_projective_chord_intersections(
            chord,
            authority,
            start,
            end,
            clip_to_finite_chord,
            certified_endpoint_incidence,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn recursive_projective_retained_chord_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        certified_endpoint_incidence: Option<BezierCertifiedFiniteChordEndpointIncidence2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleChordIntersections2>>> {
        match self.recursive_exact_parallel_normal_frame_chord_intersections(
            chord,
            clip_to_finite_chord,
            certified_endpoint_incidence,
            policy,
        )? {
            Classification::Decided(Some(intersections)) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-chord-kernel",
                    "recursive-exact-parallel-normal-frame",
                );
                return Ok(Classification::Decided(Some(intersections)));
            }
            Classification::Decided(None) | Classification::Uncertain(_) => {}
        }
        let center = match self.center_point_evidence(policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radial_point = match self.start_point_evidence(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let evidence = [chord.start(), chord.end(), &center, &radial_point];
        let points = match recursive_projective_evidence_points(&evidence, policy)? {
            Classification::Decided(Some(points)) => points,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let [start, end, center, radial_point]: [BezierRecursiveQuadraticProjectivePoint2; 4] =
            points
                .try_into()
                .expect("a recursive circle/chord bridge retains four authored points");
        let mut points = Vec::with_capacity(4);
        for point in [start, end, center, radial_point] {
            match positive_recursive_projective_point(point)? {
                Classification::Decided(point) => points.push(point),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let [start, end, center, radial_point]: [BezierRecursiveQuadraticProjectivePoint2; 4] =
            points
                .try_into()
                .expect("a recursive circle/chord bridge preserves four positive points");
        let field = center.denominator.field();
        let Some((support_x, support_y, support_denominator)) = (|| {
            let denominator = center.denominator.multiply(&radial_point.denominator)?;
            let x = center
                .x
                .multiply(&radial_point.denominator)?
                .scale(&Real::from(2_i8))?
                .subtract(&radial_point.x.multiply(&center.denominator)?)?;
            let y = center
                .y
                .multiply(&radial_point.denominator)?
                .scale(&Real::from(2_i8))?
                .subtract(&radial_point.y.multiply(&center.denominator)?)?;
            Some((x, y, denominator))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let authority = BezierRecursiveCircleFrame2 {
            field,
            center,
            support_center: BezierRecursiveQuadraticProjectivePoint2 {
                x: support_x,
                y: support_y,
                denominator: support_denominator,
            },
            normal_denominator: self.radial_distance().clone(),
        };
        self.recursive_projective_chord_intersections(
            chord,
            authority,
            start,
            end,
            clip_to_finite_chord,
            certified_endpoint_incidence,
            policy,
        )
    }

    /// Solves one finite projective chord against any selected circle in
    /// their shared recursive field. Ordinary exact lines, procedural offset
    /// supports, and mixed bevel endpoints all reduce to this quadratic
    /// authority; only construction of the projective frame differs.
    pub(in crate::bezier_offset) fn recursive_projective_chord_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        authority: BezierRecursiveCircleFrame2,
        start: BezierRecursiveQuadraticProjectivePoint2,
        end: BezierRecursiveQuadraticProjectivePoint2,
        clip_to_finite_chord: bool,
        certified_endpoint_incidence: Option<BezierCertifiedFiniteChordEndpointIncidence2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleChordIntersections2>>> {
        let parent_field = authority.field.clone();
        let Some(start) = start.lifted_to(&parent_field) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(end) = end.lifted_to(&parent_field) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(center) = authority.center.lifted_to(&parent_field) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(support_center) = authority.support_center.lifted_to(&parent_field) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if clip_to_finite_chord {
            // Circle incidence along an affine chord is a quadratic Bezier
            // scalar. If its three Bernstein coefficients have one strict
            // sign, the complete finite chord is disjoint even when its
            // supporting line crosses the circle. Sign these coefficients in
            // the parent tower before adjoining the line-contact radical.
            let finite_disjoint = (|| {
                let (start_x, start_y, start_denominator) = start.difference_numerators(&center)?;
                let (end_x, end_y, end_denominator) = end.difference_numerators(&center)?;
                let radius_squared = self.radial_distance() * self.radial_distance();
                let incidence =
                    |x: &RecursiveQuadraticValue,
                     y: &RecursiveQuadraticValue,
                     denominator: &RecursiveQuadraticValue| {
                        x.square()?
                            .add(&y.square()?)?
                            .subtract(&denominator.square()?.scale(&radius_squared)?)
                    };
                let start_incidence = incidence(&start_x, &start_y, &start_denominator)?;
                let end_incidence = incidence(&end_x, &end_y, &end_denominator)?;
                let middle_incidence = start_x
                    .multiply(&end_x)?
                    .add(&start_y.multiply(&end_y)?)?
                    .subtract(
                    &start_denominator
                        .multiply(&end_denominator)?
                        .scale(&radius_squared)?,
                )?;
                let signs = [start_incidence, middle_incidence, end_incidence]
                    .map(|value| value.sign(&CurveContext::STRICT));
                let [
                    Ok(Classification::Decided(first)),
                    Ok(Classification::Decided(middle)),
                    Ok(Classification::Decided(last)),
                ] = signs
                else {
                    return Some(false);
                };
                Some(
                    first == middle
                        && middle == last
                        && matches!(first, RealSign::Negative | RealSign::Positive),
                )
            })()
            .unwrap_or(false);
            if finite_disjoint {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-chord-kernel",
                    "recursive-finite-incidence-disjoint",
                );
                return Ok(Classification::Decided(Some(
                    BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
                )));
            }
        }
        let Some((anchor_x, anchor_y, anchor_denominator)) =
            center.difference_numerators(&support_center)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some((
            line_denominator,
            line_start_x,
            line_start_y,
            delta_x,
            delta_y,
            vx,
            vy,
            direction_x,
            direction_y,
            common_denominator,
            a,
            b,
            c,
            discriminant,
            line_center_cross,
        )) = (|| {
            let line_denominator = start.denominator.multiply(&end.denominator)?;
            let line_start_x = start.x.multiply(&end.denominator)?;
            let line_start_y = start.y.multiply(&end.denominator)?;
            let delta_x = end
                .x
                .multiply(&start.denominator)?
                .subtract(&line_start_x)?;
            let delta_y = end
                .y
                .multiply(&start.denominator)?
                .subtract(&line_start_y)?;
            let vx = line_start_x
                .multiply(&center.denominator)?
                .subtract(&center.x.multiply(&line_denominator)?)?;
            let vy = line_start_y
                .multiply(&center.denominator)?
                .subtract(&center.y.multiply(&line_denominator)?)?;
            let direction_x = delta_x.multiply(&center.denominator)?;
            let direction_y = delta_y.multiply(&center.denominator)?;
            let common_denominator = line_denominator.multiply(&center.denominator)?;
            let a = direction_x.square()?.add(&direction_y.square()?)?;
            let b = vx
                .multiply(&direction_x)?
                .add(&vy.multiply(&direction_y)?)?
                .scale(&Real::from(2_i8))?;
            let radius_squared = self.radial_distance() * self.radial_distance();
            let c = vx
                .square()?
                .add(&vy.square()?)?
                .subtract(&common_denominator.square()?.scale(&radius_squared)?)?;
            // Preserve the Lagrange identity directly instead of asking the
            // scalar layer to cancel the expanded `b^2 - 4ac` form across a
            // recursive quadratic tower:
            //
            //   discriminant = 4 (|d|^2 D^2 r^2 - cross(v, d)^2).
            //
            // Besides producing fewer field terms, this exposes exact
            // tangency as structural zero under every predicate policy.
            let cross = vx
                .multiply(&direction_y)?
                .subtract(&vy.multiply(&direction_x)?)?;
            let radial_term = a
                .multiply(&common_denominator.square()?)?
                .scale(&radius_squared)?;
            let discriminant = radial_term
                .subtract(&cross.square()?)?
                .scale(&Real::from(4_i8))?;
            Some((
                line_denominator,
                line_start_x,
                line_start_y,
                delta_x,
                delta_y,
                vx,
                vy,
                direction_x,
                direction_y,
                common_denominator,
                a,
                b,
                c,
                discriminant,
                cross,
            ))
        })()
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let a_sign = a.sign(&CurveContext::STRICT)?;
        match a_sign {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a recursive projective chord retained a nonpositive quadratic coefficient"
                        .into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        // A certified finite endpoint is already an exact root. Its
        // derivative determines the root's branch and multiplicity; factor it
        // in the parent field instead of adjoining the square root of that
        // derivative squared and later reconstructing the same endpoint.
        let certified_endpoint = if clip_to_finite_chord {
            certified_endpoint_incidence.and_then(|incidence| {
                let derivative = match incidence {
                    BezierCertifiedFiniteChordEndpointIncidence2::Start => Some(b.clone()),
                    BezierCertifiedFiniteChordEndpointIncidence2::End => a
                        .scale(&Real::from(2_i8))
                        .and_then(|derivative| derivative.add(&b)),
                }?;
                Some((incidence, derivative))
            })
        } else {
            None
        };
        let certified_endpoint = match certified_endpoint {
            Some((endpoint, derivative)) => match derivative.sign(&CurveContext::STRICT)? {
                Classification::Decided(sign) => Some((endpoint, sign)),
                Classification::Uncertain(_) => None,
            },
            None => None,
        };
        let discriminant_classification = match certified_endpoint {
            Some((_, RealSign::Zero)) => Classification::Decided(RealSign::Zero),
            Some((_, RealSign::Negative | RealSign::Positive)) => {
                Classification::Decided(RealSign::Positive)
            }
            None => discriminant.sign(&CurveContext::STRICT)?,
        };
        let discriminant_sign = match discriminant_classification {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if discriminant_sign == RealSign::Negative {
            return Ok(Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
            )));
        }
        let roots = if let Some((endpoint, derivative_sign)) = certified_endpoint {
            let (root, location) = match endpoint {
                BezierCertifiedFiniteChordEndpointIncidence2::Start => (
                    Real::zero(),
                    BezierAlgebraicCuspSemicircleContactLocation2::Start,
                ),
                BezierCertifiedFiniteChordEndpointIncidence2::End => (
                    Real::one(),
                    BezierAlgebraicCuspSemicircleContactLocation2::End,
                ),
            };
            let root = parent_field.constant(root).ok_or_else(|| {
                CurveError::Topology("a certified endpoint lost its coefficient field".into())
            })?;
            let endpoint_scalar = RecursiveQuadraticProjectiveScalar {
                numerator: root.clone(),
                denominator: parent_field.constant(Real::one()).ok_or_else(|| {
                    CurveError::Topology("a certified endpoint lost its unit denominator".into())
                })?,
            };
            let endpoint_branch = match derivative_sign {
                RealSign::Negative => -1,
                RealSign::Zero => 0,
                RealSign::Positive => 1,
            };
            let mut roots = vec![(endpoint_branch, endpoint_scalar, Some(location))];
            if derivative_sign != RealSign::Zero {
                let mut context = RecursiveQuadraticOrderedFieldContext {
                    field: parent_field.clone(),
                    policy: policy.strict_counterpart(),
                };
                let quotient = match hypersolve::ordered_field_polynomial_linear_quotient(
                    &[c.clone(), b.clone(), a.clone()],
                    &root,
                    &mut context,
                ) {
                    Ok(quotient) => quotient,
                    Err(RecursiveQuadraticOrderedFieldError::Context(error)) => return Err(error),
                    Err(RecursiveQuadraticOrderedFieldError::Uncertain) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                    }
                };
                let numerator = quotient[0].scale(&Real::from(-1_i8)).ok_or_else(|| {
                    CurveError::Topology("a factored circle contact lost its field".into())
                })?;
                roots.push((
                    -endpoint_branch,
                    RecursiveQuadraticProjectiveScalar {
                        numerator,
                        // The circle quadratic has a certified positive leading term.
                        denominator: a.clone(),
                    },
                    None,
                ));
                roots.sort_by_key(|(branch, _, _)| *branch);
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-chord-kernel",
                "recursive-certified-endpoint-factor",
            );
            roots
        } else {
            let branches: &[i8] = if discriminant_sign == RealSign::Zero {
                &[0]
            } else {
                &[-1, 1]
            };
            let extension = if discriminant_sign == RealSign::Positive {
                Some(
                    parent_field
                        .extension(discriminant.clone())
                        .ok_or_else(|| {
                            CurveError::Topology(
                                "a circle discriminant could not extend its field".into(),
                            )
                        })?,
                )
            } else {
                None
            };
            let mut roots = Vec::with_capacity(branches.len());
            for &branch in branches {
                let Some((numerator, denominator)) = (|| {
                    let retained = b.scale(&Real::from(-1_i8))?;
                    let denominator = a.scale(&Real::from(2_i8))?;
                    if let Some(extension) = extension.as_ref() {
                        Some((
                            extension
                                .element(retained, parent_field.constant(Real::from(branch))?)?,
                            extension.lift(&denominator)?,
                        ))
                    } else {
                        Some((retained, denominator))
                    }
                })() else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                roots.push((
                    branch,
                    RecursiveQuadraticProjectiveScalar {
                        numerator,
                        denominator,
                    },
                    None,
                ));
            }
            roots
        };
        let radial_scale = self.radial_distance() * &authority.normal_denominator;
        let selected_scale = self.turn_sign() * &radial_scale;
        let radius_squared_scale = self.radial_distance()
            * self.radial_distance()
            * &authority.normal_denominator
            * &authority.normal_denominator;
        let selected_linear = (|| {
            let constant = anchor_x
                .multiply(&vy)?
                .subtract(&anchor_y.multiply(&vx)?)?
                .scale(&selected_scale)?;
            let slope = anchor_x
                .multiply(&direction_y)?
                .subtract(&anchor_y.multiply(&direction_x)?)?
                .scale(&selected_scale)?;
            Some((constant, slope))
        })();
        let diameter_endpoint_line_sides = (|| {
            // The product of the selected-half predicates at the two full
            // circle contacts has the sign of the target line evaluated at
            // the selected diameter endpoints.  Keep that unsquared
            // geometry in the parent field so a deep contact radical is not
            // needed merely to decide which half owns each root.
            let center_side = vx
                .multiply(&direction_y)?
                .subtract(&vy.multiply(&direction_x)?)?;
            let anchor_side = anchor_y
                .multiply(&direction_x)?
                .subtract(&anchor_x.multiply(&direction_y)?)?;
            let normal_squared = &authority.normal_denominator * &authority.normal_denominator;
            let center_term = center_side
                .multiply(&anchor_denominator)?
                .scale(&normal_squared)?;
            let radial_term = anchor_side
                .multiply(&common_denominator)?
                .scale(&radial_scale)?;
            Some([
                center_term.add(&radial_term)?,
                center_term.subtract(&radial_term)?,
            ])
        })();
        let turn_sign = if self.is_clockwise() {
            RealSign::Negative
        } else {
            RealSign::Positive
        };
        let tangent_dot_sign = line_center_cross
            .bounded_interval_sign(0..=512)
            .map(|sign| product_sign(sign, turn_sign));
        let selected_root_signs = if let Some((constant, slope)) = selected_linear.as_ref() {
            recursive_quadratic_affine_predicate_root_signs(
                &a,
                &b,
                &c,
                discriminant_sign,
                constant,
                slope,
                diameter_endpoint_line_sides.as_ref(),
            )?
        } else {
            None
        };
        let mut retained_contacts = Vec::with_capacity(roots.len());
        let mut contacts = Vec::with_capacity(roots.len());
        for (quadratic_branch, scalar, certified_chord_location) in roots {
            let field = scalar.numerator.field();
            let field = &field;
            let parameter_numerator = scalar.numerator;
            let parameter_denominator = scalar.denominator;
            let Some((point, center, anchor_x, anchor_y, anchor_denominator, delta_x, delta_y)) =
                (|| {
                    let line_start_x = field.lift(&line_start_x)?;
                    let line_start_y = field.lift(&line_start_y)?;
                    let delta_x = field.lift(&delta_x)?;
                    let delta_y = field.lift(&delta_y)?;
                    let point = match certified_chord_location {
                        Some(BezierAlgebraicCuspSemicircleContactLocation2::Start) => start.clone(),
                        Some(BezierAlgebraicCuspSemicircleContactLocation2::End) => end.clone(),
                        _ => BezierRecursiveQuadraticProjectivePoint2 {
                            x: parameter_denominator
                                .multiply(&line_start_x)?
                                .add(&parameter_numerator.multiply(&delta_x)?)?,
                            y: parameter_denominator
                                .multiply(&line_start_y)?
                                .add(&parameter_numerator.multiply(&delta_y)?)?,
                            denominator: parameter_denominator
                                .multiply(&field.lift(&line_denominator)?)?,
                        },
                    };
                    Some((
                        point,
                        center.lifted_to(field)?,
                        field.lift(&anchor_x)?,
                        field.lift(&anchor_y)?,
                        field.lift(&anchor_denominator)?,
                        delta_x,
                        delta_y,
                    ))
                })()
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let Some((radial_x, radial_y, radial_denominator)) =
                point.difference_numerators(&center)
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let Some((
                selected_half_plane,
                diameter,
                radius_squared_denominator,
                tangent_cross,
                angular_tangent,
            )) = (|| {
                let selected_half_plane = anchor_x
                    .multiply(&radial_y)?
                    .subtract(&anchor_y.multiply(&radial_x)?)?
                    .scale(&selected_scale)?;
                let diameter = anchor_x
                    .multiply(&radial_x)?
                    .add(&anchor_y.multiply(&radial_y)?)?
                    .scale(&radial_scale)?;
                let radius_squared_denominator = anchor_denominator
                    .multiply(&radial_denominator)?
                    .scale(&radius_squared_scale)?;
                let tangent_cross = radial_x
                    .multiply(&delta_x)?
                    .add(&radial_y.multiply(&delta_y)?)?
                    .scale(&(-self.turn_sign()))?;
                let angular_tangent = radial_x
                    .multiply(&delta_y)?
                    .subtract(&radial_y.multiply(&delta_x)?)?;
                Some((
                    selected_half_plane,
                    diameter,
                    radius_squared_denominator,
                    tangent_cross,
                    angular_tangent,
                ))
            })()
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let selected_sign = if let Some(signs) = selected_root_signs {
                Classification::Decided(signs[usize::from(quadratic_branch > 0)])
            } else {
                selected_half_plane.sign(&CurveContext::STRICT)?
            };
            let cusp_location = match selected_sign {
                Classification::Decided(RealSign::Negative) => continue,
                Classification::Decided(RealSign::Positive) => {
                    BezierAlgebraicCuspSemicircleContactLocation2::Interior
                }
                Classification::Decided(RealSign::Zero) => {
                    let diameter_sign = diameter.sign(&CurveContext::STRICT)?;
                    match diameter_sign {
                        Classification::Decided(RealSign::Positive) => {
                            BezierAlgebraicCuspSemicircleContactLocation2::Start
                        }
                        Classification::Decided(RealSign::Negative) => {
                            BezierAlgebraicCuspSemicircleContactLocation2::End
                        }
                        Classification::Decided(RealSign::Zero) => {
                            return Err(CurveError::Topology(
                                "a nonzero recursive circle contact had zero local diameter".into(),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let chord_location = if let Some(location) = certified_chord_location {
                location
            } else if clip_to_finite_chord {
                let lower_sign = parameter_numerator.sign(&CurveContext::STRICT)?;
                match lower_sign {
                    Classification::Decided(RealSign::Negative) => continue,
                    Classification::Decided(RealSign::Zero) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(RealSign::Positive) => {
                        let upper = parameter_numerator
                            .subtract(&parameter_denominator)
                            .ok_or_else(|| {
                                CurveError::Topology(
                                    "a recursive chord upper-bound predicate exceeded its field budget"
                                        .into(),
                                )
                            })?;
                        let upper_sign = upper.sign(&CurveContext::STRICT)?;
                        match upper_sign {
                            Classification::Decided(RealSign::Positive) => continue,
                            Classification::Decided(RealSign::Zero) => {
                                BezierAlgebraicCuspSemicircleContactLocation2::End
                            }
                            Classification::Decided(RealSign::Negative) => {
                                BezierAlgebraicCuspSemicircleContactLocation2::Interior
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            } else {
                BezierAlgebraicCuspSemicircleContactLocation2::Interior
            };
            // The contact tangent is the derivative of the positive-leading
            // circle quadratic. Its exact sign is therefore fixed by the
            // selected root branch and circle traversal.
            let tangent_cross_sign =
                recursive_circle_contact_tangent_cross_sign(quadratic_branch, turn_sign);
            let branch = quadratic_branch;
            let certified_bounds =
                clip_to_finite_chord.then(|| chord_location.certified_unit_bounds());
            let parameter = match BezierRecursiveProjectiveParameter2::new_with_certified_bounds(
                RecursiveQuadraticProjectiveScalar {
                    numerator: parameter_numerator.clone(),
                    denominator: parameter_denominator.clone(),
                },
                certified_bounds,
                policy,
            )? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            retained_contacts.push(BezierRecursiveQuadraticLineContactSystem2 {
                branch,
                parameter,
                tangent_dot_sign,
                point,
                diameter,
                radius_squared_denominator,
                tangent_cross,
                angular_tangent,
            });
            contacts.push(BezierAlgebraicCuspSemicircleChordContact2 {
                branch,
                projective_parameter: None,
                cusp_location,
                chord_location,
                tangent_cross_sign,
            });
        }
        if contacts.is_empty() {
            return Ok(Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
            )));
        }
        if contacts.len() > 2
            || retained_contacts
                .iter()
                .map(|contact| contact.branch)
                .collect::<std::collections::BTreeSet<_>>()
                .len()
                != retained_contacts.len()
        {
            return Err(CurveError::Topology(
                "a recursive projective chord retained duplicate contact branches".into(),
            ));
        }
        let parameter_map = BezierAlgebraicCuspSemicircleChordParameterMap2::new(
            self.clone(),
            chord.clone(),
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RecursiveQuadraticLine(
                BezierRecursiveQuadraticLineParameterMapSystem2 {
                    center: authority.center,
                    contacts: retained_contacts,
                },
            ),
            clip_to_finite_chord,
            policy,
        );
        Ok(Classification::Decided(Some(
            BezierAlgebraicCuspSemicircleChordIntersections2::Contacts {
                contacts,
                parameter_map,
            },
        )))
    }

    /// Intersects a selected-radial circle with a procedural normal offset of
    /// a chord whose endpoints already live in the same recursive ancestry.
    /// The source speed is adjoined once as its certified positive square
    /// root, after which the offset endpoints enter the projective quadratic
    /// solver shared by every recursive chord.
    pub(in crate::bezier_offset) fn recursive_selected_radial_retained_offset_chord_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        certified_endpoint_incidence: Option<BezierCertifiedFiniteChordEndpointIncidence2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleChordIntersections2>>> {
        chord.validate_policy(policy)?;
        let Some((support, descendant_reversed)) = chord.retained_normal_offset_ancestor() else {
            return Ok(Classification::Decided(None));
        };
        let [support_start, support_end] = if descendant_reversed {
            [support.end(), support.start()]
        } else {
            [support.start(), support.end()]
        };
        let (
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(offset_start)),
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(offset_end)),
        ) = (support_start, support_end)
        else {
            return Ok(Classification::Decided(None));
        };
        if !offset_start.shares_carrier(offset_end)
            || offset_start.at_end == offset_end.at_end
            || offset_start.data.source_point.is_some()
            || offset_start.data.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal
        {
            return Ok(Classification::Decided(None));
        }
        let distance = if offset_start.at_end {
            -offset_start.data.distance.clone()
        } else {
            offset_start.data.distance.clone()
        };
        let translation_x = offset_start.data.translation_x.clone();
        let translation_y = offset_start.data.translation_y.clone();
        let source = &offset_start.data.source;
        let source_support = source.retained_support();
        let [source_start, source_end] = if source.retained_support_orientation_is_reversed() {
            [source_support.end(), source_support.start()]
        } else {
            [source_support.start(), source_support.end()]
        };
        let [source_start, source_end] = if offset_start.at_end {
            [source_end, source_start]
        } else {
            [source_start, source_end]
        };
        let start = match recursive_projective_point_source(source_start, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match recursive_projective_point_source(source_end, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let authority = match self.recursive_selected_pair_frame_authority(policy)? {
            Classification::Decided(Some(authority)) => authority,
            Classification::Decided(None) => {
                match self.recursive_selected_radial_frame_authority(policy)? {
                    Classification::Decided(Some(authority)) => authority,
                    Classification::Decided(None) => return Ok(Classification::Decided(None)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (authority, [start, end]) =
            match embed_recursive_projective_point_sources(authority, [start, end], policy)? {
                Classification::Decided(Some(embedded)) => embedded,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let start = match positive_recursive_projective_point(start)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match positive_recursive_projective_point(end)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let Some((delta_x, delta_y, speed_squared)) = (|| {
            let delta_x = end
                .x
                .multiply(&start.denominator)?
                .subtract(&start.x.multiply(&end.denominator)?)?;
            let delta_y = end
                .y
                .multiply(&start.denominator)?
                .subtract(&start.y.multiply(&end.denominator)?)?;
            let speed_squared = delta_x.square()?.add(&delta_y.square()?)?;
            Some((delta_x, delta_y, speed_squared))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let speed_sign = speed_squared.sign(&CurveContext::STRICT)?;
        match speed_sign {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a retained recursive offset chord had zero source length".into(),
                ));
            }
            Classification::Decided(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a retained recursive offset chord had negative squared length".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let speed_parent = authority.field.clone();
        let speed_field = speed_parent.extension(speed_squared).ok_or_else(|| {
            CurveError::Topology(
                "a retained recursive offset speed could not extend its source field".into(),
            )
        })?;
        let Some(authority) = authority.lifted_to(&speed_field) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some((start, end, delta_x, delta_y, speed)) = (|| {
            let start = start.lifted_to(&speed_field)?;
            let end = end.lifted_to(&speed_field)?;
            let delta_x = speed_field.lift(&delta_x)?;
            let delta_y = speed_field.lift(&delta_y)?;
            let speed = speed_field.element(
                speed_parent.constant(Real::zero())?,
                speed_parent.constant(Real::one())?,
            )?;
            Some((start, end, delta_x, delta_y, speed))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let displaced = |point: &BezierRecursiveQuadraticProjectivePoint2| {
            let x = point
                .x
                .add(&point.denominator.scale(&translation_x)?)?
                .multiply(&speed)?
                .subtract(&point.denominator.multiply(&delta_y)?.scale(&distance)?)?;
            let y = point
                .y
                .add(&point.denominator.scale(&translation_y)?)?
                .multiply(&speed)?
                .add(&point.denominator.multiply(&delta_x)?.scale(&distance)?)?;
            Some(BezierRecursiveQuadraticProjectivePoint2 {
                x,
                y,
                denominator: point.denominator.multiply(&speed)?,
            })
        };
        let (Some(start), Some(end)) = (displaced(&start), displaced(&end)) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        self.recursive_projective_chord_intersections(
            chord,
            authority,
            start,
            end,
            clip_to_finite_chord,
            certified_endpoint_incidence,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn axis_chord_system_contact_minus_support_sign(
        system: &BezierAlgebraicCuspSemicircleChordSystem2,
        branch: i8,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        algebraic_cusp_correlated_square_root_sum_sign(
            &system.incidence,
            &algebraic_cusp_branched_expression(&system.point_minus_support_axis, branch),
            &system.discriminant,
            &system.cusp_parameter,
            &system.support_parameter,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn axis_chord_contact_minus_point_sign(
        &self,
        point: &CurvePoint2,
        direction: BezierAlgebraicChordAxisDirection2,
        branch: i8,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let system = match self.axis_chord_system_for_support_point(point, direction, policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Self::axis_chord_system_contact_minus_support_sign(&system, branch, policy)
    }

    pub(in crate::bezier_offset) fn axis_chord_contact_minus_point_sign_cached(
        &self,
        chord: &BezierAlgebraicChord2,
        system: &BezierAlgebraicCuspSemicircleChordSystem2,
        support: &CurvePoint2,
        point: &CurvePoint2,
        cached: &mut Option<BezierAlgebraicCuspSemicircleChordSystem2>,
        direction: BezierAlgebraicChordAxisDirection2,
        branch: i8,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if support.shares_storage(point) {
            return Self::axis_chord_system_contact_minus_support_sign(system, branch, policy);
        }
        if let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point {
            if !point.accepts_policy(policy) {
                return Err(CurveError::Topology(
                    "correlated chord point was replayed under a different predicate policy".into(),
                ));
            }
            let opposite = if chord.shares_retained_support(&point.data.first) {
                Some(&point.data.second)
            } else if chord.shares_retained_support(&point.data.second) {
                Some(&point.data.first)
            } else {
                None
            };
            if let Some(opposite) = opposite
                && opposite
                    .certified_axis_direction()
                    .is_some_and(|opposite| opposite.axis() != direction.axis())
            {
                opposite.validate_policy(policy)?;
                if cached.is_none() {
                    let retained_support = opposite.retained_support();
                    let mut uncertainty = UncertaintyReason::Unsupported;
                    for endpoint in [retained_support.start(), retained_support.end()] {
                        match self
                            .axis_chord_system_for_support_point(endpoint, direction, policy)?
                        {
                            Classification::Decided(endpoint_system) => {
                                *cached = Some(endpoint_system);
                                break;
                            }
                            Classification::Uncertain(reason) => {
                                if uncertainty == UncertaintyReason::Unsupported {
                                    uncertainty = reason;
                                }
                            }
                        }
                    }
                    if cached.is_none() {
                        return Ok(Classification::Uncertain(uncertainty));
                    }
                }
                return Self::axis_chord_system_contact_minus_support_sign(
                    cached
                        .as_ref()
                        .expect("perpendicular endpoint system was initialized above"),
                    branch,
                    policy,
                );
            }
        }
        if let CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) = point {
            let (map, contact) = point.map_contact();
            map.validate_policy(policy)?;
            if map.data.semicircle == *self
                && map.data.chord.shares_retained_support(chord)
                && map
                    .axis_direction()
                    .is_some_and(|map_direction| map_direction.axis() == direction.axis())
            {
                let point_branch = if map.axis_direction() == Some(direction) {
                    contact.branch
                } else {
                    -contact.branch
                };
                return Ok(Classification::Decided(match branch.cmp(&point_branch) {
                    std::cmp::Ordering::Less => RealSign::Negative,
                    std::cmp::Ordering::Equal => RealSign::Zero,
                    std::cmp::Ordering::Greater => RealSign::Positive,
                }));
            }
        }
        if let CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point)) = point {
            match point.concentric_axis_contact_minus_point_sign(self, direction, branch, policy)? {
                Classification::Decided(Some(sign)) => {
                    return Ok(Classification::Decided(sign));
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if branch == 0
            && matches!(
                point,
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
            )
        {
            let strict_incidence = self.strict_point_incidence_sign(point, policy)?;
            if strict_incidence == Classification::Decided(RealSign::Zero) {
                // A zero discriminant gives the axis support exactly one
                // circle contact. A retained chord endpoint is already
                // certified on that support, so exact circle incidence
                // identifies the endpoint with the contact without asking two
                // independently isolated fields to converge onto equality.
                return Ok(Classification::Decided(RealSign::Zero));
            }
            if matches!(
                strict_incidence,
                Classification::Decided(RealSign::Positive)
            ) {
                let center = match self.center_point_evidence(policy)? {
                    Classification::Decided(center) => center,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let order = algebraic_chord_point_coordinate_order(
                    &center,
                    point,
                    direction.axis(),
                    policy,
                )?;
                if let Classification::Decided(
                    order @ (std::cmp::Ordering::Less | std::cmp::Ordering::Greater),
                ) = order
                {
                    let coordinate_sign = match order {
                        std::cmp::Ordering::Less => RealSign::Negative,
                        std::cmp::Ordering::Greater => RealSign::Positive,
                        std::cmp::Ordering::Equal => unreachable!("filtered above"),
                    };
                    return Ok(Classification::Decided(
                        if matches!(
                            direction,
                            BezierAlgebraicChordAxisDirection2::NegativeX
                                | BezierAlgebraicChordAxisDirection2::NegativeY
                        ) {
                            match coordinate_sign {
                                RealSign::Negative => RealSign::Positive,
                                RealSign::Positive => RealSign::Negative,
                                RealSign::Zero => unreachable!("the endpoint is distinct"),
                            }
                        } else {
                            coordinate_sign
                        },
                    ));
                }
            }
        }
        if cached.is_none() {
            *cached = Some(
                match self.axis_chord_system_for_support_point(point, direction, policy)? {
                    Classification::Decided(system) => system,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            );
        }
        Self::axis_chord_system_contact_minus_support_sign(
            cached
                .as_ref()
                .expect("endpoint system was initialized above"),
            branch,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn axis_chord_intersections_in_domain(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleChordIntersections2>> {
        chord.validate_policy(policy)?;
        let Some(direction) = chord.certified_axis_direction() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        // A finite endpoint strictly outside the conservative projection of
        // the whole circle has the same oriented sign against every eventual
        // line contact. Reuse that stronger certificate before constructing
        // a contact coordinate in an independent selected field.
        let mut endpoint_projection_signs = [None, None];
        if clip_to_finite_chord {
            for refinement_steps in [0, 2, 4, 8] {
                let circle = match self.conservative_bounds_refined(refinement_steps, policy)? {
                    Classification::Decided(bounds) => bounds,
                    Classification::Uncertain(_) => continue,
                };
                let start = match algebraic_chord_endpoint_bounds_refined(
                    chord.start(),
                    refinement_steps,
                    policy,
                ) {
                    Classification::Decided(bounds) => bounds,
                    Classification::Uncertain(_) => continue,
                };
                let end = match algebraic_chord_endpoint_bounds_refined(
                    chord.end(),
                    refinement_steps,
                    policy,
                ) {
                    Classification::Decided(bounds) => bounds,
                    Classification::Uncertain(_) => continue,
                };
                let (circle_min, circle_max, start_min, start_max, end_min, end_max) =
                    match direction.axis() {
                        Axis2::X => (
                            circle.min_x(),
                            circle.max_x(),
                            start.min_x(),
                            start.max_x(),
                            end.min_x(),
                            end.max_x(),
                        ),
                        Axis2::Y => (
                            circle.min_y(),
                            circle.max_y(),
                            start.min_y(),
                            start.max_y(),
                            end.min_y(),
                            end.max_y(),
                        ),
                    };
                let positive_direction = matches!(
                    direction,
                    BezierAlgebraicChordAxisDirection2::PositiveX
                        | BezierAlgebraicChordAxisDirection2::PositiveY
                );
                let endpoint_sign = |minimum: &Real, maximum: &Real| {
                    if positive_direction {
                        if compare_reals(maximum, circle_min, &CurveContext::STRICT)
                            == Some(std::cmp::Ordering::Less)
                        {
                            Some(RealSign::Positive)
                        } else if compare_reals(circle_max, minimum, &CurveContext::STRICT)
                            == Some(std::cmp::Ordering::Less)
                        {
                            Some(RealSign::Negative)
                        } else {
                            None
                        }
                    } else if compare_reals(circle_max, minimum, &CurveContext::STRICT)
                        == Some(std::cmp::Ordering::Less)
                    {
                        Some(RealSign::Positive)
                    } else if compare_reals(maximum, circle_min, &CurveContext::STRICT)
                        == Some(std::cmp::Ordering::Less)
                    {
                        Some(RealSign::Negative)
                    } else {
                        None
                    }
                };
                if endpoint_projection_signs[0].is_none() {
                    endpoint_projection_signs[0] = endpoint_sign(start_min, start_max);
                }
                if endpoint_projection_signs[1].is_none() {
                    endpoint_projection_signs[1] = endpoint_sign(end_min, end_max);
                }
                if endpoint_projection_signs.iter().all(Option::is_some) {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-circle-axis-chord-clip",
                        "endpoint-projection-signs",
                    );
                    break;
                }
            }
        }
        // A point strictly inside the supporting disk certifies that its line
        // is a secant. It also lies strictly between the negative and positive
        // radical contacts, so the same incidence proof orders both contacts
        // without constructing either endpoint in a second selected field.
        let mut endpoint_inside_circle = [false; 2];
        if clip_to_finite_chord {
            for (index, endpoint) in [(0, chord.start()), (1, chord.end())] {
                if endpoint_projection_signs[index].is_none() {
                    endpoint_inside_circle[index] = matches!(
                        self.strict_point_incidence_sign(endpoint, policy)?,
                        Classification::Decided(RealSign::Negative)
                    );
                }
            }
        }
        // A represented descendant may retain a procedural offset ancestor
        // solely as provenance. Its immediate endpoint is the smaller exact
        // support authority; descending first would needlessly replace a
        // canonical Real coordinate with the ancestor's normal radical.
        let support = if chord.exact_line().is_some() {
            chord.start()
        } else {
            chord.retained_support().start()
        };
        let system = match self.axis_chord_system_for_support_point(support, direction, policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let sign = |predicate: &BivariatePolynomial| {
            algebraic_selected_correlated_predicate_sign(
                &system.incidence,
                predicate,
                &system.cusp_parameter,
                &system.support_parameter,
                policy,
            )
        };
        let discriminant_sign = if endpoint_inside_circle.iter().any(|inside| *inside) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-axis-chord-clip",
                "interior-endpoint-secant",
            );
            RealSign::Positive
        } else {
            match sign(&system.discriminant)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let branches: &[i8] = match discriminant_sign {
            RealSign::Negative => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
                ));
            }
            RealSign::Zero => &[0],
            RealSign::Positive => &[-1, 1],
        };
        let radical_sign = |expression: &BezierAlgebraicCuspTwoTermExpression2, branch: i8| {
            algebraic_cusp_correlated_square_root_sum_sign(
                &system.incidence,
                &algebraic_cusp_branched_expression(expression, branch),
                &system.discriminant,
                &system.cusp_parameter,
                &system.support_parameter,
                policy,
            )
        };
        let mut contacts = Vec::with_capacity(branches.len());
        let mut start_system = None;
        let mut end_system = None;
        for &branch in branches {
            let selected = match radical_sign(&system.selected_half_plane, branch)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let cusp_location = match selected {
                RealSign::Negative => continue,
                RealSign::Positive => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                RealSign::Zero => match radical_sign(&system.diameter_side, branch)? {
                    Classification::Decided(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(RealSign::Negative) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::End
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "nonzero selected semicircle had an indeterminate chord endpoint"
                                .into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            };
            let chord_location = if clip_to_finite_chord {
                let start_sign = match endpoint_projection_signs[0] {
                    Some(sign) => sign,
                    None if endpoint_inside_circle[0] => {
                        debug_assert_ne!(branch, 0);
                        if branch < 0 {
                            RealSign::Negative
                        } else {
                            RealSign::Positive
                        }
                    }
                    None => match self.axis_chord_contact_minus_point_sign_cached(
                        chord,
                        &system,
                        support,
                        chord.start(),
                        &mut start_system,
                        direction,
                        branch,
                        policy,
                    )? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    },
                };
                let end_sign = match endpoint_projection_signs[1] {
                    Some(sign) => sign,
                    None if endpoint_inside_circle[1] => {
                        debug_assert_ne!(branch, 0);
                        if branch < 0 {
                            RealSign::Negative
                        } else {
                            RealSign::Positive
                        }
                    }
                    None => match self.axis_chord_contact_minus_point_sign_cached(
                        chord,
                        &system,
                        support,
                        chord.end(),
                        &mut end_system,
                        direction,
                        branch,
                        policy,
                    )? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    },
                };
                if start_sign == RealSign::Negative || end_sign == RealSign::Positive {
                    continue;
                }
                match (start_sign, end_sign) {
                    (RealSign::Zero, _) => BezierAlgebraicCuspSemicircleContactLocation2::Start,
                    (_, RealSign::Zero) => BezierAlgebraicCuspSemicircleContactLocation2::End,
                    _ => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                }
            } else {
                BezierAlgebraicCuspSemicircleContactLocation2::Interior
            };
            let tangent_cross_sign = match branch {
                0 => RealSign::Zero,
                -1 => {
                    if self.is_clockwise() {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    }
                }
                1 => {
                    if self.is_clockwise() {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    }
                }
                _ => unreachable!("circle/chord support has at most two branches"),
            };
            contacts.push(BezierAlgebraicCuspSemicircleChordContact2 {
                branch,
                projective_parameter: None,
                cusp_location,
                chord_location,
                tangent_cross_sign,
            });
        }
        if contacts.is_empty() {
            return Ok(Classification::Decided(
                BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
            ));
        }
        let parameter_map = BezierAlgebraicCuspSemicircleChordParameterMap2::new(
            self.clone(),
            chord.clone(),
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Axis(
                BezierAlgebraicCuspSemicircleAxisChordParameterMapSystem2 {
                    incidence: system.incidence,
                    discriminant: system.discriminant,
                    diameter_side: system.diameter_side,
                    radius_squared_denominator: system.radius_squared_denominator,
                    common_denominator: system.common_denominator,
                    center_x: system.center_x,
                    center_y: system.center_y,
                    point_x: system.point_x,
                    point_y: system.point_y,
                    cusp_parameter: system.cusp_parameter,
                    support_parameter: system.support_parameter,
                    direction: system.direction,
                },
            ),
            clip_to_finite_chord,
            policy,
        );
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleChordIntersections2::Contacts {
                contacts,
                parameter_map,
            },
        ))
    }

    pub(in crate::bezier_offset) fn oblique_chord_intersections_in_domain(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleChordIntersections2>> {
        let system = match self.oblique_chord_system(chord, policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let retained = &system.retained;
        let sign = |polynomial: &TrivariatePolynomial| {
            trivariate_parameter_triple_sign_by_refinement(
                polynomial,
                &retained.first_parameter,
                &retained.second_parameter,
                &retained.cusp_parameter,
                policy,
            )
        };
        let discriminant_sign = match sign(&retained.discriminant)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let branches: &[i8] = match discriminant_sign {
            RealSign::Negative => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
                ));
            }
            RealSign::Zero => &[0],
            RealSign::Positive => &[-1, 1],
        };
        let radical_sign = |expression, branch| {
            algebraic_cusp_trivariate_square_root_sum_sign(
                expression,
                &retained.discriminant,
                &retained.first_parameter,
                &retained.second_parameter,
                &retained.cusp_parameter,
                branch,
                policy,
            )
        };
        let mut contacts = Vec::with_capacity(branches.len());
        for &branch in branches {
            let selected = match radical_sign(&system.selected_half_plane, branch)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let cusp_location = match selected {
                RealSign::Negative => continue,
                RealSign::Positive => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                RealSign::Zero => match radical_sign(&retained.diameter_side, branch)? {
                    Classification::Decided(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(RealSign::Negative) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::End
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "nonzero selected semicircle had an indeterminate oblique chord endpoint"
                                .into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            };
            let chord_location = if clip_to_finite_chord {
                let start_sign = match radical_sign(&system.point_minus_start, branch)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let end_sign = match radical_sign(&system.point_minus_end, branch)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if start_sign == RealSign::Negative || end_sign == RealSign::Positive {
                    continue;
                }
                match (start_sign, end_sign) {
                    (RealSign::Zero, _) => BezierAlgebraicCuspSemicircleContactLocation2::Start,
                    (_, RealSign::Zero) => BezierAlgebraicCuspSemicircleContactLocation2::End,
                    _ => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                }
            } else {
                BezierAlgebraicCuspSemicircleContactLocation2::Interior
            };
            let tangent_cross_sign = match branch {
                0 => RealSign::Zero,
                -1 => {
                    if self.is_clockwise() {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    }
                }
                1 => {
                    if self.is_clockwise() {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    }
                }
                _ => unreachable!("circle/chord support has at most two branches"),
            };
            contacts.push(BezierAlgebraicCuspSemicircleChordContact2 {
                branch,
                projective_parameter: None,
                cusp_location,
                chord_location,
                tangent_cross_sign,
            });
        }
        if contacts.is_empty() {
            return Ok(Classification::Decided(
                BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
            ));
        }
        let parameter_map = BezierAlgebraicCuspSemicircleChordParameterMap2::new(
            self.clone(),
            chord.clone(),
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Oblique(system.retained),
            clip_to_finite_chord,
            policy,
        );
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleChordIntersections2::Contacts {
                contacts,
                parameter_map,
            },
        ))
    }

    /// Exact finite line/circle incidence when exactly one chord endpoint is
    /// a retained unit displacement.
    ///
    /// Materializing that endpoint first turns its correlated source speed
    /// into two independent coordinate fields.  Instead retain
    ///
    /// `Q(u) - C = B(u) + M(u)/sqrt(q)`
    ///
    /// where `q` is the squared source-chord speed and `M` is either `u*N` or
    /// `(1-u)*N`.  Multiplying circle incidence by positive `q` gives the
    /// selected one-radical equation `P(u) + sqrt(q)*R(u) = 0`; its polynomial
    /// norm only enumerates candidates, and the authored positive sheet is
    /// replayed under STRICT before any contact is published.
    pub(in crate::bezier_offset) fn represented_parallel_endpoint_oblique_chord_intersections_in_domain(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        certified_endpoint_incidence: Option<BezierCertifiedFiniteChordEndpointIncidence2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleChordIntersections2>>> {
        chord.validate_policy(policy)?;
        let (parallel, other, common_origin_shift, displaced_at_start) =
            match (chord.start(), chord.end()) {
                (
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
                ) => {
                    // A pair belonging to one displaced support has a smaller
                    // dedicated kernel. For unrelated supports, consume a
                    // cardinal displacement in its source field and leave only
                    // the genuinely normalized endpoint on this one-radical path.
                    if start.shares_carrier(end) {
                        return Ok(Classification::Decided(None));
                    }
                    let origins_are_equal =
                        |first: &BezierAlgebraicChordParallelPoint2,
                         second: &BezierAlgebraicChordParallelPoint2| {
                            let first = first.source_endpoint();
                            let second = second.source_endpoint();
                            first.shares_storage(second)
                                || first.same_point(second, policy) == Classification::Decided(true)
                        };
                    if let Some(shift) = end.strict_cardinal_shifts(policy)
                        && origins_are_equal(start, end)
                    {
                        (start, None, Some(shift), true)
                    } else if let Some(shift) = start.strict_cardinal_shifts(policy)
                        && origins_are_equal(end, start)
                    {
                        (end, None, Some(shift), false)
                    } else if let Some(end) = end.strict_cardinal_point_evidence(policy)? {
                        (start, Some(Cow::Owned(end)), None, true)
                    } else if let Some(start) = start.strict_cardinal_point_evidence(policy)? {
                        (end, Some(Cow::Owned(start)), None, false)
                    } else {
                        return Ok(Classification::Decided(None));
                    }
                }
                (CurvePoint2(CurvePointData2::AlgebraicChordParallel(parallel)), other) => {
                    (parallel, Some(Cow::Borrowed(other)), None, true)
                }
                (other, CurvePoint2(CurvePointData2::AlgebraicChordParallel(parallel))) => {
                    (parallel, Some(Cow::Borrowed(other)), None, false)
                }
                _ => return Ok(Classification::Decided(None)),
            };
        if !parallel.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "a displaced chord endpoint entered circle incidence under a different policy"
                    .into(),
            ));
        }

        let source_start =
            match represented_point_evidence_coordinates(parallel.data.source.start(), policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let source_end =
            match represented_point_evidence_coordinates(parallel.data.source.end(), policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let frame = match self.represented_circle_frame(policy)? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let origin = if let Some(source_point) = parallel.data.source_point.as_deref() {
            let shared_center = match source_point {
                CurvePoint2(CurvePointData2::AnalyticParallel(origin))
                    if policy.accepts_retained_policy(origin.data.policy)
                        && origin.data.frame_tangent.is_none()
                        && origin.data.parallel.source()
                            == self
                                .data
                                .frame
                                .parallel_normal()
                                .map(|frame| frame.center_support.source())
                                .unwrap_or(origin.data.parallel.source())
                        && matches!(
                            &origin.data.parameter,
                            BezierAnalyticParallelPointParameter2::Bezier(parameter)
                                if self
                                    .data
                                    .frame
                                    .parallel_normal()
                                    .is_some_and(|frame| frame.center_parameter.as_bezier_parameter() == Some(parameter))
                        ) =>
                {
                    let center_frame = self
                        .data
                        .frame
                        .parallel_normal()
                        .expect("the shared origin guard retained a parallel-normal frame");
                    let normal_distance =
                        origin.data.parallel.distance() - center_frame.center_support.distance();
                    let x = represented_affine_coordinate(
                        &[
                            (&frame.center[0], &Real::one()),
                            (&frame.unit_radial[0], &normal_distance),
                            (&frame.unit_radial[1], &origin.data.tangent_distance),
                        ],
                        &origin.data.translation_x,
                    );
                    let y = represented_affine_coordinate(
                        &[
                            (&frame.center[1], &Real::one()),
                            (&frame.unit_radial[1], &normal_distance),
                            (
                                &frame.unit_radial[0],
                                &(-origin.data.tangent_distance.clone()),
                            ),
                        ],
                        &origin.data.translation_y,
                    );
                    match (x, y) {
                        (Classification::Decided(x), Classification::Decided(y)) => Some([x, y]),
                        _ => None,
                    }
                }
                _ => None,
            };
            if let Some(point) = shared_center {
                point
            } else {
                match represented_point_evidence_coordinates(source_point, policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        } else if parallel.at_end {
            source_end.clone()
        } else {
            source_start.clone()
        };
        enum OtherCoordinateConstruction2 {
            Direct([AlgebraicRootRepresentation; 2]),
            AffineDerived {
                source: [AlgebraicRootRepresentation; 2],
                center: [AlgebraicRootRepresentation; 2],
                radial: Real,
                perpendicular: Real,
                translation_x: Real,
                translation_y: Real,
            },
        }
        let other = if let Some([shift_x, shift_y]) = common_origin_shift {
            let one = Real::one();
            let x = represented_affine_coordinate(&[(&origin[0], &one)], &shift_x);
            let y = represented_affine_coordinate(&[(&origin[1], &one)], &shift_y);
            match (x, y) {
                (Classification::Decided(x), Classification::Decided(y)) => {
                    OtherCoordinateConstruction2::Direct([x, y])
                }
                (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        } else {
            let other = other.expect("a non-shared endpoint retains exact point evidence");
            if let CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(derived)) = other.as_ref()
                && derived.data.source.chord_map_contact().is_none()
                && derived.data.source.coincident_pair_map_contact().is_none()
                && let BezierAlgebraicCuspDerivedPointSource2::Mapped {
                    point: Some(source),
                    ..
                } = &derived.data.source
            {
                derived.data.source.validate_policy(policy)?;
                let source = match represented_point_evidence_coordinates(source, policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let center = match derived
                    .data
                    .source
                    .semicircle()
                    .center_point_evidence(policy)?
                {
                    Classification::Decided(center) => center,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let center = match represented_point_evidence_coordinates(&center, policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                OtherCoordinateConstruction2::AffineDerived {
                    source,
                    center,
                    radial: derived.data.radial_scale.clone(),
                    perpendicular: derived.data.perpendicular_scale.clone(),
                    translation_x: derived.data.translation_x.clone(),
                    translation_y: derived.data.translation_y.clone(),
                }
            } else {
                match represented_point_evidence_coordinates(other.as_ref(), policy)? {
                    Classification::Decided(point) => OtherCoordinateConstruction2::Direct(point),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        };
        let mut represented = vec![
            source_start[0].clone(),
            source_start[1].clone(),
            source_end[0].clone(),
            source_end[1].clone(),
            origin[0].clone(),
            origin[1].clone(),
        ];
        match &other {
            OtherCoordinateConstruction2::Direct(point) => {
                represented.extend(point.iter().cloned())
            }
            OtherCoordinateConstruction2::AffineDerived { source, center, .. } => {
                represented.extend(source.iter().chain(center).cloned());
            }
        }
        represented.extend([
            frame.center[0].clone(),
            frame.center[1].clone(),
            frame.unit_radial[0].clone(),
            frame.unit_radial[1].clone(),
        ]);
        let Some((sources, coordinates)) = represented_affine_tensor_basis(&represented) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let rank = sources.len() + 1;
        let target_axis = sources.len();
        let constant = |value: &Real| {
            DenseTensorPolynomial::from_axis_polynomial(rank, 0, std::slice::from_ref(value))
        };
        let axis = |coefficients: &[Real]| {
            DenseTensorPolynomial::from_axis_polynomial(rank, target_axis, coefficients)
        };
        let affine = |terms: &[(&DenseTensorPolynomial, &Real)], offset: &Real| {
            let mut result = constant(offset)?;
            for (polynomial, scale) in terms {
                result = result.add(&polynomial.scale(scale)?)?;
            }
            Some(result)
        };
        let mut coordinates = coordinates.into_iter();
        let mut next_coordinate = || {
            coordinates
                .next()
                .expect("a displaced endpoint tensor retains every requested coordinate")
        };
        let source_start_x = next_coordinate();
        let source_start_y = next_coordinate();
        let source_end_x = next_coordinate();
        let source_end_y = next_coordinate();
        let origin_x = next_coordinate();
        let origin_y = next_coordinate();
        let (other_x, other_y) = match other {
            OtherCoordinateConstruction2::Direct(_) => (next_coordinate(), next_coordinate()),
            OtherCoordinateConstruction2::AffineDerived {
                radial,
                perpendicular,
                translation_x,
                translation_y,
                ..
            } => {
                let source_x = next_coordinate();
                let source_y = next_coordinate();
                let source_center_x = next_coordinate();
                let source_center_y = next_coordinate();
                let negative_perpendicular = -perpendicular.clone();
                let one_minus_radial = Real::one() - &radial;
                let Some(x) = affine(
                    &[
                        (&source_x, &radial),
                        (&source_y, &negative_perpendicular),
                        (&source_center_x, &one_minus_radial),
                        (&source_center_y, &perpendicular),
                    ],
                    &translation_x,
                ) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let Some(y) = affine(
                    &[
                        (&source_x, &perpendicular),
                        (&source_y, &radial),
                        (&source_center_x, &negative_perpendicular),
                        (&source_center_y, &one_minus_radial),
                    ],
                    &translation_y,
                ) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                (x, y)
            }
        };
        let center_x = next_coordinate();
        let center_y = next_coordinate();
        let unit_radial_x = next_coordinate();
        let unit_radial_y = next_coordinate();
        debug_assert!(coordinates.next().is_none());
        let Some((
            q,
            incidence_rational,
            incidence_radical,
            projection,
            point_base_x,
            point_base_y,
            displacement_x,
            displacement_y,
            angular_dot_rational,
            angular_dot_radical,
            angular_cross_rational,
            angular_cross_radical,
            tangent_cross_rational,
            tangent_cross_radical,
            angular_tangent_rational,
            angular_tangent_radical,
        )) = (|| {
            let reduce = |polynomial| dense_reduce_selected_root_relations(polynomial, &sources);
            let source_dx = reduce(source_end_x.subtract(&source_start_x)?)?;
            let source_dy = reduce(source_end_y.subtract(&source_start_y)?)?;
            let q = reduce(
                source_dx
                    .multiply(&source_dx)?
                    .add(&source_dy.multiply(&source_dy)?)?,
            )?;
            let (unit_x, unit_y) = match parallel.data.direction {
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => {
                    (source_dy.scale(&Real::from(-1_i8))?, source_dx.clone())
                }
                BezierAlgebraicChordUnitDisplacement2::Tangent => {
                    (source_dx.clone(), source_dy.clone())
                }
            };
            let normal_x = reduce(unit_x.scale(&parallel.data.distance)?)?;
            let normal_y = reduce(unit_y.scale(&parallel.data.distance)?)?;
            let translated_origin_x =
                reduce(origin_x.add(&constant(&parallel.data.translation_x)?)?)?;
            let translated_origin_y =
                reduce(origin_y.add(&constant(&parallel.data.translation_y)?)?)?;
            let (base_start_x, base_start_y, base_end_x, base_end_y) = if displaced_at_start {
                (translated_origin_x, translated_origin_y, other_x, other_y)
            } else {
                (other_x, other_y, translated_origin_x, translated_origin_y)
            };
            let target_dx = reduce(base_end_x.subtract(&base_start_x)?)?;
            let target_dy = reduce(base_end_y.subtract(&base_start_y)?)?;
            let parameter = axis(&[Real::zero(), Real::one()])?;
            let displacement_weight = if displaced_at_start {
                axis(&[Real::one(), Real::from(-1_i8)])?
            } else {
                parameter.clone()
            };
            let displacement_derivative = if displaced_at_start {
                Real::from(-1_i8)
            } else {
                Real::one()
            };
            let point_base_x = reduce(base_start_x.add(&target_dx.multiply(&parameter)?)?)?;
            let point_base_y = reduce(base_start_y.add(&target_dy.multiply(&parameter)?)?)?;
            let radial_base_x = reduce(point_base_x.subtract(&center_x)?)?;
            let radial_base_y = reduce(point_base_y.subtract(&center_y)?)?;
            let displacement_x = reduce(normal_x.multiply(&displacement_weight)?)?;
            let displacement_y = reduce(normal_y.multiply(&displacement_weight)?)?;
            let tangent_displacement_x = reduce(normal_x.scale(&displacement_derivative)?)?;
            let tangent_displacement_y = reduce(normal_y.scale(&displacement_derivative)?)?;

            let radius_squared = self.radial_distance() * self.radial_distance();
            let radial_base_squared = reduce(
                radial_base_x
                    .multiply(&radial_base_x)?
                    .add(&radial_base_y.multiply(&radial_base_y)?)?
                    .subtract(&constant(&radius_squared)?)?,
            )?;
            let displacement_squared = reduce(
                displacement_x
                    .multiply(&displacement_x)?
                    .add(&displacement_y.multiply(&displacement_y)?)?,
            )?;
            let incidence_rational = reduce(
                q.multiply(&radial_base_squared)?
                    .add(&displacement_squared)?,
            )?;
            let incidence_radical = reduce(
                radial_base_x
                    .multiply(&displacement_x)?
                    .add(&radial_base_y.multiply(&displacement_y)?)?
                    .scale(&Real::from(2_i8))?,
            )?;
            let projection = reduce(
                incidence_rational
                    .multiply(&incidence_rational)?
                    .subtract(&q.multiply(&incidence_radical.multiply(&incidence_radical)?)?)?,
            )?;
            // Dot and selected cross against the authored semicircle start
            // radial, all represented over the same positive source speed.
            let signed_radius = frame.signed_radius.clone();
            let angular_scale = &signed_radius * self.turn_sign();
            let angular_dot_rational = reduce(
                unit_radial_x
                    .multiply(&displacement_x)?
                    .add(&unit_radial_y.multiply(&displacement_y)?)?
                    .scale(&signed_radius)?,
            )?;
            let angular_dot_radical = reduce(
                unit_radial_x
                    .multiply(&radial_base_x)?
                    .add(&unit_radial_y.multiply(&radial_base_y)?)?
                    .scale(&signed_radius)?,
            )?;
            let angular_cross_rational = reduce(
                unit_radial_x
                    .multiply(&displacement_y)?
                    .subtract(&unit_radial_y.multiply(&displacement_x)?)?
                    .scale(&angular_scale)?,
            )?;
            let angular_cross_radical = reduce(
                unit_radial_x
                    .multiply(&radial_base_y)?
                    .subtract(&unit_radial_y.multiply(&radial_base_x)?)?
                    .scale(&angular_scale)?,
            )?;

            // The circle tangent crossed with the chord tangent has positive
            // denominator q.  Retain only its numerator; every later branch
            // predicate can sign that numerator directly in the selected
            // radical field.
            let tangent_cross_scale = -self.turn_sign();
            let tangent_cross_rational = reduce(
                displacement_x
                    .multiply(&tangent_displacement_x)?
                    .add(&displacement_y.multiply(&tangent_displacement_y)?)?
                    .add(
                        &q.multiply(
                            &radial_base_x
                                .multiply(&target_dx)?
                                .add(&radial_base_y.multiply(&target_dy)?)?,
                        )?,
                    )?
                    .scale(&tangent_cross_scale)?,
            )?;
            let tangent_cross_radical = reduce(
                displacement_x
                    .multiply(&target_dx)?
                    .add(&displacement_y.multiply(&target_dy)?)?
                    .add(
                        &radial_base_x
                            .multiply(&tangent_displacement_x)?
                            .add(&radial_base_y.multiply(&tangent_displacement_y)?)?,
                    )?
                    .scale(&tangent_cross_scale)?,
            )?;
            let angular_tangent_rational = reduce(
                displacement_x
                    .multiply(&tangent_displacement_y)?
                    .subtract(&displacement_y.multiply(&tangent_displacement_x)?)?
                    .add(
                        &q.multiply(
                            &radial_base_x
                                .multiply(&target_dy)?
                                .subtract(&radial_base_y.multiply(&target_dx)?)?,
                        )?,
                    )?,
            )?;
            let angular_tangent_radical = reduce(
                displacement_x
                    .multiply(&target_dy)?
                    .subtract(&displacement_y.multiply(&target_dx)?)?
                    .add(
                        &radial_base_x
                            .multiply(&tangent_displacement_y)?
                            .subtract(&radial_base_y.multiply(&tangent_displacement_x)?)?,
                    )?,
            )?;
            Some((
                q,
                incidence_rational,
                incidence_radical,
                projection,
                point_base_x,
                point_base_y,
                displacement_x,
                displacement_y,
                angular_dot_rational,
                angular_dot_radical,
                angular_cross_rational,
                angular_cross_radical,
                tangent_cross_rational,
                tangent_cross_radical,
                angular_tangent_rational,
                angular_tangent_radical,
            ))
        })()
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };

        let Some(q_selected) = dense_last_axis_coefficient(&q, 0).and_then(|q| {
            q.remove_certified_independent_axis(
                sources.len(),
                hypersolve::PredicatePolicy::MAX_REFINEMENT_PRECISION,
            )
        }) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        match dense_polynomial_tuple_sign(&q_selected, &sources, &CurveContext::STRICT)? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a displaced endpoint retained a collapsed source chord".into(),
                ));
            }
            Classification::Decided(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a displaced endpoint retained negative source speed squared".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let build_system = || {
            let radius_squared = self.radial_distance() * self.radial_distance();
            let radical_expression =
                |rational: DenseTensorPolynomial, first: DenseTensorPolynomial| {
                    TwoSquareRootExpression::from_rational(rational)?
                        .add(&TwoSquareRootExpression::from_first_radical(first)?)?
                        .reduced(&sources)
                };
            // Homogenize B + M/sqrt(q) as (q*B + sqrt(q)*M)/q.
            // The same positive q scale is used for the diameter and r^2,
            // so the recursive angular authority retains one common scale.
            let point_x = radical_expression(
                dense_reduce_selected_root_relations(q.multiply(&point_base_x)?, &sources)?,
                displacement_x.clone(),
            )?;
            let point_y = radical_expression(
                dense_reduce_selected_root_relations(q.multiply(&point_base_y)?, &sources)?,
                displacement_y.clone(),
            )?;
            let center_x = TwoSquareRootExpression::from_rational(
                dense_reduce_selected_root_relations(q.multiply(&center_x)?, &sources)?,
            )?
            .reduced(&sources)?;
            let center_y = TwoSquareRootExpression::from_rational(
                dense_reduce_selected_root_relations(q.multiply(&center_y)?, &sources)?,
            )?
            .reduced(&sources)?;
            let incidence =
                radical_expression(incidence_rational.clone(), incidence_radical.clone())?;
            let selected_half_plane = radical_expression(
                dense_reduce_selected_root_relations(
                    q.multiply(&angular_cross_radical)?,
                    &sources,
                )?,
                angular_cross_rational.clone(),
            )?;
            let diameter = radical_expression(
                dense_reduce_selected_root_relations(q.multiply(&angular_dot_radical)?, &sources)?,
                angular_dot_rational.clone(),
            )?;
            let tangent_cross = radical_expression(
                tangent_cross_rational.clone(),
                tangent_cross_radical.clone(),
            )?;
            let angular_tangent = radical_expression(
                angular_tangent_rational.clone(),
                angular_tangent_radical.clone(),
            )?;
            let radius_squared_denominator = TwoSquareRootExpression::from_rational(
                dense_reduce_selected_root_relations(q.scale(&radius_squared)?, &sources)?,
            )?
            .reduced(&sources)?;
            let map = Arc::new(BezierChordNormalDenseMapSystem2 {
                source_representations: sources.clone(),
                first_speed_squared: q.clone(),
                second_speed_squared: constant(&Real::one())?,
                diameter,
                radius_squared_denominator,
            });
            Some(BezierChordNormalDenseIntersectionSystem2 {
                map,
                incidence,
                selected_half_plane,
                tangent_cross,
                angular_tangent: Some(angular_tangent),
                geometry: Some(BezierChordNormalDenseTargetGeometry2 {
                    point_x,
                    point_y,
                    center_x,
                    center_y,
                    common_denominator: q.clone(),
                }),
            })
        };
        let Some(recursive_system) = build_system() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        match self.recursive_quadratic_chord_intersections_from_dense_system(
            chord,
            &recursive_system,
            clip_to_finite_chord,
            certified_endpoint_incidence,
            policy,
        )? {
            Classification::Decided(Some(intersections)) => {
                return Ok(Classification::Decided(Some(intersections)));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let domain = if clip_to_finite_chord {
            SelectedThirdAxisDomain2::Finite(&CurveParameterRange2::unit())
        } else {
            SelectedThirdAxisDomain2::AffineLine
        };
        // Opposite endpoint signs certify exactly one transverse contact on a
        // finite affine chord.  Isolate that authored sheet directly, then
        // use a narrow interval to select its one simple root from the global
        // norm.  This avoids square-free isolation of every conjugate norm
        // root merely to discard all but one during replay.
        let mut direct_candidate = None;
        if clip_to_finite_chord {
            let expression_sign = |parameter: &Real| {
                let exact_parameter = AlgebraicRootRepresentation::from_exact_value(parameter);
                for refinement_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
                    let mut tuple = sources
                        .iter()
                        .map(|source| refined_represented_root(source, refinement_steps))
                        .collect::<Vec<_>>();
                    tuple.push(exact_parameter.clone());
                    let value = (|| {
                        let speed = dense_polynomial_value_interval(&q, &tuple)?
                            .nonnegative_square_root(None)?;
                        let rational =
                            dense_polynomial_value_interval(&incidence_rational, &tuple)?;
                        let radical = dense_polynomial_value_interval(&incidence_radical, &tuple)?;
                        Some(rational.add(&radical.multiply(&speed)?))
                    })();
                    if let Some(sign) = value.as_ref().and_then(dense_strict_interval_sign) {
                        return Ok(Classification::Decided(sign));
                    }
                }
                let mut tuple = sources.clone();
                tuple.push(exact_parameter);
                dense_positive_square_root_sum_sign(
                    &incidence_rational,
                    &incidence_radical,
                    &q,
                    &tuple,
                    &CurveContext::STRICT,
                )
            };
            let mut lower = Real::zero();
            let mut upper = Real::one();
            let mut lower_sign = match expression_sign(&lower)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let mut upper_sign = match expression_sign(&upper)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if lower_sign == RealSign::Zero {
                direct_candidate = Some(BezierParameter2::Exact(lower.clone()));
            } else if upper_sign == RealSign::Zero {
                direct_candidate = Some(BezierParameter2::Exact(upper.clone()));
            } else if lower_sign != upper_sign {
                // A binary64 solve is only a guide to a smaller dyadic
                // bracket.  Both chosen endpoints are replayed through the
                // exact expression below before the interval can become
                // construction evidence.
                let approximate_last_axis_coefficients =
                    |polynomial: &DenseTensorPolynomial| -> Option<Vec<f64>> {
                        let dimensions = polynomial.dimensions();
                        let target_count = *dimensions.last()?;
                        let source_values = sources
                            .iter()
                            .map(|source| refined_represented_root(source, 64))
                            .map(|source| {
                                Some(
                                    (source.interval.lower.to_f64_lossy()?
                                        + source.interval.upper.to_f64_lossy()?)
                                        * 0.5,
                                )
                            })
                            .collect::<Option<Vec<_>>>()?;
                        let mut result = vec![0.0_f64; target_count];
                        for (flat_index, coefficient) in
                            polynomial.coefficients().iter().enumerate()
                        {
                            let mut remaining = flat_index;
                            let mut exponents = vec![0_usize; dimensions.len()];
                            for axis in (0..dimensions.len()).rev() {
                                exponents[axis] = remaining % dimensions[axis];
                                remaining /= dimensions[axis];
                            }
                            let mut value = coefficient.to_f64_lossy()?;
                            for (source, exponent) in
                                source_values.iter().zip(&exponents[..sources.len()])
                            {
                                value *= source.powi((*exponent).try_into().ok()?);
                            }
                            result[exponents[sources.len()]] += value;
                        }
                        result
                            .iter()
                            .all(|value| value.is_finite())
                            .then_some(result)
                    };
                let approximate = (|| {
                    let rational = approximate_last_axis_coefficients(&incidence_rational)?;
                    let radical = approximate_last_axis_coefficients(&incidence_radical)?;
                    let speed_squared = approximate_last_axis_coefficients(&q)?;
                    let speed = speed_squared.first()?.sqrt();
                    speed.is_finite().then_some(())?;
                    let coefficient_count = rational.len().max(radical.len());
                    let coefficients = (0..coefficient_count)
                        .map(|index| {
                            rational.get(index).copied().unwrap_or(0.0)
                                + speed * radical.get(index).copied().unwrap_or(0.0)
                        })
                        .collect::<Vec<_>>();
                    let evaluate = |parameter: f64| {
                        coefficients
                            .iter()
                            .rev()
                            .fold(0.0_f64, |value, coefficient| {
                                value * parameter + coefficient
                            })
                    };
                    let mut approximate_lower = 0.0_f64;
                    let mut approximate_upper = 1.0_f64;
                    let mut approximate_lower_value = evaluate(approximate_lower);
                    let approximate_upper_value = evaluate(approximate_upper);
                    (approximate_lower_value.is_finite()
                        && approximate_upper_value.is_finite()
                        && approximate_lower_value.signum() != approximate_upper_value.signum())
                    .then_some(())?;
                    for _ in 0..40 {
                        let midpoint = (approximate_lower + approximate_upper) * 0.5;
                        let value = evaluate(midpoint);
                        value.is_finite().then_some(())?;
                        if value == 0.0 {
                            approximate_lower = midpoint;
                            approximate_upper = midpoint;
                            break;
                        }
                        if value.signum() == approximate_lower_value.signum() {
                            approximate_lower = midpoint;
                            approximate_lower_value = value;
                        } else {
                            approximate_upper = midpoint;
                        }
                    }
                    Some((approximate_lower + approximate_upper) * 0.5)
                })();
                if let Some(approximate) = approximate {
                    const GUIDE_BITS: u32 = 52;
                    let denominator_integer = 1_u128 << GUIDE_BITS;
                    let denominator = Real::from(denominator_integer);
                    let center = (approximate * denominator_integer as f64)
                        .round()
                        .clamp(0.0, denominator_integer as f64)
                        as u128;
                    for radius in [16_u128, 256, 4_096, 65_536, 1_048_576] {
                        let candidate_lower =
                            Real::from(center.saturating_sub(radius)) / &denominator;
                        let candidate_upper =
                            Real::from(center.saturating_add(radius).min(denominator_integer))
                                / &denominator;
                        let (Ok(candidate_lower), Ok(candidate_upper)) =
                            (candidate_lower, candidate_upper)
                        else {
                            continue;
                        };
                        let candidate_lower_sign = match expression_sign(&candidate_lower)? {
                            Classification::Decided(sign) => sign,
                            Classification::Uncertain(_) => continue,
                        };
                        let candidate_upper_sign = match expression_sign(&candidate_upper)? {
                            Classification::Decided(sign) => sign,
                            Classification::Uncertain(_) => continue,
                        };
                        if candidate_lower_sign != RealSign::Zero
                            && candidate_upper_sign != RealSign::Zero
                            && candidate_lower_sign != candidate_upper_sign
                        {
                            lower = candidate_lower;
                            upper = candidate_upper;
                            lower_sign = candidate_lower_sign;
                            upper_sign = candidate_upper_sign;
                            break;
                        }
                    }
                }
                let univariate = match selected_dense_last_axis_projection(&projection, &sources)? {
                    Classification::Decided(projection) => projection,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let coefficients = univariate.coefficients().to_vec();
                let derivative = polynomial_derivative(&coefficients);
                for refinement_step in 0..=256_usize {
                    if refinement_step >= 16 {
                        let projection_lower_sign = real_sign(
                            &Real::eval_poly(&coefficients, &lower),
                            &CurveContext::STRICT,
                        );
                        let projection_upper_sign = real_sign(
                            &Real::eval_poly(&coefficients, &upper),
                            &CurveContext::STRICT,
                        );
                        let parameter_interval = RealInterval {
                            lower: lower.clone(),
                            upper: upper.clone(),
                        };
                        let derivative_sign =
                            RealInterval::evaluate_power_basis(&derivative, &parameter_interval)
                                .as_ref()
                                .and_then(dense_strict_interval_sign);
                        if matches!(
                            (projection_lower_sign, projection_upper_sign),
                            (Some(RealSign::Negative), Some(RealSign::Positive))
                                | (Some(RealSign::Positive), Some(RealSign::Negative))
                        ) && matches!(
                            derivative_sign,
                            Some(RealSign::Negative | RealSign::Positive)
                        ) {
                            let interval = match BezierParameterInterval::try_new_with_policy(
                                lower.clone(),
                                upper.clone(),
                                &CurveContext::STRICT,
                            )? {
                                Classification::Decided(interval) => interval,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            };
                            let Some(parameter) =
                                BezierAlgebraicParameter2::from_certified_simple_power_basis(
                                    coefficients.clone(),
                                    interval,
                                )
                            else {
                                return Ok(Classification::Uncertain(
                                    UncertaintyReason::Unsupported,
                                ));
                            };
                            direct_candidate = Some(BezierParameter2::Algebraic(parameter));
                            break;
                        }
                    }
                    let midpoint = ((&lower + &upper) / Real::from(2_i8))?;
                    match expression_sign(&midpoint)? {
                        Classification::Decided(RealSign::Zero) => {
                            direct_candidate = Some(BezierParameter2::Exact(midpoint));
                            break;
                        }
                        Classification::Decided(sign) if sign == lower_sign => {
                            lower = midpoint;
                            lower_sign = sign;
                        }
                        Classification::Decided(sign) if sign == upper_sign => {
                            upper = midpoint;
                            upper_sign = sign;
                        }
                        Classification::Decided(_) => {
                            return Err(CurveError::Topology(
                                "a one-radical chord incidence lost its endpoint sign bracket"
                                    .into(),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
        }
        let direct_projection_selected = direct_candidate.is_some();
        let candidates = if let Some(candidate) = direct_candidate {
            vec![candidate]
        } else {
            match selected_dense_last_axis_parameters(&projection, &sources, domain, policy)? {
                Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                    candidates,
                )) => candidates,
                Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
                    return Err(CurveError::Topology(
                        "a nonzero circle contained a displaced affine chord component".into(),
                    ));
                }
                Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let Some(system) = build_system() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };

        // A direct candidate is already the unique simple norm root in an
        // exact opposite-sign bracket of the authored equation. Other norm
        // candidates must still replay the selected positive speed sheet.
        let mut authored_candidates = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            if !direct_projection_selected {
                let mut selected_tuple = sources.clone();
                selected_tuple.push(bezier_parameter_root_representation(&candidate));
                match dense_positive_square_root_sum_sign(
                    &incidence_rational,
                    &incidence_radical,
                    &q,
                    &selected_tuple,
                    &CurveContext::STRICT,
                )? {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(RealSign::Negative | RealSign::Positive) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            authored_candidates.push(candidate);
        }
        Ok(self
            .chord_normal_projective_chord_intersections_from_candidates(
                chord,
                system,
                authored_candidates,
                clip_to_finite_chord,
                policy,
            )?
            .map(Some))
    }

    /// Exact circle incidence against a finite procedural parallel whose trim
    /// endpoints need not have standalone Cartesian fields.
    ///
    /// The root retained support supplies an oriented direction `D` and its
    /// positive speed `s`.  Writing the displaced line origin as the
    /// projective point `S/s` keeps both normal displacement and circle
    /// incidence polynomial in the selected tuple. Contacts are retained by
    /// the same represented circle/chord map used by the general fallback;
    /// only finite clipping replays the chord's own parameter authority.
    pub(in crate::bezier_offset) fn represented_parallel_support_chord_intersections_in_domain(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>>>
    {
        chord.validate_policy(policy)?;
        let Some(structural) = chord_parallel_support_source(chord, policy)? else {
            return Ok(Classification::Decided(None));
        };
        let traversal_reversed = match chord.start() {
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)) => point.at_end,
            _ => return Ok(Classification::Decided(None)),
        };
        // The target axis below is the source-support parameter retained by a
        // full procedural endpoint pair.  When the current finite chord is
        // exactly that pair, `[0, 1]` is already its complete domain (possibly
        // with reversed traversal); no Cartesian point replay is needed.
        let finite_parameter_reversed = chord.procedural_parallel_parameter_reversed();
        let source_support = structural.source.retained_support();
        let [source_start, source_end] =
            if structural.source.retained_support_orientation_is_reversed() {
                [source_support.end(), source_support.start()]
            } else {
                [source_support.start(), source_support.end()]
            };
        let source_start = match represented_point_evidence_coordinates(source_start, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let source_end = match represented_point_evidence_coordinates(source_end, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let frame = match self.represented_circle_frame(policy)? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let BezierChordParallelSupportSource2 {
            distance,
            translation_x,
            translation_y,
            direction,
            ..
        } = structural;
        let incidence_represented = source_start
            .iter()
            .cloned()
            .chain(source_end.iter().cloned())
            .chain(frame.center.iter().cloned())
            .collect::<Vec<_>>();
        let Some((incidence_sources, incidence_coordinates)) =
            represented_affine_tensor_basis(&incidence_represented)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let [
            incidence_start_x,
            incidence_start_y,
            incidence_end_x,
            incidence_end_y,
            incidence_center_x,
            incidence_center_y,
        ]: [DenseTensorPolynomial; 6] = incidence_coordinates
            .try_into()
            .expect("a represented parallel incidence retains six coordinates");
        let incidence_rank = incidence_sources.len() + 1;
        let incidence_target_axis = incidence_sources.len();
        let incidence_constant = |value: &Real| {
            DenseTensorPolynomial::from_axis_polynomial(
                incidence_rank,
                0,
                std::slice::from_ref(value),
            )
        };
        let incidence_axis = |coefficients: &[Real]| {
            DenseTensorPolynomial::from_axis_polynomial(
                incidence_rank,
                incidence_target_axis,
                coefficients,
            )
        };
        let Some((
            candidate_q,
            support_discriminant_rational,
            support_discriminant_radical,
            candidate_rational,
            candidate_radical,
            candidate_projection,
        )) = (|| {
            let reduce =
                |polynomial| dense_reduce_selected_root_relations(polynomial, &incidence_sources);
            let dx = reduce(incidence_end_x.subtract(&incidence_start_x)?)?;
            let dy = reduce(incidence_end_y.subtract(&incidence_start_y)?)?;
            let q = reduce(dx.multiply(&dx)?.add(&dy.multiply(&dy)?)?)?;
            let (unit_x, unit_y) = match direction {
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => {
                    (dy.scale(&Real::from(-1_i8))?, dx.clone())
                }
                BezierAlgebraicChordUnitDisplacement2::Tangent => (dx.clone(), dy.clone()),
            };
            let displacement_x = reduce(unit_x.scale(&distance)?)?;
            let displacement_y = reduce(unit_y.scale(&distance)?)?;
            let translated_start_x =
                reduce(incidence_start_x.add(&incidence_constant(&translation_x)?)?)?;
            let translated_start_y =
                reduce(incidence_start_y.add(&incidence_constant(&translation_y)?)?)?;
            let origin_radial_x = reduce(translated_start_x.subtract(&incidence_center_x)?)?;
            let origin_radial_y = reduce(translated_start_y.subtract(&incidence_center_y)?)?;
            let support_cross = reduce(
                origin_radial_x
                    .multiply(&dy)?
                    .subtract(&origin_radial_y.multiply(&dx)?)?,
            )?;
            let support_cross_squared = reduce(support_cross.multiply(&support_cross)?)?;
            let radius_squared = self.radial_distance() * self.radial_distance();
            let adjusted_radius_squared = match direction {
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => {
                    radius_squared.clone() - distance.clone() * distance.clone()
                }
                BezierAlgebraicChordUnitDisplacement2::Tangent => radius_squared.clone(),
            };
            let support_discriminant_inner = reduce(
                q.scale(&adjusted_radius_squared)?
                    .subtract(&support_cross_squared)?,
            )?;
            let support_discriminant_rational = reduce(q.multiply(&support_discriminant_inner)?)?;
            let support_discriminant_radical = match direction {
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => reduce(
                    q.multiply(&support_cross)?
                        .scale(&(Real::from(2_i8) * distance.clone()))?,
                )?,
                BezierAlgebraicChordUnitDisplacement2::Tangent => {
                    incidence_constant(&Real::zero())?
                }
            };
            let parameter = incidence_axis(&[Real::zero(), Real::one()])?;
            let point_x = reduce(translated_start_x.add(&dx.multiply(&parameter)?)?)?;
            let point_y = reduce(translated_start_y.add(&dy.multiply(&parameter)?)?)?;
            let radial_x = reduce(point_x.subtract(&incidence_center_x)?)?;
            let radial_y = reduce(point_y.subtract(&incidence_center_y)?)?;
            let radial_squared = reduce(
                radial_x
                    .multiply(&radial_x)?
                    .add(&radial_y.multiply(&radial_y)?)?
                    .subtract(&incidence_constant(&radius_squared)?)?,
            )?;
            let displacement_squared = reduce(
                displacement_x
                    .multiply(&displacement_x)?
                    .add(&displacement_y.multiply(&displacement_y)?)?,
            )?;
            let rational = reduce(q.multiply(&radial_squared)?.add(&displacement_squared)?)?;
            let radical = reduce(
                radial_x
                    .multiply(&displacement_x)?
                    .add(&radial_y.multiply(&displacement_y)?)?
                    .scale(&Real::from(2_i8))?,
            )?;
            let projection = reduce(
                rational
                    .multiply(&rational)?
                    .subtract(&q.multiply(&radical.multiply(&radical)?)?)?,
            )?;
            Some((
                q,
                support_discriminant_rational,
                support_discriminant_radical,
                rational,
                radical,
                projection,
            ))
        })()
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let mut candidate_q_sources = incidence_sources.clone();
        candidate_q_sources.push(AlgebraicRootRepresentation::from_exact_value(&Real::zero()));
        match dense_polynomial_tuple_sign(
            &candidate_q,
            &candidate_q_sources,
            &CurveContext::STRICT,
        )? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a represented parallel retained a collapsed source support".into(),
                ));
            }
            Classification::Decided(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a represented parallel retained negative source speed squared".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let support_discriminant_sign = dense_positive_square_root_sum_sign(
            &support_discriminant_rational,
            &support_discriminant_radical,
            &candidate_q,
            &candidate_q_sources,
            &CurveContext::STRICT,
        )?;
        match support_discriminant_sign {
            Classification::Decided(RealSign::Negative) => {
                return Ok(Classification::Decided(Some(Vec::new())));
            }
            Classification::Decided(RealSign::Zero | RealSign::Positive) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let guided_candidates = selected_dense_guided_quadratic_parameters(
            &candidate_rational,
            &candidate_radical,
            &candidate_q,
            &candidate_projection,
            &incidence_sources,
        )?;
        let candidates = match guided_candidates {
            Classification::Decided(Some(candidates)) => candidates,
            Classification::Decided(None) | Classification::Uncertain(_) => {
                match selected_dense_last_axis_parameters(
                    &candidate_projection,
                    &incidence_sources,
                    SelectedThirdAxisDomain2::AffineLine,
                    policy,
                )? {
                    Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                        candidates,
                    )) => candidates,
                    Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
                        return Err(CurveError::Topology(
                            "a nonzero circle contained a represented parallel support component"
                                .into(),
                        ));
                    }
                    Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        };
        let mut authored_candidates = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            match dense_positive_square_root_transverse_root(
                &candidate_rational,
                &candidate_radical,
                &candidate_q,
                &incidence_sources,
                &candidate,
            ) {
                Some(true) => authored_candidates.push(candidate),
                Some(false) => {}
                None => {
                    let mut selected_tuple = incidence_sources.clone();
                    selected_tuple.push(bezier_parameter_root_representation(&candidate));
                    match dense_positive_square_root_sum_sign(
                        &candidate_rational,
                        &candidate_radical,
                        &candidate_q,
                        &selected_tuple,
                        &CurveContext::STRICT,
                    )? {
                        Classification::Decided(RealSign::Zero) => {
                            authored_candidates.push(candidate);
                        }
                        Classification::Decided(RealSign::Negative | RealSign::Positive) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
        }
        let represented = source_start
            .into_iter()
            .chain(source_end)
            .chain(frame.center.iter().cloned())
            .chain(frame.unit_radial.iter().cloned())
            .collect::<Vec<_>>();
        let Some((sources, coordinates)) = represented_affine_tensor_basis(&represented) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let [
            source_start_x,
            source_start_y,
            source_end_x,
            source_end_y,
            center_x,
            center_y,
            unit_radial_x,
            unit_radial_y,
        ]: [DenseTensorPolynomial; 8] = coordinates
            .try_into()
            .expect("a represented parallel support retains eight frame coordinates");
        let rank = sources.len() + 1;
        let target_axis = sources.len();
        let constant = |value: &Real| {
            DenseTensorPolynomial::from_axis_polynomial(rank, 0, std::slice::from_ref(value))
        };
        let axis = |coefficients: &[Real]| {
            DenseTensorPolynomial::from_axis_polynomial(rank, target_axis, coefficients)
        };
        let Some((
            q,
            point_base_x,
            point_base_y,
            displacement_x,
            displacement_y,
            incidence_rational,
            incidence_radical,
            angular_dot_rational,
            angular_dot_radical,
            angular_cross_rational,
            angular_cross_radical,
            tangent_cross_rational,
            tangent_cross_radical,
            angular_tangent_rational,
            angular_tangent_radical,
        )) = (|| {
            let reduce = |polynomial| dense_reduce_selected_root_relations(polynomial, &sources);
            let dx = reduce(source_end_x.subtract(&source_start_x)?)?;
            let dy = reduce(source_end_y.subtract(&source_start_y)?)?;
            let q = reduce(dx.multiply(&dx)?.add(&dy.multiply(&dy)?)?)?;
            let (unit_x, unit_y) = match direction {
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => {
                    (dy.scale(&Real::from(-1_i8))?, dx.clone())
                }
                BezierAlgebraicChordUnitDisplacement2::Tangent => (dx.clone(), dy.clone()),
            };
            let displacement_x = reduce(unit_x.scale(&distance)?)?;
            let displacement_y = reduce(unit_y.scale(&distance)?)?;
            let translated_start_x = reduce(source_start_x.add(&constant(&translation_x)?)?)?;
            let translated_start_y = reduce(source_start_y.add(&constant(&translation_y)?)?)?;
            let parameter = axis(&[Real::zero(), Real::one()])?;
            let point_base_x = reduce(translated_start_x.add(&dx.multiply(&parameter)?)?)?;
            let point_base_y = reduce(translated_start_y.add(&dy.multiply(&parameter)?)?)?;
            let radial_base_x = reduce(point_base_x.subtract(&center_x)?)?;
            let radial_base_y = reduce(point_base_y.subtract(&center_y)?)?;
            let radius_squared = self.radial_distance() * self.radial_distance();
            let radial_base_squared = reduce(
                radial_base_x
                    .multiply(&radial_base_x)?
                    .add(&radial_base_y.multiply(&radial_base_y)?)?
                    .subtract(&constant(&radius_squared)?)?,
            )?;
            let displacement_squared = reduce(
                displacement_x
                    .multiply(&displacement_x)?
                    .add(&displacement_y.multiply(&displacement_y)?)?,
            )?;
            let incidence_rational = reduce(
                q.multiply(&radial_base_squared)?
                    .add(&displacement_squared)?,
            )?;
            let incidence_radical = reduce(
                radial_base_x
                    .multiply(&displacement_x)?
                    .add(&radial_base_y.multiply(&displacement_y)?)?
                    .scale(&Real::from(2_i8))?,
            )?;

            let signed_radius = frame.signed_radius.clone();
            let angular_scale = &signed_radius * self.turn_sign();
            let angular_dot_rational = reduce(
                unit_radial_x
                    .multiply(&displacement_x)?
                    .add(&unit_radial_y.multiply(&displacement_y)?)?
                    .scale(&signed_radius)?,
            )?;
            let angular_dot_radical = reduce(
                unit_radial_x
                    .multiply(&radial_base_x)?
                    .add(&unit_radial_y.multiply(&radial_base_y)?)?
                    .scale(&signed_radius)?,
            )?;
            let angular_cross_rational = reduce(
                unit_radial_x
                    .multiply(&displacement_y)?
                    .subtract(&unit_radial_y.multiply(&displacement_x)?)?
                    .scale(&angular_scale)?,
            )?;
            let angular_cross_radical = reduce(
                unit_radial_x
                    .multiply(&radial_base_y)?
                    .subtract(&unit_radial_y.multiply(&radial_base_x)?)?
                    .scale(&angular_scale)?,
            )?;

            let traversal_scale = if traversal_reversed {
                Real::from(-1_i8)
            } else {
                Real::one()
            };
            let tangent_dx = dx.scale(&traversal_scale)?;
            let tangent_dy = dy.scale(&traversal_scale)?;
            let tangent_cross_scale = -self.turn_sign();
            let tangent_cross_rational = reduce(
                q.multiply(
                    &radial_base_x
                        .multiply(&tangent_dx)?
                        .add(&radial_base_y.multiply(&tangent_dy)?)?,
                )?
                .scale(&tangent_cross_scale)?,
            )?;
            let tangent_cross_radical = reduce(
                displacement_x
                    .multiply(&tangent_dx)?
                    .add(&displacement_y.multiply(&tangent_dy)?)?
                    .scale(&tangent_cross_scale)?,
            )?;
            let angular_tangent_rational = reduce(
                q.multiply(
                    &radial_base_x
                        .multiply(&tangent_dy)?
                        .subtract(&radial_base_y.multiply(&tangent_dx)?)?,
                )?,
            )?;
            let angular_tangent_radical = reduce(
                displacement_x
                    .multiply(&tangent_dy)?
                    .subtract(&displacement_y.multiply(&tangent_dx)?)?,
            )?;
            Some((
                q,
                point_base_x,
                point_base_y,
                displacement_x,
                displacement_y,
                incidence_rational,
                incidence_radical,
                angular_dot_rational,
                angular_dot_radical,
                angular_cross_rational,
                angular_cross_radical,
                tangent_cross_rational,
                tangent_cross_radical,
                angular_tangent_rational,
                angular_tangent_radical,
            ))
        })()
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let radius_squared = self.radial_distance() * self.radial_distance();
        let radical_expression = |rational: DenseTensorPolynomial, first: DenseTensorPolynomial| {
            TwoSquareRootExpression::from_rational(rational)?
                .add(&TwoSquareRootExpression::from_first_radical(first)?)?
                .reduced(&sources)
        };
        let Some(system) = (|| {
            let point_x = radical_expression(
                dense_reduce_selected_root_relations(q.multiply(&point_base_x)?, &sources)?,
                displacement_x.clone(),
            )?;
            let point_y = radical_expression(
                dense_reduce_selected_root_relations(q.multiply(&point_base_y)?, &sources)?,
                displacement_y.clone(),
            )?;
            let center_x = TwoSquareRootExpression::from_rational(
                dense_reduce_selected_root_relations(q.multiply(&center_x)?, &sources)?,
            )?
            .reduced(&sources)?;
            let center_y = TwoSquareRootExpression::from_rational(
                dense_reduce_selected_root_relations(q.multiply(&center_y)?, &sources)?,
            )?
            .reduced(&sources)?;
            let incidence =
                radical_expression(incidence_rational.clone(), incidence_radical.clone())?;
            let selected_half_plane = radical_expression(
                dense_reduce_selected_root_relations(
                    q.multiply(&angular_cross_radical)?,
                    &sources,
                )?,
                angular_cross_rational.clone(),
            )?;
            let diameter = radical_expression(
                dense_reduce_selected_root_relations(q.multiply(&angular_dot_radical)?, &sources)?,
                angular_dot_rational.clone(),
            )?;
            let tangent_cross = radical_expression(
                tangent_cross_rational.clone(),
                tangent_cross_radical.clone(),
            )?;
            let angular_tangent = radical_expression(
                angular_tangent_rational.clone(),
                angular_tangent_radical.clone(),
            )?;
            let radius_squared_denominator = TwoSquareRootExpression::from_rational(
                dense_reduce_selected_root_relations(q.scale(&radius_squared)?, &sources)?,
            )?
            .reduced(&sources)?;
            let map = Arc::new(BezierChordNormalDenseMapSystem2 {
                source_representations: sources.clone(),
                first_speed_squared: q.clone(),
                second_speed_squared: constant(&Real::one())?,
                diameter,
                radius_squared_denominator,
            });
            Some(BezierChordNormalDenseIntersectionSystem2 {
                map,
                incidence,
                selected_half_plane,
                tangent_cross,
                angular_tangent: Some(angular_tangent),
                geometry: Some(BezierChordNormalDenseTargetGeometry2 {
                    point_x,
                    point_y,
                    center_x,
                    center_y,
                    common_denominator: q.clone(),
                }),
            })
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let intersections = match self.chord_normal_projective_chord_intersections_from_candidates(
            chord,
            system,
            authored_candidates,
            clip_to_finite_chord && finite_parameter_reversed.is_some(),
            policy,
        )? {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let BezierAlgebraicCuspSemicircleChordIntersections2::Contacts {
            contacts,
            parameter_map,
        } = intersections
        else {
            return Ok(Classification::Decided(Some(Vec::new())));
        };
        let mut retained = Vec::with_capacity(contacts.len());
        for contact in contacts {
            let (cusp_parameter, point) = parameter_map.contact_evidence(&contact);
            let chord_parameter = if clip_to_finite_chord {
                if let Some(reversed) = finite_parameter_reversed {
                    match contact.chord_location {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start => {
                            if reversed {
                                chord.end_parameter()
                            } else {
                                chord.start_parameter()
                            }
                        }
                        BezierAlgebraicCuspSemicircleContactLocation2::End => {
                            if reversed {
                                chord.start_parameter()
                            } else {
                                chord.end_parameter()
                            }
                        }
                        BezierAlgebraicCuspSemicircleContactLocation2::Interior => {
                            chord.parameter_at_certified_interior_point(point)
                        }
                    }
                } else {
                    match chord.parameter_at_certified_point(point, policy)? {
                        Classification::Decided(Some(parameter)) => parameter,
                        Classification::Decided(None) => continue,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            } else {
                chord.parameter_at_certified_support_point(point, policy)?
            };
            let point = chord_parameter.point().clone();
            retained.push(BezierAlgebraicCuspSemicircleRetainedChordContact2 {
                cusp_parameter,
                chord_parameter,
                point,
                tangent_cross_sign: contact.tangent_cross_sign,
            });
        }
        Ok(Classification::Decided(Some(retained)))
    }

    /// Complete rank-independent line/circle fallback for retained chord
    /// endpoints which cannot enter one of the compact procedural systems.
    ///
    /// Every endpoint and center coordinate is already a selected exact
    /// algebraic scalar.  We keep that tuple in one dense tensor through the
    /// quadratic line solve, select each square-root branch under STRICT, and
    /// only then compare the resulting contact with the independently
    /// represented start radial.  This covers, among other combinations,
    /// bevels joining a normal-displaced endpoint to a two-support
    /// intersection point without carrying the unused start-radial axes
    /// through the line solve.
    pub(in crate::bezier_offset) fn represented_oblique_chord_intersections_in_domain(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        certified_endpoint_incidence: Option<BezierCertifiedFiniteChordEndpointIncidence2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleChordIntersections2>> {
        chord.validate_policy(policy)?;
        #[cfg(feature = "dispatch-trace")]
        {
            let point_kind = |point: &CurvePoint2| match point {
                CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "chord-pair",
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp-chord",
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "cusp-chord-derived",
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "chord-parallel",
                CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic-parallel",
                CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    "similarity"
                }
            };
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "represented-circle-chord-start-kind",
                point_kind(chord.start()),
            );
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "represented-circle-chord-end-kind",
                point_kind(chord.end()),
            );
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "represented-circle-chord-frame-kind",
                match &self.data.frame {
                    BezierSelectedCircleFrame2::Rational(_) => "rational",
                    BezierSelectedCircleFrame2::ParallelNormal(_) => "parallel-normal",
                    BezierSelectedCircleFrame2::ChordNormal(_) => "chord-normal",
                    BezierSelectedCircleFrame2::SelectedRadial(_) => "selected-radial",
                },
            );
        }
        // Most procedural bevels are merely near a selected circle, not
        // incident with it.  Prove a negative analytic discriminant directly
        // from exact refining endpoint boxes before asking Hypersolve to
        // materialize either unit-displaced endpoint.  Interval dependency can
        // only delay this rejection; it cannot create a false negative.
        if let Classification::Decided(center) = self.center_point_evidence(policy)? {
            let radius_squared = self.radial_distance() * self.radial_distance();
            let radius_squared = RealInterval {
                lower: radius_squared.clone(),
                upper: radius_squared,
            };
            let mut start_bounds =
                AlgebraicChordEndpointBoundsRefinement2::new(chord.start(), policy);
            let mut end_bounds = AlgebraicChordEndpointBoundsRefinement2::new(chord.end(), policy);
            let mut center_bounds = AlgebraicChordEndpointBoundsRefinement2::new(&center, policy);
            for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
                let start_bounds = start_bounds.refine_to(refinement_steps);
                let end_bounds = end_bounds.refine_to(refinement_steps);
                let center_bounds = center_bounds.refine_to(refinement_steps);
                let (
                    Classification::Decided(start_bounds),
                    Classification::Decided(end_bounds),
                    Classification::Decided(center_bounds),
                ) = (start_bounds, end_bounds, center_bounds)
                else {
                    continue;
                };
                let start = [
                    real_interval_from_axis(&start_bounds, Axis2::X),
                    real_interval_from_axis(&start_bounds, Axis2::Y),
                ];
                let end = [
                    real_interval_from_axis(&end_bounds, Axis2::X),
                    real_interval_from_axis(&end_bounds, Axis2::Y),
                ];
                let center = [
                    real_interval_from_axis(&center_bounds, Axis2::X),
                    real_interval_from_axis(&center_bounds, Axis2::Y),
                ];
                let direction = [end[0].subtract(&start[0]), end[1].subtract(&start[1])];
                let radial = [start[0].subtract(&center[0]), start[1].subtract(&center[1])];
                let end_radial = [end[0].subtract(&center[0]), end[1].subtract(&center[1])];
                let Some((projection, direction_squared, radial_squared)) = (|| {
                    let projection = radial[0]
                        .multiply(&direction[0])?
                        .add(&radial[1].multiply(&direction[1])?);
                    let direction_squared = direction[0]
                        .multiply(&direction[0])?
                        .add(&direction[1].multiply(&direction[1])?);
                    let radial_squared = radial[0]
                        .multiply(&radial[0])?
                        .add(&radial[1].multiply(&radial[1])?);
                    Some((projection, direction_squared, radial_squared))
                })() else {
                    continue;
                };
                if clip_to_finite_chord {
                    let Some([start_incidence, middle_incidence, end_incidence]) = (|| {
                        let dot = |first: &[RealInterval; 2], second: &[RealInterval; 2]| {
                            Some(
                                first[0]
                                    .multiply(&second[0])?
                                    .add(&first[1].multiply(&second[1])?),
                            )
                        };
                        Some([
                            radial_squared.subtract(&radius_squared),
                            dot(&radial, &end_radial)?.subtract(&radius_squared),
                            dot(&end_radial, &end_radial)?.subtract(&radius_squared),
                        ])
                    })(
                    ) else {
                        continue;
                    };
                    let positive = [&start_incidence, &middle_incidence, &end_incidence]
                        .into_iter()
                        .all(|coefficient| {
                            compare_reals(&coefficient.lower, &Real::zero(), &CurveContext::STRICT)
                                == Some(std::cmp::Ordering::Greater)
                        });
                    let negative = [&start_incidence, &middle_incidence, &end_incidence]
                        .into_iter()
                        .all(|coefficient| {
                            compare_reals(&coefficient.upper, &Real::zero(), &CurveContext::STRICT)
                                == Some(std::cmp::Ordering::Less)
                        });
                    if positive || negative {
                        return Ok(Classification::Decided(
                            BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
                        ));
                    }
                }
                let Some(discriminant) =
                    projection
                        .multiply(&projection)
                        .and_then(|projection_squared| {
                            direction_squared
                                .multiply(&radial_squared.subtract(&radius_squared))
                                .map(|residual| projection_squared.subtract(&residual))
                        })
                else {
                    continue;
                };
                if compare_reals(&discriminant.upper, &Real::zero(), &CurveContext::STRICT)
                    == Some(std::cmp::Ordering::Less)
                {
                    return Ok(Classification::Decided(
                        BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
                    ));
                }
            }
        }
        match self.represented_parallel_endpoint_oblique_chord_intersections_in_domain(
            chord,
            clip_to_finite_chord,
            certified_endpoint_incidence,
            policy,
        )? {
            Classification::Decided(Some(intersections)) => {
                return Ok(Classification::Decided(intersections));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(_) => {}
        }
        let start = match represented_point_evidence_coordinates(chord.start(), policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-circle-chord-blocker",
                    "start-coordinate",
                );
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match represented_point_evidence_coordinates(chord.end(), policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-circle-chord-blocker",
                    "end-coordinate",
                );
                return Ok(Classification::Uncertain(reason));
            }
        };
        let frame = match self.represented_circle_frame(policy)? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-circle-chord-blocker",
                    "circle-frame",
                );
                return Ok(Classification::Uncertain(reason));
            }
        };
        let represented = [
            start[0].clone(),
            start[1].clone(),
            end[0].clone(),
            end[1].clone(),
            frame.center[0].clone(),
            frame.center[1].clone(),
        ];
        let Some((sources, coordinates)) = represented_affine_tensor_basis(&represented) else {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "represented-circle-chord-blocker",
                "tensor-basis",
            );
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let [start_x, start_y, end_x, end_y, center_x, center_y]: [DenseTensorPolynomial; 6] =
            coordinates
                .try_into()
                .expect("a represented circle/chord basis retains six incidence coordinates");
        let rank = sources.len() + 1;
        let constant = |value: &Real| {
            DenseTensorPolynomial::from_axis_polynomial(rank, 0, std::slice::from_ref(value))
        };
        let Some((
            dx,
            dy,
            direction_squared,
            center_projection,
            discriminant,
            point_retained_x,
            point_retained_y,
            tangent_dot,
            zero,
            one,
        )) = (|| {
            let reduce = |polynomial| dense_reduce_selected_root_relations(polynomial, &sources);
            let dx = reduce(end_x.subtract(&start_x)?)?;
            let dy = reduce(end_y.subtract(&start_y)?)?;
            let vx = reduce(start_x.subtract(&center_x)?)?;
            let vy = reduce(start_y.subtract(&center_y)?)?;
            let direction_squared = reduce(dx.multiply(&dx)?.add(&dy.multiply(&dy)?)?)?;
            let center_projection = reduce(vx.multiply(&dx)?.add(&vy.multiply(&dy)?)?)?;
            let radius_squared = self.radial_distance() * self.radial_distance();
            let radial_residual = reduce(
                vx.multiply(&vx)?
                    .add(&vy.multiply(&vy)?)?
                    .subtract(&constant(&radius_squared)?)?,
            )?;
            let discriminant = reduce(
                center_projection
                    .multiply(&center_projection)?
                    .subtract(&direction_squared.multiply(&radial_residual)?)?,
            )?;
            let point_retained_x = reduce(
                start_x
                    .multiply(&direction_squared)?
                    .subtract(&dx.multiply(&center_projection)?)?,
            )?;
            let point_retained_y = reduce(
                start_y
                    .multiply(&direction_squared)?
                    .subtract(&dy.multiply(&center_projection)?)?,
            )?;
            // T_circle = turn*J(P-C), hence
            // dot(T_circle, D) = turn*cross(P-C,D) = turn*cross(V,D).
            let tangent_dot = reduce(
                vx.multiply(&dy)?
                    .subtract(&vy.multiply(&dx)?)?
                    .scale(&self.turn_sign())?,
            )?;
            Some((
                dx,
                dy,
                direction_squared,
                center_projection,
                discriminant,
                point_retained_x,
                point_retained_y,
                tangent_dot,
                constant(&Real::zero())?,
                constant(&Real::one())?,
            ))
        })()
        else {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "represented-circle-chord-blocker",
                "polynomial-budget",
            );
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let direction_squared_value =
            match represented_dense_value_refined(&direction_squared, &sources) {
                Classification::Decided(value) => value,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        match represented_strict_sign(&direction_squared_value) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a retained affine chord collapsed during represented circle incidence".into(),
                ));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a represented chord direction had negative squared length".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::Predicate)),
        }
        let discriminant_value = match represented_dense_value_refined(&discriminant, &sources) {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let branches: &[i8] = match represented_strict_sign(&discriminant_value) {
            Some(RealSign::Negative) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
                ));
            }
            Some(RealSign::Zero) => &[0],
            Some(RealSign::Positive) => &[-1, 1],
            None => return Ok(Classification::Uncertain(UncertaintyReason::Predicate)),
        };
        let tangent_dot = match represented_dense_value_refined(&tangent_dot, &sources) {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let chord_normal_angular_system = if let Some(chord_normal) = self.data.frame.chord_normal()
        {
            match represented_chord_normal_line_angular_system(
                &chord_normal.anchor,
                &start,
                &end,
                &frame.center,
                &frame.signed_radius,
                &self.turn_sign(),
                policy,
            )? {
                Classification::Decided(system) => Some(system),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        } else {
            None
        };
        let start_radial = if chord_normal_angular_system.is_none() {
            Some([
                match represented_affine_coordinate(
                    &[(&frame.unit_radial[0], &frame.signed_radius)],
                    &Real::zero(),
                ) {
                    Classification::Decided(value) => value,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
                match represented_affine_coordinate(
                    &[(&frame.unit_radial[1], &frame.signed_radius)],
                    &Real::zero(),
                ) {
                    Classification::Decided(value) => value,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            ])
        } else {
            None
        };
        let radius_squared = self.radial_distance() * self.radial_distance();
        let parameter_difference_sign = |parameter: &AlgebraicRootRepresentation, value: &Real| {
            match represented_affine_coordinate(&[(parameter, &Real::one())], &(-value)) {
                Classification::Decided(difference) => match represented_strict_sign(&difference) {
                    Some(sign) => Classification::Decided(sign),
                    None => Classification::Uncertain(UncertaintyReason::Predicate),
                },
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            }
        };
        let mut contacts = Vec::with_capacity(branches.len());
        let mut retained_contacts = Vec::with_capacity(branches.len());
        for &branch in branches {
            let radical = square_root_algebraic_root_representation(&discriminant_value, branch);
            let signed_radical = match radical.status {
                AlgebraicRootSquareRootStatus::Transformed => radical
                    .representation
                    .expect("a represented chord contact retains its selected radical"),
                AlgebraicRootSquareRootStatus::UndecidedSign => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                }
                AlgebraicRootSquareRootStatus::InvalidEvidence
                | AlgebraicRootSquareRootStatus::InvalidBranch
                | AlgebraicRootSquareRootStatus::NegativeRadicand
                | AlgebraicRootSquareRootStatus::NonzeroZeroBranch
                | AlgebraicRootSquareRootStatus::InvalidTransformedEvidence => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
            };
            let negative_projection = center_projection
                .scale(&Real::from(-1_i8))
                .expect("a represented line projection remains in its tensor budget");
            let chord_parameter = match represented_tensor_nested_ratio(
                &negative_projection,
                &one,
                &direction_squared,
                &zero,
                &discriminant,
                &sources,
                &signed_radical,
            ) {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let coordinate = |retained: &DenseTensorPolynomial,
                              candidate: &DenseTensorPolynomial| {
                represented_tensor_nested_ratio(
                    retained,
                    candidate,
                    &direction_squared,
                    &zero,
                    &discriminant,
                    &sources,
                    &signed_radical,
                )
            };
            let point = match (
                coordinate(&point_retained_x, &dx),
                coordinate(&point_retained_y, &dy),
            ) {
                (Classification::Decided(x), Classification::Decided(y)) => [x, y],
                (Classification::Uncertain(UncertaintyReason::Unsupported), _)
                | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                _ => return Ok(Classification::Uncertain(UncertaintyReason::Predicate)),
            }
            .map(|coordinate| {
                hypersolve::compact_algebraic_root_low_degree_witness(&coordinate)
                    .unwrap_or(coordinate)
            });
            let angular = if let Some(system) = chord_normal_angular_system.as_ref() {
                system.contact_location_parameter(branch)
            } else {
                let radial = [
                    match represented_affine_coordinate(
                        &[
                            (&point[0], &Real::one()),
                            (&frame.center[0], &Real::from(-1_i8)),
                        ],
                        &Real::zero(),
                    ) {
                        Classification::Decided(value) => value,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    },
                    match represented_affine_coordinate(
                        &[
                            (&point[1], &Real::one()),
                            (&frame.center[1], &Real::from(-1_i8)),
                        ],
                        &Real::zero(),
                    ) {
                        Classification::Decided(value) => value,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    },
                ];
                let [dot, cross] = match represented_vector_dot_cross(
                    start_radial
                        .as_ref()
                        .expect("a non-chord-normal frame retains its start radial"),
                    &radial,
                ) {
                    Classification::Decided(values) => values,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let oriented_cross = match represented_affine_coordinate(
                    &[(&cross, &self.turn_sign())],
                    &Real::zero(),
                ) {
                    Classification::Decided(value) => value,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                represented_circle_contact_location_parameter_from_dot_cross(
                    dot,
                    oriented_cross,
                    &radius_squared,
                )
                .map(|classification| {
                    classification.map(|contact| {
                        contact.map(|(location, parameter)| {
                            (
                                location,
                                BezierRepresentedCircleChordAngularParameter2::Materialized(
                                    parameter,
                                ),
                            )
                        })
                    })
                })
            };
            let (cusp_location, cusp_parameter) = match angular? {
                Classification::Decided(Some(contact)) => contact,
                Classification::Decided(None) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let chord_location = if clip_to_finite_chord {
                let start_sign = match parameter_difference_sign(&chord_parameter, &Real::zero()) {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let end_sign = match parameter_difference_sign(&chord_parameter, &Real::one()) {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if start_sign == RealSign::Negative || end_sign == RealSign::Positive {
                    continue;
                }
                match (start_sign, end_sign) {
                    (RealSign::Zero, _) => BezierAlgebraicCuspSemicircleContactLocation2::Start,
                    (_, RealSign::Zero) => BezierAlgebraicCuspSemicircleContactLocation2::End,
                    _ => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                }
            } else {
                BezierAlgebraicCuspSemicircleContactLocation2::Interior
            };
            // cross(turn*J(R),D) = -turn*dot(R,D), and the selected line
            // root has dot(R,D) = branch*sqrt(discriminant).
            let tangent_cross = if branch == 0 {
                AlgebraicRootRepresentation::from_exact_value(&Real::zero())
            } else {
                match represented_affine_coordinate(
                    &[(&signed_radical, &(-self.turn_sign()))],
                    &Real::zero(),
                ) {
                    Classification::Decided(value) => value,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            let tangent_cross_sign = match represented_strict_sign(&tangent_cross) {
                Some(sign) => sign,
                None => return Ok(Classification::Uncertain(UncertaintyReason::Predicate)),
            };
            contacts.push(BezierAlgebraicCuspSemicircleChordContact2 {
                branch,
                projective_parameter: None,
                cusp_location,
                chord_location,
                tangent_cross_sign,
            });
            retained_contacts.push(BezierRepresentedCircleChordContactData2 {
                branch,
                point,
                cusp_parameter,
                chord_parameter,
                tangent_cross,
                tangent_dot: tangent_dot.clone(),
            });
        }
        if contacts.is_empty() {
            return Ok(Classification::Decided(
                BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
            ));
        }
        let parameter_map = BezierAlgebraicCuspSemicircleChordParameterMap2::new(
            self.clone(),
            chord.clone(),
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RepresentedOblique(
                BezierRepresentedCircleChordParameterMapSystem2 {
                    center: frame.center,
                    contacts: retained_contacts,
                },
            ),
            clip_to_finite_chord,
            policy,
        );
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleChordIntersections2::Contacts {
                contacts,
                parameter_map,
            },
        ))
    }

    pub(in crate::bezier_offset) fn retained_represented_oblique_chord_intersections_in_domain(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        certified_endpoint_incidence: Option<BezierCertifiedFiniteChordEndpointIncidence2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>> {
        match self.represented_oblique_chord_intersections_in_domain(
            chord,
            clip_to_finite_chord,
            certified_endpoint_incidence,
            policy,
        )? {
            Classification::Decided(intersections) => {
                self.retain_chord_intersections(chord, intersections, policy)
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    pub(in crate::bezier_offset) fn retained_offset_chord_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleChordIntersections2>> {
        let system = match self.retained_offset_chord_system(chord, policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let discriminant_sign = match retained_offset_chord_speed_expression_sign(
            &system.retained,
            &system.retained.contact_discriminant,
            policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let branches: &[i8] = match discriminant_sign {
            RealSign::Negative => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
                ));
            }
            RealSign::Zero => &[0],
            RealSign::Positive => &[-1, 1],
        };
        let nested_sign = |expression, branch| {
            retained_offset_chord_nested_expression_sign(
                &system.retained,
                expression,
                branch,
                policy,
            )
        };
        let mut contacts = Vec::with_capacity(branches.len());
        for &branch in branches {
            let selected = match nested_sign(&system.selected_half_plane, branch)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let cusp_location = match selected {
                RealSign::Negative => continue,
                RealSign::Positive => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                RealSign::Zero => match nested_sign(&system.retained.diameter_side, branch)? {
                    Classification::Decided(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(RealSign::Negative) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::End
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "nonzero selected semicircle had an indeterminate retained-offset chord endpoint"
                                .into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            };
            let chord_location = if clip_to_finite_chord {
                let start_sign = match nested_sign(&system.point_minus_start, branch)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let end_sign = match nested_sign(&system.point_minus_end, branch)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if start_sign == RealSign::Negative || end_sign == RealSign::Positive {
                    continue;
                }
                match (start_sign, end_sign) {
                    (RealSign::Zero, _) => BezierAlgebraicCuspSemicircleContactLocation2::Start,
                    (_, RealSign::Zero) => BezierAlgebraicCuspSemicircleContactLocation2::End,
                    _ => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                }
            } else {
                BezierAlgebraicCuspSemicircleContactLocation2::Interior
            };
            let tangent_cross_sign = match branch {
                0 => RealSign::Zero,
                -1 => {
                    if self.is_clockwise() {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    }
                }
                1 => {
                    if self.is_clockwise() {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    }
                }
                _ => unreachable!("circle/chord support has at most two branches"),
            };
            contacts.push(BezierAlgebraicCuspSemicircleChordContact2 {
                branch,
                projective_parameter: None,
                cusp_location,
                chord_location,
                tangent_cross_sign,
            });
        }
        if contacts.is_empty() {
            return Ok(Classification::Decided(
                BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
            ));
        }
        let parameter_map = BezierAlgebraicCuspSemicircleChordParameterMap2::new(
            self.clone(),
            chord.clone(),
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RetainedOffset(system.retained),
            clip_to_finite_chord,
            policy,
        );
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleChordIntersections2::Contacts {
                contacts,
                parameter_map,
            },
        ))
    }

    /// Intersects a pair-radial selected circle with a finite chord whose
    /// support is authored by two independent algebraic endpoint fields.
    /// The circle-pair radical and the line-contact radical remain a two-level
    /// tower over the four selected roots; every classification below replays
    /// that tower exactly under the caller's predicate policy.
    pub(in crate::bezier_offset) fn selected_radial_chord_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleChordIntersections2>> {
        let system = match self.selected_radial_chord_system(chord, policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let discriminant_sign = match selected_radial_chord_pair_expression_sign(
            &system.retained,
            &system.retained.chord_discriminant,
            policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let branches: &[i8] = match discriminant_sign {
            RealSign::Negative => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
                ));
            }
            RealSign::Zero => &[0],
            RealSign::Positive => &[-1, 1],
        };
        let nested_sign = |expression, branch| {
            selected_radial_chord_nested_expression_sign(
                &system.retained,
                expression,
                branch,
                policy,
            )
        };
        let mut contacts = Vec::with_capacity(branches.len());
        for &branch in branches {
            let selected = match nested_sign(&system.selected_half_plane, branch)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let cusp_location = match selected {
                RealSign::Negative => continue,
                RealSign::Positive => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                RealSign::Zero => match nested_sign(&system.retained.diameter, branch)? {
                    Classification::Decided(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(RealSign::Negative) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::End
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "nonzero pair-radial semicircle had an indeterminate chord endpoint"
                                .into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            };
            let chord_location = if clip_to_finite_chord {
                let start_sign = match nested_sign(&system.point_minus_start, branch)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let end_sign = match nested_sign(&system.point_minus_end, branch)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if start_sign == RealSign::Negative || end_sign == RealSign::Positive {
                    continue;
                }
                match (start_sign, end_sign) {
                    (RealSign::Zero, _) => BezierAlgebraicCuspSemicircleContactLocation2::Start,
                    (_, RealSign::Zero) => BezierAlgebraicCuspSemicircleContactLocation2::End,
                    _ => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                }
            } else {
                BezierAlgebraicCuspSemicircleContactLocation2::Interior
            };
            let tangent_cross_sign = match branch {
                0 => RealSign::Zero,
                -1 => {
                    if self.is_clockwise() {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    }
                }
                1 => {
                    if self.is_clockwise() {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    }
                }
                _ => unreachable!("circle/chord support has at most two branches"),
            };
            contacts.push(BezierAlgebraicCuspSemicircleChordContact2 {
                branch,
                projective_parameter: None,
                cusp_location,
                chord_location,
                tangent_cross_sign,
            });
        }
        if contacts.is_empty() {
            return Ok(Classification::Decided(
                BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
            ));
        }
        let parameter_map = BezierAlgebraicCuspSemicircleChordParameterMap2::new(
            self.clone(),
            chord.clone(),
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::SelectedRadial(system.retained),
            clip_to_finite_chord,
            policy,
        );
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleChordIntersections2::Contacts {
                contacts,
                parameter_map,
            },
        ))
    }

    /// Intersects an exact or STRICT-certified retained line with a circle
    /// frame already represented by the canonical [`Real`] scalar.
    ///
    /// This is the compact authority shared by represented parallel-normal
    /// frames and selected frames whose retained coordinates simplify exactly.
    /// A retained support still uses the chord's own endpoint predicates for
    /// finite clipping.
    pub(in crate::bezier_offset) fn exact_frame_line_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        line: &LineSeg2,
        line_parameter_is_chord_parameter: bool,
        clip_to_finite_chord: bool,
        retained_orientation_reversed: bool,
        center: Point2,
        radial: (Real, Real),
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>> {
        // Give an unoriented affine support one deterministic solve frame.
        // Reversing an authored line must not rebuild the same irrational
        // circle contact through a distinct scalar expression graph.
        let canonical_order = policy.strict_predicate_pass(|| {
            match compare_reals(line.start().x(), line.end().x(), policy) {
                Some(std::cmp::Ordering::Equal) => {
                    compare_reals(line.start().y(), line.end().y(), policy)
                }
                order => order,
            }
        });
        let canonical_reversed = canonical_order == Some(std::cmp::Ordering::Greater);
        let canonical_line = canonical_reversed
            .then(|| LineSeg2::new_unchecked(line.end().clone(), line.start().clone()));
        let line = canonical_line.as_ref().unwrap_or(line);
        let retained_orientation_reversed = retained_orientation_reversed ^ canonical_reversed;
        let turn = Real::from(if self.is_clockwise() { -1_i8 } else { 1_i8 });
        let angular = (-(&turn * &radial.1), &turn * &radial.0);
        let radius_squared = Real::dot2_refs([&radial.0, &radial.1], [&radial.0, &radial.1]);
        let relation = crate::intersect::line_circle_relation_from_supports(
            line,
            &center,
            &radius_squared,
            policy,
        )?;
        let mut candidates = Vec::with_capacity(2);
        match relation {
            crate::LineCircleRelation::Disjoint => {
                return Ok(Classification::Decided(Vec::new()));
            }
            crate::LineCircleRelation::Tangent { point, line_param } => {
                candidates.push((point, line_param, RealSign::Zero));
            }
            crate::LineCircleRelation::Secant {
                first_point,
                first_param,
                second_point,
                second_param,
            } => {
                let first_cross = if self.is_clockwise() {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                };
                let second_cross = match first_cross {
                    RealSign::Positive => RealSign::Negative,
                    RealSign::Negative => RealSign::Positive,
                    RealSign::Zero => unreachable!("a secant has nonzero tangent cross"),
                };
                candidates.push((first_point, first_param, first_cross));
                candidates.push((second_point, second_param, second_cross));
            }
            crate::LineCircleRelation::Uncertain { reason } => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        let zero = Real::zero();
        let one = Real::one();
        let mut contacts = Vec::with_capacity(candidates.len());
        for (point, line_parameter, tangent_cross_sign) in candidates {
            let chord_line_parameter = (line_parameter_is_chord_parameter && clip_to_finite_chord)
                .then(|| {
                    if canonical_reversed {
                        &one - &line_parameter
                    } else {
                        line_parameter.clone()
                    }
                });
            let tangent_cross_sign = if retained_orientation_reversed {
                match tangent_cross_sign {
                    RealSign::Positive => RealSign::Negative,
                    RealSign::Negative => RealSign::Positive,
                    RealSign::Zero => RealSign::Zero,
                }
            } else {
                tangent_cross_sign
            };
            if let Some(line_parameter) = &chord_line_parameter {
                match in_closed_unit_interval(line_parameter, policy) {
                    Some(true) => {}
                    Some(false) => continue,
                    None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
                }
            }
            let from_center = point.delta_from(&center);
            let selected_projection =
                Real::dot2_refs([&from_center.0, &from_center.1], [&angular.0, &angular.1]);
            let selected_sign = match real_sign(&selected_projection, policy) {
                Some(sign) => sign,
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
            let radial_projection =
                Real::dot2_refs([&from_center.0, &from_center.1], [&radial.0, &radial.1]);
            let cusp_parameter = match selected_sign {
                RealSign::Negative => continue,
                RealSign::Positive => {
                    // With `a=(Q-C).R/r^2` and `b=(Q-C).T/r^2`, the rational
                    // half-circle chart has `u=b/(1+a+b)`. The common `r^2`
                    // denominator cancels, preserving the exact Real root.
                    let denominator = &radius_squared + &radial_projection + &selected_projection;
                    let parameter = (&selected_projection / denominator)?;
                    BezierAlgebraicCuspSemicircleParameter2::Exact(parameter)
                }
                RealSign::Zero => match real_sign(&radial_projection, policy) {
                    Some(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleParameter2::Exact(zero.clone())
                    }
                    Some(RealSign::Negative) => {
                        BezierAlgebraicCuspSemicircleParameter2::Exact(one.clone())
                    }
                    Some(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a nonzero represented circle retained its center as a contact".into(),
                        ));
                    }
                    None => {
                        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                    }
                },
            };
            let point = CurvePoint2::from(point);
            let chord_parameter = if let Some(line_parameter) = chord_line_parameter {
                match compare_reals(&line_parameter, &zero, policy) {
                    Some(std::cmp::Ordering::Equal) => chord.start_parameter(),
                    Some(_) => match compare_reals(&line_parameter, &one, policy) {
                        Some(std::cmp::Ordering::Equal) => chord.end_parameter(),
                        Some(_) => chord.parameter_at_certified_interior_point(point.clone()),
                        None => {
                            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
                        }
                    },
                    None => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
                    }
                }
            } else if clip_to_finite_chord {
                match chord.parameter_at_certified_point(point.clone(), policy)? {
                    Classification::Decided(Some(parameter)) => parameter,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            } else {
                chord.parameter_at_certified_support_point(point.clone(), policy)?
            };
            contacts.push(BezierAlgebraicCuspSemicircleRetainedChordContact2 {
                cusp_parameter,
                chord_parameter,
                point,
                tangent_cross_sign,
            });
        }
        Ok(Classification::Decided(contacts))
    }

    /// Intersects an exact or STRICT-certified retained line with a selected
    /// parallel-normal circle whose center parameter is represented by
    /// [`Real`]. Projecting this case through a bivariate selected-fiber system
    /// only increases degree and can obscure identities already represented by
    /// the scalar DAG.
    pub(in crate::bezier_offset) fn represented_parallel_normal_line_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        line: &LineSeg2,
        line_parameter_is_chord_parameter: bool,
        clip_to_finite_chord: bool,
        retained_orientation_reversed: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>> {
        let Some(frame) = self.data.frame.parallel_normal() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a represented parallel-normal circle crossed predicate policies".into(),
            ));
        }
        let Some(center_parameter) = frame.center_parameter.scalar() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let center = match frame
            .center_support
            .point_at_with_policy(center_parameter, policy)?
        {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let source_tangent = match frame
            .center_support
            .source_tangent_at(center_parameter, policy)?
        {
            Classification::Decided(tangent) => tangent,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let source_speed = Real::dot2_refs(
            [&source_tangent.0, &source_tangent.1],
            [&source_tangent.0, &source_tangent.1],
        )
        .sqrt()?;
        let unit_normal = (
            ((-&source_tangent.1) / &source_speed)?,
            (&source_tangent.0 / source_speed)?,
        );
        let radial = (
            &unit_normal.0 * &self.data.radial_distance,
            &unit_normal.1 * &self.data.radial_distance,
        );
        self.exact_frame_line_intersections(
            chord,
            line,
            line_parameter_is_chord_parameter,
            clip_to_finite_chord,
            retained_orientation_reversed,
            center,
            radial,
            policy,
        )
    }

    /// Builds the common retained quadratic line/circle system directly over
    /// a selected parallel-normal center field.
    ///
    /// The historical local-fiber path first squared
    /// `A(alpha, u) * sqrt(S(alpha)) + B(alpha, u)` into a quartic norm and
    /// later projected the selected `u` back to an ordinary algebraic scalar.
    /// The recursive line kernel already owns the smaller exact construction:
    /// retain `alpha`, keep the positive source speed as a base generator, and
    /// adjoin only the quadratic line-contact discriminant.  This adapter only
    /// changes coefficient representation; it performs no scalar promotion or
    /// approximate construction decision.
    pub(in crate::bezier_offset) fn selected_parallel_normal_dense_line_system(
        &self,
        line: &LineSeg2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierChordNormalDenseIntersectionSystem2>> {
        let Some(frame) = self.data.frame.parallel_normal() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a selected parallel-normal line system crossed predicate policies".into(),
            ));
        }
        let center_parameter =
            match promote_curve_region_bezier_parameter(&frame.center_parameter, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        let BezierParameter2::Algebraic(center_parameter) = &center_parameter else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let rational_line = RationalBezier2::try_new(
            vec![line.start().clone(), line.end().clone()],
            vec![Real::one(), Real::one()],
        )?;
        let system = match self.selected_parallel_normal_rational_system(
            &rational_line,
            &crate::CurveParameterRange2::unit(),
            policy,
        )? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let source = frame.center_support.source_power_basis()?;
        let differential = frame.center_support.differential()?;
        let sources = vec![bezier_parameter_root_representation(
            &BezierParameter2::Algebraic(center_parameter.clone()),
        )];
        let dense = |polynomial: &BivariatePolynomial| {
            dense_reduce_selected_root_relations(bivariate_dense_tensor(polynomial)?, &sources)
        };
        let zero = || DenseTensorPolynomial::zero(vec![1, 1]);
        // `BezierAlgebraicCuspTwoTermExpression2` signs
        // `rational + radical / sqrt(S)`.  Multiplication by positive
        // `sqrt(S)` embeds that value as `radical + rational * sqrt(S)` in
        // the recursive base without changing any predicate sign.
        let radical_sum = |expression: &BezierAlgebraicCuspTwoTermExpression2| {
            Some(TwoSquareRootExpression {
                rational: dense(&expression.radical)?,
                first: dense(&expression.rational)?,
                second: zero()?,
                product: zero()?,
            })
        };
        let square_root_sum = |expression: &BezierAlgebraicCuspTwoTermExpression2| {
            Some(TwoSquareRootExpression {
                rational: dense(&expression.rational)?,
                first: dense(&expression.radical)?,
                second: zero()?,
                product: zero()?,
            })
        };
        let rational = |polynomial: &BivariatePolynomial| {
            TwoSquareRootExpression::from_rational(dense(polynomial)?)
        };
        let first_radical = |polynomial: &BivariatePolynomial| {
            TwoSquareRootExpression::from_first_radical(dense(polynomial)?)
        };

        let unit = [Real::one()];
        let source_weight = source.weight.unwrap_or(&unit);
        let weight = bivariate_outer_product(source_weight, &unit);
        let common_denominator = bivariate_multiply(&weight, &system.speed_squared);
        let (line_delta_x, line_delta_y) = line.delta();
        let line_x =
            bivariate_outer_product(&unit, &[line.start().x().clone(), line_delta_x.clone()]);
        let line_y =
            bivariate_outer_product(&unit, &[line.start().y().clone(), line_delta_y.clone()]);
        let point_x = bivariate_multiply(&line_x, &common_denominator);
        let point_y = bivariate_multiply(&line_y, &common_denominator);

        // If P=(X/W,Y/W), T=(Tx,Ty), S=Tx^2+Ty^2, and d is the
        // center-support distance, multiply
        //   P + d*(-Ty,Tx)/sqrt(S)
        // by the rational projective denominator W*S.  The contact point and
        // center then share one strictly nonzero rational denominator, as the
        // common recursive line map requires.
        let source_x = bivariate_outer_product(source.x_numerator, &unit);
        let source_y = bivariate_outer_product(source.y_numerator, &unit);
        let tangent_x = bivariate_outer_product(&differential.tangent_x, &unit);
        let tangent_y = bivariate_outer_product(&differential.tangent_y, &unit);
        let center_distance = frame.center_support.distance();
        let center_x = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_multiply(&source_x, &system.speed_squared),
            radical: bivariate_scale(
                bivariate_multiply(&weight, &tangent_y),
                &(-center_distance.clone()),
            ),
        };
        let center_y = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_multiply(&source_y, &system.speed_squared),
            radical: bivariate_scale(bivariate_multiply(&weight, &tangent_x), center_distance),
        };

        let Some((incidence, selected_half_plane, diameter, radius_squared_denominator)) = (|| {
            Some((
                radical_sum(&system.circle)?,
                rational(&system.selected_half_plane)?,
                radical_sum(&system.diameter)?,
                // The angular comparison uses the same positive-speed scale
                // as `diameter`, so the rational r^2 term occupies the first
                // radical coefficient here.
                first_radical(&system.radius_squared_denominator)?,
            ))
        })(
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some((tangent_cross, angular_tangent, point_x, point_y, center_x, center_y)) = (|| {
            Some((
                radical_sum(&system.tangent_cross)?,
                radical_sum(&system.angular_tangent)?,
                rational(&point_x)?,
                rational(&point_y)?,
                square_root_sum(&center_x)?,
                square_root_sum(&center_y)?,
            ))
        })(
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(common_denominator) = dense(&common_denominator) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(second_speed_squared) =
            DenseTensorPolynomial::try_new(vec![1, 1], vec![Real::one()])
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(first_speed_squared) = dense(&system.speed_squared) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(Classification::Decided(
            BezierChordNormalDenseIntersectionSystem2 {
                map: Arc::new(BezierChordNormalDenseMapSystem2 {
                    source_representations: sources,
                    first_speed_squared,
                    second_speed_squared,
                    diameter,
                    radius_squared_denominator,
                }),
                incidence,
                selected_half_plane,
                tangent_cross,
                angular_tangent: Some(angular_tangent),
                geometry: Some(BezierChordNormalDenseTargetGeometry2 {
                    point_x,
                    point_y,
                    center_x,
                    center_y,
                    common_denominator,
                }),
            },
        ))
    }

    /// Solves an affine line against a selected parallel-normal circle in the
    /// authoritative recursive quadratic line kernel.
    pub(in crate::bezier_offset) fn recursive_selected_parallel_normal_line_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        line: &LineSeg2,
        finite_line_domain: bool,
        certified_endpoint_incidence: Option<BezierCertifiedFiniteChordEndpointIncidence2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleChordIntersections2>> {
        let system = match self.selected_parallel_normal_dense_line_system(line, policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match self.recursive_quadratic_chord_intersections_from_dense_system(
            chord,
            &system,
            finite_line_domain,
            certified_endpoint_incidence.filter(|_| finite_line_domain),
            policy,
        )? {
            Classification::Decided(Some(intersections)) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-chord-kernel",
                    "selected-parallel-normal-recursive-line",
                );
                Ok(Classification::Decided(intersections))
            }
            Classification::Decided(None) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    pub(in crate::bezier_offset) fn chord_normal_projective_chord_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        system: BezierChordNormalDenseIntersectionSystem2,
        clip_to_finite_chord: bool,
        certified_endpoint_incidence: Option<BezierCertifiedFiniteChordEndpointIncidence2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleChordIntersections2>> {
        // The authored target is an affine line, so its circle incidence is
        // quadratic in the final parameter even when the coefficient field
        // contains many independent selected roots.  Preserve that quadratic
        // over the retained field before constructing the global norm used by
        // the general dense fallback.
        match self.recursive_quadratic_chord_intersections_from_dense_system(
            chord,
            &system,
            clip_to_finite_chord,
            certified_endpoint_incidence,
            policy,
        )? {
            Classification::Decided(Some(intersections)) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-chord-kernel",
                    "chord-normal-recursive-quadratic",
                );
                return Ok(Classification::Decided(intersections));
            }
            Classification::Decided(None) | Classification::Uncertain(_) => {}
        }
        let domain = if clip_to_finite_chord {
            SelectedThirdAxisDomain2::Finite(&CurveParameterRange2::unit())
        } else {
            SelectedThirdAxisDomain2::AffineLine
        };
        let candidates = match system.contact_parameters(domain, policy)? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
                candidates
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
                return Err(CurveError::Topology(
                    "a nonzero chord-normal circle contained an affine chord component".into(),
                ));
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.chord_normal_projective_chord_intersections_from_candidates(
            chord,
            system,
            candidates,
            clip_to_finite_chord,
            policy,
        )
    }

    /// Solves a dense two-radical affine-line incidence inside its retained
    /// coefficient field. Circle/line incidence is at most quadratic in the
    /// affine parameter, so adjoining its positive discriminant is complete;
    /// constructing a global norm over every already-selected source root is
    /// unnecessary.
    pub(in crate::bezier_offset) fn recursive_quadratic_chord_intersections_from_dense_system(
        &self,
        chord: &BezierAlgebraicChord2,
        system: &BezierChordNormalDenseIntersectionSystem2,
        clip_to_finite_chord: bool,
        certified_endpoint_incidence: Option<BezierCertifiedFiniteChordEndpointIncidence2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleChordIntersections2>>> {
        let BezierChordNormalDenseIntersectionSystem2 {
            map,
            incidence,
            selected_half_plane,
            tangent_cross,
            angular_tangent: Some(angular_tangent),
            geometry: Some(geometry),
        } = system
        else {
            return Ok(Classification::Decided(None));
        };
        let output_axis = map.source_representations.len();
        let remove_output_axis = |polynomial: DenseTensorPolynomial| {
            polynomial.remove_certified_independent_axis(
                output_axis,
                hypersolve::PredicatePolicy::MAX_REFINEMENT_PRECISION,
            )
        };
        let Some((first_speed_squared, second_speed_squared)) =
            remove_output_axis(map.first_speed_squared.clone())
                .zip(remove_output_axis(map.second_speed_squared.clone()))
        else {
            return Ok(Classification::Decided(None));
        };
        let Some(field) = RecursiveQuadraticField::base(
            map.source_representations.clone(),
            first_speed_squared,
            second_speed_squared,
        ) else {
            return Ok(Classification::Decided(None));
        };
        let RecursiveQuadraticField::Base(base) = &field else {
            unreachable!("a dense quadratic line solve begins in its retained base field")
        };
        let strict_sign = |value: &RecursiveQuadraticValue| value.sign(&CurveContext::STRICT);
        if dense_expression_last_axis_degree(incidence) != Some(2) {
            return Ok(Classification::Decided(None));
        }
        let Some((mut c, mut b, mut a)) = (|| {
            Some((
                recursive_quadratic_base_expression_coefficient(incidence, 0, base)?,
                recursive_quadratic_base_expression_coefficient(incidence, 1, base)?,
                recursive_quadratic_base_expression_coefficient(incidence, 2, base)?,
            ))
        })() else {
            return Ok(Classification::Decided(None));
        };
        match strict_sign(&a)? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Negative) => {
                let negative = Real::from(-1_i8);
                a = a.scale(&negative).ok_or_else(|| {
                    CurveError::Topology(
                        "a retained quadratic coefficient exceeded its field budget".into(),
                    )
                })?;
                b = b.scale(&negative).ok_or_else(|| {
                    CurveError::Topology(
                        "a retained linear coefficient exceeded its field budget".into(),
                    )
                })?;
                c = c.scale(&negative).ok_or_else(|| {
                    CurveError::Topology(
                        "a retained constant coefficient exceeded its field budget".into(),
                    )
                })?;
            }
            Classification::Decided(RealSign::Zero) => {
                // A nondegenerate affine line has a strictly positive
                // squared-direction coefficient in circle incidence. Leave
                // an unexpected lower-degree tensor to the general exact
                // fallback rather than silently changing its component
                // semantics.
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let discriminant = b
            .square()
            .and_then(|value| {
                a.multiply(&c)
                    .and_then(|product| product.scale(&Real::from(4_i8)))
                    .and_then(|product| value.subtract(&product))
            })
            .ok_or_else(|| {
                CurveError::Topology(
                    "a retained line discriminant exceeded its quadratic field budget".into(),
                )
            })?;
        // Opposite strict endpoint incidences already prove one transverse
        // finite crossing, hence a strictly positive quadratic discriminant.
        // Consume that cheaper geometric certificate before asking the base
        // field to sign the expanded discriminant norm.
        let finite_endpoint_signs = if clip_to_finite_chord {
            let start_sign = if certified_endpoint_incidence
                == Some(BezierCertifiedFiniteChordEndpointIncidence2::Start)
            {
                RealSign::Zero
            } else {
                match strict_sign(&c)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            let end_sign = if certified_endpoint_incidence
                == Some(BezierCertifiedFiniteChordEndpointIncidence2::End)
            {
                RealSign::Zero
            } else {
                let end = a.add(&b).and_then(|value| value.add(&c)).ok_or_else(|| {
                    CurveError::Topology(
                        "a retained endpoint incidence exceeded its field budget".into(),
                    )
                })?;
                match strict_sign(&end)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            Some((start_sign, end_sign))
        } else {
            None
        };
        let finite_transition_branch = match finite_endpoint_signs {
            Some((RealSign::Positive, RealSign::Negative)) => Some(-1),
            Some((RealSign::Negative, RealSign::Positive)) => Some(1),
            _ => None,
        };
        if finite_endpoint_signs == Some((RealSign::Negative, RealSign::Negative)) {
            // An upward-opening quadratic that is negative at both finite
            // endpoints has its lower root before zero and upper root after
            // one. Neither boundary root belongs to the chord domain.
            return Ok(Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
            )));
        }
        let shifted_linear = || a.scale(&Real::from(2_i8)).and_then(|value| value.add(&b));
        let finite_linear_signs =
            if finite_endpoint_signs == Some((RealSign::Positive, RealSign::Positive)) {
                let linear_sign = match strict_sign(&b)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if linear_sign != RealSign::Negative {
                    // The derivative starts nonnegative and increases strictly,
                    // so a positive start incidence cannot cross zero.
                    return Ok(Classification::Decided(Some(
                        BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
                    )));
                }
                let shifted_linear = shifted_linear().ok_or_else(|| {
                    CurveError::Topology(
                        "a retained shifted linear coefficient exceeded its field budget".into(),
                    )
                })?;
                let shifted_linear_sign = match strict_sign(&shifted_linear)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if shifted_linear_sign != RealSign::Positive {
                    // The derivative remains nonpositive through the finite
                    // domain, so a positive end incidence cannot cross zero.
                    return Ok(Classification::Decided(Some(
                        BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
                    )));
                }
                Some((linear_sign, shifted_linear_sign))
            } else {
                None
            };
        let discriminant_sign = if finite_transition_branch.is_some() {
            RealSign::Positive
        } else {
            match strict_sign(&discriminant)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        if discriminant_sign == RealSign::Negative {
            return Ok(Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
            )));
        }
        // Classify each quadratic root against the finite affine domain from
        // the base-field coefficient signs. For `a > 0`, the signs of
        // `p(0)`, `b`, `p(1)`, and `2a + b` place both ordered roots exactly;
        // evaluating `(-b ± sqrt(D))/(2a)` merely to replay those comparisons
        // is a much larger extension-field predicate.
        let mut retained_branches = [(
            0_i8,
            BezierAlgebraicCuspSemicircleContactLocation2::Interior,
        ); 2];
        let retained_branch_count = if !clip_to_finite_chord {
            if discriminant_sign == RealSign::Zero {
                retained_branches[0].0 = 0;
                1
            } else {
                retained_branches[0].0 = -1;
                retained_branches[1].0 = 1;
                2
            }
        } else if let Some(branch) = finite_transition_branch {
            retained_branches[0].0 = branch;
            1
        } else {
            let (start_sign, end_sign) =
                finite_endpoint_signs.expect("a finite retained quadratic has endpoint signs");
            let (linear_sign, shifted_linear_sign) = if let Some(signs) = finite_linear_signs {
                signs
            } else {
                let linear_sign = match strict_sign(&b)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let shifted_linear = shifted_linear().ok_or_else(|| {
                    CurveError::Topology(
                        "a retained shifted linear coefficient exceeded its field budget".into(),
                    )
                })?;
                let shifted_linear_sign = match strict_sign(&shifted_linear)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                (linear_sign, shifted_linear_sign)
            };
            let root_order = |branch: i8, constant_sign, linear_sign| {
                use std::cmp::Ordering::{Equal, Greater, Less};
                match discriminant_sign {
                    RealSign::Zero => match (constant_sign, linear_sign, branch) {
                        (RealSign::Zero, RealSign::Zero, 0) => Some(Equal),
                        (RealSign::Positive, RealSign::Negative, 0) => Some(Greater),
                        (RealSign::Positive, RealSign::Positive, 0) => Some(Less),
                        _ => None,
                    },
                    RealSign::Positive => match (constant_sign, linear_sign, branch) {
                        (RealSign::Negative, _, -1) => Some(Less),
                        (RealSign::Negative, _, 1) => Some(Greater),
                        (RealSign::Zero, RealSign::Negative, -1) => Some(Equal),
                        (RealSign::Zero, RealSign::Negative, 1) => Some(Greater),
                        (RealSign::Zero, RealSign::Positive, -1) => Some(Less),
                        (RealSign::Zero, RealSign::Positive, 1) => Some(Equal),
                        (RealSign::Positive, RealSign::Negative, -1 | 1) => Some(Greater),
                        (RealSign::Positive, RealSign::Positive, -1 | 1) => Some(Less),
                        _ => None,
                    },
                    RealSign::Negative => None,
                }
            };
            let branches: &[i8] = if discriminant_sign == RealSign::Zero {
                &[0]
            } else {
                &[-1, 1]
            };
            let mut count = 0;
            for &branch in branches {
                let lower = root_order(branch, start_sign, linear_sign).ok_or_else(|| {
                    CurveError::Topology(
                        "inconsistent retained quadratic signs at the finite start".into(),
                    )
                })?;
                if lower == std::cmp::Ordering::Less {
                    continue;
                }
                let upper = root_order(branch, end_sign, shifted_linear_sign).ok_or_else(|| {
                    CurveError::Topology(
                        "inconsistent retained quadratic signs at the finite end".into(),
                    )
                })?;
                let location = match (lower, upper) {
                    (_, std::cmp::Ordering::Greater) => continue,
                    (std::cmp::Ordering::Equal, std::cmp::Ordering::Less) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    (std::cmp::Ordering::Greater, std::cmp::Ordering::Equal) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::End
                    }
                    (std::cmp::Ordering::Greater, std::cmp::Ordering::Less) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Interior
                    }
                    _ => {
                        return Err(CurveError::Topology(
                            "a retained quadratic root had inconsistent finite-domain orders"
                                .into(),
                        ));
                    }
                };
                retained_branches[count] = (branch, location);
                count += 1;
            }
            count
        };
        if retained_branch_count == 0 {
            return Ok(Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
            )));
        }
        let extension = if discriminant_sign == RealSign::Positive {
            Some(field.extension(discriminant).ok_or_else(|| {
                CurveError::Topology(
                    "a dense line discriminant could not extend its retained field".into(),
                )
            })?)
        } else {
            None
        };

        let common_denominator =
            TwoSquareRootExpression::from_rational(geometry.common_denominator.clone())
                .ok_or_else(|| {
                    CurveError::Topology(
                        "a dense line denominator exceeded its tensor shape budget".into(),
                    )
                })?;
        let Some(center_degree) = [
            dense_expression_last_axis_degree(&geometry.center_x),
            dense_expression_last_axis_degree(&geometry.center_y),
            dense_expression_last_axis_degree(&common_denominator),
        ]
        .into_iter()
        .collect::<Option<Vec<_>>>()
        .map(|degrees| degrees.into_iter().max().unwrap_or(0)) else {
            return Ok(Classification::Decided(None));
        };
        if center_degree != 0 {
            return Ok(Classification::Decided(None));
        }
        let Some(mut center) = (|| {
            Some(BezierRecursiveQuadraticProjectivePoint2 {
                x: recursive_quadratic_base_expression_coefficient(&geometry.center_x, 0, base)?,
                y: recursive_quadratic_base_expression_coefficient(&geometry.center_y, 0, base)?,
                denominator: recursive_quadratic_base_expression_coefficient(
                    &common_denominator,
                    0,
                    base,
                )?,
            })
        })() else {
            return Ok(Classification::Decided(None));
        };
        let negate_contact_projective = match strict_sign(&center.denominator)? {
            Classification::Decided(RealSign::Positive) => false,
            Classification::Decided(RealSign::Negative) => {
                let negative = Real::from(-1_i8);
                center.x = center.x.scale(&negative).ok_or_else(|| {
                    CurveError::Topology("a dense center exceeded its field budget".into())
                })?;
                center.y = center.y.scale(&negative).ok_or_else(|| {
                    CurveError::Topology("a dense center exceeded its field budget".into())
                })?;
                center.denominator = center.denominator.scale(&negative).ok_or_else(|| {
                    CurveError::Topology(
                        "a dense center denominator exceeded its field budget".into(),
                    )
                })?;
                true
            }
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a dense retained circle center had a zero projective denominator".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        let maximum_degree = |expressions: &[&TwoSquareRootExpression<DenseTensorPolynomial>]| {
            expressions
                .iter()
                .map(|expression| dense_expression_last_axis_degree(expression))
                .collect::<Option<Vec<_>>>()?
                .into_iter()
                .max()
        };
        let Some(point_degree) =
            maximum_degree(&[&geometry.point_x, &geometry.point_y, &common_denominator])
        else {
            return Ok(Classification::Decided(None));
        };
        let Some(angular_degree) =
            maximum_degree(&[&map.diameter, &map.radius_squared_denominator])
        else {
            return Ok(Classification::Decided(None));
        };
        let Some(tangent_degree) = maximum_degree(&[tangent_cross, angular_tangent]) else {
            return Ok(Classification::Decided(None));
        };
        let Some(selected_degree) = dense_expression_last_axis_degree(selected_half_plane) else {
            return Ok(Classification::Decided(None));
        };
        let turn_sign = if self.is_clockwise() {
            RealSign::Negative
        } else {
            RealSign::Positive
        };
        let mut contacts = Vec::with_capacity(retained_branch_count);
        let mut retained_contacts = Vec::with_capacity(retained_branch_count);
        for &(quadratic_branch, chord_location) in &retained_branches[..retained_branch_count] {
            let contact_field = extension.as_ref().unwrap_or(&field);
            let Some((parameter_numerator, parameter_denominator)) = (|| {
                let retained = b.scale(&Real::from(-1_i8))?;
                let denominator = a.scale(&Real::from(2_i8))?;
                if let Some(extension) = extension.as_ref() {
                    Some((
                        extension
                            .element(retained, field.constant(Real::from(quadratic_branch))?)?,
                        extension.lift(&denominator)?,
                    ))
                } else {
                    Some((retained, denominator))
                }
            })() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            // `a` was normalized and certified positive above, so the common
            // root denominator `2a` remains positive through every lift.
            let evaluate = |expression, degree| {
                recursive_quadratic_expression_projective_numerator(
                    expression,
                    base,
                    contact_field,
                    &parameter_numerator,
                    &parameter_denominator,
                    degree,
                )
            };
            let Some(mut point) = (|| {
                Some(BezierRecursiveQuadraticProjectivePoint2 {
                    x: evaluate(&geometry.point_x, point_degree)?,
                    y: evaluate(&geometry.point_y, point_degree)?,
                    denominator: evaluate(&common_denominator, point_degree)?,
                })
            })() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            // The common denominator has affine degree zero. Homogeneous
            // evaluation therefore multiplies its already-certified sign by
            // a positive power of `2a`; normalize from that retained fact
            // instead of expanding and resigning the extension value.
            if negate_contact_projective {
                let negative = Real::from(-1_i8);
                point.x = point.x.scale(&negative).ok_or_else(|| {
                    CurveError::Topology("a retained contact exceeded its field budget".into())
                })?;
                point.y = point.y.scale(&negative).ok_or_else(|| {
                    CurveError::Topology("a retained contact exceeded its field budget".into())
                })?;
                point.denominator = point.denominator.scale(&negative).ok_or_else(|| {
                    CurveError::Topology(
                        "a retained contact denominator exceeded its field budget".into(),
                    )
                })?;
            }
            let Some((selected_half_plane, diameter, radius_squared_denominator)) = (|| {
                Some((
                    evaluate(selected_half_plane, selected_degree)?,
                    evaluate(&map.diameter, angular_degree)?,
                    evaluate(&map.radius_squared_denominator, angular_degree)?,
                ))
            })(
            ) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let selected_sign = match strict_sign(&selected_half_plane)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let cusp_location = match selected_sign {
                RealSign::Negative => continue,
                RealSign::Positive => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                RealSign::Zero => match strict_sign(&diameter)? {
                    Classification::Decided(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(RealSign::Negative) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::End
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a nonzero retained circle contact had zero local diameter".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            };
            let Some((tangent_cross, angular_tangent)) = (|| {
                Some((
                    evaluate(tangent_cross, tangent_degree)?,
                    evaluate(angular_tangent, tangent_degree)?,
                ))
            })() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let tangent_cross_sign = match strict_sign(&tangent_cross)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let branch = match tangent_cross_sign {
                RealSign::Zero => 0,
                sign if sign == turn_sign => -1,
                RealSign::Negative | RealSign::Positive => 1,
            };
            let certified_bounds =
                clip_to_finite_chord.then(|| chord_location.certified_unit_bounds());
            let parameter = match BezierRecursiveProjectiveParameter2::new_with_certified_bounds(
                RecursiveQuadraticProjectiveScalar {
                    numerator: parameter_numerator.clone(),
                    denominator: parameter_denominator.clone(),
                },
                certified_bounds,
                policy,
            )? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            retained_contacts.push(BezierRecursiveQuadraticLineContactSystem2 {
                branch,
                parameter,
                tangent_dot_sign: None,
                point,
                diameter,
                radius_squared_denominator,
                tangent_cross,
                angular_tangent,
            });
            contacts.push(BezierAlgebraicCuspSemicircleChordContact2 {
                branch,
                projective_parameter: None,
                cusp_location,
                chord_location,
                tangent_cross_sign,
            });
        }
        if contacts.is_empty() {
            return Ok(Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
            )));
        }
        if contacts.len() > 2
            || retained_contacts
                .iter()
                .map(|contact| contact.branch)
                .collect::<std::collections::BTreeSet<_>>()
                .len()
                != retained_contacts.len()
        {
            return Err(CurveError::Topology(
                "a retained quadratic line solve produced duplicate contact branches".into(),
            ));
        }
        let parameter_map = BezierAlgebraicCuspSemicircleChordParameterMap2::new(
            self.clone(),
            chord.clone(),
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RecursiveQuadraticLine(
                BezierRecursiveQuadraticLineParameterMapSystem2 {
                    center,
                    contacts: retained_contacts,
                },
            ),
            clip_to_finite_chord,
            policy,
        );
        Ok(Classification::Decided(Some(
            BezierAlgebraicCuspSemicircleChordIntersections2::Contacts {
                contacts,
                parameter_map,
            },
        )))
    }

    pub(in crate::bezier_offset) fn chord_normal_projective_chord_intersections_from_candidates(
        &self,
        chord: &BezierAlgebraicChord2,
        system: BezierChordNormalDenseIntersectionSystem2,
        candidates: Vec<BezierParameter2>,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleChordIntersections2>> {
        let zero = BezierParameter2::Exact(Real::zero());
        let one = BezierParameter2::Exact(Real::one());
        let turn_sign = if self.is_clockwise() {
            RealSign::Negative
        } else {
            RealSign::Positive
        };
        let mut contacts = Vec::with_capacity(candidates.len().min(2));
        for candidate in candidates {
            let selected = match system.selected_half_plane_sign(&candidate, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let cusp_location = match selected {
                RealSign::Negative => continue,
                RealSign::Positive => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                RealSign::Zero => match system.diameter_sign(&candidate, policy)? {
                    Classification::Decided(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(RealSign::Negative) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::End
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a nonzero chord-normal circle contact had zero local diameter".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            };
            let chord_location = if clip_to_finite_chord {
                match candidate.cmp_by_refinement_with_policy(&zero, policy)? {
                    Classification::Decided(std::cmp::Ordering::Equal) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(std::cmp::Ordering::Greater) => {
                        match candidate.cmp_by_refinement_with_policy(&one, policy)? {
                            Classification::Decided(std::cmp::Ordering::Equal) => {
                                BezierAlgebraicCuspSemicircleContactLocation2::End
                            }
                            Classification::Decided(std::cmp::Ordering::Less) => {
                                BezierAlgebraicCuspSemicircleContactLocation2::Interior
                            }
                            Classification::Decided(std::cmp::Ordering::Greater) => continue,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    Classification::Decided(std::cmp::Ordering::Less) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            } else {
                BezierAlgebraicCuspSemicircleContactLocation2::Interior
            };
            let tangent_cross_sign = match system.tangent_cross_sign(&candidate, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let branch = match tangent_cross_sign {
                RealSign::Zero => 0,
                sign if sign == turn_sign => -1,
                RealSign::Negative | RealSign::Positive => 1,
            };
            contacts.push(BezierAlgebraicCuspSemicircleChordContact2 {
                branch,
                projective_parameter: Some(candidate),
                cusp_location,
                chord_location,
                tangent_cross_sign,
            });
        }
        if contacts.len() > 2 {
            return Err(CurveError::Topology(
                "a chord-normal circle retained more than two affine chord contacts".into(),
            ));
        }
        if contacts.is_empty() {
            return Ok(Classification::Decided(
                BezierAlgebraicCuspSemicircleChordIntersections2::NoContacts,
            ));
        }
        let parameter_map = BezierAlgebraicCuspSemicircleChordParameterMap2::new(
            self.clone(),
            chord.clone(),
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::ChordNormalProjective(
                system.into_parameter_map_system(),
            ),
            clip_to_finite_chord,
            policy,
        );
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleChordIntersections2::Contacts {
                contacts,
                parameter_map,
            },
        ))
    }

    pub(in crate::bezier_offset) fn retain_chord_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        intersections: BezierAlgebraicCuspSemicircleChordIntersections2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>> {
        let BezierAlgebraicCuspSemicircleChordIntersections2::Contacts {
            contacts,
            parameter_map,
        } = intersections
        else {
            return Ok(Classification::Decided(Vec::new()));
        };
        let finite_chord_domain = parameter_map.data.finite_chord_domain;
        let mut retained = Vec::with_capacity(contacts.len());
        for contact in contacts {
            let (cusp_parameter, correlated_point) = parameter_map.contact_evidence(&contact);
            let point = match contact.chord_location {
                BezierAlgebraicCuspSemicircleContactLocation2::Start => chord.start().clone(),
                BezierAlgebraicCuspSemicircleContactLocation2::End => chord.end().clone(),
                BezierAlgebraicCuspSemicircleContactLocation2::Interior => {
                    match contact.cusp_location {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start => {
                            match self.start_point_evidence(policy)? {
                                Classification::Decided(point) => point,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                        BezierAlgebraicCuspSemicircleContactLocation2::End => {
                            match self.end_point_evidence(policy)? {
                                Classification::Decided(point) => point,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                        BezierAlgebraicCuspSemicircleContactLocation2::Interior => correlated_point,
                    }
                }
            };
            let chord_parameter = match contact.chord_location {
                BezierAlgebraicCuspSemicircleContactLocation2::Start => chord.start_parameter(),
                BezierAlgebraicCuspSemicircleContactLocation2::End => chord.end_parameter(),
                BezierAlgebraicCuspSemicircleContactLocation2::Interior => {
                    if finite_chord_domain {
                        // The exact finite-domain kernel has already proved
                        // incidence and both strict boundary inequalities.
                        chord.parameter_at_certified_interior_point(point.clone())
                    } else {
                        // A complete-support solve intentionally admits both
                        // exterior rays; retain no finite placement claim.
                        chord.parameter_on_retained_support(point.clone())
                    }
                }
            };
            retained.push(BezierAlgebraicCuspSemicircleRetainedChordContact2 {
                cusp_parameter,
                chord_parameter,
                point,
                tangent_cross_sign: contact.tangent_cross_sign,
            });
        }
        Ok(Classification::Decided(retained))
    }

    /// Replays an affine-support solve from an ancestral procedural offset
    /// onto the caller's finite descendant chord.
    ///
    /// Offset miters and Boolean splits introduce new endpoint evidence but
    /// do not introduce a new supporting line. Solving on the retained
    /// normal-offset ancestor keeps its three-field nested radical authority;
    /// each resulting point is then ordered on the descendant's certified
    /// support and only finite contacts are retained.
    pub(in crate::bezier_offset) fn retained_offset_ancestor_chord_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        ancestor: &BezierAlgebraicChord2,
        descendant_reversed: bool,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>> {
        let intersections =
            match self.retained_offset_chord_intersections(ancestor, false, policy)? {
                Classification::Decided(intersections) => intersections,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let retained = match self.retain_chord_intersections(ancestor, intersections, policy)? {
            Classification::Decided(retained) => retained,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.reclip_retained_chord_intersections(
            chord,
            retained,
            descendant_reversed,
            clip_to_finite_chord,
            policy,
        )
    }

    /// Clips contacts solved on an ancestral affine support to a finite
    /// descendant without rebuilding the circle/line elimination system.
    pub(in crate::bezier_offset) fn reclip_retained_chord_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        intersections: Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>,
        descendant_reversed: bool,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>> {
        let retained = intersections;
        let mut contacts = Vec::with_capacity(retained.len());
        for contact in retained {
            let chord_parameter = if clip_to_finite_chord {
                let parameter =
                    chord.parameter_at_certified_point(contact.point.clone(), policy)?;
                match parameter {
                    Classification::Decided(Some(parameter)) => parameter,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            } else {
                chord.parameter_at_certified_support_point(contact.point.clone(), policy)?
            };
            contacts.push(BezierAlgebraicCuspSemicircleRetainedChordContact2 {
                cusp_parameter: contact.cusp_parameter,
                chord_parameter,
                point: contact.point,
                tangent_cross_sign: if descendant_reversed {
                    product_sign(contact.tangent_cross_sign, RealSign::Negative)
                } else {
                    contact.tangent_cross_sign
                },
            });
        }
        Ok(Classification::Decided(contacts))
    }

    /// Bounds the complete supporting circle even when ordering a trimmed
    /// semicircle parameter interval is unavailable. A selected center box
    /// expanded by the exact absolute radius contains every support point.
    pub(in crate::bezier_offset) fn conservative_circle_cover_bounds(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Aabb2>> {
        let circle_reason = match self.conservative_bounds(policy)? {
            Classification::Decided(bounds) => return Ok(Classification::Decided(bounds)),
            Classification::Uncertain(reason) => reason,
        };
        let center = match self.center_point_evidence(policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(_) => {
                return Ok(Classification::Uncertain(circle_reason));
            }
        };
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let Classification::Decided(center_bounds) =
                algebraic_chord_endpoint_bounds_refined(&center, refinement_steps, policy)
            else {
                continue;
            };
            let radius = self.radial_distance().abs();
            return Ok(Classification::Decided(Aabb2::new_unchecked(
                Point2::new(
                    center_bounds.min().x() - &radius,
                    center_bounds.min().y() - &radius,
                ),
                Point2::new(
                    center_bounds.max().x() + &radius,
                    center_bounds.max().y() + radius,
                ),
            )));
        }
        Ok(Classification::Uncertain(circle_reason))
    }

    /// Returns a finite exact parameterization of `line` that contains the
    /// complete selected circle. This lets bounded rational and selected-fiber
    /// solvers enumerate an affine carrier without inventing a numeric extent.
    pub(in crate::bezier_offset) fn affine_line_cover(
        &self,
        line: &LineSeg2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<LineSeg2>> {
        let bounds = match self.conservative_circle_cover_bounds(policy)? {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (delta_x, delta_y) = line.delta();
        let x_sign = real_sign(&delta_x, policy);
        let y_sign = real_sign(&delta_y, policy);
        let (origin, delta, first_bound, second_bound, delta_sign) = match (x_sign, y_sign) {
            (Some(sign @ (RealSign::Positive | RealSign::Negative)), _) => (
                line.start().x(),
                &delta_x,
                bounds.min_x(),
                bounds.max_x(),
                sign,
            ),
            (_, Some(sign @ (RealSign::Positive | RealSign::Negative))) => (
                line.start().y(),
                &delta_y,
                bounds.min_y(),
                bounds.max_y(),
                sign,
            ),
            (Some(RealSign::Zero), Some(RealSign::Zero)) => {
                return Err(CurveError::ZeroLengthLine);
            }
            _ => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let first = ((first_bound - origin) / delta)?;
        let second = ((second_bound - origin) / delta)?;
        let (lower, upper) = match delta_sign {
            RealSign::Positive => (first, second),
            RealSign::Negative => (second, first),
            RealSign::Zero => unreachable!("the selected line coordinate has nonzero delta"),
        };
        let cover = LineSeg2::try_new(
            line.point_at(lower - Real::one()),
            line.point_at(upper + Real::one()),
        )?;
        Ok(Classification::Decided(cover))
    }

    /// Intersects this selected semicircle with any finite retained chord for
    /// which an exact support kernel is available.
    ///
    /// Structurally axis-aligned algebraic supports keep the compact correlated
    /// square-root fast path. A chord with represented endpoints is an exact
    /// rational line in any direction and reuses the selected-circle resultant
    /// kernel. Procedural normal offsets retain their positive speed radical;
    /// remaining chords use the general selected-field analytic support
    /// kernels. Every path retains identical cusp/chord parameter evidence for
    /// the region Boolean engine.
    pub(crate) fn chord_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>> {
        self.chord_intersections_in_domain(chord, true, policy)
    }

    /// Intersects a finite chord after its owning boundary loop has already
    /// certified one exact shared endpoint. The certificate seeds only that
    /// endpoint's circle-incidence sign; all other roots and every selected-
    /// half and tangent predicate remain responsibilities of the common
    /// circle/chord kernel.
    pub(crate) fn chord_intersections_with_certified_endpoint_incidence(
        &self,
        chord: &BezierAlgebraicChord2,
        chord_at_start: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>> {
        self.chord_intersections_in_domain_with_exact_line_preference(
            chord,
            true,
            false,
            Some(if chord_at_start {
                BezierCertifiedFiniteChordEndpointIncidence2::Start
            } else {
                BezierCertifiedFiniteChordEndpointIncidence2::End
            }),
            policy,
        )
    }

    /// Intersects this selected semicircle with the complete affine support of
    /// `chord`. Returned chord parameters intentionally retain the same
    /// monotone carrier coordinate outside the authored finite endpoints.
    pub(crate) fn chord_support_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>> {
        self.chord_intersections_in_domain(chord, false, policy)
    }

    /// Recognizes a tangent endpoint assembled entirely from retained unit
    /// normal provenance after an analytic carrier was reparameterized.
    ///
    /// If `Q` is the canonical analytic point corresponding to this selected
    /// center `C`, then both lie on parallels of the same authored source and
    /// `Q-C = (d_q-d_c)N`. A chord endpoint constructed as `Q+eN` therefore
    /// lies at the circle start or end exactly when
    /// `d_q-d_c+e = +/-r`. A retained Cartesian translation is admitted when
    /// it has zero certified tangent component; its signed normal component
    /// simply adds to `e`. Collinear certified source/chord tangents make the
    /// complete affine support tangent there, so this one endpoint is its
    /// unique circle contact. No Cartesian compositum or root isolation is
    /// required.
    pub(in crate::bezier_offset) fn radial_displaced_chord_tangent_intersections(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>>>
    {
        let Some(frame) = self.data.frame.parallel_normal() else {
            return Ok(Classification::Decided(None));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a radial displaced chord contact crossed selected-circle policies".into(),
            ));
        }
        let target_tangent = chord.certified_unit_tangent();
        let center = CurvePoint2::from(
            BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
                frame.center_support.clone(),
                &frame.center_parameter,
                Real::zero(),
                policy,
            )
            .expect("a parallel-normal frame owns a scalar parameter"),
        );
        for (at_end, endpoint) in [(false, chord.start()), (true, chord.end())] {
            let CurvePoint2(CurvePointData2::AlgebraicChordParallel(displaced)) = endpoint else {
                continue;
            };
            let accepts_policy = displaced.accepts_policy(policy);
            let left_normal =
                displaced.data.direction == BezierAlgebraicChordUnitDisplacement2::LeftNormal;
            let zero_x = displaced.data.translation_x.zero_status() == ZeroKnowledge::Zero;
            let zero_y = displaced.data.translation_y.zero_status() == ZeroKnowledge::Zero;
            let shares_center = displaced.source_endpoint().same_point(&center, policy);
            if !accepts_policy
                || !left_normal
                || !zero_x
                || !zero_y
                || shares_center != Classification::Decided(true)
            {
                continue;
            }
            let source_reversed =
                if let Some(reversed) = displaced.data.source.shared_tangent_orientation(chord) {
                    reversed
                } else {
                    match (
                        displaced.data.source.tangent_cross_sign(chord, policy)?,
                        displaced.data.source.tangent_dot_sign(chord, policy)?,
                    ) {
                        (
                            Classification::Decided(RealSign::Zero),
                            Classification::Decided(RealSign::Positive),
                        ) => false,
                        (
                            Classification::Decided(RealSign::Zero),
                            Classification::Decided(RealSign::Negative),
                        ) => true,
                        (Classification::Decided(_), Classification::Decided(_)) => continue,
                        (Classification::Uncertain(reason), _)
                        | (_, Classification::Uncertain(reason)) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                };
            let radial_distance = if source_reversed {
                -displaced.data.distance.clone()
            } else {
                displaced.data.distance.clone()
            };
            let residual = &radial_distance * &radial_distance
                - self.radial_distance() * self.radial_distance();
            let residual_sign = real_sign(&residual, policy);
            match residual_sign {
                Some(RealSign::Zero) => {}
                Some(RealSign::Positive | RealSign::Negative) => continue,
                None => {
                    return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                }
            }
            let radial_product_sign = match real_sign(
                &(self.radial_distance() * &radial_distance),
                &CurveContext::STRICT,
            ) {
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Some(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "a radial displaced tangent contact retained a zero radius".into(),
                    ));
                }
                None => {
                    return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                }
            };
            let frame_relation = if let Some(target_tangent) = target_tangent.as_ref() {
                frame.center_support.vector_tangent_cross_and_dot_signs(
                    &frame.center_parameter,
                    &target_tangent.0,
                    &target_tangent.1,
                    policy,
                )?
            } else {
                // A procedural normal offset can retain its tangent only as
                // endpoint ancestry, without a pair of represented unit-vector
                // coordinates. Build the positive analytic tangent in the
                // center parameter's own carrier and compare the two retained
                // chords directly; their shared support relations decide the
                // exact zero before any Cartesian compositum is attempted.
                let tangent_start = CurvePoint2::from(
                    BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
                        frame.center_support.clone(),
                        &frame.center_parameter,
                        Real::zero(),
                        policy,
                    )
                    .expect("a parallel-normal frame owns a scalar parameter"),
                );
                let tangent_end = CurvePoint2::from(
                    BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
                        frame.center_support.clone(),
                        &frame.center_parameter,
                        Real::one(),
                        policy,
                    )
                    .expect("a parallel-normal frame owns a scalar parameter"),
                );
                let tangent =
                    match BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                        tangent_start,
                        tangent_end,
                        policy,
                    )? {
                        Classification::Decided(tangent) => tangent,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                let (cross, dot) = policy.strict_predicate_pass(|| {
                    Ok::<_, CurveError>((
                        tangent.tangent_cross_sign(chord, policy)?,
                        tangent.tangent_dot_sign(chord, policy)?,
                    ))
                })?;
                match (cross, dot) {
                    (Classification::Decided(cross), Classification::Decided(dot)) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-circle-chord-kernel",
                            "retained-radial-analytic-tangent",
                        );
                        Classification::Decided((cross, dot))
                    }
                    (Classification::Uncertain(reason), _)
                    | (_, Classification::Uncertain(reason)) => Classification::Uncertain(reason),
                }
            };
            let (frame_cross, frame_dot) = match frame_relation {
                Classification::Decided(signs) => signs,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let cusp_parameter = if frame_cross == RealSign::Zero {
                if frame_dot == RealSign::Zero {
                    return Err(CurveError::Topology(
                        "parallel nonzero tangents had zero cross and dot products".into(),
                    ));
                }
                let along_frame_sign = product_sign(
                    match real_sign(&radial_distance, &CurveContext::STRICT) {
                        Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                        Some(RealSign::Zero) => unreachable!("the contact radius is nonzero"),
                        None => {
                            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                        }
                    },
                    frame_dot,
                );
                let radius_sign = match real_sign(self.radial_distance(), &CurveContext::STRICT) {
                    Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                    Some(RealSign::Zero) => unreachable!("the selected circle is nonzero"),
                    None => {
                        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                    }
                };
                if along_frame_sign == radius_sign {
                    BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero())
                } else {
                    BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
                }
            } else {
                // The supporting circle alone is insufficient: this carrier
                // owns only the half selected from its frame tangent. With
                // R0=r*left_normal(F) and R=d*left_normal(D), the contact is
                // on that half exactly when
                //
                //   turn * cross(R0, R)
                //     = turn * sign(r*d) * cross(F, D) > 0.
                //
                // Rejecting the complementary half here prevents a retained
                // endpoint on the same full circle from becoming a false
                // finite semicircle contact. `frame_cross` is supplied as
                // D x F, so its selected-half sign is the negation of the
                // displayed F x D predicate.
                let turn = if self.is_clockwise() {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                };
                match product_sign(turn, product_sign(radial_product_sign, frame_cross)) {
                    RealSign::Negative => {}
                    RealSign::Positive => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-circle-chord-kernel",
                            "retained-radial-complementary-half",
                        );
                        // The certified chord support is tangent to the full
                        // circle at this one point. Once that point is outside
                        // the selected half there cannot be another contact to
                        // recover from the generic kernel.
                        return Ok(Classification::Decided(Some(Vec::new())));
                    }
                    RealSign::Zero => unreachable!("the selected-half factors are nonzero"),
                }
                BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                        semicircle: self.clone(),
                        parallel: frame.center_support.clone(),
                        parallel_parameter: frame.center_parameter.clone(),
                        chord: chord.clone(),
                        radial_product_sign,
                        point: endpoint.clone(),
                        policy: *policy,
                    },
                ))
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-chord-kernel",
                "retained-radial-endpoint-tangent",
            );
            return Ok(Classification::Decided(Some(vec![
                BezierAlgebraicCuspSemicircleRetainedChordContact2 {
                    cusp_parameter,
                    chord_parameter: if at_end {
                        chord.end_parameter()
                    } else {
                        chord.start_parameter()
                    },
                    point: endpoint.clone(),
                    tangent_cross_sign: RealSign::Zero,
                },
            ])));
        }
        Ok(Classification::Decided(None))
    }

    pub(in crate::bezier_offset) fn chord_intersections_in_domain(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>> {
        self.chord_intersections_in_domain_with_exact_line_preference(
            chord,
            clip_to_finite_chord,
            false,
            None,
            policy,
        )
    }

    /// Intersects an authored exact-line witness without first replaying its
    /// procedural ancestry. This opt-in preserves the compact recursive
    /// quadratic scalar used by native lines; ordinary algebraic chords
    /// continue through the stronger retained-support authority.
    pub(crate) fn chord_intersections_prefer_exact_line(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>> {
        self.chord_intersections_in_domain_with_exact_line_preference(
            chord,
            clip_to_finite_chord,
            true,
            None,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn chord_intersections_in_domain_with_exact_line_preference(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        prefer_exact_line: bool,
        certified_endpoint_incidence: Option<BezierCertifiedFiniteChordEndpointIncidence2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>> {
        if policy.permits_approximate_512() {
            let strict_policy = policy.strict_counterpart();
            let can_replay_strict = self.data.frame.selected_radial().is_some_and(|frame| {
                strict_policy.accepts_retained_policy(frame.policy)
                    && BezierAlgebraicCuspSemicircleParameter2::Mapped(
                        frame.center_parameter.clone(),
                    )
                    .validate_policy(&strict_policy)
                    .is_ok()
            }) && strict_policy.accepts_retained_policy(chord.policy());
            let strict = if can_replay_strict {
                self.chord_intersections_in_domain_once(
                    chord,
                    clip_to_finite_chord,
                    prefer_exact_line,
                    certified_endpoint_incidence,
                    &strict_policy,
                )
            } else {
                policy.strict_predicate_pass(|| {
                    self.chord_intersections_in_domain_once(
                        chord,
                        clip_to_finite_chord,
                        prefer_exact_line,
                        certified_endpoint_incidence,
                        policy,
                    )
                })
            };
            match strict {
                Ok(decided @ Classification::Decided(_)) => return Ok(decided),
                Ok(Classification::Uncertain(_)) => {}
                Err(error) => return Err(error),
            }
        }
        self.chord_intersections_in_domain_once(
            chord,
            clip_to_finite_chord,
            prefer_exact_line,
            certified_endpoint_incidence,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn chord_intersections_in_domain_once(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        prefer_exact_line: bool,
        certified_endpoint_incidence: Option<BezierCertifiedFiniteChordEndpointIncidence2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>> {
        chord.validate_policy(policy)?;
        match self.radial_displaced_chord_tangent_intersections(chord, policy)? {
            Classification::Decided(Some(intersections)) => {
                return Ok(Classification::Decided(intersections));
            }
            Classification::Decided(None) | Classification::Uncertain(_) => {}
        }
        if clip_to_finite_chord {
            // Bounds only reject work; they never decide an overlapping
            // circle/chord predicate. Keep their refinement below the policy
            // terminal so APPROXIMATE_512 is consumed only by the exact
            // contact predicates that can decide the operation.
            let broad_phase_refinements: &[usize] = if self.uses_selected_chord_normal_frame() {
                &[0, 2]
            } else {
                &[0]
            };
            for &refinement_steps in broad_phase_refinements {
                let circle_bounds = self.conservative_bounds_refined(refinement_steps, policy)?;
                let chord_bounds = chord.conservative_bounds_refined(refinement_steps, policy)?;
                if let (
                    Classification::Decided(circle_bounds),
                    Classification::Decided(chord_bounds),
                ) = (circle_bounds, chord_bounds)
                    && circle_bounds.overlaps(&chord_bounds, &CurveContext::STRICT)
                        == Classification::Decided(false)
                {
                    return Ok(Classification::Decided(Vec::new()));
                }
            }
        }
        if self.data.frame.rational().is_some() && chord.certified_axis_direction().is_some() {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-chord-kernel",
                "axis-correlated-fast-path",
            );
            match self.axis_chord_intersections_in_domain(chord, clip_to_finite_chord, policy)? {
                Classification::Decided(intersections) => {
                    return self.retain_chord_intersections(chord, intersections, policy);
                }
                Classification::Uncertain(_) => {}
            }
        }

        // A selected parallel-normal circle with an algebraic center retains
        // its recursive quadratic line scalar most cheaply against directly
        // represented Real endpoints. Prefer that smaller authority here; every other
        // descendant still replays procedural ancestry, which can carry the
        // stronger support relation needed by its original construction.
        let exact_line = chord.exact_line();
        let line_parameter_is_chord_parameter = exact_line.is_some();
        let certified_support_line = exact_line
            .is_none()
            .then(|| chord.strict_provenance_support_line(policy))
            .flatten();
        let retained_support = chord.retained_support();
        let retained_offset_support = matches!(
            (retained_support.start(), retained_support.end()),
            (
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
            ) if start.shares_carrier(end)
                && start.at_end != end.at_end
                && start.data.direction == BezierAlgebraicChordUnitDisplacement2::LeftNormal
        );
        let prefer_authored_exact_line = prefer_exact_line
            && exact_line.is_some()
            && (!self.uses_selected_radial_frame()
                || self.has_recursive_selected_radial_line_parent());
        let prefer_selected_parallel_exact_line = prefer_authored_exact_line
            && self.uses_selected_parallel_normal_frame()
            && self
                .selected_frame_parameter()
                .is_some_and(|parameter| parameter.scalar().is_none());
        if !prefer_authored_exact_line && !Arc::ptr_eq(&retained_support.data, &chord.data) {
            match self.chord_intersections_in_domain(retained_support, false, policy)? {
                Classification::Decided(intersections) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-circle-chord-kernel",
                        "retained-support-replay",
                    );
                    return self.reclip_retained_chord_intersections(
                        chord,
                        intersections,
                        chord.retained_support_orientation_is_reversed(),
                        clip_to_finite_chord,
                        policy,
                    );
                }
                Classification::Uncertain(_) => {}
            }
        }

        // A chord-normal carrier can still be an ordinary represented circle:
        // an exact center plus a represented anchor tangent makes its entire
        // radial frame canonical. Keep the procedural carrier (and therefore
        // its authored endpoint ancestry), but intersect an exact line through
        // the compact Real line/circle primitive. Interior contacts then
        // publish Exact point evidence rather than a needless selected-field
        // correlation.
        if let (Some(frame), Some(line), Some(center)) = (
            self.data.frame.chord_normal(),
            exact_line.as_ref().or(certified_support_line.as_ref()),
            self.exact_center(policy)?,
        ) && let Some((tangent_x, tangent_y)) =
            frame.anchor.certified_unit_tangent().or_else(|| {
                frame
                    .anchor
                    .certified_axis_direction()
                    .map(BezierAlgebraicChordAxisDirection2::unit_tangent)
            })
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-chord-kernel",
                "exact-chord-normal-frame-line",
            );
            let radial = (
                -tangent_y * self.radial_distance(),
                tangent_x * self.radial_distance(),
            );
            let retained_line = (!line_parameter_is_chord_parameter)
                .then(|| chord.retained_support().exact_line())
                .flatten();
            let retained_orientation_reversed =
                retained_line.is_some() && chord.retained_support_orientation_is_reversed();
            return self.exact_frame_line_intersections(
                chord,
                retained_line.as_ref().unwrap_or(line),
                line_parameter_is_chord_parameter,
                clip_to_finite_chord,
                retained_orientation_reversed,
                center,
                radial,
                policy,
            );
        }

        // A selected-radial circle owns the complete circle/chord relation in
        // its compact recursive field.  Give that authority first refusal;
        // generic endpoint-derived support adapters duplicate the same solve
        // at much higher degree and can spend unbounded work proving a line
        // that this system already represents directly.
        if self.uses_selected_radial_frame() {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-chord-kernel",
                "selected-radial-algebraic-support",
            );
            if let Some(line) = exact_line.as_ref() {
                match self.recursive_selected_radial_quadratic_line_intersections(
                    chord,
                    line,
                    clip_to_finite_chord,
                    policy,
                )? {
                    Classification::Decided(Some(intersections)) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-circle-chord-kernel",
                            "recursive-selected-radial-quadratic-line",
                        );
                        return self.retain_chord_intersections(chord, intersections, policy);
                    }
                    Classification::Decided(None) | Classification::Uncertain(_) => {}
                }
            }
            match self.recursive_selected_radial_retained_offset_chord_intersections(
                chord,
                clip_to_finite_chord,
                certified_endpoint_incidence,
                policy,
            )? {
                Classification::Decided(Some(intersections)) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-circle-chord-kernel",
                        "recursive-selected-radial-retained-offset",
                    );
                    return self.retain_chord_intersections(chord, intersections, policy);
                }
                Classification::Decided(None) | Classification::Uncertain(_) => {}
            }
            let selected =
                self.selected_radial_chord_intersections(chord, clip_to_finite_chord, policy)?;
            if let Classification::Decided(intersections) = selected {
                return self.retain_chord_intersections(chord, intersections, policy);
            }
        }

        // A chord-normal circle already owns a rank-independent projective
        // system for arbitrary retained chord endpoints.  Consult that
        // authored authority before the generic recursive projective bridge:
        // importing the circle and both chord endpoints into one recursive
        // tower merely expands the same line/circle quadratic through every
        // independent endpoint field.
        if self.uses_selected_chord_normal_frame() {
            match self.chord_normal_projective_chord_system(chord, policy)? {
                Classification::Decided(Some(system)) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-circle-chord-kernel",
                        "chord-normal-projective",
                    );
                    match self.chord_normal_projective_chord_intersections(
                        chord,
                        system,
                        clip_to_finite_chord,
                        certified_endpoint_incidence,
                        policy,
                    )? {
                        Classification::Decided(intersections) => {
                            return self.retain_chord_intersections(chord, intersections, policy);
                        }
                        Classification::Uncertain(_) => {}
                    }
                }
                Classification::Decided(None) | Classification::Uncertain(_) => {}
            }
        }

        if !prefer_selected_parallel_exact_line {
            match self.recursive_projective_retained_chord_intersections(
                chord,
                clip_to_finite_chord,
                certified_endpoint_incidence,
                policy,
            )? {
                Classification::Decided(Some(intersections)) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-circle-chord-kernel",
                        "recursive-projective-retained-chord",
                    );
                    return self.retain_chord_intersections(chord, intersections, policy);
                }
                Classification::Decided(None) | Classification::Uncertain(_) => {}
            }
        }

        if (self.data.frame.rational().is_some() || self.uses_selected_chord_normal_frame())
            && exact_line.is_none()
            && let Some(line) = certified_support_line.as_ref()
        {
            let support = chord.adapt_to_certified_affine_support_line(line, policy)?;
            match self.chord_intersections_in_domain(&support, false, policy)? {
                Classification::Decided(intersections) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-circle-chord-kernel",
                        "exact-support-replay",
                    );
                    return self.reclip_retained_chord_intersections(
                        chord,
                        intersections,
                        false,
                        clip_to_finite_chord,
                        policy,
                    );
                }
                Classification::Uncertain(_) => {}
            }
        }
        if exact_line.is_none()
            && certified_support_line.is_none()
            && retained_offset_support
            && self.data.frame.rational().is_some()
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-chord-kernel",
                "retained-normal-offset",
            );
            let retained_offset = self.retained_offset_ancestor_chord_intersections(
                chord,
                chord,
                false,
                clip_to_finite_chord,
                policy,
            )?;
            if let decided @ Classification::Decided(_) = retained_offset {
                return Ok(decided);
            }
        }
        if exact_line.is_none()
            && certified_support_line.is_none()
            && self.data.frame.rational().is_some()
            && let Some((ancestor, descendant_reversed)) = chord.retained_normal_offset_ancestor()
            && !Arc::ptr_eq(&ancestor.data, &chord.data)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-chord-kernel",
                "retained-normal-offset-ancestor",
            );
            if let decided @ Classification::Decided(_) = self
                .retained_offset_ancestor_chord_intersections(
                    chord,
                    ancestor,
                    descendant_reversed,
                    clip_to_finite_chord,
                    policy,
                )?
            {
                return Ok(decided);
            }
        }
        // A procedural displaced support is more compact and more complete
        // than any Cartesian line synthesized from its endpoint provenance.
        // Consult it even when a strict affine support adapter exists: the
        // latter deliberately cannot materialize two correlated displaced
        // endpoints into independent coordinate fields.
        let procedural_support = self.represented_parallel_support_chord_intersections_in_domain(
            chord,
            clip_to_finite_chord,
            policy,
        )?;
        match procedural_support {
            Classification::Decided(Some(intersections)) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-chord-kernel",
                    "represented-parallel-support",
                );
                return Ok(Classification::Decided(intersections));
            }
            Classification::Decided(None) | Classification::Uncertain(_) => {}
        }
        // One procedural endpoint and one represented endpoint already have a
        // correlation-preserving dense line/circle kernel. Run it before the
        // generic polynomial projection so an authored shared center field is
        // not expanded into an unrelated quartic norm identity problem.
        match self.represented_parallel_endpoint_oblique_chord_intersections_in_domain(
            chord,
            clip_to_finite_chord,
            certified_endpoint_incidence,
            policy,
        )? {
            Classification::Decided(Some(intersections)) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-chord-kernel",
                    "represented-parallel-endpoint",
                );
                return self.retain_chord_intersections(chord, intersections, policy);
            }
            Classification::Decided(None) | Classification::Uncertain(_) => {}
        }
        if self.uses_selected_chord_normal_frame()
            && exact_line.is_none()
            && certified_support_line.is_none()
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-chord-kernel",
                "chord-normal-represented-fallback",
            );
            return self.retained_represented_oblique_chord_intersections_in_domain(
                chord,
                clip_to_finite_chord,
                certified_endpoint_incidence,
                policy,
            );
        }
        let Some(line) = exact_line.or(certified_support_line) else {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-chord-kernel",
                "general-algebraic-oblique",
            );
            let compact =
                self.oblique_chord_intersections_in_domain(chord, clip_to_finite_chord, policy)?;
            if let Classification::Decided(intersections) = compact {
                return self.retain_chord_intersections(chord, intersections, policy);
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-chord-kernel",
                "represented-oblique-complete",
            );
            let represented = self.retained_represented_oblique_chord_intersections_in_domain(
                chord,
                clip_to_finite_chord,
                certified_endpoint_incidence,
                policy,
            )?;
            return Ok(represented);
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-circle-chord-kernel",
            if line_parameter_is_chord_parameter && clip_to_finite_chord {
                "exact-oblique-rational-line"
            } else {
                "certified-oblique-support-line"
            },
        );
        let retained_line = (!line_parameter_is_chord_parameter)
            .then(|| chord.retained_support().exact_line())
            .flatten();
        let retained_orientation_reversed =
            retained_line.is_some() && chord.retained_support_orientation_is_reversed();
        let retained_or_solve_line = retained_line.as_ref().unwrap_or(&line);
        if self.uses_selected_parallel_normal_frame()
            && self
                .selected_frame_parameter()
                .as_ref()
                .and_then(|parameter| parameter.scalar())
                .is_some()
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-chord-kernel",
                "represented-parallel-normal-line",
            );
            return self.represented_parallel_normal_line_intersections(
                chord,
                retained_or_solve_line,
                line_parameter_is_chord_parameter,
                clip_to_finite_chord,
                retained_orientation_reversed,
                policy,
            );
        }
        if let Some(frame) = self.data.frame.rational()
            && let Some((normal_x, normal_y)) = frame.data.cardinal_normal
            && let Some(center) = self.exact_center(policy)?
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-chord-kernel",
                "exact-cardinal-frame-line",
            );
            let radial = (
                &self.data.radial_distance * Real::from(normal_x),
                &self.data.radial_distance * Real::from(normal_y),
            );
            return self.exact_frame_line_intersections(
                chord,
                retained_or_solve_line,
                line_parameter_is_chord_parameter,
                clip_to_finite_chord,
                retained_orientation_reversed,
                center,
                radial,
                policy,
            );
        }
        if self.uses_selected_parallel_normal_frame()
            && self
                .selected_frame_parameter()
                .is_some_and(|parameter| parameter.scalar().is_none())
        {
            let solve_line = if line_parameter_is_chord_parameter && clip_to_finite_chord {
                Cow::Borrowed(&line)
            } else {
                match self.affine_line_cover(&line, policy)? {
                    Classification::Decided(line) => Cow::Owned(line),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            let finite_line_domain = line_parameter_is_chord_parameter && clip_to_finite_chord;
            match self.recursive_selected_parallel_normal_line_intersections(
                chord,
                solve_line.as_ref(),
                finite_line_domain,
                certified_endpoint_incidence,
                policy,
            )? {
                Classification::Decided(intersections) => {
                    let retained =
                        match self.retain_chord_intersections(chord, intersections, policy)? {
                            Classification::Decided(retained) => retained,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                    if finite_line_domain {
                        return Ok(Classification::Decided(retained));
                    }
                    return self.reclip_retained_chord_intersections(
                        chord,
                        retained,
                        retained_orientation_reversed,
                        clip_to_finite_chord,
                        policy,
                    );
                }
                Classification::Uncertain(_) => {}
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-chord-kernel",
                "selected-parallel-normal-represented-fallback",
            );
            return self.retained_represented_oblique_chord_intersections_in_domain(
                chord,
                clip_to_finite_chord,
                certified_endpoint_incidence,
                policy,
            );
        }
        let solve_line = if line_parameter_is_chord_parameter && clip_to_finite_chord {
            Cow::Borrowed(&line)
        } else {
            match self.affine_line_cover(&line, policy)? {
                Classification::Decided(line) => Cow::Owned(line),
                Classification::Uncertain(_) => {
                    return self.retained_represented_oblique_chord_intersections_in_domain(
                        chord,
                        clip_to_finite_chord,
                        certified_endpoint_incidence,
                        policy,
                    );
                }
            }
        };
        let rational_line = RationalBezier2::try_new(
            vec![solve_line.start().clone(), solve_line.end().clone()],
            vec![Real::one(), Real::one()],
        )?;
        let (intersections, parameter_map) = match self.rational_intersections_with_parameter_map(
            &rational_line,
            &crate::CurveParameterRange2::unit(),
            policy,
        )? {
            Classification::Decided(result) => result,
            Classification::Uncertain(_) => {
                return self.retained_represented_oblique_chord_intersections_in_domain(
                    chord,
                    clip_to_finite_chord,
                    certified_endpoint_incidence,
                    policy,
                );
            }
        };
        let contacts =
            match intersections {
                BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped {
                    contacts,
                    overlaps,
                } if overlaps.is_empty() => contacts,
                other => return match other {
                    BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { .. } => {
                        Err(CurveError::Topology(
                            "a nonzero selected circle overlapped an exact line component".into(),
                        ))
                    }
                    BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber {
                        overlaps,
                        ..
                    } if !overlaps.is_empty() => Err(CurveError::Topology(
                        "a nonzero selected circle overlapped an exact line component".into(),
                    )),
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection => {
                        Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
                    }
                    BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber {
                        ..
                    } => Err(CurveError::Topology(
                        "the dedicated selected parallel-normal line kernel was bypassed".into(),
                    )),
                },
            };
        if contacts.is_empty() {
            return Ok(Classification::Decided(Vec::new()));
        }
        let mut retained = Vec::with_capacity(contacts.len());
        for contact in contacts {
            let cusp_parameter = algebraic_cusp_semicircle_endpoint_parameter(contact.location)
                .unwrap_or_else(|| {
                    parameter_map
                        .as_ref()
                        .expect("an interior selected-circle/line contact retains its map")
                        .contact_parameter_with_chord_tangent(&contact, chord)
                });
            let point = if line_parameter_is_chord_parameter
                && clip_to_finite_chord
                && contact.other_parameter.scalar().is_some_and(|parameter| {
                    compare_reals(parameter, &Real::zero(), policy)
                        == Some(std::cmp::Ordering::Equal)
                }) {
                chord.start().clone()
            } else if line_parameter_is_chord_parameter
                && clip_to_finite_chord
                && contact.other_parameter.scalar().is_some_and(|parameter| {
                    compare_reals(parameter, &Real::one(), policy)
                        == Some(std::cmp::Ordering::Equal)
                })
            {
                chord.end().clone()
            } else {
                contact.point.clone()
            };
            let chord_parameter = if clip_to_finite_chord {
                match chord.parameter_at_certified_point(point.clone(), policy)? {
                    Classification::Decided(Some(parameter)) => parameter,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(_) => {
                        return self.retained_represented_oblique_chord_intersections_in_domain(
                            chord,
                            clip_to_finite_chord,
                            certified_endpoint_incidence,
                            policy,
                        );
                    }
                }
            } else {
                chord.parameter_at_certified_support_point(point.clone(), policy)?
            };
            retained.push(BezierAlgebraicCuspSemicircleRetainedChordContact2 {
                cusp_parameter,
                chord_parameter,
                point,
                tangent_cross_sign: contact.tangent_cross_sign,
            });
        }
        Ok(Classification::Decided(retained))
    }

    /// Replays the retained chord-tangent relation at one parameter on this
    /// selected circle. `reversed` changes only traversal orientation; it
    /// never changes the selected angular parameter or its point.
    pub(in crate::bezier_offset) fn parameter_chord_tangent_relation(
        &self,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
        reversed: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(BezierAlgebraicChord2, RealSign, Option<RealSign>)>>>
    {
        if parameter
            .evidence_policy()
            .is_some_and(|retained| !policy.accepts_retained_policy(retained))
        {
            return Err(CurveError::Topology(
                "a selected-circle parameter crossed predicate policies".into(),
            ));
        }
        if let (Some(frame), BezierAlgebraicCuspSemicircleParameter2::Exact(parameter)) =
            (self.data.frame.chord_normal(), parameter)
        {
            if !policy.accepts_retained_policy(frame.policy) {
                return Err(CurveError::Topology(
                    "a chord-normal tangent crossed predicate policies".into(),
                ));
            }
            let diameter_sign = if parameter.zero_status() == ZeroKnowledge::Zero {
                RealSign::Negative
            } else if (parameter - Real::one()).zero_status() == ZeroKnowledge::Zero {
                RealSign::Positive
            } else {
                return Ok(Classification::Decided(None));
            };
            let radial_sign = match real_sign(self.radial_distance(), policy) {
                Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
                Some(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "a chord-normal circle retained a zero radius".into(),
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
            let turn_sign = if self.is_clockwise() {
                RealSign::Negative
            } else {
                RealSign::Positive
            };
            let traversal_sign = if reversed {
                RealSign::Negative
            } else {
                RealSign::Positive
            };
            return Ok(Classification::Decided(Some((
                frame.anchor.clone(),
                RealSign::Zero,
                Some(product_sign(
                    diameter_sign,
                    product_sign(radial_sign, product_sign(turn_sign, traversal_sign)),
                )),
            ))));
        }
        let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) = parameter else {
            return Ok(Classification::Decided(None));
        };
        let Some((mapped_source, tangent, circle_cross_chord, map_policy, mapped_reversed)) =
            data.coincident_chord_tangent_source()
        else {
            return Ok(Classification::Decided(None));
        };
        if !policy.accepts_retained_policy(map_policy) {
            return Err(CurveError::Topology(
                "circle/chord tangent authority was replayed under a different predicate policy"
                    .into(),
            ));
        }
        if self.data.frame != mapped_source.data.frame
            || self.is_clockwise() != mapped_source.is_clockwise()
        {
            return Ok(Classification::Decided(None));
        }
        let radial_scale_sign = match real_sign(
            &(self.radial_distance() * mapped_source.radial_distance()),
            policy,
        ) {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "selected circle/chord tangent retained a zero radius".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let reverse = reversed ^ mapped_reversed ^ (radial_scale_sign == RealSign::Negative);
        let cross = if reverse {
            product_sign(circle_cross_chord, RealSign::Negative)
        } else {
            circle_cross_chord
        };
        let dot = match data.coincident_base_data() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                semicircle,
                radial_product_sign,
                ..
            }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                semicircle,
                radial_product_sign,
                ..
            } => {
                let radial_sign = match real_sign(semicircle.radial_distance(), policy) {
                    Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
                    Some(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a selected chord-normal contact retained a zero radius".into(),
                        ));
                    }
                    None => {
                        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                    }
                };
                // At P=C+d*left_normal(D)/|D|, increasing local circle
                // parameter has tangent sign -d*turn relative to D. The
                // retained product signs r*d, so combine it with sign(r) to
                // recover sign(d), including complementary-chart contacts.
                let source_dot_from_radius = if semicircle.is_clockwise() {
                    radial_sign
                } else {
                    product_sign(radial_sign, RealSign::Negative)
                };
                let source_dot = product_sign(source_dot_from_radius, *radial_product_sign);
                Some(if reverse {
                    product_sign(source_dot, RealSign::Negative)
                } else {
                    source_dot
                })
            }
            _ => None,
        };
        Ok(Classification::Decided(Some((tangent.clone(), cross, dot))))
    }

    /// Returns the exact sign of this circle's oriented tangent dotted with
    /// one retained chord tangent.
    ///
    /// At every circle/line contact, with circle radius `R` and chord
    /// direction `D`, `T_circle = turn * perp(R)` and therefore
    /// `T_circle dot D = turn * cross(R, D)`.  The latter cross product is
    /// exactly the oriented side of the circle center relative to the chord.
    /// This predicate is contact-independent, so retain it once from the two
    /// carrier supports instead of adjoining either contact coordinate.
    pub(crate) fn chord_tangent_dot_sign(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        chord.validate_policy(policy)?;
        let support = chord.retained_support();
        let retained_offset_support = chord.exact_line().is_none()
            && matches!(
                (support.start(), support.end()),
                (
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
                ) if start.shares_carrier(end)
                    && start.at_end != end.at_end
                    && start.data.direction == BezierAlgebraicChordUnitDisplacement2::LeftNormal
            );
        if retained_offset_support && self.data.frame.rational().is_some() {
            let system = match self.retained_offset_chord_system(chord, policy)? {
                Classification::Decided(system) => system,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            return retained_offset_chord_speed_expression_sign(
                &system.retained,
                &system.retained.tangent_dot,
                policy,
            );
        }
        let center = match self.center_point_evidence(policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let side = match chord.oriented_support_side(&center, policy)? {
            Classification::Decided(side) => side,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let side_sign = match side {
            crate::classify::LineSide::Left => RealSign::Positive,
            crate::classify::LineSide::Right => RealSign::Negative,
            crate::classify::LineSide::On => RealSign::Zero,
        };
        Ok(Classification::Decided(if self.is_clockwise() {
            product_sign(side_sign, RealSign::Negative)
        } else {
            side_sign
        }))
    }
    pub(in crate::bezier_offset) fn coincident_endpoint_contacts(
        radial_same: bool,
    ) -> Vec<BezierAlgebraicCuspSemicirclePairContact2> {
        let second_start = if radial_same {
            BezierAlgebraicCuspSemicircleContactLocation2::Start
        } else {
            BezierAlgebraicCuspSemicircleContactLocation2::End
        };
        let second_end = if radial_same {
            BezierAlgebraicCuspSemicircleContactLocation2::End
        } else {
            BezierAlgebraicCuspSemicircleContactLocation2::Start
        };
        vec![
            BezierAlgebraicCuspSemicirclePairContact2 {
                branch: 0,
                first_location: BezierAlgebraicCuspSemicircleContactLocation2::Start,
                second_location: second_start,
                tangent_cross_sign: RealSign::Zero,
            },
            BezierAlgebraicCuspSemicirclePairContact2 {
                branch: 0,
                first_location: BezierAlgebraicCuspSemicircleContactLocation2::End,
                second_location: second_end,
                tangent_cross_sign: RealSign::Zero,
            },
        ]
    }
}
