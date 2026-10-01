//! Selected circle/parallel systems and their exact intersections.

use super::*;

impl BezierAlgebraicCuspSemicircle2 {
    /// Builds the exact three-root equations for a pair-radial circle against
    /// one rational Bezier.  The original circle-pair square root remains the
    /// only radical; the target parameter is a third independent selected
    /// axis and its point image stays on the target curve.
    pub(in crate::bezier_offset) fn selected_radial_rational_system(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSelectedRadialCircleRationalSystem2>> {
        let frame = match self.selected_radial_frame_system(policy)? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let target_weight_sign = match other.denominator_sign(range) {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a finite pair-radial rational candidate had a zero denominator".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let target = other.homogeneous_power_basis()?;
        let target_scale = if target_weight_sign == RealSign::Negative {
            Real::from(-1_i8)
        } else {
            Real::one()
        };
        let target_x = polynomial_scale(&target.x_numerator, &target_scale);
        let target_y = polynomial_scale(&target.y_numerator, &target_scale);
        let target_weight = polynomial_scale(&target.weight, &target_scale);
        let target_tangent_x = polynomial_subtract(
            &polynomial_multiply(&polynomial_derivative(&target_x), &target_weight),
            &polynomial_multiply(&target_x, &polynomial_derivative(&target_weight)),
        );
        let target_tangent_y = polynomial_subtract(
            &polynomial_multiply(&polynomial_derivative(&target_y), &target_weight),
            &polynomial_multiply(&target_y, &polynomial_derivative(&target_weight)),
        );

        let radial_distance = self.radial_distance().clone();
        let BezierSelectedRadialCircleFrameSystem2 {
            pair_map,
            canonical_pair_field: _,
            branch,
            discriminant,
            denominator: contact_denominator,
            center_x,
            center_y,
            radial_x,
            radial_y,
            normal_denominator,
        } = frame;
        let Some([first_cusp_parameter, second_cusp_parameter]) =
            pair_map.compact_source_parameters()
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let first_cusp_parameter = first_cusp_parameter.clone();
        let second_cusp_parameter = second_cusp_parameter.clone();
        let system = (|| {
            let axis = |coefficients: &[Real], axis| {
                TrivariatePolynomial2::from_axis_polynomial(coefficients, axis)
            };
            let x = axis(&target_x, 2)?;
            let y = axis(&target_y, 2)?;
            let weight = axis(&target_weight, 2)?;
            let tangent_x = axis(&target_tangent_x, 2)?;
            let tangent_y = axis(&target_tangent_y, 2)?;

            // S=Q-P over the positive denominator W*(2*q*D).
            let source_x_rational = TrivariatePolynomial2::sum_products(&[
                (&x, &contact_denominator, false),
                (&weight, &center_x.rational, true),
            ])?;
            let source_y_rational = TrivariatePolynomial2::sum_products(&[
                (&y, &contact_denominator, false),
                (&weight, &center_y.rational, true),
            ])?;
            let source_x_radical = weight
                .multiply(&center_x.radical)?
                .scale(&Real::from(-1_i8))?;
            let source_y_radical = weight
                .multiply(&center_y.radical)?
                .scale(&Real::from(-1_i8))?;
            let source_common_denominator = weight.multiply(&contact_denominator)?;

            let source_rational_squared = TrivariatePolynomial2::sum_products(&[
                (&source_x_rational, &source_x_rational, false),
                (&source_y_rational, &source_y_rational, false),
            ])?;
            let source_radical_squared = TrivariatePolynomial2::sum_products(&[
                (&source_x_radical, &source_x_radical, false),
                (&source_y_radical, &source_y_radical, false),
            ])?;
            let incidence_rational = source_rational_squared
                .add(&source_radical_squared.multiply(&discriminant)?)?
                .subtract(
                    &source_common_denominator
                        .multiply(&source_common_denominator)?
                        .scale(&(&radial_distance * &radial_distance))?,
                )?;
            let incidence_radical = TrivariatePolynomial2::sum_products(&[
                (&source_x_rational, &source_x_radical, false),
                (&source_y_rational, &source_y_radical, false),
            ])?
            .scale(&Real::from(2_i8))?;
            let incidence_projection = incidence_rational.multiply(&incidence_rational)?.subtract(
                &incidence_radical
                    .multiply(&incidence_radical)?
                    .multiply(&discriminant)?,
            )?;

            let radial_x_rational = &radial_x.rational;
            let radial_y_rational = &radial_y.rational;
            let radial_x_radical = &radial_x.radical;
            let radial_y_radical = &radial_y.radical;

            let cross_rational = TrivariatePolynomial2::sum_products(&[
                (radial_x_rational, &source_y_rational, false),
                (radial_y_rational, &source_x_rational, true),
            ])?
            .add(
                &TrivariatePolynomial2::sum_products(&[
                    (radial_x_radical, &source_y_radical, false),
                    (radial_y_radical, &source_x_radical, true),
                ])?
                .multiply(&discriminant)?,
            )?;
            let cross_radical = TrivariatePolynomial2::sum_products(&[
                (radial_x_rational, &source_y_radical, false),
                (radial_y_rational, &source_x_radical, true),
                (radial_x_radical, &source_y_rational, false),
                (radial_y_radical, &source_x_rational, true),
            ])?;
            let dot_rational = TrivariatePolynomial2::sum_products(&[
                (radial_x_rational, &source_x_rational, false),
                (radial_y_rational, &source_y_rational, false),
            ])?
            .add(
                &TrivariatePolynomial2::sum_products(&[
                    (radial_x_radical, &source_x_radical, false),
                    (radial_y_radical, &source_y_radical, false),
                ])?
                .multiply(&discriminant)?,
            )?;
            let dot_radical = TrivariatePolynomial2::sum_products(&[
                (radial_x_rational, &source_x_radical, false),
                (radial_y_rational, &source_y_radical, false),
                (radial_x_radical, &source_x_rational, false),
                (radial_y_radical, &source_y_rational, false),
            ])?;
            let radial_scale = &radial_distance * &normal_denominator;
            let half_scale = self.turn_sign() * &radial_scale;
            let selected_half_plane = BezierAlgebraicCuspTrivariateSquareRootExpression2 {
                rational: cross_rational.scale(&half_scale)?,
                radical: cross_radical.scale(&half_scale)?,
            };
            let diameter = BezierAlgebraicCuspTrivariateSquareRootExpression2 {
                rational: dot_rational.scale(&radial_scale)?,
                radical: dot_radical.scale(&radial_scale)?,
            };
            // Multiplying the physical diameter coordinate by
            // n^2*W*(2*q*D)^2 is positive and yields the expression above.
            let radius_squared_denominator = weight
                .multiply(&contact_denominator.multiply(&contact_denominator)?)?
                .scale(
                    &(&radial_distance
                        * &radial_distance
                        * &normal_denominator
                        * &normal_denominator),
                )?;

            // cross(T_circle,Q') = -turn*dot(S,Q'); all omitted projective
            // denominators are positive after weight normalization.
            let tangent_scale = -self.turn_sign();
            let tangent_cross = BezierAlgebraicCuspTrivariateSquareRootExpression2 {
                rational: TrivariatePolynomial2::sum_products(&[
                    (&source_x_rational, &tangent_x, false),
                    (&source_y_rational, &tangent_y, false),
                ])?
                .scale(&tangent_scale)?,
                radical: TrivariatePolynomial2::sum_products(&[
                    (&source_x_radical, &tangent_x, false),
                    (&source_y_radical, &tangent_y, false),
                ])?
                .scale(&tangent_scale)?,
            };
            let angular_tangent = BezierAlgebraicCuspTrivariateSquareRootExpression2 {
                rational: TrivariatePolynomial2::sum_products(&[
                    (&source_x_rational, &tangent_y, false),
                    (&source_y_rational, &tangent_x, true),
                ])?,
                radical: TrivariatePolynomial2::sum_products(&[
                    (&source_x_radical, &tangent_y, false),
                    (&source_y_radical, &tangent_x, true),
                ])?,
            };
            let reduce = |polynomial: TrivariatePolynomial2| {
                trivariate_reduce_parameter_pair_relations(
                    &polynomial,
                    &first_cusp_parameter,
                    &second_cusp_parameter,
                )
                .unwrap_or(polynomial)
            };
            let discriminant = reduce(discriminant);
            let incidence = BezierAlgebraicCuspTrivariateSquareRootExpression2 {
                rational: reduce(incidence_rational),
                radical: reduce(incidence_radical),
            };
            let incidence_projection = reduce(incidence_projection);
            let selected_half_plane = BezierAlgebraicCuspTrivariateSquareRootExpression2 {
                rational: reduce(selected_half_plane.rational),
                radical: reduce(selected_half_plane.radical),
            };
            let diameter = BezierAlgebraicCuspTrivariateSquareRootExpression2 {
                rational: reduce(diameter.rational),
                radical: reduce(diameter.radical),
            };
            let radius_squared_denominator = reduce(radius_squared_denominator);
            let tangent_cross = BezierAlgebraicCuspTrivariateSquareRootExpression2 {
                rational: reduce(tangent_cross.rational),
                radical: reduce(tangent_cross.radical),
            };
            let angular_tangent = BezierAlgebraicCuspTrivariateSquareRootExpression2 {
                rational: reduce(angular_tangent.rational),
                radical: reduce(angular_tangent.radical),
            };
            Some(BezierSelectedRadialCircleRationalSystem2 {
                pair_map,
                branch,
                discriminant,
                incidence,
                incidence_projection,
                selected_half_plane,
                diameter,
                radius_squared_denominator,
                tangent_cross,
                angular_tangent,
            })
        })();
        Ok(system.map_or(
            Classification::Uncertain(UncertaintyReason::Unsupported),
            Classification::Decided,
        ))
    }

    /// Builds the minimal-degree three-axis backend for a direct pair-radial circle
    /// against one analytic parallel. The target point is
    ///
    /// `Q = (X/W,Y/W) + d*J(H)/sqrt(S)`.
    ///
    /// If the selected circle center numerator is `C_r+C_k*k` over positive
    /// denominator `D`, set `V=(X*D-W*C)` and `M=W*D*d*J(H)`. Multiplication
    /// by the positive scale `W^2*D^2*sqrt(S)` turns circle incidence into
    ///
    /// `sqrt(S) * (|V|^2 + W^2*D^2*(d^2-r^2)) + 2*V dot M = 0`.
    ///
    /// Both vector terms remain affine in the authored pair radical `k`.
    /// Thus every predicate fits the shared two-square-root expression and
    /// avoids independently eliminating center and radial coordinates.
    pub(in crate::bezier_offset) fn direct_pair_radial_parallel_fast_path(
        &self,
        other: &BezierParallel2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Arc<BezierDirectPairRadialParallelFastPath2>>> {
        let frame = match self.selected_radial_frame_system(policy)? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let source = other.source_power_basis()?;
        let differential = other.differential()?;
        let target_weight = source
            .weight
            .map_or_else(|| vec![Real::one()], <[Real]>::to_vec);
        let target_speed_squared = polynomial_add(
            &polynomial_multiply(&differential.tangent_x, &differential.tangent_x),
            &polynomial_multiply(&differential.tangent_y, &differential.tangent_y),
        );
        let radial_distance = self.radial_distance().clone();
        let turn = self.turn_sign();
        let BezierSelectedRadialCircleFrameSystem2 {
            pair_map,
            canonical_pair_field: _,
            branch,
            discriminant: pair_discriminant,
            denominator: contact_denominator,
            center_x,
            center_y,
            radial_x,
            radial_y,
            normal_denominator,
        } = frame;
        let Some([first_parameter, second_parameter]) = pair_map.compact_source_parameters() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let system = (|| {
            let axis = |coefficients: &[Real]| {
                TrivariatePolynomial2::from_axis_polynomial_or_zero(coefficients, 2)
            };
            let zero = || TrivariatePolynomial2::from_axis_polynomial(&[Real::zero()], 2);
            let x = axis(source.x_numerator)?;
            let y = axis(source.y_numerator)?;
            let weight = axis(&target_weight)?;
            let tangent_x = axis(&differential.tangent_x)?;
            let tangent_y = axis(&differential.tangent_y)?;
            let candidate_speed_squared = axis(&target_speed_squared)?;
            let common_denominator = weight.multiply(&contact_denominator)?;
            let common_denominator_squared = common_denominator.multiply(&common_denominator)?;

            // V=W*D*(P-C), retained as one value in the pair field.
            let source_x = BezierAlgebraicCuspTrivariateSquareRootExpression2::from_rational(
                x.multiply(&contact_denominator)?,
            )?
            .subtract(&center_x.multiply_rational(&weight)?)?;
            let source_y = BezierAlgebraicCuspTrivariateSquareRootExpression2::from_rational(
                y.multiply(&contact_denominator)?,
            )?
            .subtract(&center_y.multiply_rational(&weight)?)?;

            // M=W*D*d*J(H). It is rational in the retained pair field and
            // perpendicular to H by construction.
            let normal_x = tangent_y.scale(&(-other.distance()))?;
            let normal_y = tangent_x.scale(other.distance())?;
            let normal_common_x = common_denominator.multiply(&normal_x)?;
            let normal_common_y = common_denominator.multiply(&normal_y)?;

            let source_squared = source_x
                .square(&pair_discriminant)?
                .add(&source_y.square(&pair_discriminant)?)?;
            let distance_radius =
                other.distance() * other.distance() - &radial_distance * &radial_distance;
            let circle_speed_group = source_squared.add(
                &BezierAlgebraicCuspTrivariateSquareRootExpression2::from_rational(
                    common_denominator_squared.scale(&distance_radius)?,
                )?,
            )?;
            let circle_retained_group = source_x
                .multiply_rational(&normal_common_x)?
                .add(&source_y.multiply_rational(&normal_common_y)?)?
                .scale(&Real::from(2_i8))?;
            let incidence = BezierAlgebraicCuspTrivariateTwoSquareRootExpression2 {
                product: circle_speed_group.radical,
                pair: circle_retained_group.radical,
                candidate: circle_speed_group.rational,
                rational: circle_retained_group.rational,
            };

            let pair_cross =
                |first_x: &BezierAlgebraicCuspTrivariateSquareRootExpression2,
                 first_y: &BezierAlgebraicCuspTrivariateSquareRootExpression2,
                 second_x: &BezierAlgebraicCuspTrivariateSquareRootExpression2,
                 second_y: &BezierAlgebraicCuspTrivariateSquareRootExpression2| {
                    first_x
                        .multiply(second_y, &pair_discriminant)?
                        .subtract(&first_y.multiply(second_x, &pair_discriminant)?)
                };
            let pair_dot =
                |first_x: &BezierAlgebraicCuspTrivariateSquareRootExpression2,
                 first_y: &BezierAlgebraicCuspTrivariateSquareRootExpression2,
                 second_x: &BezierAlgebraicCuspTrivariateSquareRootExpression2,
                 second_y: &BezierAlgebraicCuspTrivariateSquareRootExpression2| {
                    first_x
                        .multiply(second_x, &pair_discriminant)?
                        .add(&first_y.multiply(second_y, &pair_discriminant)?)
                };
            let rational_pair = |polynomial: TrivariatePolynomial2| {
                BezierAlgebraicCuspTrivariateSquareRootExpression2::from_rational(polynomial)
            };
            let normal_common_x_pair = rational_pair(normal_common_x.clone())?;
            let normal_common_y_pair = rational_pair(normal_common_y.clone())?;
            let source_cross = pair_cross(&radial_x, &radial_y, &source_x, &source_y)?;
            let normal_cross = pair_cross(
                &radial_x,
                &radial_y,
                &normal_common_x_pair,
                &normal_common_y_pair,
            )?;
            let source_dot = pair_dot(&radial_x, &radial_y, &source_x, &source_y)?;
            let normal_dot = pair_dot(
                &radial_x,
                &radial_y,
                &normal_common_x_pair,
                &normal_common_y_pair,
            )?;
            let radial_scale = &radial_distance * &normal_denominator;
            let angular_expression =
                |speed_group: BezierAlgebraicCuspTrivariateSquareRootExpression2,
                 retained_group: BezierAlgebraicCuspTrivariateSquareRootExpression2,
                 scale: &Real| {
                    Some(BezierAlgebraicCuspTrivariateTwoSquareRootExpression2 {
                        product: speed_group.radical.multiply(&weight)?.scale(scale)?,
                        pair: retained_group.radical.multiply(&weight)?.scale(scale)?,
                        candidate: speed_group.rational.multiply(&weight)?.scale(scale)?,
                        rational: retained_group.rational.multiply(&weight)?.scale(scale)?,
                    })
                };
            let selected_half_plane =
                angular_expression(source_cross, normal_cross, &(&turn * &radial_scale))?;
            let diameter = angular_expression(source_dot, normal_dot, &radial_scale)?;
            let radius_squared_denominator =
                BezierAlgebraicCuspTrivariateTwoSquareRootExpression2 {
                    product: zero()?,
                    pair: zero()?,
                    candidate: common_denominator_squared.scale(
                        &(&radial_distance
                            * &radial_distance
                            * &normal_denominator
                            * &normal_denominator),
                    )?,
                    rational: zero()?,
                };

            // The parallel derivative is a scalar multiple of H on every
            // regular cell. These retain the circle/source-tangent cross and
            // dot signs under one common positive W^2*D scale.
            let tangent_x_pair = rational_pair(tangent_x.clone())?;
            let tangent_y_pair = rational_pair(tangent_y.clone())?;
            let source_dot_tangent =
                pair_dot(&source_x, &source_y, &tangent_x_pair, &tangent_y_pair)?;
            let tangent_cross_source = source_dot_tangent
                .multiply_rational(&weight)?
                .scale(&(-&turn))?;
            let source_cross_tangent =
                pair_cross(&source_x, &source_y, &tangent_x_pair, &tangent_y_pair)?;
            let normal_cross_tangent = TrivariatePolynomial2::sum_products(&[
                (&normal_common_x, &tangent_y, false),
                (&normal_common_y, &tangent_x, true),
            ])?;
            let tangent_dot_source = BezierAlgebraicCuspTrivariateTwoSquareRootExpression2 {
                product: source_cross_tangent
                    .radical
                    .multiply(&weight)?
                    .scale(&turn)?,
                pair: zero()?,
                candidate: source_cross_tangent
                    .rational
                    .multiply(&weight)?
                    .scale(&turn)?,
                rational: normal_cross_tangent.multiply(&weight)?.scale(&turn)?,
            };

            let reduce = |polynomial: TrivariatePolynomial2| {
                trivariate_reduce_parameter_pair_relations(
                    &polynomial,
                    &first_parameter,
                    &second_parameter,
                )
                .unwrap_or(polynomial)
            };
            let reduce_pair = |expression: BezierAlgebraicCuspTrivariateSquareRootExpression2| {
                BezierAlgebraicCuspTrivariateSquareRootExpression2 {
                    rational: reduce(expression.rational),
                    radical: reduce(expression.radical),
                }
            };
            let reduce_two = |expression: BezierAlgebraicCuspTrivariateTwoSquareRootExpression2| {
                BezierAlgebraicCuspTrivariateTwoSquareRootExpression2 {
                    product: reduce(expression.product),
                    pair: reduce(expression.pair),
                    candidate: reduce(expression.candidate),
                    rational: reduce(expression.rational),
                }
            };
            let pair_discriminant = reduce(pair_discriminant);
            let candidate_speed_squared = reduce(candidate_speed_squared);
            let incidence = reduce_two(incidence);
            let incidence_candidate_norm = reduce_pair(
                incidence.candidate_norm(&pair_discriminant, &candidate_speed_squared)?,
            );
            let incidence_projection =
                reduce(incidence_candidate_norm.projection(&pair_discriminant)?);
            Some(Arc::new(BezierDirectPairRadialParallelFastPath2 {
                pair_map,
                branch,
                pair_discriminant,
                candidate_speed_squared,
                incidence,
                incidence_candidate_norm,
                incidence_projection,
                selected_half_plane: reduce_two(selected_half_plane),
                diameter: reduce_two(diameter),
                radius_squared_denominator: reduce_two(radius_squared_denominator),
                tangent_cross_source: reduce_pair(tangent_cross_source),
                tangent_dot_source: reduce_two(tangent_dot_source),
                target_weight,
                target_speed_squared,
            }))
        })();
        Ok(system.map_or(
            Classification::Uncertain(UncertaintyReason::Unsupported),
            Classification::Decided,
        ))
    }

    /// Builds the exact equations needed to intersect this selected half circle
    /// with one finite rational Bezier. The first variable is the retained cusp
    /// parameter; the second is the rational curve parameter.
    pub(in crate::bezier_offset) fn selected_parallel_normal_rational_system(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSelectedParallelNormalCircleRationalSystem2>> {
        let frame = self.data.frame.parallel_normal().ok_or_else(|| {
            CurveError::Topology(
                "a rational selected-circle frame entered the parallel-normal system".into(),
            )
        })?;
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a parallel-normal selected circle crossed predicate policies".into(),
            ));
        }
        // Circle construction certified the finite source and its nonzero
        // tangent at this selected parameter. Other source parameters are not
        // part of the circle: a remote cusp or pole cannot invalidate it.
        let source = frame.center_support.source_power_basis()?;
        let differential = frame.center_support.differential()?;
        match other.denominator_sign(range) {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a finite rational circle candidate had a zero denominator".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        let other = other.homogeneous_power_basis()?;
        let unit = [Real::one()];
        let source_weight = source.weight.unwrap_or(&unit);
        let delta_x = bivariate_parameter_difference(
            source_weight,
            &other.x_numerator,
            source.x_numerator,
            &other.weight,
        );
        let delta_y = bivariate_parameter_difference(
            source_weight,
            &other.y_numerator,
            source.y_numerator,
            &other.weight,
        );
        let weight = bivariate_outer_product(source_weight, &other.weight);
        let weight_squared = bivariate_multiply(&weight, &weight);
        let speed_squared = bivariate_outer_product(
            &polynomial_add(
                &polynomial_multiply(&differential.tangent_x, &differential.tangent_x),
                &polynomial_multiply(&differential.tangent_y, &differential.tangent_y),
            ),
            &unit,
        );
        let normal_projection = bivariate_subtract(
            &bivariate_multiply_first_parameter(&delta_y, &differential.tangent_x),
            &bivariate_multiply_first_parameter(&delta_x, &differential.tangent_y),
        );
        let tangent_projection = bivariate_add(
            &bivariate_multiply_first_parameter(&delta_x, &differential.tangent_x),
            &bivariate_multiply_first_parameter(&delta_y, &differential.tangent_y),
        );
        let center_distance = frame.center_support.distance();
        let radius = self.radial_distance();
        let rational = bivariate_add(
            &bivariate_add(
                &bivariate_multiply(&delta_x, &delta_x),
                &bivariate_multiply(&delta_y, &delta_y),
            ),
            &bivariate_scale(
                weight_squared.clone(),
                &(center_distance * center_distance - radius * radius),
            ),
        );
        let radical = bivariate_scale(
            bivariate_multiply(&weight, &normal_projection),
            &(Real::from(-2_i8) * center_distance),
        );
        let incidence = bivariate_subtract(
            &bivariate_multiply(&bivariate_multiply(&rational, &rational), &speed_squared),
            &bivariate_multiply(&radical, &radical),
        );
        let selected_half_plane = bivariate_scale(
            bivariate_multiply(&weight, &tangent_projection),
            &(-self.turn_sign() * radius),
        );
        let diameter = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_scale(weight_squared.clone(), &(-radius * center_distance)),
            radical: bivariate_scale(bivariate_multiply(&weight, &normal_projection), radius),
        };
        let radius_squared_denominator =
            bivariate_scale(weight_squared.clone(), &(radius * radius));

        let other_x_derivative = polynomial_subtract(
            &polynomial_multiply(&polynomial_derivative(&other.x_numerator), &other.weight),
            &polynomial_multiply(&other.x_numerator, &polynomial_derivative(&other.weight)),
        );
        let other_y_derivative = polynomial_subtract(
            &polynomial_multiply(&polynomial_derivative(&other.y_numerator), &other.weight),
            &polynomial_multiply(&other.y_numerator, &polynomial_derivative(&other.weight)),
        );
        let other_x_derivative = bivariate_outer_product(&unit, &other_x_derivative);
        let other_y_derivative = bivariate_outer_product(&unit, &other_y_derivative);
        let radial_dot_tangent = bivariate_add(
            &bivariate_multiply(&delta_x, &other_x_derivative),
            &bivariate_multiply(&delta_y, &other_y_derivative),
        );
        let center_cross_tangent = bivariate_subtract(
            &bivariate_multiply_first_parameter(&other_y_derivative, &differential.tangent_x),
            &bivariate_multiply_first_parameter(&other_x_derivative, &differential.tangent_y),
        );
        let radial_cross_tangent = bivariate_subtract(
            &bivariate_multiply(&delta_x, &other_y_derivative),
            &bivariate_multiply(&delta_y, &other_x_derivative),
        );
        let source_tangent_dot_tangent = bivariate_add(
            &bivariate_multiply_first_parameter(&other_x_derivative, &differential.tangent_x),
            &bivariate_multiply_first_parameter(&other_y_derivative, &differential.tangent_y),
        );
        let tangent_scale = -self.turn_sign();
        let tangent_cross = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_scale(
                bivariate_multiply(&weight, &radial_dot_tangent),
                &tangent_scale,
            ),
            radical: bivariate_scale(
                bivariate_multiply(&weight_squared, &center_cross_tangent),
                &(&tangent_scale * -center_distance.clone()),
            ),
        };
        let angular_tangent = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_multiply(&weight, &radial_cross_tangent),
            radical: bivariate_scale(
                bivariate_multiply(&weight_squared, &source_tangent_dot_tangent),
                center_distance,
            ),
        };
        Ok(Classification::Decided(
            BezierSelectedParallelNormalCircleRationalSystem2 {
                incidence,
                circle: BezierAlgebraicCuspTwoTermExpression2 { rational, radical },
                selected_half_plane,
                diameter,
                radius_squared_denominator,
                speed_squared,
                tangent_cross,
                angular_tangent,
            },
        ))
    }

    /// Builds the retained two-normal system for a general selected circle
    /// against an independently parameterized analytic parallel.
    ///
    /// The fixed-distance projection is shared with analytic chamfers. The
    /// additional expressions select this finite semicircle, distinguish its
    /// diameter endpoints, and orient target tangency without flattening
    /// either positive source-speed radical.
    pub(in crate::bezier_offset) fn selected_parallel_normal_parallel_system(
        &self,
        other: &BezierParallel2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSelectedParallelNormalCircleParallelSystem2>> {
        let frame = self.data.frame.parallel_normal().ok_or_else(|| {
            CurveError::Topology(
                "a rational selected-circle frame entered the two-normal system".into(),
            )
        })?;
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a parallel-normal selected circle crossed predicate policies".into(),
            ));
        }
        let radius_squared = self.radial_distance() * self.radial_distance();
        let BezierParallelFixedDistanceSystem2 {
            incidence,
            center_speed_squared,
            candidate_speed_squared,
            squared_branch,
            circle,
        } = match parallel_fixed_distance_system(
            &frame.center_support,
            other,
            &radius_squared,
            range,
            &frame.center_parameter,
            policy,
        )? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        let center_source = frame.center_support.source_power_basis()?;
        let candidate_source = other.source_power_basis()?;
        let center_differential = frame.center_support.differential()?;
        let candidate_differential = other.differential()?;
        let unit = [Real::one()];
        let center_weight = center_source.weight.unwrap_or(&unit);
        let candidate_weight = candidate_source.weight.unwrap_or(&unit);
        let weight = bivariate_outer_product(center_weight, candidate_weight);
        let weight_squared = bivariate_multiply(&weight, &weight);
        let delta_x = bivariate_parameter_difference(
            center_weight,
            candidate_source.x_numerator,
            center_source.x_numerator,
            candidate_weight,
        );
        let delta_y = bivariate_parameter_difference(
            center_weight,
            candidate_source.y_numerator,
            center_source.y_numerator,
            candidate_weight,
        );
        let center_tangent_projection = bivariate_add(
            &bivariate_multiply_first_parameter(&delta_x, &center_differential.tangent_x),
            &bivariate_multiply_first_parameter(&delta_y, &center_differential.tangent_y),
        );
        let candidate_tangent_x = bivariate_outer_product(&unit, &candidate_differential.tangent_x);
        let candidate_tangent_y = bivariate_outer_product(&unit, &candidate_differential.tangent_y);
        let candidate_tangent_projection = bivariate_add(
            &bivariate_multiply(&delta_x, &candidate_tangent_x),
            &bivariate_multiply(&delta_y, &candidate_tangent_y),
        );
        let candidate_tangent_cross = bivariate_subtract(
            &bivariate_multiply(&delta_x, &candidate_tangent_y),
            &bivariate_multiply(&delta_y, &candidate_tangent_x),
        );
        let center_normal_projection = bivariate_subtract(
            &bivariate_multiply_first_parameter(&delta_y, &center_differential.tangent_x),
            &bivariate_multiply_first_parameter(&delta_x, &center_differential.tangent_y),
        );
        let tangent_cross = bivariate_subtract(
            &bivariate_outer_product(
                &center_differential.tangent_x,
                &candidate_differential.tangent_y,
            ),
            &bivariate_outer_product(
                &center_differential.tangent_y,
                &candidate_differential.tangent_x,
            ),
        );
        let tangent_dot = bivariate_add(
            &bivariate_outer_product(
                &center_differential.tangent_x,
                &candidate_differential.tangent_x,
            ),
            &bivariate_outer_product(
                &center_differential.tangent_y,
                &candidate_differential.tangent_y,
            ),
        );
        let half_scale = -self.turn_sign() * self.radial_distance();
        let selected_half_plane = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_scale(
                bivariate_multiply(&weight, &center_tangent_projection),
                &half_scale,
            ),
            radical: bivariate_scale(
                bivariate_multiply(&weight_squared, &tangent_cross),
                &(-half_scale.clone() * other.distance()),
            ),
        };
        let diameter = BezierParallelTwoNormalExpression2 {
            product: bivariate_scale(
                weight_squared.clone(),
                &(-self.radial_distance() * frame.center_support.distance()),
            ),
            center: BivariatePolynomial::new(vec![vec![Real::zero()]]),
            candidate: bivariate_scale(
                bivariate_multiply(&weight, &center_normal_projection),
                self.radial_distance(),
            ),
            rational: bivariate_scale(
                bivariate_multiply(&weight_squared, &tangent_dot),
                &(self.radial_distance() * other.distance()),
            ),
        };
        let radius_squared_denominator = bivariate_scale(weight_squared.clone(), &radius_squared);
        let tangent_cross_source = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_scale(
                bivariate_multiply(&weight, &candidate_tangent_projection),
                &(-self.turn_sign()),
            ),
            radical: bivariate_scale(
                bivariate_multiply(&weight_squared, &tangent_cross),
                &(self.turn_sign() * frame.center_support.distance()),
            ),
        };
        // If A and B are the two homogeneous source tangents, x=|A|,
        // y=|B|, W is the common position denominator, and D is the
        // center-to-candidate position numerator, the selected-circle
        // tangent dot B has the sign of
        //
        //   turn * (W*x*(D cross B) - d_b*W^2*x*y
        //           + d_a*W^2*(A dot B)).
        //
        // This has the same positive W^2*x scale as
        // `tangent_cross_source` after that one-normal expression is lifted
        // into the two-normal basis. Retaining both expressions lets later
        // round and miter predicates sign arbitrary linear combinations
        // without constructing either unit tangent.
        let tangent_dot_source = BezierParallelTwoNormalExpression2 {
            product: bivariate_scale(
                weight_squared.clone(),
                &(-self.turn_sign() * other.distance()),
            ),
            center: bivariate_scale(
                bivariate_multiply(&weight, &candidate_tangent_cross),
                &self.turn_sign(),
            ),
            candidate: BivariatePolynomial::new(vec![vec![Real::zero()]]),
            rational: bivariate_scale(
                bivariate_multiply(&weight_squared, &tangent_dot),
                &(self.turn_sign() * frame.center_support.distance()),
            ),
        };
        Ok(Classification::Decided(
            BezierSelectedParallelNormalCircleParallelSystem2 {
                incidence,
                squared_branch,
                circle,
                selected_half_plane,
                diameter,
                radius_squared_denominator,
                tangent_cross_source,
                tangent_dot_source,
                center_speed_squared,
                candidate_speed_squared,
            },
        ))
    }

    pub(crate) fn rational_system(
        &self,
        other: &RationalBezier2,
    ) -> CurveResult<BezierAlgebraicCuspCircleRationalSystem2> {
        let frame = &self.data.frame.rational_required()?.data;
        let other = other.homogeneous_power_basis()?;
        let (center_x, center_y) = self
            .data
            .frame
            .point_numerators_at_parallel_distance(&self.center_parallel_distance())?;
        let dx = bivariate_parameter_difference(
            &frame.denominator,
            &other.x_numerator,
            &center_x,
            &other.weight,
        );
        let dy = bivariate_parameter_difference(
            &frame.denominator,
            &other.y_numerator,
            &center_y,
            &other.weight,
        );
        let common_denominator = bivariate_outer_product(&frame.denominator, &other.weight);
        let radius_squared_denominator = bivariate_scale(
            bivariate_multiply(&common_denominator, &common_denominator),
            &(&self.data.radial_distance * &self.data.radial_distance),
        );
        let incidence = bivariate_subtract(
            &bivariate_add(&bivariate_multiply(&dx, &dx), &bivariate_multiply(&dy, &dy)),
            &radius_squared_denominator,
        );

        // Multiplication by W changes `cross(R, Q-C)` into a quantity with
        // denominator W^2 D_c^2, so its sign is independent of projective
        // weight and frame-denominator orientation.
        let radial_cross = bivariate_subtract(
            &bivariate_multiply_first_parameter(&dy, &frame.normal_x_numerator),
            &bivariate_multiply_first_parameter(&dx, &frame.normal_y_numerator),
        );
        let other_weight = bivariate_outer_product(&[Real::one()], &other.weight);
        let selected_half_plane = bivariate_scale(
            bivariate_multiply(&radial_cross, &other_weight),
            &(&self.turn_sign() * &self.data.radial_distance),
        );
        let radial_dot = bivariate_add(
            &bivariate_multiply_first_parameter(&dx, &frame.normal_x_numerator),
            &bivariate_multiply_first_parameter(&dy, &frame.normal_y_numerator),
        );
        let diameter_side = bivariate_scale(
            bivariate_multiply(&radial_dot, &other_weight),
            &self.data.radial_distance,
        );

        let other_x_derivative = polynomial_subtract(
            &polynomial_multiply(&polynomial_derivative(&other.x_numerator), &other.weight),
            &polynomial_multiply(&other.x_numerator, &polynomial_derivative(&other.weight)),
        );
        let other_y_derivative = polynomial_subtract(
            &polynomial_multiply(&polynomial_derivative(&other.y_numerator), &other.weight),
            &polynomial_multiply(&other.y_numerator, &polynomial_derivative(&other.weight)),
        );
        let other_x_derivative_bivariate =
            bivariate_outer_product(&[Real::one()], &other_x_derivative);
        let other_y_derivative_bivariate =
            bivariate_outer_product(&[Real::one()], &other_y_derivative);
        let radial_dot_tangent = bivariate_add(
            &bivariate_multiply(&dx, &other_x_derivative_bivariate),
            &bivariate_multiply(&dy, &other_y_derivative_bivariate),
        );
        let tangent_cross = bivariate_scale(
            bivariate_multiply(&radial_dot_tangent, &common_denominator),
            &(-self.turn_sign()),
        );
        let angular_tangent = bivariate_multiply(
            &bivariate_subtract(
                &bivariate_multiply(&dx, &other_y_derivative_bivariate),
                &bivariate_multiply(&dy, &other_x_derivative_bivariate),
            ),
            &common_denominator,
        );
        Ok(BezierAlgebraicCuspCircleRationalSystem2 {
            incidence,
            selected_half_plane,
            diameter_side,
            radius_squared_denominator,
            tangent_cross,
            angular_tangent,
        })
    }

    /// Formal homogeneous equations. Intersection admission owns the finite
    /// source/normal proof; retained maps only replay its certified contacts.
    pub(in crate::bezier_offset) fn parallel_system(
        &self,
        other: &BezierParallel2,
    ) -> CurveResult<BezierAlgebraicCuspSemicircleParallelSystem2> {
        let source = other.source_power_basis()?;
        let differential = other.differential()?;

        let unit_weight = [Real::one()];
        let weight = source.weight.unwrap_or(&unit_weight);
        let frame = &self.data.frame.rational_required()?.data;
        let (center_x, center_y) = self
            .data
            .frame
            .point_numerators_at_parallel_distance(&self.center_parallel_distance())?;
        let delta_x = bivariate_parameter_difference(
            &frame.denominator,
            source.x_numerator,
            &center_x,
            weight,
        );
        let delta_y = bivariate_parameter_difference(
            &frame.denominator,
            source.y_numerator,
            &center_y,
            weight,
        );
        let common_denominator = bivariate_outer_product(&frame.denominator, weight);
        let common_denominator_squared =
            bivariate_multiply(&common_denominator, &common_denominator);
        let speed_squared = polynomial_add(
            &polynomial_multiply(&differential.tangent_x, &differential.tangent_x),
            &polynomial_multiply(&differential.tangent_y, &differential.tangent_y),
        );
        let speed_squared = bivariate_outer_product(&unit_weight, &speed_squared);
        let tangent_x = bivariate_outer_product(&unit_weight, &differential.tangent_x);
        let tangent_y = bivariate_outer_product(&unit_weight, &differential.tangent_y);
        let squared_delta = bivariate_add(
            &bivariate_multiply(&delta_x, &delta_x),
            &bivariate_multiply(&delta_y, &delta_y),
        );
        let distance_square_delta = other.distance() * other.distance()
            - &self.data.radial_distance * &self.data.radial_distance;
        let circle_rational = bivariate_add(
            &squared_delta,
            &bivariate_scale(common_denominator_squared.clone(), &distance_square_delta),
        );
        let delta_dot_normal_numerator = bivariate_subtract(
            &bivariate_multiply(&delta_x, &tangent_y),
            &bivariate_multiply(&delta_y, &tangent_x),
        );
        let circle_radical = bivariate_scale(
            bivariate_multiply(&delta_dot_normal_numerator, &common_denominator),
            &(-Real::from(2_i8) * other.distance()),
        );
        let incidence = bivariate_subtract(
            &bivariate_multiply(
                &bivariate_multiply(&circle_rational, &circle_rational),
                &speed_squared,
            ),
            &bivariate_multiply(&circle_radical, &circle_radical),
        );

        let weight_polynomial = bivariate_outer_product(&unit_weight, weight);
        let weight_squared = polynomial_multiply(weight, weight);
        let frame_denominator_weight_squared =
            bivariate_outer_product(&frame.denominator, &weight_squared);
        let frame_normal_cross_delta = bivariate_subtract(
            &bivariate_multiply_first_parameter(&delta_y, &frame.normal_x_numerator),
            &bivariate_multiply_first_parameter(&delta_x, &frame.normal_y_numerator),
        );
        let frame_normal_dot_tangent = bivariate_add(
            &bivariate_outer_product(&frame.normal_x_numerator, &differential.tangent_x),
            &bivariate_outer_product(&frame.normal_y_numerator, &differential.tangent_y),
        );
        let half_scale = &self.turn_sign() * &self.data.radial_distance;
        let selected_half_plane = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_scale(
                bivariate_multiply(&frame_normal_cross_delta, &weight_polynomial),
                &half_scale,
            ),
            radical: bivariate_scale(
                bivariate_multiply(&frame_normal_dot_tangent, &frame_denominator_weight_squared),
                &(other.distance() * &half_scale),
            ),
        };

        let frame_normal_dot_delta = bivariate_add(
            &bivariate_multiply_first_parameter(&delta_x, &frame.normal_x_numerator),
            &bivariate_multiply_first_parameter(&delta_y, &frame.normal_y_numerator),
        );
        let frame_normal_cross_tangent = bivariate_subtract(
            &bivariate_outer_product(&frame.normal_x_numerator, &differential.tangent_y),
            &bivariate_outer_product(&frame.normal_y_numerator, &differential.tangent_x),
        );
        let diameter_side = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_scale(
                bivariate_multiply(&frame_normal_dot_delta, &weight_polynomial),
                &self.data.radial_distance,
            ),
            radical: bivariate_scale(
                bivariate_multiply(
                    &frame_normal_cross_tangent,
                    &frame_denominator_weight_squared,
                ),
                &(-other.distance() * &self.data.radial_distance),
            ),
        };
        let radius_squared_denominator = bivariate_scale(
            common_denominator_squared.clone(),
            &(&self.data.radial_distance * &self.data.radial_distance),
        );
        let delta_dot_tangent = bivariate_add(
            &bivariate_multiply(&delta_x, &tangent_x),
            &bivariate_multiply(&delta_y, &tangent_y),
        );
        let tangent_cross_source = bivariate_scale(
            bivariate_multiply(&delta_dot_tangent, &common_denominator),
            &(-self.turn_sign()),
        );
        // The selected-circle tangent is `turn * J(radius)`.  Its dot with
        // the target source tangent therefore has the sign of
        //
        //   turn * (delta cross tangent - distance * |tangent|).
        //
        // Multiplying by the square of the common point denominator keeps its
        // sign independent of both projective gauges and leaves one positive
        // speed radical for exact branch replay.
        let delta_cross_tangent = bivariate_subtract(
            &bivariate_multiply(&delta_x, &tangent_y),
            &bivariate_multiply(&delta_y, &tangent_x),
        );
        let tangent_dot_source = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_scale(
                bivariate_multiply(&delta_cross_tangent, &common_denominator),
                &self.turn_sign(),
            ),
            radical: bivariate_scale(
                bivariate_multiply(&common_denominator_squared, &speed_squared),
                &(-self.turn_sign() * other.distance()),
            ),
        };
        Ok(BezierAlgebraicCuspSemicircleParallelSystem2 {
            incidence,
            circle: BezierAlgebraicCuspTwoTermExpression2 {
                rational: circle_rational,
                radical: circle_radical,
            },
            selected_half_plane,
            diameter_side,
            radius_squared_denominator,
            speed_squared,
            tangent_cross_source,
            tangent_dot_source,
        })
    }

    /// Signs one linear combination of this circle tangent crossed and dotted
    /// with the analytic parallel's source tangent at a published contact.
    pub(in crate::bezier_offset) fn parallel_contact_tangent_cross_dot_source_sign(
        &self,
        other: &BezierParallel2,
        contact: &BezierAlgebraicCuspSemicircleParallelContact2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if let Some((_, distance)) =
            self.retained_parallel_normal_contact(other, &contact.parallel_parameter, policy)?
            && let Some(sign) = real_sign(&(-self.turn_sign() * distance * dot_scale), policy)
        {
            // turn*J(distance*N) is -turn*distance times the unit source
            // tangent. Its cross is zero and its dot has this scalar sign.
            return Ok(Classification::Decided(sign));
        }
        if self.uses_selected_radial_frame() || self.uses_selected_chord_normal_frame() {
            let map = match self.parallel_parameter_map(other, policy)? {
                Classification::Decided(map) => map,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            return map.tangent_cross_dot_source_sign(contact, cross_scale, dot_scale, policy);
        }
        let system = self.parallel_system(other)?;
        let incidence =
            match reduce_algebraic_cusp_bivariate(system.incidence, self.cusp_parameter(), policy)?
            {
                Classification::Decided(incidence) => incidence,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let mut tangent_dot = match reduce_algebraic_cusp_radical_expression(
            system.tangent_dot_source,
            self.cusp_parameter(),
            policy,
        )? {
            Classification::Decided(tangent_dot) => tangent_dot,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let tangent_cross = match reduce_algebraic_cusp_bivariate(
            system.tangent_cross_source,
            self.cusp_parameter(),
            policy,
        )? {
            Classification::Decided(tangent_cross) => tangent_cross,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        tangent_dot.rational = bivariate_add(
            &bivariate_scale(tangent_cross, cross_scale),
            &bivariate_scale(tangent_dot.rational, dot_scale),
        );
        tangent_dot.radical = bivariate_scale(tangent_dot.radical, dot_scale);
        let speed_squared = match reduce_algebraic_cusp_bivariate(
            system.speed_squared,
            self.cusp_parameter(),
            policy,
        )? {
            Classification::Decided(speed_squared) => speed_squared,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let cusp_parameter = BezierParameter2::Algebraic(self.cusp_parameter().clone());
        let sign = if matches!(
            contact.correlation,
            BezierAlgebraicCuspSemicircleParallelCorrelation2::Map
        ) {
            algebraic_cusp_correlated_radical_sum_sign(
                &incidence,
                &tangent_dot,
                &speed_squared,
                &cusp_parameter,
                &contact.parallel_parameter,
                policy,
            )?
        } else {
            algebraic_cusp_independent_radical_sum_sign(
                &tangent_dot,
                &speed_squared,
                &cusp_parameter,
                &contact.parallel_parameter,
                policy,
            )?
        };
        Ok(sign)
    }

    /// Returns the exact selected-circle tangent dot analytic-parallel tangent
    /// sign for one contact published by [`Self::parallel_intersections`].
    pub(crate) fn parallel_contact_tangent_dot_sign(
        &self,
        other: &BezierParallel2,
        contact: &BezierAlgebraicCuspSemicircleParallelContact2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let sign = self.parallel_contact_tangent_cross_dot_source_sign(
            other,
            contact,
            &Real::zero(),
            &Real::one(),
            policy,
        )?;
        Ok(
            match other.apply_parallel_derivative_scale_to_tangent_sign(
                sign,
                &contact.parallel_parameter,
                policy,
            )? {
                Some(sign) => Classification::Decided(sign),
                None => Classification::Uncertain(UncertaintyReason::Predicate),
            },
        )
    }

    /// Replays the exact tangent topology at a selected-normal diameter
    /// endpoint retained in the same source chart.
    ///
    /// At the start/end of the rational half-circle, both the circle tangent
    /// orientation and the side containing its center are signed products of
    /// the retained radius, circle turn, and parallel derivative scale. No
    /// Cartesian center or endpoint field is required.
    pub(crate) fn parallel_contact_endpoint_tangent_topology(
        &self,
        other: &BezierParallel2,
        contact: &BezierAlgebraicCuspSemicircleParallelContact2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(RealSign, crate::classify::LineSide)>>> {
        let Some(frame) = self.data.frame.parallel_normal() else {
            return Ok(Classification::Decided(None));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a selected-normal endpoint tangent crossed predicate policies".into(),
            ));
        }
        let shares_tangent_parameter = other.source() == frame.center_support.source()
            && frame
                .center_parameter
                .same_value(&contact.parallel_parameter.clone().into(), policy)?
                == Classification::Decided(true);
        if !shares_tangent_parameter {
            return Ok(Classification::Decided(None));
        }
        let endpoint_factor = match contact.location {
            BezierAlgebraicCuspSemicircleContactLocation2::Start => RealSign::Negative,
            BezierAlgebraicCuspSemicircleContactLocation2::End => RealSign::Positive,
            BezierAlgebraicCuspSemicircleContactLocation2::Interior => {
                return Ok(Classification::Decided(None));
            }
        };
        let radial_sign = match real_sign(self.radial_distance(), policy) {
            Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a selected-normal endpoint tangent retained a zero radius".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let derivative_scale = match other
            .parallel_derivative_scale_sign(&contact.parallel_parameter.clone().into(), policy)?
        {
            Classification::Decided(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center_side_sign =
            product_sign(endpoint_factor, product_sign(radial_sign, derivative_scale));
        let center_side = match center_side_sign {
            RealSign::Positive => crate::classify::LineSide::Left,
            RealSign::Negative => crate::classify::LineSide::Right,
            RealSign::Zero => unreachable!("nonzero endpoint factors have a nonzero product"),
        };
        let turn_sign = if self.is_clockwise() {
            RealSign::Negative
        } else {
            RealSign::Positive
        };
        let tangent_dot_sign = product_sign(center_side_sign, turn_sign);
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "selected-circle-parallel-tangent-topology",
            "retained-affine-endpoint",
        );
        Ok(Classification::Decided(Some((
            tangent_dot_sign,
            center_side,
        ))))
    }

    pub(in crate::bezier_offset) fn selected_parallel_normal_parallel_intersections(
        &self,
        other: &BezierParallel2,
        range: &CurveParameterRange2,
        incident: Option<&BezierParallelIncidentDomain2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParallelIntersections2>> {
        let frame_parameter = self.selected_frame_parameter().ok_or_else(|| {
            CurveError::Topology(
                "the parallel-normal kernel received a frame without one center parameter".into(),
            )
        })?;
        let frame_parameter = match promote_curve_region_bezier_parameter(&frame_parameter, policy)?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        let center_parameter = match frame_parameter {
            BezierParameter2::Algebraic(parameter) => parameter,
            BezierParameter2::Exact(parameter) => {
                let polynomial = match BezierParameterPolynomial::try_new_power_basis(
                    vec![-&parameter, Real::one()],
                    &CurveContext::STRICT,
                )? {
                    Classification::Decided(polynomial) => polynomial,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let interval = match BezierParameterInterval::try_new(
                    &parameter - Real::one(),
                    &parameter + Real::one(),
                    &CurveContext::STRICT,
                )? {
                    Classification::Decided(interval) => interval,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match BezierAlgebraicParameter2::try_isolate(
                    polynomial,
                    interval,
                    &CurveContext::STRICT,
                )? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        };
        let BezierSelectedParallelNormalCircleParallelSystem2 {
            incidence,
            squared_branch,
            circle,
            selected_half_plane,
            diameter,
            radius_squared_denominator,
            tangent_cross_source,
            tangent_dot_source,
            center_speed_squared,
            candidate_speed_squared,
        } = match self.selected_parallel_normal_parallel_system(other, range, policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        // A selected round join centered on one analytic parallel and ending
        // on another parallel of the same source owns the diagonal contact
        // `candidate_parameter == center_parameter` by construction. Remove
        // that known root before asking the general selected-fiber isolator to
        // search the finite target range. This is the selected-normal analogue
        // of the rational-frame diagonal deflation below; without it a high-
        // multiplicity authored tangency can dominate unary regularization.
        let frame = self
            .data
            .frame
            .parallel_normal()
            .expect("the selected-normal kernel owns its frame");
        let diagonal_location = if other.source() == frame.center_support.source() {
            let start_distance = frame.center_support.distance() + self.radial_distance();
            let end_distance = frame.center_support.distance() - self.radial_distance();
            if compare_reals(other.distance(), &start_distance, policy)
                == Some(std::cmp::Ordering::Equal)
            {
                Some(BezierAlgebraicCuspSemicircleContactLocation2::Start)
            } else if compare_reals(other.distance(), &end_distance, policy)
                == Some(std::cmp::Ordering::Equal)
            {
                Some(BezierAlgebraicCuspSemicircleContactLocation2::End)
            } else {
                None
            }
        } else {
            None
        };
        if let Some(location) = diagonal_location {
            // Equal-source parallels at the authored radial separation meet
            // the circle on the complete parameter diagonal, not merely at
            // this selected center fiber. Remove that global factor first:
            // exact multivariate division works over arbitrary `Real`
            // coefficients and therefore avoids constructing Q(alpha) when
            // the center polynomial itself has non-rational coefficients.
            let exact_diagonal = deflate_bivariate_parameter_diagonal_exact(&incidence);
            let residual = if let Some(residual) = exact_diagonal {
                residual
            } else {
                let report = deflate_bivariate_fiber_diagonal_root_at_algebraic_parameter(
                    &incidence,
                    CurveResultantParameter::First,
                    &parameter_representation(&center_parameter, policy),
                    policy.predicate_policy(),
                );
                if report.certainty == PredicateCertainty::Approximate {
                    policy.observe_approximate_512();
                }
                match report.status {
                    AlgebraicFiberDiagonalDeflationStatus::Deflated => {
                        report.reduced_polynomial.ok_or_else(|| {
                            CurveError::Topology(
                                "deflated selected-normal diagonal did not retain its residual"
                                    .into(),
                            )
                        })?
                    }
                    AlgebraicFiberDiagonalDeflationStatus::NotARoot => {
                        return Err(CurveError::Topology(
                            "source-related selected-normal endpoint was not on its squared circle"
                                .into(),
                        ));
                    }
                    AlgebraicFiberDiagonalDeflationStatus::IdenticallyZeroFiber => {
                        incidence.clone()
                    }
                    AlgebraicFiberDiagonalDeflationStatus::InvalidEvidence => {
                        return Err(CurveError::InvalidBezierAlgebraicParameter);
                    }
                    AlgebraicFiberDiagonalDeflationStatus::UnsupportedCoefficient => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                    AlgebraicFiberDiagonalDeflationStatus::Undecided => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                    }
                }
            };
            let residual = match reduce_bivariate_in_selected_parameter(
                residual,
                &BezierParameter2::Algebraic(center_parameter.clone()),
                policy,
            )? {
                Classification::Decided(residual) => residual,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let search_range = range;
            let residual_sign = if incident.is_none() {
                bivariate_fiber_strict_sign_on_parameter_range(
                    &residual,
                    &center_parameter,
                    search_range,
                    policy,
                )?
            } else {
                None
            };
            if residual_sign.is_some() {
                let parameter = BezierParameter2::Algebraic(center_parameter.clone());
                let inside = match CurveParameterDomain2::new(search_range, None)
                    .contains_finite_parameter(&parameter.clone().into(), policy)?
                {
                    Classification::Decided(inside) => inside,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let contacts = inside
                    .then(|| BezierAlgebraicCuspSemicircleParallelContact2 {
                        parallel_parameter: parameter,
                        tangent_cross_sign: Some(RealSign::Zero),
                        location,
                        correlation: BezierAlgebraicCuspSemicircleParallelCorrelation2::Map,
                    })
                    .into_iter()
                    .collect();
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-parallel-kernel",
                    "selected-normal-diagonal-rootless",
                );
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped {
                        contacts,
                        overlaps: Vec::new(),
                    },
                ));
            }
        }
        let center = BezierParameter2::Algebraic(center_parameter.clone());
        let incidence = match reduce_bivariate_in_selected_parameter(incidence, &center, policy)? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let circle =
            match reduce_two_normal_expression_in_selected_parameter(circle, &center, policy)? {
                Classification::Decided(expression) => expression,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let squared_branch =
            match reduce_radical_expression_in_selected_parameter(squared_branch, &center, policy)?
            {
                Classification::Decided(expression) => expression,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let selected_half_plane = match reduce_radical_expression_in_selected_parameter(
            selected_half_plane,
            &center,
            policy,
        )? {
            Classification::Decided(expression) => expression,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let diameter =
            match reduce_two_normal_expression_in_selected_parameter(diameter, &center, policy)? {
                Classification::Decided(expression) => expression,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let radius_squared_denominator = match reduce_bivariate_in_selected_parameter(
            radius_squared_denominator,
            &center,
            policy,
        )? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let tangent_cross_source = match reduce_radical_expression_in_selected_parameter(
            tangent_cross_source,
            &center,
            policy,
        )? {
            Classification::Decided(expression) => expression,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let tangent_dot_source = match reduce_two_normal_expression_in_selected_parameter(
            tangent_dot_source,
            &center,
            policy,
        )? {
            Classification::Decided(expression) => expression,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center_speed_squared =
            match reduce_bivariate_in_selected_parameter(center_speed_squared, &center, policy)? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let candidate_speed_squared =
            match reduce_bivariate_in_selected_parameter(candidate_speed_squared, &center, policy)?
            {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let (mut projection_identically_zero, mut candidates) =
            match selected_fiber_parameters_in_range(&incidence, &center_parameter, range, policy)?
            {
                Classification::Decided(Some(parameters)) => (false, parameters),
                Classification::Decided(None) => (true, Vec::new()),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        if !projection_identically_zero && let Some(incident) = incident {
            match selected_fiber_parameters_on_incident_ray(
                &incidence,
                &center_parameter,
                &incident.anchor,
                incident.direction,
                incident.barrier.as_ref(),
                policy,
            )? {
                Classification::Decided(Some(exterior)) => {
                    for parameter in exterior {
                        match CurveParameterDomain2::new(range, None).contains_finite_parameter(
                            &CurveParameter2::from_selected_fiber(parameter.clone()),
                            policy,
                        )? {
                            Classification::Decided(true) => {}
                            Classification::Decided(false) => candidates.push(parameter),
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                }
                Classification::Decided(None) => {
                    projection_identically_zero = true;
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if projection_identically_zero {
            candidates = match selected_parallel_normal_positive_dimensional_projection(
                &circle,
                &squared_branch,
                &center_speed_squared,
                &candidate_speed_squared,
                &center_parameter,
                range,
                incident,
                policy,
            )? {
                Classification::Decided(
                    BezierSelectedParallelNormalPositiveProjection2::Candidates(candidates),
                ) => candidates,
                Classification::Decided(
                    BezierSelectedParallelNormalPositiveProjection2::AuthoredPolynomialCandidates {
                        parameters,
                        ..
                    },
                ) => parameters,
                Classification::Decided(
                    BezierSelectedParallelNormalPositiveProjection2::CoincidentCircleComponent,
                ) => {
                    return Ok(Classification::Decided(
                        BezierAlgebraicCuspSemicircleParallelIntersections2::CoincidentCircleComponent,
                    ));
                }
                Classification::Decided(
                    BezierSelectedParallelNormalPositiveProjection2::Degenerate,
                ) => {
                    return Ok(Classification::Decided(
                        BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection,
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        }
        let mut retained = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            let circle_sign = candidate.two_normal_sum_sign(
                &circle,
                &center_speed_squared,
                &candidate_speed_squared,
                policy,
            )?;
            match circle_sign {
                Classification::Decided(RealSign::Zero) => {}
                Classification::Decided(RealSign::Positive | RealSign::Negative) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let selected = match candidate.radical_sum_sign(
                &selected_half_plane,
                &candidate_speed_squared,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let location = match selected {
                RealSign::Negative => continue,
                RealSign::Positive => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                RealSign::Zero => match candidate.two_normal_sum_sign(
                    &diameter,
                    &center_speed_squared,
                    &candidate_speed_squared,
                    policy,
                )? {
                    Classification::Decided(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(RealSign::Negative) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::End
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a nonzero selected circle had an indeterminate two-normal endpoint"
                                .into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            };
            let tangent_cross_source = match candidate.radical_sum_sign(
                &tangent_cross_source,
                &center_speed_squared,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let derivative_scale = match other.parallel_derivative_scale_sign(
                &CurveParameter2::from_selected_fiber(candidate.clone()),
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let tangent_cross_sign = product_sign(tangent_cross_source, derivative_scale);
            retained.push((candidate, location, tangent_cross_sign));
        }
        let map = BezierAlgebraicCuspSemicircleSelectedFiberParallelParameterMap2 {
            data: Arc::new(
                BezierAlgebraicCuspSemicircleSelectedFiberParallelParameterMapData2 {
                    semicircle: self.clone(),
                    parallel: other.clone(),
                    diameter,
                    radius_squared_denominator,
                    tangent_cross_source,
                    tangent_dot_source,
                    center_speed_squared,
                    candidate_speed_squared,
                    policy: policy.retained_object_policy(),
                },
            ),
        };
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiber {
                contacts: retained
                    .into_iter()
                    .map(|(parameter, location, tangent_cross_sign)| {
                        map.contact(parameter, location, tangent_cross_sign)
                    })
                    .collect(),
                overlaps: Vec::new(),
            },
        ))
    }

    pub(in crate::bezier_offset) fn parallel_contact_at_certified_parameter(
        &self,
        other: &BezierParallel2,
        candidate: BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleParallelContact2>>> {
        let system = self.parallel_system(other)?;
        let reduce_expression = |expression| {
            reduce_algebraic_cusp_radical_expression(expression, self.cusp_parameter(), policy)
        };
        let circle = match reduce_expression(system.circle)? {
            Classification::Decided(circle) => circle,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let selected_half_plane = match reduce_expression(system.selected_half_plane)? {
            Classification::Decided(selected_half_plane) => selected_half_plane,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let diameter = match reduce_expression(system.diameter_side)? {
            Classification::Decided(diameter) => diameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let speed_squared = match reduce_algebraic_cusp_bivariate(
            system.speed_squared,
            self.cusp_parameter(),
            policy,
        )? {
            Classification::Decided(speed_squared) => speed_squared,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let tangent_cross_source = match reduce_algebraic_cusp_bivariate(
            system.tangent_cross_source,
            self.cusp_parameter(),
            policy,
        )? {
            Classification::Decided(tangent_cross_source) => tangent_cross_source,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let cusp_parameter = BezierParameter2::Algebraic(self.cusp_parameter().clone());
        let radical_sign = |expression: &BezierAlgebraicCuspTwoTermExpression2| {
            algebraic_cusp_independent_radical_sum_sign(
                expression,
                &speed_squared,
                &cusp_parameter,
                &candidate,
                policy,
            )
        };
        match radical_sign(&circle)? {
            Classification::Decided(RealSign::Zero) => {}
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let selected = match radical_sign(&selected_half_plane)? {
            Classification::Decided(selected) => selected,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let location = match selected {
            RealSign::Negative => return Ok(Classification::Decided(None)),
            RealSign::Positive => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
            RealSign::Zero => match radical_sign(&diameter)? {
                Classification::Decided(RealSign::Positive) => {
                    BezierAlgebraicCuspSemicircleContactLocation2::Start
                }
                Classification::Decided(RealSign::Negative) => {
                    BezierAlgebraicCuspSemicircleContactLocation2::End
                }
                Classification::Decided(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "a certified full-circle contact had an indeterminate diameter endpoint"
                            .into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        };
        let source_cross = match signed_bivariate_at_parameter_pair(
            &tangent_cross_source,
            &cusp_parameter,
            &candidate,
            policy,
        )? {
            Classification::Decided(source_cross) => source_cross,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let tangent_cross_sign = if source_cross == RealSign::Zero {
            Some(RealSign::Zero)
        } else {
            other.apply_parallel_derivative_scale_to_tangent_sign(
                Classification::Decided(source_cross),
                &candidate,
                policy,
            )?
        };
        Ok(Classification::Decided(Some(
            BezierAlgebraicCuspSemicircleParallelContact2 {
                parallel_parameter: candidate,
                tangent_cross_sign,
                location,
                correlation: BezierAlgebraicCuspSemicircleParallelCorrelation2::Independent,
            },
        )))
    }

    /// Enumerates analytic-parallel candidates from only a represented center
    /// when the complete recursive tower would make the target resultant much
    /// larger. This is a schedule, not topology evidence: the retained
    /// recursive system replays incidence, half selection, and tangency for
    /// every returned parameter.
    pub(in crate::bezier_offset) fn represented_center_parallel_candidates(
        &self,
        system: &BezierRecursiveCircleTargetSystem2,
        other: &BezierParallel2,
        range: &CurveParameterRange2,
        incident: Option<&BezierParallelIncidentDomain2>,
        policy: &CurveContext,
    ) -> CurveResult<Option<Vec<BezierParameter2>>> {
        let domain = CurveParameterDomain2::new(
            range,
            incident.map(BezierParallelIncidentDomain2::parameter_ray),
        );
        let schedule = if let Some(schedule) = system.represented_center_schedule.get() {
            schedule
        } else {
            let center = match self.center_point_evidence(policy)? {
                Classification::Decided(center) => center,
                Classification::Uncertain(_) => return Ok(None),
            };
            let center = match represented_point_evidence_coordinates(&center, policy)? {
                Classification::Decided(center) => center.map(|coordinate| {
                    hypersolve::compact_algebraic_root_low_degree_witness(&coordinate)
                        .unwrap_or(coordinate)
                }),
                Classification::Uncertain(_) => match self.represented_circle_frame(policy)? {
                    Classification::Decided(frame) => frame.center,
                    Classification::Uncertain(_) => return Ok(None),
                },
            };
            let common = match self.represented_center_parallel_system(other, &center)? {
                Classification::Decided(common) => common,
                Classification::Uncertain(_) => return Ok(None),
            };
            let univariate = match selected_dense_last_axis_univariate(
                &common.projection,
                &common.sources,
                policy,
            )? {
                Classification::Decided(univariate) => univariate,
                Classification::Uncertain(_) => return Ok(None),
            };
            let _ =
                system
                    .represented_center_schedule
                    .set(BezierRepresentedCenterParallelSchedule2 {
                        univariate,
                        unit_interval: OnceLock::new(),
                    });
            system
                .represented_center_schedule
                .get()
                .expect("a represented-center schedule was just retained")
        };
        let projected = selected_axis_parameters_in_domain(domain, policy, |domain| {
            let unit_interval = matches!(domain, SelectedThirdAxisDomain2::Finite(range) if range == &CurveParameterRange2::unit());
            if unit_interval && let Some(projected) = schedule.unit_interval.get() {
                return Ok(Classification::Decided(projected.clone()));
            }
            let projected =
                isolate_selected_dense_last_axis_univariate(&schedule.univariate, domain, policy)?;
            if unit_interval && let Classification::Decided(projected) = &projected {
                let _ = schedule.unit_interval.set(projected.clone());
            }
            Ok(projected)
        })?;
        Ok(match projected {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
                Some(candidates)
            }
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            )
            | Classification::Uncertain(_) => None,
        })
    }

    /// For C=P(a)+d0*N(a), the point P(a)+d1*N(a) lies on this
    /// circle precisely when (d1-d0)^2=r^2. Reuse that source identity before
    /// adjoining a second copy of a and replaying a zero in their tensor field.
    /// The source speed must be positive; a custom one-sided frame is a
    /// different premise and remains with the general contact authority.
    pub(in crate::bezier_offset) fn retained_parallel_normal_contact(
        &self,
        other: &BezierParallel2,
        parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Option<(CurveParameter2, Real)>> {
        let Some(frame) = self.data.frame.chord_normal() else {
            return Ok(None);
        };
        if !CurveContext::STRICT.accepts_retained_policy(frame.policy) {
            return Ok(None);
        }
        let CurvePoint2(CurvePointData2::AnalyticParallel(point)) = &frame.center else {
            return Ok(None);
        };
        let Some((center_parallel, center_parameter)) = point.native_parallel_evaluation() else {
            return Ok(None);
        };
        if center_parallel.source() != other.source() {
            return Ok(None);
        }
        policy.bounded_exact_predicate_pass(|| {
            let strict = policy.strict_counterpart();
            let distance = other.distance() - center_parallel.distance();
            if real_sign(
                &(&distance * &distance - self.radial_distance() * self.radial_distance()),
                &strict,
            ) != Some(RealSign::Zero)
                || center_parameter.same_value(&parameter.clone().into(), &strict)?
                    != Classification::Decided(true)
            {
                return Ok(None);
            }
            let speed_squared = parallel_speed_squared_polynomial(other.differential()?);
            if signed_coefficients_at_parameter(&speed_squared, parameter, &strict)?
                != Classification::Decided(RealSign::Positive)
            {
                return Ok(None);
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-parallel-kernel",
                "retained-source-normal",
            );
            Ok(Some((center_parameter, distance)))
        })
    }

    /// At a certified source-normal contact, the circle's half-plane and
    /// diameter predicates are tangent cross/dot signs. Their positive speed
    /// denominators cancel, so no Cartesian contact coordinates are needed.
    pub(in crate::bezier_offset) fn retained_parallel_normal_contact_location(
        &self,
        other: &BezierParallel2,
        parameter: &CurveParameter2,
        distance: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleContactLocation2>>> {
        let frame = self
            .data
            .frame
            .chord_normal()
            .expect("a retained normal contact owns its chord frame");
        let radial_product = self.radial_distance() * distance;
        let half = frame
            .anchor
            .tangent_cross_dot_parallel_source_linear_combination_sign(
                other,
                parameter,
                &(&self.turn_sign() * &radial_product),
                &Real::zero(),
                policy,
            )?;
        Ok(match half {
            Classification::Decided(RealSign::Negative) => Classification::Decided(None),
            Classification::Decided(RealSign::Positive) => Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleContactLocation2::Interior,
            )),
            Classification::Decided(RealSign::Zero) => {
                match frame
                    .anchor
                    .tangent_cross_dot_parallel_source_linear_combination_sign(
                        other,
                        parameter,
                        &Real::zero(),
                        &radial_product,
                        policy,
                    )? {
                    Classification::Decided(RealSign::Positive) => Classification::Decided(Some(
                        BezierAlgebraicCuspSemicircleContactLocation2::Start,
                    )),
                    Classification::Decided(RealSign::Negative) => Classification::Decided(Some(
                        BezierAlgebraicCuspSemicircleContactLocation2::End,
                    )),
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a regular source-normal contact lost its radial direction".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                }
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        })
    }

    pub(in crate::bezier_offset) fn recursive_circle_parallel_intersections(
        &self,
        other: &BezierParallel2,
        range: &CurveParameterRange2,
        incident: Option<&BezierParallelIncidentDomain2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParallelIntersections2>> {
        let domain = CurveParameterDomain2::new(
            range,
            incident.map(BezierParallelIncidentDomain2::parameter_ray),
        );
        let system = match self.recursive_circle_parallel_system(other, policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        // A recursively retained frame can still prove that its center is an
        // exact point (for example, after two unrelated circle contacts
        // cancel to a rational center). In that case the ordinary analytic
        // parallel/circle eliminant is the minimal exact authority: it has no
        // recursive source axes and replays the authored normal sheet before
        // returning candidates. Keep the recursive system only for angular
        // and tangent topology at those certified parameters.
        let exact_center_candidates =
            (|| -> CurveResult<Option<Vec<(BezierParameter2, Option<RealSign>)>>> {
                let Some(center) = self.exact_center(policy)? else {
                    return Ok(None);
                };
                let radius_squared = self.radial_distance() * self.radial_distance();
                let mut candidates =
                    match other.circle_incidence(&center, &radius_squared, range, &[], policy)? {
                        Classification::Decided(candidates) => candidates,
                        Classification::Uncertain(_) => return Ok(None),
                    };
                if let Some(incident) = incident {
                    let exterior = match other.circle_incidence_on_incident_ray(
                        &center,
                        &radius_squared,
                        incident,
                        policy,
                    )? {
                        Classification::Decided(candidates) => candidates,
                        Classification::Uncertain(_) => return Ok(None),
                    };
                    for (parameter, crossing) in exterior {
                        match domain.contains_finite_parameter(&parameter.clone().into(), policy)? {
                            Classification::Decided(true) => {}
                            Classification::Decided(false) => {
                                candidates.push((parameter, crossing))
                            }
                            Classification::Uncertain(_) => return Ok(None),
                        }
                    }
                }
                Ok(Some(candidates))
            })()?;
        // A chord-normal center is already retained in the imported field.
        // The selected-radial projection schedule can instead cancel a deeper
        // circle-pair dependency, so retain that existing optional schedule.
        let represented_center_candidates = if exact_center_candidates.is_none()
            && self.uses_selected_radial_frame()
        {
            self.represented_center_parallel_candidates(&system, other, range, incident, policy)?
        } else {
            None
        };
        let (candidates, transverse_projection, direct_pair_candidates, incidence_certified) =
            if let Some(candidates) = exact_center_candidates {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-parallel-kernel",
                    "recursive-exact-center",
                );
                (candidates, false, false, true)
            } else if let Some(candidates) = represented_center_candidates {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-parallel-kernel",
                    "recursive-represented-center-schedule",
                );
                (
                    candidates
                        .into_iter()
                        .map(|parameter| (parameter, None))
                        .collect(),
                    true,
                    false,
                    false,
                )
            } else {
                let (parameters, transverse, direct, certified) = match system
                    .incidence_parameters_with_incident_domain(domain, policy)?
                {
                    Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                        parameters,
                    )) => (
                        parameters,
                        true,
                        system.direct_pair_fast_path.is_some(),
                        false,
                    ),
                    Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
                        let sample = strict_sample_for_parallel_domain(range, incident, policy)?;
                        let sample = match sample {
                            Classification::Decided(sample) => BezierParameter2::Exact(sample),
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        let evaluation = match system.candidate_evaluation(&sample, policy)? {
                            Classification::Decided(Some(evaluation)) => evaluation,
                            Classification::Decided(None) => {
                                return Ok(Classification::Decided(
                                    BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection,
                                ));
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        match system.expression_sign_with_evaluation(
                            &system.circle,
                            &evaluation,
                            policy,
                        )? {
                            Classification::Decided(RealSign::Zero) => {
                                #[cfg(feature = "dispatch-trace")]
                                hyperreal::dispatch_trace::record(
                                    "hypercurve",
                                    "recursive-circle-parallel-degenerate",
                                    "coincident-circle",
                                );
                                return Ok(Classification::Decided(
                                    BezierAlgebraicCuspSemicircleParallelIntersections2::CoincidentCircleComponent,
                                ));
                            }
                            Classification::Decided(RealSign::Negative | RealSign::Positive) => {}
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }

                        // An identically zero norm may belong to the opposite target
                        // speed sheet. Geometric roots on the authored sheet then
                        // occur only where both unsquared terms vanish. Enumerate one
                        // residual term and replay the complete expression below.
                        let Some(radical_projection) =
                            system.projected_polynomial(&system.circle.radical)
                        else {
                            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                        };
                        match system.parameters_with_incident_domain(
                            &radical_projection,
                            domain,
                            policy,
                        )? {
                            Classification::Decided(
                                BezierAlgebraicFiberProjection2::Parameters(parameters),
                            ) => (parameters, false, false, false),
                            Classification::Decided(
                                BezierAlgebraicFiberProjection2::IdenticallyZero,
                            ) => {
                                let Some(rational_projection) =
                                    system.projected_polynomial(&system.circle.rational)
                                else {
                                    return Ok(Classification::Uncertain(
                                        UncertaintyReason::Unsupported,
                                    ));
                                };
                                match system.parameters_with_incident_domain(
                                    &rational_projection,
                                    domain,
                                    policy,
                                )? {
                                    Classification::Decided(
                                        BezierAlgebraicFiberProjection2::Parameters(parameters),
                                    ) => (parameters, false, false, false),
                                    Classification::Decided(
                                        BezierAlgebraicFiberProjection2::IdenticallyZero
                                        | BezierAlgebraicFiberProjection2::Degenerate,
                                    ) => {
                                        return Ok(Classification::Decided(
                                            BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection,
                                        ));
                                    }
                                    Classification::Uncertain(reason) => {
                                        return Ok(Classification::Uncertain(reason));
                                    }
                                }
                            }
                            Classification::Decided(
                                BezierAlgebraicFiberProjection2::Degenerate,
                            ) => {
                                return Ok(Classification::Decided(
                                    BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection,
                                ));
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
                        return Ok(Classification::Decided(
                            BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection,
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                (
                    parameters
                        .into_iter()
                        .map(|parameter| (parameter, None))
                        .collect(),
                    transverse,
                    direct,
                    certified,
                )
            };
        let mut contacts = Vec::with_capacity(candidates.len());
        for (mut candidate, radial_crossing_sign) in candidates {
            if direct_pair_candidates {
                let fast = system
                    .direct_pair_fast_path
                    .as_ref()
                    .expect("direct pair candidates retain their fast backend");
                match fast.target_is_regular(&candidate, policy)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                let location = match fast.contact_location(&candidate, policy)? {
                    Classification::Decided(Some(location)) => location,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let source_cross = match fast.pair_expression_sign(
                    &fast.tangent_cross_source,
                    &candidate,
                    policy,
                )? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let tangent_cross_sign = if source_cross == RealSign::Zero {
                    Some(RealSign::Zero)
                } else {
                    other.apply_parallel_derivative_scale_to_tangent_sign(
                        Classification::Decided(source_cross),
                        &candidate,
                        policy,
                    )?
                };
                contacts.push(BezierAlgebraicCuspSemicircleParallelContact2 {
                    parallel_parameter: candidate,
                    tangent_cross_sign,
                    location,
                    correlation: BezierAlgebraicCuspSemicircleParallelCorrelation2::Map,
                });
                continue;
            }
            let normal_contact =
                self.retained_parallel_normal_contact(other, &candidate, policy)?;
            if let Some((parameter, _)) = &normal_contact
                && let Some(native) = parameter.as_bezier_parameter()
            {
                candidate = native.clone();
            }
            let incidence_certified = incidence_certified || normal_contact.is_some();
            let interval_incidence = if incidence_certified {
                None
            } else {
                transverse_projection
                    .then(|| system.expression_root_by_interval(&system.circle, &candidate))
                    .flatten()
            };
            if !incidence_certified && interval_incidence == Some(false) {
                continue;
            }
            let normal_location = if let Some((parameter, distance)) = &normal_contact {
                match self
                    .retained_parallel_normal_contact_location(other, parameter, distance, policy)?
                {
                    Classification::Decided(location) => Some(location),
                    Classification::Uncertain(_) => None,
                }
            } else {
                None
            };
            let interval_location =
                normal_location.or_else(|| system.contact_location_by_interval(&candidate));
            if interval_location == Some(None) {
                continue;
            }
            // d|Q-C|²/dt has the sign of (Q-C) dot Q'. A circle
            // tangent is turn*J(Q-C), so its cross with Q' has the opposite
            // turn times that already-certified sign. It includes the
            // target's derivative scale and must not apply that scale twice.
            let certified_cross = radial_crossing_sign.map(|sign| {
                product_sign(
                    sign,
                    if self.data.clockwise {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    },
                )
            });
            let interval_cross = if certified_cross.is_some() {
                None
            } else if normal_contact.is_some() {
                Some(RealSign::Zero)
            } else {
                system.polynomial_interval_sign(&system.tangent_cross_source, &candidate)
            };
            let needs_incidence_replay = !incidence_certified && interval_incidence != Some(true);
            let evaluation = if needs_incidence_replay
                || interval_location.is_none()
                || (certified_cross.is_none() && interval_cross.is_none())
            {
                match system.candidate_evaluation(&candidate, policy)? {
                    Classification::Decided(Some(evaluation)) => Some(evaluation),
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            } else {
                None
            };
            if needs_incidence_replay {
                match system.expression_sign_with_evaluation(
                    &system.circle,
                    evaluation
                        .as_ref()
                        .expect("an uncertified recursive incidence retains its evaluation"),
                    policy,
                )? {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(RealSign::Negative | RealSign::Positive) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            let location = if let Some(Some(location)) = interval_location {
                location
            } else {
                match system.contact_location_with_evaluation(
                    evaluation
                        .as_ref()
                        .expect("an uncertified recursive location retains its evaluation"),
                    policy,
                )? {
                    Classification::Decided(Some(location)) => location,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            let tangent_cross_sign = if let Some(sign) = certified_cross {
                Some(sign)
            } else {
                let source_cross = if let Some(sign) = interval_cross {
                    sign
                } else {
                    match system.polynomial_sign_with_evaluation(
                        &system.tangent_cross_source,
                        evaluation
                            .as_ref()
                            .expect("an uncertified recursive tangent retains its evaluation"),
                        policy,
                    )? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                };
                if source_cross == RealSign::Zero {
                    Some(RealSign::Zero)
                } else {
                    other.apply_parallel_derivative_scale_to_tangent_sign(
                        Classification::Decided(source_cross),
                        &candidate,
                        policy,
                    )?
                }
            };
            contacts.push(BezierAlgebraicCuspSemicircleParallelContact2 {
                parallel_parameter: candidate,
                tangent_cross_sign,
                location,
                correlation: BezierAlgebraicCuspSemicircleParallelCorrelation2::Map,
            });
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-circle-parallel-kernel",
            "recursive-quadratic",
        );
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped {
                contacts,
                overlaps: Vec::new(),
            },
        ))
    }

    pub(in crate::bezier_offset) fn represented_parallel_intersections(
        &self,
        other: &BezierParallel2,
        range: &CurveParameterRange2,
        incident: Option<&BezierParallelIncidentDomain2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParallelIntersections2>> {
        let domain = CurveParameterDomain2::new(
            range,
            incident.map(BezierParallelIncidentDomain2::parameter_ray),
        );
        let system = match self.represented_parallel_system(other, policy)? {
            Classification::Decided(system) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-circle-parallel-system",
                    "constructed",
                );
                system
            }
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-circle-parallel-system",
                    match reason {
                        UncertaintyReason::Unsupported => "unsupported",
                        UncertaintyReason::Predicate => "predicate",
                        UncertaintyReason::Ordering => "ordering",
                        UncertaintyReason::RealSign => "real-sign",
                        UncertaintyReason::Boundary => "boundary",
                    },
                );
                return Ok(Classification::Uncertain(reason));
            }
        };
        let projection =
            match system.parameters_with_incident_domain(&system.projection, domain, policy)? {
                Classification::Decided(projection) => projection,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let mut candidates = match projection {
            BezierAlgebraicFiberProjection2::Parameters(parameters) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-circle-parallel-projection",
                    "finite-parameters",
                );
                parameters
            }
            BezierAlgebraicFiberProjection2::IdenticallyZero => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-circle-parallel-projection",
                    "identically-zero",
                );
                let sample = strict_sample_for_parallel_domain(range, incident, policy)?;
                let sample = match sample {
                    Classification::Decided(sample) => BezierParameter2::Exact(sample),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match system.target_is_regular(&sample, policy)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "represented-circle-parallel-degenerate",
                            "irregular-sample",
                        );
                        return Ok(Classification::Decided(
                            BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection,
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                match system.expression_sign(&system.circle, &sample, policy)? {
                    Classification::Decided(RealSign::Zero) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "represented-circle-parallel-degenerate",
                            "coincident-circle",
                        );
                        return Ok(Classification::Decided(
                            BezierAlgebraicCuspSemicircleParallelIntersections2::CoincidentCircleComponent,
                        ));
                    }
                    Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }

                // The squared norm can vanish identically on the conjugate
                // target-speed sheet. On the authored sheet only common zeros
                // of its rational and radical terms remain geometric.
                let first = match system.parameters_with_incident_domain(
                    &system.circle.radical,
                    domain,
                    policy,
                )? {
                    Classification::Decided(projection) => projection,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match first {
                    BezierAlgebraicFiberProjection2::Parameters(parameters) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "represented-circle-parallel-residual",
                            "radical-parameters",
                        );
                        parameters
                    }
                    BezierAlgebraicFiberProjection2::IdenticallyZero => {
                        match system.parameters_with_incident_domain(
                            &system.circle.rational,
                            domain,
                            policy,
                        )? {
                            Classification::Decided(
                                BezierAlgebraicFiberProjection2::Parameters(parameters),
                            ) => {
                                #[cfg(feature = "dispatch-trace")]
                                hyperreal::dispatch_trace::record(
                                    "hypercurve",
                                    "represented-circle-parallel-residual",
                                    "rational-parameters",
                                );
                                parameters
                            }
                            Classification::Decided(
                                BezierAlgebraicFiberProjection2::IdenticallyZero
                                | BezierAlgebraicFiberProjection2::Degenerate,
                            ) => {
                                #[cfg(feature = "dispatch-trace")]
                                hyperreal::dispatch_trace::record(
                                    "hypercurve",
                                    "represented-circle-parallel-degenerate",
                                    "both-residual-terms-degenerate",
                                );
                                return Ok(Classification::Decided(
                                    BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection,
                                ));
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    BezierAlgebraicFiberProjection2::Degenerate => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "represented-circle-parallel-degenerate",
                            "radical-term-degenerate",
                        );
                        return Ok(Classification::Decided(
                            BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection,
                        ));
                    }
                }
            }
            BezierAlgebraicFiberProjection2::Degenerate => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-circle-parallel-projection",
                    "degenerate",
                );
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection,
                ));
            }
        };
        let mut contacts = Vec::with_capacity(candidates.len());
        for candidate in candidates.drain(..) {
            match system.target_is_regular(&candidate, policy)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            match system.expression_sign(&system.circle, &candidate, policy)? {
                Classification::Decided(RealSign::Zero) => {}
                Classification::Decided(RealSign::Positive | RealSign::Negative) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let location = match system.contact_location(&candidate, policy)? {
                Classification::Decided(Some(location)) => location,
                Classification::Decided(None) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let source_cross = match system.tangent_cross_dot_source_sign(
                &candidate,
                &Real::one(),
                &Real::zero(),
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let tangent_cross_sign = if source_cross == RealSign::Zero {
                Some(RealSign::Zero)
            } else {
                other.apply_parallel_derivative_scale_to_tangent_sign(
                    Classification::Decided(source_cross),
                    &candidate,
                    policy,
                )?
            };
            contacts.push(BezierAlgebraicCuspSemicircleParallelContact2 {
                parallel_parameter: candidate,
                tangent_cross_sign,
                location,
                correlation: BezierAlgebraicCuspSemicircleParallelCorrelation2::Map,
            });
        }
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped {
                contacts,
                overlaps: Vec::new(),
            },
        ))
    }

    pub(in crate::bezier_offset) fn finite_parallel_intersections_from_rational_component(
        &self,
        curve: &RationalBezier2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleParallelIntersections2>>>
    {
        let active = range;
        Ok(match self.rational_intersections(curve, active, policy)? {
            Classification::Decided(
                BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps },
            ) => {
                let mut retained = Vec::with_capacity(contacts.len());
                for contact in contacts {
                    let parallel_parameter = match promote_curve_region_bezier_parameter(
                        &contact.other_parameter,
                        policy,
                    )? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    retained.push(BezierAlgebraicCuspSemicircleParallelContact2 {
                        parallel_parameter,
                        tangent_cross_sign: Some(contact.tangent_cross_sign),
                        location: contact.location,
                        correlation: BezierAlgebraicCuspSemicircleParallelCorrelation2::Map,
                    });
                }
                let overlaps = {
                    let original_contacts = retained.len();
                    let mut clipped = Vec::with_capacity(overlaps.len());
                    for overlap in overlaps {
                        let source =
                            crate::curve_intersection::CurveCircleOverlap2::Mapped(overlap.clone());
                        let (circle_range, _) = source.parameter_ranges();
                        match source.clipped_ranges(&circle_range, active, policy)? {
                            Classification::Decided(Some((circle, other))) => {
                                let [start, end] = match other.ordered_endpoints(policy)? {
                                    Classification::Decided(bounds) => bounds,
                                    Classification::Uncertain(reason) => {
                                        return Ok(Classification::Uncertain(reason));
                                    }
                                };
                                clipped.push(BezierAlgebraicCuspSemicircleMappedOverlap2 {
                                    other_range: CurveParameterRange2::new_validated(
                                        start.clone(),
                                        end.clone(),
                                    ),
                                    cusp_start: circle
                                        .start()
                                        .as_algebraic_cusp()
                                        .ok_or(CurveError::InvalidCurveParameter)?
                                        .clone(),
                                    cusp_end: circle
                                        .end()
                                        .as_algebraic_cusp()
                                        .ok_or(CurveError::InvalidCurveParameter)?
                                        .clone(),
                                    ..overlap
                                });
                            }
                            Classification::Decided(None) => {
                                let pair = match source.singleton_contact(
                                    &circle_range,
                                    active,
                                    policy,
                                )? {
                                    Classification::Decided(pair) => pair,
                                    Classification::Uncertain(reason) => {
                                        return Ok(Classification::Uncertain(reason));
                                    }
                                };
                                if let Some([circle, other]) = pair {
                                    let parameter = circle
                                        .as_algebraic_cusp()
                                        .ok_or(CurveError::InvalidCurveParameter)?
                                        .clone();
                                    let location = match &parameter {
                                        BezierAlgebraicCuspSemicircleParameter2::Exact(value)
                                            if value == &Real::zero() =>
                                        {
                                            BezierAlgebraicCuspSemicircleContactLocation2::Start
                                        }
                                        BezierAlgebraicCuspSemicircleParameter2::Exact(value)
                                            if value == &Real::one() =>
                                        {
                                            BezierAlgebraicCuspSemicircleContactLocation2::End
                                        }
                                        _ => {
                                            BezierAlgebraicCuspSemicircleContactLocation2::Interior
                                        }
                                    };
                                    let parallel_parameter =
                                        match policy.strict_predicate_pass(|| {
                                            other.promoted_bezier_parameter_complete(policy)
                                        })? {
                                            Classification::Decided(parameter) => parameter,
                                            Classification::Uncertain(reason) => {
                                                return Ok(Classification::Uncertain(reason));
                                            }
                                        };
                                    retained.push(BezierAlgebraicCuspSemicircleParallelContact2 {
                                        parallel_parameter,
                                        tangent_cross_sign: Some(RealSign::Zero),
                                        location,
                                        correlation: BezierAlgebraicCuspSemicircleParallelCorrelation2::Retained(parameter),
                                    });
                                }
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    // A neighboring monotone cell can own the same clipped
                    // endpoint. Keep one parameter visit, and let positive
                    // cells own their boundaries as in the component publisher.
                    let mut index = original_contacts;
                    while index < retained.len() {
                        let contact = &retained[index];
                        let mut covered = false;
                        for overlap in &clipped {
                            match CurveParameterDomain2::new(overlap.other_range(), None)
                                .contains_finite_parameter(
                                    &contact.parallel_parameter.clone().into(),
                                    policy,
                                )? {
                                Classification::Decided(true) => {
                                    covered = true;
                                    break;
                                }
                                Classification::Decided(false) => {}
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                        if !covered {
                            for existing in &retained[..index] {
                                match contact
                                    .parallel_parameter
                                    .same_value(&existing.parallel_parameter, policy)?
                                {
                                    Classification::Decided(true) => {
                                        covered = true;
                                        break;
                                    }
                                    Classification::Decided(false) => {}
                                    Classification::Uncertain(reason) => {
                                        return Ok(Classification::Uncertain(reason));
                                    }
                                }
                            }
                        }
                        if covered {
                            retained.swap_remove(index);
                        } else {
                            index += 1;
                        }
                    }
                    clipped
                };
                Classification::Decided(Some(
                    BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped {
                        contacts: retained,
                        overlaps,
                    },
                ))
            }
            Classification::Decided(
                BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber {
                    contacts,
                    overlaps,
                },
            ) => Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiber {
                    contacts,
                    overlaps,
                },
            )),
            Classification::Decided(
                BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
            )
            | Classification::Uncertain(_) => Classification::Decided(None),
        })
    }

    /// Reuses the authoritative circle/chord kernel for an analytic parallel
    /// whose exact rational image is affine in the same source parameter.
    /// The adapter performs no scalar globalization: each line coordinate is
    /// published in the contact point's existing recursive field.
    pub(in crate::bezier_offset) fn exact_linear_parallel_intersections(
        &self,
        curve: &RationalBezier2,
        range: &CurveParameterRange2,
        incident: Option<&BezierParallelIncidentDomain2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleParallelIntersections2>>>
    {
        let Some(line) = curve.exact_linear_parameterization_line() else {
            return Ok(Classification::Decided(None));
        };
        let chord = match BezierAlgebraicChord2::try_new(
            CurvePoint2::from(line.start().clone()),
            CurvePoint2::from(line.end().clone()),
            policy,
        )? {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
        };
        let intersections =
            match self.chord_intersections_prefer_exact_line(&chord, false, policy)? {
                Classification::Decided(intersections) => intersections,
                Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
            };
        let contacts = intersections;
        let mut retained = Vec::with_capacity(contacts.len());
        for contact in contacts {
            let other_parameter =
                match contact.chord_parameter.exact_line_curve_parameter(policy)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
                };
            let in_range = CurveParameterDomain2::new(range, None)
                .contains_finite_parameter(&other_parameter, policy)?;
            let in_incident = match incident {
                Some(incident) => {
                    incident.contains_extension_parameter(&other_parameter, policy)?
                }
                None => Classification::Decided(false),
            };
            match (in_range, in_incident) {
                (Classification::Decided(true), _) | (_, Classification::Decided(true)) => {}
                (Classification::Decided(false), Classification::Decided(false)) => continue,
                (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let (tangent_dot_sign, circle_side_of_parallel) =
                if contact.tangent_cross_sign == RealSign::Zero {
                    match contact.tangent_topology(self, &chord, policy)? {
                        Classification::Decided(Some((dot, side))) => (dot, Some(side)),
                        Classification::Decided(None) => {
                            unreachable!("a zero-cross contact is tangent")
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                } else {
                    match contact.tangent_dot_sign(self, &chord, policy)? {
                        Classification::Decided(dot) => (dot, None),
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                };
            retained.push(BezierAlgebraicCuspSemicircleRetainedParallelContact2 {
                retained: contact,
                other_parameter,
                tangent_dot_sign,
                circle_side_of_parallel,
            });
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-circle-parallel-kernel",
            "exact-linear-chord-authority",
        );
        Ok(Classification::Decided(Some(
            BezierAlgebraicCuspSemicircleParallelIntersections2::RetainedContacts(retained),
        )))
    }

    /// Finds a canonical source scalar that is exactly the retained cusp root.
    ///
    /// A separately authored analytic carrier can encode the same algebraic
    /// translation as a canonical [`Real`] coefficient. Keeping that value as
    /// an unrelated base-field constant makes the selected quotient ring
    /// reducible over its own coefficient field. A STRICT equality proof lets
    /// the common projection specialize the selected axis first, without
    /// flattening any unproved root or changing the published cusp authority.
    pub(in crate::bezier_offset) fn parallel_source_cusp_witness(
        &self,
        other: &BezierParallel2,
    ) -> CurveResult<Option<Real>> {
        let selected = BezierParameter2::Algebraic(self.cusp_parameter().clone());
        let matches_selected = |candidate: Real| -> CurveResult<bool> {
            Ok(
                selected.same_value(&BezierParameter2::Exact(candidate), &CurveContext::STRICT)?
                    == Classification::Decided(true),
            )
        };
        for parameter in [Real::zero(), Real::one()] {
            let Classification::Decided(point) =
                other.source_point_at(&parameter, &CurveContext::STRICT)
            else {
                continue;
            };
            for coordinate in [point.x(), point.y()] {
                if coordinate.exact_rational_ref().is_some() {
                    continue;
                }
                for candidate in [coordinate.clone(), -coordinate.clone()] {
                    if matches_selected(candidate.clone())? {
                        return Ok(Some(candidate));
                    }
                }
            }
        }

        let source = other.source_power_basis()?;
        for coefficient in source
            .x_numerator
            .iter()
            .chain(source.y_numerator)
            .chain(source.weight.into_iter().flatten())
            .chain(std::iter::once(other.distance()))
        {
            if coefficient.exact_rational_ref().is_some() {
                continue;
            }
            for candidate in [coefficient.clone(), -coefficient.clone()] {
                if matches_selected(candidate.clone())? {
                    return Ok(Some(candidate));
                }
            }
        }
        Ok(None)
    }

    /// Intersects the exact finite parallel range and its optional regular
    /// incident extension with this selected circle. All projection, bounds
    /// and normal-sheet decisions consume the retained finite domain.
    pub(crate) fn parallel_intersections(
        &self,
        other: &BezierParallel2,
        range: &CurveParameterRange2,
        incident: Option<&BezierParallelIncidentDomain2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParallelIntersections2>> {
        let frame_parallel = self.source_parallel();
        let same_source =
            frame_parallel.is_some_and(|parallel| other.source() == parallel.source());
        let normalize_source_reversal = !self.uses_selected_parallel_normal_frame();
        if normalize_source_reversal
            && let Some(frame_parallel) = frame_parallel
            && !same_source
            && other.source().is_reversal_of(frame_parallel.source())
        {
            let normalized = other.reversed();
            let normalized_range = CurveParameterRange2::new_validated(
                range
                    .end()
                    .unit_complement()
                    .ok_or(CurveError::InvalidCurveParameter)?,
                range
                    .start()
                    .unit_complement()
                    .ok_or(CurveError::InvalidCurveParameter)?,
            );
            let normalized_incident = incident.map(BezierParallelIncidentDomain2::reversed);
            return Ok(self
                .parallel_intersections(
                    &normalized,
                    &normalized_range,
                    normalized_incident.as_ref(),
                    policy,
                )?
                .map(|intersections| match intersections {
                    BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped { contacts, overlaps } => {
                        BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped { contacts: contacts
                                .into_iter()
                                .map(|contact| BezierAlgebraicCuspSemicircleParallelContact2 {
                                    parallel_parameter: contact
                                        .parallel_parameter
                                        .unit_complement(),
                                    tangent_cross_sign: contact.tangent_cross_sign.map(|sign| {
                                        match sign {
                                            RealSign::Positive => RealSign::Negative,
                                            RealSign::Negative => RealSign::Positive,
                                            RealSign::Zero => RealSign::Zero,
                                        }
                                    }),
                                    location: contact.location,
                                    correlation: contact.correlation,
                                })
                                .collect(), overlaps: overlaps
                                .into_iter()
                                .map(|overlap| {
                                    let range = overlap.other_range();
                                    BezierAlgebraicCuspSemicircleMappedOverlap2 {
                                        other_range: CurveParameterRange2::new_validated(
                                            range.end().unit_complement().expect("a mapped parallel endpoint is a scalar"),
                                            range.start().unit_complement().expect("a mapped parallel endpoint is a scalar"),
                                        ),
                                        cusp_start: overlap.cusp_start_parameter(),
                                        cusp_end: overlap.cusp_end_parameter(),
                                        orientation: match overlap.orientation() {
                                            CurveOverlapOrientation2::Same => {
                                                CurveOverlapOrientation2::Reversed
                                            }
                                            CurveOverlapOrientation2::Reversed => {
                                                CurveOverlapOrientation2::Same
                                            }
                                        },
                                        parameter_map: overlap.parameter_map.clone(),
                                        map_reversed: !overlap.map_reversed,
                                    }
                                })
                                .collect() }
                    }
                    BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiber { .. } => unreachable!(
                        "rational-frame source reversal produced selected-fiber evidence"
                    ),
                    BezierAlgebraicCuspSemicircleParallelIntersections2::RetainedContacts(
                        contacts,
                    ) => BezierAlgebraicCuspSemicircleParallelIntersections2::RetainedContacts(
                        contacts
                            .into_iter()
                            .map(|mut contact| {
                                contact.other_parameter = contact
                                    .other_parameter
                                    .unit_complement()
                                    .expect("a retained line parameter has a unit complement");
                                contact.retained.tangent_cross_sign = match contact
                                    .retained
                                    .tangent_cross_sign
                                {
                                    RealSign::Positive => RealSign::Negative,
                                    RealSign::Negative => RealSign::Positive,
                                    RealSign::Zero => RealSign::Zero,
                                };
                                contact.tangent_dot_sign = match contact.tangent_dot_sign {
                                    RealSign::Positive => RealSign::Negative,
                                    RealSign::Negative => RealSign::Positive,
                                    RealSign::Zero => RealSign::Zero,
                                };
                                contact.circle_side_of_parallel =
                                    contact.circle_side_of_parallel.map(|side| match side {
                                        crate::classify::LineSide::Left => {
                                            crate::classify::LineSide::Right
                                        }
                                        crate::classify::LineSide::Right => {
                                            crate::classify::LineSide::Left
                                        }
                                        crate::classify::LineSide::On => {
                                            crate::classify::LineSide::On
                                        }
                                    });
                                contact
                            })
                            .collect(),
                    ),
                    BezierAlgebraicCuspSemicircleParallelIntersections2::CoincidentCircleComponent => {
                        BezierAlgebraicCuspSemicircleParallelIntersections2::CoincidentCircleComponent
                    }
                    BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection => {
                        BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection
                    }
                }));
        }
        let domain = CurveParameterDomain2::new(
            range,
            incident.map(BezierParallelIncidentDomain2::parameter_ray),
        );
        let rational_component = policy.bounded_exact_predicate_pass(|| {
            if incident.is_some() {
                other
                    .rational_parallel_components_in_domains([domain], policy)
                    .map(|result| result.map(|curves| curves.map(|[curve]| curve)))
            } else {
                other
                    .exact_rational_parallel_component_on_regular_range(range, policy)
                    .map(|result| {
                        result.map(|component| component.map(|component| component.curve().clone()))
                    })
            }
        })?;
        if let Classification::Decided(Some(curve)) = &rational_component {
            match self.exact_linear_parallel_intersections(curve, range, incident, policy)? {
                Classification::Decided(Some(intersections)) => {
                    return Ok(Classification::Decided(intersections));
                }
                Classification::Decided(None) | Classification::Uncertain(_) => {}
            }
        }
        // These legacy native accelerators prove only the unit chart. A
        // finite exterior request proceeds through the common domain kernel.
        let unit = CurveParameterRange2::unit();
        let native_range = matches!(
            CurveParameterDomain2::new(&unit, None).contains_finite_range(range, policy),
            Ok(Classification::Decided(true)),
        );
        if self.data.frame.rational().is_some()
            && let (Some(incident), Some(center)) = (incident, self.exact_center(policy)?)
        {
            let radius_squared = self.radial_distance() * self.radial_distance();
            let finite = other.circle_incidence(&center, &radius_squared, range, &[], policy)?;
            let exterior = other.circle_incidence_on_incident_ray(
                &center,
                &radius_squared,
                incident,
                policy,
            )?;
            if let (Classification::Decided(finite), Classification::Decided(exterior)) =
                (finite, exterior)
            {
                let mut parameters: Vec<_> =
                    finite.into_iter().map(|(parameter, _)| parameter).collect();
                parameters.reserve(exterior.len());
                for (parameter, _) in exterior {
                    match domain.contains_finite_parameter(&parameter.clone().into(), policy)? {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) => parameters.push(parameter),
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                let mut contacts = Vec::with_capacity(parameters.len());
                for parameter in parameters {
                    match self.parallel_contact_at_certified_parameter(other, parameter, policy)? {
                        Classification::Decided(Some(contact)) => contacts.push(contact),
                        Classification::Decided(None) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-parallel-kernel",
                    "exact-center-incident-ray",
                );
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped {
                        contacts,
                        overlaps: Vec::new(),
                    },
                ));
            }
        }
        // General selected-normal frames benefit from rational substitution.
        // Other circle frames already own a compact direct analytic system;
        // rationalizing a nonlinear image can enlarge its coefficient field.
        // Dispatch depends on that evidence, never on an absent finite range.
        if incident.is_none()
            && self.uses_selected_parallel_normal_frame()
            && let Classification::Decided(Some(curve)) = rational_component
        {
            match self
                .finite_parallel_intersections_from_rational_component(&curve, range, policy)?
            {
                Classification::Decided(Some(intersections)) => {
                    return Ok(Classification::Decided(intersections));
                }
                Classification::Decided(None) | Classification::Uncertain(_) => {}
            }
        }
        let represented_rational_frame = if self
            .rational_circle_frame_is_compactly_representable(&policy.strict_counterpart())?
        {
            matches!(
                self.represented_circle_frame(&policy.strict_counterpart())?,
                Classification::Decided(_)
            )
        } else {
            false
        };
        // Homogeneous equation builders and their cached parameter maps are
        // independent of the target domain. Prove the denominators needed by
        // this query on its actual finite cell, including retained endpoints.
        // The incident domain separately owns its rootless bridge and open
        // pole/speed barrier. A remote native singularity is not a premise.
        // The selected-normal builder shares this admission with fixed-distance
        // queries below; its target range is passed explicitly.
        if !self.uses_selected_parallel_normal_frame() {
            let source = other.source_power_basis()?;
            let needs_normal = represented_rational_frame
                || self.data.frame.rational().is_none()
                || real_sign(other.distance(), policy) != Some(RealSign::Zero);
            let speed = needs_normal
                .then(|| other.differential().map(parallel_speed_squared_polynomial))
                .transpose()?;
            for coefficients in source.weight.into_iter().chain(speed.as_deref()) {
                match polynomial_is_nonzero_on_parameter_range(coefficients, range, policy)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        if self.uses_selected_chord_normal_frame() && !self.uses_retained_circle_parallel_system() {
            return self.represented_parallel_intersections(other, range, incident, policy);
        }
        if represented_rational_frame {
            let represented =
                self.represented_parallel_intersections(other, range, incident, policy)?;
            if incident.is_some()
                || !matches!(
                    &represented,
                    Classification::Decided(
                        BezierAlgebraicCuspSemicircleParallelIntersections2::CoincidentCircleComponent
                    )
                )
            {
                return Ok(represented);
            }
            // A finite coincident component still needs an authored parameter
            // map so booleans can split and retain its selected cells. The
            // represented system proves the topology cheaply; replay the
            // general analytic system below to construct that overlap evidence.
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-parallel-kernel",
                "represented-coincident-component-replay",
            );
        }
        if self.uses_selected_parallel_normal_frame() {
            return self
                .selected_parallel_normal_parallel_intersections(other, range, incident, policy);
        }
        if incident.is_none()
            && let (Classification::Decided(first_bounds), Classification::Decided(second_bounds)) = (
                self.conservative_bounds(policy)?,
                crate::curve_support::CurveSupport2::Parallel(other.clone())
                    .certified_outer_bounds(range, 0, policy),
            )
            && first_bounds.overlaps(&second_bounds, policy) == Classification::Decided(false)
        {
            return Ok(Classification::Decided(
                BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped {
                    contacts: Vec::new(),
                    overlaps: Vec::new(),
                },
            ));
        }
        if self.uses_retained_circle_parallel_system() {
            let intersections =
                self.recursive_circle_parallel_intersections(other, range, incident, policy)?;
            if matches!(intersections, Classification::Uncertain(_))
                && self.uses_selected_chord_normal_frame()
            {
                return self.represented_parallel_intersections(other, range, incident, policy);
            }
            return Ok(intersections);
        }

        let system = self.parallel_system(other)?;
        let (shared_cusp_witness, witnessed_circular_component) = if incident.is_none() {
            if native_range && other.rational_source().is_some() {
                match other.exact_circular_parallel_component(&CurveContext::STRICT)? {
                    Classification::Decided(Some(curve)) => (None, Some(curve)),
                    Classification::Decided(None) | Classification::Uncertain(_) => {
                        (self.parallel_source_cusp_witness(other)?, None)
                    }
                }
            } else {
                (self.parallel_source_cusp_witness(other)?, None)
            }
        } else {
            (None, None)
        };
        #[cfg(feature = "dispatch-trace")]
        if shared_cusp_witness.is_some() {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-parallel-kernel",
                "shared-real-cusp-witness",
            );
        }
        let reduce_bivariate = |polynomial: BivariatePolynomial| {
            if let Some(witness) = shared_cusp_witness.as_ref() {
                Ok(Classification::Decided(BivariatePolynomial::new(vec![
                    bivariate_specialize_first(&polynomial, witness),
                ])))
            } else {
                reduce_algebraic_cusp_bivariate(polynomial, self.cusp_parameter(), policy)
            }
        };
        let reduce_expression = |expression: BezierAlgebraicCuspTwoTermExpression2| {
            if let Some(witness) = shared_cusp_witness.as_ref() {
                Ok(Classification::Decided(
                    BezierAlgebraicCuspTwoTermExpression2 {
                        rational: BivariatePolynomial::new(vec![bivariate_specialize_first(
                            &expression.rational,
                            witness,
                        )]),
                        radical: BivariatePolynomial::new(vec![bivariate_specialize_first(
                            &expression.radical,
                            witness,
                        )]),
                    },
                ))
            } else {
                reduce_algebraic_cusp_radical_expression(expression, self.cusp_parameter(), policy)
            }
        };
        let incidence = match reduce_bivariate(system.incidence)? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let circle = match reduce_expression(system.circle)? {
            Classification::Decided(expression) => expression,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let selected_half_plane = match reduce_expression(system.selected_half_plane)? {
            Classification::Decided(expression) => expression,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let speed_squared = match reduce_bivariate(system.speed_squared)? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let tangent_cross_source = match reduce_bivariate(system.tangent_cross_source)? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let diagonal_location = if same_source {
            let frame_parallel = frame_parallel.expect("same-source frame retained its parallel");
            let start_distance = frame_parallel.distance() + &self.data.radial_distance;
            let end_distance = frame_parallel.distance() - &self.data.radial_distance;
            if compare_reals(other.distance(), &start_distance, policy)
                == Some(std::cmp::Ordering::Equal)
            {
                Some(BezierAlgebraicCuspSemicircleContactLocation2::Start)
            } else if compare_reals(other.distance(), &end_distance, policy)
                == Some(std::cmp::Ordering::Equal)
            {
                Some(BezierAlgebraicCuspSemicircleContactLocation2::End)
            } else {
                None
            }
        } else {
            None
        };
        let projection_incidence = if diagonal_location.is_some() {
            let report = deflate_bivariate_fiber_diagonal_root_at_algebraic_parameter(
                &incidence,
                CurveResultantParameter::First,
                &parameter_representation(self.cusp_parameter(), policy),
                policy.predicate_policy(),
            );
            if report.certainty == PredicateCertainty::Approximate {
                policy.observe_approximate_512();
            }
            match report.status {
                AlgebraicFiberDiagonalDeflationStatus::Deflated => {
                    report.reduced_polynomial.ok_or_else(|| {
                        CurveError::Topology(
                            "deflated cusp/parallel diagonal did not retain its residual".into(),
                        )
                    })?
                }
                AlgebraicFiberDiagonalDeflationStatus::NotARoot => {
                    return Err(CurveError::Topology(
                        "source-related cusp/parallel endpoint was not on the squared circle"
                            .into(),
                    ));
                }
                AlgebraicFiberDiagonalDeflationStatus::IdenticallyZeroFiber => {
                    // A positive-dimensional support fiber has no isolated
                    // diagonal factor to remove. Replay its authored radical
                    // branch below and let the overlap own the endpoint.
                    incidence.clone()
                }
                AlgebraicFiberDiagonalDeflationStatus::InvalidEvidence => {
                    return Err(CurveError::InvalidBezierAlgebraicParameter);
                }
                AlgebraicFiberDiagonalDeflationStatus::UnsupportedCoefficient => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                AlgebraicFiberDiagonalDeflationStatus::Undecided => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                }
            }
        } else {
            incidence.clone()
        };
        let residual_is_rootless = diagonal_location.is_some()
            && incident.is_none()
            && bivariate_fiber_strict_sign_on_parameter_range(
                &projection_incidence,
                self.cusp_parameter(),
                range,
                policy,
            )?
            .is_some();
        let projection = if residual_is_rootless {
            BezierAlgebraicFiberProjection2::Parameters(Vec::new())
        } else {
            let projected = if incident.is_some() {
                algebraic_selected_fiber_parameters_with_incident_ray(
                    &projection_incidence,
                    self.cusp_parameter(),
                    domain,
                    usize::MAX,
                    MAX_SELECTED_FIBER_QUOTIENT_DEGREE,
                    policy,
                )?
            } else if let Some(witness) = shared_cusp_witness.as_ref() {
                selected_parameter_fiber_parameters(
                    &projection_incidence,
                    &BezierParameter2::Exact(witness.clone()),
                    MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
                    MAX_SELECTED_FIBER_QUOTIENT_DEGREE,
                    range,
                    policy,
                )?
            } else {
                // The common system was reduced at the selected root above;
                // retain that quotient authority and reject conjugate roots
                // before the complete general-resultant fallback.
                algebraic_selected_reduced_fiber_parameters(
                    &projection_incidence,
                    self.cusp_parameter(),
                    range,
                    policy,
                )?
            };
            match projected {
                Classification::Decided(projection) => projection,
                Classification::Uncertain(UncertaintyReason::Unsupported) if incident.is_none() => {
                    // An exactly recognized rational circle is the one supported
                    // coefficient-field collapse: it preserves the source
                    // parameter and constructs no approximate geometry. Keep
                    // every other unsupported projection explicit.
                    let circular_component =
                        if let Some(curve) = witnessed_circular_component.as_ref() {
                            Classification::Decided(Some(curve.clone()))
                        } else if native_range {
                            other.exact_circular_parallel_component(&policy.strict_counterpart())?
                        } else {
                            Classification::Decided(None)
                        };
                    if let Classification::Decided(Some(curve)) = circular_component {
                        match self.finite_parallel_intersections_from_rational_component(
                            &curve, range, policy,
                        )? {
                            Classification::Decided(Some(intersections)) => {
                                #[cfg(feature = "dispatch-trace")]
                                hyperreal::dispatch_trace::record(
                                    "hypercurve",
                                    "algebraic-circle-parallel-kernel",
                                    "exact-circular-component-fallback",
                                );
                                return Ok(Classification::Decided(intersections));
                            }
                            Classification::Decided(None) => {}
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let mut contact_incidence = incidence.clone();
        let candidates = match projection {
            BezierAlgebraicFiberProjection2::Parameters(parameters) => parameters,
            BezierAlgebraicFiberProjection2::IdenticallyZero => {
                if let Some(incident) = incident {
                    // The squared circle relation vanishes on the whole
                    // selected center fiber. One exact sample in the regular
                    // incident cell distinguishes a genuinely coincident
                    // supporting circle from the opposite radical sheet. A
                    // coincident component has infinitely many fillet
                    // centers, so retain that topology without allocating an
                    // artificial unbounded overlap chart. On the opposite
                    // sheet only common zeros of the two radical terms are
                    // geometric contacts; isolate all such finite and
                    // incident roots through the same quotient authority.
                    let sample = strict_sample_for_parallel_domain(range, Some(incident), policy)?;
                    let sample = match sample {
                        Classification::Decided(sample) => BezierParameter2::Exact(sample),
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let cusp_parameter = BezierParameter2::Algebraic(self.cusp_parameter().clone());
                    match algebraic_cusp_independent_radical_sum_sign(
                        &circle,
                        &speed_squared,
                        &cusp_parameter,
                        &sample,
                        policy,
                    )? {
                        Classification::Decided(RealSign::Zero) => {
                            return Ok(Classification::Decided(
                                BezierAlgebraicCuspSemicircleParallelIntersections2::CoincidentCircleComponent,
                            ));
                        }
                        Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                    let projected = algebraic_selected_fiber_parameters_with_incident_ray(
                        &circle.rational,
                        self.cusp_parameter(),
                        domain,
                        usize::MAX,
                        MAX_SELECTED_FIBER_QUOTIENT_DEGREE,
                        policy,
                    )?;
                    match projected {
                        Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                            parameters,
                        )) => {
                            contact_incidence = circle.rational.clone();
                            parameters
                        }
                        Classification::Decided(
                            BezierAlgebraicFiberProjection2::IdenticallyZero
                            | BezierAlgebraicFiberProjection2::Degenerate,
                        ) => {
                            return Ok(Classification::Decided(
                                BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection,
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                } else {
                    match self.replay_positive_dimensional_parallel_circle(
                        other,
                        range,
                        &circle,
                        &selected_half_plane,
                        system.diameter_side.clone(),
                        system.radius_squared_denominator.clone(),
                        &speed_squared,
                        policy,
                    )? {
                        Classification::Decided(
                            BezierAlgebraicCuspParallelComponentReplay2::Resolved(result),
                        ) => return Ok(Classification::Decided(result)),
                        Classification::Decided(
                            BezierAlgebraicCuspParallelComponentReplay2::IsolatedCandidates {
                                incidence,
                                parameters,
                            },
                        ) => {
                            contact_incidence = incidence;
                            parameters
                        }
                        Classification::Decided(
                            BezierAlgebraicCuspParallelComponentReplay2::Degenerate,
                        ) => {
                            return Ok(Classification::Decided(
                                BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection,
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
            BezierAlgebraicFiberProjection2::Degenerate => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection,
                ));
            }
        };
        let cusp_parameter = BezierParameter2::Algebraic(self.cusp_parameter().clone());
        let mut diameter_side = Some(system.diameter_side);
        let finite_parameter_is_retained =
            |parameter: &BezierParameter2| -> CurveResult<Classification<bool>> {
                match domain.contains_finite_parameter(&parameter.clone().into(), policy)? {
                    Classification::Decided(true) => Ok(Classification::Decided(true)),
                    Classification::Decided(false) => match incident {
                        Some(incident) => {
                            incident.contains_extension_parameter(&parameter.clone().into(), policy)
                        }
                        None => Ok(Classification::Decided(false)),
                    },
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                }
            };
        // A source-related start/end parallel meets this semicircle at the
        // retained cusp parameter by construction.  Its tangent is the same
        // source-tangent line as the semicircle endpoint tangent, so this is
        // an exact nontransverse contact.  Do not replay that deliberately
        // correlated diagonal root through the independent-root radical
        // predicate below: the latter is authoritative only for residual
        // projection roots.
        let mut contacts =
            Vec::with_capacity(candidates.len() + usize::from(diagonal_location.is_some()));
        if let Some(location) = diagonal_location {
            let retained = match finite_parameter_is_retained(&cusp_parameter)? {
                Classification::Decided(retained) => retained,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if retained {
                contacts.push(BezierAlgebraicCuspSemicircleParallelContact2 {
                    parallel_parameter: cusp_parameter.clone(),
                    tangent_cross_sign: Some(RealSign::Zero),
                    location,
                    correlation: BezierAlgebraicCuspSemicircleParallelCorrelation2::Map,
                });
            }
        }
        for candidate in candidates {
            match finite_parameter_is_retained(&candidate)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let circle_sign = match algebraic_cusp_correlated_radical_sum_sign(
                &contact_incidence,
                &circle,
                &speed_squared,
                &cusp_parameter,
                &candidate,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if circle_sign != RealSign::Zero {
                continue;
            }
            let selected_sign = match algebraic_cusp_correlated_radical_sum_sign(
                &contact_incidence,
                &selected_half_plane,
                &speed_squared,
                &cusp_parameter,
                &candidate,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if selected_sign == RealSign::Negative {
                continue;
            }
            let location = if selected_sign == RealSign::Zero {
                let expression = diameter_side
                    .take()
                    .expect("diameter-side expression is reduced at most once");
                let reduced = match reduce_expression(expression)? {
                    Classification::Decided(expression) => expression,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                diameter_side = Some(reduced);
                match algebraic_cusp_correlated_radical_sum_sign(
                    &contact_incidence,
                    diameter_side
                        .as_ref()
                        .expect("reduced diameter-side expression was retained"),
                    &speed_squared,
                    &cusp_parameter,
                    &candidate,
                    policy,
                )? {
                    Classification::Decided(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(RealSign::Negative) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::End
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "nonzero algebraic semicircle radius had an indeterminate diameter endpoint"
                                .into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            } else {
                BezierAlgebraicCuspSemicircleContactLocation2::Interior
            };
            let source_cross = match algebraic_selected_correlated_predicate_sign(
                &contact_incidence,
                &tangent_cross_source,
                &cusp_parameter,
                &candidate,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let tangent_cross_sign = if source_cross == RealSign::Zero {
                Some(RealSign::Zero)
            } else {
                other.apply_parallel_derivative_scale_to_tangent_sign(
                    Classification::Decided(source_cross),
                    &candidate,
                    policy,
                )?
            };
            contacts.push(BezierAlgebraicCuspSemicircleParallelContact2 {
                parallel_parameter: candidate,
                tangent_cross_sign,
                location,
                correlation: BezierAlgebraicCuspSemicircleParallelCorrelation2::Map,
            });
        }
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped {
                contacts,
                overlaps: Vec::new(),
            },
        ))
    }

    /// Replays an identically-zero squared circle fiber on the authored
    /// radical branch and, when selected, clips it to the retained semicircle.
    ///
    /// Regularity gives `S > 0` on the complete parallel range. From
    /// `A^2 S - B^2 == 0`, one exact interior sign therefore distinguishes the
    /// selected circle branch from its opposite on every nonzero cell. Common
    /// zeros remain isolated candidates on the opposite branch. On the
    /// selected branch, roots of the squared half-plane expression partition
    /// the parameter range; exact unsquared replay retains only its positive
    /// cells as overlap evidence.
    #[allow(clippy::too_many_arguments)]
    pub(in crate::bezier_offset) fn replay_positive_dimensional_parallel_circle(
        &self,
        other: &BezierParallel2,
        range: &CurveParameterRange2,
        circle: &BezierAlgebraicCuspTwoTermExpression2,
        selected_half_plane: &BezierAlgebraicCuspTwoTermExpression2,
        diameter_side: BezierAlgebraicCuspTwoTermExpression2,
        radius_squared_denominator: BivariatePolynomial,
        speed_squared: &BivariatePolynomial,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspParallelComponentReplay2>> {
        let retained_range = range;
        let [retained_start, retained_end] = match range.ordered_endpoints(policy)? {
            Classification::Decided(endpoints) => endpoints,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        // Only the one-field sign replay needs ordinary algebraic parameters.
        // Publish the original boundaries, so clipping retains their identity.
        let promote = |parameter: &CurveParameter2| {
            policy.strict_predicate_pass(|| parameter.promoted_bezier_parameter_complete(policy))
        };
        let start = match promote(retained_start)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let end = match promote(retained_end)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let range = &BezierParameterRange2::new_validated(start, end);
        let cusp_parameter = BezierParameter2::Algebraic(self.cusp_parameter().clone());
        let mut circle_zeros = Vec::new();
        let circle_is_rootless = bivariate_fiber_strict_sign_on_parameter_range(
            &circle.rational,
            self.cusp_parameter(),
            retained_range,
            policy,
        )?
        .is_some();
        if !circle_is_rootless {
            let projection = match selected_fiber_parameters(
                &circle.rational,
                &cusp_parameter,
                retained_range,
                policy,
            )? {
                Classification::Decided(projection) => projection,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let BezierAlgebraicFiberProjection2::Parameters(parameters) = projection else {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspParallelComponentReplay2::Degenerate,
                ));
            };
            for parameter in parameters {
                let after_start = match parameter.cmp_by_refinement(range.start(), policy)? {
                    Classification::Decided(std::cmp::Ordering::Less) => false,
                    Classification::Decided(
                        std::cmp::Ordering::Equal | std::cmp::Ordering::Greater,
                    ) => true,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let before_end = match parameter.cmp_by_refinement(range.end(), policy)? {
                    Classification::Decided(std::cmp::Ordering::Greater) => false,
                    Classification::Decided(
                        std::cmp::Ordering::Equal | std::cmp::Ordering::Less,
                    ) => true,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if after_start && before_end {
                    circle_zeros.push(parameter);
                }
            }
        }

        // Do not sample a common zero of the two radical branches. The
        // projected rational-term roots partition the range into cells on
        // which the selected-versus-opposite branch is constant.
        let sample = if circle_is_rootless {
            match range
                .start()
                .strict_scalar_between_ordered(range.end(), policy)?
            {
                Classification::Decided(sample) => Some(BezierParameter2::Exact(sample)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        } else {
            let mut left = range.start();
            let mut sample = None;
            for zero in &circle_zeros {
                match left.cmp_by_refinement(zero, policy)? {
                    Classification::Decided(std::cmp::Ordering::Less) => {
                        sample = Some(match left.strict_scalar_between_ordered(zero, policy)? {
                            Classification::Decided(sample) => BezierParameter2::Exact(sample),
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        });
                        break;
                    }
                    Classification::Decided(std::cmp::Ordering::Equal) => left = zero,
                    Classification::Decided(std::cmp::Ordering::Greater) => {
                        return Err(CurveError::Topology(
                            "algebraic cusp circle roots were not ordered".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            if sample.is_none() {
                sample = match left.cmp_by_refinement(range.end(), policy)? {
                    Classification::Decided(std::cmp::Ordering::Less) => Some(
                        match left.strict_scalar_between_ordered(range.end(), policy)? {
                            Classification::Decided(sample) => BezierParameter2::Exact(sample),
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        },
                    ),
                    Classification::Decided(std::cmp::Ordering::Equal) => None,
                    Classification::Decided(std::cmp::Ordering::Greater) => {
                        return Err(CurveError::Topology(
                            "algebraic cusp circle roots exceeded their range".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            }
            sample
        };
        let Some(sample) = sample else {
            return Ok(Classification::Decided(
                BezierAlgebraicCuspParallelComponentReplay2::Degenerate,
            ));
        };
        let branch_sign = match algebraic_cusp_independent_radical_sum_sign(
            circle,
            speed_squared,
            &cusp_parameter,
            &sample,
            policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if branch_sign != RealSign::Zero {
            return Ok(Classification::Decided(
                BezierAlgebraicCuspParallelComponentReplay2::IsolatedCandidates {
                    incidence: circle.rational.clone(),
                    parameters: circle_zeros,
                },
            ));
        }

        let half_incidence = bivariate_subtract(
            &bivariate_multiply(
                &bivariate_multiply(&selected_half_plane.rational, &selected_half_plane.rational),
                speed_squared,
            ),
            &bivariate_multiply(&selected_half_plane.radical, &selected_half_plane.radical),
        );
        let half_incidence =
            match reduce_algebraic_cusp_bivariate(half_incidence, self.cusp_parameter(), policy)? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let projected = match selected_fiber_parameters(
            &half_incidence,
            &cusp_parameter,
            retained_range,
            policy,
        )? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(parameters)) => {
                parameters
            }
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspParallelComponentReplay2::Degenerate,
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        #[derive(Clone)]
        struct Boundary {
            parameter: BezierParameter2,
            retained_parameter: CurveParameter2,
            selected_relation: bool,
        }

        let mut start = Boundary {
            parameter: range.start().clone(),
            retained_parameter: retained_start.clone(),
            selected_relation: false,
        };
        let mut end = Boundary {
            parameter: range.end().clone(),
            retained_parameter: retained_end.clone(),
            selected_relation: false,
        };
        let mut interior = Vec::new();
        for parameter in projected {
            let selected_sign = match algebraic_cusp_correlated_radical_sum_sign(
                &half_incidence,
                selected_half_plane,
                speed_squared,
                &cusp_parameter,
                &parameter,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if selected_sign != RealSign::Zero {
                continue;
            }
            let start_order = match parameter.cmp_by_refinement(range.start(), policy)? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let end_order = match parameter.cmp_by_refinement(range.end(), policy)? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match (start_order, end_order) {
                (std::cmp::Ordering::Equal, _) => {
                    start.parameter = parameter.clone();
                    start.selected_relation = true;
                }
                (_, std::cmp::Ordering::Equal) => {
                    end.parameter = parameter.clone();
                    end.selected_relation = true;
                }
                (std::cmp::Ordering::Greater, std::cmp::Ordering::Less) => {
                    let boundary = Boundary {
                        retained_parameter: parameter.clone().into(),
                        parameter,
                        selected_relation: true,
                    };
                    interior.push(boundary);
                }
                _ => {}
            }
        }
        let mut boundaries = Vec::with_capacity(interior.len() + 2);
        boundaries.push(start);
        boundaries.extend(interior);
        boundaries.push(end);

        let diameter_side = match reduce_algebraic_cusp_radical_expression(
            diameter_side,
            self.cusp_parameter(),
            policy,
        )? {
            Classification::Decided(expression) => expression,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radius_squared_denominator = match reduce_algebraic_cusp_bivariate(
            radius_squared_denominator,
            self.cusp_parameter(),
            policy,
        )? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let parameter_map = self.parallel_parameter_map_from_reduced(
            other,
            half_incidence.clone(),
            diameter_side.clone(),
            radius_squared_denominator,
            speed_squared.clone(),
            policy,
        );
        let endpoint = |boundary: &Boundary| {
            let selected_sign = if boundary.selected_relation {
                algebraic_cusp_correlated_radical_sum_sign(
                    &half_incidence,
                    selected_half_plane,
                    speed_squared,
                    &cusp_parameter,
                    &boundary.parameter,
                    policy,
                )
            } else {
                algebraic_cusp_independent_radical_sum_sign(
                    selected_half_plane,
                    speed_squared,
                    &cusp_parameter,
                    &boundary.parameter,
                    policy,
                )
            }?;
            let location = match selected_sign {
                Classification::Decided(RealSign::Positive) => {
                    BezierAlgebraicCuspSemicircleContactLocation2::Interior
                }
                Classification::Decided(RealSign::Zero) => {
                    let diameter_sign = if boundary.selected_relation {
                        algebraic_cusp_correlated_radical_sum_sign(
                            &half_incidence,
                            &diameter_side,
                            speed_squared,
                            &cusp_parameter,
                            &boundary.parameter,
                            policy,
                        )
                    } else {
                        algebraic_cusp_independent_radical_sum_sign(
                            &diameter_side,
                            speed_squared,
                            &cusp_parameter,
                            &boundary.parameter,
                            policy,
                        )
                    }?;
                    match diameter_sign {
                        Classification::Decided(RealSign::Positive) => {
                            BezierAlgebraicCuspSemicircleContactLocation2::Start
                        }
                        Classification::Decided(RealSign::Negative) => {
                            BezierAlgebraicCuspSemicircleContactLocation2::End
                        }
                        Classification::Decided(RealSign::Zero) => {
                            return Err(CurveError::Topology(
                                "positive-radius cusp overlap endpoint had zero diameter side"
                                    .into(),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Classification::Decided(RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "selected cusp overlap cell acquired a negative endpoint".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let contact = BezierAlgebraicCuspSemicircleParallelContact2 {
                parallel_parameter: boundary.parameter.clone(),
                tangent_cross_sign: Some(RealSign::Zero),
                location,
                correlation: if boundary.selected_relation {
                    BezierAlgebraicCuspSemicircleParallelCorrelation2::Map
                } else {
                    BezierAlgebraicCuspSemicircleParallelCorrelation2::Independent
                },
            };
            Ok(Classification::Decided((
                parameter_map.contact_parameter(&contact),
                contact,
            )))
        };

        let mut overlaps = Vec::new();
        let mut covered_boundaries = vec![false; boundaries.len()];
        for (index, pair) in boundaries.windows(2).enumerate() {
            let cell_sample = match pair[0]
                .parameter
                .strict_scalar_between_ordered(&pair[1].parameter, policy)?
            {
                Classification::Decided(sample) => BezierParameter2::Exact(sample),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let sign = match algebraic_cusp_independent_radical_sum_sign(
                selected_half_plane,
                speed_squared,
                &cusp_parameter,
                &cell_sample,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if sign == RealSign::Negative {
                continue;
            }
            if sign == RealSign::Zero {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspParallelComponentReplay2::Degenerate,
                ));
            }
            let (first_cusp, _) = match endpoint(&pair[0])? {
                Classification::Decided(endpoint) => endpoint,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let (second_cusp, _) = match endpoint(&pair[1])? {
                Classification::Decided(endpoint) => endpoint,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let order = match first_cusp.cmp_by_refinement(&second_cusp, policy)? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let (cusp_start, cusp_end, orientation) = match order {
                std::cmp::Ordering::Less => {
                    (first_cusp, second_cusp, CurveOverlapOrientation2::Same)
                }
                std::cmp::Ordering::Greater => {
                    (second_cusp, first_cusp, CurveOverlapOrientation2::Reversed)
                }
                std::cmp::Ordering::Equal => {
                    return Err(CurveError::Topology(
                        "positive parallel overlap mapped to a zero cusp range".into(),
                    ));
                }
            };
            covered_boundaries[index] = true;
            covered_boundaries[index + 1] = true;
            overlaps.push(BezierAlgebraicCuspSemicircleMappedOverlap2 {
                other_range: CurveParameterRange2::new_validated(
                    pair[0].retained_parameter.clone(),
                    pair[1].retained_parameter.clone(),
                ),
                cusp_start,
                cusp_end,
                orientation,
                parameter_map: BezierAlgebraicCuspSemicircleMappedOverlapMap2::Parallel(
                    parameter_map.clone(),
                ),
                map_reversed: false,
            });
        }
        let mut contacts = Vec::new();
        for (index, boundary) in boundaries.iter().enumerate() {
            if covered_boundaries[index] || !boundary.selected_relation {
                continue;
            }
            let (_, contact) = match endpoint(boundary)? {
                Classification::Decided(endpoint) => endpoint,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if !contacts
                .iter()
                .any(|existing: &BezierAlgebraicCuspSemicircleParallelContact2| {
                    matches!(
                        existing
                            .parallel_parameter
                            .same_value(&contact.parallel_parameter, policy),
                        Ok(Classification::Decided(true))
                    )
                })
            {
                contacts.push(contact);
            }
        }
        Ok(Classification::Decided(
            BezierAlgebraicCuspParallelComponentReplay2::Resolved(
                BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped { contacts, overlaps },
            ),
        ))
    }

    /// Builds one pair-shared exact contact-to-semicircle parameter map for an
    /// analytic parallel without introducing another algebraic-number tower.
    pub(crate) fn parallel_parameter_map(
        &self,
        other: &BezierParallel2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParallelParameterMap2>> {
        let publish = |system| {
            Classification::Decided(BezierAlgebraicCuspSemicircleParallelParameterMap2 {
                data: Arc::new(BezierAlgebraicCuspSemicircleParallelParameterMapData2 {
                    semicircle: self.clone(),
                    parallel: other.clone(),
                    system,
                    policy: policy.retained_object_policy(),
                    parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
                }),
            })
        };
        if self.uses_retained_circle_parallel_system() {
            match self.recursive_circle_parallel_system(other, policy)? {
                Classification::Decided(system) => {
                    return Ok(publish(
                        BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::Recursive {
                            system,
                        },
                    ));
                }
                Classification::Uncertain(reason) if !self.uses_selected_chord_normal_frame() => {
                    return Ok(Classification::Uncertain(reason));
                }
                Classification::Uncertain(_) => {}
            }
        }
        if self.uses_selected_chord_normal_frame() {
            return Ok(match self.represented_parallel_system(other, policy)? {
                Classification::Decided(system) => publish(
                    BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::Represented {
                        system,
                    },
                ),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            });
        }
        let system = self.parallel_system(other)?;
        let incidence =
            match reduce_algebraic_cusp_bivariate(system.incidence, self.cusp_parameter(), policy)?
            {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let diameter_rational = match reduce_algebraic_cusp_bivariate(
            system.diameter_side.rational,
            self.cusp_parameter(),
            policy,
        )? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let diameter_radical = match reduce_algebraic_cusp_bivariate(
            system.diameter_side.radical,
            self.cusp_parameter(),
            policy,
        )? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radius_squared_denominator = match reduce_algebraic_cusp_bivariate(
            system.radius_squared_denominator,
            self.cusp_parameter(),
            policy,
        )? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let speed_squared = match reduce_algebraic_cusp_bivariate(
            system.speed_squared,
            self.cusp_parameter(),
            policy,
        )? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(
            self.parallel_parameter_map_from_reduced(
                other,
                incidence,
                BezierAlgebraicCuspTwoTermExpression2 {
                    rational: diameter_rational,
                    radical: diameter_radical,
                },
                radius_squared_denominator,
                speed_squared,
                policy,
            ),
        ))
    }

    pub(in crate::bezier_offset) fn parallel_parameter_map_from_reduced(
        &self,
        other: &BezierParallel2,
        incidence: BivariatePolynomial,
        diameter_side: BezierAlgebraicCuspTwoTermExpression2,
        radius_squared_denominator: BivariatePolynomial,
        speed_squared: BivariatePolynomial,
        policy: &CurveContext,
    ) -> BezierAlgebraicCuspSemicircleParallelParameterMap2 {
        BezierAlgebraicCuspSemicircleParallelParameterMap2 {
            data: Arc::new(BezierAlgebraicCuspSemicircleParallelParameterMapData2 {
                semicircle: self.clone(),
                parallel: other.clone(),
                system: BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::OneField {
                    cusp_parameter: BezierParameter2::Algebraic(self.cusp_parameter().clone()),
                    incidence,
                    diameter: diameter_side,
                    radius_squared_denominator,
                    speed_squared,
                },
                policy: policy.retained_object_policy(),
                parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
            }),
        }
    }
}
