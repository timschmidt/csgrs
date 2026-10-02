//! Represented and recursive circle-frame authorities.

use super::*;

impl BezierAlgebraicCuspSemicircle2 {
    /// Materializes an arbitrary retained chord-normal frame without adjoining
    /// its endpoint and center fields eagerly to the circle carrier.
    ///
    /// The positive speed root is selected under STRICT.  Both unit-normal
    /// components divide by that same represented root, preserving the
    /// authored left-normal sheet independently of the enclosing operation's
    /// terminal equality policy.
    pub(in crate::bezier_offset) fn represented_chord_normal_frame(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierRepresentedSelectedRadialCircleFrame2>> {
        let Some(frame) = self.data.frame.chord_normal() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a represented chord-normal frame crossed predicate policies".into(),
            ));
        }
        let center = match represented_point_evidence_coordinates(&frame.center, policy)? {
            Classification::Decided(center) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-chord-normal-frame",
                    "center-represented",
                );
                center
            }
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-chord-normal-frame",
                    match reason {
                        UncertaintyReason::Unsupported => "center-unsupported",
                        UncertaintyReason::Predicate => "center-predicate",
                        UncertaintyReason::Ordering => "center-ordering",
                        UncertaintyReason::RealSign => "center-real-sign",
                        UncertaintyReason::Boundary => "center-boundary",
                    },
                );
                return Ok(Classification::Uncertain(reason));
            }
        };
        let unit_radial = match represented_chord_unit_direction(
            &frame.anchor,
            BezierAlgebraicChordUnitDisplacement2::LeftNormal,
            policy,
        )? {
            Classification::Decided(unit_radial) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-chord-normal-frame",
                    "normal-represented",
                );
                unit_radial
            }
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-chord-normal-frame",
                    match reason {
                        UncertaintyReason::Unsupported => "normal-unsupported",
                        UncertaintyReason::Predicate => "normal-predicate",
                        UncertaintyReason::Ordering => "normal-ordering",
                        UncertaintyReason::RealSign => "normal-real-sign",
                        UncertaintyReason::Boundary => "normal-boundary",
                    },
                );
                return Ok(Classification::Uncertain(reason));
            }
        };
        let compact = |coordinate| {
            hypersolve::compact_algebraic_root_low_degree_witness(&coordinate).unwrap_or(coordinate)
        };
        Ok(Classification::Decided(
            BezierRepresentedSelectedRadialCircleFrame2 {
                center: center.map(compact),
                unit_radial: unit_radial.map(compact),
                signed_radius: self.radial_distance().clone(),
            },
        ))
    }

    /// Materializes the rank-independent frame used only when the compact
    /// two-root source-pair frame cannot represent a recursive construction.
    /// The authored center and parameter-zero radial are each selected under
    /// STRICT before any enclosing operation may use APPROXIMATE_512.
    pub(in crate::bezier_offset) fn represented_selected_radial_frame(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierRepresentedSelectedRadialCircleFrame2>> {
        let BezierSelectedRadialCircleFrameSource2 {
            frame,
            pair_map,
            pair_contact,
            support_first,
            similarity,
        } = match self.selected_radial_frame_source(policy)? {
            Classification::Decided(source) => source,
            Classification::Uncertain(UncertaintyReason::Unsupported) => {
                return self.represented_recursive_selected_radial_frame(policy);
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut center = match pair_map.represented_selected_radial_contact_point(pair_contact)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        // Keep the unscaled retained radial through a similarity. Scaling its
        // coordinates first would form two scaled projective eliminants and
        // then a second tensor image, discarding the cheap shared-frame form.
        // Normalize in the final transformed image instead.
        let unit_radial_scale = if similarity.is_some() {
            Real::one()
        } else {
            (Real::one() / &frame.normal_denominator)?
        };
        let mut unit_radial = match pair_map.represented_selected_radial_vector(
            pair_contact,
            support_first,
            &unit_radial_scale,
            &Real::zero(),
            policy,
        )? {
            Classification::Decided(radial) => radial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let signed_radius = self.radial_distance().clone();
        center = center.map(|coordinate| {
            hypersolve::compact_algebraic_root_low_degree_witness(&coordinate).unwrap_or(coordinate)
        });
        unit_radial = unit_radial.map(|coordinate| {
            hypersolve::compact_algebraic_root_low_degree_witness(&coordinate).unwrap_or(coordinate)
        });
        let Some(similarity) = similarity else {
            return Ok(Classification::Decided(
                BezierRepresentedSelectedRadialCircleFrame2 {
                    center,
                    unit_radial,
                    signed_radius,
                },
            ));
        };
        let center = match represented_similarity_point(&center, &similarity) {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let inverse_normal_denominator = (Real::one() / &frame.normal_denominator)?;
        let unit_radial = match represented_similarity_vector(
            &unit_radial,
            &similarity,
            &inverse_normal_denominator,
        ) {
            Classification::Decided(unit_radial) => unit_radial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(
            BezierRepresentedSelectedRadialCircleFrame2 {
                center,
                unit_radial,
                signed_radius,
            },
        ))
    }

    /// Materializes a selected-radial frame whose center parameter is not a
    /// direct circle-pair root.
    ///
    /// The mapped parameter already certifies one point `P` on its support
    /// circle.  If `C` is that support's represented center, the authored
    /// frame is exactly `(P-C)/normal_denominator`.  Keeping the contact and
    /// support coordinates as independently selected algebraic numbers until
    /// this cold affine composition avoids imposing a primitive-element
    /// requirement on every recursively authored circle.
    pub(in crate::bezier_offset) fn represented_recursive_selected_radial_frame(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierRepresentedSelectedRadialCircleFrame2>> {
        let frame = self.data.frame.selected_radial().ok_or_else(|| {
            CurveError::Topology(
                "a non-radial selected circle entered recursive frame representation".into(),
            )
        })?;
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a recursive selected-radial frame crossed predicate policies".into(),
            ));
        }
        let support = frame.center_parameter.semicircle_carrier();
        let center_parameter =
            BezierAlgebraicCuspSemicircleParameter2::Mapped(frame.center_parameter.clone());
        let center = match center_parameter.coincident_point_evidence(support, policy)? {
            Classification::Decided(Some(center)) => center,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center = match represented_point_evidence_coordinates(&center, policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let support_center = match support.represented_circle_frame(policy)? {
            Classification::Decided(frame) => frame.center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let inverse_normal_denominator = (Real::one() / &frame.normal_denominator)?;
        let unit_radial = [
            represented_affine_coordinate(
                &[
                    (&center[0], &inverse_normal_denominator),
                    (&support_center[0], &(-inverse_normal_denominator.clone())),
                ],
                &Real::zero(),
            ),
            represented_affine_coordinate(
                &[
                    (&center[1], &inverse_normal_denominator),
                    (&support_center[1], &(-inverse_normal_denominator.clone())),
                ],
                &Real::zero(),
            ),
        ];
        let [
            Classification::Decided(unit_x),
            Classification::Decided(unit_y),
        ] = unit_radial
        else {
            let reason = unit_radial
                .into_iter()
                .find_map(|coordinate| match coordinate {
                    Classification::Decided(_) => None,
                    Classification::Uncertain(reason) => Some(reason),
                })
                .unwrap_or(UncertaintyReason::Predicate);
            return Ok(Classification::Uncertain(reason));
        };
        let compact = |coordinate| {
            hypersolve::compact_algebraic_root_low_degree_witness(&coordinate).unwrap_or(coordinate)
        };
        Ok(Classification::Decided(
            BezierRepresentedSelectedRadialCircleFrame2 {
                center: center.map(compact),
                unit_radial: [compact(unit_x), compact(unit_y)],
                signed_radius: self.radial_distance().clone(),
            },
        ))
    }

    pub(in crate::bezier_offset) fn represented_circle_frame(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierRepresentedSelectedRadialCircleFrame2>> {
        if self.uses_selected_radial_frame() {
            return self.represented_selected_radial_frame(policy);
        }
        if self.uses_selected_chord_normal_frame() {
            return self.represented_chord_normal_frame(policy);
        }
        if let Some(frame) = self.data.frame.rational() {
            let (center_x, center_y) = self
                .data
                .frame
                .point_numerators_at_parallel_distance(&self.center_parallel_distance())?;
            if let ([center_x], [center_y], [unit_x], [unit_y], [denominator]) = (
                center_x.as_slice(),
                center_y.as_slice(),
                frame.data.normal_x_numerator.as_slice(),
                frame.data.normal_y_numerator.as_slice(),
                frame.data.denominator.as_slice(),
            ) {
                let center_x = (center_x / denominator)?;
                let center_y = (center_y / denominator)?;
                let unit_x = (unit_x / denominator)?;
                let unit_y = (unit_y / denominator)?;
                return Ok(Classification::Decided(
                    BezierRepresentedSelectedRadialCircleFrame2 {
                        center: [
                            AlgebraicRootRepresentation::from_exact_value(&center_x),
                            AlgebraicRootRepresentation::from_exact_value(&center_y),
                        ],
                        unit_radial: [
                            AlgebraicRootRepresentation::from_exact_value(&unit_x),
                            AlgebraicRootRepresentation::from_exact_value(&unit_y),
                        ],
                        signed_radius: self.radial_distance().clone(),
                    },
                ));
            }
            let representation = parameter_representation(self.cusp_parameter(), policy);
            if let Some(parameter) =
                hypersolve::compact_algebraic_root_low_degree_witness(&representation)
                    .and_then(|root| root.exact_point_witness().cloned())
            {
                let denominator = Real::eval_poly(&frame.data.denominator, &parameter);
                match real_sign(&denominator, &CurveContext::STRICT) {
                    Some(RealSign::Positive | RealSign::Negative) => {}
                    Some(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a compact selected-circle frame had zero homogeneous weight".into(),
                        ));
                    }
                    None => {
                        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                    }
                }
                let represented = |coefficients: &[Real]| {
                    Real::eval_poly(coefficients, &parameter) / &denominator
                };
                let center_x = represented(&center_x)?;
                let center_y = represented(&center_y)?;
                let unit_x = represented(&frame.data.normal_x_numerator)?;
                let unit_y = represented(&frame.data.normal_y_numerator)?;
                match real_sign(
                    &(Real::dot2_refs([&unit_x, &unit_y], [&unit_x, &unit_y]) - Real::one()),
                    &CurveContext::STRICT,
                ) {
                    Some(RealSign::Zero) => {}
                    Some(RealSign::Positive | RealSign::Negative) => {
                        return Err(CurveError::Topology(
                            "a compact selected-circle frame lost its unit radial".into(),
                        ));
                    }
                    None => {
                        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                    }
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "selected-circle-represented-frame",
                    "compact-low-degree-cusp",
                );
                return Ok(Classification::Decided(
                    BezierRepresentedSelectedRadialCircleFrame2 {
                        center: [
                            AlgebraicRootRepresentation::from_exact_value(&center_x),
                            AlgebraicRootRepresentation::from_exact_value(&center_y),
                        ],
                        unit_radial: [
                            AlgebraicRootRepresentation::from_exact_value(&unit_x),
                            AlgebraicRootRepresentation::from_exact_value(&unit_y),
                        ],
                        signed_radius: self.radial_distance().clone(),
                    },
                ));
            }
        }
        let center = match self.center_point_evidence(policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let start = match self.point_evidence_at(&Real::zero(), policy)? {
            Classification::Decided(start) => start,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center = match represented_point_evidence_coordinates(&center, policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let start = match represented_point_evidence_coordinates(&start, policy)? {
            Classification::Decided(start) => start,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let compact = |coordinate| {
            hypersolve::compact_algebraic_root_low_degree_witness(&coordinate).unwrap_or(coordinate)
        };
        let center = center.map(compact);
        let start = start.map(compact);
        let radial = [
            match represented_affine_coordinate(
                &[(&start[0], &Real::one()), (&center[0], &Real::from(-1_i8))],
                &Real::zero(),
            ) {
                Classification::Decided(radial) => radial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
            match represented_affine_coordinate(
                &[(&start[1], &Real::one()), (&center[1], &Real::from(-1_i8))],
                &Real::zero(),
            ) {
                Classification::Decided(radial) => radial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        ];
        let inverse_radius = (Real::one() / self.radial_distance())?;
        let unit_radial = radial.map(|radial| {
            represented_affine_coordinate(&[(&radial, &inverse_radius)], &Real::zero())
        });
        let [
            Classification::Decided(unit_x),
            Classification::Decided(unit_y),
        ] = unit_radial
        else {
            let reason = unit_radial
                .into_iter()
                .find_map(|coordinate| match coordinate {
                    Classification::Decided(_) => None,
                    Classification::Uncertain(reason) => Some(reason),
                })
                .unwrap_or(UncertaintyReason::Predicate);
            return Ok(Classification::Uncertain(reason));
        };
        Ok(Classification::Decided(
            BezierRepresentedSelectedRadialCircleFrame2 {
                center,
                unit_radial: [unit_x, unit_y],
                signed_radius: self.radial_distance().clone(),
            },
        ))
    }

    /// Reports whether the rational selected-circle frame has a compact
    /// represented form without eliminating a procedural point expression.
    ///
    /// The general circle/parallel system already owns exact incidence in the
    /// frame's selected parameter field.  Representation is only a fast path;
    /// attempting it for an analytic-parallel center can instead construct a
    /// high-degree tensor norm before falling back to that smaller authority.
    pub(in crate::bezier_offset) fn rational_circle_frame_is_compactly_representable(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<bool> {
        let Some(frame) = self.data.frame.rational() else {
            return Ok(false);
        };
        let (center_x, center_y) = self
            .data
            .frame
            .point_numerators_at_parallel_distance(&self.center_parallel_distance())?;
        if center_x.len() == 1
            && center_y.len() == 1
            && frame.data.normal_x_numerator.len() == 1
            && frame.data.normal_y_numerator.len() == 1
            && frame.data.denominator.len() == 1
        {
            return Ok(true);
        }
        let representation = parameter_representation(self.cusp_parameter(), policy);
        Ok(
            hypersolve::compact_algebraic_root_low_degree_witness(&representation)
                .and_then(|root| root.exact_point_witness().cloned())
                .is_some(),
        )
    }

    /// Collapses a recursively retained frame only when both its center and
    /// parameter-zero unit radial have exact point witnesses. A coincident
    /// rational carrier can then partition its sole parameter axis without
    /// projecting constant exact-`Real` coefficients. The general recursive
    /// tower remains authoritative when either coordinate has no materialized
    /// exact point.
    pub(in crate::bezier_offset) fn exact_point_component_frame(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<BezierRepresentedSelectedRadialCircleFrame2>> {
        let Some(center) = self.exact_center(policy)? else {
            return Ok(None);
        };
        let frame = match self.represented_circle_frame(policy)? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(_) => return Ok(None),
        };
        let [Some(unit_x), Some(unit_y)] = [
            frame.unit_radial[0].exact_point_witness(),
            frame.unit_radial[1].exact_point_witness(),
        ] else {
            return Ok(None);
        };
        Ok(Some(BezierRepresentedSelectedRadialCircleFrame2 {
            center: [
                AlgebraicRootRepresentation::from_exact_value(center.x()),
                AlgebraicRootRepresentation::from_exact_value(center.y()),
            ],
            unit_radial: [
                AlgebraicRootRepresentation::from_exact_value(unit_x),
                AlgebraicRootRepresentation::from_exact_value(unit_y),
            ],
            signed_radius: frame.signed_radius,
        }))
    }

    pub(in crate::bezier_offset) fn recursive_circle_frame_authority(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveCircleFrame2>>> {
        match &self.data.frame {
            BezierSelectedCircleFrame2::Rational(_) => {
                return self.recursive_rational_circle_frame_authority(policy);
            }
            BezierSelectedCircleFrame2::ParallelNormal(_) => {
                return self.recursive_parallel_normal_frame_authority(policy);
            }
            BezierSelectedCircleFrame2::ChordNormal(_) => {
                return self.recursive_chord_normal_frame_authority(policy);
            }
            BezierSelectedCircleFrame2::SelectedRadial(_) => {}
        }
        match self.recursive_selected_pair_frame_authority(policy)? {
            Classification::Decided(Some(authority)) => {
                Ok(Classification::Decided(Some(authority)))
            }
            Classification::Decided(None) => {
                match self.recursive_selected_radial_frame_authority(policy)? {
                    Classification::Decided(Some(authority)) => {
                        Ok(Classification::Decided(Some(authority)))
                    }
                    Classification::Decided(None) => {
                        self.recursive_selected_radial_evidence_frame_authority(policy)
                    }
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                }
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Imports the original parameter and positive source-speed radical,
    /// keeping the center and unit normal in one field. Reconstructing their
    /// Cartesian coordinates separately loses this correlation and can turn
    /// a tangent contact into a large independent-root elimination.
    pub(in crate::bezier_offset) fn recursive_parallel_normal_frame_authority(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveCircleFrame2>>> {
        let Some(frame) = self.data.frame.parallel_normal() else {
            return Ok(Classification::Decided(None));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a recursive parallel-normal circle frame crossed predicate policies".into(),
            ));
        }
        if let Classification::Uncertain(reason) = frame
            .center_support
            .certify_source_frame_at(&frame.center_parameter, &CurveContext::STRICT)?
        {
            return Ok(Classification::Uncertain(reason));
        }
        let center_parameter =
            match promote_curve_region_bezier_parameter(&frame.center_parameter, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        let source = frame.center_support.source_power_basis()?;
        let differential = frame.center_support.differential()?;
        let dense =
            |coefficients: &[Real]| DenseTensorPolynomial::from_axis_polynomial(1, 0, coefficients);
        let Some((field, center, support_center)) = (|| {
            let field = BezierRecursiveQuadraticField2::base(
                vec![bezier_parameter_root_representation(&center_parameter)],
                dense(&parallel_speed_squared_polynomial(differential))?,
                dense(&[Real::one()])?,
            )?;
            let BezierRecursiveQuadraticField2::Base(base) = &field else {
                unreachable!("a parallel-normal frame begins in its source field")
            };
            let value = |coefficients: &[Real]| {
                recursive_quadratic_rational_value(base, dense(coefficients)?)
            };
            let speed = recursive_quadratic_base_generator(base, true)?;
            let weight = value(source.weight.unwrap_or(&[Real::one()]))?;
            let denominator = weight.multiply(&speed)?;
            let normal_x = value(&differential.tangent_y)?
                .multiply(&weight)?
                .scale(&Real::from(-1_i8))?;
            let normal_y = value(&differential.tangent_x)?.multiply(&weight)?;
            let distance = frame.center_support.distance();
            // P=(X/W,Y/W), N=(-Ty,Tx)/sqrt(S), C=P+dN. Over the
            // common denominator W*sqrt(S), C-N supplies the unit radial
            // anchor even when d is zero or negative.
            let center_x = value(source.x_numerator)?
                .multiply(&speed)?
                .add(&normal_x.scale(distance)?)?;
            let center_y = value(source.y_numerator)?
                .multiply(&speed)?
                .add(&normal_y.scale(distance)?)?;
            let support_center = BezierRecursiveQuadraticProjectivePoint2 {
                x: center_x.subtract(&normal_x)?,
                y: center_y.subtract(&normal_y)?,
                denominator: denominator.clone(),
            };
            let center = BezierRecursiveQuadraticProjectivePoint2 {
                x: center_x,
                y: center_y,
                denominator,
            };
            Some((field, center, support_center))
        })() else {
            return Ok(Classification::Decided(None));
        };
        let center = match positive_recursive_projective_point(center)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let support_center = match positive_recursive_projective_point(support_center)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-circle-frame-authority",
            "parallel-normal-import",
        );
        Ok(Classification::Decided(Some(BezierRecursiveCircleFrame2 {
            field,
            center,
            support_center,
            normal_denominator: Real::one(),
        })))
    }

    /// Reuses the chord's normalized displacement authority for C-N. The
    /// center and direction keep their selected fields and positive speed;
    /// no Cartesian projection or second normalization kernel is needed.
    pub(in crate::bezier_offset) fn recursive_chord_normal_frame_authority(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveCircleFrame2>>> {
        let Some(frame) = self.data.frame.chord_normal() else {
            return Ok(Classification::Decided(None));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a recursive chord-normal circle frame crossed predicate policies".into(),
            ));
        }
        let support_center = frame.anchor.normal_displaced_point_evidence(
            frame.center.clone(),
            -Real::one(),
            policy,
        )?;
        let result = BezierRecursiveCircleFrame2::from_point_evidence(
            &frame.center,
            &support_center,
            Real::one(),
            policy,
        )?;
        #[cfg(feature = "dispatch-trace")]
        if matches!(&result, Classification::Decided(Some(_))) {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-circle-frame-authority",
                "chord-normal-import",
            );
        }
        Ok(result)
    }

    /// Recovers a selected-radial frame from its retained center and parent
    /// center point evidences.  This is the similarity-covariant fallback for
    /// a transformed recursive center whose original chord-map specialization
    /// no longer appears directly at the outer parameter node.
    pub(in crate::bezier_offset) fn recursive_selected_radial_evidence_frame_authority(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveCircleFrame2>>> {
        let Some(frame) = self.data.frame.selected_radial() else {
            return Ok(Classification::Decided(None));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a recursive selected-radial evidence frame crossed predicate policies".into(),
            ));
        }
        let support = frame.center_parameter.semicircle_carrier();
        let center_parameter =
            BezierAlgebraicCuspSemicircleParameter2::Mapped(frame.center_parameter.clone());
        let center = match center_parameter.coincident_point_evidence(support, policy)? {
            Classification::Decided(Some(center)) => center,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let support_center = match support.center_point_evidence(policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        BezierRecursiveCircleFrame2::from_point_evidence(
            &center,
            &support_center,
            frame.normal_denominator.clone(),
            policy,
        )
    }

    pub(in crate::bezier_offset) fn recursive_rational_circle_frame_authority_with_values(
        &self,
        field: BezierRecursiveQuadraticField2,
        mut value: impl FnMut(Vec<Real>) -> Option<BezierRecursiveQuadraticValue2>,
    ) -> CurveResult<Classification<Option<BezierRecursiveCircleFrame2>>> {
        let Some(frame) = self.data.frame.rational() else {
            return Ok(Classification::Decided(None));
        };
        let center_distance = frame.center_parallel_distance();
        let (center_x, center_y) = frame.point_numerators_at_parallel_distance(&center_distance);
        let Some((denominator, center_x, center_y, normal_x, normal_y)) = (|| {
            Some((
                value(frame.data.denominator.clone())?,
                value(center_x)?,
                value(center_y)?,
                value(frame.data.normal_x_numerator.clone())?,
                value(frame.data.normal_y_numerator.clone())?,
            ))
        })() else {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-circle-frame-authority-blocker",
                "rational-value",
            );
            return Ok(Classification::Decided(None));
        };
        let Some(support_center) = center_x
            .subtract(&normal_x)
            .zip(center_y.subtract(&normal_y))
            .map(|(x, y)| BezierRecursiveQuadraticProjectivePoint2 {
                x,
                y,
                denominator: denominator.clone(),
            })
        else {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-circle-frame-authority-blocker",
                "rational-support",
            );
            return Ok(Classification::Decided(None));
        };
        let center = BezierRecursiveQuadraticProjectivePoint2 {
            x: center_x,
            y: center_y,
            denominator,
        };
        let center = match positive_recursive_projective_point(center)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-circle-frame-authority-blocker",
                    "rational-center-sign",
                );
                return Ok(Classification::Uncertain(reason));
            }
        };
        let support_center = match positive_recursive_projective_point(support_center)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-circle-frame-authority-blocker",
                    "rational-support-sign",
                );
                return Ok(Classification::Uncertain(reason));
            }
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-circle-frame-authority",
            "rational-import",
        );
        Ok(Classification::Decided(Some(BezierRecursiveCircleFrame2 {
            field,
            center,
            support_center,
            normal_denominator: Real::one(),
        })))
    }

    /// Imports a one-field rational selected-circle frame into the recursive
    /// projective authority used by a deeper companion. The rational frame
    /// already stores its center and unit normal over one selected parameter;
    /// `C-N` is therefore a compact synthetic support point with normal
    /// denominator one. This avoids adding a separate rational/recursive
    /// circle-pair engine; the represented pair remains the fast path whenever
    /// it decides.
    pub(in crate::bezier_offset) fn recursive_rational_circle_frame_authority(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveCircleFrame2>>> {
        let Some(frame) = self.data.frame.rational() else {
            return Ok(Classification::Decided(None));
        };
        let source = parameter_representation(&frame.data.parameter, policy);
        let Some(field) = (|| {
            let one = DenseTensorPolynomial::try_new(vec![1], vec![Real::one()])?;
            BezierRecursiveQuadraticField2::base(vec![source], one.clone(), one)
        })() else {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-circle-frame-authority-blocker",
                "rational-base",
            );
            return Ok(Classification::Decided(None));
        };
        let BezierRecursiveQuadraticField2::Base(base) = &field else {
            unreachable!("a rational recursive circle frame begins in its dense base")
        };
        let base = base.clone();
        self.recursive_rational_circle_frame_authority_with_values(field, move |coefficients| {
            let coefficients = if coefficients.is_empty() {
                vec![Real::zero()]
            } else {
                coefficients
            };
            let dimension = coefficients.len();
            recursive_quadratic_rational_value(
                &base,
                DenseTensorPolynomial::try_new(vec![dimension], coefficients)?,
            )
        })
    }

    /// Embeds a rational circle frame directly in an existing recursive
    /// field when its selected parameter is constant or affine-related to one
    /// of that field's source axes. This preserves the known correlation and
    /// avoids projecting five coordinates through a foreign-field resultant.
    pub(in crate::bezier_offset) fn recursive_rational_circle_frame_authority_in_field(
        &self,
        field: &BezierRecursiveQuadraticField2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveCircleFrame2>>> {
        let Some(frame) = self.data.frame.rational() else {
            return Ok(Classification::Decided(None));
        };
        let source = parameter_representation(&frame.data.parameter, policy);
        let (base, _) = field.base_and_extension_path();
        let exact = source.exact_point_witness().cloned();
        let relation = if exact.is_none() {
            base.sources
                .iter()
                .enumerate()
                .find_map(|(axis, candidate)| {
                    let relation = if candidate == &source {
                        hypersolve::AlgebraicRootAffineRelation {
                            scale: Real::one(),
                            offset: Real::zero(),
                        }
                    } else if let Some(relation) =
                        algebraic_root_affine_relation(candidate, &source)
                    {
                        relation
                    } else if represented_roots_strictly_equal(candidate, &source) {
                        hypersolve::AlgebraicRootAffineRelation {
                            scale: Real::one(),
                            offset: Real::zero(),
                        }
                    } else {
                        return None;
                    };
                    Some((axis, relation.scale, relation.offset))
                })
        } else {
            None
        };
        if exact.is_none() && relation.is_none() {
            return Ok(Classification::Decided(None));
        }
        let target = field.clone();
        let target_value = target.clone();
        let result = self.recursive_rational_circle_frame_authority_with_values(
            target,
            move |coefficients| {
                if let Some(parameter) = exact.as_ref() {
                    return target_value.constant(Real::eval_poly(&coefficients, parameter));
                }
                let (axis, scale, offset) = relation.as_ref()?;
                let linear = [offset.clone(), scale.clone()];
                let mut composed = vec![Real::zero()];
                for coefficient in coefficients.iter().rev() {
                    composed = polynomial_multiply(&composed, &linear);
                    composed[0] += coefficient;
                }
                let polynomial = DenseTensorPolynomial::from_axis_polynomial(
                    base.sources.len(),
                    *axis,
                    &polynomial_trim_structural_zeros(composed),
                )?;
                let value = recursive_quadratic_rational_value(&base, polynomial)?;
                target_value.lift(&value)
            },
        )?;
        if matches!(result, Classification::Decided(Some(_))) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-circle-pair-frame-join",
                "rational-to-companion-field",
            );
        }
        Ok(result)
    }
}
