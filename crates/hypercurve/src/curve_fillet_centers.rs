//! Exact fillet sources, prepared carriers, offset centers and fillet cuts.

use super::*;

impl FilletLinearSource2<'_> {
    pub(super) const fn native_line(&self) -> Option<&LineSeg2> {
        match self {
            Self::Native { source, .. } => Some(source),
            Self::AlgebraicChord(_) => None,
        }
    }

    pub(super) const fn algebraic_chord(&self) -> Option<&crate::BezierAlgebraicChord2> {
        match self {
            Self::Native { .. } => None,
            Self::AlgebraicChord(source) => Some(source),
        }
    }

    pub(super) fn parallel_tangent_contacts(
        &self,
    ) -> &[crate::bezier::BezierParallelLineTangentContact2] {
        match self {
            Self::Native {
                parallel_tangent_contacts,
                ..
            } => parallel_tangent_contacts,
            Self::AlgebraicChord(source) => source.parallel_tangent_contacts(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) enum FilletParallelSource2<'a> {
    Direct(ExactCornerBezier2<'a>),
    Retained(&'a crate::BezierParallelFragment2),
    Selected(&'a crate::bezier_split::BezierSelectedFiberFragment2),
}

impl<'a> FilletParallelSource2<'a> {
    pub(super) fn corner_carrier(self) -> ExactCornerCarrier2<'a> {
        match self {
            Self::Direct(ExactCornerBezier2::Direct(source)) => ExactCornerCarrier2::Bezier(source),
            Self::Direct(ExactCornerBezier2::NativeSpan(source)) => {
                ExactCornerCarrier2::NativeBezierSpan(source)
            }
            Self::Retained(source) => ExactCornerCarrier2::AnalyticParallel(source),
            Self::Selected(source) => ExactCornerCarrier2::SelectedFiber(source),
        }
    }

    pub(super) const fn is_reversed(self) -> bool {
        match self {
            Self::Direct(_) => false,
            Self::Retained(source) => source.is_reversed(),
            Self::Selected(source) => source.is_reversed(),
        }
    }

    pub(super) const fn retained(&self) -> Option<&crate::BezierParallelFragment2> {
        match self {
            Self::Direct(_) => None,
            Self::Retained(source) => Some(source),
            Self::Selected(_) => None,
        }
    }

    pub(super) fn parameter_range(&self) -> Option<BezierParameterRange2> {
        match self {
            Self::Direct(_) => Some(BezierParameterRange2::new_validated(
                BezierParameter2::Exact(Real::zero()),
                BezierParameter2::Exact(Real::one()),
            )),
            Self::Retained(source) => Some(source.range().clone()),
            Self::Selected(_) => None,
        }
    }

    pub(super) fn curve_parameter_range(&self) -> crate::CurveParameterRange2 {
        match self {
            Self::Direct(_) => crate::CurveParameterRange2::from_bezier_range(
                BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                ),
            ),
            Self::Retained(source) => {
                crate::CurveParameterRange2::from_bezier_range(source.range().clone())
            }
            Self::Selected(source) => source.range().clone(),
        }
    }

    /// Returns a finite rational envelope for intersection enumeration.
    ///
    /// Selected endpoints remain authoritative for final admissibility. Their
    /// exact isolating bounds merely enlarge the solve interval, so no bound
    /// can become a construction parameter or discard an authored contact.
    pub(super) fn intersection_parameter_range(
        &self,
        family: CurveFamily2,
    ) -> ExactCurveResult<BezierParameterRange2> {
        if let Some(range) = self.parameter_range() {
            return Ok(range);
        }
        let range = self.curve_parameter_range();
        let (Some((start, _)), Some((_, end))) = (
            range.start().finite_envelope_bounds(),
            range.end().finite_envelope_bounds(),
        ) else {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                crate::UncertaintyReason::Unsupported,
            ));
        };
        Ok(BezierParameterRange2::new_validated(
            BezierParameter2::Exact(start.clone()),
            BezierParameter2::Exact(end.clone()),
        ))
    }

    pub(super) fn incident_domain(
        &self,
        support: &BezierParallel2,
        previous: bool,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<crate::bezier_offset::BezierParallelIncidentDomain2> {
        let source_reversed = match self {
            Self::Direct(_) => false,
            Self::Retained(source) => source.is_reversed(),
            Self::Selected(source) => source.is_reversed(),
        };
        let extends_toward_higher_parameter = previous != source_reversed;
        let direction = if extends_toward_higher_parameter {
            crate::BezierParameterRayDirection2::Increasing
        } else {
            crate::BezierParameterRayDirection2::Decreasing
        };
        let range = self.curve_parameter_range();
        let endpoint = if extends_toward_higher_parameter {
            range.end()
        } else {
            range.start()
        };
        let domain = support.incident_domain_from_parameter(endpoint, direction, policy);
        match domain
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
        {
            Classification::Decided(domain) => Ok(domain),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                reason,
            )),
        }
    }

    pub(super) fn parameter_placement(
        &self,
        parameter: &CurveParameter2,
        previous: bool,
        domain: FilletContactDomain2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<CornerPlacement2>> {
        let mode = domain.mode();
        let placement = match self {
            Self::Direct(_) => curve_region_corner_parameter_placement(
                parameter,
                previous,
                mode,
                CurveOperation2::Fillet,
                family,
                policy,
            )?,
            Self::Retained(source) => retained_parallel_corner_parameter_placement(
                parameter,
                source,
                previous,
                mode,
                CurveOperation2::Fillet,
                family,
                policy,
            )?,
            Self::Selected(source) => selected_fiber_corner_parameter_placement(
                parameter,
                source,
                previous,
                mode,
                CurveOperation2::Fillet,
                family,
                policy,
            )?,
        };
        domain.with_boundary_contact(
            placement,
            parameter,
            || {
                let reversed = match self {
                    Self::Direct(_) => false,
                    Self::Retained(source) => source.is_reversed(),
                    Self::Selected(source) => source.is_reversed(),
                };
                let range = self.curve_parameter_range();
                if previous != reversed {
                    range.end().clone()
                } else {
                    range.start().clone()
                }
            },
            family,
            policy,
        )
    }

    pub(super) fn parameter_is_admissible(
        &self,
        parameter: &CurveParameter2,
        previous: bool,
        domain: FilletContactDomain2,
        incident_domain: Option<&crate::bezier_offset::BezierParallelIncidentDomain2>,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        let placement = self.parameter_placement(parameter, previous, domain, family, policy)?;
        match placement {
            Some(CornerPlacement2::Trim | CornerPlacement2::Corner) => Ok(true),
            Some(CornerPlacement2::Extension) => {
                let domain = incident_domain.ok_or_else(|| {
                    ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        crate::UncertaintyReason::Unsupported,
                    )
                })?;
                match domain
                    .contains_extension_parameter(parameter, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })? {
                    Classification::Decided(inside) => Ok(inside),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        reason,
                    )),
                }
            }
            None => Ok(false),
        }
    }

    /// The curve segment retained by this corner determines the tangent side
    /// at an original-offset cusp, including contacts beyond the authored end.
    pub(super) fn retained_contact_side(
        &self,
        previous: bool,
    ) -> crate::BezierParameterRayDirection2 {
        if previous != self.is_reversed() {
            crate::BezierParameterRayDirection2::Decreasing
        } else {
            crate::BezierParameterRayDirection2::Increasing
        }
    }

    pub(super) fn support_reverses_source_at(
        &self,
        support: &BezierParallel2,
        parameter: &CurveParameter2,
        previous: bool,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<bool>> {
        let derivative_scale = |parallel: &BezierParallel2, original: bool| {
            let result = if original {
                parallel.parallel_derivative_scale_sign_at_side(
                    parameter,
                    self.retained_contact_side(previous),
                    policy,
                )
            } else {
                parallel.parallel_derivative_scale_sign(parameter, policy)
            };
            match result
                .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
            {
                Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => {
                    Ok(Some(sign))
                }
                Classification::Decided(RealSign::Zero) => Ok(None),
                Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    family,
                    reason,
                )),
            }
        };
        let (source_scale, reversed) = match self {
            Self::Direct(_) => (Some(RealSign::Positive), false),
            Self::Retained(source) => (
                derivative_scale(source.parallel(), true)?,
                source.is_reversed(),
            ),
            Self::Selected(source) => (
                derivative_scale(&source.parallel_carrier(), true)?,
                source.is_reversed(),
            ),
        };
        // Compare the surviving original tangent with the pointwise center
        // derivative. A center-locus cusp still supplies no tangent evidence;
        // it is not a singularity of the original contact curve.
        Ok(source_scale
            .zip(derivative_scale(support, false)?)
            .map(|(source, center)| (source != center) != reversed))
    }

    pub(super) fn parallel_distance(&self) -> Real {
        match self {
            Self::Direct(_) => Real::zero(),
            Self::Retained(source) => source.parallel().distance().clone(),
            Self::Selected(source) => source.parallel_carrier().distance().clone(),
        }
    }
}

/// Authored trim boundaries are open. An internal partition belongs to the
/// chart that survives the cut, including its one-sided tangent. The complete
/// authored domain places that contact after transport to the source chart.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum FilletContactDomain2 {
    AuthoredCurve(CurveCornerMode2),
    SourceChart(CurveCornerMode2),
}

impl FilletContactDomain2 {
    pub(crate) const fn mode(self) -> CurveCornerMode2 {
        match self {
            Self::AuthoredCurve(mode) | Self::SourceChart(mode) => mode,
        }
    }

    pub(super) fn with_boundary_contact(
        self,
        placement: Option<CornerPlacement2>,
        parameter: &CurveParameter2,
        endpoint: impl FnOnce() -> CurveParameter2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<CornerPlacement2>> {
        if matches!(self, Self::AuthoredCurve(_)) || placement.is_some() {
            return Ok(placement);
        }
        match parameter
            .cmp_by_refinement(&endpoint(), policy)
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
        {
            Classification::Decided(order) => Ok(order.is_eq().then_some(CornerPlacement2::Trim)),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                reason,
            )),
        }
    }
}

pub(super) enum PreparedFilletCarrier2<'a> {
    Line {
        source: FilletLinearSource2<'a>,
        /// Only retained algebraic chords need an owned represented support;
        /// native line sources are already their own support.
        chord_support: Option<LineSeg2>,
        unit_x: Real,
        unit_y: Real,
    },
    Arc {
        source: ExactCornerArc2,
        radius: Real,
    },
    AlgebraicCusp {
        source: &'a crate::BezierAlgebraicCuspSemicircleFragment2,
    },
    AlgebraicChord {
        source: &'a crate::BezierAlgebraicChord2,
    },
    Parallel {
        source: FilletParallelSource2<'a>,
        parallel: BezierParallel2,
        /// Possible traversal directions relative to the increasing source
        /// parameter. Each selected contact certifies its actual direction.
        directions: [Option<RealSign>; 2],
    },
}

impl<'a> PreparedFilletCarrier2<'a> {
    pub(super) fn new(
        carrier: ExactCornerCarrier2<'a>,
        family: CurveFamily2,
        domain: FilletContactDomain2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        match carrier {
            ExactCornerCarrier2::Line(source) => {
                let (dx, dy) = source.delta();
                let (unit_x, unit_y, _) =
                    line_unit_direction(&dx, &dy, CurveOperation2::Fillet, family, policy)?;
                Ok(Self::Line {
                    source: FilletLinearSource2::Native {
                        source,
                        parameterization: None,
                        parallel_tangent_contacts: &[],
                    },
                    chord_support: None,
                    unit_x,
                    unit_y,
                })
            }
            ExactCornerCarrier2::PromotedLine(curve) => {
                let source = curve
                    .retained_exact_line_image()
                    .expect("a promoted-line carrier retains its exact line image");
                let (dx, dy) = source.delta();
                let (unit_x, unit_y, _) =
                    line_unit_direction(&dx, &dy, CurveOperation2::Fillet, family, policy)?;
                Ok(Self::Line {
                    source: FilletLinearSource2::Native {
                        source,
                        parameterization: Some(curve),
                        parallel_tangent_contacts: curve.retained_parallel_line_tangent_contacts(),
                    },
                    chord_support: None,
                    unit_x,
                    unit_y,
                })
            }
            ExactCornerCarrier2::Arc(source) => {
                let radius =
                    exact_corner_arc_radius(source, CurveOperation2::Fillet, family, policy)?;
                Ok(Self::Arc {
                    source: ExactCornerArc2::Native(source.clone()),
                    radius,
                })
            }
            ExactCornerCarrier2::RetainedRationalArc(source) => {
                let source = ExactCornerArc2::RetainedRational(source);
                let radius = exact_corner_arc_radius(
                    source.support(),
                    CurveOperation2::Fillet,
                    family,
                    policy,
                )?;
                Ok(Self::Arc { source, radius })
            }
            ExactCornerCarrier2::Bezier(source) => {
                let source = ExactCornerBezier2::Direct(source);
                let parallel = exact_corner_bezier_parallel(
                    source,
                    Real::zero(),
                    CurveOperation2::Fillet,
                    family,
                )?;
                Ok(Self::parallel(
                    FilletParallelSource2::Direct(source),
                    parallel,
                    domain,
                    policy,
                ))
            }
            ExactCornerCarrier2::NativeBezierSpan(fragment) => {
                let source = ExactCornerBezier2::NativeSpan(fragment);
                let parallel = exact_corner_bezier_parallel(
                    source,
                    Real::zero(),
                    CurveOperation2::Fillet,
                    family,
                )?;
                Ok(Self::parallel(
                    FilletParallelSource2::Direct(source),
                    parallel,
                    domain,
                    policy,
                ))
            }
            ExactCornerCarrier2::AlgebraicCusp(source) => Ok(Self::AlgebraicCusp { source }),
            ExactCornerCarrier2::AlgebraicChord(source) => {
                let canonical_axis_support = if let Some(direction) =
                    source.certified_axis_direction()
                    && let Some(coordinate) = source
                        .constant_axis_coordinate(
                            match direction.axis() {
                                crate::Axis2::X => crate::Axis2::Y,
                                crate::Axis2::Y => crate::Axis2::X,
                            },
                            policy,
                        )
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                        })? {
                    let (unit_x, unit_y) = direction.unit_tangent();
                    let start = match direction.axis() {
                        crate::Axis2::X => Point2::new(Real::zero(), coordinate),
                        crate::Axis2::Y => Point2::new(coordinate, Real::zero()),
                    };
                    Some(LineSeg2::new_unchecked(
                        start.clone(),
                        start.translated(unit_x, unit_y),
                    ))
                } else {
                    None
                };
                let Some(support) = canonical_axis_support
                    .or_else(|| source.exact_line())
                    .or_else(|| source.strict_provenance_support_line(policy))
                else {
                    return Ok(Self::AlgebraicChord { source });
                };
                let (unit_x, unit_y) = if let Some(unit) = source.certified_unit_tangent() {
                    unit
                } else {
                    let (dx, dy) = support.delta();
                    let (unit_x, unit_y, _) =
                        line_unit_direction(&dx, &dy, CurveOperation2::Fillet, family, policy)?;
                    (unit_x, unit_y)
                };
                Ok(Self::Line {
                    source: FilletLinearSource2::AlgebraicChord(source),
                    chord_support: Some(support),
                    unit_x,
                    unit_y,
                })
            }
            ExactCornerCarrier2::AnalyticParallel(source) => Ok(Self::parallel(
                FilletParallelSource2::Retained(source),
                source.parallel().clone(),
                domain,
                policy,
            )),
            ExactCornerCarrier2::SelectedFiber(source) => Ok(Self::parallel(
                FilletParallelSource2::Selected(source),
                source.parallel_carrier(),
                domain,
                policy,
            )),
        }
    }

    pub(super) fn parallel(
        source: FilletParallelSource2<'a>,
        parallel: BezierParallel2,
        domain: FilletContactDomain2,
        policy: &CurveContext,
    ) -> Self {
        let mut directions = [Some(RealSign::Positive), Some(RealSign::Negative)];
        if parallel.distance() == &Real::zero() || parallel.has_exact_affine_line_parameterization()
        {
            directions[1] = None;
        } else if domain.mode() == CurveCornerMode2::TrimOnly {
            // A single normal sheet is an optional scheduling certificate.
            // Otherwise both d+r and d-r remain available and every contact
            // replays its own orientation. Extension rays may cross old cusps.
            let constant = policy.bounded_exact_predicate_pass(
                || -> crate::CurveResult<Classification<Option<RealSign>>> {
                    let analysis = match parallel
                        .singularity_analysis_with_policy(&source.curve_parameter_range(), policy)?
                    {
                        Classification::Decided(analysis) => analysis,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let ranges = match analysis.regular_subranges(policy)? {
                        Classification::Decided(ranges) => ranges,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let mut common = None;
                    for range in ranges {
                        let parameter = match range.strict_interior_scalar(policy)? {
                            Classification::Decided(parameter) => parameter,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        let sign = match parallel
                            .parallel_derivative_scale_sign(&parameter.into(), policy)?
                        {
                            Classification::Decided(RealSign::Zero) => {
                                return Ok(Classification::Decided(None));
                            }
                            Classification::Decided(sign) => sign,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        if common.is_some_and(|previous| previous != sign) {
                            return Ok(Classification::Decided(None));
                        }
                        common = Some(sign);
                    }
                    Ok(Classification::Decided(common))
                },
            );
            if let Ok(Classification::Decided(Some(sign))) = constant {
                directions = [Some(sign), None];
            }
        }
        if source.is_reversed() {
            directions = directions.map(|sign| sign.map(reverse_fillet_sign));
        }
        Self::Parallel {
            source,
            parallel,
            directions,
        }
    }

    pub(super) fn component_normal_constraint(
        &self,
        offset: &FilletOffsetCarrier2<'_, '_>,
        signed_distance: &Real,
        axis: hypersolve::CurveResultantParameter,
        family: CurveFamily2,
    ) -> ExactCurveResult<Option<crate::bezier_offset::BezierParallelDerivativeConstraint2>> {
        let Self::Parallel {
            source, parallel, ..
        } = self
        else {
            return Ok(None);
        };
        let FilletOffsetCarrier2::Parallel { support, .. } = offset else {
            unreachable!("a prepared parallel produces parallel center supports")
        };
        let forward = support.distance() == &(parallel.distance() + signed_distance);
        let expected = if forward != source.is_reversed() {
            RealSign::Positive
        } else {
            RealSign::Negative
        };
        if !forward && support.distance() != &(parallel.distance() - signed_distance) {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Fillet,
                family,
                CurveError::Topology(
                    "a fillet center support lost its signed-distance construction".into(),
                ),
            ));
        }
        Ok(Some(parallel.derivative_scale_constraint(
            axis,
            expected,
            Some(source.retained_contact_side(axis == hypersolve::CurveResultantParameter::First)),
        )))
    }

    pub(super) fn accepts_offset_contact(
        &self,
        offset: &FilletOffsetCarrier2<'_, '_>,
        parameter: Option<&CurveParameter2>,
        previous: bool,
        signed_distance: &Real,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        let Self::Parallel {
            source, parallel, ..
        } = self
        else {
            return Ok(true);
        };
        let parameter = parameter.expect("a parallel center retains its source parameter");
        let sign = match parallel
            .parallel_derivative_scale_sign_at_side(
                parameter,
                source.retained_contact_side(previous),
                policy,
            )
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
        {
            Classification::Decided(RealSign::Zero) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    family,
                    crate::UncertaintyReason::Boundary,
                ));
            }
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    family,
                    reason,
                ));
            }
        };
        Ok(self.accepts_offset_direction(offset, sign, signed_distance))
    }

    pub(super) fn accepts_offset_direction(
        &self,
        offset: &FilletOffsetCarrier2<'_, '_>,
        source_sign: RealSign,
        signed_distance: &Real,
    ) -> bool {
        let Self::Parallel {
            source, parallel, ..
        } = self
        else {
            return true;
        };
        let FilletOffsetCarrier2::Parallel { support, .. } = offset else {
            unreachable!("a prepared parallel produces parallel center supports")
        };
        let distance = if (source_sign == RealSign::Positive) != source.is_reversed() {
            parallel.distance() + signed_distance
        } else {
            parallel.distance() - signed_distance
        };
        // Both alternatives were constructed from these same scalar operands.
        // This selects their construction identity; it does not reconstruct a
        // field or infer geometric inequality from unrelated Real expressions.
        support.distance() == &distance
    }

    /// The caller has certified that the entire center support is one point.
    /// On each regular source cell, 1 - d_center * curvature = 0, so the
    /// original parallel scale has the sign of (d_center-d_source)*d_center.
    /// This validates its normal branch without choosing a contact sample.
    pub(super) fn accepts_collapsed_offset(
        &self,
        offset: &FilletOffsetCarrier2<'_, '_>,
        signed_distance: &Real,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        let Self::Parallel { parallel, .. } = self else {
            return Ok(true);
        };
        let FilletOffsetCarrier2::Parallel { support, .. } = offset else {
            unreachable!("a collapsed parallel center retains its support")
        };
        let scale_sign = match crate::classify::real_sign(
            &((support.distance() - parallel.distance()) * support.distance()),
            policy,
        ) {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Some(RealSign::Zero) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    family,
                    crate::UncertaintyReason::Boundary,
                ));
            }
            None => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    family,
                    crate::UncertaintyReason::RealSign,
                ));
            }
        };
        Ok(self.accepts_offset_direction(offset, scale_sign, signed_distance))
    }

    pub(super) fn offsets<'b>(
        &'b self,
        signed_distance: &Real,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<[Option<FilletOffsetCarrier2<'a, 'b>>; 2]> {
        let offset = match self {
            Self::Line {
                source,
                chord_support,
                unit_x,
                unit_y,
            } => {
                let source_support = source
                    .native_line()
                    .or(chord_support.as_ref())
                    .expect("a prepared linear fillet carrier retains one support");
                let offset_x = -unit_y * signed_distance;
                let offset_y = unit_x * signed_distance;
                // Translation preserves the already-validated nonzero source
                // direction, so rebuilding an endpoint-distance proof would
                // only allocate an algebraically identical norm.
                let support = LineSeg2::new_unchecked(
                    source_support
                        .start()
                        .translated(offset_x.clone(), offset_y.clone()),
                    source_support.end().translated(offset_x, offset_y),
                );
                Ok(FilletOffsetCarrier2::Line {
                    source: *source,
                    support,
                    unit_x,
                    unit_y,
                    signed_distance: signed_distance.clone(),
                })
            }
            Self::Arc { source, radius } => {
                let support = source.support();
                let signed_radius = if support.is_clockwise() {
                    radius + signed_distance
                } else {
                    radius - signed_distance
                };
                match crate::classify::real_sign(&signed_radius, policy) {
                    Some(RealSign::Zero) => Ok(FilletOffsetCarrier2::Point {
                        source: self,
                        point: CurvePoint2::from(support.center().clone()),
                    }),
                    Some(RealSign::Positive | RealSign::Negative) => {
                        Ok(FilletOffsetCarrier2::Arc {
                            source,
                            source_radius: radius,
                            signed_radius,
                        })
                    }
                    None => Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        crate::UncertaintyReason::RealSign,
                    )),
                }
            }
            Self::AlgebraicCusp { source } => {
                let support =
                    match source
                        .offset_left(signed_distance, policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                        })? {
                        Classification::Decided(Some(support)) => support,
                        Classification::Decided(None) => {
                            let point = match source
                                .semicircle()
                                .center_point_evidence(policy)
                                .map_err(|cause| {
                                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                                })? {
                                Classification::Decided(point) => point,
                                Classification::Uncertain(reason) => {
                                    return Err(ExactCurveError::blocked(
                                        CurveOperation2::Fillet,
                                        family,
                                        reason,
                                    ));
                                }
                            };
                            return Ok([
                                Some(FilletOffsetCarrier2::Point {
                                    source: self,
                                    point,
                                }),
                                None,
                            ]);
                        }
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                family,
                                reason,
                            ));
                        }
                    };
                Ok(FilletOffsetCarrier2::AlgebraicCusp { source, support })
            }
            Self::AlgebraicChord { source } => {
                let support = source
                    .parallel_left_retained(signed_distance.clone(), policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })?;
                Ok(FilletOffsetCarrier2::AlgebraicChord {
                    source,
                    support,
                    signed_distance: signed_distance.clone(),
                    finite_source_domain: true,
                })
            }
            Self::Parallel {
                source,
                parallel,
                directions,
            } => {
                return Ok(directions.map(|direction| {
                    direction.map(|direction| {
                        let distance = if direction == RealSign::Positive {
                            parallel.distance() + signed_distance
                        } else {
                            parallel.distance() - signed_distance
                        };
                        FilletOffsetCarrier2::Parallel {
                            source: *source,
                            support: parallel.with_distance(distance),
                        }
                    })
                }));
            }
        }?;
        Ok([Some(offset), None])
    }
}

pub(super) enum FilletOffsetCarrier2<'a, 'b> {
    Line {
        source: FilletLinearSource2<'a>,
        support: LineSeg2,
        unit_x: &'b Real,
        unit_y: &'b Real,
        signed_distance: Real,
    },
    Arc {
        source: &'b ExactCornerArc2,
        source_radius: &'b Real,
        signed_radius: Real,
    },
    Point {
        point: CurvePoint2,
        source: &'b PreparedFilletCarrier2<'a>,
    },
    Parallel {
        source: FilletParallelSource2<'a>,
        support: BezierParallel2,
    },
    AlgebraicCusp {
        source: &'a crate::BezierAlgebraicCuspSemicircleFragment2,
        support: crate::BezierAlgebraicCuspSemicircleFragment2,
    },
    AlgebraicChord {
        source: &'a crate::BezierAlgebraicChord2,
        support: crate::BezierAlgebraicChord2,
        signed_distance: Real,
        /// Whether the support witness endpoints are the authored finite
        /// domain. A canonical line witness can name only the infinite
        /// support; its source chord owns final trim/extension classification.
        finite_source_domain: bool,
    },
}

impl FilletOffsetCarrier2<'_, '_> {
    pub(super) fn retained_fillet_frame(
        &self,
        anchor_is_previous: bool,
        anchor_parameter: Option<&CurveParameter2>,
        mut anchor_evidence: Option<RetainedFilletAnchorEvidence2>,
        force_chord_normal: bool,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<RetainedFilletFrame2>> {
        let (radial_frame, radial_distance) = match self {
            Self::Line {
                source,
                support,
                unit_x,
                unit_y,
                signed_distance,
                ..
            } => {
                let radial_frame = if force_chord_normal {
                    let anchor = if let Some(anchor) = source.algebraic_chord() {
                        anchor.clone()
                    } else {
                        algebraic_chord_from_line_support(
                            support,
                            CurveOperation2::Fillet,
                            family,
                            policy,
                        )?
                    };
                    #[cfg(feature = "dispatch-trace")]
                    if let Some((anchor_x, anchor_y)) = anchor.certified_unit_tangent() {
                        let dot = &anchor_x * *unit_x + &anchor_y * *unit_y;
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "curve-region-fillet-chord-frame-orientation",
                            match crate::classify::real_sign(&dot, policy) {
                                Some(RealSign::Positive) => "agrees-with-line",
                                Some(RealSign::Negative) => "reverses-line",
                                Some(RealSign::Zero) => "orthogonal-to-line",
                                None => "uncertain",
                            },
                        );
                    }
                    RetainedFilletRadialFrame2::ChordNormal {
                        anchor,
                        policy: *policy,
                    }
                } else if let Some(center_frame) = anchor_evidence
                    .as_ref()
                    .and_then(|evidence| evidence.center_parallel.clone())
                {
                    let Some(center_parameter) = center_frame
                        .parameter
                        .as_ref()
                        .cloned()
                        .or_else(|| anchor_parameter.cloned())
                    else {
                        return Ok(None);
                    };
                    RetainedFilletRadialFrame2::ParallelNormal {
                        center_support: center_frame.support,
                        center_parameter,
                        policy: *policy,
                    }
                } else {
                    RetainedFilletRadialFrame2::RepresentedUnitNormal((
                        -(*unit_y).clone(),
                        (*unit_x).clone(),
                    ))
                };
                (radial_frame, -signed_distance.clone())
            }
            Self::Arc {
                source,
                source_radius,
                signed_radius,
            } => {
                let support = source.support();
                let (normal_denominator, radial_distance) = if support.is_clockwise() {
                    (signed_radius.clone(), *source_radius - signed_radius)
                } else {
                    (-signed_radius.clone(), signed_radius - *source_radius)
                };
                let signed_radius_sign =
                    crate::classify::real_sign(signed_radius, &CurveContext::STRICT);
                let selected_center = anchor_evidence
                    .as_mut()
                    .and_then(|evidence| evidence.deferred_arc_contact.as_mut())
                    .and_then(|deferred| deferred.selected_center.take());
                let radial_frame = if let Some(selected_center) = selected_center.as_ref() {
                    RetainedFilletRadialFrame2::SelectedConcentric {
                        support: selected_center
                            .mapped_semicircle_carrier()
                            .expect("a retained selected arc center is mapped")
                            .clone(),
                        center_parameter: selected_center.clone(),
                        normal_denominator: normal_denominator.clone(),
                    }
                } else if let (Some(center_frame), Some(center_parameter)) = (
                    anchor_evidence
                        .as_ref()
                        .and_then(|evidence| evidence.center_parallel.as_ref()),
                    anchor_parameter.filter(|parameter| parameter.is_retained_scalar()),
                ) {
                    let anchor = match crate::BezierAlgebraicChord2::from_certified_retained_parallel_unit_tangent(
                        center_frame.support.clone(),
                        center_parameter,
                        policy,
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })? {
                        Classification::Decided(anchor) => anchor,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                family,
                                reason,
                            ));
                        }
                    };
                    // Positive concentric scaling preserves the offset cell's
                    // left normal. Past-center scaling reverses it. Never
                    // choose an orientation when that nonzero sign was not
                    // proved by the center construction.
                    let source_direction = anchor_evidence
                        .as_ref()
                        .and_then(|evidence| evidence.source_direction)
                        .or(signed_radius_sign);
                    let anchor = match source_direction {
                        Some(RealSign::Positive) => anchor,
                        Some(RealSign::Negative) => anchor.reversed(),
                        Some(RealSign::Zero) => {
                            return Err(ExactCurveError::invalid(
                                CurveOperation2::Fillet,
                                family,
                                CurveError::Topology(
                                    "a retained arc fillet frame had zero center radius".into(),
                                ),
                            ));
                        }
                        None => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                family,
                                crate::UncertaintyReason::RealSign,
                            ));
                        }
                    };
                    RetainedFilletRadialFrame2::ChordNormal {
                        anchor,
                        policy: *policy,
                    }
                } else if signed_radius_sign == Some(RealSign::Positive) {
                    match (
                        anchor_evidence
                            .as_ref()
                            .and_then(|evidence| evidence.center_parallel.as_ref())
                            .map(|frame| frame.support.clone()),
                        anchor_parameter.cloned(),
                    ) {
                        (Some(center_support), Some(center_parameter)) => {
                            RetainedFilletRadialFrame2::ParallelNormal {
                                center_support,
                                center_parameter,
                                policy: *policy,
                            }
                        }
                        _ => RetainedFilletRadialFrame2::ConcentricArc {
                            support_center: support.center().clone(),
                            normal_denominator,
                        },
                    }
                } else {
                    // A past-center concentric image reverses its rational
                    // tangent. Keep the orientation-independent radial
                    // frame unless that reversal is proved and retained by
                    // a future mapped-contact authority.
                    RetainedFilletRadialFrame2::ConcentricArc {
                        support_center: support.center().clone(),
                        normal_denominator,
                    }
                };
                (radial_frame, radial_distance)
            }
            Self::AlgebraicCusp { source, support } => {
                let complementary =
                    anchor_parameter.is_some_and(CurveParameter2::is_algebraic_cusp_complement);
                let support_circle = if complementary {
                    support.semicircle().complementary_half()
                } else {
                    support.semicircle().clone()
                };
                let source_circle = if complementary {
                    source.semicircle().complementary_half()
                } else {
                    source.semicircle().clone()
                };
                let center =
                    match support_circle
                        .center_point_evidence(policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                        })? {
                        Classification::Decided(center) => center,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                family,
                                reason,
                            ));
                        }
                    };
                let support_radius = support_circle.radial_distance();
                let source_radius = source_circle.radial_distance();
                let clockwise = support_circle.is_clockwise() != support.is_reversed();
                let (normal_denominator, radial_distance) = if clockwise {
                    (support_radius.clone(), source_radius - support_radius)
                } else {
                    (-support_radius.clone(), support_radius - source_radius)
                };
                let support_center = match &center {
                    CurvePoint2(CurvePointData2::Exact(point)) => Some(point.clone()),
                    CurvePoint2(CurvePointData2::Algebraic(image)) => {
                        image.exact_point(&CurveContext::STRICT)
                    }
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                    | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
                    | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
                    | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                    | CurvePoint2(CurvePointData2::AnalyticParallel(_))
                    | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                        None
                    }
                };
                let selected_center_parameter = anchor_parameter
                    .and_then(CurveParameter2::as_algebraic_cusp)
                    .cloned();
                let retain_pair_frame = selected_center_parameter
                    .as_ref()
                    .is_some_and(|parameter| parameter.retains_pair_contact());
                let radial_frame =
                    if let Some(support_center) = support_center.filter(|_| !retain_pair_frame) {
                        RetainedFilletRadialFrame2::ConcentricArc {
                            support_center,
                            normal_denominator,
                        }
                    } else {
                        let Some(center_parameter) = selected_center_parameter else {
                            return Ok(None);
                        };
                        RetainedFilletRadialFrame2::SelectedConcentric {
                            support: support_circle,
                            center_parameter,
                            normal_denominator,
                        }
                    };
                (radial_frame, radial_distance)
            }
            Self::Parallel { source, support } => {
                let Some(center_parameter) = anchor_parameter.cloned() else {
                    return Ok(None);
                };
                (
                    RetainedFilletRadialFrame2::ParallelNormal {
                        center_support: support.clone(),
                        center_parameter,
                        policy: *policy,
                    },
                    source.parallel_distance() - support.distance(),
                )
            }
            Self::AlgebraicChord {
                source,
                signed_distance,
                ..
            } => {
                let radial_frame = RetainedFilletRadialFrame2::ChordNormal {
                    anchor: (*source).clone(),
                    policy: *policy,
                };
                (radial_frame, -signed_distance.clone())
            }
            _ => return Ok(None),
        };
        Ok(Some(RetainedFilletFrame2 {
            anchor_is_previous,
            radial_frame,
            radial_distance,
            anchor_evidence,
        }))
    }
}

/// A contact on a stationary source owns the regular side that survives the
/// cut. Its unit tangent chord retains that side without reconstructing a
/// vanishing hodograph; the scale orients the original offset's traversal.
pub(super) struct FilletSourceFrame2 {
    pub(super) tangent: crate::BezierAlgebraicChord2,
    pub(super) derivative_scale: RealSign,
}

pub(super) struct FilletCenterWitness2 {
    pub(super) source_frames: [Option<FilletSourceFrame2>; 2],
    pub(super) point: CurvePoint2,
    pub(super) previous_parameter: Option<CurveParameter2>,
    pub(super) next_parameter: Option<CurveParameter2>,
    pub(super) retained_anchor_evidence: Option<RetainedFilletAnchorEvidence2>,
}

pub(super) const fn reverse_fillet_sign(sign: RealSign) -> RealSign {
    match sign {
        RealSign::Positive => RealSign::Negative,
        RealSign::Negative => RealSign::Positive,
        RealSign::Zero => RealSign::Zero,
    }
}

pub(super) fn retained_fillet_cusp_support_reverses_source(
    source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    support: &crate::BezierAlgebraicCuspSemicircleFragment2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let nonzero_radius_sign = |radius: &Real| match crate::classify::real_sign(radius, policy) {
        Some(sign @ (RealSign::Positive | RealSign::Negative)) => Ok(sign),
        Some(RealSign::Zero) => Err(ExactCurveError::invalid(
            CurveOperation2::Fillet,
            family,
            CurveError::Topology(
                "a retained selected-circle fillet support had zero radius".into(),
            ),
        )),
        None => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            crate::UncertaintyReason::RealSign,
        )),
    };
    Ok((nonzero_radius_sign(source.semicircle().radial_distance())?
        != nonzero_radius_sign(support.semicircle().radial_distance())?)
        != source.is_reversed())
}

impl FilletCenterWitness2 {
    pub(super) fn parameter(&self, previous: bool) -> Option<&CurveParameter2> {
        if previous {
            self.previous_parameter.as_ref()
        } else {
            self.next_parameter.as_ref()
        }
    }
}

pub(super) enum FilletCenterCoincidence2 {
    Support,
    /// A represented chart of the original retained chord's support. Its
    /// original endpoint fields continue to own finite-domain admission.
    LinearSource(LineSeg2),
}

#[derive(Default)]
pub(super) struct FilletCenters2 {
    pub(super) first: Option<FilletCenterWitness2>,
    pub(super) second: Option<FilletCenterWitness2>,
    pub(super) overflow: Vec<FilletCenterWitness2>,
    pub(super) components: Vec<crate::bezier_offset::CurveParameterComponent2>,
    pub(super) coincident: Option<FilletCenterCoincidence2>,
    pub(super) outside_domain: bool,
}

impl FilletCenters2 {
    pub(super) fn push(&mut self, witness: FilletCenterWitness2) {
        if self.first.is_none() {
            self.first = Some(witness);
        } else if self.second.is_none() {
            self.second = Some(witness);
        } else {
            self.overflow.push(witness);
        }
    }

    pub(super) fn iter(&self) -> impl Iterator<Item = &FilletCenterWitness2> {
        self.first
            .iter()
            .chain(self.second.iter())
            .chain(self.overflow.iter())
    }

    pub(super) fn iter_mut(&mut self) -> impl Iterator<Item = &mut FilletCenterWitness2> {
        self.first
            .iter_mut()
            .chain(self.second.iter_mut())
            .chain(self.overflow.iter_mut())
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn solve_carrier_fillet_corner(
    previous: ExactCornerCarrier2<'_>,
    next: ExactCornerCarrier2<'_>,
    radius: &Real,
    retain_selected_circle_endpoints: bool,
    domains: [FilletContactDomain2; 2],
    previous_family: CurveFamily2,
    next_family: CurveFamily2,
    constraints: Option<&FilletConstraintBinding2<'_>>,
    policy: &CurveContext,
) -> ExactCurveResult<CurveCornerSolutions2<FilletCorner2>> {
    let previous = PreparedFilletCarrier2::new(previous, previous_family, domains[0], policy)?;
    let next = PreparedFilletCarrier2::new(next, next_family, domains[1], policy)?;
    let mut candidates = CornerSolutionAccumulator::Empty;
    let mut saw_outside_domain = false;
    let mut saw_degenerate = false;
    let mut saw_unsatisfied = false;

    // Positive signed distance is the common left offset and therefore gives
    // a counterclockwise fillet. Preserve that documented candidate order.
    for clockwise in [false, true] {
        let signed_distance = if clockwise {
            -radius.clone()
        } else {
            radius.clone()
        };
        let previous_offsets = previous.offsets(&signed_distance, previous_family, policy)?;
        let next_offsets = next.offsets(&signed_distance, next_family, policy)?;
        for previous_offset in previous_offsets.iter().flatten() {
            'offset_pair: for next_offset in next_offsets.iter().flatten() {
                if let Some(center) =
                    constraints.and_then(|binding| binding.request.center.as_ref())
                {
                    for (axis, (offset, family)) in [
                        (previous_offset, previous_family),
                        (next_offset, next_family),
                    ]
                    .into_iter()
                    .enumerate()
                    {
                        if matches!(
                            offset,
                            FilletOffsetCarrier2::Line { .. }
                                | FilletOffsetCarrier2::Arc { .. }
                                | FilletOffsetCarrier2::Point { .. }
                        ) && matches!(
                            policy.strict_predicate_pass(|| point_on_fillet_offset(
                                center,
                                offset,
                                axis == 0,
                                domains[axis],
                                family,
                                policy
                            )),
                            Ok(false)
                        ) {
                            // A certified center constraint can exclude a
                            // native support before unrelated elimination.
                            // An unavailable proof leaves the general solve.
                            saw_unsatisfied = true;
                            continue 'offset_pair;
                        }
                    }
                }
                if let Some(centers) = curve_fillet::constrained_fillet_centers(
                    [&previous, &next],
                    [previous_offset, next_offset],
                    &signed_distance,
                    domains,
                    [previous_family, next_family],
                    constraints,
                    policy,
                )? {
                    saw_unsatisfied |= centers.no_solution_reason()
                        == Some(CurveCornerNoSolution2::UnsatisfiedConstraints);
                    for center in centers.solutions() {
                        let solutions = curve_fillet::fillet_at_center(
                            [&previous, &next],
                            [previous_offset, next_offset],
                            center,
                            clockwise,
                            &signed_distance,
                            retain_selected_circle_endpoints,
                            domains,
                            [previous_family, next_family],
                            constraints,
                            policy,
                        )?;
                        match candidates.append(solutions) {
                            Some(CurveCornerNoSolution2::DegenerateCandidate) => {
                                saw_degenerate = true
                            }
                            Some(CurveCornerNoSolution2::OutsideTrimDomain) => {
                                saw_outside_domain = true
                            }
                            Some(CurveCornerNoSolution2::UnsatisfiedConstraints) => {
                                saw_unsatisfied = true
                            }
                            _ => (),
                        }
                    }
                    continue;
                }
                let normal_constraints = match [
                    previous.component_normal_constraint(
                        previous_offset,
                        &signed_distance,
                        hypersolve::CurveResultantParameter::First,
                        previous_family,
                    )?,
                    next.component_normal_constraint(
                        next_offset,
                        &signed_distance,
                        hypersolve::CurveResultantParameter::Second,
                        next_family,
                    )?,
                ] {
                    [Some(first), Some(second)] => Some([first, second]),
                    _ => None,
                };
                let centers = fillet_offset_centers(
                    previous_offset,
                    next_offset,
                    domains,
                    previous_family,
                    next_family,
                    normal_constraints.as_ref(),
                    policy,
                )?;
                saw_outside_domain |= centers.outside_domain;
                if let Some(coincidence) = &centers.coincident {
                    if matches!(
                        (previous_offset, next_offset),
                        (
                            FilletOffsetCarrier2::Line { .. }
                                | FilletOffsetCarrier2::AlgebraicChord { .. },
                            FilletOffsetCarrier2::Parallel { .. }
                        ) | (
                            FilletOffsetCarrier2::Parallel { .. },
                            FilletOffsetCarrier2::Line { .. }
                                | FilletOffsetCarrier2::AlgebraicChord { .. }
                        )
                    ) {
                        return curve_fillet::replay_coincident_linear_parallel_fillet(
                            [&previous, &next],
                            match coincidence {
                                FilletCenterCoincidence2::LinearSource(line) => Some(line),
                                FilletCenterCoincidence2::Support => None,
                            },
                            radius,
                            retain_selected_circle_endpoints,
                            domains,
                            [previous_family, next_family],
                            constraints,
                            policy,
                        );
                    }
                    let solutions = if [previous_offset, next_offset].iter().all(|offset| {
                        matches!(
                            offset,
                            FilletOffsetCarrier2::Line { .. }
                                | FilletOffsetCarrier2::AlgebraicChord { .. }
                        )
                    }) {
                        curve_fillet::constrained_coincident_linear_fillet(
                            [previous_offset, next_offset],
                            &signed_distance,
                            clockwise,
                            retain_selected_circle_endpoints,
                            domains,
                            [previous_family, next_family],
                            constraints,
                            policy,
                        )?
                    } else if [previous_offset, next_offset]
                        .iter()
                        .all(|offset| matches!(offset, FilletOffsetCarrier2::Arc { .. }))
                    {
                        curve_fillet::constrained_coincident_circular_fillet(
                            [previous_offset, next_offset],
                            clockwise,
                            retain_selected_circle_endpoints,
                            domains,
                            [previous_family, next_family],
                            constraints,
                            policy,
                        )?
                    } else {
                        CurveCornerSolutions2::NoSolution(
                            CurveCornerNoSolution2::DegenerateCandidate,
                        )
                    };
                    match candidates.append(solutions) {
                        Some(CurveCornerNoSolution2::DegenerateCandidate) => saw_degenerate = true,
                        Some(CurveCornerNoSolution2::OutsideTrimDomain) => {
                            saw_outside_domain = true
                        }
                        Some(CurveCornerNoSolution2::UnsatisfiedConstraints) => {
                            saw_unsatisfied = true
                        }
                        _ => (),
                    }
                }
                for center in centers.iter() {
                    let accepts = |prepared: &PreparedFilletCarrier2<'_>,
                                   offset: &FilletOffsetCarrier2<'_, '_>,
                                   axis: usize,
                                   family| {
                        if let Some(frame) = &center.source_frames[axis] {
                            Ok(prepared.accepts_offset_direction(
                                offset,
                                frame.derivative_scale,
                                &signed_distance,
                            ))
                        } else {
                            prepared.accepts_offset_contact(
                                offset,
                                center.parameter(axis == 0),
                                axis == 0,
                                &signed_distance,
                                family,
                                policy,
                            )
                        }
                    };
                    if !accepts(&previous, previous_offset, 0, previous_family)?
                        || !accepts(&next, next_offset, 1, next_family)?
                    {
                        continue;
                    }
                    match fillet_corner_from_center(
                        previous_offset,
                        next_offset,
                        center,
                        clockwise,
                        retain_selected_circle_endpoints,
                        domains,
                        previous_family,
                        next_family,
                        policy,
                    )? {
                        FilletCornerSelection2::Selected(candidate) => {
                            if let Some(binding) = constraints
                                && !binding.matches(&candidate, policy)?
                            {
                                saw_unsatisfied = true;
                                continue;
                            }
                            candidates.push(candidate);
                        }
                        FilletCornerSelection2::Outside => saw_outside_domain = true,
                        FilletCornerSelection2::Degenerate => saw_degenerate = true,
                    }
                }
                curve_fillet::FilletComponentReplay2::solve(
                    &centers.components,
                    [previous_offset, next_offset],
                    clockwise,
                    retain_selected_circle_endpoints,
                    domains,
                    [previous_family, next_family],
                    constraints,
                    &mut candidates,
                    policy,
                )?;
                saw_unsatisfied |= !centers.components.is_empty();
            }
        }
    }

    // An in-domain candidate that collapses is more specific than unrelated
    // support intersections outside the authored trims. In particular, it
    // must not be relabeled as an extendable trim-domain miss.
    let empty_reason = if saw_unsatisfied {
        CurveCornerNoSolution2::UnsatisfiedConstraints
    } else if saw_degenerate {
        CurveCornerNoSolution2::DegenerateCandidate
    } else if saw_outside_domain {
        CurveCornerNoSolution2::OutsideTrimDomain
    } else {
        CurveCornerNoSolution2::NoTangentCircle
    };
    Ok(candidates.finish(empty_reason))
}

pub(super) fn retained_fillet_cusp_fragment_range(
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
) -> crate::CurveParameterRange2 {
    crate::CurveParameterRange2::new_validated(
        CurveParameter2::from_algebraic_cusp(fragment.start_parameter().clone()),
        CurveParameter2::from_algebraic_cusp(fragment.end_parameter().clone()),
    )
}

pub(super) fn retained_fillet_positive_overlap(
    result: crate::CurveResult<Classification<bool>>,
    family: CurveFamily2,
) -> ExactCurveResult<bool> {
    match result
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(positive) => Ok(positive),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    }
}

pub(super) fn retained_fillet_cusp_mapped_overlap_is_positive(
    cusp: &crate::BezierAlgebraicCuspSemicircleFragment2,
    overlap: &crate::bezier_offset::BezierAlgebraicCuspSemicircleMappedOverlap2,
    other_range: &crate::CurveParameterRange2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    retained_fillet_positive_overlap(
        overlap.has_positive_overlap(
            &retained_fillet_cusp_fragment_range(cusp),
            other_range,
            policy,
        ),
        family,
    )
}

pub(super) fn retained_fillet_cusp_pair_overlap_is_positive(
    first: &crate::BezierAlgebraicCuspSemicircleFragment2,
    second: &crate::BezierAlgebraicCuspSemicircleFragment2,
    overlap: &crate::bezier_offset::BezierAlgebraicCuspSemicirclePairOverlap2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    retained_fillet_positive_overlap(
        overlap.has_positive_overlap(
            &retained_fillet_cusp_fragment_range(first),
            &retained_fillet_cusp_fragment_range(second),
            policy,
        ),
        family,
    )
}

/// Publishes a compact one-field center when one side of a direct circle pair
/// already has an exact rational image. Contact selection remains wholly owned
/// by the pair map; the temporary conic spans only encode the chosen point for
/// retained curve storage.
pub(super) fn retained_fillet_pair_contact_rational_point(
    support: &crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
    parameter_map: &crate::bezier_offset::BezierAlgebraicCuspSemicirclePairParameterMap2,
    contact: &crate::bezier_offset::BezierAlgebraicCuspSemicirclePairContact2,
    first: bool,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurvePoint2>> {
    if !support.has_rational_frame() {
        return Ok(None);
    }
    let center = support
        .center_point_image(policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?;
    let start = support
        .start_point_image(policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?;
    let end = support
        .end_point_image(policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?;
    let (Some(center), Some(start), Some(end)) = (
        center.exact_point(&CurveContext::STRICT),
        start.exact_point(&CurveContext::STRICT),
        end.exact_point(&CurveContext::STRICT),
    ) else {
        return Ok(None);
    };
    let arc = CircularArc2::try_from_center(start, end, center, support.is_clockwise())
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?;
    retained_fillet_pair_contact_rational_point_on_arc(
        &arc,
        parameter_map,
        contact,
        first,
        family,
        policy,
    )
}

/// Publishes a selected-circle pair contact through an already-certified
/// rational chart of one supporting half circle. The circle-pair map remains
/// the sole root authority; this inverse chart only chooses the compact
/// one-field point carrier used by the retained CurveRegion boundary.
pub(super) fn retained_fillet_pair_contact_rational_point_on_arc(
    arc: &CircularArc2,
    parameter_map: &crate::bezier_offset::BezierAlgebraicCuspSemicirclePairParameterMap2,
    contact: &crate::bezier_offset::BezierAlgebraicCuspSemicirclePairContact2,
    first: bool,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurvePoint2>> {
    let decomposition = match arc
        .rational_bezier_decomposition_raw(policy)
        .map_err(|error| error.with_operation(CurveOperation2::Fillet))?
    {
        Classification::Decided(decomposition) => decomposition,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                reason,
            ));
        }
    };
    for span in decomposition.spans() {
        let curve: RationalBezier2 = span.curve().clone().into();
        let points = match parameter_map
            .rational_point_evidence_for_contact(contact, first, &curve, false, policy)
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
        {
            Classification::Decided(points) => points,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    family,
                    reason,
                ));
            }
        };
        if let Some(point) = points.into_iter().next() {
            return Ok(Some(point));
        }
    }
    Ok(None)
}

pub(super) fn retained_fillet_incident_overlap_range(
    overlap: &crate::CurveParameterRange2,
    domain: &crate::bezier_offset::BezierParallelIncidentDomain2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<crate::CurveParameterRange2>> {
    let barrier = || {
        domain
            .barrier()
            .map(|parameter| CurveParameter2::from(parameter.clone()))
    };
    let (start, end) = match domain.direction() {
        crate::BezierParameterRayDirection2::Decreasing => (
            barrier().unwrap_or_else(|| overlap.start().clone()),
            domain.endpoint().clone(),
        ),
        crate::BezierParameterRayDirection2::Increasing => (
            domain.endpoint().clone(),
            barrier().unwrap_or_else(|| overlap.end().clone()),
        ),
    };
    Ok(
        match retained_fillet_curve_region_parameter_order(&start, &end, family, policy)? {
            std::cmp::Ordering::Equal => None,
            std::cmp::Ordering::Less | std::cmp::Ordering::Greater => {
                Some(crate::CurveParameterRange2::new_validated(start, end))
            }
        },
    )
}

pub(super) fn retained_fillet_curve_region_parameter_order(
    first: &CurveParameter2,
    second: &CurveParameter2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<std::cmp::Ordering> {
    match first
        .cmp_by_refinement(second, policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(order) => Ok(order),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    }
}

/// Clips a positive-dimensional mixed arc/selected-circle center component to
/// the two authored finite domains. Isolated centers are owned by the common
/// selected-circle pair kernel; rational arc cells survive here only as an
/// exact inverse-domain adapter for coincident supporting circles.
pub(super) fn retained_fillet_arc_cusp_overlap_is_positive(
    arc_support: &CircularArc2,
    cusp: &crate::BezierAlgebraicCuspSemicircleFragment2,
    arc_family: CurveFamily2,
    cusp_family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let range =
        crate::CurveParameterRange2::from_bezier_range(BezierParameterRange2::new_validated(
            BezierParameter2::Exact(Real::zero()),
            BezierParameter2::Exact(Real::one()),
        ));
    for (cell, _, _, _) in retained_arc_fillet_projective_cells(
        arc_support,
        CurveCornerMode2::TrimOnly,
        arc_family,
        policy,
    )? {
        let (intersections, _) = match cusp
            .semicircle()
            .rational_intersections_with_parameter_map(
                &cell,
                &crate::CurveParameterRange2::unit(),
                policy,
            )
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, cusp_family, cause)
            })? {
            Classification::Decided(result) => result,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    cusp_family,
                    reason,
                ));
            }
        };
        match intersections {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber { overlaps, .. } => {
                for overlap in overlaps {
                    if retained_selected_fillet_overlap_is_positive(
                        &overlap,
                        &range,
                        None,
                        cusp,
                        arc_family,
                        cusp_family,
                        policy,
                    )? {
                        return Ok(true);
                    }
                }
            }
            crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { overlaps, .. } => {
                for overlap in overlaps {
                    let cell_overlap = overlap.other_range().clone();
                    if retained_fillet_cusp_mapped_overlap_is_positive(
                        cusp,
                        &overlap,
                        &cell_overlap,
                        cusp_family,
                        policy,
                    )? {
                        return Ok(true);
                    }
                }
            }
            crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    cusp_family,
                    crate::UncertaintyReason::Unsupported,
                ));
            }
        }
    }
    Ok(false)
}

pub(super) fn retained_selected_fillet_overlap_is_positive(
    overlap: &crate::bezier_offset::BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2,
    analytic_range: &crate::CurveParameterRange2,
    incident_domain: Option<&crate::bezier_offset::BezierParallelIncidentDomain2>,
    cusp_source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    analytic_family: CurveFamily2,
    cusp_family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let cusp_range = retained_fillet_cusp_fragment_range(cusp_source);
    let overlaps_authored = retained_fillet_positive_overlap(
        overlap.has_positive_overlap(&cusp_range, analytic_range, policy),
        cusp_family,
    )?;
    if overlaps_authored {
        return Ok(true);
    }
    let Some(domain) = incident_domain else {
        return Ok(false);
    };
    let other_overlap = crate::CurveParameterRange2::new_validated(
        CurveParameter2::from_selected_fiber(overlap.other_start_parameter()),
        CurveParameter2::from_selected_fiber(overlap.other_end_parameter()),
    );
    let Some(incident_range) =
        retained_fillet_incident_overlap_range(&other_overlap, domain, analytic_family, policy)?
    else {
        return Ok(false);
    };
    retained_fillet_positive_overlap(
        overlap.has_positive_overlap(&cusp_range, &incident_range, policy),
        cusp_family,
    )
}

#[allow(clippy::too_many_arguments)]
pub(super) fn retain_cusp_parallel_fillet_contact(
    centers: &mut FilletCenters2,
    cusp_source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    parallel_source: FilletParallelSource2<'_>,
    analytic_support: &BezierParallel2,
    cusp_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
    analytic_parameter: CurveParameter2,
    point: CurvePoint2,
    location: crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2,
    mut cross: RealSign,
    mut dot: RealSign,
    complementary: bool,
    cusp_support_reverses_source: bool,
    cusp_is_previous: bool,
    domains: [FilletContactDomain2; 2],
    incident_domain: Option<&crate::bezier_offset::BezierParallelIncidentDomain2>,
    cusp_family: CurveFamily2,
    analytic_family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    let cusp_mode = domains[usize::from(!cusp_is_previous)].mode();
    let analytic_domain = domains[usize::from(cusp_is_previous)];
    if complementary
        && location != crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior
    {
        return Ok(());
    }
    if cusp_mode != CurveCornerMode2::TrimOrExtend {
        match cusp_source
            .contains_parameter(&cusp_parameter, false, false, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, cusp_family, cause)
            })? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(()),
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    cusp_family,
                    reason,
                ));
            }
        }
    }
    if !parallel_source.parameter_is_admissible(
        &analytic_parameter,
        !cusp_is_previous,
        analytic_domain,
        incident_domain,
        analytic_family,
        policy,
    )? {
        return Ok(());
    }
    if cusp_support_reverses_source {
        cross = reverse_fillet_sign(cross);
        dot = reverse_fillet_sign(dot);
    }
    let analytic_support_reverses_source = parallel_source.support_reverses_source_at(
        analytic_support,
        &analytic_parameter,
        !cusp_is_previous,
        analytic_family,
        policy,
    )?;
    if analytic_support_reverses_source == Some(true) {
        cross = reverse_fillet_sign(cross);
        dot = reverse_fillet_sign(dot);
    }
    let cusp_parameter = if complementary {
        CurveParameter2::from_algebraic_cusp_complement(cusp_parameter)
    } else {
        CurveParameter2::from_algebraic_cusp(cusp_parameter)
    };
    let (previous_parameter, next_parameter) = if cusp_is_previous {
        (Some(cusp_parameter), Some(analytic_parameter.clone()))
    } else {
        (Some(analytic_parameter.clone()), Some(cusp_parameter))
    };
    centers.push(FilletCenterWitness2 {
        source_frames: [None, None],
        point,
        previous_parameter,
        next_parameter,
        // The retained frame is the analytic carrier, so store analytic x
        // cusp rather than the direct kernel's cusp x analytic relation.
        retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
            cross: analytic_support_reverses_source.map(|_| reverse_fillet_sign(cross)),
            dot: analytic_support_reverses_source.map(|_| dot),
            center_parallel: Some(RetainedFilletCenterParallel2 {
                support: analytic_support.clone(),
                parameter: Some(analytic_parameter),
            }),
            source_direction: analytic_support_reverses_source.map(|reversed| {
                if reversed {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                }
            }),
            canonical_anchor_curve: None,
            deferred_arc_contact: None,
        }),
    });
    Ok(())
}

pub(super) fn fillet_offset_centers(
    previous: &FilletOffsetCarrier2<'_, '_>,
    next: &FilletOffsetCarrier2<'_, '_>,
    domains: [FilletContactDomain2; 2],
    previous_family: CurveFamily2,
    next_family: CurveFamily2,
    normal_constraints: Option<&[crate::bezier_offset::BezierParallelDerivativeConstraint2; 2]>,
    policy: &CurveContext,
) -> ExactCurveResult<FilletCenters2> {
    let [previous_mode, next_mode] = domains.map(FilletContactDomain2::mode);
    let exact_parameter = |parameter| CurveParameter2::from(BezierParameter2::Exact(parameter));
    let mut centers = FilletCenters2::default();
    match (previous, next) {
        (FilletOffsetCarrier2::Point { .. }, _) | (_, FilletOffsetCarrier2::Point { .. }) => {
            unreachable!("collapsed offsets resolve from their source contact constraints")
        }
        (FilletOffsetCarrier2::Line { .. }, FilletOffsetCarrier2::Arc { .. })
        | (FilletOffsetCarrier2::Arc { .. }, FilletOffsetCarrier2::Line { .. }) => {
            let (support, line_source, source, signed_radius, line_is_previous) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::Line {
                            source: line_source,
                            support,
                            ..
                        },
                        FilletOffsetCarrier2::Arc {
                            source,
                            signed_radius,
                            ..
                        },
                    ) => (support, line_source, source, signed_radius, true),
                    (
                        FilletOffsetCarrier2::Arc {
                            source,
                            signed_radius,
                            ..
                        },
                        FilletOffsetCarrier2::Line {
                            source: line_source,
                            support,
                            ..
                        },
                    ) => (support, line_source, source, signed_radius, false),
                    _ => unreachable!(),
                };
            let relation = crate::intersect::line_circle_relation_from_supports(
                support,
                source.support().center(),
                &(signed_radius * signed_radius),
                policy,
            )
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause)
            })?;
            let mut push = |point: Point2, parameter: Real| {
                let parameter = line_source
                    .native_line()
                    .is_some()
                    .then(|| exact_parameter(parameter));
                let (previous_parameter, next_parameter) = if line_is_previous {
                    (parameter, None)
                } else {
                    (None, parameter)
                };
                centers.push(FilletCenterWitness2 {
                    source_frames: [None, None],
                    point: point.into(),
                    previous_parameter,
                    next_parameter,
                    retained_anchor_evidence: None,
                });
            };
            match relation {
                crate::LineCircleRelation::Disjoint => {}
                crate::LineCircleRelation::Tangent { point, line_param } => {
                    push(point, line_param);
                }
                crate::LineCircleRelation::Secant {
                    first_point,
                    first_param,
                    second_point,
                    second_param,
                } => {
                    push(first_point, first_param);
                    push(second_point, second_param);
                }
                crate::LineCircleRelation::Uncertain { reason } => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        previous_family,
                        reason,
                    ));
                }
            }
        }
        (
            FilletOffsetCarrier2::Arc {
                source: previous,
                signed_radius: previous_radius,
                ..
            },
            FilletOffsetCarrier2::Arc {
                source: next,
                signed_radius: next_radius,
                ..
            },
        ) => match crate::intersect::circle_relation_from_supports(
            previous.support().center(),
            &(previous_radius * previous_radius),
            next.support().center(),
            &(next_radius * next_radius),
            policy,
        )
        .map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause)
        })? {
            crate::CircleCircleRelation::Disjoint => {}
            crate::CircleCircleRelation::Tangent { point } => {
                centers.push(FilletCenterWitness2 {
                    source_frames: [None, None],
                    point: point.into(),
                    previous_parameter: None,
                    next_parameter: None,
                    retained_anchor_evidence: None,
                });
            }
            crate::CircleCircleRelation::Secant {
                first_point,
                second_point,
            } => {
                centers.push(FilletCenterWitness2 {
                    source_frames: [None, None],
                    point: first_point.into(),
                    previous_parameter: None,
                    next_parameter: None,
                    retained_anchor_evidence: None,
                });
                centers.push(FilletCenterWitness2 {
                    source_frames: [None, None],
                    point: second_point.into(),
                    previous_parameter: None,
                    next_parameter: None,
                    retained_anchor_evidence: None,
                });
            }
            crate::CircleCircleRelation::Coincident => {
                centers.coincident = Some(FilletCenterCoincidence2::Support)
            }
            crate::CircleCircleRelation::Uncertain { reason } => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    previous_family,
                    reason,
                ));
            }
        },
        (
            FilletOffsetCarrier2::Arc { .. },
            FilletOffsetCarrier2::Parallel {
                source: parallel_source,
                ..
            },
        )
        | (
            FilletOffsetCarrier2::Parallel {
                source: parallel_source,
                ..
            },
            FilletOffsetCarrier2::Arc { .. },
        ) => {
            let (arc, source_radius, signed_radius, bezier, bezier_is_previous) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::Arc {
                            source,
                            source_radius,
                            signed_radius,
                        },
                        FilletOffsetCarrier2::Parallel {
                            support: bezier, ..
                        },
                    ) => (source, source_radius, signed_radius, bezier, false),
                    (
                        FilletOffsetCarrier2::Parallel {
                            support: bezier, ..
                        },
                        FilletOffsetCarrier2::Arc {
                            source,
                            source_radius,
                            signed_radius,
                        },
                    ) => (source, source_radius, signed_radius, bezier, true),
                    _ => unreachable!(),
                };
            let bezier_family = if bezier_is_previous {
                previous_family
            } else {
                next_family
            };
            let mode = domains[usize::from(!bezier_is_previous)].mode();
            let incident_domain = if mode == CurveCornerMode2::TrimOrExtend {
                Some(parallel_source.incident_domain(
                    bezier,
                    bezier_is_previous,
                    bezier_family,
                    policy,
                )?)
            } else {
                None
            };
            let mut parameters = match bezier
                .circle_incidence(
                    arc.support().center(),
                    &(signed_radius * signed_radius),
                    &parallel_source.curve_parameter_range(),
                    &[],
                    policy,
                )
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, bezier_family, cause)
                })? {
                Classification::Decided(parameters) => parameters,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        bezier_family,
                        reason,
                    ));
                }
            };
            if let Some(domain) = incident_domain.as_ref() {
                match bezier
                    .circle_incidence_on_incident_ray(
                        arc.support().center(),
                        &(signed_radius * signed_radius),
                        domain,
                        policy,
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, bezier_family, cause)
                    })? {
                    Classification::Decided(exterior) => parameters.extend(exterior),
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            bezier_family,
                            reason,
                        ));
                    }
                }
            }
            for (parameter, _) in parameters {
                if !parallel_source.parameter_is_admissible(
                    &parameter.clone().into(),
                    bezier_is_previous,
                    domains[usize::from(!bezier_is_previous)],
                    incident_domain.as_ref(),
                    bezier_family,
                    policy,
                )? {
                    continue;
                }
                let point = analytic_parallel_point_evidence(
                    bezier,
                    &parameter.clone().into(),
                    CurveOperation2::Fillet,
                    bezier_family,
                    policy,
                )?;
                let retained_anchor_evidence =
                    point
                        .coordinates()
                        .is_none()
                        .then(|| RetainedFilletAnchorEvidence2 {
                            cross: None,
                            dot: None,
                            center_parallel: None,
                            source_direction: None,
                            canonical_anchor_curve: None,
                            deferred_arc_contact: Some(RetainedDeferredArcFilletContact2 {
                                source: ExactCornerArc2::clone(arc),
                                source_radius: (*source_radius).clone(),
                                signed_center_radius: signed_radius.clone(),
                                arc_is_previous: !bezier_is_previous,
                                domain: domains[usize::from(bezier_is_previous)],
                                selected_center: None,
                                contact_seed: None,
                            }),
                        });
                let bezier_parameter = CurveParameter2::from(parameter);
                centers.push(FilletCenterWitness2 {
                    source_frames: [None, None],
                    point,
                    previous_parameter: bezier_is_previous.then(|| bezier_parameter.clone()),
                    next_parameter: (!bezier_is_previous).then_some(bezier_parameter),
                    retained_anchor_evidence,
                });
            }
        }
        (
            FilletOffsetCarrier2::Parallel {
                source: previous_source,
                support: previous,
            },
            FilletOffsetCarrier2::Parallel {
                source: next_source,
                support: next,
            },
        ) => {
            return curve_fillet::parallel_pair_centers(
                [*previous_source, *next_source],
                [previous, next],
                domains,
                [previous_family, next_family],
                normal_constraints,
                policy,
            );
        }
        (FilletOffsetCarrier2::Line { .. }, FilletOffsetCarrier2::Parallel { .. })
        | (FilletOffsetCarrier2::Parallel { .. }, FilletOffsetCarrier2::Line { .. }) => {
            let (line, line_source, line_unit_x, line_unit_y, parallel, line_is_previous) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::Line {
                            source,
                            support,
                            unit_x,
                            unit_y,
                            ..
                        },
                        parallel @ FilletOffsetCarrier2::Parallel { .. },
                    ) => (support, source, *unit_x, *unit_y, parallel, true),
                    (
                        parallel @ FilletOffsetCarrier2::Parallel { .. },
                        FilletOffsetCarrier2::Line {
                            source,
                            support,
                            unit_x,
                            unit_y,
                            ..
                        },
                    ) => (support, source, *unit_x, *unit_y, parallel, false),
                    _ => unreachable!(),
                };
            let FilletOffsetCarrier2::Parallel { source, support } = parallel else {
                unreachable!()
            };
            let parallel_family = if line_is_previous {
                next_family
            } else {
                previous_family
            };
            let line_endpoint = if line_is_previous {
                BezierEndpoint::End
            } else {
                BezierEndpoint::Start
            };
            let certified_tangency = source.retained().and_then(|source| {
                let corner_parameter = if line_is_previous == source.is_reversed() {
                    source.range().end()
                } else {
                    source.range().start()
                };
                corner_parameter.scalar().and_then(|parameter| {
                    line_source
                        .parallel_tangent_contacts()
                        .iter()
                        .find(|contact| {
                            contact.line_endpoint() == line_endpoint
                                && contact.parallel() == source.parallel()
                                && contact.parallel_fragment_reversed() == source.is_reversed()
                                && contact.parameter() == parameter
                        })
                })
            });
            let certified_tangencies = certified_tangency
                .map(|contact| std::slice::from_ref(contact.parameter()))
                .unwrap_or_default();
            let parallel_is_previous = !line_is_previous;
            let mode = domains[usize::from(!parallel_is_previous)].mode();
            let incident_domain = if mode == CurveCornerMode2::TrimOrExtend {
                Some(source.incident_domain(
                    support,
                    parallel_is_previous,
                    parallel_family,
                    policy,
                )?)
            } else {
                None
            };
            let invalid =
                |cause| ExactCurveError::invalid(CurveOperation2::Fillet, parallel_family, cause);
            let blocked =
                |reason| ExactCurveError::blocked(CurveOperation2::Fillet, parallel_family, reason);
            let source_parallel = OnceLock::new();
            let source_parallel = || {
                source_parallel.get_or_init(|| support.with_distance(source.parallel_distance()))
            };
            let finite_range = source.curve_parameter_range();
            // The open ray starts at a represented chart anchor. Include the
            // certified bridge from the actual endpoint in the finite query;
            // the source domain still owns the final trim/extension decision.
            let finite_range = if let Some(incident) = &incident_domain {
                match incident
                    .expanded_range(&finite_range, policy)
                    .map_err(invalid)?
                {
                    Classification::Decided(range) => range,
                    Classification::Uncertain(reason) => return Err(blocked(reason)),
                }
            } else {
                finite_range
            };
            let mut parameters = Vec::new();
            let incidence = support
                .supporting_line_incidence_with_direction(
                    line,
                    (line_unit_x, line_unit_y),
                    certified_tangencies,
                    &finite_range,
                    false,
                    policy,
                )
                .map_err(invalid)?;
            let ranges =
                if incidence == Classification::Uncertain(crate::UncertaintyReason::Boundary) {
                    let analysis = match source_parallel()
                        .singularity_analysis_with_policy(&finite_range, policy)
                        .map_err(invalid)?
                    {
                        Classification::Decided(analysis) => analysis,
                        Classification::Uncertain(reason) => return Err(blocked(reason)),
                    };
                    if analysis.source_is_regular() {
                        return Err(blocked(crate::UncertaintyReason::Boundary));
                    }
                    Some(match analysis.regular_subranges(policy).map_err(invalid)? {
                        Classification::Decided(ranges) => ranges,
                        Classification::Uncertain(reason) => return Err(blocked(reason)),
                    })
                } else {
                    None
                };
            if let Some(ranges) = ranges {
                let retains_lower_side = parallel_is_previous != source.is_reversed();
                for range in ranges {
                    let incidence = support
                        .supporting_line_incidence_with_direction(
                            line,
                            (line_unit_x, line_unit_y),
                            certified_tangencies,
                            &range,
                            true,
                            policy,
                        )
                        .map_err(invalid)?;
                    match incidence {
                        Classification::Decided(crate::BezierParallelIncidence2::EntireCurve) => {
                            centers.coincident = Some(FilletCenterCoincidence2::Support);
                        }
                        Classification::Decided(crate::BezierParallelIncidence2::Parameters(
                            contacts,
                        )) => {
                            for parameter in contacts {
                                // A stationary seam belongs to the regular cell
                                // retained by the cut. Opposite normal sheets
                                // must neither be merged nor both published.
                                let cell =
                                    CurveParameterDomain2::new(&range, None).with_finite_inclusion(
                                        [!retains_lower_side, retains_lower_side],
                                    );
                                match cell
                                    .contains_finite_parameter(&parameter.clone().into(), policy)
                                    .map_err(invalid)?
                                {
                                    Classification::Decided(false) => continue,
                                    Classification::Decided(true) => {}
                                    Classification::Uncertain(reason) => {
                                        return Err(blocked(reason));
                                    }
                                }
                                parameters.push((parameter, Some(range.clone())));
                            }
                        }
                        Classification::Uncertain(reason) => return Err(blocked(reason)),
                    }
                }
            } else {
                match incidence {
                    Classification::Decided(crate::BezierParallelIncidence2::EntireCurve) => {
                        centers.coincident = Some(FilletCenterCoincidence2::Support);
                    }
                    Classification::Decided(crate::BezierParallelIncidence2::Parameters(
                        contacts,
                    )) => {
                        parameters.extend(contacts.into_iter().map(|parameter| (parameter, None)));
                    }
                    Classification::Uncertain(reason) => return Err(blocked(reason)),
                }
            }
            if let Some(domain) = incident_domain.as_ref() {
                match support
                    .supporting_line_incidence_on_incident_ray_with_direction(
                        line,
                        line_unit_x,
                        line_unit_y,
                        domain,
                        policy,
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, parallel_family, cause)
                    })? {
                    Classification::Decided(crate::BezierParallelIncidence2::EntireCurve) => {
                        centers.coincident = Some(FilletCenterCoincidence2::Support);
                    }
                    Classification::Decided(crate::BezierParallelIncidence2::Parameters(
                        exterior,
                    )) => {
                        parameters.extend(exterior.into_iter().map(|parameter| (parameter, None)))
                    }
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            parallel_family,
                            reason,
                        ));
                    }
                }
            }
            for (parameter, regular_range) in parameters {
                if !source.parameter_is_admissible(
                    &parameter.clone().into(),
                    parallel_is_previous,
                    domains[usize::from(!parallel_is_previous)],
                    incident_domain.as_ref(),
                    parallel_family,
                    policy,
                )? {
                    continue;
                }
                // A selected center already owns its exact point and normal.
                // Recover the line cut from that evidence in either mode;
                // rebuilding its scalar image can force an unnecessary
                // resultant over nonrational source coefficients.
                let procedural_affine_contact =
                    parameter.scalar().is_none() || regular_range.is_some();
                let line_parameter = if procedural_affine_contact {
                    None
                } else {
                    let contact = if mode == CurveCornerMode2::TrimOrExtend {
                        support.supporting_line_contact_evidence_affine(line, &parameter, policy)
                    } else {
                        support.supporting_line_contact_evidence(line, &parameter, policy)
                    };
                    let (_, line_parameter) = match contact.map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, parallel_family, cause)
                    })? {
                        Classification::Decided(contact) => contact,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                parallel_family,
                                reason,
                            ));
                        }
                    };
                    line_parameter
                };
                // The selected center is natively one point of this analytic
                // parallel. Keep that one-parameter authority and use the
                // line solve only for its affine cut parameter; publishing an
                // independent Cartesian algebraic image here would force
                // later circle/chord replay to prove equality across two
                // avoidable coordinate constructions.
                let (point, source_frame) = if let Some(range) = &regular_range {
                    let (point, tangent) = match support
                        .regular_source_point_and_tangent_support(
                            support,
                            &parameter.clone().into(),
                            range,
                            RealSign::Positive,
                            policy,
                        )
                        .map_err(invalid)?
                    {
                        Classification::Decided(frame) => frame,
                        Classification::Uncertain(reason) => return Err(blocked(reason)),
                    };
                    let interior = match range.strict_interior_scalar(policy).map_err(invalid)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => return Err(blocked(reason)),
                    };
                    let derivative_scale = match source_parallel()
                        .parallel_derivative_scale_sign(&interior.into(), policy)
                        .map_err(invalid)?
                    {
                        Classification::Decided(
                            sign @ (RealSign::Positive | RealSign::Negative),
                        ) => sign,
                        Classification::Decided(RealSign::Zero) => {
                            return Err(blocked(crate::UncertaintyReason::Boundary));
                        }
                        Classification::Uncertain(reason) => return Err(blocked(reason)),
                    };
                    (
                        point,
                        Some(FilletSourceFrame2 {
                            tangent,
                            derivative_scale,
                        }),
                    )
                } else {
                    (
                        analytic_parallel_point_evidence(
                            support,
                            &parameter.clone().into(),
                            CurveOperation2::Fillet,
                            parallel_family,
                            policy,
                        )?,
                        None,
                    )
                };
                let line_parameter =
                    if line_source.algebraic_chord().is_some() || procedural_affine_contact {
                        None
                    } else {
                        let Some(parameter) = line_parameter else {
                            continue;
                        };
                        Some(CurveParameter2::from(parameter))
                    };
                let retained_anchor_evidence = if let Some(range) = &regular_range {
                    let (mut cross, mut dot) = match source_parallel()
                        .vector_tangent_cross_and_dot_signs_on_regular_range(
                            &parameter.clone().into(),
                            line_unit_x,
                            line_unit_y,
                            range,
                            policy,
                        )
                        .map_err(invalid)?
                    {
                        Classification::Decided(signs) => signs,
                        Classification::Uncertain(reason) => return Err(blocked(reason)),
                    };
                    if source.is_reversed() {
                        cross = reverse_fillet_sign(cross);
                        dot = reverse_fillet_sign(dot);
                    }
                    let direction = source_frame.as_ref().unwrap().derivative_scale;
                    Some(RetainedFilletAnchorEvidence2 {
                        cross: Some(reverse_fillet_sign(cross)),
                        dot: Some(dot),
                        center_parallel: None,
                        source_direction: Some(if source.is_reversed() {
                            reverse_fillet_sign(direction)
                        } else {
                            direction
                        }),
                        canonical_anchor_curve: None,
                        deferred_arc_contact: None,
                    })
                } else {
                    let (mut cross, mut dot) = match support
                        .vector_tangent_cross_and_dot_signs(
                            &parameter.clone().into(),
                            line_unit_x,
                            line_unit_y,
                            policy,
                        )
                        .map_err(|cause| {
                            ExactCurveError::invalid(
                                CurveOperation2::Fillet,
                                parallel_family,
                                cause,
                            )
                        })? {
                        Classification::Decided(signs) => signs,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                parallel_family,
                                reason,
                            ));
                        }
                    };
                    let support_reverses_source = source.support_reverses_source_at(
                        support,
                        &parameter.clone().into(),
                        parallel_is_previous,
                        parallel_family,
                        policy,
                    )?;
                    if support_reverses_source == Some(true) {
                        cross = reverse_fillet_sign(cross);
                        dot = reverse_fillet_sign(dot);
                    }
                    Some(RetainedFilletAnchorEvidence2 {
                        // The selected analytic circle frame is the anchor;
                        // the predicate above reports line x analytic.
                        cross: support_reverses_source.map(|_| reverse_fillet_sign(cross)),
                        dot: support_reverses_source.map(|_| dot),
                        center_parallel: None,
                        source_direction: support_reverses_source.map(|reversed| {
                            if reversed {
                                RealSign::Negative
                            } else {
                                RealSign::Positive
                            }
                        }),
                        canonical_anchor_curve: None,
                        deferred_arc_contact: None,
                    })
                };
                let parallel_parameter = Some(CurveParameter2::from(parameter));
                let (previous_parameter, next_parameter) = if line_is_previous {
                    (line_parameter, parallel_parameter)
                } else {
                    (parallel_parameter, line_parameter)
                };
                let source_frames = if line_is_previous {
                    [None, source_frame]
                } else {
                    [source_frame, None]
                };
                centers.push(FilletCenterWitness2 {
                    source_frames,
                    point,
                    previous_parameter,
                    next_parameter,
                    retained_anchor_evidence,
                });
            }
        }
        (FilletOffsetCarrier2::AlgebraicCusp { .. }, FilletOffsetCarrier2::Parallel { .. })
        | (FilletOffsetCarrier2::Parallel { .. }, FilletOffsetCarrier2::AlgebraicCusp { .. }) => {
            let (cusp_source, cusp_support, parallel_source, analytic_support, cusp_is_previous) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::AlgebraicCusp { source, support },
                        FilletOffsetCarrier2::Parallel {
                            source: analytic,
                            support: analytic_support,
                        },
                    ) => (source, support, analytic, analytic_support, true),
                    (
                        FilletOffsetCarrier2::Parallel {
                            source: analytic,
                            support: analytic_support,
                        },
                        FilletOffsetCarrier2::AlgebraicCusp { source, support },
                    ) => (source, support, analytic, analytic_support, false),
                    _ => unreachable!(),
                };
            let cusp_family = if cusp_is_previous {
                previous_family
            } else {
                next_family
            };
            let analytic_family = if cusp_is_previous {
                next_family
            } else {
                previous_family
            };
            let analytic_authored_range = parallel_source.curve_parameter_range();
            let mode = domains[usize::from(cusp_is_previous)].mode();
            let cusp_mode = domains[usize::from(!cusp_is_previous)].mode();
            let analytic_range = parallel_source.intersection_parameter_range(analytic_family)?;
            let cusp_support_reverses_source = retained_fillet_cusp_support_reverses_source(
                cusp_source,
                cusp_support,
                cusp_family,
                policy,
            )?;
            let incident_domain = if mode == CurveCornerMode2::TrimOrExtend {
                Some(parallel_source.incident_domain(
                    analytic_support,
                    !cusp_is_previous,
                    analytic_family,
                    policy,
                )?)
            } else {
                None
            };
            let projected_range = if let Some(domain) = incident_domain.as_ref() {
                match domain
                    .expanded_range(
                        &CurveParameterRange2::from_bezier_range(analytic_range.clone()),
                        policy,
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, analytic_family, cause)
                    })? {
                    Classification::Decided(range) => range,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            analytic_family,
                            reason,
                        ));
                    }
                }
            } else {
                CurveParameterRange2::from_bezier_range(analytic_range.clone())
            };
            let complementary_support = (cusp_mode == CurveCornerMode2::TrimOrExtend)
                .then(|| cusp_support.semicircle().complementary_half());
            // Every selected-circle frame enters the same complete
            // circle/parallel authority. Extension changes only the chart
            // domains: the authored half owns both diameter endpoints, its
            // complement owns neither, and the analytic carrier contributes
            // only its regular incident ray beyond the authored range.
            for (cusp_circle, complementary) in std::iter::once((cusp_support.semicircle(), false))
                .chain(complementary_support.as_ref().map(|circle| (circle, true)))
            {
                let result = cusp_circle.parallel_intersections(
                    analytic_support,
                    &projected_range,
                    incident_domain.as_ref(),
                    policy,
                );
                let intersections = match result.map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, cusp_family, cause)
                })? {
                    Classification::Decided(intersections) => intersections,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            cusp_family,
                            reason,
                        ));
                    }
                };
                match intersections {
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiber { contacts, overlaps } => {
                        for contact in contacts {
                            let dot = match contact
                                .tangent_dot_sign(policy)
                                .map_err(|cause| {
                                    ExactCurveError::invalid(
                                        CurveOperation2::Fillet,
                                        analytic_family,
                                        cause,
                                    )
                                })? {
                                Classification::Decided(sign) => sign,
                                Classification::Uncertain(reason) => {
                                    return Err(ExactCurveError::blocked(
                                        CurveOperation2::Fillet,
                                        analytic_family,
                                        reason,
                                    ));
                                }
                            };
                            retain_cusp_parallel_fillet_contact(
                                &mut centers,
                                cusp_source,
                                *parallel_source,
                                analytic_support,
                                contact.cusp_parameter(),
                                CurveParameter2::from_selected_fiber(
                                    contact.other_parameter().clone(),
                                ),
                                contact.point_evidence(),
                                contact.location(),
                                contact.tangent_cross_sign(),
                                dot,
                                complementary,
                                cusp_support_reverses_source,
                                cusp_is_previous,
                                domains,
                                incident_domain.as_ref(),
                                cusp_family,
                                analytic_family,
                                policy,
                            )?;
                        }

                        for overlap in overlaps {
                            if retained_selected_fillet_overlap_is_positive(
                                &overlap,
                                &analytic_authored_range,
                                incident_domain.as_ref(),
                                cusp_source,
                                analytic_family,
                                cusp_family,
                                policy,
                            )? {
                                centers.coincident = Some(FilletCenterCoincidence2::Support);
                                break;
                            }
                        }
                    }
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParallelIntersections2::RetainedContacts(contacts) => {
                        for contact in contacts {
                            retain_cusp_parallel_fillet_contact(
                                &mut centers,
                                cusp_source,
                                *parallel_source,
                                analytic_support,
                                contact.cusp_parameter(),
                                contact.other_parameter().clone(),
                                contact.point_evidence(),
                                contact.location(),
                                contact.tangent_cross_sign(),
                                contact.tangent_dot_sign(),
                                complementary,
                                cusp_support_reverses_source,
                                cusp_is_previous,
                                domains,
                                incident_domain.as_ref(),
                                cusp_family,
                                analytic_family,
                                policy,
                            )?;
                        }
                    }

                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped { contacts, overlaps } => {
                        let parameter_map = if contacts
                            .iter()
                            .any(|contact| contact.retained_cusp_parameter().is_none())
                        {
                            Some(match cusp_circle
                            .parallel_parameter_map(analytic_support, policy)
                            .map_err(|cause| {
                                ExactCurveError::invalid(
                                    CurveOperation2::Fillet,
                                    cusp_family,
                                    cause,
                                )
                            })? {
                            Classification::Decided(map) => map,
                            Classification::Uncertain(reason) => {
                                return Err(ExactCurveError::blocked(
                                    CurveOperation2::Fillet,
                                    cusp_family,
                                    reason,
                                ));
                            }
                            })
                        } else {
                            None
                        };
                        for contact in contacts {
                            if (complementary
                                && contact.location
                                    != crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior)
                                || !parallel_source.parameter_is_admissible(
                                &contact.parallel_parameter.clone().into(),
                                !cusp_is_previous,
                                domains[usize::from(cusp_is_previous)],
                                incident_domain.as_ref(),
                                analytic_family,
                                policy,
                            )? {
                                continue;
                            }
                            let cusp_parameter = contact.retained_cusp_parameter().unwrap_or_else(|| {
                                parameter_map
                                    .as_ref()
                                    .expect("an unresolved cusp contact retains its map")
                                    .contact_parameter(&contact)
                            });
                            if cusp_mode != CurveCornerMode2::TrimOrExtend {
                                match cusp_source
                                    .contains_parameter(&cusp_parameter, false, false, policy)
                                    .map_err(|cause| {
                                        ExactCurveError::invalid(
                                            CurveOperation2::Fillet,
                                            cusp_family,
                                            cause,
                                        )
                                    })? {
                                    Classification::Decided(true) => {}
                                    Classification::Decided(false) => continue,
                                    Classification::Uncertain(reason) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            cusp_family,
                                            reason,
                                        ));
                                    }
                                }
                            }
                            let mut cross = contact.tangent_cross_sign.ok_or_else(|| {
                                ExactCurveError::blocked(
                                    CurveOperation2::Fillet,
                                    analytic_family,
                                    crate::UncertaintyReason::Predicate,
                                )
                            })?;
                            let mut dot = match cusp_circle
                                .parallel_contact_tangent_dot_sign(
                                    analytic_support,
                                    &contact,
                                    policy,
                                )
                                .map_err(|cause| {
                                    ExactCurveError::invalid(
                                        CurveOperation2::Fillet,
                                        analytic_family,
                                        cause,
                                    )
                                })? {
                                Classification::Decided(sign) => sign,
                                Classification::Uncertain(reason) => {
                                    return Err(ExactCurveError::blocked(
                                        CurveOperation2::Fillet,
                                        analytic_family,
                                        reason,
                                    ));
                                }
                            };
                            if cusp_support_reverses_source {
                                cross = reverse_fillet_sign(cross);
                                dot = reverse_fillet_sign(dot);
                            }
                            let analytic_support_reverses_source = parallel_source.support_reverses_source_at(
                                analytic_support, &contact.parallel_parameter.clone().into(), !cusp_is_previous, analytic_family, policy,
                            )?;
                            if analytic_support_reverses_source == Some(true) {
                                cross = reverse_fillet_sign(cross);
                                dot = reverse_fillet_sign(dot);
                            }
                            let point = match cusp_parameter
                                .coincident_point_evidence(cusp_circle, policy)
                                .map_err(|cause| {
                                    ExactCurveError::invalid(
                                        CurveOperation2::Fillet,
                                        cusp_family,
                                        cause,
                                    )
                                })? {
                                Classification::Decided(Some(point)) => point,
                                Classification::Decided(None) => {
                                    return Err(ExactCurveError::blocked(
                                        CurveOperation2::Fillet,
                                        cusp_family,
                                        crate::UncertaintyReason::Unsupported,
                                    ));
                                }
                                Classification::Uncertain(reason) => {
                                    return Err(ExactCurveError::blocked(
                                        CurveOperation2::Fillet,
                                        cusp_family,
                                        reason,
                                    ));
                                }
                            };
                            let cusp_parameter = if complementary {
                                CurveParameter2::from_algebraic_cusp_complement(
                                    cusp_parameter,
                                )
                            } else {
                                CurveParameter2::from_algebraic_cusp(cusp_parameter)
                            };
                            let analytic_parameter = CurveParameter2::from(
                                contact.parallel_parameter,
                            );
                            let (previous_parameter, next_parameter) = if cusp_is_previous {
                                (Some(cusp_parameter), Some(analytic_parameter.clone()))
                            } else {
                                (Some(analytic_parameter.clone()), Some(cusp_parameter))
                            };
                            centers.push(FilletCenterWitness2 {
                                source_frames: [None, None],
                                point,
                                previous_parameter,
                                next_parameter,
                                retained_anchor_evidence: Some(
                                    RetainedFilletAnchorEvidence2 {
                                        cross: analytic_support_reverses_source.map(|_| reverse_fillet_sign(cross)),
                                        dot: analytic_support_reverses_source.map(|_| dot),
                                        center_parallel: Some(RetainedFilletCenterParallel2 {
                                            support: analytic_support.clone(),
                                            parameter: Some(analytic_parameter),
                                        }),
                                        source_direction: analytic_support_reverses_source.map(|reversed| if reversed { RealSign::Negative } else { RealSign::Positive }),
                                        canonical_anchor_curve: None,
                                        deferred_arc_contact: None,
                                    },
                                ),
                            });
                        }

                        for overlap in overlaps {
                            let overlaps_authored = retained_fillet_cusp_mapped_overlap_is_positive(
                                cusp_source,
                                &overlap,
                                &analytic_authored_range,
                                cusp_family,
                                policy,
                            )?;
                            let overlaps_incident = if let Some(domain) = incident_domain.as_ref() {
                                let other_overlap = overlap.other_range().clone();
                                let incident_range = retained_fillet_incident_overlap_range(
                                    &other_overlap,
                                    domain,
                                    analytic_family,
                                    policy,
                                )?;
                                match incident_range {
                                    Some(incident_range) => retained_fillet_cusp_mapped_overlap_is_positive(
                                        cusp_source,
                                        &overlap,
                                        &incident_range,
                                        cusp_family,
                                        policy,
                                    )?,
                                    None => false,
                                }
                            } else {
                                false
                            };
                            if overlaps_authored || overlaps_incident {
                                centers.coincident = Some(FilletCenterCoincidence2::Support);
                                break;
                            }
                        }
                    }

                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParallelIntersections2::CoincidentCircleComponent => {
                        centers.coincident = Some(FilletCenterCoincidence2::Support);
                    }
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            cusp_family,
                            crate::UncertaintyReason::Predicate,
                        ));
                    }
                }
            }
            return Ok(centers);
        }
        (FilletOffsetCarrier2::Arc { .. }, FilletOffsetCarrier2::AlgebraicCusp { .. })
        | (FilletOffsetCarrier2::AlgebraicCusp { .. }, FilletOffsetCarrier2::Arc { .. }) => {
            let (arc, source_radius, signed_radius, cusp_source, cusp, arc_is_previous) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::Arc {
                            source,
                            source_radius,
                            signed_radius,
                        },
                        FilletOffsetCarrier2::AlgebraicCusp {
                            source: cusp_source,
                            support,
                        },
                    ) => (
                        (*source),
                        *source_radius,
                        signed_radius,
                        *cusp_source,
                        support,
                        true,
                    ),
                    (
                        FilletOffsetCarrier2::AlgebraicCusp {
                            source: cusp_source,
                            support,
                        },
                        FilletOffsetCarrier2::Arc {
                            source,
                            source_radius,
                            signed_radius,
                        },
                    ) => (
                        (*source),
                        *source_radius,
                        signed_radius,
                        *cusp_source,
                        support,
                        false,
                    ),
                    _ => unreachable!(),
                };
            let arc_family = if arc_is_previous {
                previous_family
            } else {
                next_family
            };
            let cusp_family = if arc_is_previous {
                next_family
            } else {
                previous_family
            };
            let offset_support = arc.concentric_offset_support(
                source_radius,
                signed_radius,
                CurveOperation2::Fillet,
                arc_family,
            )?;
            let signed_radius_sign = match crate::classify::real_sign(signed_radius, policy) {
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Some(RealSign::Zero) => unreachable!("collapsed arc offsets use the point carrier"),
                None => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        arc_family,
                        crate::UncertaintyReason::RealSign,
                    ));
                }
            };
            let cusp_support_reverses_source = retained_fillet_cusp_support_reverses_source(
                cusp_source,
                cusp,
                cusp_family,
                policy,
            )?;
            // A concentric arc offset is one exact circle, independent of the
            // authored rational parameterization. Give it a fixed +x radial
            // chart and solve both half charts against the selected cusp in
            // the same rank-independent circle-pair authority used by every
            // selected/selected carrier combination. The rational arc cells
            // below survive only as deferred inverse-domain maps.
            let center = arc.support().center();
            let axis_source = QuadraticBezier2::new(
                center.clone(),
                center.translated(Real::zero(), Real::from(-1_i8)),
                center.translated(Real::zero(), Real::from(-2_i8)),
            );
            let axis_parallel = axis_source.parallel_left(Real::zero()).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, arc_family, cause)
            })?;
            let offset_circle = match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                axis_parallel,
                BezierParameter2::Exact(Real::zero()).into(),
                signed_radius.clone(),
                arc.support().is_clockwise(),
                policy,
            )
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, arc_family, cause)
            })? {
                Classification::Decided(Some(circle)) => circle,
                Classification::Decided(None) => {
                    unreachable!("the nonzero concentric arc offset defines a circle")
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        arc_family,
                        reason,
                    ));
                }
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-fillet-arc-algebraic-cusp",
                "selected-circle-pair-authority",
            );

            let offset_complement = offset_circle.complementary_half();
            let arc_mode = domains[usize::from(!arc_is_previous)].mode();
            let cusp_mode = domains[usize::from(arc_is_previous)].mode();
            let cusp_complement = (cusp_mode == CurveCornerMode2::TrimOrExtend)
                .then(|| cusp.semicircle().complementary_half());
            let cusp_charts = [
                Some((cusp.semicircle(), false)),
                cusp_complement.as_ref().map(|circle| (circle, true)),
            ];
            let chart_owns_endpoint = |complementary: bool, location| {
                !complementary
                    || location
                        == crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior
            };
            let endpoint_parameter = |location| match location {
                crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Start => {
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(
                        Real::zero(),
                    )
                }
                crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::End => {
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
                }
                crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior => {
                    unreachable!("endpoint-only pair contact was interior")
                }
            };
            let reverse_pair_relation =
                cusp_support_reverses_source != (signed_radius_sign == RealSign::Negative);
            for (arc_circle, arc_complementary) in
                [(&offset_circle, false), (&offset_complement, true)]
            {
                let radial = arc_circle.radial_distance();
                let rational_half = CircularArc2::try_from_center(
                    center.translated(radial.clone(), Real::zero()),
                    center.translated(-radial.clone(), Real::zero()),
                    center.clone(),
                    arc_circle.is_clockwise(),
                )
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, arc_family, cause)
                })?;
                for (cusp_circle, cusp_complementary) in cusp_charts.iter().flatten().copied() {
                    let intersections = match arc_circle
                        .pair_intersections(cusp_circle, policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, arc_family, cause)
                        })? {
                        Classification::Decided(intersections) => intersections,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                arc_family,
                                reason,
                            ));
                        }
                    };
                    let mut chart_contacts = Vec::with_capacity(2);
                    match intersections {
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts => {}
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Contacts {
                            contacts,
                            parameter_map,
                        } => {
                            for contact in contacts {
                                if !chart_owns_endpoint(
                                    arc_complementary,
                                    contact.first_location,
                                ) || !chart_owns_endpoint(
                                    cusp_complementary,
                                    contact.second_location,
                                ) {
                                    continue;
                                }
                                let tangent_cross = contact.tangent_cross_sign;
                                let tangent_dot = if tangent_cross == RealSign::Zero {
                                    Some(match parameter_map
                                        .tangent_dot_sign(&contact, policy)
                                        .map_err(|cause| {
                                            ExactCurveError::invalid(
                                                CurveOperation2::Fillet,
                                                arc_family,
                                                cause,
                                            )
                                        })? {
                                        Classification::Decided(sign) => sign,
                                        Classification::Uncertain(reason) => {
                                            return Err(ExactCurveError::blocked(
                                                CurveOperation2::Fillet,
                                                arc_family,
                                                reason,
                                            ));
                                        }
                                    })
                                } else {
                                    None
                                };
                                let arc_parameter =
                                    parameter_map.first_contact_parameter(&contact);
                                let cusp_parameter =
                                    parameter_map.second_contact_parameter(&contact);
                                let direct_point = match arc_parameter
                                    .coincident_point_evidence(arc_circle, policy)
                                    .map_err(|cause| {
                                        ExactCurveError::invalid(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            cause,
                                        )
                                    })? {
                                    Classification::Decided(Some(point)) => point,
                                    Classification::Decided(None) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            crate::UncertaintyReason::Unsupported,
                                        ));
                                    }
                                    Classification::Uncertain(reason) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            reason,
                                        ));
                                    }
                                };
                                let point = retained_fillet_pair_contact_rational_point_on_arc(
                                    &rational_half,
                                    &parameter_map,
                                    &contact,
                                    true,
                                    arc_family,
                                    policy,
                                )?
                                .unwrap_or(direct_point);
                                chart_contacts.push((
                                    arc_parameter,
                                    cusp_parameter,
                                    tangent_cross,
                                    tangent_dot,
                                    point,
                                ));
                            }
                        }
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::EndpointContacts(contacts) => {
                            for contact in contacts {
                                if !chart_owns_endpoint(
                                    arc_complementary,
                                    contact.first_location,
                                ) || !chart_owns_endpoint(
                                    cusp_complementary,
                                    contact.second_location,
                                ) {
                                    continue;
                                }
                                let arc_parameter = endpoint_parameter(contact.first_location);
                                let cusp_parameter = endpoint_parameter(contact.second_location);
                                let point = match arc_parameter
                                    .coincident_point_evidence(arc_circle, policy)
                                    .map_err(|cause| {
                                        ExactCurveError::invalid(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            cause,
                                        )
                                    })? {
                                    Classification::Decided(Some(point)) => point,
                                    Classification::Decided(None) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            crate::UncertaintyReason::Unsupported,
                                        ));
                                    }
                                    Classification::Uncertain(reason) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            reason,
                                        ));
                                    }
                                };
                                let tangent_cross = contact.tangent_cross_sign;
                                let tangent_dot = if tangent_cross == RealSign::Zero {
                                    let endpoint_tangent = |circle: &crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
                                                            location|
                                     -> ExactCurveResult<crate::BezierAlgebraicChord2> {
                                        let start = match location {
                                            crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Start => true,
                                            crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::End => false,
                                            crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior => unreachable!("endpoint-only pair contact was interior"),
                                        };
                                        match crate::BezierAlgebraicCuspSemicircleFragment2::full(
                                            circle.clone(),
                                            policy,
                                        )
                                            .endpoint_tangent_chord(start, policy)
                                            .map_err(|cause| {
                                                ExactCurveError::invalid(
                                                    CurveOperation2::Fillet,
                                                    arc_family,
                                                    cause,
                                                )
                                            })? {
                                            Classification::Decided(Some(chord)) => Ok(chord),
                                            Classification::Decided(None) => Err(
                                                ExactCurveError::blocked(
                                                    CurveOperation2::Fillet,
                                                    arc_family,
                                                    crate::UncertaintyReason::Unsupported,
                                                ),
                                            ),
                                            Classification::Uncertain(reason) => Err(
                                                ExactCurveError::blocked(
                                                    CurveOperation2::Fillet,
                                                    arc_family,
                                                    reason,
                                                ),
                                            ),
                                        }
                                    };
                                    let first = endpoint_tangent(
                                        arc_circle,
                                        contact.first_location,
                                    )?;
                                    let second = endpoint_tangent(
                                        cusp_circle,
                                        contact.second_location,
                                    )?;
                                    Some(match first
                                        .tangent_dot_sign(&second, policy)
                                        .map_err(|cause| {
                                            ExactCurveError::invalid(
                                                CurveOperation2::Fillet,
                                                arc_family,
                                                cause,
                                            )
                                        })? {
                                        Classification::Decided(sign) => sign,
                                        Classification::Uncertain(reason) => {
                                            return Err(ExactCurveError::blocked(
                                                CurveOperation2::Fillet,
                                                arc_family,
                                                reason,
                                            ));
                                        }
                                    })
                                } else {
                                    None
                                };
                                chart_contacts.push((
                                    arc_parameter,
                                    cusp_parameter,
                                    tangent_cross,
                                    tangent_dot,
                                    point,
                                ));
                            }
                        }
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(_) => {
                            centers.coincident = (previous_mode == CurveCornerMode2::TrimOrExtend
                                || next_mode == CurveCornerMode2::TrimOrExtend
                                || retained_fillet_arc_cusp_overlap_is_positive(
                                    &offset_support,
                                    cusp,
                                    arc_family,
                                    cusp_family,
                                    policy,
                                )?).then_some(FilletCenterCoincidence2::Support);
                            return Ok(centers);
                        }
                    }
                    for (
                        arc_parameter,
                        cusp_parameter,
                        mut tangent_cross,
                        mut tangent_dot,
                        point,
                    ) in chart_contacts
                    {
                        if cusp_mode == CurveCornerMode2::TrimOnly {
                            match cusp
                                .contains_parameter(&cusp_parameter, false, false, policy)
                                .map_err(|cause| {
                                    ExactCurveError::invalid(
                                        CurveOperation2::Fillet,
                                        cusp_family,
                                        cause,
                                    )
                                })? {
                                Classification::Decided(true) => {}
                                Classification::Decided(false) => {
                                    centers.outside_domain = true;
                                    continue;
                                }
                                Classification::Uncertain(reason) => {
                                    return Err(ExactCurveError::blocked(
                                        CurveOperation2::Fillet,
                                        cusp_family,
                                        reason,
                                    ));
                                }
                            }
                        }
                        let Some(contact_seed) = retained_arc_fillet_contact_seed(
                            arc.support(),
                            arc_circle,
                            &arc_parameter,
                            source_radius,
                            signed_radius,
                            arc_mode,
                            arc_family,
                            policy,
                        )?
                        else {
                            centers.outside_domain = true;
                            continue;
                        };
                        if reverse_pair_relation {
                            tangent_cross = reverse_fillet_sign(tangent_cross);
                            tangent_dot = tangent_dot.map(reverse_fillet_sign);
                        }
                        let cusp_parameter = if cusp_complementary {
                            CurveParameter2::from_algebraic_cusp_complement(cusp_parameter)
                        } else {
                            CurveParameter2::from_algebraic_cusp(cusp_parameter)
                        };
                        let (previous_parameter, next_parameter) = if arc_is_previous {
                            (None, Some(cusp_parameter))
                        } else {
                            (Some(cusp_parameter), None)
                        };
                        centers.push(FilletCenterWitness2 {
                            source_frames: [None, None],
                            point,
                            previous_parameter,
                            next_parameter,
                            retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                                // The common pair authority reports offset
                                // arc x selected cusp. The two exact source
                                // reversal factors above map that relation to
                                // the authored arc anchor and cusp traversal.
                                cross: Some(tangent_cross),
                                dot: tangent_dot,
                                center_parallel: None,
                                source_direction: None,
                                canonical_anchor_curve: None,
                                deferred_arc_contact: Some(RetainedDeferredArcFilletContact2 {
                                    source: ExactCornerArc2::clone(arc),
                                    source_radius: source_radius.clone(),
                                    signed_center_radius: signed_radius.clone(),
                                    arc_is_previous,
                                    domain: domains[usize::from(!arc_is_previous)],
                                    selected_center: matches!(
                                        &arc_parameter,
                                        crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Mapped(_)
                                    )
                                    .then_some(arc_parameter),
                                    contact_seed: Some(contact_seed),
                                }),
                            }),
                        });
                    }
                }
            }
        }
        (FilletOffsetCarrier2::Line { .. }, FilletOffsetCarrier2::AlgebraicCusp { .. })
        | (FilletOffsetCarrier2::AlgebraicCusp { .. }, FilletOffsetCarrier2::Line { .. }) => {
            let (line_source, line_support, line_signed_distance, line_is_previous) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::Line {
                            source,
                            support,
                            signed_distance,
                            ..
                        },
                        FilletOffsetCarrier2::AlgebraicCusp {
                            source: _,
                            support: _,
                        },
                    ) => (source, support, signed_distance, true),
                    (
                        FilletOffsetCarrier2::AlgebraicCusp {
                            source: _,
                            support: _,
                        },
                        FilletOffsetCarrier2::Line {
                            source,
                            support,
                            signed_distance,
                            ..
                        },
                    ) => (source, support, signed_distance, false),
                    _ => unreachable!(),
                };
            // Lower every represented line to the authoritative chord cell.
            // Native-line witnesses retain their authored finite domain for
            // TrimOnly. A canonical witness for an algebraic chord names only
            // its complete affine support; final cut publication reclassifies
            // the contact on the authored source chord. TrimOrExtend selects
            // both selected-circle charts and complete support in either case.
            let line_family = if line_is_previous {
                previous_family
            } else {
                next_family
            };
            let source_chord = match line_source.algebraic_chord() {
                Some(source) => source.clone(),
                None => algebraic_chord_from_line_support(
                    line_source
                        .native_line()
                        .expect("a represented line carrier retains one source line"),
                    CurveOperation2::Fillet,
                    line_family,
                    policy,
                )?,
            };
            let retained_support = source_chord
                .parallel_left_retained(line_signed_distance.clone(), policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, line_family, cause)
                })?;
            let represented_line_names_finite_domain = line_source.native_line().is_some()
                || algebraic_chord_domain_matches_line_witness(
                    &retained_support,
                    line_support,
                    line_family,
                    policy,
                )?;
            let mode = domains[usize::from(!line_is_previous)].mode();
            // Keep canonical-Real line endpoints as the intersection hot
            // path while retaining the procedural normal-offset support as
            // ancestry. The former selects the compact selected-fiber line
            // kernel; the latter is the exact tangent relation needed when a
            // recursively selected terminal circle is reconstructed.
            let (support_chord, finite_source_domain) =
                if mode == CurveCornerMode2::TrimOnly && !represented_line_names_finite_domain {
                    // A canonical represented line names only the infinite
                    // support of this algebraic chord. For TrimOnly, intersect
                    // the translated authored chord itself so the common kernel
                    // owns and retains both exact finite-boundary inequalities.
                    (retained_support.clone(), true)
                } else {
                    let support = retained_support
                        .chord_between_certified_ordered_support_points(
                            CurvePoint2::from(line_support.start().clone()),
                            CurvePoint2::from(line_support.end().clone()),
                            policy,
                        )
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, line_family, cause)
                        })?;
                    (support, represented_line_names_finite_domain)
                };
            let promoted = FilletOffsetCarrier2::AlgebraicChord {
                source: &source_chord,
                support: support_chord,
                signed_distance: line_signed_distance.clone(),
                finite_source_domain,
            };
            let mut promoted_centers = if line_is_previous {
                fillet_offset_centers(
                    &promoted,
                    next,
                    domains,
                    previous_family,
                    next_family,
                    normal_constraints,
                    policy,
                )
            } else {
                fillet_offset_centers(
                    previous,
                    &promoted,
                    domains,
                    previous_family,
                    next_family,
                    normal_constraints,
                    policy,
                )
            }?;
            // Retain the promoted contact parameter needed by the original
            // carrier. Native lines reuse the recursive quadratic scalar;
            // algebraic chords retain contact provenance while their final
            // cut is classified on the authored source domain.
            for center in promoted_centers.iter_mut() {
                let line_parameter = if line_is_previous {
                    center.previous_parameter.as_ref()
                } else {
                    center.next_parameter.as_ref()
                };
                let recursive_line_parameter = line_parameter
                    .and_then(CurveParameter2::as_algebraic_chord)
                    .and_then(|parameter| parameter.point().as_algebraic_cusp_chord())
                    .and_then(|point| point.recursive_quadratic_line_parameter())
                    .map(CurveParameter2::from_recursive_projective);
                let retained_line_parameter = finite_source_domain
                    .then(|| {
                        recursive_line_parameter.or_else(|| {
                            line_source
                                .algebraic_chord()
                                .and_then(|_| line_parameter.cloned())
                        })
                    })
                    .flatten();
                if line_is_previous {
                    center.previous_parameter = retained_line_parameter;
                } else {
                    center.next_parameter = retained_line_parameter;
                }
            }
            return Ok(promoted_centers);
        }
        (
            FilletOffsetCarrier2::AlgebraicCusp {
                source: previous_source,
                support: previous_support,
            },
            FilletOffsetCarrier2::AlgebraicCusp {
                source: next_source,
                support: next_support,
            },
        ) => {
            let previous_support_reverses_source = retained_fillet_cusp_support_reverses_source(
                previous_source,
                previous_support,
                previous_family,
                policy,
            )?;
            let next_support_reverses_source = retained_fillet_cusp_support_reverses_source(
                next_source,
                next_support,
                next_family,
                policy,
            )?;
            let reverse_pair_relation =
                previous_support_reverses_source != next_support_reverses_source;
            let witness = |previous_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
                           previous_complementary: bool,
                           next_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
                           next_complementary: bool,
                           point: CurvePoint2,
                           retained_anchor_evidence: Option<RetainedFilletAnchorEvidence2>|
             -> ExactCurveResult<Option<FilletCenterWitness2>> {
                for (source, parameter, family, mode) in [
                    (previous_source, &previous_parameter, previous_family, previous_mode),
                    (next_source, &next_parameter, next_family, next_mode),
                ] {
                    if mode == CurveCornerMode2::TrimOrExtend {
                        continue;
                    }
                    match source
                        .contains_parameter(parameter, false, false, policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                        })? {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) => return Ok(None),
                        // This is only a conservative fast rejection. The
                        // authoritative cut classifier below owns every
                        // case this cheaper parameter predicate cannot
                        // decide exactly.
                        Classification::Uncertain(_) => {}
                    }
                }
                let previous_parameter = if previous_complementary {
                    CurveParameter2::from_algebraic_cusp_complement(previous_parameter)
                } else {
                    CurveParameter2::from_algebraic_cusp(previous_parameter)
                };
                let next_parameter = if next_complementary {
                    CurveParameter2::from_algebraic_cusp_complement(next_parameter)
                } else {
                    CurveParameter2::from_algebraic_cusp(next_parameter)
                };
                Ok(Some(FilletCenterWitness2 {
                    source_frames: [None, None],
                    point,
                    previous_parameter: Some(previous_parameter),
                    next_parameter: Some(next_parameter),
                    retained_anchor_evidence,
                }))
            };
            let previous_complement = (previous_mode == CurveCornerMode2::TrimOrExtend)
                .then(|| previous_support.semicircle().complementary_half());
            let next_complement = (next_mode == CurveCornerMode2::TrimOrExtend)
                .then(|| next_support.semicircle().complementary_half());
            let previous_charts = [
                Some((previous_support.semicircle(), false)),
                previous_complement.as_ref().map(|circle| (circle, true)),
            ];
            let next_charts = [
                Some((next_support.semicircle(), false)),
                next_complement.as_ref().map(|circle| (circle, true)),
            ];
            let chart_owns_endpoint = |complementary: bool, location| {
                !complementary
                    || location
                        == crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior
            };
            for (previous_circle, previous_complementary) in
                previous_charts.iter().flatten().copied()
            {
                for (next_circle, next_complementary) in next_charts.iter().flatten().copied() {
                    let intersections = match previous_circle
                        .pair_intersections(next_circle, policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(
                                CurveOperation2::Fillet,
                                previous_family,
                                cause,
                            )
                        })? {
                        Classification::Decided(intersections) => intersections,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                previous_family,
                                reason,
                            ));
                        }
                    };
                    match intersections {
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts => {}
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Contacts {
                            contacts,
                            parameter_map,
                        } => {
                            for contact in contacts {
                                if !chart_owns_endpoint(
                                    previous_complementary,
                                    contact.first_location,
                                ) || !chart_owns_endpoint(
                                    next_complementary,
                                    contact.second_location,
                                ) {
                                    continue;
                                }
                                let mut tangent_cross = contact.tangent_cross_sign;
                                // A transverse contact's oriented cross sign
                                // alone selects the one- or two-half fillet
                                // sweep. Do not ask the represented pair for
                                // an independent high-degree dot predicate
                                // unless the tangents are parallel.
                                let mut tangent_dot = if tangent_cross == RealSign::Zero {
                                    Some(
                                        match parameter_map
                                            .tangent_dot_sign(&contact, policy)
                                            .map_err(|cause| {
                                                ExactCurveError::invalid(
                                                    CurveOperation2::Fillet,
                                                    previous_family,
                                                    cause,
                                                )
                                            })? {
                                            Classification::Decided(sign) => sign,
                                            Classification::Uncertain(reason) => {
                                                return Err(ExactCurveError::blocked(
                                                    CurveOperation2::Fillet,
                                                    previous_family,
                                                    reason,
                                                ));
                                            }
                                        },
                                    )
                                } else {
                                    None
                                };
                                if reverse_pair_relation {
                                    tangent_cross = reverse_fillet_sign(tangent_cross);
                                    tangent_dot = tangent_dot.map(reverse_fillet_sign);
                                }
                                let previous_parameter =
                                    parameter_map.first_contact_parameter(&contact);
                                let next_parameter =
                                    parameter_map.second_contact_parameter(&contact);
                                let direct_point = match previous_parameter
                                    .coincident_point_evidence(previous_circle, policy)
                                    .map_err(|cause| {
                                        ExactCurveError::invalid(
                                            CurveOperation2::Fillet,
                                            previous_family,
                                            cause,
                                        )
                                    })? {
                                    Classification::Decided(Some(point)) => point,
                                    Classification::Decided(None) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            previous_family,
                                            crate::UncertaintyReason::Unsupported,
                                        ));
                                    }
                                    Classification::Uncertain(reason) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            previous_family,
                                            reason,
                                        ));
                                    }
                                };
                                let point = retained_fillet_pair_contact_rational_point(
                                    previous_circle,
                                    &parameter_map,
                                    &contact,
                                    true,
                                    previous_family,
                                    policy,
                                )?
                                .unwrap_or(direct_point);
                                if let Some(witness) = witness(
                                    previous_parameter,
                                    previous_complementary,
                                    next_parameter,
                                    next_complementary,
                                    point,
                                    Some(RetainedFilletAnchorEvidence2 {
                                        cross: Some(tangent_cross),
                                        dot: tangent_dot,
                                        center_parallel: None,
                                        source_direction: None,
                                        canonical_anchor_curve: None,
                                        deferred_arc_contact: None,
                                    }),
                                )? {
                                    centers.push(witness);
                                } else {
                                    centers.outside_domain = true;
                                }
                            }
                        }
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::EndpointContacts(contacts) => {
                            let endpoint = |location| match location {
                                crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Start => crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
                                crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::End => crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one()),
                                crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior => unreachable!("endpoint-only pair contact was interior"),
                            };
                            for contact in contacts {
                                if !chart_owns_endpoint(
                                    previous_complementary,
                                    contact.first_location,
                                ) || !chart_owns_endpoint(
                                    next_complementary,
                                    contact.second_location,
                                ) {
                                    continue;
                                }
                                let previous_parameter = endpoint(contact.first_location);
                                let next_parameter = endpoint(contact.second_location);
                                let point = match previous_parameter
                                    .coincident_point_evidence(previous_circle, policy)
                                    .map_err(|cause| {
                                        ExactCurveError::invalid(
                                            CurveOperation2::Fillet,
                                            previous_family,
                                            cause,
                                        )
                                    })? {
                                    Classification::Decided(Some(point)) => point,
                                    Classification::Decided(None) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            previous_family,
                                            crate::UncertaintyReason::Unsupported,
                                        ));
                                    }
                                    Classification::Uncertain(reason) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            previous_family,
                                            reason,
                                        ));
                                    }
                                };
                                if let Some(witness) = witness(
                                    previous_parameter,
                                    previous_complementary,
                                    next_parameter,
                                    next_complementary,
                                    point,
                                    None,
                                )? {
                                    centers.push(witness);
                                } else {
                                    centers.outside_domain = true;
                                }
                            }
                        }
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(overlap) => {
                            if previous_mode == CurveCornerMode2::TrimOrExtend || next_mode == CurveCornerMode2::TrimOrExtend {
                                centers.coincident = Some(FilletCenterCoincidence2::Support);
                                return Ok(centers);
                            }
                            centers.coincident = (retained_fillet_cusp_pair_overlap_is_positive(
                                previous_source,
                                next_source,
                                &overlap,
                                previous_family,
                                policy,
                            )?).then_some(FilletCenterCoincidence2::Support);
                        }
                    }
                }
            }
        }
        (
            FilletOffsetCarrier2::AlgebraicChord {
                source: previous_source,
                support: previous_support,
                signed_distance: previous_distance,
                ..
            },
            FilletOffsetCarrier2::AlgebraicChord {
                source: next_source,
                support: next_support,
                signed_distance: next_distance,
                ..
            },
        ) => {
            let tangent_relation = |cross| {
                let relation = if cross {
                    previous_support.tangent_cross_sign(next_support, policy)
                } else {
                    previous_support.tangent_dot_sign(next_support, policy)
                }
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause)
                })?;
                match relation {
                    Classification::Decided(sign) => Ok(sign),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        previous_family,
                        reason,
                    )),
                }
            };
            let common_corner = previous_source.end();
            let shares_corner = common_corner.shares_storage(next_source.start())
                || common_corner.same_point(next_source.start(), policy)
                    == Classification::Decided(true);
            let common_corner_center = if shares_corner {
                match (
                    previous_source.certified_unit_tangent(),
                    next_source.certified_unit_tangent(),
                ) {
                    (Some(previous_tangent), Some(next_tangent)) => {
                        let offset_origin = |tangent: &(Real, Real), distance: &Real| {
                            Point2::new(-(&tangent.1 * distance), &tangent.0 * distance)
                        };
                        let previous_origin = offset_origin(&previous_tangent, previous_distance);
                        let next_origin = offset_origin(&next_tangent, next_distance);
                        let previous_line = LineSeg2::new_unchecked(
                            previous_origin.clone(),
                            previous_origin.translated(previous_tangent.0, previous_tangent.1),
                        );
                        let next_line = LineSeg2::new_unchecked(
                            next_origin.clone(),
                            next_origin.translated(next_tangent.0, next_tangent.1),
                        );
                        match crate::offset::line_support_intersection(
                            &previous_line,
                            &next_line,
                            policy,
                        )
                        .map_err(|cause| {
                            ExactCurveError::invalid(
                                CurveOperation2::Fillet,
                                previous_family,
                                cause,
                            )
                        })? {
                            Classification::Decided(Some(delta)) => {
                                match crate::BezierAlgebraicChord2::translated_endpoint(
                                    common_corner,
                                    delta.x(),
                                    delta.y(),
                                    policy,
                                )
                                .map_err(|cause| {
                                    ExactCurveError::invalid(
                                        CurveOperation2::Fillet,
                                        previous_family,
                                        cause,
                                    )
                                })? {
                                    Classification::Decided(point) => Some((
                                        point,
                                        tangent_relation(true)?,
                                        tangent_relation(false)?,
                                    )),
                                    Classification::Uncertain(reason) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            previous_family,
                                            reason,
                                        ));
                                    }
                                }
                            }
                            Classification::Decided(None) => None,
                            Classification::Uncertain(reason) => {
                                return Err(ExactCurveError::blocked(
                                    CurveOperation2::Fillet,
                                    previous_family,
                                    reason,
                                ));
                            }
                        }
                    }
                    _ => None,
                }
            } else {
                None
            };
            if let Some((point, tangent_cross, tangent_dot)) = common_corner_center {
                centers.push(FilletCenterWitness2 {
                    source_frames: [None, None],
                    point,
                    previous_parameter: None,
                    next_parameter: None,
                    retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                        cross: Some(tangent_cross),
                        dot: Some(tangent_dot),
                        center_parallel: None,
                        source_direction: None,
                        canonical_anchor_curve: None,
                        deferred_arc_contact: None,
                    }),
                });
                return Ok(centers);
            }
            let tangent_cross = tangent_relation(true)?;
            if tangent_cross == RealSign::Zero {
                let side = match previous_support
                    .oriented_support_side(next_support.start(), policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause)
                    })? {
                    Classification::Decided(side) => side,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            previous_family,
                            reason,
                        ));
                    }
                };
                centers.coincident = (side == crate::classify::LineSide::On)
                    .then_some(FilletCenterCoincidence2::Support);
                return Ok(centers);
            }
            let point = match previous_support
                .supporting_line_intersection(next_support, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause)
                })? {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) => {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Fillet,
                        previous_family,
                        CurveError::Topology(
                            "nonparallel retained fillet supports omitted their intersection"
                                .into(),
                        ),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        previous_family,
                        reason,
                    ));
                }
            };
            centers.push(FilletCenterWitness2 {
                source_frames: [None, None],
                point,
                previous_parameter: None,
                next_parameter: None,
                retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                    cross: Some(tangent_cross),
                    dot: Some(tangent_relation(false)?),
                    center_parallel: None,
                    source_direction: None,
                    canonical_anchor_curve: None,
                    deferred_arc_contact: None,
                }),
            });
        }
        (
            FilletOffsetCarrier2::AlgebraicChord { .. },
            FilletOffsetCarrier2::AlgebraicCusp { .. },
        )
        | (
            FilletOffsetCarrier2::AlgebraicCusp { .. },
            FilletOffsetCarrier2::AlgebraicChord { .. },
        ) => {
            let (chord_support, cusp_source, cusp_support, chord_is_previous, finite_source_domain) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::AlgebraicChord {
                            support,
                            finite_source_domain,
                            ..
                        },
                        FilletOffsetCarrier2::AlgebraicCusp {
                            source,
                            support: cusp,
                        },
                    ) => (support, source, cusp, true, *finite_source_domain),
                    (
                        FilletOffsetCarrier2::AlgebraicCusp {
                            source,
                            support: cusp,
                        },
                        FilletOffsetCarrier2::AlgebraicChord {
                            support,
                            finite_source_domain,
                            ..
                        },
                    ) => (support, source, cusp, false, *finite_source_domain),
                    _ => unreachable!(),
                };
            let chord_family = if chord_is_previous {
                previous_family
            } else {
                next_family
            };
            let cusp_family = if chord_is_previous {
                next_family
            } else {
                previous_family
            };
            let chord_mode = domains[usize::from(!chord_is_previous)].mode();
            let cusp_mode = domains[usize::from(chord_is_previous)].mode();
            let intersections = cusp_support
                .semicircle()
                .chord_intersections_prefer_exact_line(
                    chord_support,
                    finite_source_domain && chord_mode != CurveCornerMode2::TrimOrExtend,
                    policy,
                );
            let base_contacts = match intersections.map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, cusp_family, cause)
            })? {
                Classification::Decided(contacts) => contacts,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        cusp_family,
                        reason,
                    ));
                }
            };
            let mut contacts = base_contacts
                .into_iter()
                .map(|contact| (contact, false))
                .collect::<Vec<_>>();
            if cusp_mode == CurveCornerMode2::TrimOrExtend {
                let complementary = cusp_support.semicircle().complementary_half();
                let complementary_contacts = match complementary
                    .chord_support_intersections(chord_support, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, cusp_family, cause)
                    })? {
                    Classification::Decided(contacts) => contacts,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            cusp_family,
                            reason,
                        ));
                    }
                };
                for contact in complementary_contacts {
                    // Both half charts name their common diameter endpoints.
                    // Keep the authored-half copy so one geometric center is
                    // never emitted twice.
                    let at_start = match contact
                        .cusp_parameter
                        .order_to_real(&Real::zero(), policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, cusp_family, cause)
                        })? {
                        Classification::Decided(order) => order == std::cmp::Ordering::Equal,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                cusp_family,
                                reason,
                            ));
                        }
                    };
                    let at_end = match contact
                        .cusp_parameter
                        .order_to_real(&Real::one(), policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, cusp_family, cause)
                        })? {
                        Classification::Decided(order) => order == std::cmp::Ordering::Equal,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                cusp_family,
                                reason,
                            ));
                        }
                    };
                    if !at_start && !at_end {
                        contacts.push((contact, true));
                    }
                }
            }
            let cusp_support_reverses_source = retained_fillet_cusp_support_reverses_source(
                cusp_source,
                cusp_support,
                cusp_family,
                policy,
            )?;
            for (contact, complementary) in contacts {
                let mut tangent_dot = match contact
                    .tangent_dot_sign(cusp_support.semicircle(), chord_support, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, chord_family, cause)
                    })? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            chord_family,
                            reason,
                        ));
                    }
                };
                if cusp_support_reverses_source {
                    tangent_dot = reverse_fillet_sign(tangent_dot);
                }
                let tangent_cross = if cusp_support_reverses_source {
                    reverse_fillet_sign(contact.tangent_cross_sign)
                } else {
                    contact.tangent_cross_sign
                };
                let cusp_parameter = if complementary {
                    CurveParameter2::from_algebraic_cusp_complement(contact.cusp_parameter)
                } else {
                    CurveParameter2::from_algebraic_cusp(contact.cusp_parameter)
                };
                let chord_parameter =
                    CurveParameter2::from_algebraic_chord(contact.chord_parameter);
                let (previous_parameter, next_parameter) = if chord_is_previous {
                    (Some(chord_parameter), Some(cusp_parameter))
                } else {
                    (Some(cusp_parameter), Some(chord_parameter))
                };
                centers.push(FilletCenterWitness2 {
                    source_frames: [None, None],
                    point: contact.point,
                    previous_parameter,
                    next_parameter,
                    // The selected circle is the reconstruction anchor. The
                    // lower kernel reports exactly circle x chord.
                    retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                        cross: Some(tangent_cross),
                        dot: Some(tangent_dot),
                        center_parallel: None,
                        source_direction: None,
                        canonical_anchor_curve: None,
                        deferred_arc_contact: None,
                    }),
                });
            }
        }
        (FilletOffsetCarrier2::AlgebraicChord { .. }, FilletOffsetCarrier2::Arc { .. })
        | (FilletOffsetCarrier2::Arc { .. }, FilletOffsetCarrier2::AlgebraicChord { .. }) => {
            let (
                chord_source,
                chord_support,
                chord_signed_distance,
                arc,
                source_radius,
                signed_radius,
                chord_is_previous,
            ) = match (previous, next) {
                (
                    FilletOffsetCarrier2::AlgebraicChord {
                        source: chord_source,
                        support,
                        signed_distance,
                        ..
                    },
                    FilletOffsetCarrier2::Arc {
                        source: arc_source,
                        source_radius,
                        signed_radius,
                    },
                ) => (
                    *chord_source,
                    support,
                    signed_distance,
                    *arc_source,
                    *source_radius,
                    signed_radius,
                    true,
                ),
                (
                    FilletOffsetCarrier2::Arc {
                        source,
                        source_radius,
                        signed_radius,
                    },
                    FilletOffsetCarrier2::AlgebraicChord {
                        source: chord_source,
                        support,
                        signed_distance,
                        ..
                    },
                ) => (
                    *chord_source,
                    support,
                    signed_distance,
                    *source,
                    *source_radius,
                    signed_radius,
                    false,
                ),
                _ => unreachable!(),
            };
            let chord_family = if chord_is_previous {
                previous_family
            } else {
                next_family
            };
            let arc_family = if chord_is_previous {
                next_family
            } else {
                previous_family
            };

            // Adjacent retained carriers sometimes name their common vertex
            // in independent exact fields. When STRICT can replay that
            // identity and the chord already owns a represented unit tangent,
            // the common arc endpoint is an exact represented point on the
            // chord support. Translate that point by the authored chord
            // offset and use the primitive line/circle relation. This is the
            // same authoritative geometry as the general selected-axis
            // projection below, but it avoids constructing a three-parameter
            // compositum for the overwhelmingly common incident-corner case.
            let arc_corner = if chord_is_previous {
                arc.support().start()
            } else {
                arc.support().end()
            };
            let chord_corner = if chord_is_previous {
                chord_source.end()
            } else {
                chord_source.start()
            };
            let arc_corner_evidence = CurvePoint2::from(arc_corner.clone());
            let shares_strict_corner = chord_corner.shares_storage(&arc_corner_evidence)
                || chord_corner.same_point(&arc_corner_evidence, &CurveContext::STRICT)
                    == Classification::Decided(true);
            if shares_strict_corner
                && let Some((tangent_x, tangent_y)) = chord_source.certified_unit_tangent()
            {
                let line_start = arc_corner.translated(
                    -(&tangent_y * chord_signed_distance),
                    &tangent_x * chord_signed_distance,
                );
                let line_support = LineSeg2::new_unchecked(
                    line_start.clone(),
                    line_start.translated(tangent_x, tangent_y),
                );
                let relation = crate::intersect::line_circle_relation_from_supports(
                    &line_support,
                    arc.support().center(),
                    &(signed_radius * signed_radius),
                    policy,
                )
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, chord_family, cause)
                })?;
                let mut push = |point: Point2| {
                    centers.push(FilletCenterWitness2 {
                        source_frames: [None, None],
                        point: point.into(),
                        previous_parameter: None,
                        next_parameter: None,
                        retained_anchor_evidence: None,
                    });
                };
                match relation {
                    crate::LineCircleRelation::Disjoint => {}
                    crate::LineCircleRelation::Tangent { point, .. } => push(point),
                    crate::LineCircleRelation::Secant {
                        first_point,
                        second_point,
                        ..
                    } => {
                        push(first_point);
                        push(second_point);
                    }
                    crate::LineCircleRelation::Uncertain { reason } => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            chord_family,
                            reason,
                        ));
                    }
                }
                return Ok(centers);
            }

            let canonical_chord;
            let (intersection_chord, chord_tangent_reversed) = if chord_support.is_reversed() {
                canonical_chord = chord_support.reversed();
                (&canonical_chord, true)
            } else {
                (chord_support, false)
            };
            let arc_is_previous = !chord_is_previous;
            let signed_radius_sign = match crate::classify::real_sign(signed_radius, policy) {
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Some(RealSign::Zero) => {
                    unreachable!("collapsed arc offsets use the point carrier")
                }
                None => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        arc_family,
                        crate::UncertaintyReason::RealSign,
                    ));
                }
            };

            // A concentric arc offset is still exactly one circle. Solve its
            // complete carrier against the retained chord in the common
            // selected-circle/chord kernel instead of projecting the same
            // quadratic incidence independently through every rational arc
            // cell. The chord supplies a compact orthonormal chart; both half
            // charts enumerate the full circle, while the deferred tangent
            // replay below owns the authored/extension arc domain.
            let offset_circle = match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_chord_normal(
                CurvePoint2::from(
                    arc.support().center().clone(),
                ),
                intersection_chord.clone(),
                signed_radius.clone(),
                arc.support().is_clockwise(),
                policy,
            )
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, arc_family, cause)
            })? {
                Classification::Decided(Some(circle)) => circle,
                Classification::Decided(None) => {
                    unreachable!("the nonzero concentric arc offset defines a circle")
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        arc_family,
                        reason,
                    ));
                }
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-fillet-algebraic-chord-arc",
                "selected-circle-authority",
            );
            let mut contacts = Vec::new();
            let chord_mode = domains[usize::from(!chord_is_previous)].mode();
            let arc_mode = domains[usize::from(chord_is_previous)].mode();
            for (circle, complementary) in [
                (offset_circle.clone(), false),
                (offset_circle.complementary_half(), true),
            ] {
                let intersections = if chord_mode == CurveCornerMode2::TrimOrExtend {
                    circle.chord_support_intersections(intersection_chord, policy)
                } else {
                    circle.chord_intersections(intersection_chord, policy)
                }
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, chord_family, cause)
                })?;
                let circle_contacts = match intersections {
                    Classification::Decided(contacts) => contacts,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            chord_family,
                            reason,
                        ));
                    }
                };
                for contact in circle_contacts {
                    if complementary {
                        let at_diameter_endpoint = [Real::zero(), Real::one()]
                            .into_iter()
                            .try_fold(false, |at_endpoint, endpoint| {
                                if at_endpoint {
                                    return Ok(true);
                                }
                                match contact
                                    .cusp_parameter
                                    .order_to_real(&endpoint, policy)
                                    .map_err(|cause| {
                                        ExactCurveError::invalid(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            cause,
                                        )
                                    })? {
                                    Classification::Decided(order) => {
                                        Ok(order == std::cmp::Ordering::Equal)
                                    }
                                    Classification::Uncertain(reason) => {
                                        Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            reason,
                                        ))
                                    }
                                }
                            })?;
                        if at_diameter_endpoint {
                            continue;
                        }
                    }
                    contacts.push((contact, circle.clone()));
                }
            }
            let mut dot = match offset_circle
                .chord_tangent_dot_sign(intersection_chord, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, chord_family, cause)
                })? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        chord_family,
                        reason,
                    ));
                }
            };
            if chord_tangent_reversed {
                dot = reverse_fillet_sign(dot);
            }
            if signed_radius_sign == RealSign::Negative {
                dot = reverse_fillet_sign(dot);
            }
            for (contact, circle) in contacts {
                let Some(contact_seed) = retained_arc_fillet_contact_seed(
                    arc.support(),
                    &circle,
                    &contact.cusp_parameter,
                    source_radius,
                    signed_radius,
                    arc_mode,
                    arc_family,
                    policy,
                )?
                else {
                    continue;
                };
                let mut cross = contact.tangent_cross_sign;
                if chord_tangent_reversed {
                    cross = reverse_fillet_sign(cross);
                }
                if signed_radius_sign == RealSign::Negative {
                    cross = reverse_fillet_sign(cross);
                }
                let chord_parameter =
                    CurveParameter2::from_algebraic_chord(contact.chord_parameter);
                let (previous_parameter, next_parameter) = if chord_is_previous {
                    (Some(chord_parameter), None)
                } else {
                    (None, Some(chord_parameter))
                };
                centers.push(FilletCenterWitness2 {
                    source_frames: [None, None],
                    point: contact.point,
                    previous_parameter,
                    next_parameter,
                    retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                        // The selected circle reports offset-arc x chord.
                        // Signed-radius reversal above maps this back to the
                        // authored arc tangent used to select the fillet sweep.
                        cross: Some(cross),
                        dot: Some(dot),
                        center_parallel: None,
                        source_direction: None,
                        canonical_anchor_curve: None,
                        deferred_arc_contact: Some(RetainedDeferredArcFilletContact2 {
                            source: ExactCornerArc2::clone(arc),
                            source_radius: source_radius.clone(),
                            signed_center_radius: signed_radius.clone(),
                            arc_is_previous,
                            domain: domains[usize::from(!arc_is_previous)],
                            selected_center: None,
                            contact_seed: Some(contact_seed),
                        }),
                    }),
                });
            }
        }
        (FilletOffsetCarrier2::AlgebraicChord { .. }, FilletOffsetCarrier2::Parallel { .. })
        | (FilletOffsetCarrier2::Parallel { .. }, FilletOffsetCarrier2::AlgebraicChord { .. }) => {
            let (
                chord_source,
                chord_support,
                chord_distance,
                parallel_source,
                analytic_support,
                chord_is_previous,
            ) = match (previous, next) {
                (
                    FilletOffsetCarrier2::AlgebraicChord {
                        source: chord_source,
                        support,
                        signed_distance,
                        ..
                    },
                    FilletOffsetCarrier2::Parallel {
                        source,
                        support: analytic,
                    },
                ) => (
                    *chord_source,
                    support,
                    signed_distance,
                    source,
                    analytic,
                    true,
                ),
                (
                    FilletOffsetCarrier2::Parallel {
                        source,
                        support: analytic,
                    },
                    FilletOffsetCarrier2::AlgebraicChord {
                        source: chord_source,
                        support,
                        signed_distance,
                        ..
                    },
                ) => (
                    *chord_source,
                    support,
                    signed_distance,
                    source,
                    analytic,
                    false,
                ),
                _ => unreachable!(),
            };
            let chord_family = if chord_is_previous {
                previous_family
            } else {
                next_family
            };
            let analytic_family = if chord_is_previous {
                next_family
            } else {
                previous_family
            };
            let analytic_is_previous = !chord_is_previous;
            let mode = domains[usize::from(!analytic_is_previous)].mode();
            let incident_domain = if mode == CurveCornerMode2::TrimOrExtend {
                Some(parallel_source.incident_domain(
                    analytic_support,
                    analytic_is_previous,
                    analytic_family,
                    policy,
                )?)
            } else {
                None
            };
            let intersection_result = if let Some(domain) = incident_domain.as_ref() {
                chord_support.parallel_intersections_with_incident_ray(
                    analytic_support,
                    domain,
                    policy,
                )
            } else {
                chord_support.parallel_intersections(analytic_support, policy)
            };
            let intersections = match intersection_result.map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, chord_family, cause)
                })? {
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicChordParallelIntersections2::Contacts(
                        contacts,
                    ),
                ) => contacts,
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicChordParallelIntersections2::CoincidentSupportComponent { sample },
                ) => {
                    let source_line = curve_fillet::coincident_linear_source_chart(
                        chord_source, analytic_support, &sample, chord_distance, chord_family, policy,
                    )?;
                    centers.coincident = Some(FilletCenterCoincidence2::LinearSource(source_line));
                    return Ok(centers);
                }
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                ) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        chord_family,
                        crate::UncertaintyReason::Boundary,
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        chord_family,
                        reason,
                    ));
                }
            };
            for contact in intersections {
                if !parallel_source.parameter_is_admissible(
                    contact.parallel_parameter(),
                    analytic_is_previous,
                    domains[usize::from(!analytic_is_previous)],
                    incident_domain.as_ref(),
                    analytic_family,
                    policy,
                )? {
                    continue;
                }
                let mut cross = contact.tangent_cross_sign();
                let mut dot = contact.tangent_dot_sign();
                let analytic_support_reverses_source = parallel_source.support_reverses_source_at(
                    analytic_support,
                    contact.parallel_parameter(),
                    analytic_is_previous,
                    analytic_family,
                    policy,
                )?;
                if analytic_support_reverses_source == Some(true) {
                    cross = reverse_fillet_sign(cross);
                    dot = reverse_fillet_sign(dot);
                }
                let analytic_parameter = contact.parallel_parameter().clone();
                let (previous_parameter, next_parameter) = if chord_is_previous {
                    (None, Some(analytic_parameter))
                } else {
                    (Some(analytic_parameter), None)
                };
                centers.push(FilletCenterWitness2 {
                    source_frames: [None, None],
                    point: contact.point().clone(),
                    previous_parameter,
                    next_parameter,
                    retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                        // The retained circle frame is analytic, while the
                        // intersection kernel reports chord x analytic.
                        cross: analytic_support_reverses_source.map(|_| reverse_fillet_sign(cross)),
                        dot: analytic_support_reverses_source.map(|_| dot),
                        center_parallel: None,
                        source_direction: analytic_support_reverses_source.map(|reversed| {
                            if reversed {
                                RealSign::Negative
                            } else {
                                RealSign::Positive
                            }
                        }),
                        canonical_anchor_curve: None,
                        deferred_arc_contact: None,
                    }),
                });
            }
        }
        (FilletOffsetCarrier2::AlgebraicChord { .. }, FilletOffsetCarrier2::Line { .. })
        | (FilletOffsetCarrier2::Line { .. }, FilletOffsetCarrier2::AlgebraicChord { .. }) => {
            let (chord_support, line_support, chord_is_previous) = match (previous, next) {
                (
                    FilletOffsetCarrier2::AlgebraicChord { support, .. },
                    FilletOffsetCarrier2::Line { support: line, .. },
                ) => (support, line, true),
                (
                    FilletOffsetCarrier2::Line { support: line, .. },
                    FilletOffsetCarrier2::AlgebraicChord { support, .. },
                ) => (support, line, false),
                _ => unreachable!(),
            };
            let line_family = if chord_is_previous {
                next_family
            } else {
                previous_family
            };
            let line_chord = algebraic_chord_from_line_support(
                line_support,
                CurveOperation2::Fillet,
                line_family,
                policy,
            )?;
            let tangent_relation = |cross| {
                let relation = if cross {
                    line_chord.tangent_cross_sign(chord_support, policy)
                } else {
                    line_chord.tangent_dot_sign(chord_support, policy)
                }
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, line_family, cause)
                })?;
                match relation {
                    Classification::Decided(sign) => Ok(sign),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        line_family,
                        reason,
                    )),
                }
            };
            let tangent_cross = tangent_relation(true)?;
            if tangent_cross == RealSign::Zero {
                let side = match line_chord
                    .oriented_support_side(chord_support.start(), policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, line_family, cause)
                    })? {
                    Classification::Decided(side) => side,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            line_family,
                            reason,
                        ));
                    }
                };
                centers.coincident = (side == crate::classify::LineSide::On)
                    .then_some(FilletCenterCoincidence2::Support);
                return Ok(centers);
            }
            let point = match line_chord
                .supporting_line_intersection(chord_support, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, line_family, cause)
                })? {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) => {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Fillet,
                        line_family,
                        CurveError::Topology(
                            "nonparallel retained line/chord fillet supports omitted their intersection"
                                .into(),
                        ),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        line_family,
                        reason,
                    ));
                }
            };
            centers.push(FilletCenterWitness2 {
                source_frames: [None, None],
                point,
                previous_parameter: None,
                next_parameter: None,
                retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                    // The algebraic chord's exact center frame is shared by
                    // trim and extension; cut classifiers own finite domains.
                    cross: Some(reverse_fillet_sign(tangent_cross)),
                    dot: Some(tangent_relation(false)?),
                    center_parallel: None,
                    source_direction: None,
                    canonical_anchor_curve: None,
                    deferred_arc_contact: None,
                }),
            });
            return Ok(centers);
        }
        (
            FilletOffsetCarrier2::Line {
                source: previous_source,
                support: previous_support,
                ..
            },
            FilletOffsetCarrier2::Line {
                source: next_source,
                support: next_support,
                ..
            },
        ) => {
            // Distinct finite charts need not meet at the authored corner.
            // They share the support intersection and exact affine cut map
            // with retained lines; only connected inputs take the fast path.
            let point = match crate::offset::line_support_intersection(
                previous_support,
                next_support,
                policy,
            )
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause)
            })? {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) => {
                    centers.coincident = (point_on_fillet_offset(
                        &next_support.start().clone().into(),
                        previous,
                        true,
                        domains[0],
                        previous_family,
                        policy,
                    )?)
                    .then_some(FilletCenterCoincidence2::Support);
                    return Ok(centers);
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        previous_family,
                        reason,
                    ));
                }
            };
            let retained_parameter = |source: &FilletLinearSource2<'_>,
                                      support: &LineSeg2,
                                      family: CurveFamily2|
             -> ExactCurveResult<Option<CurveParameter2>> {
                source
                    .native_line()
                    .map(|_| {
                        line_parameter_at_point(support, &point, CurveOperation2::Fillet, family)
                            .map(exact_parameter)
                    })
                    .transpose()
            };
            centers.push(FilletCenterWitness2 {
                source_frames: [None, None],
                previous_parameter: retained_parameter(
                    previous_source,
                    previous_support,
                    previous_family,
                )?,
                next_parameter: retained_parameter(next_source, next_support, next_family)?,
                point: point.into(),
                retained_anchor_evidence: None,
            });
        }
    }
    Ok(centers)
}

pub(super) fn point_on_fillet_offset(
    point: &CurvePoint2,
    support: &FilletOffsetCarrier2<'_, '_>,
    previous: bool,
    domain: FilletContactDomain2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let decided = |classification| match classification {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    };
    match support {
        FilletOffsetCarrier2::Point { point: other, .. } => {
            decided(point.same_point(other, policy))
        }
        FilletOffsetCarrier2::Arc {
            source,
            signed_radius,
            ..
        } => match crate::bezier_offset::retained_point_circle_incidence_sign(
            point,
            source.support().center(),
            &(signed_radius * signed_radius),
            policy,
        )
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
        {
            Classification::Decided(RealSign::Zero) => Ok(true),
            Classification::Decided(RealSign::Positive | RealSign::Negative) => Ok(false),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                reason,
            )),
        },
        FilletOffsetCarrier2::Line { support, .. } => {
            if let Some(point) = point.coordinates() {
                let (dx, dy) = support.delta();
                let from_start = point.delta_from(support.start());
                return crate::classify::is_zero(
                    &(&dx * &from_start.1 - &dy * &from_start.0),
                    policy,
                )
                .ok_or_else(|| {
                    ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        crate::UncertaintyReason::RealSign,
                    )
                });
            }
            let chord = match crate::BezierAlgebraicChord2::try_new(
                CurvePoint2::from(support.start().clone()),
                CurvePoint2::from(support.end().clone()),
                policy,
            )
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
            {
                Classification::Decided(chord) => chord,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        reason,
                    ));
                }
            };
            let side = chord
                .oriented_support_side(point, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                })?;
            decided(side.map(|side| side == LineSide::On))
        }
        FilletOffsetCarrier2::Parallel { source, support } => {
            let incident = if domain.mode() == CurveCornerMode2::TrimOrExtend {
                Some(source.incident_domain(support, previous, family, policy)?)
            } else {
                None
            };
            decided(
                support
                    .contains_point_evidence(
                        point,
                        &source.curve_parameter_range(),
                        incident.as_ref(),
                        policy,
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })?,
            )
        }
        FilletOffsetCarrier2::AlgebraicChord { support, .. } => {
            decided(support.contains_point(point, policy).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
            })?)
        }
        FilletOffsetCarrier2::AlgebraicCusp { support, .. } => {
            if domain.mode() == CurveCornerMode2::TrimOrExtend {
                return decided(
                    support
                        .semicircle()
                        .retained_point_incidence_sign(point, policy)
                        .map(|sign| sign.map(|sign| sign == RealSign::Zero))
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                        })?,
                );
            }
            decided(support.contains_point(point, policy).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
            })?)
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn fillet_cut_from_center(
    offset: &FilletOffsetCarrier2<'_, '_>,
    center: &CurvePoint2,
    retained_parameter: Option<&CurveParameter2>,
    source_frame: Option<&FilletSourceFrame2>,
    deferred_arc_contact: bool,
    previous: bool,
    retain_selected_circle_endpoints: bool,
    domain: FilletContactDomain2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerCut2>> {
    let mode = domain.mode();
    match offset {
        FilletOffsetCarrier2::Line {
            source,
            unit_x,
            unit_y,
            signed_distance,
            ..
        } => {
            if let Some(source) = source.algebraic_chord() {
                return algebraic_chord_fillet_cut_from_center(
                    source,
                    center,
                    retained_parameter,
                    signed_distance,
                    previous,
                    mode,
                    family,
                    policy,
                );
            }
            let source = source
                .native_line()
                .expect("a non-chord linear fillet carrier retains its native line");
            if retained_parameter.is_none() {
                let support = algebraic_chord_from_line_support(
                    source,
                    CurveOperation2::Fillet,
                    family,
                    policy,
                )?;
                let point = support
                    .normal_displaced_point_evidence(
                        center.clone(),
                        -signed_distance.clone(),
                        policy,
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })?;
                if matches!(domain, FilletContactDomain2::AuthoredCurve(_)) {
                    // An unpartitioned line can retain its geometric chord
                    // coordinate. Only transport into an authored multi-chart
                    // source needs the normalized affine scalar below.
                    return algebraic_chord_corner_cut_from_support_point(
                        &support,
                        point,
                        previous,
                        mode,
                        CurveOperation2::Fillet,
                        family,
                        policy,
                    );
                }
                let parameter = support
                    .parameter_at_certified_support_point(point.clone(), policy)
                    .and_then(|parameter| parameter.exact_line_curve_parameter(policy))
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })?;
                let parameter = match parameter {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            family,
                            reason,
                        ));
                    }
                };
                let placement = curve_region_corner_parameter_placement(
                    &parameter,
                    previous,
                    mode,
                    CurveOperation2::Fillet,
                    family,
                    policy,
                )?;
                let placement = domain.with_boundary_contact(
                    placement,
                    &parameter,
                    || if previous { Real::one() } else { Real::zero() }.into(),
                    family,
                    policy,
                )?;
                return Ok(placement.map(|placement| CornerCut2 {
                    point,
                    parameter: Some(parameter),
                    placement,
                }));
            }
            let parameter = retained_parameter
                .expect("a line offset intersection retains its affine parameter")
                .clone();
            let placement = curve_region_corner_parameter_placement(
                &parameter,
                previous,
                mode,
                CurveOperation2::Fillet,
                family,
                policy,
            )?;
            let placement = domain.with_boundary_contact(
                placement,
                &parameter,
                || if previous { Real::one() } else { Real::zero() }.into(),
                family,
                policy,
            )?;
            let Some(placement) = placement else {
                return Ok(None);
            };
            let point = if let Some(parameter) = parameter.scalar() {
                source.point_at(parameter.clone()).into()
            } else {
                {
                    {
                        match crate::BezierAlgebraicChord2::translated_endpoint(
                            center,
                            &(signed_distance * *unit_y),
                            &(-(signed_distance * *unit_x)),
                            policy,
                        )
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                        })? {
                            Classification::Decided(point) => point,
                            Classification::Uncertain(reason) => {
                                return Err(ExactCurveError::blocked(
                                    CurveOperation2::Fillet,
                                    family,
                                    reason,
                                ));
                            }
                        }
                    }
                }
            };
            Ok(Some(CornerCut2 {
                point,
                parameter: Some(parameter),
                placement,
            }))
        }
        FilletOffsetCarrier2::Arc {
            source,
            source_radius,
            signed_radius,
        } => {
            let Some(center) = center.coordinates() else {
                let radial_scale = (*source_radius / signed_radius).map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause.into())
                })?;
                let point = match crate::BezierAlgebraicChord2::scaled_about_point_endpoint(
                    center,
                    source.support().center(),
                    &radial_scale,
                    policy,
                )
                .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
                {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            family,
                            reason,
                        ));
                    }
                };
                return arc_fillet_cut_from_incident_point(
                    source,
                    point,
                    deferred_arc_contact,
                    previous,
                    domain,
                    family,
                    policy,
                );
            };
            let scale = (*source_radius / signed_radius).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, family, cause.into())
            })?;
            let support = source.support();
            let radial = center.delta_from(support.center());
            let point = source
                .support()
                .center()
                .translated(&radial.0 * &scale, &radial.1 * scale);
            arc_fillet_cut_from_incident_point(
                source,
                point.into(),
                deferred_arc_contact,
                previous,
                domain,
                family,
                policy,
            )
        }
        FilletOffsetCarrier2::Point { .. } => {
            unreachable!("a collapsed arc offset has no isolated tangency contact")
        }
        FilletOffsetCarrier2::Parallel { source, support } => {
            let parameter = retained_parameter
                .expect("a parallel offset intersection retains its parameter")
                .clone();
            let Some(placement) =
                source.parameter_placement(&parameter, previous, domain, family, policy)?
            else {
                return Ok(None);
            };
            if let Some(frame) = source_frame {
                let point = frame
                    .tangent
                    .normal_displaced_point_evidence(
                        center.clone(),
                        source.parallel_distance() - support.distance(),
                        policy,
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })?;
                let parameter = match source {
                    FilletParallelSource2::Direct(source) => source.curve_parameter(
                        &parameter,
                        CurveOperation2::Fillet,
                        family,
                        policy,
                    )?,
                    _ => parameter,
                };
                return Ok(Some(CornerCut2 {
                    point,
                    parameter: Some(parameter),
                    placement,
                }));
            }
            let (point, parameter) = match source {
                FilletParallelSource2::Direct(source) => (
                    curve_region_parallel_point_evidence(
                        support,
                        &parameter,
                        true,
                        CurveOperation2::Fillet,
                        family,
                        policy,
                    )?,
                    source.curve_parameter(&parameter, CurveOperation2::Fillet, family, policy)?,
                ),
                FilletParallelSource2::Retained(source) => (
                    curve_region_parallel_point_evidence(
                        source.parallel(),
                        &parameter,
                        false,
                        CurveOperation2::Fillet,
                        family,
                        policy,
                    )?,
                    parameter,
                ),
                FilletParallelSource2::Selected(source) => (
                    curve_region_parallel_point_evidence(
                        &source.parallel_carrier(),
                        &parameter,
                        false,
                        CurveOperation2::Fillet,
                        family,
                        policy,
                    )?,
                    parameter,
                ),
            };
            Ok(Some(CornerCut2 {
                point,
                parameter: Some(parameter),
                placement,
            }))
        }
        FilletOffsetCarrier2::AlgebraicChord {
            source,
            signed_distance,
            ..
        } => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-fillet-algebraic-chord-cut",
                match retained_parameter {
                    Some(parameter) if parameter.is_algebraic_chord() => "algebraic-chord",
                    Some(_) => "other",
                    None => "missing",
                },
            );
            algebraic_chord_fillet_cut_from_center(
                source,
                center,
                retained_parameter,
                signed_distance,
                previous,
                mode,
                family,
                policy,
            )
        }
        FilletOffsetCarrier2::AlgebraicCusp { source, support } => {
            let retained_parameter = retained_parameter
                .expect("a selected-circle offset contact retains its local parameter");
            let complementary = retained_parameter.is_algebraic_cusp_complement();
            let parameter = retained_parameter
                .as_algebraic_cusp()
                .cloned()
                .ok_or_else(|| {
                    ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        crate::UncertaintyReason::Unsupported,
                    )
                })?;
            let strict_support_interior = if complementary {
                Classification::Decided(false)
            } else {
                let translated_pair_interior = source
                    .translated_pair_parameter_is_strict_interior(&parameter, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })?;
                match translated_pair_interior {
                    Some(Classification::Decided(interior)) => Classification::Decided(interior),
                    Some(Classification::Uncertain(_)) | None => {
                        use crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::Interior;
                        match policy
                            .strict_predicate_pass(|| {
                                support.certified_incident_point_evidence_location(
                                    &parameter, center, policy,
                                )
                            })
                            .map_err(|cause| {
                                ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                            })? {
                            Classification::Decided(location) => {
                                Classification::Decided(location == Interior)
                            }
                            Classification::Uncertain(_) => support
                                .certified_incident_point_evidence_is_strict_interior(
                                    center, policy,
                                )
                                .map_err(|cause| {
                                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                                })?,
                        }
                    }
                }
            };
            let placement = if complementary {
                if mode == CurveCornerMode2::TrimOrExtend {
                    CornerPlacement2::Extension
                } else {
                    return Ok(None);
                }
            } else if strict_support_interior == Classification::Decided(true) {
                CornerPlacement2::Trim
            } else if strict_support_interior == Classification::Decided(false)
                && mode != CurveCornerMode2::TrimOrExtend
            {
                // `support` is the concentric offset image of `source` over
                // the identical angular range. The center constructor has
                // already certified circle incidence, so a decided failure
                // of strict support-fragment interior proves that this
                // candidate is either an endpoint or exterior on the source
                // as well. TrimOnly rejects both; comparing independently
                // represented angular parameters cannot add information.
                return Ok(None);
            } else {
                use crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::{
                    End, Exterior, Interior, Start,
                };
                // Incidence on the offset circle is already certified by the
                // intersection constructor. Its endpoint chord is therefore
                // the cheapest exact finite-domain authority and avoids
                // rebuilding two unrelated dense angular fields for a
                // retained smooth run. Parameter comparison remains the
                // complete fallback when that structural certificate cannot
                // decide.
                let incident_location = if retain_selected_circle_endpoints {
                    Some(
                        support
                            .certified_incident_point_evidence_location(&parameter, center, policy)
                            .map_err(|cause| {
                                ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                            })?,
                    )
                } else {
                    None
                };
                match incident_location {
                    Some(Classification::Decided(Interior)) => CornerPlacement2::Trim,
                    Some(Classification::Decided(Start | End | Exterior))
                        if mode == CurveCornerMode2::TrimOrExtend =>
                    {
                        CornerPlacement2::Extension
                    }
                    Some(Classification::Decided(Start | End | Exterior)) => return Ok(None),
                    Some(Classification::Uncertain(_))
                        if mode == CurveCornerMode2::TrimOrExtend =>
                    {
                        // A logical selected-circle run is solved on one
                        // ancestral full support, then rebound to its authored
                        // fragments by the retained coincident-circle overlap
                        // map. Hand off an unresolved finite-domain candidate
                        // immediately: the run rebinder is its authoritative
                        // exact domain owner, while dense angular refinement
                        // here would duplicate that decision.
                        CornerPlacement2::Extension
                    }
                    Some(Classification::Uncertain(_)) | None => {
                        let source_contains = source
                            .contains_parameter(&parameter, false, false, policy)
                            .map_err(|cause| {
                                ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                            })?;
                        match source_contains {
                        Classification::Decided(true) => CornerPlacement2::Trim,
                        Classification::Decided(false)
                            if retain_selected_circle_endpoints
                                && mode == CurveCornerMode2::TrimOrExtend =>
                        {
                            CornerPlacement2::Extension
                        }
                        Classification::Decided(false) => {
                            let endpoint_order = |endpoint| {
                                parameter
                                    .cmp_by_refinement(endpoint, policy)
                                    .map_err(|cause| {
                                        ExactCurveError::invalid(
                                            CurveOperation2::Fillet,
                                            family,
                                            cause,
                                        )
                                    })
                                    .and_then(|order| match order {
                                        Classification::Decided(order) => Ok(order),
                                        Classification::Uncertain(reason) => {
                                            Err(ExactCurveError::blocked(
                                                CurveOperation2::Fillet,
                                                family,
                                                reason,
                                            ))
                                        }
                                    })
                            };
                            let start_order = endpoint_order(source.start_parameter())?;
                            let end_order = endpoint_order(source.end_parameter())?;
                            if start_order == std::cmp::Ordering::Equal
                                || end_order == std::cmp::Ordering::Equal
                            {
                                return Ok(None);
                            }
                            if mode == CurveCornerMode2::TrimOrExtend {
                                CornerPlacement2::Extension
                            } else {
                                return Ok(None);
                            }
                        }
                        Classification::Uncertain(parameter_reason)
                            if retain_selected_circle_endpoints =>
                        {
                            match incident_location.expect(
                                "selected-circle endpoint retention computed its incident location",
                            ) {
                                Classification::Decided(Interior) => CornerPlacement2::Trim,
                                Classification::Decided(Start | End | Exterior)
                                    if mode == CurveCornerMode2::TrimOrExtend =>
                                {
                                    CornerPlacement2::Extension
                                }
                                Classification::Decided(Start | End | Exterior) => return Ok(None),
                                Classification::Uncertain(_) => {
                                    return Err(ExactCurveError::blocked(
                                        CurveOperation2::Fillet,
                                        family,
                                        parameter_reason,
                                    ));
                                }
                            }
                        }
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                family,
                                reason,
                            ));
                        }
                    }
                    }
                }
            };
            let complementary_support;
            let complementary_source;
            let (support_semicircle, source_semicircle) = if complementary {
                complementary_support = support.semicircle().complementary_half();
                complementary_source = source.semicircle().complementary_half();
                (&complementary_support, &complementary_source)
            } else {
                (support.semicircle(), source.semicircle())
            };
            let point = if let (Some(center), Some(support_center)) = (
                center.coordinates(),
                support_semicircle.exact_center(policy).map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                })?,
            ) {
                // The selected offset contact and circle center already live
                // in canonical Real. Replay the concentric radial map there
                // instead of wrapping the same coordinates in a selected
                // point image that reconstruction would immediately have to
                // eliminate again.
                let radial_scale = (source_semicircle.radial_distance()
                    / support_semicircle.radial_distance())
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause.into())
                })?;
                let radial = center.delta_from(&support_center);
                CurvePoint2::from(
                    support_center.translated(&radial.0 * &radial_scale, &radial.1 * radial_scale),
                )
            } else {
                let point = parameter
                    .concentric_offset_point_evidence(support_semicircle, source_semicircle, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })?;
                match point {
                    Classification::Decided(Some(point)) => point,
                    Classification::Decided(None) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            family,
                            crate::UncertaintyReason::Unsupported,
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            family,
                            reason,
                        ));
                    }
                }
            };
            Ok(Some(CornerCut2 {
                point,
                parameter: Some(if complementary {
                    CurveParameter2::from_algebraic_cusp_complement(parameter)
                } else {
                    CurveParameter2::from_algebraic_cusp(parameter)
                }),
                placement,
            }))
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn algebraic_chord_fillet_cut_from_center(
    source: &crate::BezierAlgebraicChord2,
    center: &CurvePoint2,
    retained_parameter: Option<&CurveParameter2>,
    signed_distance: &Real,
    previous: bool,
    mode: CurveCornerMode2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerCut2>> {
    let point = source
        .normal_displaced_point_evidence(center.clone(), -signed_distance.clone(), policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?;
    if let Some(retained_parameter) = retained_parameter {
        let placement = if retained_parameter.is_retained_scalar() {
            curve_region_corner_parameter_placement(
                retained_parameter,
                previous,
                mode,
                CurveOperation2::Fillet,
                family,
                policy,
            )?
        } else if let Some(retained_parameter) = retained_parameter.as_algebraic_chord() {
            algebraic_chord_parallel_parameter_placement(
                source,
                retained_parameter,
                previous,
                mode,
                family,
                policy,
            )?
        } else {
            None
        };
        if let Some(placement) = placement {
            let parameter = source
                .parameter_at_certified_support_point(point.clone(), policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                })?;
            return Ok(Some(CornerCut2 {
                point,
                parameter: Some(CurveParameter2::from_algebraic_chord(parameter)),
                placement,
            }));
        }
        return Ok(None);
    }
    algebraic_chord_corner_cut_from_support_point(
        source,
        point,
        previous,
        mode,
        CurveOperation2::Fillet,
        family,
        policy,
    )
}

pub(super) fn algebraic_chord_parallel_parameter_placement(
    source: &crate::BezierAlgebraicChord2,
    parameter: &crate::bezier_offset::BezierAlgebraicChordParameter2,
    previous: bool,
    mode: CurveCornerMode2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerPlacement2>> {
    let chord = parameter.chord();
    let reversed = chord
        .retained_normal_offset_tangent_reversal_to(source)
        .ok_or_else(|| {
            ExactCurveError::invalid(
                CurveOperation2::Fillet,
                family,
                CurveError::Topology(
                    "a retained fillet contact did not descend from its source chord".into(),
                ),
            )
        })?;
    if parameter.has_certified_strict_interior_location() {
        return Ok(Some(CornerPlacement2::Trim));
    }
    let compare = |boundary| {
        parameter
            .cmp_by_refinement(boundary, policy)
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))
            .and_then(|order| match order {
                Classification::Decided(order) => Ok(order),
                Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    family,
                    reason,
                )),
            })
    };
    let start = chord.start_parameter();
    let end = chord.end_parameter();
    let start_order = compare(&start)?;
    let end_order = compare(&end)?;
    if start_order.is_gt() && end_order.is_lt() {
        return Ok(Some(CornerPlacement2::Trim));
    }
    let extends_toward_end = previous != reversed;
    Ok((mode == CurveCornerMode2::TrimOrExtend
        && ((extends_toward_end && end_order.is_gt())
            || (!extends_toward_end && start_order.is_lt())))
    .then_some(CornerPlacement2::Extension))
}

#[allow(clippy::too_many_arguments)]
pub(super) fn algebraic_chord_corner_cut_from_support_point(
    source: &crate::BezierAlgebraicChord2,
    point: CurvePoint2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerCut2>> {
    let parameter = source
        .parameter_at_certified_support_point(point.clone(), policy)
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?;
    let start = source.start_parameter();
    let end = source.end_parameter();
    let compare = |boundary| {
        parameter
            .cmp_by_refinement(boundary, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))
            .and_then(|order| match order {
                Classification::Decided(order) => Ok(order),
                Classification::Uncertain(reason) => {
                    Err(ExactCurveError::blocked(operation, family, reason))
                }
            })
    };
    let start_order = compare(&start)?;
    let end_order = compare(&end)?;
    let placement = if start_order.is_gt() && end_order.is_lt() {
        CornerPlacement2::Trim
    } else if mode == CurveCornerMode2::TrimOrExtend
        && ((previous && end_order.is_gt()) || (!previous && start_order.is_lt()))
    {
        CornerPlacement2::Extension
    } else {
        return Ok(None);
    };
    Ok(Some(CornerCut2 {
        point,
        parameter: Some(CurveParameter2::from_algebraic_chord(parameter)),
        placement,
    }))
}

pub(super) fn line_parameter_at_point(
    line: &LineSeg2,
    point: &Point2,
    operation: CurveOperation2,
    family: CurveFamily2,
) -> ExactCurveResult<Real> {
    let delta = line.delta();
    let from_start = point.delta_from(line.start());
    let numerator = &from_start.0 * &delta.0 + &from_start.1 * &delta.1;
    let denominator = &delta.0 * &delta.0 + &delta.1 * &delta.1;
    (numerator / denominator)
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause.into()))
}

/// Proves that an exact line witness carries the same finite parameter domain
/// as an algebraic chord. Canonical unit witnesses otherwise name only the
/// infinite support and must not classify trim/extension against `[0, 1]`.
pub(super) fn algebraic_chord_domain_matches_line_witness(
    source: &crate::BezierAlgebraicChord2,
    witness: &LineSeg2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    if let Some(line) = source.exact_line() {
        return Ok(
            line == *witness || (line.start() == witness.end() && line.end() == witness.start())
        );
    }
    let Some(direction) = source.certified_axis_direction() else {
        return Ok(false);
    };
    let axis = direction.axis();
    let coordinate = |point: &Point2| match axis {
        crate::Axis2::X => point.x().clone(),
        crate::Axis2::Y => point.y().clone(),
    };
    // Prepared algebraic axis chords use a canonical unit witness solely to
    // name their affine support.  Its active coordinate is structurally
    // `[0, +/-1]`; final cut publication still classifies every center on the
    // authored algebraic chord.  Treat that witness as unbounded immediately
    // instead of trying to rediscover two potentially recursive endpoint
    // equalities through the terminal predicate schedule.
    let witness_start = coordinate(witness.start());
    let witness_end = coordinate(witness.end());
    if witness_start.zero_status() == hyperreal::ZeroKnowledge::Zero
        && ((&witness_end - &witness_start).abs() - Real::one()).zero_status()
            == hyperreal::ZeroKnowledge::Zero
    {
        return Ok(false);
    }
    let equal = |point, value| {
        policy
            .strict_predicate_pass(|| {
                crate::BezierAlgebraicChord2::point_axis_order_to_real(point, axis, value, policy)
            })
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))
            .map(|order| order == Classification::Decided(std::cmp::Ordering::Equal))
    };
    let forward = equal(source.start(), &witness_start)? && equal(source.end(), &witness_end)?;
    if forward {
        return Ok(true);
    }
    Ok(equal(source.start(), &witness_end)? && equal(source.end(), &witness_start)?)
}

pub(super) fn algebraic_chord_from_line_support(
    line: &LineSeg2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<crate::BezierAlgebraicChord2> {
    match crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
        CurvePoint2::from(line.start().clone()),
        CurvePoint2::from(line.end().clone()),
        policy,
    )
    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
    {
        Classification::Decided(chord) => Ok(chord),
        Classification::Uncertain(reason) => {
            Err(ExactCurveError::blocked(operation, family, reason))
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn solve_line_fillet_corner(
    previous: &LineSeg2,
    next: &LineSeg2,
    radius: &Real,
    mode: CurveCornerMode2,
    previous_family: CurveFamily2,
    next_family: CurveFamily2,
    constraints: Option<&FilletConstraintBinding2<'_>>,
    policy: &CurveContext,
) -> ExactCurveResult<CurveCornerSolutions2<FilletCorner2>> {
    let previous_delta = previous.delta();
    let next_delta = next.delta();
    let previous_unit = line_unit_direction(
        &previous_delta.0,
        &previous_delta.1,
        CurveOperation2::Fillet,
        previous_family,
        policy,
    )?;
    let next_unit = line_unit_direction(
        &next_delta.0,
        &next_delta.1,
        CurveOperation2::Fillet,
        next_family,
        policy,
    )?;
    let denominator = &previous_delta.0 * &next_delta.1 - &previous_delta.1 * &next_delta.0;
    let denominator_sign = match crate::classify::real_sign(&denominator, policy) {
        Some(RealSign::Zero) => {
            return Ok(CurveCornerSolutions2::NoSolution(
                CurveCornerNoSolution2::ParallelTangents,
            ));
        }
        Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
        None => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                previous_family,
                crate::UncertaintyReason::RealSign,
            ));
        }
    };
    let denominator_reciprocal = denominator
        .inverse_ref_assuming_nonzero()
        .map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause.into())
        })?;

    let mut candidates = CornerSolutionAccumulator::Empty;
    let mut saw_unsatisfied = false;
    // For connected incoming/outgoing lines, only the offset side matching the
    // turn can have both contacts in the open trim domains. Extension mode must
    // retain both exact carrier solutions.
    let sides: &[(bool, bool)] = match (mode, denominator_sign) {
        (CurveCornerMode2::TrimOnly, RealSign::Positive) => &[(true, false)],
        (CurveCornerMode2::TrimOnly, RealSign::Negative) => &[(false, true)],
        (CurveCornerMode2::TrimOrExtend, _) => &[(true, false), (false, true)],
        (_, RealSign::Zero) => unreachable!("parallel line directions return before solving"),
    };
    for &(positive_radius, clockwise) in sides {
        let signed_radius = if positive_radius {
            radius.clone()
        } else {
            -radius.clone()
        };
        let previous_offset_start = previous.start().translated(
            -&previous_unit.1 * &signed_radius,
            &previous_unit.0 * &signed_radius,
        );
        let next_offset_start = next.start().translated(
            -&next_unit.1 * &signed_radius,
            &next_unit.0 * &signed_radius,
        );
        let between_offsets = next_offset_start.delta_from(&previous_offset_start);
        let previous_numerator =
            &between_offsets.0 * &next_delta.1 - &between_offsets.1 * &next_delta.0;
        let next_numerator =
            &between_offsets.0 * &previous_delta.1 - &between_offsets.1 * &previous_delta.0;
        let previous_parameter = previous_numerator * &denominator_reciprocal;
        let next_parameter = next_numerator * &denominator_reciprocal;
        let Some(previous_placement) = corner_parameter_placement(
            &previous_parameter,
            true,
            mode,
            CurveOperation2::Fillet,
            previous_family,
            policy,
        )?
        else {
            continue;
        };
        let Some(next_placement) = corner_parameter_placement(
            &next_parameter,
            false,
            mode,
            CurveOperation2::Fillet,
            next_family,
            policy,
        )?
        else {
            continue;
        };
        let previous_point = previous.point_at(previous_parameter.clone());
        let next_point = next.point_at(next_parameter.clone());
        let center = previous_offset_start.translated(
            &previous_delta.0 * &previous_parameter,
            &previous_delta.1 * &previous_parameter,
        );
        match crate::classify::is_zero(&previous_point.distance_squared(&next_point), policy) {
            Some(true) => continue,
            Some(false) => {
                let candidate = FilletCorner2 {
                    previous: CornerCut2 {
                        parameter: exact_corner_parameter(previous_parameter),
                        point: previous_point.into(),
                        placement: previous_placement,
                    },
                    next: CornerCut2 {
                        parameter: exact_corner_parameter(next_parameter),
                        point: next_point.into(),
                        placement: next_placement,
                    },
                    center: center.into(),
                    clockwise,
                    retained_frame: None,
                };
                if let Some(binding) = constraints
                    && !binding.matches(&candidate, policy)?
                {
                    saw_unsatisfied = true;
                    continue;
                }
                candidates.push(candidate);
            }
            None => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    previous_family,
                    crate::UncertaintyReason::RealSign,
                ));
            }
        }
    }
    Ok(candidates.finish(if saw_unsatisfied {
        CurveCornerNoSolution2::UnsatisfiedConstraints
    } else {
        CurveCornerNoSolution2::OutsideTrimDomain
    }))
}

pub(super) fn line_unit_direction(
    dx: &Real,
    dy: &Real,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<(Real, Real, Real)> {
    // Axis-aligned edges are common and their exact norm is already one
    // coordinate; avoid constructing and reducing a redundant square root.
    match (dx.structural_facts().sign, dy.structural_facts().sign) {
        (Some(RealSign::Zero), Some(RealSign::Positive)) => {
            return Ok((Real::zero(), Real::one(), dy.clone()));
        }
        (Some(RealSign::Zero), Some(RealSign::Negative)) => {
            return Ok((Real::zero(), -Real::one(), -dy.clone()));
        }
        (Some(RealSign::Positive), Some(RealSign::Zero)) => {
            return Ok((Real::one(), Real::zero(), dx.clone()));
        }
        (Some(RealSign::Negative), Some(RealSign::Zero)) => {
            return Ok((-Real::one(), Real::zero(), -dx.clone()));
        }
        _ => {}
    }
    let length_squared = dx * dx + dy * dy;
    match crate::classify::real_sign(&length_squared, policy) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Zero | RealSign::Negative) => {
            return Err(ExactCurveError::invalid(
                operation,
                family,
                CurveError::ZeroLengthLine,
            ));
        }
        None => {
            return Err(ExactCurveError::blocked(
                operation,
                family,
                crate::UncertaintyReason::RealSign,
            ));
        }
    }
    let length = length_squared
        .sqrt()
        .map_err(|cause| ExactCurveError::invalid(operation, family, CurveError::from(cause)))?;
    let unit_x = (dx / &length)
        .map_err(|cause| ExactCurveError::invalid(operation, family, CurveError::from(cause)))?;
    let unit_y = (dy / &length)
        .map_err(|cause| ExactCurveError::invalid(operation, family, CurveError::from(cause)))?;
    Ok((unit_x, unit_y, length))
}
