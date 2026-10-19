//! Retained surface-parameter images and their exact spatial correspondence.

use super::*;

impl SurfaceIntersectionPcurve {
    pub(crate) fn materializes_native_boundary_segment(&self) -> bool {
        self.materializes_native_line()
            || match &self.mapping {
                SurfaceIntersectionPcurveMapping::PlaneProjection { spatial, .. } => {
                    spatial.kind() == Curve3Kind::CircleArc
                }
                SurfaceIntersectionPcurveMapping::RetainedCurve { curve } => {
                    matches!(curve.geometry(), Some(CurveGeometry2::CircularArc(_)))
                }
                _ => false,
            }
    }

    /// Whether materialization uses a native line carrier. This describes the
    /// retained representation, not a geometric straightness classification:
    /// rational/spline mappings retain their families even when collinear.
    pub(crate) fn materializes_native_line(&self) -> bool {
        match &self.mapping {
            SurfaceIntersectionPcurveMapping::TensorIsoU { .. }
            | SurfaceIntersectionPcurveMapping::TensorIsoV { .. } => {
                self.domain.try_real_endpoints().is_some()
            }
            SurfaceIntersectionPcurveMapping::PlaneProjection { spatial, .. } => {
                spatial.kind() == Curve3Kind::Line
            }
            SurfaceIntersectionPcurveMapping::LinearPlaneSection { profile, .. } => {
                profile.kind() == Curve3Kind::Line
            }
            SurfaceIntersectionPcurveMapping::RetainedCurve { curve } => {
                matches!(curve.geometry(), Some(CurveGeometry2::Line(_)))
            }
            SurfaceIntersectionPcurveMapping::Unsupported => false,
        }
    }

    pub(crate) fn is_iso_parameter_line(&self) -> bool {
        matches!(
            self.mapping,
            SurfaceIntersectionPcurveMapping::TensorIsoU { .. }
                | SurfaceIntersectionPcurveMapping::TensorIsoV { .. }
        )
    }

    pub(super) fn unsupported(domain: impl Into<ParameterRange>) -> Self {
        Self {
            domain: domain.into(),
            source_scale: Real::one(),
            source_offset: Real::zero(),
            mapping: SurfaceIntersectionPcurveMapping::Unsupported,
        }
    }

    pub(super) fn retained_curve(curve: Curve2) -> GeometryResult<Self> {
        let curve_domain = curve.parameter_domain();
        Ok(Self {
            domain: ParameterRange::new(
                Parameter::from_curve(curve_domain.start())?,
                Parameter::from_curve(curve_domain.end())?,
            )?,
            source_scale: Real::one(),
            source_offset: Real::zero(),
            mapping: SurfaceIntersectionPcurveMapping::RetainedCurve { curve },
        })
    }

    pub(super) fn plane_projection(curve: Curve3, plane: &PlaneSurface) -> Self {
        Self {
            domain: curve.domain().clone(),
            source_scale: Real::one(),
            source_offset: Real::zero(),
            mapping: SurfaceIntersectionPcurveMapping::PlaneProjection {
                spatial: curve,
                origin: plane.origin.clone(),
                u: plane.u.clone(),
                v: plane.v.clone(),
            },
        }
    }

    pub(super) fn linear_plane_section(
        profile: Curve3,
        plane: &PlaneSurface,
        denominator: Real,
        profile_axis: TensorAxis,
        coefficient_scale: Real,
        coefficient_offset: Real,
    ) -> Self {
        Self {
            domain: profile.domain().clone(),
            source_scale: Real::one(),
            source_offset: Real::zero(),
            mapping: SurfaceIntersectionPcurveMapping::LinearPlaneSection {
                profile,
                plane_origin: plane.origin.clone(),
                plane_normal: plane.u.cross(&plane.v),
                denominator,
                profile_axis,
                coefficient_scale,
                coefficient_offset,
            },
        }
    }

    pub(super) fn tensor_iso_v(domain: impl Into<ParameterRange>, constant: Real) -> Self {
        Self {
            domain: domain.into(),
            source_scale: Real::one(),
            source_offset: Real::zero(),
            mapping: SurfaceIntersectionPcurveMapping::TensorIsoV { constant },
        }
    }

    pub(super) fn tensor_iso(
        domain: impl Into<ParameterRange>,
        constant: Real,
        profile_axis: TensorAxis,
        coefficient_scale: Real,
        coefficient_offset: Real,
    ) -> Self {
        Self {
            domain: domain.into(),
            source_scale: coefficient_scale,
            source_offset: coefficient_offset,
            mapping: match profile_axis {
                TensorAxis::U => SurfaceIntersectionPcurveMapping::TensorIsoU { constant },
                TensorAxis::V => SurfaceIntersectionPcurveMapping::TensorIsoV { constant },
            },
        }
    }

    /// Returns the pcurve's parameter domain, shared exactly with the spatial
    /// intersection curve.
    pub const fn domain(&self) -> &ParameterRange {
        &self.domain
    }

    pub(crate) fn is_available(&self) -> bool {
        !matches!(self.mapping, SurfaceIntersectionPcurveMapping::Unsupported)
    }

    /// Evaluates the retained UV image at one spatial-curve parameter.
    ///
    /// Selected parameters and point images keep their source evidence. A
    /// stored coordinate view is optional, not a prerequisite for evaluation.
    /// Membership is checked before applying the source map. Native rational
    /// projections and section spans retain their charts; planar procedural
    /// projections require an admitted similarity of their source chart.
    pub fn point_at(&self, parameter: &Parameter) -> GeometryResult<hypercurve::CurvePoint2> {
        crate::context::strict(|| {
            if !self.domain.contains(parameter)? {
                return Err(GeometryError::ParameterOutsideDomain);
            }
            let source = parameter.affine_image(&self.source_scale, &self.source_offset)?;
            match &self.mapping {
                SurfaceIntersectionPcurveMapping::Unsupported => {
                    Err(GeometryError::UnsupportedIntersection)
                }
                SurfaceIntersectionPcurveMapping::RetainedCurve { curve } => {
                    Ok(curve.point_at(&source.curve_parameter())?)
                }
                SurfaceIntersectionPcurveMapping::PlaneProjection {
                    spatial,
                    origin,
                    u,
                    v,
                } => {
                    let point = spatial.point_at(&source)?;
                    if let Some(point) = point.try_coordinates() {
                        let uv = project_point_to_plane_frame(origin, u, v, point)?;
                        return Ok(CurvePoint2::new(uv.x, uv.y).into());
                    }
                    if let CurveGeometry3::PlanarImage(image) = &spatial.data.geometry {
                        let projected = image.materialize_plane_projection(origin, u, v)?;
                        let local =
                            source.affine_image(&image.source_scale, &image.source_offset)?;
                        return Ok(projected.curve.point_at(&local.curve_parameter())?);
                    }
                    let projected = project_curve_to_plane_frame(spatial, origin, u, v)?
                        .ok_or(GeometryError::UnsupportedSelectedEvaluation)?;
                    Ok(projected.point_at(&source.curve_parameter())?)
                }
                SurfaceIntersectionPcurveMapping::LinearPlaneSection {
                    profile,
                    plane_origin,
                    plane_normal,
                    denominator,
                    profile_axis,
                    coefficient_scale,
                    coefficient_offset,
                } => {
                    if let Some(parameter) = source.try_real() {
                        let profile_point = profile.evaluate_real(parameter)?;
                        let normalized = (plane_normal.dot(&(plane_origin - &profile_point))
                            / denominator)
                            .map_err(GeometryError::Numeric)?;
                        let coefficient = coefficient_offset + coefficient_scale * normalized;
                        return Ok(section_parameter_point(
                            parameter.clone(),
                            coefficient,
                            *profile_axis,
                        )
                        .into());
                    }
                    let carriers = linear_section_pcurve_carriers(
                        profile,
                        plane_origin,
                        plane_normal,
                        denominator,
                        *profile_axis,
                        coefficient_scale,
                        coefficient_offset,
                    )?
                    .ok_or(GeometryError::UnsupportedSelectedEvaluation)?;
                    for carrier in carriers {
                        let (scale, offset) = carrier
                            .correspondence
                            .affine_coefficients()
                            .ok_or(GeometryError::UnsupportedPcurveContour)?;
                        let inverse = (Real::one() / scale).map_err(GeometryError::Numeric)?;
                        let local = source
                            .affine_image(&inverse, &(-offset * &inverse))?
                            .curve_parameter();
                        let domain = carrier.curve.parameter_domain();
                        if local.compare(domain.start())? != Ordering::Less
                            && local.compare(domain.end())? != Ordering::Greater
                        {
                            return Ok(carrier.curve.point_at(&local)?);
                        }
                    }
                    Err(GeometryError::ParameterOutsideDomain)
                }
                SurfaceIntersectionPcurveMapping::TensorIsoV { constant } => Ok(source
                    .curve_parameter()
                    .affine_point(Real::one(), Real::zero(), Real::zero(), constant.clone())?),
                SurfaceIntersectionPcurveMapping::TensorIsoU { constant } => Ok(source
                    .curve_parameter()
                    .affine_point(Real::zero(), constant.clone(), Real::one(), Real::zero())?),
            }
        })
    }

    /// Restricts this exact surface-parameter mapping without changing its
    /// authoritative spatial-curve parameterization.
    /// Selected endpoints keep their defining relation and source chart.
    pub fn subcurve(&self, start: &Parameter, end: &Parameter) -> GeometryResult<Self> {
        crate::context::strict(|| {
            if !self.domain.contains(start)? || !self.domain.contains(end)? {
                return Err(GeometryError::ParameterOutsideDomain);
            }
            Ok(Self {
                domain: ParameterRange::new(start.clone(), end.clone())?,
                source_scale: self.source_scale.clone(),
                source_offset: self.source_offset.clone(),
                mapping: self.mapping.clone(),
            })
        })
    }

    /// Returns the same exact parameter-space image with reversed traversal.
    pub fn reversed(&self) -> GeometryResult<Self> {
        crate::context::strict(|| {
            let domain_sum = self
                .domain
                .try_real_endpoints()
                .map_or_else(Real::zero, |(a, b)| a + b);
            Ok(Self {
                domain: self.domain.affine_image(&-Real::one(), &domain_sum)?,
                source_scale: -self.source_scale.clone(),
                source_offset: &self.source_scale * domain_sum + &self.source_offset,
                mapping: self.mapping.clone(),
            })
        })
    }

    /// Materializes this retained mapping as one exact Hypercurve carrier.
    ///
    /// The returned correspondence maps native pcurve parameters to spatial
    /// parameters, including angular sweep correspondence for circular arcs.
    /// A mapping that cannot be represented by one Hypercurve carrier remains
    /// explicitly unsupported.
    pub fn materialize(&self) -> GeometryResult<MaterializedSurfacePcurve> {
        crate::context::strict(|| {
            if let Some(carrier) = self.materialize_selected_chart()? {
                return Ok(carrier);
            }
            if let SurfaceIntersectionPcurveMapping::PlaneProjection {
                spatial,
                origin,
                u,
                v,
            } = &self.mapping
                && let CurveGeometry3::PlanarImage(image) = &spatial.data.geometry
            {
                let range = self
                    .domain
                    .affine_image(&self.source_scale, &self.source_offset)?;
                let projected = image
                    .restricted_to(&range)?
                    .materialize_plane_projection(origin, u, v)?;
                let SurfacePcurveCorrespondence::Affine { scale, offset } =
                    projected.correspondence
                else {
                    unreachable!("planar projection retains an affine source chart");
                };
                return orient_affine_surface_pcurve(
                    projected.curve,
                    (scale / &self.source_scale).map_err(GeometryError::Numeric)?,
                    ((offset - &self.source_offset) / &self.source_scale)
                        .map_err(GeometryError::Numeric)?,
                    &self.domain,
                );
            }
            let source_start =
                &self.source_scale * self.domain.represented_start()? + &self.source_offset;
            let source_end =
                &self.source_scale * self.domain.represented_end()? + &self.source_offset;
            let source_order = decided_order(compare_reals(
                &source_start,
                &source_end,
                crate::STRICT_PREDICATES,
            ))?;
            if source_order == Ordering::Equal {
                return Err(GeometryError::InvalidParameterDomain);
            }
            let descending = source_order == Ordering::Greater;
            let (ordered_source_start, ordered_source_end) = if descending {
                (&source_end, &source_start)
            } else {
                (&source_start, &source_end)
            };
            let orient_curve = |curve: Curve2| -> GeometryResult<Curve2> {
                if descending {
                    curve.reversed().map_err(GeometryError::from)
                } else {
                    Ok(curve)
                }
            };
            // Materialized iso coordinates become boundary coefficients and
            // are interpolated by later edits. Preserve scalar-proven compact
            // angular forms before that expansion; no recognition limit or
            // spatial parameter correspondence changes.
            let compact =
                |value: &Real| value.compact_pi_laurent().unwrap_or_else(|| value.clone());
            match &self.mapping {
                SurfaceIntersectionPcurveMapping::Unsupported => {
                    Err(GeometryError::UnsupportedIntersection)
                }
                SurfaceIntersectionPcurveMapping::RetainedCurve { curve } => {
                    let domain = curve.parameter_domain();
                    let restricted = if decided_order(compare_reals(
                        ordered_source_start,
                        (domain.start())
                            .scalar()
                            .ok_or(GeometryError::UnsupportedPcurveContour)?,
                        crate::STRICT_PREDICATES,
                    ))? == Ordering::Equal
                        && decided_order(compare_reals(
                            ordered_source_end,
                            (domain.end())
                                .scalar()
                                .ok_or(GeometryError::UnsupportedPcurveContour)?,
                            crate::STRICT_PREDICATES,
                        ))? == Ordering::Equal
                    {
                        curve.clone()
                    } else {
                        curve.subcurve(
                            ordered_source_start.clone().into(),
                            ordered_source_end.clone().into(),
                        )?
                    };
                    materialized_surface_pcurve_from_matching_domains(
                        orient_curve(restricted)?,
                        &self.domain.represented_domain()?,
                    )
                }
                SurfaceIntersectionPcurveMapping::PlaneProjection {
                    spatial,
                    origin,
                    u,
                    v,
                } => {
                    let source_curve =
                        spatial.subcurve(ordered_source_start, ordered_source_end)?;
                    let curve = orient_curve(
                        project_curve_to_plane_frame(&source_curve, origin, u, v)?
                            .ok_or(GeometryError::UnsupportedIntersection)?,
                    )?;
                    if curve.family() == CurveFamily2::CircularArc {
                        Ok(MaterializedSurfacePcurve {
                            curve,
                            correspondence: SurfacePcurveCorrespondence::AngularSweep {
                                spatial_start: self.domain.represented_start()?.clone(),
                                spatial_end: self.domain.represented_end()?.clone(),
                            },
                        })
                    } else {
                        materialized_surface_pcurve_from_matching_domains(
                            curve,
                            &self.domain.represented_domain()?,
                        )
                    }
                }
                SurfaceIntersectionPcurveMapping::LinearPlaneSection { .. } => {
                    let mut carriers = self
                        .clipping_carriers()?
                        .ok_or(GeometryError::UnsupportedIntersection)?;
                    if carriers.len() == 1 {
                        return Ok(carriers.remove(0));
                    }
                    let mut curves = Vec::with_capacity(carriers.len());
                    let mut boundaries = Vec::with_capacity(carriers.len() + 1);
                    for carrier in carriers {
                        let domain = carrier.curve.parameter_domain();
                        let (start, end) = domain
                            .scalar_endpoints()
                            .ok_or(GeometryError::UnsupportedPcurveContour)?;
                        if boundaries.is_empty() {
                            boundaries.push(
                                carrier
                                    .spatial_parameter_at(&start.clone().into())?
                                    .try_real()
                                    .cloned()
                                    .ok_or(GeometryError::UnrepresentableParameter)?,
                            );
                        }
                        boundaries.push(
                            carrier
                                .spatial_parameter_at(&end.clone().into())?
                                .try_real()
                                .cloned()
                                .ok_or(GeometryError::UnrepresentableParameter)?,
                        );
                        curves.push(carrier.curve);
                    }
                    materialized_surface_pcurve_from_matching_domains(
                        concatenate_rational_bezier_spans_as_nurbs(&curves, &boundaries)?,
                        &self.domain.represented_domain()?,
                    )
                }
                SurfaceIntersectionPcurveMapping::TensorIsoV { constant } => {
                    let curve = orient_curve(Curve2::from(LineSeg2::try_new(
                        CurvePoint2::new(compact(ordered_source_start), compact(constant)),
                        CurvePoint2::new(compact(ordered_source_end), compact(constant)),
                    )?))?;
                    materialized_surface_pcurve_from_matching_domains(
                        curve,
                        &self.domain.represented_domain()?,
                    )
                }
                SurfaceIntersectionPcurveMapping::TensorIsoU { constant } => {
                    let curve = orient_curve(Curve2::from(LineSeg2::try_new(
                        CurvePoint2::new(compact(constant), compact(ordered_source_start)),
                        CurvePoint2::new(compact(constant), compact(ordered_source_end)),
                    )?))?;
                    materialized_surface_pcurve_from_matching_domains(
                        curve,
                        &self.domain.represented_domain()?,
                    )
                }
            }
        })
    }

    /// A selected interval keeps its source chart: dividing its endpoints to
    /// force a unit interval would discard their common root authority.
    fn materialize_selected_chart(&self) -> GeometryResult<Option<MaterializedSurfacePcurve>> {
        if !matches!(
            self.mapping,
            SurfaceIntersectionPcurveMapping::RetainedCurve { .. }
                | SurfaceIntersectionPcurveMapping::TensorIsoU { .. }
                | SurfaceIntersectionPcurveMapping::TensorIsoV { .. }
        ) {
            return Ok(None);
        }
        let selected = self.domain.try_real_endpoints().is_none()
            || matches!(&self.mapping, SurfaceIntersectionPcurveMapping::RetainedCurve { curve }
                if curve.parameter_domain().scalar_endpoints().is_none());
        if !selected {
            return Ok(None);
        }
        let range = self
            .domain
            .affine_image(&self.source_scale, &self.source_offset)?;
        let range = hypercurve::CurveParameterRange2::try_new(
            range.start().curve_parameter(),
            range.end().curve_parameter(),
        )?;
        let curve = match &self.mapping {
            SurfaceIntersectionPcurveMapping::RetainedCurve { curve } => {
                let domain = curve.parameter_domain();
                if range.start().compare(domain.start())? == Ordering::Equal
                    && range.end().compare(domain.end())? == Ordering::Equal
                {
                    curve.clone()
                } else {
                    curve.restrict_source(range)?
                }
            }
            SurfaceIntersectionPcurveMapping::TensorIsoU { constant }
            | SurfaceIntersectionPcurveMapping::TensorIsoV { constant } => {
                let constant = constant
                    .compact_pi_laurent()
                    .unwrap_or_else(|| constant.clone());
                let point = |t| match &self.mapping {
                    SurfaceIntersectionPcurveMapping::TensorIsoU { .. } => {
                        CurvePoint2::new(constant.clone(), t)
                    }
                    _ => CurvePoint2::new(t, constant.clone()),
                };
                // Enclosures choose a finite authored support, never the
                // retained endpoints. A degree-one spline has the original
                // source coordinate as its knot parameter, including exterior
                // values, without another normalized selected-root chart.
                // Hypercurve checks the exact restriction; SourceRange replays it.
                let lower = range
                    .start()
                    .scalar_bounds()
                    .ok_or(GeometryError::UnrepresentableParameter)?
                    .0
                    .clone();
                let upper = range
                    .end()
                    .scalar_bounds()
                    .ok_or(GeometryError::UnrepresentableParameter)?
                    .1
                    .clone();
                let span = &upper - &lower;
                require_positive(&span, GeometryError::InvalidParameterDomain)?;
                Curve2::from(hypercurve::NurbsCurve2::try_new(
                    1,
                    vec![point(lower.clone()), point(upper.clone())],
                    vec![Real::one(); 2],
                    vec![lower.clone(), lower, upper.clone(), upper],
                )?)
                .restrict_source(range)?
            }
            _ => return Ok(None),
        };
        let scale = (Real::one() / &self.source_scale).map_err(GeometryError::Numeric)?;
        orient_affine_surface_pcurve(
            curve,
            scale.clone(),
            -&self.source_offset * scale,
            &self.domain,
        )
        .map(Some)
    }

    pub(super) fn reparameterized_subcurve(
        &self,
        start: &Real,
        end: &Real,
        target_domain: &ParameterDomain,
    ) -> GeometryResult<Self> {
        if !self.domain.contains_real(start)? || !self.domain.contains_real(end)? {
            return Err(GeometryError::ParameterOutsideDomain);
        }
        let source_start = &self.source_scale * start + &self.source_offset;
        let source_end = &self.source_scale * end + &self.source_offset;
        let target_span = target_domain.end() - target_domain.start();
        let source_scale =
            ((&source_end - &source_start) / target_span).map_err(GeometryError::Numeric)?;
        let source_offset = source_start - &source_scale * target_domain.start();
        Ok(Self {
            domain: target_domain.clone().into(),
            source_scale,
            source_offset,
            mapping: self.mapping.clone(),
        })
    }

    pub(crate) fn clipping_carriers(
        &self,
    ) -> GeometryResult<Option<Vec<MaterializedSurfacePcurve>>> {
        let SurfaceIntersectionPcurveMapping::LinearPlaneSection {
            profile,
            plane_origin,
            plane_normal,
            denominator,
            profile_axis,
            coefficient_scale,
            coefficient_offset,
        } = &self.mapping
        else {
            return match self.materialize() {
                Ok(carrier) => Ok(Some(vec![carrier])),
                Err(GeometryError::UnsupportedIntersection) => Ok(None),
                Err(error) => Err(error),
            };
        };
        // Keep the native rational spans available to the trimming kernel.
        // Restriction and reversal must transport both their geometry and
        // parameters; the source profile's domain need not equal this trace's.
        let Some(carriers) = linear_section_pcurve_carriers(
            profile,
            plane_origin,
            plane_normal,
            denominator,
            *profile_axis,
            coefficient_scale,
            coefficient_offset,
        )?
        else {
            return Ok(None);
        };
        let source_start =
            &self.source_scale * self.domain.represented_start()? + &self.source_offset;
        let source_end = &self.source_scale * self.domain.represented_end()? + &self.source_offset;
        let descending = decided_order(compare_reals(
            &source_start,
            &source_end,
            crate::STRICT_PREDICATES,
        ))? == Ordering::Greater;
        let (source_start, source_end) = if descending {
            (source_end, source_start)
        } else {
            (source_start, source_end)
        };
        let mut result = Vec::with_capacity(carriers.len());
        for carrier in carriers {
            let (scale, offset) = carrier
                .correspondence
                .affine_coefficients()
                .ok_or(GeometryError::UnsupportedPcurveContour)?;
            let domain = carrier.curve.parameter_domain();
            let (local_start, local_end) = domain
                .scalar_endpoints()
                .ok_or(GeometryError::UnsupportedPcurveContour)?;
            let carrier_start = scale * local_start + offset;
            let carrier_end = scale * local_end + offset;
            let overlap_start = if decided_order(compare_reals(
                &source_start,
                &carrier_start,
                crate::STRICT_PREDICATES,
            ))? == Ordering::Greater
            {
                source_start.clone()
            } else {
                carrier_start
            };
            let overlap_end = if decided_order(compare_reals(
                &source_end,
                &carrier_end,
                crate::STRICT_PREDICATES,
            ))? == Ordering::Less
            {
                source_end.clone()
            } else {
                carrier_end
            };
            if decided_order(compare_reals(
                &overlap_start,
                &overlap_end,
                crate::STRICT_PREDICATES,
            ))? != Ordering::Less
            {
                continue;
            }
            let start = ((&overlap_start - offset) / scale).map_err(GeometryError::Numeric)?;
            let end = ((&overlap_end - offset) / scale).map_err(GeometryError::Numeric)?;
            let mut curve =
                if decided_order(compare_reals(&start, local_start, crate::STRICT_PREDICATES))?
                    == Ordering::Equal
                    && decided_order(compare_reals(&end, local_end, crate::STRICT_PREDICATES))?
                        == Ordering::Equal
                {
                    carrier.curve
                } else {
                    carrier.curve.subcurve(start.into(), end.into())?
                };
            let mut spatial_start = ((overlap_start - &self.source_offset) / &self.source_scale)
                .map_err(GeometryError::Numeric)?;
            let mut spatial_end = ((overlap_end - &self.source_offset) / &self.source_scale)
                .map_err(GeometryError::Numeric)?;
            if descending {
                curve = curve.reversed()?;
                std::mem::swap(&mut spatial_start, &mut spatial_end);
            }
            result.push(materialized_surface_pcurve_from_matching_domains(
                curve,
                &ParameterDomain::new(spatial_start, spatial_end)?,
            )?);
        }
        if descending {
            result.reverse();
        }
        Ok(Some(result))
    }
}

impl MaterializedSurfacePcurve {
    /// Returns the exact Hypercurve carrier.
    pub const fn curve(&self) -> &Curve2 {
        &self.curve
    }

    /// Returns the exact pcurve-to-spatial parameter correspondence.
    pub const fn correspondence(&self) -> &SurfacePcurveCorrespondence {
        &self.correspondence
    }

    /// Maps an incident pcurve parameter to the authoritative spatial parameter.
    ///
    /// Affine correspondence retains selected roots and their source evidence.
    /// Out-of-domain queries fail before applying the map. Angular endpoints
    /// preserve the two seam identities; an interior angular image still needs
    /// represented coordinates and otherwise fails with a typed blocker.
    pub fn spatial_parameter_at(
        &self,
        parameter: &hypercurve::CurveParameter2,
    ) -> GeometryResult<Parameter> {
        crate::context::strict(|| {
            let domain = self.curve.parameter_domain();
            if parameter.compare(domain.start())? == Ordering::Less
                || parameter.compare(domain.end())? == Ordering::Greater
            {
                return Err(GeometryError::ParameterOutsideDomain);
            }
            match &self.correspondence {
                SurfacePcurveCorrespondence::Affine { scale, offset } => {
                    Parameter::from_curve(parameter)?.affine_image(scale, offset)
                }
                SurfacePcurveCorrespondence::AngularSweep {
                    spatial_start,
                    spatial_end,
                } => {
                    // A periodic seam has two parameter identities even though
                    // its endpoint coordinates agree. Preserve the selected end
                    // before recovering an angle from the point's position.
                    if parameter.compare(domain.start())? == Ordering::Equal {
                        return Ok(spatial_start.clone().into());
                    }
                    if parameter.compare(domain.end())? == Ordering::Equal {
                        return Ok(spatial_end.clone().into());
                    }
                    let Some(CurveGeometry2::CircularArc(arc)) = self.curve.geometry() else {
                        return Err(GeometryError::UnsupportedPcurveContour);
                    };
                    let point = self.curve.point_at(parameter)?;
                    let point = point
                        .coordinates()
                        .ok_or(GeometryError::UnsupportedPcurveContour)?;
                    let fraction = match crate::error::classified(arc.sweep_fraction(point))? {
                        Classification::Decided(fraction) => fraction,
                        Classification::Uncertain(reason) => {
                            return Err(GeometryError::PlanarClassificationUnresolved(reason));
                        }
                    };
                    Ok((spatial_start + (spatial_end - spatial_start) * fraction).into())
                }
            }
        })
    }
}
