//! Validated construction: validation analytic_image.

use crate::topology::*;

/// Compare complete profiles in charts differing only by parameter origin.
/// A reflected trimmed profile has the same span as its edge but can occupy
/// the opposite end of the parent domain. Preserve every carrier coefficient
/// and move only its parameter chart before using the existing full proof.
fn profile_parameterizations_equal(actual: &Curve3, expected: &Curve3) -> Result<bool, BuildError> {
    if curve_parameterizations_equal(actual, expected)? {
        return Ok(true);
    }
    let shift = actual.domain().represented_start()? - expected.domain().represented_start()?;
    if shift.zero_status() == hyperreal::ZeroKnowledge::Zero
        || !real_values_equal(
            actual.domain().represented_end()?,
            &(expected.domain().represented_end()? + &shift),
        )?
    {
        return Ok(false);
    }
    let data = match expected.exact_data() {
        Curve3ExactData::EllipseArc(mut arc) => {
            arc.domain_start = actual.domain().represented_start()?.clone();
            arc.domain_end = actual.domain().represented_end()?.clone();
            Curve3ExactData::EllipseArc(arc)
        }
        Curve3ExactData::Nurbs {
            degree,
            control_points,
            weights,
            knots,
        } => Curve3ExactData::Nurbs {
            degree,
            control_points,
            weights,
            knots: knots.into_iter().map(|knot| knot + &shift).collect(),
        },
        _ => return Ok(false),
    };
    curve_parameterizations_equal(actual, &Curve3::from_exact_data(data)?)
}

impl ArenaModelBuilder {
    pub(in crate::topology) fn validate_coordinate_surface_image(
        &self,
        curve: &Curve3,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let ParameterCorrespondence::Coordinate {
            axis,
            scale,
            offset,
        } = &edge_use.parameter_correspondence
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        // Exact line geometry plus a constant orthogonal coordinate proves the
        // full iso image, including selected endpoints. Endpoint coincidence
        // alone does not establish either assertion.
        if pcurve.kind() != CurveFamily2::Line {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let first = pcurve.curve().start();
        let last = pcurve.curve().end();
        let (a, b) = match axis {
            hypercurve::Axis2::X => (Real::zero(), Real::one()),
            hypercurve::Axis2::Y => (Real::one(), Real::zero()),
        };
        let constant = Parameter::from_curve(
            &first
                .affine_form(a.clone(), b.clone(), Real::zero())
                .parameter()
                .map_err(GeometryError::from)?,
        )?;
        let last_constant = Parameter::from_curve(
            &last
                .affine_form(a, b, Real::zero())
                .parameter()
                .map_err(GeometryError::from)?,
        )?;
        if constant.compare(&last_constant)? != std::cmp::Ordering::Equal {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        if surface.kind() == SurfaceKind::Revolution {
            return self.validate_coordinate_revolution_image(
                curve, *axis, scale, offset, &constant, surface,
            );
        }
        let Some((profile, direction)) = surface.extrusion_profile_and_direction() else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        match axis {
            hypercurve::Axis2::X => {
                let v = constant
                    .try_real()
                    .ok_or(GeometryError::UnrepresentableParameter)?;
                let expected = translated_curve(profile, &(direction.clone() * v))?;
                if curve_parameterizations_equal(curve, &expected)? {
                    require_real_equal(scale, &Real::one(), BuildError::EdgeUseSupportMismatch)?;
                    require_real_equal(offset, &Real::zero(), BuildError::EdgeUseSupportMismatch)
                } else if curve_parameterizations_equal(curve, &expected.reversed()?)? {
                    require_real_equal(scale, &(-Real::one()), BuildError::EdgeUseSupportMismatch)?;
                    require_real_equal(
                        offset,
                        &expected.reversal_pivot(),
                        BuildError::EdgeUseSupportMismatch,
                    )
                } else {
                    Err(BuildError::EdgeUseSupportMismatch)
                }
            }
            hypercurve::Axis2::Y => {
                let origin = profile.point_at(&constant)?;
                let inverse = (Real::one() / scale).map_err(GeometryError::Numeric)?;
                let translation = direction.clone() * (-offset * &inverse);
                let expected_origin =
                    origin.transformed(&Matrix4::affine_translation(translation.0))?;
                let expected_displacement = direction.clone() * inverse;
                let expected = Curve3::translation(expected_origin, expected_displacement)?;
                if curve_parameterizations_equal(curve, &expected)? {
                    Ok(())
                } else {
                    Err(BuildError::EdgeUseSupportMismatch)
                }
            }
        }
    }

    fn validate_coordinate_revolution_image(
        &self,
        curve: &Curve3,
        axis: hypercurve::Axis2,
        scale: &Real,
        offset: &Real,
        constant: &Parameter,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        match axis {
            hypercurve::Axis2::X => {
                let direction = if real_values_equal(scale, &Real::one())? {
                    1
                } else if real_values_equal(scale, &(-Real::one()))? {
                    -1
                } else {
                    return Err(BuildError::EdgeUseSupportMismatch);
                };
                // The complete angular chart is u = direction * (edge - offset).
                let expected = surface.revolution_rotation_curve(
                    constant,
                    curve.domain().represented_domain()?,
                    direction,
                    -offset * scale,
                )?;
                if curve_parameterizations_equal(curve, &expected)? {
                    Ok(())
                } else {
                    Err(BuildError::EdgeUseSupportMismatch)
                }
            }
            hypercurve::Axis2::Y => {
                let angle = constant
                    .try_real()
                    .ok_or(GeometryError::UnrepresentableParameter)?;
                let expected = surface.revolution_meridian_curve(angle)?;
                if curve_parameterizations_equal(curve, &expected)? {
                    require_real_equal(scale, &Real::one(), BuildError::EdgeUseSupportMismatch)?;
                    require_real_equal(offset, &Real::zero(), BuildError::EdgeUseSupportMismatch)
                } else if curve_parameterizations_equal(curve, &expected.reversed()?)? {
                    require_real_equal(scale, &(-Real::one()), BuildError::EdgeUseSupportMismatch)?;
                    require_real_equal(
                        offset,
                        &expected.reversal_pivot(),
                        BuildError::EdgeUseSupportMismatch,
                    )
                } else {
                    Err(BuildError::EdgeUseSupportMismatch)
                }
            }
        }
    }

    pub(in crate::topology) fn validate_planar_curve_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let SurfaceExactData::Plane { origin, u, v } = surface.exact_data() else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let ParameterCorrespondence::Affine { scale, offset, .. } =
            &edge_use.parameter_correspondence
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        if pcurve.curve().polynomial_branch().is_some()
            || edge.domain.try_real_endpoints().is_none()
        {
            let Curve3ExactData::PlanarImage(actual) = curve.exact_data() else {
                return Err(BuildError::EdgeUseSupportMismatch);
            };
            // Compare the complete definitions in the spatial chart,
            // without evaluating beyond the pcurve's selected restriction.
            // Its actual endpoints and their chart correspondence were
            // checked by validate_edge_use_image. Restriction preserves this
            // identity; no independent scalar endpoint is needed.
            let expected = crate::geometry::PlanarCurve3 {
                source: pcurve.curve().clone(),
                origin,
                u,
                v,
                source_scale: (Real::one() / scale).map_err(GeometryError::Numeric)?,
                source_offset: (-offset / scale).map_err(GeometryError::Numeric)?,
            };
            return if actual.parameterization_equals(&expected)? {
                Ok(())
            } else {
                Err(BuildError::EdgeUseSupportMismatch)
            };
        };
        let (start, end) = edge
            .domain
            .try_real_endpoints()
            .expect("represented endpoints checked above");
        let expected = Curve3::from_planar_image(crate::geometry::PlanarCurve3 {
            source: pcurve.curve().clone(),
            origin,
            u,
            v,
            source_scale: ((end - start) / scale).map_err(GeometryError::Numeric)?,
            source_offset: ((start - offset) / scale).map_err(GeometryError::Numeric)?,
        })?;
        if curve_parameterizations_equal(&curve.subcurve(start, end)?, &expected)? {
            Ok(())
        } else {
            Err(BuildError::EdgeUseSupportMismatch)
        }
    }

    pub(in crate::topology) fn validate_planar_circle_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let Curve3ExactData::EllipseArc(data) = curve.exact_data() else {
            unreachable!("circle kind carries ellipse-arc exact data");
        };
        debug_assert!(data.circle);
        let arc = pcurve
            .circular_arc()
            .ok_or(GeometryError::UnsupportedPcurveContour)?;
        let center_parameter = Point2::new(arc.center().x().clone(), arc.center().y().clone());
        let mapped_center = surface.evaluate_real(&center_parameter)?;
        require_point_equal(
            &mapped_center,
            &data.center,
            BuildError::EdgeUseSupportMismatch,
        )?;

        let mapped_start = surface.evaluate_real(&Point2::new(
            arc.start().x().clone(),
            arc.start().y().clone(),
        ))?;
        let mapped_end =
            surface.evaluate_real(&Point2::new(arc.end().x().clone(), arc.end().y().clone()))?;
        let (directed_start_vertex, directed_end_vertex) = match edge_use.direction {
            Direction::Forward => (edge.start, edge.end),
            Direction::Reversed => (edge.end, edge.start),
        };
        require_point_equal(
            &mapped_start,
            self.vertex_ref(directed_start_vertex)?
                .represented_point()?,
            BuildError::EdgeUseSupportMismatch,
        )?;
        require_point_equal(
            &mapped_end,
            self.vertex_ref(directed_end_vertex)?.represented_point()?,
            BuildError::EdgeUseSupportMismatch,
        )?;

        // A linear plane map carries a circle to the same spatial circle for
        // every angle once it carries one radial/tangent frame exactly. This
        // certifies the complete image and its traversal without asking atan
        // to reconstruct a persisted arc's authored sweep from its endpoints.
        let radial_u = arc.start().x() - arc.center().x();
        let radial_v = arc.start().y() - arc.center().y();
        let (tangent_u, tangent_v) = if arc.is_clockwise() {
            (radial_v.clone(), -radial_u.clone())
        } else {
            (-radial_v.clone(), radial_u.clone())
        };
        let (plane_u, plane_v) = surface
            .plane_directions()
            .expect("plane kind carries plane directions");
        let mapped_tangent = plane_u.clone() * tangent_u + plane_v.clone() * tangent_v;

        let spatial_radial = &mapped_start - &data.center;
        let radial_x = data.x.dot(&spatial_radial);
        let radial_y = data.y.dot(&spatial_radial);
        let mut expected_tangent = data.y.clone() * radial_x - data.x.clone() * radial_y;
        if data.direction < 0 {
            expected_tangent = -expected_tangent;
        }
        if edge_use.direction == Direction::Reversed {
            expected_tangent = -expected_tangent;
        }
        require_vector_equal(
            &mapped_tangent,
            &expected_tangent,
            BuildError::EdgeUseSupportMismatch,
        )
    }

    pub(in crate::topology) fn validate_cylinder_axial_line_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .ok_or(GeometryError::UnsupportedPcurveContour)?;
        require_real_equal(
            line.start().x(),
            line.end().x(),
            BuildError::EdgeUseSupportMismatch,
        )?;
        let Curve3ExactData::Line(curve_data) = curve.exact_data() else {
            unreachable!("line kind carries line exact data");
        };
        let SurfaceExactData::Cylinder {
            origin,
            x,
            y,
            axis,
            radius,
        } = surface.exact_data()
        else {
            unreachable!("cylinder kind carries cylinder exact data");
        };
        let u = line.start().x().clone();
        let expected_x = &radius * u.clone().cos();
        let expected_y = &radius * u.sin();
        for point in [&curve_data.start, &curve_data.end] {
            let offset = point - &origin;
            require_real_equal(
                &offset.dot(&x),
                &expected_x,
                BuildError::EdgeUseSupportMismatch,
            )?;
            require_real_equal(
                &offset.dot(&y),
                &expected_y,
                BuildError::EdgeUseSupportMismatch,
            )?;
        }
        for parameter in [pcurve.represented_start()?, pcurve.represented_end()?] {
            let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
                pcurve,
                &edge.domain,
                edge_use.direction,
                &crate::Parameter::from_real((parameter).clone()),
            )?;
            let edge_point = curve.evaluate_real(
                edge_parameter
                    .try_real()
                    .ok_or(crate::GeometryError::UnrepresentableParameter)?,
            )?;
            let axial = (&edge_point - &origin).dot(&axis);
            let surface_parameter = pcurve.evaluate_real(parameter)?;
            require_real_equal(
                &axial,
                &surface_parameter.y,
                BuildError::EdgeUseSupportMismatch,
            )?;
        }
        Ok(())
    }

    pub(in crate::topology) fn validate_cylinder_circle_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .ok_or(GeometryError::UnsupportedPcurveContour)?;
        require_real_equal(
            line.start().y(),
            line.end().y(),
            BuildError::EdgeUseSupportMismatch,
        )?;
        let Curve3ExactData::EllipseArc(curve_data) = curve.exact_data() else {
            unreachable!("circle kind carries ellipse-arc exact data");
        };
        let SurfaceExactData::Cylinder {
            origin,
            x,
            y,
            axis,
            radius,
        } = surface.exact_data()
        else {
            unreachable!("cylinder kind carries cylinder exact data");
        };
        validate_circle_iso_image(
            &curve_data,
            edge,
            edge_use.direction,
            origin + axis * line.start().y(),
            [x, y],
            radius,
            [line.start().x().clone(), line.end().x().clone()],
        )
    }

    pub(in crate::topology) fn validate_sphere_circle_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .ok_or(GeometryError::UnsupportedPcurveContour)?;
        let u_constant = real_values_equal(line.start().x(), line.end().x())?;
        let v_constant = real_values_equal(line.start().y(), line.end().y())?;
        if u_constant == v_constant {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let Curve3ExactData::EllipseArc(curve_data) = curve.exact_data() else {
            unreachable!("circle kind carries ellipse-arc exact data");
        };
        if !curve_data.circle {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let SurfaceExactData::Sphere {
            center,
            x,
            y,
            axis,
            radius,
        } = surface.exact_data()
        else {
            unreachable!("sphere kind carries sphere exact data");
        };
        let (circle_center, axes, circle_radius, angles) = if v_constant {
            let latitude = line.start().y().clone();
            (
                center + axis * (&radius * latitude.clone().sin()),
                [x, y],
                radius * latitude.cos(),
                [line.start().x().clone(), line.end().x().clone()],
            )
        } else {
            let longitude = line.start().x().clone();
            let radial = x * longitude.clone().cos() + y * longitude.sin();
            (
                center,
                [radial, axis],
                radius,
                [line.start().y().clone(), line.end().y().clone()],
            )
        };
        validate_circle_iso_image(
            &curve_data,
            edge,
            edge_use.direction,
            circle_center,
            axes,
            circle_radius,
            angles,
        )
    }

    pub(in crate::topology) fn validate_torus_circle_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .ok_or(GeometryError::UnsupportedPcurveContour)?;
        let Curve3ExactData::EllipseArc(curve_data) = curve.exact_data() else {
            unreachable!("circle kind carries ellipse-arc exact data");
        };
        if !curve_data.circle {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let SurfaceExactData::Torus {
            center,
            x,
            y,
            axis,
            major_radius,
            minor_radius,
        } = surface.exact_data()
        else {
            unreachable!("torus kind carries torus exact data");
        };
        let u_constant = real_values_equal(line.start().x(), line.end().x())?;
        let v_constant = real_values_equal(line.start().y(), line.end().y())?;
        if u_constant == v_constant {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let (expected_center, expected_x, expected_y, expected_radius, first_angle, second_angle) =
            if v_constant {
                let v = line.start().y().clone();
                let center_offset = &curve_data.center - &center;
                require_real_equal(
                    &center_offset.dot(&x),
                    &Real::zero(),
                    BuildError::EdgeUseSupportMismatch,
                )?;
                require_real_equal(
                    &center_offset.dot(&y),
                    &Real::zero(),
                    BuildError::EdgeUseSupportMismatch,
                )?;
                require_real_equal(
                    &center_offset.dot(&axis),
                    &(&minor_radius * v.clone().sin()),
                    BuildError::EdgeUseSupportMismatch,
                )?;
                (
                    curve_data.center.clone(),
                    x,
                    y,
                    major_radius + &minor_radius * v.cos(),
                    line.start().x().clone(),
                    line.end().x().clone(),
                )
            } else {
                let u = line.start().x().clone();
                let sine = u.clone().sin();
                let cosine = u.cos();
                let radial = x.clone() * &cosine + y.clone() * &sine;
                let tangent = -x * sine + y * cosine;
                let center_offset = &curve_data.center - &center;
                require_real_equal(
                    &center_offset.dot(&axis),
                    &Real::zero(),
                    BuildError::EdgeUseSupportMismatch,
                )?;
                require_real_equal(
                    &center_offset.dot(&radial),
                    &major_radius,
                    BuildError::EdgeUseSupportMismatch,
                )?;
                require_real_equal(
                    &center_offset.dot(&tangent),
                    &Real::zero(),
                    BuildError::EdgeUseSupportMismatch,
                )?;
                (
                    curve_data.center.clone(),
                    radial,
                    axis,
                    minor_radius,
                    line.start().y().clone(),
                    line.end().y().clone(),
                )
            };
        validate_circle_iso_image(
            &curve_data,
            edge,
            edge_use.direction,
            expected_center,
            [expected_x, expected_y],
            expected_radius,
            [first_angle, second_angle],
        )
    }

    pub(in crate::topology) fn validate_cone_generator_image(
        &self,
        pcurve: &Pcurve,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .ok_or(GeometryError::UnsupportedPcurveContour)?;
        require_real_equal(
            line.start().x(),
            line.end().x(),
            BuildError::EdgeUseSupportMismatch,
        )
    }

    pub(in crate::topology) fn validate_cone_circle_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .ok_or(GeometryError::UnsupportedPcurveContour)?;
        require_real_equal(
            line.start().y(),
            line.end().y(),
            BuildError::EdgeUseSupportMismatch,
        )?;
        let Curve3ExactData::EllipseArc(curve_data) = curve.exact_data() else {
            unreachable!("circle kind carries ellipse-arc exact data");
        };
        if !curve_data.circle {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let SurfaceExactData::Cone {
            apex,
            x,
            y,
            axis,
            semi_angle,
        } = surface.exact_data()
        else {
            unreachable!("cone kind carries cone exact data");
        };
        let v = line.start().y().clone();
        validate_circle_iso_image(
            &curve_data,
            edge,
            edge_use.direction,
            apex + axis * (&v * semi_angle.clone().cos()),
            [x, y],
            v * semi_angle.sin(),
            [line.start().x().clone(), line.end().x().clone()],
        )
    }

    pub(in crate::topology) fn validate_extrusion_line_image(
        &self,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .ok_or(GeometryError::UnsupportedPcurveContour)?;
        let u_constant = real_values_equal(line.start().x(), line.end().x())?;
        if u_constant {
            return Ok(());
        }
        let v_constant = real_values_equal(line.start().y(), line.end().y())?;
        let SurfaceExactData::Extrusion { profile, .. } = surface.exact_data() else {
            unreachable!("extrusion kind carries extrusion exact data");
        };
        if v_constant && matches!(*profile, Curve3ExactData::Line(_)) {
            Ok(())
        } else {
            Err(BuildError::EdgeUseSupportMismatch)
        }
    }

    pub(in crate::topology) fn validate_extrusion_profile_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .ok_or(GeometryError::UnsupportedPcurveContour)?;
        require_real_equal(
            line.start().y(),
            line.end().y(),
            BuildError::EdgeUseSupportMismatch,
        )?;
        if real_values_equal(line.start().x(), line.end().x())? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let (profile_start, profile_end) = if decided_model_order(compare_reals(
            line.start().x(),
            line.end().x(),
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Less
        {
            (line.start().x(), line.end().x())
        } else {
            (line.end().x(), line.start().x())
        };
        let (profile, direction) = surface
            .extrusion_profile_and_direction()
            .expect("extrusion kind carries extrusion geometry");
        let offset = direction.clone() * line.start().y();
        let expected = translated_curve(&profile.subcurve(profile_start, profile_end)?, &offset)?;
        let actual = curve.subcurve(
            edge.domain.represented_start()?,
            edge.domain.represented_end()?,
        )?;
        let reversed = if profile_parameterizations_equal(&actual, &expected)? {
            false
        } else if profile_parameterizations_equal(&actual, &expected.reversed()?)? {
            true
        } else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };

        let edge_domain_span = edge.domain.represented_end()? - edge.domain.represented_start()?;
        let actual_span =
            actual.domain().represented_end()? - actual.domain().represented_start()?;
        let profile_span = profile_end - profile_start;
        let expected_span =
            expected.domain().represented_end()? - expected.domain().represented_start()?;
        for (pcurve_parameter, surface_parameter) in [
            (pcurve.represented_start()?, line.start().x()),
            (pcurve.represented_end()?, line.end().x()),
        ] {
            let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
                pcurve,
                &edge.domain,
                edge_use.direction,
                &crate::Parameter::from_real((pcurve_parameter).clone()),
            )?;
            let actual_parameter = actual.domain().represented_start()?
                + ((edge_parameter
                    .try_real()
                    .ok_or(GeometryError::UnrepresentableParameter)?
                    - edge.domain.represented_start()?)
                    * &actual_span
                    / &edge_domain_span)
                    .map_err(GeometryError::Numeric)?;
            let offset = ((surface_parameter - profile_start) * &expected_span / &profile_span)
                .map_err(GeometryError::Numeric)?;
            let expected_parameter = actual.domain().represented_start()?
                + if reversed {
                    &expected_span - offset
                } else {
                    offset
                };
            require_real_equal(
                &actual_parameter,
                &expected_parameter,
                BuildError::EdgeUseSupportMismatch,
            )?;
        }
        Ok(())
    }

    pub(in crate::topology) fn validate_revolution_meridian_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .ok_or(GeometryError::UnsupportedPcurveContour)?;
        require_real_equal(
            line.start().x(),
            line.end().x(),
            BuildError::EdgeUseSupportMismatch,
        )?;
        if real_values_equal(line.start().y(), line.end().y())? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let (profile_start, profile_end) = if decided_model_order(compare_reals(
            line.start().y(),
            line.end().y(),
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Less
        {
            (line.start().y(), line.end().y())
        } else {
            (line.end().y(), line.start().y())
        };
        let expected = surface
            .revolution_meridian_curve(line.start().x())?
            .subcurve(profile_start, profile_end)?;
        let actual = curve.subcurve(
            edge.domain.represented_start()?,
            edge.domain.represented_end()?,
        )?;
        if !curve_parameterizations_equal(&actual, &expected)? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let edge_domain_span = edge.domain.represented_end()? - edge.domain.represented_start()?;
        let actual_span =
            actual.domain().represented_end()? - actual.domain().represented_start()?;
        let profile_span = profile_end - profile_start;
        let expected_span =
            expected.domain().represented_end()? - expected.domain().represented_start()?;
        for (pcurve_parameter, surface_parameter) in [
            (pcurve.represented_start()?, line.start().y()),
            (pcurve.represented_end()?, line.end().y()),
        ] {
            let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
                pcurve,
                &edge.domain,
                edge_use.direction,
                &crate::Parameter::from_real((pcurve_parameter).clone()),
            )?;
            let actual_parameter = actual.domain().represented_start()?
                + ((edge_parameter
                    .try_real()
                    .ok_or(GeometryError::UnrepresentableParameter)?
                    - edge.domain.represented_start()?)
                    * &actual_span
                    / &edge_domain_span)
                    .map_err(GeometryError::Numeric)?;
            let expected_parameter = expected.domain().represented_start()?
                + ((surface_parameter - profile_start) * &expected_span / &profile_span)
                    .map_err(GeometryError::Numeric)?;
            require_real_equal(
                &actual_parameter,
                &expected_parameter,
                BuildError::EdgeUseSupportMismatch,
            )?;
        }
        Ok(())
    }

    pub(in crate::topology) fn validate_revolution_circle_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .ok_or(GeometryError::UnsupportedPcurveContour)?;
        require_real_equal(
            line.start().y(),
            line.end().y(),
            BuildError::EdgeUseSupportMismatch,
        )?;
        if real_values_equal(line.start().x(), line.end().x())? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let actual = curve.subcurve(
            edge.domain.represented_start()?,
            edge.domain.represented_end()?,
        )?;
        let actual_span =
            actual.domain().represented_end()? - actual.domain().represented_start()?;
        let first_edge_parameter = edge_use.parameter_correspondence.edge_parameter(
            pcurve,
            &edge.domain,
            edge_use.direction,
            &crate::Parameter::from_real((pcurve.represented_start()?).clone()),
        )?;
        let second_edge_parameter = edge_use.parameter_correspondence.edge_parameter(
            pcurve,
            &edge.domain,
            edge_use.direction,
            &crate::Parameter::from_real((pcurve.represented_end()?).clone()),
        )?;
        let (angle_at_start, angle_at_end) = if real_values_equal(
            first_edge_parameter
                .try_real()
                .ok_or(crate::GeometryError::UnrepresentableParameter)?,
            actual.domain().represented_start()?,
        )? {
            require_real_equal(
                second_edge_parameter
                    .try_real()
                    .ok_or(crate::GeometryError::UnrepresentableParameter)?,
                actual.domain().represented_end()?,
                BuildError::EdgeUseSupportMismatch,
            )?;
            (line.start().x(), line.end().x())
        } else {
            require_real_equal(
                first_edge_parameter
                    .try_real()
                    .ok_or(crate::GeometryError::UnrepresentableParameter)?,
                actual.domain().represented_end()?,
                BuildError::EdgeUseSupportMismatch,
            )?;
            require_real_equal(
                second_edge_parameter
                    .try_real()
                    .ok_or(crate::GeometryError::UnrepresentableParameter)?,
                actual.domain().represented_start()?,
                BuildError::EdgeUseSupportMismatch,
            )?;
            (line.end().x(), line.start().x())
        };
        let angle_delta = angle_at_end - angle_at_start;
        let expected_direction = if real_values_equal(&angle_delta, &actual_span)? {
            1
        } else if real_values_equal(&angle_delta, &(-actual_span.clone()))? {
            -1
        } else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let support =
            surface.revolution_latitude_curve(line.start().y(), Real::zero(), Real::one())?;
        let Curve3ExactData::EllipseArc(support) = support.exact_data() else {
            unreachable!("a nonsingular revolution latitude is a circle");
        };
        let expected =
            Curve3::from_exact_data(Curve3ExactData::EllipseArc(Box::new(EllipseArcExactData {
                domain_start: actual.domain().represented_start()?.clone(),
                domain_end: actual.domain().represented_end()?.clone(),
                angle_at_start: angle_at_start.clone(),
                direction: expected_direction,
                ..*support
            })))?;
        if !circle_parameterizations_equal(&actual, &expected)? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        Ok(())
    }
}

/// Two orthonormal radial/tangent frames and the angular rate determine the
/// complete circle parameterization. A center and one tangent do not: a wrong
/// phase or a tilted plane can retain both while changing the radial vector.
fn validate_circle_iso_image(
    curve_data: &EllipseArcExactData,
    edge: &Edge,
    direction: Direction,
    center: Point3,
    axes: [Vector3; 2],
    radius: Real,
    angles: [Real; 2],
) -> Result<(), BuildError> {
    let edge_offset = edge.domain.represented_start()? - &curve_data.domain_start;
    let edge_angle_at_start = if curve_data.direction > 0 {
        &curve_data.angle_at_start + edge_offset
    } else {
        &curve_data.angle_at_start - edge_offset
    };
    let actual = EllipseArcExactData {
        domain_start: edge.domain.represented_start()?.clone(),
        domain_end: edge.domain.represented_end()?.clone(),
        angle_at_start: edge_angle_at_start,
        ..curve_data.clone()
    };
    let actual_span = &actual.domain_end - &actual.domain_start;
    let [first_angle, second_angle] = angles;
    let (angle_at_start, angle_at_end) = match direction {
        Direction::Forward => (first_angle, second_angle),
        Direction::Reversed => (second_angle, first_angle),
    };
    let angle_delta = &angle_at_end - &angle_at_start;
    let expected_direction = if real_values_equal(&angle_delta, &actual_span)? {
        1
    } else if real_values_equal(&angle_delta, &(-actual_span.clone()))? {
        -1
    } else {
        return Err(BuildError::EdgeUseSupportMismatch);
    };
    let [x, y] = axes;
    let expected = EllipseArcExactData {
        circle: true,
        center,
        x,
        y,
        x_radius: radius.clone(),
        y_radius: radius,
        domain_start: actual.domain_start.clone(),
        domain_end: actual.domain_end.clone(),
        angle_at_start,
        direction: expected_direction,
    };
    if circle_data_parameterizations_equal(&actual, &expected)? {
        Ok(())
    } else {
        Err(BuildError::EdgeUseSupportMismatch)
    }
}
