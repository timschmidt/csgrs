//! Validated construction: validation analytic_image.

use crate::topology::*;

impl ArenaModelBuilder {
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
            .expect("circular pcurve kind carries circular geometry");
        let center_parameter = Point2::new(arc.center().x().clone(), arc.center().y().clone());
        let mapped_center = surface.point_at(&center_parameter)?;
        require_point_equal(
            &mapped_center,
            &data.center,
            BuildError::EdgeUseSupportMismatch,
        )?;

        let mapped_start = surface.point_at(&Point2::new(
            arc.start().x().clone(),
            arc.start().y().clone(),
        ))?;
        let mapped_end =
            surface.point_at(&Point2::new(arc.end().x().clone(), arc.end().y().clone()))?;
        let (directed_start_vertex, directed_end_vertex) = match edge_use.direction {
            Direction::Forward => (edge.start, edge.end),
            Direction::Reversed => (edge.end, edge.start),
        };
        require_point_equal(
            &mapped_start,
            self.vertex_ref(directed_start_vertex)?.point(),
            BuildError::EdgeUseSupportMismatch,
        )?;
        require_point_equal(
            &mapped_end,
            self.vertex_ref(directed_end_vertex)?.point(),
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
            .expect("line pcurve kind carries line geometry");
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
        for parameter in [pcurve.domain_start()?, pcurve.domain_end()?] {
            let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
                pcurve,
                &edge.domain,
                edge_use.direction,
                parameter,
            )?;
            let edge_point = curve.evaluate_real(&edge_parameter)?;
            let axial = (&edge_point - &origin).dot(&axis);
            let surface_parameter = pcurve.point_at(parameter)?;
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
            .expect("line pcurve kind carries line geometry");
        require_real_equal(
            line.start().y(),
            line.end().y(),
            BuildError::EdgeUseSupportMismatch,
        )?;
        let Curve3ExactData::EllipseArc(curve_data) = curve.exact_data() else {
            unreachable!("circle kind carries ellipse-arc exact data");
        };
        let SurfaceExactData::Cylinder { origin, axis, .. } = surface.exact_data() else {
            unreachable!("cylinder kind carries cylinder exact data");
        };
        let expected_center = origin + axis * line.start().y();
        require_point_equal(
            &curve_data.center,
            &expected_center,
            BuildError::EdgeUseSupportMismatch,
        )?;

        let pcurve_span = pcurve.domain_end()? - pcurve.domain_start()?;
        let du = line.end().x() - line.start().x();
        let du_dt = (du / pcurve_span).map_err(GeometryError::Numeric)?;
        let surface_parameter = Point2::new(line.start().x().clone(), line.start().y().clone());
        let surface_tangent = surface.partials_at(&surface_parameter)?.u().clone() * du_dt;
        let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
            pcurve,
            &edge.domain,
            edge_use.direction,
            pcurve.domain_start()?,
        )?;
        let edge_rate = match &edge_use.parameter_correspondence {
            ParameterCorrespondence::Affine { scale, .. } => scale,
            ParameterCorrespondence::AngularSweep => unreachable!("matched affine relation"),
        };
        let edge_tangent = curve.derivative_at(&edge_parameter, 1)?.vector().clone() * edge_rate;
        require_vector_equal(
            &surface_tangent,
            &edge_tangent,
            BuildError::EdgeUseSupportMismatch,
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
            .expect("line pcurve kind carries line geometry");
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
        let (expected_radius, surface_parameter, varying_u) = if v_constant {
            let latitude = line.start().y().clone();
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
                &(&radius * latitude.clone().sin()),
                BuildError::EdgeUseSupportMismatch,
            )?;
            (
                &radius * latitude.clone().cos(),
                Point2::new(line.start().x().clone(), latitude),
                true,
            )
        } else {
            require_point_equal(
                &curve_data.center,
                &center,
                BuildError::EdgeUseSupportMismatch,
            )?;
            (
                radius,
                Point2::new(line.start().x().clone(), line.start().y().clone()),
                false,
            )
        };
        require_real_equal(
            &curve_data.x_radius,
            &expected_radius,
            BuildError::EdgeUseSupportMismatch,
        )?;
        require_real_equal(
            &curve_data.y_radius,
            &expected_radius,
            BuildError::EdgeUseSupportMismatch,
        )?;

        let pcurve_span = pcurve.domain_end()? - pcurve.domain_start()?;
        let parameter_delta = if varying_u {
            line.end().x() - line.start().x()
        } else {
            line.end().y() - line.start().y()
        };
        let parameter_rate = (parameter_delta / pcurve_span).map_err(GeometryError::Numeric)?;
        let partials = surface.partials_at(&surface_parameter)?;
        let surface_tangent = if varying_u {
            partials.u().clone()
        } else {
            partials.v().clone()
        } * parameter_rate;
        let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
            pcurve,
            &edge.domain,
            edge_use.direction,
            pcurve.domain_start()?,
        )?;
        let edge_rate = match &edge_use.parameter_correspondence {
            ParameterCorrespondence::Affine { scale, .. } => scale,
            ParameterCorrespondence::AngularSweep => unreachable!("matched affine relation"),
        };
        let edge_tangent = curve.derivative_at(&edge_parameter, 1)?.vector().clone() * edge_rate;
        require_vector_equal(
            &surface_tangent,
            &edge_tangent,
            BuildError::EdgeUseSupportMismatch,
        )?;
        Ok(())
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
            .expect("line pcurve kind carries line geometry");
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
        require_real_equal(
            &curve_data.x_radius,
            &expected_radius,
            BuildError::EdgeUseSupportMismatch,
        )?;

        let edge_offset = edge.domain.start() - &curve_data.domain_start;
        let edge_angle_at_start = if curve_data.direction > 0 {
            &curve_data.angle_at_start + edge_offset
        } else {
            &curve_data.angle_at_start - edge_offset
        };
        let actual = EllipseArcExactData {
            domain_start: edge.domain.start().clone(),
            domain_end: edge.domain.end().clone(),
            angle_at_start: edge_angle_at_start,
            ..*curve_data
        };
        let actual_span = &actual.domain_end - &actual.domain_start;
        let (angle_at_start, angle_at_end) = match edge_use.direction {
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
        let expected = EllipseArcExactData {
            circle: true,
            center: expected_center,
            x: expected_x,
            y: expected_y,
            x_radius: expected_radius.clone(),
            y_radius: expected_radius,
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

    pub(in crate::topology) fn validate_cone_generator_image(
        &self,
        pcurve: &Pcurve,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .expect("line pcurve kind carries line geometry");
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
            .expect("line pcurve kind carries line geometry");
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
            axis,
            semi_angle,
            ..
        } = surface.exact_data()
        else {
            unreachable!("cone kind carries cone exact data");
        };
        let v = line.start().y().clone();
        let expected_center = apex + axis * (&v * semi_angle.clone().cos());
        let expected_radius = v * semi_angle.sin();
        require_point_equal(
            &curve_data.center,
            &expected_center,
            BuildError::EdgeUseSupportMismatch,
        )?;
        require_real_equal(
            &curve_data.x_radius,
            &expected_radius,
            BuildError::EdgeUseSupportMismatch,
        )?;
        require_real_equal(
            &curve_data.y_radius,
            &expected_radius,
            BuildError::EdgeUseSupportMismatch,
        )?;

        let pcurve_span = pcurve.domain_end()? - pcurve.domain_start()?;
        let du_dt =
            ((line.end().x() - line.start().x()) / &pcurve_span).map_err(GeometryError::Numeric)?;
        let surface_parameter = Point2::new(line.start().x().clone(), line.start().y().clone());
        let surface_tangent = surface.partials_at(&surface_parameter)?.u().clone() * du_dt;
        let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
            pcurve,
            &edge.domain,
            edge_use.direction,
            pcurve.domain_start()?,
        )?;
        let edge_rate = match &edge_use.parameter_correspondence {
            ParameterCorrespondence::Affine { scale, .. } => scale,
            ParameterCorrespondence::AngularSweep => unreachable!("matched affine relation"),
        };
        let edge_tangent = curve.derivative_at(&edge_parameter, 1)?.vector().clone() * edge_rate;
        require_vector_equal(
            &surface_tangent,
            &edge_tangent,
            BuildError::EdgeUseSupportMismatch,
        )
    }

    pub(in crate::topology) fn validate_extrusion_line_image(
        &self,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .expect("line pcurve kind carries line geometry");
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
            .expect("line pcurve kind carries line geometry");
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
        let actual = curve.subcurve(edge.domain.start(), edge.domain.end())?;
        if !curve_parameterizations_equal(&actual, &expected)? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }

        let edge_domain_span = edge.domain.end() - edge.domain.start();
        let actual_span = actual.domain().end() - actual.domain().start();
        let profile_span = profile_end - profile_start;
        let expected_span = expected.domain().end() - expected.domain().start();
        for (pcurve_parameter, surface_parameter) in [
            (pcurve.domain_start()?, line.start().x()),
            (pcurve.domain_end()?, line.end().x()),
        ] {
            let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
                pcurve,
                &edge.domain,
                edge_use.direction,
                pcurve_parameter,
            )?;
            let actual_parameter = actual.domain().start()
                + ((&edge_parameter - edge.domain.start()) * &actual_span / &edge_domain_span)
                    .map_err(GeometryError::Numeric)?;
            let expected_parameter = expected.domain().start()
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
            .expect("line pcurve kind carries line geometry");
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
        let actual = curve.subcurve(edge.domain.start(), edge.domain.end())?;
        if !curve_parameterizations_equal(&actual, &expected)? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let edge_domain_span = edge.domain.end() - edge.domain.start();
        let actual_span = actual.domain().end() - actual.domain().start();
        let profile_span = profile_end - profile_start;
        let expected_span = expected.domain().end() - expected.domain().start();
        for (pcurve_parameter, surface_parameter) in [
            (pcurve.domain_start()?, line.start().y()),
            (pcurve.domain_end()?, line.end().y()),
        ] {
            let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
                pcurve,
                &edge.domain,
                edge_use.direction,
                pcurve_parameter,
            )?;
            let actual_parameter = actual.domain().start()
                + ((&edge_parameter - edge.domain.start()) * &actual_span / &edge_domain_span)
                    .map_err(GeometryError::Numeric)?;
            let expected_parameter = expected.domain().start()
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
            .expect("line pcurve kind carries line geometry");
        require_real_equal(
            line.start().y(),
            line.end().y(),
            BuildError::EdgeUseSupportMismatch,
        )?;
        if real_values_equal(line.start().x(), line.end().x())? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let actual = curve.subcurve(edge.domain.start(), edge.domain.end())?;
        let actual_span = actual.domain().end() - actual.domain().start();
        let first_edge_parameter = edge_use.parameter_correspondence.edge_parameter(
            pcurve,
            &edge.domain,
            edge_use.direction,
            pcurve.domain_start()?,
        )?;
        let second_edge_parameter = edge_use.parameter_correspondence.edge_parameter(
            pcurve,
            &edge.domain,
            edge_use.direction,
            pcurve.domain_end()?,
        )?;
        let (angle_at_start, angle_at_end) =
            if real_values_equal(&first_edge_parameter, actual.domain().start())? {
                require_real_equal(
                    &second_edge_parameter,
                    actual.domain().end(),
                    BuildError::EdgeUseSupportMismatch,
                )?;
                (line.start().x(), line.end().x())
            } else {
                require_real_equal(
                    &first_edge_parameter,
                    actual.domain().end(),
                    BuildError::EdgeUseSupportMismatch,
                )?;
                require_real_equal(
                    &second_edge_parameter,
                    actual.domain().start(),
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
                domain_start: actual.domain().start().clone(),
                domain_end: actual.domain().end().clone(),
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
