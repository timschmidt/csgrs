//! Validated construction: validation image.

use crate::topology::*;

impl ArenaModelBuilder {
    pub(in crate::topology) fn validate_wire_image(
        &self,
        wire: WireId,
        surface: SurfaceId,
    ) -> Result<(), BuildError> {
        let surface = self.surface_ref(surface)?;
        let wire = self.wire_ref(wire)?;
        for edge_use_id in &wire.edge_uses {
            let edge_use = self.edge_use_ref(*edge_use_id)?;
            let edge = self.edge_ref(edge_use.edge)?;
            let curve = self.curve_ref(edge.curve)?;
            let pcurve = self.pcurve_ref(edge_use.pcurve)?;
            self.validate_edge_use_image(curve, edge, edge_use, pcurve, surface)?;
        }
        Ok(())
    }

    fn validate_edge_use_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        if !edge_use.image_certificate
            && let Some((source, _, _)) = pcurve.curve().source_restriction()
        {
            // The trim's exact endpoint map is checked independently. The
            // complete authored image is then certified by the native validator;
            // restriction cannot change that image agreement. No proof flag or
            // sampled approximation is accepted from an archive.
            let expected = match edge_use.direction {
                Direction::Forward => [edge.domain.start(), edge.domain.end()],
                Direction::Reversed => [edge.domain.end(), edge.domain.start()],
            };
            for (index, (local, expected)) in [pcurve.domain_start()?, pcurve.domain_end()?]
                .into_iter()
                .zip(expected)
                .enumerate()
            {
                let mapped = edge_use.parameter_correspondence.edge_parameter(
                    pcurve,
                    &edge.domain,
                    edge_use.direction,
                    &local,
                )?;
                if mapped.compare(expected)? != std::cmp::Ordering::Equal {
                    return Err(BuildError::ParameterCorrespondenceMismatch {
                        endpoint: if index == 0 {
                            Endpoint::Start
                        } else {
                            Endpoint::End
                        },
                    });
                }
            }
            let ParameterCorrespondence::Affine { scale, offset, .. } =
                &edge_use.parameter_correspondence
            else {
                return Err(GeometryError::UnsupportedPcurveContour.into());
            };
            let source = Pcurve::new(source.clone());
            let start: Parameter = (scale * source.represented_start()? + offset).into();
            let end: Parameter = (scale * source.represented_end()? + offset).into();
            let mut source_edge = edge.clone();
            let mut source_use = edge_use.clone();
            if start.compare(&end)? == std::cmp::Ordering::Less {
                source_edge.domain = ParameterRange::new(start, end)?;
                source_use.direction = Direction::Forward;
            } else {
                source_edge.domain = ParameterRange::new(end, start)?;
                source_use.direction = Direction::Reversed;
            }
            return self.validate_edge_use_image(
                curve,
                &source_edge,
                &source_use,
                &source,
                surface,
            );
        }
        let inherited_image = edge_use.image_certificate;
        let complete_image_certificate_subsumes_endpoint_sampling = matches!(
            (
                curve.kind(),
                pcurve.kind(),
                surface.kind(),
                &edge_use.parameter_correspondence,
            ),
            (
                Curve3Kind::CircleArc,
                CurveFamily2::Line,
                SurfaceKind::Cylinder
                    | SurfaceKind::Sphere
                    | SurfaceKind::Cone
                    | SurfaceKind::Torus,
                ParameterCorrespondence::Affine { .. },
            ) | (
                Curve3Kind::Line,
                CurveFamily2::Line,
                SurfaceKind::Cylinder,
                ParameterCorrespondence::Affine { .. },
            ) | (
                Curve3Kind::CircleArc,
                CurveFamily2::CircularArc,
                SurfaceKind::Plane,
                ParameterCorrespondence::AngularSweep,
            )
        );

        let pcurve_parameters = [pcurve.domain_start()?, pcurve.domain_end()?];
        let expected_edge_parameters = match edge_use.direction {
            Direction::Forward => [edge.domain.start(), edge.domain.end()],
            Direction::Reversed => [edge.domain.end(), edge.domain.start()],
        };
        for (index, (pcurve_parameter, expected_edge_parameter)) in pcurve_parameters
            .into_iter()
            .zip(expected_edge_parameters)
            .enumerate()
        {
            let endpoint = if index == 0 {
                Endpoint::Start
            } else {
                Endpoint::End
            };
            let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
                pcurve,
                &edge.domain,
                edge_use.direction,
                &pcurve_parameter,
            )?;
            if edge_parameter.compare(expected_edge_parameter)? != std::cmp::Ordering::Equal {
                return Err(BuildError::ParameterCorrespondenceMismatch { endpoint });
            }
            if inherited_image {
                continue;
            }
            let edge_point = curve.point_at(&edge_parameter)?;
            let surface_parameter = pcurve.evaluate_real(
                pcurve_parameter
                    .try_real()
                    .ok_or(GeometryError::UnrepresentableParameter)?,
            )?;
            let surface_point = surface.point_at(&surface_parameter)?;
            if !complete_image_certificate_subsumes_endpoint_sampling
                && !edge_point.coincides_with(&surface_point.into())?
            {
                return Err(BuildError::EdgeUseImageMismatch { endpoint });
            }
        }
        if inherited_image {
            return Ok(());
        }

        match (
            curve.kind(),
            pcurve.kind(),
            surface.kind(),
            &edge_use.parameter_correspondence,
        ) {
            (
                Curve3Kind::Line,
                CurveFamily2::Line,
                SurfaceKind::Plane,
                ParameterCorrespondence::Affine { .. },
            ) => {
                // Both composed images are affine. Exact endpoint equality
                // therefore certifies the complete interval.
            }
            (
                Curve3Kind::CircleArc,
                CurveFamily2::CircularArc,
                SurfaceKind::Plane,
                ParameterCorrespondence::AngularSweep,
            ) => {
                self.validate_planar_circle_image(curve, edge, edge_use, pcurve, surface)?;
            }
            (
                Curve3Kind::RationalBezier,
                CurveFamily2::RationalQuadraticBezier | CurveFamily2::RationalBezier,
                SurfaceKind::Plane,
                ParameterCorrespondence::Affine { .. },
            )
            | (
                Curve3Kind::Nurbs,
                CurveFamily2::Nurbs,
                SurfaceKind::Plane,
                ParameterCorrespondence::Affine { .. },
            ) => {
                self.validate_planar_spline_image(curve, edge_use, pcurve, surface)?;
            }
            (
                Curve3Kind::Line,
                CurveFamily2::Line,
                SurfaceKind::Cylinder,
                ParameterCorrespondence::Affine { .. },
            ) => self.validate_cylinder_axial_line_image(curve, edge, edge_use, pcurve, surface)?,
            (
                Curve3Kind::CircleArc,
                CurveFamily2::Line,
                SurfaceKind::Cylinder,
                ParameterCorrespondence::Affine { .. },
            ) => {
                self.validate_cylinder_circle_image(curve, edge, edge_use, pcurve, surface)?;
            }
            (
                Curve3Kind::CircleArc,
                CurveFamily2::Line,
                SurfaceKind::Sphere,
                ParameterCorrespondence::Affine { .. },
            ) => {
                self.validate_sphere_circle_image(curve, edge, edge_use, pcurve, surface)?;
            }
            (
                Curve3Kind::CircleArc,
                CurveFamily2::Line,
                SurfaceKind::Torus,
                ParameterCorrespondence::Affine { .. },
            ) => {
                self.validate_torus_circle_image(curve, edge, edge_use, pcurve, surface)?;
            }
            (
                Curve3Kind::Line,
                CurveFamily2::Line,
                SurfaceKind::Cone,
                ParameterCorrespondence::Affine { .. },
            ) => self.validate_cone_generator_image(pcurve)?,
            (
                Curve3Kind::CircleArc,
                CurveFamily2::Line,
                SurfaceKind::Cone,
                ParameterCorrespondence::Affine { .. },
            ) => {
                self.validate_cone_circle_image(curve, edge, edge_use, pcurve, surface)?;
            }
            (
                Curve3Kind::Line,
                CurveFamily2::Line,
                SurfaceKind::Extrusion,
                ParameterCorrespondence::Affine { .. },
            ) => self.validate_extrusion_line_image(pcurve, surface)?,
            (
                Curve3Kind::CircleArc
                | Curve3Kind::EllipseArc
                | Curve3Kind::RationalBezier
                | Curve3Kind::Nurbs,
                CurveFamily2::Line,
                SurfaceKind::Extrusion,
                ParameterCorrespondence::Affine { .. },
            ) => {
                self.validate_extrusion_profile_image(curve, edge, edge_use, pcurve, surface)?;
            }
            (
                Curve3Kind::Line,
                CurveFamily2::Line,
                SurfaceKind::Revolution,
                ParameterCorrespondence::Affine { .. },
            )
            | (
                Curve3Kind::RationalBezier,
                CurveFamily2::Line,
                SurfaceKind::Revolution,
                ParameterCorrespondence::Affine { .. },
            )
            | (
                Curve3Kind::Nurbs,
                CurveFamily2::Line,
                SurfaceKind::Revolution,
                ParameterCorrespondence::Affine { .. },
            ) => self.validate_revolution_meridian_image(curve, edge, edge_use, pcurve, surface)?,
            (
                Curve3Kind::CircleArc,
                CurveFamily2::Line,
                SurfaceKind::Revolution,
                ParameterCorrespondence::Affine { .. },
            ) => {
                let line = pcurve
                    .line_segment()
                    .ok_or(GeometryError::UnsupportedPcurveContour)?;
                if real_values_equal(line.start().x(), line.end().x())? {
                    self.validate_revolution_meridian_image(
                        curve, edge, edge_use, pcurve, surface,
                    )?;
                } else {
                    self.validate_revolution_circle_image(curve, edge, edge_use, pcurve, surface)?;
                }
            }
            (
                Curve3Kind::RationalBezier,
                CurveFamily2::Line,
                SurfaceKind::RationalBezier,
                ParameterCorrespondence::Affine { .. },
            )
            | (
                Curve3Kind::RationalBezier,
                CurveFamily2::Line,
                SurfaceKind::Nurbs,
                ParameterCorrespondence::Affine { .. },
            )
            | (
                Curve3Kind::Line,
                CurveFamily2::Line,
                SurfaceKind::RationalBezier,
                ParameterCorrespondence::Affine { .. },
            )
            | (
                Curve3Kind::Nurbs,
                CurveFamily2::Line,
                SurfaceKind::Nurbs,
                ParameterCorrespondence::Affine { .. },
            )
            | (
                Curve3Kind::Line,
                CurveFamily2::Line,
                SurfaceKind::Nurbs,
                ParameterCorrespondence::Affine { .. },
            ) => {
                self.validate_tensor_surface_iso_image(curve, edge, edge_use, pcurve, surface)?;
            }
            (
                Curve3Kind::RationalBezier,
                CurveFamily2::RationalBezier,
                SurfaceKind::RationalBezier,
                ParameterCorrespondence::Affine { .. },
            ) => {
                self.validate_rational_tensor_graph_image(curve, edge, edge_use, pcurve, surface)?;
            }
            (
                Curve3Kind::RationalBezier,
                CurveFamily2::RationalBezier,
                SurfaceKind::Nurbs,
                ParameterCorrespondence::Affine { .. },
            ) => {
                self.validate_rational_tensor_graph_image(curve, edge, edge_use, pcurve, surface)?;
            }
            (
                Curve3Kind::Nurbs,
                CurveFamily2::Nurbs,
                SurfaceKind::RationalBezier,
                ParameterCorrespondence::Affine { .. },
            ) => {
                self.validate_rational_tensor_graph_image(curve, edge, edge_use, pcurve, surface)?;
            }
            (
                Curve3Kind::Nurbs,
                CurveFamily2::RationalBezier,
                SurfaceKind::Nurbs,
                ParameterCorrespondence::Affine { .. },
            )
            | (
                Curve3Kind::Nurbs,
                CurveFamily2::Nurbs,
                SurfaceKind::Nurbs,
                ParameterCorrespondence::Affine { .. },
            ) => {
                self.validate_nurbs_tensor_graph_image(curve, edge_use, pcurve, surface)?;
            }
            _ => {
                return Err(BuildError::UnsupportedEdgeUseAgreement {
                    curve: curve.kind(),
                    pcurve: pcurve.kind(),
                    surface: surface.kind(),
                });
            }
        }
        Ok(())
    }

    pub(in crate::topology) fn validate_planar_spline_image(
        &self,
        curve: &Curve3,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let lift = |point: &CurvePoint2| {
            surface.point_at(&Point2::new(point.x().clone(), point.y().clone()))
        };
        let expected = match pcurve.curve().geometry() {
            Some(CurveGeometry2::RationalBezier(planar)) => Curve3::rational_bezier(
                planar
                    .affine_control_points()
                    .ok_or(GeometryError::UnsupportedIntersection)?
                    .iter()
                    .map(lift)
                    .collect::<Result<Vec<_>, _>>()?,
                planar.weights().to_vec(),
            )?,
            Some(CurveGeometry2::RationalQuadraticBezier(planar)) => Curve3::rational_bezier(
                planar
                    .control_points()
                    .into_iter()
                    .map(lift)
                    .collect::<Result<Vec<_>, _>>()?,
                planar.weights().into_iter().cloned().collect(),
            )?,
            Some(CurveGeometry2::Nurbs(planar)) => Curve3::nurbs(
                planar.degree(),
                planar
                    .affine_control_points()
                    .ok_or(BuildError::EdgeUseSupportMismatch)?
                    .iter()
                    .map(lift)
                    .collect::<Result<Vec<_>, _>>()?,
                planar.weights().to_vec(),
                planar.knots().to_vec(),
            )?,
            _ => return Err(BuildError::EdgeUseSupportMismatch),
        };
        let expected = match edge_use.direction {
            Direction::Forward => expected,
            Direction::Reversed => expected.reversed()?,
        };
        if curve_parameterizations_equal(curve, &expected)? {
            Ok(())
        } else {
            Err(BuildError::EdgeUseSupportMismatch)
        }
    }
}
