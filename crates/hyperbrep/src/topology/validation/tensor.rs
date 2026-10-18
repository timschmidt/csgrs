//! Validated construction: validation tensor.

use crate::topology::*;

impl ArenaModelBuilder {
    pub(in crate::topology) fn validate_rational_tensor_graph_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        if self.validate_affine_bilinear_inverse_pcurve(curve, edge, edge_use, pcurve, surface)? {
            return Ok(());
        }
        if self.validate_affine_tensor_parameter_image(curve, edge_use, pcurve, surface)? {
            return Ok(());
        }
        if let SurfaceExactData::Nurbs {
            u_degree,
            v_degree,
            control_points: surface_points,
            weights: surface_weights,
            u_knots,
            v_knots,
        } = surface.exact_data()
            && u_degree == 1
            && v_degree == 1
            && surface_points.len() == 2
            && surface_points
                .iter()
                .zip(&surface_weights)
                .all(|(points, weights)| points.len() == 2 && weights.len() == 2)
        {
            let oriented_pcurve = match edge_use.direction {
                Direction::Forward => pcurve.clone(),
                Direction::Reversed => pcurve.reversed()?,
            };
            let normalized = normalize_single_span_nurbs_pcurve(
                oriented_pcurve.curve(),
                &u_knots[1],
                &u_knots[2],
                &v_knots[1],
                &v_knots[2],
            )?;
            if let Some(expected) =
                rational_bilinear_parameter_curve(&surface_points, &surface_weights, &normalized)?
                && curve_parameterizations_equal(curve, &expected)?
            {
                return Ok(());
            }
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let SurfaceExactData::RationalBezier {
            control_points: surface_points,
            weights: surface_weights,
        } = surface.exact_data()
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let Curve3ExactData::RationalBezier {
            control_points: curve_points,
            weights: curve_weights,
        } = curve.exact_data()
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let oriented_pcurve = match edge_use.direction {
            Direction::Forward => pcurve.clone(),
            Direction::Reversed => pcurve.reversed()?,
        };
        if let Some(expected) = rational_bilinear_parameter_curve(
            &surface_points,
            &surface_weights,
            oriented_pcurve.curve(),
        )? && curve_parameterizations_equal(curve, &expected)?
        {
            return Ok(());
        }
        if surface_points
            .iter()
            .zip(&surface_weights)
            .all(|(points, weights)| points.len() == 2 && weights.len() == 2)
        {
            let trimmed = self.trim_rational_tensor_profile(
                &surface_points,
                &surface_weights,
                edge_use,
                pcurve,
                SurfaceIsoAxis::V,
            )?;
            match self.validate_tensor_graph_controls(
                &curve_points,
                &curve_weights,
                &trimmed.points,
                &trimmed.weights,
                edge_use,
                pcurve,
                &trimmed.start,
                &(&trimmed.end - &trimmed.start),
                &Real::one(),
                &Real::zero(),
                SurfaceIsoAxis::V,
            ) {
                Ok(()) => return Ok(()),
                Err(BuildError::EdgeUseSupportMismatch) => {}
                Err(error) => return Err(error),
            }
        }
        let Some((transposed_points, transposed_weights)) =
            transpose_two_row_tensor(&surface_points, &surface_weights)
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let trimmed = self.trim_rational_tensor_profile(
            &transposed_points,
            &transposed_weights,
            edge_use,
            pcurve,
            SurfaceIsoAxis::U,
        )?;
        self.validate_tensor_graph_controls(
            &curve_points,
            &curve_weights,
            &trimmed.points,
            &trimmed.weights,
            edge_use,
            pcurve,
            &trimmed.start,
            &(&trimmed.end - &trimmed.start),
            &Real::one(),
            &Real::zero(),
            SurfaceIsoAxis::U,
        )
    }

    pub(in crate::topology) fn validate_affine_bilinear_inverse_pcurve(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<bool, BuildError> {
        let restricted = curve.subcurve(
            (edge.domain.start())
                .try_real()
                .ok_or(crate::GeometryError::UnrepresentableParameter)?,
            (edge.domain.end())
                .try_real()
                .ok_or(crate::GeometryError::UnrepresentableParameter)?,
        )?;
        let directed = match edge_use.direction {
            Direction::Forward => restricted,
            Direction::Reversed => restricted.reversed()?,
        };
        let Some(expected) = surface.affine_bilinear_inverse_pcurve(&directed)? else {
            return Ok(false);
        };
        validate_projective_pcurve_equal(pcurve.curve(), &expected)?;
        Ok(true)
    }

    pub(in crate::topology) fn validate_nurbs_tensor_graph_image(
        &self,
        curve: &Curve3,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        if self.validate_affine_tensor_parameter_image(curve, edge_use, pcurve, surface)? {
            return Ok(());
        }
        let SurfaceExactData::Nurbs {
            u_degree,
            v_degree,
            control_points: surface_points,
            weights: surface_weights,
            u_knots,
            v_knots,
            ..
        } = surface.exact_data()
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let Curve3ExactData::Nurbs {
            degree,
            control_points: curve_points,
            weights: curve_weights,
            knots: curve_knots,
        } = curve.exact_data()
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        if u_degree == 1
            && degree == v_degree
            && !surface_points.is_empty()
            && surface_points
                .iter()
                .zip(&surface_weights)
                .all(|(points, weights)| points.len() == 2 && weights.len() == 2)
        {
            let trimmed = self.trim_nurbs_tensor_profile(
                degree,
                &v_knots,
                &surface_points,
                &surface_weights,
                edge_use,
                pcurve,
                SurfaceIsoAxis::V,
            )?;
            let coefficient_offset = u_knots[u_degree].clone();
            let coefficient_scale = &u_knots[surface_points[0].len()] - &coefficient_offset;
            if real_slices_equal(&curve_knots, &trimmed.knots)? {
                match self.validate_nurbs_tensor_graph_controls(
                    degree,
                    &trimmed.knots,
                    &curve_points,
                    &curve_weights,
                    &trimmed.points,
                    &trimmed.weights,
                    edge_use,
                    pcurve,
                    &coefficient_scale,
                    &coefficient_offset,
                    SurfaceIsoAxis::V,
                ) {
                    Ok(()) => return Ok(()),
                    Err(BuildError::EdgeUseSupportMismatch) => {}
                    Err(error) => return Err(error),
                }
            }
        }
        if v_degree != 1
            || degree != u_degree
            || surface_points.len() != 2
            || surface_points
                .iter()
                .zip(&surface_weights)
                .any(|(points, weights)| points.is_empty() || points.len() != weights.len())
        {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let Some((transposed_points, transposed_weights)) =
            transpose_two_row_tensor(&surface_points, &surface_weights)
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let trimmed = self.trim_nurbs_tensor_profile(
            degree,
            &u_knots,
            &transposed_points,
            &transposed_weights,
            edge_use,
            pcurve,
            SurfaceIsoAxis::U,
        )?;
        if !real_slices_equal(&curve_knots, &trimmed.knots)? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let coefficient_offset = v_knots[v_degree].clone();
        let coefficient_scale = &v_knots[surface_points.len()] - &coefficient_offset;
        self.validate_nurbs_tensor_graph_controls(
            degree,
            &trimmed.knots,
            &curve_points,
            &curve_weights,
            &trimmed.points,
            &trimmed.weights,
            edge_use,
            pcurve,
            &coefficient_scale,
            &coefficient_offset,
            SurfaceIsoAxis::U,
        )
    }

    pub(in crate::topology) fn validate_affine_tensor_parameter_image(
        &self,
        curve: &Curve3,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<bool, BuildError> {
        let Some(parameter_plane) = surface.affine_parameter_plane()? else {
            return Ok(false);
        };
        let Some((scale, offset)) = edge_use.parameter_correspondence.affine_coefficients() else {
            return Ok(false);
        };
        let origin = parameter_plane
            .plane_origin()
            .expect("affine parameter certificate is a plane");
        let (u, v) = parameter_plane
            .plane_directions()
            .expect("affine parameter certificate is a plane");
        let Some(expected) = crate::geometry::lift_curve_from_plane_frame_with_affine_parameter(
            pcurve.curve(),
            origin,
            u,
            v,
            scale,
            offset,
        )?
        else {
            return Ok(false);
        };
        curve_parameterizations_equal(curve, &expected)
    }

    pub(in crate::topology) fn tensor_graph_profile_interval(
        &self,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        profile_axis: SurfaceIsoAxis,
    ) -> Result<(Real, Real), BuildError> {
        let oriented = match edge_use.direction {
            Direction::Forward => pcurve.clone(),
            Direction::Reversed => pcurve.reversed()?,
        };
        let start = oriented.curve().start();
        let end = oriented.curve().end();
        let (start, end) = match profile_axis {
            SurfaceIsoAxis::U => (
                (start)
                    .coordinates()
                    .ok_or(GeometryError::UnsupportedPcurveContour)?
                    .x(),
                (end)
                    .coordinates()
                    .ok_or(GeometryError::UnsupportedPcurveContour)?
                    .x(),
            ),
            SurfaceIsoAxis::V => (
                (start)
                    .coordinates()
                    .ok_or(GeometryError::UnsupportedPcurveContour)?
                    .y(),
                (end)
                    .coordinates()
                    .ok_or(GeometryError::UnsupportedPcurveContour)?
                    .y(),
            ),
        };
        if decided_model_order(compare_reals(start, end, crate::STRICT_PREDICATES))?
            != std::cmp::Ordering::Less
        {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        Ok((start.clone(), end.clone()))
    }

    pub(in crate::topology) fn trim_rational_tensor_profile(
        &self,
        surface_points: &[Vec<Point3>],
        surface_weights: &[Vec<Real>],
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        profile_axis: SurfaceIsoAxis,
    ) -> Result<TrimmedRationalTensorProfile, BuildError> {
        let (start, end) = self.tensor_graph_profile_interval(edge_use, pcurve, profile_axis)?;
        let profile = Curve3::rational_bezier(
            surface_points.iter().map(|row| row[0].clone()).collect(),
            surface_weights.iter().map(|row| row[0].clone()).collect(),
        )?
        .subcurve(&start, &end)?;
        let Curve3ExactData::RationalBezier {
            control_points,
            weights,
        } = profile.exact_data()
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let direction = &surface_points[0][1] - &surface_points[0][0];
        Ok(TrimmedRationalTensorProfile {
            points: translation_tensor_rows(&control_points, &direction),
            weights: duplicated_tensor_weights(&weights),
            start,
            end,
        })
    }

    #[allow(clippy::too_many_arguments)]
    pub(in crate::topology) fn trim_nurbs_tensor_profile(
        &self,
        degree: usize,
        knots: &[Real],
        surface_points: &[Vec<Point3>],
        surface_weights: &[Vec<Real>],
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        profile_axis: SurfaceIsoAxis,
    ) -> Result<TrimmedNurbsTensorProfile, BuildError> {
        let (start, end) = self.tensor_graph_profile_interval(edge_use, pcurve, profile_axis)?;
        let profile = Curve3::nurbs(
            degree,
            surface_points.iter().map(|row| row[0].clone()).collect(),
            surface_weights.iter().map(|row| row[0].clone()).collect(),
            knots.to_vec(),
        )?
        .subcurve(&start, &end)?;
        let Curve3ExactData::Nurbs {
            control_points,
            weights,
            knots,
            ..
        } = profile.exact_data()
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let direction = &surface_points[0][1] - &surface_points[0][0];
        Ok(TrimmedNurbsTensorProfile {
            points: translation_tensor_rows(&control_points, &direction),
            weights: duplicated_tensor_weights(&weights),
            knots,
        })
    }

    #[allow(clippy::too_many_arguments)]
    pub(in crate::topology) fn validate_nurbs_tensor_graph_controls(
        &self,
        degree: usize,
        knots: &[Real],
        curve_points: &[Point3],
        curve_weights: &[Real],
        surface_points: &[Vec<Point3>],
        surface_weights: &[Vec<Real>],
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        coefficient_scale: &Real,
        coefficient_offset: &Real,
        profile_axis: SurfaceIsoAxis,
    ) -> Result<(), BuildError> {
        let coefficients = self.validate_tensor_graph_spatial_controls(
            curve_points,
            curve_weights,
            surface_points,
            surface_weights,
        )?;
        let graph_coefficients = coefficients
            .iter()
            .map(|coefficient| coefficient_offset + coefficient_scale * coefficient)
            .collect::<Vec<_>>();
        let expected = materialize_nurbs_parameter_graph(
            degree,
            &graph_coefficients,
            &surface_weights
                .iter()
                .map(|weights| weights[0].clone())
                .collect::<Vec<_>>(),
            knots,
            profile_axis,
        )?;
        let oriented_pcurve = match edge_use.direction {
            Direction::Forward => pcurve.clone(),
            Direction::Reversed => pcurve.reversed()?,
        };
        validate_projective_pcurve_equal(oriented_pcurve.curve(), &expected)
    }

    #[allow(clippy::too_many_arguments)]
    pub(in crate::topology) fn validate_tensor_graph_controls(
        &self,
        curve_points: &[Point3],
        curve_weights: &[Real],
        surface_points: &[Vec<Point3>],
        surface_weights: &[Vec<Real>],
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        parameter_start: &Real,
        parameter_span: &Real,
        coefficient_scale: &Real,
        coefficient_offset: &Real,
        profile_axis: SurfaceIsoAxis,
    ) -> Result<(), BuildError> {
        let coefficients = self.validate_tensor_graph_spatial_controls(
            curve_points,
            curve_weights,
            surface_points,
            surface_weights,
        )?;
        let oriented_pcurve = match edge_use.direction {
            Direction::Forward => pcurve.clone(),
            Direction::Reversed => pcurve.reversed()?,
        };
        let Some(CurveGeometry2::RationalBezier(graph)) = oriented_pcurve.curve().geometry() else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let graph_coefficients = coefficients
            .iter()
            .map(|coefficient| coefficient_offset + coefficient_scale * coefficient)
            .collect::<Vec<_>>();
        let (expected_points, expected_weights) = rational_tensor_graph_controls(
            &surface_weights
                .iter()
                .map(|weights| weights[0].clone())
                .collect::<Vec<_>>(),
            &graph_coefficients,
            parameter_start,
            parameter_span,
            profile_axis,
        )?;
        validate_rational_pcurve_controls(graph, &expected_points, &expected_weights)
    }

    pub(in crate::topology) fn validate_tensor_graph_spatial_controls(
        &self,
        curve_points: &[Point3],
        curve_weights: &[Real],
        surface_points: &[Vec<Point3>],
        surface_weights: &[Vec<Real>],
    ) -> Result<Vec<Real>, BuildError> {
        if surface_points.is_empty()
            || surface_points.len() != surface_weights.len()
            || surface_points
                .iter()
                .zip(surface_weights)
                .any(|(points, weights)| points.len() != 2 || weights.len() != 2)
        {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        for weights in surface_weights {
            if !real_values_equal(&weights[0], &weights[1])? {
                return Err(BuildError::EdgeUseSupportMismatch);
            }
        }
        let direction = &surface_points[0][1] - &surface_points[0][0];
        let direction_axis = (0..3)
            .find_map(|axis| {
                match compare_reals(&direction.0[axis], &Real::zero(), crate::STRICT_PREDICATES) {
                    PredicateOutcome::Decided {
                        value: std::cmp::Ordering::Equal,
                        ..
                    } => None,
                    PredicateOutcome::Decided { .. } => Some(Ok(axis)),
                    PredicateOutcome::Unknown { needed, stage } => Some(Err(BuildError::Geometry(
                        GeometryError::PredicateUnresolved { needed, stage },
                    ))),
                }
            })
            .transpose()?
            .ok_or(BuildError::EdgeUseSupportMismatch)?;
        for row in surface_points {
            if !points_equal(&(row[0].clone() + direction.clone()), &row[1])? {
                return Err(BuildError::EdgeUseSupportMismatch);
            }
        }

        if curve_points.len() != surface_points.len()
            || curve_weights.len() != surface_weights.len()
        {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let weight_scale =
            (&curve_weights[0] / &surface_weights[0][0]).map_err(GeometryError::Numeric)?;
        let mut coefficients = Vec::with_capacity(curve_points.len());
        for ((curve_point, curve_weight), (surface_row, surface_row_weights)) in curve_points
            .iter()
            .zip(curve_weights)
            .zip(surface_points.iter().zip(surface_weights))
        {
            if !real_values_equal(curve_weight, &(&surface_row_weights[0] * &weight_scale))? {
                return Err(BuildError::EdgeUseSupportMismatch);
            }
            let coefficient = ((point3_component(curve_point, direction_axis)
                - point3_component(&surface_row[0], direction_axis))
                / &direction.0[direction_axis])
                .map_err(GeometryError::Numeric)?;
            if !points_equal(
                &(surface_row[0].clone() + direction.clone() * &coefficient),
                curve_point,
            )? {
                return Err(BuildError::EdgeUseSupportMismatch);
            }
            coefficients.push(coefficient);
        }
        Ok(coefficients)
    }

    pub(in crate::topology) fn validate_tensor_surface_iso_image(
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
        let (axis, constant, lower, upper) = if v_constant {
            (
                SurfaceIsoAxis::U,
                line.start().y(),
                line.start().x(),
                line.end().x(),
            )
        } else {
            (
                SurfaceIsoAxis::V,
                line.start().x(),
                line.start().y(),
                line.end().y(),
            )
        };
        let complete = surface.iso_curve(axis, constant)?;
        if !complete.domain().contains_real(lower)? || !complete.domain().contains_real(upper)? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let (source_start, source_end) = match edge_use.direction {
            Direction::Forward => (lower, upper),
            Direction::Reversed => (upper, lower),
        };
        let segment = match decided_model_order(compare_reals(
            source_start,
            source_end,
            crate::STRICT_PREDICATES,
        ))? {
            std::cmp::Ordering::Less => complete.subcurve(source_start, source_end)?,
            std::cmp::Ordering::Greater => {
                complete.subcurve(source_end, source_start)?.reversed()?
            }
            std::cmp::Ordering::Equal => return Err(BuildError::EdgeUseSupportMismatch),
        };
        let edge_curve = curve.subcurve(
            (edge.domain.start())
                .try_real()
                .ok_or(crate::GeometryError::UnrepresentableParameter)?,
            (edge.domain.end())
                .try_real()
                .ok_or(crate::GeometryError::UnrepresentableParameter)?,
        )?;
        if !tensor_curve_images_equal(&edge_curve, &complete)?
            && !tensor_curve_images_equal(&edge_curve, &segment)?
        {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        Ok(())
    }
}
