//! Validated construction: validation revolution.

use crate::topology::*;

impl ArenaModelBuilder {
    /// Coordinate-chart revolution faces require complete axis clearance on
    /// the retained profile, including during untrusted archive reconstruction.
    /// An image certificate for each boundary alone does not establish this.
    pub(in crate::topology) fn validate_retained_revolution_chart(
        &self,
        wires: &[WireId],
        surface: SurfaceId,
    ) -> Result<(), BuildError> {
        let surface = self.surface_ref(surface)?;
        if surface.kind() != SurfaceKind::Revolution {
            return Ok(());
        }
        for wire in wires {
            for edge_use in &self.wire_ref(*wire)?.edge_uses {
                if matches!(
                    self.edge_use_ref(*edge_use)?.parameter_correspondence,
                    ParameterCorrespondence::Coordinate { .. }
                ) {
                    return if surface.revolution_profile_is_strictly_off_axis()? {
                        Ok(())
                    } else {
                        Err(GeometryError::SingularSurfaceParameter.into())
                    };
                }
            }
        }
        Ok(())
    }

    pub(in crate::topology) fn certified_revolution_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedRevolutionShell>, BuildError> {
        self.certified_oriented_revolution_shell(shell, Orientation::Forward)
    }

    pub(in crate::topology) fn certified_revolution_solid(
        &self,
        solid: &Solid,
    ) -> Result<Option<CertifiedRevolutionShell>, BuildError> {
        let Some(mut certificate) =
            self.certified_oriented_revolution_shell(solid.outer, Orientation::Forward)?
        else {
            return Ok(None);
        };
        for shell in &solid.voids {
            let Some(void) =
                self.certified_oriented_revolution_shell(*shell, Orientation::Reversed)?
            else {
                return Ok(None);
            };
            if !points_equal(&certificate.axis_origin, &void.axis_origin)?
                || !vectors_equal(&certificate.axis, &void.axis)?
            {
                return Ok(None);
            }
            certificate.voids.push(void.profile);
        }
        Ok(Some(certificate))
    }

    pub(in crate::topology) fn certified_oriented_revolution_shell(
        &self,
        shell: ShellId,
        orientation: Orientation,
    ) -> Result<Option<CertifiedRevolutionShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        if faces.len() < 8 {
            return Ok(None);
        }
        // The first source seeds the retained meridian cycle and its frame.
        // Preserve arena order across independent admissions so downstream
        // Booleans receive the same source charts, not a random cycle rotation.
        let mut groups = std::collections::BTreeMap::<SurfaceId, Vec<FaceId>>::new();
        let mut planar_faces = 0;
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::Revolution
                    if face.orientation == orientation && face.inner().is_empty() =>
                {
                    groups.entry(face.surface).or_default().push(*face_id);
                }
                SurfaceKind::Plane => planar_faces += 1,
                _ => return Ok(None),
            }
        }
        if groups.is_empty() || groups.values().any(|group| group.len() < 4) {
            return Ok(None);
        }

        let quarter = (Real::pi() / Real::from(2)).map_err(GeometryError::Numeric)?;
        let mut axis_origin = None;
        let mut axis = None;
        let mut meridian_ray: Option<Vector3> = None;
        let mut angular_origin: Option<Real> = None;
        let mut profile_curves = Vec::with_capacity(groups.len() + planar_faces);
        for (surface_id, group) in groups {
            let SurfaceExactData::Revolution {
                profile,
                axis_origin: this_origin,
                axis: this_axis,
            } = self.surface_ref(surface_id)?.exact_data()
            else {
                unreachable!("revolution kind carries revolution data");
            };
            if let Some(expected) = &axis_origin
                && !points_equal(expected, &this_origin)?
            {
                return Ok(None);
            }
            if let Some(expected) = &axis
                && !vectors_equal(expected, &this_axis)?
            {
                return Ok(None);
            }
            axis_origin.get_or_insert_with(|| this_origin.clone());
            axis.get_or_insert_with(|| this_axis.clone());
            let profile_curve = Curve3::from_exact_data(*profile)?;
            if profile_curve.kind() == Curve3Kind::PlanarImage {
                return self.certified_retained_revolution_shell(shell, orientation);
            }
            let mut to_profile_point = |point: &Point3,
                                        require_positive: bool|
             -> Result<Option<CurvePoint2>, BuildError> {
                let relative = point - &this_origin;
                let axial = this_axis.dot(&relative);
                let radial = relative - this_axis.clone() * &axial;
                let radial_squared = radial.norm_squared();
                let radial_order = decided_model_order(compare_reals(
                    &radial_squared,
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))?;
                if let Some(expected) = &meridian_ray {
                    if decided_model_order(compare_reals(
                        &expected.cross(&radial).norm_squared(),
                        &Real::zero(),
                        crate::STRICT_PREDICATES,
                    ))? != std::cmp::Ordering::Equal
                    {
                        return Ok(None);
                    }
                    let radius = expected.dot(&radial);
                    if require_positive
                        && decided_model_order(compare_reals(
                            &radius,
                            &Real::zero(),
                            crate::STRICT_PREDICATES,
                        ))? != std::cmp::Ordering::Greater
                    {
                        return Ok(None);
                    }
                    return Ok(Some(CurvePoint2::new(radius, axial)));
                }
                if radial_order != std::cmp::Ordering::Greater {
                    return if require_positive {
                        Ok(None)
                    } else {
                        Ok(Some(CurvePoint2::new(Real::zero(), axial)))
                    };
                }
                let radius = radial_squared.sqrt().map_err(GeometryError::Numeric)?;
                if require_positive {
                    meridian_ray = Some((radial / &radius).map_err(GeometryError::Numeric)?);
                } else {
                    return Ok(None);
                }
                Ok(Some(CurvePoint2::new(radius, axial)))
            };
            let mut u_values = Vec::new();
            let mut v_values = Vec::new();
            let mut face_coordinates = Vec::with_capacity(group.len());
            for face_id in group {
                let outer = self
                    .face_ref(face_id)?
                    .outer()
                    .expect("trimmed revolution face");
                let wire = self.wire_ref(outer)?;
                if wire.edge_uses.len() < 4 {
                    return Ok(None);
                }
                let mut face_u = Vec::with_capacity(wire.edge_uses.len() * 2);
                let mut face_v = Vec::with_capacity(wire.edge_uses.len() * 2);
                let mut parameter_segments = Vec::with_capacity(wire.edge_uses.len());
                for use_id in &self.wire_ref(outer)?.edge_uses {
                    let pcurve = self.pcurve_ref(self.edge_use_ref(*use_id)?.pcurve)?;
                    let Some(line) = pcurve.line_segment() else {
                        return Ok(None);
                    };
                    let u_constant = real_values_equal(line.start().x(), line.end().x())?;
                    let v_constant = real_values_equal(line.start().y(), line.end().y())?;
                    if u_constant == v_constant {
                        return Ok(None);
                    }
                    face_u.extend([line.start().x().clone(), line.end().x().clone()]);
                    face_v.extend([line.start().y().clone(), line.end().y().clone()]);
                    parameter_segments.push(Segment2::Line(line.clone()));
                }
                let (u_min, u_max) = exact_real_min_max(&face_u)?;
                let (v_min, v_max) = exact_real_min_max(&face_v)?;
                let contour = Contour2::try_new(parameter_segments).map_err(GeometryError::from)?;
                let represented_area = contour
                    .signed_area()
                    .map_err(GeometryError::from)?
                    .ok_or(BuildError::DegenerateShellVolume(shell))?
                    .abs();
                let rectangle_area = (&u_max - &u_min) * (&v_max - &v_min);
                if !real_values_equal(&represented_area, &rectangle_area)? {
                    return Ok(None);
                }
                insert_sorted_real(&mut u_values, &u_min)?;
                insert_sorted_real(&mut u_values, &u_max)?;
                insert_sorted_real(&mut v_values, &v_min)?;
                insert_sorted_real(&mut v_values, &v_max)?;
                face_coordinates.push((u_min, u_max, v_min, v_max));
            }
            if u_values.len() != 5 || v_values.len() < 2 {
                return Ok(None);
            }
            for pair in u_values.windows(2) {
                if !real_values_equal(&(&pair[1] - &pair[0]), &quarter)? {
                    return Ok(None);
                }
            }
            let mut covered_cells = HashSet::new();
            for (u_min, u_max, v_min, v_max) in face_coordinates {
                let u_start = exact_real_index(&u_values, &u_min)?;
                let u_end = exact_real_index(&u_values, &u_max)?;
                let v_start = exact_real_index(&v_values, &v_min)?;
                let v_end = exact_real_index(&v_values, &v_max)?;
                if u_end != u_start + 1 || v_start >= v_end {
                    return Ok(None);
                }
                for v_cell in v_start..v_end {
                    if !covered_cells.insert((u_start, v_cell)) {
                        return Ok(None);
                    }
                }
            }
            if covered_cells.len() != (u_values.len() - 1) * (v_values.len() - 1) {
                return Ok(None);
            }
            if !real_values_equal(
                &(u_values[3].clone() + &quarter - &u_values[0]),
                &(Real::from(2) * Real::pi()),
            )? {
                return Ok(None);
            }
            if let Some(expected) = &angular_origin {
                if !real_values_equal(expected, &u_values[0])? {
                    return Ok(None);
                }
            } else {
                angular_origin = Some(u_values[0].clone());
            }

            let represented_profile = profile_curve.subcurve(
                &v_values[0],
                v_values.last().expect("revolution group has v values"),
            )?;
            let start_point = represented_profile
                .evaluate_real(represented_profile.domain().represented_start()?)?;
            let end_point = represented_profile
                .evaluate_real(represented_profile.domain().represented_end()?)?;
            let Some(start) = to_profile_point(&start_point, true)? else {
                return Ok(None);
            };
            let Some(end) = to_profile_point(&end_point, true)? else {
                return Ok(None);
            };
            let profile_curve = match represented_profile.exact_data() {
                Curve3ExactData::Line(_) => {
                    Curve2::from(LineSeg2::try_new(start, end).map_err(GeometryError::from)?)
                }
                Curve3ExactData::RationalBezier {
                    control_points,
                    weights,
                } => {
                    let controls = control_points
                        .iter()
                        .map(|point| to_profile_point(point, false))
                        .collect::<Result<Option<Vec<_>>, _>>()?;
                    let Some(controls) = controls else {
                        return Ok(None);
                    };
                    planar_rational_bezier_curve(controls, weights.clone())?
                }
                Curve3ExactData::Nurbs {
                    degree,
                    control_points,
                    weights,
                    knots,
                } => {
                    let controls = control_points
                        .iter()
                        .map(|point| to_profile_point(point, false))
                        .collect::<Result<Option<Vec<_>>, _>>()?;
                    let Some(controls) = controls else {
                        return Ok(None);
                    };
                    if weights.iter().all(|weight| weight == &weights[0]) {
                        Curve2::try_polynomial_bspline(degree, controls, knots.clone())
                            .map_err(GeometryError::from)?
                    } else {
                        Curve2::try_nurbs(degree, controls, weights.clone(), knots.clone())
                            .map_err(GeometryError::from)?
                    }
                }
                Curve3ExactData::EllipseArc(data) if data.circle => {
                    let Some(center) = to_profile_point(&data.center, false)? else {
                        return Ok(None);
                    };
                    let ray = meridian_ray
                        .as_ref()
                        .expect("positive profile endpoint establishes a meridian ray");
                    let meridian_normal = this_axis.cross(ray);
                    if decided_model_order(compare_reals(
                        &data.x.dot(&meridian_normal),
                        &Real::zero(),
                        crate::STRICT_PREDICATES,
                    ))? != std::cmp::Ordering::Equal
                        || decided_model_order(compare_reals(
                            &data.y.dot(&meridian_normal),
                            &Real::zero(),
                            crate::STRICT_PREDICATES,
                        ))? != std::cmp::Ordering::Equal
                    {
                        return Ok(None);
                    }
                    let tangent = represented_profile
                        .derivative_at(
                            &represented_profile
                                .domain()
                                .represented_start()?
                                .clone()
                                .into(),
                            1,
                        )?
                        .require_coordinates()?
                        .clone();
                    let tangent_radial = ray.dot(&tangent);
                    let tangent_axial = this_axis.dot(&tangent);
                    let turn = (start.x() - center.x()) * &tangent_axial
                        - (start.y() - center.y()) * &tangent_radial;
                    let clockwise = match decided_model_order(compare_reals(
                        &turn,
                        &Real::zero(),
                        crate::STRICT_PREDICATES,
                    ))? {
                        std::cmp::Ordering::Less => true,
                        std::cmp::Ordering::Greater => false,
                        std::cmp::Ordering::Equal => return Ok(None),
                    };
                    let arc = CircularArc2::try_from_center(start, end, center, clockwise)
                        .map_err(GeometryError::from)?;
                    let sweep = match crate::error::classified(arc.directed_sweep_angle())
                        .map_err(GeometryError::from)?
                    {
                        Classification::Decided(sweep) => sweep,
                        Classification::Uncertain(reason) => {
                            return Err(BuildError::Geometry(
                                GeometryError::PlanarClassificationUnresolved(reason),
                            ));
                        }
                    };
                    if !real_values_equal(
                        sweep,
                        &(represented_profile.domain().represented_end()?
                            - represented_profile.domain().represented_start()?),
                    )? {
                        return Ok(None);
                    }
                    Curve2::from(arc)
                }
                _ => return Ok(None),
            };
            profile_curves.push(profile_curve);
        }
        if planar_faces > 0 {
            if orientation != Orientation::Forward {
                return Ok(None);
            }
            let Some(planar_segments) = self.certified_revolution_planar_profile_segments(
                faces,
                axis_origin
                    .as_ref()
                    .expect("revolution side faces establish an axis origin"),
                axis.as_ref()
                    .expect("revolution side faces establish an axis"),
            )?
            else {
                return Ok(None);
            };
            profile_curves.extend(planar_segments.into_iter().map(curve2_from_segment));
        }

        let mut ordered = Vec::with_capacity(profile_curves.len());
        ordered.push(profile_curves.remove(0));
        while !profile_curves.is_empty() {
            let end = ordered.last().expect("seeded profile").end();
            let mut matching = None;
            for (index, curve) in profile_curves.iter().enumerate() {
                if curve_points_equal(
                    (curve.start())
                        .coordinates()
                        .ok_or(GeometryError::UnsupportedPcurveContour)?,
                    (end)
                        .coordinates()
                        .ok_or(GeometryError::UnsupportedPcurveContour)?,
                )? && matching.replace(index).is_some()
                {
                    return Ok(None);
                }
            }
            let Some(index) = matching else {
                return Ok(None);
            };
            ordered.push(profile_curves.remove(index));
        }
        if !curve_points_equal(
            (ordered.last().expect("nonempty profile").end())
                .coordinates()
                .ok_or(GeometryError::UnsupportedPcurveContour)?,
            (ordered[0].start())
                .coordinates()
                .ok_or(GeometryError::UnsupportedPcurveContour)?,
        )? {
            return Ok(None);
        }
        let profile = if ordered.iter().all(|curve| {
            matches!(
                curve.geometry(),
                Some(CurveGeometry2::Line(_)) | Some(CurveGeometry2::CircularArc(_))
            )
        }) {
            let segments = ordered
                .into_iter()
                .map(|curve| match curve.geometry() {
                    Some(CurveGeometry2::Line(line)) => Segment2::Line(line.clone()),
                    Some(CurveGeometry2::CircularArc(arc)) => Segment2::Arc(arc.clone()),
                    _ => unreachable!("native revolution profile was prefiltered"),
                })
                .collect();
            let contour = Contour2::try_new(segments).map_err(GeometryError::from)?;
            let revolution_profile_area = contour.signed_area().map_err(GeometryError::from)?;
            if !contour
                .intersect_self()
                .map_err(GeometryError::from)?
                .is_empty()
                || decided_model_order(compare_reals(
                    &revolution_profile_area.ok_or(BuildError::DegenerateShellVolume(shell))?,
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? != std::cmp::Ordering::Greater
            {
                return Ok(None);
            }
            CertifiedRevolutionBoundary::Native(contour)
        } else {
            let path = CurvePath2::try_new(ordered).map_err(GeometryError::from)?;
            let area = crate::error::classified(
                path.boundary_loop()
                    .map_err(GeometryError::from)?
                    .signed_area(),
            )
            .map_err(GeometryError::from)?;
            let area = resolve_planar_classification(area)?
                .ok_or(BuildError::DegenerateShellVolume(shell))?;
            if decided_model_order(compare_reals(
                &area,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Greater
            {
                return Ok(None);
            }
            CertifiedRevolutionBoundary::Curved(path)
        };
        Ok(Some(CertifiedRevolutionShell {
            axis_origin: axis_origin.expect("revolution has faces"),
            axis: axis.expect("revolution has faces"),
            profile,
            voids: Vec::new(),
        }))
    }

    pub(in crate::topology) fn certified_revolution_planar_profile_segments(
        &self,
        faces: &[FaceId],
        axis_origin: &Point3,
        axis: &Vector3,
    ) -> Result<Option<Vec<Segment2>>, BuildError> {
        let groups = self.planar_face_groups(faces)?;
        if groups.is_empty() {
            return Ok(Some(Vec::new()));
        }
        let mut segments = Vec::with_capacity(groups.len());
        for group in groups {
            let Some(boundaries) = self.cap_boundary_use_loops(faces, &group)? else {
                return Ok(None);
            };
            if boundaries.len() != 2 {
                return Ok(None);
            }
            let mut center = None;
            let mut normal_direction = None;
            for index in &group {
                let face = self.face_ref(faces[*index])?;
                let surface = self.surface_ref(face.surface)?;
                let Some(origin) = surface.plane_origin() else {
                    return Ok(None);
                };
                let Some((u, v)) = surface.plane_directions() else {
                    return Ok(None);
                };
                let normal = match face.orientation {
                    Orientation::Forward => u.cross(v),
                    Orientation::Reversed => -u.cross(v),
                };
                if decided_model_order(compare_reals(
                    &normal.cross(axis).norm_squared(),
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? != std::cmp::Ordering::Equal
                {
                    return Ok(None);
                }
                let direction = decided_model_order(compare_reals(
                    &normal.dot(axis),
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))?;
                if !matches!(
                    direction,
                    std::cmp::Ordering::Less | std::cmp::Ordering::Greater
                ) || normal_direction.is_some_and(|expected| expected != direction)
                {
                    return Ok(None);
                }
                normal_direction = Some(direction);
                let axial = axis.dot(&(origin - axis_origin));
                let candidate = axis_origin.clone() + axis.clone() * axial;
                if let Some(expected) = &center {
                    if !points_equal(expected, &candidate)? {
                        return Ok(None);
                    }
                } else {
                    center = Some(candidate);
                }
            }
            let center = center.expect("nonempty planar face group");
            let mut radii = Vec::with_capacity(2);
            for boundary in boundaries {
                let mut radius = None;
                let mut sweep = Real::zero();
                for edge_use_id in boundary {
                    let edge_use = self.edge_use_ref(edge_use_id)?;
                    let edge = self.edge_ref(edge_use.edge)?;
                    let curve = self.curve_ref(edge.curve)?;
                    let Curve3ExactData::EllipseArc(data) = curve.exact_data() else {
                        return Ok(None);
                    };
                    if !data.circle || !points_equal(&data.center, &center)? {
                        return Ok(None);
                    }
                    if let Some(expected) = &radius {
                        if !real_values_equal(expected, &data.x_radius)?
                            || !real_values_equal(expected, &data.y_radius)?
                        {
                            return Ok(None);
                        }
                    } else if !real_values_equal(&data.x_radius, &data.y_radius)? {
                        return Ok(None);
                    } else {
                        radius = Some(data.x_radius);
                    }
                    sweep += edge.domain.represented_end()? - edge.domain.represented_start()?;
                }
                if !real_values_equal(&sweep, &Real::tau())? {
                    return Ok(None);
                }
                radii.push(radius.expect("nonempty cap boundary"));
            }
            if decided_model_order(compare_reals(
                &radii[0],
                &radii[1],
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Equal
            {
                return Ok(None);
            }
            let (inner, outer) = if decided_model_order(compare_reals(
                &radii[0],
                &radii[1],
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Less
            {
                (radii.remove(0), radii.remove(0))
            } else {
                (radii.remove(1), radii.remove(0))
            };
            let axial = axis.dot(&(&center - axis_origin));
            let (start, end) = if normal_direction.expect("cap group has an effective normal")
                == std::cmp::Ordering::Greater
            {
                (
                    CurvePoint2::new(outer, axial.clone()),
                    CurvePoint2::new(inner, axial),
                )
            } else {
                (
                    CurvePoint2::new(inner, axial.clone()),
                    CurvePoint2::new(outer, axial),
                )
            };
            segments.push(Segment2::Line(
                LineSeg2::try_new(start, end).map_err(GeometryError::from)?,
            ));
        }
        Ok(Some(segments))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cold_revolution_admission_preserves_profile_source_order() {
        let p = |x, y| CurvePoint2::from_values(x, y);
        let polygon = crate::construction::revolve(
            &[(4, -3), (6, -3), (6, 3), (4, 3)]
                .map(|(x, y)| crate::Point2::new(x.into(), y.into())),
        )
        .unwrap();
        let spline = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p(2, 0), p(4, 0)).unwrap()),
            Curve2::try_nurbs(
                2,
                vec![p(4, 0), p(5, 1), p(4, 2)],
                vec![Real::one(); 3],
                vec![0.into(), 0.into(), 0.into(), 1.into(), 1.into(), 1.into()],
            )
            .unwrap(),
            Curve2::from(LineSeg2::try_new(p(4, 2), p(2, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(2, 2), p(2, 0)).unwrap()),
        ])
        .unwrap();
        let spline = crate::construction::revolve_path(&spline).unwrap();
        for (model, solid) in [polygon, spline] {
            let source = model.to_json().unwrap();
            let expected = model
                .certified_revolution_profile(solid)
                .unwrap()
                .profile
                .as_curve_path()
                .unwrap();
            // A fresh admission gets independent map seeds. A later Boolean
            // must see the same retained source sequence on every admission.
            for _ in 0..32 {
                let cold = crate::persistence::ImportedModel::from_json(
                    &source,
                    &crate::ArchiveLimits::default(),
                )
                .unwrap()
                .validate()
                .unwrap();
                let profile = cold
                    .certified_revolution_profile(solid)
                    .unwrap()
                    .profile
                    .as_curve_path()
                    .unwrap();
                assert_eq!(profile.curves(), expected.curves());
                assert_eq!(cold.to_json().unwrap(), source);
            }
        }
    }
}
