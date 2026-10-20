//! Replay the product of a simple positive-radius profile and a full turn.
//!
//! Every source chart is checked from ordinary face/edge records. Neither
//! constructor identity nor an optional area/volume integral is evidence.

use crate::topology::*;
use std::cmp::Ordering;

fn insert_parameter(values: &mut Vec<Parameter>, value: &Parameter) -> Result<(), BuildError> {
    for (index, existing) in values.iter().enumerate() {
        match value.compare(existing)? {
            Ordering::Equal => return Ok(()),
            Ordering::Less => {
                values.insert(index, value.clone());
                return Ok(());
            }
            Ordering::Greater => {}
        }
    }
    values.push(value.clone());
    Ok(())
}

fn parameter_index(values: &[Parameter], value: &Parameter) -> Result<usize, BuildError> {
    for (index, existing) in values.iter().enumerate() {
        if value.compare(existing)? == Ordering::Equal {
            return Ok(index);
        }
    }
    // Every query below uses a value inserted into this same exact grid.
    Err(GeometryError::InvalidParameterDomain.into())
}

impl ArenaModelBuilder {
    /// A simple axis-aligned line boundary lies on all four sides of its
    /// envelope. Extra collinear vertices are allowed, but an interior detour
    /// cannot stand in for a rectangle. Wire simplicity/closure and orientation
    /// have already been checked by face admission.
    fn retained_revolution_rectangle(
        &self,
        face: FaceId,
    ) -> Result<Option<[Parameter; 4]>, BuildError> {
        let face = self.face_ref(face)?;
        let Some(outer) = face.outer() else {
            return Ok(None);
        };
        let wire = self.wire_ref(outer)?;
        let mut endpoints = Vec::with_capacity(wire.edge_uses.len());
        let mut u_values = Vec::new();
        let mut v_values = Vec::new();
        for usage in &wire.edge_uses {
            let curve = self.pcurve_ref(self.edge_use_ref(*usage)?.pcurve)?.curve();
            if curve.family() != CurveFamily2::Line {
                return Ok(None);
            }
            let coordinate = |point: hypercurve::CurvePoint2, x: bool| {
                let parameter = point
                    .affine_form(
                        Real::from(i32::from(x)),
                        Real::from(i32::from(!x)),
                        Real::zero(),
                    )
                    .parameter()
                    .map_err(GeometryError::from)?;
                Parameter::from_curve(&parameter)
            };
            let u0 = coordinate(curve.start(), true)?;
            let u1 = coordinate(curve.end(), true)?;
            let v0 = coordinate(curve.start(), false)?;
            let v1 = coordinate(curve.end(), false)?;
            let u_constant = u0.compare(&u1)? == Ordering::Equal;
            let v_constant = v0.compare(&v1)? == Ordering::Equal;
            if u_constant == v_constant {
                return Ok(None);
            }
            insert_parameter(&mut u_values, &u0)?;
            insert_parameter(&mut u_values, &u1)?;
            insert_parameter(&mut v_values, &v0)?;
            insert_parameter(&mut v_values, &v1)?;
            endpoints.push((u0, v0, u_constant));
        }
        if u_values.len() < 2 || v_values.len() < 2 {
            return Ok(None);
        }
        let bounds = [
            u_values[0].clone(),
            u_values.last().unwrap().clone(),
            v_values[0].clone(),
            v_values.last().unwrap().clone(),
        ];
        for (u, v, u_constant) in endpoints {
            let (value, lower, upper) = if u_constant {
                (&u, &bounds[0], &bounds[1])
            } else {
                (&v, &bounds[2], &bounds[3])
            };
            if value.compare(lower)? != Ordering::Equal && value.compare(upper)? != Ordering::Equal
            {
                return Ok(None);
            }
        }
        Ok(Some(bounds))
    }

    pub(in crate::topology) fn certified_retained_revolution_shell(
        &self,
        shell: ShellId,
        orientation: Orientation,
    ) -> Result<Option<CertifiedRevolutionShell>, BuildError> {
        let mut groups = std::collections::BTreeMap::<SurfaceId, Vec<FaceId>>::new();
        for id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*id)?;
            if face.orientation != orientation
                || !face.inner().is_empty()
                || self.surface_ref(face.surface)?.kind() != SurfaceKind::Revolution
            {
                return Ok(None);
            }
            groups.entry(face.surface).or_default().push(*id);
        }
        let mut common_origin = None;
        let mut common_axis = None;
        let mut common_ray: Option<Vector3> = None;
        let mut angular_origin: Option<Real> = None;
        let mut profiles = Vec::with_capacity(groups.len());
        for (surface, faces) in groups {
            let SurfaceExactData::Revolution {
                profile,
                axis_origin,
                axis,
            } = self.surface_ref(surface)?.exact_data()
            else {
                return Ok(None);
            };
            let Curve3ExactData::PlanarImage(image) = *profile else {
                return Ok(None);
            };
            if faces.len() < 4 {
                return Ok(None);
            }
            if let Some(expected) = &common_origin
                && !points_equal(expected, &axis_origin)?
            {
                return Ok(None);
            }
            if let Some(expected) = &common_axis
                && !vectors_equal(expected, &axis)?
            {
                return Ok(None);
            }
            common_origin.get_or_insert_with(|| axis_origin.clone());
            common_axis.get_or_insert_with(|| axis.clone());
            let spatial = Curve3::from_planar_image(*image.clone())?;
            if common_ray.is_none() {
                let mut radial = &image.u - axis.clone() * axis.dot(&image.u);
                if real_values_equal(&radial.norm_squared(), &Real::zero())? {
                    radial = &image.v - axis.clone() * axis.dot(&image.v);
                }
                let length = radial
                    .norm_squared()
                    .sqrt()
                    .map_err(GeometryError::Numeric)?;
                if real_values_equal(&length, &Real::zero())? {
                    return Ok(None);
                }
                let mut ray = (radial / length).map_err(GeometryError::Numeric)?;
                let start = spatial.point_at(spatial.domain().start())?;
                match start
                    .compare_linear_form(&ray, &ray.dot(&(&axis_origin - Point3::origin())))?
                {
                    Ordering::Less => ray = -ray,
                    Ordering::Equal => return Ok(None),
                    Ordering::Greater => {}
                }
                common_ray = Some(ray);
            }
            let ray = common_ray
                .as_ref()
                .expect("nonzero radial frame establishes ray");
            let normal = ray.cross(&axis);
            for direction in [
                &image.origin - &axis_origin,
                image.u.clone(),
                image.v.clone(),
            ] {
                if !real_values_equal(&normal.dot(&direction), &Real::zero())? {
                    return Ok(None);
                }
            }
            let mut u_values = Vec::new();
            let mut v_values = Vec::new();
            let mut rectangles = Vec::with_capacity(faces.len());
            for face in faces {
                let Some(rectangle) = self.retained_revolution_rectangle(face)? else {
                    return Ok(None);
                };
                for u in &rectangle[..2] {
                    insert_parameter(&mut u_values, u)?;
                }
                for v in &rectangle[2..] {
                    insert_parameter(&mut v_values, v)?;
                }
                rectangles.push(rectangle);
            }
            if u_values.len() < 2 || v_values.len() < 2 {
                return Ok(None);
            }
            let (Some(u_min), Some(u_max)) =
                (u_values[0].try_real(), u_values.last().unwrap().try_real())
            else {
                return Ok(None);
            };
            if !real_values_equal(&(u_max - u_min), &Real::tau())?
                || v_values[0].compare(spatial.domain().start())? != Ordering::Equal
                || v_values.last().unwrap().compare(spatial.domain().end())? != Ordering::Equal
            {
                return Ok(None);
            }
            if let Some(expected) = &angular_origin
                && !real_values_equal(expected, u_min)?
            {
                return Ok(None);
            }
            angular_origin.get_or_insert_with(|| u_min.clone());
            let rectangles = rectangles
                .iter()
                .map(|[u0, u1, v0, v1]| {
                    Ok([
                        parameter_index(&u_values, u0)?,
                        parameter_index(&u_values, u1)?,
                        parameter_index(&v_values, v0)?,
                        parameter_index(&v_values, v1)?,
                    ])
                })
                .collect::<Result<Vec<_>, BuildError>>()?;
            if !super::rectangular_cover::exactly_covers(
                u_values.len() - 1,
                v_values.len() - 1,
                &rectangles,
            )? {
                return Ok(None);
            }
            let projected = image.materialize_plane_projection(&axis_origin, ray, &axis)?;
            let curve = projected.curve();
            // The source traversal is not an orientation certificate for an
            // independently authored spatial parameter map. Follow increasing
            // spatial v, even if a cold archive stores the source backwards.
            let source_increases = curve
                .parameter_endpoints()
                .0
                .compare(curve.parameter_endpoints().1)
                .map_err(GeometryError::from)?
                == Ordering::Less;
            let map_increases = decided_model_order(compare_reals(
                &image.source_scale,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? == Ordering::Greater;
            profiles.push(if source_increases == map_increases {
                curve.clone()
            } else {
                curve.reversed().map_err(GeometryError::from)?
            });
        }
        if profiles.is_empty() {
            return Ok(None);
        }
        let mut ordered = Vec::with_capacity(profiles.len());
        ordered.push(profiles.remove(0));
        while !profiles.is_empty() {
            let end = ordered.last().unwrap().end();
            let mut next = None;
            for (i, curve) in profiles.iter().enumerate() {
                if end
                    .coincides_with(&curve.start())
                    .map_err(GeometryError::from)?
                    && next.replace(i).is_some()
                {
                    return Ok(None);
                }
            }
            let Some(index) = next else { return Ok(None) };
            ordered.push(profiles.remove(index));
        }
        if !ordered
            .last()
            .unwrap()
            .end()
            .coincides_with(&ordered[0].start())
            .map_err(GeometryError::from)?
        {
            return Ok(None);
        }
        let path = CurvePath2::try_new(ordered).map_err(GeometryError::from)?;
        if path
            .simple_loop_orientation()
            .map_err(GeometryError::from)?
            != hypercurve::CurveLoopOrientation2::Counterclockwise
        {
            return Ok(None);
        }
        if decided_model_order(compare_reals(
            path.bounds().map_err(GeometryError::from)?.min_x(),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != Ordering::Greater
        {
            return Ok(None);
        }
        Ok(Some(CertifiedRevolutionShell {
            axis_origin: common_origin.expect("nonempty revolution groups"),
            axis: common_axis.expect("nonempty revolution groups"),
            profile: CertifiedRevolutionBoundary::Curved(path),
            voids: Vec::new(),
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn retained_shell_cover_rejects_a_gap_hidden_by_a_duplicate_face() {
        let p = |x, y| CurvePoint2::from_values(x, y);
        let parallel = QuadraticBezier2::new(p(2, 0), p(3, 0), p(4, 0))
            .parallel_left(Real::one())
            .unwrap();
        let bottom = Curve2::try_analytic_parallel(
            parallel,
            hypercurve::BezierParameterRange2::try_new(
                hypercurve::BezierParameter2::Exact(Real::zero()),
                hypercurve::BezierParameter2::Exact(Real::one()),
            )
            .unwrap(),
        )
        .unwrap();
        let path = CurvePath2::try_new(vec![
            bottom,
            Curve2::from(LineSeg2::try_new(p(4, 1), p(4, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(4, 2), p(2, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(2, 2), p(2, 1)).unwrap()),
        ])
        .unwrap();
        let (model, _) = crate::construction::revolve_path(&path).unwrap();
        let mut builder = ArenaModelBuilder::new();
        let solids = model.append_to_builder(&mut builder).unwrap();
        let shell = builder.solids[solids[0].index()].outer;
        assert!(
            builder
                .certified_retained_revolution_shell(shell, Orientation::Forward)
                .unwrap()
                .is_some()
        );
        // Retain the number of faces and each face's already-admitted image.
        // Counting faces or total interval extent alone would miss this hole.
        let faces = &mut builder.shells[shell.index()].faces;
        assert_eq!(faces.len(), 16);
        faces[1] = faces[0];
        assert!(
            builder
                .certified_retained_revolution_shell(shell, Orientation::Forward)
                .unwrap()
                .is_none()
        );
    }
}
