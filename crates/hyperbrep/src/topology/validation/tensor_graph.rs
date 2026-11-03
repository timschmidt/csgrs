//! Validated construction: solids capped by a rational Bézier tensor graph.

use crate::SurfaceSurfaceIntersection;
use crate::topology::*;

impl ArenaModelBuilder {
    /// A closed shell of rational Bézier top faces on one tensor, one planar
    /// base face without holes, planar walls parallel to the base's inward
    /// normal `d`, and optional planar caps parallel to the base. The tensor
    /// must be certified as a graph strictly above the base along `d`.
    ///
    /// With one tensor face and no caps, the walls being parallel to `d` and
    /// the shell being closed and oriented make the top's projection exactly
    /// the base region. Otherwise every top face must also face along `d`
    /// (caps by their constant normal, tensor faces by the sign of the
    /// certified graph's normal, constant because its projection is
    /// injective), and each cap must lie strictly above the base: a vertical
    /// line over the profile then leaves the solid through exactly one top
    /// face, so the tops' regions (caps minus their holes) tile the profile.
    pub(in crate::topology) fn certified_tensor_graph_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedTensorGraphShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut tensors = Vec::new();
        let mut planes = Vec::new();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::RationalBezier => tensors.push(*face_id),
                SurfaceKind::Plane => planes.push(*face_id),
                _ => return Ok(None),
            }
        }
        let Some(first) = tensors.first() else {
            return Ok(None);
        };
        let top_surface = self.surface_ref(self.face_ref(*first)?.surface)?.clone();
        for face_id in &tensors[1..] {
            let surface = self.surface_ref(self.face_ref(*face_id)?.surface)?;
            if !top_surface.same_rational_tensor(surface)? {
                return Ok(None);
            }
        }
        let zero = Real::zero();
        let order = |value: &Real| {
            decided_model_order(compare_reals(value, &zero, crate::STRICT_PREDICATES))
        };
        for base_id in &planes {
            let base = self.face_ref(*base_id)?;
            let Some(outer) = base.outer() else {
                continue;
            };

            let SurfaceExactData::Plane { origin, u, v } =
                self.surface_ref(base.surface)?.exact_data()
            else {
                continue;
            };
            // The base's outward normal points away from the material.
            let normal = u.cross(&v);
            let direction = match base.orientation {
                Orientation::Forward => -normal.clone(),
                Orientation::Reversed => normal.clone(),
            };
            let determinant = u.dot(&v.cross(&direction));
            let mut admissible = true;
            let mut caps = Vec::new();
            for other_id in &planes {
                if other_id == base_id {
                    continue;
                }
                let other = self.face_ref(*other_id)?;
                let SurfaceExactData::Plane {
                    origin: cap_origin,
                    u: cap_u,
                    v: cap_v,
                } = self.surface_ref(other.surface)?.exact_data()
                else {
                    admissible = false;
                    break;
                };
                let other_normal = cap_u.cross(&cap_v);
                if order(&other_normal.dot(&direction))? == std::cmp::Ordering::Equal {
                    // A wall parallel to `d`.
                    continue;
                }
                // Otherwise a cap: parallel to the base, facing along `d`,
                // strictly above the base and without holes.
                let parallel = other_normal.cross(&normal);
                let outward = match other.orientation {
                    Orientation::Forward => other_normal,
                    Orientation::Reversed => -other_normal,
                };
                let height = ((&cap_origin - &origin).dot(&u.cross(&v)) / &determinant)
                    .map_err(GeometryError::Numeric)?;
                let Some(cap_outer) = other.outer() else {
                    admissible = false;
                    break;
                };
                if order(&parallel.dot(&parallel))? != std::cmp::Ordering::Equal
                    || order(&outward.dot(&direction))? != std::cmp::Ordering::Greater
                    || order(&height)? != std::cmp::Ordering::Greater
                {
                    admissible = false;
                    break;
                }
                caps.push(CertifiedGraphCap {
                    origin: cap_origin,
                    u: cap_u,
                    v: cap_v,
                    height,
                    boundary: self.build_wire_curve_path(cap_outer)?,
                    holes: other
                        .inner()
                        .iter()
                        .map(|wire| self.build_wire_curve_path(*wire))
                        .collect::<Result<Vec<_>, _>>()?,
                });
            }
            if !admissible {
                continue;
            }
            let base_surface = self.surface_ref(base.surface)?.clone();
            let profile = self.build_wire_curve_path(outer)?;
            let profile_holes = base
                .inner()
                .iter()
                .map(|wire| self.build_wire_curve_path(*wire))
                .collect::<Result<Vec<_>, _>>()?;
            if !top_surface.certify_tensor_graph(&origin, &u, &v, &direction)?
                && !(caps.is_empty()
                    && top_above_base_on_profile(
                        &base_surface,
                        &top_surface,
                        (&origin, &u, &v, &direction),
                        &profile,
                        &profile_holes,
                    )?)
            {
                continue;
            }
            if (tensors.len() > 1 || !caps.is_empty())
                && !self.tensor_faces_face_along(&tensors, &top_surface, &direction)?
            {
                continue;
            }
            return Ok(Some(CertifiedTensorGraphShell {
                origin,
                u,
                v,
                direction,
                profile,
                profile_holes,
                top: top_surface,
                caps,
            }));
        }
        Ok(None)
    }

    /// Whether every tensor face's outward normal points along `direction`.
    /// The certified graph's projection is injective, so its normal has one
    /// sign against `direction` across the patch; the patch centre decides it.
    fn tensor_faces_face_along(
        &self,
        tensors: &[FaceId],
        surface: &Surface,
        direction: &Vector3,
    ) -> Result<bool, BuildError> {
        let half = (Real::one() / Real::from(2)).map_err(GeometryError::Numeric)?;
        let normal = match surface.normal_at(&CurvePoint2::new(half.clone(), half).into()) {
            Ok(normal) => normal,
            Err(GeometryError::SingularSurfaceParameter) => return Ok(false),
            Err(error) => return Err(error.into()),
        };
        let sign = normal.compare_linear_form(direction, &Real::zero())?;
        if sign == std::cmp::Ordering::Equal {
            return Ok(false);
        }
        for face_id in tensors {
            let along = match self.face_ref(*face_id)?.orientation {
                Orientation::Forward => sign,
                Orientation::Reversed => sign.reverse(),
            };
            if along != std::cmp::Ordering::Greater {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

/// Whether a tensor with a certified injective base projection lies on or
/// above the base plane over the whole base profile, without requiring it
/// above the plane over the entire patch. Its height above the plane is zero
/// exactly on the complete plane/tensor section; when every section curve
/// lies along the profile boundary, the height keeps one sign inside the
/// connected profile, which an interior point then fixes as positive.
fn top_above_base_on_profile(
    base: &Surface,
    top: &Surface,
    frame: (&Point3, &Vector3, &Vector3, &Vector3),
    profile: &CurvePath2,
    holes: &[CurvePath2],
) -> Result<bool, BuildError> {
    let (origin, u, v, direction) = frame;
    let boundary_curves = std::iter::once(profile)
        .chain(holes)
        .flat_map(|path| path.curves().iter())
        .collect::<Vec<_>>();
    if !top.certify_tensor_projection(origin, u, v, direction)? {
        return Ok(false);
    }
    let sections = match base.intersect_surface(top) {
        Ok(SurfaceSurfaceIntersection::None) => Vec::new(),
        Ok(SurfaceSurfaceIntersection::Curves(curves)) => curves,
        Ok(_) | Err(_) => return Ok(false),
    };
    for section in &sections {
        let trace = section
            .pcurve(SurfaceIntersectionOperand::First)
            .materialize()?
            .curve()
            .clone();
        // Each connected section either runs along the profile boundary, or
        // never meets it and has an outside point, so lies wholly outside.
        let mut on_boundary = false;
        let mut touches = false;
        for boundary in boundary_curves.iter().copied() {
            let Ok(relation) = trace.intersect_curve(boundary) else {
                return Ok(false);
            };
            if !relation.is_complete() {
                return Ok(false);
            }
            if let [overlap] = relation.overlaps()
                && relation.contacts().is_empty()
                && overlap.includes_start()
                && overlap.includes_end()
                && covers_whole_domain(overlap.first_range(), &trace)?
            {
                on_boundary = true;
                break;
            }
            touches |= !relation.overlaps().is_empty() || !relation.contacts().is_empty();
        }
        if on_boundary {
            continue;
        }
        if touches {
            return Ok(false);
        }
        // Any point of a section that never meets the boundary decides it.
        let outside = |point: &hypercurve::CurvePoint2| {
            region_location(profile, holes, point) == Some(ContourPointLocation::Outside)
        };
        if !outside(&trace.start()) {
            let domain = trace.parameter_domain();
            let (Some(lower), Some(upper)) = (domain.start().scalar(), domain.end().scalar())
            else {
                return Ok(false);
            };
            let middle = ((lower + upper) / Real::from(2)).map_err(GeometryError::Numeric)?;
            if !outside(
                &trace
                    .point_at(&middle.into())
                    .map_err(GeometryError::from)?,
            ) {
                return Ok(false);
            }
        }
    }
    // One interior point of the profile fixes the sign.
    // Selected vertices of polynomial boundary pieces are certified in their
    // branch's source chart so the search box spans the whole profile.
    let mut vertices = Vec::with_capacity(profile.curves().len());
    for curve in profile.curves() {
        let start = curve.start();
        if let Some(point) = start.coordinates() {
            vertices.push(point.clone());
            continue;
        }
        let original = curve
            .source_restriction()
            .map_or(curve, |(original, _, _)| original);
        if let Some(branch) = original.polynomial_branch()
            && let Some(point) = crate::geometry::certified_branch_point(&start, branch)?
        {
            vertices.push(point);
        }
    }
    if vertices.is_empty() {
        return Ok(false);
    }
    let (mut min_x, mut max_x) = (vertices[0].x().clone(), vertices[0].x().clone());
    let (mut min_y, mut max_y) = (vertices[0].y().clone(), vertices[0].y().clone());
    let less = |a: &Real, b: &Real| -> Result<bool, BuildError> {
        Ok(
            decided_model_order(compare_reals(a, b, crate::STRICT_PREDICATES))?
                == std::cmp::Ordering::Less,
        )
    };
    for point in &vertices[1..] {
        if less(point.x(), &min_x)? {
            min_x = point.x().clone();
        }
        if less(&max_x, point.x())? {
            max_x = point.x().clone();
        }
        if less(point.y(), &min_y)? {
            min_y = point.y().clone();
        }
        if less(&max_y, point.y())? {
            max_y = point.y().clone();
        }
    }
    for denominator in [2_i64, 4, 8] {
        for i in 1..denominator {
            for j in 1..denominator {
                let fraction = |k: i64| {
                    (Real::from(k) / Real::from(denominator)).map_err(GeometryError::Numeric)
                };
                let a = &min_x + (&max_x - &min_x) * fraction(i)?;
                let b = &min_y + (&max_y - &min_y) * fraction(j)?;
                let inside = region_location(
                    profile,
                    holes,
                    &CurvePoint2::new(a.clone(), b.clone()).into(),
                ) == Some(ContourPointLocation::Inside);
                if !inside {
                    continue;
                }
                return Ok(matches!(
                    top.tensor_graph_height_order(frame, &a, &b, &Real::zero())?,
                    Some(std::cmp::Ordering::Less)
                ));
            }
        }
    }
    Ok(false)
}

/// Whether an overlap range spans the curve's whole parameter domain, by
/// exact comparison of (possibly selected) parameters.
fn covers_whole_domain(
    range: &hypercurve::CurveParameterRange2,
    curve: &hypercurve::Curve2,
) -> Result<bool, BuildError> {
    let domain = curve.parameter_domain();
    let parameter = |value: &hypercurve::CurveParameter2| crate::Parameter::from_curve(value);
    let (start, end, lower, upper) = (
        parameter(range.start())?,
        parameter(range.end())?,
        parameter(domain.start())?,
        parameter(domain.end())?,
    );
    let equal = |a: &crate::Parameter, b: &crate::Parameter| -> Result<bool, BuildError> {
        Ok(a.compare(b)? == std::cmp::Ordering::Equal)
    };
    Ok((equal(&start, &lower)? && equal(&end, &upper)?)
        || (equal(&start, &upper)? && equal(&end, &lower)?))
}

/// The decided location of a point in the profile region minus its holes.
fn region_location(
    profile: &CurvePath2,
    holes: &[CurvePath2],
    point: &hypercurve::CurvePoint2,
) -> Option<ContourPointLocation> {
    let classify = |path: &CurvePath2| match crate::error::classified(path.classify_point(point)) {
        Ok(Classification::Decided(location)) => Some(location),
        _ => None,
    };
    let mut location = classify(profile)?;
    for hole in holes {
        if location == ContourPointLocation::Outside {
            break;
        }
        match classify(hole)? {
            ContourPointLocation::Inside => location = ContourPointLocation::Outside,
            ContourPointLocation::Boundary => location = ContourPointLocation::Boundary,
            ContourPointLocation::Outside => {}
        }
    }
    Some(location)
}
