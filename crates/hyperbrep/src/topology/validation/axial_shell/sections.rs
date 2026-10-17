//! Whole-band section extraction and oriented multiplicity proof.
use super::*;

impl ArenaModelBuilder {
    pub(super) fn axial_side(&self, id: FaceId, frame: &Frame) -> Result<Option<Side>, BuildError> {
        let face = self.face_ref(id)?;
        let surface = self.surface_ref(face.surface)?;
        let (plane, height_offset, height_scale, reversed) = match surface.exact_data() {
            SurfaceExactData::Plane { origin, u, v } => {
                let mut normal = u.cross(&v);
                if face.orientation == Orientation::Reversed {
                    normal = -normal;
                }
                let tangent = frame
                    .axis
                    .cross(&normal)
                    .normalize()
                    .map_err(GeometryError::Numeric)?;
                (Some((origin, tangent)), Real::zero(), Real::one(), false)
            }
            SurfaceExactData::Cylinder { origin, axis, .. } => {
                let scale = axis.dot(&frame.axis);
                let reversed = (order(&scale, &Real::zero())? == Ordering::Less)
                    != (face.orientation == Orientation::Reversed);
                (None, frame.height(&origin), scale, reversed)
            }
            _ => return Ok(None),
        };
        let mut contours = Vec::new();
        let mut boundaries = Vec::new();
        let mut angles = Vec::new();
        for wire in face.boundary_wires() {
            let mut segments = Vec::new();
            for usage in &self.wire_ref(*wire)?.edge_uses {
                let record = self.edge_use_ref(*usage)?;
                let (a, b) = if let Some((origin, tangent)) = &plane {
                    if self.curve_ref(self.edge_ref(record.edge)?.curve)?.kind() != Curve3Kind::Line
                    {
                        return Ok(None);
                    }
                    let (a, b) = self.directed_vertices(*usage)?;
                    let point = |id| -> Result<CurvePoint2, BuildError> {
                        let p = self.vertex_ref(id)?.represented_point()?;
                        Ok(CurvePoint2::new((p - origin).dot(tangent), frame.height(p)))
                    };
                    (point(a)?, point(b)?)
                } else {
                    let Some(line) = self.pcurve_ref(record.pcurve)?.line_segment() else {
                        return Ok(None);
                    };
                    let point = |p: &CurvePoint2| {
                        CurvePoint2::new(p.x().clone(), &height_offset + &height_scale * p.y())
                    };
                    angles.extend([line.start().x().clone(), line.end().x().clone()]);
                    (point(line.start()), point(line.end()))
                };
                // Structural orthogonality of a native line proves a constant
                // section over every band between all endpoint-height events.
                if real_values_equal(a.x(), b.x())? == real_values_equal(a.y(), b.y())? {
                    return Ok(None);
                }
                let line = LineSeg2::try_new(a, b).map_err(GeometryError::from)?;
                boundaries.push((record.edge, line.clone()));
                segments.push(Segment2::Line(line));
            }
            contours.push(Contour2::try_new(segments).map_err(GeometryError::from)?);
        }
        if contours.is_empty() {
            return Ok(None);
        }
        if !angles.is_empty() {
            let (low, high) = exact_real_min_max(&angles)?;
            // The chart may be shifted by any number of turns, but its image
            // cannot wrap twice and hide multiplicity at a canonical seam.
            if order(&(high - low), &Real::tau())? == Ordering::Greater {
                return Ok(None);
            }
        }
        Ok(Some(Side {
            surface: surface.clone(),
            plane,
            height_offset,
            height_scale,
            reversed,
            contours,
            boundaries,
        }))
    }
}

impl Side {
    pub(super) fn section(
        &self,
        frame: &Frame,
        height: &Real,
        shell: ShellId,
    ) -> Result<Vec<SectionEdge>, BuildError> {
        let mut crossing = Vec::<(Real, EdgeId)>::new();
        for (edge, line) in &self.boundaries {
            if !real_values_equal(line.start().x(), line.end().x())? {
                continue;
            }
            let a = order(line.start().y(), height)?;
            let b = order(line.end().y(), height)?;
            if !matches!(
                (a, b),
                (Ordering::Less, Ordering::Greater) | (Ordering::Greater, Ordering::Less)
            ) {
                continue;
            }
            let x = line.start().x();
            let mut position = crossing.len();
            for (i, (existing, _)) in crossing.iter().enumerate() {
                match order(x, existing)? {
                    Ordering::Less => {
                        position = i;
                        break;
                    }
                    Ordering::Equal => return Err(BuildError::SelfIntersectingSolidShell(shell)),
                    Ordering::Greater => {}
                }
            }
            crossing.insert(position, (x.clone(), *edge));
        }
        let mut result = Vec::new();
        for pair in crossing.windows(2) {
            let point = CurvePoint2::new(
                ((&pair[0].0 + &pair[1].0) / Real::from(2)).map_err(GeometryError::Numeric)?,
                height.clone(),
            );
            if self.contours[0]
                .classify_point(&point)
                .map_err(GeometryError::from)?
                != ContourPointLocation::Inside
            {
                continue;
            }
            let mut hole = false;
            for contour in &self.contours[1..] {
                if contour
                    .classify_point(&point)
                    .map_err(GeometryError::from)?
                    != ContourPointLocation::Outside
                {
                    hole = true;
                    break;
                }
            }
            if hole {
                continue;
            }
            let ((start, first), (end, second)) = (&pair[0], &pair[1]);
            let segment = if let Some((origin, tangent)) = &self.plane {
                let project = |x: &Real| frame.project(&(origin.clone() + tangent.clone() * x));
                Segment2::Line(
                    LineSeg2::try_new(project(start), project(end)).map_err(GeometryError::from)?,
                )
            } else {
                let v = ((height - &self.height_offset) / &self.height_scale)
                    .map_err(GeometryError::Numeric)?;
                let a = frame.project(
                    &self
                        .surface
                        .point_at(&Point2::new(start.clone(), v.clone()))?,
                );
                let b = frame.project(&self.surface.point_at(&Point2::new(end.clone(), v))?);
                let SurfaceExactData::Cylinder { origin, axis, .. } = self.surface.exact_data()
                else {
                    unreachable!("prevalidated cylinder side")
                };
                Segment2::Arc(
                    CircularArc2::try_from_center(
                        a,
                        b,
                        frame.project(&origin),
                        order(&axis.dot(&frame.axis), &Real::zero())? == Ordering::Less,
                    )
                    .map_err(GeometryError::from)?,
                )
            };
            result.push(if self.reversed {
                SectionEdge {
                    segment: segment.reversed(),
                    start: *second,
                    end: *first,
                }
            } else {
                SectionEdge {
                    segment,
                    start: *first,
                    end: *second,
                }
            });
        }
        Ok(result)
    }
}

pub(super) fn section_region(
    edges: Vec<SectionEdge>,
    shell: ShellId,
) -> Result<Option<CurveRegion2>, BuildError> {
    if edges.is_empty() {
        return Ok(None);
    }
    // Every contact in a section must be the shared axial edge, not merely a
    // coincident endpoint reconstructed with a different topology identity.
    for (i, a) in edges.iter().enumerate() {
        for b in &edges[i + 1..] {
            let mut allowed = Vec::new();
            for (aid, ap) in [(a.start, a.segment.start()), (a.end, a.segment.end())] {
                for (bid, bp) in [(b.start, b.segment.start()), (b.end, b.segment.end())] {
                    if aid == bid && equal_point(ap, bp)? {
                        allowed.push(ap.clone())
                    }
                }
            }
            if !contacts::only_contacts_at(&a.segment, &b.segment, &allowed)? {
                return Err(BuildError::SelfIntersectingSolidShell(shell));
            }
        }
    }
    let mut outgoing = std::collections::BTreeMap::new();
    for (i, e) in edges.iter().enumerate() {
        if outgoing.insert(e.start, i).is_some() {
            return Ok(None);
        }
    }
    let mut used = vec![false; edges.len()];
    let mut contours = Vec::new();
    for root in 0..edges.len() {
        if used[root] {
            continue;
        }
        let mut index = root;
        let mut segments = Vec::new();
        loop {
            if used[index] {
                return Ok(None);
            }
            used[index] = true;
            segments.push(edges[index].segment.clone());
            let Some(next) = outgoing.get(&edges[index].end) else {
                return Ok(None);
            };
            if *next == root {
                break;
            }
            index = *next;
        }
        contours.push(Contour2::try_new(segments).map_err(GeometryError::from)?);
    }
    // Pair contacts above prove simple, disjoint loops. Compute the winding on
    // both sides of EACH loop; summed area alone could hide positive and
    // negative multiplicity errors on different material components.
    let mut signs = Vec::new();
    for contour in &contours {
        let Some(area) = contour.signed_area().map_err(GeometryError::from)? else {
            return Ok(None);
        };
        signs.push(match order(&area, &Real::zero())? {
            Ordering::Greater => 1_i64,
            Ordering::Less => -1,
            Ordering::Equal => return Ok(None),
        });
    }
    for (i, contour) in contours.iter().enumerate() {
        let point = contour.segments()[0].start();
        let mut outside = 0_i64;
        for (j, other) in contours.iter().enumerate() {
            if i == j {
                continue;
            }
            match other.classify_point(point).map_err(GeometryError::from)? {
                ContourPointLocation::Outside => {}
                ContourPointLocation::Boundary => {
                    return Err(BuildError::SelfIntersectingSolidShell(shell));
                }
                ContourPointLocation::Inside => {
                    outside = outside
                        .checked_add(signs[j])
                        .ok_or(GeometryError::Numeric(hyperreal::Problem::Exhausted))?
                }
            }
        }
        if (signs[i] == 1 && outside != 0) || (signs[i] == -1 && outside != 1) {
            return Ok(None);
        }
    }
    Ok(Some(
        CurveRegion2::try_from_native_boundary_contours(&contours, hypercurve::FillRule::NonZero)
            .map_err(GeometryError::from)?,
    ))
}
