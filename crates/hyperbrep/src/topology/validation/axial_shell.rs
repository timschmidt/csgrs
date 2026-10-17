//! Complete boundary admission for piecewise prismatic plane/cylinder shells.
//!
//! A representative section is used only after every lateral boundary image
//! proves constant topology throughout its open height band. Simple disjoint
//! oriented section loops prove unit winding; cap differences prove the exact
//! transitions. Complete event contacts and vertex links establish embedding.
use crate::topology::*;
use std::cmp::Ordering;

mod contacts;
mod sections;
#[cfg(test)]
mod tests;

struct Frame {
    origin: Point3,
    u: Vector3,
    v: Vector3,
    axis: Vector3,
}

impl Frame {
    fn height(&self, point: &Point3) -> Real {
        (point - &self.origin).dot(&self.axis)
    }

    fn project(&self, point: &Point3) -> CurvePoint2 {
        let offset = point - &self.origin;
        CurvePoint2::new(offset.dot(&self.u), offset.dot(&self.v))
    }
}

struct Side {
    surface: Surface,
    // Plane coordinate increases along axis × outward normal. Cylinder
    // coordinate is the authored angular chart, with its orientation retained.
    plane: Option<(Point3, Vector3)>,
    height_offset: Real,
    height_scale: Real,
    reversed: bool,
    contours: Vec<Contour2>,
    boundaries: Vec<(EdgeId, LineSeg2)>,
}

#[derive(Clone)]
struct SectionEdge {
    segment: Segment2,
    start: EdgeId,
    end: EdgeId,
}

struct Cap {
    height: Real,
    upward: bool,
    region: CurveRegion2,
}

fn order(a: &Real, b: &Real) -> Result<Ordering, BuildError> {
    Ok(decided_model_order(compare_reals(
        a,
        b,
        crate::STRICT_PREDICATES,
    ))?)
}

fn zero(value: Real) -> Result<bool, BuildError> {
    Ok(order(&value, &Real::zero())? == Ordering::Equal)
}

fn equal_point(a: &CurvePoint2, b: &CurvePoint2) -> Result<bool, BuildError> {
    Ok(real_values_equal(a.x(), b.x())? && real_values_equal(a.y(), b.y())?)
}

impl ArenaModelBuilder {
    pub(in crate::topology) fn certified_axial_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedAxialShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut frame = None;
        for id in faces {
            let surface = self.surface_ref(self.face_ref(*id)?.surface)?;
            match surface.exact_data() {
                SurfaceExactData::Cylinder {
                    origin, x, y, axis, ..
                } => {
                    frame.get_or_insert(Frame {
                        origin,
                        u: x,
                        v: y,
                        axis,
                    });
                }
                SurfaceExactData::Plane { .. } => {}
                _ => return Ok(None),
            }
        }
        let Some(frame) = frame else { return Ok(None) };
        let mut heights = Vec::new();
        let mut sides = Vec::new();
        let mut caps = Vec::new();
        let mut edges = std::collections::BTreeSet::new();
        let mut vertices = std::collections::BTreeSet::new();
        for id in faces {
            let face = self.face_ref(*id)?;
            for wire in face.boundary_wires() {
                for edge_use in &self.wire_ref(*wire)?.edge_uses {
                    let edge_id = self.edge_use_ref(*edge_use)?.edge;
                    edges.insert(edge_id);
                    let edge = self.edge_ref(edge_id)?;
                    vertices.extend([edge.start, edge.end]);
                }
            }
            let surface = self.surface_ref(face.surface)?;
            match surface.exact_data() {
                SurfaceExactData::Plane { origin, u, v } => {
                    let normal = u.cross(&v);
                    if zero(normal.cross(&frame.axis).norm_squared())? {
                        let Some(region) = self.axial_cap_region(*id, &frame)? else {
                            return Ok(None);
                        };
                        let mut sign = normal.dot(&frame.axis);
                        if face.orientation == Orientation::Reversed {
                            sign = -sign;
                        }
                        caps.push(Cap {
                            height: frame.height(&origin),
                            upward: order(&sign, &Real::zero())? == Ordering::Greater,
                            region,
                        });
                    } else if zero(normal.dot(&frame.axis))? {
                        let Some(side) = self.axial_side(*id, &frame)? else {
                            return Ok(None);
                        };
                        sides.push(side);
                    } else {
                        return Ok(None);
                    }
                }
                SurfaceExactData::Cylinder { axis, .. } => {
                    if !zero(axis.cross(&frame.axis).norm_squared())? {
                        return Ok(None);
                    }
                    let Some(side) = self.axial_side(*id, &frame)? else {
                        return Ok(None);
                    };
                    sides.push(side);
                }
                _ => return Ok(None),
            }
        }
        for vertex in &vertices {
            insert_sorted_real(
                &mut heights,
                &frame.height(self.vertex_ref(*vertex)?.represented_point()?),
            )?;
        }
        if heights.len() < 2 || sides.is_empty() || caps.is_empty() {
            return Ok(None);
        }
        // Only these complete native edge images are admitted. Endpoint levels
        // alone would not exclude a bowed or helical boundary between events.
        for id in &edges {
            let edge = self.edge_ref(*id)?;
            let a = self.vertex_ref(edge.start)?.represented_point()?;
            let b = self.vertex_ref(edge.end)?.represented_point()?;
            match self.curve_ref(edge.curve)?.exact_data() {
                Curve3ExactData::Line(_) => {
                    if !real_values_equal(&frame.height(a), &frame.height(b))?
                        && !zero((b - a).cross(&frame.axis).norm_squared())?
                    {
                        return Ok(None);
                    }
                }
                Curve3ExactData::EllipseArc(circle) if circle.circle => {
                    if !zero(circle.x.dot(&frame.axis))? || !zero(circle.y.dot(&frame.axis))? {
                        return Ok(None);
                    }
                }
                _ => return Ok(None),
            }
        }
        self.axial_vertex_links(shell)?;
        self.axial_event_contacts(shell, &frame, &edges, &vertices)?;
        let mut bands = Vec::with_capacity(heights.len() - 1);
        for interval in heights.windows(2) {
            let middle =
                ((&interval[0] + &interval[1]) / Real::from(2)).map_err(GeometryError::Numeric)?;
            let mut section = Vec::new();
            for side in &sides {
                section.extend(side.section(&frame, &middle, shell)?);
            }
            let Some(region) = sections::section_region(section, shell)? else {
                return Ok(None);
            };
            let Some(area) = region.filled_area().map_err(GeometryError::from)? else {
                return Ok(None);
            };
            if order(&area, &Real::zero())? != Ordering::Greater {
                return Ok(None);
            }
            bands.push(CertifiedAxialBand { region, area });
        }
        let empty = CurveRegion2::empty();
        for (index, height) in heights.iter().enumerate() {
            let lower = index.checked_sub(1).map_or(&empty, |i| &bands[i].region);
            let upper = bands.get(index).map_or(&empty, |b| &b.region);
            for upward in [true, false] {
                let mut actual = CurveRegion2::empty();
                for cap in &caps {
                    if cap.upward != upward || !real_values_equal(&cap.height, height)? {
                        continue;
                    }
                    let outcomes = actual
                        .boolean_regions(&cap.region)
                        .map_err(GeometryError::from)?;
                    // Union alone would discard overlapping, duplicate caps.
                    if !outcomes.intersection().is_empty() {
                        return Err(BuildError::SelfIntersectingSolidShell(shell));
                    }
                    actual = outcomes.union().clone();
                }
                let (from, to) = if upward {
                    (lower, upper)
                } else {
                    (upper, lower)
                };
                let expected = from
                    .boolean_region(to, BooleanOp::Difference)
                    .map_err(GeometryError::from)?;
                if !actual
                    .boolean_region(&expected, BooleanOp::Xor)
                    .map_err(GeometryError::from)?
                    .is_empty()
                {
                    return Ok(None);
                }
            }
        }
        Ok(Some(CertifiedAxialShell {
            origin: frame.origin,
            u: frame.u,
            v: frame.v,
            axis: frame.axis,
            heights,
            bands,
        }))
    }

    fn axial_cap_region(
        &self,
        id: FaceId,
        frame: &Frame,
    ) -> Result<Option<CurveRegion2>, BuildError> {
        let mut contours = Vec::new();
        for wire in self.face_ref(id)?.boundary_wires() {
            let mut segments = Vec::new();
            for edge_use in &self.wire_ref(*wire)?.edge_uses {
                let Some(segment) = self.axial_projected_edge(*edge_use, frame)? else {
                    return Ok(None);
                };
                segments.push(segment);
            }
            contours.push(Contour2::try_new(segments).map_err(GeometryError::from)?);
        }
        if contours.is_empty() {
            return Ok(None);
        }
        // Face construction already proves the authored outer/hole roles.
        let outer = contours.remove(0);
        Ok(Some(
            CurveRegion2::try_from_native_contours(vec![outer], contours)
                .map_err(GeometryError::from)?,
        ))
    }

    fn axial_projected_edge(
        &self,
        edge_use: EdgeUseId,
        frame: &Frame,
    ) -> Result<Option<Segment2>, BuildError> {
        let usage = self.edge_use_ref(edge_use)?;
        let edge = self.edge_ref(usage.edge)?;
        let source = self.curve_ref(edge.curve)?;
        if !matches!(source.kind(), Curve3Kind::Line | Curve3Kind::CircleArc) {
            return Ok(None);
        }
        let curve = source.subcurve(
            edge.domain.represented_start()?,
            edge.domain.represented_end()?,
        )?;
        let Some(mut projected) = crate::geometry::project_curve_to_plane_frame(
            &curve,
            &frame.origin,
            &frame.u,
            &frame.v,
        )?
        else {
            return Ok(None);
        };
        if usage.direction == Direction::Reversed {
            projected = projected.reversed().map_err(GeometryError::from)?;
        }
        Ok(Some(Pcurve::new(projected).segment()?))
    }
}
