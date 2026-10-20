use super::{
    FacetedStep, StepBody, StepWriteOptions, WriteError, WriteErrorKind, WrittenBody,
    WrittenEdgeUse, WrittenFace, WrittenVertex, WrittenWire,
    error::{Site, push},
    metadata, number,
    output::Output,
};
use hyperbrep::{
    Curve3Kind, Direction, EntityKind, FaceRef, Point3, Real, SurfaceKind, Vector3, VertexRef,
    WireRef,
};
use std::collections::HashMap;

pub(super) fn write<'a>(
    sources: &[StepBody<'a>],
    options: &StepWriteOptions<'_>,
) -> Result<FacetedStep<'a>, WriteError> {
    let mut out = Output::new(options.limits);
    if sources.is_empty() {
        return Err(out
            .site
            .error(WriteErrorKind::Invalid("empty solid selection")));
    }
    metadata::header(&mut out, options, metadata::GeometryEncoding::Faceted)?;
    let context = metadata::context(&mut out, options)?;
    let mut bodies = Vec::new();
    let mut items = Vec::new();
    for (index, source) in sources.iter().enumerate() {
        let body = body(
            &mut out,
            *source,
            index,
            &options.length_unit.metres_per_unit,
        )?;
        push(&mut items, body.entity, out.site)?;
        push(&mut bodies, body, out.site)?;
    }
    out.site = Site::default();
    let representation = out.entity("FACETED_BREP_SHAPE_REPRESENTATION", |out| {
        out.string(options.representation_name)?;
        out.put(",")?;
        out.references(&items)?;
        out.put(",")?;
        out.reference(context)
    })?;
    metadata::product(&mut out, options.representation_name, representation)?;
    out.put("ENDSEC;\nEND-ISO-10303-21;\n")?;
    Ok(FacetedStep {
        text: out.text,
        representation,
        bodies,
    })
}

struct Body<'a> {
    points: HashMap<VertexRef<'a>, (u64, Point3)>,
    vertices: Vec<WrittenVertex<'a>>,
    faces: Vec<WrittenFace<'a>>,
    wires: Vec<WrittenWire<'a>>,
    edge_uses: Vec<WrittenEdgeUse<'a>>,
}
fn body<'a>(
    out: &mut Output,
    source: StepBody<'a>,
    index: usize,
    scale: &Real,
) -> Result<WrittenBody<'a>, WriteError> {
    let site = Site {
        body: Some(index),
        entity: Some((EntityKind::Solid, source.solid.index())),
    };
    out.site = site;
    if source.solid.voids().len() != 0 {
        return Err(site.error(WriteErrorKind::VoidShells));
    }
    let mut body = Body {
        points: HashMap::new(),
        vertices: Vec::new(),
        faces: Vec::new(),
        wires: Vec::new(),
        edge_uses: Vec::new(),
    };
    let mut faces = Vec::new();
    for face in source.solid.outer().faces() {
        let id = body.face(out, face, scale)?;
        push(&mut faces, id, out.site)?;
    }
    out.site = Site {
        body: Some(index),
        entity: Some((EntityKind::Shell, source.solid.outer().index())),
    };
    let shell = out.entity("CLOSED_SHELL", |out| {
        out.put("'',")?;
        out.references(&faces)
    })?;
    out.site = site;
    let entity = out.entity("FACETED_BREP", |out| {
        out.string(source.name)?;
        out.put(",")?;
        out.reference(shell)
    })?;
    Ok(WrittenBody {
        source: source.solid,
        entity,
        shell,
        vertices: body.vertices,
        faces: body.faces,
        wires: body.wires,
        edge_uses: body.edge_uses,
    })
}
impl<'a> Body<'a> {
    fn vertex(
        &mut self,
        out: &mut Output,
        source: VertexRef<'a>,
        scale: &Real,
    ) -> Result<u64, WriteError> {
        if let Some((id, _)) = self.points.get(&source) {
            return Ok(*id);
        }
        out.site.entity = Some((EntityKind::Vertex, source.index()));
        let point = source
            .point()
            .try_coordinates()
            .ok_or_else(|| out.site.error(WriteErrorKind::UnmaterializedPoint))?;
        // Admission applies before the requested unit conversion, and again to its exact result.
        for value in [&point.x, &point.y, &point.z] {
            number::exact(value, &out.limits, out.site)?;
        }
        let convert = |value: &Real| {
            (value / scale).map_err(|problem| out.site.error(WriteErrorKind::Numeric(problem)))
        };
        let point = Point3::new(convert(&point.x)?, convert(&point.y)?, convert(&point.z)?);
        let entity = out.entity("CARTESIAN_POINT", |out| {
            out.put("'',(")?;
            out.number(&point.x)?;
            out.put(",")?;
            out.number(&point.y)?;
            out.put(",")?;
            out.number(&point.z)?;
            out.put(")")
        })?;
        self.points
            .try_reserve(1)
            .map_err(|_| out.site.error(WriteErrorKind::Allocation))?;
        self.points.insert(source, (entity, point));
        push(
            &mut self.vertices,
            WrittenVertex { source, entity },
            out.site,
        )?;
        Ok(entity)
    }
    fn wire(
        &mut self,
        out: &mut Output,
        wire: WireRef<'a>,
        scale: &Real,
        outer: bool,
    ) -> Result<(u64, Vec<VertexRef<'a>>), WriteError> {
        let site = Site {
            body: out.site.body,
            entity: Some((EntityKind::Wire, wire.index())),
        };
        out.site = site;
        let mut points = Vec::new();
        let mut sources = Vec::new();
        let mut uses = Vec::new();
        for source in wire.edge_uses() {
            out.site.entity = Some((EntityKind::EdgeUse, source.index()));
            out.segment()?;
            let edge = source.edge();
            if edge.curve().kind() != Curve3Kind::Line {
                return Err(out
                    .site
                    .error(WriteErrorKind::UnsupportedCurve(edge.curve().kind())));
            }
            let vertex = match source.direction() {
                Direction::Forward => edge.start(),
                Direction::Reversed => edge.end(),
            };
            let id = self.vertex(out, vertex, scale)?;
            push(&mut points, id, out.site)?;
            push(&mut sources, vertex, out.site)?;
            push(&mut uses, source, out.site)?;
        }
        out.site = site;
        if points.len() < 3 {
            return Err(site.error(WriteErrorKind::Invalid("poly-loop requires three vertices")));
        }
        let polygon = out.entity("POLY_LOOP", |out| {
            out.put("'',")?;
            out.references(&points)
        })?;
        let bound = out.entity(
            if outer {
                "FACE_OUTER_BOUND"
            } else {
                "FACE_BOUND"
            },
            |out| {
                out.put("'',")?;
                out.reference(polygon)?;
                out.put(",.T.")
            },
        )?;
        push(
            &mut self.wires,
            WrittenWire {
                source: wire,
                polygon,
                bound,
            },
            site,
        )?;
        for (segment, source) in uses.into_iter().enumerate() {
            push(
                &mut self.edge_uses,
                WrittenEdgeUse {
                    source,
                    polygon,
                    segment,
                },
                site,
            )?;
        }
        Ok((bound, sources))
    }
    fn face(
        &mut self,
        out: &mut Output,
        face: FaceRef<'a>,
        scale: &Real,
    ) -> Result<u64, WriteError> {
        let site = Site {
            body: out.site.body,
            entity: Some((EntityKind::Face, face.index())),
        };
        out.site = site;
        if face.surface().kind() != SurfaceKind::Plane {
            return Err(site.error(WriteErrorKind::UnsupportedSurface(face.surface().kind())));
        }
        let outer = face.outer().ok_or_else(|| {
            site.error(WriteErrorKind::Invalid("faceted face needs an outer wire"))
        })?;
        let (bound, vertices) = self.wire(out, outer, scale, true)?;
        let mut bounds = Vec::new();
        push(&mut bounds, bound, site)?;
        for wire in face.inner() {
            let (bound, _) = self.wire(out, wire, scale, false)?;
            push(&mut bounds, bound, site)?;
        }
        out.site = site;
        let (origin_id, origin) = &self.points[&vertices[0]];
        let mut normal = Vector3::new([Real::zero(), Real::zero(), Real::zero()]);
        for index in 1..vertices.len() - 1 {
            let a = &self.points[&vertices[index]].1 - origin;
            let b = &self.points[&vertices[index + 1]].1 - origin;
            normal = &normal + &a.cross(&b);
        }
        let reference = &self.points[&vertices[1]].1 - origin;
        let normal = number::primitive_direction(&normal, &out.limits, site)?;
        let reference = number::primitive_direction(&reference, &out.limits, site)?;
        let mut direction = |vector: &Vector3| {
            out.entity("DIRECTION", |out| {
                out.put("'',(")?;
                out.number(&vector[0])?;
                out.put(",")?;
                out.number(&vector[1])?;
                out.put(",")?;
                out.number(&vector[2])?;
                out.put(")")
            })
        };
        let axis = direction(&normal)?;
        let x = direction(&reference)?;
        let placement = out.entity("AXIS2_PLACEMENT_3D", |out| {
            out.put("'',")?;
            out.reference(*origin_id)?;
            out.put(",")?;
            out.reference(axis)?;
            out.put(",")?;
            out.reference(x)
        })?;
        let plane = out.entity("PLANE", |out| {
            out.put("'',")?;
            out.reference(placement)
        })?;
        let entity = out.entity("FACE_SURFACE", |out| {
            out.put("'',")?;
            out.references(&bounds)?;
            out.put(",")?;
            out.reference(plane)?;
            out.put(",.T.")
        })?;
        push(
            &mut self.faces,
            WrittenFace {
                source: face,
                entity,
                plane,
            },
            site,
        )?;
        Ok(entity)
    }
}
