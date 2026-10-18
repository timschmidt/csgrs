//! Explicit native topology over the selected exact analytic STEP carriers.
mod cone;
mod geometry;
mod spline;
mod types;

use super::{
    StepBody, StepWriteOptions, WriteError, WriteErrorKind,
    error::{Site, push},
    metadata,
    output::Output,
};
use hyperbrep::{
    Curve3Kind, Direction, EdgeRef, EntityKind, FaceRef, GeometryError, Orientation, Point3, Real,
    SurfaceKind, SurfaceRef, VertexRef, WireRef,
};
use std::collections::HashMap;
pub use types::*;

pub(super) fn write<'a>(
    sources: &[StepBody<'a>],
    options: &StepWriteOptions<'_>,
) -> Result<AdvancedStep<'a>, WriteError> {
    let mut out = Output::new(options.limits);
    if sources.is_empty() {
        return Err(out
            .site
            .error(WriteErrorKind::Invalid("empty solid selection")));
    }
    metadata::header(&mut out, options, metadata::GeometryEncoding::Advanced)?;
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
    let representation = out.entity("ADVANCED_BREP_SHAPE_REPRESENTATION", |out| {
        out.string(options.representation_name)?;
        out.put(",")?;
        out.references(&items)?;
        out.put(",")?;
        out.reference(context)
    })?;
    metadata::product(&mut out, options.representation_name, representation)?;
    out.put("ENDSEC;\nEND-ISO-10303-21;\n")?;
    Ok(AdvancedStep {
        text: out.text,
        representation,
        bodies,
    })
}

#[derive(Default)]
struct Body<'a> {
    points: HashMap<VertexRef<'a>, (usize, Point3)>,
    edge_ids: HashMap<EdgeRef<'a>, u64>,
    supports: HashMap<SurfaceRef<'a>, (u64, WrittenSurfaceParameters, bool)>,
    vertices: Vec<WrittenAdvancedVertex<'a>>,
    edges: Vec<WrittenAdvancedEdge<'a>>,
    faces: Vec<WrittenAdvancedFace<'a>>,
    wires: Vec<WrittenAdvancedWire<'a>>,
    edge_uses: Vec<WrittenAdvancedEdgeUse<'a>>,
}

fn body<'a>(
    out: &mut Output,
    source: StepBody<'a>,
    index: usize,
    scale: &Real,
) -> Result<WrittenAdvancedBody<'a>, WriteError> {
    let site = Site {
        body: Some(index),
        entity: Some((EntityKind::Solid, source.solid.index())),
    };
    out.site = site;
    if source.solid.voids().len() != 0 {
        return Err(site.error(WriteErrorKind::VoidShells));
    }
    let mut body = Body::default();
    let mut faces = Vec::new();
    for face in source.solid.outer().faces() {
        let id = body.face(out, face, scale)?;
        push(&mut faces, id, out.site)?;
    }
    out.site.entity = Some((EntityKind::Shell, source.solid.outer().index()));
    let shell = out.entity("CLOSED_SHELL", |out| {
        out.put("'',")?;
        out.references(&faces)
    })?;
    out.site = site;
    let entity = out.entity("MANIFOLD_SOLID_BREP", |out| {
        out.string(source.name)?;
        out.put(",")?;
        out.reference(shell)
    })?;
    Ok(WrittenAdvancedBody {
        source: source.solid,
        entity,
        shell,
        vertices: body.vertices,
        edges: body.edges,
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
    ) -> Result<WrittenAdvancedVertex<'a>, WriteError> {
        if let Some((index, _)) = self.points.get(&source) {
            return Ok(self.vertices[*index]);
        }
        out.site.entity = Some((EntityKind::Vertex, source.index()));
        let coordinates = source
            .point()
            .try_coordinates()
            .ok_or_else(|| out.site.error(WriteErrorKind::UnmaterializedPoint))?;
        let coordinates = geometry::converted(coordinates, scale, out)?;
        let point = geometry::point(out, &coordinates)?;
        let entity = out.entity("VERTEX_POINT", |out| {
            out.put("'',")?;
            out.reference(point)
        })?;
        let vertex = WrittenAdvancedVertex {
            source,
            entity,
            point,
        };
        self.points
            .try_reserve(1)
            .map_err(|_| out.site.error(WriteErrorKind::Allocation))?;
        self.points
            .insert(source, (self.vertices.len(), coordinates));
        push(&mut self.vertices, vertex, out.site)?;
        Ok(vertex)
    }

    fn edge(
        &mut self,
        out: &mut Output,
        source: EdgeRef<'a>,
        scale: &Real,
    ) -> Result<u64, WriteError> {
        if let Some(id) = self.edge_ids.get(&source) {
            return Ok(*id);
        }
        let site = Site {
            body: out.site.body,
            entity: Some((EntityKind::Edge, source.index())),
        };
        out.site = site;
        let curve = source.curve();
        if !matches!(
            curve.kind(),
            Curve3Kind::Line
                | Curve3Kind::CircleArc
                | Curve3Kind::RationalBezier
                | Curve3Kind::Nurbs
        ) {
            return Err(site.error(WriteErrorKind::UnsupportedCurve(curve.kind())));
        }
        let start = self.vertex(out, source.start(), scale)?;
        let end = self.vertex(out, source.end(), scale)?;
        out.site = site;
        let (geometry, parameter_start, parameter_end, same_sense) = match curve.kind() {
            Curve3Kind::Line => {
                let velocity = &self.points[&source.end()].1 - &self.points[&source.start()].1;
                let length = velocity.norm();
                (
                    geometry::line(out, start.point, &velocity)?,
                    Real::zero(),
                    length,
                    true,
                )
            }
            Curve3Kind::CircleArc => {
                let circle = curve.circular_geometry().expect("circular carrier");
                let radius = geometry::length(circle.radius, scale, out)?;
                let placement = geometry::placement(
                    out,
                    circle.center,
                    &circle.x.cross(circle.y),
                    circle.x,
                    scale,
                )?;
                let id = out.entity("CIRCLE", |out| {
                    out.put("'',")?;
                    out.reference(placement)?;
                    out.put(",")?;
                    out.number(&radius)
                })?;
                let (a, b) = source.domain().try_real_endpoints().ok_or_else(|| {
                    site.error(WriteErrorKind::Geometry(
                        GeometryError::UnrepresentableParameter,
                    ))
                })?;
                let angle = |parameter: &Real| {
                    let delta = parameter - curve.domain().start();
                    if circle.increasing_angle {
                        circle.angle_at_start + delta
                    } else {
                        circle.angle_at_start - delta
                    }
                };
                (id, angle(a), angle(b), circle.increasing_angle)
            }
            Curve3Kind::RationalBezier | Curve3Kind::Nurbs => {
                let (a, b) = source.domain().try_real_endpoints().ok_or_else(|| {
                    site.error(WriteErrorKind::Geometry(
                        GeometryError::UnrepresentableParameter,
                    ))
                })?;
                (
                    spline::curve(out, &curve, scale)?,
                    a.clone(),
                    b.clone(),
                    true,
                )
            }
            _ => unreachable!("checked carrier"),
        };
        let entity = out.entity("EDGE_CURVE", |out| {
            out.put("'',")?;
            out.reference(start.entity)?;
            out.put(",")?;
            out.reference(end.entity)?;
            out.put(",")?;
            out.reference(geometry)?;
            out.put(if same_sense { ",.T." } else { ",.F." })
        })?;
        self.edge_ids
            .try_reserve(1)
            .map_err(|_| site.error(WriteErrorKind::Allocation))?;
        self.edge_ids.insert(source, entity);
        push(
            &mut self.edges,
            WrittenAdvancedEdge {
                source,
                entity,
                geometry,
                parameter_start,
                parameter_end,
            },
            site,
        )?;
        Ok(entity)
    }

    fn wire(
        &mut self,
        out: &mut Output,
        source: WireRef<'a>,
        scale: &Real,
        outer: bool,
    ) -> Result<u64, WriteError> {
        let site = Site {
            body: out.site.body,
            entity: Some((EntityKind::Wire, source.index())),
        };
        let mut uses = Vec::new();
        for source in source.edge_uses() {
            let use_site = Site {
                body: site.body,
                entity: Some((EntityKind::EdgeUse, source.index())),
            };
            out.site = use_site;
            out.segment()?;
            let edge = self.edge(out, source.edge(), scale)?;
            out.site = use_site;
            let entity = out.entity("ORIENTED_EDGE", |out| {
                out.put("'',*,*,")?;
                out.reference(edge)?;
                out.put(match source.direction() {
                    Direction::Forward => ",.T.",
                    Direction::Reversed => ",.F.",
                })
            })?;
            push(
                &mut self.edge_uses,
                WrittenAdvancedEdgeUse { source, entity },
                use_site,
            )?;
            push(&mut uses, entity, use_site)?;
        }
        out.site = site;
        let entity = out.entity("EDGE_LOOP", |out| {
            out.put("'',")?;
            out.references(&uses)
        })?;
        let bound = out.entity(
            if outer {
                "FACE_OUTER_BOUND"
            } else {
                "FACE_BOUND"
            },
            |out| {
                out.put("'',")?;
                out.reference(entity)?;
                out.put(",.T.")
            },
        )?;
        push(
            &mut self.wires,
            WrittenAdvancedWire {
                source,
                entity,
                bound,
            },
            site,
        )?;
        Ok(bound)
    }

    fn face(
        &mut self,
        out: &mut Output,
        source: FaceRef<'a>,
        scale: &Real,
    ) -> Result<u64, WriteError> {
        let site = Site {
            body: out.site.body,
            entity: Some((EntityKind::Face, source.index())),
        };
        out.site = site;
        let support = source.surface();
        let (surface, parameters, reversed) = if let Some(value) = self.supports.get(&support) {
            value.clone()
        } else {
            let value = if support.kind() == SurfaceKind::Cone {
                cone::surface(out, source, scale)?
            } else {
                geometry::surface(out, &support, scale)?
            };
            self.supports
                .try_reserve(1)
                .map_err(|_| site.error(WriteErrorKind::Allocation))?;
            self.supports.insert(support, value.clone());
            value
        };
        let outer = source.outer().ok_or_else(|| {
            site.error(WriteErrorKind::Invalid("advanced face needs an outer wire"))
        })?;
        let mut bounds = Vec::new();
        let bound = self.wire(out, outer, scale, true)?;
        push(&mut bounds, bound, site)?;
        for wire in source.inner() {
            let bound = self.wire(out, wire, scale, false)?;
            push(&mut bounds, bound, site)?;
        }
        out.site = site;
        let orientation = if reversed {
            source.orientation().reversed()
        } else {
            source.orientation()
        };
        let entity = out.entity("ADVANCED_FACE", |out| {
            out.put("'',")?;
            out.references(&bounds)?;
            out.put(",")?;
            out.reference(surface)?;
            out.put(match orientation {
                Orientation::Forward => ",.T.",
                Orientation::Reversed => ",.F.",
            })
        })?;
        push(
            &mut self.faces,
            WrittenAdvancedFace {
                source,
                entity,
                surface,
                parameters,
                orientation,
            },
            site,
        )?;
        Ok(entity)
    }
}
