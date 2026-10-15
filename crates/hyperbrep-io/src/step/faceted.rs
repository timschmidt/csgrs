use super::{
    Error, ErrorKind,
    decode::{Args, Resolver, Site, reference},
    error::{insert, push},
    imported::{EdgeUseSource, ImportedBody, UseSource, VertexSource},
};
use hyperbrep::{
    Curve3, Direction, EdgeKey, FaceKey, ModelBuilder, Orientation, ParameterCorrespondence,
    ParameterDomain, Pcurve, Point3, Real, VertexKey, WireKey,
};
use hypercurve::{Curve2, LineSeg2, Point2};
use hyperreal::Rational;
use std::collections::{HashMap, HashSet};

struct Builder {
    model: ModelBuilder,
    by_source: HashMap<u64, (usize, VertexKey)>,
    by_point: HashMap<(Rational, Rational, Rational), (usize, VertexKey)>,
    points: Vec<Point3>,
    vertices: Vec<(VertexSource, VertexKey)>,
    faces: Vec<(u64, FaceKey)>,
    loops: Vec<(u64, u64, WireKey)>,
    edge_uses: Vec<UseSource>,
    edges: HashMap<(usize, usize), EdgeKey>,
}
pub(super) fn body(
    resolver: &mut Resolver<'_, '_, '_>,
    id: u64,
    scale: &Real,
) -> Result<ImportedBody, Error> {
    let mut args = resolver.simple(id, "FACETED_BREP")?;
    let body_site = args.site;
    let name = resolver.text(args.next()?, body_site)?;
    let shell_id = args.reference()?;
    args.finish()?;
    let mut shell = resolver.simple(shell_id, "CLOSED_SHELL")?;
    let site = shell.site;
    resolver.text(shell.next()?, site)?;
    let face_values = shell.list()?;
    shell.finish()?;
    let mut builder = Builder {
        model: ModelBuilder::new(),
        by_source: HashMap::new(),
        by_point: HashMap::new(),
        points: Vec::new(),
        vertices: Vec::new(),
        faces: Vec::new(),
        loops: Vec::new(),
        edge_uses: Vec::new(),
        edges: HashMap::new(),
    };
    let mut faces = Vec::new();
    let mut face_ids = HashSet::new();
    for value in face_values {
        let face_id = reference(value, site)?;
        if !insert(&mut face_ids, face_id).map_err(|kind| site.error(kind))? {
            return Err(site.invalid("duplicate shell face"));
        }
        let face = builder.face(resolver, face_id, scale)?;
        push(&mut builder.faces, (face_id, face.clone())).map_err(|kind| site.error(kind))?;
        push(&mut faces, face).map_err(|kind| site.error(kind))?;
    }
    if faces.is_empty() {
        return Err(site.invalid("empty closed shell"));
    }
    let shell = builder
        .model
        .shell(&faces)
        .map_err(|e| site.error(ErrorKind::Build(e)))?;
    let solid = builder
        .model
        .solid(&shell, &[])
        .map_err(|e| body_site.error(ErrorKind::Build(e)))?;
    let model = builder
        .model
        .finish()
        .map_err(|e| body_site.error(ErrorKind::Validation(e)))?;
    Ok(ImportedBody {
        source: id,
        name,
        model,
        solid,
        vertices: builder.vertices,
        faces: builder.faces,
        loops: builder.loops,
        edge_uses: builder.edge_uses,
        edges: Vec::new(),
    })
}
impl Builder {
    fn vertex(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        id: u64,
        scale: &Real,
        site: Site,
    ) -> Result<usize, Error> {
        if let Some((index, _)) = self.by_source.get(&id) {
            return Ok(*index);
        }
        let point = resolver.point(id)?;
        let point = Point3::new(&point.x * scale, &point.y * scale, &point.z * scale);
        let rational = |value: &Real| {
            value
                .exact_rational_ref()
                .cloned()
                .ok_or_else(|| site.error(ErrorKind::Unsupported("non-rational imported point")))
        };
        let canonical = (
            rational(&point.x)?,
            rational(&point.y)?,
            rational(&point.z)?,
        );
        let (index, key) = if let Some(entry) = self.by_point.get(&canonical) {
            entry.clone()
        } else {
            let index = self.points.len();
            let key = self
                .model
                .vertex(point.clone().into())
                .map_err(|e| site.error(ErrorKind::Build(e)))?;
            self.by_point
                .try_reserve(1)
                .map_err(|_| site.error(ErrorKind::Allocation))?;
            self.by_point.insert(canonical, (index, key.clone()));
            push(&mut self.points, point).map_err(|kind| site.error(kind))?;
            (index, key)
        };
        self.by_source
            .try_reserve(1)
            .map_err(|_| site.error(ErrorKind::Allocation))?;
        self.by_source.insert(id, (index, key.clone()));
        push(
            &mut self.vertices,
            (VertexSource::CartesianPoint { point: id }, key),
        )
        .map_err(|kind| site.error(kind))?;
        Ok(index)
    }
    fn face(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        id: u64,
        scale: &Real,
    ) -> Result<FaceKey, Error> {
        let mut args = resolver.simple(id, "FACE_SURFACE")?;
        let site = args.site;
        resolver.text(args.next()?, site)?;
        let bounds = args.list()?;
        let plane = resolver.plane(args.reference()?, scale)?;
        let orientation = if args.boolean()? {
            Orientation::Forward
        } else {
            Orientation::Reversed
        };
        args.finish()?;
        let surface = self
            .model
            .surface(plane.surface)
            .map_err(|e| site.error(ErrorKind::Build(e)))?;
        let mut outer = None;
        let mut holes = Vec::new();
        let mut bound_ids = HashSet::new();
        for value in bounds {
            let bound_id = reference(value, site)?;
            if !insert(&mut bound_ids, bound_id).map_err(|kind| site.error(kind))? {
                return Err(site.invalid("duplicate face bound"));
            }
            let entity = resolver.entity(bound_id)?;
            let bound_site = Site::entity(entity);
            let record = entity.components().next().expect("syntax component");
            if entity.is_complex() || !matches!(record.name(), "FACE_OUTER_BOUND" | "FACE_BOUND") {
                return Err(bound_site.error(ErrorKind::Expected("face bound")));
            }
            let is_outer = record.name() == "FACE_OUTER_BOUND";
            let mut bound = Args::new(record, bound_site);
            resolver.text(bound.next()?, bound_site)?;
            let loop_id = bound.reference()?;
            let forward = bound.boolean()?;
            bound.finish()?;
            let mut polygon = resolver.simple(loop_id, "POLY_LOOP")?;
            let loop_site = polygon.site;
            resolver.text(polygon.next()?, loop_site)?;
            let mut points = Vec::new();
            let mut unique_points = HashSet::new();
            for point in polygon.list()? {
                resolver.edge_use(loop_site)?;
                let point_id = reference(point, loop_site)?;
                let index = self.vertex(resolver, point_id, scale, loop_site)?;
                if !insert(&mut unique_points, index).map_err(|kind| loop_site.error(kind))? {
                    return Err(loop_site.invalid("poly-loop repeats a geometric vertex"));
                }
                let segment = points.len();
                push(&mut points, (index, point_id, segment))
                    .map_err(|kind| loop_site.error(kind))?;
            }
            polygon.finish()?;
            if points.len() < 3 {
                return Err(loop_site.invalid("poly-loop requires at least three points"));
            }
            if !forward {
                points.reverse();
            }
            let mut uses = Vec::new();
            for index in 0..points.len() {
                let (from, from_source, from_segment) = points[index];
                let (to, to_source, to_segment) = points[(index + 1) % points.len()];
                let pair = if from < to { (from, to) } else { (to, from) };
                let edge = if let Some(edge) = self.edges.get(&pair) {
                    edge.clone()
                } else {
                    let curve =
                        Curve3::line(self.points[pair.0].clone(), self.points[pair.1].clone())
                            .map_err(|e| loop_site.error(ErrorKind::Geometry(e)))?;
                    let curve = self
                        .model
                        .curve(curve)
                        .map_err(|e| loop_site.error(ErrorKind::Build(e)))?;
                    let (first, second) = if from < to {
                        (from_source, to_source)
                    } else {
                        (to_source, from_source)
                    };
                    let edge = self
                        .model
                        .edge(
                            &self.by_source[&first].1,
                            &self.by_source[&second].1,
                            &curve,
                            ParameterDomain::unit().into(),
                        )
                        .map_err(|e| loop_site.error(ErrorKind::Build(e)))?;
                    self.edges
                        .try_reserve(1)
                        .map_err(|_| loop_site.error(ErrorKind::Allocation))?;
                    self.edges.insert(pair, edge.clone());
                    edge
                };
                let uv = |index| {
                    let delta = &self.points[index] - &plane.origin;
                    Point2::new(delta.dot(&plane.x), delta.dot(&plane.y))
                };
                let pcurve = LineSeg2::try_new(uv(from), uv(to))
                    .map_err(|e| loop_site.error(ErrorKind::Geometry(e.into())))?;
                let pcurve = self
                    .model
                    .pcurve(Pcurve::new(Curve2::from(pcurve)))
                    .map_err(|e| loop_site.error(ErrorKind::Build(e)))?;
                let (direction, correspondence) = if from < to {
                    (Direction::Forward, ParameterCorrespondence::identity())
                } else {
                    (
                        Direction::Reversed,
                        ParameterCorrespondence::affine(-Real::one(), Real::one())
                            .map_err(|e| loop_site.error(ErrorKind::Build(e)))?,
                    )
                };
                let edge_use = self
                    .model
                    .edge_use(&edge, direction, &pcurve, correspondence)
                    .map_err(|e| loop_site.error(ErrorKind::Build(e)))?;
                push(
                    &mut self.edge_uses,
                    UseSource {
                        bound: bound_id,
                        loop_id,
                        source: EdgeUseSource::PolygonSegment {
                            segment: if forward { from_segment } else { to_segment },
                            from_point: from_source,
                            to_point: to_source,
                        },
                        key: edge_use.clone(),
                    },
                )
                .map_err(|kind| loop_site.error(kind))?;
                push(&mut uses, edge_use).map_err(|kind| loop_site.error(kind))?;
            }
            let wire = self
                .model
                .wire(&uses)
                .map_err(|e| loop_site.error(ErrorKind::Build(e)))?;
            push(&mut self.loops, (bound_id, loop_id, wire.clone()))
                .map_err(|kind| loop_site.error(kind))?;
            if is_outer {
                if outer.replace(wire).is_some() {
                    return Err(site.invalid("more than one outer bound"));
                }
            } else {
                push(&mut holes, wire).map_err(|kind| site.error(kind))?;
            }
        }
        let outer = outer.ok_or_else(|| site.error(ErrorKind::Expected("one FACE_OUTER_BOUND")))?;
        self.model
            .face(&surface, orientation, &outer, &holes)
            .map_err(|e| site.error(ErrorKind::Build(e)))
    }
}
