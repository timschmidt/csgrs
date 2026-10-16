//! Explicit STEP topology, with exact curve evidence admitted before publication.
mod circles;
mod curves;
mod faces;
mod surfaces;

use super::{
    ContextUnits, Error, ErrorKind,
    decode::{Args, Resolver, Site, reference},
    error::{insert, push},
    geometry::Plane,
    imported::{
        EdgeSource, EdgeUseSource, ImportedBody, SourceCurveParameters, UseSource, VertexSource,
    },
};
use hyperbrep::{
    Curve3, Direction, EdgeKey, FaceKey, GeometryError, ModelBuilder, Orientation,
    ParameterCorrespondence, ParameterDomain, Pcurve, Point3, Real, STRICT_PREDICATES, SurfaceKey,
    Vector3, VertexKey, WireKey,
};
use hypercurve::{Contour2, Curve2, LineSeg2, Point2, Segment2};
use hyperlimit::{PredicateOutcome, compare_reals};
use std::{
    cmp::Ordering,
    collections::{HashMap, HashSet},
};

fn order(site: Site, a: &Real, b: &Real) -> Result<Ordering, Error> {
    match compare_reals(a, b, STRICT_PREDICATES) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => Err(site.error(ErrorKind::Geometry(
            GeometryError::PredicateUnresolved { needed, stage },
        ))),
    }
}
fn same_vector(site: Site, a: &Vector3, b: &Vector3) -> Result<bool, Error> {
    for i in 0..3 {
        if order(site, &a[i], &b[i])? != Ordering::Equal {
            return Ok(false);
        }
    }
    Ok(true)
}
fn cache<K: Eq + std::hash::Hash, V>(
    map: &mut HashMap<K, V>,
    key: K,
    value: V,
    site: Site,
) -> Result<(), Error> {
    map.try_reserve(1)
        .map_err(|_| site.error(ErrorKind::Allocation))?;
    map.insert(key, value);
    Ok(())
}
#[derive(Clone)]
struct Vertex {
    key: VertexKey,
    point: Point3,
}
#[derive(Clone)]
struct Edge {
    key: EdgeKey,
    start: u64,
    end: u64,
    circle: Option<circles::CircleEdge>,
}
#[derive(Clone)]
struct FaceSurface {
    geometry: Plane,
    key: SurfaceKey,
    cylinder: Option<surfaces::Cylinder>,
}
struct Builder {
    model: ModelBuilder,
    by_vertex: HashMap<u64, Vertex>,
    by_edge: HashMap<u64, Edge>,
    by_surface: HashMap<u64, FaceSurface>,
    angle_scale: Option<Real>,
    vertices: Vec<(VertexSource, VertexKey)>,
    edges: Vec<EdgeSource>,
    faces: Vec<(u64, FaceKey)>,
    loops: Vec<(u64, u64, WireKey)>,
    edge_uses: Vec<UseSource>,
}
pub(super) fn body(
    resolver: &mut Resolver<'_, '_, '_>,
    id: u64,
    units: &ContextUnits,
) -> Result<ImportedBody, Error> {
    let scale = &units.length.scale_to_si;
    let mut args = resolver.simple(id, "MANIFOLD_SOLID_BREP")?;
    let body_site = args.site;
    let name = resolver.text(args.next()?, body_site)?;
    let shell_id = args.reference()?;
    args.finish()?;
    let mut args = resolver.simple(shell_id, "CLOSED_SHELL")?;
    let site = args.site;
    resolver.text(args.next()?, site)?;
    let face_values = args.list()?;
    args.finish()?;
    let mut builder = Builder {
        model: ModelBuilder::new(),
        by_vertex: HashMap::new(),
        by_edge: HashMap::new(),
        by_surface: HashMap::new(),
        angle_scale: units
            .plane_angle
            .as_ref()
            .map(|unit| unit.scale_to_si.clone()),
        vertices: Vec::new(),
        edges: Vec::new(),
        faces: Vec::new(),
        loops: Vec::new(),
        edge_uses: Vec::new(),
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
        edges: builder.edges,
        faces: builder.faces,
        loops: builder.loops,
        edge_uses: builder.edge_uses,
    })
}
impl Builder {
    fn vertex(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        id: u64,
        scale: &Real,
    ) -> Result<Vertex, Error> {
        let mut args = resolver.simple(id, "VERTEX_POINT")?;
        if let Some(vertex) = self.by_vertex.get(&id) {
            return Ok(vertex.clone());
        }
        let site = args.site;
        resolver.text(args.next()?, site)?;
        let point_id = args.reference()?;
        args.finish()?;
        let point = resolver.point(point_id)?;
        let point = Point3::new(&point.x * scale, &point.y * scale, &point.z * scale);
        let key = self
            .model
            .vertex(point.clone().into())
            .map_err(|e| site.error(ErrorKind::Build(e)))?;
        let vertex = Vertex {
            key: key.clone(),
            point,
        };
        cache(&mut self.by_vertex, id, vertex.clone(), site)?;
        push(
            &mut self.vertices,
            (
                VertexSource::VertexPoint {
                    vertex: id,
                    point: point_id,
                },
                key,
            ),
        )
        .map_err(|kind| site.error(kind))?;
        Ok(vertex)
    }
}
