//! Exact construction helpers for retained BREP objects.
//!
//! This module is deliberately narrow. It turns an already exact
//! `hypercurve::Region2` made of line contours into one retained planar BREP
//! face on an already retained exact surface frame. It does not sample curves,
//! infer a plane, heal loops, or tessellate. Those would be separate
//! topology-changing decisions and must be certified before they can produce
//! trusted BREP evidence.

use std::collections::BTreeSet;

use hypercurve::{Contour2, Region2, Segment2};
use hyperlimit::{Plane3, Point2 as LimitPoint2, Point3};
use hyperreal::Real;

use crate::provenance::{
    BrepConstructionKind, BrepConstructionManifest, BrepFeatureId, BrepSourceVersion,
};
use crate::surface::{BrepSurface, BrepSurfaceId, BrepSurfaceSource};
use crate::topology::{
    BrepCoedge, BrepEdge, BrepEdgeId, BrepEdgeOrientation, BrepFace, BrepFaceId, BrepLoop,
    BrepLoopId, BrepShell, BrepVertex, BrepVertexId,
};

/// Explicit blocker for exact planar-region BREP construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepPlanarRegionConstructionBlocker {
    /// The source region has no material contour.
    EmptyRegion,
    /// The first construction slice accepts one material contour only.
    MultipleMaterialContours,
    /// A contour has no boundary segments.
    EmptyContour,
    /// A contour contains an unsupported non-line segment.
    UnsupportedCurveSegment,
    /// Retained contour segment endpoints do not form a closed exact chain.
    BrokenContourChain,
    /// The supplied surface frame cannot evaluate exact UV coordinates.
    SurfaceFrameNotReady,
    /// UV-to-3D evaluation failed for a retained region point.
    SurfaceEvaluationFailed,
    /// Exact construction produced topology that failed validation.
    ConstructedShellNotExactReady,
    /// Exact construction provenance could not be marked fresh.
    ConstructionManifestNotFresh,
}

/// Exact planar-region construction artifact.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepPlanarRegionConstruction {
    /// Constructed retained BREP shell, when every construction gate succeeds.
    pub shell: Option<BrepShell>,
    /// Construction manifest tied to the emitted shell fingerprint.
    pub manifest: Option<BrepConstructionManifest>,
    /// Number of material contours in the source region.
    pub material_contour_count: usize,
    /// Number of hole contours in the source region.
    pub hole_contour_count: usize,
    /// Number of retained exact vertices emitted.
    pub vertex_count: usize,
    /// Number of retained exact edges emitted.
    pub edge_count: usize,
    /// Explicit construction blockers.
    pub blockers: Vec<BrepPlanarRegionConstructionBlocker>,
    /// Whether the retained shell and manifest are exact construction evidence.
    pub exact_construction_ready: bool,
}

/// Explicit blocker for exact linear extrusion construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepPlanarExtrusionConstructionBlocker {
    /// The source region has no material contour.
    EmptyRegion,
    /// This exact extrusion slice accepts one material contour only.
    MultipleMaterialContours,
    /// A hole contour could not be lowered into an inner wall loop.
    HoleContourInvalid,
    /// A source contour contains no boundary segments.
    EmptyContour,
    /// A source contour contains an unsupported non-line segment.
    UnsupportedCurveSegment,
    /// Source contour segment endpoints do not form a closed exact chain.
    BrokenContourChain,
    /// Extrusion height is exactly zero or negative.
    NonPositiveHeight,
    /// Extrusion height sign could not be certified.
    UnknownHeightSign,
    /// Exact construction produced topology that failed solid validation.
    ConstructedSolidNotExactReady,
    /// Exact construction provenance could not be marked fresh.
    ConstructionManifestNotFresh,
}

/// Exact vertical extrusion construction artifact.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepPlanarExtrusionConstruction {
    /// Constructed retained BREP shell, when every construction gate succeeds.
    pub shell: Option<BrepShell>,
    /// Construction manifest tied to the emitted shell fingerprint.
    pub manifest: Option<BrepConstructionManifest>,
    /// Number of source contour vertices across material and hole contours.
    pub source_vertex_count: usize,
    /// Number of retained exact vertices emitted.
    pub vertex_count: usize,
    /// Number of retained exact edges emitted.
    pub edge_count: usize,
    /// Number of retained exact faces emitted.
    pub face_count: usize,
    /// Explicit construction blockers.
    pub blockers: Vec<BrepPlanarExtrusionConstructionBlocker>,
    /// Whether the retained shell and manifest are exact solid evidence.
    pub exact_construction_ready: bool,
}

impl BrepPlanarRegionConstruction {
    /// Construct a single planar BREP face from a line-only `Region2`.
    ///
    /// This minimal bridge from `hypercurve` retains the exact curve object
    /// until surface-frame evaluation constructs model-space vertices.
    /// Unsupported segments block construction instead of falling back to a
    /// display polyline.
    pub fn from_region_on_surface(
        region: &Region2,
        surface: BrepSurface,
        feature: BrepFeatureId,
        sources: Vec<BrepSourceVersion>,
    ) -> Self {
        let mut blockers = BTreeSet::new();
        let material_contour_count = region.material_contours().len();
        let hole_contour_count = region.hole_contours().len();
        if material_contour_count == 0 {
            blockers.insert(BrepPlanarRegionConstructionBlocker::EmptyRegion);
        }
        if material_contour_count > 1 {
            blockers.insert(BrepPlanarRegionConstructionBlocker::MultipleMaterialContours);
        }
        let frame = surface.frame_report();
        if !frame.exact_frame_ready {
            blockers.insert(BrepPlanarRegionConstructionBlocker::SurfaceFrameNotReady);
        }

        let mut vertices = Vec::new();
        let mut edges = Vec::new();
        let mut next_vertex_id = 0_u64;
        let mut next_edge_id = 0_u64;
        let mut next_loop_id = 0_u64;

        let (outer, inner) = {
            let mut loop_builder = PlanarLoopBuilder {
                surface: &surface,
                vertices: &mut vertices,
                edges: &mut edges,
                next_vertex_id: &mut next_vertex_id,
                next_edge_id: &mut next_edge_id,
                next_loop_id: &mut next_loop_id,
                blockers: &mut blockers,
            };
            let outer = region
                .material_contours()
                .first()
                .and_then(|contour| loop_builder.build(contour));

            let mut inner = Vec::new();
            for contour in region.hole_contours() {
                if let Some(face_loop) = loop_builder.build(contour) {
                    inner.push(face_loop);
                }
            }
            (outer, inner)
        };

        let shell = if blockers.is_empty() {
            outer.map(|outer| BrepShell {
                vertices,
                edges,
                faces: vec![BrepFace::with_inner(
                    BrepFaceId(0),
                    surface.id,
                    outer,
                    inner,
                )],
                surfaces: vec![surface],
            })
        } else {
            None
        };

        let (shell, manifest) = match shell {
            Some(shell) => {
                let validation = shell.face_validation_report(BrepFaceId(0), None);
                if !validation.exact_face_boundary_ready {
                    blockers
                        .insert(BrepPlanarRegionConstructionBlocker::ConstructedShellNotExactReady);
                }
                let manifest = BrepConstructionManifest::exact(
                    feature,
                    BrepConstructionKind::PlanarFace,
                    sources,
                    &shell,
                );
                if !manifest.report(&shell).construction_fresh {
                    blockers
                        .insert(BrepPlanarRegionConstructionBlocker::ConstructionManifestNotFresh);
                }
                if blockers.is_empty() {
                    (Some(shell), Some(manifest))
                } else {
                    (None, None)
                }
            }
            None => (None, None),
        };

        let blockers = blockers.into_iter().collect::<Vec<_>>();
        Self {
            material_contour_count,
            hole_contour_count,
            vertex_count: shell.as_ref().map_or(0, |shell| shell.vertices.len()),
            edge_count: shell.as_ref().map_or(0, |shell| shell.edges.len()),
            exact_construction_ready: blockers.is_empty() && shell.is_some() && manifest.is_some(),
            shell,
            manifest,
            blockers,
        }
    }
}

impl BrepPlanarExtrusionConstruction {
    /// Construct a closed vertical prism shell from a line-only `Region2`.
    ///
    /// This is the first solid construction bridge from `hypercurve` into
    /// `hyperbrep`. It preserves material and hole contours as BREP edges and
    /// emits analytic side planes instead of triangulating or sampling.
    /// Unsupported source families and unresolved signs block the shell before
    /// downstream physics or voxel handoffs can trust it.
    pub fn vertical_prism_from_region(
        region: &Region2,
        base_z: Real,
        height: Real,
        feature: BrepFeatureId,
        sources: Vec<BrepSourceVersion>,
    ) -> Self {
        let mut blockers = BTreeSet::new();
        let material_contour_count = region.material_contours().len();
        if material_contour_count == 0 {
            blockers.insert(BrepPlanarExtrusionConstructionBlocker::EmptyRegion);
        }
        if material_contour_count > 1 {
            blockers.insert(BrepPlanarExtrusionConstructionBlocker::MultipleMaterialContours);
        }
        match height.partial_cmp(&Real::from(0)) {
            Some(core::cmp::Ordering::Greater) => {}
            Some(_) => {
                blockers.insert(BrepPlanarExtrusionConstructionBlocker::NonPositiveHeight);
            }
            None => {
                blockers.insert(BrepPlanarExtrusionConstructionBlocker::UnknownHeightSign);
            }
        }

        let outer = region
            .material_contours()
            .first()
            .and_then(|contour| collect_line_contour_points(contour, &mut blockers));
        let mut loops = Vec::new();
        if let Some(points) = outer {
            loops.push(PrismSourceLoop {
                points,
                is_hole: false,
            });
        }
        for contour in region.hole_contours() {
            match collect_line_contour_points(contour, &mut blockers) {
                Some(points) => loops.push(PrismSourceLoop {
                    points,
                    is_hole: true,
                }),
                None => {
                    blockers.insert(BrepPlanarExtrusionConstructionBlocker::HoleContourInvalid);
                }
            }
        }
        let source_vertex_count = loops
            .iter()
            .map(|source_loop| source_loop.points.len())
            .sum();
        let shell = if blockers.is_empty() {
            Some(build_vertical_prism_shell(loops, base_z, height))
        } else {
            None
        };

        let (shell, manifest) = match shell {
            Some(shell) => {
                let solid = shell.solid_readiness_report(None);
                if !solid.exact_solid_boundary_ready {
                    blockers.insert(
                        BrepPlanarExtrusionConstructionBlocker::ConstructedSolidNotExactReady,
                    );
                }
                let manifest = BrepConstructionManifest::exact(
                    feature,
                    BrepConstructionKind::Extrusion,
                    sources,
                    &shell,
                );
                if !manifest.report(&shell).construction_fresh {
                    blockers.insert(
                        BrepPlanarExtrusionConstructionBlocker::ConstructionManifestNotFresh,
                    );
                }
                if blockers.is_empty() {
                    (Some(shell), Some(manifest))
                } else {
                    (None, None)
                }
            }
            None => (None, None),
        };

        let blockers = blockers.into_iter().collect::<Vec<_>>();
        Self {
            source_vertex_count,
            vertex_count: shell.as_ref().map_or(0, |shell| shell.vertices.len()),
            edge_count: shell.as_ref().map_or(0, |shell| shell.edges.len()),
            face_count: shell.as_ref().map_or(0, |shell| shell.faces.len()),
            exact_construction_ready: blockers.is_empty() && shell.is_some() && manifest.is_some(),
            shell,
            manifest,
            blockers,
        }
    }
}

fn collect_line_contour_points(
    contour: &Contour2,
    blockers: &mut BTreeSet<BrepPlanarExtrusionConstructionBlocker>,
) -> Option<Vec<hypercurve::Point2>> {
    if contour.is_empty() {
        blockers.insert(BrepPlanarExtrusionConstructionBlocker::EmptyContour);
        return None;
    }
    let mut points = Vec::with_capacity(contour.len());
    let mut previous_end = None;
    let mut first_start = None;
    for segment in contour.segments() {
        let Segment2::Line(line) = segment else {
            blockers.insert(BrepPlanarExtrusionConstructionBlocker::UnsupportedCurveSegment);
            return None;
        };
        if previous_end.as_ref().is_some_and(|end| end != line.start()) {
            blockers.insert(BrepPlanarExtrusionConstructionBlocker::BrokenContourChain);
        }
        first_start.get_or_insert_with(|| line.start().clone());
        previous_end = Some(line.end().clone());
        points.push(line.start().clone());
    }
    if first_start.as_ref() != previous_end.as_ref() {
        blockers.insert(BrepPlanarExtrusionConstructionBlocker::BrokenContourChain);
    }
    Some(points)
}

struct PrismSourceLoop {
    points: Vec<hypercurve::Point2>,
    is_hole: bool,
}

struct PrismBuiltLoop {
    bottom_edges: Vec<BrepEdgeId>,
    top_edges: Vec<BrepEdgeId>,
    vertical_edges: Vec<BrepEdgeId>,
    source: PrismSourceLoop,
}

fn build_vertical_prism_shell(
    source_loops: Vec<PrismSourceLoop>,
    base_z: Real,
    height: Real,
) -> BrepShell {
    let top_z = &base_z + &height;
    let total_vertex_count = source_loops
        .iter()
        .map(|source_loop| source_loop.points.len())
        .sum::<usize>();
    let mut vertices = Vec::with_capacity(total_vertex_count * 2);
    let mut edges = Vec::with_capacity(total_vertex_count * 3);
    let mut built_loops = Vec::with_capacity(source_loops.len());

    for source in source_loops {
        let count = source.points.len();
        let mut bottom_vertices = Vec::with_capacity(count);
        let mut top_vertices = Vec::with_capacity(count);
        for point in &source.points {
            let bottom = BrepVertexId(vertices.len() as u64);
            vertices.push(BrepVertex::new(
                bottom,
                Point3::new(point.x().clone(), point.y().clone(), base_z.clone()),
            ));
            bottom_vertices.push(bottom);
        }
        for point in &source.points {
            let top = BrepVertexId(vertices.len() as u64);
            vertices.push(BrepVertex::new(
                top,
                Point3::new(point.x().clone(), point.y().clone(), top_z.clone()),
            ));
            top_vertices.push(top);
        }

        let mut bottom_edges = Vec::with_capacity(count);
        let mut top_edges = Vec::with_capacity(count);
        let mut vertical_edges = Vec::with_capacity(count);
        for index in 0..count {
            let next = (index + 1) % count;
            let edge = BrepEdgeId(edges.len() as u64);
            edges.push(BrepEdge::new(
                edge,
                bottom_vertices[index],
                bottom_vertices[next],
            ));
            bottom_edges.push(edge);
        }
        for index in 0..count {
            let next = (index + 1) % count;
            let edge = BrepEdgeId(edges.len() as u64);
            edges.push(BrepEdge::new(edge, top_vertices[index], top_vertices[next]));
            top_edges.push(edge);
        }
        for index in 0..count {
            let edge = BrepEdgeId(edges.len() as u64);
            edges.push(BrepEdge::new(
                edge,
                bottom_vertices[index],
                top_vertices[index],
            ));
            vertical_edges.push(edge);
        }
        built_loops.push(PrismBuiltLoop {
            bottom_edges,
            top_edges,
            vertical_edges,
            source,
        });
    }

    let mut surfaces = Vec::with_capacity(total_vertex_count + 2);
    surfaces.push(BrepSurface::plane(
        BrepSurfaceId(0),
        Plane3::new(
            Point3::new(Real::from(0), Real::from(0), Real::from(-1)),
            base_z,
        ),
        BrepSurfaceSource::ExactConstruction,
    ));
    surfaces.push(BrepSurface::plane(
        BrepSurfaceId(1),
        Plane3::new(
            Point3::new(Real::from(0), Real::from(0), Real::from(1)),
            -top_z,
        ),
        BrepSurfaceSource::ExactConstruction,
    ));

    let outer = built_loops
        .iter()
        .find(|built_loop| !built_loop.source.is_hole)
        .expect("validated extrusion has one material loop");
    let bottom_outer = BrepLoop::new(
        BrepLoopId(0),
        outer
            .bottom_edges
            .iter()
            .rev()
            .map(|edge| BrepCoedge::new(*edge, BrepEdgeOrientation::Reversed))
            .collect(),
    );
    let bottom_inner = built_loops
        .iter()
        .filter(|built_loop| built_loop.source.is_hole)
        .enumerate()
        .map(|(index, built_loop)| {
            BrepLoop::new(
                BrepLoopId((2 + index) as u64),
                built_loop
                    .bottom_edges
                    .iter()
                    .map(|edge| BrepCoedge::new(*edge, BrepEdgeOrientation::Forward))
                    .collect(),
            )
        })
        .collect::<Vec<_>>();
    let top_outer = BrepLoop::new(
        BrepLoopId(1),
        outer
            .top_edges
            .iter()
            .map(|edge| BrepCoedge::new(*edge, BrepEdgeOrientation::Forward))
            .collect(),
    );
    let top_inner_offset = 2 + bottom_inner.len();
    let top_inner = built_loops
        .iter()
        .filter(|built_loop| built_loop.source.is_hole)
        .enumerate()
        .map(|(index, built_loop)| {
            BrepLoop::new(
                BrepLoopId((top_inner_offset + index) as u64),
                built_loop
                    .top_edges
                    .iter()
                    .rev()
                    .map(|edge| BrepCoedge::new(*edge, BrepEdgeOrientation::Reversed))
                    .collect(),
            )
        })
        .collect::<Vec<_>>();

    let mut faces = Vec::with_capacity(total_vertex_count + 2);
    faces.push(BrepFace::with_inner(
        BrepFaceId(0),
        BrepSurfaceId(0),
        bottom_outer,
        bottom_inner,
    ));
    faces.push(BrepFace::with_inner(
        BrepFaceId(1),
        BrepSurfaceId(1),
        top_outer,
        top_inner,
    ));

    let mut next_loop_id = (2 + 2 * built_loops
        .iter()
        .filter(|built_loop| built_loop.source.is_hole)
        .count()) as u64;
    for built_loop in &built_loops {
        let count = built_loop.source.points.len();
        for index in 0..count {
            let next = (index + 1) % count;
            let start = &built_loop.source.points[index];
            let end = &built_loop.source.points[next];
            let dx = end.x() - start.x();
            let dy = end.y() - start.y();
            let (normal_x, normal_y) = if built_loop.source.is_hole {
                (Real::from(0) - &dy, dx)
            } else {
                (dy, Real::from(0) - &dx)
            };
            let offset = Real::from(0) - &(&normal_x * start.x() + &normal_y * start.y());
            let surface = BrepSurfaceId(surfaces.len() as u64);
            surfaces.push(BrepSurface::plane(
                surface,
                Plane3::new(Point3::new(normal_x, normal_y, Real::from(0)), offset),
                BrepSurfaceSource::ExactConstruction,
            ));
            let loop_id = BrepLoopId(next_loop_id);
            next_loop_id += 1;
            let coedges = if built_loop.source.is_hole {
                vec![
                    BrepCoedge::new(
                        built_loop.vertical_edges[index],
                        BrepEdgeOrientation::Forward,
                    ),
                    BrepCoedge::new(built_loop.top_edges[index], BrepEdgeOrientation::Forward),
                    BrepCoedge::new(
                        built_loop.vertical_edges[next],
                        BrepEdgeOrientation::Reversed,
                    ),
                    BrepCoedge::new(
                        built_loop.bottom_edges[index],
                        BrepEdgeOrientation::Reversed,
                    ),
                ]
            } else {
                vec![
                    BrepCoedge::new(built_loop.bottom_edges[index], BrepEdgeOrientation::Forward),
                    BrepCoedge::new(
                        built_loop.vertical_edges[next],
                        BrepEdgeOrientation::Forward,
                    ),
                    BrepCoedge::new(built_loop.top_edges[index], BrepEdgeOrientation::Reversed),
                    BrepCoedge::new(
                        built_loop.vertical_edges[index],
                        BrepEdgeOrientation::Reversed,
                    ),
                ]
            };
            faces.push(BrepFace::new(
                BrepFaceId(faces.len() as u64),
                surface,
                BrepLoop::new(loop_id, coedges),
            ));
        }
    }

    BrepShell {
        vertices,
        edges,
        surfaces,
        faces,
    }
}

struct PlanarLoopBuilder<'a> {
    surface: &'a BrepSurface,
    vertices: &'a mut Vec<BrepVertex>,
    edges: &'a mut Vec<BrepEdge>,
    next_vertex_id: &'a mut u64,
    next_edge_id: &'a mut u64,
    next_loop_id: &'a mut u64,
    blockers: &'a mut BTreeSet<BrepPlanarRegionConstructionBlocker>,
}

impl PlanarLoopBuilder<'_> {
    fn build(&mut self, contour: &Contour2) -> Option<BrepLoop> {
        if contour.is_empty() {
            self.blockers
                .insert(BrepPlanarRegionConstructionBlocker::EmptyContour);
            return None;
        }

        let mut coedges = Vec::with_capacity(contour.len());
        let mut previous_end = None;
        let mut first_start = None;

        for segment in contour.segments() {
            let Segment2::Line(line) = segment else {
                self.blockers
                    .insert(BrepPlanarRegionConstructionBlocker::UnsupportedCurveSegment);
                return None;
            };
            if previous_end.as_ref().is_some_and(|end| end != line.start()) {
                self.blockers
                    .insert(BrepPlanarRegionConstructionBlocker::BrokenContourChain);
            }
            first_start.get_or_insert_with(|| line.start().clone());
            previous_end = Some(line.end().clone());

            let start = lift_or_insert_vertex(
                self.surface,
                line.start(),
                self.vertices,
                self.next_vertex_id,
                self.blockers,
            )?;
            let end = lift_or_insert_vertex(
                self.surface,
                line.end(),
                self.vertices,
                self.next_vertex_id,
                self.blockers,
            )?;
            let edge = BrepEdgeId(*self.next_edge_id);
            *self.next_edge_id += 1;
            self.edges.push(BrepEdge::new(edge, start, end));
            coedges.push(BrepCoedge::new(edge, BrepEdgeOrientation::Forward));
        }

        if first_start.as_ref() != previous_end.as_ref() {
            self.blockers
                .insert(BrepPlanarRegionConstructionBlocker::BrokenContourChain);
        }
        let face_loop = BrepLoop::new(BrepLoopId(*self.next_loop_id), coedges);
        *self.next_loop_id += 1;
        Some(face_loop)
    }
}

fn lift_or_insert_vertex(
    surface: &BrepSurface,
    point: &hypercurve::Point2,
    vertices: &mut Vec<BrepVertex>,
    next_vertex_id: &mut u64,
    blockers: &mut BTreeSet<BrepPlanarRegionConstructionBlocker>,
) -> Option<BrepVertexId> {
    let point = LimitPoint2::new(point.x().clone(), point.y().clone());
    let evaluated = surface.evaluate_frame_uv(point);
    let Some(point) = evaluated.point else {
        blockers.insert(BrepPlanarRegionConstructionBlocker::SurfaceEvaluationFailed);
        return None;
    };
    Some(insert_vertex(point, vertices, next_vertex_id))
}

fn insert_vertex(
    point: Point3,
    vertices: &mut Vec<BrepVertex>,
    next_vertex_id: &mut u64,
) -> BrepVertexId {
    if let Some(vertex) = vertices.iter().find(|vertex| vertex.point == point) {
        return vertex.id;
    }
    let id = BrepVertexId(*next_vertex_id);
    *next_vertex_id += 1;
    vertices.push(BrepVertex::new(id, point));
    id
}
