//! Public boolean operation entry points.

use std::sync::{Arc, OnceLock};

use hyperlattice::{
    HomogeneousPoint3, Point3, Rational, Real, Vector3, homogeneous_point_plane_expression,
};
use hyperreal::PreparedRationalLinearForm4Query;

use crate::error::HypermeshResult;
use crate::geometry::{Aabb, Classification, Plane, axis_mut, axis_ref, classify_point};
use crate::mesh::{
    MeshRef, prepare_input_with_certified_convex_inputs, prepare_input_with_deferred_edges,
};
use crate::output::{
    ARRANGEMENT_CLASSIFICATION, BooleanResult, ClassifiedPolygon, certify_output_polygon_closure,
};
use crate::polygon::{ConstructionEdgeIdentity, ConstructionPlaneIdentity, ConvexPolygon};
use crate::predicate::PreparedProjectivePoint3;
use crate::storage_hash::StorageHashMap;
use crate::subdivision::{SubdivisionConfig, SubdivisionTask};
use crate::winding::{BooleanOp, WindingPair, make_indicator};

const ALL_BOOLEAN_OPERATIONS: [BooleanOp; 4] = [
    BooleanOp::Union,
    BooleanOp::Intersection,
    BooleanOp::Difference,
    BooleanOp::SymmetricDifference,
];

/// A certified mesh arrangement that can be extracted for multiple Boolean
/// operations without repeating input preparation, intersection, BSP, or
/// winding classification work.
#[derive(Clone, Debug)]
pub struct BooleanArrangement {
    soup: crate::mesh::PolygonSoup,
    classified: Vec<crate::output::ClassifiedPolygon>,
    supported_operations: Vec<BooleanOp>,
    extraction_cache: Arc<ExtractionCache>,
    operation_scoped_triangle_extraction: bool,
    input_edges_deferred: bool,
}

#[derive(Debug, Default)]
struct ExtractionCache {
    results: [OnceLock<HypermeshResult<Arc<BooleanResult>>>; 4],
    triangle_soups: [OnceLock<HypermeshResult<Arc<crate::output::TriangleSoup>>>; 4],
}

struct PreparedConvexCandidate {
    classified: Vec<ClassifiedPolygon>,
    triangle_soups: Vec<(BooleanOp, Arc<crate::output::TriangleSoup>)>,
}

impl PartialEq for BooleanArrangement {
    fn eq(&self, other: &Self) -> bool {
        self.soup == other.soup
            && self.classified == other.classified
            && self.supported_operations == other.supported_operations
            && self.operation_scoped_triangle_extraction
                == other.operation_scoped_triangle_extraction
            && self.input_edges_deferred == other.input_edges_deferred
    }
}

impl BooleanArrangement {
    /// Returns the retained exact source-plane normal with output orientation.
    ///
    /// Triangle extraction keeps a global source index and an orientation for
    /// every emitted triangle. This accessor lets adapters reuse the support
    /// normal already constructed during exact input preparation instead of
    /// rebuilding the same cross product at an output boundary.
    pub fn oriented_source_normal(&self, source: crate::output::TriangleSource) -> Option<Vector3> {
        let index = usize::try_from(source.triangle).ok()?;
        let polygon = self.soup.polygons.get(index)?;
        if polygon.mesh_index != source.mesh || polygon.polygon_index != source.triangle {
            return None;
        }
        let normal = polygon.support.normal.to_vector();
        match source.orientation {
            1 => Some(normal),
            -1 => Some(-normal),
            _ => None,
        }
    }

    /// Extracts and closure-certifies one Boolean operation from this
    /// arrangement's stored front/back winding evidence.
    pub fn extract(&self, op: BooleanOp) -> HypermeshResult<BooleanResult> {
        self.cached_extract(op)
            .map(|result| result.as_ref().clone())
    }

    fn cached_extract(&self, op: BooleanOp) -> HypermeshResult<Arc<BooleanResult>> {
        self.extraction_cache.results[boolean_operation_index(op)]
            .get_or_init(|| self.extract_uncached(op).map(Arc::new))
            .clone()
    }

    fn extract_uncached(&self, op: BooleanOp) -> HypermeshResult<BooleanResult> {
        let result = self.select_result(op)?;
        certify_output_polygon_closure(&result)?;
        Ok(result)
    }

    fn select_result(&self, op: BooleanOp) -> HypermeshResult<BooleanResult> {
        if !self.supported_operations.contains(&op) {
            return Err(crate::error::HypermeshError::UnsupportedBooleanExtraction);
        }
        let indicator = make_indicator(op, self.soup.num_meshes);
        let mut selected = Vec::new();
        for polygon in &self.classified {
            let winding = polygon
                .winding()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let classification = crate::winding::classify_polygon_output(
                &winding.w_front,
                &winding.w_back,
                &indicator,
            );
            if classification != 0 {
                let mut polygon = polygon.clone();
                polygon.classification = classification;
                if self.input_edges_deferred {
                    polygon.polygon = polygon.polygon.with_rebuilt_edge_planes()?;
                }
                selected.push(polygon);
            }
        }
        Ok(BooleanResult::from_classified(self.soup.clone(), selected))
    }

    /// Extracts one Boolean operation directly as a closure-certified triangle
    /// soup.
    ///
    /// This preserves both polygon-arrangement and final triangle-soup
    /// certification while avoiding a redundant second polygon closure pass
    /// between the two stages.
    pub fn extract_triangle_soup(
        &self,
        op: BooleanOp,
    ) -> HypermeshResult<Arc<crate::output::TriangleSoup>> {
        self.extraction_cache.triangle_soups[boolean_operation_index(op)]
            .get_or_init(|| {
                if !self.supported_operations.contains(&op) {
                    return Err(crate::error::HypermeshError::UnsupportedBooleanExtraction);
                }
                if self.operation_scoped_triangle_extraction {
                    let selected =
                        select_classified_fragments(&self.classified, op, self.soup.num_meshes)?;
                    let arrangement =
                        crate::output::triangulate_classified_arrangement_precomputed_f64_scan(
                            &selected,
                        )?;
                    select_triangle_arrangement(&arrangement, op, self.soup.num_meshes)
                        .map(Arc::new)
                } else {
                    // Preserve the public extraction order of
                    // `triangulate_and_resolve_certified(extract(op))`.  A
                    // shared all-operation triangle arrangement contains
                    // vertices from fragments rejected by `op`, so selecting
                    // from it changes the indexed soup even when the exact
                    // boundary is equivalent.
                    let result = if let Some(result) =
                        self.extraction_cache.results[boolean_operation_index(op)].get()
                    {
                        result.clone()?
                    } else {
                        Arc::new(self.select_result(op)?)
                    };
                    crate::output::triangulate_and_resolve_polygon_certified(&result).map(Arc::new)
                }
            })
            .clone()
    }

    /// Returns the number of certified arrangement fragments retained for
    /// subsequent extraction.
    pub fn fragment_count(&self) -> usize {
        self.classified.len()
    }

    /// Returns whether this retained arrangement can extract `operation`.
    pub fn supports(&self, operation: BooleanOp) -> bool {
        self.supported_operations.contains(&operation)
    }
}

fn select_triangle_arrangement(
    arrangement: &crate::output::ClassifiedTriangleArrangement,
    op: BooleanOp,
    num_meshes: usize,
) -> HypermeshResult<crate::output::TriangleSoup> {
    if arrangement.soup.triangles.len() != arrangement.windings.len()
        || arrangement.soup.triangles.len() != arrangement.soup.sources.len()
    {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
    let indicator = make_indicator(op, num_meshes);
    let mut triangles = Vec::new();
    let mut sources = Vec::new();
    for ((triangle, source), winding) in arrangement
        .soup
        .triangles
        .iter()
        .zip(&arrangement.soup.sources)
        .zip(&arrangement.windings)
    {
        let classification =
            crate::winding::classify_polygon_output(&winding.w_front, &winding.w_back, &indicator);
        if classification == 0 {
            continue;
        }
        let mut triangle = *triangle;
        if classification == -1 {
            triangle.swap(1, 2);
        }
        let mut source = *source;
        source.orientation = classification;
        triangles.push(triangle);
        sources.push(source);
    }
    let soup = crate::output::TriangleSoup {
        vertices: arrangement.soup.vertices.clone(),
        triangles,
        sources,
    };
    certify_triangle_soup_closure(soup)
}

fn certify_triangle_soup_closure(
    soup: crate::output::TriangleSoup,
) -> HypermeshResult<crate::output::TriangleSoup> {
    let closure = crate::output::triangle_soup_closure_report(&soup);
    if !closure.has_no_boundary() {
        return Err(crate::error::HypermeshError::OpenOutput {
            boundary_edges: closure.boundary_edges,
            unbalanced_edges: closure.unbalanced_edges,
            non_manifold_edges: closure.non_manifold_edges,
        });
    }
    Ok(soup)
}

const fn boolean_operation_index(operation: BooleanOp) -> usize {
    match operation {
        BooleanOp::Union => 0,
        BooleanOp::Intersection => 1,
        BooleanOp::Difference => 2,
        BooleanOp::SymmetricDifference => 3,
    }
}

/// Configuration for boolean operations.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EmberConfig {
    /// Maximum recursive subdivision depth, or `usize::MAX` for no
    /// caller-selected limit.
    ///
    /// Reaching this bound is not treated as implicit success. If the current
    /// task has not certified as a complete leaf and an exact root-basis
    /// arrangement split remains, the operation fails with
    /// `HypermeshError::SubdivisionDepthLimit`.
    pub max_depth: usize,
}

impl Default for EmberConfig {
    fn default() -> Self {
        Self {
            max_depth: crate::subdivision::DEFAULT_MAX_DEPTH,
        }
    }
}

/// Performs a boolean operation on borrowed mesh views.
pub fn boolean_operation(
    meshes: &[MeshRef<'_>],
    op: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    crate::trace_dispatch!("boolean-operation", "start");
    let prepared = prepare_boolean_operations(meshes, &[op], config)?;
    crate::trace_dispatch!("boolean-operation", "certify-output-closure");
    let result = prepared.extract(op)?;
    crate::trace_dispatch!("boolean-operation", "complete");
    Ok(result)
}

/// Builds a certified arrangement once for extraction under multiple Boolean
/// operations.
///
/// This is the all-operation convenience form of
/// [`prepare_boolean_operations`]. [`boolean_operation`] uses the same prepared
/// pipeline with a one-operation scope, retaining its operation-specific
/// pruning without maintaining a separate execution path.
pub fn build_boolean_arrangement(
    meshes: &[MeshRef<'_>],
    config: EmberConfig,
) -> HypermeshResult<BooleanArrangement> {
    prepare_boolean_operations(meshes, &ALL_BOOLEAN_OPERATIONS, config)
}

/// Prepares a certified arrangement for exactly the requested Boolean
/// operations.
///
/// A single-operation preparation retains operation-specific winding
/// reachability pruning. Multi-operation preparation retains the transition
/// evidence needed to extract every requested result without repeating input
/// preparation, intersection, BSP, or winding classification work.
pub fn prepare_boolean_operations(
    meshes: &[MeshRef<'_>],
    operations: &[BooleanOp],
    config: EmberConfig,
) -> HypermeshResult<BooleanArrangement> {
    prepare_boolean_operations_with_certified_convex_inputs(
        meshes,
        operations,
        &vec![false; meshes.len()],
        config,
    )
}

/// Prepares Boolean operations while accepting exact convex-input
/// certificates supplied by the mesh owner.
///
/// A `true` entry certifies that the corresponding input is one closed,
/// non-self-intersecting, outward-oriented convex shell. Its triangulation
/// needs no self-arrangement cuts, its face-front winding is zero, and exact
/// support-plane tests may classify points against it. Cross-input
/// intersections and every output certification remain exact.
pub fn prepare_boolean_operations_with_certified_convex_inputs(
    meshes: &[MeshRef<'_>],
    operations: &[BooleanOp],
    certified_convex_inputs: &[bool],
    config: EmberConfig,
) -> HypermeshResult<BooleanArrangement> {
    if operations.is_empty() {
        return Err(crate::error::HypermeshError::EmptyBooleanOperationSet);
    }
    if certified_convex_inputs.len() != meshes.len() {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
    validate_mesh_refs(meshes)?;
    let supported_operations = ALL_BOOLEAN_OPERATIONS
        .into_iter()
        .filter(|operation| operations.contains(operation))
        .collect::<Vec<_>>();
    let use_two_convex_candidate = meshes.len() == 2 && certified_convex_inputs == [true, true];
    let mut soup = if use_two_convex_candidate {
        prepare_input_with_deferred_edges(meshes, certified_convex_inputs)?
    } else {
        prepare_input_with_certified_convex_inputs(meshes, certified_convex_inputs)?
    };
    let convex_candidate = if use_two_convex_candidate {
        prepare_two_convex_inputs_projectively(&soup.polygons, &supported_operations)
            .ok()
            .flatten()
    } else {
        None
    };
    let operation_scoped_triangle_extraction = convex_candidate.is_some();
    let (classified, triangle_soups, input_edges_deferred) =
        if let Some(candidate) = convex_candidate {
            (candidate.classified, candidate.triangle_soups, true)
        } else {
            if use_two_convex_candidate {
                soup = prepare_input_with_certified_convex_inputs(meshes, certified_convex_inputs)?;
            }
            let process_bounds = expanded_bounds(&soup.bounds);
            let ref_point = outside_reference_point(&process_bounds);
            let ref_wnv = vec![0; soup.num_meshes];
            (
                crate::subdivision::subdivide_prepared_with_certified_convex_inputs(
                    SubdivisionTask::new(
                        std::mem::take(&mut soup.polygons),
                        process_bounds,
                        ref_point,
                        ref_wnv,
                    ),
                    &supported_operations,
                    certified_convex_inputs,
                    SubdivisionConfig {
                        max_depth: config.max_depth,
                    },
                )?,
                Vec::new(),
                false,
            )
        };
    let extraction_cache = Arc::new(ExtractionCache::default());
    for (operation, triangle_soup) in triangle_soups {
        extraction_cache.triangle_soups[boolean_operation_index(operation)]
            .set(Ok(triangle_soup))
            .expect("fresh operation extraction cache is unset");
    }
    Ok(BooleanArrangement {
        soup,
        classified,
        supported_operations,
        extraction_cache,
        operation_scoped_triangle_extraction,
        input_edges_deferred,
    })
}

#[derive(Clone)]
struct ProjectiveCycle {
    points: Vec<HomogeneousPoint3>,
    edges: Vec<Plane>,
    edge_identities: Vec<ConstructionEdgeIdentity>,
    source_plane: ConstructionPlaneIdentity,
    source_unchanged: bool,
}

struct ProjectiveClip {
    negative: ProjectiveCycle,
    positive: ProjectiveCycle,
    side: ProjectiveClipSide,
}

#[derive(Default)]
struct ProjectiveAffineCache {
    points: StorageHashMap<[usize; 4], ProjectiveAffineCacheEntry>,
}

struct ProjectiveAffineCacheEntry {
    _coordinates: [Rational; 4],
    affine: Point3,
}

#[derive(Clone, Eq, Hash, PartialEq)]
enum ProjectiveVertexIdentity {
    SourceEdgePlane {
        mesh: usize,
        endpoints: [usize; 2],
        plane: ConstructionPlaneIdentity,
    },
    PlaneTriple {
        planes: [ConstructionPlaneIdentity; 3],
    },
}

#[derive(Default)]
struct ProjectivePointCache {
    points: StorageHashMap<ProjectiveVertexIdentity, HomogeneousPoint3>,
}

impl ConstructionEdgeIdentity {
    fn intersection_identity(&self, plane: ConstructionPlaneIdentity) -> ProjectiveVertexIdentity {
        match self {
            Self::Source { mesh, endpoints } => ProjectiveVertexIdentity::SourceEdgePlane {
                mesh: *mesh,
                endpoints: *endpoints,
                plane,
            },
            Self::Split { planes: existing } => {
                let mut planes = [existing[0], existing[1], plane];
                planes.sort_unstable();
                ProjectiveVertexIdentity::PlaneTriple { planes }
            }
        }
    }
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum ProjectiveClipSide {
    Negative,
    Positive,
    Both,
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum SourcePlaneRelation {
    Inside,
    Outside,
    Crossing,
}

#[derive(Clone, Copy, Eq, Hash, PartialEq)]
struct PointClassificationKey([usize; 3]);

#[derive(Default)]
struct PointPlaneClassificationCache {
    source_points: Vec<Option<CachedPointPlaneClassifications>>,
    points: StorageHashMap<PointClassificationKey, CachedPointPlaneClassifications>,
}

struct CachedPointPlaneClassifications {
    prepared_query: Option<PreparedRationalLinearForm4Query>,
    classifications: Vec<Option<Classification>>,
}

impl PointPlaneClassificationCache {
    fn source_relation(
        &mut self,
        polygon: &ConvexPolygon,
        plane: &Plane,
        plane_index: usize,
        plane_count: usize,
    ) -> HypermeshResult<SourcePlaneRelation> {
        let mut has_negative = false;
        let mut has_positive = false;
        let edge_identities = polygon.known_edge_identities.as_deref();
        for (point_index, point) in polygon
            .known_vertices
            .as_ref()
            .ok_or(crate::error::HypermeshError::UnknownClassification)?
            .iter()
            .enumerate()
        {
            let source_vertex =
                edge_identities.and_then(|identities| source_vertex_index(identities, point_index));
            match self.classify(point, source_vertex, plane, plane_index, plane_count)? {
                Classification::Negative => has_negative = true,
                Classification::Positive => has_positive = true,
                Classification::On => {}
            }
            if has_positive && has_negative {
                return Ok(SourcePlaneRelation::Crossing);
            }
        }
        Ok(if has_positive {
            SourcePlaneRelation::Outside
        } else {
            SourcePlaneRelation::Inside
        })
    }

    fn classify(
        &mut self,
        point: &Point3,
        source_vertex: Option<usize>,
        plane: &Plane,
        plane_index: usize,
        plane_count: usize,
    ) -> HypermeshResult<Classification> {
        let [Some(x), Some(y), Some(z)] = [
            point.x.exact_rational_ref(),
            point.y.exact_rational_ref(),
            point.z.exact_rational_ref(),
        ] else {
            return classify_point(point, plane);
        };
        let make_cached = || CachedPointPlaneClassifications {
            prepared_query: Real::prepare_rational_affine_point3_query([x, y, z]),
            classifications: vec![None; plane_count],
        };
        let cached = if let Some(source_vertex) = source_vertex {
            if self.source_points.len() <= source_vertex {
                self.source_points.resize_with(source_vertex + 1, || None);
            }
            self.source_points[source_vertex].get_or_insert_with(make_cached)
        } else {
            let key =
                PointClassificationKey([x, y, z].map(hyperlattice::Rational::storage_identity));
            self.points.entry(key).or_insert_with(make_cached)
        };
        if let Some(classification) = cached.classifications[plane_index] {
            return Ok(classification);
        }
        let classification = crate::predicate::classify_point_with_prepared_query(
            point,
            plane,
            cached.prepared_query.as_ref(),
        )?;
        cached.classifications[plane_index] = Some(classification);
        Ok(classification)
    }
}

fn source_vertex_index(
    edge_identities: &[ConstructionEdgeIdentity],
    point_index: usize,
) -> Option<usize> {
    let current = edge_identities.get(point_index)?;
    let previous_index = if point_index == 0 {
        edge_identities.len().checked_sub(1)?
    } else {
        point_index - 1
    };
    let previous = edge_identities.get(previous_index)?;
    let (
        ConstructionEdgeIdentity::Source {
            mesh: current_mesh,
            endpoints: current_endpoints,
        },
        ConstructionEdgeIdentity::Source {
            mesh: previous_mesh,
            endpoints: previous_endpoints,
        },
    ) = (current, previous)
    else {
        return None;
    };
    if current_mesh != previous_mesh {
        return None;
    }
    if previous_endpoints.contains(&current_endpoints[0]) {
        Some(current_endpoints[0])
    } else if previous_endpoints.contains(&current_endpoints[1]) {
        Some(current_endpoints[1])
    } else {
        None
    }
}

impl ProjectiveCycle {
    fn from_polygon(
        polygon: &ConvexPolygon,
        source_plane: ConstructionPlaneIdentity,
    ) -> HypermeshResult<Self> {
        let source_points = polygon
            .known_vertices
            .as_ref()
            .ok_or(crate::error::HypermeshError::UnknownClassification)?;
        let points = source_points
            .iter()
            .map(|point| {
                HomogeneousPoint3::new(
                    point.x.clone(),
                    point.y.clone(),
                    point.z.clone(),
                    Real::one(),
                )
            })
            .collect::<Vec<_>>();
        let edge_identities = polygon
            .known_edge_identities
            .as_ref()
            .ok_or(crate::error::HypermeshError::UnknownClassification)?
            .as_ref()
            .clone();
        if edge_identities.len() != points.len() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let edges = match polygon.edges.len() {
            len if len == points.len() => polygon.edges.as_ref().clone(),
            1 => vec![polygon.edges[0].clone(); points.len()],
            _ => return Err(crate::error::HypermeshError::UnknownClassification),
        };
        Ok(Self {
            points,
            edges,
            edge_identities,
            source_plane,
            source_unchanged: true,
        })
    }

    fn clip(
        &self,
        plane: &Plane,
        plane_identity: ConstructionPlaneIdentity,
        point_cache: &mut ProjectivePointCache,
    ) -> HypermeshResult<ProjectiveClip> {
        let evaluated = self
            .points
            .iter()
            .map(|point| projective_plane_value(point, plane))
            .collect::<HypermeshResult<Vec<_>>>()?;
        let has_negative = evaluated
            .iter()
            .any(|(_, classification)| classification.is_negative());
        let has_positive = evaluated
            .iter()
            .any(|(_, classification)| classification.is_positive());
        if !has_positive {
            return Ok(ProjectiveClip {
                negative: self.clone(),
                positive: Self::empty(),
                side: ProjectiveClipSide::Negative,
            });
        }
        if !has_negative {
            return Ok(ProjectiveClip {
                negative: Self::empty(),
                positive: self.clone(),
                side: ProjectiveClipSide::Positive,
            });
        }

        let inverted = plane.inverted();
        let mut negative = Vec::with_capacity(self.points.len() + 1);
        let mut negative_edges = Vec::with_capacity(self.edges.len() + 1);
        let mut negative_edge_identities = Vec::with_capacity(self.edge_identities.len() + 1);
        let mut positive = Vec::with_capacity(self.points.len() + 1);
        let mut positive_edges = Vec::with_capacity(self.edges.len() + 1);
        let mut positive_edge_identities = Vec::with_capacity(self.edge_identities.len() + 1);
        let mut split_planes = [self.source_plane, plane_identity];
        split_planes.sort_unstable();
        let split_identity = ConstructionEdgeIdentity::Split {
            planes: split_planes,
        };
        for index in 0..self.points.len() {
            let next = (index + 1) % self.points.len();
            let current_classification = evaluated[index].1;
            let next_classification = evaluated[next].1;
            let crossing = (current_classification.is_negative()
                && next_classification.is_positive())
                || (current_classification.is_positive() && next_classification.is_negative());
            let intersection = crossing.then(|| {
                self.cached_crossing_point(
                    index,
                    plane_identity,
                    &self.points[index],
                    &evaluated[index].0,
                    current_classification,
                    &self.points[next],
                    &evaluated[next].0,
                    point_cache,
                )
            });
            self.append_clipped_transition(
                index,
                current_classification,
                next_classification,
                intersection.as_ref(),
                plane,
                &split_identity,
                false,
                &mut negative,
                &mut negative_edges,
                &mut negative_edge_identities,
            );
            self.append_clipped_transition(
                index,
                current_classification,
                next_classification,
                intersection.as_ref(),
                &inverted,
                &split_identity,
                true,
                &mut positive,
                &mut positive_edges,
                &mut positive_edge_identities,
            );
        }
        remove_closing_labeled_duplicate(
            &mut negative,
            &mut negative_edges,
            &mut negative_edge_identities,
        );
        remove_closing_labeled_duplicate(
            &mut positive,
            &mut positive_edges,
            &mut positive_edge_identities,
        );
        Ok(ProjectiveClip {
            negative: Self {
                points: negative,
                edges: negative_edges,
                edge_identities: negative_edge_identities,
                source_plane: self.source_plane,
                source_unchanged: false,
            },
            positive: Self {
                points: positive,
                edges: positive_edges,
                edge_identities: positive_edge_identities,
                source_plane: self.source_plane,
                source_unchanged: false,
            },
            side: ProjectiveClipSide::Both,
        })
    }

    fn clip_negative(
        &self,
        plane: &Plane,
        plane_identity: ConstructionPlaneIdentity,
        point_cache: &mut ProjectivePointCache,
    ) -> HypermeshResult<Self> {
        let evaluated = self
            .points
            .iter()
            .map(|point| projective_plane_value(point, plane))
            .collect::<HypermeshResult<Vec<_>>>()?;
        let has_negative = evaluated
            .iter()
            .any(|(_, classification)| classification.is_negative());
        let has_positive = evaluated
            .iter()
            .any(|(_, classification)| classification.is_positive());
        if !has_positive {
            return Ok(self.clone());
        }
        if !has_negative {
            return Ok(Self::empty());
        }
        let mut points = Vec::with_capacity(self.points.len() + 1);
        let mut edges = Vec::with_capacity(self.edges.len() + 1);
        let mut edge_identities = Vec::with_capacity(self.edge_identities.len() + 1);
        let mut split_planes = [self.source_plane, plane_identity];
        split_planes.sort_unstable();
        let split_identity = ConstructionEdgeIdentity::Split {
            planes: split_planes,
        };
        for index in 0..self.points.len() {
            let next = (index + 1) % self.points.len();
            let current_classification = evaluated[index].1;
            let next_classification = evaluated[next].1;
            let crossing = (current_classification.is_negative()
                && next_classification.is_positive())
                || (current_classification.is_positive() && next_classification.is_negative());
            let intersection = crossing.then(|| {
                self.cached_crossing_point(
                    index,
                    plane_identity,
                    &self.points[index],
                    &evaluated[index].0,
                    current_classification,
                    &self.points[next],
                    &evaluated[next].0,
                    point_cache,
                )
            });
            self.append_clipped_transition(
                index,
                current_classification,
                next_classification,
                intersection.as_ref(),
                plane,
                &split_identity,
                false,
                &mut points,
                &mut edges,
                &mut edge_identities,
            );
        }
        remove_closing_labeled_duplicate(&mut points, &mut edges, &mut edge_identities);
        Ok(Self {
            points,
            edges,
            edge_identities,
            source_plane: self.source_plane,
            source_unchanged: false,
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn cached_crossing_point(
        &self,
        edge_index: usize,
        plane_identity: ConstructionPlaneIdentity,
        current: &HomogeneousPoint3,
        current_value: &Real,
        current_classification: Classification,
        next: &HomogeneousPoint3,
        next_value: &Real,
        point_cache: &mut ProjectivePointCache,
    ) -> HomogeneousPoint3 {
        let identity = self.edge_identities[edge_index].intersection_identity(plane_identity);
        if let Some(point) = point_cache.points.get(&identity) {
            return point.clone();
        }
        let point = projective_crossing_point(
            current,
            current_value,
            current_classification,
            next,
            next_value,
        );
        point_cache.points.insert(identity, point.clone());
        point
    }

    #[allow(clippy::too_many_arguments)]
    fn append_clipped_transition(
        &self,
        index: usize,
        current_classification: Classification,
        next_classification: Classification,
        intersection: Option<&HomogeneousPoint3>,
        split_edge: &Plane,
        split_identity: &ConstructionEdgeIdentity,
        positive: bool,
        points: &mut Vec<HomogeneousPoint3>,
        edges: &mut Vec<Plane>,
        edge_identities: &mut Vec<ConstructionEdgeIdentity>,
    ) {
        let current_inside = if positive {
            current_classification.is_non_negative()
        } else {
            current_classification.is_non_positive()
        };
        let next_inside = if positive {
            next_classification.is_non_negative()
        } else {
            next_classification.is_non_positive()
        };
        if current_inside && next_inside {
            push_labeled_projective(
                points,
                edges,
                edge_identities,
                self.points[index].clone(),
                self.edges[index].clone(),
                self.edge_identities[index].clone(),
            );
        } else if current_inside {
            if current_classification == Classification::On {
                push_labeled_projective(
                    points,
                    edges,
                    edge_identities,
                    self.points[index].clone(),
                    split_edge.clone(),
                    split_identity.clone(),
                );
            } else {
                push_labeled_projective(
                    points,
                    edges,
                    edge_identities,
                    self.points[index].clone(),
                    self.edges[index].clone(),
                    self.edge_identities[index].clone(),
                );
                push_labeled_projective(
                    points,
                    edges,
                    edge_identities,
                    intersection
                        .expect("strict side transition has an intersection")
                        .clone(),
                    split_edge.clone(),
                    split_identity.clone(),
                );
            }
        } else if next_inside && next_classification != Classification::On {
            push_labeled_projective(
                points,
                edges,
                edge_identities,
                intersection
                    .expect("strict side transition has an intersection")
                    .clone(),
                self.edges[index].clone(),
                self.edge_identities[index].clone(),
            );
        }
    }

    fn materialize(
        &self,
        source: &ConvexPolygon,
        affine_cache: &mut ProjectiveAffineCache,
    ) -> HypermeshResult<ConvexPolygon> {
        if self.source_unchanged {
            return Ok(source.clone());
        }
        let vertices = self
            .points
            .iter()
            .map(|point| affine_cache.resolve(point))
            .collect::<HypermeshResult<Vec<_>>>()?;
        Ok(source.with_known_vertex_cycle_and_edges(
            vertices,
            self.edges.clone(),
            self.edge_identities.clone(),
        ))
    }

    fn empty() -> Self {
        Self {
            points: Vec::new(),
            edges: Vec::new(),
            edge_identities: Vec::new(),
            source_plane: ConstructionPlaneIdentity {
                mesh: usize::MAX,
                plane: usize::MAX,
            },
            source_unchanged: false,
        }
    }
}

impl ProjectiveAffineCache {
    fn resolve(&mut self, point: &HomogeneousPoint3) -> HypermeshResult<Point3> {
        let coordinates = [
            point.x.exact_rational_ref(),
            point.y.exact_rational_ref(),
            point.z.exact_rational_ref(),
            point.w.exact_rational_ref(),
        ];
        if let [Some(x), Some(y), Some(z), Some(w)] = coordinates {
            let key = [x, y, z, w].map(Rational::storage_identity);
            if let Some(entry) = self.points.get(&key) {
                return Ok(entry.affine.clone());
            }
            let affine = affine_projective_point(point)?;
            self.points.insert(
                key,
                ProjectiveAffineCacheEntry {
                    _coordinates: [x.clone(), y.clone(), z.clone(), w.clone()],
                    affine: affine.clone(),
                },
            );
            return Ok(affine);
        }
        affine_projective_point(point)
    }
}

fn affine_projective_point(point: &HomogeneousPoint3) -> HypermeshResult<Point3> {
    point.to_affine_point().map_err(|_| {
        if point.w.definitely_zero() {
            crate::error::HypermeshError::PointAtInfinity
        } else {
            crate::error::HypermeshError::UnknownClassification
        }
    })
}

fn prepare_two_convex_inputs_projectively(
    polygons: &[ConvexPolygon],
    operations: &[BooleanOp],
) -> HypermeshResult<Option<PreparedConvexCandidate>> {
    let mut support_planes: [Vec<&Plane>; 2] = std::array::from_fn(|_| Vec::new());
    let mut storage_support_planes: [StorageHashMap<[usize; 4], usize>; 2] =
        std::array::from_fn(|_| StorageHashMap::default());
    let mut approximate_support_planes: [StorageHashMap<[u64; 4], Vec<usize>>; 2] =
        std::array::from_fn(|_| StorageHashMap::default());
    let mut non_exact_support_planes: [Vec<usize>; 2] = std::array::from_fn(|_| Vec::new());
    let mut support_plane_f64_values: [Vec<Option<[f64; 4]>>; 2] =
        std::array::from_fn(|_| Vec::new());
    let mut polygon_support_planes = Vec::with_capacity(polygons.len());
    for polygon in polygons {
        let mesh = usize::try_from(polygon.mesh_index)
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
        if mesh >= support_planes.len() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let storage_key = exact_plane_storage_key(&polygon.support);
        let plane = if let Some(index) =
            storage_key.and_then(|key| storage_support_planes[mesh].get(&key).copied())
        {
            index
        } else if let Some(values) = exact_plane_f64(&polygon.support) {
            let key = values.map(f64::to_bits);
            if let Some(index) = approximate_support_planes[mesh]
                .get(&key)
                .into_iter()
                .flatten()
                .copied()
                .find(|&index| support_planes[mesh][index] == &polygon.support)
            {
                index
            } else {
                let index = support_planes[mesh].len();
                support_planes[mesh].push(&polygon.support);
                support_plane_f64_values[mesh].push(Some(values));
                approximate_support_planes[mesh]
                    .entry(key)
                    .or_default()
                    .push(index);
                index
            }
        } else if let Some(index) = support_planes[mesh]
            .iter()
            .position(|plane| *plane == &polygon.support)
        {
            index
        } else {
            let index = support_planes[mesh].len();
            support_planes[mesh].push(&polygon.support);
            support_plane_f64_values[mesh].push(None);
            non_exact_support_planes[mesh].push(index);
            index
        };
        if let Some(key) = storage_key {
            storage_support_planes[mesh].insert(key, plane);
        }
        polygon_support_planes.push(ConstructionPlaneIdentity { mesh, plane });
    }
    let support_planes_f64 =
        support_plane_f64_values.map(|planes| planes.into_iter().collect::<Option<Vec<_>>>());

    let mut classified = Vec::new();
    let mut point_plane_caches: [PointPlaneClassificationCache; 2] =
        std::array::from_fn(|_| PointPlaneClassificationCache::default());
    let mut affine_cache = ProjectiveAffineCache::default();
    let mut projective_point_cache = ProjectivePointCache::default();
    for (polygon, source_plane) in polygons.iter().zip(polygon_support_planes) {
        let host = usize::try_from(polygon.mesh_index)
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
        let other = 1 - host;
        let emit_outside = projective_transition_is_emitted(host, other, false, operations);
        let emit_inside = projective_transition_is_emitted(host, other, true, operations);
        if !emit_outside && !emit_inside {
            continue;
        }
        let mut candidate_planes = Vec::new();
        let mut excluded = false;
        for (plane_index, &plane) in support_planes[other].iter().enumerate() {
            match point_plane_caches[host].source_relation(
                polygon,
                plane,
                plane_index,
                support_planes[other].len(),
            )? {
                SourcePlaneRelation::Inside => {}
                SourcePlaneRelation::Outside => {
                    excluded = true;
                    break;
                }
                SourcePlaneRelation::Crossing => candidate_planes.push(plane_index),
            }
        }
        if excluded {
            if emit_outside {
                push_source_transition(&mut classified, polygon, host, other, false)?;
            }
            continue;
        }
        if candidate_planes.is_empty() {
            if emit_inside {
                push_source_transition(&mut classified, polygon, host, other, true)?;
            }
            continue;
        }
        let source = ProjectiveCycle::from_polygon(polygon, source_plane)?;

        let active_result = exact_inside_and_active_planes(
            polygon,
            &source,
            &support_planes[other],
            support_planes_f64[other].as_deref(),
            &candidate_planes,
            other,
            &mut projective_point_cache,
        )?;
        let Some((inside, active_planes)) = active_result else {
            if emit_outside {
                push_projective_transition(
                    &mut classified,
                    &source,
                    polygon,
                    &mut affine_cache,
                    host,
                    other,
                    false,
                    operations,
                )?;
            }
            continue;
        };
        if !emit_outside {
            if emit_inside {
                push_projective_transition(
                    &mut classified,
                    &inside,
                    polygon,
                    &mut affine_cache,
                    host,
                    other,
                    true,
                    operations,
                )?;
            }
            continue;
        }
        let mut remainder = source;
        let mut has_inside = true;
        for plane_index in active_planes {
            let clipped = remainder.clip(
                support_planes[other][plane_index],
                ConstructionPlaneIdentity {
                    mesh: other,
                    plane: plane_index,
                },
                &mut projective_point_cache,
            )?;
            match clipped.side {
                ProjectiveClipSide::Negative => {
                    remainder = clipped.negative;
                }
                ProjectiveClipSide::Positive => {
                    push_projective_transition(
                        &mut classified,
                        &clipped.positive,
                        polygon,
                        &mut affine_cache,
                        host,
                        other,
                        false,
                        operations,
                    )?;
                    has_inside = false;
                    break;
                }
                ProjectiveClipSide::Both => {
                    push_projective_transition(
                        &mut classified,
                        &clipped.positive,
                        polygon,
                        &mut affine_cache,
                        host,
                        other,
                        false,
                        operations,
                    )?;
                    remainder = clipped.negative;
                }
            }
        }
        if has_inside && emit_inside {
            push_projective_transition(
                &mut classified,
                &remainder,
                polygon,
                &mut affine_cache,
                host,
                other,
                true,
                operations,
            )?;
        }
    }

    let triangle_soups = if let [operation] = operations {
        if *operation != BooleanOp::SymmetricDifference {
            let indicator = make_indicator(*operation, support_planes.len());
            for fragment in &mut classified {
                let winding = fragment
                    .winding()
                    .ok_or(crate::error::HypermeshError::UnknownClassification)?;
                fragment.classification = crate::winding::classify_polygon_output(
                    &winding.w_front,
                    &winding.w_back,
                    &indicator,
                );
            }
        }
        let soup = if matches!(operation, BooleanOp::Difference | BooleanOp::Intersection) {
            if classified
                .iter()
                .any(|fragment| !matches!(fragment.classification, -1 | 1))
            {
                return Ok(None);
            }
            crate::output::triangulate_preclassified_arrangement_construction_candidates(
                &classified,
                false,
            )
            .and_then(certify_triangle_soup_closure)
        } else if *operation == BooleanOp::Union {
            crate::output::triangulate_selected_preclassified_arrangement_construction_candidates(
                &classified,
                true,
            )
            .and_then(certify_triangle_soup_closure)
        } else {
            crate::output::triangulate_classified_arrangement_construction_candidates(
                &classified,
                true,
            )
            .and_then(|triangles| {
                select_triangle_arrangement(&triangles, *operation, support_planes.len())
            })
        }
        .or_else(|_| {
            crate::output::triangulate_classified_arrangement_precomputed_f64_scan(&classified)
                .and_then(|triangles| {
                    select_triangle_arrangement(&triangles, *operation, support_planes.len())
                })
        });
        let soup = match soup {
            Ok(soup) => soup,
            Err(_) => return Ok(None),
        };
        vec![(*operation, Arc::new(soup))]
    } else {
        let triangles = crate::output::triangulate_classified_arrangement_construction_candidates(
            &classified,
            true,
        )
        .and_then(|triangles| {
            for &operation in operations {
                select_triangle_arrangement(&triangles, operation, support_planes.len())?;
            }
            Ok(triangles)
        })
        .or_else(|_| {
            crate::output::triangulate_classified_arrangement_precomputed_f64_scan(&classified)
                .and_then(|triangles| {
                    for &operation in operations {
                        select_triangle_arrangement(&triangles, operation, support_planes.len())?;
                    }
                    Ok(triangles)
                })
        });
        match triangles {
            Ok(_) => {}
            Err(_) => return Ok(None),
        }
        Vec::new()
    };
    Ok(Some(PreparedConvexCandidate {
        classified,
        triangle_soups,
    }))
}

fn exact_plane_storage_key(plane: &Plane) -> Option<[usize; 4]> {
    let [Some(a), Some(b), Some(c), Some(d)] = [
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ]
    .map(Real::exact_rational_ref) else {
        return None;
    };
    Some([a, b, c, d].map(Rational::storage_identity))
}

fn exact_plane_f64(plane: &Plane) -> Option<[f64; 4]> {
    let coefficients = [
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ];
    if coefficients
        .iter()
        .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        return None;
    }
    let [Some(a), Some(b), Some(c), Some(d)] = coefficients.map(Real::to_f64_lossy) else {
        return None;
    };
    Some([a, b, c, d])
}

fn select_classified_fragments(
    classified: &[ClassifiedPolygon],
    operation: BooleanOp,
    num_meshes: usize,
) -> HypermeshResult<Vec<ClassifiedPolygon>> {
    let indicator = make_indicator(operation, num_meshes);
    let mut selected = Vec::new();
    for polygon in classified {
        let winding = polygon
            .winding()
            .ok_or(crate::error::HypermeshError::UnknownClassification)?;
        if crate::winding::classify_polygon_output(&winding.w_front, &winding.w_back, &indicator)
            != 0
        {
            selected.push(polygon.clone());
        }
    }
    Ok(selected)
}

fn exact_inside_and_active_planes(
    polygon: &ConvexPolygon,
    source: &ProjectiveCycle,
    support_planes: &[&Plane],
    support_planes_f64: Option<&[[f64; 4]]>,
    candidate_planes: &[usize],
    support_plane_mesh: usize,
    point_cache: &mut ProjectivePointCache,
) -> HypermeshResult<Option<(ProjectiveCycle, Vec<usize>)>> {
    if let Some(proposed_planes) = support_planes_f64
        .and_then(|planes| propose_active_planes_f64(polygon, planes, candidate_planes))
    {
        let inside = clip_inside_cycle(
            source,
            support_planes,
            &proposed_planes,
            support_plane_mesh,
            point_cache,
        )?;
        if inside.points.len() < 3 {
            return Ok(None);
        }
        if cycle_satisfies_planes(&inside, support_planes, candidate_planes)? {
            let active = active_cycle_planes(&inside, proposed_planes, support_plane_mesh);
            return Ok(Some((inside, active)));
        }
    }

    let inside = clip_inside_cycle(
        source,
        support_planes,
        candidate_planes,
        support_plane_mesh,
        point_cache,
    )?;
    if inside.points.len() < 3 {
        return Ok(None);
    }
    let active = active_cycle_planes(
        &inside,
        candidate_planes.iter().copied(),
        support_plane_mesh,
    );
    Ok(Some((inside, active)))
}

fn clip_inside_cycle(
    source: &ProjectiveCycle,
    support_planes: &[&Plane],
    plane_indices: &[usize],
    support_plane_mesh: usize,
    point_cache: &mut ProjectivePointCache,
) -> HypermeshResult<ProjectiveCycle> {
    let mut inside = source.clone();
    for &plane_index in plane_indices {
        inside = inside.clip_negative(
            support_planes[plane_index],
            ConstructionPlaneIdentity {
                mesh: support_plane_mesh,
                plane: plane_index,
            },
            point_cache,
        )?;
        if inside.points.len() < 3 {
            return Ok(ProjectiveCycle::empty());
        }
    }
    Ok(inside)
}

fn active_cycle_planes(
    inside: &ProjectiveCycle,
    plane_indices: impl IntoIterator<Item = usize>,
    support_plane_mesh: usize,
) -> Vec<usize> {
    plane_indices
        .into_iter()
        .filter(|&plane_index| {
            let identity = ConstructionPlaneIdentity {
                mesh: support_plane_mesh,
                plane: plane_index,
            };
            inside.edge_identities.iter().any(|edge| {
                matches!(
                    edge,
                    ConstructionEdgeIdentity::Split { planes }
                        if planes.contains(&identity)
                )
            })
        })
        .collect()
}

fn cycle_satisfies_planes(
    cycle: &ProjectiveCycle,
    support_planes: &[&Plane],
    plane_indices: &[usize],
) -> HypermeshResult<bool> {
    for point in &cycle.points {
        let prepared = PreparedProjectivePoint3::new(point);
        for &plane_index in plane_indices {
            if prepared
                .classify(support_planes[plane_index])?
                .is_positive()
            {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

fn propose_active_planes_f64(
    polygon: &ConvexPolygon,
    planes: &[[f64; 4]],
    candidate_planes: &[usize],
) -> Option<Vec<usize>> {
    let mut cycle = polygon
        .known_vertices
        .as_ref()?
        .iter()
        .map(|point| {
            Some([
                point.x.to_f64_lossy()?,
                point.y.to_f64_lossy()?,
                point.z.to_f64_lossy()?,
            ])
        })
        .collect::<Option<Vec<_>>>()?;
    for &plane_index in candidate_planes {
        cycle = clip_f64_cycle(&cycle, planes[plane_index]);
        if cycle.len() < 3 {
            return Some(Vec::new());
        }
    }
    let mut active = Vec::new();
    for &plane_index in candidate_planes {
        let plane = planes[plane_index];
        let points_on_plane = cycle
            .iter()
            .filter(|point| {
                let value = f64_plane_value(**point, plane);
                let scale = (plane[0] * point[0]).abs()
                    + (plane[1] * point[1]).abs()
                    + (plane[2] * point[2]).abs()
                    + plane[3].abs();
                value.abs() <= 1.0e-8 * scale.max(1.0)
            })
            .take(2)
            .count();
        if points_on_plane == 2 {
            active.push(plane_index);
        }
    }
    Some(active)
}

fn clip_f64_cycle(points: &[[f64; 3]], plane: [f64; 4]) -> Vec<[f64; 3]> {
    let mut clipped = Vec::with_capacity(points.len() + 1);
    for index in 0..points.len() {
        let next = (index + 1) % points.len();
        let current_value = f64_plane_value(points[index], plane);
        let next_value = f64_plane_value(points[next], plane);
        let current_inside = current_value <= 0.0;
        let next_inside = next_value <= 0.0;
        match (current_inside, next_inside) {
            (true, true) => clipped.push(points[next]),
            (true, false) => clipped.push(f64_segment_plane_intersection(
                points[index],
                points[next],
                current_value,
                next_value,
            )),
            (false, true) => {
                clipped.push(f64_segment_plane_intersection(
                    points[index],
                    points[next],
                    current_value,
                    next_value,
                ));
                clipped.push(points[next]);
            }
            (false, false) => {}
        }
    }
    clipped
}

fn f64_segment_plane_intersection(
    start: [f64; 3],
    end: [f64; 3],
    start_value: f64,
    end_value: f64,
) -> [f64; 3] {
    let parameter = start_value / (start_value - end_value);
    std::array::from_fn(|axis| start[axis] + parameter * (end[axis] - start[axis]))
}

fn f64_plane_value(point: [f64; 3], plane: [f64; 4]) -> f64 {
    plane[0].mul_add(
        point[0],
        plane[1].mul_add(point[1], plane[2].mul_add(point[2], plane[3])),
    )
}

fn projective_plane_value(
    point: &HomogeneousPoint3,
    plane: &Plane,
) -> HypermeshResult<(Real, Classification)> {
    let value = homogeneous_point_plane_expression(point, plane);
    let classification = crate::predicate::classify_real(&value)?;
    Ok((value, classification))
}

fn projective_crossing_point(
    current: &HomogeneousPoint3,
    current_value: &Real,
    current_classification: Classification,
    next: &HomogeneousPoint3,
    next_value: &Real,
) -> HomogeneousPoint3 {
    let (negative, negative_value, positive, positive_value) =
        if current_classification.is_negative() {
            (current, current_value, next, next_value)
        } else {
            (next, next_value, current, current_value)
        };
    let coordinate = |negative_coordinate: &Real, positive_coordinate: &Real| {
        Real::signed_product_sum(
            [true, false],
            [
                [positive_value, negative_coordinate],
                [negative_value, positive_coordinate],
            ],
        )
    };
    HomogeneousPoint3::new(
        coordinate(&negative.x, &positive.x),
        coordinate(&negative.y, &positive.y),
        coordinate(&negative.z, &positive.z),
        coordinate(&negative.w, &positive.w),
    )
}

fn push_labeled_projective(
    points: &mut Vec<HomogeneousPoint3>,
    edges: &mut Vec<Plane>,
    edge_identities: &mut Vec<ConstructionEdgeIdentity>,
    point: HomogeneousPoint3,
    edge: Plane,
    edge_identity: ConstructionEdgeIdentity,
) {
    if points.last() == Some(&point) {
        if let Some(last_edge) = edges.last_mut() {
            *last_edge = edge;
        }
        if let Some(last_identity) = edge_identities.last_mut() {
            *last_identity = edge_identity;
        }
        return;
    }
    points.push(point);
    edges.push(edge);
    edge_identities.push(edge_identity);
}

fn remove_closing_labeled_duplicate(
    points: &mut Vec<HomogeneousPoint3>,
    edges: &mut Vec<Plane>,
    edge_identities: &mut Vec<ConstructionEdgeIdentity>,
) {
    if points.len() > 1 && points.first() == points.last() {
        points.pop();
        edges.pop();
        edge_identities.pop();
    }
}

fn push_projective_transition(
    classified: &mut Vec<ClassifiedPolygon>,
    cycle: &ProjectiveCycle,
    source: &ConvexPolygon,
    affine_cache: &mut ProjectiveAffineCache,
    host: usize,
    other: usize,
    inside_other: bool,
    operations: &[BooleanOp],
) -> HypermeshResult<()> {
    if cycle.points.len() < 3 {
        return Ok(());
    }
    let winding = projective_transition_winding(host, other, inside_other);
    if !projective_transition_is_emitted(host, other, inside_other, operations) {
        return Ok(());
    }
    let polygon = cycle.materialize(source, affine_cache)?;
    let mut fragment = ClassifiedPolygon::new(polygon, ARRANGEMENT_CLASSIFICATION);
    fragment.winding = Some(winding);
    fragment.is_bsp_fragment = true;
    classified.push(fragment);
    Ok(())
}

fn push_source_transition(
    classified: &mut Vec<ClassifiedPolygon>,
    source: &ConvexPolygon,
    host: usize,
    other: usize,
    inside_other: bool,
) -> HypermeshResult<()> {
    if source.vertex_count() < 3 {
        return Ok(());
    }
    let mut fragment = ClassifiedPolygon::new(source.clone(), ARRANGEMENT_CLASSIFICATION);
    fragment.winding = Some(projective_transition_winding(host, other, inside_other));
    fragment.is_bsp_fragment = true;
    classified.push(fragment);
    Ok(())
}

fn projective_transition_winding(host: usize, other: usize, inside_other: bool) -> WindingPair {
    let mut w_front = vec![0; 2];
    w_front[other] = i32::from(inside_other);
    let mut w_back = w_front.clone();
    w_back[host] = 1;
    WindingPair { w_front, w_back }
}

fn projective_transition_is_emitted(
    host: usize,
    _other: usize,
    inside_other: bool,
    operations: &[BooleanOp],
) -> bool {
    operations.iter().copied().any(|operation| match operation {
        BooleanOp::Union => !inside_other,
        BooleanOp::Intersection => inside_other,
        BooleanOp::Difference => (host == 0 && !inside_other) || (host == 1 && inside_other),
        BooleanOp::SymmetricDifference => true,
    })
}

fn validate_mesh_refs(meshes: &[MeshRef<'_>]) -> HypermeshResult<()> {
    if meshes.is_empty() {
        return Err(crate::error::HypermeshError::EmptyInput);
    }

    for (mesh_index, mesh) in meshes.iter().enumerate() {
        if mesh.positions.is_empty() || mesh.triangles.is_empty() {
            return Err(crate::error::HypermeshError::EmptyMesh { mesh_index });
        }
        for triangle in mesh.triangles {
            for index in triangle.indices() {
                if index >= mesh.positions.len() {
                    return Err(crate::error::HypermeshError::VertexIndexOutOfBounds {
                        index,
                        vertex_count: mesh.positions.len(),
                    });
                }
            }
        }
    }

    Ok(())
}

/// Union convenience wrapper.
pub fn boolean_union(
    a: MeshRef<'_>,
    b: MeshRef<'_>,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_operation(&[a, b], BooleanOp::Union, config)
}

/// Intersection convenience wrapper.
pub fn boolean_intersection(
    a: MeshRef<'_>,
    b: MeshRef<'_>,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_operation(&[a, b], BooleanOp::Intersection, config)
}

/// Difference convenience wrapper.
pub fn boolean_difference(
    a: MeshRef<'_>,
    b: MeshRef<'_>,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_operation(&[a, b], BooleanOp::Difference, config)
}

fn expanded_bounds(bounds: &Aabb) -> Aabb {
    let one = Real::one();
    Aabb::new(
        Point3::new(
            &bounds.min.x - &one,
            &bounds.min.y - &one,
            &bounds.min.z - &one,
        ),
        Point3::new(
            &bounds.max.x + &one,
            &bounds.max.y + &one,
            &bounds.max.z + &one,
        ),
    )
}

fn outside_reference_point(bounds: &Aabb) -> Point3 {
    let one = Real::one();
    let mut point = Point3::new(bounds.midpoint(0), bounds.midpoint(1), bounds.midpoint(2));
    *axis_mut(&mut point, 0) = axis_ref(&bounds.min, 0) - &one;
    point
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn outside_reference_point_uses_exterior_face_center() {
        let bounds = Aabb::new(p(0, 2, 4), p(10, 8, 14));
        let point = outside_reference_point(&bounds);

        assert_eq!(point, p(-1, 5, 9));
    }

    #[test]
    fn default_config_uses_finite_split_basis_without_a_depth_budget() {
        assert_eq!(EmberConfig::default().max_depth, usize::MAX);
    }

    #[test]
    fn projective_cycle_expands_deferred_source_edges_on_demand() {
        let polygon = crate::polygon::make_triangle_with_deferred_edges(
            &p(0, 0, 0),
            &p(1, 0, 0),
            &p(0, 1, 0),
            0,
            0,
        )
        .with_source_triangle_edge_identities(0, [0, 1, 2]);
        assert_eq!(polygon.edges.len(), 1);
        assert_eq!(polygon.vertex_count(), 3);

        let cycle = ProjectiveCycle::from_polygon(
            &polygon,
            ConstructionPlaneIdentity { mesh: 0, plane: 0 },
        )
        .unwrap();
        assert_eq!(cycle.edges.len(), 3);
        assert!(cycle.edges.iter().all(|edge| edge == &polygon.support));
    }

    #[test]
    fn source_relation_stops_after_exact_crossing_is_certified() {
        let polygon = crate::polygon::make_triangle(&p(0, 0, 1), &p(0, 0, -1), &p(1, 0, 0), 0, 0);
        let plane = Plane::axis_aligned(2, Real::zero());
        let mut cache = PointPlaneClassificationCache::default();

        assert!(matches!(
            cache.source_relation(&polygon, &plane, 0, 1).unwrap(),
            SourcePlaneRelation::Crossing
        ));
        assert_eq!(cache.points.len(), 2);
    }

    #[test]
    fn source_relation_indexes_certified_source_vertices_without_coordinate_hashing() {
        let polygon = crate::polygon::make_triangle(&p(0, 0, 1), &p(0, 0, -1), &p(1, 0, 0), 0, 0)
            .with_source_triangle_edge_identities(0, [7, 9, 11]);
        let plane = Plane::axis_aligned(2, Real::zero());
        let mut cache = PointPlaneClassificationCache::default();

        assert!(matches!(
            cache.source_relation(&polygon, &plane, 0, 1).unwrap(),
            SourcePlaneRelation::Crossing
        ));
        assert!(cache.points.is_empty());
        assert_eq!(
            cache
                .source_points
                .iter()
                .filter(|cached| cached.is_some())
                .count(),
            2
        );
    }
}
