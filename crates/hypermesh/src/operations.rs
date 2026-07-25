//! Public boolean operation entry points.

use hyperlattice::{
    HomogeneousPoint3, Point3, Rational, Real, homogeneous_point_plane_expression,
    intersect_three_planes,
};
use hyperreal::PreparedRationalLinearForm4Query;

use crate::error::HypermeshResult;
use crate::geometry::{Aabb, Classification, Plane, axis_mut, axis_ref, classify_point};
use crate::mesh::{
    MeshRef, build_polygon_soup_with_certified_convex_inputs,
    build_polygon_soup_with_deferred_edges,
};
use crate::output::{
    ARRANGEMENT_CLASSIFICATION, BooleanResult, ClassifiedPolygon, certify_output_polygon_closure,
};
use crate::polygon::{
    ConstructionEdgeIdentity, ConstructionPlaneIdentity, ConstructionVertexIdentity, ConvexPolygon,
    InputTrianglePlanes,
};
use crate::predicate::PreparedProjectivePoint3;
use crate::storage_hash::StorageHashMap;
use crate::subdivision::{SubdivisionConfig, SubdivisionTask};
use crate::winding::{BooleanOp, WindingPair, make_indicator};

struct BooleanComputation {
    soup: crate::mesh::PolygonSoup,
    classified: Vec<crate::output::ClassifiedPolygon>,
    triangle_soup: Option<crate::output::TriangleSoup>,
    input_edges_deferred: bool,
}

struct ConvexCandidate {
    classified: Vec<ClassifiedPolygon>,
    triangle_soup: crate::output::TriangleSoup,
}

impl BooleanComputation {
    fn into_result(self, operation: BooleanOp) -> HypermeshResult<BooleanResult> {
        let result = self.into_selected_result(operation)?;
        certify_output_polygon_closure(&result)?;
        Ok(result)
    }

    fn into_selected_result(self, operation: BooleanOp) -> HypermeshResult<BooleanResult> {
        let indicator = make_indicator(operation, self.soup.num_meshes);
        let mut selected = Vec::with_capacity(self.classified.len());
        for mut polygon in self.classified {
            let winding = polygon
                .winding()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let classification = crate::winding::classify_polygon_output(
                &winding.w_front,
                &winding.w_back,
                &indicator,
            );
            if classification != 0 {
                polygon.classification = classification;
                if self.input_edges_deferred {
                    polygon.polygon = polygon.polygon.with_rebuilt_edge_planes()?;
                }
                selected.push(polygon);
            }
        }
        Ok(BooleanResult::from_classified(self.soup, selected))
    }

    fn into_triangle_soup(
        self,
        operation: BooleanOp,
    ) -> HypermeshResult<crate::output::TriangleSoup> {
        if let Some(soup) = self.triangle_soup {
            return Ok(soup);
        }
        let result = self.into_selected_result(operation)?;
        crate::output::triangulate_and_resolve_polygon_certified(&result)
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
    let closure = crate::output::triangle_soup_closure_evidence(&soup);
    if !closure.has_no_boundary() {
        return Err(crate::error::HypermeshError::OpenOutput {
            boundary_edges: closure.boundary_edges,
            unbalanced_edges: closure.unbalanced_edges,
            non_manifold_edges: closure.non_manifold_edges,
        });
    }
    Ok(soup)
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
    operation: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    crate::trace_dispatch!("boolean-operation", "start");
    let computation = compute_boolean(meshes, operation, None, None, config)?;
    crate::trace_dispatch!("boolean-operation", "certify-output-closure");
    let result = computation.into_result(operation)?;
    crate::trace_dispatch!("boolean-operation", "complete");
    Ok(result)
}

/// Performs a Boolean operation using exact convex-input facts supplied by
/// the mesh owner.
///
/// A `true` entry certifies that the corresponding input is one closed,
/// non-self-intersecting, outward-oriented convex shell. Cross-input
/// intersections and output closure remain exactly certified.
pub fn boolean_operation_with_certified_convex_inputs(
    meshes: &[MeshRef<'_>],
    operation: BooleanOp,
    certified_convex_inputs: &[bool],
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    crate::trace_dispatch!("boolean-operation", "start");
    let computation = compute_boolean(
        meshes,
        operation,
        Some(certified_convex_inputs),
        None,
        config,
    )?;
    crate::trace_dispatch!("boolean-operation", "certify-output-closure");
    let result = computation.into_result(operation)?;
    crate::trace_dispatch!("boolean-operation", "complete");
    Ok(result)
}

/// Performs a Boolean operation and immediately returns a closure-certified
/// triangle soup.
///
/// This avoids materializing an intermediate polygon result when the caller
/// needs indexed triangles.
pub fn boolean_triangle_soup(
    meshes: &[MeshRef<'_>],
    operation: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<crate::output::TriangleSoup> {
    crate::trace_dispatch!("boolean-operation", "start");
    let computation = compute_boolean(meshes, operation, None, None, config)?;
    crate::trace_dispatch!("boolean-operation", "triangulate-output");
    let soup = computation.into_triangle_soup(operation)?;
    crate::trace_dispatch!("boolean-operation", "complete");
    Ok(soup)
}

/// Performs a Boolean operation with exact convex-input facts and immediately
/// returns a closure-certified triangle soup.
///
/// This is the direct triangle-output counterpart of
/// [`boolean_operation_with_certified_convex_inputs`].
pub fn boolean_triangle_soup_with_certified_convex_inputs(
    meshes: &[MeshRef<'_>],
    operation: BooleanOp,
    certified_convex_inputs: &[bool],
    config: EmberConfig,
) -> HypermeshResult<crate::output::TriangleSoup> {
    crate::trace_dispatch!("boolean-operation", "start");
    let computation = compute_boolean(
        meshes,
        operation,
        Some(certified_convex_inputs),
        None,
        config,
    )?;
    crate::trace_dispatch!("boolean-operation", "triangulate-output");
    let soup = computation.into_triangle_soup(operation)?;
    crate::trace_dispatch!("boolean-operation", "complete");
    Ok(soup)
}

/// Performs a Boolean operation with exact convex-input facts and exact
/// per-triangle plane certificates.
///
/// Each plane slice must align with the corresponding mesh's triangle slice.
/// The supplied oriented support and boundary planes are used instead of
/// reconstructing the same objects from transformed vertex coordinates,
/// preserving affine-transform identities at the geometric-object boundary.
pub fn boolean_triangle_soup_with_certified_convex_inputs_and_planes(
    meshes: &[MeshRef<'_>],
    operation: BooleanOp,
    certified_convex_inputs: &[bool],
    input_planes: &[&[InputTrianglePlanes]],
    config: EmberConfig,
) -> HypermeshResult<crate::output::TriangleSoup> {
    crate::trace_dispatch!("boolean-operation", "start");
    let computation = compute_boolean(
        meshes,
        operation,
        Some(certified_convex_inputs),
        Some(input_planes),
        config,
    )?;
    crate::trace_dispatch!("boolean-operation", "triangulate-output");
    let soup = computation.into_triangle_soup(operation)?;
    crate::trace_dispatch!("boolean-operation", "complete");
    Ok(soup)
}

fn compute_boolean(
    meshes: &[MeshRef<'_>],
    operation: BooleanOp,
    certified_convex_inputs: Option<&[bool]>,
    input_planes: Option<&[&[InputTrianglePlanes]]>,
    config: EmberConfig,
) -> HypermeshResult<BooleanComputation> {
    if certified_convex_inputs.is_some_and(|certified| certified.len() != meshes.len()) {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
    let certified_convex_inputs = certified_convex_inputs.unwrap_or(&[]);
    let use_two_convex_candidate = meshes.len() == 2 && certified_convex_inputs == [true, true];
    let mut soup = if use_two_convex_candidate {
        build_polygon_soup_with_deferred_edges(meshes, certified_convex_inputs, input_planes)?
    } else if certified_convex_inputs.is_empty() {
        crate::mesh::build_polygon_soup(meshes)?
    } else {
        build_polygon_soup_with_certified_convex_inputs(
            meshes,
            certified_convex_inputs,
            input_planes,
        )?
    };
    let convex_candidate = if use_two_convex_candidate {
        match compute_two_convex_inputs_projectively(&soup.polygons, operation) {
            Ok(candidate) => candidate,
            Err(error) => {
                if cfg!(debug_assertions) {
                    eprintln!("[DEBUG] projective convex candidate failed: {error}");
                }
                None
            }
        }
    } else {
        None
    };
    let (classified, triangle_soup, input_edges_deferred) =
        if let Some(candidate) = convex_candidate {
            (candidate.classified, Some(candidate.triangle_soup), true)
        } else {
            if use_two_convex_candidate {
                soup = build_polygon_soup_with_certified_convex_inputs(
                    meshes,
                    certified_convex_inputs,
                    input_planes,
                )?;
            }
            let process_bounds = expanded_bounds(&soup.bounds);
            let ref_point = outside_reference_point(&process_bounds);
            let ref_wnv = vec![0; soup.num_meshes];
            (
                crate::subdivision::subdivide_boolean_with_certified_convex_inputs(
                    SubdivisionTask::new(
                        std::mem::take(&mut soup.polygons),
                        process_bounds,
                        ref_point,
                        ref_wnv,
                    ),
                    operation,
                    certified_convex_inputs,
                    SubdivisionConfig {
                        max_depth: config.max_depth,
                    },
                )?,
                None,
                false,
            )
        };
    Ok(BooleanComputation {
        soup,
        classified,
        triangle_soup,
        input_edges_deferred,
    })
}

#[derive(Clone)]
struct ProjectiveCycle {
    points: Vec<HomogeneousPoint3>,
    point_identities: Vec<ConstructionVertexIdentity>,
    edges: Vec<Plane>,
    edge_identities: Vec<ConstructionEdgeIdentity>,
    support: Plane,
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
    identities: StorageHashMap<ConstructionVertexIdentity, Point3>,
}

struct ProjectiveAffineCacheEntry {
    _coordinates: [Rational; 4],
    affine: Point3,
}

#[derive(Default)]
struct ProjectivePointCache {
    points: StorageHashMap<ConstructionVertexIdentity, HomogeneousPoint3>,
    canonical_identities: StorageHashMap<ConstructionVertexIdentity, ConstructionVertexIdentity>,
    canonical_planes: StorageHashMap<ConstructionPlaneIdentity, ConstructionPlaneIdentity>,
    planes: StorageHashMap<ConstructionPlaneIdentity, Plane>,
    source_edges: StorageHashMap<ConstructionEdgeIdentity, [Plane; 2]>,
    source_edge_supports: StorageHashMap<ConstructionEdgeIdentity, Vec<ConstructionPlaneIdentity>>,
    source_vertices: StorageHashMap<ConstructionVertexIdentity, [Plane; 3]>,
    point_incidences: StorageHashMap<ConstructionVertexIdentity, Vec<ConstructionPlaneIdentity>>,
}

struct AtomicDisjointSets {
    parents: Vec<usize>,
}

impl AtomicDisjointSets {
    fn new(len: usize) -> Self {
        Self {
            parents: (0..len).collect(),
        }
    }

    fn representative(&mut self, mut index: usize) -> usize {
        while self.parents[index] != index {
            let parent = self.parents[index];
            self.parents[index] = self.parents[parent];
            index = self.parents[index];
        }
        index
    }

    fn merge(&mut self, left: usize, right: usize) {
        let left = self.representative(left);
        let right = self.representative(right);
        if left == right {
            return;
        }
        let (representative, merged) = if left < right {
            (left, right)
        } else {
            (right, left)
        };
        self.parents[merged] = representative;
    }
}

impl ConstructionEdgeIdentity {
    fn intersection_identity(
        &self,
        plane: ConstructionPlaneIdentity,
    ) -> ConstructionVertexIdentity {
        match self {
            Self::Source { mesh, endpoints } => ConstructionVertexIdentity::SourceEdgePlane {
                mesh: *mesh,
                endpoints: *endpoints,
                plane,
            },
            Self::Split { planes: existing } => {
                let mut planes = [existing[0], existing[1], plane];
                planes.sort_unstable();
                ConstructionVertexIdentity::PlaneTriple { planes }
            }
        }
    }
}

impl ProjectivePointCache {
    fn canonical_plane_identity(
        &self,
        identity: ConstructionPlaneIdentity,
    ) -> ConstructionPlaneIdentity {
        self.canonical_planes
            .get(&identity)
            .copied()
            .unwrap_or(identity)
    }

    fn edge_plane_intersection_identity(
        &self,
        edge: &ConstructionEdgeIdentity,
        plane: ConstructionPlaneIdentity,
    ) -> ConstructionVertexIdentity {
        let plane = self.canonical_plane_identity(plane);
        if let ConstructionEdgeIdentity::Source { .. } = edge
            && let Some(supports) = self.source_edge_supports.get(edge)
            && supports.len() >= 2
        {
            let mut planes = [supports[0], supports[1], plane];
            planes.sort_unstable();
            return ConstructionVertexIdentity::PlaneTriple { planes };
        }
        edge.intersection_identity(plane)
    }

    fn record_incidence(
        &mut self,
        identity: &ConstructionVertexIdentity,
        plane: ConstructionPlaneIdentity,
    ) {
        let plane = self.canonical_plane_identity(plane);
        let incidences = self.point_incidences.entry(identity.clone()).or_default();
        if !incidences.contains(&plane) {
            incidences.push(plane);
        }
    }

    fn canonical_vertex_identity(
        &self,
        identity: &ConstructionVertexIdentity,
    ) -> ConstructionVertexIdentity {
        let mut canonical = identity.clone();
        while let Some(next) = self.canonical_identities.get(&canonical) {
            if *next == canonical {
                break;
            }
            canonical = next.clone();
        }
        canonical
    }

    fn record_definition_incidences(&mut self, identity: &ConstructionVertexIdentity) {
        let planes = match identity {
            ConstructionVertexIdentity::Source { .. } => return,
            ConstructionVertexIdentity::SourceEdgePlane {
                mesh,
                endpoints,
                plane,
            } => {
                let edge = ConstructionEdgeIdentity::Source {
                    mesh: *mesh,
                    endpoints: *endpoints,
                };
                let mut planes = self
                    .source_edge_supports
                    .get(&edge)
                    .cloned()
                    .unwrap_or_default();
                planes.push(*plane);
                planes
            }
            ConstructionVertexIdentity::PlaneTriple { planes } => planes.to_vec(),
        };
        for plane in planes {
            self.record_incidence(identity, plane);
        }
    }

    fn definition_planes(&self, identity: &ConstructionVertexIdentity) -> Option<[Plane; 3]> {
        match identity {
            ConstructionVertexIdentity::Source { .. } => {
                self.source_vertices.get(identity).cloned()
            }
            ConstructionVertexIdentity::SourceEdgePlane {
                mesh,
                endpoints,
                plane,
            } => {
                let edge = ConstructionEdgeIdentity::Source {
                    mesh: *mesh,
                    endpoints: *endpoints,
                };
                let [support, boundary] = self.source_edges.get(&edge)?;
                Some([
                    support.clone(),
                    boundary.clone(),
                    self.planes.get(plane)?.clone(),
                ])
            }
            ConstructionVertexIdentity::PlaneTriple { planes } => Some([
                self.planes.get(&planes[0])?.clone(),
                self.planes.get(&planes[1])?.clone(),
                self.planes.get(&planes[2])?.clone(),
            ]),
        }
    }

    fn identities_certifiably_equal(
        &self,
        left_identity: &ConstructionVertexIdentity,
        left: &HomogeneousPoint3,
        right_identity: &ConstructionVertexIdentity,
        right: &HomogeneousPoint3,
    ) -> bool {
        let identity_on_triple =
            |identity: &ConstructionVertexIdentity, triple: &ConstructionVertexIdentity| {
                let ConstructionVertexIdentity::PlaneTriple { planes } = triple else {
                    return false;
                };
                self.point_incidences
                    .get(identity)
                    .is_some_and(|incidences| planes.iter().all(|plane| incidences.contains(plane)))
            };
        if (identity_on_triple(left_identity, right_identity)
            || identity_on_triple(right_identity, left_identity))
            && projective_points_certifiably_equal(left, right)
        {
            return true;
        }
        let left_definition = self.definition_planes(left_identity);
        let right_definition = self.definition_planes(right_identity);
        if let (Some(left_definition), Some(right_definition)) =
            (&left_definition, &right_definition)
        {
            let definitions_equal = right_definition.iter().all(|plane| {
                let determinant = crate::intersection::four_plane_determinant(
                    &left_definition[0],
                    &left_definition[1],
                    &left_definition[2],
                    plane,
                );
                crate::predicate::classify_real(&determinant) == Ok(Classification::On)
            }) && left_definition.iter().all(|plane| {
                let determinant = crate::intersection::four_plane_determinant(
                    &right_definition[0],
                    &right_definition[1],
                    &right_definition[2],
                    plane,
                );
                crate::predicate::classify_real(&determinant) == Ok(Classification::On)
            });
            if definitions_equal {
                return true;
            }
        }
        let point_satisfies = |point: &HomogeneousPoint3, definition: &[Plane; 3]| {
            definition.iter().all(|plane| {
                crate::predicate::classify_real(&homogeneous_point_plane_expression(point, plane))
                    == Ok(Classification::On)
            })
        };
        match (left_definition.as_ref(), right_definition.as_ref()) {
            (Some(definition), None) => point_satisfies(right, definition),
            (None, Some(definition)) => point_satisfies(left, definition),
            (None, None) => projective_points_certifiably_equal(left, right),
            (Some(_), Some(_)) => projective_points_certifiably_equal(left, right),
        }
    }

    fn resolve_vertex_coincidences(&mut self) {
        let mut entries = self
            .points
            .iter()
            .map(|(identity, point)| (identity.clone(), point.clone(), projective_point_f64(point)))
            .collect::<Vec<_>>();
        entries.sort_unstable_by(|left, right| left.0.cmp(&right.0));

        let mut sets = AtomicDisjointSets::new(entries.len());
        let mut finite = entries
            .iter()
            .enumerate()
            .filter_map(|(index, entry)| entry.2.map(|point| (index, point)))
            .collect::<Vec<_>>();
        finite.sort_unstable_by(|left, right| left.1[0].total_cmp(&right.1[0]));
        for right in 0..finite.len() {
            let (right_index, right_point) = finite[right];
            for &(left_index, left_point) in finite[..right].iter().rev() {
                let x_scale = left_point[0].abs().max(right_point[0].abs()).max(1.0);
                if right_point[0] - left_point[0] > x_scale * 1.0e-9 {
                    break;
                }
                if left_point.iter().zip(right_point).all(|(left, right)| {
                    let scale = left.abs().max(right.abs()).max(1.0);
                    (left - right).abs() <= scale * 1.0e-9
                }) && self.identities_certifiably_equal(
                    &entries[left_index].0,
                    &entries[left_index].1,
                    &entries[right_index].0,
                    &entries[right_index].1,
                ) {
                    sets.merge(left_index, right_index);
                }
            }
        }
        for right in 0..entries.len() {
            for left in 0..right {
                if (entries[left].2.is_none() || entries[right].2.is_none())
                    && self.identities_certifiably_equal(
                        &entries[left].0,
                        &entries[left].1,
                        &entries[right].0,
                        &entries[right].1,
                    )
                {
                    sets.merge(left, right);
                }
            }
        }

        let representatives = (0..entries.len())
            .map(|index| sets.representative(index))
            .collect::<Vec<_>>();
        let mut class_incidences = vec![Vec::new(); entries.len()];
        for (index, (identity, _, _)) in entries.iter().enumerate() {
            let representative = representatives[index];
            if let Some(incidences) = self.point_incidences.get(identity) {
                for &incidence in incidences {
                    if !class_incidences[representative].contains(&incidence) {
                        class_incidences[representative].push(incidence);
                    }
                }
            }
        }
        for incidences in &mut class_incidences {
            incidences.sort_unstable();
        }

        self.canonical_identities.clear();
        for (index, (identity, _, _)) in entries.iter().enumerate() {
            let representative = representatives[index];
            let canonical_identity = entries[representative].0.clone();
            let canonical_point = entries[representative].1.clone();
            self.canonical_identities
                .insert(identity.clone(), canonical_identity);
            self.points.insert(identity.clone(), canonical_point);
            self.point_incidences
                .insert(identity.clone(), class_incidences[representative].clone());
        }
    }

    fn intern(
        &mut self,
        identity: ConstructionVertexIdentity,
        point: HomogeneousPoint3,
    ) -> (HomogeneousPoint3, ConstructionVertexIdentity) {
        self.record_definition_incidences(&identity);
        if let Some(existing) = self.points.get(&identity) {
            return (existing.clone(), identity);
        }
        self.points.insert(identity.clone(), point.clone());
        (point, identity)
    }
}

fn projective_point_f64(point: &HomogeneousPoint3) -> Option<[f64; 3]> {
    let weight = point.w.to_f64_lossy()?;
    if weight == 0.0 || !weight.is_finite() {
        return None;
    }
    let coordinates = [&point.x, &point.y, &point.z].map(|coordinate| {
        let value = coordinate.to_f64_lossy()? / weight;
        value.is_finite().then_some(value)
    });
    let [Some(x), Some(y), Some(z)] = coordinates else {
        return None;
    };
    Some([x, y, z])
}

fn affine_point_f64(point: &Point3) -> Option<[f64; 3]> {
    let point = [
        point.x.to_f64_lossy()?,
        point.y.to_f64_lossy()?,
        point.z.to_f64_lossy()?,
    ];
    point.into_iter().all(f64::is_finite).then_some(point)
}

fn projective_point_plane_may_be_on(point: &HomogeneousPoint3, plane: &Plane) -> bool {
    projective_point_f64(point).is_none_or(|point| affine_point_plane_may_be_on(point, plane))
}

fn affine_point_plane_may_be_on(point: [f64; 3], plane: &Plane) -> bool {
    match plane_f64(plane) {
        Some(plane) => {
            let value = plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3];
            let scale = point.into_iter().map(f64::abs).fold(1.0_f64, f64::max);
            value.abs() <= scale * 1.0e-9
        }
        None => true,
    }
}

fn projective_points_certifiably_equal(
    left: &HomogeneousPoint3,
    right: &HomogeneousPoint3,
) -> bool {
    let left = [&left.x, &left.y, &left.z, &left.w];
    let right = [&right.x, &right.y, &right.z, &right.w];
    for first in 0..left.len() {
        for second in (first + 1)..left.len() {
            let minor = Real::signed_product_sum(
                [true, false],
                [[left[first], right[second]], [left[second], right[first]]],
            );
            if crate::predicate::classify_real(&minor) != Ok(Classification::On) {
                return false;
            }
        }
    }
    true
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
    source_queries: Vec<Option<Option<PreparedRationalLinearForm4Query>>>,
    source_classifications: Vec<Option<Classification>>,
    source_plane_count: Option<usize>,
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
    ) -> HypermeshResult<(SourcePlaneRelation, Vec<usize>)> {
        if certifiably_same_unoriented_plane(&polygon.support, plane) {
            let on_source_vertices = polygon
                .known_vertex_identities
                .as_deref()
                .into_iter()
                .flatten()
                .filter_map(|identity| match identity {
                    ConstructionVertexIdentity::Source { vertex, .. } => Some(*vertex),
                    _ => None,
                })
                .collect();
            return Ok((SourcePlaneRelation::Inside, on_source_vertices));
        }
        let mut has_negative = false;
        let mut has_positive = false;
        let mut on_source_vertices = Vec::new();
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
                Classification::On => {
                    if let Some(source_vertex) = source_vertex {
                        on_source_vertices.push(source_vertex);
                    }
                }
            }
            if has_positive && has_negative {
                return Ok((SourcePlaneRelation::Crossing, on_source_vertices));
            }
        }
        Ok((
            if has_positive {
                SourcePlaneRelation::Outside
            } else {
                SourcePlaneRelation::Inside
            },
            on_source_vertices,
        ))
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
        if let Some(source_vertex) = source_vertex {
            if let Some(cached_plane_count) = self.source_plane_count {
                if cached_plane_count != plane_count {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
            } else {
                self.source_plane_count = Some(plane_count);
            }
            let source_count = source_vertex
                .checked_add(1)
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            if self.source_queries.len() < source_count {
                self.source_queries.resize(source_count, None);
            }
            let classification_count = source_count
                .checked_mul(plane_count)
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            if self.source_classifications.len() < classification_count {
                self.source_classifications
                    .resize(classification_count, None);
            }
            let classification_index = source_vertex * plane_count + plane_index;
            if let Some(classification) = self.source_classifications[classification_index] {
                return Ok(classification);
            }
            let prepared_query = *self.source_queries[source_vertex]
                .get_or_insert_with(|| Real::prepare_rational_affine_point3_query([x, y, z]));
            let classification = crate::predicate::classify_point_with_prepared_query(
                point,
                plane,
                prepared_query.as_ref(),
            )?;
            self.source_classifications[classification_index] = Some(classification);
            return Ok(classification);
        }

        let key = PointClassificationKey([x, y, z].map(hyperlattice::Rational::storage_identity));
        let cached = self.points.entry(key).or_insert_with(make_cached);
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
    fn point_has_plane_incidence(
        &self,
        point_index: usize,
        plane_identity: ConstructionPlaneIdentity,
        plane: &Plane,
    ) -> bool {
        if self.source_plane == plane_identity
            || certifiably_same_unoriented_plane(&self.support, plane)
        {
            return true;
        }
        if self.edge_identities.is_empty() {
            return false;
        }
        let previous = if point_index == 0 {
            self.edge_identities.len() - 1
        } else {
            point_index - 1
        };
        [previous, point_index].into_iter().any(|edge_index| {
            matches!(
                self.edge_identities.get(edge_index),
                Some(ConstructionEdgeIdentity::Split { planes })
                    if planes.contains(&plane_identity)
            )
        }) || (projective_point_plane_may_be_on(&self.points[point_index], plane)
            && crate::intersection::four_plane_determinant(
                &self.support,
                &self.edges[previous],
                &self.edges[point_index],
                plane,
            )
            .definitely_zero())
    }

    fn from_polygon(
        polygon: &ConvexPolygon,
        source_plane: ConstructionPlaneIdentity,
        point_cache: &mut ProjectivePointCache,
    ) -> HypermeshResult<Self> {
        let source_points = polygon
            .known_vertices
            .as_ref()
            .ok_or(crate::error::HypermeshError::UnknownClassification)?;
        let edge_identities = polygon
            .known_edge_identities
            .as_ref()
            .ok_or(crate::error::HypermeshError::UnknownClassification)?
            .to_vec();
        if edge_identities.len() != source_points.len() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let point_identities = (0..source_points.len())
            .map(|point_index| {
                let vertex = source_vertex_index(&edge_identities, point_index)
                    .ok_or(crate::error::HypermeshError::UnknownClassification)?;
                let mesh = match &edge_identities[point_index] {
                    ConstructionEdgeIdentity::Source { mesh, .. } => *mesh,
                    ConstructionEdgeIdentity::Split { .. } => {
                        return Err(crate::error::HypermeshError::UnknownClassification);
                    }
                };
                Ok(ConstructionVertexIdentity::Source { mesh, vertex })
            })
            .collect::<HypermeshResult<Vec<_>>>()?;
        let (points, point_identities): (Vec<_>, Vec<_>) = source_points
            .iter()
            .zip(point_identities.iter().cloned())
            .map(|(point, identity)| {
                point_cache.intern(
                    identity,
                    HomogeneousPoint3::new(
                        point.x.clone(),
                        point.y.clone(),
                        point.z.clone(),
                        Real::one(),
                    ),
                )
            })
            .unzip();
        let edges = match polygon.edges.len() {
            len if len == points.len() => polygon.edges.as_ref().clone(),
            1 => vec![polygon.edges[0].clone(); points.len()],
            _ => return Err(crate::error::HypermeshError::UnknownClassification),
        };
        Ok(Self {
            points,
            point_identities,
            edges,
            edge_identities,
            support: polygon.support.clone(),
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
        let plane_identity = point_cache.canonical_plane_identity(plane_identity);
        let evaluated = self
            .points
            .iter()
            .enumerate()
            .map(|(point_index, point)| {
                if self.point_has_plane_incidence(point_index, plane_identity, plane) {
                    Ok((Real::zero(), Classification::On))
                } else {
                    projective_plane_value(point, plane).inspect_err(|_error| {
                        if cfg!(debug_assertions) {
                            eprintln!(
                                "[DEBUG] projective clip point failed: source={:?} target={:?} point={point_index} value={:?}",
                                self.source_plane,
                                plane_identity,
                                hyperlattice::homogeneous_point_plane_expression(point, plane)
                                    .to_f64_lossy(),
                            );
                        }
                    })
                }
            })
            .collect::<HypermeshResult<Vec<_>>>()?;
        for (point_index, (_, classification)) in evaluated.iter().enumerate() {
            if *classification == Classification::On {
                point_cache.record_incidence(&self.point_identities[point_index], plane_identity);
            }
        }
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
        let mut negative_point_identities = Vec::with_capacity(self.points.len() + 1);
        let mut negative_edges = Vec::with_capacity(self.edges.len() + 1);
        let mut negative_edge_identities = Vec::with_capacity(self.edge_identities.len() + 1);
        let mut positive = Vec::with_capacity(self.points.len() + 1);
        let mut positive_point_identities = Vec::with_capacity(self.points.len() + 1);
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
                &mut negative_point_identities,
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
                &mut positive_point_identities,
                &mut positive_edges,
                &mut positive_edge_identities,
            );
        }
        remove_closing_labeled_duplicate(
            &mut negative,
            &mut negative_point_identities,
            &mut negative_edges,
            &mut negative_edge_identities,
        );
        remove_closing_labeled_duplicate(
            &mut positive,
            &mut positive_point_identities,
            &mut positive_edges,
            &mut positive_edge_identities,
        );
        Ok(ProjectiveClip {
            negative: Self {
                points: negative,
                point_identities: negative_point_identities,
                edges: negative_edges,
                edge_identities: negative_edge_identities,
                support: self.support.clone(),
                source_plane: self.source_plane,
                source_unchanged: false,
            },
            positive: Self {
                points: positive,
                point_identities: positive_point_identities,
                edges: positive_edges,
                edge_identities: positive_edge_identities,
                support: self.support.clone(),
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
        let plane_identity = point_cache.canonical_plane_identity(plane_identity);
        let evaluated = self
            .points
            .iter()
            .enumerate()
            .map(|(point_index, point)| {
                if self.point_has_plane_incidence(point_index, plane_identity, plane) {
                    Ok((Real::zero(), Classification::On))
                } else {
                    projective_plane_value(point, plane).inspect_err(|_error| {
                        if cfg!(debug_assertions) {
                            eprintln!(
                                "[DEBUG] projective negative clip point failed: source={:?} target={:?} point={point_index} identity={:?} adjacent={:?} point_xyz={:?} plane={:?} exact={:?} value={:?}",
                                self.source_plane,
                                plane_identity,
                                self.point_identities.get(point_index),
                                [
                                    self.edge_identities.get(if point_index == 0 { self.edge_identities.len() - 1 } else { point_index - 1 }),
                                    self.edge_identities.get(point_index),
                                ],
                                [
                                    point.x.to_f64_lossy(),
                                    point.y.to_f64_lossy(),
                                    point.z.to_f64_lossy(),
                                    point.w.to_f64_lossy(),
                                ],
                                [
                                    plane.normal.x.to_f64_lossy(),
                                    plane.normal.y.to_f64_lossy(),
                                    plane.normal.z.to_f64_lossy(),
                                    plane.offset.to_f64_lossy(),
                                ],
                                [
                                    plane.normal.x.exact_rational_ref().is_some(),
                                    plane.normal.y.exact_rational_ref().is_some(),
                                    plane.normal.z.exact_rational_ref().is_some(),
                                    plane.offset.exact_rational_ref().is_some(),
                                ],
                                hyperlattice::homogeneous_point_plane_expression(point, plane)
                                    .to_f64_lossy(),
                            );
                        }
                    })
                }
            })
            .collect::<HypermeshResult<Vec<_>>>()?;
        for (point_index, (_, classification)) in evaluated.iter().enumerate() {
            if *classification == Classification::On {
                point_cache.record_incidence(&self.point_identities[point_index], plane_identity);
            }
        }
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
        let mut point_identities = Vec::with_capacity(self.points.len() + 1);
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
                &mut point_identities,
                &mut edges,
                &mut edge_identities,
            );
        }
        remove_closing_labeled_duplicate(
            &mut points,
            &mut point_identities,
            &mut edges,
            &mut edge_identities,
        );
        Ok(Self {
            points,
            point_identities,
            edges,
            edge_identities,
            support: self.support.clone(),
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
    ) -> (HomogeneousPoint3, ConstructionVertexIdentity) {
        let identity = point_cache
            .edge_plane_intersection_identity(&self.edge_identities[edge_index], plane_identity);
        let point = point_cache
            .definition_planes(&identity)
            .and_then(|planes| positive_weight_plane_intersection(&planes))
            .unwrap_or_else(|| {
                projective_crossing_point(
                    current,
                    current_value,
                    current_classification,
                    next,
                    next_value,
                )
            });
        point_cache.intern(identity, point)
    }

    #[allow(clippy::too_many_arguments)]
    fn append_clipped_transition(
        &self,
        index: usize,
        current_classification: Classification,
        next_classification: Classification,
        intersection: Option<&(HomogeneousPoint3, ConstructionVertexIdentity)>,
        split_edge: &Plane,
        split_identity: &ConstructionEdgeIdentity,
        positive: bool,
        points: &mut Vec<HomogeneousPoint3>,
        point_identities: &mut Vec<ConstructionVertexIdentity>,
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
                point_identities,
                edges,
                edge_identities,
                self.points[index].clone(),
                self.point_identities[index].clone(),
                self.edges[index].clone(),
                self.edge_identities[index].clone(),
            );
        } else if current_inside {
            if current_classification == Classification::On {
                push_labeled_projective(
                    points,
                    point_identities,
                    edges,
                    edge_identities,
                    self.points[index].clone(),
                    self.point_identities[index].clone(),
                    split_edge.clone(),
                    split_identity.clone(),
                );
            } else {
                push_labeled_projective(
                    points,
                    point_identities,
                    edges,
                    edge_identities,
                    self.points[index].clone(),
                    self.point_identities[index].clone(),
                    self.edges[index].clone(),
                    self.edge_identities[index].clone(),
                );
                push_labeled_projective(
                    points,
                    point_identities,
                    edges,
                    edge_identities,
                    intersection
                        .expect("strict side transition has an intersection")
                        .0
                        .clone(),
                    intersection
                        .expect("strict side transition has an intersection")
                        .1
                        .clone(),
                    split_edge.clone(),
                    split_identity.clone(),
                );
            }
        } else if next_inside && next_classification != Classification::On {
            push_labeled_projective(
                points,
                point_identities,
                edges,
                edge_identities,
                intersection
                    .expect("strict side transition has an intersection")
                    .0
                    .clone(),
                intersection
                    .expect("strict side transition has an intersection")
                    .1
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
            .enumerate()
            .map(|(point_index, point)| {
                affine_cache.resolve(point, Some(self.point_identities[point_index].clone()))
            })
            .collect::<HypermeshResult<Vec<_>>>()?;
        let vertex_identities = self.point_identities.clone();
        Ok(source.with_known_vertex_cycle_and_edges(
            vertices,
            vertex_identities,
            self.edges.clone(),
            self.edge_identities.clone(),
        ))
    }

    fn empty() -> Self {
        Self {
            points: Vec::new(),
            point_identities: Vec::new(),
            edges: Vec::new(),
            edge_identities: Vec::new(),
            support: Plane::from_coefficients(
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::zero(),
            ),
            source_plane: ConstructionPlaneIdentity {
                mesh: usize::MAX,
                plane: usize::MAX,
            },
            source_unchanged: false,
        }
    }
}

impl ProjectiveAffineCache {
    fn resolve(
        &mut self,
        point: &HomogeneousPoint3,
        identity: Option<ConstructionVertexIdentity>,
    ) -> HypermeshResult<Point3> {
        if let Some(identity) = identity.as_ref()
            && let Some(affine) = self.identities.get(identity)
        {
            return Ok(affine.clone());
        }
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
            if let Some(identity) = identity {
                self.identities.insert(identity, affine.clone());
            }
            return Ok(affine);
        }
        let affine = affine_projective_point(point)?;
        if let Some(identity) = identity {
            self.identities.insert(identity, affine.clone());
        }
        Ok(affine)
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

fn compute_two_convex_inputs_projectively(
    polygons: &[ConvexPolygon],
    operation: BooleanOp,
) -> HypermeshResult<Option<ConvexCandidate>> {
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
        let exact_f64 = exact_plane_f64(&polygon.support);
        let plane = if let Some(index) =
            storage_key.and_then(|key| storage_support_planes[mesh].get(&key).copied())
        {
            index
        } else if let Some(values) = exact_f64 {
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
        } else if let Some(index) = support_planes[mesh].iter().position(|existing| {
            planes_may_be_same(existing, &polygon.support)
                && certifiably_same_oriented_plane(existing, &polygon.support).unwrap_or(false)
        }) {
            index
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
    let canonical_plane_identities = canonical_plane_identities(&support_planes);
    let (projective_polygons, mut projective_polygon_support_planes) =
        match collapse_certified_convex_faces(polygons, &polygon_support_planes, &support_planes) {
            Ok(collapsed) => collapsed,
            Err(crate::error::HypermeshError::UnknownClassification) => {
                (polygons.to_vec(), polygon_support_planes)
            }
            Err(error) => return Err(error),
        };
    for identity in &mut projective_polygon_support_planes {
        *identity = canonical_plane_identities[identity.mesh][identity.plane];
    }
    let polygons = projective_polygons.as_slice();
    let polygon_support_planes = projective_polygon_support_planes;
    let mut classified = Vec::new();
    let mut point_plane_caches: [PointPlaneClassificationCache; 2] =
        std::array::from_fn(|_| PointPlaneClassificationCache::default());
    let mut affine_cache = ProjectiveAffineCache::default();
    let mut projective_point_cache = ProjectivePointCache::default();
    for (mesh, planes) in support_planes.iter().enumerate() {
        for (plane, value) in planes.iter().enumerate() {
            let identity = ConstructionPlaneIdentity { mesh, plane };
            let canonical = canonical_plane_identities[mesh][plane];
            projective_point_cache
                .planes
                .entry(canonical)
                .or_insert_with(|| (*value).clone());
            projective_point_cache
                .canonical_planes
                .insert(identity, canonical);
        }
    }
    let mut source_vertex_supports: StorageHashMap<
        ConstructionVertexIdentity,
        Vec<ConstructionPlaneIdentity>,
    > = StorageHashMap::default();
    let mut source_vertex_points: StorageHashMap<ConstructionVertexIdentity, Point3> =
        StorageHashMap::default();
    for (polygon, support_identity) in polygons.iter().zip(&polygon_support_planes) {
        if let Some(vertex_identities) = polygon.known_vertex_identities.as_ref() {
            let retained_vertices = polygon.known_vertices.as_ref();
            for (vertex_index, vertex_identity) in vertex_identities.iter().enumerate() {
                if matches!(vertex_identity, ConstructionVertexIdentity::Source { .. }) {
                    let supports = source_vertex_supports
                        .entry(vertex_identity.clone())
                        .or_default();
                    if !supports.contains(support_identity) {
                        supports.push(*support_identity);
                    }
                    let incidences = projective_point_cache
                        .point_incidences
                        .entry(vertex_identity.clone())
                        .or_default();
                    if !incidences.contains(support_identity) {
                        incidences.push(*support_identity);
                    }
                    if let Some(point) =
                        retained_vertices.and_then(|vertices| vertices.get(vertex_index))
                    {
                        source_vertex_points
                            .entry(vertex_identity.clone())
                            .or_insert_with(|| point.clone());
                    }
                }
            }
        }
        let Some(edge_identities) = polygon.known_edge_identities.as_ref() else {
            continue;
        };
        if edge_identities.len() != polygon.edges.len() {
            continue;
        }
        let Some(support) = projective_point_cache.planes.get(support_identity).cloned() else {
            continue;
        };
        for (edge_identity, edge_plane) in edge_identities.iter().zip(polygon.edges.iter()) {
            if matches!(edge_identity, ConstructionEdgeIdentity::Source { .. }) {
                projective_point_cache
                    .source_edges
                    .entry(edge_identity.clone())
                    .or_insert_with(|| [support.clone(), edge_plane.clone()]);
                let supports = projective_point_cache
                    .source_edge_supports
                    .entry(edge_identity.clone())
                    .or_default();
                if !supports.contains(support_identity) {
                    supports.push(*support_identity);
                }
            }
        }
    }
    for (identity, point) in &source_vertex_points {
        let ConstructionVertexIdentity::Source { mesh, .. } = identity else {
            continue;
        };
        let other = 1 - *mesh;
        for (plane, value) in support_planes[other].iter().enumerate() {
            if affine_point_f64(point)
                .is_none_or(|point| affine_point_plane_may_be_on(point, value))
                && classify_point(point, value) == Ok(Classification::On)
            {
                let plane_identity = canonical_plane_identities[other][plane];
                let incidences = projective_point_cache
                    .point_incidences
                    .entry(identity.clone())
                    .or_default();
                if !incidences.contains(&plane_identity) {
                    incidences.push(plane_identity);
                }
            }
        }
    }
    for (identity, supports) in source_vertex_supports {
        'definition: for first in 0..supports.len() {
            for second in (first + 1)..supports.len() {
                for third in (second + 1)..supports.len() {
                    let planes = [
                        projective_point_cache.planes[&supports[first]].clone(),
                        projective_point_cache.planes[&supports[second]].clone(),
                        projective_point_cache.planes[&supports[third]].clone(),
                    ];
                    let point = intersect_three_planes(&planes[0], &planes[1], &planes[2]);
                    if crate::predicate::classify_real(&point.w)
                        .is_ok_and(|classification| classification != Classification::On)
                    {
                        projective_point_cache
                            .source_vertices
                            .insert(identity, planes);
                        break 'definition;
                    }
                }
            }
        }
    }
    for (polygon, source_plane) in polygons.iter().zip(polygon_support_planes) {
        let host = usize::try_from(polygon.mesh_index)
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
        let other = 1 - host;
        let emit_outside = projective_transition_is_emitted(host, false, operation);
        let default_emit_inside = projective_transition_is_emitted(host, true, operation);
        let mut candidate_planes = Vec::new();
        let mut excluded = false;
        let mut has_cooriented_coincident_support = false;
        for (plane_index, &plane) in support_planes[other].iter().enumerate() {
            has_cooriented_coincident_support |= planes_may_be_same(&polygon.support, plane)
                && certifiably_same_oriented_plane(&polygon.support, plane).unwrap_or(false);
            let (relation, on_source_vertices) = point_plane_caches[host]
                .source_relation(
                polygon,
                plane,
                plane_index,
                support_planes[other].len(),
            )
            .inspect_err(|_error| {
                if cfg!(debug_assertions) {
                    eprintln!(
                        "[DEBUG] projective source relation failed: host={host} polygon={} other_plane={plane_index}",
                        polygon.polygon_index,
                    );
                }
            })?;
            let plane_identity = canonical_plane_identities[other][plane_index];
            for vertex in on_source_vertices {
                let incidences = projective_point_cache
                    .point_incidences
                    .entry(ConstructionVertexIdentity::Source { mesh: host, vertex })
                    .or_default();
                if !incidences.contains(&plane_identity) {
                    incidences.push(plane_identity);
                }
            }
            match relation {
                SourcePlaneRelation::Inside => {}
                SourcePlaneRelation::Outside => {
                    excluded = true;
                    break;
                }
                SourcePlaneRelation::Crossing => candidate_planes.push(plane_index),
            }
        }
        let (emit_inside, inside_winding) = if has_cooriented_coincident_support {
            match operation {
                BooleanOp::Union => (host == 0, false),
                BooleanOp::Intersection => (host == 0, true),
                BooleanOp::Difference | BooleanOp::SymmetricDifference => (false, true),
            }
        } else {
            (default_emit_inside, true)
        };
        if !emit_outside && !emit_inside {
            continue;
        }
        if excluded {
            if emit_outside {
                push_source_transition(&mut classified, polygon, host, other, false)?;
            }
            continue;
        }
        if candidate_planes.is_empty() {
            if emit_inside {
                push_source_transition(&mut classified, polygon, host, other, inside_winding)?;
            }
            continue;
        }
        let source =
            ProjectiveCycle::from_polygon(polygon, source_plane, &mut projective_point_cache)
                .inspect_err(|_error| {
                    if cfg!(debug_assertions) {
                        eprintln!(
                            "[DEBUG] projective source cycle failed: host={host} polygon={}",
                            polygon.polygon_index,
                        );
                    }
                })?;

        let active_result = exact_inside_and_active_planes(
            polygon,
            &source,
            &support_planes[other],
            support_planes_f64[other].as_deref(),
            &candidate_planes,
            other,
            &mut projective_point_cache,
        )
        .inspect_err(|_error| {
            if cfg!(debug_assertions) {
                eprintln!(
                    "[DEBUG] projective active planes failed: host={host} polygon={}",
                    polygon.polygon_index,
                );
            }
        })?;
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
                    operation,
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
                    inside_winding,
                    operation,
                )?;
            }
            continue;
        }
        let mut remainder = source;
        let mut has_inside = true;
        for plane_index in active_planes {
            let clipped = remainder.clip(
                support_planes[other][plane_index],
                canonical_plane_identities[other][plane_index],
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
                        operation,
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
                        operation,
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
                inside_winding,
                operation,
            )?;
        }
    }
    projective_point_cache.resolve_vertex_coincidences();
    affine_cache.identities.clear();
    for fragment in &mut classified {
        if let Some(vertex_identities) = fragment.polygon.known_vertex_identities.as_ref() {
            let canonical_identities = vertex_identities
                .iter()
                .map(|identity| projective_point_cache.canonical_vertex_identity(identity))
                .collect::<Vec<_>>();
            let original_vertices = fragment
                .polygon
                .known_vertices
                .as_ref()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?
                .iter()
                .cloned()
                .collect::<Vec<_>>();
            let vertices = canonical_identities
                .iter()
                .zip(original_vertices)
                .map(|(identity, original)| {
                    let Some(point) = projective_point_cache.points.get(identity) else {
                        return Ok(original);
                    };
                    affine_cache.resolve(point, Some(identity.clone()))
                })
                .collect::<HypermeshResult<Vec<_>>>()?;
            let edge_identities = fragment
                .polygon
                .known_edge_identities
                .as_deref()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?
                .to_vec();
            fragment.polygon = fragment.polygon.with_known_vertex_cycle_and_edges(
                vertices,
                canonical_identities,
                fragment.polygon.edges.as_ref().clone(),
                edge_identities,
            );
        }
    }

    let triangle_soup = {
        if operation != BooleanOp::SymmetricDifference {
            let indicator = make_indicator(operation, support_planes.len());
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
            let triangulate = |recover| {
                crate::output::triangulate_preclassified_arrangement_construction_candidates(
                    &classified,
                    recover,
                )
                .and_then(certify_triangle_soup_closure)
            };
            triangulate(false).or_else(|_| triangulate(true))
        } else if operation == BooleanOp::Union {
            let triangulate = |recover| {
                crate::output::triangulate_selected_preclassified_arrangement_construction_candidates(
                    &classified,
                    recover,
                )
                .and_then(certify_triangle_soup_closure)
            };
            triangulate(false).or_else(|_| triangulate(true))
        } else {
            let triangulate = |recover| {
                crate::output::triangulate_classified_arrangement_construction_candidates(
                    &classified,
                    recover,
                )
                .and_then(|triangles| {
                    select_triangle_arrangement(&triangles, operation, support_planes.len())
                })
            };
            triangulate(false).or_else(|_| triangulate(true))
        }
        .or_else(|error| {
            if cfg!(debug_assertions) {
                eprintln!("[DEBUG] construction-candidate triangulation failed: {error}");
            }
            crate::output::triangulate_classified_arrangement_precomputed_f64_scan(&classified)
                .and_then(|triangles| {
                    select_triangle_arrangement(&triangles, operation, support_planes.len())
                })
        });
        match soup {
            Ok(soup) => soup,
            Err(_) => return Ok(None),
        }
    };
    Ok(Some(ConvexCandidate {
        classified,
        triangle_soup,
    }))
}

fn certifiably_same_oriented_plane(left: &Plane, right: &Plane) -> HypermeshResult<bool> {
    let left_coefficients = [&left.normal.x, &left.normal.y, &left.normal.z, &left.offset];
    let right_coefficients = [
        &right.normal.x,
        &right.normal.y,
        &right.normal.z,
        &right.offset,
    ];
    let mut unknown_minor = false;
    for first in 0..left_coefficients.len() {
        for second in (first + 1)..left_coefficients.len() {
            let minor = Real::signed_product_sum(
                [true, false],
                [
                    [left_coefficients[first], right_coefficients[second]],
                    [left_coefficients[second], right_coefficients[first]],
                ],
            );
            match crate::predicate::classify_real(&minor) {
                Ok(Classification::On) => {}
                Ok(Classification::Negative | Classification::Positive) => return Ok(false),
                Err(crate::error::HypermeshError::UnknownClassification) => unknown_minor = true,
                Err(error) => return Err(error),
            }
        }
    }
    if unknown_minor {
        return Ok(false);
    }
    let orientation = Real::signed_product_sum(
        [true, true, true],
        [
            [&left.normal.x, &right.normal.x],
            [&left.normal.y, &right.normal.y],
            [&left.normal.z, &right.normal.z],
        ],
    );
    Ok(crate::predicate::classify_real(&orientation)? == Classification::Positive)
}

fn certifiably_same_unoriented_plane(left: &Plane, right: &Plane) -> bool {
    certifiably_same_oriented_plane(left, right).unwrap_or(false)
        || certifiably_same_oriented_plane(left, &right.inverted()).unwrap_or(false)
}

fn plane_f64(plane: &Plane) -> Option<[f64; 4]> {
    let mut values = [
        plane.normal.x.to_f64_lossy()?,
        plane.normal.y.to_f64_lossy()?,
        plane.normal.z.to_f64_lossy()?,
        plane.offset.to_f64_lossy()?,
    ];
    if !values.into_iter().all(f64::is_finite) {
        return None;
    }
    let norm = values[..3]
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    if norm == 0.0 || !norm.is_finite() {
        return None;
    }
    let orientation = if values[..3]
        .iter()
        .max_by(|left, right| left.abs().total_cmp(&right.abs()))
        .copied()?
        .is_sign_negative()
    {
        -1.0
    } else {
        1.0
    };
    for value in &mut values {
        *value *= orientation / norm;
    }
    Some(values)
}

fn planes_may_be_same(left: &Plane, right: &Plane) -> bool {
    match (plane_f64(left), plane_f64(right)) {
        (Some(left), Some(right)) => left.iter().zip(right).all(|(left, right)| {
            let scale = left.abs().max(right.abs()).max(1.0);
            (left - right).abs() <= scale * 1.0e-9
        }),
        _ => true,
    }
}

fn canonical_plane_identities(
    support_planes: &[Vec<&Plane>; 2],
) -> [Vec<ConstructionPlaneIdentity>; 2] {
    let mut representatives = Vec::<(ConstructionPlaneIdentity, &Plane, Option<[f64; 4]>)>::new();
    std::array::from_fn(|mesh| {
        support_planes[mesh]
            .iter()
            .enumerate()
            .map(|(plane, value)| {
                let identity = ConstructionPlaneIdentity { mesh, plane };
                let approximate = plane_f64(value);
                let canonical = representatives
                    .iter()
                    .find_map(|(candidate, candidate_value, candidate_approximate)| {
                        let approximate_match =
                            match (candidate_approximate.as_ref(), approximate.as_ref()) {
                                (Some(candidate), Some(value)) => {
                                    candidate.iter().zip(value).all(|(left, right)| {
                                        let scale = left.abs().max(right.abs()).max(1.0);
                                        (left - right).abs() <= scale * 1.0e-9
                                    })
                                }
                                _ => true,
                            };
                        (approximate_match
                            && certifiably_same_unoriented_plane(candidate_value, value))
                        .then_some(*candidate)
                    })
                    .unwrap_or(identity);
                if canonical == identity {
                    representatives.push((identity, value, approximate));
                }
                canonical
            })
            .collect()
    })
}

fn collapse_certified_convex_faces(
    polygons: &[ConvexPolygon],
    polygon_support_planes: &[ConstructionPlaneIdentity],
    support_planes: &[Vec<&Plane>; 2],
) -> HypermeshResult<(Vec<ConvexPolygon>, Vec<ConstructionPlaneIdentity>)> {
    let mut groups: std::collections::BTreeMap<ConstructionPlaneIdentity, Vec<usize>> =
        std::collections::BTreeMap::new();
    for (polygon_index, &support) in polygon_support_planes.iter().enumerate() {
        groups.entry(support).or_default().push(polygon_index);
    }

    let mut faces = Vec::with_capacity(groups.len());
    let mut face_supports = Vec::with_capacity(groups.len());
    for (support_identity, polygon_indices) in groups {
        let mut edge_uses: StorageHashMap<ConstructionEdgeIdentity, usize> =
            StorageHashMap::default();
        let mut vertices: StorageHashMap<usize, Point3> = StorageHashMap::default();
        for &polygon_index in &polygon_indices {
            let polygon = &polygons[polygon_index];
            let edge_identities = polygon
                .known_edge_identities
                .as_ref()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let vertex_identities = polygon
                .known_vertex_identities
                .as_ref()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let points = polygon
                .known_vertices
                .as_ref()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            if edge_identities.len() != vertex_identities.len()
                || points.len() != vertex_identities.len()
            {
                return Err(crate::error::HypermeshError::UnknownClassification);
            }
            for (vertex_index, identity) in vertex_identities.iter().enumerate() {
                let ConstructionVertexIdentity::Source { mesh, vertex } = identity else {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                };
                if *mesh != support_identity.mesh {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
                vertices
                    .entry(*vertex)
                    .or_insert_with(|| points.get(vertex_index).expect("aligned vertex").clone());
            }
            for identity in edge_identities.iter() {
                *edge_uses.entry(identity.clone()).or_default() += 1;
            }
        }

        let mut outgoing: std::collections::BTreeMap<
            usize,
            (usize, Plane, ConstructionEdgeIdentity),
        > = std::collections::BTreeMap::new();
        for &polygon_index in &polygon_indices {
            let polygon = &polygons[polygon_index];
            let edge_identities = polygon
                .known_edge_identities
                .as_ref()
                .expect("validated above");
            let vertex_identities = polygon
                .known_vertex_identities
                .as_ref()
                .expect("validated above");
            let points = polygon.known_vertices.as_ref().expect("validated above");
            let rebuilt_planes = (polygon.edges.len() != edge_identities.len()).then(|| {
                InputTrianglePlanes::from_points(
                    points.get(0).expect("source triangle"),
                    points.get(1).expect("source triangle"),
                    points.get(2).expect("source triangle"),
                )
            });
            for edge_index in 0..edge_identities.len() {
                let edge_identity = &edge_identities[edge_index];
                if edge_uses.get(edge_identity).copied() != Some(1) {
                    continue;
                }
                let ConstructionVertexIdentity::Source { vertex: start, .. } =
                    &vertex_identities[edge_index]
                else {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                };
                let ConstructionVertexIdentity::Source { vertex: end, .. } =
                    &vertex_identities[(edge_index + 1) % vertex_identities.len()]
                else {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                };
                let edge_plane = rebuilt_planes.as_ref().map_or_else(
                    || polygon.edges[edge_index].clone(),
                    |planes| planes.edges[edge_index].clone(),
                );
                if outgoing
                    .insert(*start, (*end, edge_plane, edge_identity.clone()))
                    .is_some()
                {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
            }
        }
        let Some(&start) = outgoing.keys().next() else {
            return Err(crate::error::HypermeshError::UnknownClassification);
        };
        let mut face_vertices = Vec::with_capacity(outgoing.len());
        let mut vertex_identities = Vec::with_capacity(outgoing.len());
        let mut edge_planes = Vec::with_capacity(outgoing.len());
        let mut edge_identities = Vec::with_capacity(outgoing.len());
        let mut current = start;
        while face_vertices.len() < outgoing.len() {
            let Some((next, edge_plane, edge_identity)) = outgoing.get(&current) else {
                return Err(crate::error::HypermeshError::UnknownClassification);
            };
            face_vertices.push(
                vertices
                    .get(&current)
                    .ok_or(crate::error::HypermeshError::UnknownClassification)?
                    .clone(),
            );
            vertex_identities.push(ConstructionVertexIdentity::Source {
                mesh: support_identity.mesh,
                vertex: current,
            });
            edge_planes.push(edge_plane.clone());
            edge_identities.push(edge_identity.clone());
            current = *next;
            if current == start {
                break;
            }
        }
        if current != start || face_vertices.len() != outgoing.len() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let source = &polygons[polygon_indices[0]];
        faces.push(ConvexPolygon::from_certified_convex_face(
            support_planes[support_identity.mesh][support_identity.plane].clone(),
            face_vertices,
            vertex_identities,
            edge_planes,
            edge_identities,
            source.mesh_index,
            source.polygon_index,
            source.delta_w.clone(),
        ));
        face_supports.push(support_identity);
    }
    Ok((faces, face_supports))
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
        )
        .inspect_err(|_error| {
            if cfg!(debug_assertions) {
                eprintln!("[DEBUG] proposed projective clipping failed");
            }
        })?;
        if inside.points.len() < 3 {
            return Ok(None);
        }
        if cycle_satisfies_planes(
            &inside,
            support_planes,
            candidate_planes,
            support_plane_mesh,
        )
        .inspect_err(|_error| {
            if cfg!(debug_assertions) {
                eprintln!("[DEBUG] proposed projective verification failed");
            }
        })? {
            let active =
                active_cycle_planes(&inside, proposed_planes, support_plane_mesh, point_cache);
            return Ok(Some((inside, active)));
        }
    }

    let inside = clip_inside_cycle(
        source,
        support_planes,
        candidate_planes,
        support_plane_mesh,
        point_cache,
    )
    .inspect_err(|_error| {
        if cfg!(debug_assertions) {
            eprintln!("[DEBUG] full projective clipping failed");
        }
    })?;
    if inside.points.len() < 3 {
        return Ok(None);
    }
    let active = active_cycle_planes(
        &inside,
        candidate_planes.iter().copied(),
        support_plane_mesh,
        point_cache,
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
    point_cache: &ProjectivePointCache,
) -> Vec<usize> {
    plane_indices
        .into_iter()
        .filter(|&plane_index| {
            let identity = point_cache.canonical_plane_identity(ConstructionPlaneIdentity {
                mesh: support_plane_mesh,
                plane: plane_index,
            });
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
    support_plane_mesh: usize,
) -> HypermeshResult<bool> {
    for (point_index, point) in cycle.points.iter().enumerate() {
        let prepared = PreparedProjectivePoint3::new(point);
        for &plane_index in plane_indices {
            let plane_identity = ConstructionPlaneIdentity {
                mesh: support_plane_mesh,
                plane: plane_index,
            };
            if cycle.point_has_plane_incidence(
                point_index,
                plane_identity,
                support_planes[plane_index],
            ) {
                continue;
            }
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

fn positive_weight_plane_intersection(planes: &[Plane; 3]) -> Option<HomogeneousPoint3> {
    let point = intersect_three_planes(&planes[0], &planes[1], &planes[2]);
    let weight = point.w.exact_rational_ref()?;
    if weight.is_positive() {
        Some(point)
    } else if weight.is_negative() {
        Some(HomogeneousPoint3::new(
            -point.x, -point.y, -point.z, -point.w,
        ))
    } else {
        None
    }
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
    point_identities: &mut Vec<ConstructionVertexIdentity>,
    edges: &mut Vec<Plane>,
    edge_identities: &mut Vec<ConstructionEdgeIdentity>,
    point: HomogeneousPoint3,
    point_identity: ConstructionVertexIdentity,
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
    point_identities.push(point_identity);
    edges.push(edge);
    edge_identities.push(edge_identity);
}

fn remove_closing_labeled_duplicate(
    points: &mut Vec<HomogeneousPoint3>,
    point_identities: &mut Vec<ConstructionVertexIdentity>,
    edges: &mut Vec<Plane>,
    edge_identities: &mut Vec<ConstructionEdgeIdentity>,
) {
    if points.len() > 1 && points.first() == points.last() {
        points.pop();
        point_identities.pop();
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
    operation: BooleanOp,
) -> HypermeshResult<()> {
    if cycle.points.len() < 3 {
        return Ok(());
    }
    let winding = projective_transition_winding(host, other, inside_other);
    if !projective_transition_is_emitted(host, inside_other, operation) {
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

fn projective_transition_is_emitted(host: usize, inside_other: bool, operation: BooleanOp) -> bool {
    match operation {
        BooleanOp::Union => !inside_other,
        BooleanOp::Intersection => inside_other,
        BooleanOp::Difference => (host == 0 && !inside_other) || (host == 1 && inside_other),
        BooleanOp::SymmetricDifference => true,
    }
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

/// Symmetric-difference convenience wrapper.
pub fn boolean_symmetric_difference(
    a: MeshRef<'_>,
    b: MeshRef<'_>,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_operation(&[a, b], BooleanOp::SymmetricDifference, config)
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

        let mut point_cache = ProjectivePointCache::default();
        let cycle = ProjectiveCycle::from_polygon(
            &polygon,
            ConstructionPlaneIdentity { mesh: 0, plane: 0 },
            &mut point_cache,
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
            (SourcePlaneRelation::Crossing, _)
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
            (SourcePlaneRelation::Crossing, _)
        ));
        assert!(cache.points.is_empty());
        assert_eq!(
            cache
                .source_queries
                .iter()
                .filter(|cached| cached.is_some())
                .count(),
            2
        );
        assert_eq!(
            cache
                .source_classifications
                .iter()
                .filter(|classification| classification.is_some())
                .count(),
            2
        );
        assert_eq!(cache.source_plane_count, Some(1));
    }

    #[test]
    fn canonical_plane_identities_unify_cross_mesh_geometric_planes() {
        let bottom = Plane::axis_aligned(2, Real::zero());
        let opposite_bottom = bottom.inverted();
        let top = Plane::axis_aligned(2, Real::one());
        let support_planes = [vec![&bottom, &top], vec![&opposite_bottom, &bottom]];

        let identities = canonical_plane_identities(&support_planes);
        assert_eq!(identities[0][0], identities[1][0]);
        assert_eq!(identities[0][0], identities[1][1]);
        assert_ne!(identities[0][0], identities[0][1]);
    }

    #[test]
    fn plane_intersection_normalizes_negative_homogeneous_weight() {
        let planes = [
            Plane::axis_aligned(1, Real::from(2)),
            Plane::axis_aligned(0, Real::from(1)),
            Plane::axis_aligned(2, Real::from(3)),
        ];

        let point = positive_weight_plane_intersection(&planes).unwrap();
        assert!(
            point
                .w
                .exact_rational_ref()
                .is_some_and(Rational::is_positive)
        );
        assert_eq!(point.to_affine_point().unwrap(), p(1, 2, 3));
    }

    #[test]
    fn vertex_coincidences_resolve_atomically_to_order_independent_identity() {
        let planes = [
            Plane::axis_aligned(0, Real::from(1)),
            Plane::axis_aligned(1, Real::from(2)),
            Plane::axis_aligned(2, Real::from(3)),
            Plane::from_coefficients(Real::one(), Real::zero(), Real::one(), Real::from(-4)),
        ];
        let plane_ids: [ConstructionPlaneIdentity; 4] =
            std::array::from_fn(|plane| ConstructionPlaneIdentity { mesh: 0, plane });
        let identities = [
            ConstructionVertexIdentity::PlaneTriple {
                planes: [plane_ids[0], plane_ids[1], plane_ids[2]],
            },
            ConstructionVertexIdentity::PlaneTriple {
                planes: [plane_ids[0], plane_ids[1], plane_ids[3]],
            },
        ];

        let resolve = |order: [usize; 2]| {
            let mut cache = ProjectivePointCache::default();
            for (identity, plane) in plane_ids.into_iter().zip(planes.iter().cloned()) {
                cache.planes.insert(identity, plane);
            }
            for index in order {
                let definition = cache.definition_planes(&identities[index]).unwrap();
                let point = positive_weight_plane_intersection(&definition).unwrap();
                let (_, interned) = cache.intern(identities[index].clone(), point);
                assert_eq!(interned, identities[index]);
            }

            cache.resolve_vertex_coincidences();
            identities
                .each_ref()
                .map(|identity| cache.canonical_vertex_identity(identity))
        };

        let forward = resolve([0, 1]);
        let reverse = resolve([1, 0]);
        assert_eq!(forward, reverse);
        assert_eq!(forward[0], forward[1]);
        assert_eq!(
            forward[0],
            std::cmp::min(identities[0].clone(), identities[1].clone())
        );
    }
}
