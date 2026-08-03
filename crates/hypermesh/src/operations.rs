//! Public boolean operation entry points.

use std::borrow::Borrow;
use std::collections::hash_map::Entry;
use std::sync::Arc;

use hyperlattice::{
    HomogeneousPoint3, Point3, Rational, Real, homogeneous_point_plane_expression,
    intersect_three_planes,
};
use hyperreal::RationalLinearForm4Query;

use crate::context::{DecisionContext, MeshCertainty, MeshContext, MeshOutcome};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{
    Aabb, Classification, Plane, affine_projective_point_decision, axis_mut, axis_ref,
    compare_real_decision,
};
use crate::mesh::{
    OutputVertex, ProjectiveInputSoup, Triangle, TriangleMesh, TriangleMeshRef,
    build_polygon_soup_internal, build_polygon_soup_with_certified_convex_inputs,
    build_projective_input_soup,
};
use crate::output::{
    ARRANGEMENT_CLASSIFICATION, BooleanMesh, BooleanResult, ClassifiedPolygon, TriangleSource,
    certify_output_polygon_closure_decision,
};
use crate::polygon::{
    ConstructionEdgeIdentity, ConstructionPlaneIdentity, ConstructionVertexIdentity, ConvexPolygon,
    InputTrianglePlanes, KnownEdgeIdentityCycle, edge_plane,
    make_indexed_triangle_with_deferred_edges,
    make_indexed_triangle_with_deferred_edges_and_input_planes,
};
use crate::predicate::{
    ProjectivePoint3PredicateEvidence, RationalPlane4PredicateEvidence, classify_point_decision,
    classify_projective_point_decision,
};
use crate::storage_hash::StorageHashMap;
use crate::subdivision::{SubdivisionConfig, SubdivisionTask};
use crate::winding::{BooleanOp, WindingPair};

struct BooleanComputation {
    soup: crate::mesh::PolygonSoup,
    classified: Vec<crate::output::ClassifiedPolygon>,
    boolean_mesh: Option<crate::output::BooleanMesh>,
    input_edges_deferred: bool,
}

struct ConvexCandidate {
    classified: Vec<ClassifiedPolygon>,
    boolean_mesh: crate::output::BooleanMesh,
}

impl BooleanComputation {
    fn into_result(
        self,
        decisions: &DecisionContext,
        operation: BooleanOp,
    ) -> HypermeshResult<BooleanResult> {
        let has_certified_triangle_arrangement = self.boolean_mesh.is_some();
        let (result, finalization_preserved_polygon_count) =
            self.into_selected_result(decisions, operation)?;
        if !has_certified_triangle_arrangement || !finalization_preserved_polygon_count {
            certify_output_polygon_closure_decision(decisions, &result)?;
        }
        Ok(result)
    }

    fn into_selected_result(
        self,
        decisions: &DecisionContext,
        operation: BooleanOp,
    ) -> HypermeshResult<(BooleanResult, bool)> {
        let mut selected = Vec::with_capacity(self.classified.len());
        for mut polygon in self.classified {
            let winding = polygon
                .winding()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let classification = crate::winding::classify_polygon_output(
                &winding.w_front,
                &winding.w_back,
                operation,
            );
            if classification != 0 {
                polygon.classification = classification;
                if self.input_edges_deferred && polygon.polygon.edges.is_empty() {
                    polygon.polygon = polygon.polygon.with_rebuilt_edge_planes(decisions)?;
                }
                selected.push(polygon);
            }
        }
        let selected_count = selected.len();
        let result = BooleanResult::from_classified(self.soup, selected);
        // The projective candidate's triangle arrangement is an exact closure
        // certificate for these same oriented polygons. Finalization can only
        // invalidate that correspondence by merging duplicates; rebuilding a
        // deferred boundary plane leaves the vertex cycle unchanged.
        let finalization_preserved_polygon_count = result.output().polygons.len() == selected_count;
        Ok((result, finalization_preserved_polygon_count))
    }

    fn into_boolean_mesh(
        self,
        decisions: &DecisionContext,
        operation: BooleanOp,
    ) -> HypermeshResult<crate::output::BooleanMesh> {
        if let Some(soup) = self.boolean_mesh {
            return Ok(soup);
        }
        let (result, _) = self.into_selected_result(decisions, operation)?;
        crate::output::triangulate_and_resolve_polygon_certified(decisions, &result)
    }

    fn into_native_materialization(
        self,
        decisions: &DecisionContext,
        operation: BooleanOp,
    ) -> HypermeshResult<(crate::output::BooleanMesh, Vec<ConvexPolygon>)> {
        if let Some(mesh) = self.boolean_mesh {
            let result = BooleanResult::from_classified(self.soup, self.classified);
            return Ok((mesh, result.into_output().polygons));
        }
        let (result, _) = self.into_selected_result(decisions, operation)?;
        let mesh = crate::output::triangulate_and_resolve_polygon_certified(decisions, &result)?;
        Ok((mesh, result.into_output().polygons))
    }
}

fn select_triangle_arrangement(
    decisions: &DecisionContext,
    arrangement: &crate::output::ClassifiedTriangleArrangement,
    op: BooleanOp,
) -> HypermeshResult<crate::output::BooleanMesh> {
    if arrangement.soup.triangles.len() != arrangement.windings.len()
        || arrangement.soup.triangles.len() != arrangement.soup.sources.len()
    {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
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
            crate::winding::classify_polygon_output(&winding.w_front, &winding.w_back, op);
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
    let soup = crate::output::BooleanMesh {
        vertices: arrangement.soup.vertices.clone(),
        triangles,
        sources,
    };
    certify_boolean_mesh_closure(decisions, soup)
}

fn certify_boolean_mesh_closure(
    decisions: &DecisionContext,
    soup: crate::output::BooleanMesh,
) -> HypermeshResult<crate::output::BooleanMesh> {
    let soup = crate::output::resolve_tjunctions(decisions, &soup)?;
    if !soup.has_unique_nondegenerate_triangles_decision(decisions)? {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
    let closure = crate::output::boolean_mesh_closure_evidence(&soup);
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
    context: &MeshContext,
    meshes: &[TriangleMeshRef<'_>],
    operation: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<MeshOutcome<BooleanResult>> {
    let decisions = DecisionContext::new(context);
    crate::trace_dispatch!("boolean-operation", "start");
    let result = if let Some(native) = native_meshes(meshes) {
        compute_native_boolean_with_raw_retry(
            &decisions,
            &native,
            operation,
            config,
            true,
            |computation| {
                crate::trace_dispatch!("boolean-operation", "certify-output-closure");
                computation.into_result(&decisions, operation)
            },
        )?
    } else {
        let computation = compute_boolean(
            &decisions, meshes, operation, None, None, None, config, true,
        )?;
        crate::trace_dispatch!("boolean-operation", "certify-output-closure");
        computation.into_result(&decisions, operation)?
    };
    crate::trace_dispatch!("boolean-operation", "complete");
    Ok(decisions.finish(result))
}

/// Performs a Boolean operation and immediately returns a closure-certified
/// triangle soup.
///
/// This avoids materializing an intermediate polygon result when the caller
/// needs indexed triangles.
pub fn boolean_mesh(
    context: &MeshContext,
    meshes: &[TriangleMeshRef<'_>],
    operation: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<MeshOutcome<crate::output::BooleanMesh>> {
    let decisions = DecisionContext::new(context);
    crate::trace_dispatch!("boolean-operation", "start");
    if let [left, right] = meshes
        && let (Some(left), Some(right)) = (left.native, right.native)
        && let Some(soup) = axis_aligned_box_boolean_mesh(&decisions, left, right, operation)?
    {
        crate::trace_dispatch!("boolean-operation", "exact-box-cell");
        crate::trace_dispatch!("boolean-operation", "complete");
        return Ok(decisions.finish(soup));
    }
    let soup = if let Some(native) = native_meshes(meshes) {
        compute_native_boolean_with_raw_retry(
            &decisions,
            &native,
            operation,
            config,
            false,
            |computation| {
                crate::trace_dispatch!("boolean-operation", "triangulate-output");
                computation.into_boolean_mesh(&decisions, operation)
            },
        )?
    } else {
        let computation = compute_boolean(
            &decisions, meshes, operation, None, None, None, config, false,
        )?;
        crate::trace_dispatch!("boolean-operation", "triangulate-output");
        computation.into_boolean_mesh(&decisions, operation)?
    };
    crate::trace_dispatch!("boolean-operation", "complete");
    Ok(decisions.finish(soup))
}

fn compute_native_boolean(
    decisions: &DecisionContext,
    meshes: &[&TriangleMesh],
    operation: BooleanOp,
    config: EmberConfig,
    retain_winding: bool,
) -> HypermeshResult<BooleanComputation> {
    let has_retained_polygons = meshes
        .iter()
        .any(|mesh| mesh.retained_input_polygons(decisions).is_some());
    let result = compute_native_boolean_with_polygon_reuse(
        decisions,
        meshes,
        operation,
        config,
        retain_winding,
        true,
    );
    match result {
        Err(error) if has_retained_polygons && is_retryable_boolean_path_error(&error) => {
            compute_native_boolean_with_polygon_reuse(
                decisions,
                meshes,
                operation,
                config,
                retain_winding,
                false,
            )
        }
        result => result,
    }
}

fn compute_native_boolean_with_raw_retry<T>(
    decisions: &DecisionContext,
    meshes: &[&TriangleMesh],
    operation: BooleanOp,
    config: EmberConfig,
    retain_winding: bool,
    finish: impl Fn(BooleanComputation) -> HypermeshResult<T>,
) -> HypermeshResult<T> {
    let result = compute_native_boolean(decisions, meshes, operation, config, retain_winding)
        .and_then(&finish);
    match result {
        Err(error) if is_retryable_boolean_path_error(&error) => {
            crate::trace_dispatch!("boolean-operation", "retry-without-native-facts");
            let views = meshes.iter().map(|mesh| mesh.as_ref()).collect::<Vec<_>>();
            compute_boolean(
                decisions,
                &views,
                operation,
                None,
                None,
                None,
                config,
                retain_winding,
            )
            .and_then(finish)
        }
        result => result,
    }
}

fn compute_native_boolean_with_polygon_reuse(
    decisions: &DecisionContext,
    meshes: &[&TriangleMesh],
    operation: BooleanOp,
    config: EmberConfig,
    retain_winding: bool,
    reuse_polygons: bool,
) -> HypermeshResult<BooleanComputation> {
    let views = meshes.iter().map(|mesh| mesh.as_ref()).collect::<Vec<_>>();
    let convex = meshes
        .iter()
        .map(|mesh| mesh.has_certified_convex_fact(decisions))
        .collect::<Vec<_>>();
    let planes = meshes
        .iter()
        .any(|mesh| mesh.has_retained_input_plane_sources(decisions))
        .then(|| {
            meshes
                .iter()
                .map(|mesh| input_triangle_planes(decisions, mesh))
                .collect::<HypermeshResult<Vec<_>>>()
        })
        .transpose()?;
    let plane_views = planes.as_ref().map(|planes| {
        planes
            .iter()
            .map(|planes| planes.as_ref())
            .collect::<Vec<_>>()
    });
    let retained_polygons = reuse_polygons
        .then(|| {
            meshes
                .iter()
                .map(|mesh| mesh.retained_input_polygons(decisions))
                .collect::<Vec<_>>()
        })
        .filter(|polygons| polygons.iter().any(Option::is_some));
    compute_boolean(
        decisions,
        &views,
        operation,
        Some(&convex),
        plane_views.as_deref(),
        retained_polygons.as_deref(),
        config,
        retain_winding,
    )
}

fn native_meshes<'a>(meshes: &[TriangleMeshRef<'a>]) -> Option<Vec<&'a TriangleMesh>> {
    meshes.iter().map(|mesh| mesh.native).collect()
}

/// Performs one exact regularized Boolean and returns reusable native geometry.
///
/// This carrier-level entry point owns exact algebraic fast paths for empty,
/// identical, disjoint, and axis-aligned-box inputs. Inputs outside those
/// certified cases use the same general EMBER path as [`boolean_operation`].
pub fn boolean_triangle_meshes(
    context: &MeshContext,
    left: &TriangleMesh,
    right: &TriangleMesh,
    operation: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<MeshOutcome<TriangleMesh>> {
    let decisions = DecisionContext::new(context);
    let result = boolean_triangle_meshes_decision(&decisions, left, right, operation, config)?;
    Ok(decisions.finish(result))
}

fn boolean_triangle_meshes_decision(
    decisions: &DecisionContext,
    left: &TriangleMesh,
    right: &TriangleMesh,
    operation: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<TriangleMesh> {
    if left.triangles.is_empty() || right.triangles.is_empty() {
        certify_nonempty_shortcut_operand(decisions, left, 0)?;
        certify_nonempty_shortcut_operand(decisions, right, 1)?;
        return Ok(match operation {
            BooleanOp::Union | BooleanOp::SymmetricDifference => {
                if left.triangles.is_empty() {
                    right.clone()
                } else {
                    left.clone()
                }
            }
            BooleanOp::Difference => left.clone(),
            BooleanOp::Intersection => empty_triangle_mesh(),
        });
    }
    if Arc::ptr_eq(&left.positions, &right.positions)
        && Arc::ptr_eq(&left.triangles, &right.triangles)
    {
        left.certify_valid_pwn_decision(decisions, 0)?;
        return Ok(match operation {
            BooleanOp::Union | BooleanOp::Intersection => left.clone(),
            BooleanOp::Difference | BooleanOp::SymmetricDifference => empty_triangle_mesh(),
        });
    }
    if let (Some(left_bounds), Some(right_bounds)) = (
        optional_predicate_fact(left.exact_bounds_decision(decisions))?,
        optional_predicate_fact(right.exact_bounds_decision(decisions))?,
    ) && matches!(
        decisions.probe(hyperlimit::ordered_aabb3s_intersect(
            &left_bounds.mins,
            &left_bounds.maxs,
            &right_bounds.mins,
            &right_bounds.maxs,
            decisions.policy(),
        )),
        Some(false)
    ) {
        left.certify_valid_pwn_decision(decisions, 0)?;
        right.certify_valid_pwn_decision(decisions, 1)?;
        return Ok(match operation {
            BooleanOp::Union | BooleanOp::SymmetricDifference => {
                merge_triangle_meshes(&[left, right])
            }
            BooleanOp::Difference => left.clone(),
            BooleanOp::Intersection => empty_triangle_mesh(),
        });
    }
    if let (Some(left_box), Some(right_box)) = (
        optional_predicate_fact(left.axis_aligned_box_bounds_decision(decisions))?,
        optional_predicate_fact(right.axis_aligned_box_bounds_decision(decisions))?,
    ) {
        if optional_predicate_decision(aabb_contains(decisions, &left_box, &right_box))?
            == Some(true)
        {
            match operation {
                BooleanOp::Union => return Ok(left.clone()),
                BooleanOp::Intersection => return Ok(right.clone()),
                BooleanOp::Difference | BooleanOp::SymmetricDifference => {}
            }
        } else if optional_predicate_decision(aabb_contains(decisions, &right_box, &left_box))?
            == Some(true)
        {
            match operation {
                BooleanOp::Union => return Ok(right.clone()),
                BooleanOp::Intersection => return Ok(left.clone()),
                BooleanOp::Difference => return Ok(empty_triangle_mesh()),
                BooleanOp::SymmetricDifference => {}
            }
        }
        if operation == BooleanOp::Union
            && let Some(bounds) =
                optional_predicate_fact(adjacent_box_union(decisions, &left_box, &right_box))?
        {
            return Ok(box_from_bounds(&bounds));
        }
        if operation == BooleanOp::Intersection
            && let Some(bounds) =
                optional_predicate_fact(box_intersection(decisions, &left_box, &right_box))?
        {
            return Ok(box_from_bounds(&bounds));
        }
    }
    compute_native_boolean_with_raw_retry(
        decisions,
        &[left, right],
        operation,
        config,
        false,
        |computation| {
            let (mesh, provenance) =
                computation.into_native_materialization(decisions, operation)?;
            materialize_boolean_mesh(decisions, mesh, provenance)
        },
    )
}

fn certify_nonempty_shortcut_operand(
    decisions: &DecisionContext,
    mesh: &TriangleMesh,
    mesh_index: usize,
) -> HypermeshResult<()> {
    if mesh.triangles.is_empty() {
        Ok(())
    } else {
        mesh.certify_valid_pwn_decision(decisions, mesh_index)
    }
}

fn axis_aligned_box_boolean_mesh(
    decisions: &DecisionContext,
    left: &TriangleMesh,
    right: &TriangleMesh,
    operation: BooleanOp,
) -> HypermeshResult<Option<BooleanMesh>> {
    let (Some(left_bounds), Some(right_bounds)) = (
        optional_predicate_fact(left.axis_aligned_box_bounds_decision(decisions))?,
        optional_predicate_fact(right.axis_aligned_box_bounds_decision(decisions))?,
    ) else {
        return Ok(None);
    };

    match exact_box_cell_boolean_mesh(
        decisions,
        left,
        right,
        &left_bounds,
        &right_bounds,
        operation,
    ) {
        Ok(mesh) => Ok(Some(mesh)),
        Err(HypermeshError::PredicateUndecided { .. }) => Ok(None),
        Err(error) => Err(error),
    }
}

fn append_input_box_boolean_mesh(
    result: &mut BooleanMesh,
    mesh: &TriangleMesh,
    source_mesh: isize,
    source_triangle_offset: isize,
) {
    // The only callers hold fresh output and meshes accepted by the box
    // detector, which proves 8 valid vertices and 12 valid triangles.
    debug_assert_eq!(mesh.positions.len(), 8);
    debug_assert_eq!(mesh.triangles.len(), 12);
    let vertex_offset = result.vertices.len();
    result.vertices.reserve(mesh.positions.len());
    result
        .vertices
        .extend(mesh.positions.iter().map(|point| OutputVertex {
            x: point.x.clone(),
            y: point.y.clone(),
            z: point.z.clone(),
        }));
    result.triangles.reserve(mesh.triangles.len());
    result.sources.reserve(mesh.triangles.len());
    for (triangle_index, triangle) in mesh.triangles.iter().enumerate() {
        let [a, b, c] = triangle.indices();
        result
            .triangles
            .push([vertex_offset + a, vertex_offset + b, vertex_offset + c]);
        result.sources.push(TriangleSource {
            mesh: source_mesh,
            triangle: source_triangle_offset + triangle_index as isize,
            orientation: 1,
        });
    }
}

struct BoxAxisCoordinates {
    values: [Real; 4],
    len: usize,
}

fn sorted_unique_box_coordinates(
    decisions: &DecisionContext,
    mut values: [Real; 4],
) -> HypermeshResult<BoxAxisCoordinates> {
    for index in 1..values.len() {
        let mut cursor = index;
        while cursor != 0 {
            if compare_real_decision(decisions, &values[cursor], &values[cursor - 1])?.is_lt() {
                values.swap(cursor, cursor - 1);
                cursor -= 1;
            } else {
                break;
            }
        }
    }
    let mut len = 1;
    for index in 1..values.len() {
        if !compare_real_decision(decisions, &values[index], &values[len - 1])?.is_eq() {
            values.swap(len, index);
            len += 1;
        }
    }
    if decisions.certainty() == MeshCertainty::Approximate512Consumed {
        for left in 0..len {
            for right in left + 1..len {
                if !compare_real_decision(decisions, &values[left], &values[right])?.is_lt() {
                    return Err(HypermeshError::UnknownClassification);
                }
            }
        }
    }
    Ok(BoxAxisCoordinates { values, len })
}

fn box_grid_ranges(
    decisions: &DecisionContext,
    axes: &[BoxAxisCoordinates; 3],
    bounds: &hyperlattice::Aabb,
) -> HypermeshResult<[[usize; 2]; 3]> {
    let mut ranges = [[0; 2]; 3];
    for axis in 0..3 {
        for (range_index, value) in [axis_ref(&bounds.mins, axis), axis_ref(&bounds.maxs, axis)]
            .into_iter()
            .enumerate()
        {
            let mut found = None;
            for (index, candidate) in axes[axis].values[..axes[axis].len].iter().enumerate() {
                if compare_real_decision(decisions, candidate, value)?.is_eq() {
                    found = Some(index);
                    break;
                }
            }
            ranges[axis][range_index] = found.ok_or(HypermeshError::UnknownClassification)?;
        }
    }
    Ok(ranges)
}

fn box_grid_vertex(
    axes: &[BoxAxisCoordinates; 3],
    coordinates: [usize; 3],
    indices: &mut [usize; 64],
    vertices: &mut Vec<OutputVertex>,
) -> usize {
    let slot = (coordinates[0] * axes[1].len + coordinates[1]) * axes[2].len + coordinates[2];
    if indices[slot] != usize::MAX {
        return indices[slot];
    }
    let index = vertices.len();
    vertices.push(OutputVertex {
        x: axes[0].values[coordinates[0]].clone(),
        y: axes[1].values[coordinates[1]].clone(),
        z: axes[2].values[coordinates[2]].clone(),
    });
    indices[slot] = index;
    index
}

fn append_box_grid_face(
    axes: &[BoxAxisCoordinates; 3],
    ranges: &[[usize; 2]; 3],
    axis: usize,
    side: usize,
    source_mesh: u8,
    orientation: i8,
    vertex_indices: &mut [usize; 64],
    result: &mut BooleanMesh,
) {
    let first_axis = (axis + if side == 0 { 2 } else { 1 }) % 3;
    let second_axis = (axis + if side == 0 { 1 } else { 2 }) % 3;
    let mut face = [[ranges[0][0], ranges[1][0], ranges[2][0]]; 4];
    for corner in &mut face {
        corner[axis] = ranges[axis][side];
    }
    face[1][first_axis] = ranges[first_axis][1];
    face[2][first_axis] = ranges[first_axis][1];
    face[2][second_axis] = ranges[second_axis][1];
    face[3][second_axis] = ranges[second_axis][1];
    let face = face.map(|coordinates| {
        box_grid_vertex(axes, coordinates, vertex_indices, &mut result.vertices)
    });
    result
        .triangles
        .extend([[face[0], face[1], face[2]], [face[0], face[2], face[3]]]);
    result.sources.extend([
        TriangleSource {
            mesh: isize::from(source_mesh),
            triangle: -1,
            orientation,
        },
        TriangleSource {
            mesh: isize::from(source_mesh),
            triangle: -1,
            orientation,
        },
    ]);
}

fn exact_box_cell_boolean_mesh(
    decisions: &DecisionContext,
    left_mesh: &TriangleMesh,
    right_mesh: &TriangleMesh,
    left: &hyperlattice::Aabb,
    right: &hyperlattice::Aabb,
    operation: BooleanOp,
) -> HypermeshResult<BooleanMesh> {
    let axes = [
        sorted_unique_box_coordinates(
            decisions,
            [
                left.mins.x.clone(),
                left.maxs.x.clone(),
                right.mins.x.clone(),
                right.maxs.x.clone(),
            ],
        )?,
        sorted_unique_box_coordinates(
            decisions,
            [
                left.mins.y.clone(),
                left.maxs.y.clone(),
                right.mins.y.clone(),
                right.maxs.y.clone(),
            ],
        )?,
        sorted_unique_box_coordinates(
            decisions,
            [
                left.mins.z.clone(),
                left.maxs.z.clone(),
                right.mins.z.clone(),
                right.maxs.z.clone(),
            ],
        )?,
    ];
    let dimensions = [axes[0].len - 1, axes[1].len - 1, axes[2].len - 1];
    let left_ranges = box_grid_ranges(decisions, &axes, left)?;
    let right_ranges = box_grid_ranges(decisions, &axes, right)?;
    if left_ranges
        .iter()
        .chain(&right_ranges)
        .any(|range| range[0] >= range[1])
    {
        return Err(HypermeshError::UnknownClassification);
    }
    let cell_index = |x: usize, y: usize, z: usize| (x * dimensions[1] + y) * dimensions[2] + z;
    let mut cell_inputs = [0_u8; 27];
    for x in 0..dimensions[0] {
        for y in 0..dimensions[1] {
            for z in 0..dimensions[2] {
                let coordinates = [x, y, z];
                let in_box = |ranges: &[[usize; 2]; 3]| {
                    coordinates[0] >= ranges[0][0]
                        && coordinates[0] < ranges[0][1]
                        && coordinates[1] >= ranges[1][0]
                        && coordinates[1] < ranges[1][1]
                        && coordinates[2] >= ranges[2][0]
                        && coordinates[2] < ranges[2][1]
                };
                cell_inputs[cell_index(x, y, z)] =
                    u8::from(in_box(&left_ranges)) | (u8::from(in_box(&right_ranges)) << 1);
            }
        }
    }
    let is_material = |inputs: u8| match operation {
        BooleanOp::Union => inputs != 0,
        BooleanOp::Intersection => inputs == 3,
        BooleanOp::Difference => inputs == 1,
        BooleanOp::SymmetricDifference => inputs == 1 || inputs == 2,
    };
    let cells = &cell_inputs[..dimensions.into_iter().product()];
    if cells.iter().all(|inputs| !is_material(*inputs)) {
        return Ok(BooleanMesh::default());
    }
    for (source_mesh, (mesh, source_bit, source_offset)) in [
        (left_mesh, 1_u8, 0),
        (right_mesh, 2_u8, left_mesh.triangles.len()),
    ]
    .into_iter()
    .enumerate()
    {
        if cells
            .iter()
            .all(|inputs| is_material(*inputs) == (inputs & source_bit != 0))
        {
            let mut result = BooleanMesh::default();
            append_input_box_boolean_mesh(
                &mut result,
                mesh,
                source_mesh as isize,
                source_offset as isize,
            );
            return Ok(result);
        }
    }
    let mut separated = false;
    for axis in 0..3 {
        separated |= left_ranges[axis][1] < right_ranges[axis][0]
            || right_ranges[axis][1] < left_ranges[axis][0];
    }
    if separated && matches!(operation, BooleanOp::Union | BooleanOp::SymmetricDifference) {
        let mut result = BooleanMesh::default();
        append_input_box_boolean_mesh(&mut result, left_mesh, 0, 0);
        append_input_box_boolean_mesh(
            &mut result,
            right_mesh,
            1,
            left_mesh.triangles.len() as isize,
        );
        return Ok(result);
    }

    let mut material_ranges = [[usize::MAX, 0]; 3];
    for x in 0..dimensions[0] {
        for y in 0..dimensions[1] {
            for z in 0..dimensions[2] {
                if is_material(cell_inputs[cell_index(x, y, z)]) {
                    for (axis, coordinate) in [x, y, z].into_iter().enumerate() {
                        material_ranges[axis][0] = material_ranges[axis][0].min(coordinate);
                        material_ranges[axis][1] = material_ranges[axis][1].max(coordinate + 1);
                    }
                }
            }
        }
    }
    let mut material_is_box = true;
    'cells: for x in 0..dimensions[0] {
        for y in 0..dimensions[1] {
            for z in 0..dimensions[2] {
                let coordinates = [x, y, z];
                let inside_range = coordinates[0] >= material_ranges[0][0]
                    && coordinates[0] < material_ranges[0][1]
                    && coordinates[1] >= material_ranges[1][0]
                    && coordinates[1] < material_ranges[1][1]
                    && coordinates[2] >= material_ranges[2][0]
                    && coordinates[2] < material_ranges[2][1];
                if is_material(cell_inputs[cell_index(x, y, z)]) != inside_range {
                    material_is_box = false;
                    break 'cells;
                }
            }
        }
    }
    if material_is_box {
        let mut result = BooleanMesh {
            vertices: Vec::with_capacity(8),
            triangles: Vec::with_capacity(12),
            sources: Vec::with_capacity(12),
        };
        let mut vertex_indices = [usize::MAX; 64];
        for axis in 0..3 {
            for side in 0..2 {
                let mut inside = material_ranges.map(|range| range[0]);
                inside[axis] = material_ranges[axis][side] - side;
                let inputs = cell_inputs[cell_index(inside[0], inside[1], inside[2])];
                let neighbor_inputs = if material_ranges[axis][side] == side * dimensions[axis] {
                    0
                } else {
                    let mut neighbor = inside;
                    neighbor[axis] = material_ranges[axis][side] - (1 - side);
                    cell_inputs[cell_index(neighbor[0], neighbor[1], neighbor[2])]
                };
                let changed_inputs = inputs ^ neighbor_inputs;
                let source_mesh = if changed_inputs & 1 != 0 { 0_u8 } else { 1_u8 };
                let orientation = if inputs & (1_u8 << source_mesh) != 0 {
                    1
                } else {
                    -1
                };
                append_box_grid_face(
                    &axes,
                    &material_ranges,
                    axis,
                    side,
                    source_mesh,
                    orientation,
                    &mut vertex_indices,
                    &mut result,
                );
            }
        }
        return Ok(result);
    }

    let mut result = BooleanMesh {
        vertices: Vec::with_capacity(8),
        triangles: Vec::with_capacity(12),
        sources: Vec::with_capacity(12),
    };
    let mut vertex_indices = [usize::MAX; 64];
    for x in 0..dimensions[0] {
        for y in 0..dimensions[1] {
            for z in 0..dimensions[2] {
                let inputs = cell_inputs[cell_index(x, y, z)];
                if !is_material(inputs) {
                    continue;
                }
                let coordinates = [x, y, z];
                for axis in 0..3 {
                    for side in 0..2 {
                        let neighbor_coordinate = if side == 0 {
                            coordinates[axis].checked_sub(1)
                        } else {
                            (coordinates[axis] + 1 < dimensions[axis])
                                .then_some(coordinates[axis] + 1)
                        };
                        let neighbor_inputs = neighbor_coordinate
                            .map(|neighbor| {
                                let mut neighbor_coordinates = coordinates;
                                neighbor_coordinates[axis] = neighbor;
                                cell_inputs[cell_index(
                                    neighbor_coordinates[0],
                                    neighbor_coordinates[1],
                                    neighbor_coordinates[2],
                                )]
                            })
                            .unwrap_or(0);
                        if is_material(neighbor_inputs) {
                            continue;
                        }
                        let changed_inputs = inputs ^ neighbor_inputs;
                        let source_mesh = if changed_inputs & 1 != 0 { 0_u8 } else { 1_u8 };
                        let orientation = if inputs & (1_u8 << source_mesh) != 0 {
                            1
                        } else {
                            -1
                        };
                        let ranges = [[x, x + 1], [y, y + 1], [z, z + 1]];
                        append_box_grid_face(
                            &axes,
                            &ranges,
                            axis,
                            side,
                            source_mesh,
                            orientation,
                            &mut vertex_indices,
                            &mut result,
                        );
                    }
                }
            }
        }
    }
    Ok(result)
}

fn optional_predicate_fact<T>(result: HypermeshResult<Option<T>>) -> HypermeshResult<Option<T>> {
    Ok(optional_predicate_decision(result)?.flatten())
}

fn optional_predicate_decision<T>(result: HypermeshResult<T>) -> HypermeshResult<Option<T>> {
    match result {
        Ok(value) => Ok(Some(value)),
        Err(crate::error::HypermeshError::PredicateUndecided { .. }) => Ok(None),
        Err(error) => Err(error),
    }
}

fn is_retryable_boolean_path_error(error: &HypermeshError) -> bool {
    matches!(
        error,
        HypermeshError::PredicateUndecided { .. }
            | HypermeshError::UnknownClassification
            | HypermeshError::ReferencePropagationFailed
            | HypermeshError::SubdivisionDepthLimit { .. }
            | HypermeshError::OpenOutput { .. }
            | HypermeshError::OutputPlanarizationFailed { .. }
            | HypermeshError::PointAtInfinity
    )
}

fn retry_boolean_path<T>(
    result: HypermeshResult<T>,
    fallback: impl FnOnce() -> HypermeshResult<T>,
) -> HypermeshResult<T> {
    match result {
        Err(error) if is_retryable_boolean_path_error(&error) => {
            if std::env::var_os("HYPERMESH_OUTPUT_DIAGNOSTIC").is_some() {
                eprintln!("retrying boolean path after: {error:?}");
            }
            fallback()
        }
        result => result,
    }
}

fn input_triangle_planes(
    decisions: &DecisionContext,
    mesh: &TriangleMesh,
) -> HypermeshResult<Vec<InputTrianglePlanes>> {
    if let Some(planes) = mesh.retained_input_planes(decisions)? {
        return Ok(planes);
    }
    mesh.triangles
        .iter()
        .map(|triangle| {
            let [a, b, c] = triangle.indices();
            InputTrianglePlanes::from_points_decision(
                decisions,
                &mesh.positions[a],
                &mesh.positions[b],
                &mesh.positions[c],
            )
        })
        .collect()
}

fn materialize_boolean_mesh(
    decisions: &DecisionContext,
    result: crate::output::BooleanMesh,
    polygons: Vec<ConvexPolygon>,
) -> HypermeshResult<TriangleMesh> {
    if result.triangles.len() != result.sources.len() {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
    let sources = result.sources.clone();
    let mesh = result.into_triangle_mesh();
    Ok(mesh.with_boolean_provenance(sources, polygons, decisions.certainty()))
}

fn empty_triangle_mesh() -> TriangleMesh {
    TriangleMesh::new(Vec::new(), Vec::new())
}

fn merge_triangle_meshes(meshes: &[&TriangleMesh]) -> TriangleMesh {
    let position_count = meshes.iter().map(|mesh| mesh.positions.len()).sum();
    let triangle_count = meshes.iter().map(|mesh| mesh.triangles.len()).sum();
    let mut positions = Vec::with_capacity(position_count);
    let mut triangles = Vec::with_capacity(triangle_count);
    for mesh in meshes {
        let base = positions.len();
        positions.extend(mesh.positions.iter().cloned());
        triangles.extend(mesh.triangles.iter().map(|triangle| {
            let [a, b, c] = triangle.indices();
            Triangle::new(base + a, base + b, base + c)
        }));
    }
    TriangleMesh::new(positions, triangles)
}

fn aabb_contains(
    decisions: &DecisionContext,
    outer: &hyperlattice::Aabb,
    inner: &hyperlattice::Aabb,
) -> HypermeshResult<bool> {
    for axis in 0..3 {
        if compare_real_decision(
            decisions,
            axis_ref(&outer.mins, axis),
            axis_ref(&inner.mins, axis),
        )?
        .is_gt()
            || compare_real_decision(
                decisions,
                axis_ref(&outer.maxs, axis),
                axis_ref(&inner.maxs, axis),
            )?
            .is_lt()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn adjacent_box_union(
    decisions: &DecisionContext,
    left: &hyperlattice::Aabb,
    right: &hyperlattice::Aabb,
) -> HypermeshResult<Option<hyperlattice::Aabb>> {
    const fn coordinate(point: &Point3, axis: usize) -> &Real {
        match axis {
            0 => &point.x,
            1 => &point.y,
            _ => &point.z,
        }
    }

    const fn coordinate_mut(point: &mut Point3, axis: usize) -> &mut Real {
        match axis {
            0 => &mut point.x,
            1 => &mut point.y,
            _ => &mut point.z,
        }
    }

    for axis in 0..3 {
        let mut identical_other_axes = true;
        for other in (0..3).filter(|other| *other != axis) {
            if !compare_real_decision(
                decisions,
                coordinate(&left.mins, other),
                coordinate(&right.mins, other),
            )?
            .is_eq()
                || !compare_real_decision(
                    decisions,
                    coordinate(&left.maxs, other),
                    coordinate(&right.maxs, other),
                )?
                .is_eq()
            {
                identical_other_axes = false;
                break;
            }
        }
        if identical_other_axes
            && compare_real_decision(
                decisions,
                coordinate(&left.mins, axis),
                coordinate(&right.maxs, axis),
            )?
            .is_le()
            && compare_real_decision(
                decisions,
                coordinate(&right.mins, axis),
                coordinate(&left.maxs, axis),
            )?
            .is_le()
        {
            let mut bounds = left.clone();
            *coordinate_mut(&mut bounds.mins, axis) = if compare_real_decision(
                decisions,
                coordinate(&left.mins, axis),
                coordinate(&right.mins, axis),
            )?
            .is_le()
            {
                coordinate(&left.mins, axis).clone()
            } else {
                coordinate(&right.mins, axis).clone()
            };
            *coordinate_mut(&mut bounds.maxs, axis) = if compare_real_decision(
                decisions,
                coordinate(&left.maxs, axis),
                coordinate(&right.maxs, axis),
            )?
            .is_ge()
            {
                coordinate(&left.maxs, axis).clone()
            } else {
                coordinate(&right.maxs, axis).clone()
            };
            return Ok(Some(bounds));
        }
    }
    Ok(None)
}

fn box_intersection(
    decisions: &DecisionContext,
    left: &hyperlattice::Aabb,
    right: &hyperlattice::Aabb,
) -> HypermeshResult<Option<hyperlattice::Aabb>> {
    let mut mins = Point3::origin();
    let mut maxs = Point3::origin();
    for axis in 0..3 {
        *axis_mut(&mut mins, axis) = if compare_real_decision(
            decisions,
            axis_ref(&left.mins, axis),
            axis_ref(&right.mins, axis),
        )?
        .is_ge()
        {
            axis_ref(&left.mins, axis).clone()
        } else {
            axis_ref(&right.mins, axis).clone()
        };
        *axis_mut(&mut maxs, axis) = if compare_real_decision(
            decisions,
            axis_ref(&left.maxs, axis),
            axis_ref(&right.maxs, axis),
        )?
        .is_le()
        {
            axis_ref(&left.maxs, axis).clone()
        } else {
            axis_ref(&right.maxs, axis).clone()
        };
        if !compare_real_decision(decisions, axis_ref(&mins, axis), axis_ref(&maxs, axis))?.is_lt()
        {
            return Ok(None);
        }
    }
    Ok(Some(hyperlattice::Aabb::new(mins, maxs)))
}

fn box_from_bounds(bounds: &hyperlattice::Aabb) -> TriangleMesh {
    let [min_x, min_y, min_z] = [
        bounds.mins.x.clone(),
        bounds.mins.y.clone(),
        bounds.mins.z.clone(),
    ];
    let [max_x, max_y, max_z] = [
        bounds.maxs.x.clone(),
        bounds.maxs.y.clone(),
        bounds.maxs.z.clone(),
    ];
    TriangleMesh::new(
        vec![
            Point3::new(min_x.clone(), min_y.clone(), min_z.clone()),
            Point3::new(max_x.clone(), min_y.clone(), min_z.clone()),
            Point3::new(max_x.clone(), max_y.clone(), min_z.clone()),
            Point3::new(min_x.clone(), max_y.clone(), min_z),
            Point3::new(min_x.clone(), min_y.clone(), max_z.clone()),
            Point3::new(max_x.clone(), min_y, max_z.clone()),
            Point3::new(max_x, max_y.clone(), max_z.clone()),
            Point3::new(min_x, max_y, max_z),
        ],
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 3, 2),
            Triangle::new(4, 5, 6),
            Triangle::new(4, 6, 7),
            Triangle::new(0, 1, 5),
            Triangle::new(0, 5, 4),
            Triangle::new(1, 2, 6),
            Triangle::new(1, 6, 5),
            Triangle::new(2, 3, 7),
            Triangle::new(2, 7, 6),
            Triangle::new(3, 0, 4),
            Triangle::new(3, 4, 7),
        ],
    )
}

fn compute_boolean(
    decisions: &DecisionContext,
    meshes: &[TriangleMeshRef<'_>],
    operation: BooleanOp,
    certified_convex_inputs: Option<&[bool]>,
    input_planes: Option<&[&[InputTrianglePlanes]]>,
    retained_polygons: Option<&[Option<&[ConvexPolygon]>]>,
    config: EmberConfig,
    retain_winding: bool,
) -> HypermeshResult<BooleanComputation> {
    if certified_convex_inputs.is_some_and(|certified| certified.len() != meshes.len()) {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
    let certified_convex_inputs = certified_convex_inputs.unwrap_or(&[]);
    let use_two_convex_candidate = meshes.len() == 2 && certified_convex_inputs == [true, true];
    let mut retryable_convex_candidate_error = None;
    if use_two_convex_candidate && retained_polygons.is_none() {
        match build_projective_input_soup(decisions, meshes, input_planes) {
            Ok(projective_input) => {
                match compute_projective_input_soup(
                    decisions,
                    &projective_input,
                    input_planes,
                    operation,
                    retain_winding || operation == BooleanOp::SymmetricDifference,
                ) {
                    Ok(Some(candidate)) => {
                        return Ok(BooleanComputation {
                            soup: crate::mesh::PolygonSoup {
                                polygons: Vec::new(),
                                bounds: projective_input.bounds,
                                num_meshes: projective_input.meshes.len(),
                            },
                            classified: candidate.classified,
                            boolean_mesh: Some(candidate.boolean_mesh),
                            input_edges_deferred: true,
                        });
                    }
                    Ok(None) => {}
                    Err(error) if is_retryable_boolean_path_error(&error) => {
                        if cfg!(debug_assertions) {
                            eprintln!(
                                "[DEBUG] compact projective convex candidate failed: {error}"
                            );
                        }
                        retryable_convex_candidate_error = Some(error);
                    }
                    Err(error) => return Err(error),
                }
            }
            Err(error) if is_retryable_boolean_path_error(&error) => {
                if cfg!(debug_assertions) {
                    eprintln!("[DEBUG] compact projective input preparation failed: {error}");
                }
            }
            Err(error) => return Err(error),
        }
    }
    let mut soup = if certified_convex_inputs.is_empty() {
        build_polygon_soup_internal(decisions, meshes, None, input_planes)?
    } else {
        build_polygon_soup_with_certified_convex_inputs(
            decisions,
            meshes,
            certified_convex_inputs,
            input_planes,
        )?
    };
    if let Some(retained_polygons) = retained_polygons {
        replace_retained_input_polygons(decisions, &mut soup, retained_polygons)?;
    }
    let convex_candidate = if use_two_convex_candidate {
        match compute_two_convex_inputs_projectively(
            decisions,
            &soup.polygons,
            operation,
            retain_winding || operation == BooleanOp::SymmetricDifference,
        ) {
            Ok(candidate) => candidate,
            Err(error) if is_retryable_boolean_path_error(&error) => {
                if cfg!(debug_assertions) {
                    eprintln!("[DEBUG] projective convex candidate failed: {error}");
                }
                retryable_convex_candidate_error = Some(error);
                None
            }
            Err(error) => return Err(error),
        }
    } else {
        None
    };
    if convex_candidate.is_none()
        && let Some(error) = retryable_convex_candidate_error
    {
        return Err(error);
    }
    let (classified, boolean_mesh, input_edges_deferred) = if let Some(candidate) = convex_candidate
    {
        (candidate.classified, Some(candidate.boolean_mesh), true)
    } else {
        let process_bounds = expanded_bounds(&soup.bounds);
        let ref_point = outside_reference_point(&process_bounds);
        let ref_wnv = vec![0; soup.num_meshes];
        (
            crate::subdivision::subdivide_boolean_with_certified_convex_inputs(
                decisions,
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
        boolean_mesh,
        input_edges_deferred,
    })
}

fn replace_retained_input_polygons(
    decisions: &DecisionContext,
    soup: &mut crate::mesh::PolygonSoup,
    retained: &[Option<&[ConvexPolygon]>],
) -> HypermeshResult<()> {
    if retained.len() != soup.num_meshes {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
    for (mesh_index, retained) in retained.iter().enumerate() {
        let Some(retained) = retained else {
            continue;
        };
        soup.polygons
            .retain(|polygon| polygon.mesh_index != mesh_index as isize);
        for polygon in retained.iter().cloned() {
            let mut polygon = polygon_with_geometric_edge_halfspaces(decisions, polygon)?;
            polygon.mesh_index = mesh_index as isize;
            polygon.delta_w = vec![0; soup.num_meshes];
            polygon.delta_w[mesh_index] = 1;
            polygon.known_identities = None;
            soup.polygons.push(polygon);
        }
    }
    soup.polygons.sort_by_key(|polygon| polygon.mesh_index);
    for (polygon_index, polygon) in soup.polygons.iter_mut().enumerate() {
        polygon.polygon_index =
            isize::try_from(polygon_index).map_err(|_| HypermeshError::UnknownClassification)?;
    }
    Ok(())
}

fn polygon_with_geometric_edge_halfspaces(
    decisions: &DecisionContext,
    mut polygon: ConvexPolygon,
) -> HypermeshResult<ConvexPolygon> {
    let vertices = polygon.vertices_decision(decisions)?;
    if vertices.is_empty() {
        return Err(HypermeshError::UnknownClassification);
    }
    let mut sum = Point3::origin();
    for vertex in &vertices {
        sum.x += vertex.x.clone();
        sum.y += vertex.y.clone();
        sum.z += vertex.z.clone();
    }
    let count = Real::from(
        u64::try_from(vertices.len()).map_err(|_| HypermeshError::UnknownClassification)?,
    );
    let interior = Point3::new(
        (sum.x / count.clone()).map_err(|_| HypermeshError::UnknownClassification)?,
        (sum.y / count.clone()).map_err(|_| HypermeshError::UnknownClassification)?,
        (sum.z / count).map_err(|_| HypermeshError::UnknownClassification)?,
    );
    polygon.edges = Arc::new(
        polygon
            .edges
            .iter()
            .map(
                |edge| match classify_point_decision(decisions, &interior, edge)? {
                    Classification::Negative => Ok(edge.clone()),
                    Classification::Positive => Ok(edge.inverted()),
                    Classification::On => Err(HypermeshError::UnknownClassification),
                },
            )
            .collect::<HypermeshResult<Vec<_>>>()?,
    );
    Ok(polygon)
}

#[derive(Clone)]
struct ProjectiveCycle {
    boundary: Vec<ProjectiveBoundaryEntry>,
    support_index: usize,
    source_plane: ConstructionPlaneIdentity,
    source_unchanged: bool,
}

#[derive(Clone)]
struct ProjectiveBoundaryEntry {
    // Clipping moves this complete incidence record atomically. Keeping the
    // point and outgoing edge evidence together also makes length skew between
    // parallel identity arrays unrepresentable.
    point_index: usize,
    evidence: ProjectivePointEvidence,
    point_identity: ConstructionVertexIdentity,
    // Exact edge planes are single-owned by the computation arena. Moving an
    // index with its identity preserves the complete oriented incidence record.
    edge_index: usize,
    edge_identity: ConstructionEdgeIdentity,
}

#[derive(Clone, Copy)]
struct ProjectivePointEvidence {
    approximate: Option<[f64; 3]>,
    rational_filter_query: Option<RationalLinearForm4Query>,
}

#[derive(Clone, Debug, Eq, Hash, PartialEq)]
struct ProjectiveBoundaryPlaneIdentity {
    support: ConstructionPlaneIdentity,
    edge: ConstructionEdgeIdentity,
}

struct ProjectiveClip {
    negative: ProjectiveCycle,
    positive: ProjectiveCycle,
    side: ProjectiveClipSide,
}

struct ProjectiveBoundary {
    entries: Vec<ProjectiveBoundaryEntry>,
}

impl ProjectiveBoundary {
    fn with_capacity(capacity: usize) -> Self {
        Self {
            entries: Vec::with_capacity(capacity),
        }
    }

    fn push(
        &mut self,
        point_index: usize,
        evidence: ProjectivePointEvidence,
        point_identity: ConstructionVertexIdentity,
        edge_index: usize,
        edge_identity: ConstructionEdgeIdentity,
        point_cache: &ProjectivePointCache,
    ) {
        // Approximation inequality is only a rejection filter: equal exact
        // points have the same retained finite view. Deduplication still
        // requires exact homogeneous equality below.
        let approximations_differ = self
            .entries
            .last()
            .and_then(|last| last.evidence.approximate)
            .zip(evidence.approximate)
            .is_some_and(|(last, current)| last != current);
        if !approximations_differ
            && self.entries.last().is_some_and(|last| {
                point_cache.point(last.point_index) == point_cache.point(point_index)
            })
        {
            let last = self
                .entries
                .last_mut()
                .expect("nonempty boundary has a last entry");
            last.edge_index = edge_index;
            last.edge_identity = edge_identity;
            return;
        }
        self.entries.push(ProjectiveBoundaryEntry {
            point_index,
            evidence,
            point_identity,
            edge_index,
            edge_identity,
        });
    }

    fn into_cycle(
        mut self,
        support_index: usize,
        source_plane: ConstructionPlaneIdentity,
        point_cache: &ProjectivePointCache,
    ) -> ProjectiveCycle {
        if self.entries.len() > 1
            && self
                .entries
                .first()
                .zip(self.entries.last())
                .is_some_and(|(first, last)| {
                    point_cache.point(first.point_index) == point_cache.point(last.point_index)
                })
        {
            self.entries.pop();
        }
        ProjectiveCycle {
            boundary: self.entries,
            support_index,
            source_plane,
            source_unchanged: false,
        }
    }
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

type SourceEdgeKey = [usize; 3];

struct ProjectiveSourceEdge {
    definition_planes: [usize; 2],
    supports: [ConstructionPlaneIdentity; 2],
    support_count: u8,
}

impl ProjectiveSourceEdge {
    fn new(definition_planes: [usize; 2], support: ConstructionPlaneIdentity) -> Self {
        Self {
            definition_planes,
            supports: [support; 2],
            support_count: 1,
        }
    }

    fn supports(&self) -> &[ConstructionPlaneIdentity] {
        &self.supports[..usize::from(self.support_count)]
    }

    fn insert_support(&mut self, support: ConstructionPlaneIdentity) -> bool {
        if self.supports().contains(&support) {
            return true;
        }
        if self.support_count == 2 {
            return false;
        }
        self.supports[usize::from(self.support_count)] = support;
        self.support_count += 1;
        true
    }
}

#[derive(Default)]
struct ProjectivePointCache {
    // Cycles retain stable indices instead of cloning 192-byte exact points
    // and planes. Identity remapping can therefore update a complete
    // coincidence class atomically, while split cycles share exact geometry.
    point_storage: Vec<HomogeneousPoint3>,
    plane_storage: Vec<Plane>,
    points: StorageHashMap<ConstructionVertexIdentity, CachedProjectivePoint>,
    canonical_identities: StorageHashMap<ConstructionVertexIdentity, ConstructionVertexIdentity>,
    canonical_planes: [Vec<ConstructionPlaneIdentity>; 2],
    planes: StorageHashMap<ConstructionPlaneIdentity, usize>,
    source_edges: StorageHashMap<SourceEdgeKey, ProjectiveSourceEdge>,
    boundary_planes: StorageHashMap<ProjectiveBoundaryPlaneIdentity, Vec<usize>>,
    inverted_planes: StorageHashMap<ConstructionPlaneIdentity, usize>,
    point_incidences: StorageHashMap<ConstructionVertexIdentity, Vec<ConstructionPlaneIdentity>>,
}

struct CachedProjectivePoint {
    point_index: usize,
    approximate: Option<[f64; 3]>,
}

const PROJECTIVE_CROSSING_CACHE_MIN_POINTS: usize = 128;

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
    fn source_edge_key(edge: &ConstructionEdgeIdentity) -> Option<SourceEdgeKey> {
        let ConstructionEdgeIdentity::Source { mesh, endpoints } = edge else {
            return None;
        };
        Some([*mesh, endpoints[0], endpoints[1]])
    }

    fn point(&self, index: usize) -> &HomogeneousPoint3 {
        self.point_storage
            .get(index)
            .expect("projective point index is retained for the computation")
    }

    fn plane(&self, index: usize) -> &Plane {
        self.plane_storage
            .get(index)
            .expect("projective plane index is retained for the computation")
    }

    fn support_plane_index(&mut self, identity: ConstructionPlaneIdentity, plane: &Plane) -> usize {
        let identity = self.canonical_plane_identity(identity);
        if let Some(&index) = self.planes.get(&identity) {
            return index;
        }
        let index = self.plane_storage.len();
        self.plane_storage.push(plane.clone());
        self.planes.insert(identity, index);
        index
    }

    fn boundary_plane_index(
        &mut self,
        support: ConstructionPlaneIdentity,
        edge: &ConstructionEdgeIdentity,
        plane: &Plane,
    ) -> usize {
        let identity = ProjectiveBoundaryPlaneIdentity {
            support: self.canonical_plane_identity(support),
            edge: edge.clone(),
        };
        if let Some(indices) = self.boundary_planes.get(&identity) {
            for &index in indices {
                if self.plane(index) == plane {
                    return index;
                }
            }
        }
        let index = self.plane_storage.len();
        self.plane_storage.push(plane.clone());
        self.boundary_planes
            .entry(identity)
            .or_default()
            .push(index);
        index
    }

    fn inverted_plane_index(&mut self, identity: ConstructionPlaneIdentity) -> usize {
        let identity = self.canonical_plane_identity(identity);
        if let Some(&index) = self.inverted_planes.get(&identity) {
            return index;
        }
        let plane = self
            .planes
            .get(&identity)
            .map(|&index| self.plane(index).inverted())
            .expect("canonical support plane is retained for projective clipping");
        let index = self.plane_storage.len();
        self.plane_storage.push(plane);
        self.inverted_planes.insert(identity, index);
        index
    }

    fn canonical_plane_identity(
        &self,
        identity: ConstructionPlaneIdentity,
    ) -> ConstructionPlaneIdentity {
        self.canonical_planes[identity.mesh]
            .get(identity.plane)
            .copied()
            .unwrap_or(identity)
    }

    fn edge_plane_intersection_identity(
        &self,
        edge: &ConstructionEdgeIdentity,
        plane: ConstructionPlaneIdentity,
    ) -> ConstructionVertexIdentity {
        let plane = self.canonical_plane_identity(plane);
        if let Some(key) = Self::source_edge_key(edge)
            && let Some(source_edge) = self.source_edges.get(&key)
            && source_edge.support_count >= 2
        {
            let supports = source_edge.supports();
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

    fn has_incidence(
        &self,
        identity: &ConstructionVertexIdentity,
        plane: ConstructionPlaneIdentity,
    ) -> bool {
        let plane = self.canonical_plane_identity(plane);
        if self
            .point_incidences
            .get(identity)
            .is_some_and(|incidences| incidences.contains(&plane))
        {
            return true;
        }
        let canonical = self.canonical_vertex_identity(identity);
        canonical != *identity
            && self
                .point_incidences
                .get(&canonical)
                .is_some_and(|incidences| incidences.contains(&plane))
    }

    fn canonical_vertex_identity(
        &self,
        identity: &ConstructionVertexIdentity,
    ) -> ConstructionVertexIdentity {
        self.canonical_identities
            .get(identity)
            .cloned()
            .unwrap_or_else(|| identity.clone())
    }

    fn record_definition_incidences(&mut self, identity: &ConstructionVertexIdentity) {
        match identity {
            ConstructionVertexIdentity::Source { .. } => {}
            ConstructionVertexIdentity::SourceEdgePlane {
                mesh,
                endpoints,
                plane,
            } => {
                let key = [*mesh, endpoints[0], endpoints[1]];
                let supports = self
                    .source_edges
                    .get(&key)
                    .map(|source_edge| (source_edge.supports, source_edge.support_count));
                if let Some((supports, support_count)) = supports {
                    for support in &supports[..usize::from(support_count)] {
                        self.record_incidence(identity, *support);
                    }
                }
                self.record_incidence(identity, *plane);
            }
            ConstructionVertexIdentity::PlaneTriple { planes } => {
                for plane in planes {
                    self.record_incidence(identity, *plane);
                }
            }
        }
    }

    fn definition_planes(&self, identity: &ConstructionVertexIdentity) -> Option<[&Plane; 3]> {
        match identity {
            ConstructionVertexIdentity::Source { .. } => None,
            ConstructionVertexIdentity::SourceEdgePlane {
                mesh,
                endpoints,
                plane,
            } => {
                let source_edge = self
                    .source_edges
                    .get(&[*mesh, endpoints[0], endpoints[1]])?;
                let [support, boundary] = source_edge.definition_planes;
                Some([
                    self.plane(support),
                    self.plane(boundary),
                    self.plane(*self.planes.get(plane)?),
                ])
            }
            ConstructionVertexIdentity::PlaneTriple { planes } => Some([
                self.plane(*self.planes.get(&planes[0])?),
                self.plane(*self.planes.get(&planes[1])?),
                self.plane(*self.planes.get(&planes[2])?),
            ]),
        }
    }

    /// Classify a constructed positive-weight point through its compact plane
    /// triple instead of its recursively expanded crossing coordinates.
    ///
    /// Projective clipping constructs crossings as positive combinations of
    /// positive-weight endpoints. `intersect_three_planes` may choose the
    /// opposite homogeneous orientation, so its plane-side result is inverted
    /// when its weight is negative. Both weight and side still go through the
    /// centralized predicate policy; this only changes the expression supplied
    /// to that cascade.
    fn classify_definition_against_plane(
        &self,
        decisions: &DecisionContext,
        identity: &ConstructionVertexIdentity,
        plane: &Plane,
    ) -> Option<HypermeshResult<Classification>> {
        let definitions = self.definition_planes(identity)?;
        let defined = intersect_three_planes(definitions[0], definitions[1], definitions[2]);
        let weight = match crate::predicate::classify_real(decisions, &defined.w) {
            Ok(Classification::Negative) => -1_i8,
            Ok(Classification::Positive) => 1_i8,
            Ok(Classification::On) | Err(_) => return None,
        };
        Some(
            classify_projective_point_decision(decisions, &defined, plane).map(|classification| {
                if weight > 0 {
                    classification
                } else {
                    match classification {
                        Classification::Negative => Classification::Positive,
                        Classification::On => Classification::On,
                        Classification::Positive => Classification::Negative,
                    }
                }
            }),
        )
    }

    fn identities_certifiably_equal(
        &self,
        decisions: &DecisionContext,
        left_identity: &ConstructionVertexIdentity,
        left: &HomogeneousPoint3,
        right_identity: &ConstructionVertexIdentity,
        right: &HomogeneousPoint3,
    ) -> bool {
        let left_definition = self.definition_planes(left_identity);
        let right_definition = self.definition_planes(right_identity);
        let left_compact = left_definition
            .as_ref()
            .map(|planes| intersect_three_planes(planes[0], planes[1], planes[2]));
        let right_compact = right_definition
            .as_ref()
            .map(|planes| intersect_three_planes(planes[0], planes[1], planes[2]));
        let left_for_equality = left_compact.as_ref().unwrap_or(left);
        let right_for_equality = right_compact.as_ref().unwrap_or(right);
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
            && projective_points_certifiably_equal(decisions, left_for_equality, right_for_equality)
        {
            return true;
        }
        let point_satisfies = |point: &HomogeneousPoint3, definition: &[&Plane; 3]| {
            let defined = intersect_three_planes(definition[0], definition[1], definition[2]);
            [&defined.x, &defined.y, &defined.z, &defined.w]
                .into_iter()
                .any(|coordinate| {
                    matches!(
                        crate::predicate::classify_real(decisions, coordinate),
                        Ok(Classification::Negative | Classification::Positive)
                    )
                })
                && definition.iter().all(|plane| {
                    crate::predicate::classify_real(
                        decisions,
                        &homogeneous_point_plane_expression(point, *plane),
                    ) == Ok(Classification::On)
                })
        };
        match (left_definition.as_ref(), right_definition.as_ref()) {
            (Some(definition), None) => point_satisfies(right, definition),
            (None, Some(definition)) => point_satisfies(left, definition),
            (None, None) => projective_points_certifiably_equal(decisions, left, right),
            (Some(_), Some(_)) => projective_points_certifiably_equal(
                decisions,
                left_for_equality,
                right_for_equality,
            ),
        }
    }

    fn resolve_vertex_coincidences(&mut self, decisions: &DecisionContext) {
        let mut entries = self
            .points
            .drain()
            .map(|(identity, cached)| (identity, cached.point_index))
            .collect::<Vec<_>>();
        entries.sort_unstable_by(|left, right| left.0.cmp(&right.0));

        let mut sets = AtomicDisjointSets::new(entries.len());
        // Construction can retain multiple identities for the same coordinate.
        // Three quarters avoids selecting the next table size when coincidences
        // are present; an all-unique input needs at most one growth.
        let fingerprint_capacity = entries.len().saturating_sub(entries.len() / 4);
        let mut fingerprint_buckets: StorageHashMap<[u64; 3], (usize, Vec<usize>)> =
            StorageHashMap::with_capacity_and_hasher(fingerprint_capacity, Default::default());
        let mut unkeyed: Vec<usize> = Vec::new();
        for right in 0..entries.len() {
            let exact_key = exact_projective_affine_fingerprint(self.point(entries[right].1));
            if let Some(key) = exact_key {
                if let Some((first, collisions)) = fingerprint_buckets.get_mut(&key) {
                    let mut matched = false;
                    for left in std::iter::once(*first).chain(collisions.iter().copied()) {
                        if self.identities_certifiably_equal(
                            decisions,
                            &entries[left].0,
                            self.point(entries[left].1),
                            &entries[right].0,
                            self.point(entries[right].1),
                        ) {
                            sets.merge(left, right);
                            matched = true;
                            break;
                        }
                    }
                    if !matched {
                        collisions.push(right);
                    }
                } else {
                    fingerprint_buckets.insert(key, (right, Vec::new()));
                }
                for &left in &unkeyed {
                    if self.identities_certifiably_equal(
                        decisions,
                        &entries[left].0,
                        self.point(entries[left].1),
                        &entries[right].0,
                        self.point(entries[right].1),
                    ) {
                        sets.merge(left, right);
                    }
                }
                continue;
            }
            for left in 0..right {
                if self.identities_certifiably_equal(
                    decisions,
                    &entries[left].0,
                    self.point(entries[left].1),
                    &entries[right].0,
                    self.point(entries[right].1),
                ) {
                    sets.merge(left, right);
                }
            }
            unkeyed.push(right);
        }

        let representatives = (0..entries.len())
            .map(|index| sets.representative(index))
            .collect::<Vec<_>>();
        let mut merged_classes = vec![false; entries.len()];
        for (index, &representative) in representatives.iter().enumerate() {
            if index != representative {
                merged_classes[representative] = true;
            }
        }
        let mut class_incidences = vec![Vec::new(); entries.len()];
        for (index, (identity, _)) in entries.iter().enumerate() {
            let representative = representatives[index];
            if !merged_classes[representative] {
                continue;
            }
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
        for (index, (identity, _)) in entries.iter().enumerate() {
            let representative = representatives[index];
            let canonical_identity = entries[representative].0.clone();
            if *identity != canonical_identity {
                self.canonical_identities
                    .insert(identity.clone(), canonical_identity);
            }
            if merged_classes[representative] {
                self.point_incidences
                    .insert(identity.clone(), class_incidences[representative].clone());
            }
        }
        for (identity, point_index) in entries {
            self.points.insert(
                identity,
                CachedProjectivePoint {
                    point_index,
                    approximate: None,
                },
            );
        }
        for (identity, canonical_identity) in &self.canonical_identities {
            let canonical_point_index = self
                .points
                .get(canonical_identity)
                .expect("canonical projective point is available")
                .point_index;
            self.points.insert(
                identity.clone(),
                CachedProjectivePoint {
                    point_index: canonical_point_index,
                    approximate: None,
                },
            );
        }
    }

    fn intern_with_approximation(
        &mut self,
        identity: ConstructionVertexIdentity,
        point: HomogeneousPoint3,
    ) -> (usize, Option<[f64; 3]>, ConstructionVertexIdentity) {
        self.intern_with_approximation_by(identity, || point)
    }

    fn intern_with_approximation_by(
        &mut self,
        identity: ConstructionVertexIdentity,
        make_point: impl FnOnce() -> HomogeneousPoint3,
    ) -> (usize, Option<[f64; 3]>, ConstructionVertexIdentity) {
        self.intern_with_optional_approximation_by(identity, None, make_point)
    }

    fn intern_with_known_approximation_by(
        &mut self,
        identity: ConstructionVertexIdentity,
        approximate: Option<[f64; 3]>,
        make_point: impl FnOnce() -> HomogeneousPoint3,
    ) -> (usize, Option<[f64; 3]>, ConstructionVertexIdentity) {
        self.intern_with_optional_approximation_by(identity, Some(approximate), make_point)
    }

    fn intern_with_optional_approximation_by(
        &mut self,
        identity: ConstructionVertexIdentity,
        known_approximate: Option<Option<[f64; 3]>>,
        make_point: impl FnOnce() -> HomogeneousPoint3,
    ) -> (usize, Option<[f64; 3]>, ConstructionVertexIdentity) {
        self.record_definition_incidences(&identity);
        if let Some(existing) = self.points.get(&identity) {
            return (existing.point_index, existing.approximate, identity);
        }
        let point = make_point();
        let approximate = known_approximate.unwrap_or_else(|| projective_point_f64(&point));
        let point_index = self.point_storage.len();
        self.point_storage.push(point);
        self.points.insert(
            identity.clone(),
            CachedProjectivePoint {
                point_index,
                approximate,
            },
        );
        (point_index, approximate, identity)
    }
}

const PROJECTIVE_FINGERPRINT_MODULUS: u64 = (1_u64 << 61) - 1;

/// Return an exact modular affine fingerprint for a finite rational point.
///
/// Rational equality is preserved by reduction in the prime field, so two
/// distinct fingerprints certify inequality without allocating normalized
/// affine rationals. Matching fingerprints are only candidates: they still
/// take the complete policy-aware equality path, making modular collisions
/// harmless. A coordinate whose denominator, or a weight whose numerator,
/// vanishes modulo the prime declines the filter and retains all-pairs checks.
fn exact_projective_affine_fingerprint(point: &HomogeneousPoint3) -> Option<[u64; 3]> {
    let [Some(x), Some(y), Some(z), Some(weight)] = [
        point.x.exact_rational_ref(),
        point.y.exact_rational_ref(),
        point.z.exact_rational_ref(),
        point.w.exact_rational_ref(),
    ] else {
        return None;
    };
    if weight.is_zero() {
        return None;
    }
    let weight = exact_rational_modulus(weight)?;
    let inverse_weight = modular_inverse(weight)?;
    Some([
        modular_product(exact_rational_modulus(x)?, inverse_weight),
        modular_product(exact_rational_modulus(y)?, inverse_weight),
        modular_product(exact_rational_modulus(z)?, inverse_weight),
    ])
}

/// Return an order-independent exact modular fingerprint of a retained affine
/// vertex set. Different fingerprints certify that two rational polygon
/// cycles cannot be equal; matching fingerprints remain collision candidates
/// for the complete policy-aware cycle comparison.
fn exact_polygon_vertex_set_fingerprint(polygon: &ConvexPolygon) -> Option<[u64; 4]> {
    let vertices = polygon.known_vertices.as_ref()?;
    let mut fingerprint = [u64::try_from(vertices.len()).ok()?, 0, 0, 0];
    for point in vertices.iter() {
        let [x, y, z] = [
            exact_rational_modulus(point.x.exact_rational_ref()?)?,
            exact_rational_modulus(point.y.exact_rational_ref()?)?,
            exact_rational_modulus(point.z.exact_rational_ref()?)?,
        ];
        let mut mixed = x
            .wrapping_add(y.rotate_left(21))
            .wrapping_add(z.rotate_left(42));
        mixed = (mixed ^ (mixed >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        mixed = (mixed ^ (mixed >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        mixed ^= mixed >> 31;
        fingerprint[1] = fingerprint[1].wrapping_add(mixed);
        fingerprint[2] ^= mixed;
        fingerprint[3] = fingerprint[3].wrapping_add(mixed.wrapping_mul(mixed));
    }
    Some(fingerprint)
}

fn exact_rational_modulus(value: &Rational) -> Option<u64> {
    let mut numerator = 0_u64;
    for digit in value.numerator().iter_u64_digits().rev() {
        numerator = ((((numerator as u128) << 64) + u128::from(digit))
            % u128::from(PROJECTIVE_FINGERPRINT_MODULUS)) as u64;
    }
    let mut denominator = 0_u64;
    for digit in value.denominator().iter_u64_digits().rev() {
        denominator = ((((denominator as u128) << 64) + u128::from(digit))
            % u128::from(PROJECTIVE_FINGERPRINT_MODULUS)) as u64;
    }
    let inverse_denominator = modular_inverse(denominator)?;
    let magnitude = modular_product(numerator, inverse_denominator);
    Some(if value.is_negative() && magnitude != 0 {
        PROJECTIVE_FINGERPRINT_MODULUS - magnitude
    } else {
        magnitude
    })
}

fn modular_product(left: u64, right: u64) -> u64 {
    (u128::from(left) * u128::from(right) % u128::from(PROJECTIVE_FINGERPRINT_MODULUS))
        .try_into()
        .expect("a modular product is less than the u64 modulus")
}

fn modular_inverse(value: u64) -> Option<u64> {
    if value == 0 {
        return None;
    }
    let modulus = i128::from(PROJECTIVE_FINGERPRINT_MODULUS);
    let (mut old_remainder, mut remainder) = (modulus, i128::from(value));
    let (mut old_coefficient, mut coefficient) = (0_i128, 1_i128);
    while remainder != 0 {
        let quotient = old_remainder / remainder;
        (old_remainder, remainder) = (remainder, old_remainder - quotient * remainder);
        (old_coefficient, coefficient) = (coefficient, old_coefficient - quotient * coefficient);
    }
    (old_remainder == 1)
        .then(|| old_coefficient.rem_euclid(modulus))
        .and_then(|inverse| inverse.try_into().ok())
}

fn projective_point_f64(point: &HomogeneousPoint3) -> Option<[f64; 3]> {
    let weight = bounded_f64_hint(&point.w)?;
    if weight == 0.0 || !weight.is_finite() {
        return None;
    }
    let coordinates = [&point.x, &point.y, &point.z].map(|coordinate| {
        let value = bounded_f64_hint(coordinate)? / weight;
        value.is_finite().then_some(value)
    });
    let [Some(x), Some(y), Some(z)] = coordinates else {
        return None;
    };
    Some([x, y, z])
}

fn affine_point_f64(point: &Point3) -> Option<[f64; 3]> {
    let point = [
        bounded_f64_hint(&point.x)?,
        bounded_f64_hint(&point.y)?,
        bounded_f64_hint(&point.z)?,
    ];
    point.into_iter().all(f64::is_finite).then_some(point)
}

/// Return a deliberately low-cost primitive hint for non-topological caches.
///
/// `Real::to_f64_lossy` is the right export boundary for rendering and IO, but
/// its generic fallback may refine a computable expression deeply enough to
/// recover the full binary64 exponent range. Crossing-point interning only
/// needs a coarse spatial hint, so forcing that fallback can make an otherwise
/// modest exact boolean spend minutes approximating nested trigonometric
/// expressions. Exact rationals retain their constant-time conversion path.
/// Symbolic values currently decline the optional hint; deriving their
/// approximations from already-approximated construction inputs is preferable
/// to evaluating a deeply expanded projective expression here. Failure simply
/// disables the hint and never affects a predicate or topology decision.
fn bounded_f64_hint(value: &Real) -> Option<f64> {
    value.exact_rational_ref()?;
    value.to_f64_lossy()
}

fn projective_points_certifiably_equal(
    decisions: &DecisionContext,
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
            if crate::predicate::classify_real(decisions, &minor) != Ok(Classification::On) {
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
    source_queries: Vec<Option<Option<RationalLinearForm4Query>>>,
    source_classifications: Vec<Option<Classification>>,
    source_plane_count: Option<usize>,
    points: StorageHashMap<PointClassificationKey, CachedPointPlaneClassifications>,
}

struct CachedPointPlaneClassifications {
    rational_query: Option<RationalLinearForm4Query>,
    classifications: Vec<Option<Classification>>,
}

impl PointPlaneClassificationCache {
    fn source_relation(
        &mut self,
        decisions: &DecisionContext,
        polygon: &ConvexPolygon,
        plane: &Plane,
        plane_index: usize,
        plane_count: usize,
        on_source_vertices: &mut Vec<usize>,
    ) -> HypermeshResult<SourcePlaneRelation> {
        on_source_vertices.clear();
        if certifiably_same_unoriented_plane(decisions, &polygon.support, plane) {
            on_source_vertices.extend(
                polygon
                    .known_vertex_identities()
                    .into_iter()
                    .flatten()
                    .filter_map(|identity| match identity {
                        ConstructionVertexIdentity::Source { vertex, .. } => Some(vertex),
                        _ => None,
                    }),
            );
            return Ok(SourcePlaneRelation::Inside);
        }
        let mut has_negative = false;
        let mut has_positive = false;
        let edge_identities = polygon.known_edge_identities();
        for (point_index, point) in polygon
            .known_vertices
            .as_ref()
            .ok_or(crate::error::HypermeshError::UnknownClassification)?
            .iter()
            .enumerate()
        {
            let source_vertex =
                edge_identities.and_then(|identities| source_vertex_index(identities, point_index));
            match self.classify(
                decisions,
                point,
                source_vertex,
                plane,
                plane_index,
                plane_count,
            )? {
                Classification::Negative => has_negative = true,
                Classification::Positive => has_positive = true,
                Classification::On => {
                    if let Some(source_vertex) = source_vertex {
                        on_source_vertices.push(source_vertex);
                    }
                }
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
        decisions: &DecisionContext,
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
            return classify_point_decision(decisions, point, plane);
        };
        let make_cached = || CachedPointPlaneClassifications {
            rational_query: RationalLinearForm4Query::from_affine_point3([x, y, z]),
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
            let rational_query = *self.source_queries[source_vertex]
                .get_or_insert_with(|| RationalLinearForm4Query::from_affine_point3([x, y, z]));
            let classification = crate::predicate::classify_point_with_rational_query(
                decisions,
                point,
                plane,
                rational_query.as_ref(),
            )?;
            self.source_classifications[classification_index] = Some(classification);
            return Ok(classification);
        }

        let key = PointClassificationKey([x, y, z].map(hyperlattice::Rational::storage_identity));
        let cached = self.points.entry(key).or_insert_with(make_cached);
        if let Some(classification) = cached.classifications[plane_index] {
            return Ok(classification);
        }
        let classification = crate::predicate::classify_point_with_rational_query(
            decisions,
            point,
            plane,
            cached.rational_query.as_ref(),
        )?;
        cached.classifications[plane_index] = Some(classification);
        Ok(classification)
    }
}

fn source_vertex_index(
    edge_identities: KnownEdgeIdentityCycle<'_>,
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
    ) = (&current, &previous)
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
        point_cache: &ProjectivePointCache,
    ) -> bool {
        if self.source_plane == plane_identity {
            return true;
        }
        if self.boundary.is_empty() {
            return false;
        }
        let previous = if point_index == 0 {
            self.boundary.len() - 1
        } else {
            point_index - 1
        };
        point_cache.has_incidence(&self.boundary[point_index].point_identity, plane_identity)
            || [previous, point_index].into_iter().any(|edge_index| {
                matches!(
                    self.boundary.get(edge_index).map(|entry| &entry.edge_identity),
                    Some(ConstructionEdgeIdentity::Split { planes })
                        if planes.contains(&plane_identity)
                )
            })
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
        let source_edge_identities = polygon
            .known_edge_identities()
            .ok_or(crate::error::HypermeshError::UnknownClassification)?;
        if source_edge_identities.len() != source_points.len() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        match polygon.edges.len() {
            len if len == source_points.len() || len <= 1 => {}
            _ => return Err(crate::error::HypermeshError::UnknownClassification),
        }
        let source_plane = point_cache.canonical_plane_identity(source_plane);
        let support_index = point_cache.support_plane_index(source_plane, &polygon.support);
        let mut boundary = Vec::with_capacity(source_points.len());
        for (point_index, point) in source_points.iter().enumerate() {
            let approximate = affine_point_f64(point);
            let vertex = source_vertex_index(source_edge_identities, point_index)
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let edge_identity = source_edge_identities
                .get(point_index)
                .expect("source edge identity indices are aligned");
            let mesh = match &edge_identity {
                ConstructionEdgeIdentity::Source { mesh, .. } => *mesh,
                ConstructionEdgeIdentity::Split { .. } => {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
            };
            let identity = ConstructionVertexIdentity::Source { mesh, vertex };
            let (projective_point_index, approximate, identity) = point_cache
                .intern_with_known_approximation_by(identity, approximate, || {
                    HomogeneousPoint3::new(
                        point.x.clone(),
                        point.y.clone(),
                        point.z.clone(),
                        Real::one(),
                    )
                });
            let edge = match polygon.edges.len() {
                0 => &polygon.support,
                1 => &polygon.edges[0],
                _ => &polygon.edges[point_index],
            };
            let edge_index = point_cache.boundary_plane_index(source_plane, &edge_identity, edge);
            boundary.push(ProjectiveBoundaryEntry {
                point_index: projective_point_index,
                evidence: ProjectivePointEvidence {
                    approximate,
                    // Retain filter queries only for constructed crossings that
                    // survive into later clips; source vertices use the one-shot path.
                    rational_filter_query: None,
                },
                point_identity: identity,
                edge_index,
                edge_identity,
            });
        }
        Ok(Self {
            boundary,
            support_index,
            source_plane,
            source_unchanged: true,
        })
    }

    fn crossing_points(
        &self,
        classifications: &[Classification],
        plane: &Plane,
        plane_identity: ConstructionPlaneIdentity,
        point_cache: &mut ProjectivePointCache,
    ) -> Vec<(
        usize,
        usize,
        ProjectivePointEvidence,
        ConstructionVertexIdentity,
    )> {
        let mut crossings = Vec::with_capacity(2);
        for index in 0..self.boundary.len() {
            let next = (index + 1) % self.boundary.len();
            let current_classification = classifications[index];
            let next_classification = classifications[next];
            let crossing = (current_classification.is_negative()
                && next_classification.is_positive())
                || (current_classification.is_positive() && next_classification.is_negative());
            if crossing {
                let (point, approximate_point, identity) = self.cached_crossing_point(
                    index,
                    plane_identity,
                    current_classification,
                    next,
                    plane,
                    point_cache,
                );
                let rational_filter_query =
                    ProjectivePoint3PredicateEvidence::new(point_cache.point(point))
                        .rational_filter_query();
                crossings.push((
                    index,
                    point,
                    ProjectivePointEvidence {
                        approximate: approximate_point,
                        rational_filter_query,
                    },
                    identity,
                ));
            }
        }
        crossings
    }

    fn clip(
        self,
        decisions: &DecisionContext,
        plane: &Plane,
        plane_identity: ConstructionPlaneIdentity,
        point_cache: &mut ProjectivePointCache,
    ) -> HypermeshResult<ProjectiveClip> {
        let plane_identity = point_cache.canonical_plane_identity(plane_identity);
        let plane_evidence = RationalPlane4PredicateEvidence::new(decisions, plane);
        let classifications = self
            .boundary
            .iter()
            .enumerate()
            .map(|(point_index, entry)| {
                let point = point_cache.point(entry.point_index);
                if self.point_has_plane_incidence(
                    point_index,
                    plane_identity,
                    point_cache,
                ) {
                    Ok(Classification::On)
                } else if let Some(classification) = point_cache
                    .classify_definition_against_plane(decisions, &entry.point_identity, plane)
                {
                    classification
                } else if let Some(plane_filter) = &plane_evidence {
                    let point_evidence =
                        ProjectivePoint3PredicateEvidence::with_rational_filter_query(
                        point,
                        entry.evidence.rational_filter_query,
                    );
                    match point_evidence
                        .classify_rational_plane_filter(plane_filter)
                        .or_else(|| point_evidence.classify_rational_plane_exact(plane_filter))
                    {
                        Some(classification) => Ok(classification),
                        None => point_evidence.classify(decisions, plane),
                    }
                } else {
                    classify_projective_point_decision(decisions, point, plane).inspect_err(
                        |_error| {
                        if cfg!(debug_assertions) {
                            eprintln!(
                                "[DEBUG] projective clip point failed: source={:?} target={:?} point={point_index} value={:?}",
                                self.source_plane,
                                plane_identity,
                                bounded_f64_hint(
                                    &hyperlattice::homogeneous_point_plane_expression(point, plane),
                                ),
                            );
                        }
                        },
                    )
                }
            })
            .collect::<HypermeshResult<Vec<_>>>()?;
        for (point_index, classification) in classifications.iter().enumerate() {
            if *classification == Classification::On {
                point_cache
                    .record_incidence(&self.boundary[point_index].point_identity, plane_identity);
            }
        }
        let has_negative = classifications
            .iter()
            .any(|classification| classification.is_negative());
        let has_positive = classifications
            .iter()
            .any(|classification| classification.is_positive());
        if !has_positive {
            crate::trace_dispatch!("projective-clip", "negative-only");
            return Ok(ProjectiveClip {
                negative: self,
                positive: Self::empty(),
                side: ProjectiveClipSide::Negative,
            });
        }
        if !has_negative {
            crate::trace_dispatch!("projective-clip", "positive-only");
            return Ok(ProjectiveClip {
                negative: Self::empty(),
                positive: self,
                side: ProjectiveClipSide::Positive,
            });
        }

        crate::trace_dispatch!("projective-clip", "split");
        let clipping_plane_index = point_cache.support_plane_index(plane_identity, plane);
        let inverted_plane_index = point_cache.inverted_plane_index(plane_identity);
        let intersections =
            self.crossing_points(&classifications, plane, plane_identity, point_cache);
        let mut negative = ProjectiveBoundary::with_capacity(self.boundary.len() + 1);
        let mut positive = ProjectiveBoundary::with_capacity(self.boundary.len() + 1);
        let mut split_planes = [self.source_plane, plane_identity];
        split_planes.sort_unstable();
        let split_identity = ConstructionEdgeIdentity::Split {
            planes: split_planes,
        };
        let Self {
            boundary,
            support_index,
            source_plane,
            ..
        } = self;
        let mut intersections = intersections.into_iter();
        for (index, entry) in boundary.into_iter().enumerate() {
            let ProjectiveBoundaryEntry {
                point_index: point,
                evidence: point_evidence,
                point_identity,
                edge_index,
                edge_identity,
            } = entry;
            let next = (index + 1) % classifications.len();
            let current_classification = classifications[index];
            let next_classification = classifications[next];
            match (current_classification, next_classification) {
                (Classification::Negative, Classification::Negative | Classification::On) => {
                    negative.push(
                        point,
                        point_evidence,
                        point_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                }
                (Classification::Negative, Classification::Positive) => {
                    let (
                        crossing_index,
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                    ) = intersections
                        .next()
                        .expect("strict side transition has an intersection");
                    debug_assert_eq!(crossing_index, index);
                    negative.push(
                        point,
                        point_evidence,
                        point_identity,
                        edge_index,
                        edge_identity.clone(),
                        point_cache,
                    );
                    negative.push(
                        intersection,
                        intersection_preparation,
                        intersection_identity.clone(),
                        clipping_plane_index,
                        split_identity.clone(),
                        point_cache,
                    );
                    positive.push(
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                }
                (Classification::On, Classification::Negative) => {
                    negative.push(
                        point,
                        point_evidence,
                        point_identity.clone(),
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                    positive.push(
                        point,
                        point_evidence,
                        point_identity,
                        inverted_plane_index,
                        split_identity.clone(),
                        point_cache,
                    );
                }
                (Classification::On, Classification::On) => {
                    negative.push(
                        point,
                        point_evidence,
                        point_identity.clone(),
                        edge_index,
                        edge_identity.clone(),
                        point_cache,
                    );
                    positive.push(
                        point,
                        point_evidence,
                        point_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                }
                (Classification::On, Classification::Positive) => {
                    negative.push(
                        point,
                        point_evidence,
                        point_identity.clone(),
                        clipping_plane_index,
                        split_identity.clone(),
                        point_cache,
                    );
                    positive.push(
                        point,
                        point_evidence,
                        point_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                }
                (Classification::Positive, Classification::Negative) => {
                    let (
                        crossing_index,
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                    ) = intersections
                        .next()
                        .expect("strict side transition has an intersection");
                    debug_assert_eq!(crossing_index, index);
                    negative.push(
                        intersection,
                        intersection_preparation,
                        intersection_identity.clone(),
                        edge_index,
                        edge_identity.clone(),
                        point_cache,
                    );
                    positive.push(
                        point,
                        point_evidence,
                        point_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                    positive.push(
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                        inverted_plane_index,
                        split_identity.clone(),
                        point_cache,
                    );
                }
                (Classification::Positive, Classification::On | Classification::Positive) => {
                    positive.push(
                        point,
                        point_evidence,
                        point_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                }
            }
        }
        debug_assert!(intersections.next().is_none());
        Ok(ProjectiveClip {
            negative: negative.into_cycle(support_index, source_plane, point_cache),
            positive: positive.into_cycle(support_index, source_plane, point_cache),
            side: ProjectiveClipSide::Both,
        })
    }

    fn clip_negative(
        self,
        decisions: &DecisionContext,
        plane: &Plane,
        plane_identity: ConstructionPlaneIdentity,
        point_cache: &mut ProjectivePointCache,
    ) -> HypermeshResult<Self> {
        let plane_identity = point_cache.canonical_plane_identity(plane_identity);
        let plane_evidence = RationalPlane4PredicateEvidence::new(decisions, plane);
        let classifications = self
            .boundary
            .iter()
            .enumerate()
            .map(|(point_index, entry)| {
                let point = point_cache.point(entry.point_index);
                if self.point_has_plane_incidence(
                    point_index,
                    plane_identity,
                    point_cache,
                ) {
                    Ok(Classification::On)
                } else if let Some(classification) = point_cache
                    .classify_definition_against_plane(decisions, &entry.point_identity, plane)
                {
                    classification
                } else if let Some(plane_filter) = &plane_evidence {
                    let point_evidence =
                        ProjectivePoint3PredicateEvidence::with_rational_filter_query(
                        point,
                        entry.evidence.rational_filter_query,
                    );
                    match point_evidence
                        .classify_rational_plane_filter(plane_filter)
                        .or_else(|| point_evidence.classify_rational_plane_exact(plane_filter))
                    {
                        Some(classification) => Ok(classification),
                        None => point_evidence.classify(decisions, plane),
                    }
                } else {
                    classify_projective_point_decision(decisions, point, plane).inspect_err(
                        |_error| {
                        if cfg!(debug_assertions) {
                            eprintln!(
                                "[DEBUG] projective negative clip point failed: source={:?} target={:?} point={point_index} identity={:?} adjacent={:?} point_xyz={:?} plane={:?} exact={:?} value={:?}",
                                self.source_plane,
                                plane_identity,
                                self.boundary.get(point_index).map(|entry| &entry.point_identity),
                                [
                                    self.boundary.get(if point_index == 0 { self.boundary.len() - 1 } else { point_index - 1 }).map(|entry| &entry.edge_identity),
                                    self.boundary.get(point_index).map(|entry| &entry.edge_identity),
                                ],
                                [
                                    bounded_f64_hint(&point.x),
                                    bounded_f64_hint(&point.y),
                                    bounded_f64_hint(&point.z),
                                    bounded_f64_hint(&point.w),
                                ],
                                [
                                    bounded_f64_hint(&plane.normal.x),
                                    bounded_f64_hint(&plane.normal.y),
                                    bounded_f64_hint(&plane.normal.z),
                                    bounded_f64_hint(&plane.offset),
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
                        },
                    )
                }
            })
            .collect::<HypermeshResult<Vec<_>>>()?;
        for (point_index, classification) in classifications.iter().enumerate() {
            if *classification == Classification::On {
                point_cache
                    .record_incidence(&self.boundary[point_index].point_identity, plane_identity);
            }
        }
        let has_negative = classifications
            .iter()
            .any(|classification| classification.is_negative());
        let has_positive = classifications
            .iter()
            .any(|classification| classification.is_positive());
        if !has_positive {
            crate::trace_dispatch!("projective-clip-negative", "kept");
            return Ok(self);
        }
        if !has_negative {
            crate::trace_dispatch!("projective-clip-negative", "empty");
            return Ok(Self::empty());
        }
        crate::trace_dispatch!("projective-clip-negative", "split");
        let clipping_plane_index = point_cache.support_plane_index(plane_identity, plane);
        let intersections =
            self.crossing_points(&classifications, plane, plane_identity, point_cache);
        let mut negative = ProjectiveBoundary::with_capacity(self.boundary.len() + 1);
        let mut split_planes = [self.source_plane, plane_identity];
        split_planes.sort_unstable();
        let split_identity = ConstructionEdgeIdentity::Split {
            planes: split_planes,
        };
        let Self {
            boundary,
            support_index,
            source_plane,
            ..
        } = self;
        let mut intersections = intersections.into_iter();
        for (index, entry) in boundary.into_iter().enumerate() {
            let ProjectiveBoundaryEntry {
                point_index: point,
                evidence: point_evidence,
                point_identity,
                edge_index,
                edge_identity,
            } = entry;
            let next = (index + 1) % classifications.len();
            let current_classification = classifications[index];
            let next_classification = classifications[next];
            match (current_classification, next_classification) {
                (
                    Classification::Negative | Classification::On,
                    Classification::Negative | Classification::On,
                ) => {
                    negative.push(
                        point,
                        point_evidence,
                        point_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                }
                (Classification::Negative, Classification::Positive) => {
                    let (
                        crossing_index,
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                    ) = intersections
                        .next()
                        .expect("strict side transition has an intersection");
                    debug_assert_eq!(crossing_index, index);
                    negative.push(
                        point,
                        point_evidence,
                        point_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                    negative.push(
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                        clipping_plane_index,
                        split_identity.clone(),
                        point_cache,
                    );
                }
                (Classification::On, Classification::Positive) => {
                    negative.push(
                        point,
                        point_evidence,
                        point_identity,
                        clipping_plane_index,
                        split_identity.clone(),
                        point_cache,
                    );
                }
                (Classification::Positive, Classification::Negative) => {
                    let (
                        crossing_index,
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                    ) = intersections
                        .next()
                        .expect("strict side transition has an intersection");
                    debug_assert_eq!(crossing_index, index);
                    negative.push(
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                }
                (Classification::Positive, Classification::On | Classification::Positive) => {}
            }
        }
        debug_assert!(intersections.next().is_none());
        Ok(negative.into_cycle(support_index, source_plane, point_cache))
    }

    #[allow(clippy::too_many_arguments)]
    fn cached_crossing_point(
        &self,
        edge_index: usize,
        plane_identity: ConstructionPlaneIdentity,
        current_classification: Classification,
        next_index: usize,
        plane: &Plane,
        point_cache: &mut ProjectivePointCache,
    ) -> (usize, Option<[f64; 3]>, ConstructionVertexIdentity) {
        let identity = point_cache.edge_plane_intersection_identity(
            &self.boundary[edge_index].edge_identity,
            plane_identity,
        );
        if point_cache.points.len() >= PROJECTIVE_CROSSING_CACHE_MIN_POINTS
            && let Some(existing) = point_cache.points.get(&identity)
        {
            return (existing.point_index, existing.approximate, identity);
        }
        let point = if let Some(point) = point_cache
            .definition_planes(&identity)
            .and_then(positive_weight_plane_intersection)
        {
            point
        } else {
            let current = point_cache.point(self.boundary[edge_index].point_index);
            let next = point_cache.point(self.boundary[next_index].point_index);
            let current_value = homogeneous_point_plane_expression(current, plane);
            let next_value = homogeneous_point_plane_expression(next, plane);
            projective_crossing_point(
                current,
                &current_value,
                current_classification,
                next,
                &next_value,
            )
        };
        point_cache.intern_with_approximation(identity, point)
    }

    fn materialize(
        self,
        decisions: &DecisionContext,
        source: &ConvexPolygon,
        affine_cache: &mut ProjectiveAffineCache,
        point_cache: &ProjectivePointCache,
    ) -> HypermeshResult<ConvexPolygon> {
        if self.source_unchanged {
            return Ok(source.clone());
        }
        let mut vertices = Vec::with_capacity(self.boundary.len());
        let mut point_identities = Vec::with_capacity(self.boundary.len());
        let mut edges = Vec::with_capacity(self.boundary.len());
        let mut edge_identities = Vec::with_capacity(self.boundary.len());
        for entry in self.boundary {
            let compact = point_cache
                .definition_planes(&entry.point_identity)
                .map(|planes| intersect_three_planes(planes[0], planes[1], planes[2]));
            let point = compact
                .as_ref()
                .unwrap_or_else(|| point_cache.point(entry.point_index));
            vertices.push(affine_cache.resolve(decisions, point, Some(&entry.point_identity))?);
            point_identities.push(entry.point_identity);
            edges.push(point_cache.plane(entry.edge_index).clone());
            edge_identities.push(entry.edge_identity);
        }
        source.with_known_vertex_cycle_and_edges(
            decisions,
            vertices,
            point_identities,
            edges,
            edge_identities,
        )
    }

    fn empty() -> Self {
        Self {
            boundary: Vec::new(),
            support_index: usize::MAX,
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
        decisions: &DecisionContext,
        point: &HomogeneousPoint3,
        identity: Option<&ConstructionVertexIdentity>,
    ) -> HypermeshResult<Point3> {
        if let Some(identity) = identity
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
            let affine = affine_projective_point_decision(decisions, point)?;
            self.points.insert(
                key,
                ProjectiveAffineCacheEntry {
                    _coordinates: [x.clone(), y.clone(), z.clone(), w.clone()],
                    affine: affine.clone(),
                },
            );
            if let Some(identity) = identity {
                self.identities.insert(identity.clone(), affine.clone());
            }
            return Ok(affine);
        }
        let affine = affine_projective_point_decision(decisions, point)?;
        if let Some(identity) = identity {
            self.identities.insert(identity.clone(), affine.clone());
        }
        Ok(affine)
    }
}

fn compute_projective_input_soup(
    decisions: &DecisionContext,
    input: &ProjectiveInputSoup,
    input_planes: Option<&[&[InputTrianglePlanes]]>,
    operation: BooleanOp,
    retain_winding: bool,
) -> HypermeshResult<Option<ConvexCandidate>> {
    let [first, second] = input.meshes.as_slice() else {
        return Err(crate::error::HypermeshError::UnknownClassification);
    };
    let input_meshes = [first, second];
    let mut support_planes: [Vec<&Plane>; 2] = std::array::from_fn(|_| Vec::new());
    let mut storage_support_planes: [StorageHashMap<[usize; 4], usize>; 2] =
        std::array::from_fn(|_| StorageHashMap::default());
    let mut approximate_support_planes: [ApproximateSupportPlaneIndex; 2] =
        std::array::from_fn(|_| ApproximateSupportPlaneIndex::default());
    let mut non_exact_support_planes: [Vec<usize>; 2] = std::array::from_fn(|_| Vec::new());
    let mut support_plane_f64_values: [Vec<Option<[f64; 4]>>; 2] =
        std::array::from_fn(|_| Vec::new());
    let mut indexed_support_planes: [Vec<usize>; 2] = std::array::from_fn(|_| Vec::new());
    for (mesh, source) in input_meshes.iter().enumerate() {
        indexed_support_planes[mesh].reserve(source.support_planes.len());
        for support in &source.support_planes {
            let storage_key = exact_plane_storage_key(support);
            let stored_plane =
                storage_key.and_then(|key| storage_support_planes[mesh].get(&key).copied());
            let plane = if let Some(index) = stored_plane {
                index
            } else if let Some(values) = exact_plane_f64(decisions, support) {
                let key = values.map(f64::to_bits);
                let (index, inserted) = approximate_support_planes[mesh].intern(
                    key,
                    &mut support_planes[mesh],
                    support,
                );
                if inserted {
                    support_plane_f64_values[mesh].push(Some(values));
                }
                index
            } else if let Some(index) =
                non_exact_support_planes[mesh]
                    .iter()
                    .copied()
                    .find(|&index| {
                        certifiably_same_oriented_plane(
                            decisions,
                            support_planes[mesh][index],
                            support,
                        )
                        .unwrap_or(false)
                    })
            {
                index
            } else if let Some(index) = support_planes[mesh]
                .iter()
                .position(|plane| *plane == support)
            {
                index
            } else {
                let index = support_planes[mesh].len();
                support_planes[mesh].push(support);
                support_plane_f64_values[mesh].push(None);
                non_exact_support_planes[mesh].push(index);
                index
            };
            if let Some(key) = storage_key
                && stored_plane.is_none()
            {
                storage_support_planes[mesh].insert(key, plane);
            }
            indexed_support_planes[mesh].push(plane);
        }
    }
    let support_planes_f64 =
        support_plane_f64_values.map(|planes| planes.into_iter().collect::<Option<Vec<_>>>());
    let canonical_plane_identities = canonical_plane_identities(decisions, &support_planes);
    let (faces, mut face_supports) = collapse_projective_input_faces(
        decisions,
        input_meshes,
        input_planes,
        &support_planes,
        &indexed_support_planes,
    )?;
    for identity in &mut face_supports {
        *identity = canonical_plane_identities[identity.mesh][identity.plane];
    }
    compute_projective_convex_faces(
        decisions,
        &faces,
        &support_planes,
        &support_planes_f64,
        canonical_plane_identities,
        face_supports,
        operation,
        retain_winding,
    )
}

fn collapse_projective_input_faces(
    decisions: &DecisionContext,
    input_meshes: [&crate::mesh::ProjectiveInputMesh; 2],
    input_planes: Option<&[&[InputTrianglePlanes]]>,
    support_planes: &[Vec<&Plane>; 2],
    indexed_support_planes: &[Vec<usize>; 2],
) -> HypermeshResult<(Vec<ConvexPolygon>, Vec<ConstructionPlaneIdentity>)> {
    const MAX_SINGLE_USE_CERTIFICATE_TRIANGLES: usize = 16;
    let first_mesh_planes = support_planes[0].len();
    let group_count = first_mesh_planes
        .checked_add(support_planes[1].len())
        .ok_or(crate::error::HypermeshError::UnknownClassification)?;
    let triangle_count = input_meshes
        .iter()
        .try_fold(0usize, |total, mesh| {
            total.checked_add(mesh.triangles.len())
        })
        .ok_or(crate::error::HypermeshError::UnknownClassification)?;
    let mut group_offsets = vec![0usize; group_count + 1];
    for (mesh, source) in input_meshes.iter().enumerate() {
        for triangle in &source.triangles {
            let Some(&support) = indexed_support_planes[mesh].get(triangle.support_plane) else {
                return Err(crate::error::HypermeshError::UnknownClassification);
            };
            let group = if mesh == 0 {
                support
            } else {
                first_mesh_planes + support
            };
            group_offsets[group + 1] += 1;
        }
    }
    for group in 0..group_count {
        group_offsets[group + 1] += group_offsets[group];
    }
    let mut grouped_triangles = vec![[0usize; 2]; triangle_count];
    for mesh in (0..input_meshes.len()).rev() {
        for (triangle_index, triangle) in input_meshes[mesh].triangles.iter().enumerate().rev() {
            let support = indexed_support_planes[mesh][triangle.support_plane];
            let group = if mesh == 0 {
                support
            } else {
                first_mesh_planes + support
            };
            group_offsets[group + 1] -= 1;
            grouped_triangles[group_offsets[group + 1]] = [mesh, triangle_index];
        }
    }

    let deferred_edges = Arc::new(Vec::<Plane>::new());
    let mut faces = Vec::with_capacity(group_count);
    let mut face_supports = Vec::with_capacity(group_count);
    for group in 0..group_count {
        let start = group_offsets[group + 1];
        let end = if group + 1 == group_count {
            grouped_triangles.len()
        } else {
            group_offsets[group + 2]
        };
        let triangle_refs = &grouped_triangles[start..end];
        if triangle_refs.is_empty() {
            continue;
        }
        let (mesh, support) = if group < first_mesh_planes {
            (0, group)
        } else {
            (1, group - first_mesh_planes)
        };
        if let [[source_mesh, triangle_index]] = triangle_refs {
            if *source_mesh != mesh {
                return Err(crate::error::HypermeshError::UnknownClassification);
            }
            let source = &input_meshes[mesh];
            let triangle = source
                .triangles
                .get(*triangle_index)
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let mut face = if let Some(planes) = input_planes
                .and_then(|planes| planes.get(mesh))
                .and_then(|planes| planes.get(*triangle_index))
            {
                let mut planes = planes.clone();
                planes.support = support_planes[mesh][support].clone();
                make_indexed_triangle_with_deferred_edges_and_input_planes(
                    Arc::clone(&source.positions),
                    triangle.indices,
                    planes,
                    mesh as isize,
                    source.polygon_index(*triangle_index)?,
                )
            } else {
                make_indexed_triangle_with_deferred_edges(
                    Arc::clone(&source.positions),
                    triangle.indices,
                    Some(support_planes[mesh][support].clone()),
                    Arc::clone(&deferred_edges),
                    mesh as isize,
                    source.polygon_index(*triangle_index)?,
                )
            };
            face.set_source_triangle_edge_identities(mesh, triangle.indices);
            face.delta_w = vec![0; input_meshes.len()];
            face.delta_w[mesh] = 1;
            faces.push(face);
            face_supports.push(ConstructionPlaneIdentity {
                mesh,
                plane: support,
            });
            continue;
        }

        let source_edge_count = triangle_refs.len().saturating_mul(3);
        let mut source_edges = SourceEdgeOccurrences::with_capacity(
            source_edge_count,
            input_meshes[mesh].positions.len(),
        );
        for &[source_mesh, triangle_index] in triangle_refs {
            if source_mesh != mesh {
                return Err(crate::error::HypermeshError::UnknownClassification);
            }
            let triangle = input_meshes[mesh]
                .triangles
                .get(triangle_index)
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let [i0, i1, i2] = triangle.indices;
            for mut endpoints in [[i0, i1], [i1, i2], [i2, i0]] {
                endpoints.sort_unstable();
                source_edges.push(endpoints);
            }
        }

        let mut single_use_vertices = [0usize; MAX_SINGLE_USE_CERTIFICATE_TRIANGLES * 3];
        let mut single_use_vertex_count = 0;
        if triangle_refs.len() <= MAX_SINGLE_USE_CERTIFICATE_TRIANGLES {
            let mut source_vertex_count = 0;
            for &[_, triangle_index] in triangle_refs {
                for vertex in input_meshes[mesh].triangles[triangle_index].indices {
                    single_use_vertices[source_vertex_count] = vertex;
                    source_vertex_count += 1;
                }
            }
            let source_vertices = &mut single_use_vertices[..source_vertex_count];
            source_vertices.sort_unstable();
            let mut vertex_index = 0;
            while vertex_index < source_vertices.len() {
                let mut next = vertex_index + 1;
                while next < source_vertices.len()
                    && source_vertices[next] == source_vertices[vertex_index]
                {
                    next += 1;
                }
                if next == vertex_index + 1 {
                    source_vertices[single_use_vertex_count] = source_vertices[vertex_index];
                    single_use_vertex_count += 1;
                }
                vertex_index = next;
            }
        }
        let boundary_edges = source_edges.into_unique_occurrences();
        let mut outgoing = Vec::with_capacity(boundary_edges.len());
        for occurrence in boundary_edges {
            let (group_triangle_index, edge_index) = (occurrence / 3, occurrence % 3);
            let [_, triangle_index] = triangle_refs[group_triangle_index];
            let triangle = &input_meshes[mesh].triangles[triangle_index];
            let start = triangle.indices[edge_index];
            let end = triangle.indices[(edge_index + 1) % 3];
            let point = input_meshes[mesh]
                .positions
                .get(start)
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let edge_plane = input_planes
                .and_then(|planes| planes.get(mesh))
                .and_then(|planes| planes.get(triangle_index))
                .map(|planes| &planes.edges[edge_index]);
            outgoing.push((start, end, point, edge_plane));
        }
        outgoing.sort_unstable_by_key(|entry| entry.0);
        if outgoing.windows(2).any(|pair| pair[0].0 == pair[1].0) {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let Some(mut current) = outgoing.first().map(|entry| entry.0) else {
            return Err(crate::error::HypermeshError::UnknownClassification);
        };
        let cycle_start = current;
        let mut face_vertices = Vec::with_capacity(outgoing.len());
        let mut vertex_identities = Vec::with_capacity(outgoing.len());
        let mut edge_planes = Vec::new();
        let mut edge_planes_complete = true;
        let mut edge_identities = Vec::with_capacity(outgoing.len());
        while face_vertices.len() < outgoing.len() {
            let Ok(outgoing_index) = outgoing.binary_search_by_key(&current, |entry| entry.0)
            else {
                return Err(crate::error::HypermeshError::UnknownClassification);
            };
            let (_, next, point, edge_plane) = outgoing[outgoing_index];
            face_vertices.push(point);
            vertex_identities.push(ConstructionVertexIdentity::Source {
                mesh,
                vertex: current,
            });
            if edge_planes_complete {
                if let Some(edge_plane) = edge_plane {
                    if edge_planes.is_empty() {
                        edge_planes.reserve(outgoing.len());
                    }
                    edge_planes.push(edge_plane.clone());
                } else {
                    edge_planes.clear();
                    edge_planes_complete = false;
                }
            }
            let mut endpoints = [current, next];
            endpoints.sort_unstable();
            edge_identities.push(ConstructionEdgeIdentity::Source { mesh, endpoints });
            current = next;
            if current == cycle_start {
                break;
            }
        }
        if current != cycle_start || face_vertices.len() != outgoing.len() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let certified_noncollinear_source_vertices = (single_use_vertex_count != 0)
            .then_some(&single_use_vertices[..single_use_vertex_count]);
        collapse_certified_collinear_face_vertices(
            decisions,
            mesh,
            support_planes[mesh][support],
            &mut face_vertices,
            &mut vertex_identities,
            &mut edge_planes,
            &mut edge_identities,
            certified_noncollinear_source_vertices,
        )?;
        let mut delta_w = vec![0; input_meshes.len()];
        delta_w[mesh] = 1;
        faces.push(ConvexPolygon::from_certified_convex_face(
            support_planes[mesh][support].clone(),
            &face_vertices,
            Some(Arc::clone(&input_meshes[mesh].positions)),
            vertex_identities,
            edge_planes,
            edge_identities,
            mesh as isize,
            input_meshes[mesh].polygon_index(triangle_refs[0][1])?,
            delta_w,
        ));
        face_supports.push(ConstructionPlaneIdentity {
            mesh,
            plane: support,
        });
    }
    Ok((faces, face_supports))
}

fn compute_two_convex_inputs_projectively(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    operation: BooleanOp,
    retain_winding: bool,
) -> HypermeshResult<Option<ConvexCandidate>> {
    let mut support_planes: [Vec<&Plane>; 2] = std::array::from_fn(|_| Vec::new());
    let mut storage_support_planes: [StorageHashMap<[usize; 4], usize>; 2] =
        std::array::from_fn(|_| StorageHashMap::default());
    let mut approximate_support_planes: [ApproximateSupportPlaneIndex; 2] =
        std::array::from_fn(|_| ApproximateSupportPlaneIndex::default());
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
        let stored_plane =
            storage_key.and_then(|key| storage_support_planes[mesh].get(&key).copied());
        let plane = if let Some(index) = stored_plane {
            index
        } else if let Some(values) = exact_plane_f64(decisions, &polygon.support) {
            let key = values.map(f64::to_bits);
            let (index, inserted) = approximate_support_planes[mesh].intern(
                key,
                &mut support_planes[mesh],
                &polygon.support,
            );
            if inserted {
                support_plane_f64_values[mesh].push(Some(values));
            }
            index
        } else if let Some(index) = support_planes[mesh].iter().position(|existing| {
            certifiably_same_oriented_plane(decisions, existing, &polygon.support).unwrap_or(false)
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
            index
        };
        if let Some(key) = storage_key
            && stored_plane.is_none()
        {
            storage_support_planes[mesh].insert(key, plane);
        }
        polygon_support_planes.push(ConstructionPlaneIdentity { mesh, plane });
    }
    let support_planes_f64 =
        support_plane_f64_values.map(|planes| planes.into_iter().collect::<Option<Vec<_>>>());
    let canonical_plane_identities = canonical_plane_identities(decisions, &support_planes);
    let (projective_polygons, mut projective_polygon_support_planes) =
        match collapse_certified_convex_faces(
            decisions,
            polygons,
            &polygon_support_planes,
            &support_planes,
        ) {
            Ok(collapsed) => collapsed,
            Err(
                crate::error::HypermeshError::PredicateUndecided { .. }
                | crate::error::HypermeshError::UnknownClassification,
            ) => (polygons.to_vec(), polygon_support_planes),
            Err(error) => return Err(error),
        };
    for identity in &mut projective_polygon_support_planes {
        *identity = canonical_plane_identities[identity.mesh][identity.plane];
    }
    compute_projective_convex_faces(
        decisions,
        &projective_polygons,
        &support_planes,
        &support_planes_f64,
        canonical_plane_identities,
        projective_polygon_support_planes,
        operation,
        retain_winding,
    )
}

#[allow(clippy::too_many_arguments)]
fn compute_projective_convex_faces(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    support_planes: &[Vec<&Plane>; 2],
    support_planes_f64: &[Option<Vec<[f64; 4]>>; 2],
    canonical_plane_identities: [Vec<ConstructionPlaneIdentity>; 2],
    polygon_support_planes: Vec<ConstructionPlaneIdentity>,
    operation: BooleanOp,
    retain_winding: bool,
) -> HypermeshResult<Option<ConvexCandidate>> {
    let mut classified = Vec::new();
    let mut point_plane_caches: [PointPlaneClassificationCache; 2] =
        std::array::from_fn(|_| PointPlaneClassificationCache::default());
    let mut affine_cache = ProjectiveAffineCache::default();
    let mut projective_point_cache = ProjectivePointCache {
        canonical_planes: canonical_plane_identities,
        ..ProjectivePointCache::default()
    };
    for (mesh, planes) in support_planes.iter().enumerate() {
        for (plane, value) in planes.iter().enumerate() {
            let identity = ConstructionPlaneIdentity { mesh, plane };
            let canonical = projective_point_cache.canonical_plane_identity(identity);
            projective_point_cache.support_plane_index(canonical, value);
        }
    }
    let mut source_vertex_points: StorageHashMap<ConstructionVertexIdentity, &Point3> =
        StorageHashMap::default();
    for (polygon, support_identity) in polygons.iter().zip(&polygon_support_planes) {
        if let Some(vertex_identities) = polygon.known_vertex_identities() {
            let retained_vertices = polygon.known_vertices.as_ref();
            for (vertex_index, vertex_identity) in vertex_identities.iter().enumerate() {
                if matches!(vertex_identity, ConstructionVertexIdentity::Source { .. }) {
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
                            .or_insert(point);
                    }
                }
            }
        }
        let Some(edge_identities) = polygon.known_edge_identities() else {
            continue;
        };
        if edge_identities.len() != polygon.edges.len() {
            continue;
        }
        let Some(&support) = projective_point_cache.planes.get(support_identity) else {
            continue;
        };
        for (edge_identity, edge_plane) in edge_identities.iter().zip(polygon.edges.iter()) {
            if matches!(edge_identity, ConstructionEdgeIdentity::Source { .. }) {
                let key = ProjectivePointCache::source_edge_key(&edge_identity)
                    .expect("source edge identity was matched above");
                if let Some(source_edge) = projective_point_cache.source_edges.get_mut(&key) {
                    if !source_edge.insert_support(*support_identity) {
                        return Err(crate::error::HypermeshError::UnknownClassification);
                    }
                } else {
                    let boundary = projective_point_cache.boundary_plane_index(
                        *support_identity,
                        &edge_identity,
                        edge_plane,
                    );
                    projective_point_cache.source_edges.insert(
                        key,
                        ProjectiveSourceEdge::new([support, boundary], *support_identity),
                    );
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
            if classify_point_decision(decisions, point, value)? == Classification::On {
                let plane_identity = projective_point_cache
                    .canonical_plane_identity(ConstructionPlaneIdentity { mesh: other, plane });
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
    let mut verification_plane_evidence = Vec::new();
    let mut candidate_planes = Vec::new();
    let mut on_source_vertices = Vec::new();
    let mut active_plane_proposal_scratch = ActivePlaneProposalScratch::default();
    for (polygon, source_plane) in polygons.iter().zip(polygon_support_planes) {
        let host = usize::try_from(polygon.mesh_index)
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
        let other = 1 - host;
        let emit_outside = projective_transition_is_emitted(host, false, operation);
        let default_emit_inside = projective_transition_is_emitted(host, true, operation);
        candidate_planes.clear();
        let mut excluded = false;
        let mut has_cooriented_coincident_support = false;
        for (plane_index, &plane) in support_planes[other].iter().enumerate() {
            if !has_cooriented_coincident_support
                && source_plane
                    == projective_point_cache.canonical_plane_identity(ConstructionPlaneIdentity {
                        mesh: other,
                        plane: plane_index,
                    })
                && certifiably_same_oriented_plane(decisions, &polygon.support, plane)
                    .unwrap_or(false)
            {
                has_cooriented_coincident_support = true;
            }
            let relation = point_plane_caches[host]
                .source_relation(
                decisions,
                polygon,
                plane,
                plane_index,
                support_planes[other].len(),
                &mut on_source_vertices,
            )
            .inspect_err(|_error| {
                if cfg!(debug_assertions) {
                    eprintln!(
                        "[DEBUG] projective source relation failed: host={host} polygon={} other_plane={plane_index}",
                        polygon.polygon_index,
                    );
                }
            })?;
            let plane_identity =
                projective_point_cache.canonical_plane_identity(ConstructionPlaneIdentity {
                    mesh: other,
                    plane: plane_index,
                });
            for &vertex in &on_source_vertices {
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
                push_source_transition(
                    &mut classified,
                    polygon,
                    host,
                    other,
                    false,
                    operation,
                    retain_winding,
                )?;
            }
            continue;
        }
        if candidate_planes.is_empty() {
            if emit_inside {
                push_source_transition(
                    &mut classified,
                    polygon,
                    host,
                    other,
                    inside_winding,
                    operation,
                    retain_winding,
                )?;
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

        let clipped_result = exact_inside_and_outside_cycles(
            decisions,
            source,
            polygon,
            source_plane,
            &support_planes[other],
            support_planes_f64[other].as_deref(),
            &candidate_planes,
            other,
            emit_outside,
            &mut verification_plane_evidence,
            &mut active_plane_proposal_scratch,
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
        let Some((inside, outside_cycles)) = clipped_result else {
            if emit_outside {
                let source = ProjectiveCycle::from_polygon(
                    polygon,
                    source_plane,
                    &mut projective_point_cache,
                )?;
                push_projective_transition(
                    decisions,
                    &mut classified,
                    source,
                    polygon,
                    &mut affine_cache,
                    &projective_point_cache,
                    host,
                    other,
                    false,
                    operation,
                    retain_winding,
                )?;
            }
            continue;
        };
        if emit_outside {
            let outside_cycles =
                outside_cycles.expect("outside cycles are retained when outside is emitted");
            for outside in outside_cycles {
                push_projective_transition(
                    decisions,
                    &mut classified,
                    outside,
                    polygon,
                    &mut affine_cache,
                    &projective_point_cache,
                    host,
                    other,
                    false,
                    operation,
                    retain_winding,
                )?;
            }
            if emit_inside {
                push_projective_transition(
                    decisions,
                    &mut classified,
                    inside,
                    polygon,
                    &mut affine_cache,
                    &projective_point_cache,
                    host,
                    other,
                    inside_winding,
                    operation,
                    retain_winding,
                )?;
            }
            continue;
        }
        if emit_inside {
            push_projective_transition(
                decisions,
                &mut classified,
                inside,
                polygon,
                &mut affine_cache,
                &projective_point_cache,
                host,
                other,
                inside_winding,
                operation,
                retain_winding,
            )?;
        }
    }
    projective_point_cache.resolve_vertex_coincidences(decisions);
    if !projective_point_cache.canonical_identities.is_empty() {
        affine_cache.identities.clear();
        // Hash-probing each cycle amortizes only for a substantial fragment
        // family. Small candidates retain the direct rebuild loop.
        const SELECTIVE_REBUILD_MIN_FRAGMENTS: usize = 32;
        let select_changed_fragments = classified.len() >= SELECTIVE_REBUILD_MIN_FRAGMENTS;
        for fragment in &mut classified {
            if let Some(vertex_identities) = fragment.polygon.known_vertex_identities() {
                // Coincidence resolution records only identities whose complete
                // equivalence class selected a different representative. A cycle
                // containing none of those identities already has both its final
                // identities and the affine points materialized from them.
                if select_changed_fragments
                    && !vertex_identities.iter().any(|identity| {
                        projective_point_cache
                            .canonical_identities
                            .contains_key(&identity)
                    })
                {
                    continue;
                }
                let canonical_identities = vertex_identities
                    .iter()
                    .map(|identity| projective_point_cache.canonical_vertex_identity(&identity))
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
                        let compact = projective_point_cache
                            .definition_planes(identity)
                            .map(|planes| intersect_three_planes(planes[0], planes[1], planes[2]));
                        let point = compact
                            .as_ref()
                            .unwrap_or_else(|| projective_point_cache.point(point.point_index));
                        affine_cache.resolve(decisions, point, Some(identity))
                    })
                    .collect::<HypermeshResult<Vec<_>>>()?;
                fragment.polygon = fragment.polygon.with_known_vertex_cycle_and_identities(
                    decisions,
                    vertices,
                    canonical_identities,
                )?;
            }
        }
    }

    if retain_winding && operation != BooleanOp::SymmetricDifference {
        for fragment in &mut classified {
            let winding = fragment
                .winding()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            fragment.classification = crate::winding::classify_polygon_output(
                &winding.w_front,
                &winding.w_back,
                operation,
            );
        }
    }
    if projective_output_has_coincident_polygons(decisions, &classified, operation)? {
        crate::trace_dispatch!("convex-candidate", "coincident-output-fallback");
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
    let boolean_mesh = {
        let triangulate_fallback = || {
            if retain_winding {
                crate::output::triangulate_classified_arrangement_precomputed_f64_scan(
                    decisions,
                    &classified,
                )
                .and_then(|triangles| select_triangle_arrangement(decisions, &triangles, operation))
            } else {
                crate::output::triangulate_preclassified_arrangement_precomputed_f64_scan(
                    decisions,
                    &classified,
                )
                .and_then(|mesh| certify_boolean_mesh_closure(decisions, mesh))
            }
        };
        let soup = if matches!(operation, BooleanOp::Difference | BooleanOp::Intersection) {
            if classified
                .iter()
                .any(|fragment| !matches!(fragment.classification, -1 | 1))
            {
                return Ok(None);
            }
            let triangulate = |recover| {
                crate::output::triangulate_preclassified_arrangement_construction_candidates(
                    decisions,
                    &classified,
                    recover,
                )
                .and_then(|mesh| certify_boolean_mesh_closure(decisions, mesh))
            };
            let soup = retry_boolean_path(triangulate(false), triangulate_fallback);
            retry_boolean_path(soup, || triangulate(true))
        } else if operation == BooleanOp::Union {
            let triangulate = |recover| {
                crate::output::triangulate_selected_preclassified_arrangement_construction_candidates(
                    decisions,
                    &classified,
                    recover,
                )
                .and_then(|mesh| certify_boolean_mesh_closure(decisions, mesh))
            };
            let soup = retry_boolean_path(triangulate(false), triangulate_fallback);
            retry_boolean_path(soup, || triangulate(true))
        } else {
            let triangulate = |recover| {
                crate::output::triangulate_classified_arrangement_construction_candidates(
                    decisions,
                    &classified,
                    recover,
                )
                .and_then(|triangles| {
                    select_triangle_arrangement(decisions, &triangles, operation)
                })
            };
            let soup = retry_boolean_path(triangulate(false), triangulate_fallback);
            retry_boolean_path(soup, || triangulate(true))
        }
        .inspect_err(|error| {
            if cfg!(debug_assertions) {
                eprintln!("[DEBUG] projective triangulation failed: {error}");
            }
        });
        soup?
    };
    Ok(Some(ConvexCandidate {
        classified,
        boolean_mesh,
    }))
}

fn projective_output_has_coincident_polygons(
    decisions: &DecisionContext,
    classified: &[ClassifiedPolygon],
    operation: BooleanOp,
) -> HypermeshResult<bool> {
    let fingerprint_capacity = classified.len().saturating_sub(classified.len() / 4);
    let mut fingerprint_buckets: StorageHashMap<[u64; 4], (usize, Vec<usize>)> =
        StorageHashMap::with_capacity_and_hasher(fingerprint_capacity, Default::default());
    let mut unkeyed: Vec<usize> = Vec::new();

    for (right_index, fragment) in classified.iter().enumerate() {
        let orientation = fragment
            .winding()
            .map_or(fragment.classification, |winding| {
                crate::winding::classify_polygon_output(
                    &winding.w_front,
                    &winding.w_back,
                    operation,
                )
            });
        if !matches!(orientation, -1..=1) {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        if orientation == 0 {
            continue;
        }

        let right = &fragment.polygon;
        if let Some(fingerprint) = exact_polygon_vertex_set_fingerprint(right) {
            if let Some((first, collisions)) = fingerprint_buckets.get(&fingerprint) {
                for left_index in std::iter::once(*first).chain(collisions.iter().copied()) {
                    if projective_output_polygons_coincide(
                        decisions,
                        &classified[left_index].polygon,
                        right,
                    )? {
                        return Ok(true);
                    }
                }
            }
            for &left_index in &unkeyed {
                if projective_output_polygons_coincide(
                    decisions,
                    &classified[left_index].polygon,
                    right,
                )? {
                    return Ok(true);
                }
            }
            if let Some((_, collisions)) = fingerprint_buckets.get_mut(&fingerprint) {
                collisions.push(right_index);
            } else {
                fingerprint_buckets.insert(fingerprint, (right_index, Vec::new()));
            }
        } else {
            for (left_index, left) in classified[..right_index].iter().enumerate() {
                let orientation = left.winding().map_or(left.classification, |winding| {
                    crate::winding::classify_polygon_output(
                        &winding.w_front,
                        &winding.w_back,
                        operation,
                    )
                });
                if orientation != 0
                    && projective_output_polygons_coincide(
                        decisions,
                        &classified[left_index].polygon,
                        right,
                    )?
                {
                    return Ok(true);
                }
            }
            unkeyed.push(right_index);
        }
    }
    Ok(false)
}

fn projective_output_polygons_coincide(
    decisions: &DecisionContext,
    left: &ConvexPolygon,
    right: &ConvexPolygon,
) -> HypermeshResult<bool> {
    if left.vertex_count() != right.vertex_count()
        || !certifiably_proportional_plane(decisions, &left.support, &right.support)?
    {
        return Ok(false);
    }
    polygon_vertex_cycles_match_unoriented(decisions, left, right)
}

fn polygon_vertex_cycles_match_unoriented(
    decisions: &DecisionContext,
    left: &ConvexPolygon,
    right: &ConvexPolygon,
) -> HypermeshResult<bool> {
    let left = left.vertices_decision(decisions)?;
    let right = right.vertices_decision(decisions)?;
    if left.len() != right.len() || left.is_empty() {
        return Ok(false);
    }
    let count = left.len();
    for offset in 0..count {
        if !crate::predicate::points_equal(decisions, &left[0], &right[offset])? {
            continue;
        }
        for reverse in [false, true] {
            let mut equal = true;
            for (index, point) in left.iter().enumerate().skip(1) {
                let right_index = if reverse {
                    (offset + count - index) % count
                } else {
                    (offset + index) % count
                };
                if !crate::predicate::points_equal(decisions, point, &right[right_index])? {
                    equal = false;
                    break;
                }
            }
            if equal {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn exact_rational_product_sum_classification<const TERMS: usize, const FACTORS: usize>(
    positive_terms: [bool; TERMS],
    terms: [[&Real; FACTORS]; TERMS],
) -> Option<Classification> {
    let rational_terms = terms.map(|term| term.map(Real::exact_rational_ref));
    if rational_terms.iter().flatten().any(Option::is_none) {
        return None;
    }
    let rational_terms = rational_terms.map(|term| {
        term.map(|factor| factor.expect("all product-sum factors were checked exact rational"))
    });
    Some(
        match Rational::signed_product_sum_ordering(positive_terms, rational_terms) {
            std::cmp::Ordering::Less => Classification::Negative,
            std::cmp::Ordering::Equal => Classification::On,
            std::cmp::Ordering::Greater => Classification::Positive,
        },
    )
}

fn certifiably_proportional_plane(
    decisions: &DecisionContext,
    left: &Plane,
    right: &Plane,
) -> HypermeshResult<bool> {
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
            let terms = [
                [left_coefficients[first], right_coefficients[second]],
                [left_coefficients[second], right_coefficients[first]],
            ];
            let classification = exact_rational_product_sum_classification([true, false], terms)
                .map(Ok)
                .unwrap_or_else(|| {
                    let minor = Real::signed_product_sum([true, false], terms);
                    crate::predicate::classify_real(decisions, &minor)
                });
            match classification {
                Ok(Classification::On) => {}
                Ok(Classification::Negative | Classification::Positive) => return Ok(false),
                Err(
                    crate::error::HypermeshError::UnknownClassification
                    | crate::error::HypermeshError::PredicateUndecided { .. },
                ) => unknown_minor = true,
                Err(error) => return Err(error),
            }
        }
    }
    if unknown_minor {
        return Ok(false);
    }
    Ok(true)
}

fn certifiably_same_oriented_plane(
    decisions: &DecisionContext,
    left: &Plane,
    right: &Plane,
) -> HypermeshResult<bool> {
    if !certifiably_proportional_plane(decisions, left, right)? {
        return Ok(false);
    }
    let terms = [
        [&left.normal.x, &right.normal.x],
        [&left.normal.y, &right.normal.y],
        [&left.normal.z, &right.normal.z],
    ];
    let classification = exact_rational_product_sum_classification([true, true, true], terms)
        .map(Ok)
        .unwrap_or_else(|| {
            let orientation = Real::signed_product_sum([true, true, true], terms);
            crate::predicate::classify_real(decisions, &orientation)
        });
    Ok(classification? == Classification::Positive)
}

fn certifiably_same_unoriented_plane(
    decisions: &DecisionContext,
    left: &Plane,
    right: &Plane,
) -> bool {
    certifiably_proportional_plane(decisions, left, right).unwrap_or(false)
}

#[cfg(test)]
fn plane_f64(plane: &Plane) -> Option<[f64; 4]> {
    normalize_plane_f64([
        plane.normal.x.to_f64_lossy()?,
        plane.normal.y.to_f64_lossy()?,
        plane.normal.z.to_f64_lossy()?,
        plane.offset.to_f64_lossy()?,
    ])
}

#[cfg(test)]
fn normalize_plane_f64(mut values: [f64; 4]) -> Option<[f64; 4]> {
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

fn canonical_plane_identities(
    decisions: &DecisionContext,
    support_planes: &[Vec<&Plane>; 2],
) -> [Vec<ConstructionPlaneIdentity>; 2] {
    let first = (0..support_planes[0].len())
        .map(|plane| ConstructionPlaneIdentity { mesh: 0, plane })
        .collect();
    let second = support_planes[1]
        .iter()
        .enumerate()
        .map(|(plane, value)| {
            support_planes[0]
                .iter()
                .enumerate()
                .find_map(|(candidate, candidate_value)| {
                    let exact_match =
                        certifiably_same_unoriented_plane(decisions, candidate_value, value);
                    exact_match.then_some(ConstructionPlaneIdentity {
                        mesh: 0,
                        plane: candidate,
                    })
                })
                .unwrap_or(ConstructionPlaneIdentity { mesh: 1, plane })
        })
        .collect();
    [first, second]
}

fn collapse_certified_convex_faces(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    polygon_support_planes: &[ConstructionPlaneIdentity],
    support_planes: &[Vec<&Plane>; 2],
) -> HypermeshResult<(Vec<ConvexPolygon>, Vec<ConstructionPlaneIdentity>)> {
    // Counting source-vertex uses pays off for small subdivision patches. On
    // large coplanar groups the few provable corners do not amortize a second
    // sort, so those groups retain the ordinary exact collinearity scan.
    const MAX_SINGLE_USE_CERTIFICATE_TRIANGLES: usize = 16;
    if polygon_support_planes.len() != polygons.len() {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }

    let first_mesh_planes = support_planes[0].len();
    let group_count = first_mesh_planes
        .checked_add(support_planes[1].len())
        .ok_or(crate::error::HypermeshError::UnknownClassification)?;
    // Support identities are dense indices. Stable counting partitioning
    // preserves source order within each face while avoiding one tree node
    // and one growable index vector per distinct support plane.
    let mut group_offsets = vec![0usize; group_count + 1];
    for &support in polygon_support_planes {
        if support.mesh >= support_planes.len()
            || support.plane >= support_planes[support.mesh].len()
        {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let group = support.mesh * first_mesh_planes + support.plane;
        group_offsets[group + 1] += 1;
    }
    for group in 0..group_count {
        group_offsets[group + 1] += group_offsets[group];
    }
    let mut grouped_polygon_indices = vec![0usize; polygons.len()];
    // Fill each partition backward so its cumulative end becomes its start.
    // Reverse source traversal keeps indices in their original order and lets
    // the offset buffer double as the insertion cursors.
    for (polygon_index, &support) in polygon_support_planes.iter().enumerate().rev() {
        let group = support.mesh * first_mesh_planes + support.plane;
        group_offsets[group + 1] -= 1;
        grouped_polygon_indices[group_offsets[group + 1]] = polygon_index;
    }

    let mut faces = Vec::with_capacity(group_count);
    let mut face_supports = Vec::with_capacity(group_count);
    for group in 0..group_count {
        let start = group_offsets[group + 1];
        let end = if group + 1 == group_count {
            grouped_polygon_indices.len()
        } else {
            group_offsets[group + 2]
        };
        let polygon_indices = &grouped_polygon_indices[start..end];
        if polygon_indices.is_empty() {
            continue;
        }
        let support_identity = if group < first_mesh_planes {
            ConstructionPlaneIdentity {
                mesh: 0,
                plane: group,
            }
        } else {
            ConstructionPlaneIdentity {
                mesh: 1,
                plane: group - first_mesh_planes,
            }
        };
        if let [polygon_index] = polygon_indices {
            let source = &polygons[*polygon_index];
            let mesh_index = usize::try_from(source.mesh_index)
                .ok()
                .filter(|&mesh| mesh < support_planes.len())
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let mut face = source.clone();
            face.support = support_planes[support_identity.mesh][support_identity.plane].clone();
            face.delta_w = vec![0; support_planes.len()];
            face.delta_w[mesh_index] = 1;
            faces.push(face);
            face_supports.push(support_identity);
            continue;
        }
        let source_edge_count = polygon_indices.len().saturating_mul(3);
        let source_vertex_count = polygons[polygon_indices[0]]
            .known_vertices
            .as_ref()
            .and_then(|vertices| vertices.source_positions())
            .map_or(0, |positions| positions.len());
        let mut source_edges =
            SourceEdgeOccurrences::with_capacity(source_edge_count, source_vertex_count);
        for &polygon_index in polygon_indices {
            let polygon = &polygons[polygon_index];
            let edge_identities = polygon
                .known_edge_identities()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let vertex_identities = polygon
                .known_vertex_identities()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let points = polygon
                .known_vertices
                .as_ref()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            if edge_identities.len() != 3
                || edge_identities.len() != vertex_identities.len()
                || points.len() != vertex_identities.len()
            {
                return Err(crate::error::HypermeshError::UnknownClassification);
            }
            for identity in vertex_identities {
                let ConstructionVertexIdentity::Source { mesh, .. } = identity else {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                };
                if mesh != support_identity.mesh {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
            }
            for identity in edge_identities.iter() {
                let ConstructionEdgeIdentity::Source { mesh, endpoints } = identity else {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                };
                if mesh != support_identity.mesh {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
                source_edges.push(endpoints);
            }
        }

        let mut single_use_vertices = [0usize; MAX_SINGLE_USE_CERTIFICATE_TRIANGLES * 3];
        let mut single_use_vertex_count = 0;
        if polygon_indices.len() <= MAX_SINGLE_USE_CERTIFICATE_TRIANGLES {
            let mut source_vertex_count = 0;
            for &polygon_index in polygon_indices {
                for identity in polygons[polygon_index]
                    .known_vertex_identities()
                    .expect("validated above")
                {
                    let ConstructionVertexIdentity::Source { vertex, .. } = identity else {
                        unreachable!("validated source vertex identity");
                    };
                    single_use_vertices[source_vertex_count] = vertex;
                    source_vertex_count += 1;
                }
            }
            let source_vertices = &mut single_use_vertices[..source_vertex_count];
            source_vertices.sort_unstable();
            let mut vertex_index = 0;
            while vertex_index < source_vertices.len() {
                let mut next = vertex_index + 1;
                while next < source_vertices.len()
                    && source_vertices[next] == source_vertices[vertex_index]
                {
                    next += 1;
                }
                // Its sole incident source triangle contains both boundary
                // neighbors. The supported input model excludes degenerate
                // source triangles, so this vertex is certifiably a corner.
                if next == vertex_index + 1 {
                    source_vertices[single_use_vertex_count] = source_vertices[vertex_index];
                    single_use_vertex_count += 1;
                }
                vertex_index = next;
            }
        }
        let boundary_edges = source_edges.into_unique_occurrences();
        let mut outgoing = Vec::with_capacity(boundary_edges.len());
        for occurrence in boundary_edges {
            let (group_polygon_index, edge_index) = (occurrence / 3, occurrence % 3);
            let polygon_index = polygon_indices[group_polygon_index];
            let polygon = &polygons[polygon_index];
            let edge_identities = polygon.known_edge_identities().expect("validated above");
            let vertex_identities = polygon.known_vertex_identities().expect("validated above");
            let points = polygon.known_vertices.as_ref().expect("validated above");
            let edge_identity = edge_identities
                .get(edge_index)
                .expect("validated edge identity index");
            let ConstructionVertexIdentity::Source { vertex: start, .. } = vertex_identities
                .get(edge_index)
                .expect("validated vertex identity index")
            else {
                return Err(crate::error::HypermeshError::UnknownClassification);
            };
            let ConstructionVertexIdentity::Source { vertex: end, .. } = vertex_identities
                .get((edge_index + 1) % vertex_identities.len())
                .expect("validated vertex identity index")
            else {
                return Err(crate::error::HypermeshError::UnknownClassification);
            };
            let edge_plane = if polygon.edges.len() == edge_identities.len() {
                Some(&polygon.edges[edge_index])
            } else if polygon.vertex_count() == 3 {
                None
            } else {
                return Err(crate::error::HypermeshError::UnknownClassification);
            };
            outgoing.push((
                start,
                end,
                points.get(edge_index).expect("aligned vertex"),
                edge_plane,
                edge_identity,
            ));
        }
        outgoing.sort_unstable_by_key(|entry| entry.0);
        if outgoing.windows(2).any(|pair| pair[0].0 == pair[1].0) {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let Some(mut current) = outgoing.first().map(|entry| entry.0) else {
            return Err(crate::error::HypermeshError::UnknownClassification);
        };
        let start = current;
        let mut face_vertices = Vec::with_capacity(outgoing.len());
        let mut vertex_identities = Vec::with_capacity(outgoing.len());
        let mut edge_planes = Vec::new();
        let mut edge_planes_complete = true;
        let mut edge_identities = Vec::with_capacity(outgoing.len());
        while face_vertices.len() < outgoing.len() {
            let Ok(outgoing_index) = outgoing.binary_search_by_key(&current, |entry| entry.0)
            else {
                return Err(crate::error::HypermeshError::UnknownClassification);
            };
            let (_, next, point, edge_plane, edge_identity) = &outgoing[outgoing_index];
            face_vertices.push(*point);
            vertex_identities.push(ConstructionVertexIdentity::Source {
                mesh: support_identity.mesh,
                vertex: current,
            });
            if edge_planes_complete {
                if let Some(edge_plane) = edge_plane {
                    if edge_planes.is_empty() {
                        edge_planes.reserve(outgoing.len());
                    }
                    edge_planes.push((*edge_plane).clone());
                } else {
                    edge_planes.clear();
                    edge_planes_complete = false;
                }
            }
            edge_identities.push(edge_identity.clone());
            current = *next;
            if current == start {
                break;
            }
        }
        if current != start || face_vertices.len() != outgoing.len() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let certified_noncollinear_source_vertices = (single_use_vertex_count != 0)
            .then_some(&single_use_vertices[..single_use_vertex_count]);
        collapse_certified_collinear_face_vertices(
            decisions,
            support_identity.mesh,
            support_planes[support_identity.mesh][support_identity.plane],
            &mut face_vertices,
            &mut vertex_identities,
            &mut edge_planes,
            &mut edge_identities,
            certified_noncollinear_source_vertices,
        )?;
        let source = &polygons[polygon_indices[0]];
        let mesh_index = usize::try_from(source.mesh_index)
            .ok()
            .filter(|&mesh| mesh < support_planes.len())
            .ok_or(crate::error::HypermeshError::UnknownClassification)?;
        // Every retained input polygon for a source mesh indexes the same
        // position arena. The source identities validated above can therefore
        // retain that arena instead of cloning the merged face's points.
        let indexed_positions = source
            .known_vertices
            .as_ref()
            .and_then(|vertices| vertices.source_positions())
            .cloned();
        let mut delta_w = vec![0; support_planes.len()];
        delta_w[mesh_index] = 1;
        faces.push(ConvexPolygon::from_certified_convex_face(
            support_planes[support_identity.mesh][support_identity.plane].clone(),
            &face_vertices,
            indexed_positions,
            vertex_identities,
            edge_planes,
            edge_identities,
            source.mesh_index,
            source.polygon_index,
            delta_w,
        ));
        face_supports.push(support_identity);
    }
    Ok((faces, face_supports))
}

enum SourceEdgeOccurrences {
    Packed {
        endpoint_bits: u32,
        occurrence_bits: u32,
        edges: Vec<u64>,
    },
    Wide(Vec<([usize; 2], usize)>),
}

impl SourceEdgeOccurrences {
    const PACKED_SORT_MIN_EDGES: usize = 1;

    fn with_capacity(capacity: usize, vertex_count: usize) -> Self {
        let endpoint_bits = usize::BITS - vertex_count.saturating_sub(1).leading_zeros();
        let occurrence_bits = usize::BITS - capacity.saturating_sub(1).leading_zeros();
        if vertex_count > 0
            && capacity >= Self::PACKED_SORT_MIN_EDGES
            && occurrence_bits < u64::BITS
            && endpoint_bits
                .saturating_mul(2)
                .saturating_add(occurrence_bits)
                <= u64::BITS
        {
            Self::Packed {
                endpoint_bits,
                occurrence_bits,
                edges: Vec::with_capacity(capacity),
            }
        } else {
            Self::Wide(Vec::with_capacity(capacity))
        }
    }

    fn push(&mut self, edge: [usize; 2]) {
        if let Self::Packed {
            endpoint_bits,
            occurrence_bits,
            edges,
        } = self
        {
            let occurrence = edges.len() as u64;
            let key = ((edge[0] as u64) << *endpoint_bits) | edge[1] as u64;
            edges.push((key << *occurrence_bits) | occurrence);
            return;
        }
        let Self::Wide(wide) = self else {
            unreachable!("packed edge path returns above");
        };
        wide.push((edge, wide.len()));
    }

    fn into_unique_occurrences(self) -> UniqueSourceEdgeOccurrences {
        match self {
            Self::Packed {
                endpoint_bits: _,
                occurrence_bits,
                mut edges,
            } => {
                edges.sort_unstable();
                let occurrence_mask = (1_u64 << occurrence_bits) - 1;
                let len = unique_sorted_runs_by_key(&edges, |edge| edge >> occurrence_bits).count();
                UniqueSourceEdgeOccurrences::Packed {
                    occurrence_bits,
                    occurrence_mask,
                    edges,
                    index: 0,
                    len,
                }
            }
            Self::Wide(mut edges) => {
                edges.sort_unstable_by_key(|entry| entry.0);
                UniqueSourceEdgeOccurrences::Wide(
                    unique_sorted_runs_by_key(&edges, |entry| entry.0)
                        .map(|entry| entry.1)
                        .collect::<Vec<_>>()
                        .into_iter(),
                )
            }
        }
    }
}

enum UniqueSourceEdgeOccurrences {
    Packed {
        occurrence_bits: u32,
        occurrence_mask: u64,
        edges: Vec<u64>,
        index: usize,
        len: usize,
    },
    Wide(std::vec::IntoIter<usize>),
}

impl ExactSizeIterator for UniqueSourceEdgeOccurrences {
    fn len(&self) -> usize {
        match self {
            Self::Packed { len, .. } => *len,
            Self::Wide(occurrences) => occurrences.len(),
        }
    }
}

impl Iterator for UniqueSourceEdgeOccurrences {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Self::Packed {
                occurrence_bits,
                occurrence_mask,
                edges,
                index,
                len,
            } => {
                while *index < edges.len() {
                    let current = *index;
                    *index += 1;
                    let key = edges[current] >> *occurrence_bits;
                    while *index < edges.len() && edges[*index] >> *occurrence_bits == key {
                        *index += 1;
                    }
                    if *index == current + 1 {
                        *len -= 1;
                        return Some((edges[current] & *occurrence_mask) as usize);
                    }
                }
                debug_assert_eq!(*len, 0);
                None
            }
            Self::Wide(occurrences) => occurrences.next(),
        }
    }
}

fn unique_sorted_runs_by_key<T, K: Eq>(
    values: &[T],
    key: impl Fn(&T) -> K,
) -> impl Iterator<Item = &T> {
    let mut index = 0;
    std::iter::from_fn(move || {
        while index < values.len() {
            let current = index;
            index += 1;
            while index < values.len() && key(&values[index]) == key(&values[current]) {
                index += 1;
            }
            if index == current + 1 {
                return Some(&values[current]);
            }
        }
        None
    })
}

#[inline]
fn retain_indices<T>(values: &mut Vec<T>, retained: &[usize]) {
    let mut source_index = 0;
    let mut retained_index = 0;
    values.retain(|_| {
        let keep = retained.get(retained_index) == Some(&source_index);
        source_index += 1;
        retained_index += usize::from(keep);
        keep
    });
    debug_assert_eq!(retained_index, retained.len());
}

fn collapse_certified_collinear_face_vertices<P: Borrow<Point3>>(
    decisions: &DecisionContext,
    mesh: usize,
    support: &Plane,
    vertices: &mut Vec<P>,
    vertex_identities: &mut Vec<ConstructionVertexIdentity>,
    edges: &mut Vec<Plane>,
    edge_identities: &mut Vec<ConstructionEdgeIdentity>,
    certified_noncollinear_source_vertices: Option<&[usize]>,
) -> HypermeshResult<()> {
    let len = vertices.len();
    if vertex_identities.len() != len
        || (!edges.is_empty() && edges.len() != len)
        || edge_identities.len() != len
    {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
    if len <= 3 {
        return Ok(());
    }
    let rebuild_edge_planes = !edges.is_empty();
    if len > 3 {
        // Keep ordinary merged-face cycles allocation-free without imposing a
        // size limit on exact inputs.
        const STACK_RETAINED_VERTICES: usize = 32;
        let mut retained_stack = [0; STACK_RETAINED_VERTICES];
        let mut retained_heap = Vec::new();
        let mut retained_len = 0;
        for index in 0..len {
            let certified_noncollinear =
                certified_noncollinear_source_vertices.is_some_and(|certified| {
                    let ConstructionVertexIdentity::Source {
                        mesh: vertex_mesh,
                        vertex,
                    } = vertex_identities[index]
                    else {
                        return false;
                    };
                    vertex_mesh == mesh && certified.contains(&vertex)
                });
            let keep = if certified_noncollinear {
                true
            } else {
                !support.points_are_collinear_on_support(
                    decisions,
                    vertices[(index + len - 1) % len].borrow(),
                    vertices[index].borrow(),
                    vertices[(index + 1) % len].borrow(),
                )?
            };
            if keep {
                if retained_len < retained_stack.len() {
                    retained_stack[retained_len] = index;
                } else {
                    if retained_heap.is_empty() {
                        retained_heap = Vec::with_capacity(len);
                        retained_heap.extend_from_slice(&retained_stack);
                    }
                    retained_heap.push(index);
                }
                retained_len += 1;
            }
        }
        let retained = if retained_len <= retained_stack.len() {
            &retained_stack[..retained_len]
        } else {
            &retained_heap
        };
        if retained.len() < 3 {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        if retained.len() == len {
            return Ok(());
        }
        retain_indices(vertices, retained);
        retain_indices(vertex_identities, retained);
    }

    edges.clear();
    edge_identities.clear();
    if rebuild_edge_planes {
        edges.reserve(vertices.len());
    }
    edge_identities.reserve(vertices.len());
    for index in 0..vertices.len() {
        let next = (index + 1) % vertices.len();
        if rebuild_edge_planes {
            let after_next = (index + 2) % vertices.len();
            edges.push(edge_plane(
                decisions,
                vertices[index].borrow(),
                vertices[next].borrow(),
                vertices[after_next].borrow(),
                support,
            )?);
        }
        let ConstructionVertexIdentity::Source {
            mesh: start_mesh,
            vertex: start,
        } = vertex_identities[index]
        else {
            return Err(crate::error::HypermeshError::UnknownClassification);
        };
        let ConstructionVertexIdentity::Source {
            mesh: end_mesh,
            vertex: end,
        } = vertex_identities[next]
        else {
            return Err(crate::error::HypermeshError::UnknownClassification);
        };
        if start_mesh != mesh || end_mesh != mesh {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let mut endpoints = [start, end];
        endpoints.sort_unstable();
        edge_identities.push(ConstructionEdgeIdentity::Source { mesh, endpoints });
    }
    Ok(())
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

// A non-ZST Vec cannot reach usize::MAX, so this cannot name an arena entry.
const NO_APPROXIMATE_SUPPORT_COLLISION: usize = usize::MAX;

// The floating-point key only proposes candidates. Every returned support is
// still verified by exact Plane equality. Keep the common unique-key case
// inline so it needs neither a child allocation nor a three-word Vec value.
#[derive(Default)]
struct ApproximateSupportPlaneIndex {
    buckets: StorageHashMap<[u64; 4], ApproximateSupportPlaneBucket>,
    collisions: Vec<ApproximateSupportPlaneCollision>,
}

struct ApproximateSupportPlaneBucket {
    first: usize,
    collision_head: usize,
}

struct ApproximateSupportPlaneCollision {
    support: usize,
    next: usize,
}

impl ApproximateSupportPlaneBucket {
    fn find(
        &self,
        collisions: &[ApproximateSupportPlaneCollision],
        support_planes: &[&Plane],
        support: &Plane,
    ) -> Option<usize> {
        if support_planes[self.first] == support {
            return Some(self.first);
        }
        let mut collision = self.collision_head;
        while collision != NO_APPROXIMATE_SUPPORT_COLLISION {
            let entry = &collisions[collision];
            if support_planes[entry.support] == support {
                return Some(entry.support);
            }
            collision = entry.next;
        }
        None
    }
}

impl ApproximateSupportPlaneIndex {
    fn intern<'a>(
        &mut self,
        key: [u64; 4],
        support_planes: &mut Vec<&'a Plane>,
        support: &'a Plane,
    ) -> (usize, bool) {
        let collisions = &mut self.collisions;
        match self.buckets.entry(key) {
            Entry::Occupied(mut entry) => {
                if let Some(index) = entry.get().find(collisions, support_planes, support) {
                    return (index, false);
                }
                let bucket = entry.get_mut();
                let index = support_planes.len();
                support_planes.push(support);
                let collision = collisions.len();
                collisions.push(ApproximateSupportPlaneCollision {
                    support: index,
                    next: bucket.collision_head,
                });
                bucket.collision_head = collision;
                (index, true)
            }
            Entry::Vacant(entry) => {
                let index = support_planes.len();
                support_planes.push(support);
                entry.insert(ApproximateSupportPlaneBucket {
                    first: index,
                    collision_head: NO_APPROXIMATE_SUPPORT_COLLISION,
                });
                (index, true)
            }
        }
    }
}

fn exact_plane_f64(decisions: &DecisionContext, plane: &Plane) -> Option<[f64; 4]> {
    let evidence = RationalPlane4PredicateEvidence::new(decisions, plane)?;
    // Reuse the certified filter's positive-scale normalization for the
    // floating proposal only; every proposed cycle is verified exactly.
    if let Some(coefficients) = evidence.normalized_coefficients() {
        return Some(coefficients);
    }
    let coefficients = [
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ];
    let [Some(a), Some(b), Some(c), Some(d)] = coefficients.map(Real::to_f64_lossy) else {
        return None;
    };
    Some([a, b, c, d])
}

fn exact_inside_and_outside_cycles<'a>(
    decisions: &DecisionContext,
    source: ProjectiveCycle,
    source_polygon: &ConvexPolygon,
    source_plane: ConstructionPlaneIdentity,
    support_planes: &[&'a Plane],
    support_planes_f64: Option<&[[f64; 4]]>,
    candidate_planes: &[usize],
    support_plane_mesh: usize,
    retain_outside: bool,
    verification_plane_evidence: &mut Vec<(usize, Option<RationalPlane4PredicateEvidence<'a>>)>,
    active_plane_proposal_scratch: &mut ActivePlaneProposalScratch,
    point_cache: &mut ProjectivePointCache,
) -> HypermeshResult<Option<(ProjectiveCycle, Option<Vec<ProjectiveCycle>>)>> {
    let mut source = Some(source);
    if let Some(proposed_planes) = support_planes_f64.and_then(|planes| {
        propose_active_planes_f64(
            source
                .as_ref()
                .expect("projective source is available for proposal"),
            planes,
            candidate_planes,
            active_plane_proposal_scratch,
        )
    }) {
        crate::trace_dispatch!("projective-active-planes", "proposed");
        let (inside, outside) = clip_inside_cycle_for_output(
            decisions,
            source
                .take()
                .expect("projective source is available for proposal"),
            support_planes,
            proposed_planes,
            support_plane_mesh,
            retain_outside,
            point_cache,
        )
        .inspect_err(|_error| {
            if cfg!(debug_assertions) {
                eprintln!("[DEBUG] proposed projective clipping failed");
            }
        })?;
        if inside.boundary.len() < 3 {
            crate::trace_dispatch!("projective-active-planes", "proposed-empty");
        } else {
            if cycle_satisfies_planes(
                decisions,
                &inside,
                support_planes,
                candidate_planes,
                proposed_planes,
                support_plane_mesh,
                verification_plane_evidence,
                point_cache,
            )
            .inspect_err(|_error| {
                if cfg!(debug_assertions) {
                    eprintln!("[DEBUG] proposed projective verification failed");
                }
            })? {
                crate::trace_dispatch!("projective-active-planes", "proposed-certified");
                return Ok(Some((inside, outside)));
            }
            crate::trace_dispatch!("projective-active-planes", "proposed-rejected");
        }
        source = Some(ProjectiveCycle::from_polygon(
            source_polygon,
            source_plane,
            point_cache,
        )?);
    }

    crate::trace_dispatch!("projective-active-planes", "full");
    let (inside, outside) = clip_inside_cycle_for_output(
        decisions,
        source
            .take()
            .expect("projective source is available for full clipping"),
        support_planes,
        candidate_planes,
        support_plane_mesh,
        retain_outside,
        point_cache,
    )
    .inspect_err(|_error| {
        if cfg!(debug_assertions) {
            eprintln!("[DEBUG] full projective clipping failed");
        }
    })?;
    if inside.boundary.len() < 3 {
        crate::trace_dispatch!("projective-active-planes", "full-empty");
        return Ok(None);
    }
    crate::trace_dispatch!("projective-active-planes", "full-retained");
    Ok(Some((inside, outside)))
}

fn clip_inside_cycle_for_output(
    decisions: &DecisionContext,
    source: ProjectiveCycle,
    support_planes: &[&Plane],
    plane_indices: &[usize],
    support_plane_mesh: usize,
    retain_outside: bool,
    point_cache: &mut ProjectivePointCache,
) -> HypermeshResult<(ProjectiveCycle, Option<Vec<ProjectiveCycle>>)> {
    if retain_outside {
        let (inside, outside) = partition_inside_cycle(
            decisions,
            source,
            support_planes,
            plane_indices,
            support_plane_mesh,
            point_cache,
        )?;
        Ok((inside, Some(outside)))
    } else {
        Ok((
            clip_inside_cycle(
                decisions,
                source,
                support_planes,
                plane_indices,
                support_plane_mesh,
                point_cache,
            )?,
            None,
        ))
    }
}

fn clip_inside_cycle(
    decisions: &DecisionContext,
    source: ProjectiveCycle,
    support_planes: &[&Plane],
    plane_indices: &[usize],
    support_plane_mesh: usize,
    point_cache: &mut ProjectivePointCache,
) -> HypermeshResult<ProjectiveCycle> {
    let mut inside = source;
    for &plane_index in plane_indices {
        inside = inside.clip_negative(
            decisions,
            support_planes[plane_index],
            ConstructionPlaneIdentity {
                mesh: support_plane_mesh,
                plane: plane_index,
            },
            point_cache,
        )?;
        if inside.boundary.len() < 3 {
            return Ok(ProjectiveCycle::empty());
        }
    }
    Ok(inside)
}

fn partition_inside_cycle(
    decisions: &DecisionContext,
    source: ProjectiveCycle,
    support_planes: &[&Plane],
    plane_indices: &[usize],
    support_plane_mesh: usize,
    point_cache: &mut ProjectivePointCache,
) -> HypermeshResult<(ProjectiveCycle, Vec<ProjectiveCycle>)> {
    let mut inside = source;
    let mut outside = Vec::new();
    for &plane_index in plane_indices {
        let clipped = inside.clip(
            decisions,
            support_planes[plane_index],
            ConstructionPlaneIdentity {
                mesh: support_plane_mesh,
                plane: plane_index,
            },
            point_cache,
        )?;
        match clipped.side {
            ProjectiveClipSide::Negative => inside = clipped.negative,
            ProjectiveClipSide::Positive => {
                outside.push(clipped.positive);
                return Ok((ProjectiveCycle::empty(), outside));
            }
            ProjectiveClipSide::Both => {
                outside.push(clipped.positive);
                inside = clipped.negative;
            }
        }
        if inside.boundary.len() < 3 {
            return Ok((ProjectiveCycle::empty(), outside));
        }
    }
    Ok((inside, outside))
}

fn cycle_satisfies_planes<'a>(
    decisions: &DecisionContext,
    cycle: &ProjectiveCycle,
    support_planes: &[&'a Plane],
    plane_indices: &[usize],
    excluded_planes: &[usize],
    support_plane_mesh: usize,
    rational_plane_evidence: &mut Vec<(usize, Option<RationalPlane4PredicateEvidence<'a>>)>,
    point_cache: &ProjectivePointCache,
) -> HypermeshResult<bool> {
    debug_assert!(plane_indices.is_sorted());
    debug_assert!(excluded_planes.is_sorted());
    rational_plane_evidence.clear();
    let mut excluded_index = 0;
    for &plane_index in plane_indices {
        while excluded_planes
            .get(excluded_index)
            .is_some_and(|&excluded| excluded < plane_index)
        {
            excluded_index += 1;
        }
        if excluded_planes.get(excluded_index) == Some(&plane_index) {
            excluded_index += 1;
            continue;
        }
        rational_plane_evidence.push((
            plane_index,
            RationalPlane4PredicateEvidence::new(decisions, support_planes[plane_index]),
        ));
    }
    for entry in &cycle.boundary {
        let point_evidence = ProjectivePoint3PredicateEvidence::with_rational_filter_query(
            point_cache.point(entry.point_index),
            entry.evidence.rational_filter_query,
        );
        for (plane_index, plane_evidence) in rational_plane_evidence.iter() {
            let classification = match plane_evidence {
                Some(plane) => match point_evidence.classify_rational_plane_filter(plane) {
                    Some(classification) => classification,
                    None if point_cache.has_incidence(
                        &entry.point_identity,
                        ConstructionPlaneIdentity {
                            mesh: support_plane_mesh,
                            plane: *plane_index,
                        },
                    ) =>
                    {
                        Classification::On
                    }
                    None => match point_evidence.classify_rational_plane_exact(plane) {
                        Some(classification) => classification,
                        None => point_evidence.classify(decisions, support_planes[*plane_index])?,
                    },
                },
                None => point_evidence.classify(decisions, support_planes[*plane_index])?,
            };
            if classification.is_positive() {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

#[derive(Default)]
struct ActivePlaneProposalScratch {
    cycle: Vec<[f64; 3]>,
    clipped: Vec<[f64; 3]>,
    crossed_planes: Vec<usize>,
    active: Vec<usize>,
}

fn propose_active_planes_f64<'a>(
    source: &ProjectiveCycle,
    planes: &[[f64; 4]],
    candidate_planes: &[usize],
    scratch: &'a mut ActivePlaneProposalScratch,
) -> Option<&'a [usize]> {
    scratch.cycle.clear();
    for entry in &source.boundary {
        scratch.cycle.push(entry.evidence.approximate?);
    }
    scratch.crossed_planes.clear();
    scratch.active.clear();
    for &plane_index in candidate_planes {
        let crossed =
            clip_f64_cycle_into(&scratch.cycle, planes[plane_index], &mut scratch.clipped);
        std::mem::swap(&mut scratch.cycle, &mut scratch.clipped);
        if scratch.cycle.len() < 3 {
            return Some(&scratch.active);
        }
        if crossed {
            scratch.crossed_planes.push(plane_index);
        }
    }
    for &plane_index in &scratch.crossed_planes {
        let plane = planes[plane_index];
        let points_on_plane = scratch
            .cycle
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
            scratch.active.push(plane_index);
        }
    }
    Some(&scratch.active)
}

fn clip_f64_cycle_into(points: &[[f64; 3]], plane: [f64; 4], clipped: &mut Vec<[f64; 3]>) -> bool {
    clipped.clear();
    clipped.reserve(points.len() + 1);
    let mut crossed = false;
    let Some(first) = points.first() else {
        return crossed;
    };
    let first_value = f64_plane_value(*first, plane);
    let mut current_value = first_value;
    for index in 0..points.len() {
        let next = (index + 1) % points.len();
        let next_value = if next == 0 {
            first_value
        } else {
            f64_plane_value(points[next], plane)
        };
        let current_inside = current_value <= 0.0;
        let next_inside = next_value <= 0.0;
        match (current_inside, next_inside) {
            (true, true) => clipped.push(points[next]),
            (true, false) => {
                crossed = true;
                clipped.push(f64_segment_plane_intersection(
                    points[index],
                    points[next],
                    current_value,
                    next_value,
                ));
            }
            (false, true) => {
                crossed = true;
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
        current_value = next_value;
    }
    crossed
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
    plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3]
}

fn positive_weight_plane_intersection(planes: [&Plane; 3]) -> Option<HomogeneousPoint3> {
    let point = intersect_three_planes(planes[0], planes[1], planes[2]);
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
    let exact_dyadic = [
        positive_value,
        negative_value,
        &negative.x,
        &negative.y,
        &negative.z,
        &negative.w,
        &positive.x,
        &positive.y,
        &positive.z,
        &positive.w,
    ]
    .into_iter()
    .all(|value| value.exact_rational_ref().is_some_and(Rational::is_dyadic));
    let coordinate = |negative_coordinate: &Real, positive_coordinate: &Real| {
        let terms = [
            [positive_value, negative_coordinate],
            [negative_value, positive_coordinate],
        ];
        if exact_dyadic {
            Real::exact_rational_signed_product_sum_known_dyadic([true, false], terms)
        } else {
            Real::signed_product_sum([true, false], terms)
        }
    };
    HomogeneousPoint3::new(
        coordinate(&negative.x, &positive.x),
        coordinate(&negative.y, &positive.y),
        coordinate(&negative.z, &positive.z),
        coordinate(&negative.w, &positive.w),
    )
}

fn push_projective_transition(
    decisions: &DecisionContext,
    classified: &mut Vec<ClassifiedPolygon>,
    cycle: ProjectiveCycle,
    source: &ConvexPolygon,
    affine_cache: &mut ProjectiveAffineCache,
    point_cache: &ProjectivePointCache,
    host: usize,
    other: usize,
    inside_other: bool,
    operation: BooleanOp,
    retain_winding: bool,
) -> HypermeshResult<()> {
    if cycle.boundary.len() < 3 {
        return Ok(());
    }
    if !projective_transition_is_emitted(host, inside_other, operation) {
        return Ok(());
    }
    let polygon = cycle.materialize(decisions, source, affine_cache, point_cache)?;
    let classification = if retain_winding {
        ARRANGEMENT_CLASSIFICATION
    } else {
        projective_transition_classification(host, other, inside_other, operation)
    };
    let mut fragment = ClassifiedPolygon::new(polygon, classification);
    if retain_winding {
        fragment.winding = Some(projective_transition_winding(host, other, inside_other));
    }
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
    operation: BooleanOp,
    retain_winding: bool,
) -> HypermeshResult<()> {
    if source.vertex_count() < 3 {
        return Ok(());
    }
    let classification = if retain_winding {
        ARRANGEMENT_CLASSIFICATION
    } else {
        projective_transition_classification(host, other, inside_other, operation)
    };
    let mut fragment = ClassifiedPolygon::new(source.clone(), classification);
    if retain_winding {
        fragment.winding = Some(projective_transition_winding(host, other, inside_other));
    }
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

fn projective_transition_classification(
    host: usize,
    other: usize,
    inside_other: bool,
    operation: BooleanOp,
) -> i8 {
    let mut front = [0; 2];
    front[other] = i32::from(inside_other);
    let mut back = front;
    back[host] = 1;
    let is_inside = |winding: [i32; 2]| match operation {
        BooleanOp::Union => winding != [0, 0],
        BooleanOp::Intersection => winding[0] != 0 && winding[1] != 0,
        BooleanOp::Difference => winding[0] != 0 && winding[1] == 0,
        BooleanOp::SymmetricDifference => (winding[0] != 0) != (winding[1] != 0),
    };
    match (is_inside(front), is_inside(back)) {
        (false, true) => 1,
        (true, false) => -1,
        _ => 0,
    }
}

fn projective_transition_is_emitted(host: usize, inside_other: bool, operation: BooleanOp) -> bool {
    match operation {
        BooleanOp::Union => !inside_other,
        BooleanOp::Intersection => inside_other,
        BooleanOp::Difference => (host == 0 && !inside_other) || (host == 1 && inside_other),
        BooleanOp::SymmetricDifference => true,
    }
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
    use crate::test_support::approximate_convex_triangle;

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn signed_six_volume(mesh: &BooleanMesh) -> Real {
        let mut volume = Real::zero();
        for triangle in &mesh.triangles {
            let a = &mesh.vertices[triangle[0]];
            let b = &mesh.vertices[triangle[1]];
            let c = &mesh.vertices[triangle[2]];
            volume += &a.x * &(&b.y * &c.z - &b.z * &c.y)
                + &a.y * &(&b.z * &c.x - &b.x * &c.z)
                + &a.z * &(&b.x * &c.y - &b.y * &c.x);
        }
        volume
    }

    #[test]
    fn boolean_materialization_removes_exact_duplicate_faces_before_certification() {
        let mut soup = crate::output::BooleanMesh {
            vertices: vec![
                crate::OutputVertex {
                    x: Real::zero(),
                    y: Real::zero(),
                    z: Real::zero(),
                },
                crate::OutputVertex {
                    x: Real::one(),
                    y: Real::zero(),
                    z: Real::zero(),
                },
                crate::OutputVertex {
                    x: Real::zero(),
                    y: Real::one(),
                    z: Real::zero(),
                },
                crate::OutputVertex {
                    x: Real::zero(),
                    y: Real::zero(),
                    z: Real::one(),
                },
            ],
            triangles: vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
            sources: vec![crate::TriangleSource::default(); 4],
        };
        soup.triangles.extend(soup.triangles.clone());
        soup.sources.extend(soup.sources.clone());

        let certified =
            certify_boolean_mesh_closure(&crate::test_support::approximate_decisions(), soup)
                .unwrap();

        assert_eq!(certified.triangles.len(), 4);
        assert!(
            certified
                .has_unique_nondegenerate_triangles_decision(
                    &crate::test_support::approximate_decisions()
                )
                .unwrap()
        );
        assert!(crate::output::boolean_mesh_closure_evidence(&certified).is_closed());
    }

    #[test]
    fn touching_certified_convex_intersection_regularizes_to_empty() {
        let tetrahedron = TriangleMesh::new(
            vec![p(0, 1, 2), p(4, 1, 2), p(0, 2, 2), p(0, 1, 4)],
            vec![
                Triangle::new(0, 2, 1),
                Triangle::new(0, 1, 3),
                Triangle::new(0, 3, 2),
                Triangle::new(1, 2, 3),
            ],
        )
        .with_certified_convexity();
        let box_mesh = box_from_bounds(&hyperlattice::Aabb::new(p(-1, -3, 2), p(3, 1, 3)))
            .with_certified_convexity();
        let raw = [
            TriangleMeshRef::new(&tetrahedron.positions, &tetrahedron.triangles),
            TriangleMeshRef::new(&box_mesh.positions, &box_mesh.triangles),
        ];
        let certified = [tetrahedron.as_ref(), box_mesh.as_ref()];

        for policy in [
            crate::PredicatePolicy::STRICT,
            crate::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            for meshes in [&raw, &certified] {
                let result = boolean_operation(
                    &context,
                    meshes,
                    BooleanOp::Intersection,
                    EmberConfig::default(),
                )
                .unwrap();
                assert_eq!(result.certainty, crate::MeshCertainty::Certified);
                let materialized =
                    crate::triangulate_and_resolve_certified(&context, &result.into_value())
                        .unwrap()
                        .into_value();
                assert!(materialized.triangles.is_empty(), "{materialized:#?}");
            }

            let immediate = boolean_mesh(
                &context,
                &certified,
                BooleanOp::Intersection,
                EmberConfig::default(),
            )
            .unwrap();
            assert_eq!(immediate.certainty, crate::MeshCertainty::Certified);
            assert!(
                immediate.value.triangles.is_empty(),
                "{:#?}",
                immediate.value
            );
        }
    }

    #[test]
    fn native_finalization_revalidates_a_structurally_invalid_convex_fact() {
        let box_mesh = box_from_bounds(&hyperlattice::Aabb::new(p(-2, -2, -2), p(2, 2, 2)));
        let mut triangles = box_mesh.triangles.to_vec();
        triangles.pop();
        let invalid =
            TriangleMesh::new(box_mesh.positions.to_vec(), triangles).with_certified_convexity();

        for policy in [
            crate::PredicatePolicy::STRICT,
            crate::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            for operation in [
                BooleanOp::Union,
                BooleanOp::Intersection,
                BooleanOp::Difference,
                BooleanOp::SymmetricDifference,
            ] {
                let meshes = [invalid.as_ref()];
                for error in [
                    boolean_operation(&context, &meshes, operation, EmberConfig::default())
                        .unwrap_err(),
                    boolean_mesh(&context, &meshes, operation, EmberConfig::default()).unwrap_err(),
                ] {
                    assert_eq!(
                        error,
                        HypermeshError::OpenInput {
                            mesh_index: 0,
                            boundary_edges: 3,
                        }
                    );
                }
            }
        }
    }

    #[test]
    fn symbolically_translated_box_xor_obeys_terminal_policy() {
        let base = Real::e();
        let point = |offset: i64| {
            Point3::new(
                &base + Real::from(offset),
                &base + Real::from(offset),
                &base + Real::from(offset),
            )
        };
        let left = box_from_bounds(&hyperlattice::Aabb::new(point(0), point(3)));
        let right = box_from_bounds(&hyperlattice::Aabb::new(point(-1), point(2)));
        let meshes = [
            TriangleMeshRef::new(&left.positions, &left.triangles),
            TriangleMeshRef::new(&right.positions, &right.triangles),
        ];

        let operation = |context| {
            boolean_operation(
                context,
                &meshes,
                BooleanOp::SymmetricDifference,
                EmberConfig::default(),
            )
        };
        let strict = MeshContext::new(crate::PredicatePolicy::STRICT);
        assert!(matches!(
            operation(&strict),
            Err(HypermeshError::PredicateUndecided { .. })
        ));

        let context = MeshContext::new(crate::PredicatePolicy::APPROXIMATE_512);
        let result = operation(&context).unwrap();
        assert_eq!(
            result.certainty,
            crate::MeshCertainty::Approximate512Consumed
        );
        let soup = crate::triangulate_and_resolve_certified(&context, &result.into_value())
            .unwrap()
            .into_value();
        assert!(crate::output::boolean_mesh_closure_evidence(&soup).has_no_boundary());
        assert!(
            soup.has_unique_nondegenerate_triangles_decision(&DecisionContext::new(&context))
                .unwrap()
        );
    }

    #[test]
    fn algebraic_shortcuts_reject_invalid_nonempty_operands() {
        let context = MeshContext::new(crate::PredicatePolicy::STRICT);
        let open = TriangleMesh::new(
            vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)],
            vec![Triangle::new(0, 1, 2)],
        );
        let empty = TriangleMesh::new(Vec::new(), Vec::new());
        let distant_box = box_from_bounds(&hyperlattice::Aabb::new(p(10, 10, 10), p(11, 11, 11)));

        let identical = boolean_triangle_meshes(
            &context,
            &open,
            &open,
            BooleanOp::Union,
            EmberConfig::default(),
        );
        assert_eq!(
            identical.unwrap_err(),
            HypermeshError::OpenInput {
                mesh_index: 0,
                boundary_edges: 3,
            }
        );

        let empty_passthrough = boolean_triangle_meshes(
            &context,
            &empty,
            &open,
            BooleanOp::Union,
            EmberConfig::default(),
        );
        assert_eq!(
            empty_passthrough.unwrap_err(),
            HypermeshError::OpenInput {
                mesh_index: 1,
                boundary_edges: 3,
            }
        );

        let disjoint = boolean_triangle_meshes(
            &context,
            &open,
            &distant_box,
            BooleanOp::Union,
            EmberConfig::default(),
        );
        assert_eq!(
            disjoint.unwrap_err(),
            HypermeshError::OpenInput {
                mesh_index: 0,
                boundary_edges: 3,
            }
        );

        let degenerate = TriangleMesh::new(
            vec![p(0, 0, 0), p(1, 0, 0), p(2, 0, 0)],
            vec![Triangle::new(0, 1, 2)],
        );
        assert_eq!(
            boolean_triangle_meshes(
                &context,
                &degenerate,
                &degenerate,
                BooleanOp::Intersection,
                EmberConfig::default(),
            )
            .unwrap_err(),
            HypermeshError::DegenerateTriangle {
                mesh_index: 0,
                triangle_index: 0,
            }
        );
    }

    #[test]
    fn exact_box_cell_boolean_covers_every_operation_and_geometric_relation() {
        let context = MeshContext::new(crate::PredicatePolicy::STRICT);
        let decisions = DecisionContext::new(&context);
        let cases = [
            (
                "disjoint",
                hyperlattice::Aabb::new(p(0, 0, 0), p(1, 1, 1)),
                hyperlattice::Aabb::new(p(2, 0, 0), p(3, 1, 1)),
                [2_i64, 0, 1, 2],
                [false; 4],
            ),
            (
                "equal",
                hyperlattice::Aabb::new(p(0, 0, 0), p(2, 2, 2)),
                hyperlattice::Aabb::new(p(0, 0, 0), p(2, 2, 2)),
                [8, 8, 0, 0],
                [false; 4],
            ),
            (
                "left-contains-right",
                hyperlattice::Aabb::new(p(0, 0, 0), p(4, 4, 4)),
                hyperlattice::Aabb::new(p(1, 1, 1), p(3, 3, 3)),
                [64, 8, 56, 56],
                [false, false, true, true],
            ),
            (
                "right-contains-left",
                hyperlattice::Aabb::new(p(1, 1, 1), p(3, 3, 3)),
                hyperlattice::Aabb::new(p(0, 0, 0), p(4, 4, 4)),
                [64, 8, 0, 56],
                [false, false, false, true],
            ),
            (
                "adjacent",
                hyperlattice::Aabb::new(p(0, 0, 0), p(2, 2, 2)),
                hyperlattice::Aabb::new(p(2, 0, 0), p(4, 2, 2)),
                [16, 0, 8, 16],
                [true, false, false, true],
            ),
            (
                "partial-contact",
                hyperlattice::Aabb::new(p(0, 0, 0), p(2, 2, 2)),
                hyperlattice::Aabb::new(p(2, 1, 0), p(4, 3, 2)),
                [16, 0, 8, 16],
                [true, false, false, true],
            ),
            (
                "overlap",
                hyperlattice::Aabb::new(p(0, 0, 0), p(4, 4, 4)),
                hyperlattice::Aabb::new(p(2, 1, 1), p(6, 3, 3)),
                [72, 8, 56, 64],
                [true; 4],
            ),
        ];
        let operations = [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::SymmetricDifference,
        ];

        for (relation, left_bounds, right_bounds, expected_volumes, synthesized) in cases {
            let left = box_from_bounds(&left_bounds);
            let right = box_from_bounds(&right_bounds);
            for (index, operation) in operations.into_iter().enumerate() {
                let direct = axis_aligned_box_boolean_mesh(&decisions, &left, &right, operation)
                    .unwrap()
                    .expect("both inputs are certified axis-aligned boxes");
                let public = boolean_mesh(
                    &context,
                    &[left.as_ref(), right.as_ref()],
                    operation,
                    EmberConfig::default(),
                )
                .unwrap();
                assert_eq!(public.certainty, crate::MeshCertainty::Certified);
                assert_eq!(public.value, direct);
                assert!(
                    direct
                        .has_unique_nondegenerate_triangles_decision(&decisions)
                        .unwrap(),
                    "{relation}: {operation:?}"
                );
                let closure = crate::output::boolean_mesh_closure_evidence(&direct);
                assert!(
                    closure.has_no_boundary(),
                    "{relation}: {operation:?}: {closure:?}"
                );
                assert_eq!(
                    signed_six_volume(&direct),
                    Real::from(6 * expected_volumes[index]),
                    "{relation}: {operation:?}"
                );
                if !direct.triangles.is_empty() {
                    assert_eq!(
                        direct.sources.iter().all(|source| source.triangle == -1),
                        synthesized[index],
                        "{relation}: {operation:?}"
                    );
                }
                if relation == "overlap" && operation == BooleanOp::Difference {
                    assert!(
                        direct
                            .sources
                            .iter()
                            .any(|source| source.mesh == 1 && source.orientation == -1)
                    );
                }
            }
        }
    }

    #[test]
    fn exact_box_cell_boolean_obeys_terminal_equality_policy() {
        let left_boundary = Real::pi() + Real::e();
        let right_boundary = Real::e() + Real::pi();
        let left_bounds = hyperlattice::Aabb::new(
            Point3::origin(),
            Point3::new(left_boundary, Real::one(), Real::one()),
        );
        let right_bounds = hyperlattice::Aabb::new(
            Point3::new(right_boundary.clone(), Real::zero(), Real::zero()),
            Point3::new(&right_boundary + &Real::one(), Real::one(), Real::one()),
        );
        let left = box_from_bounds(&left_bounds);
        let right = box_from_bounds(&right_bounds);

        let strict_context = MeshContext::new(crate::PredicatePolicy::STRICT);
        let strict = DecisionContext::new(&strict_context);
        assert_eq!(
            axis_aligned_box_boolean_mesh(&strict, &left, &right, BooleanOp::Union).unwrap(),
            None
        );
        assert_eq!(strict.certainty(), crate::MeshCertainty::Certified);

        let approximate_context = MeshContext::new(crate::PredicatePolicy::APPROXIMATE_512);
        let approximate = boolean_mesh(
            &approximate_context,
            &[left.as_ref(), right.as_ref()],
            BooleanOp::Union,
            EmberConfig::default(),
        )
        .unwrap();
        assert_eq!(
            approximate.certainty,
            crate::MeshCertainty::Approximate512Consumed
        );
        assert!(crate::output::boolean_mesh_closure_evidence(&approximate.value).has_no_boundary());
    }

    #[test]
    fn iterated_native_booleans_retain_polygon_arrangements() {
        let bounds = |min_x, max_x| hyperlattice::Aabb::new(p(min_x, 0, 0), p(max_x, 4, 4));
        let block = box_from_bounds(&bounds(0, 6));
        let first_tool = box_from_bounds(&bounds(2, 4));
        let second_tool = box_from_bounds(&hyperlattice::Aabb::new(p(1, 1, 1), p(5, 3, 3)));

        let first = boolean_triangle_meshes_decision(
            &crate::test_support::approximate_decisions(),
            &block,
            &first_tool,
            BooleanOp::Difference,
            EmberConfig::default(),
        )
        .unwrap();
        assert!(first.is_closed_manifold());
        assert_eq!(
            first
                .retained_input_planes(&crate::test_support::approximate_decisions(),)
                .unwrap()
                .map(|planes| planes.len()),
            Some(first.triangles.len())
        );
        assert!(
            first
                .retained_input_polygons(&crate::test_support::approximate_decisions())
                .is_some_and(|polygons| !polygons.is_empty())
        );

        let second = boolean_triangle_meshes_decision(
            &crate::test_support::approximate_decisions(),
            &first,
            &second_tool,
            BooleanOp::Difference,
            EmberConfig::default(),
        )
        .unwrap();
        assert!(second.is_closed_manifold());
        assert!(
            second
                .retained_input_polygons(&crate::test_support::approximate_decisions())
                .is_some_and(|polygons| !polygons.is_empty())
        );
    }

    #[test]
    fn coextensive_overlapping_box_union_materializes_one_boundary() {
        let left = hyperlattice::Aabb::new(p(0, 0, 0), p(2, 2, 2));
        let right = hyperlattice::Aabb::new(p(1, 0, 0), p(3, 2, 2));

        let union =
            adjacent_box_union(&crate::test_support::approximate_decisions(), &right, &left)
                .unwrap()
                .expect("the overlapping boxes share a certifiable full face");

        assert_eq!(union, hyperlattice::Aabb::new(p(0, 0, 0), p(3, 2, 2)));
        let mesh = box_from_bounds(&union);
        assert!(
            mesh.has_unique_nondegenerate_triangles_decision(
                &crate::test_support::approximate_decisions()
            )
            .unwrap()
        );
        assert!(
            mesh.is_closed_manifold_geometry_decision(
                &crate::test_support::approximate_decisions()
            )
            .unwrap()
        );
    }

    #[test]
    fn direct_projective_classification_matches_winding_evidence() {
        for operation in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::SymmetricDifference,
        ] {
            for host in 0..2 {
                let other = 1 - host;
                for inside_other in [false, true] {
                    let winding = projective_transition_winding(host, other, inside_other);
                    let expected = crate::winding::classify_polygon_output(
                        &winding.w_front,
                        &winding.w_back,
                        operation,
                    );
                    assert_eq!(
                        projective_transition_classification(host, other, inside_other, operation,),
                        expected,
                    );
                }
            }
        }
    }

    #[test]
    fn projective_crossing_dyadic_batch_matches_general_exact_coordinates() {
        let rational = |numerator, denominator| {
            Real::from(Rational::fraction(numerator, denominator).unwrap())
        };
        let current = HomogeneousPoint3::new(
            rational(1, 2),
            rational(3, 4),
            rational(-5, 8),
            rational(1, 1),
        );
        let next = HomogeneousPoint3::new(
            rational(7, 8),
            rational(-9, 16),
            rational(11, 32),
            rational(3, 2),
        );
        let current_value = rational(-5, 4);
        let next_value = rational(7, 8);
        let actual = projective_crossing_point(
            &current,
            &current_value,
            Classification::Negative,
            &next,
            &next_value,
        );
        let expected = current
            .coordinates()
            .into_iter()
            .zip(next.coordinates())
            .map(|(negative, positive)| {
                Real::signed_product_sum(
                    [true, false],
                    [[&next_value, negative], [&current_value, positive]],
                )
            })
            .collect::<Vec<_>>();

        assert_eq!(
            actual.coordinates(),
            [&expected[0], &expected[1], &expected[2], &expected[3]],
        );
    }

    #[test]
    fn projective_source_edge_supports_are_unique_and_bounded() {
        let first = ConstructionPlaneIdentity { mesh: 0, plane: 4 };
        let second = ConstructionPlaneIdentity { mesh: 0, plane: 7 };
        let third = ConstructionPlaneIdentity { mesh: 0, plane: 9 };
        let mut source_edge = ProjectiveSourceEdge::new([2, 3], first);

        assert_eq!(source_edge.supports(), &[first]);
        assert!(source_edge.insert_support(first));
        assert!(source_edge.insert_support(second));
        assert_eq!(source_edge.supports(), &[first, second]);
        assert!(!source_edge.insert_support(third));
        assert_eq!(source_edge.supports(), &[first, second]);
    }

    #[test]
    fn source_edge_occurrences_preserve_unique_runs_across_packed_and_wide_storage() {
        let edges = [[1, 2], [4, 5], [2, 3], [1, 2], [4, 5], [4, 5]];
        let collect = |capacity| {
            let mut keys = SourceEdgeOccurrences::with_capacity(capacity, 6);
            for edge in edges {
                keys.push(edge);
            }
            keys.into_unique_occurrences().collect::<Vec<_>>()
        };

        assert_eq!(collect(edges.len()), vec![2]);

        if usize::BITS > u32::BITS {
            let oversized = usize::try_from(u64::from(u32::MAX) + 1).unwrap();
            let mut fallback = SourceEdgeOccurrences::with_capacity(
                SourceEdgeOccurrences::PACKED_SORT_MIN_EDGES,
                oversized + 2,
            );
            fallback.push([1, 2]);
            fallback.push([oversized, oversized + 1]);
            fallback.push([1, 2]);
            assert_eq!(
                fallback.into_unique_occurrences().collect::<Vec<_>>(),
                vec![1]
            );
        }
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
    fn plane_equivalence_separates_proportionality_from_orientation() {
        let plane =
            Plane::from_coefficients(Real::from(2), Real::from(4), Real::from(6), Real::from(8));
        let same =
            Plane::from_coefficients(Real::from(3), Real::from(6), Real::from(9), Real::from(12));
        let opposite = Plane::from_coefficients(
            Real::from(-3),
            Real::from(-6),
            Real::from(-9),
            Real::from(-12),
        );
        let distinct =
            Plane::from_coefficients(Real::from(3), Real::from(6), Real::from(9), Real::from(13));

        assert!(
            certifiably_same_oriented_plane(
                &crate::test_support::approximate_decisions(),
                &same,
                &plane
            )
            .unwrap()
        );
        assert!(
            !certifiably_same_oriented_plane(
                &crate::test_support::approximate_decisions(),
                &opposite,
                &plane
            )
            .unwrap()
        );
        assert!(certifiably_same_unoriented_plane(
            &crate::test_support::approximate_decisions(),
            &opposite,
            &plane
        ));
        assert!(!certifiably_same_unoriented_plane(
            &crate::test_support::approximate_decisions(),
            &distinct,
            &plane
        ));

        let symbolic =
            Plane::from_coefficients(Real::pi(), Real::one(), Real::zero(), Real::from(2));
        assert!(
            certifiably_same_oriented_plane(
                &crate::test_support::approximate_decisions(),
                &symbolic,
                &symbolic
            )
            .unwrap()
        );
    }

    #[test]
    fn projective_cycle_expands_deferred_source_edges_on_demand() {
        let mut polygon = crate::polygon::make_triangle_with_deferred_edges(
            &crate::test_support::approximate_decisions(),
            &p(1, 0, 0),
            &p(0, 1, 0),
            &p(0, 0, 0),
            0,
            0,
        )
        .unwrap();
        polygon.set_source_triangle_edge_identities(0, [0, 1, 2]);
        assert!(polygon.edges.is_empty());
        assert_eq!(polygon.vertex_count(), 3);

        let mut point_cache = ProjectivePointCache::default();
        let cycle = ProjectiveCycle::from_polygon(
            &polygon,
            ConstructionPlaneIdentity { mesh: 0, plane: 0 },
            &mut point_cache,
        )
        .unwrap();
        assert_eq!(cycle.boundary.len(), 3);
        assert!(
            cycle
                .boundary
                .iter()
                .all(|entry| point_cache.plane(entry.edge_index) == &polygon.support)
        );
    }

    #[test]
    fn consuming_projective_split_preserves_shared_boundary_identities() {
        let mut polygon = approximate_convex_triangle(&p(0, 0, -1), &p(2, 0, 1), &p(0, 2, 0), 0, 0);
        polygon.set_source_triangle_edge_identities(0, [0, 1, 2]);
        let mut point_cache = ProjectivePointCache::default();
        let cycle = ProjectiveCycle::from_polygon(
            &polygon,
            ConstructionPlaneIdentity { mesh: 0, plane: 0 },
            &mut point_cache,
        )
        .unwrap();
        let plane = Plane::axis_aligned(2, Real::zero());
        let negative_only = cycle
            .clone()
            .clip_negative(
                &crate::test_support::approximate_decisions(),
                &plane,
                ConstructionPlaneIdentity { mesh: 1, plane: 0 },
                &mut point_cache,
            )
            .unwrap();
        let split = cycle
            .clip(
                &crate::test_support::approximate_decisions(),
                &plane,
                ConstructionPlaneIdentity { mesh: 1, plane: 0 },
                &mut point_cache,
            )
            .unwrap();
        let affine = |cycle: &ProjectiveCycle| {
            cycle
                .boundary
                .iter()
                .map(|entry| {
                    point_cache
                        .point(entry.point_index)
                        .to_affine_point()
                        .unwrap()
                })
                .collect::<Vec<_>>()
        };

        assert!(matches!(split.side, ProjectiveClipSide::Both));
        assert_eq!(
            affine(&split.negative),
            vec![p(0, 0, -1), p(1, 0, 0), p(0, 2, 0)]
        );
        assert_eq!(
            affine(&split.positive),
            vec![p(1, 0, 0), p(2, 0, 1), p(0, 2, 0)]
        );
        assert_eq!(
            split.negative.boundary[1].point_identity,
            split.positive.boundary[0].point_identity
        );
        assert!(
            split.negative.boundary[1]
                .evidence
                .rational_filter_query
                .is_some()
        );
        assert!(
            split.positive.boundary[0]
                .evidence
                .rational_filter_query
                .is_some()
        );
        assert_eq!(
            split.negative.boundary[2].point_identity,
            split.positive.boundary[2].point_identity
        );
        assert_eq!(
            split.negative.boundary[0].edge_identity,
            split.positive.boundary[0].edge_identity
        );
        assert_eq!(
            split.negative.boundary[1].edge_identity,
            split.positive.boundary[2].edge_identity
        );
        assert_eq!(affine(&negative_only), affine(&split.negative));
        assert!(
            negative_only
                .boundary
                .iter()
                .zip(&split.negative.boundary)
                .all(|(negative, split)| {
                    negative.point_identity == split.point_identity
                        && negative.edge_index == split.edge_index
                        && negative.edge_identity == split.edge_identity
                })
        );
    }

    #[test]
    fn rank_deficient_boundary_planes_do_not_prove_unrecorded_incidence() {
        let mut polygon = approximate_convex_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        polygon.set_source_triangle_edge_identities(0, [0, 1, 2]);
        let source_identity = ConstructionPlaneIdentity { mesh: 0, plane: 0 };
        let target_identity = ConstructionPlaneIdentity { mesh: 1, plane: 0 };
        let mut point_cache = ProjectivePointCache::default();
        let mut cycle =
            ProjectiveCycle::from_polygon(&polygon, source_identity, &mut point_cache).unwrap();

        // A repeated defining plane makes the old four-plane determinant
        // vanish identically, even when the represented point is not on the
        // queried plane. Incidence must come from construction identity or an
        // exact point-plane classification instead.
        cycle.boundary[2].edge_index = cycle.boundary[0].edge_index;
        let target = Plane::axis_aligned(
            2,
            Real::from(Rational::fraction(1, 1_000_000_000_000).unwrap()),
        );
        let point = point_cache.point(cycle.boundary[0].point_index);
        assert_ne!(
            crate::predicate::classify_real(
                &crate::test_support::approximate_decisions(),
                &homogeneous_point_plane_expression(point, &target)
            )
            .unwrap(),
            Classification::On
        );
        assert!(!cycle.point_has_plane_incidence(0, target_identity, &point_cache,));
    }

    #[test]
    fn rank_deficient_plane_triple_does_not_identify_distinct_points() {
        let plane_ids = [
            ConstructionPlaneIdentity { mesh: 0, plane: 0 },
            ConstructionPlaneIdentity { mesh: 0, plane: 1 },
            ConstructionPlaneIdentity { mesh: 0, plane: 2 },
        ];
        let planes = [
            Plane::axis_aligned(0, Real::zero()),
            Plane::axis_aligned(0, Real::zero()).inverted(),
            Plane::axis_aligned(1, Real::zero()),
        ];
        let mut point_cache = ProjectivePointCache::default();
        for (identity, plane) in plane_ids.into_iter().zip(&planes) {
            point_cache.support_plane_index(identity, plane);
        }
        let defined_identity = ConstructionVertexIdentity::PlaneTriple { planes: plane_ids };
        let source_identity = ConstructionVertexIdentity::Source { mesh: 1, vertex: 0 };
        let left = HomogeneousPoint3::new(Real::zero(), Real::zero(), Real::zero(), Real::one());
        let right = HomogeneousPoint3::new(Real::zero(), Real::zero(), Real::one(), Real::one());

        assert!(!point_cache.identities_certifiably_equal(
            &crate::test_support::approximate_decisions(),
            &defined_identity,
            &left,
            &source_identity,
            &right,
        ));
    }

    #[test]
    fn singleton_certified_face_preserves_deferred_edges() {
        let mut polygon = crate::polygon::make_triangle_with_deferred_edges(
            &crate::test_support::approximate_decisions(),
            &p(1, 0, 0),
            &p(0, 1, 0),
            &p(0, 0, 0),
            0,
            0,
        )
        .unwrap();
        polygon.set_source_triangle_edge_identities(0, [0, 1, 2]);
        let support_identity = ConstructionPlaneIdentity { mesh: 0, plane: 0 };
        let polygons = [polygon];
        let supports = [vec![&polygons[0].support], Vec::new()];

        let (faces, face_supports) = collapse_certified_convex_faces(
            &crate::test_support::approximate_decisions(),
            &polygons,
            &[support_identity],
            &supports,
        )
        .unwrap();

        assert_eq!(faces.len(), 1);
        assert!(faces[0].edges.is_empty());
        assert_eq!(faces[0].vertex_count(), 3);
        assert_eq!(faces[0].delta_w, vec![1, 0]);
        assert_eq!(face_supports, vec![support_identity]);
    }

    #[test]
    fn certified_face_grouping_rejects_unaligned_or_out_of_range_supports() {
        let mut polygon = crate::polygon::make_triangle_with_deferred_edges(
            &crate::test_support::approximate_decisions(),
            &p(1, 0, 0),
            &p(0, 1, 0),
            &p(0, 0, 0),
            0,
            0,
        )
        .unwrap();
        polygon.set_source_triangle_edge_identities(0, [0, 1, 2]);
        let polygons = [polygon];
        let supports = [vec![&polygons[0].support], Vec::new()];

        assert!(
            collapse_certified_convex_faces(
                &crate::test_support::approximate_decisions(),
                &polygons,
                &[],
                &supports,
            )
            .is_err()
        );
        assert!(
            collapse_certified_convex_faces(
                &crate::test_support::approximate_decisions(),
                &polygons,
                &[ConstructionPlaneIdentity { mesh: 0, plane: 1 }],
                &supports,
            )
            .is_err()
        );
        assert!(
            collapse_certified_convex_faces(
                &crate::test_support::approximate_decisions(),
                &polygons,
                &[ConstructionPlaneIdentity { mesh: 2, plane: 0 }],
                &supports,
            )
            .is_err()
        );
    }

    fn compact_projective_square_input() -> [crate::mesh::ProjectiveInputMesh; 2] {
        let positions: std::sync::Arc<[Point3]> =
            std::sync::Arc::from([p(0, 0, 0), p(1, 0, 0), p(1, 1, 0), p(0, 1, 0)]);
        let support = Plane::from_points(&positions[0], &positions[1], &positions[2]);
        [
            crate::mesh::ProjectiveInputMesh {
                positions,
                support_planes: vec![support],
                triangles: vec![
                    crate::mesh::ProjectiveInputTriangle {
                        indices: [0, 1, 2],
                        support_plane: 0,
                    },
                    crate::mesh::ProjectiveInputTriangle {
                        indices: [0, 2, 3],
                        support_plane: 0,
                    },
                ],
                polygon_offset: 0,
            },
            crate::mesh::ProjectiveInputMesh {
                positions: std::sync::Arc::from([]),
                support_planes: Vec::new(),
                triangles: Vec::new(),
                polygon_offset: 2,
            },
        ]
    }

    #[test]
    fn compact_projective_face_collapse_traces_undirected_source_edges() {
        let input = compact_projective_square_input();
        let support_planes = [vec![&input[0].support_planes[0]], Vec::new()];
        let indexed_support_planes = [vec![0], Vec::new()];

        let (faces, face_supports) = collapse_projective_input_faces(
            &crate::test_support::approximate_decisions(),
            [&input[0], &input[1]],
            None,
            &support_planes,
            &indexed_support_planes,
        )
        .unwrap();

        assert_eq!(faces.len(), 1);
        assert_eq!(
            face_supports,
            vec![ConstructionPlaneIdentity { mesh: 0, plane: 0 }]
        );
        assert!(faces[0].edges.is_empty());
        assert_eq!(
            faces[0]
                .known_vertex_identities()
                .unwrap()
                .iter()
                .collect::<Vec<_>>(),
            (0..4)
                .map(|vertex| ConstructionVertexIdentity::Source { mesh: 0, vertex })
                .collect::<Vec<_>>()
        );
        assert_eq!(
            faces[0]
                .known_edge_identities()
                .unwrap()
                .iter()
                .collect::<Vec<_>>(),
            [[0, 1], [1, 2], [2, 3], [0, 3]]
                .map(|endpoints| { ConstructionEdgeIdentity::Source { mesh: 0, endpoints } })
                .to_vec()
        );
    }

    #[test]
    fn compact_projective_face_collapse_preserves_supplied_boundary_planes() {
        let input = compact_projective_square_input();
        let decisions = crate::test_support::approximate_decisions();
        let triangle_planes = input[0]
            .triangles
            .iter()
            .map(|triangle| {
                let [a, b, c] = triangle.indices;
                InputTrianglePlanes::from_points_decision(
                    &decisions,
                    &input[0].positions[a],
                    &input[0].positions[b],
                    &input[0].positions[c],
                )
                .unwrap()
            })
            .collect::<Vec<_>>();
        let empty_planes = [];
        let input_planes = [&triangle_planes[..], &empty_planes[..]];
        let support_planes = [vec![&input[0].support_planes[0]], Vec::new()];
        let indexed_support_planes = [vec![0], Vec::new()];

        let (faces, _) = collapse_projective_input_faces(
            &decisions,
            [&input[0], &input[1]],
            Some(&input_planes),
            &support_planes,
            &indexed_support_planes,
        )
        .unwrap();

        assert_eq!(faces.len(), 1);
        assert_eq!(faces[0].edges.len(), 4);
        let vertices = faces[0]
            .known_vertices
            .as_ref()
            .unwrap()
            .iter()
            .collect::<Vec<_>>();
        for (edge, plane) in faces[0].edges.iter().enumerate() {
            assert_eq!(
                classify_point_decision(&decisions, vertices[edge], plane).unwrap(),
                Classification::On
            );
            assert_eq!(
                classify_point_decision(&decisions, vertices[(edge + 1) % vertices.len()], plane,)
                    .unwrap(),
                Classification::On
            );
        }
    }

    #[test]
    fn merged_certified_face_cycle_is_independent_of_triangle_order() {
        let positions: std::sync::Arc<[Point3]> =
            std::sync::Arc::from([p(0, 0, 0), p(1, 0, 0), p(1, 1, 0), p(0, 1, 0)]);
        let mut first = crate::polygon::make_indexed_triangle_with_deferred_edges(
            positions.clone(),
            [0, 1, 2],
            None,
            std::sync::Arc::new(Vec::new()),
            0,
            0,
        );
        first.set_source_triangle_edge_identities(0, [0, 1, 2]);
        let mut second = crate::polygon::make_indexed_triangle_with_deferred_edges(
            positions,
            [0, 2, 3],
            Some(first.support.clone()),
            std::sync::Arc::new(Vec::new()),
            0,
            1,
        );
        second.set_source_triangle_edge_identities(0, [0, 2, 3]);
        let support_identity = ConstructionPlaneIdentity { mesh: 0, plane: 0 };

        let collapse = |polygons: &[ConvexPolygon]| {
            let supports = [vec![&polygons[0].support], Vec::new()];
            collapse_certified_convex_faces(
                &crate::test_support::approximate_decisions(),
                polygons,
                &[support_identity, support_identity],
                &supports,
            )
            .unwrap()
            .0
            .pop()
            .unwrap()
        };
        let forward = collapse(&[first.clone(), second.clone()]);
        let reverse = collapse(&[second, first]);

        let expected_vertices =
            [0, 1, 2, 3].map(|vertex| ConstructionVertexIdentity::Source { mesh: 0, vertex });
        assert_eq!(
            forward
                .known_vertex_identities()
                .unwrap()
                .iter()
                .collect::<Vec<_>>(),
            expected_vertices
        );
        assert_eq!(
            reverse
                .known_vertex_identities()
                .unwrap()
                .iter()
                .collect::<Vec<_>>(),
            expected_vertices
        );
        assert!(forward.approx_bounds.is_none());
        assert!(reverse.approx_bounds.is_none());
        assert_eq!(
            forward.known_edge_identities(),
            reverse.known_edge_identities()
        );
    }

    #[test]
    fn certified_collinear_face_vertices_collapse_to_composite_source_edges() {
        let support = Plane::axis_aligned(2, Real::zero());
        let mut vertices = vec![
            p(0, 0, 0),
            p(1, 0, 0),
            p(2, 0, 0),
            p(2, 1, 0),
            p(2, 2, 0),
            p(1, 2, 0),
            p(0, 2, 0),
            p(0, 1, 0),
        ];
        let mut vertex_identities = (0..vertices.len())
            .map(|vertex| ConstructionVertexIdentity::Source { mesh: 0, vertex })
            .collect::<Vec<_>>();
        let mut edges = (0..vertices.len())
            .map(|index| {
                edge_plane(
                    &crate::test_support::approximate_decisions(),
                    &vertices[(index + 1) % vertices.len()],
                    &vertices[(index + 2) % vertices.len()],
                    &vertices[index],
                    &support,
                )
            })
            .collect::<HypermeshResult<Vec<_>>>()
            .unwrap();
        let mut edge_identities = (0..vertices.len())
            .map(|start| ConstructionEdgeIdentity::Source {
                mesh: 0,
                endpoints: [start, (start + 1) % vertices.len()],
            })
            .collect::<Vec<_>>();

        collapse_certified_collinear_face_vertices(
            &crate::test_support::approximate_decisions(),
            0,
            &support,
            &mut vertices,
            &mut vertex_identities,
            &mut edges,
            &mut edge_identities,
            None,
        )
        .unwrap();

        assert_eq!(
            vertices,
            vec![p(0, 0, 0), p(2, 0, 0), p(2, 2, 0), p(0, 2, 0)]
        );
        assert_eq!(
            vertex_identities,
            [0, 2, 4, 6].map(|vertex| ConstructionVertexIdentity::Source { mesh: 0, vertex })
        );
        assert_eq!(edges.len(), 4);
        assert_eq!(
            edge_identities,
            [[0, 2], [2, 4], [4, 6], [0, 6]]
                .map(|endpoints| { ConstructionEdgeIdentity::Source { mesh: 0, endpoints } })
        );
    }

    #[test]
    fn certified_collinear_face_compaction_falls_back_beyond_stack_capacity() {
        let support = Plane::axis_aligned(2, Real::zero());
        let expected_vertices = (0..40)
            .map(|index| {
                let x = 2 * index;
                p(x, x * x, 0)
            })
            .collect::<Vec<_>>();
        let mut vertices = expected_vertices.clone();
        vertices.insert(1, p(1, 2, 0));
        let mut vertex_identities = (0..vertices.len())
            .map(|vertex| ConstructionVertexIdentity::Source { mesh: 0, vertex })
            .collect::<Vec<_>>();
        let mut edges = Vec::new();
        let mut edge_identities = (0..vertices.len())
            .map(|start| ConstructionEdgeIdentity::Source {
                mesh: 0,
                endpoints: [start, (start + 1) % vertices.len()],
            })
            .collect::<Vec<_>>();

        collapse_certified_collinear_face_vertices(
            &crate::test_support::approximate_decisions(),
            0,
            &support,
            &mut vertices,
            &mut vertex_identities,
            &mut edges,
            &mut edge_identities,
            None,
        )
        .unwrap();

        assert_eq!(vertices, expected_vertices);
        assert_eq!(vertex_identities.len(), 40);
        assert_eq!(
            vertex_identities[0],
            ConstructionVertexIdentity::Source { mesh: 0, vertex: 0 }
        );
        assert_eq!(
            vertex_identities[1],
            ConstructionVertexIdentity::Source { mesh: 0, vertex: 2 }
        );
        assert_eq!(edge_identities.len(), 40);
    }

    #[test]
    fn certified_nondegenerate_face_preserves_supplied_boundaries() {
        let support = Plane::axis_aligned(2, Real::zero());
        let mut vertices = vec![p(0, 0, 0), p(2, 0, 0), p(2, 2, 0), p(0, 2, 0)];
        let mut vertex_identities = (0..vertices.len())
            .map(|vertex| ConstructionVertexIdentity::Source { mesh: 0, vertex })
            .collect::<Vec<_>>();
        let mut edges = (0..vertices.len())
            .map(|index| Plane::axis_aligned(index % 3, Real::from(index as i64 + 7)))
            .collect::<Vec<_>>();
        let mut edge_identities = (0..vertices.len())
            .map(|start| ConstructionEdgeIdentity::Source {
                mesh: 0,
                endpoints: [start, (start + 1) % vertices.len()],
            })
            .collect::<Vec<_>>();
        let expected_vertices = vertices.clone();
        let expected_vertex_identities = vertex_identities.clone();
        let expected_edges = edges.clone();
        let expected_edge_identities = edge_identities.clone();

        collapse_certified_collinear_face_vertices(
            &crate::test_support::approximate_decisions(),
            0,
            &support,
            &mut vertices,
            &mut vertex_identities,
            &mut edges,
            &mut edge_identities,
            None,
        )
        .unwrap();

        assert_eq!(vertices, expected_vertices);
        assert_eq!(vertex_identities, expected_vertex_identities);
        assert_eq!(edges, expected_edges);
        assert_eq!(edge_identities, expected_edge_identities);
    }

    #[test]
    fn certified_face_preserves_deferred_boundaries_after_validation() {
        let support = Plane::axis_aligned(2, Real::zero());
        let mut vertices = vec![p(0, 0, 0), p(2, 0, 0), p(2, 2, 0), p(0, 2, 0)];
        let mut vertex_identities = (0..vertices.len())
            .map(|vertex| ConstructionVertexIdentity::Source { mesh: 0, vertex })
            .collect::<Vec<_>>();
        let mut edges = Vec::new();
        let mut edge_identities = [[0, 1], [1, 2], [2, 3], [0, 3]]
            .map(|endpoints| ConstructionEdgeIdentity::Source { mesh: 0, endpoints })
            .to_vec();
        let expected_edge_identities = edge_identities.clone();

        collapse_certified_collinear_face_vertices(
            &crate::test_support::approximate_decisions(),
            0,
            &support,
            &mut vertices,
            &mut vertex_identities,
            &mut edges,
            &mut edge_identities,
            None,
        )
        .unwrap();

        assert!(edges.is_empty());
        assert_eq!(edge_identities, expected_edge_identities);
    }

    #[test]
    fn exact_support_projection_certifies_collinear_face_vertices() {
        let support = Plane::from_points(&p(0, 0, 0), &p(1, 0, 1), &p(0, 1, 1));

        assert!(
            support
                .points_are_collinear_on_support(
                    &crate::test_support::approximate_decisions(),
                    &p(1, 1, 2),
                    &p(2, 2, 4),
                    &p(0, 0, 0)
                )
                .unwrap()
        );
        assert!(
            !support
                .points_are_collinear_on_support(
                    &crate::test_support::approximate_decisions(),
                    &p(1, 0, 1),
                    &p(0, 1, 1),
                    &p(0, 0, 0)
                )
                .unwrap()
        );
    }

    #[test]
    fn projective_cycle_verification_reuses_exact_plane_incidences() {
        let mut polygon = crate::polygon::make_triangle_with_deferred_edges(
            &crate::test_support::approximate_decisions(),
            &p(1, 0, 0),
            &p(0, 1, 0),
            &p(0, 0, 0),
            0,
            0,
        )
        .unwrap();
        polygon.set_source_triangle_edge_identities(0, [0, 1, 2]);
        let mut point_cache = ProjectivePointCache::default();
        let cycle = ProjectiveCycle::from_polygon(
            &polygon,
            ConstructionPlaneIdentity { mesh: 0, plane: 0 },
            &mut point_cache,
        )
        .unwrap();
        for entry in &cycle.boundary {
            point_cache.record_incidence(
                &entry.point_identity,
                ConstructionPlaneIdentity { mesh: 0, plane: 0 },
            );
        }

        let mut rational_plane_evidence = Vec::new();
        assert!(
            cycle_satisfies_planes(
                &crate::test_support::approximate_decisions(),
                &cycle,
                &[&polygon.support],
                &[0],
                &[],
                0,
                &mut rational_plane_evidence,
                &point_cache,
            )
            .unwrap()
        );
    }

    #[test]
    fn source_relation_stops_after_exact_crossing_is_certified() {
        let polygon = approximate_convex_triangle(&p(0, 0, 1), &p(0, 0, -1), &p(1, 0, 0), 0, 0);
        let plane = Plane::axis_aligned(2, Real::zero());
        let mut cache = PointPlaneClassificationCache::default();
        let mut on_source_vertices = Vec::new();

        assert!(matches!(
            cache
                .source_relation(
                    &crate::test_support::approximate_decisions(),
                    &polygon,
                    &plane,
                    0,
                    1,
                    &mut on_source_vertices,
                )
                .unwrap(),
            SourcePlaneRelation::Crossing
        ));
        assert_eq!(cache.points.len(), 2);
    }

    #[test]
    fn source_relation_indexes_certified_source_vertices_without_coordinate_hashing() {
        let mut polygon = approximate_convex_triangle(&p(0, 0, 1), &p(0, 0, -1), &p(1, 0, 0), 0, 0);
        polygon.set_source_triangle_edge_identities(0, [7, 9, 11]);
        let plane = Plane::axis_aligned(2, Real::zero());
        let mut cache = PointPlaneClassificationCache::default();
        let mut on_source_vertices = Vec::new();

        assert!(matches!(
            cache
                .source_relation(
                    &crate::test_support::approximate_decisions(),
                    &polygon,
                    &plane,
                    0,
                    1,
                    &mut on_source_vertices
                )
                .unwrap(),
            SourcePlaneRelation::Crossing
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

        let identities = canonical_plane_identities(
            &crate::test_support::approximate_decisions(),
            &support_planes,
        );
        assert_eq!(identities[0][0], identities[1][0]);
        assert_eq!(identities[0][0], identities[1][1]);
        assert_ne!(identities[0][0], identities[0][1]);
    }

    #[test]
    fn approximate_support_buckets_verify_every_exact_collision() {
        assert_eq!(
            std::mem::size_of::<ApproximateSupportPlaneBucket>(),
            2 * std::mem::size_of::<usize>()
        );
        assert_eq!(
            std::mem::size_of::<ApproximateSupportPlaneCollision>(),
            2 * std::mem::size_of::<usize>()
        );

        let rounded = |shift: u32| {
            let denominator = 1_u64 << shift;
            Plane::from_coefficients(
                Real::from(
                    Rational::fraction(i64::try_from(denominator + 1).unwrap(), denominator)
                        .unwrap(),
                ),
                Real::zero(),
                Real::zero(),
                Real::zero(),
            )
        };
        let first = Plane::from_coefficients(Real::one(), Real::zero(), Real::zero(), Real::zero());
        let second = rounded(54);
        let third = rounded(55);
        let decisions = crate::test_support::approximate_decisions();
        let key = exact_plane_f64(&decisions, &first)
            .unwrap()
            .map(f64::to_bits);
        assert_eq!(
            exact_plane_f64(&decisions, &second)
                .unwrap()
                .map(f64::to_bits),
            key
        );
        assert_eq!(
            exact_plane_f64(&decisions, &third)
                .unwrap()
                .map(f64::to_bits),
            key
        );
        assert_ne!(first, second);
        assert_ne!(first, third);
        assert_ne!(second, third);

        let mut supports = Vec::new();
        let mut index = ApproximateSupportPlaneIndex::default();
        assert_eq!(index.intern(key, &mut supports, &first), (0, true));
        assert_eq!(index.collisions.capacity(), 0);
        assert_eq!(index.intern(key, &mut supports, &first), (0, false));

        assert_eq!(index.intern(key, &mut supports, &second), (1, true));
        assert_eq!(index.intern(key, &mut supports, &third), (2, true));
        assert_eq!(index.intern(key, &mut supports, &first), (0, false));
        assert_eq!(index.intern(key, &mut supports, &second), (1, false));
        assert_eq!(index.intern(key, &mut supports, &third), (2, false));
        assert_eq!(supports.len(), 3);
    }

    #[test]
    fn exact_plane_f64_reuses_filter_normalization_for_both_policies() {
        let plane =
            Plane::from_coefficients(Real::from(3), Real::from(-4), Real::from(12), Real::from(7));
        for policy in [
            crate::PredicatePolicy::STRICT,
            crate::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let values = exact_plane_f64(&DecisionContext::new(&context), &plane).unwrap();
            assert_eq!(values, [0.375, -0.5, 1.5, 0.875]);
            assert_eq!(normalize_plane_f64(values), plane_f64(&plane));
        }
    }

    #[test]
    fn exact_plane_f64_preserves_filter_unavailable_and_nonexact_paths() {
        let huge = Real::try_from(f64::MAX).unwrap();
        let tiny = Real::try_from(f64::MIN_POSITIVE).unwrap();
        let unsafe_span = Plane::from_coefficients(huge, tiny, Real::zero(), Real::zero());
        let decisions = crate::test_support::approximate_decisions();
        assert_eq!(
            exact_plane_f64(&decisions, &unsafe_span),
            Some([f64::MAX, f64::MIN_POSITIVE, 0.0, 0.0]),
        );

        let nonexact =
            Plane::from_coefficients(Real::pi(), Real::zero(), Real::zero(), Real::zero());
        assert_eq!(exact_plane_f64(&decisions, &nonexact), None);
    }

    #[test]
    fn plane_intersection_normalizes_negative_homogeneous_weight() {
        let planes = [
            Plane::axis_aligned(1, Real::from(2)),
            Plane::axis_aligned(0, Real::from(1)),
            Plane::axis_aligned(2, Real::from(3)),
        ];

        let point = positive_weight_plane_intersection(planes.each_ref()).unwrap();
        assert!(
            point
                .w
                .exact_rational_ref()
                .is_some_and(Rational::is_positive)
        );
        assert_eq!(point.to_affine_point().unwrap(), p(1, 2, 3));
    }

    #[test]
    fn exact_projective_affine_fingerprint_is_scale_invariant_and_conservative() {
        let point =
            HomogeneousPoint3::new(Real::from(1), Real::from(2), Real::from(3), Real::one());
        let scaled =
            HomogeneousPoint3::new(Real::from(7), Real::from(14), Real::from(21), Real::from(7));
        let distinct =
            HomogeneousPoint3::new(Real::from(1), Real::from(2), Real::from(4), Real::one());
        let modular_collision = HomogeneousPoint3::new(
            Real::from(PROJECTIVE_FINGERPRINT_MODULUS as i64 + 1),
            Real::from(2),
            Real::from(3),
            Real::one(),
        );
        let zero_weight =
            HomogeneousPoint3::new(Real::from(1), Real::from(2), Real::from(3), Real::zero());
        let modular_zero_weight = HomogeneousPoint3::new(
            Real::from(1),
            Real::from(2),
            Real::from(3),
            Real::from(PROJECTIVE_FINGERPRINT_MODULUS as i64),
        );
        let modular_denominator = HomogeneousPoint3::new(
            Real::new(
                Rational::fraction(1_i64, PROJECTIVE_FINGERPRINT_MODULUS)
                    .expect("the prime is a valid denominator"),
            ),
            Real::from(2),
            Real::from(3),
            Real::one(),
        );
        let symbolic =
            HomogeneousPoint3::new(Real::pi(), Real::from(2), Real::from(3), Real::one());

        assert_eq!(
            exact_projective_affine_fingerprint(&point),
            exact_projective_affine_fingerprint(&scaled)
        );
        assert_ne!(
            exact_projective_affine_fingerprint(&point),
            exact_projective_affine_fingerprint(&distinct)
        );
        assert_eq!(
            exact_projective_affine_fingerprint(&point),
            exact_projective_affine_fingerprint(&modular_collision)
        );
        assert!(exact_projective_affine_fingerprint(&zero_weight).is_none());
        assert!(exact_projective_affine_fingerprint(&modular_zero_weight).is_none());
        assert!(exact_projective_affine_fingerprint(&modular_denominator).is_none());
        assert!(exact_projective_affine_fingerprint(&symbolic).is_none());
    }

    #[test]
    fn polygon_vertex_fingerprint_is_unoriented_and_collisions_are_rechecked() {
        let polygon = |vertices: Vec<Point3>| {
            let mut polygon = ConvexPolygon::empty();
            polygon.known_vertices = Some(crate::polygon::RetainedVertexCycle::Owned(Arc::from(
                vertices,
            )));
            polygon
        };
        let original = polygon(vec![p(1, 2, 3), p(4, 5, 6), p(7, 8, 9)]);
        let reversed = polygon(vec![p(7, 8, 9), p(4, 5, 6), p(1, 2, 3)]);
        let modular_collision = polygon(vec![
            p(PROJECTIVE_FINGERPRINT_MODULUS as i64 + 1, 2, 3),
            p(4, 5, 6),
            p(7, 8, 9),
        ]);

        assert_eq!(
            exact_polygon_vertex_set_fingerprint(&original),
            exact_polygon_vertex_set_fingerprint(&reversed)
        );
        assert_eq!(
            exact_polygon_vertex_set_fingerprint(&original),
            exact_polygon_vertex_set_fingerprint(&modular_collision)
        );

        let decisions = crate::test_support::approximate_decisions();
        assert!(
            projective_output_has_coincident_polygons(
                &decisions,
                &[
                    ClassifiedPolygon::new(original.clone(), 1),
                    ClassifiedPolygon::new(reversed, 1),
                ],
                BooleanOp::Union,
            )
            .unwrap()
        );
        assert!(
            !projective_output_has_coincident_polygons(
                &decisions,
                &[
                    ClassifiedPolygon::new(original, 1),
                    ClassifiedPolygon::new(modular_collision, 1),
                ],
                BooleanOp::Union,
            )
            .unwrap()
        );
    }

    #[test]
    fn projective_fingerprint_collisions_still_take_policy_equality_path() {
        for policy in [
            crate::PredicatePolicy::STRICT,
            crate::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = crate::MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let identities = [
                ConstructionVertexIdentity::Source { mesh: 0, vertex: 0 },
                ConstructionVertexIdentity::Source { mesh: 0, vertex: 1 },
                ConstructionVertexIdentity::Source { mesh: 0, vertex: 2 },
            ];
            let points = [
                HomogeneousPoint3::new(Real::from(1), Real::from(2), Real::from(3), Real::one()),
                HomogeneousPoint3::new(
                    Real::from(PROJECTIVE_FINGERPRINT_MODULUS as i64 + 1),
                    Real::from(2),
                    Real::from(3),
                    Real::one(),
                ),
                HomogeneousPoint3::new(
                    Real::from(7),
                    Real::from(14),
                    Real::from(21),
                    Real::from(7),
                ),
            ];
            let mut cache = ProjectivePointCache::default();
            for (identity, point) in identities.iter().cloned().zip(points) {
                cache.intern_with_approximation(identity, point);
            }

            cache.resolve_vertex_coincidences(&decisions);

            assert_eq!(
                cache.canonical_vertex_identity(&identities[2]),
                identities[0]
            );
            assert_eq!(
                cache.canonical_vertex_identity(&identities[1]),
                identities[1]
            );
            assert_eq!(cache.canonical_identities.len(), 1);
            assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
        }
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
            ConstructionVertexIdentity::Source { mesh: 0, vertex: 0 },
            ConstructionVertexIdentity::PlaneTriple {
                planes: [plane_ids[0], plane_ids[1], plane_ids[2]],
            },
            ConstructionVertexIdentity::PlaneTriple {
                planes: [plane_ids[0], plane_ids[1], plane_ids[3]],
            },
        ];

        let resolve = |order: [usize; 3]| {
            let mut cache = ProjectivePointCache::default();
            for (identity, plane) in plane_ids.into_iter().zip(planes.iter().cloned()) {
                cache.support_plane_index(identity, &plane);
            }
            for index in order {
                let point = cache
                    .definition_planes(&identities[index])
                    .and_then(positive_weight_plane_intersection)
                    .unwrap_or_else(|| {
                        HomogeneousPoint3::new(
                            Real::from(1),
                            Real::from(2),
                            Real::from(3),
                            Real::one(),
                        )
                    });
                let (_, _, interned) =
                    cache.intern_with_approximation_by(identities[index].clone(), || point);
                assert_eq!(interned, identities[index]);
            }

            cache.resolve_vertex_coincidences(&crate::test_support::approximate_decisions());
            let canonical = identities
                .each_ref()
                .map(|identity| cache.canonical_vertex_identity(identity));
            let canonical_point_indices = identities
                .each_ref()
                .map(|identity| cache.points[identity].point_index);
            assert_eq!(cache.canonical_identities.len(), 2);
            assert!(
                canonical_point_indices
                    .iter()
                    .all(|&point_index| point_index == canonical_point_indices[0])
            );
            for identity in &identities {
                if *identity == canonical[0] {
                    assert!(!cache.canonical_identities.contains_key(identity));
                } else {
                    assert_eq!(
                        cache.canonical_identities.get(identity),
                        Some(&canonical[0])
                    );
                }
            }
            canonical
        };

        let forward = resolve([0, 1, 2]);
        let reverse = resolve([2, 1, 0]);
        let shuffled = resolve([1, 0, 2]);
        assert_eq!(forward, reverse);
        assert_eq!(forward, shuffled);
        assert!(forward.iter().all(|identity| *identity == forward[0]));
        assert_eq!(forward[0], identities.iter().min().unwrap().clone(),);
    }
}
