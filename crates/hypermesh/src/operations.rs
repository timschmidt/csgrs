//! Public boolean operation entry points.

use std::collections::{BTreeMap, BTreeSet};

use hyperlattice::{HomogeneousPoint3, Point3, Real};

use crate::error::HypermeshResult;
use crate::geometry::{Aabb, axis_mut, axis_ref, compare_real};
use crate::intersection::{PairwiseIntersectionType, intersect_polygons};
use crate::mesh::{InputMesh, MeshRef, PolygonSoup, Triangle, prepare_input_refs};
use crate::output::BooleanResult;
use crate::segment_trace::trace_segment;
use crate::subdivision::{SubdivisionConfig, SubdivisionTask, subdivide};
use crate::winding::{BooleanOp, make_indicator};

/// Configuration for boolean operations.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EmberConfig {
    /// Polygon-count threshold for leaf processing.
    pub leaf_threshold: usize,
    /// Maximum recursive subdivision depth.
    pub max_depth: usize,
    /// Enable WNV reachability early-out.
    pub use_early_termination: bool,
    /// Enable proven exact shortcut paths before the general subdivision path.
    pub use_proven_shortcuts: bool,
    /// Assume every source mesh has no self-intersections.
    pub assume_nsi: bool,
    /// Assume every source mesh has no nested components.
    pub assume_nnc: bool,
}

impl Default for EmberConfig {
    fn default() -> Self {
        Self {
            leaf_threshold: crate::subdivision::DEFAULT_LEAF_THRESHOLD,
            max_depth: crate::subdivision::DEFAULT_MAX_DEPTH,
            use_early_termination: true,
            use_proven_shortcuts: true,
            assume_nsi: false,
            assume_nnc: false,
        }
    }
}

/// Performs a boolean operation on borrowed mesh views.
pub fn boolean_operation_refs(
    meshes: &[MeshRef<'_>],
    op: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    validate_mesh_refs(meshes)?;

    if config.use_proven_shortcuts {
        if let Some(result) = disjoint_bounds_boolean(meshes, op)? {
            return Ok(result);
        }
        if let Some(result) = same_surface_boolean(meshes, op)? {
            return Ok(result);
        }
        if let Some(result) = containment_boolean(meshes, op)? {
            return Ok(result);
        }
        if let Some(result) = boundary_only_contact_boolean(meshes, op)? {
            return Ok(result);
        }
        if let Some(result) = oriented_box_boolean(meshes, op)? {
            return Ok(result);
        }
    }

    let mut soup = prepare_input_refs(meshes)?;
    for polygon in &mut soup.polygons {
        if config.assume_nsi {
            polygon.no_self_intersections = true;
        }
        if config.assume_nnc {
            polygon.no_nested_components = true;
        }
    }

    let process_bounds = expanded_bounds(&soup.bounds);
    let ref_point = outside_reference_point(&process_bounds);
    let ref_wnv = vec![0; soup.num_meshes];
    let indicator = make_indicator(op, soup.num_meshes);
    let classified = subdivide(
        SubdivisionTask::new(soup.polygons.clone(), process_bounds, ref_point, ref_wnv),
        &indicator,
        SubdivisionConfig {
            leaf_threshold: config.leaf_threshold,
            max_depth: config.max_depth,
            use_early_termination: config.use_early_termination,
        },
    )?;

    Ok(BooleanResult::from_classified_with_operation(
        soup,
        classified,
        Some(op),
    ))
}

fn validate_mesh_refs(meshes: &[MeshRef<'_>]) -> HypermeshResult<()> {
    if meshes.iter().all(|mesh| mesh.positions.is_empty()) {
        return Err(crate::error::HypermeshError::EmptyInput);
    }

    for mesh in meshes {
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

fn same_surface_boolean(
    meshes: &[MeshRef<'_>],
    op: BooleanOp,
) -> HypermeshResult<Option<BooleanResult>> {
    if meshes.len() != 2 || !same_surface(meshes[0], meshes[1]) {
        return Ok(None);
    }

    match op {
        BooleanOp::Union | BooleanOp::Intersection => result_from_mesh_refs(&[meshes[0]]).map(Some),
        BooleanOp::Difference | BooleanOp::SymmetricDifference => {
            let bounds = meshes
                .iter()
                .map(mesh_ref_bounds)
                .collect::<HypermeshResult<Vec<_>>>()?;
            Ok(Some(empty_result(meshes, &bounds)))
        }
    }
}

fn containment_boolean(
    meshes: &[MeshRef<'_>],
    op: BooleanOp,
) -> HypermeshResult<Option<BooleanResult>> {
    if meshes.len() != 2 {
        return Ok(None);
    }

    let left_contains_right = mesh_contains_all_vertices(meshes[0], meshes[1])?;
    let right_contains_left = mesh_contains_all_vertices(meshes[1], meshes[0])?;

    match (op, left_contains_right, right_contains_left) {
        (BooleanOp::Union, true, false) => result_from_mesh_refs(&[meshes[0]]).map(Some),
        (BooleanOp::Union, false, true) => result_from_mesh_refs(&[meshes[1]]).map(Some),
        (BooleanOp::Intersection, true, false) => result_from_mesh_refs(&[meshes[1]]).map(Some),
        (BooleanOp::Intersection, false, true) => result_from_mesh_refs(&[meshes[0]]).map(Some),
        (BooleanOp::Difference, true, false) => {
            result_from_owned_mesh(&combine_mesh_ref_with_inverted_hole(meshes[0], meshes[1]))
                .map(Some)
        }
        (BooleanOp::Difference, false, true) => {
            let bounds = meshes
                .iter()
                .map(mesh_ref_bounds)
                .collect::<HypermeshResult<Vec<_>>>()?;
            Ok(Some(empty_result(meshes, &bounds)))
        }
        _ => Ok(None),
    }
}

fn boundary_only_contact_boolean(
    meshes: &[MeshRef<'_>],
    op: BooleanOp,
) -> HypermeshResult<Option<BooleanResult>> {
    if meshes.len() != 2 || !matches!(op, BooleanOp::Intersection | BooleanOp::Difference) {
        return Ok(None);
    }

    let left_has_strict_sample_inside_right =
        mesh_has_strict_surface_sample_inside(meshes[1], meshes[0])?;
    let right_has_strict_sample_inside_left =
        mesh_has_strict_surface_sample_inside(meshes[0], meshes[1])?;
    let (Some(left_has_strict_sample_inside_right), Some(right_has_strict_sample_inside_left)) = (
        left_has_strict_sample_inside_right,
        right_has_strict_sample_inside_left,
    ) else {
        return Ok(None);
    };
    if left_has_strict_sample_inside_right || right_has_strict_sample_inside_left {
        return Ok(None);
    }

    let left_soup = prepare_input_refs(&[meshes[0]])?;
    let right_soup = prepare_input_refs(&[meshes[1]])?;
    if soups_have_transverse_surface_crossing(&left_soup, &right_soup)? {
        return Ok(None);
    }

    match op {
        BooleanOp::Intersection => {
            let bounds = meshes
                .iter()
                .map(mesh_ref_bounds)
                .collect::<HypermeshResult<Vec<_>>>()?;
            Ok(Some(empty_result(meshes, &bounds)))
        }
        BooleanOp::Difference => result_from_mesh_refs(&[meshes[0]]).map(Some),
        _ => Ok(None),
    }
}

fn result_from_mesh_refs(meshes: &[MeshRef<'_>]) -> HypermeshResult<BooleanResult> {
    let soup = prepare_input_refs(meshes)?;
    let classifications = vec![1; soup.polygons.len()];
    Ok(BooleanResult::new(soup, classifications))
}

fn result_from_owned_mesh(mesh: &InputMesh) -> HypermeshResult<BooleanResult> {
    result_from_mesh_refs(&[mesh.as_ref()])
}

fn disjoint_bounds_boolean(
    meshes: &[MeshRef<'_>],
    op: BooleanOp,
) -> HypermeshResult<Option<BooleanResult>> {
    if meshes.is_empty() {
        return Ok(None);
    }

    let bounds = meshes
        .iter()
        .map(mesh_ref_bounds)
        .collect::<HypermeshResult<Vec<_>>>()?;
    let all_pairwise_disjoint = (0..bounds.len()).all(|left| {
        ((left + 1)..bounds.len())
            .all(|right| bounds_are_disjoint(&bounds[left], &bounds[right]).unwrap_or(false))
    });
    match op {
        BooleanOp::Union | BooleanOp::SymmetricDifference if all_pairwise_disjoint => {
            let soup = prepare_input_refs(meshes)?;
            let classifications = vec![1; soup.polygons.len()];
            Ok(Some(BooleanResult::new(soup, classifications)))
        }
        BooleanOp::Intersection if bounds.len() > 1 && any_pair_interior_disjoint(&bounds)? => {
            Ok(Some(empty_result(meshes, &bounds)))
        }
        BooleanOp::Difference
            if bounds.iter().skip(1).all(|right| {
                bounds_have_disjoint_interiors(&bounds[0], right).unwrap_or(false)
            }) =>
        {
            let soup = prepare_input_refs(&[meshes[0]])?;
            let classifications = vec![1; soup.polygons.len()];
            Ok(Some(BooleanResult::new(soup, classifications)))
        }
        _ => Ok(None),
    }
}

fn mesh_ref_bounds(mesh: &MeshRef<'_>) -> HypermeshResult<Aabb> {
    let first = mesh
        .positions
        .first()
        .ok_or(crate::error::HypermeshError::EmptyInput)?;
    let mut min = first.clone();
    let mut max = min.clone();
    for point in &mesh.positions[1..] {
        for axis in 0..3 {
            if compare_real(axis_ref(point, axis), axis_ref(&min, axis))?.is_lt() {
                *axis_mut(&mut min, axis) = axis_ref(point, axis).clone();
            }
            if compare_real(axis_ref(point, axis), axis_ref(&max, axis))?.is_gt() {
                *axis_mut(&mut max, axis) = axis_ref(point, axis).clone();
            }
        }
    }
    Ok(Aabb::new(min, max))
}

fn bounds_are_disjoint(left: &Aabb, right: &Aabb) -> HypermeshResult<bool> {
    for axis in 0..3 {
        if compare_real(axis_ref(&left.max, axis), axis_ref(&right.min, axis))?.is_lt()
            || compare_real(axis_ref(&right.max, axis), axis_ref(&left.min, axis))?.is_lt()
        {
            return Ok(true);
        }
    }
    Ok(false)
}

fn bounds_have_disjoint_interiors(left: &Aabb, right: &Aabb) -> HypermeshResult<bool> {
    for axis in 0..3 {
        if !compare_real(axis_ref(&left.max, axis), axis_ref(&right.min, axis))?.is_gt()
            || !compare_real(axis_ref(&right.max, axis), axis_ref(&left.min, axis))?.is_gt()
        {
            return Ok(true);
        }
    }
    Ok(false)
}

fn any_pair_interior_disjoint(bounds: &[Aabb]) -> HypermeshResult<bool> {
    for left in 0..bounds.len() {
        for right in (left + 1)..bounds.len() {
            if bounds_have_disjoint_interiors(&bounds[left], &bounds[right])? {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn empty_result(meshes: &[MeshRef<'_>], bounds: &[Aabb]) -> BooleanResult {
    let mut min = bounds[0].min.clone();
    let mut max = bounds[0].max.clone();
    for bound in &bounds[1..] {
        for axis in 0..3 {
            if compare_real(axis_ref(&bound.min, axis), axis_ref(&min, axis))
                .expect("input bounds should compare exactly")
                .is_lt()
            {
                *axis_mut(&mut min, axis) = axis_ref(&bound.min, axis).clone();
            }
            if compare_real(axis_ref(&bound.max, axis), axis_ref(&max, axis))
                .expect("input bounds should compare exactly")
                .is_gt()
            {
                *axis_mut(&mut max, axis) = axis_ref(&bound.max, axis).clone();
            }
        }
    }
    BooleanResult::new(
        PolygonSoup {
            polygons: Vec::new(),
            bounds: Aabb::new(min, max),
            num_meshes: meshes.len(),
        },
        Vec::new(),
    )
}

fn same_surface(left: MeshRef<'_>, right: MeshRef<'_>) -> bool {
    if left.triangles.len() != right.triangles.len() {
        return false;
    }
    surface_keys(left) == surface_keys(right)
}

fn surface_keys(mesh: MeshRef<'_>) -> BTreeSet<[[String; 3]; 3]> {
    mesh.triangles
        .iter()
        .map(|triangle| {
            let mut points = triangle
                .indices()
                .map(|index| point_string_key(&mesh.positions[index]));
            points.sort();
            points
        })
        .collect()
}

fn point_string_key(point: &Point3) -> [String; 3] {
    [
        point.x.to_string(),
        point.y.to_string(),
        point.z.to_string(),
    ]
}

fn mesh_contains_all_vertices(
    container: MeshRef<'_>,
    candidate: MeshRef<'_>,
) -> HypermeshResult<bool> {
    if candidate.positions.is_empty() {
        return Ok(false);
    }

    let soup = prepare_input_refs(&[container])?;
    let candidate_soup = prepare_input_refs(&[candidate])?;
    if soups_have_surface_intersection(&soup, &candidate_soup)? {
        return Ok(false);
    }

    let process_bounds = expanded_bounds(&soup.bounds);
    let ref_point = outside_reference_point(&process_bounds);
    for point in candidate.positions {
        if point_lies_on_surface(point, &soup.polygons)? {
            return Ok(false);
        }
        let winding = trace_segment(&ref_point, point, &[0], &soup.polygons)?;
        if winding.first().copied().unwrap_or_default() == 0 {
            return Ok(false);
        }
    }
    Ok(true)
}

fn soups_have_surface_intersection(
    left: &PolygonSoup,
    right: &PolygonSoup,
) -> HypermeshResult<bool> {
    for left_polygon in &left.polygons {
        for (right_index, right_polygon) in right.polygons.iter().enumerate() {
            let intersection = intersect_polygons(left_polygon, right_polygon, right_index)?;
            if intersection.kind != PairwiseIntersectionType::None {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn soups_have_transverse_surface_crossing(
    left: &PolygonSoup,
    right: &PolygonSoup,
) -> HypermeshResult<bool> {
    for left_polygon in &left.polygons {
        for (right_index, right_polygon) in right.polygons.iter().enumerate() {
            let intersection = intersect_polygons(left_polygon, right_polygon, right_index)?;
            if intersection.kind != PairwiseIntersectionType::Segment {
                continue;
            }
            let Some(segment) = intersection.segment else {
                continue;
            };
            let mid = segment_midpoint(&segment.v0, &segment.v1)?;
            if left_polygon.contains_point_strictly(&mid)?
                && right_polygon.contains_point_strictly(&mid)?
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn segment_midpoint(left: &Point3, right: &Point3) -> HypermeshResult<HomogeneousPoint3> {
    let two = Real::from(2);
    Ok(HomogeneousPoint3::new(
        ((&left.x + &right.x) / two.clone())
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        ((&left.y + &right.y) / two.clone())
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        ((&left.z + &right.z) / two)
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        Real::one(),
    ))
}

fn mesh_has_strict_surface_sample_inside(
    container: MeshRef<'_>,
    candidate: MeshRef<'_>,
) -> HypermeshResult<Option<bool>> {
    let soup = prepare_input_refs(&[container])?;
    let process_bounds = expanded_bounds(&soup.bounds);
    let ref_points = outside_reference_points(&process_bounds);
    for point in candidate.positions {
        match point_has_nonzero_winding(point, &ref_points, &soup.polygons)? {
            Some(true) => return Ok(Some(true)),
            Some(false) => {}
            None => return Ok(None),
        }
    }
    let candidate_soup = prepare_input_refs(&[candidate])?;
    for polygon in &candidate_soup.polygons {
        let point = polygon_centroid(polygon)?;
        match point_has_nonzero_winding(&point, &ref_points, &soup.polygons)? {
            Some(true) => return Ok(Some(true)),
            Some(false) => {}
            None => return Ok(None),
        }
    }
    Ok(Some(false))
}

fn point_has_nonzero_winding(
    point: &Point3,
    ref_points: &[Point3],
    polygons: &[crate::polygon::ConvexPolygon],
) -> HypermeshResult<Option<bool>> {
    if point_lies_on_surface(point, polygons)? {
        return Ok(Some(false));
    }
    for ref_point in ref_points {
        if let Ok(winding) = trace_segment(ref_point, point, &[0], polygons) {
            return Ok(Some(winding.first().copied().unwrap_or_default() != 0));
        }
    }
    Ok(None)
}

fn polygon_centroid(polygon: &crate::polygon::ConvexPolygon) -> HypermeshResult<Point3> {
    let vertices = polygon.vertices()?;
    let mut sum = Point3::origin();
    for point in &vertices {
        sum.x += point.x.clone();
        sum.y += point.y.clone();
        sum.z += point.z.clone();
    }
    let denom = Real::from(vertices.len() as u64);
    Ok(Point3::new(
        (sum.x / denom.clone()).map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        (sum.y / denom.clone()).map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        (sum.z / denom).map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
    ))
}

fn point_lies_on_surface(
    point: &Point3,
    polygons: &[crate::polygon::ConvexPolygon],
) -> HypermeshResult<bool> {
    let homogeneous = hyperlattice::HomogeneousPoint3::new(
        point.x.clone(),
        point.y.clone(),
        point.z.clone(),
        Real::one(),
    );
    for polygon in polygons {
        if polygon.contains_point(&homogeneous)? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn combine_mesh_ref_with_inverted_hole(outer: MeshRef<'_>, hole: MeshRef<'_>) -> InputMesh {
    let mut positions = outer.positions.to_vec();
    let offset = positions.len();
    positions.extend(hole.positions.iter().cloned());

    let mut triangles = outer.triangles.to_vec();
    triangles.extend(hole.triangles.iter().map(|triangle| {
        Triangle::new(
            triangle.v0 + offset,
            triangle.v2 + offset,
            triangle.v1 + offset,
        )
    }));

    let mut mesh = InputMesh::new(positions, triangles);
    mesh.nsi = outer.nsi && hole.nsi;
    mesh.nnc = outer.nnc && hole.nnc;
    mesh
}

#[derive(Clone, Debug, PartialEq)]
struct OrientedBox {
    min: [Real; 3],
    max: [Real; 3],
}

fn oriented_box_boolean(
    meshes: &[MeshRef<'_>],
    op: BooleanOp,
) -> HypermeshResult<Option<BooleanResult>> {
    if meshes.is_empty() {
        return Ok(None);
    }

    let Some((origin, axes)) = detect_ordered_box_basis(&meshes[0])? else {
        return Ok(None);
    };
    let mut boxes = Vec::with_capacity(meshes.len());
    for mesh in meshes {
        let Some((box_origin, box_axes)) = detect_ordered_box_basis(mesh)? else {
            return Ok(None);
        };
        if box_axes != axes {
            return Ok(None);
        }
        let min = coordinates_in_basis(
            &sub_points_array(&point_to_array(&box_origin), &point_to_array(&origin)),
            &axes,
        )?;
        boxes.push(OrientedBox {
            max: [
                &min[0] + &Real::one(),
                &min[1] + &Real::one(),
                &min[2] + &Real::one(),
            ],
            min,
        });
    }

    let mut xs = sorted_unique_reals(
        boxes
            .iter()
            .flat_map(|bounds| [bounds.min[0].clone(), bounds.max[0].clone()]),
    )?;
    let mut ys = sorted_unique_reals(
        boxes
            .iter()
            .flat_map(|bounds| [bounds.min[1].clone(), bounds.max[1].clone()]),
    )?;
    let mut zs = sorted_unique_reals(
        boxes
            .iter()
            .flat_map(|bounds| [bounds.min[2].clone(), bounds.max[2].clone()]),
    )?;
    xs.retain_adjacent_unique();
    ys.retain_adjacent_unique();
    zs.retain_adjacent_unique();

    let indicator = make_indicator(op, boxes.len());
    let mut selected = BTreeSet::new();
    for ix in 0..xs.len().saturating_sub(1) {
        for iy in 0..ys.len().saturating_sub(1) {
            for iz in 0..zs.len().saturating_sub(1) {
                let mid = [
                    midpoint_real(&xs[ix], &xs[ix + 1]),
                    midpoint_real(&ys[iy], &ys[iy + 1]),
                    midpoint_real(&zs[iz], &zs[iz + 1]),
                ];
                let w = boxes
                    .iter()
                    .map(|bounds| {
                        if point_strictly_inside_oriented_box(&mid, bounds)? {
                            Ok(1)
                        } else {
                            Ok(0)
                        }
                    })
                    .collect::<HypermeshResult<Vec<_>>>()?;
                if indicator(&w) {
                    selected.insert((ix, iy, iz));
                }
            }
        }
    }

    let output_bounds = bounds_from_meshes(meshes)?;
    if selected.is_empty() {
        return Ok(Some(BooleanResult::new(
            PolygonSoup {
                polygons: Vec::new(),
                bounds: output_bounds,
                num_meshes: meshes.len(),
            },
            Vec::new(),
        )));
    }

    let mut builder = CellMeshBuilder::default();
    for &(ix, iy, iz) in &selected {
        let min = [xs[ix].clone(), ys[iy].clone(), zs[iz].clone()];
        let max = [xs[ix + 1].clone(), ys[iy + 1].clone(), zs[iz + 1].clone()];

        for (axis, positive) in [
            (0usize, false),
            (0, true),
            (1, false),
            (1, true),
            (2, false),
            (2, true),
        ] {
            let neighbor = neighbor_cell((ix, iy, iz), axis, positive);
            if neighbor.is_some_and(|cell| selected.contains(&cell)) {
                continue;
            }
            builder.add_oriented_cell_face(&min, &max, &origin, &axes, axis, positive);
        }
    }

    let mut mesh = InputMesh::new(builder.positions, builder.triangles);
    mesh.nsi = true;
    mesh.nnc = true;
    let soup = prepare_input_refs(&[mesh.as_ref()])?;
    let classifications = vec![1; soup.polygons.len()];
    Ok(Some(BooleanResult::new(soup, classifications)))
}

fn detect_ordered_box_basis(
    mesh: &MeshRef<'_>,
) -> HypermeshResult<Option<(Point3, [[Real; 3]; 3])>> {
    if mesh.positions.len() != 8 || mesh.triangles.len() != 12 {
        return Ok(None);
    }
    let origin = mesh.positions[0].clone();
    let axes = [
        sub_points_array(
            &point_to_array(&mesh.positions[1]),
            &point_to_array(&origin),
        ),
        sub_points_array(
            &point_to_array(&mesh.positions[3]),
            &point_to_array(&origin),
        ),
        sub_points_array(
            &point_to_array(&mesh.positions[4]),
            &point_to_array(&origin),
        ),
    ];
    let expected = [
        [0, 0, 0],
        [1, 0, 0],
        [1, 1, 0],
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 1],
        [1, 1, 1],
        [0, 1, 1],
    ];
    for (index, coeffs) in expected.iter().enumerate() {
        if mesh.positions[index] != point_from_basis(&origin, &axes, coeffs) {
            return Ok(None);
        }
    }
    Ok(Some((origin, axes)))
}

fn coordinates_in_basis(vector: &[Real; 3], axes: &[[Real; 3]; 3]) -> HypermeshResult<[Real; 3]> {
    let denom = dot_arrays(&axes[0], &cross_arrays(&axes[1], &axes[2]));
    if denom.definitely_zero() {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
    Ok([
        (dot_arrays(vector, &cross_arrays(&axes[1], &axes[2])) / denom.clone())
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        (dot_arrays(&axes[0], &cross_arrays(vector, &axes[2])) / denom.clone())
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        (dot_arrays(&axes[0], &cross_arrays(&axes[1], vector)) / denom)
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
    ])
}

fn point_strictly_inside_oriented_box(
    point: &[Real; 3],
    bounds: &OrientedBox,
) -> HypermeshResult<bool> {
    for (axis, value) in point.iter().enumerate() {
        if !compare_real(value, &bounds.min[axis])?.is_gt()
            || !compare_real(value, &bounds.max[axis])?.is_lt()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn bounds_from_meshes(meshes: &[MeshRef<'_>]) -> HypermeshResult<Aabb> {
    let bounds = meshes
        .iter()
        .map(mesh_ref_bounds)
        .collect::<HypermeshResult<Vec<_>>>()?;
    let mut min = bounds[0].min.clone();
    let mut max = bounds[0].max.clone();
    for bound in &bounds[1..] {
        for axis in 0..3 {
            if compare_real(axis_ref(&bound.min, axis), axis_ref(&min, axis))?.is_lt() {
                *axis_mut(&mut min, axis) = axis_ref(&bound.min, axis).clone();
            }
            if compare_real(axis_ref(&bound.max, axis), axis_ref(&max, axis))?.is_gt() {
                *axis_mut(&mut max, axis) = axis_ref(&bound.max, axis).clone();
            }
        }
    }
    Ok(Aabb::new(min, max))
}

fn sorted_unique_reals(values: impl IntoIterator<Item = Real>) -> HypermeshResult<Vec<Real>> {
    let mut sorted = Vec::new();
    for value in values {
        if sorted.iter().any(|existing| existing == &value) {
            continue;
        }
        let mut insert_at = sorted.len();
        for (index, existing) in sorted.iter().enumerate() {
            if compare_real(&value, existing)?.is_lt() {
                insert_at = index;
                break;
            }
        }
        sorted.insert(insert_at, value);
    }
    Ok(sorted)
}

trait RetainAdjacentUnique {
    fn retain_adjacent_unique(&mut self);
}

impl RetainAdjacentUnique for Vec<Real> {
    fn retain_adjacent_unique(&mut self) {
        let mut index = 1;
        while index < self.len() {
            if self[index] == self[index - 1] {
                self.remove(index);
            } else {
                index += 1;
            }
        }
    }
}

fn midpoint_real(left: &Real, right: &Real) -> Real {
    ((left + right) / Real::from(2)).expect("division by literal two is valid")
}

fn neighbor_cell(
    (ix, iy, iz): (usize, usize, usize),
    axis: usize,
    positive: bool,
) -> Option<(usize, usize, usize)> {
    match (axis, positive) {
        (0, true) => Some((ix + 1, iy, iz)),
        (0, false) => ix.checked_sub(1).map(|x| (x, iy, iz)),
        (1, true) => Some((ix, iy + 1, iz)),
        (1, false) => iy.checked_sub(1).map(|y| (ix, y, iz)),
        (2, true) => Some((ix, iy, iz + 1)),
        (2, false) => iz.checked_sub(1).map(|z| (ix, iy, z)),
        _ => None,
    }
}

#[derive(Default)]
struct CellMeshBuilder {
    positions: Vec<Point3>,
    triangles: Vec<Triangle>,
    vertex_map: BTreeMap<[String; 3], usize>,
}

impl CellMeshBuilder {
    fn add_oriented_cell_face(
        &mut self,
        min: &[Real; 3],
        max: &[Real; 3],
        origin: &Point3,
        axes: &[[Real; 3]; 3],
        axis: usize,
        positive: bool,
    ) {
        let corners = [
            self.vertex(point_to_array(&point_from_local(
                origin,
                axes,
                [min[0].clone(), min[1].clone(), min[2].clone()],
            ))),
            self.vertex(point_to_array(&point_from_local(
                origin,
                axes,
                [max[0].clone(), min[1].clone(), min[2].clone()],
            ))),
            self.vertex(point_to_array(&point_from_local(
                origin,
                axes,
                [max[0].clone(), max[1].clone(), min[2].clone()],
            ))),
            self.vertex(point_to_array(&point_from_local(
                origin,
                axes,
                [min[0].clone(), max[1].clone(), min[2].clone()],
            ))),
            self.vertex(point_to_array(&point_from_local(
                origin,
                axes,
                [min[0].clone(), min[1].clone(), max[2].clone()],
            ))),
            self.vertex(point_to_array(&point_from_local(
                origin,
                axes,
                [max[0].clone(), min[1].clone(), max[2].clone()],
            ))),
            self.vertex(point_to_array(&point_from_local(
                origin,
                axes,
                [max[0].clone(), max[1].clone(), max[2].clone()],
            ))),
            self.vertex(point_to_array(&point_from_local(
                origin,
                axes,
                [min[0].clone(), max[1].clone(), max[2].clone()],
            ))),
        ];
        let faces = match (axis, positive) {
            (0, false) => [[0, 4, 7], [0, 7, 3]],
            (0, true) => [[1, 2, 6], [1, 6, 5]],
            (1, false) => [[0, 1, 5], [0, 5, 4]],
            (1, true) => [[3, 7, 6], [3, 6, 2]],
            (2, false) => [[0, 3, 2], [0, 2, 1]],
            (2, true) => [[4, 5, 6], [4, 6, 7]],
            _ => unreachable!("axis must be in 0..3"),
        };
        for [a, b, c] in faces {
            self.triangles
                .push(Triangle::new(corners[a], corners[b], corners[c]));
        }
    }

    fn vertex(&mut self, coords: [Real; 3]) -> usize {
        let key = [
            coords[0].to_string(),
            coords[1].to_string(),
            coords[2].to_string(),
        ];
        if let Some(index) = self.vertex_map.get(&key) {
            return *index;
        }
        let index = self.positions.len();
        self.positions.push(Point3::new(
            coords[0].clone(),
            coords[1].clone(),
            coords[2].clone(),
        ));
        self.vertex_map.insert(key, index);
        index
    }
}

fn point_from_basis(origin: &Point3, axes: &[[Real; 3]; 3], coeffs: &[i32; 3]) -> Point3 {
    point_from_local(
        origin,
        axes,
        [
            Real::from(coeffs[0]),
            Real::from(coeffs[1]),
            Real::from(coeffs[2]),
        ],
    )
}

fn point_from_local(origin: &Point3, axes: &[[Real; 3]; 3], coords: [Real; 3]) -> Point3 {
    Point3::new(
        &origin.x
            + &(&coords[0] * &axes[0][0])
            + &(&coords[1] * &axes[1][0])
            + &(&coords[2] * &axes[2][0]),
        &origin.y
            + &(&coords[0] * &axes[0][1])
            + &(&coords[1] * &axes[1][1])
            + &(&coords[2] * &axes[2][1]),
        &origin.z
            + &(&coords[0] * &axes[0][2])
            + &(&coords[1] * &axes[1][2])
            + &(&coords[2] * &axes[2][2]),
    )
}

fn point_to_array(point: &Point3) -> [Real; 3] {
    [point.x.clone(), point.y.clone(), point.z.clone()]
}

fn sub_points_array(left: &[Real; 3], right: &[Real; 3]) -> [Real; 3] {
    [
        &left[0] - &right[0],
        &left[1] - &right[1],
        &left[2] - &right[2],
    ]
}

fn cross_arrays(left: &[Real; 3], right: &[Real; 3]) -> [Real; 3] {
    [
        (&left[1] * &right[2]) - (&left[2] * &right[1]),
        (&left[2] * &right[0]) - (&left[0] * &right[2]),
        (&left[0] * &right[1]) - (&left[1] * &right[0]),
    ]
}

fn dot_arrays(left: &[Real; 3], right: &[Real; 3]) -> Real {
    (&left[0] * &right[0]) + (&left[1] * &right[1]) + (&left[2] * &right[2])
}

/// Performs a boolean operation on owned mesh values through the borrowed API.
pub fn boolean_operation(
    meshes: &[InputMesh],
    op: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    let refs = meshes.iter().map(InputMesh::as_ref).collect::<Vec<_>>();
    boolean_operation_refs(&refs, op, config)
}

/// Borrowed union convenience wrapper.
pub fn boolean_union_refs(
    a: MeshRef<'_>,
    b: MeshRef<'_>,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_operation_refs(&[a, b], BooleanOp::Union, config)
}

/// Owned union convenience wrapper.
pub fn boolean_union(
    a: &InputMesh,
    b: &InputMesh,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_union_refs(a.as_ref(), b.as_ref(), config)
}

/// Borrowed intersection convenience wrapper.
pub fn boolean_intersection_refs(
    a: MeshRef<'_>,
    b: MeshRef<'_>,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_operation_refs(&[a, b], BooleanOp::Intersection, config)
}

/// Owned intersection convenience wrapper.
pub fn boolean_intersection(
    a: &InputMesh,
    b: &InputMesh,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_intersection_refs(a.as_ref(), b.as_ref(), config)
}

/// Borrowed difference convenience wrapper.
pub fn boolean_difference_refs(
    a: MeshRef<'_>,
    b: MeshRef<'_>,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_operation_refs(&[a, b], BooleanOp::Difference, config)
}

/// Owned difference convenience wrapper.
pub fn boolean_difference(
    a: &InputMesh,
    b: &InputMesh,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_difference_refs(a.as_ref(), b.as_ref(), config)
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
    let mut point = bounds.min.clone();
    for axis in 0..3 {
        *axis_mut(&mut point, axis) = axis_ref(&point, axis) - &one;
    }
    point
}

fn outside_reference_points(bounds: &Aabb) -> Vec<Point3> {
    let one = Real::one();
    let x = [&bounds.min.x - &one, &bounds.max.x + &one];
    let y = [&bounds.min.y - &one, &bounds.max.y + &one];
    let z = [&bounds.min.z - &one, &bounds.max.z + &one];
    let mut points = Vec::with_capacity(8);
    for x_value in &x {
        for y_value in &y {
            for z_value in &z {
                points.push(Point3::new(
                    x_value.clone(),
                    y_value.clone(),
                    z_value.clone(),
                ));
            }
        }
    }
    points
}
