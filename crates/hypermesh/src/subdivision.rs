//! Leaf processing for the subdivision pipeline.

use crate::bvh::ExactBvh;
use crate::clip::{ClipSide, clip_polygon};
use crate::error::HypermeshResult;
use crate::geometry::{Aabb, Classification, Plane, axis_mut, axis_ref, compare_real};
use crate::intersection::{PairwiseIntersection, PairwiseIntersectionType, intersect_polygons};
use crate::local_bsp::LocalBsp;
use crate::output::ClassifiedPolygon;
use crate::polygon::ConvexPolygon;
use crate::segment_trace::{
    affine_from_planes, axis_plane_definition, classify_leaf_polygon, trace_plane_replacement_path,
};
use crate::winding::{
    BooleanOp, Indicator, WindingPair, can_boolean_op_be_inside_with_component_ranges,
    classify_polygon_output, propagate_wnv,
};
use hyperlattice::{HomogeneousPoint3, Point3, Real};
use hyperlimit::{
    HalfspaceFeasibility, Plane3 as LimitPlane3, PredicateOutcome, classify_halfspace_feasibility3,
};

/// Default leaf threshold for subdivision.
pub const DEFAULT_LEAF_THRESHOLD: usize = 25;

/// Default maximum subdivision depth.
pub const DEFAULT_MAX_DEPTH: usize = 40;

/// Configuration for recursive subdivision.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SubdivisionConfig {
    /// Polygon-count threshold for leaf processing.
    pub leaf_threshold: usize,
    /// Maximum recursive depth.
    pub max_depth: usize,
}

impl Default for SubdivisionConfig {
    fn default() -> Self {
        Self {
            leaf_threshold: DEFAULT_LEAF_THRESHOLD,
            max_depth: DEFAULT_MAX_DEPTH,
        }
    }
}

/// A subproblem in the subdivision tree.
#[derive(Clone, Debug, PartialEq)]
pub struct SubdivisionTask {
    /// Polygons clipped to this task.
    pub polygons: Vec<ConvexPolygon>,
    /// Task bounds.
    pub bounds: Aabb,
    /// Reference point with known winding.
    pub ref_point: Point3,
    /// Plane triples that certify constructions of `ref_point`.
    pub ref_definitions: Vec<[crate::geometry::Plane; 3]>,
    /// Winding number at `ref_point`.
    pub ref_wnv: Vec<i32>,
    /// Recursive depth.
    pub depth: usize,
}

impl SubdivisionTask {
    /// Constructs a root subdivision task.
    pub fn new(
        polygons: Vec<ConvexPolygon>,
        bounds: Aabb,
        ref_point: Point3,
        ref_wnv: Vec<i32>,
    ) -> Self {
        Self {
            ref_definitions: vec![axis_plane_definition(&ref_point)],
            polygons,
            bounds,
            ref_point,
            ref_wnv,
            depth: 0,
        }
    }
}

/// Basic counters from leaf processing.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LeafProcessingStats {
    /// Number of polygons inspected in this leaf.
    pub polygon_count: usize,
    /// Number of non-empty pairwise intersections.
    pub intersection_count: usize,
    /// Number of polygons emitted through direct leaf classification.
    pub direct_polygon_count: usize,
    /// Number of enabled face-local BSP leaves classified.
    pub bsp_leaf_count: usize,
    /// Number of BSP fragments emitted.
    pub bsp_fragment_count: usize,
    /// Whether every emitted or discarded output decision in this leaf was
    /// certified after exact local BSP isolation checks and exact classifier
    /// traces.
    pub certified_complete: bool,
}

/// Processes one leaf and returns classified output polygons.
pub fn process_leaf(
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    indicator: &Indicator,
) -> HypermeshResult<Vec<ClassifiedPolygon>> {
    let mut output = Vec::new();
    process_leaf_into(
        polygons,
        bounds,
        ref_point,
        ref_definitions,
        ref_wnv,
        indicator,
        &mut output,
    )?;
    Ok(output)
}

/// Processes one leaf into an existing output buffer.
pub fn process_leaf_into(
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    indicator: &Indicator,
    output: &mut Vec<ClassifiedPolygon>,
) -> HypermeshResult<LeafProcessingStats> {
    let mut certified_output = Vec::new();
    let stats = process_leaf_into_inner(
        polygons,
        bounds,
        ref_point,
        ref_definitions,
        ref_wnv,
        indicator,
        &mut certified_output,
    )?;
    output.extend(certified_output);
    Ok(stats)
}

fn process_leaf_into_inner(
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    indicator: &Indicator,
    output: &mut Vec<ClassifiedPolygon>,
) -> HypermeshResult<LeafProcessingStats> {
    let mut stats = LeafProcessingStats {
        polygon_count: polygons.len(),
        ..LeafProcessingStats::default()
    };
    if polygons.is_empty() {
        stats.certified_complete = true;
        return Ok(stats);
    }

    let intersections = pairwise_intersections_by_polygon(polygons)?;
    stats.intersection_count = intersections.iter().map(Vec::len).sum();

    for (index, polygon) in polygons.iter().enumerate() {
        if intersections[index].is_empty() {
            let emitted = emit_one_direct(
                polygon,
                bounds,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                indicator,
                output,
            )?;
            stats.direct_polygon_count += usize::from(emitted);
            continue;
        }

        let mut bsp = LocalBsp::new(polygon);
        for intersection in &intersections[index] {
            match intersection.kind {
                PairwiseIntersectionType::Segment => {
                    if let Some(segment) = &intersection.segment {
                        bsp.add_segment(segment)?;
                    }
                }
                PairwiseIntersectionType::Overlap => {
                    if let Some(overlap) = &intersection.overlap {
                        bsp.add_overlap(&polygons[overlap.other_polygon_idx], overlap)?;
                    }
                }
                PairwiseIntersectionType::None | PairwiseIntersectionType::Point => {}
            }
        }

        for leaf in bsp.collect_leaves() {
            if leaf.edges.len() < 3 {
                continue;
            }
            if !certify_bsp_leaf_has_no_interior_intersections(polygon, &leaf.edges, polygons)? {
                return Err(crate::error::HypermeshError::UnknownClassification);
            }
            stats.bsp_leaf_count += 1;
            let effective_delta_w = effective_leaf_delta_w(polygon, &leaf.edges, polygons)?;
            let w_front = classify_leaf_polygon(
                &polygon.support,
                &leaf.edges,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                bounds,
                &effective_delta_w,
            )?;
            let w_back = propagate_wnv(&w_front, 1, &effective_delta_w)?;
            let classification = classify_polygon_output(&w_front, &w_back, indicator);
            if classification != 0 {
                let mut fragment = polygon.clone();
                fragment.edges = leaf.edges.clone();
                fragment.delta_w = effective_delta_w;
                let mut classified = ClassifiedPolygon::new(fragment, classification);
                classified.winding = Some(WindingPair { w_front, w_back });
                classified.is_bsp_fragment = true;
                output.push(classified);
                stats.bsp_fragment_count += 1;
            }
        }
    }

    stats.certified_complete = true;
    Ok(stats)
}

/// Recursively subdivides a task and returns classified output polygons.
pub fn subdivide(
    task: SubdivisionTask,
    indicator: &Indicator,
    config: SubdivisionConfig,
) -> HypermeshResult<Vec<ClassifiedPolygon>> {
    let mut output = Vec::new();
    subdivide_into_inner(task, indicator, config, None, &mut output)?;
    Ok(output)
}

pub(crate) fn subdivide_for_operation(
    task: SubdivisionTask,
    indicator: &Indicator,
    config: SubdivisionConfig,
    op: BooleanOp,
) -> HypermeshResult<Vec<ClassifiedPolygon>> {
    let mut output = Vec::new();
    subdivide_into_inner(task, indicator, config, Some(op), &mut output)?;
    Ok(output)
}

/// Recursively subdivides a task into an existing output buffer.
///
/// The caller-visible buffer is extended only after the whole task certifies.
/// If subdivision or leaf classification returns an error, no partial output
/// from that task is retained.
pub fn subdivide_into(
    task: SubdivisionTask,
    indicator: &Indicator,
    config: SubdivisionConfig,
    output: &mut Vec<ClassifiedPolygon>,
) -> HypermeshResult<()> {
    let mut certified_output = Vec::new();
    subdivide_into_inner(task, indicator, config, None, &mut certified_output)?;
    output.extend(certified_output);
    Ok(())
}

fn subdivide_into_inner(
    task: SubdivisionTask,
    indicator: &Indicator,
    config: SubdivisionConfig,
    reachability_op: Option<BooleanOp>,
    output: &mut Vec<ClassifiedPolygon>,
) -> HypermeshResult<()> {
    if task.polygons.is_empty() {
        return Ok(());
    }

    if let Some(op) = reachability_op {
        if can_discard_by_winding_reachability(op, &task.ref_wnv, &task.polygons)? {
            return Ok(());
        }
    }

    if task.polygons.len() <= config.leaf_threshold || !can_split_bounds(&task.bounds)? {
        process_leaf_into(
            &task.polygons,
            &task.bounds,
            &task.ref_point,
            &task.ref_definitions,
            &task.ref_wnv,
            indicator,
            output,
        )?;
        return Ok(());
    }

    if task.depth >= config.max_depth {
        let mut certified_output = Vec::new();
        let stats = match process_leaf_into(
            &task.polygons,
            &task.bounds,
            &task.ref_point,
            &task.ref_definitions,
            &task.ref_wnv,
            indicator,
            &mut certified_output,
        ) {
            Ok(stats) => stats,
            Err(crate::error::HypermeshError::UnknownClassification) => {
                return Err(crate::error::HypermeshError::SubdivisionDepthLimit {
                    depth: task.depth,
                    polygon_count: task.polygons.len(),
                });
            }
            Err(err) => return Err(err),
        };
        if !stats.certified_complete {
            return Err(crate::error::HypermeshError::SubdivisionDepthLimit {
                depth: task.depth,
                polygon_count: task.polygons.len(),
            });
        }
        output.extend(certified_output);
        return Ok(());
    }

    let split_axis = task.bounds.longest_axis()?;
    let split_value = task.bounds.midpoint(split_axis);
    let split_plane = crate::geometry::Plane::axis_aligned(split_axis, split_value.clone());
    let left_bounds = task.bounds.left_half(split_axis, split_value.clone());
    let right_bounds = task.bounds.right_half(split_axis, split_value);

    let mut left_polys = Vec::with_capacity(task.polygons.len());
    let mut right_polys = Vec::with_capacity(task.polygons.len());
    for polygon in &task.polygons {
        let clipped = clip_polygon(polygon, &split_plane)?;
        match clipped.side {
            ClipSide::Left => left_polys.push(polygon.clone()),
            ClipSide::Right => right_polys.push(polygon.clone()),
            ClipSide::Both => {
                left_polys.push(clipped.left);
                right_polys.push(clipped.right);
            }
        }
    }

    if !left_polys.is_empty() {
        let (left_ref, left_ref_definitions, left_wnv) = compute_new_reference(
            &task.ref_point,
            &task.ref_definitions,
            &task.ref_wnv,
            &left_bounds,
            &task.polygons,
        )?;
        subdivide_into_inner(
            SubdivisionTask {
                polygons: left_polys,
                bounds: left_bounds,
                ref_point: left_ref,
                ref_definitions: left_ref_definitions,
                ref_wnv: left_wnv,
                depth: task.depth + 1,
            },
            indicator,
            config,
            reachability_op,
            output,
        )?;
    }

    if !right_polys.is_empty() {
        let (right_ref, right_ref_definitions, right_wnv) = compute_new_reference(
            &task.ref_point,
            &task.ref_definitions,
            &task.ref_wnv,
            &right_bounds,
            &task.polygons,
        )?;
        subdivide_into_inner(
            SubdivisionTask {
                polygons: right_polys,
                bounds: right_bounds,
                ref_point: right_ref,
                ref_definitions: right_ref_definitions,
                ref_wnv: right_wnv,
                depth: task.depth + 1,
            },
            indicator,
            config,
            reachability_op,
            output,
        )?;
    }

    Ok(())
}

fn can_discard_by_winding_reachability(
    op: BooleanOp,
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let mut lower = ref_wnv.to_vec();
    let mut upper = ref_wnv.to_vec();
    for polygon in polygons {
        if polygon.delta_w.len() != ref_wnv.len() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        for ((lower, upper), delta) in lower.iter_mut().zip(&mut upper).zip(&polygon.delta_w) {
            let span = delta
                .checked_abs()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            *lower = lower
                .checked_sub(span)
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            *upper = upper
                .checked_add(span)
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
        }
    }

    Ok(!can_boolean_op_be_inside_with_component_ranges(
        op, &lower, &upper,
    )?)
}

fn emit_one_direct(
    polygon: &ConvexPolygon,
    bounds: &Aabb,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    class_polygons: &[ConvexPolygon],
    indicator: &Indicator,
    output: &mut Vec<ClassifiedPolygon>,
) -> HypermeshResult<bool> {
    let w_front = classify_leaf_polygon(
        &polygon.support,
        &polygon.edges,
        ref_point,
        ref_definitions,
        ref_wnv,
        class_polygons,
        bounds,
        &polygon.delta_w,
    )?;
    let w_back = propagate_wnv(&w_front, 1, &polygon.delta_w)?;
    let classification = classify_polygon_output(&w_front, &w_back, indicator);
    if classification != 0 {
        let mut classified = ClassifiedPolygon::new(polygon.clone(), classification);
        classified.winding = Some(WindingPair { w_front, w_back });
        output.push(classified);
        return Ok(true);
    }
    Ok(false)
}

fn effective_leaf_delta_w(
    polygon: &ConvexPolygon,
    leaf_edges: &[crate::geometry::Plane],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<i32>> {
    let mut delta_w = polygon.delta_w.clone();
    let test_point = leaf_interior_point(&polygon.support, leaf_edges)?;

    for other in polygons {
        if other.polygon_index == polygon.polygon_index && other.mesh_index == polygon.mesh_index {
            continue;
        }
        if delta_w.len() != other.delta_w.len() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let inside_or_on = other.contains_point(&test_point)?;
        let strictly_inside = other.contains_point_strictly(&test_point)?;
        if inside_or_on && !strictly_inside {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        if strictly_inside {
            let sign = if supports_have_same_direction(&polygon.support, &other.support)? {
                1
            } else {
                -1
            };
            for (value, delta) in delta_w.iter_mut().zip(&other.delta_w) {
                *value += sign * *delta;
            }
        }
    }

    Ok(delta_w)
}

fn leaf_interior_point(
    support: &crate::geometry::Plane,
    edges: &[crate::geometry::Plane],
) -> HypermeshResult<HomogeneousPoint3> {
    let leaf = ConvexPolygon {
        support: support.clone(),
        edges: edges.to_vec(),
        mesh_index: -1,
        polygon_index: -1,
        delta_w: Vec::new(),
        approx_bounds: None,
    };
    let vertices = leaf.vertices()?;
    let mut sum = Point3::origin();
    for point in &vertices {
        sum.x += point.x.clone();
        sum.y += point.y.clone();
        sum.z += point.z.clone();
    }
    let denom = Real::from(vertices.len() as u64);
    Ok(HomogeneousPoint3::new(
        (sum.x / denom.clone()).map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        (sum.y / denom.clone()).map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        (sum.z / denom).map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        Real::one(),
    ))
}

fn supports_have_same_direction(
    left: &crate::geometry::Plane,
    right: &crate::geometry::Plane,
) -> HypermeshResult<bool> {
    let dot = (&left.normal.x * &right.normal.x)
        + (&left.normal.y * &right.normal.y)
        + (&left.normal.z * &right.normal.z);
    Ok(crate::geometry::classify_real(&dot)? != Classification::Negative)
}

fn certify_bsp_leaf_has_no_interior_intersections(
    host: &ConvexPolygon,
    leaf_edges: &[crate::geometry::Plane],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let leaf_polygon = ConvexPolygon {
        support: host.support.clone(),
        edges: leaf_edges.to_vec(),
        mesh_index: host.mesh_index,
        polygon_index: host.polygon_index,
        delta_w: host.delta_w.clone(),
        approx_bounds: None,
    };
    let leaf_test_point = leaf_interior_point(&leaf_polygon.support, &leaf_polygon.edges)?;

    for other in polygons {
        if other.mesh_index == host.mesh_index && other.polygon_index == host.polygon_index {
            continue;
        }

        let intersection = intersect_polygons(&leaf_polygon, other, 0)?;
        match intersection.kind {
            PairwiseIntersectionType::None | PairwiseIntersectionType::Point => {}
            PairwiseIntersectionType::Segment => {
                let Some(segment) = intersection.segment else {
                    return Ok(false);
                };
                if segment_midpoint_is_strictly_inside_both(
                    &segment.v0,
                    &segment.v1,
                    &leaf_polygon,
                    other,
                )? {
                    return Ok(false);
                }
            }
            PairwiseIntersectionType::Overlap => {
                let inside_or_on = other.contains_point(&leaf_test_point)?;
                let strictly_inside = other.contains_point_strictly(&leaf_test_point)?;
                if inside_or_on && !strictly_inside {
                    return Ok(false);
                }
                if leaf_polygon_key(host) > leaf_polygon_key(other) && strictly_inside {
                    return Ok(false);
                }
            }
        }
    }

    Ok(true)
}

fn segment_midpoint_is_strictly_inside_both(
    a: &Point3,
    b: &Point3,
    left: &ConvexPolygon,
    right: &ConvexPolygon,
) -> HypermeshResult<bool> {
    let midpoint = HomogeneousPoint3::new(
        ((&a.x + &b.x) / Real::from(2))
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        ((&a.y + &b.y) / Real::from(2))
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        ((&a.z + &b.z) / Real::from(2))
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        Real::one(),
    );
    Ok(left.contains_point_strictly(&midpoint)? && right.contains_point_strictly(&midpoint)?)
}

fn leaf_polygon_key(polygon: &ConvexPolygon) -> (isize, isize) {
    (polygon.mesh_index, polygon.polygon_index)
}

fn pairwise_intersections_by_polygon(
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<Vec<PairwiseIntersection>>> {
    let mut by_polygon = vec![Vec::new(); polygons.len()];
    let bvh = ExactBvh::build(polygons)?;
    let mut candidate_pairs = Vec::new();
    bvh.intersect_pairs(&bvh, |left, right| {
        if left < right {
            candidate_pairs.push((left, right));
        }
    })?;

    for (global_i, global_j) in candidate_pairs {
        let intersection = intersect_polygons(&polygons[global_i], &polygons[global_j], global_j)?;
        if matches!(
            intersection.kind,
            PairwiseIntersectionType::Segment | PairwiseIntersectionType::Overlap
        ) {
            by_polygon[global_i].push(intersection);
        }

        let intersection = intersect_polygons(&polygons[global_j], &polygons[global_i], global_i)?;
        if matches!(
            intersection.kind,
            PairwiseIntersectionType::Segment | PairwiseIntersectionType::Overlap
        ) {
            by_polygon[global_j].push(intersection);
        }
    }

    Ok(by_polygon)
}

fn can_split_bounds(bounds: &Aabb) -> HypermeshResult<bool> {
    for axis in 0..3 {
        if compare_real(&bounds.extent(axis), &Real::zero())?.is_gt() {
            return Ok(true);
        }
    }
    Ok(false)
}

fn compute_new_reference(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<(Point3, Vec<[Plane; 3]>, Vec<i32>)> {
    if is_valid_reference_for_bounds(old_ref, bounds, polygons)? {
        return Ok((
            old_ref.clone(),
            old_ref_definitions.to_vec(),
            old_wnv.to_vec(),
        ));
    }

    let projected = project_reference_point(old_ref, bounds)?;
    let projected_target = ReferenceTarget::axis_defined(projected.clone());
    if let Some(winding) = trace_reference_target(
        old_ref,
        old_ref_definitions,
        old_wnv,
        bounds,
        polygons,
        &projected_target,
    )? {
        return Ok((
            projected_target.point,
            projected_target.definitions,
            winding,
        ));
    }

    if let Some((target, winding)) = projection_axis_escape_reference(
        old_ref,
        old_ref_definitions,
        old_wnv,
        &projected,
        bounds,
        polygons,
    )? {
        return Ok((target.point, target.definitions, winding));
    }

    if let Some((target, winding)) = projection_escape_reference(
        old_ref,
        old_ref_definitions,
        old_wnv,
        &projected,
        bounds,
        polygons,
    )? {
        return Ok((target.point, target.definitions, winding));
    }

    if let Some((target, winding)) =
        support_plane_cell_reference(old_ref, old_ref_definitions, old_wnv, bounds, polygons)?
    {
        return Ok((target.point, target.definitions, winding));
    }

    Err(crate::error::HypermeshError::ReferencePropagationFailed)
}

fn projection_escape_reference(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let Some(escape_bounds) = projection_escape_bounds(projected, bounds, polygons)? else {
        return Ok(None);
    };
    if escape_bounds == *bounds {
        return Ok(None);
    }
    support_plane_cell_reference(
        old_ref,
        old_ref_definitions,
        old_wnv,
        &escape_bounds,
        polygons,
    )
}

fn projection_escape_bounds(
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<Aabb>> {
    let mut min = projected.clone();
    let mut max = projected.clone();

    for axis in 0..3 {
        let bound_min = axis_ref(&bounds.min, axis);
        let bound_max = axis_ref(&bounds.max, axis);
        if compare_real(bound_min, bound_max)?.is_eq() {
            *axis_mut(&mut min, axis) = bound_min.clone();
            *axis_mut(&mut max, axis) = bound_max.clone();
            continue;
        }

        let Some(lower) =
            escaped_reference_axis_stop_value(projected, bounds, polygons, axis, false)?
        else {
            return Ok(None);
        };
        let Some(upper) =
            escaped_reference_axis_stop_value(projected, bounds, polygons, axis, true)?
        else {
            return Ok(None);
        };
        if !compare_real(&lower, &upper)?.is_lt() {
            return Ok(None);
        }
        *axis_mut(&mut min, axis) = lower;
        *axis_mut(&mut max, axis) = upper;
    }

    Ok(Some(Aabb::new(min, max)))
}

fn escaped_reference_axis_stop_value(
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
    direction_positive: bool,
) -> HypermeshResult<Option<Real>> {
    let start_value = axis_ref(projected, axis);
    let bound_value = if direction_positive {
        axis_ref(&bounds.max, axis)
    } else {
        axis_ref(&bounds.min, axis)
    };
    let room = if direction_positive {
        bound_value - start_value
    } else {
        start_value - bound_value
    };
    if !compare_real(&room, &Real::zero())?.is_gt() {
        return Ok(None);
    }

    let mut endpoint = projected.clone();
    *axis_mut(&mut endpoint, axis) = bound_value.clone();
    let mut stop_value = bound_value.clone();

    for polygon in polygons {
        let Some(crossing) = reference_axis_surface_crossing(projected, &endpoint, polygon, axis)?
        else {
            continue;
        };
        if !point_lies_on_local_polygon(&crossing, polygon)? {
            continue;
        }

        let crossing_value = axis_ref(&crossing, axis);
        let order = compare_real(crossing_value, &stop_value)?;
        if (direction_positive && order.is_lt()) || (!direction_positive && order.is_gt()) {
            stop_value = crossing_value.clone();
        }
    }

    if compare_real(&stop_value, start_value)?.is_eq() {
        return Ok(None);
    }
    Ok(Some(stop_value))
}

#[cfg(test)]
fn push_unique_reference_target(targets: &mut Vec<ReferenceTarget>, target: ReferenceTarget) {
    if let Some(existing) = targets
        .iter_mut()
        .find(|existing| existing.point == target.point)
    {
        for definition in target.definitions {
            if !existing
                .definitions
                .iter()
                .any(|candidate| candidate == &definition)
            {
                existing.definitions.push(definition);
            }
        }
    } else {
        targets.push(target);
    }
}

fn projection_axis_escape_reference(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    for axis in 0..3 {
        for direction_positive in [true, false] {
            let Some(stop_value) = escaped_reference_axis_stop_value(
                projected,
                bounds,
                polygons,
                axis,
                direction_positive,
            )?
            else {
                continue;
            };
            let corridor = axis_escape_bounds(projected, axis, stop_value)?;
            if let Some(found) = support_plane_cell_reference(
                old_ref,
                old_ref_definitions,
                old_wnv,
                &corridor,
                polygons,
            )? {
                return Ok(Some(found));
            }
        }
    }
    Ok(None)
}

fn axis_escape_bounds(projected: &Point3, axis: usize, stop_value: Real) -> HypermeshResult<Aabb> {
    let mut min = projected.clone();
    let mut max = projected.clone();
    let start_value = axis_ref(projected, axis);
    if compare_real(start_value, &stop_value)?.is_lt() {
        *axis_mut(&mut max, axis) = stop_value;
    } else {
        *axis_mut(&mut min, axis) = stop_value;
    }
    Ok(Aabb::new(min, max))
}
#[derive(Clone, Debug, PartialEq)]
struct ReferenceTarget {
    point: Point3,
    definitions: Vec<[Plane; 3]>,
}

impl ReferenceTarget {
    fn axis_defined(point: Point3) -> Self {
        Self {
            definitions: vec![axis_plane_definition(&point)],
            point,
        }
    }

    fn with_definitions(point: Point3, definitions: Vec<[Plane; 3]>) -> Self {
        Self { point, definitions }
    }
}

fn trace_reference_target(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    target: &ReferenceTarget,
) -> HypermeshResult<Option<Vec<i32>>> {
    if !is_valid_reference_for_bounds(&target.point, bounds, polygons)? {
        return Ok(None);
    }

    match crate::segment_trace::trace_segment(old_ref, &target.point, old_wnv, polygons) {
        Ok(winding) => return Ok(Some(winding)),
        Err(crate::error::HypermeshError::UnknownClassification) => {}
        Err(err) => return Err(err),
    }

    for start_definition in old_ref_definitions {
        for end_definition in &target.definitions {
            match trace_plane_replacement_path(start_definition, end_definition, old_wnv, polygons)
            {
                Ok(winding) => return Ok(Some(winding)),
                Err(crate::error::HypermeshError::UnknownClassification) => {}
                Err(err) => return Err(err),
            }
        }
    }

    Ok(None)
}

fn is_valid_reference_for_bounds(
    point: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    Ok(point_strictly_inside_bounds(point, bounds)?
        && !point_lies_on_local_surface(point, polygons)?)
}

fn support_plane_cell_reference(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    for margin in support_cell_margins(bounds, polygons.len() + 1)? {
        let mut halfspaces = aabb_core_halfspaces(bounds, &margin)?;
        if halfspaces.is_empty() {
            continue;
        }
        if !halfspace_system_is_feasible(&halfspaces)? {
            continue;
        }

        let mut accept = |halfspaces: &[LimitPlane3],
                          report: hyperlimit::HalfspaceFeasibilityReport|
         -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
            let active_planes = report.active_planes;
            let Some(witness) = report.witness else {
                return Ok(None);
            };
            if !is_valid_reference_for_bounds(&witness, bounds, polygons)? {
                return Ok(None);
            }
            let definitions =
                reference_definitions_from_active_halfspaces(&witness, halfspaces, active_planes)?;
            let target = ReferenceTarget::with_definitions(witness, definitions);
            if let Some(winding) = trace_reference_target(
                old_ref,
                old_ref_definitions,
                old_wnv,
                bounds,
                polygons,
                &target,
            )? {
                return Ok(Some((target, winding)));
            }
            Ok(None)
        };

        if let Some(found) = support_plane_cell_search_from(
            bounds,
            polygons,
            &margin,
            0,
            &mut halfspaces,
            &mut accept,
        )? {
            return Ok(Some(found));
        }
    }
    Ok(None)
}

#[cfg(test)]
fn support_plane_cell_target(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    margin: &Real,
) -> HypermeshResult<Option<ReferenceTarget>> {
    let mut halfspaces = aabb_core_halfspaces(bounds, margin)?;
    if halfspaces.is_empty() {
        return Ok(None);
    }
    if !halfspace_system_is_feasible(&halfspaces)? {
        return Ok(None);
    }

    support_plane_cell_target_from(bounds, polygons, margin, 0, &mut halfspaces)
}

#[cfg(test)]
fn support_plane_cell_target_from(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    margin: &Real,
    polygon_index: usize,
    halfspaces: &mut Vec<LimitPlane3>,
) -> HypermeshResult<Option<ReferenceTarget>> {
    let mut accept = |halfspaces: &[LimitPlane3],
                      report: hyperlimit::HalfspaceFeasibilityReport|
     -> HypermeshResult<Option<ReferenceTarget>> {
        let active_planes = report.active_planes;
        let Some(witness) = report.witness else {
            return Ok(None);
        };
        if is_valid_reference_for_bounds(&witness, bounds, polygons)? {
            let definitions =
                reference_definitions_from_active_halfspaces(&witness, halfspaces, active_planes)?;
            Ok(Some(ReferenceTarget::with_definitions(
                witness,
                definitions,
            )))
        } else {
            Ok(None)
        }
    };
    support_plane_cell_search_from(
        bounds,
        polygons,
        margin,
        polygon_index,
        halfspaces,
        &mut accept,
    )
}

fn support_plane_cell_search_from<T>(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    margin: &Real,
    polygon_index: usize,
    halfspaces: &mut Vec<LimitPlane3>,
    accept: &mut impl FnMut(
        &[LimitPlane3],
        hyperlimit::HalfspaceFeasibilityReport,
    ) -> HypermeshResult<Option<T>>,
) -> HypermeshResult<Option<T>> {
    if polygon_index < polygons.len() {
        for positive in [false, true] {
            halfspaces.push(support_side_halfspace(
                &polygons[polygon_index].support,
                margin,
                positive,
            ));
            let feasible = halfspace_system_is_feasible(halfspaces)?;
            if feasible
                && let Some(target) = support_plane_cell_search_from(
                    bounds,
                    polygons,
                    margin,
                    polygon_index + 1,
                    halfspaces,
                    accept,
                )?
            {
                halfspaces.pop();
                return Ok(Some(target));
            }
            halfspaces.pop();
        }
        return Ok(None);
    }

    let Some(report) = halfspace_system_report(&halfspaces)? else {
        return Ok(None);
    };
    accept(halfspaces, report)
}

fn support_cell_margins(bounds: &Aabb, support_count: usize) -> HypermeshResult<Vec<Real>> {
    let Some(min_extent) = smallest_positive_extent(bounds)? else {
        return Ok(Vec::new());
    };

    let mut margins = Vec::new();
    for scale in [4_u64, 16, 64, 256] {
        let denominator = Real::from(scale * (support_count as u64 + 1));
        margins.push(
            (min_extent.clone() / denominator)
                .map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        );
    }
    Ok(margins)
}

fn smallest_positive_extent(bounds: &Aabb) -> HypermeshResult<Option<Real>> {
    let mut result: Option<Real> = None;
    for axis in 0..3 {
        let extent = bounds.extent(axis);
        if !compare_real(&extent, &Real::zero())?.is_gt() {
            continue;
        }
        if result
            .as_ref()
            .is_none_or(|current| compare_real(&extent, current).is_ok_and(|order| order.is_lt()))
        {
            result = Some(extent);
        }
    }
    Ok(result)
}

fn aabb_core_halfspaces(bounds: &Aabb, margin: &Real) -> HypermeshResult<Vec<LimitPlane3>> {
    let mut halfspaces = Vec::with_capacity(6);
    for axis in 0..3 {
        let min = axis_ref(&bounds.min, axis);
        let max = axis_ref(&bounds.max, axis);
        let extent = max - min;
        if compare_real(&extent, &Real::zero())?.is_eq() {
            halfspaces.push(axis_halfspace(axis, true, min.clone()));
            halfspaces.push(axis_halfspace(axis, false, min.clone()));
            continue;
        }

        let lower = min + margin;
        let upper = max - margin;
        if !compare_real(&lower, &upper)?.is_lt() {
            return Ok(Vec::new());
        }
        halfspaces.push(axis_halfspace(axis, true, lower));
        halfspaces.push(axis_halfspace(axis, false, upper));
    }
    Ok(halfspaces)
}

fn axis_halfspace(axis: usize, lower_bound: bool, value: Real) -> LimitPlane3 {
    let zero = Real::zero();
    let one = Real::one();
    let minus_one = -Real::one();
    let normal = match (axis, lower_bound) {
        (0, true) => Point3::new(minus_one, zero.clone(), zero),
        (1, true) => Point3::new(zero.clone(), minus_one, zero),
        (2, true) => Point3::new(zero.clone(), zero, minus_one),
        (0, false) => Point3::new(one, zero.clone(), zero),
        (1, false) => Point3::new(zero.clone(), one, zero),
        (2, false) => Point3::new(zero.clone(), zero, one),
        _ => panic!("axis must be in 0..3"),
    };
    let offset = if lower_bound { value } else { -value };
    LimitPlane3::new(normal, offset)
}

fn support_side_halfspace(
    plane: &crate::geometry::Plane,
    margin: &Real,
    positive: bool,
) -> LimitPlane3 {
    if positive {
        LimitPlane3::new(
            Point3::new(
                -plane.normal.x.clone(),
                -plane.normal.y.clone(),
                -plane.normal.z.clone(),
            ),
            &(-plane.offset.clone()) + margin,
        )
    } else {
        LimitPlane3::new(plane.normal.clone(), &plane.offset + margin)
    }
}

fn halfspace_system_is_feasible(halfspaces: &[LimitPlane3]) -> HypermeshResult<bool> {
    Ok(matches!(
        halfspace_system_report(halfspaces)?,
        Some(report) if report.status == HalfspaceFeasibility::Feasible
    ))
}

fn halfspace_system_report(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<Option<hyperlimit::HalfspaceFeasibilityReport>> {
    match classify_halfspace_feasibility3(halfspaces) {
        PredicateOutcome::Decided { value, .. } => Ok(Some(value)),
        PredicateOutcome::Unknown { .. } => {
            Err(crate::error::HypermeshError::UnknownClassification)
        }
    }
}

fn reference_definitions_from_active_halfspaces(
    witness: &Point3,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
) -> HypermeshResult<Vec<[Plane; 3]>> {
    let axis_definition = axis_plane_definition(witness);
    let mut definitions = Vec::new();
    let mut active = Vec::new();
    for index in active_planes.into_iter().flatten() {
        let Some(halfspace) = halfspaces.get(index) else {
            return Err(crate::error::HypermeshError::UnknownClassification);
        };
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if !active.iter().any(|existing| existing == &plane) {
            active.push(plane);
        }
    }

    match active.len() {
        3 => {
            push_verified_definition(
                &mut definitions,
                [active[0].clone(), active[1].clone(), active[2].clone()],
                witness,
            )?;
        }
        2 => {
            for axis in 0..3 {
                push_verified_definition(
                    &mut definitions,
                    [
                        active[0].clone(),
                        active[1].clone(),
                        axis_definition[axis].clone(),
                    ],
                    witness,
                )?;
            }
        }
        1 => {
            for first_axis in 0..3 {
                for second_axis in (first_axis + 1)..3 {
                    push_verified_definition(
                        &mut definitions,
                        [
                            active[0].clone(),
                            axis_definition[first_axis].clone(),
                            axis_definition[second_axis].clone(),
                        ],
                        witness,
                    )?;
                }
            }
        }
        0 => {}
        _ => return Err(crate::error::HypermeshError::UnknownClassification),
    }

    push_verified_definition(&mut definitions, axis_definition, witness)?;
    Ok(definitions)
}

fn push_verified_definition(
    definitions: &mut Vec<[Plane; 3]>,
    definition: [Plane; 3],
    witness: &Point3,
) -> HypermeshResult<()> {
    match affine_from_planes(&definition) {
        Ok(point) if point == *witness => {
            if !definitions.iter().any(|existing| existing == &definition) {
                definitions.push(definition);
            }
        }
        Ok(_) | Err(crate::error::HypermeshError::UnknownClassification) => {}
        Err(err) => return Err(err),
    }
    Ok(())
}

#[cfg(test)]
fn point_lies_on_any_support_plane(
    point: &Point3,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    for polygon in polygons {
        if crate::geometry::classify_point(point, &polygon.support)?
            == crate::geometry::Classification::On
        {
            return Ok(true);
        }
    }
    Ok(false)
}

fn reference_axis_surface_crossing(
    start: &Point3,
    endpoint: &Point3,
    polygon: &ConvexPolygon,
    axis: usize,
) -> HypermeshResult<Option<Point3>> {
    let start_class = crate::geometry::classify_point(start, &polygon.support)?;
    let endpoint_class = crate::geometry::classify_point(endpoint, &polygon.support)?;
    if start_class == crate::geometry::Classification::On {
        return Ok(None);
    }
    if endpoint_class == crate::geometry::Classification::On {
        return Ok(Some(endpoint.clone()));
    }
    if start_class == endpoint_class {
        return Ok(None);
    }

    let start_value = polygon.support.expression_at_point(start);
    let endpoint_value = polygon.support.expression_at_point(endpoint);
    let denom = &start_value - &endpoint_value;
    let t =
        (start_value / denom).map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
    let axis_value =
        axis_ref(start, axis) + &(t * (axis_ref(endpoint, axis) - axis_ref(start, axis)));
    let mut crossing = start.clone();
    *axis_mut(&mut crossing, axis) = axis_value;
    Ok(Some(crossing))
}

fn project_reference_point(old_ref: &Point3, bounds: &Aabb) -> HypermeshResult<Point3> {
    let mut target = old_ref.clone();
    for axis in 0..3 {
        let min_order = compare_real(axis_ref(&target, axis), axis_ref(&bounds.min, axis))?;
        let max_order = compare_real(axis_ref(&target, axis), axis_ref(&bounds.max, axis))?;
        if !min_order.is_gt() || !max_order.is_lt() {
            *axis_mut(&mut target, axis) = interior_axis_value(bounds, axis)?;
        }
    }
    Ok(target)
}

fn interior_axis_value(bounds: &Aabb, axis: usize) -> HypermeshResult<Real> {
    let min = axis_ref(&bounds.min, axis);
    let max = axis_ref(&bounds.max, axis);
    let extent = max - min;
    if extent.definitely_zero() {
        return Ok(min.clone());
    }
    Ok(min
        + &((extent * Real::from(1)) / Real::from(2))
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?)
}

fn point_strictly_inside_bounds(point: &Point3, bounds: &Aabb) -> HypermeshResult<bool> {
    for axis in 0..3 {
        let min = axis_ref(&bounds.min, axis);
        let max = axis_ref(&bounds.max, axis);
        if compare_real(min, max)?.is_eq() {
            if compare_real(axis_ref(point, axis), min)?.is_ne() {
                return Ok(false);
            }
            continue;
        }
        if !compare_real(axis_ref(point, axis), min)?.is_gt()
            || !compare_real(axis_ref(point, axis), max)?.is_lt()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn point_lies_on_local_surface(
    point: &Point3,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    for polygon in polygons {
        if point_lies_on_local_polygon(point, polygon)? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn point_lies_on_local_polygon(point: &Point3, polygon: &ConvexPolygon) -> HypermeshResult<bool> {
    if crate::geometry::classify_point(point, &polygon.support)?
        != crate::geometry::Classification::On
    {
        return Ok(false);
    }
    for edge in &polygon.edges {
        if crate::geometry::classify_point(point, edge)?.is_positive() {
            return Ok(false);
        }
    }
    Ok(true)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Plane;
    use crate::polygon::make_triangle;

    fn r(value: i32) -> Real {
        value.into()
    }

    fn q(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    }

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    fn axis_defs(point: &Point3) -> Vec<[Plane; 3]> {
        vec![axis_plane_definition(point)]
    }

    fn definition_uses_non_axis_plane(definition: &[Plane; 3]) -> bool {
        definition.iter().any(|plane| {
            plane.normal != p(1, 0, 0) && plane.normal != p(0, 1, 0) && plane.normal != p(0, 0, 1)
        })
    }

    #[test]
    fn can_split_any_certified_positive_extent() {
        let bounds = Aabb::new(p(0, 0, 0), p(1, 0, 0));

        assert!(can_split_bounds(&bounds).unwrap());
    }

    #[test]
    fn cannot_split_zero_extent_bounds() {
        let bounds = Aabb::new(p(0, 0, 0), p(0, 0, 0));

        assert!(!can_split_bounds(&bounds).unwrap());
    }

    #[test]
    fn point_strictly_inside_bounds_rejects_positive_extent_boundary() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

        assert!(!point_strictly_inside_bounds(&p(0, 2, 2), &bounds).unwrap());
        assert!(point_strictly_inside_bounds(&p(2, 2, 2), &bounds).unwrap());
    }

    #[test]
    fn point_strictly_inside_bounds_accepts_zero_extent_axis_on_plane() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 0, 4));

        assert!(point_strictly_inside_bounds(&p(2, 0, 2), &bounds).unwrap());
        assert!(!point_strictly_inside_bounds(&p(2, 1, 2), &bounds).unwrap());
    }

    #[test]
    fn project_reference_point_moves_non_strict_axes_to_midpoint() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

        assert_eq!(
            project_reference_point(&p(0, 2, 5), &bounds).unwrap(),
            p(2, 2, 2)
        );
    }

    #[test]
    fn valid_reference_rejects_local_surface_points() {
        let mut wall = make_triangle(&p(2, 0, 0), &p(2, 4, 0), &p(2, 2, 4), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

        assert!(!is_valid_reference_for_bounds(&p(2, 2, 1), &bounds, &[wall]).unwrap());
    }

    #[test]
    fn trace_reference_target_rejects_uncertified_targets() {
        let mut wall = make_triangle(&p(2, 0, 0), &p(2, 4, 0), &p(2, 2, 4), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

        assert_eq!(
            trace_reference_target(
                &p(-1, -1, -1),
                &axis_defs(&p(-1, -1, -1)),
                &[0],
                &bounds,
                &[wall.clone()],
                &ReferenceTarget::axis_defined(p(2, 2, 1))
            )
            .unwrap(),
            None
        );
        assert_eq!(
            trace_reference_target(
                &p(-1, -1, -1),
                &axis_defs(&p(-1, -1, -1)),
                &[0],
                &bounds,
                &[wall],
                &ReferenceTarget::axis_defined(p(5, 2, 2))
            )
            .unwrap(),
            None
        );
    }

    #[test]
    fn projection_escape_bounds_stop_at_nearest_axis_surfaces() {
        let mut left = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
        left.delta_w = vec![1];
        let mut right = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
        right.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

        let escape = projection_escape_bounds(&p(1, 3, 3), &bounds, &[left, right])
            .unwrap()
            .expect("parallel walls should define a strict projection escape box");

        assert_eq!(escape.min.x, r(0));
        assert_eq!(escape.max.x, r(4));
        assert_eq!(escape.min.y, r(0));
        assert_eq!(escape.max.y, r(6));
        assert_eq!(escape.min.z, r(0));
        assert_eq!(escape.max.z, r(6));
    }

    #[test]
    fn projection_axis_escape_reference_finds_corridor_witness() {
        let mut left = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
        left.delta_w = vec![1];
        let mut right = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
        right.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

        let (target, winding) = projection_axis_escape_reference(
            &p(-1, 3, 3),
            &axis_defs(&p(-1, 3, 3)),
            &[0],
            &p(1, 3, 3),
            &bounds,
            &[left, right],
        )
        .unwrap()
        .expect("axis escape corridor should contain a certified witness");

        assert_eq!(winding.len(), 1);
        assert_ne!(winding[0], 0);
        assert_eq!(target.point.y, r(3));
        assert_eq!(target.point.z, r(3));
        assert!(compare_real(&target.point.x, &r(1)).unwrap().is_gt());
        assert!(compare_real(&target.point.x, &r(4)).unwrap().is_lt());
        assert!(!target.definitions.is_empty());
    }

    #[test]
    fn support_plane_cell_finds_target_when_midpoint_is_blocked() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let polygons = vec![
            support_only_polygon(Plane::axis_aligned(0, r(5))),
            support_only_polygon(Plane::axis_aligned(1, r(5))),
            support_only_polygon(Plane::axis_aligned(2, r(5))),
            support_only_polygon(Plane::from_coefficients(r(1), r(1), r(1), r(-15))),
        ];

        assert!(point_lies_on_any_support_plane(&p(5, 5, 5), &polygons).unwrap());

        let margin = support_cell_margins(&bounds, polygons.len() + 1).unwrap()[0].clone();
        let target = support_plane_cell_target(&bounds, &polygons, &margin)
            .unwrap()
            .expect("strict support cell should have a feasible witness");

        assert!(point_strictly_inside_bounds(&target.point, &bounds).unwrap());
        assert!(!point_lies_on_any_support_plane(&target.point, &polygons).unwrap());
        assert!(
            target
                .definitions
                .iter()
                .any(|definition| affine_from_planes(definition).unwrap() == target.point)
        );
    }

    #[test]
    fn duplicate_reference_targets_merge_definitions() {
        let point = p(1, 2, 3);
        let mut targets = vec![ReferenceTarget::axis_defined(point.clone())];
        let slanted_definition = [
            Plane::from_coefficients(r(1), r(1), r(0), r(-3)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(2, r(3)),
        ];

        push_unique_reference_target(
            &mut targets,
            ReferenceTarget::with_definitions(point, vec![slanted_definition.clone()]),
        );
        push_unique_reference_target(
            &mut targets,
            ReferenceTarget::with_definitions(p(1, 2, 3), vec![slanted_definition]),
        );

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].definitions.len(), 2);
        assert!(
            targets[0]
                .definitions
                .iter()
                .any(definition_uses_non_axis_plane)
        );
    }

    #[test]
    fn winding_reachability_prunes_difference_when_other_mesh_cannot_reach_zero() {
        let mut first = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        first.delta_w = vec![0, 1];
        let mut second = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
        second.delta_w = vec![0, 1];

        assert!(
            can_discard_by_winding_reachability(BooleanOp::Difference, &[1, 3], &[first, second])
                .unwrap()
        );
    }

    #[test]
    fn winding_reachability_keeps_difference_when_other_mesh_can_reach_zero() {
        let mut first = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        first.delta_w = vec![0, 1];

        assert!(
            !can_discard_by_winding_reachability(BooleanOp::Difference, &[1, 1], &[first]).unwrap()
        );
    }

    #[test]
    fn support_plane_cell_backtracks_when_first_feasible_side_dead_ends() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let polygons = vec![
            support_only_polygon(Plane::from_coefficients(r(-1), r(0), r(0), q(7, 2))),
            support_only_polygon(Plane::from_coefficients(r(1), r(0), r(0), q(-13, 2))),
            support_only_polygon(Plane::axis_aligned(0, r(5))),
        ];

        let target = support_plane_cell_target(&bounds, &polygons, &r(1))
            .unwrap()
            .expect("backtracking should find an alternate feasible support cell");

        assert!(point_strictly_inside_bounds(&target.point, &bounds).unwrap());
        assert!(!point_lies_on_any_support_plane(&target.point, &polygons).unwrap());
        assert!(compare_real(&target.point.x, &r(6)).unwrap().is_gt());
        assert!(
            target
                .definitions
                .iter()
                .any(definition_uses_non_axis_plane)
        );
    }

    #[test]
    fn support_plane_cell_search_backtracks_after_leaf_rejection() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(5)))];
        let margin = r(1);
        let mut halfspaces = aabb_core_halfspaces(&bounds, &margin).unwrap();
        let mut rejected_first_leaf = false;
        let mut accept = |_halfspaces: &[LimitPlane3],
                          report: hyperlimit::HalfspaceFeasibilityReport|
         -> HypermeshResult<Option<Point3>> {
            let Some(witness) = report.witness else {
                return Ok(None);
            };
            if compare_real(&witness.x, &r(5))?.is_lt() {
                rejected_first_leaf = true;
                return Ok(None);
            }
            Ok(Some(witness))
        };

        let target = support_plane_cell_search_from(
            &bounds,
            &polygons,
            &margin,
            0,
            &mut halfspaces,
            &mut accept,
        )
        .unwrap()
        .expect("search should continue after the first accepted leaf rejects");

        assert!(rejected_first_leaf);
        assert!(compare_real(&target.x, &r(5)).unwrap().is_gt());
    }

    #[test]
    fn support_plane_cell_reference_traces_certified_winding() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let polygons = vec![
            support_only_polygon(Plane::axis_aligned(0, r(5))),
            support_only_polygon(Plane::axis_aligned(1, r(5))),
            support_only_polygon(Plane::axis_aligned(2, r(5))),
        ];

        let (target, winding) = support_plane_cell_reference(
            &p(-1, -1, -1),
            &axis_defs(&p(-1, -1, -1)),
            &[7],
            &bounds,
            &polygons,
        )
        .unwrap()
        .expect("strict support cell target should trace from old reference");

        assert_eq!(winding, vec![7]);
        assert!(point_strictly_inside_bounds(&target.point, &bounds).unwrap());
        assert!(!point_lies_on_any_support_plane(&target.point, &polygons).unwrap());
        assert!(!target.definitions.is_empty());
    }

    #[test]
    fn support_plane_cell_reference_retains_active_plane_definitions() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let polygons = vec![
            support_only_polygon(Plane::axis_aligned(0, r(5))),
            support_only_polygon(Plane::axis_aligned(1, r(5))),
            support_only_polygon(Plane::axis_aligned(2, r(5))),
            support_only_polygon(Plane::from_coefficients(r(1), r(1), r(1), r(-15))),
        ];

        let (target, winding) = support_plane_cell_reference(
            &p(-1, -1, -1),
            &axis_defs(&p(-1, -1, -1)),
            &[3],
            &bounds,
            &polygons,
        )
        .unwrap()
        .expect("support-cell witness should be traceable");

        assert_eq!(winding, vec![3]);
        assert!(
            target
                .definitions
                .iter()
                .any(definition_uses_non_axis_plane)
        );
        for definition in &target.definitions {
            assert_eq!(affine_from_planes(definition).unwrap(), target.point);
        }
    }

    #[test]
    fn reference_propagation_reports_exhausted_construction() {
        let bounds = Aabb::new(p(0, 0, 0), p(0, 0, 0));
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(0)))];

        let err = compute_new_reference(
            &p(-1, -1, -1),
            &axis_defs(&p(-1, -1, -1)),
            &[0],
            &bounds,
            &polygons,
        )
        .unwrap_err();

        assert_eq!(
            err,
            crate::error::HypermeshError::ReferencePropagationFailed
        );
    }

    #[test]
    fn subdivide_into_keeps_output_unchanged_on_uncertified_failure() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(1, -1, -1), p(1, 1, 1));
        let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Union, 1);
        let sentinel = ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 99),
            1,
        );
        let mut output = vec![sentinel.clone()];

        let err = subdivide_into(
            SubdivisionTask::new(vec![wall], bounds, p(0, 0, 0), vec![0]),
            &indicator,
            SubdivisionConfig {
                leaf_threshold: 0,
                max_depth: 0,
            },
            &mut output,
        )
        .unwrap_err();

        assert_eq!(
            err,
            crate::error::HypermeshError::SubdivisionDepthLimit {
                depth: 0,
                polygon_count: 1
            }
        );
        assert_eq!(output, vec![sentinel]);
    }

    #[test]
    fn operation_subdivision_discards_fixed_difference_outside_region() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 1, 0);
        wall.delta_w = vec![0, 1];
        let bounds = Aabb::new(p(1, -1, -1), p(1, 1, 1));
        let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Difference, 2);

        let output = subdivide_for_operation(
            SubdivisionTask::new(vec![wall], bounds, p(0, 0, 0), vec![0, 0]),
            &indicator,
            SubdivisionConfig {
                leaf_threshold: 0,
                max_depth: 0,
            },
            BooleanOp::Difference,
        )
        .unwrap();

        assert!(output.is_empty());
    }

    #[test]
    fn operation_subdivision_keeps_potential_difference_region() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1, 0];
        let bounds = Aabb::new(p(1, -1, -1), p(1, 1, 1));
        let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Difference, 2);

        let err = subdivide_for_operation(
            SubdivisionTask::new(vec![wall], bounds, p(0, 0, 0), vec![0, 0]),
            &indicator,
            SubdivisionConfig {
                leaf_threshold: 0,
                max_depth: 0,
            },
            BooleanOp::Difference,
        )
        .unwrap_err();

        assert_eq!(
            err,
            crate::error::HypermeshError::SubdivisionDepthLimit {
                depth: 0,
                polygon_count: 1
            }
        );
    }

    #[test]
    fn process_leaf_into_keeps_output_unchanged_on_uncertified_failure() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(1, -1, -1), p(1, 1, 1));
        let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Union, 1);
        let sentinel = ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 99),
            1,
        );
        let mut output = vec![sentinel.clone()];

        let err = process_leaf_into(
            &[wall],
            &bounds,
            &p(0, 0, 0),
            &axis_defs(&p(0, 0, 0)),
            &[0],
            &indicator,
            &mut output,
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
        assert_eq!(output, vec![sentinel]);
    }

    #[test]
    fn bsp_leaf_certification_rejects_unsplit_interior_segment() {
        let mut host = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        host.delta_w = vec![1, 0];
        let mut cutter = make_triangle(&p(1, 0, -1), &p(1, 0, 1), &p(1, 2, 0), 1, 0);
        cutter.delta_w = vec![0, 1];
        let polygons = vec![host.clone(), cutter];

        assert!(
            !certify_bsp_leaf_has_no_interior_intersections(&host, &host.edges, &polygons).unwrap()
        );
    }

    #[test]
    fn bsp_leaf_certification_rejects_boundary_ambiguous_overlap() {
        let mut host = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 1, 0);
        host.delta_w = vec![0, 1];
        let mut overlap = make_triangle(
            &p(0, 0, 0),
            &Point3::new(q(4, 3), r(0), r(0)),
            &Point3::new(r(0), q(4, 3), r(0)),
            0,
            0,
        );
        overlap.delta_w = vec![1, 0];
        let polygons = vec![host.clone(), overlap];

        assert!(
            !certify_bsp_leaf_has_no_interior_intersections(&host, &host.edges, &polygons).unwrap()
        );
    }

    fn support_only_polygon(support: Plane) -> ConvexPolygon {
        ConvexPolygon {
            support,
            edges: Vec::new(),
            mesh_index: 0,
            polygon_index: 0,
            delta_w: Vec::new(),
            approx_bounds: None,
        }
    }
}
