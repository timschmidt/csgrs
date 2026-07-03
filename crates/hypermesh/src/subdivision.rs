//! Leaf processing for the subdivision pipeline.

use std::collections::BTreeMap;

use crate::bvh::ExactBvh;
use crate::clip::{ClipSide, clip_polygon};
use crate::error::HypermeshResult;
use crate::geometry::{Aabb, Classification, axis_mut, axis_ref, compare_real};
use crate::intersection::{PairwiseIntersection, PairwiseIntersectionType, intersect_polygons};
use crate::local_bsp::LocalBsp;
use crate::output::ClassifiedPolygon;
use crate::polygon::ConvexPolygon;
use crate::segment_trace::classify_leaf_polygon;
use crate::winding::{Indicator, WindingPair, classify_polygon_output, propagate_wnv};
use hyperlattice::{HomogeneousPoint3, Point3, Real};

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
    ref_wnv: &[i32],
    indicator: &Indicator,
) -> HypermeshResult<Vec<ClassifiedPolygon>> {
    let mut output = Vec::new();
    process_leaf_into(polygons, bounds, ref_point, ref_wnv, indicator, &mut output)?;
    Ok(output)
}

/// Processes one leaf into an existing output buffer.
pub fn process_leaf_into(
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    ref_point: &Point3,
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
                polygon, bounds, ref_point, ref_wnv, polygons, indicator, output,
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
                ref_wnv,
                polygons,
                bounds,
                &effective_delta_w,
            )?;
            let w_back = propagate_wnv(&w_front, 1, &effective_delta_w);
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
    subdivide_into(task, indicator, config, &mut output)?;
    Ok(output)
}

/// Recursively subdivides a task into an existing output buffer.
pub fn subdivide_into(
    task: SubdivisionTask,
    indicator: &Indicator,
    config: SubdivisionConfig,
    output: &mut Vec<ClassifiedPolygon>,
) -> HypermeshResult<()> {
    if task.polygons.is_empty() {
        return Ok(());
    }

    if task.polygons.len() <= config.leaf_threshold || !can_split_bounds(&task.bounds)? {
        process_leaf_into(
            &task.polygons,
            &task.bounds,
            &task.ref_point,
            &task.ref_wnv,
            indicator,
            output,
        )?;
        return Ok(());
    }

    if task.depth >= config.max_depth {
        let mut certified_output = Vec::new();
        let stats = process_leaf_into(
            &task.polygons,
            &task.bounds,
            &task.ref_point,
            &task.ref_wnv,
            indicator,
            &mut certified_output,
        )?;
        if !stats.certified_complete {
            return Err(crate::error::HypermeshError::UnknownClassification);
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

    let (left_ref, left_wnv) =
        compute_new_reference(&task.ref_point, &task.ref_wnv, &left_bounds, &task.polygons)?;
    let (right_ref, right_wnv) = compute_new_reference(
        &task.ref_point,
        &task.ref_wnv,
        &right_bounds,
        &task.polygons,
    )?;

    if !left_polys.is_empty() {
        subdivide_into(
            SubdivisionTask {
                polygons: left_polys,
                bounds: left_bounds,
                ref_point: left_ref,
                ref_wnv: left_wnv,
                depth: task.depth + 1,
            },
            indicator,
            config,
            output,
        )?;
    }

    if !right_polys.is_empty() {
        subdivide_into(
            SubdivisionTask {
                polygons: right_polys,
                bounds: right_bounds,
                ref_point: right_ref,
                ref_wnv: right_wnv,
                depth: task.depth + 1,
            },
            indicator,
            config,
            output,
        )?;
    }

    Ok(())
}

fn emit_one_direct(
    polygon: &ConvexPolygon,
    bounds: &Aabb,
    ref_point: &Point3,
    ref_wnv: &[i32],
    class_polygons: &[ConvexPolygon],
    indicator: &Indicator,
    output: &mut Vec<ClassifiedPolygon>,
) -> HypermeshResult<bool> {
    let w_front = classify_leaf_polygon(
        &polygon.support,
        &polygon.edges,
        ref_point,
        ref_wnv,
        class_polygons,
        bounds,
        &polygon.delta_w,
    )?;
    let w_back = propagate_wnv(&w_front, 1, &polygon.delta_w);
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
        if other.contains_point_strictly(&test_point)? {
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
                if leaf_polygon_key(host) > leaf_polygon_key(other)
                    && other.contains_point_strictly(&leaf_test_point)?
                {
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
    let mesh_groups = mesh_groups(polygons);

    for (mesh_i, ids_i) in mesh_groups.iter() {
        for (mesh_j, ids_j) in mesh_groups.iter() {
            if mesh_i >= mesh_j {
                continue;
            }

            let polys_i = ids_i
                .iter()
                .map(|index| polygons[*index].clone())
                .collect::<Vec<_>>();
            let polys_j = ids_j
                .iter()
                .map(|index| polygons[*index].clone())
                .collect::<Vec<_>>();
            let bvh_i = ExactBvh::build(&polys_i)?;
            let bvh_j = ExactBvh::build(&polys_j)?;

            let mut candidate_pairs = Vec::new();
            bvh_i.intersect_pairs(&bvh_j, |local_i, local_j| {
                candidate_pairs.push((ids_i[local_i], ids_j[local_j]));
            })?;

            for (global_i, global_j) in candidate_pairs {
                let intersection =
                    intersect_polygons(&polygons[global_i], &polygons[global_j], global_j)?;
                if matches!(
                    intersection.kind,
                    PairwiseIntersectionType::Segment | PairwiseIntersectionType::Overlap
                ) {
                    by_polygon[global_i].push(intersection);
                }

                let intersection =
                    intersect_polygons(&polygons[global_j], &polygons[global_i], global_i)?;
                if matches!(
                    intersection.kind,
                    PairwiseIntersectionType::Segment | PairwiseIntersectionType::Overlap
                ) {
                    by_polygon[global_j].push(intersection);
                }
            }
        }
    }

    Ok(by_polygon)
}

fn mesh_groups(polygons: &[ConvexPolygon]) -> BTreeMap<isize, Vec<usize>> {
    let mut groups = BTreeMap::new();
    for (index, polygon) in polygons.iter().enumerate() {
        groups
            .entry(polygon.mesh_index)
            .or_insert_with(Vec::new)
            .push(index);
    }
    groups
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
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<(Point3, Vec<i32>)> {
    if is_valid_reference_for_bounds(old_ref, bounds, polygons)? {
        return Ok((old_ref.clone(), old_wnv.to_vec()));
    }

    let projected = project_reference_point(old_ref, bounds)?;
    for target in reference_targets_from_projection(&projected, bounds, polygons)? {
        if point_lies_on_local_surface(&target, polygons)? {
            continue;
        }
        if let Ok(winding) =
            crate::segment_trace::trace_segment(old_ref, &target, old_wnv, polygons)
        {
            return Ok((target, winding));
        }
    }

    if let Some((target, winding)) =
        support_plane_grid_reference(old_ref, old_wnv, bounds, polygons)?
    {
        return Ok((target, winding));
    }

    Err(crate::error::HypermeshError::UnknownClassification)
}

fn is_valid_reference_for_bounds(
    point: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    Ok(point_strictly_inside_bounds(point, bounds)?
        && !point_lies_on_local_surface(point, polygons)?)
}

fn reference_targets_from_projection(
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<Point3>> {
    let mut targets = Vec::new();
    if !point_lies_on_local_surface(projected, polygons)? {
        targets.push(projected.clone());
    }

    let mut axis_values = vec![Vec::new(), Vec::new(), Vec::new()];
    for axis in 0..3 {
        for direction_positive in [true, false] {
            if let Some(value) =
                escaped_reference_axis_value(projected, bounds, polygons, axis, direction_positive)?
            {
                push_unique_real(&mut axis_values[axis], value.clone());
                let mut target = projected.clone();
                *axis_mut(&mut target, axis) = value;
                push_unique_point(&mut targets, target);
            }
        }
    }

    for first_axis in 0..3 {
        for second_axis in (first_axis + 1)..3 {
            for first_value in &axis_values[first_axis] {
                for second_value in &axis_values[second_axis] {
                    let mut target = projected.clone();
                    *axis_mut(&mut target, first_axis) = first_value.clone();
                    *axis_mut(&mut target, second_axis) = second_value.clone();
                    push_unique_point(&mut targets, target);
                }
            }
        }
    }

    for x in &axis_values[0] {
        for y in &axis_values[1] {
            for z in &axis_values[2] {
                push_unique_point(&mut targets, Point3::new(x.clone(), y.clone(), z.clone()));
            }
        }
    }
    Ok(targets)
}

fn push_unique_real(values: &mut Vec<Real>, value: Real) {
    if !values.iter().any(|existing| existing == &value) {
        values.push(value);
    }
}

fn push_unique_point(points: &mut Vec<Point3>, point: Point3) {
    if !points.iter().any(|existing| existing == &point) {
        points.push(point);
    }
}

fn support_plane_grid_reference(
    old_ref: &Point3,
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<(Point3, Vec<i32>)>> {
    let values = support_plane_grid_axis_values(bounds, polygons.len() + 1)?;
    for x in &values[0] {
        for y in &values[1] {
            for z in &values[2] {
                let target = Point3::new(x.clone(), y.clone(), z.clone());
                if point_lies_on_any_support_plane(&target, polygons)? {
                    continue;
                }
                if point_lies_on_local_surface(&target, polygons)? {
                    continue;
                }
                if let Ok(winding) =
                    crate::segment_trace::trace_segment(old_ref, &target, old_wnv, polygons)
                {
                    return Ok(Some((target, winding)));
                }
            }
        }
    }
    Ok(None)
}

fn support_plane_grid_axis_values(bounds: &Aabb, slots: usize) -> HypermeshResult<[Vec<Real>; 3]> {
    Ok([
        support_plane_grid_values_for_axis(bounds, 0, slots)?,
        support_plane_grid_values_for_axis(bounds, 1, slots)?,
        support_plane_grid_values_for_axis(bounds, 2, slots)?,
    ])
}

fn support_plane_grid_values_for_axis(
    bounds: &Aabb,
    axis: usize,
    slots: usize,
) -> HypermeshResult<Vec<Real>> {
    let min = axis_ref(&bounds.min, axis);
    let max = axis_ref(&bounds.max, axis);
    let extent = max - min;
    if compare_real(&extent, &Real::zero())?.is_eq() {
        return Ok(vec![min.clone()]);
    }

    let denominator = Real::from((slots + 1) as u64);
    let mut values = Vec::with_capacity(slots);
    for index in 1..=slots {
        values.push(
            min + &((extent.clone() * Real::from(index as u64)) / denominator.clone())
                .map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        );
    }
    Ok(values)
}

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

fn escaped_reference_axis_value(
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
    Ok(Some(
        ((start_value + &stop_value) / Real::from(2))
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
    ))
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

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(r(x), r(y), r(z))
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
    fn support_plane_grid_finds_target_when_midpoint_is_blocked() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let polygons = vec![
            support_only_polygon(Plane::axis_aligned(0, r(5))),
            support_only_polygon(Plane::axis_aligned(1, r(5))),
            support_only_polygon(Plane::axis_aligned(2, r(5))),
            support_only_polygon(Plane::from_coefficients(r(1), r(1), r(1), r(-15))),
        ];

        assert!(point_lies_on_any_support_plane(&p(5, 5, 5), &polygons).unwrap());

        let values = support_plane_grid_axis_values(&bounds, polygons.len() + 1).unwrap();
        let mut target = None;
        'outer: for x in &values[0] {
            for y in &values[1] {
                for z in &values[2] {
                    let candidate = Point3::new(x.clone(), y.clone(), z.clone());
                    if !point_lies_on_any_support_plane(&candidate, &polygons).unwrap() {
                        target = Some(candidate);
                        break 'outer;
                    }
                }
            }
        }
        let target = target.expect("grid should contain a point off every finite support plane");

        assert!(point_strictly_inside_bounds(&target, &bounds).unwrap());
        assert!(!point_lies_on_any_support_plane(&target, &polygons).unwrap());
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
