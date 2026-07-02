//! Leaf processing for the subdivision pipeline.

use std::collections::BTreeMap;

use crate::bvh::ExactBvh;
use crate::clip::{ClipSide, clip_polygon};
use crate::error::HypermeshResult;
use crate::geometry::{Aabb, axis_mut, axis_ref, compare_real};
use crate::intersection::{PairwiseIntersection, PairwiseIntersectionType, intersect_polygons};
use crate::local_bsp::LocalBsp;
use crate::output::ClassifiedPolygon;
use crate::polygon::ConvexPolygon;
use crate::segment_trace::classify_leaf_polygon;
use crate::winding::{
    Indicator, WindingPair, can_early_terminate, classify_polygon_output, propagate_wnv,
};
use hyperlattice::{Point3, Real};

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
    /// Enable WNV reachability early-out.
    pub use_early_termination: bool,
}

impl Default for SubdivisionConfig {
    fn default() -> Self {
        Self {
            leaf_threshold: DEFAULT_LEAF_THRESHOLD,
            max_depth: DEFAULT_MAX_DEPTH,
            use_early_termination: true,
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
    /// Number of BSP fragments emitted.
    pub bsp_fragment_count: usize,
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
        return Ok(stats);
    }

    if can_skip_bsp_for_leaf(polygons) {
        emit_direct(polygons, bounds, ref_point, ref_wnv, indicator, output)?;
        return Ok(stats);
    }

    let intersections = pairwise_intersections_by_polygon(polygons)?;
    stats.intersection_count = intersections.iter().map(Vec::len).sum();

    for (index, polygon) in polygons.iter().enumerate() {
        if intersections[index].is_empty() {
            emit_one_direct(
                polygon, bounds, ref_point, ref_wnv, polygons, indicator, output,
            )?;
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
            let w_front = classify_leaf_polygon(
                &polygon.support,
                &leaf.edges,
                ref_point,
                ref_wnv,
                polygons,
                bounds,
                &polygon.delta_w,
            )?;
            let w_back = propagate_wnv(&w_front, 1, &polygon.delta_w);
            let classification = classify_polygon_output(&w_front, &w_back, indicator);
            if classification != 0 {
                let mut fragment = polygon.clone();
                fragment.edges = leaf.edges.clone();
                let mut classified = ClassifiedPolygon::new(fragment, classification);
                classified.winding = Some(WindingPair { w_front, w_back });
                classified.is_bsp_fragment = true;
                output.push(classified);
                stats.bsp_fragment_count += 1;
            }
        }
    }

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

    if config.use_early_termination {
        let available_wntvs = unique_wntvs(&task.polygons);
        if can_early_terminate(&task.ref_wnv, &available_wntvs, indicator) {
            return Ok(());
        }
    }

    if task.polygons.len() <= config.leaf_threshold
        || task.depth >= config.max_depth
        || !can_split_bounds(&task.bounds)?
    {
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

fn can_skip_bsp_for_leaf(polygons: &[ConvexPolygon]) -> bool {
    polygons
        .first()
        .map(|first| {
            polygons
                .iter()
                .all(|polygon| polygon.no_self_intersections && polygon.delta_w == first.delta_w)
        })
        .unwrap_or(true)
}

fn emit_direct(
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    ref_point: &Point3,
    ref_wnv: &[i32],
    indicator: &Indicator,
    output: &mut Vec<ClassifiedPolygon>,
) -> HypermeshResult<()> {
    let all_nnc = polygons.iter().all(|polygon| polygon.no_nested_components);
    if all_nnc {
        let first = &polygons[0];
        let w_front = classify_leaf_polygon(
            &first.support,
            &first.edges,
            ref_point,
            ref_wnv,
            polygons,
            bounds,
            &first.delta_w,
        )?;
        let w_back = propagate_wnv(&w_front, 1, &first.delta_w);
        let classification = classify_polygon_output(&w_front, &w_back, indicator);
        if classification != 0 {
            for polygon in polygons {
                let mut classified = ClassifiedPolygon::new(polygon.clone(), classification);
                classified.winding = Some(WindingPair {
                    w_front: w_front.clone(),
                    w_back: w_back.clone(),
                });
                output.push(classified);
            }
        }
        return Ok(());
    }

    for polygon in polygons {
        emit_one_direct(
            polygon, bounds, ref_point, ref_wnv, polygons, indicator, output,
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
) -> HypermeshResult<()> {
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
    }
    Ok(())
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
    let two = Real::from(2);
    for axis in 0..3 {
        if compare_real(&bounds.extent(axis), &two)?.is_gt() {
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
    if bounds.contains_point(old_ref)? {
        return Ok((old_ref.clone(), old_wnv.to_vec()));
    }

    let mut target = old_ref.clone();
    let three = Real::from(3);
    for axis in 0..3 {
        let min = axis_ref(&bounds.min, axis);
        let max = axis_ref(&bounds.max, axis);
        let extent = max - min;
        if compare_real(axis_ref(&target, axis), axis_ref(&bounds.min, axis))?.is_lt() {
            *axis_mut(&mut target, axis) = if extent.definitely_zero() {
                min.clone()
            } else {
                min + &(extent / three.clone()).expect("division by literal three is valid")
            };
        } else if compare_real(axis_ref(&target, axis), axis_ref(&bounds.max, axis))?.is_gt() {
            *axis_mut(&mut target, axis) = if extent.definitely_zero() {
                max.clone()
            } else {
                max - &(extent / three.clone()).expect("division by literal three is valid")
            };
        }
    }

    let winding = crate::segment_trace::trace_segment(old_ref, &target, old_wnv, polygons)?;
    Ok((target, winding))
}

fn unique_wntvs(polygons: &[ConvexPolygon]) -> Vec<Vec<i32>> {
    let mut result = Vec::new();
    for polygon in polygons {
        if !result.iter().any(|existing| existing == &polygon.delta_w) {
            result.push(polygon.delta_w.clone());
        }
    }
    result
}
