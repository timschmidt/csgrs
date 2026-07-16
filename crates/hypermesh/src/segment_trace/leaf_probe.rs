//! Leaf classification and certified adjacent-probe search.

use super::probe_cache::{
    DirectProbeReachabilityCacheEntry, HalfspaceReportCacheEntry, HalfspaceSeedFamilyCacheEntry,
    SurfaceCacheEntry, cached_direct_probe_reachability_with,
    cached_halfspace_cell_seed_families_from_optional_report_with,
    cached_optional_halfspace_feasibility_report_with, cached_surface_query_with,
};
use super::{
    AxisOrderedSegmentTraceCacheEntry, AxisProbeStopCacheEntry,
    DefinitionCycleGuardReachabilityCache, DefinitionNoDetourReachabilityCache,
    DefinitionNoDetourTraceCacheEntry, DefinitionNoPlaneReplacementCycleGuardCache,
    DefinitionNoPlaneReplacementReachabilityCache, DetourTargetFamilyCache,
    InteriorBoxAxisIntervalsCache, InteriorLeafPoint, LeafProbeQueryCaches,
    NormalProbeStopCacheEntry, PlaneReplacementAffineCache,
    PlaneReplacementNoNestedOrderingWarmupCache, PlaneReplacementReachabilityPathCache,
    PlaneReplacementReachabilityStepCache, PlaneReplacementStepCacheEntry, ProbePoint,
    ProbeReachabilityCacheEntry, ProbeWindingCacheEntry, active_planes_from_optional_report,
    adjacent_axis_probe_stop_values_with_queries, adjacent_normal_probe_stop_values_with_queries,
    apply_winding_transition_in_place, axis_plane_defined_point, axis_plane_definition,
    axis_probe_bounds, axis_probe_definition_preserves_axis_direction, axis_value_after_start,
    bounds_between_points, build_axis_probe_point, build_axis_probe_point_from_shifted_witness,
    build_probe_point, build_probe_point_from_shifted_witness,
    cached_definition_no_detour_reachability_with, classify_point_in_polygon,
    collect_strict_halfspace_seed_family, dedupe_shifted_halfspace_seed_families,
    definition_families_match_as_sets, definition_planes_match_as_sets, dot_direction,
    endpoint_definition_family, extend_strict_halfspace_seed_families_collect_unknown,
    interior_leaf_points, normal_probe_extra_planes, normal_probe_shifted_seed_families,
    normal_stop_halfspace, offset_point,
    plane_replacement_path_reaches_adjacent_cell_without_step_detours_with_caches,
    point_lies_on_traced_surface, point_strictly_inside_halfspace_cell_or_unknown, probe_axes,
    probe_reaches_adjacent_cell, probe_reaches_adjacent_cell_from_interior_with_caches,
    probe_reaches_adjacent_cell_with_definition_search, push_plane_equality_halfspaces,
    seed_family_search_failed_without_any_seed, segment_plane_crossing,
    shifted_halfspace_cell_witnesses_from_seed, shifted_halfspace_seed_families_with_report_seed,
    strict_axis_probe_targets, strict_normal_probe_targets_with_query_caches,
    take_new_halfspace_seed_family, trace_axis_segment_ignoring_mesh, trace_bounds_including_point,
    trace_segment_from_definitions_with_step_detoured_plane_replacement_with_caches,
    unique_definition_family, unique_normal_probe_search_definitions,
};
#[cfg(test)]
use super::{AxisProbeFamilyCacheEntry, NormalProbeFamilyCacheEntry, ProbePointFamilyCacheEntry};
use crate::clip::clip_polygon_to_aabb;
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Aabb, Classification, Plane, axis_ref, classify_point, compare_real};
use crate::halfspace::{aabb_core_halfspaces, support_side_halfspace};
use crate::polygon::ConvexPolygon;
use crate::winding::{WindingNumberTransitionVector, WindingNumberVector};
use hyperlattice::{Point3, Real};
use hyperlimit::HalfspaceFeasibility;
use std::sync::Arc;

/// Classifies a leaf polygon by tracing from a reference point to an off-face
/// probe and applying the host transition correction.
pub fn classify_leaf_polygon(
    support: &Plane,
    leaf_edges: &[Plane],
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
) -> HypermeshResult<WindingNumberVector> {
    let mut probe_query_caches = LeafProbeQueryCaches::default();
    classify_leaf_polygon_with_probe_query_caches(
        support,
        leaf_edges,
        ref_point,
        ref_definitions,
        ref_wnv,
        polygons,
        bounds,
        host_delta_w,
        &mut probe_query_caches,
    )
}

pub(crate) fn classify_leaf_polygon_with_probe_query_caches(
    support: &Plane,
    leaf_edges: &[Plane],
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
) -> HypermeshResult<WindingNumberVector> {
    let leaf = ConvexPolygon {
        support: support.clone(),
        edges: Arc::new(leaf_edges.to_vec()),
        mesh_index: -1,
        polygon_index: -1,
        delta_w: WindingNumberTransitionVector::new(),
        approx_bounds: None,
        known_vertices: None,
    };
    let clipped_leaf = clip_polygon_to_aabb(&leaf, bounds)?;
    let interior_points = interior_leaf_points(&clipped_leaf)?;
    classify_leaf_polygon_from_interior_points_with_probe_query_caches(
        &interior_points,
        support,
        ref_point,
        ref_definitions,
        ref_wnv,
        polygons,
        bounds,
        host_delta_w,
        probe_query_caches,
    )
}

#[cfg(test)]
pub(crate) fn classify_leaf_polygon_from_interior_points(
    interior_points: &[InteriorLeafPoint],
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
) -> HypermeshResult<WindingNumberVector> {
    let mut probe_query_caches = LeafProbeQueryCaches::default();
    classify_leaf_polygon_from_interior_points_with_probe_query_caches(
        interior_points,
        support,
        ref_point,
        ref_definitions,
        ref_wnv,
        polygons,
        bounds,
        host_delta_w,
        &mut probe_query_caches,
    )
}

pub(crate) fn classify_leaf_polygon_from_interior_points_with_probe_query_caches(
    interior_points: &[InteriorLeafPoint],
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
) -> HypermeshResult<WindingNumberVector> {
    probe_query_caches.prepare_for_trace_bounds(bounds);
    let mut saw_unknown = false;

    for point in ordered_interior_points_for_probe_search_with_support(interior_points, support)? {
        if let Some(winding) = classify_leaf_polygon_interior_point_with_probe_query_caches(
            point,
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            bounds,
            host_delta_w,
            probe_query_caches,
            &mut saw_unknown,
            None,
            &[],
        )? {
            return Ok(winding);
        }
    }

    let _ = saw_unknown;
    Err(HypermeshError::UnknownClassification)
}

#[cfg(test)]
pub(crate) fn ordered_interior_points_for_probe_search(
    interior_points: &[InteriorLeafPoint],
) -> Vec<&InteriorLeafPoint> {
    let mut ordered = interior_points.iter().enumerate().collect::<Vec<_>>();
    ordered.sort_by_key(|(index, point)| {
        (
            std::cmp::Reverse(max_axis_aligned_planes_in_definition_family(point)),
            *index,
        )
    });
    ordered.into_iter().map(|(_, point)| point).collect()
}

fn max_axis_aligned_planes_in_definition_family(point: &InteriorLeafPoint) -> usize {
    point
        .planes
        .iter()
        .map(|definition| {
            definition
                .iter()
                .filter(|plane| plane.axis_split_value().is_some())
                .count()
        })
        .max()
        .unwrap_or(0)
}

pub(crate) fn classify_leaf_polygon_interior_point_with_probe_query_caches(
    point: &InteriorLeafPoint,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
    certified_convex_host_mesh: Option<usize>,
    certified_convex_inputs: &[bool],
) -> HypermeshResult<Option<WindingNumberVector>> {
    probe_query_caches.prepare_for_trace_bounds(bounds);
    if let Some(host_mesh) = certified_convex_host_mesh
        && ref_wnv.iter().all(|winding| *winding == 0)
        && let Some(winding) = classify_point_against_certified_convex_inputs_with_cache(
            &point.point,
            ref_wnv.len(),
            polygons,
            host_mesh,
            certified_convex_inputs,
            probe_query_caches,
        )?
    {
        return Ok(Some(winding));
    }
    for positive_side in [true, false] {
        if let Some(winding) = search_adjacent_normal_probe_winding_with_queries(
            point,
            positive_side,
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            bounds,
            host_delta_w,
            probe_query_caches,
            saw_unknown,
            certified_convex_host_mesh,
        )? {
            return Ok(Some(winding));
        }

        for axis in probe_axes(support)? {
            let normal_sign = crate::geometry::classify_real(axis_ref(&support.normal, axis))?;
            if normal_sign == Classification::On {
                continue;
            }

            let direction_positive = (normal_sign == Classification::Positive) == positive_side;
            let axis_value = axis_ref(&point.point, axis);
            let room = if direction_positive {
                axis_ref(&bounds.max, axis) - axis_value
            } else {
                axis_value - axis_ref(&bounds.min, axis)
            };
            if !compare_real(&room, &Real::zero())?.is_gt() {
                continue;
            }

            if let Some(winding) = search_adjacent_axis_probe_winding_with_queries(
                point,
                positive_side,
                axis,
                direction_positive,
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                probe_query_caches,
                saw_unknown,
                bounds,
                host_delta_w,
            )? {
                return Ok(Some(winding));
            }
        }
    }

    Ok(None)
}

pub(crate) fn ordered_interior_points_for_probe_search_with_support<'a>(
    interior_points: &'a [InteriorLeafPoint],
    support: &Plane,
) -> HypermeshResult<Vec<&'a InteriorLeafPoint>> {
    let mut scored = interior_points
        .iter()
        .enumerate()
        .map(|(index, point)| {
            Ok((
                index,
                point,
                unique_normal_probe_search_definitions(&point.planes, support)?.len(),
            ))
        })
        .collect::<HypermeshResult<Vec<_>>>()?;
    scored.sort_by_key(|(index, point, retained_definition_count)| {
        (
            std::cmp::Reverse(*retained_definition_count),
            std::cmp::Reverse(max_axis_aligned_planes_in_definition_family(point)),
            *index,
        )
    });
    Ok(scored.into_iter().map(|(_, point, _)| point).collect())
}

fn search_adjacent_normal_probe_winding_with_queries(
    point: &InteriorLeafPoint,
    positive_side: bool,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
    certified_convex_host_mesh: Option<usize>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let retained_definitions = unique_normal_probe_search_definitions(&point.planes, support)?;
    let direction = if positive_side {
        support.normal.clone()
    } else {
        Point3::new(
            -support.normal.x.clone(),
            -support.normal.y.clone(),
            -support.normal.z.clone(),
        )
    };
    let (stop_values, local_unknown) = cached_adjacent_normal_probe_stop_values_with(
        &mut probe_query_caches.normal_probe_stop_values,
        &point.point,
        &direction,
        support,
        bounds,
        || {
            adjacent_normal_probe_stop_values_with_queries(
                &point.point,
                &direction,
                support,
                bounds,
                polygons,
                &mut |_interior, direction, polygon| {
                    Ok(dot_direction(&polygon.support.normal, direction))
                },
                &mut |candidate, polygon| classify_point_in_polygon(candidate, polygon),
            )
        },
    )?;
    *saw_unknown |= local_unknown;

    for stop_t in stop_values {
        if !compare_real(&stop_t, &Real::zero())?.is_gt() {
            continue;
        }
        let stop_point = offset_point(&point.point, &direction, &stop_t);
        let corridor = bounds_between_points(&point.point, &stop_point)?;
        let half =
            (Real::one() / Real::from(2)).map_err(|_| HypermeshError::UnknownClassification)?;
        let direct_point = offset_point(&point.point, &direction, &(stop_t.clone() * half));
        let direct_side = classify_point(&direct_point, support)?;
        if direct_side != Classification::On {
            let direct_probe = ProbePoint {
                planes: vec![axis_plane_definition(&direct_point)],
                point: direct_point,
                side: direct_side,
                uncertified_definition_fallback: false,
            };
            if !local_unknown {
                if let Some(host_mesh) = certified_convex_host_mesh
                    && ref_wnv.iter().all(|winding| *winding == 0)
                    && let Some(winding) = trace_certified_simple_outward_host_probe_winding(
                        &direct_probe,
                        bounds,
                        ref_wnv.len(),
                        polygons,
                        host_mesh,
                    )?
                {
                    return Ok(Some(winding));
                }
                match trace_certified_adjacent_probe_winding(
                    &direct_probe,
                    ref_point,
                    ref_definitions,
                    ref_wnv,
                    polygons,
                    host_delta_w,
                    probe_query_caches,
                ) {
                    Ok(winding) => return Ok(Some(winding)),
                    Err(HypermeshError::UnknownClassification) => {
                        *saw_unknown = true;
                    }
                    Err(err) => return Err(err),
                }
            }
            if let Some(winding) = try_leaf_probe_family_with_queries(
                point,
                positive_side,
                Ok(vec![direct_probe]),
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                host_delta_w,
                probe_query_caches,
                saw_unknown,
            )? {
                return Ok(Some(winding));
            }
        }

        for definition in &retained_definitions {
            if let Some(winding) = try_strict_normal_probe_report_witness_winding_with_queries(
                point,
                positive_side,
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                host_delta_w,
                probe_query_caches,
                saw_unknown,
                &corridor,
                Some(definition),
                &stop_point,
            )? {
                return Ok(Some(winding));
            }
            if let Some(winding) = try_strict_normal_seed_winding_with_queries(
                point,
                positive_side,
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                host_delta_w,
                probe_query_caches,
                saw_unknown,
                &corridor,
                Some(definition),
                &stop_point,
            )? {
                return Ok(Some(winding));
            }
            let probes = strict_normal_probe_targets_with_query_caches(
                point,
                support,
                &corridor,
                Some(definition),
                &stop_point,
                positive_side,
                probe_query_caches,
            );
            if let Some(winding) = try_leaf_probe_family_with_queries(
                point,
                positive_side,
                probes,
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                host_delta_w,
                probe_query_caches,
                saw_unknown,
            )? {
                return Ok(Some(winding));
            }
        }

        let unrestricted_report = try_strict_normal_probe_report_witness_winding_with_queries(
            point,
            positive_side,
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            host_delta_w,
            probe_query_caches,
            saw_unknown,
            &corridor,
            None,
            &stop_point,
        )?;
        if let Some(winding) = unrestricted_report {
            return Ok(Some(winding));
        }
        let unrestricted_progressive =
            try_strict_normal_probe_targets_progressively_with_query_caches(
                point,
                positive_side,
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                host_delta_w,
                probe_query_caches,
                saw_unknown,
                &corridor,
                None,
                &stop_point,
            )?;
        if let Some(winding) = unrestricted_progressive {
            return Ok(Some(winding));
        }
        let probes = strict_normal_probe_targets_with_query_caches(
            point,
            support,
            &corridor,
            None,
            &stop_point,
            positive_side,
            probe_query_caches,
        );
        if let Some(winding) = try_leaf_probe_family_with_queries(
            point,
            positive_side,
            probes,
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            host_delta_w,
            probe_query_caches,
            saw_unknown,
        )? {
            return Ok(Some(winding));
        }
    }

    Ok(None)
}

/// Classifies an adjacent probe against every input except a certified host.
///
/// A certified simple outward shell has winding zero on the front of each of
/// its faces. Starting outside the root bounds therefore lets an exact
/// axis-aligned trace recover every non-host component directly; the host
/// component remains zero without tracing the host's own triangulation.
fn trace_certified_simple_outward_host_probe_winding(
    probe: &ProbePoint,
    bounds: &Aabb,
    winding_len: usize,
    polygons: &[ConvexPolygon],
    host_mesh: usize,
) -> HypermeshResult<Option<WindingNumberVector>> {
    if host_mesh >= winding_len {
        return Err(HypermeshError::UnknownClassification);
    }
    let host_mesh =
        isize::try_from(host_mesh).map_err(|_| HypermeshError::UnknownClassification)?;
    let one = Real::one();
    let zero_winding = vec![0; winding_len];

    for axis in 0..3 {
        for start_from_min in [true, false] {
            let mut start = probe.point.clone();
            *crate::geometry::axis_mut(&mut start, axis) = if start_from_min {
                axis_ref(&bounds.min, axis) - one.clone()
            } else {
                axis_ref(&bounds.max, axis) + one.clone()
            };
            match trace_axis_segment_ignoring_mesh(
                &start,
                &probe.point,
                axis,
                &zero_winding,
                polygons,
                Some(host_mesh),
            ) {
                Ok(trace) if trace.valid => return Ok(Some(trace.winding)),
                Ok(_) | Err(HypermeshError::UnknownClassification) => {}
                Err(err) => return Err(err),
            }
        }
    }

    Ok(None)
}

fn classify_point_against_certified_convex_inputs_with_cache(
    point: &Point3,
    winding_len: usize,
    polygons: &[ConvexPolygon],
    host_mesh: usize,
    certified_convex_inputs: &[bool],
    probe_query_caches: &mut LeafProbeQueryCaches,
) -> HypermeshResult<Option<WindingNumberVector>> {
    if probe_query_caches.certified_convex_mesh_supports.is_none() {
        let mut supports = vec![Vec::new(); winding_len];
        for polygon in polygons {
            if let Ok(mesh) = usize::try_from(polygon.mesh_index)
                && let Some(mesh_supports) = supports.get_mut(mesh)
            {
                mesh_supports.push(polygon.support.clone());
            }
        }
        probe_query_caches.certified_convex_mesh_supports = Some(supports);
        probe_query_caches.certified_convex_last_outside_support = vec![None; winding_len];
    }
    let supports = probe_query_caches
        .certified_convex_mesh_supports
        .as_ref()
        .ok_or(HypermeshError::UnknownClassification)?;
    classify_point_against_certified_convex_inputs(
        point,
        winding_len,
        isize::try_from(host_mesh).map_err(|_| HypermeshError::UnknownClassification)?,
        certified_convex_inputs,
        supports,
        &mut probe_query_caches.certified_convex_last_outside_support,
    )
}

fn classify_point_against_certified_convex_inputs(
    point: &Point3,
    winding_len: usize,
    host_mesh: isize,
    certified_convex_inputs: &[bool],
    supports: &[Vec<Plane>],
    last_outside_support: &mut [Option<usize>],
) -> HypermeshResult<Option<WindingNumberVector>> {
    if certified_convex_inputs.len() != winding_len
        || supports.len() != winding_len
        || last_outside_support.len() != winding_len
    {
        return Ok(None);
    }
    let mut winding = vec![0; winding_len];

    for mesh in 0..winding_len {
        let mesh_index =
            isize::try_from(mesh).map_err(|_| HypermeshError::UnknownClassification)?;
        if mesh_index == host_mesh {
            continue;
        }
        if !certified_convex_inputs[mesh] {
            return Ok(None);
        }

        let mesh_supports = &supports[mesh];
        if mesh_supports.is_empty() {
            return Err(HypermeshError::UnknownClassification);
        }
        let mut on_boundary = false;
        let mut outside = false;
        if let Some(index) = last_outside_support[mesh]
            && let Some(support) = mesh_supports.get(index)
        {
            match classify_point(point, support)? {
                Classification::Positive => outside = true,
                Classification::On => on_boundary = true,
                Classification::Negative => {}
            }
        }
        if outside {
            continue;
        }
        for (index, support) in mesh_supports.iter().enumerate() {
            if last_outside_support[mesh] == Some(index) {
                continue;
            }
            match classify_point(point, support)? {
                Classification::Positive => {
                    outside = true;
                    last_outside_support[mesh] = Some(index);
                    break;
                }
                Classification::On => on_boundary = true,
                Classification::Negative => {}
            }
        }
        if !outside {
            if on_boundary {
                return Ok(None);
            }
            winding[mesh] = 1;
        }
    }

    Ok(Some(winding))
}

fn trace_certified_adjacent_probe_winding(
    probe: &ProbePoint,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
) -> HypermeshResult<WindingNumberVector> {
    let LeafProbeQueryCaches {
        trace_bounds,
        probe_winding,
        probe_surface,
        axis_ordered_segment_traces,
        plane_replacement_affine,
        plane_replacement_trace_steps,
        definition_no_detour_trace,
        detour_target_families,
        ..
    } = probe_query_caches;
    let trace_bounds = trace_bounds
        .as_ref()
        .ok_or(HypermeshError::UnknownClassification)?;
    let winding_trace_bounds = trace_bounds_including_point(trace_bounds, ref_point)?;
    let mut winding = cached_probe_winding_with(probe_winding, probe, || {
        trace_probe_winding_with_caches(
            ref_point,
            ref_definitions,
            probe,
            ref_wnv,
            polygons,
            probe_surface,
            axis_ordered_segment_traces,
            plane_replacement_affine,
            plane_replacement_trace_steps,
            definition_no_detour_trace,
            detour_target_families,
            Some(&winding_trace_bounds),
        )
    })?;
    if probe.side == Classification::Negative {
        apply_winding_transition_in_place(&mut winding, -1, host_delta_w)?;
    }
    Ok(winding)
}

fn try_strict_normal_probe_targets_progressively_with_query_caches(
    point: &InteriorLeafPoint,
    positive_side: bool,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
    corridor: &Aabb,
    definition: Option<&[Plane; 3]>,
    stop_point: &Point3,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));
    halfspaces.push(normal_stop_halfspace(support, stop_point, positive_side));

    let mut local_unknown = false;
    let report = cached_optional_halfspace_feasibility_report_with(
        &mut probe_query_caches.halfspace_reports,
        &halfspaces,
        &mut local_unknown,
    )?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        *saw_unknown |= local_unknown;
        return Ok(None);
    }

    let extra_planes = normal_probe_extra_planes(point, definition);
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        cached_halfspace_cell_seed_families_from_optional_report_with(
            &mut probe_query_caches.halfspace_seed_families,
            corridor,
            &halfspaces,
            report.as_ref(),
            &mut local_unknown,
        )?;
    let mut seen_direct_seeds = Vec::new();
    let mut seeds = take_new_halfspace_seed_family(seeds, &mut seen_direct_seeds);
    let mut certified_probe_points = Vec::new();
    let mut prioritized_probes: Vec<ProbePoint> = Vec::new();
    let mut deferred_probes: Vec<ProbePoint> = Vec::new();
    let mut saw_any_probe = false;
    let mut queue_probe = |probe: ProbePoint,
                           local_unknown: &mut bool|
     -> HypermeshResult<Option<WindingNumberVector>> {
        saw_any_probe = true;
        let probe_fallback = probe.uncertified_definition_fallback;
        let LeafProbeQueryCaches {
            trace_bounds,
            probe_winding,
            probe_surface,
            probe_reachability,
            axis_ordered_segment_traces,
            plane_replacement_affine,
            plane_replacement_trace_steps,
            plane_replacement_reachability_paths,
            plane_replacement_reachability_steps,
            plane_replacement_no_nested_ordering_warmups,
            interior_box_axis_intervals,
            definition_cycle_guard_reachability,
            definition_no_step_detour_reachability,
            definition_no_plane_replacement_cycle_guard,
            definition_no_plane_replacement_reachability,
            halfspace_reports,
            halfspace_seed_families,
            no_step_detour_target_families,
            definition_full_no_detour_reachability,
            definition_no_detour_trace,
            definition_no_detour_reachability,
            direct_probe_reachability,
            detour_target_families,
            ..
        } = probe_query_caches;
        let trace_bounds = trace_bounds
            .as_ref()
            .ok_or(HypermeshError::UnknownClassification)?;

        if cached_surface_query_with(probe_surface, &probe.point, || {
            point_lies_on_traced_surface(&probe.point, polygons)
        })? {
            if point.uncertified_definition_fallback || probe_fallback {
                *local_unknown = true;
            }
            return Ok(None);
        }

        let no_step_result =
            probe_reaches_adjacent_cell_from_interior_without_step_detours_with_caches(
                point,
                &probe,
                support,
                polygons,
                plane_replacement_affine,
                plane_replacement_reachability_paths,
                plane_replacement_reachability_steps,
                definition_no_step_detour_reachability,
                direct_probe_reachability,
                trace_bounds,
            );
        match no_step_result {
            Ok(true) => {
                for deferred in deferred_probes.drain(..) {
                    if let Some(winding) = evaluate_leaf_probe_with_query_caches(
                        point,
                        positive_side,
                        deferred,
                        support,
                        ref_point,
                        ref_definitions,
                        ref_wnv,
                        polygons,
                        host_delta_w,
                        probe_surface,
                        probe_reachability,
                        probe_winding,
                        axis_ordered_segment_traces,
                        plane_replacement_affine,
                        plane_replacement_trace_steps,
                        plane_replacement_reachability_paths,
                        plane_replacement_reachability_steps,
                        plane_replacement_no_nested_ordering_warmups,
                        interior_box_axis_intervals,
                        definition_cycle_guard_reachability,
                        definition_no_step_detour_reachability,
                        definition_no_plane_replacement_cycle_guard,
                        definition_no_plane_replacement_reachability,
                        halfspace_reports,
                        halfspace_seed_families,
                        no_step_detour_target_families,
                        definition_full_no_detour_reachability,
                        definition_no_detour_trace,
                        definition_no_detour_reachability,
                        direct_probe_reachability,
                        detour_target_families,
                        trace_bounds,
                        saw_unknown,
                    )? {
                        return Ok(Some(winding));
                    }
                }
                if prioritized_probes.is_empty() {
                    if let Some(winding) = evaluate_leaf_probe_with_query_caches(
                        point,
                        positive_side,
                        probe,
                        support,
                        ref_point,
                        ref_definitions,
                        ref_wnv,
                        polygons,
                        host_delta_w,
                        probe_surface,
                        probe_reachability,
                        probe_winding,
                        axis_ordered_segment_traces,
                        plane_replacement_affine,
                        plane_replacement_trace_steps,
                        plane_replacement_reachability_paths,
                        plane_replacement_reachability_steps,
                        plane_replacement_no_nested_ordering_warmups,
                        interior_box_axis_intervals,
                        definition_cycle_guard_reachability,
                        definition_no_step_detour_reachability,
                        definition_no_plane_replacement_cycle_guard,
                        definition_no_plane_replacement_reachability,
                        halfspace_reports,
                        halfspace_seed_families,
                        no_step_detour_target_families,
                        definition_full_no_detour_reachability,
                        definition_no_detour_trace,
                        definition_no_detour_reachability,
                        direct_probe_reachability,
                        detour_target_families,
                        trace_bounds,
                        saw_unknown,
                    )? {
                        return Ok(Some(winding));
                    }
                } else {
                    prioritized_probes.push(probe);
                }
            }
            Ok(false) => deferred_probes.push(probe),
            Err(HypermeshError::UnknownClassification) => {
                *local_unknown = true;
                deferred_probes.push(probe);
            }
            Err(err) => return Err(err),
        }
        Ok(None)
    };

    for witness in &seeds {
        let probe = match build_probe_point(
            witness,
            corridor,
            support,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), witness),
            &extra_planes,
            false,
        ) {
            Ok(Some(probe)) => probe,
            Ok(None) => continue,
            Err(HypermeshError::UnknownClassification) => {
                local_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if !probe.uncertified_definition_fallback && !certified_probe_points.contains(&probe.point)
        {
            certified_probe_points.push(probe.point.clone());
        }
        if let Some(winding) = queue_probe(probe, &mut local_unknown)? {
            return Ok(Some(winding));
        }
    }

    if definition.is_some() || certified_probe_points.is_empty() {
        let mut geometry_strict_seeds = Vec::new();
        local_unknown |= extend_strict_halfspace_seed_families_collect_unknown(
            &mut geometry_strict_seeds,
            [collect_strict_halfspace_seed_family(
                Ok(shifted_geometry_seeds.clone()),
                |candidate| {
                    point_strictly_inside_halfspace_cell_or_unknown(
                        candidate,
                        corridor,
                        &halfspaces,
                    )
                },
            )],
        )?;
        let mut seen_all_direct_seeds = seeds.clone();
        let geometry_strict_seeds =
            take_new_halfspace_seed_family(geometry_strict_seeds, &mut seen_all_direct_seeds);
        for witness in &geometry_strict_seeds {
            let probe = match build_probe_point(
                witness,
                corridor,
                support,
                &halfspaces,
                active_planes_from_optional_report(report.as_ref(), witness),
                &extra_planes,
                false,
            ) {
                Ok(Some(probe)) => probe,
                Ok(None) => continue,
                Err(HypermeshError::UnknownClassification) => {
                    local_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
            if !probe.uncertified_definition_fallback
                && !certified_probe_points.contains(&probe.point)
            {
                certified_probe_points.push(probe.point.clone());
            }
            if let Some(winding) = queue_probe(probe, &mut local_unknown)? {
                return Ok(Some(winding));
            }
        }
        seeds.extend(geometry_strict_seeds);
    }

    let shifted_geometry_seeds = if definition.is_some() || certified_probe_points.is_empty() {
        shifted_geometry_seeds
    } else {
        Vec::new()
    };
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);
    if seed_family_search_failed_without_any_seed(
        &seeds,
        &shifted_vertices,
        &shifted_geometry_seeds,
        local_unknown,
    ) {
        *saw_unknown |= local_unknown;
        return Ok(None);
    }
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        normal_probe_shifted_seed_families(
            definition,
            report_witness,
            &certified_probe_points,
            seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );

    let mut seen_shifted_roots = Vec::new();
    for family in [strict_shift_seeds, shifted_vertices, shifted_geometry_seeds] {
        let fresh = take_new_halfspace_seed_family(family, &mut seen_shifted_roots);
        for seed in fresh {
            let shifted_witnesses =
                match shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, &seed) {
                    Ok(witnesses) => witnesses,
                    Err(HypermeshError::UnknownClassification) => {
                        local_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
            for shifted in &shifted_witnesses {
                let probe = match build_probe_point_from_shifted_witness(
                    shifted,
                    corridor,
                    support,
                    &extra_planes,
                ) {
                    Ok(Some(probe)) => probe,
                    Ok(None) => continue,
                    Err(HypermeshError::UnknownClassification) => {
                        local_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
                if let Some(winding) = queue_probe(probe, &mut local_unknown)? {
                    return Ok(Some(winding));
                }
            }
        }
    }

    *saw_unknown |= local_unknown;
    if !saw_any_probe {
        return Ok(None);
    }
    let mut probes = prioritized_probes;
    probes.extend(deferred_probes);
    try_leaf_probe_family_with_queries(
        point,
        positive_side,
        Ok(probes),
        support,
        ref_point,
        ref_definitions,
        ref_wnv,
        polygons,
        host_delta_w,
        probe_query_caches,
        saw_unknown,
    )
}
fn try_strict_normal_probe_report_witness_winding_with_queries(
    point: &InteriorLeafPoint,
    positive_side: bool,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
    corridor: &Aabb,
    definition: Option<&[Plane; 3]>,
    stop_point: &Point3,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));
    halfspaces.push(normal_stop_halfspace(support, stop_point, positive_side));

    let report = cached_optional_halfspace_feasibility_report_with(
        &mut probe_query_caches.halfspace_reports,
        &halfspaces,
        saw_unknown,
    )?;
    let Some(report) = report.as_ref() else {
        return Ok(None);
    };
    if report.status != HalfspaceFeasibility::Feasible {
        return Ok(None);
    }
    let Some(witness) = report.witness.as_ref() else {
        return Ok(None);
    };
    let extra_planes = normal_probe_extra_planes(point, definition);
    let probe_result = match build_probe_point(
        witness,
        corridor,
        support,
        &halfspaces,
        active_planes_from_optional_report(Some(report), witness),
        &extra_planes,
        false,
    ) {
        Ok(Some(probe)) => Ok(vec![probe]),
        Ok(None) => Ok(Vec::new()),
        Err(err) => Err(err),
    };

    try_leaf_probe_family_with_queries(
        point,
        positive_side,
        probe_result,
        support,
        ref_point,
        ref_definitions,
        ref_wnv,
        polygons,
        host_delta_w,
        probe_query_caches,
        saw_unknown,
    )
}

fn try_strict_normal_seed_winding_with_queries(
    point: &InteriorLeafPoint,
    positive_side: bool,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
    corridor: &Aabb,
    definition: Option<&[Plane; 3]>,
    stop_point: &Point3,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));
    halfspaces.push(normal_stop_halfspace(support, stop_point, positive_side));

    let report = cached_optional_halfspace_feasibility_report_with(
        &mut probe_query_caches.halfspace_reports,
        &halfspaces,
        saw_unknown,
    )?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(None);
    }

    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let extra_planes = normal_probe_extra_planes(point, definition);
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        cached_halfspace_cell_seed_families_from_optional_report_with(
            &mut probe_query_caches.halfspace_seed_families,
            corridor,
            &halfspaces,
            report.as_ref(),
            saw_unknown,
        )?;
    let mut seen = Vec::new();
    let mut strict_seeds = take_new_halfspace_seed_family(strict_seeds, &mut seen);
    if let Some(report_witness) = report_witness {
        strict_seeds.retain(|seed| *seed != *report_witness);
    }
    let mut certified_probe_points = Vec::new();

    for witness in &strict_seeds {
        let probe = match build_probe_point(
            witness,
            corridor,
            support,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), witness),
            &extra_planes,
            false,
        ) {
            Ok(Some(probe)) => probe,
            Ok(None) => continue,
            Err(HypermeshError::UnknownClassification) => {
                *saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if !probe.uncertified_definition_fallback && !certified_probe_points.contains(&probe.point)
        {
            certified_probe_points.push(probe.point.clone());
        }
        if let Some(winding) = try_leaf_probe_family_with_queries(
            point,
            positive_side,
            Ok(vec![probe]),
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            host_delta_w,
            probe_query_caches,
            saw_unknown,
        )? {
            return Ok(Some(winding));
        }
    }

    let allow_geometry_seed_expansion = definition.is_some();
    if allow_geometry_seed_expansion && certified_probe_points.is_empty() {
        let mut geometry_strict_seeds = Vec::new();
        *saw_unknown |= extend_strict_halfspace_seed_families_collect_unknown(
            &mut geometry_strict_seeds,
            [collect_strict_halfspace_seed_family(
                Ok(shifted_geometry_seeds.clone()),
                |candidate| {
                    point_strictly_inside_halfspace_cell_or_unknown(
                        candidate,
                        corridor,
                        &halfspaces,
                    )
                },
            )],
        )?;
        let mut seen_all_direct_seeds = strict_seeds.clone();
        let geometry_strict_seeds =
            take_new_halfspace_seed_family(geometry_strict_seeds, &mut seen_all_direct_seeds);
        for witness in &geometry_strict_seeds {
            let probe = match build_probe_point(
                witness,
                corridor,
                support,
                &halfspaces,
                active_planes_from_optional_report(report.as_ref(), witness),
                &extra_planes,
                false,
            ) {
                Ok(Some(probe)) => probe,
                Ok(None) => continue,
                Err(HypermeshError::UnknownClassification) => {
                    *saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
            if !probe.uncertified_definition_fallback
                && !certified_probe_points.contains(&probe.point)
            {
                certified_probe_points.push(probe.point.clone());
            }
            if let Some(winding) = try_leaf_probe_family_with_queries(
                point,
                positive_side,
                Ok(vec![probe]),
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                host_delta_w,
                probe_query_caches,
                saw_unknown,
            )? {
                return Ok(Some(winding));
            }
        }
        strict_seeds.extend(geometry_strict_seeds);
    }

    let shifted_geometry_seeds = if allow_geometry_seed_expansion {
        shifted_geometry_seeds
    } else {
        Vec::new()
    };
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(
            strict_seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );
    if seed_family_search_failed_without_any_seed(
        &strict_seeds,
        &shifted_vertices,
        &shifted_geometry_seeds,
        *saw_unknown,
    ) {
        return Ok(None);
    }
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        normal_probe_shifted_seed_families(
            definition,
            report_witness,
            &certified_probe_points,
            strict_seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );
    let mut seen_shifted_roots = Vec::new();
    for family in [strict_shift_seeds, shifted_vertices, shifted_geometry_seeds] {
        let fresh = take_new_halfspace_seed_family(family, &mut seen_shifted_roots);
        for seed in fresh {
            let shifted_witnesses =
                match shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, &seed) {
                    Ok(witnesses) => witnesses,
                    Err(HypermeshError::UnknownClassification) => {
                        *saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
            for shifted in &shifted_witnesses {
                let probe = match build_probe_point_from_shifted_witness(
                    shifted,
                    corridor,
                    support,
                    &extra_planes,
                ) {
                    Ok(Some(probe)) => probe,
                    Ok(None) => continue,
                    Err(HypermeshError::UnknownClassification) => {
                        *saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
                if let Some(winding) = try_leaf_probe_family_with_queries(
                    point,
                    positive_side,
                    Ok(vec![probe]),
                    support,
                    ref_point,
                    ref_definitions,
                    ref_wnv,
                    polygons,
                    host_delta_w,
                    probe_query_caches,
                    saw_unknown,
                )? {
                    return Ok(Some(winding));
                }
            }
        }
    }

    Ok(None)
}

fn search_adjacent_axis_probe_winding_with_queries(
    point: &InteriorLeafPoint,
    positive_side: bool,
    axis: usize,
    direction_positive: bool,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
    bounds: &Aabb,
    host_delta_w: &[i32],
) -> HypermeshResult<Option<WindingNumberVector>> {
    let (stop_values, local_unknown) = cached_adjacent_axis_probe_stop_values_with(
        &mut probe_query_caches.axis_probe_stop_values,
        &point.point,
        bounds,
        axis,
        direction_positive,
        || {
            adjacent_axis_probe_stop_values_with_queries(
                &point.point,
                bounds,
                polygons,
                axis,
                direction_positive,
                &mut |interior, endpoint, polygon, _axis| {
                    let start_class = classify_point(interior, &polygon.support)?;
                    let endpoint_class = classify_point(endpoint, &polygon.support)?;
                    if start_class == Classification::On {
                        return Ok(Some(interior.clone()));
                    }
                    if endpoint_class == Classification::On {
                        return Ok(Some(endpoint.clone()));
                    }
                    segment_plane_crossing(interior, endpoint, &polygon.support)
                },
                &mut |crossing, polygon| classify_point_in_polygon(crossing, polygon),
            )
        },
    )?;
    *saw_unknown |= local_unknown;

    let definitions = unique_definition_family(&point.planes);
    let start_value = axis_ref(&point.point, axis);
    for stop_value in stop_values {
        if !axis_value_after_start(start_value, &stop_value, direction_positive)? {
            continue;
        }
        let corridor = axis_probe_bounds(&point.point, axis, &stop_value)?;

        for definition in &definitions {
            if !axis_probe_definition_preserves_axis_direction(definition, axis)? {
                continue;
            }
            if let Some(winding) = try_strict_axis_seed_winding_with_queries(
                point,
                positive_side,
                axis,
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                host_delta_w,
                probe_query_caches,
                saw_unknown,
                &corridor,
                Some(definition),
            )? {
                return Ok(Some(winding));
            }
            if let Some(winding) = try_leaf_probe_family_with_queries(
                point,
                positive_side,
                strict_axis_probe_targets(
                    point,
                    support,
                    &corridor,
                    axis,
                    direction_positive,
                    Some(definition),
                ),
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                host_delta_w,
                probe_query_caches,
                saw_unknown,
            )? {
                return Ok(Some(winding));
            }
        }

        if let Some(winding) = try_strict_axis_seed_winding_with_queries(
            point,
            positive_side,
            axis,
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            host_delta_w,
            probe_query_caches,
            saw_unknown,
            &corridor,
            None,
        )? {
            return Ok(Some(winding));
        }
        if let Some(winding) = try_leaf_probe_family_with_queries(
            point,
            positive_side,
            strict_axis_probe_targets(point, support, &corridor, axis, direction_positive, None),
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            host_delta_w,
            probe_query_caches,
            saw_unknown,
        )? {
            return Ok(Some(winding));
        }
    }

    Ok(None)
}

fn try_strict_axis_seed_winding_with_queries(
    point: &InteriorLeafPoint,
    positive_side: bool,
    axis: usize,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
    corridor: &Aabb,
    definition: Option<&[Plane; 3]>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));

    let report = cached_optional_halfspace_feasibility_report_with(
        &mut probe_query_caches.halfspace_reports,
        &halfspaces,
        saw_unknown,
    )?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(None);
    }

    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        cached_halfspace_cell_seed_families_from_optional_report_with(
            &mut probe_query_caches.halfspace_seed_families,
            corridor,
            &halfspaces,
            report.as_ref(),
            saw_unknown,
        )?;
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);

    let mut certified_probe_points = Vec::new();
    for witness in &seeds {
        let probe = match build_axis_probe_point(
            witness,
            point,
            corridor,
            support,
            axis,
            definition,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), witness),
            false,
        ) {
            Ok(Some(probe)) => probe,
            Ok(None) => continue,
            Err(HypermeshError::UnknownClassification) => {
                *saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if !probe.uncertified_definition_fallback && !certified_probe_points.contains(&probe.point)
        {
            certified_probe_points.push(probe.point.clone());
        }
        if let Some(winding) = try_leaf_probe_family_with_queries(
            point,
            positive_side,
            Ok(vec![probe]),
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            host_delta_w,
            probe_query_caches,
            saw_unknown,
        )? {
            return Ok(Some(winding));
        }
    }

    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_halfspace_seed_families_with_report_seed(
            report_witness,
            seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );

    let mut seen_shifted_roots = Vec::new();
    for family in [strict_shift_seeds, shifted_vertices, shifted_geometry_seeds] {
        let fresh = take_new_halfspace_seed_family(family, &mut seen_shifted_roots);
        for seed in fresh {
            let shifted_witnesses =
                match shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, &seed) {
                    Ok(witnesses) => witnesses,
                    Err(HypermeshError::UnknownClassification) => {
                        *saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
            for shifted in &shifted_witnesses {
                let probe = match build_axis_probe_point_from_shifted_witness(
                    shifted, point, corridor, support, axis, definition,
                ) {
                    Ok(Some(probe)) => probe,
                    Ok(None) => continue,
                    Err(HypermeshError::UnknownClassification) => {
                        *saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
                if let Some(winding) = try_leaf_probe_family_with_queries(
                    point,
                    positive_side,
                    Ok(vec![probe]),
                    support,
                    ref_point,
                    ref_definitions,
                    ref_wnv,
                    polygons,
                    host_delta_w,
                    probe_query_caches,
                    saw_unknown,
                )? {
                    return Ok(Some(winding));
                }
            }
        }
    }

    Ok(None)
}

fn try_leaf_probe_family_with_queries(
    point: &InteriorLeafPoint,
    positive_side: bool,
    probes: HypermeshResult<Vec<ProbePoint>>,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let LeafProbeQueryCaches {
        trace_bounds,
        probe_winding,
        probe_surface,
        probe_reachability,
        axis_ordered_segment_traces,
        plane_replacement_affine,
        plane_replacement_trace_steps,
        plane_replacement_reachability_paths,
        plane_replacement_reachability_steps,
        plane_replacement_no_nested_ordering_warmups,
        interior_box_axis_intervals,
        definition_cycle_guard_reachability,
        definition_no_step_detour_reachability,
        definition_no_plane_replacement_cycle_guard,
        definition_no_plane_replacement_reachability,
        halfspace_reports,
        halfspace_seed_families,
        no_step_detour_target_families,
        definition_full_no_detour_reachability,
        definition_no_detour_trace,
        definition_no_detour_reachability,
        direct_probe_reachability,
        detour_target_families,
        ..
    } = probe_query_caches;
    let trace_bounds = trace_bounds
        .as_ref()
        .ok_or(HypermeshError::UnknownClassification)?;
    let probes = match probes {
        Ok(probes) => probes,
        Err(HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
            return Ok(None);
        }
        Err(err) => return Err(err),
    };
    if probes.is_empty() && point.uncertified_definition_fallback {
        *saw_unknown = true;
    }
    let mut prioritized_probes: Vec<ProbePoint> = Vec::new();
    let mut deferred_probes: Vec<ProbePoint> = Vec::new();
    for probe in probes {
        let probe_fallback = probe.uncertified_definition_fallback;
        if cached_surface_query_with(probe_surface, &probe.point, || {
            point_lies_on_traced_surface(&probe.point, polygons)
        })? {
            if point.uncertified_definition_fallback || probe_fallback {
                *saw_unknown = true;
            }
            continue;
        }

        match probe_reaches_adjacent_cell_from_interior_without_step_detours_with_caches(
            point,
            &probe,
            support,
            polygons,
            plane_replacement_affine,
            plane_replacement_reachability_paths,
            plane_replacement_reachability_steps,
            definition_no_step_detour_reachability,
            direct_probe_reachability,
            trace_bounds,
        ) {
            Ok(true) => {
                for deferred in deferred_probes.drain(..) {
                    if let Some(winding) = evaluate_leaf_probe_with_query_caches(
                        point,
                        positive_side,
                        deferred,
                        support,
                        ref_point,
                        ref_definitions,
                        ref_wnv,
                        polygons,
                        host_delta_w,
                        probe_surface,
                        probe_reachability,
                        probe_winding,
                        axis_ordered_segment_traces,
                        plane_replacement_affine,
                        plane_replacement_trace_steps,
                        plane_replacement_reachability_paths,
                        plane_replacement_reachability_steps,
                        plane_replacement_no_nested_ordering_warmups,
                        interior_box_axis_intervals,
                        definition_cycle_guard_reachability,
                        definition_no_step_detour_reachability,
                        definition_no_plane_replacement_cycle_guard,
                        definition_no_plane_replacement_reachability,
                        halfspace_reports,
                        halfspace_seed_families,
                        no_step_detour_target_families,
                        definition_full_no_detour_reachability,
                        definition_no_detour_trace,
                        definition_no_detour_reachability,
                        direct_probe_reachability,
                        detour_target_families,
                        trace_bounds,
                        saw_unknown,
                    )? {
                        return Ok(Some(winding));
                    }
                }
                prioritized_probes.push(probe);
            }
            Ok(false) => deferred_probes.push(probe),
            Err(HypermeshError::UnknownClassification) => {
                *saw_unknown = true;
                deferred_probes.push(probe);
            }
            Err(err) => return Err(err),
        }
    }

    for probe in prioritized_probes.into_iter().chain(deferred_probes) {
        if let Some(winding) = evaluate_leaf_probe_with_query_caches(
            point,
            positive_side,
            probe,
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            host_delta_w,
            probe_surface,
            probe_reachability,
            probe_winding,
            axis_ordered_segment_traces,
            plane_replacement_affine,
            plane_replacement_trace_steps,
            plane_replacement_reachability_paths,
            plane_replacement_reachability_steps,
            plane_replacement_no_nested_ordering_warmups,
            interior_box_axis_intervals,
            definition_cycle_guard_reachability,
            definition_no_step_detour_reachability,
            definition_no_plane_replacement_cycle_guard,
            definition_no_plane_replacement_reachability,
            halfspace_reports,
            halfspace_seed_families,
            no_step_detour_target_families,
            definition_full_no_detour_reachability,
            definition_no_detour_trace,
            definition_no_detour_reachability,
            direct_probe_reachability,
            detour_target_families,
            trace_bounds,
            saw_unknown,
        )? {
            return Ok(Some(winding));
        }
    }

    Ok(None)
}

fn evaluate_leaf_probe_with_query_caches(
    point: &InteriorLeafPoint,
    _positive_side: bool,
    probe: ProbePoint,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    host_delta_w: &[i32],
    probe_surface: &mut Vec<SurfaceCacheEntry>,
    probe_reachability: &mut Vec<ProbeReachabilityCacheEntry>,
    probe_winding: &mut Vec<ProbeWindingCacheEntry>,
    axis_ordered_segment_traces: &mut Vec<AxisOrderedSegmentTraceCacheEntry>,
    plane_replacement_affine: &mut PlaneReplacementAffineCache,
    plane_replacement_trace_steps: &mut Vec<PlaneReplacementStepCacheEntry>,
    plane_replacement_reachability_paths: &mut PlaneReplacementReachabilityPathCache,
    plane_replacement_reachability_steps: &mut PlaneReplacementReachabilityStepCache,
    plane_replacement_no_nested_ordering_warmups: &mut PlaneReplacementNoNestedOrderingWarmupCache,
    interior_box_axis_intervals: &mut InteriorBoxAxisIntervalsCache,
    definition_cycle_guard_reachability: &mut DefinitionCycleGuardReachabilityCache,
    definition_no_step_detour_reachability: &mut DefinitionNoDetourReachabilityCache,
    definition_no_plane_replacement_cycle_guard: &mut DefinitionNoPlaneReplacementCycleGuardCache,
    definition_no_plane_replacement_reachability:
        &mut DefinitionNoPlaneReplacementReachabilityCache,
    halfspace_reports: &mut Vec<HalfspaceReportCacheEntry>,
    halfspace_seed_families: &mut Vec<HalfspaceSeedFamilyCacheEntry>,
    no_step_detour_target_families: &mut DetourTargetFamilyCache,
    definition_full_no_detour_reachability: &mut DefinitionNoDetourReachabilityCache,
    definition_no_detour_trace: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    definition_no_detour_reachability: &mut DefinitionNoDetourReachabilityCache,
    direct_probe_reachability: &mut Vec<DirectProbeReachabilityCacheEntry>,
    detour_target_families: &mut DetourTargetFamilyCache,
    trace_bounds: &Aabb,
    saw_unknown: &mut bool,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let probe_fallback = probe.uncertified_definition_fallback;
    let winding = if cached_surface_query_with(probe_surface, &probe.point, || {
        point_lies_on_traced_surface(&probe.point, polygons)
    })? {
        None
    } else {
        let reaches = cached_probe_reachability_with(probe_reachability, point, &probe, || {
            probe_reaches_adjacent_cell_from_interior_with_caches(
                point,
                &probe,
                support,
                polygons,
                probe_surface,
                plane_replacement_affine,
                plane_replacement_reachability_paths,
                plane_replacement_reachability_steps,
                plane_replacement_no_nested_ordering_warmups,
                interior_box_axis_intervals,
                definition_cycle_guard_reachability,
                definition_no_step_detour_reachability,
                definition_no_plane_replacement_cycle_guard,
                definition_no_plane_replacement_reachability,
                halfspace_reports,
                halfspace_seed_families,
                no_step_detour_target_families,
                definition_full_no_detour_reachability,
                definition_no_detour_reachability,
                direct_probe_reachability,
                detour_target_families,
                Some(trace_bounds),
            )
        })?;
        if !reaches {
            None
        } else {
            let winding_trace_bounds = trace_bounds_including_point(trace_bounds, ref_point)?;
            let mut winding = cached_probe_winding_with(probe_winding, &probe, || {
                trace_probe_winding_with_caches(
                    ref_point,
                    ref_definitions,
                    &probe,
                    ref_wnv,
                    polygons,
                    probe_surface,
                    axis_ordered_segment_traces,
                    plane_replacement_affine,
                    plane_replacement_trace_steps,
                    definition_no_detour_trace,
                    detour_target_families,
                    Some(&winding_trace_bounds),
                )
            })?;
            if probe.side == Classification::Negative {
                apply_winding_transition_in_place(&mut winding, -1, host_delta_w)?;
            }
            Some(winding)
        }
    };

    match winding {
        Some(winding) => {
            // Strict leaf membership, adjacent-cell reachability, and the
            // reference-to-probe winding trace have all certified this pair.
            Ok(Some(winding))
        }
        None => {
            if point.uncertified_definition_fallback || probe_fallback {
                *saw_unknown = true;
            }
            Ok(None)
        }
    }
}

fn probe_reaches_adjacent_cell_from_interior_without_step_detours_with_caches(
    interior: &InteriorLeafPoint,
    probe: &ProbePoint,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    plane_replacement_affine: &mut PlaneReplacementAffineCache,
    plane_replacement_reachability_paths: &mut PlaneReplacementReachabilityPathCache,
    plane_replacement_reachability_steps: &mut PlaneReplacementReachabilityStepCache,
    no_step_cache: &mut DefinitionNoDetourReachabilityCache,
    direct_probe_reachability: &mut Vec<DirectProbeReachabilityCacheEntry>,
    trace_bounds: &Aabb,
) -> HypermeshResult<bool> {
    if !trace_bounds.contains_point(&interior.point)?
        || !trace_bounds.contains_point(&probe.point)?
    {
        return Err(HypermeshError::UnknownClassification);
    }
    let start_family = endpoint_definition_family(&interior.point, &interior.planes)?;
    let end_family = endpoint_definition_family(&probe.point, &probe.planes)?;
    let saw_unknown = start_family.saw_unknown || end_family.saw_unknown;

    let result = cached_definition_no_detour_reachability_with(
        no_step_cache,
        &interior.point,
        &probe.point,
        &start_family.definitions,
        &end_family.definitions,
        || {
            probe_reaches_adjacent_cell_with_definition_search(
                &interior.point,
                &probe.point,
                &start_family.definitions,
                &end_family.definitions,
                || {
                    cached_direct_probe_reachability_with(
                        direct_probe_reachability,
                        &interior.point,
                        &probe.point,
                        host_support,
                        polygons,
                        || {
                            probe_reaches_adjacent_cell(
                                &interior.point,
                                &probe.point,
                                host_support,
                                polygons,
                            )
                        },
                    )
                },
                |start_definition, end_definition| {
                    plane_replacement_path_reaches_adjacent_cell_without_step_detours_with_caches(
                        start_definition,
                        end_definition,
                        host_support,
                        polygons,
                        plane_replacement_affine,
                        plane_replacement_reachability_paths,
                        plane_replacement_reachability_steps,
                        Some(trace_bounds),
                    )
                },
            )
        },
    );
    match result {
        Ok(false) if saw_unknown => Err(HypermeshError::UnknownClassification),
        result => result,
    }
}

#[cfg(test)]
pub(super) fn cached_adjacent_normal_probes_with(
    cache: &mut Vec<NormalProbeFamilyCacheEntry>,
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    positive_side: bool,
    query: impl FnOnce() -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.interior_point == interior.point
            && definition_families_match_as_sets(&existing.interior_planes, &interior.planes)
            && existing.support == *support
            && existing.bounds == *bounds
            && existing.positive_side == positive_side
    }) {
        return existing.probes.clone();
    }

    let probes = query();
    cache.push(NormalProbeFamilyCacheEntry {
        interior_point: interior.point.clone(),
        interior_planes: interior.planes.clone(),
        support: support.clone(),
        bounds: bounds.clone(),
        positive_side,
        probes: probes.clone(),
    });
    probes
}

#[cfg(test)]
pub(super) fn cached_adjacent_axis_probes_with(
    cache: &mut Vec<AxisProbeFamilyCacheEntry>,
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    axis: usize,
    direction_positive: bool,
    query: impl FnOnce() -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.interior_point == interior.point
            && definition_families_match_as_sets(&existing.interior_planes, &interior.planes)
            && existing.support == *support
            && existing.bounds == *bounds
            && existing.axis == axis
            && existing.direction_positive == direction_positive
    }) {
        return existing.probes.clone();
    }

    let probes = query();
    cache.push(AxisProbeFamilyCacheEntry {
        interior_point: interior.point.clone(),
        interior_planes: interior.planes.clone(),
        support: support.clone(),
        bounds: bounds.clone(),
        axis,
        direction_positive,
        probes: probes.clone(),
    });
    probes
}

pub(super) fn cached_adjacent_normal_probe_stop_values_with(
    cache: &mut Vec<NormalProbeStopCacheEntry>,
    interior: &Point3,
    direction: &Point3,
    support: &Plane,
    bounds: &Aabb,
    query: impl FnOnce() -> HypermeshResult<(Vec<Real>, bool)>,
) -> HypermeshResult<(Vec<Real>, bool)> {
    if let Some(existing) = cache.iter().rev().find(|existing| {
        existing.interior_point == *interior
            && existing.direction == *direction
            && existing.support == *support
            && existing.bounds == *bounds
    }) {
        return Ok((existing.stop_values.clone()?, existing.saw_unknown));
    }

    let (stop_values, saw_unknown) = query()?;
    cache.push(NormalProbeStopCacheEntry {
        interior_point: interior.clone(),
        direction: direction.clone(),
        support: support.clone(),
        bounds: bounds.clone(),
        saw_unknown,
        stop_values: Ok(stop_values.clone()),
    });
    Ok((stop_values, saw_unknown))
}

pub(super) fn cached_adjacent_axis_probe_stop_values_with(
    cache: &mut Vec<AxisProbeStopCacheEntry>,
    interior: &Point3,
    bounds: &Aabb,
    axis: usize,
    direction_positive: bool,
    query: impl FnOnce() -> HypermeshResult<(Vec<Real>, bool)>,
) -> HypermeshResult<(Vec<Real>, bool)> {
    if let Some(existing) = cache.iter().rev().find(|existing| {
        existing.interior_point == *interior
            && existing.bounds == *bounds
            && existing.axis == axis
            && existing.direction_positive == direction_positive
    }) {
        return Ok((existing.stop_values.clone()?, existing.saw_unknown));
    }

    let (stop_values, saw_unknown) = query()?;
    cache.push(AxisProbeStopCacheEntry {
        interior_point: interior.clone(),
        bounds: bounds.clone(),
        axis,
        direction_positive,
        saw_unknown,
        stop_values: Ok(stop_values.clone()),
    });
    Ok((stop_values, saw_unknown))
}

#[cfg(test)]
pub(super) fn cached_bounded_probes_from_interior_with(
    cache: &mut Vec<ProbePointFamilyCacheEntry>,
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    positive_side: bool,
    query: impl FnOnce() -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.interior_point == interior.point
            && definition_families_match_as_sets(&existing.interior_planes, &interior.planes)
            && existing.support == *support
            && existing.bounds == *bounds
            && existing.positive_side == positive_side
    }) {
        return existing.probes.clone();
    }

    let probes = query();
    cache.push(ProbePointFamilyCacheEntry {
        interior_point: interior.point.clone(),
        interior_planes: interior.planes.clone(),
        support: support.clone(),
        bounds: bounds.clone(),
        positive_side,
        probes: probes.clone(),
    });
    probes
}

pub(super) fn cached_probe_winding_with(
    cache: &mut Vec<ProbeWindingCacheEntry>,
    probe: &ProbePoint,
    trace: impl FnOnce() -> HypermeshResult<WindingNumberVector>,
) -> HypermeshResult<WindingNumberVector> {
    if let Some(existing) = cache.iter().rev().find(|existing| {
        existing.point == probe.point
            && definition_families_match_as_sets(&existing.planes, &probe.planes)
    }) {
        return existing.winding.clone();
    }

    let winding = trace();
    cache.push(ProbeWindingCacheEntry {
        point: probe.point.clone(),
        planes: probe.planes.clone(),
        winding: winding.clone(),
    });
    winding
}

fn probe_reachability_cache_entry_matches(
    existing: &ProbeReachabilityCacheEntry,
    interior: &InteriorLeafPoint,
    probe: &ProbePoint,
) -> bool {
    existing.interior_point == interior.point
        && definition_families_match_as_sets(&existing.interior_planes, &interior.planes)
        && existing.probe_point == probe.point
        && definition_families_match_as_sets(&existing.probe_planes, &probe.planes)
}

fn begin_probe_reachability_result(
    cache: &mut Vec<ProbeReachabilityCacheEntry>,
    interior: &InteriorLeafPoint,
    probe: &ProbePoint,
) -> usize {
    cache.push(ProbeReachabilityCacheEntry {
        interior_point: interior.point.clone(),
        interior_planes: interior.planes.clone(),
        probe_point: probe.point.clone(),
        probe_planes: probe.planes.clone(),
        reachable: Err(HypermeshError::UnknownClassification),
    });
    cache.len() - 1
}

pub(super) fn cached_probe_reachability_with(
    cache: &mut Vec<ProbeReachabilityCacheEntry>,
    interior: &InteriorLeafPoint,
    probe: &ProbePoint,
    query: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache
        .iter()
        .rev()
        .find(|existing| probe_reachability_cache_entry_matches(existing, interior, probe))
    {
        return existing.reachable.clone();
    }

    let cache_index = begin_probe_reachability_result(cache, interior, probe);
    let reachable = query();
    cache[cache_index].reachable = reachable.clone();
    reachable
}

#[cfg(test)]
pub(super) fn search_leaf_probe_families<'a>(
    interior_points: &'a [InteriorLeafPoint],
    mut probes_for: impl FnMut(&'a InteriorLeafPoint, bool) -> HypermeshResult<Vec<ProbePoint>>,
    mut handle_probe: impl FnMut(
        &'a InteriorLeafPoint,
        bool,
        ProbePoint,
    ) -> HypermeshResult<Option<WindingNumberVector>>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut saw_unknown = false;

    for point in interior_points {
        for positive_side in [true, false] {
            let probes = match probes_for(point, positive_side) {
                Ok(probes) => probes,
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
            if probes.is_empty() && point.uncertified_definition_fallback {
                saw_unknown = true;
            }

            for probe in probes {
                let probe_fallback = probe.uncertified_definition_fallback;
                match handle_probe(point, positive_side, probe) {
                    Ok(Some(winding)) => return Ok(Some(winding)),
                    Ok(None) => {
                        if point.uncertified_definition_fallback || probe_fallback {
                            saw_unknown = true;
                        }
                    }
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                    }
                    Err(err) => return Err(err),
                }
            }
        }
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(None)
    }
}

#[cfg(test)]
pub(super) fn trace_probe_winding(
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    probe: &ProbePoint,
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    let mut surface_cache = Vec::new();
    let mut axis_ordered_segment_traces = Vec::new();
    let mut plane_replacement_affine = PlaneReplacementAffineCache::default();
    let mut plane_replacement_trace_steps = Vec::new();
    let mut definition_no_detour_trace = Vec::new();
    let mut detour_target_families = DetourTargetFamilyCache::default();
    trace_probe_winding_with_caches(
        ref_point,
        ref_definitions,
        probe,
        ref_wnv,
        polygons,
        &mut surface_cache,
        &mut axis_ordered_segment_traces,
        &mut plane_replacement_affine,
        &mut plane_replacement_trace_steps,
        &mut definition_no_detour_trace,
        &mut detour_target_families,
        None,
    )
}

pub(super) fn trace_probe_winding_with_caches(
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    probe: &ProbePoint,
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    axis_ordered_segment_traces: &mut Vec<AxisOrderedSegmentTraceCacheEntry>,
    plane_replacement_affine: &mut PlaneReplacementAffineCache,
    plane_replacement_trace_steps: &mut Vec<PlaneReplacementStepCacheEntry>,
    definition_no_detour_trace: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    detour_target_families: &mut DetourTargetFamilyCache,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<WindingNumberVector> {
    let mut probe_definitions = probe.planes.clone();
    let axis_definition = axis_plane_defined_point(&probe.point).planes;
    if !probe_definitions
        .iter()
        .any(|definition| definition_planes_match_as_sets(definition, &axis_definition))
    {
        probe_definitions.push(axis_definition);
    }
    probe_definitions = unique_definition_family(&probe_definitions);

    trace_segment_from_definitions_with_step_detoured_plane_replacement_with_caches(
        ref_point,
        &probe.point,
        ref_wnv,
        polygons,
        ref_definitions,
        &probe_definitions,
        surface_cache,
        axis_ordered_segment_traces,
        plane_replacement_affine,
        plane_replacement_trace_steps,
        definition_no_detour_trace,
        detour_target_families,
        trace_bounds,
    )
}
