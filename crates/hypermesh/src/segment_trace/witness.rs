//! Exact leaf interior witnesses.

#[cfg(test)]
use super::halfspace_witness::feasible_halfspace_cell_vertices;
use super::halfspace_witness::{
    DefinitionFamilyState, HalfspaceSeedFamilyState, ShiftedHalfspaceWitness,
    active_planes_from_optional_report, collect_strict_halfspace_seed_family,
    dedupe_shifted_halfspace_seed_families,
    extend_shifted_halfspace_seed_families_backtracking_unknown,
    extend_strict_halfspace_seed_families_backtracking_unknown,
    halfspace_cell_seed_families_from_optional_report, optional_halfspace_feasibility_report,
    push_unique_halfspace_seed, seed_family_search_failed_without_any_seed,
    shifted_halfspace_cell_witnesses_from_seed, shifted_halfspace_seed_families_with_report_seed,
    shifted_halfspace_witness_family_or_empty,
};
use super::path::{
    axis_plane_definition, definition_planes_match_as_sets, extend_unique_definition_families,
    finalize_interior_point_family,
};
use super::{CrossingEvent, InteriorLeafPoint, LeafWitnessSeedFamilies};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{
    Aabb, Classification, Plane, PreparedPoint3, axis_mut, axis_ref, classify_real, compare_real,
};
use crate::polygon::ConvexPolygon;
use crate::winding::WindingNumberTransitionVector;
use hyperlattice::{HomogeneousPoint3, Point3, Real, intersect_three_planes};
use hyperlimit::{HalfspaceFeasibility, Plane3 as LimitPlane3};
use std::sync::Arc;

pub(super) fn planes_are_coplanar(left: &Plane, right: &Plane) -> HypermeshResult<bool> {
    let left_coefficients = [&left.normal.x, &left.normal.y, &left.normal.z, &left.offset];
    let right_coefficients = [
        &right.normal.x,
        &right.normal.y,
        &right.normal.z,
        &right.offset,
    ];

    for i in 0..left_coefficients.len() {
        for j in (i + 1)..left_coefficients.len() {
            let determinant = Real::signed_product_sum(
                [true, false],
                [
                    [left_coefficients[i], right_coefficients[j]],
                    [left_coefficients[j], right_coefficients[i]],
                ],
            );
            if classify_real(&determinant)? != Classification::On {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum PolygonPointLocation {
    Outside,
    Boundary,
    Interior,
}

pub(super) fn classify_point_in_polygon(
    point: &Point3,
    polygon: &ConvexPolygon,
) -> HypermeshResult<PolygonPointLocation> {
    let point = PreparedPoint3::new(point);
    if point.classify(&polygon.support)? != Classification::On {
        return Ok(PolygonPointLocation::Outside);
    }
    let mut on_edge = false;
    for edge in polygon.edges.iter() {
        match point.classify(edge)? {
            Classification::Positive => return Ok(PolygonPointLocation::Outside),
            Classification::On => on_edge = true,
            Classification::Negative => {}
        }
    }
    if on_edge {
        Ok(PolygonPointLocation::Boundary)
    } else {
        Ok(PolygonPointLocation::Interior)
    }
}

pub(super) fn segment_plane_crossing(
    start: &Point3,
    end: &Point3,
    plane: &Plane,
) -> HypermeshResult<Option<Point3>> {
    let start_value = plane.expression_at_point(start);
    let end_value = plane.expression_at_point(end);
    let start_class = crate::geometry::classify_real(&start_value)?;
    let end_class = crate::geometry::classify_real(&end_value)?;

    if start_class == Classification::On || end_class == Classification::On {
        return Ok(None);
    }
    if start_class == end_class {
        return Ok(None);
    }

    let denom = &start_value - &end_value;
    let t = (start_value / denom).map_err(|_| HypermeshError::UnknownClassification)?;
    Ok(Some(Point3::new(
        &start.x + &(t.clone() * (&end.x - &start.x)),
        &start.y + &(t.clone() * (&end.y - &start.y)),
        &start.z + &(t * (&end.z - &start.z)),
    )))
}

pub(super) fn point_strictly_between_axis(
    point: &Point3,
    start: &Point3,
    end: &Point3,
    axis: usize,
) -> HypermeshResult<bool> {
    let start_to_point = compare_real(axis_ref(point, axis), axis_ref(start, axis))?;
    let point_to_end = compare_real(axis_ref(point, axis), axis_ref(end, axis))?;
    Ok((start_to_point.is_gt() && point_to_end.is_lt())
        || (start_to_point.is_lt() && point_to_end.is_gt()))
}

pub(super) fn sort_crossing_events(
    events: &mut Vec<CrossingEvent>,
    axis: usize,
    dir_sign: i32,
) -> HypermeshResult<()> {
    let mut sorted: Vec<CrossingEvent> = Vec::with_capacity(events.len());
    for event in events.drain(..) {
        let mut insert_at = sorted.len();
        for (index, existing) in sorted.iter().enumerate() {
            let order = compare_real(
                axis_ref(&event.point, axis),
                axis_ref(&existing.point, axis),
            )?;
            if (dir_sign > 0 && order.is_lt()) || (dir_sign < 0 && order.is_gt()) {
                insert_at = index;
                break;
            }
        }
        sorted.insert(insert_at, event);
    }
    *events = sorted;
    Ok(())
}

pub(super) fn dominant_normal_axis(plane: &Plane) -> HypermeshResult<usize> {
    let abs = [
        plane.normal.x.clone().abs(),
        plane.normal.y.clone().abs(),
        plane.normal.z.clone().abs(),
    ];
    let mut best = 0;
    for axis in 1..3 {
        if compare_real(&abs[axis], &abs[best])?.is_gt() {
            best = axis;
        }
    }
    Ok(best)
}

pub(super) fn centroid(points: &[Point3]) -> HypermeshResult<Option<Point3>> {
    if points.is_empty() {
        return Ok(None);
    }
    let mut sum = Point3::origin();
    for point in points {
        sum.x += point.x.clone();
        sum.y += point.y.clone();
        sum.z += point.z.clone();
    }
    let denom = Real::from(points.len() as u64);
    Ok(Some(Point3::new(
        (sum.x / denom.clone()).map_err(|_| HypermeshError::UnknownClassification)?,
        (sum.y / denom.clone()).map_err(|_| HypermeshError::UnknownClassification)?,
        (sum.z / denom).map_err(|_| HypermeshError::UnknownClassification)?,
    )))
}

#[cfg(test)]
pub(super) fn halfspace_cell_geometry_seed_candidates(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<Vec<Point3>> {
    let vertices = feasible_halfspace_cell_vertices(halfspaces)?;
    halfspace_cell_geometry_seed_candidates_from_vertices(&vertices)
}

#[cfg(test)]
pub(super) fn halfspace_cell_geometry_seed_candidates_from_vertices(
    vertices: &[Point3],
) -> HypermeshResult<Vec<Point3>> {
    Ok(halfspace_centroid_subset_seed_family_from_vertices(vertices)?.seeds)
}

pub(super) fn halfspace_centroid_subset_seed_family_from_vertices(
    vertices: &[Point3],
) -> HypermeshResult<HalfspaceSeedFamilyState> {
    halfspace_centroid_subset_seed_family_from_vertices_with(vertices, centroid)
}

pub(super) fn halfspace_centroid_subset_seed_family_from_vertices_with(
    vertices: &[Point3],
    mut center_of: impl FnMut(&[Point3]) -> HypermeshResult<Option<Point3>>,
) -> HypermeshResult<HalfspaceSeedFamilyState> {
    let mut candidates = Vec::new();
    let mut subset = Vec::new();
    let mut saw_unknown = false;
    collect_halfspace_centroid_subset_candidates(
        &mut candidates,
        vertices,
        0,
        &mut subset,
        &mut saw_unknown,
        &mut center_of,
    )?;
    if candidates.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(HalfspaceSeedFamilyState {
            seeds: candidates,
            saw_unknown,
        })
    }
}

fn collect_halfspace_centroid_subset_candidates(
    candidates: &mut Vec<Point3>,
    vertices: &[Point3],
    start: usize,
    subset: &mut Vec<Point3>,
    saw_unknown: &mut bool,
    center_of: &mut impl FnMut(&[Point3]) -> HypermeshResult<Option<Point3>>,
) -> HypermeshResult<()> {
    for index in start..vertices.len() {
        subset.push(vertices[index].clone());
        if subset.len() >= 2 {
            match center_of(subset) {
                Ok(Some(center)) => push_unique_halfspace_seed(candidates, center),
                Ok(None) => {}
                Err(HypermeshError::UnknownClassification) => {
                    *saw_unknown = true;
                }
                Err(err) => return Err(err),
            }
        }
        collect_halfspace_centroid_subset_candidates(
            candidates,
            vertices,
            index + 1,
            subset,
            saw_unknown,
            center_of,
        )?;
        subset.pop();
    }
    Ok(())
}

pub(super) fn interior_leaf_points(
    leaf: &ConvexPolygon,
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let vertices = leaf.vertices()?;
    if vertices.is_empty() {
        return Ok(Vec::new());
    }

    if let Some(center) = centroid(&vertices)? {
        match point_strictly_inside_leaf_or_unknown(&center, leaf) {
            Ok(true) => {
                let mut points = vec![InteriorLeafPoint {
                    point: center.clone(),
                    planes: Vec::new(),
                    uncertified_definition_fallback: false,
                }];
                extend_interior_leaf_points_backtracking_unknown(
                    &mut points,
                    std::iter::once(&center),
                    |witness| shifted_edge_interior_points(leaf, witness),
                )?;
                if points.iter().any(|point| !point.planes.is_empty()) {
                    points.retain(|point| !point.planes.is_empty());
                }
                if !points.is_empty() {
                    return Ok(points);
                }
            }
            Ok(false) => {}
            Err(HypermeshError::UnknownClassification) => {}
            Err(err) => return Err(err),
        }
    }

    let mut points = strict_leaf_witness_points(leaf, &vertices)?;
    let witness_points = points
        .iter()
        .map(|point| point.point.clone())
        .collect::<Vec<_>>();
    extend_interior_leaf_points_backtracking_unknown(
        &mut points,
        witness_points.iter(),
        |witness| shifted_edge_interior_points(leaf, witness),
    )?;

    Ok(points)
}

pub(super) fn strict_leaf_witness_points(
    leaf: &ConvexPolygon,
    vertices: &[Point3],
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    strict_leaf_witness_points_with_seed_families(
        leaf,
        vertices,
        |leaf, vertices, bounds, halfspaces, report| {
            leaf_witness_seed_families(leaf, vertices, bounds, halfspaces, report)
        },
    )
}

pub(super) fn strict_leaf_witness_points_with_seed_families(
    leaf: &ConvexPolygon,
    vertices: &[Point3],
    mut seed_families_for: impl FnMut(
        &ConvexPolygon,
        &[Point3],
        &Aabb,
        &[LimitPlane3],
        Option<&hyperlimit::HalfspaceFeasibilityReport>,
    ) -> HypermeshResult<LeafWitnessSeedFamilies>,
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    strict_leaf_witness_points_with_seed_families_and_stricter_replay(
        leaf,
        vertices,
        &mut seed_families_for,
        strict_leaf_cell_points,
    )
}

pub(super) fn strict_leaf_witness_points_with_seed_families_and_stricter_replay(
    leaf: &ConvexPolygon,
    vertices: &[Point3],
    seed_families_for: &mut impl FnMut(
        &ConvexPolygon,
        &[Point3],
        &Aabb,
        &[LimitPlane3],
        Option<&hyperlimit::HalfspaceFeasibilityReport>,
    ) -> HypermeshResult<LeafWitnessSeedFamilies>,
    mut stricter_points_for: impl FnMut(
        &ConvexPolygon,
        &Point3,
    ) -> HypermeshResult<Vec<InteriorLeafPoint>>,
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let bounds = leaf_bounds(vertices)?;
    let halfspaces = leaf_halfspaces(leaf);
    let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(Vec::new());
    }

    let mut points = Vec::new();
    let LeafWitnessSeedFamilies {
        seeds,
        shifted_vertices,
        shifted_geometry_seeds,
        saw_unknown: seed_saw_unknown,
    } = seed_families_for(leaf, vertices, &bounds, &halfspaces, report.as_ref())?;
    saw_unknown |= seed_saw_unknown;
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);

    extend_leaf_point_builds_backtracking_unknown(&mut points, seeds.iter(), |seed| {
        let active_planes = active_planes_from_optional_report(report.as_ref(), seed);
        build_strict_leaf_point(leaf, seed, &halfspaces, active_planes, false)
    })?;

    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_halfspace_seed_families_with_report_seed(
            report_witness,
            seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );

    let shifted_witnesses = shifted_halfspace_witness_family_or_empty(
        {
            let mut shifted_witnesses = Vec::new();
            extend_shifted_halfspace_seed_families_backtracking_unknown(
                &mut shifted_witnesses,
                [strict_shift_seeds, shifted_vertices, shifted_geometry_seeds],
                |seed| shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    extend_leaf_point_builds_backtracking_unknown(
        &mut points,
        shifted_witnesses.iter(),
        |shifted| build_strict_leaf_point_from_shifted_witness(leaf, shifted),
    )?;
    let direct_witnesses = points
        .iter()
        .map(|point| point.point.clone())
        .collect::<Vec<_>>();
    let mut stricter_points = Vec::new();
    match extend_interior_leaf_points_backtracking_unknown(
        &mut stricter_points,
        direct_witnesses.iter(),
        |witness| stricter_points_for(leaf, witness),
    ) {
        Ok(()) => {}
        Err(HypermeshError::UnknownClassification) => {
            saw_unknown = true;
        }
        Err(err) => return Err(err),
    }
    for point in stricter_points {
        push_unique_interior_point(&mut points, point);
    }

    finalize_interior_point_family(&mut points, saw_unknown)?;
    Ok(points)
}

fn leaf_witness_seed_families(
    leaf: &ConvexPolygon,
    _vertices: &[Point3],
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
) -> HypermeshResult<LeafWitnessSeedFamilies> {
    let mut saw_unknown = false;
    let (generic_seeds, shifted_vertices, shifted_geometry_seeds) =
        halfspace_cell_seed_families_from_optional_report(
            bounds,
            halfspaces,
            report,
            &mut saw_unknown,
        )?;
    let mut seeds = generic_seeds;

    extend_strict_halfspace_seed_families_backtracking_unknown(
        &mut seeds,
        [collect_strict_halfspace_seed_family(
            Ok(shifted_geometry_seeds.clone()),
            |candidate| point_strictly_inside_leaf_or_unknown(candidate, leaf),
        )],
    )?;

    if seed_family_search_failed_without_any_seed(
        &seeds,
        &shifted_vertices,
        &shifted_geometry_seeds,
        saw_unknown,
    ) {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(LeafWitnessSeedFamilies {
            seeds,
            shifted_vertices,
            shifted_geometry_seeds,
            saw_unknown,
        })
    }
}

#[cfg(test)]
pub(super) fn strict_leaf_witness_seeds(
    leaf: &ConvexPolygon,
    vertices: &[Point3],
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
) -> HypermeshResult<Vec<Point3>> {
    leaf_witness_seed_families(leaf, vertices, bounds, halfspaces, report)
        .map(|families| families.seeds)
}

pub(super) fn leaf_bounds(vertices: &[Point3]) -> HypermeshResult<Aabb> {
    let Some(first) = vertices.first() else {
        return Err(HypermeshError::UnknownClassification);
    };

    let mut min = first.clone();
    let mut max = first.clone();
    for vertex in &vertices[1..] {
        for axis in 0..3 {
            if compare_real(axis_ref(vertex, axis), axis_ref(&min, axis))?.is_lt() {
                *axis_mut(&mut min, axis) = axis_ref(vertex, axis).clone();
            }
            if compare_real(axis_ref(vertex, axis), axis_ref(&max, axis))?.is_gt() {
                *axis_mut(&mut max, axis) = axis_ref(vertex, axis).clone();
            }
        }
    }

    Ok(Aabb::new(min, max))
}

pub(super) fn leaf_halfspaces(leaf: &ConvexPolygon) -> Vec<LimitPlane3> {
    let mut halfspaces = Vec::with_capacity(leaf.edges.len() + 2);
    halfspaces.push(limit_plane_from_plane(&leaf.support));
    halfspaces.push(limit_plane_from_plane(&leaf.support.inverted()));
    for edge in leaf.edges.iter() {
        halfspaces.push(limit_plane_from_plane(edge));
    }
    halfspaces
}

#[cfg(test)]
pub(crate) fn certified_leaf_test_point(
    support: &Plane,
    edges: &[Plane],
) -> HypermeshResult<Option<HomogeneousPoint3>> {
    let points = certified_leaf_interior_points(support, edges)?;
    let Some(point) = points
        .iter()
        .find(|point| !point.planes.is_empty())
        .or_else(|| points.first())
    else {
        return Ok(None);
    };
    Ok(Some(HomogeneousPoint3::new(
        point.point.x.clone(),
        point.point.y.clone(),
        point.point.z.clone(),
        Real::one(),
    )))
}

pub(crate) fn certified_leaf_test_points(
    support: &Plane,
    edges: &[Plane],
) -> HypermeshResult<Vec<HomogeneousPoint3>> {
    Ok(certified_leaf_interior_points(support, edges)?
        .into_iter()
        .map(|point| {
            HomogeneousPoint3::new(point.point.x, point.point.y, point.point.z, Real::one())
        })
        .collect())
}

pub(crate) fn certified_leaf_interior_points(
    support: &Plane,
    edges: &[Plane],
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let leaf = ConvexPolygon {
        support: support.clone(),
        edges: Arc::new(edges.to_vec()),
        mesh_index: -1,
        polygon_index: -1,
        delta_w: WindingNumberTransitionVector::new(),
        approx_bounds: None,
    };
    interior_leaf_points(&leaf)
}

pub(super) fn shifted_edge_interior_points(
    leaf: &ConvexPolygon,
    strict_interior: &Point3,
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let mut points = Vec::with_capacity(leaf.vertex_count());
    let half = (Real::one() / Real::from(2)).map_err(|_| HypermeshError::UnknownClassification)?;

    for first_edge in 0..leaf.vertex_count() {
        let second_edge = (first_edge + 1) % leaf.vertex_count();
        let first_margin = leaf.edges[first_edge].expression_at_point(strict_interior);
        let second_margin = leaf.edges[second_edge].expression_at_point(strict_interior);
        if classify_real(&first_margin)? != Classification::Negative
            || classify_real(&second_margin)? != Classification::Negative
        {
            continue;
        }

        let first_shifted =
            inward_shifted_edge_plane(&leaf.edges[first_edge], &first_margin, &half);
        let second_shifted =
            inward_shifted_edge_plane(&leaf.edges[second_edge], &second_margin, &half);
        let candidate = intersect_three_planes(&leaf.support, &first_shifted, &second_shifted)
            .to_affine_point()
            .map_err(|_| HypermeshError::UnknownClassification)?;

        if point_strictly_inside_leaf_or_unknown(&candidate, leaf)? {
            push_unique_interior_point(
                &mut points,
                InteriorLeafPoint {
                    point: candidate,
                    planes: vec![[leaf.support.clone(), first_shifted, second_shifted]],
                    uncertified_definition_fallback: false,
                },
            );
        }
    }

    Ok(points)
}

pub(super) fn inward_shifted_edge_plane(
    edge: &Plane,
    strict_interior_margin: &Real,
    fraction: &Real,
) -> Plane {
    let inward_offset = strict_interior_margin * fraction;
    Plane::new(edge.normal.clone(), &edge.offset - &inward_offset)
}

pub(super) fn push_unique_interior_point(
    points: &mut Vec<InteriorLeafPoint>,
    point: InteriorLeafPoint,
) -> bool {
    if let Some(existing) = points
        .iter_mut()
        .find(|existing| existing.point == point.point)
    {
        let incoming_planes = point.planes;
        let incoming_is_fallback = point.uncertified_definition_fallback;
        let existing_covered_by_incoming = existing.planes.iter().all(|existing_planes| {
            incoming_planes.iter().any(|incoming_plane_set| {
                definition_planes_match_as_sets(existing_planes, incoming_plane_set)
            })
        });
        let mut introduced_new_definition = false;
        for planes in incoming_planes {
            if !existing
                .planes
                .iter()
                .any(|candidate| definition_planes_match_as_sets(candidate, &planes))
            {
                existing.planes.push(planes);
                introduced_new_definition = true;
            }
        }
        if incoming_is_fallback {
            if introduced_new_definition {
                existing.uncertified_definition_fallback = true;
                true
            } else {
                false
            }
        } else {
            if existing_covered_by_incoming {
                existing.uncertified_definition_fallback = false;
            }
            false
        }
    } else {
        let introduced_uncertified_state = point.uncertified_definition_fallback;
        points.push(point);
        introduced_uncertified_state
    }
}

pub(super) fn extend_interior_leaf_points_backtracking_unknown<'a, T: 'a>(
    points: &mut Vec<InteriorLeafPoint>,
    candidates: impl IntoIterator<Item = &'a T>,
    mut build: impl FnMut(&'a T) -> HypermeshResult<Vec<InteriorLeafPoint>>,
) -> HypermeshResult<()> {
    let mut saw_hard_unknown = false;
    for candidate in candidates {
        match build(candidate) {
            Ok(found) => {
                for point in found {
                    push_unique_interior_point(points, point);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_interior_point_family(points, saw_hard_unknown)
}

pub(super) fn extend_leaf_point_builds_backtracking_unknown<'a, T: 'a>(
    points: &mut Vec<InteriorLeafPoint>,
    candidates: impl IntoIterator<Item = &'a T>,
    mut build: impl FnMut(&'a T) -> HypermeshResult<Option<InteriorLeafPoint>>,
) -> HypermeshResult<()> {
    let mut saw_hard_unknown = false;
    for candidate in candidates {
        match build(candidate) {
            Ok(Some(point)) => {
                push_unique_interior_point(points, point);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_interior_point_family(points, saw_hard_unknown)
}

pub(super) fn strict_leaf_cell_points(
    leaf: &ConvexPolygon,
    strict_interior: &Point3,
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let vertices = leaf.vertices()?;
    let bounds = leaf_bounds(&vertices)?;
    let half = (Real::one() / Real::from(2)).map_err(|_| HypermeshError::UnknownClassification)?;
    let mut halfspaces = Vec::with_capacity(leaf.edges.len() + 2);
    halfspaces.push(limit_plane_from_plane(&leaf.support));
    halfspaces.push(limit_plane_from_plane(&leaf.support.inverted()));

    for edge in leaf.edges.iter() {
        let margin = edge.expression_at_point(strict_interior);
        if classify_real(&margin)? != Classification::Negative {
            return Ok(Vec::new());
        }
        halfspaces.push(limit_plane_from_plane(&inward_shifted_edge_plane(
            edge, &margin, &half,
        )));
    }

    let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(Vec::new());
    }

    let mut points = Vec::new();
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        halfspace_cell_seed_families_from_optional_report(
            &bounds,
            &halfspaces,
            report.as_ref(),
            &mut saw_unknown,
        )?;
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);
    extend_leaf_point_builds_backtracking_unknown(&mut points, seeds.iter(), |witness| {
        let active_planes = active_planes_from_optional_report(report.as_ref(), witness);
        build_strict_leaf_point(leaf, witness, &halfspaces, active_planes, false)
    })?;

    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_halfspace_seed_families_with_report_seed(
            report_witness,
            seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );

    let shifted_witnesses = shifted_halfspace_witness_family_or_empty(
        {
            let mut shifted_witnesses = Vec::new();
            extend_shifted_halfspace_seed_families_backtracking_unknown(
                &mut shifted_witnesses,
                [strict_shift_seeds, shifted_vertices, shifted_geometry_seeds],
                |seed| shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    for shifted in &shifted_witnesses {
        match build_strict_leaf_point_from_shifted_witness(leaf, shifted) {
            Ok(Some(point)) => {
                push_unique_interior_point(&mut points, point);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_interior_point_family(&mut points, saw_unknown)?;
    Ok(points)
}

#[cfg(test)]
pub(super) fn strict_leaf_cell_points_from_seed_families_with_tracking_unknown(
    leaf: &ConvexPolygon,
    strict_interior: &Point3,
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    mut build_shifted_witnesses: impl FnMut(&Point3) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>>,
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let vertices = leaf.vertices()?;
    let _bounds = leaf_bounds(&vertices)?;
    let half = (Real::one() / Real::from(2)).map_err(|_| HypermeshError::UnknownClassification)?;
    let mut halfspaces = Vec::with_capacity(leaf.edges.len() + 2);
    halfspaces.push(limit_plane_from_plane(&leaf.support));
    halfspaces.push(limit_plane_from_plane(&leaf.support.inverted()));

    for edge in leaf.edges.iter() {
        let margin = edge.expression_at_point(strict_interior);
        if classify_real(&margin)? != Classification::Negative {
            return Ok(Vec::new());
        }
        halfspaces.push(limit_plane_from_plane(&inward_shifted_edge_plane(
            edge, &margin, &half,
        )));
    }

    let mut points = Vec::new();
    let mut saw_unknown = false;
    let report_witness = report.and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);
    extend_leaf_point_builds_backtracking_unknown(&mut points, seeds.iter(), |witness| {
        let active_planes = active_planes_from_optional_report(report, witness);
        build_strict_leaf_point(leaf, witness, &halfspaces, active_planes, false)
    })?;

    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_halfspace_seed_families_with_report_seed(
            report_witness,
            seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );

    let shifted_witnesses = shifted_halfspace_witness_family_or_empty(
        {
            let mut shifted_witnesses = Vec::new();
            extend_shifted_halfspace_seed_families_backtracking_unknown(
                &mut shifted_witnesses,
                [strict_shift_seeds, shifted_vertices, shifted_geometry_seeds],
                |seed| build_shifted_witnesses(seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    for shifted in &shifted_witnesses {
        match build_strict_leaf_point_from_shifted_witness(leaf, shifted) {
            Ok(Some(point)) => {
                push_unique_interior_point(&mut points, point);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_interior_point_family(&mut points, saw_unknown)?;
    Ok(points)
}

pub(super) fn build_strict_leaf_point(
    leaf: &ConvexPolygon,
    witness: &Point3,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    inherited_uncertified_definition_fallback: bool,
) -> HypermeshResult<Option<InteriorLeafPoint>> {
    match classify_point_in_polygon(witness, leaf)? {
        PolygonPointLocation::Outside => return Ok(None),
        PolygonPointLocation::Boundary => {
            return Err(HypermeshError::UnknownClassification);
        }
        PolygonPointLocation::Interior => {}
    }

    let (planes, uncertified_definition_fallback) =
        match leaf_interior_definitions_from_active_halfspaces(
            witness,
            &leaf.support,
            halfspaces,
            active_planes,
        ) {
            Ok(found) => (found.definitions, false),
            Err(HypermeshError::UnknownClassification) => {
                (vec![axis_plane_definition(witness)], true)
            }
            Err(err) => return Err(err),
        };
    Ok(Some(InteriorLeafPoint {
        point: witness.clone(),
        planes,
        uncertified_definition_fallback: inherited_uncertified_definition_fallback
            || uncertified_definition_fallback,
    }))
}

pub(super) fn build_strict_leaf_point_from_shifted_witness(
    leaf: &ConvexPolygon,
    witness: &ShiftedHalfspaceWitness,
) -> HypermeshResult<Option<InteriorLeafPoint>> {
    match classify_point_in_polygon(&witness.point, leaf)? {
        PolygonPointLocation::Outside => return Ok(None),
        PolygonPointLocation::Boundary => {
            return Err(HypermeshError::UnknownClassification);
        }
        PolygonPointLocation::Interior => {}
    }

    let mut planes = Vec::new();
    let mut saw_unknown = false;
    for family in &witness.families {
        match leaf_interior_definitions_from_active_halfspaces(
            &witness.point,
            &leaf.support,
            &family.halfspaces,
            family.active_planes,
        ) {
            Ok(found) => {
                saw_unknown |= found.saw_unknown;
                extend_unique_definition_families(&mut planes, found.definitions);
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }

    let used_axis_fallback = planes.is_empty() && saw_unknown;
    if planes.is_empty() {
        if used_axis_fallback {
            planes.push(axis_plane_definition(&witness.point));
        } else {
            return Ok(None);
        }
    }

    Ok(Some(InteriorLeafPoint {
        point: witness.point.clone(),
        planes,
        uncertified_definition_fallback: witness.uncertified_definition_fallback
            || used_axis_fallback,
    }))
}

pub(super) fn witness_active_planes(
    report_witness: Option<&Point3>,
    active_planes: [Option<usize>; 3],
    witness: &Point3,
) -> [Option<usize>; 3] {
    if report_witness.is_some_and(|point| point == witness) {
        active_planes
    } else {
        [None, None, None]
    }
}

pub(super) fn limit_plane_from_plane(plane: &Plane) -> LimitPlane3 {
    LimitPlane3::new(plane.normal.clone(), plane.offset.clone())
}

pub(super) fn leaf_interior_definitions_from_active_halfspaces(
    witness: &Point3,
    support: &Plane,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
) -> HypermeshResult<DefinitionFamilyState> {
    let axis_definition = axis_plane_definition(witness);
    let mut definitions = Vec::new();
    let mut saw_unknown = false;
    let mut active = Vec::new();
    for index in active_planes.into_iter().flatten() {
        let Some(halfspace) = halfspaces.get(index) else {
            continue;
        };
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if plane == *support || plane == support.inverted() {
            continue;
        }
        if !active.iter().any(|existing| existing == &plane) {
            active.push(plane);
        }
    }

    for halfspace in halfspaces {
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if plane == *support || plane == support.inverted() {
            continue;
        }
        if !compare_real(&plane.expression_at_point(witness), &Real::zero())?.is_eq() {
            continue;
        }
        if !active.iter().any(|existing| existing == &plane) {
            active.push(plane);
        }
    }

    if active.len() >= 2 {
        for first in 0..active.len() {
            for second in (first + 1)..active.len() {
                push_verified_leaf_definition(
                    &mut definitions,
                    [
                        support.clone(),
                        active[first].clone(),
                        active[second].clone(),
                    ],
                    witness,
                    &mut saw_unknown,
                )?;
            }
        }
    }

    for plane in &active {
        for axis in axis_definition.iter().cloned() {
            push_verified_leaf_definition(
                &mut definitions,
                [support.clone(), plane.clone(), axis],
                witness,
                &mut saw_unknown,
            )?;
        }
    }

    for first_axis in 0..3 {
        for second_axis in (first_axis + 1)..3 {
            push_verified_leaf_definition(
                &mut definitions,
                [
                    support.clone(),
                    axis_definition[first_axis].clone(),
                    axis_definition[second_axis].clone(),
                ],
                witness,
                &mut saw_unknown,
            )?;
        }
    }

    if definitions.is_empty() {
        return Err(HypermeshError::UnknownClassification);
    }
    Ok(DefinitionFamilyState {
        definitions,
        saw_unknown,
    })
}

fn push_verified_leaf_definition(
    definitions: &mut Vec<[Plane; 3]>,
    definition: [Plane; 3],
    witness: &Point3,
    saw_unknown: &mut bool,
) -> HypermeshResult<()> {
    match intersect_three_planes(&definition[0], &definition[1], &definition[2]).to_affine_point() {
        Ok(point) if point == *witness => {
            if !definitions
                .iter()
                .any(|existing| definition_planes_match_as_sets(existing, &definition))
            {
                definitions.push(definition);
            }
        }
        Ok(_) => {}
        Err(_) => {
            *saw_unknown = true;
        }
    }
    Ok(())
}

#[cfg(test)]
pub(super) fn point_strictly_inside_leaf(
    point: &Point3,
    leaf: &ConvexPolygon,
) -> HypermeshResult<bool> {
    let homogeneous = HomogeneousPoint3::new(
        point.x.clone(),
        point.y.clone(),
        point.z.clone(),
        Real::one(),
    );
    leaf.contains_point_strictly(&homogeneous)
}

pub(super) fn point_strictly_inside_leaf_or_unknown(
    point: &Point3,
    leaf: &ConvexPolygon,
) -> HypermeshResult<bool> {
    match classify_point_in_polygon(point, leaf)? {
        PolygonPointLocation::Outside => Ok(false),
        PolygonPointLocation::Boundary => Err(HypermeshError::UnknownClassification),
        PolygonPointLocation::Interior => Ok(true),
    }
}
