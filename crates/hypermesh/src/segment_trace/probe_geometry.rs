//! Exact normal and axis probe geometry.

#[cfg(test)]
use super::DetourTarget;
use super::halfspace_witness::{
    DefinitionFamilyState, ShiftedHalfspaceWitness, active_planes_from_optional_report,
    collect_strict_halfspace_seed_family, dedupe_shifted_halfspace_seed_families,
    extend_shifted_halfspace_seed_families_backtracking_unknown,
    extend_strict_halfspace_seed_families_collect_unknown,
    halfspace_cell_seed_families_from_optional_report, optional_halfspace_feasibility_report,
    point_strictly_inside_halfspace_cell_or_unknown, seed_family_search_failed_without_any_seed,
    shifted_halfspace_cell_witnesses_from_seed, shifted_halfspace_seed_families_with_report_seed,
    shifted_halfspace_witness_family_or_empty, take_new_halfspace_seed_family,
};
#[cfg(test)]
use super::halfspace_witness::{HalfspaceSeedFamilyState, feasible_halfspace_cell_vertex_family};
#[cfg(test)]
use super::path::point_lies_on_traced_surface;
use super::path::{
    axis_plane_definition, definition_planes_match_as_sets, extend_unique_definition_families,
    finalize_probe_point_family, other_axes, unique_definition_family,
};
use super::probe_cache::{
    cached_halfspace_cell_seed_families_from_optional_report_with,
    cached_optional_halfspace_feasibility_report_with,
};
#[cfg(test)]
use super::witness::halfspace_centroid_subset_seed_family_from_vertices;
use super::witness::{
    PolygonPointLocation, classify_point_in_polygon, dominant_normal_axis, planes_are_coplanar,
    point_strictly_between_axis, segment_plane_crossing,
};
use super::{InteriorLeafPoint, LeafProbeQueryCaches, ProbePoint};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{
    Aabb, Classification, Plane, axis_mut, axis_ref, classify_point, classify_real, compare_real,
};
use crate::halfspace::{aabb_core_halfspaces, negated_halfspace, support_side_halfspace};
use crate::polygon::ConvexPolygon;
use hyperlattice::{Point3, Real, intersect_three_planes};
use hyperlimit::{HalfspaceFeasibility, Plane3 as LimitPlane3};

#[cfg(test)]
pub(super) fn bounded_probes_from_interior(
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    positive_side: bool,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut probes = Vec::new();
    let mut saw_unknown = false;

    extend_probe_families_backtracking_unknown(
        &mut probes,
        adjacent_normal_probes(interior, support, bounds, polygons, positive_side),
        &mut saw_unknown,
    )?;

    for axis in probe_axes(support)? {
        let normal_sign = crate::geometry::classify_real(axis_ref(&support.normal, axis))?;
        if normal_sign == Classification::On {
            continue;
        }

        let direction_positive = (normal_sign == Classification::Positive) == positive_side;
        let axis_value = axis_ref(&interior.point, axis);
        let room = if direction_positive {
            axis_ref(&bounds.max, axis) - axis_value
        } else {
            axis_value - axis_ref(&bounds.min, axis)
        };
        if !compare_real(&room, &Real::zero())?.is_gt() {
            continue;
        }

        extend_probe_families_backtracking_unknown(
            &mut probes,
            adjacent_axis_probes(
                interior,
                support,
                bounds,
                polygons,
                axis,
                direction_positive,
            ),
            &mut saw_unknown,
        )?;
    }

    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
}

#[cfg_attr(not(test), allow(dead_code))]
pub(super) fn extend_probe_families_backtracking_unknown(
    probes: &mut Vec<ProbePoint>,
    family: HypermeshResult<Vec<ProbePoint>>,
    saw_unknown: &mut bool,
) -> HypermeshResult<()> {
    match family {
        Ok(found) => {
            for probe in found {
                push_unique_probe_point(probes, probe);
            }
            Ok(())
        }
        Err(HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
            Ok(())
        }
        Err(err) => Err(err),
    }
}

pub(super) fn probe_definitions_from_active_halfspaces(
    witness: &Point3,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    extra_planes: &[Plane],
) -> HypermeshResult<DefinitionFamilyState> {
    let axis_definition = axis_plane_definition(witness);
    let mut definitions = Vec::new();
    let mut saw_unknown = false;
    let mut active = Vec::new();

    for plane in extra_planes {
        if !active.iter().any(|existing| existing == plane) {
            active.push(plane.clone());
        }
    }

    for index in active_planes.into_iter().flatten() {
        let Some(halfspace) = halfspaces.get(index) else {
            continue;
        };
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if !active.iter().any(|existing| existing == &plane) {
            active.push(plane);
        }
    }

    for halfspace in halfspaces {
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if !compare_real(&plane.expression_at_point(witness), &Real::zero())?.is_eq() {
            continue;
        }
        if !active.iter().any(|existing| existing == &plane) {
            active.push(plane);
        }
    }

    for first in 0..active.len() {
        for second in (first + 1)..active.len() {
            for third in (second + 1)..active.len() {
                push_verified_probe_definition(
                    &mut definitions,
                    [
                        active[first].clone(),
                        active[second].clone(),
                        active[third].clone(),
                    ],
                    witness,
                    &mut saw_unknown,
                )?;
            }
        }
    }

    for first in 0..active.len() {
        for second in (first + 1)..active.len() {
            for axis_plane in &axis_definition {
                push_verified_probe_definition(
                    &mut definitions,
                    [
                        active[first].clone(),
                        active[second].clone(),
                        axis_plane.clone(),
                    ],
                    witness,
                    &mut saw_unknown,
                )?;
            }
        }
    }

    for plane in &active {
        for first_axis in 0..3 {
            for second_axis in (first_axis + 1)..3 {
                push_verified_probe_definition(
                    &mut definitions,
                    [
                        plane.clone(),
                        axis_definition[first_axis].clone(),
                        axis_definition[second_axis].clone(),
                    ],
                    witness,
                    &mut saw_unknown,
                )?;
            }
        }
    }

    push_verified_probe_definition(&mut definitions, axis_definition, witness, &mut saw_unknown)?;
    Ok(DefinitionFamilyState {
        definitions,
        saw_unknown,
    })
}

pub(super) fn probe_definitions_or_axis(
    witness: &Point3,
    result: HypermeshResult<DefinitionFamilyState>,
) -> HypermeshResult<(Vec<[Plane; 3]>, bool)> {
    match result {
        Ok(found) => Ok((found.definitions, false)),
        Err(HypermeshError::UnknownClassification) => {
            Ok((vec![axis_plane_definition(witness)], true))
        }
        Err(err) => Err(err),
    }
}

fn push_verified_probe_definition(
    definitions: &mut Vec<[Plane; 3]>,
    definition: [Plane; 3],
    witness: &Point3,
    saw_unknown: &mut bool,
) -> HypermeshResult<()> {
    if definition_has_coplanar_pair(&definition)? {
        return Ok(());
    }
    let homogeneous = intersect_three_planes(&definition[0], &definition[1], &definition[2]);
    let w_class = match classify_real(&homogeneous.w) {
        Ok(classification) => classification,
        Err(HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
            return Ok(());
        }
        Err(err) => return Err(err),
    };
    if w_class == Classification::On {
        return Ok(());
    }
    match homogeneous.to_affine_point() {
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

fn definition_has_coplanar_pair(definition: &[Plane; 3]) -> HypermeshResult<bool> {
    for first in 0..3 {
        for second in (first + 1)..3 {
            if planes_are_coplanar(&definition[first], &definition[second])? {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

#[cfg(test)]
pub(super) fn adjacent_normal_probes(
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    positive_side: bool,
) -> HypermeshResult<Vec<ProbePoint>> {
    let retained_definitions = unique_normal_probe_search_definitions(&interior.planes, support)?;
    adjacent_normal_probes_with_queries(
        interior,
        support,
        bounds,
        polygons,
        positive_side,
        |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        classify_point_in_polygon,
        |corridor, stop_point| {
            collect_normal_probe_targets(&retained_definitions, |definition| {
                strict_normal_probe_targets(
                    interior,
                    support,
                    corridor,
                    definition,
                    stop_point,
                    positive_side,
                )
            })
        },
    )
}

#[cfg(test)]
pub(super) fn adjacent_normal_probes_with_queries(
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    positive_side: bool,
    mut denom_for: impl FnMut(&Point3, &Point3, &ConvexPolygon) -> HypermeshResult<Real>,
    mut classify_point_on_polygon: impl FnMut(
        &Point3,
        &ConvexPolygon,
    ) -> HypermeshResult<PolygonPointLocation>,
    mut build: impl FnMut(&Aabb, &Point3) -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let direction = if positive_side {
        support.normal.clone()
    } else {
        Point3::new(
            -support.normal.x.clone(),
            -support.normal.y.clone(),
            -support.normal.z.clone(),
        )
    };

    let (stop_values, mut saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
        &interior.point,
        &direction,
        support,
        bounds,
        polygons,
        &mut denom_for,
        &mut classify_point_on_polygon,
    )?;
    let mut probes = Vec::new();

    for stop_t in stop_values {
        if !compare_real(&stop_t, &Real::zero())?.is_gt() {
            continue;
        }
        let stop_point = offset_point(&interior.point, &direction, &stop_t);
        let corridor = bounds_between_points(&interior.point, &stop_point)?;
        extend_probe_families_backtracking_unknown(
            &mut probes,
            build(&corridor, &stop_point),
            &mut saw_unknown,
        )?;
    }

    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
}

pub(super) fn adjacent_normal_probe_stop_values_with_queries(
    interior: &Point3,
    direction: &Point3,
    support: &Plane,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    denom_for: &mut impl FnMut(&Point3, &Point3, &ConvexPolygon) -> HypermeshResult<Real>,
    classify_point_on_polygon: &mut impl FnMut(
        &Point3,
        &ConvexPolygon,
    ) -> HypermeshResult<PolygonPointLocation>,
) -> HypermeshResult<(Vec<Real>, bool)> {
    let (bound_stop, mut saw_unknown) = normal_probe_bounds_stop(interior, direction, bounds)?;
    let Some(bound_stop) = bound_stop else {
        return Ok((Vec::new(), saw_unknown));
    };

    let mut stop_values = vec![bound_stop.clone()];

    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }
        if planes_are_coplanar(&polygon.support, support)? {
            continue;
        }

        let start_value = polygon.support.expression_at_point(interior);
        let start_class = match classify_real(&start_value) {
            Ok(classification) => classification,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if start_class == Classification::On {
            let point_location = match classify_point_on_polygon(interior, polygon) {
                Ok(point_location) => point_location,
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
            match point_location {
                PolygonPointLocation::Outside => {}
                PolygonPointLocation::Boundary => {
                    saw_unknown = true;
                }
                PolygonPointLocation::Interior => {
                    return Ok((Vec::new(), saw_unknown));
                }
            }
            continue;
        }

        let denom = match denom_for(interior, direction, polygon) {
            Ok(denom) => denom,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        let denom_class = match classify_real(&denom) {
            Ok(classification) => classification,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if denom_class == Classification::On {
            continue;
        }
        let crossing_t =
            match ((-start_value) / denom).map_err(|_| HypermeshError::UnknownClassification) {
                Ok(crossing_t) => crossing_t,
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
        let crossing = offset_point(interior, direction, &crossing_t);
        if !positive_real_strictly_before(&crossing_t, &bound_stop)? {
            if compare_real(&crossing_t, &bound_stop)?.is_eq() {
                let point_location = match classify_point_on_polygon(&crossing, polygon) {
                    Ok(point_location) => point_location,
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
                if matches!(
                    point_location,
                    PolygonPointLocation::Boundary | PolygonPointLocation::Interior
                ) {
                    saw_unknown = true;
                }
            }
            continue;
        }

        let point_location = match classify_point_on_polygon(&crossing, polygon) {
            Ok(point_location) => point_location,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        match point_location {
            PolygonPointLocation::Outside => continue,
            PolygonPointLocation::Boundary => {
                saw_unknown = true;
            }
            PolygonPointLocation::Interior => {}
        }

        let mut insert_at = stop_values.len();
        let mut duplicate = false;
        for (index, existing) in stop_values.iter().enumerate() {
            let order = compare_real(&crossing_t, existing)?;
            if order.is_eq() {
                duplicate = true;
                break;
            }
            if order.is_lt() {
                insert_at = index;
                break;
            }
        }
        if !duplicate {
            stop_values.insert(insert_at, crossing_t);
        }
    }

    Ok((stop_values, saw_unknown))
}

pub(super) fn push_unique_probe_point(probes: &mut Vec<ProbePoint>, probe: ProbePoint) -> bool {
    if let Some(existing) = probes
        .iter_mut()
        .find(|existing| existing.point == probe.point && existing.side == probe.side)
    {
        let incoming_planes = probe.planes;
        let incoming_is_fallback = probe.uncertified_definition_fallback;
        let existing_covered_by_incoming = existing.planes.iter().all(|existing_plane_set| {
            incoming_planes.iter().any(|incoming_plane_set| {
                definition_planes_match_as_sets(existing_plane_set, incoming_plane_set)
            })
        });
        let mut introduced_new_definition = false;
        for definition in incoming_planes {
            if !existing
                .planes
                .iter()
                .any(|candidate| definition_planes_match_as_sets(candidate, &definition))
            {
                existing.planes.push(definition);
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
        let introduced_uncertified_state = probe.uncertified_definition_fallback;
        probes.push(probe);
        introduced_uncertified_state
    }
}

#[cfg(test)]
pub(super) fn collect_normal_probe_targets(
    definitions: &[[Plane; 3]],
    mut search: impl FnMut(Option<&[Plane; 3]>) -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut probes = Vec::new();
    let mut saw_unknown = false;
    let definitions = unique_definition_family(definitions);
    for definition in &definitions {
        match search(Some(definition)) {
            Ok(found) => {
                for probe in found {
                    push_unique_probe_point(&mut probes, probe);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    match search(None) {
        Ok(found) => {
            for probe in found {
                push_unique_probe_point(&mut probes, probe);
            }
        }
        Err(HypermeshError::UnknownClassification) => {
            saw_unknown = true;
        }
        Err(err) => return Err(err),
    }
    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
}

pub(super) fn normal_probe_extra_planes(
    _interior: &InteriorLeafPoint,
    definition: Option<&[Plane; 3]>,
) -> Vec<Plane> {
    let mut extra_planes = Vec::new();
    if let Some(definition) = definition {
        for plane in &definition[1..] {
            if !extra_planes.iter().any(|existing| existing == plane) {
                extra_planes.push(plane.clone());
            }
        }
    }
    extra_planes
}

pub(super) fn normal_probe_shifted_seed_families(
    definition: Option<&[Plane; 3]>,
    report_witness: Option<&Point3>,
    certified_probe_points: &[Point3],
    seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
) -> (Vec<Point3>, Vec<Point3>, Vec<Point3>) {
    if definition.is_none() {
        if !certified_probe_points.is_empty() {
            let mut shifted_roots = Vec::new();
            if let Some(report_witness) = report_witness {
                shifted_roots.push(report_witness.clone());
            } else if let Some(first_probe_point) = certified_probe_points.first() {
                shifted_roots.push(first_probe_point.clone());
            }
            return dedupe_shifted_halfspace_seed_families(shifted_roots, Vec::new(), Vec::new());
        }
        if !seeds.is_empty() {
            return shifted_halfspace_seed_families_with_report_seed(
                report_witness,
                seeds,
                Vec::new(),
                Vec::new(),
            );
        }
    }
    shifted_halfspace_seed_families_with_report_seed(
        report_witness,
        seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    )
}

fn normal_probe_definition_preserves_support_direction(
    definition: &[Plane; 3],
    support: &Plane,
) -> HypermeshResult<bool> {
    Ok(
        classify_real(&dot_direction(&definition[1].normal, &support.normal))?
            == Classification::On
            && classify_real(&dot_direction(&definition[2].normal, &support.normal))?
                == Classification::On,
    )
}

pub(super) fn retained_plane_pairs_match_as_sets(left: &[Plane; 3], right: &[Plane; 3]) -> bool {
    (left[1] == right[1] && left[2] == right[2]) || (left[1] == right[2] && left[2] == right[1])
}

pub(super) fn unique_normal_probe_search_definitions(
    definitions: &[[Plane; 3]],
    support: &Plane,
) -> HypermeshResult<Vec<[Plane; 3]>> {
    let mut unique = Vec::new();
    for definition in unique_definition_family(definitions) {
        if !normal_probe_definition_preserves_support_direction(&definition, support)? {
            continue;
        }
        if unique
            .iter()
            .all(|existing| !retained_plane_pairs_match_as_sets(existing, &definition))
        {
            unique.push(definition);
        }
    }
    Ok(unique)
}

#[cfg(test)]
pub(super) fn strict_normal_probe_targets(
    interior: &InteriorLeafPoint,
    support: &Plane,
    corridor: &Aabb,
    definition: Option<&[Plane; 3]>,
    stop_point: &Point3,
    positive_side: bool,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));
    halfspaces.push(normal_stop_halfspace(support, stop_point, positive_side));

    let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(Vec::new());
    }
    let mut probes = Vec::new();
    let extra_planes = normal_probe_extra_planes(interior, definition);
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let shifted_vertex_family = feasible_halfspace_cell_vertex_family(&halfspaces)?;
    saw_unknown |= shifted_vertex_family.saw_unknown;
    let shifted_vertices = shifted_vertex_family.seeds;
    let mut shifted_geometry_seeds = Vec::new();
    let mut seeds = Vec::new();

    saw_unknown |= extend_strict_halfspace_seed_families_collect_unknown(
        &mut seeds,
        [
            if report
                .as_ref()
                .is_some_and(|report| report.status == HalfspaceFeasibility::Feasible)
                && let Some(witness) = report_witness
            {
                collect_strict_halfspace_seed_family(Ok(vec![witness.clone()]), |candidate| {
                    point_strictly_inside_halfspace_cell_or_unknown(
                        candidate,
                        corridor,
                        &halfspaces,
                    )
                })
            } else {
                Ok(HalfspaceSeedFamilyState {
                    seeds: Vec::new(),
                    saw_unknown: false,
                })
            },
            collect_strict_halfspace_seed_family(Ok(shifted_vertices.clone()), |candidate| {
                point_strictly_inside_halfspace_cell_or_unknown(candidate, corridor, &halfspaces)
            }),
        ],
    )?;

    let mut seen_direct_seeds = Vec::new();
    let mut seeds = take_new_halfspace_seed_family(seeds, &mut seen_direct_seeds);

    extend_probe_point_builds_backtracking_unknown(&mut probes, seeds.iter(), |witness| {
        build_probe_point(
            witness,
            corridor,
            support,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), witness),
            &extra_planes,
            false,
        )
    })?;

    let mut certified_probe_points = probes
        .iter()
        .filter(|probe| !probe.uncertified_definition_fallback)
        .map(|probe| probe.point.clone())
        .collect::<Vec<_>>();

    if definition.is_some() || certified_probe_points.is_empty() {
        let shifted_geometry_seed_family =
            halfspace_centroid_subset_seed_family_from_vertices(&shifted_vertices)?;
        saw_unknown |= shifted_geometry_seed_family.saw_unknown;
        shifted_geometry_seeds = shifted_geometry_seed_family.seeds;

        let mut geometry_strict_seeds = Vec::new();
        saw_unknown |= extend_strict_halfspace_seed_families_collect_unknown(
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
        extend_probe_point_builds_backtracking_unknown(
            &mut probes,
            geometry_strict_seeds.iter(),
            |witness| {
                build_probe_point(
                    witness,
                    corridor,
                    support,
                    &halfspaces,
                    active_planes_from_optional_report(report.as_ref(), witness),
                    &extra_planes,
                    false,
                )
            },
        )?;
        seeds.extend(geometry_strict_seeds);
        certified_probe_points = probes
            .iter()
            .filter(|probe| !probe.uncertified_definition_fallback)
            .map(|probe| probe.point.clone())
            .collect::<Vec<_>>();
    }

    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);
    if seed_family_search_failed_without_any_seed(
        &seeds,
        &shifted_vertices,
        &shifted_geometry_seeds,
        saw_unknown,
    ) {
        return Err(HypermeshError::UnknownClassification);
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

    let shifted_witnesses = shifted_halfspace_witness_family_or_empty(
        {
            let mut shifted_witnesses = Vec::new();
            extend_shifted_halfspace_seed_families_backtracking_unknown(
                &mut shifted_witnesses,
                [strict_shift_seeds, shifted_vertices, shifted_geometry_seeds],
                |seed| shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    for shifted in &shifted_witnesses {
        match build_probe_point_from_shifted_witness(shifted, corridor, support, &extra_planes) {
            Ok(Some(probe)) => {
                push_unique_probe_point(&mut probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
}

pub(super) fn strict_normal_probe_targets_with_query_caches(
    interior: &InteriorLeafPoint,
    support: &Plane,
    corridor: &Aabb,
    definition: Option<&[Plane; 3]>,
    stop_point: &Point3,
    positive_side: bool,
    probe_query_caches: &mut LeafProbeQueryCaches,
) -> HypermeshResult<Vec<ProbePoint>> {
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
        return Ok(Vec::new());
    }
    let mut probes = Vec::new();
    let extra_planes = normal_probe_extra_planes(interior, definition);
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

    extend_probe_point_builds_backtracking_unknown(&mut probes, seeds.iter(), |witness| {
        build_probe_point(
            witness,
            corridor,
            support,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), witness),
            &extra_planes,
            false,
        )
    })?;

    let mut certified_probe_points = probes
        .iter()
        .filter(|probe| !probe.uncertified_definition_fallback)
        .map(|probe| probe.point.clone())
        .collect::<Vec<_>>();

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
        extend_probe_point_builds_backtracking_unknown(
            &mut probes,
            geometry_strict_seeds.iter(),
            |witness| {
                build_probe_point(
                    witness,
                    corridor,
                    support,
                    &halfspaces,
                    active_planes_from_optional_report(report.as_ref(), witness),
                    &extra_planes,
                    false,
                )
            },
        )?;
        seeds.extend(geometry_strict_seeds);
        certified_probe_points = probes
            .iter()
            .filter(|probe| !probe.uncertified_definition_fallback)
            .map(|probe| probe.point.clone())
            .collect::<Vec<_>>();
    }

    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);
    if seed_family_search_failed_without_any_seed(
        &seeds,
        &shifted_vertices,
        &shifted_geometry_seeds,
        local_unknown,
    ) {
        return Err(HypermeshError::UnknownClassification);
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

    let shifted_witnesses = shifted_halfspace_witness_family_or_empty(
        {
            let mut shifted_witnesses = Vec::new();
            extend_shifted_halfspace_seed_families_backtracking_unknown(
                &mut shifted_witnesses,
                [strict_shift_seeds, shifted_vertices, shifted_geometry_seeds],
                |seed| shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut local_unknown,
    )?;
    for shifted in &shifted_witnesses {
        match build_probe_point_from_shifted_witness(shifted, corridor, support, &extra_planes) {
            Ok(Some(probe)) => {
                push_unique_probe_point(&mut probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                local_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_probe_point_family(&mut probes, local_unknown)?;
    Ok(probes)
}

#[cfg(test)]
pub(super) fn strict_normal_probe_targets_from_seed_families_with_tracking_unknown(
    interior: &InteriorLeafPoint,
    support: &Plane,
    corridor: &Aabb,
    definition: Option<&[Plane; 3]>,
    stop_point: &Point3,
    positive_side: bool,
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    mut build_shifted_witnesses: impl FnMut(&Point3) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));
    halfspaces.push(normal_stop_halfspace(support, stop_point, positive_side));

    let mut probes = Vec::new();
    let mut saw_unknown = false;
    let extra_planes = normal_probe_extra_planes(interior, definition);
    let report_witness = report.and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);

    extend_probe_point_builds_backtracking_unknown(&mut probes, seeds.iter(), |witness| {
        build_probe_point(
            witness,
            corridor,
            support,
            &halfspaces,
            active_planes_from_optional_report(report, witness),
            &extra_planes,
            false,
        )
    })?;

    let certified_probe_points = probes
        .iter()
        .filter(|probe| !probe.uncertified_definition_fallback)
        .map(|probe| probe.point.clone())
        .collect::<Vec<_>>();
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        normal_probe_shifted_seed_families(
            definition,
            report_witness,
            &certified_probe_points,
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
        match build_probe_point_from_shifted_witness(shifted, corridor, support, &extra_planes) {
            Ok(Some(probe)) => {
                push_unique_probe_point(&mut probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
}

pub(super) fn bounds_between_points(start: &Point3, end: &Point3) -> HypermeshResult<Aabb> {
    let mut min = Point3::origin();
    let mut max = Point3::origin();
    for axis in 0..3 {
        let start_value = axis_ref(start, axis);
        let end_value = axis_ref(end, axis);
        if compare_real(start_value, end_value)?.is_le() {
            *axis_mut(&mut min, axis) = start_value.clone();
            *axis_mut(&mut max, axis) = end_value.clone();
        } else {
            *axis_mut(&mut min, axis) = end_value.clone();
            *axis_mut(&mut max, axis) = start_value.clone();
        }
    }
    Ok(Aabb::new(min, max))
}

fn normal_probe_bounds_stop(
    interior: &Point3,
    direction: &Point3,
    bounds: &Aabb,
) -> HypermeshResult<(Option<Real>, bool)> {
    let mut stop_t: Option<Real> = None;
    let mut saw_unknown = false;
    for axis in 0..3 {
        let component = axis_ref(direction, axis);
        match classify_real(component)? {
            Classification::Positive => {
                let room = axis_ref(&bounds.max, axis) - axis_ref(interior, axis);
                let room_order = compare_real(&room, &Real::zero())?;
                if !room_order.is_gt() {
                    saw_unknown = room_order.is_eq();
                    return Ok((None, saw_unknown));
                }
                update_positive_stop(
                    &mut stop_t,
                    (room / component.clone())
                        .map_err(|_| HypermeshError::UnknownClassification)?,
                )?;
            }
            Classification::Negative => {
                let room = axis_ref(interior, axis) - axis_ref(&bounds.min, axis);
                let room_order = compare_real(&room, &Real::zero())?;
                if !room_order.is_gt() {
                    saw_unknown = room_order.is_eq();
                    return Ok((None, saw_unknown));
                }
                update_positive_stop(
                    &mut stop_t,
                    (room / (-component.clone()))
                        .map_err(|_| HypermeshError::UnknownClassification)?,
                )?;
            }
            Classification::On => {}
        }
    }
    Ok((stop_t, saw_unknown))
}

fn update_positive_stop(stop_t: &mut Option<Real>, candidate: Real) -> HypermeshResult<()> {
    if !compare_real(&candidate, &Real::zero())?.is_gt() {
        return Ok(());
    }
    if stop_t
        .as_ref()
        .is_none_or(|current| compare_real(&candidate, current).is_ok_and(|order| order.is_lt()))
    {
        *stop_t = Some(candidate);
    }
    Ok(())
}

fn positive_real_strictly_before(value: &Real, stop: &Real) -> HypermeshResult<bool> {
    Ok(compare_real(value, &Real::zero())?.is_gt() && compare_real(value, stop)?.is_lt())
}

pub(super) fn dot_direction(left: &Point3, right: &Point3) -> Real {
    Real::signed_product_sum(
        [true, true, true],
        [
            [&left.x, &right.x],
            [&left.y, &right.y],
            [&left.z, &right.z],
        ],
    )
}

pub(super) fn offset_point(point: &Point3, direction: &Point3, amount: &Real) -> Point3 {
    Point3::new(
        &point.x + &(amount * &direction.x),
        &point.y + &(amount * &direction.y),
        &point.z + &(amount * &direction.z),
    )
}

#[cfg_attr(not(test), allow(dead_code))]
pub(super) fn adjacent_axis_probes(
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
    direction_positive: bool,
) -> HypermeshResult<Vec<ProbePoint>> {
    adjacent_axis_probes_with_queries(
        interior,
        support,
        bounds,
        polygons,
        axis,
        direction_positive,
        |interior, endpoint, polygon, _axis| {
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
        classify_point_in_polygon,
        |corridor| {
            collect_axis_probe_targets(&interior.planes, |definition| {
                if let Some(definition) = definition
                    && !axis_probe_definition_preserves_axis_direction(definition, axis)?
                {
                    return Ok(Vec::new());
                }
                strict_axis_probe_targets(
                    interior,
                    support,
                    corridor,
                    axis,
                    direction_positive,
                    definition,
                )
            })
        },
    )
}

#[cfg_attr(not(test), allow(dead_code))]
pub(super) fn adjacent_axis_probes_with_queries(
    interior: &InteriorLeafPoint,
    _support: &Plane,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
    direction_positive: bool,
    mut crossing_for: impl FnMut(
        &Point3,
        &Point3,
        &ConvexPolygon,
        usize,
    ) -> HypermeshResult<Option<Point3>>,
    mut classify_point_on_polygon: impl FnMut(
        &Point3,
        &ConvexPolygon,
    ) -> HypermeshResult<PolygonPointLocation>,
    mut build: impl FnMut(&Aabb) -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let (stop_values, mut saw_unknown) = adjacent_axis_probe_stop_values_with_queries(
        &interior.point,
        bounds,
        polygons,
        axis,
        direction_positive,
        &mut crossing_for,
        &mut classify_point_on_polygon,
    )?;
    let start_value = axis_ref(&interior.point, axis);
    let mut probes = Vec::new();

    for stop_value in stop_values {
        if !axis_value_after_start(start_value, &stop_value, direction_positive)? {
            continue;
        }
        let corridor = axis_probe_bounds(&interior.point, axis, &stop_value)?;
        extend_probe_families_backtracking_unknown(
            &mut probes,
            build(&corridor),
            &mut saw_unknown,
        )?;
    }

    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
}

pub(super) fn adjacent_axis_probe_stop_values_with_queries(
    interior: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
    direction_positive: bool,
    crossing_for: &mut impl FnMut(
        &Point3,
        &Point3,
        &ConvexPolygon,
        usize,
    ) -> HypermeshResult<Option<Point3>>,
    classify_point_on_polygon: &mut impl FnMut(
        &Point3,
        &ConvexPolygon,
    ) -> HypermeshResult<PolygonPointLocation>,
) -> HypermeshResult<(Vec<Real>, bool)> {
    let start_value = axis_ref(interior, axis);
    let bound_value = if direction_positive {
        axis_ref(&bounds.max, axis)
    } else {
        axis_ref(&bounds.min, axis)
    };
    if !axis_value_after_start(start_value, bound_value, direction_positive)? {
        return Ok((Vec::new(), compare_real(start_value, bound_value)?.is_eq()));
    }

    let mut endpoint = interior.clone();
    *axis_mut(&mut endpoint, axis) = bound_value.clone();
    let mut stop_values = vec![bound_value.clone()];
    let mut saw_unknown = false;

    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        let Some(crossing) = (match crossing_for(interior, &endpoint, polygon, axis) {
            Ok(crossing) => crossing,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        }) else {
            continue;
        };
        let point_location = match classify_point_on_polygon(&crossing, polygon) {
            Ok(point_location) => point_location,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if !point_strictly_between_axis(&crossing, interior, &endpoint, axis)? {
            if crossing == *interior
                && matches!(
                    point_location,
                    PolygonPointLocation::Boundary | PolygonPointLocation::Interior
                )
            {
                saw_unknown = true;
            }
            if crossing == endpoint
                && matches!(
                    point_location,
                    PolygonPointLocation::Boundary | PolygonPointLocation::Interior
                )
            {
                saw_unknown = true;
            }
            continue;
        }
        match point_location {
            PolygonPointLocation::Outside => continue,
            PolygonPointLocation::Boundary => {
                saw_unknown = true;
            }
            PolygonPointLocation::Interior => {}
        }

        let crossing_value = axis_ref(&crossing, axis);
        if !axis_value_after_start(start_value, crossing_value, direction_positive)? {
            continue;
        }

        let mut insert_at = stop_values.len();
        let mut duplicate = false;
        for (index, existing) in stop_values.iter().enumerate() {
            if compare_real(crossing_value, existing)?.is_eq() {
                duplicate = true;
                break;
            }
            if axis_value_before_stop(crossing_value, existing, direction_positive)? {
                insert_at = index;
                break;
            }
        }
        if !duplicate {
            stop_values.insert(insert_at, crossing_value.clone());
        }
    }

    Ok((stop_values, saw_unknown))
}

pub(super) fn axis_probe_bounds(
    interior: &Point3,
    axis: usize,
    stop_value: &Real,
) -> HypermeshResult<Aabb> {
    let mut min = interior.clone();
    let mut max = interior.clone();
    let start_value = axis_ref(interior, axis);
    if compare_real(start_value, stop_value)?.is_lt() {
        *axis_mut(&mut max, axis) = stop_value.clone();
    } else {
        *axis_mut(&mut min, axis) = stop_value.clone();
    }
    Ok(Aabb::new(min, max))
}

#[cfg_attr(not(test), allow(dead_code))]
pub(super) fn collect_axis_probe_targets(
    definitions: &[[Plane; 3]],
    mut search: impl FnMut(Option<&[Plane; 3]>) -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut probes = Vec::new();
    let mut saw_unknown = false;
    let definitions = unique_definition_family(definitions);
    for definition in &definitions {
        match search(Some(definition)) {
            Ok(found) => {
                for probe in found {
                    push_unique_probe_point(&mut probes, probe);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    match search(None) {
        Ok(found) => {
            for probe in found {
                push_unique_probe_point(&mut probes, probe);
            }
        }
        Err(HypermeshError::UnknownClassification) => {
            saw_unknown = true;
        }
        Err(err) => return Err(err),
    }
    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
}

pub(super) fn extend_probe_point_builds_backtracking_unknown<'a, T: 'a>(
    probes: &mut Vec<ProbePoint>,
    candidates: impl IntoIterator<Item = &'a T>,
    mut build: impl FnMut(&'a T) -> HypermeshResult<Option<ProbePoint>>,
) -> HypermeshResult<()> {
    let mut saw_hard_unknown = false;
    for candidate in candidates {
        match build(candidate) {
            Ok(Some(probe)) => {
                push_unique_probe_point(probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_probe_point_family(probes, saw_hard_unknown)
}

pub(super) fn axis_probe_definition_preserves_axis_direction(
    definition: &[Plane; 3],
    axis: usize,
) -> HypermeshResult<bool> {
    Ok(
        classify_real(axis_ref(&definition[1].normal, axis))? == Classification::On
            && classify_real(axis_ref(&definition[2].normal, axis))? == Classification::On,
    )
}

pub(super) fn strict_axis_probe_targets(
    interior: &InteriorLeafPoint,
    support: &Plane,
    corridor: &Aabb,
    axis: usize,
    positive_side: bool,
    definition: Option<&[Plane; 3]>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));
    let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(Vec::new());
    }
    let mut probes = Vec::new();
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        halfspace_cell_seed_families_from_optional_report(
            corridor,
            &halfspaces,
            report.as_ref(),
            &mut saw_unknown,
        )?;
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);

    extend_probe_point_builds_backtracking_unknown(&mut probes, seeds.iter(), |witness| {
        build_axis_probe_point(
            witness,
            interior,
            corridor,
            support,
            axis,
            definition,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), witness),
            false,
        )
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
                |seed| shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    for shifted in &shifted_witnesses {
        match build_axis_probe_point_from_shifted_witness(
            shifted, interior, corridor, support, axis, definition,
        ) {
            Ok(Some(probe)) => {
                push_unique_probe_point(&mut probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
}

#[cfg(test)]
pub(super) fn strict_axis_probe_targets_from_seed_families_with_tracking_unknown(
    interior: &InteriorLeafPoint,
    support: &Plane,
    corridor: &Aabb,
    axis: usize,
    positive_side: bool,
    definition: Option<&[Plane; 3]>,
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    mut build_shifted_witnesses: impl FnMut(&Point3) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));

    let mut probes = Vec::new();
    let mut saw_unknown = false;
    let report_witness = report.and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);

    extend_probe_point_builds_backtracking_unknown(&mut probes, seeds.iter(), |witness| {
        build_axis_probe_point(
            witness,
            interior,
            corridor,
            support,
            axis,
            definition,
            &halfspaces,
            active_planes_from_optional_report(report, witness),
            false,
        )
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
        match build_axis_probe_point_from_shifted_witness(
            shifted, interior, corridor, support, axis, definition,
        ) {
            Ok(Some(probe)) => {
                push_unique_probe_point(&mut probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
}

pub(super) fn build_probe_point(
    witness: &Point3,
    corridor: &Aabb,
    support: &Plane,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    extra_planes: &[Plane],
    inherited_uncertified_definition_fallback: bool,
) -> HypermeshResult<Option<ProbePoint>> {
    if !point_strictly_inside_halfspace_cell_or_unknown(witness, corridor, halfspaces)? {
        return Ok(None);
    }
    let side = classify_point(witness, support)?;
    if side == Classification::On {
        return Err(HypermeshError::UnknownClassification);
    }

    let shifted_support = Plane::new(
        support.normal.clone(),
        &support.offset - &support.expression_at_point(witness),
    );
    let mut all_extra_planes = vec![shifted_support];
    for plane in extra_planes {
        if !all_extra_planes.iter().any(|existing| existing == plane) {
            all_extra_planes.push(plane.clone());
        }
    }

    let (planes, uncertified_definition_fallback) = probe_definitions_or_axis(
        witness,
        probe_definitions_from_active_halfspaces(
            witness,
            halfspaces,
            active_planes,
            &all_extra_planes,
        ),
    )?;
    Ok(Some(ProbePoint {
        point: witness.clone(),
        side,
        planes,
        uncertified_definition_fallback: inherited_uncertified_definition_fallback
            || uncertified_definition_fallback,
    }))
}

pub(super) fn build_probe_point_from_shifted_witness(
    witness: &ShiftedHalfspaceWitness,
    corridor: &Aabb,
    support: &Plane,
    extra_planes: &[Plane],
) -> HypermeshResult<Option<ProbePoint>> {
    let side = classify_point(&witness.point, support)?;
    if side == Classification::On {
        return Err(HypermeshError::UnknownClassification);
    }

    let shifted_support = Plane::new(
        support.normal.clone(),
        &support.offset - &support.expression_at_point(&witness.point),
    );
    let mut all_extra_planes = vec![shifted_support];
    for plane in extra_planes {
        if !all_extra_planes.iter().any(|existing| existing == plane) {
            all_extra_planes.push(plane.clone());
        }
    }

    let mut planes = Vec::new();
    let mut saw_unknown = false;
    for family in &witness.families {
        match point_strictly_inside_halfspace_cell_or_unknown(
            &witness.point,
            corridor,
            &family.halfspaces,
        )? {
            true => {}
            false => continue,
        }
        match probe_definitions_from_active_halfspaces(
            &witness.point,
            &family.halfspaces,
            family.active_planes,
            &all_extra_planes,
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

    Ok(Some(ProbePoint {
        point: witness.point.clone(),
        side,
        planes,
        uncertified_definition_fallback: witness.uncertified_definition_fallback
            || used_axis_fallback,
    }))
}

pub(super) fn build_axis_probe_point(
    witness: &Point3,
    interior: &InteriorLeafPoint,
    corridor: &Aabb,
    support: &Plane,
    axis: usize,
    definition: Option<&[Plane; 3]>,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    inherited_uncertified_definition_fallback: bool,
) -> HypermeshResult<Option<ProbePoint>> {
    if !point_strictly_inside_halfspace_cell_or_unknown(witness, corridor, halfspaces)? {
        return Ok(None);
    }
    let side = classify_point(witness, support)?;
    if side == Classification::On {
        return Err(HypermeshError::UnknownClassification);
    }

    let (planes, uncertified_definition_fallback) = probe_definitions_or_axis(
        witness,
        axis_probe_definitions(
            interior,
            support,
            axis,
            definition,
            halfspaces,
            active_planes,
            witness,
        ),
    )?;
    Ok(Some(ProbePoint {
        point: witness.clone(),
        side,
        planes,
        uncertified_definition_fallback: inherited_uncertified_definition_fallback
            || uncertified_definition_fallback,
    }))
}

pub(super) fn build_axis_probe_point_from_shifted_witness(
    witness: &ShiftedHalfspaceWitness,
    interior: &InteriorLeafPoint,
    corridor: &Aabb,
    support: &Plane,
    axis: usize,
    definition: Option<&[Plane; 3]>,
) -> HypermeshResult<Option<ProbePoint>> {
    let side = classify_point(&witness.point, support)?;
    if side == Classification::On {
        return Err(HypermeshError::UnknownClassification);
    }

    let mut planes = Vec::new();
    let mut saw_unknown = false;
    for family in &witness.families {
        match point_strictly_inside_halfspace_cell_or_unknown(
            &witness.point,
            corridor,
            &family.halfspaces,
        )? {
            true => {}
            false => continue,
        }
        match axis_probe_definitions(
            interior,
            support,
            axis,
            definition,
            &family.halfspaces,
            family.active_planes,
            &witness.point,
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

    Ok(Some(ProbePoint {
        point: witness.point.clone(),
        side,
        planes,
        uncertified_definition_fallback: witness.uncertified_definition_fallback
            || used_axis_fallback,
    }))
}

fn axis_probe_definitions(
    interior: &InteriorLeafPoint,
    support: &Plane,
    axis: usize,
    definition: Option<&[Plane; 3]>,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    witness: &Point3,
) -> HypermeshResult<DefinitionFamilyState> {
    let shifted_support = Plane::new(
        support.normal.clone(),
        &support.offset - &support.expression_at_point(witness),
    );
    let axes = other_axes(axis);
    let mut extra_planes = vec![
        shifted_support,
        Plane::axis_aligned(axes[0], axis_ref(&interior.point, axes[0]).clone()),
        Plane::axis_aligned(axes[1], axis_ref(&interior.point, axes[1]).clone()),
    ];
    if let Some(definition) = definition {
        for plane in &definition[1..] {
            if !extra_planes.iter().any(|existing| existing == plane) {
                extra_planes.push(plane.clone());
            }
        }
    }
    probe_definitions_from_active_halfspaces(witness, halfspaces, active_planes, &extra_planes)
}

fn plane_halfspace(plane: &Plane) -> LimitPlane3 {
    LimitPlane3::new(plane.normal.clone(), plane.offset.clone())
}

pub(super) fn push_plane_equality_halfspaces(halfspaces: &mut Vec<LimitPlane3>, plane: &Plane) {
    let halfspace = plane_halfspace(plane);
    halfspaces.push(halfspace.clone());
    halfspaces.push(negated_halfspace(&halfspace));
}

pub(super) fn normal_stop_halfspace(
    plane: &Plane,
    stop_point: &Point3,
    positive_side: bool,
) -> LimitPlane3 {
    let stop_plane = Plane::new(
        plane.normal.clone(),
        &plane.offset - &plane.expression_at_point(stop_point),
    );
    let halfspace = plane_halfspace(&stop_plane);
    if positive_side {
        halfspace
    } else {
        negated_halfspace(&halfspace)
    }
}

#[cfg(test)]
pub(super) fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    remaining_detours: usize,
    trace_without_detours: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<bool>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    if trace_without_detours(start, end)? {
        return Ok(true);
    }

    if remaining_detours == 0 {
        return Ok(false);
    }

    for detour in detours_for(start, end)? {
        if detour.point == *start
            || detour.point == *end
            || point_lies_on_traced_surface(&detour.point, polygons)?
        {
            continue;
        }
        if !probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
            start,
            &detour.point,
            polygons,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        )? {
            continue;
        }
        if probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
            &detour.point,
            end,
            polygons,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        )? {
            return Ok(true);
        }
    }

    Ok(false)
}

pub(super) fn axis_value_after_start(
    start: &Real,
    value: &Real,
    direction_positive: bool,
) -> HypermeshResult<bool> {
    let order = compare_real(value, start)?;
    Ok((direction_positive && order.is_gt()) || (!direction_positive && order.is_lt()))
}

fn axis_value_before_stop(
    value: &Real,
    stop: &Real,
    direction_positive: bool,
) -> HypermeshResult<bool> {
    let order = compare_real(value, stop)?;
    Ok((direction_positive && order.is_lt()) || (!direction_positive && order.is_gt()))
}

pub(super) fn probe_axes(support: &Plane) -> HypermeshResult<Vec<usize>> {
    let dominant = dominant_normal_axis(support)?;
    let mut axes = vec![dominant];
    for axis in 0..3 {
        if axis != dominant && !axis_ref(&support.normal, axis).definitely_zero() {
            axes.push(axis);
        }
    }
    Ok(axes)
}
